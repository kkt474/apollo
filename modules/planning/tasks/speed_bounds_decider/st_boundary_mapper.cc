/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file st_boundary_mapper.cc
 * @brief ST边界映射器实现文件
 *
 * 本文件实现了STBoundaryMapper类，负责将障碍物映射到ST（时空）图中。
 *
 * 核心功能：
 * 1. 将静态障碍物映射为ST边界矩形
 * 2. 将动态障碍物（带预测轨迹）映射为ST边界多边形
 * 3. 根据纵向决策（STOP/FOLLOW/YIELD/OVERTAKE）调整ST边界
 *
 * ST图概念：
 * - S轴：沿路径的累积距离
 * - T轴：时间
 * - ST边界：障碍物在时空上的占用区域，速度规划必须在此边界之外
 *
 * 算法流程：
 * 1. 遍历路径上的所有点
 * 2. 检查自车边界框与障碍物的重叠
 * 3. 如果重叠，记录该时刻的S上下界
 * 4. 最终形成ST边界多边形
 *
 * C++语法说明：
 * - 初始化列表: 构造函数使用初始化列表初始化成员变量
 * - std::numeric_limits: 获取数值极限
 * - std::fmax/std::fmin: 浮点数极值函数
 * - const引用: 避免拷贝提高效率
 */

#include "modules/planning/tasks/speed_bounds_decider/st_boundary_mapper.h"

/**
 * @brief 标准库头文件
 * <algorithm>: 提供std::min, std::max, std::fmax等算法
 * <limits>: 提供std::numeric_limits获取数值极限
 * <memory>: 提供std::shared_ptr智能指针
 * <utility>: 提供std::pair等工具
 */
#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
/**
 * @brief PNC点消息protobuf定义
 * 包含STPoint, PathPoint等数据结构
 */
#include "modules/common_msgs/planning_msgs/decision.pb.h"
/**
 * @brief 规划决策消息protobuf定义
 * 包含ObjectDecisionType等
 */

#include "cyber/common/log.h"
/**
 * @brief Cyber RT日志系统
 * AERROR, ADEBUG, ACHECK等日志宏
 */
#include "modules/common/configs/vehicle_config_helper.h"
/**
 * @brief 车辆配置辅助类
 */
#include "modules/common/math/line_segment2d.h"
/**
 * @brief 2D线段数学工具
 */
#include "modules/common/math/vec2d.h"
/**
 * @brief 2D向量数学工具
 */
#include "modules/common/util/string_util.h"
/**
 * @brief 字符串工具函数
 */
#include "modules/common/util/util.h"
/**
 * @brief 通用工具函数
 */
#include "modules/common/vehicle_state/vehicle_state_provider.h"
/**
 * @brief 车辆状态提供者
 */
#include "modules/planning/planning_base/common/frame.h"
/**
 * @brief 规划帧数据结构
 */
#include "modules/planning/planning_base/common/planning_context.h"
/**
 * @brief 规划上下文
 */
#include "modules/planning/planning_base/gflags/planning_gflags.h"
/**
 * @brief Planning模块GFlags配置
 */

namespace apollo {
/**
 * @brief Apollo顶层命名空间
 */
namespace planning {

/**
 * @brief 类型别名定义
 * using: 类型别名，简化代码书写
 */
using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::Status;
using apollo::common::math::Box2d;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;

/**
 * @brief STBoundaryMapper构造函数
 *
 * @param config 速度边界决策配置
 * @param reference_line 参考线
 * @param path_data 路径数据
 * @param planning_distance 规划距离
 * @param planning_time 规划时间
 * @param injector 依赖注入器
 *
 * 使用初始化列表初始化成员变量
 *
 * C++语法说明：
 * : speed_bounds_config_(config), reference_line_(reference_line), ...
 *   初始化列表语法，在构造函数参数列表后、函数体前使用冒号
 *   直接初始化成员变量，比在函数体中赋值更高效
 */
STBoundaryMapper::STBoundaryMapper(
    const SpeedBoundsDeciderConfig& config, const ReferenceLine& reference_line,
    const PathData& path_data, const double planning_distance,
    const double planning_time,
    const std::shared_ptr<DependencyInjector>& injector)
    : speed_bounds_config_(config),
      reference_line_(reference_line),
      path_data_(path_data),
      vehicle_param_(common::VehicleConfigHelper::GetConfig().vehicle_param()),
      planning_max_distance_(planning_distance),
      planning_max_time_(planning_time),
      injector_(injector) {}

/**
 * @brief 计算ST边界主函数
 *
 * @param path_decision 路径决策
 * @return Status 执行状态
 *
 * 主要流程：
 * 1. 检查输入有效性
 * 2. 遍历所有障碍物
 * 3. 如果无纵向决策，直接映射到ST图
 * 4. 如果有纵向决策（STOP/FOLLOW/YIELD/OVERTAKE），精细调整边界
 * 5. 处理停车决策
 *
 * C++语法说明：
 * - PathDecision*: 原始指针，需要保证生命周期
 * - std::numeric_limits<double>::max(): 获取double最大值
 * - const auto*: 指向常量的指针
 */
Status STBoundaryMapper::ComputeSTBoundary(PathDecision* path_decision) const {
  /**
   * @brief 健全性检查
   * CHECK_GT: Apollo断言宏，Greater Than
   * 确保规划时间大于0
   */
  CHECK_GT(planning_max_time_, 0.0);

  /**
   * @brief 检查路径点数量
   * path_data_.discretized_path().size() < 2:
   *   如果路径点少于2个，无法进行ST映射
   */
  if (path_data_.discretized_path().size() < 2) {
    AERROR << "Fail to get params because of too few path points. path points "
              "size: "
           << path_data_.discretized_path().size() << ".";
    return Status(ErrorCode::PLANNING_ERROR,
                  "Fail to get params because of too few path points");
  }

  /**
   * @brief 遍历所有障碍物
   * stop_obstacle: 停车障碍物指针
   * min_stop_s: 最小停车距离
   */
  Obstacle* stop_obstacle = nullptr;
  ObjectDecisionType stop_decision;
  double min_stop_s = std::numeric_limits<double>::max();

  /**
   * @brief 遍历路径决策中的所有障碍物
   * path_decision->obstacles().Items():
   *   获取障碍物列表
   */
  for (const auto* ptr_obstacle_item : path_decision->obstacles().Items()) {
    /**
     * @brief 查找障碍物
     */
    Obstacle* ptr_obstacle = path_decision->Find(ptr_obstacle_item->Id());
    ACHECK(ptr_obstacle != nullptr);

    /**
     * @brief 如果没有纵向决策，直接映射到ST图
     * HasLongitudinalDecision():
     *   检查障碍物是否有纵向决策
     */
    if (!ptr_obstacle->HasLongitudinalDecision()) {
      ComputeSTBoundary(ptr_obstacle);
      continue;
    }

    /**
     * @brief 获取纵向决策
     */
    const auto& decision = ptr_obstacle->LongitudinalDecision();

    /**
     * @brief 处理STOP决策
     * 记录最近的停车障碍物
     */
    if (decision.has_stop()) {
      /**
       * @brief 将停车点坐标转换为SL坐标
       * reference_line_.XYToSL():
       *   参考线的XY坐标转SL坐标
       */
      common::SLPoint stop_sl_point;
      reference_line_.XYToSL(decision.stop().stop_point(), &stop_sl_point);
      const double stop_s = stop_sl_point.s();

      /**
       * @brief 记录最近的停车信息
       */
      if (stop_s < min_stop_s) {
        stop_obstacle = ptr_obstacle;
        min_stop_s = stop_s;
        stop_decision = decision;
      }
    }

    /**
     * @brief 处理FOLLOW/YIELD/OVERTAKE决策
     * 根据决策类型调整ST边界的上/下界
     */
    else if (decision.has_follow() || decision.has_overtake() ||
               decision.has_yield()) {
      /**
       * @brief ComputeSTBoundaryWithDecision():
       *   根据纵向决策精细调整ST边界
       */
      ComputeSTBoundaryWithDecision(ptr_obstacle, decision);
    }

    /**
     * @brief 忽略其他决策
     */
    else if (!decision.has_ignore()) {
      AWARN << "No mapping for decision: " << decision.DebugString();
    }
  }

  /**
   * @brief 处理停车障碍物
   */
  if (stop_obstacle) {
    bool success = MapStopDecision(stop_obstacle, stop_decision);
    if (!success) {
      const std::string msg = "Fail to MapStopDecision.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }

  return Status::OK();
}

/**
 * @brief 映射停车决策到ST边界
 *
 * @param stop_obstacle 停车障碍物
 * @param stop_decision 停车决策
 * @return bool 是否成功
 *
 * 停车ST边界：
 * - 时间T=[0, planning_max_time_]
 * - S下界从停车点开始
 * - S上界从停车点延伸到planning_max_distance_
 *
 * C++语法说明：
 * - std::fmax: 浮点数取较大值
 * - std::vector<std::pair<STPoint, STPoint>>:
 *   存储ST边界点对，每对包含上下边界点
 */
bool STBoundaryMapper::MapStopDecision(
    Obstacle* stop_obstacle, const ObjectDecisionType& stop_decision) const {
  /**
   * @brief 断言检查
   * DCHECK: 仅DEBUG模式生效的断言
   */
  DCHECK(stop_decision.has_stop()) << "Must have stop decision";

  /**
   * @brief 转换停车点坐标
   */
  common::SLPoint stop_sl_point;
  reference_line_.XYToSL(stop_decision.stop().stop_point(), &stop_sl_point);

  /**
   * @brief 计算停车参考S坐标
   * stop_sl_point.s(): 停车点S坐标
   * vehicle_param_.front_edge_to_center(): 前轴到车辆中心的距离
   * 减去此距离得到后轴参考的停车S坐标
   */
  double st_stop_s = 0.0;
  const double stop_ref_s =
      stop_sl_point.s() - vehicle_param_.front_edge_to_center();

  /**
   * @brief 计算ST边界的S坐标
   */
  if (stop_ref_s > path_data_.frenet_frame_path().back().s()) {
    /**
     * @brief 停车点在路径终点之后
     */
    st_stop_s = path_data_.discretized_path().back().s() +
                (stop_ref_s - path_data_.frenet_frame_path().back().s());
  } else {
    /**
     * @brief 停车点在路径范围内
     */
    PathPoint stop_point;
    if (!path_data_.GetPathPointWithRefS(stop_ref_s, &stop_point)) {
      return false;
    }
    st_stop_s = stop_point.s();
  }

  /**
   * @brief 计算S边界
   * s_min: 下界，不能为负
   * s_max: 上界，不能小于s_min
   */
  const double s_min = std::fmax(0.0, st_stop_s);
  const double s_max = std::fmax(
      s_min, std::fmax(planning_max_distance_, reference_line_.Length()));

  /**
   * @brief 创建ST边界点对
   * point_pairs: 存储ST多边形的边
   * 每对包含上下边界点
   */
  std::vector<std::pair<STPoint, STPoint>> point_pairs;

  /**
   * @brief T=0时刻的边界线
   */
  point_pairs.emplace_back(STPoint(s_min, 0.0), STPoint(s_max, 0.0));

  /**
   * @brief T=planning_max_time_时刻的边界线
   * 添加boundary_buffer作为缓冲区
   */
  point_pairs.emplace_back(
      STPoint(s_min, planning_max_time_),
      STPoint(s_max + speed_bounds_config_.boundary_buffer(),   // 0.25
              planning_max_time_));

  /**
   * @brief 创建ST边界对象
   */
  auto boundary = STBoundary(point_pairs);
  boundary.SetBoundaryType(STBoundary::BoundaryType::STOP);
  boundary.SetCharacteristicLength(speed_bounds_config_.boundary_buffer());
  boundary.set_id(stop_obstacle->Id());

  /**
   * @brief 设置障碍物的路径ST边界
   */
  stop_obstacle->set_path_st_boundary(boundary);
  return true;
}

/**
 * @brief 计算障碍物的ST边界（无纵向决策版本）
 *
 * @param obstacle 障碍物
 *
 * 功能说明：
 * - 用于没有纵向决策的障碍物
 * - 直接将障碍物映射到ST图
 *
 * C++语法说明：
 * - FLAGS_use_st_drivable_boundary:
 *   配置开关，是否使用ST可行驶边界
 */
void STBoundaryMapper::ComputeSTBoundary(Obstacle* obstacle) const {
  /**
   * @brief 检查配置开关
   */
  if (FLAGS_use_st_drivable_boundary) {
    return;
  }

  /**
   * @brief 创建上下边界点容器
   */
  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;

  /**
   * @brief 获取重叠边界点
   * GetOverlapBoundaryPoints():
   *   核心算法，计算障碍物与路径的重叠区域
   */
  if (!GetOverlapBoundaryPoints(path_data_.discretized_path(), *obstacle,
                                &upper_points, &lower_points)) {
    return;
  }

  /**
   * @brief 创建ST边界实例
   * STBoundary::CreateInstance():
   *   根据上下边界点创建边界对象
   */
  auto boundary = STBoundary::CreateInstance(lower_points, upper_points);
  boundary.set_id(obstacle->Id());

  /**
   * @brief 设置边界类型
   * 如果之前有边界类型，保留
   */
  const auto& prev_st_boundary = obstacle->path_st_boundary();
  const auto& ref_line_st_boundary = obstacle->reference_line_st_boundary();
  if (!prev_st_boundary.IsEmpty()) {
    boundary.SetBoundaryType(prev_st_boundary.boundary_type());
  } else if (!ref_line_st_boundary.IsEmpty()) {
    boundary.SetBoundaryType(ref_line_st_boundary.boundary_type());
  }

  /**
   * @brief 设置障碍物的ST边界
   */
  obstacle->set_path_st_boundary(boundary);
}

/**
 * @brief 获取重叠边界点（核心算法）
 *
 * @param path_points 路径点序列
 * @param obstacle 障碍物
 * @param upper_points 输出：上边界点
 * @param lower_points 输出：下边界点
 * @return bool 是否成功
 *
 * 核心算法流程：
 * 1. 静态障碍物（无轨迹）：
 *    - 遍历路径点，检查与障碍物边界框的重叠
 *    - 如果重叠，记录该点的S作为边界
 *
 * 2. 动态障碍物（有轨迹）：
 *    - 遍历预测轨迹的每个时间点
 *    - 检查自车与移动中障碍物的重叠
 *    - 记录各时刻的S上下界
 *
 * C++语法说明：
 * - std::vector<STPoint>*: 指针参数，输出结果
 * - const auto*: 指向常量的指针
 * - std::fmax/std::fmin: 浮点数极值函数
 */
bool STBoundaryMapper::GetOverlapBoundaryPoints(
    const std::vector<PathPoint>& path_points, const Obstacle& obstacle,
    std::vector<STPoint>* upper_points,
    std::vector<STPoint>* lower_points) const {
  /**
   * @brief 健全性检查
   * DCHECK: 仅DEBUG模式生效的断言
   * 检查输出参数是否为空
   */
  DCHECK(upper_points->empty());
  DCHECK(lower_points->empty());

  /**
   * @brief 检查路径点是否为空
   */
  if (path_points.empty()) {
    AERROR << "No points in path_data_.discretized_path().";
    return false;
  }

  /**
   * @brief 获取换道状态
   * 根据是否在换道中调整L方向缓冲区
   */
  const auto* planning_status = injector_->planning_context()
                                    ->mutable_planning_status()
                                    ->mutable_change_lane();

  /**
   * @brief 计算L方向缓冲区
   * lane_change_obstacle_nudge_l_buffer: 换道时的缓冲区
   * FLAGS_nonstatic_obstacle_nudge_l_buffer: 普通障碍物的缓冲区
   */
  double l_buffer =
      planning_status->status() == ChangeLaneStatus::IN_CHANGE_LANE
          ? speed_bounds_config_.lane_change_obstacle_nudge_l_buffer()  // 0.3
          : FLAGS_nonstatic_obstacle_nudge_l_buffer;   //  0.4

  /**
   * @brief 获取障碍物预测轨迹
   */
  const auto& trajectory = obstacle.Trajectory();

  /**
   * @brief 获取障碍物尺寸
   */
  const double obstacle_length = obstacle.Perception().length();
  const double obstacle_width = obstacle.Perception().width();


  PrintCurves print_path_collision;

  /**
   * @brief 处理静态障碍物（无预测轨迹）
   */
  if (trajectory.trajectory_point().empty()) {
    bool box_check_collision = false;

    /**
     * @brief 非静态障碍物无轨迹的警告
     */
    if (!obstacle.IsStatic()) {
      AWARN << "Non-static obstacle[" << obstacle.Id()
            << "] has NO prediction trajectory."
            << obstacle.Perception().ShortDebugString();
    }

    /**
     * @brief 获取障碍物边界框
     */
    const Box2d& obs_box = obstacle.PerceptionBoundingBox();

    /**
     * @brief 遍历路径点，检查碰撞
     */
    for (const auto& curr_point_on_path : path_points) {
      /**
       * @brief 超出规划距离则停止
       */
      if (curr_point_on_path.s() > planning_max_distance_) {
        break;
      }

      /**
       * @brief 检查是否重叠
       */
      if (CheckOverlap(curr_point_on_path, obs_box, l_buffer)) {
        box_check_collision = true;
        break;
      }
    }

    /**
     * @brief 如果有碰撞，计算边界
     */
    if (box_check_collision) {
      /**
       * @brief 计算前后扩展距离
       * backward_distance: 向后扩展（设为0）
       * forward_distance: 向前扩展（障碍物长度）
       */
      const double backward_distance = 0;
      const double forward_distance = obs_box.length();

      /**
       * @brief 再次遍历，找到精确的重叠边界
       */
      for (const auto& curr_point_on_path : path_points) {
        if (curr_point_on_path.s() > planning_max_distance_) {
          break;
        }

        /**
         * @brief 获取障碍物多边形
         */
        const Polygon2d& obs_polygon = obstacle.PerceptionPolygon();
        Polygon2d ego_collision_polygon;

        /**
         * @brief 检查多边形重叠
         */
        if (CheckOverlap(curr_point_on_path, obs_polygon, l_buffer, &ego_collision_polygon)) {
          /**
           * @brief 计算S边界
           */
          double low_s =
              std::fmax(0.0, curr_point_on_path.s() + backward_distance);
          double high_s = std::fmin(planning_max_distance_,
                                    curr_point_on_path.s() + forward_distance);

          /**
           * @brief 记录调试信息
           */
          AINFO << "check colllision for obstacle[" << obstacle.Id()
                << "], at: " << curr_point_on_path.DebugString();
          for (const auto& point : ego_collision_polygon.points()) {
            print_path_collision.AddPoint(obstacle.Id() + "_collision_path_point",
                                        point.x(), point.y());
          }
          print_path_collision.AddPoint(obstacle.Id() + "_collision_path_point",
              ego_collision_polygon.points().front().x(),
              ego_collision_polygon.points().front().y());

          /**
           * @brief 添加ST边界点
           * point_extension: 点扩展距离
           * 创建T=0和T=planning_max_time_的两条线
           */
          lower_points->emplace_back(low_s - speed_bounds_config_.point_extension(), 0.0);
          lower_points->emplace_back(low_s - speed_bounds_config_.point_extension(), planning_max_time_);
          upper_points->emplace_back(high_s + speed_bounds_config_.point_extension(), 0.0);
          upper_points->emplace_back(high_s + speed_bounds_config_.point_extension(), planning_max_time_);
          break;
        }
      }
      print_path_collision.PrintToLog();
    }
  }

  /**
   * @brief 处理动态障碍物（有预测轨迹）
   */
  else {
    /**
     * @brief 步骤1：降采样减少计算量
     * default_num_point: 默认采样点数（50）
     */
    const int default_num_point = 50;
    DiscretizedPath discretized_path;

    /**
     * @brief 如果路径点太多，进行降采样
     */
    if (path_points.size() > 2 * default_num_point) {
      const auto ratio = path_points.size() / default_num_point;
      std::vector<PathPoint> sampled_path_points;

      /**
       * @brief 等间隔采样
       */
      for (size_t i = 0; i < path_points.size(); ++i) {
        if (i % ratio == 0) {
          sampled_path_points.push_back(path_points[i]);
        }
      }
      discretized_path = DiscretizedPath(std::move(sampled_path_points));
    } else {
      discretized_path = DiscretizedPath(path_points);
    }

    /**
     * @brief 步骤2：遍历预测轨迹的每个时间点
     */
    double trajectory_time_interval =
        obstacle.Trajectory().trajectory_point()[1].relative_time();

    /**
     * @brief 计算轨迹步长
     * 取FLAGS_trajectory_check_collision_time_step和计算值的较小者
     */
    int trajectory_step =
        std::min(FLAGS_trajectory_check_collision_time_step,
                 std::max(vehicle_param_.width() / obstacle.speed() /
                              trajectory_time_interval,
                          1.0));

    bool trajectory_point_collision_status = false;
    int previous_index = 0;

    /**
     * @brief 遍历轨迹点
     */
    for (int i = 0; i < trajectory.trajectory_point_size();
         i = std::min(i + trajectory_step,
                      trajectory.trajectory_point_size() - 1)) {
      /**
       * @brief 获取轨迹点
       */
      const auto& trajectory_point = trajectory.trajectory_point(i);

      /**
       * @brief 获取移动中的障碍物形状
       * GetObstacleTrajectoryPolygon():
       *   根据轨迹点计算旋转后的障碍物多边形
       */
      Polygon2d obstacle_shape =
          obstacle.GetObstacleTrajectoryPolygon(trajectory_point);

      /**
       * @brief 获取时间
       */
      double trajectory_point_time = trajectory_point.relative_time();

      /**
       * @brief 跳过负时间点（历史轨迹）
       */
      static constexpr double kNegtiveTimeThreshold = -1.0;
      if (trajectory_point_time < kNegtiveTimeThreshold) {
        continue;
      }

      /**
       * @brief 检查与自车的重叠
       */
      bool collision = CheckOverlapWithTrajectoryPoint(
          discretized_path, obstacle_shape, upper_points, lower_points,
          l_buffer, default_num_point, obstacle_length, obstacle_width,
          trajectory_point_time);

      /**
       * @brief 检测碰撞状态变化（进入/离开碰撞区域）
       */
      if ((trajectory_point_collision_status ^ collision) && i != 0) {
        /**
         * @brief 开始回溯轨迹点
         */
        int index = i - 1;
        while ((trajectory_point_collision_status ^ collision) &&
               index > previous_index) {
          const auto& point = trajectory.trajectory_point(index);
          trajectory_point_time = point.relative_time();
          obstacle_shape = obstacle.GetObstacleTrajectoryPolygon(point);
          collision = CheckOverlapWithTrajectoryPoint(
              discretized_path, obstacle_shape, upper_points, lower_points,
              l_buffer, default_num_point, obstacle_length, obstacle_width,
              trajectory_point_time);
          index--;
        }
        trajectory_point_collision_status = !trajectory_point_collision_status;
      }

      /**
       * @brief 如果到达最后一个点，停止
       */
      if (i == trajectory.trajectory_point_size() - 1) break;
      previous_index = i;
    }
  }

  /**
   * @brief 步骤3：排序边界点并返回
   * 按时间T排序
   */
  std::sort(lower_points->begin(), lower_points->end(),
            [](const STPoint& a, const STPoint& b) { return a.t() < b.t(); });
  std::sort(upper_points->begin(), upper_points->end(),
            [](const STPoint& a, const STPoint& b) { return a.t() < b.t(); });

  /**
   * @brief 检查边界点数量
   */
  DCHECK_EQ(lower_points->size(), upper_points->size());

  /**
   * @brief 返回成功条件：上下边界都至少有2个点
   */
  return (lower_points->size() > 1 && upper_points->size() > 1);
}

/**
 * @brief 检查轨迹点与路径的重叠（动态障碍物）
 *
 * @param discretized_path 离散化的路径
 * @param obstacle_shape 障碍物形状（多边形）
 * @param upper_points 输出：上边界点
 * @param lower_points 输出：下边界点
 * @param l_buffer L方向缓冲区
 * @param default_num_point 默认采样点数
 * @param obstacle_length 障碍物长度
 * @param obstacle_width 障碍物宽度
 * @param trajectory_point_time 轨迹点时间
 * @return bool 是否发现重叠
 *
 * 核心算法：
 * 1. 遍历路径上的点
 * 2. 如果发现重叠，高精度搜索边界
 * 3. 双向扫描精确定位S上下界
 *
 * C++语法说明：
 * - std::fmin/std::fmax: 浮点数极值函数
 * - while循环: 双向扫描直到找到精确边界
 */
bool STBoundaryMapper::CheckOverlapWithTrajectoryPoint(
    const DiscretizedPath& discretized_path, const Polygon2d& obstacle_shape,
    std::vector<STPoint>* upper_points, std::vector<STPoint>* lower_points,
    const double l_buffer, int default_num_point, const double obstacle_length,
    const double obstacle_width, const double trajectory_point_time) const {
  /**
   * @brief 计算路径步长
   * front_edge_to_center: 前轴到车辆中心的距离
   */
  const double step_length = vehicle_param_.front_edge_to_center();

  /**
   * @brief 计算路径长度
   */
  auto path_len = std::min(speed_bounds_config_.max_trajectory_len(),
                           discretized_path.Length());

  /**
   * @brief 遍历路径上的点
   */
  for (double path_s = 0.0; path_s < path_len; path_s += step_length) {
    /**
     * @brief 获取路径点
     */
    const auto curr_adc_path_point =
        discretized_path.Evaluate(path_s + discretized_path.front().s());

    /**
     * @brief 检查重叠
     */
    if (CheckOverlap(curr_adc_path_point, obstacle_shape, l_buffer)) {
      /**
       * @brief 发现重叠，开始高精度搜索
       */

      /**
       * @brief 计算前后扩展距离
       */
      const double backward_distance = 0.0;
      const double forward_distance = vehicle_param_.length() +
                                      vehicle_param_.width() + obstacle_length +
                                      obstacle_width;

      /**
       * @brief 精细调整步长
       * 取0.1米和路径长度/采样点数的较小者
       */
      const double default_min_step = 0.1;  // in meters
      const double fine_tuning_step_length = std::fmin(
          default_min_step, discretized_path.Length() / default_num_point);

      bool find_low = false;
      bool find_high = false;
      double low_s = std::fmax(0.0, path_s + backward_distance);
      double high_s =
          std::fmin(discretized_path.Length(), path_s + forward_distance);

      /**
       * @brief 双向扫描精确定位边界
       * while循环：直到找到上下边界
       */
      while (low_s < high_s) {
        if (find_low && find_high) {
          break;
        }

        /**
         * @brief 搜索下边界
         */
        if (!find_low) {
          const auto& point_low =
              discretized_path.Evaluate(low_s + discretized_path.front().s());
          if (!CheckOverlap(point_low, obstacle_shape, l_buffer)) {
            low_s += fine_tuning_step_length;
          } else {
            find_low = true;
          }
        }

        /**
         * @brief 搜索上边界
         */
        if (!find_high) {
          const auto& point_high =
              discretized_path.Evaluate(high_s + discretized_path.front().s());
          if (!CheckOverlap(point_high, obstacle_shape, l_buffer)) {
            high_s -= fine_tuning_step_length;
          } else {
            find_high = true;
          }
        }
      }

      /**
       * @brief 如果找到有效边界，添加点
       */
      if (find_high && find_low) {
        lower_points->emplace_back(
            low_s - speed_bounds_config_.point_extension(),
            trajectory_point_time);
        upper_points->emplace_back(
            high_s + speed_bounds_config_.point_extension(),
            trajectory_point_time);
      }
      return true;
    }
  }
  return false;
}

/**
 * @brief 根据纵向决策计算ST边界
 *
 * @param obstacle 障碍物
 * @param decision 纵向决策
 *
 * 功能说明：
 * - FOLLOW: 扩展下边界
 * - YIELD: 扩展下边界
 * - OVERTAKE: 不扩展，保持原样
 *
 * C++语法说明：
 * - std::fabs: 浮点数绝对值
 */
void STBoundaryMapper::ComputeSTBoundaryWithDecision(
    Obstacle* obstacle, const ObjectDecisionType& decision) const {
  /**
   * @brief 断言检查
   */
  DCHECK(decision.has_follow() || decision.has_yield() ||
         decision.has_overtake())
      << "decision is " << decision.DebugString()
      << ", but it must be follow or yield or overtake.";

  /**
   * @brief 获取边界点
   */
  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;

  /**
   * @brief 检查是否使用ST可行驶边界
   */
  if (FLAGS_use_st_drivable_boundary &&
      obstacle->is_path_st_boundary_initialized()) {
    /**
     * @brief 使用已有的ST边界
     */
    const auto& path_st_boundary = obstacle->path_st_boundary();
    lower_points = path_st_boundary.lower_points();
    upper_points = path_st_boundary.upper_points();
  } else {
    /**
     * @brief 重新计算边界
     */
    if (!GetOverlapBoundaryPoints(path_data_.discretized_path(), *obstacle,
                                  &upper_points, &lower_points)) {
      return;
    }
  }

  /**
   * @brief 创建边界
   */
  auto boundary = STBoundary::CreateInstance(lower_points, upper_points);

  /**
   * @brief 获取特征长度和边界类型
   * characteristic_length: 决策距离
   */
  STBoundary::BoundaryType b_type = STBoundary::BoundaryType::UNKNOWN;
  double characteristic_length = 0.0;

  /**
   * @brief FOLLOW决策
   */
  if (decision.has_follow()) {
    characteristic_length = std::fabs(decision.follow().distance_s());
    AINFO << "characteristic_length: " << characteristic_length;
    boundary = STBoundary::CreateInstance(lower_points, upper_points)
                   .ExpandByS(characteristic_length);
    b_type = STBoundary::BoundaryType::FOLLOW;
  }

  /**
   * @brief YIELD决策
   */
  else if (decision.has_yield()) {
    characteristic_length = std::fabs(decision.yield().distance_s());
    boundary = STBoundary::CreateInstance(lower_points, upper_points)
                   .ExpandByS(characteristic_length);
    b_type = STBoundary::BoundaryType::YIELD;
  }

  /**
   * @brief OVERTAKE决策
   */
  else if (decision.has_overtake()) {
    characteristic_length = std::fabs(decision.overtake().distance_s());
    b_type = STBoundary::BoundaryType::OVERTAKE;
  }

  /**
   * @brief 设置边界属性
   */
  boundary.SetBoundaryType(b_type);
  boundary.set_id(obstacle->Id());
  boundary.SetCharacteristicLength(characteristic_length);
  obstacle->set_path_st_boundary(boundary);
}

/**
 * @brief 检查路径点与边界框的重叠（Box2d版本）
 *
 * @param path_point 路径点
 * @param obs_box 障碍物边界框
 * @param l_buffer L方向缓冲区
 * @return bool 是否重叠
 *
 * 算法流程：
 * 1. 将路径点从后轴中心转换到ADC中心
 * 2. 构建ADC边界框
 * 3. 检查ADC边界框与障碍物边界框的重叠
 *
 * C++语法说明：
 * - Vec2d: 2D向量
 * - SelfRotate: 旋转向量
 * - Box2d: 2D边界框
 */
bool STBoundaryMapper::CheckOverlap(const PathPoint& path_point,
                                    const Box2d& obs_box,
                                    const double l_buffer) const {
  /**
   * @brief 将路径点从后轴中心转换到ADC中心
   * 计算前后轴中心的中点
   */
  Vec2d ego_center_map_frame((vehicle_param_.front_edge_to_center() -
                              vehicle_param_.back_edge_to_center()) *
                                 0.5,
                             (vehicle_param_.left_edge_to_center() -
                              vehicle_param_.right_edge_to_center()) *
                                 0.5);

  /**
   * @brief 旋转变换
   * SelfRotate: 就地旋转
   */
  ego_center_map_frame.SelfRotate(path_point.theta());

  /**
   * @brief 平移变换
   */
  ego_center_map_frame.set_x(ego_center_map_frame.x() + path_point.x());
  ego_center_map_frame.set_y(ego_center_map_frame.y() + path_point.y());

  /**
   * @brief 构建ADC边界框
   * Box2d: 以(ego_center_map_frame, heading, length, width)构造
   */
  Box2d adc_box(ego_center_map_frame, path_point.theta(),
                vehicle_param_.length(), vehicle_param_.width() + l_buffer * 2);

  /**
   * @brief 检查重叠
   * HasOverlap: 判断两个边界框是否重叠
   */
  return obs_box.HasOverlap(adc_box);
}

/**
 * @brief 检查路径点与多边形的重叠（Polygon2d版本）
 *
 * @param path_point 路径点
 * @param obs_polygon 障碍物多边形
 * @param l_buffer L方向缓冲区
 * @param collision_ego_polygon 输出：重叠时的ADC多边形
 * @return bool 是否重叠
 *
 * 与Box2d版本区别：
 * - 使用多边形而非边界框
 * - 可输出重叠时的ADC多边形
 */
bool STBoundaryMapper::CheckOverlap(const PathPoint& path_point,
                                    const Polygon2d& obs_polygon,
                                    const double l_buffer,
                                    Polygon2d* collision_ego_polygon) const {
  /**
   * @brief 转换到ADC中心
   */
  Vec2d ego_center_map_frame((vehicle_param_.front_edge_to_center() -
                              vehicle_param_.back_edge_to_center()) *
                                 0.5,
                             (vehicle_param_.left_edge_to_center() -
                              vehicle_param_.right_edge_to_center()) *
                                 0.5);
  ego_center_map_frame.SelfRotate(path_point.theta());
  ego_center_map_frame.set_x(ego_center_map_frame.x() + path_point.x());
  ego_center_map_frame.set_y(ego_center_map_frame.y() + path_point.y());

  /**
   * @brief 构建ADC边界框
   */
  Box2d adc_box(ego_center_map_frame, path_point.theta(),
                vehicle_param_.length(), vehicle_param_.width() + l_buffer * 2);

  /**
   * @brief 转换为ADC多边形
   */
  Polygon2d adc_polygon(adc_box);

  /**
   * @brief 检查多边形重叠
   */
  if (obs_polygon.HasOverlap(adc_polygon)) {
    if (collision_ego_polygon != nullptr) {
      *collision_ego_polygon = adc_polygon;
    }
    return true;
  } else {
    return false;
  }
}

/**
 * @brief 命名空间结束标记
 */
}  // namespace planning
}  // namespace apollo
