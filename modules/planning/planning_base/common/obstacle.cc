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
 * @file obstacle.cc
 * @brief 障碍物类实现文件
 *
 * 本文件实现了Obstacle类，表示规划模块中的障碍物对象。
 *
 * 核心功能：
 * 1. 障碍物数据结构管理（位置、速度、边界框、多边形）
 * 2. 障碍物决策管理（纵向决策、横向决策）
 * 3. ST边界构建（时空障碍区域）
 * 4. SL边界管理（障碍物在参考线上的投影）
 *
 * 障碍物类型：
 * - 静态障碍物：位置固定，如停放的车辆
 * - 动态障碍物：有预测轨迹，如行驶的车辆
 * - 虚拟障碍物：用于测试或特殊用途
 *
 * C++语法说明：
 * - std::unique_ptr<T>: 独占所有权的智能指针
 * - std::list<T>: 双向链表容器
 * - protobuf消息: PerceptionObstacle, Trajectory等
 * - mutable_xxx(): protobuf消息的可修改访问器
 */

#include "modules/planning/planning_base/common/obstacle.h"

/**
 * @brief 标准库头文件
 * <algorithm>: 提供std::max, std::min, std::fmax, std::fabs等算法
 * <iomanip>: 提供输入输出格式化（如setprecision）
 * <utility>: 提供std::pair, std::move等工具
 */
#include <algorithm>
#include <iomanip>
#include <utility>

#include "cyber/common/log.h"
/**
 * @brief Cyber RT日志系统
 * AERROR, ADEBUG, ACHECK等日志宏
 */
#include "modules/common/configs/vehicle_config_helper.h"
/**
 * @brief 车辆配置辅助类
 */
#include "modules/common/math/linear_interpolation.h"
/**
 * @brief 线性插值数学工具
 */
#include "modules/common/util/map_util.h"
/**
 * @brief 地图工具函数
 */
#include "modules/common/util/util.h"
/**
 * @brief 通用工具函数
 */
#include "modules/planning/planning_base/common/speed/st_boundary.h"
/**
 * @brief ST边界定义
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
 */
using apollo::common::VehicleConfigHelper;
using apollo::common::util::FindOrDie;
/**
 * @brief FindOrDie: map查找工具，如果找不到则abort
 */
using apollo::perception::PerceptionObstacle;
using apollo::prediction::ObstaclePriority;

/**
 * @brief 匿名命名空间
 * 定义文件范围内的常量
 */
namespace {
/**
 * @brief ST边界参数常量
 * kStBoundaryDeltaS: 密集采样的S方向间距（0.2米）
 * kStBoundarySparseDeltaS: 稀疏采样的S方向间距（1.0米）
 * kStBoundaryDeltaT: T方向的时间间距（0.05秒）
 */
const double kStBoundaryDeltaS = 0.2;        // meters
const double kStBoundarySparseDeltaS = 1.0;  // meters
const double kStBoundaryDeltaT = 0.05;       // seconds
}  // namespace

/**
 * @brief 纵向决策安全排序器
 *
 * 用于合并纵向决策时判断优先级
 * 数值越大优先级越高
 *
 * 排序：
 * - IGNORE (0): 忽略，优先级最低
 * - OVERTAKE (100): 超车
 * - FOLLOW (300): 跟随
 * - YIELD (400): 让行
 * - STOP (500): 停车，优先级最高
 */
const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                         Obstacle::ObjectTagCaseHash>
    Obstacle::s_longitudinal_decision_safety_sorter_ = {
        {ObjectDecisionType::kIgnore, 0},
        {ObjectDecisionType::kOvertake, 100},
        {ObjectDecisionType::kFollow, 300},
        {ObjectDecisionType::kYield, 400},
        {ObjectDecisionType::kStop, 500}};

/**
 * @brief 横向决策安全排序器
 *
 * 用于合并横向决策时判断优先级
 * - IGNORE (0): 忽略
 * - NUDGE (100): 绕行
 */
const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                         Obstacle::ObjectTagCaseHash>
    Obstacle::s_lateral_decision_safety_sorter_ = {
        {ObjectDecisionType::kIgnore, 0}, {ObjectDecisionType::kNudge, 100}};

/**
 * @brief Obstacle构造函数
 *
 * @param id 障碍物ID
 * @param perception_obstacle 感知障碍物信息
 * @param obstacle_priority 障碍物优先级
 * @param is_static 是否为静态障碍物
 *
 * 初始化流程：
 * 1. 初始化基础成员变量
 * 2. 创建感知边界框
 * 3. 根据配置决定多边形顶点来源
 * 4. 计算凸包
 * 5. 判断静态/虚拟属性
 * 6. 计算速度
 *
 * C++语法说明：
 * - 初始化列表: id_(id), perception_id_(...), perception_obstacle_(...)
 *   直接初始化成员变量，比在函数体中赋值更高效
 * - {x, y}: 大括号初始化器，用于Vec2d构造
 */
Obstacle::Obstacle(const std::string& id,
                   const PerceptionObstacle& perception_obstacle,
                   const ObstaclePriority::Priority& obstacle_priority,
                   const bool is_static)
    : id_(id),
      perception_id_(perception_obstacle.id()),
      perception_obstacle_(perception_obstacle),
      perception_bounding_box_({perception_obstacle_.position().x(),
                                perception_obstacle_.position().y()},
                               perception_obstacle_.theta(),
                               perception_obstacle_.length(),
                               perception_obstacle_.width()) {
  /**
   * @brief 判断是否为危险等级障碍物
   */
  is_caution_level_obstacle_ = (obstacle_priority == ObstaclePriority::CAUTION);

  /**
   * @brief 多边形顶点容器
   */
  std::vector<common::math::Vec2d> polygon_points;

  /**
   * @brief 根据配置决定多边形来源
   * FLAGS_use_navigation_mode: 是否使用导航模式
   * polygon_point_size <= 2: 感知多边形顶点数少于3
   * 两种情况都使用边界框的四个角点
   */
  if (FLAGS_use_navigation_mode ||
      perception_obstacle.polygon_point_size() <= 2) {
    perception_bounding_box_.GetAllCorners(&polygon_points);
  } else {
    /**
     * @brief 使用感知提供的多边形顶点
     */
    ACHECK(perception_obstacle.polygon_point_size() > 2)
        << "object " << id << "has less than 3 polygon points";
    for (const auto& point : perception_obstacle.polygon_point()) {
      polygon_points.emplace_back(point.x(), point.y());
    }
  }

  /**
   * @brief 计算凸包
   * ComputeConvexHull: 将任意多边形转换为凸包
   * 用于简化碰撞检测计算
   */
  ACHECK(common::math::Polygon2d::ComputeConvexHull(polygon_points,
                                                    &perception_polygon_))
      << "object[" << id << "] polygon is not a valid convex hull.\n"
      << perception_obstacle.DebugString();

  /**
   * @brief 判断静态属性
   * is_static = 传入参数 || 优先级为IGNORE
   */
  is_static_ = (is_static || obstacle_priority == ObstaclePriority::IGNORE);

  /**
   * @brief 判断虚拟属性
   * id < 0 为虚拟障碍物（用于仿真器测试）
   */
  is_virtual_ = (perception_obstacle.id() < 0);

  /**
   * @brief 计算速度大小
   * std::hypot(x, y) = sqrt(x² + y²)，比直接计算更数值稳定
   */
  speed_ = std::hypot(perception_obstacle.velocity().x(),
                      perception_obstacle.velocity().y());
}

/**
 * @brief Obstacle构造函数（带预测轨迹）
 *
 * @param id 障碍物ID
 * @param perception_obstacle 感知障碍物信息
 * @param trajectory 预测轨迹
 * @param obstacle_priority 障碍物优先级
 * @param is_static 是否为静态障碍物
 *
 * 使用委托构造调用上面的构造函数
 * 然后额外处理轨迹信息
 *
 * C++语法说明：
 * : Obstacle(id, perception_obstacle, obstacle_priority, is_static)
 *   委托构造，调用另一个构造函数
 */
Obstacle::Obstacle(const std::string& id,
                   const PerceptionObstacle& perception_obstacle,
                   const prediction::Trajectory& trajectory,
                   const ObstaclePriority::Priority& obstacle_priority,
                   const bool is_static)
    : Obstacle(id, perception_obstacle, obstacle_priority, is_static) {  // 委托构造
  /**
   * @brief 保存轨迹副本
   */
  trajectory_ = trajectory;

  /**
   * @brief 获取可修改的轨迹点序列
   */
  auto& trajectory_points = *trajectory_.mutable_trajectory_point();

  /**
   * @brief 累积S距离
   */
  double cumulative_s = 0.0;

  /**
   * @brief 第一个点s=0
   */
  if (trajectory_points.size() > 0) {
    trajectory_points[0].mutable_path_point()->set_s(0.0);
  }

  /**
   * @brief 遍历轨迹点，计算累积距离
   */
  for (int i = 1; i < trajectory_points.size(); ++i) {
    const auto& prev = trajectory_points[i - 1];
    const auto& cur = trajectory_points[i];

    /**
     * @brief 检查时间是否递增
     */
    if (prev.relative_time() >= cur.relative_time()) {
      AERROR << "prediction time is not increasing."
             << "current point: " << cur.ShortDebugString()
             << "previous point: " << prev.ShortDebugString();
    }

    /**
     * @brief 计算相邻点XY距离并累加
     */
    // 计算累计弧长
    // 这个s值后续用于:
    // ST边界计算: 确定障碍物在不同时刻覆盖的s范围
    // 时间插值: GetPointAtTime() 查找任意时刻的位置
    cumulative_s +=
        common::util::DistanceXY(prev.path_point(), cur.path_point());

    /**
     * @brief 设置当前点的累积距离s
     */
    trajectory_points[i].mutable_path_point()->set_s(cumulative_s);
  }
}

/**
 * @brief 获取指定时间的轨迹点
 *
 * @param relative_time 相对时间（秒）
 * @return TrajectoryPoint 插值后的轨迹点
 *
 * 使用线性插值计算指定时间的轨迹点位置
 *
 * C++语法说明：
 * - std::lower_bound: 二分查找，返回第一个不小于给定值的迭代器
 * - lambda表达式: comp作为比较函数
 * - auto it_lower = std::lower_bound(...): 自动类型推导
 */
common::TrajectoryPoint Obstacle::GetPointAtTime(
    const double relative_time) const {
  const auto& points = trajectory_.trajectory_point();

  /**
   * @brief 如果轨迹点少于2个，返回当前位置
   */
  if (points.size() < 2) {
    common::TrajectoryPoint point;

    /**
     * @brief 设置路径点坐标
     */
    point.mutable_path_point()->set_x(perception_obstacle_.position().x());
    point.mutable_path_point()->set_y(perception_obstacle_.position().y());
    point.mutable_path_point()->set_z(perception_obstacle_.position().z());
    point.mutable_path_point()->set_theta(perception_obstacle_.theta());
    point.mutable_path_point()->set_s(0.0);
    point.mutable_path_point()->set_kappa(0.0);
    point.mutable_path_point()->set_dkappa(0.0);
    point.mutable_path_point()->set_ddkappa(0.0);
    point.set_v(0.0);
    point.set_a(0.0);
    point.set_relative_time(0.0);
    return point;
  } else {
    /**
     * @brief 创建比较函数对象
     * 比较轨迹点的relative_time与给定时间
     */
    auto comp = [](const common::TrajectoryPoint p, const double time) {
      return p.relative_time() < time;
    };

    /**
     * @brief 二分查找定位时间点
     * lower_bound返回第一个>=relative_time的位置
     */
    auto it_lower =
        std::lower_bound(points.begin(), points.end(), relative_time, comp);

    if (it_lower == points.begin()) {
      /**
       * @brief 如果在第一个点之前，返回第一个点
       */
      return *points.begin();
    } else if (it_lower == points.end()) {
      /**
       * @brief 如果在最后一个点之后，返回最后一个点
       * rbegin(): 反向迭代器的起始位置
       */
      return *points.rbegin();
    }

    /**
     * @brief 线性插值计算
     * InterpolateUsingLinearApproximation:
     *   在(it_lower-1)和it_lower之间线性插值
     */
    return common::math::InterpolateUsingLinearApproximation(
        *(it_lower - 1), *it_lower, relative_time);
  }
}

/**
 * @brief 获取指定轨迹点的边界框
 *
 * @param point 轨迹点
 * @return Box2d 障碍物边界框
 *
 * 创建一个以轨迹点位置为中心的边界框
 */
common::math::Box2d Obstacle::GetBoundingBox(
    const common::TrajectoryPoint& point) const {
  return common::math::Box2d({point.path_point().x(), point.path_point().y()},
                             point.path_point().theta(),
                             perception_obstacle_.length(),
                             perception_obstacle_.width());
}

/**
 * @brief 验证感知障碍物是否有效
 *
 * @param obstacle 感知障碍物
 * @return bool 是否有效
 *
 * 检查项：
 * 1. 长宽高必须为正
 * 2. 如果有速度，x和y不能是NaN
 * 3. 多边形顶点坐标不能是NaN
 */
bool Obstacle::IsValidPerceptionObstacle(const PerceptionObstacle& obstacle) {
  /**
   * @brief 检查长度
   */
  if (obstacle.length() <= 0.0) {
    AERROR << "invalid obstacle length:" << obstacle.length();
    return false;
  }

  /**
   * @brief 检查宽度
   */
  if (obstacle.width() <= 0.0) {
    AERROR << "invalid obstacle width:" << obstacle.width();
    return false;
  }

  /**
   * @brief 检查高度
   */
  if (obstacle.height() <= 0.0) {
    AERROR << "invalid obstacle height:" << obstacle.height();
    return false;
  }

  /**
   * @brief 检查速度
   */
  if (obstacle.has_velocity()) {
    if (std::isnan(obstacle.velocity().x()) ||
        std::isnan(obstacle.velocity().y())) {
      AERROR << "invalid obstacle velocity:"
             << obstacle.velocity().DebugString();
      return false;
    }
  }

  /**
   * @brief 检查多边形顶点
   */
  for (auto pt : obstacle.polygon_point()) {
    if (std::isnan(pt.x()) || std::isnan(pt.y())) {
      AERROR << "invalid obstacle polygon point:" << pt.DebugString();
      return false;
    }
  }
  return true;
}

/**
 * @brief 从预测结果创建障碍物列表
 *
 * @param predictions 预测障碍物列表
 * @return std::list<std::unique_ptr<Obstacle>> 障碍物列表
 *
 * 工厂函数：
 * - 遍历每个预测障碍物
 * - 验证感知障碍物
 * - 如果没有轨迹，创建静态障碍物
 * - 如果有轨迹，为每条轨迹创建动态障碍物
 *
 * C++语法说明：
 * - std::list<std::unique_ptr<Obstacle>>:
 *   存储unique_ptr的链表
 * - std::unique_ptr: 独占所有权的智能指针
 * - emplace_back(new Obstacle(...)): 直接构造避免拷贝
 */
std::list<std::unique_ptr<Obstacle>> Obstacle::CreateObstacles(
    const prediction::PredictionObstacles& predictions) {
  std::list<std::unique_ptr<Obstacle>> obstacles;

  /**
   * @brief 遍历每个预测障碍物
   */
  for (const auto& prediction_obstacle : predictions.prediction_obstacle()) {
    /**
     * @brief 验证感知障碍物有效性
     */
    if (!IsValidPerceptionObstacle(prediction_obstacle.perception_obstacle())) {
      AERROR << "Invalid perception obstacle: "
             << prediction_obstacle.perception_obstacle().DebugString();
      continue;  // 跳过无效障碍物
    }
    
    // 分支A：无预测轨迹--> 创建静态障碍物
    /**
     * @brief 转换感知ID为字符串
     */
    const auto perception_id =
        std::to_string(prediction_obstacle.perception_obstacle().id());

    /**
     * @brief 如果没有预测轨迹
     */
    if (prediction_obstacle.trajectory().empty()) {
      obstacles.emplace_back(
          new Obstacle(perception_id, prediction_obstacle.perception_obstacle(),
                       prediction_obstacle.priority().priority(),
                       prediction_obstacle.is_static()));
      continue;
    }
    
    // 分支B：有预测轨迹 --> 每条轨迹创建独立障碍物
    /**
     * @brief 遍历每条预测轨迹
     */
    // 预测模块对同一个障碍物可能给出多条预测轨迹(不同的运动意图)
    int trajectory_index = 0;
    for (const auto& trajectory : prediction_obstacle.trajectory()) {
      /**
       * @brief 验证轨迹有效性
       */
      bool is_valid_trajectory = true;
      for (const auto& point : trajectory.trajectory_point()) {
        if (!IsValidTrajectoryPoint(point)) {
          AERROR << "obj:" << perception_id
                 << " TrajectoryPoint: " << trajectory.ShortDebugString()
                 << " is NOT valid.";
          is_valid_trajectory = false;
          break;
        }
      }
      if (!is_valid_trajectory) {
        continue;
      }

      /**
       * @brief 创建带轨迹的障碍物ID
       * 格式: perception_id_trajectory_index
       */
      // 构造带轨迹后缀的ID
      const std::string obstacle_id =
          absl::StrCat(perception_id, "_", trajectory_index);
      obstacles.emplace_back(
          new Obstacle(obstacle_id, prediction_obstacle.perception_obstacle(),
                       trajectory, prediction_obstacle.priority().priority(),
                       prediction_obstacle.is_static()));
      ++trajectory_index;
    }
  }
  // std::vector	连续内存，emplace_back 可能触发重分配和移动，但 unique_ptr 不可复制
  // std::list	节点式分配，emplace_back 不涉及重分配，unique_ptr 只需移动
  return obstacles;
}

/**
 * @brief 创建静态虚拟障碍物
 *
 * @param id 障碍物ID
 * @param obstacle_box 障碍物边界框
 * @return std::unique_ptr<Obstacle> 创建的障碍物智能指针
 *
 * 用于创建测试用的虚拟障碍物，如停车墙
 *
 * C++语法说明：
 * - std::hash<std::string>: 字符串哈希函数
 * - |= (0x1 << 31): 将第一位设为1，使ID变为负数
 */
std::unique_ptr<Obstacle> Obstacle::CreateStaticVirtualObstacles(
    const std::string& id, const common::math::Box2d& obstacle_box) {
  /**
   * @brief 创建虚拟感知障碍物
   */
  perception::PerceptionObstacle perception_obstacle;

  /**
   * @brief 生成负数ID
   * 仿真器需要有效的整数ID
   */
  size_t negative_id = std::hash<std::string>{}(id);
  negative_id |= (0x1 << 31);  /**< 设置符号位 */
  perception_obstacle.set_id(static_cast<int32_t>(negative_id));

  /**
   * @brief 设置位置和方向
   */
  perception_obstacle.mutable_position()->set_x(obstacle_box.center().x());
  perception_obstacle.mutable_position()->set_y(obstacle_box.center().y());
  perception_obstacle.set_theta(obstacle_box.heading());

  /**
   * @brief 设置零速度
   */
  perception_obstacle.mutable_velocity()->set_x(0);
  perception_obstacle.mutable_velocity()->set_y(0);

  /**
   * @brief 设置尺寸
   */
  perception_obstacle.set_length(obstacle_box.length());
  perception_obstacle.set_width(obstacle_box.width());
  perception_obstacle.set_height(FLAGS_virtual_stop_wall_height);
  perception_obstacle.set_type(
      perception::PerceptionObstacle::UNKNOWN_UNMOVABLE);
  perception_obstacle.set_tracking_time(1.0);

  /**
   * @brief 设置多边形顶点
   */
  std::vector<common::math::Vec2d> corner_points;
  obstacle_box.GetAllCorners(&corner_points);
  for (const auto& corner_point : corner_points) {
    auto* point = perception_obstacle.add_polygon_point();
    point->set_x(corner_point.x());
    point->set_y(corner_point.y());
  }

  /**
   * @brief 创建障碍物并设置虚拟标志
   */
  auto* obstacle =
      new Obstacle(id, perception_obstacle, ObstaclePriority::NORMAL, true);
  obstacle->is_virtual_ = true;
  return std::unique_ptr<Obstacle>(obstacle);
}

/**
 * @brief 验证轨迹点是否有效
 *
 * @param point 轨迹点
 * @return bool 是否有效
 *
 * 检查所有数值字段是否为NaN
 */
bool Obstacle::IsValidTrajectoryPoint(const common::TrajectoryPoint& point) {
  return !((!point.has_path_point()) || std::isnan(point.path_point().x()) ||
           std::isnan(point.path_point().y()) ||
           std::isnan(point.path_point().z()) ||
           std::isnan(point.path_point().kappa()) ||
           std::isnan(point.path_point().s()) ||
           std::isnan(point.path_point().dkappa()) ||
           std::isnan(point.path_point().ddkappa()) || std::isnan(point.v()) ||
           std::isnan(point.a()) || std::isnan(point.relative_time()));
}

/**
 * @brief 设置感知SL边界
 *
 * @param sl_boundary SL边界
 */
void Obstacle::SetPerceptionSlBoundary(const SLBoundary& sl_boundary) {
  sl_boundary_ = sl_boundary;
}

/**
 * @brief 计算最小停车距离
 *
 * @param vehicle_param 车辆参数
 * @return double 最小停车距离
 *
 * 公式推导：
 * - 最小转弯半径 = wheel_base / tan(max_steer_angle)
 * - lateral_diff = 车辆半宽 + max(|start_l|, |end_l|)
 * - stop_distance = sqrt(min_turn_radius² - (min_turn_radius - lateral_diff)²) + buffer
 *
 * 物理意义：
 * - 车辆在转弯时需要更大的停车距离
 * - 障碍物越靠近道路边缘，需要的停车距离越大
 */
double Obstacle::MinRadiusStopDistance(
    const common::VehicleParam& vehicle_param) const {
  /**
   * @brief 如果已计算过，直接返回缓存值
   */
  if (min_radius_stop_distance_ > 0) {
    return min_radius_stop_distance_;
  }

  static constexpr double stop_distance_buffer = 0.5;  /**< 停车距离缓冲区 */
  const double min_turn_radius = VehicleConfigHelper::MinSafeTurnRadius();

  /**
   * @brief 计算横向距离差
   * 车辆半宽 + 障碍物在L方向的最大偏移
   */
  double lateral_diff =
      vehicle_param.width() / 2.0 + std::max(std::fabs(sl_boundary_.start_l()),
                                             std::fabs(sl_boundary_.end_l()));

  const double kEpison = 1e-5;
  lateral_diff = std::min(lateral_diff, min_turn_radius - kEpison);

  /**
   * @brief 计算停车距离：在“最小转弯半径 r”约束下，车辆最早能停的位置
   * 基于勾股定理：sqrt(r² - (r-d)²) = sqrt(2rd - d²)
   */
  double stop_distance =
      std::sqrt(std::fabs(min_turn_radius * min_turn_radius -
                          (min_turn_radius - lateral_diff) *
                              (min_turn_radius - lateral_diff))) +
      stop_distance_buffer;

  /**
   * @brief 减去前轴到中心的距离
   * 停车距离是从前轴计算的
   * 把“车头参考” → 转换成“后轴参考”
   */
  stop_distance -= vehicle_param.front_edge_to_center();

  /**
   * @brief 应用距离限制
   */
  stop_distance = std::min(stop_distance, FLAGS_max_stop_distance_obstacle);
  stop_distance = std::max(stop_distance, FLAGS_min_stop_distance_obstacle);
  return stop_distance;
}

/**
 * @brief 构建参考线ST边界
 *
 * @param reference_line 参考线
 * @param adc_start_s 自车起点S坐标
 *
 * 流程：
 * 1. 静态障碍物：直接在SL边界上扩展为ST矩形
 * 2. 动态障碍物：调用BuildTrajectoryStBoundary构建
 */
void Obstacle::BuildReferenceLineStBoundary(const ReferenceLine& reference_line,
                                            const double adc_start_s) {
  const auto& adc_param =
      VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
  const double half_adc_width = adc_param.width() / 2;

   // ★ 分支A：静态障碍物 → 垂直矩形
  if (is_static_ || trajectory_.trajectory_point().empty()) {
    std::vector<std::pair<STPoint, STPoint>> point_pairs;
    double start_s = sl_boundary_.start_s();
    double end_s = sl_boundary_.end_s();

    /**
     * @brief 确保S范围足够大
     */
    if (end_s - start_s < kStBoundaryDeltaS) {
      end_s = start_s + kStBoundaryDeltaS;
    }

    /**
     * @brief 检查是否阻塞道路
     */
    if (!reference_line.IsBlockRoad(perception_bounding_box_, half_adc_width)) {
      return;
    }

    /**
     * @brief 创建ST矩形
     * t=0时刻和t=FLAGS_st_max_t时刻的两条线段
     */
    point_pairs.emplace_back(STPoint(start_s - adc_start_s, 0.0),
                             STPoint(end_s - adc_start_s, 0.0));
    point_pairs.emplace_back(STPoint(start_s - adc_start_s, FLAGS_st_max_t),
                             STPoint(end_s - adc_start_s, FLAGS_st_max_t));
    reference_line_st_boundary_ = STBoundary(point_pairs);
  } else {

// ★ 分支B：动态障碍物 → 倾斜多边形
    if (BuildTrajectoryStBoundary(reference_line, adc_start_s,
                                  &reference_line_st_boundary_)) {
      ADEBUG << "Found st_boundary for obstacle " << id_;
      ADEBUG << "st_boundary: min_t = " << reference_line_st_boundary_.min_t()
             << ", max_t = " << reference_line_st_boundary_.max_t()
             << ", min_s = " << reference_line_st_boundary_.min_s()
             << ", max_s = " << reference_line_st_boundary_.max_s();
    } else {
      ADEBUG << "No st_boundary for obstacle " << id_;
    }
  }
}

/**
 * @brief 构建轨迹ST边界
 *
 * @param reference_line 参考线
 * @param adc_start_s 自车起点S坐标
 * @param st_boundary 输出：ST边界
 * @return bool 是否成功
 *
 * 核心算法：
 * 1. 遍历轨迹的相邻两点
 * 2. 计算移动中的障碍物边界框
 * 3. 获取SL边界
 * 4. 构建ST多边形
 *
 * C++语法说明：
 * - std::fmax/std::fmin: 浮点数极值函数
 * - std::make_pair: 创建pair对象
 */
bool Obstacle::BuildTrajectoryStBoundary(const ReferenceLine& reference_line,
                                         const double adc_start_s,
                                         STBoundary* const st_boundary) {
  /**
   * @brief 验证障碍物有效性
   */
  if (!IsValidObstacle(perception_obstacle_)) {
    AERROR << "Fail to build trajectory st boundary because object is not "
              "valid. PerceptionObstacle: "
           << perception_obstacle_.DebugString();
    return false;
  }

  const double object_width = perception_obstacle_.width();
  const double object_length = perception_obstacle_.length();
  const auto& trajectory_points = trajectory_.trajectory_point();

  if (trajectory_points.empty()) {
    AWARN << "object " << id_ << " has no trajectory points";
    return false;
  }

  const auto& adc_param =
      VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
  const double adc_length = adc_param.length();
  const double adc_half_length = adc_length / 2.0;
  const double adc_width = adc_param.width();
  common::math::Box2d min_box({0, 0}, 1.0, 1.0, 1.0);
  common::math::Box2d max_box({0, 0}, 1.0, 1.0, 1.0);
  std::vector<std::pair<STPoint, STPoint>> polygon_points;  // 最终ST多边形的顶点

  SLBoundary last_sl_boundary;
  int last_index = 0;

  // 遍历相邻轨迹点，构建移动包围盒
  for (int i = 1; i < trajectory_points.size(); ++i) {
    ADEBUG << "last_sl_boundary: " << last_sl_boundary.ShortDebugString();

    const auto& first_traj_point = trajectory_points[i - 1];
    const auto& second_traj_point = trajectory_points[i];
    const auto& first_point = first_traj_point.path_point();
    const auto& second_point = second_traj_point.path_point();

    // 移动包围盒长度 = 静止长度 + 两点间移动距离
    double object_moving_box_length =
        object_length + common::util::DistanceXY(first_point, second_point);

    // 包围盒中心 = 两点中点
    common::math::Vec2d center((first_point.x() + second_point.x()) / 2.0,
                               (first_point.y() + second_point.y()) / 2.0);
    // 构造移动包围盒
    common::math::Box2d object_moving_box(
        center, first_point.theta(), object_moving_box_length, object_width);
    SLBoundary object_boundary;

    /**
     * @brief 跳过与last_sl_boundary距离太远的点
     */
    const double distance_xy =
        common::util::DistanceXY(trajectory_points[last_index].path_point(),
                                 trajectory_points[i].path_point());
    if (last_sl_boundary.start_l() > distance_xy ||
        last_sl_boundary.end_l() < -distance_xy) {
      continue;  // 横向太远 → 不可能与参考线有交互
    }

    /**
     * @brief 计算S范围
     */
    const double mid_s =
        (last_sl_boundary.start_s() + last_sl_boundary.end_s()) / 2.0;
    const double start_s = std::fmax(0.0, mid_s - 2.0 * distance_xy);
    const double end_s = (i == 1) ? reference_line.Length()
                                  : std::fmin(reference_line.Length(),
                                              mid_s + 2.0 * distance_xy);

    if (!reference_line.GetApproximateSLBoundary(object_moving_box, start_s,
                                                 end_s, &object_boundary)) {
      AERROR << "failed to calculate boundary";
      return false;
    }

    /**
     * @brief 更新历史记录
     */
    last_sl_boundary = object_boundary;
    last_index = i;

    // 3b. 跳过在参考线侧面的障碍物
    static constexpr double kSkipLDistanceFactor = 0.4;
    const double skip_l_distance =
        (object_boundary.end_s() - object_boundary.start_s()) *
            kSkipLDistanceFactor +
        adc_width / 2.0;

    if (!IsCautionLevelObstacle() &&
        (std::fmin(object_boundary.start_l(), object_boundary.end_l()) >
             skip_l_distance ||
         std::fmax(object_boundary.start_l(), object_boundary.end_l()) <
             -skip_l_distance)) {  // 障碍物完全在参考线侧面 → 不影响纵向规划
      continue;
    }

    /**
     * @brief 跳过在参考线后方的障碍物
     */
    if (!IsCautionLevelObstacle() && object_boundary.end_s() < 0) {
      continue; // 已经在自车后方 → 不需要规划
    }

    // 自适应采样密度
    static constexpr double kSparseMappingS = 20.0;
    const double st_boundary_delta_s =
        (std::fabs(object_boundary.start_s() - adc_start_s) > kSparseMappingS)
            ? kStBoundarySparseDeltaS   // 远处: 1.0m 步长
            : kStBoundaryDeltaS;         // 近处: 0.2m 步长

    const double object_s_diff =
        object_boundary.end_s() - object_boundary.start_s();
    if (object_s_diff < st_boundary_delta_s) {
      continue;
    }

    /**
     * @brief 计算时间间隔
     */
    const double delta_t =
        second_traj_point.relative_time() - first_traj_point.relative_time();

    // 双向扫描确定精确 s 边界
    double low_s = std::max(object_boundary.start_s() - adc_half_length, 0.0);
    bool has_low = false;
    double high_s =
        std::min(object_boundary.end_s() + adc_half_length, FLAGS_st_max_s);
    bool has_high = false;

// 从两端向中间扫描，找到与自车有碰撞重叠的s范围
    while (low_s + st_boundary_delta_s < high_s && !(has_low && has_high)) {
      if (!has_low) {
        auto low_ref = reference_line.GetReferencePoint(low_s);
        has_low = object_moving_box.HasOverlap(
            {low_ref, low_ref.heading(), adc_length,
             adc_width + FLAGS_nonstatic_obstacle_nudge_l_buffer});
        low_s += st_boundary_delta_s;  // 从下往上扫
      }
      if (!has_high) {
        auto high_ref = reference_line.GetReferencePoint(high_s);
        has_high = object_moving_box.HasOverlap(
            {high_ref, high_ref.heading(), adc_length,
             adc_width + FLAGS_nonstatic_obstacle_nudge_l_buffer});
        high_s -= st_boundary_delta_s;  // 从上往下扫
      }
    }

    // 生成 ST 多边形点
    if (has_low && has_high) {
      low_s -= st_boundary_delta_s;  // 回退一步（保守扩展）
      high_s += st_boundary_delta_s;

      // 计算时间：按s位置在SL边界中的比例插值
      double low_t =
          (first_traj_point.relative_time() +
           std::fabs((low_s - object_boundary.start_s()) / object_s_diff) *
               delta_t);
      // 添加下边界点对 (low_s, high_s) at low_t
      polygon_points.emplace_back(
          std::make_pair(STPoint{low_s - adc_start_s, low_t},
                         STPoint{high_s - adc_start_s, low_t}));
      // 计算上边界时间
      double high_t =
          (first_traj_point.relative_time() +
           std::fabs((high_s - object_boundary.start_s()) / object_s_diff) *
               delta_t);
      // 如果时间差够大，添加上边界点对/*  */
      if (high_t - low_t > 0.05) {
        polygon_points.emplace_back(
            std::make_pair(STPoint{low_s - adc_start_s, high_t},
                           STPoint{high_s - adc_start_s, high_t}));
      }
    }
  }

  /**
   * @brief 处理多边形点
   */
  if (!polygon_points.empty()) {
    /**
     * @brief 按时间排序
     */
    std::sort(polygon_points.begin(), polygon_points.end(),
              [](const std::pair<STPoint, STPoint>& a,
                 const std::pair<STPoint, STPoint>& b) {
                return a.first.t() < b.first.t();
              });

    /**
     * @brief 去除时间相近的重复点
     */
    auto last = std::unique(polygon_points.begin(), polygon_points.end(),
                            [](const std::pair<STPoint, STPoint>& a,
                               const std::pair<STPoint, STPoint>& b) {
                              return std::fabs(a.first.t() - b.first.t()) <
                                     kStBoundaryDeltaT;
                            });
    polygon_points.erase(last, polygon_points.end());

    if (polygon_points.size() > 2) {
      *st_boundary = STBoundary(polygon_points);
    }
  } else {
    return false;
  }
  return true;
}

/**
 * @brief 获取参考线ST边界
 */
const STBoundary& Obstacle::reference_line_st_boundary() const {
  return reference_line_st_boundary_;
}

/**
 * @brief 获取路径ST边界
 */
const STBoundary& Obstacle::path_st_boundary() const {
  return path_st_boundary_;
}

/**
 * @brief 获取决策标签列表
 */
const std::vector<std::string>& Obstacle::decider_tags() const {
  return decider_tags_;
}

/**
 * @brief 获取决策列表
 */
const std::vector<ObjectDecisionType>& Obstacle::decisions() const {
  return decisions_;
}

/**
 * @brief 判断是否为横向决策
 */
bool Obstacle::IsLateralDecision(const ObjectDecisionType& decision) {
  return decision.has_ignore() || decision.has_nudge();
}

/**
 * @brief 判断是否为纵向决策
 */
bool Obstacle::IsLongitudinalDecision(const ObjectDecisionType& decision) {
  return decision.has_ignore() || decision.has_stop() || decision.has_yield() ||
         decision.has_follow() || decision.has_overtake();
}

/**
 * @brief 合并纵向决策
 *
 * @param lhs 左决策
 * @param rhs 右决策
 * @return ObjectDecisionType 合并后的决策
 *
 * 合并规则：
 * 1. IGNORE优先（优先级最低）
 * 2. 否则选择优先级较高的
 * 3. 优先级相同时，根据具体决策类型选择
 */
ObjectDecisionType Obstacle::MergeLongitudinalDecision(
    const ObjectDecisionType& lhs, const ObjectDecisionType& rhs) {
  /**
   * @brief 处理空决策
   */
  if (lhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return rhs;
  }
  if (rhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return lhs;
  }

  /**
   * @brief 获取优先级
   */
  const auto lhs_val =
      FindOrDie(s_longitudinal_decision_safety_sorter_, lhs.object_tag_case());
  const auto rhs_val =
      FindOrDie(s_longitudinal_decision_safety_sorter_, rhs.object_tag_case());

  /**
   * @brief 选择优先级高的
   */
  if (lhs_val < rhs_val) {
    return rhs;
  } else if (lhs_val > rhs_val) {
    return lhs;
  } else {
    /**
     * @brief 优先级相同时的处理
     */
    if (lhs.has_ignore()) {
      return rhs;
    } else if (lhs.has_stop()) {
      return lhs.stop().distance_s() < rhs.stop().distance_s() ? lhs : rhs;
    } else if (lhs.has_yield()) {
      return lhs.yield().distance_s() < rhs.yield().distance_s() ? lhs : rhs;
    } else if (lhs.has_follow()) {
      return lhs.follow().distance_s() < rhs.follow().distance_s() ? lhs : rhs;
    } else if (lhs.has_overtake()) {
      return lhs.overtake().distance_s() > rhs.overtake().distance_s() ? lhs
                                                                       : rhs;
    } else {
      DCHECK(false) << "Unknown decision";
    }
  }
  return lhs;  /**< 避免编译警告 */
}

/**
 * @brief 获取纵向决策
 */
const ObjectDecisionType& Obstacle::LongitudinalDecision() const {
  return longitudinal_decision_;
}

/**
 * @brief 获取横向决策
 */
const ObjectDecisionType& Obstacle::LateralDecision() const {
  return lateral_decision_;
}

/**
 * @brief 判断是否忽略
 */
bool Obstacle::IsIgnore() const {
  return IsLongitudinalIgnore() && IsLateralIgnore();
}

/**
 * @brief 判断纵向是否忽略
 */
bool Obstacle::IsLongitudinalIgnore() const {
  return longitudinal_decision_.has_ignore();
}

/**
 * @brief 判断横向是否忽略
 */
bool Obstacle::IsLateralIgnore() const {
  return lateral_decision_.has_ignore();
}

/**
 * @brief 合并横向决策
 */
ObjectDecisionType Obstacle::MergeLateralDecision(
    const ObjectDecisionType& lhs, const ObjectDecisionType& rhs) {
  // 1. 空决策处理
  if (lhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return rhs;  // 左边为空 → 取右边
  }
  if (rhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return lhs;  // 右边为空 → 取左边
  }
  
  // 2. 查优先级表
  const auto lhs_val =
      FindOrDie(s_lateral_decision_safety_sorter_, lhs.object_tag_case());
  const auto rhs_val =
      FindOrDie(s_lateral_decision_safety_sorter_, rhs.object_tag_case());
  
  // 3. 不同优先级 → 取高优先级
  if (lhs_val < rhs_val) {
    return rhs;
  } else if (lhs_val > rhs_val) {
    return lhs;
  }
  // 4. 相同优先级 → 进一步处理
  else {
    if (lhs.has_ignore()) {
      return rhs; // 两个IGNORE → 取右边（都一样）
    } else if (lhs.has_nudge()) {
      DCHECK(lhs.nudge().type() == rhs.nudge().type())
          << "could not merge left nudge and right nudge";
      // 两个NUDGE → 取距离更大的（更保守）
      return std::fabs(lhs.nudge().distance_l()) >
                     std::fabs(rhs.nudge().distance_l())
                 ? lhs
                 : rhs;
    }
  }
  DCHECK(false) << "Does not have rule to merge decision: "
                << lhs.ShortDebugString()
                << " and decision: " << rhs.ShortDebugString();
  return lhs;
}

/**
 * @brief 是否有横向决策
 */
bool Obstacle::HasLateralDecision() const {
  return lateral_decision_.object_tag_case() !=
         ObjectDecisionType::OBJECT_TAG_NOT_SET;
}

/**
 * @brief 是否有纵向决策
 */
bool Obstacle::HasLongitudinalDecision() const {
  return longitudinal_decision_.object_tag_case() !=
         ObjectDecisionType::OBJECT_TAG_NOT_SET;
}

/**
 * @brief 是否有非忽略决策
 */
bool Obstacle::HasNonIgnoreDecision() const {
  return (HasLateralDecision() && !IsLateralIgnore()) ||
         (HasLongitudinalDecision() && !IsLongitudinalIgnore());
}

/**
 * @brief 添加纵向决策
 *
 * @param decider_tag 决策者标签
 * @param decision 决策
 */
void Obstacle::AddLongitudinalDecision(const std::string& decider_tag,
                                       const ObjectDecisionType& decision) {
  DCHECK(IsLongitudinalDecision(decision))
      << "Decision: " << decision.ShortDebugString()
      << " is not a longitudinal decision";

  /**
   * @brief 合并到现有决策
   */
  longitudinal_decision_ =
      MergeLongitudinalDecision(longitudinal_decision_, decision);

  ADEBUG << decider_tag << " added obstacle " << Id()
         << " longitudinal decision: " << decision.ShortDebugString()
         << ". The merged decision is: "
         << longitudinal_decision_.ShortDebugString();

  /**
   * @brief 添加到历史列表
   */
  decisions_.push_back(decision);
  decider_tags_.push_back(decider_tag);
}

/**
 * @brief 添加横向决策
 */
void Obstacle::AddLateralDecision(const std::string& decider_tag,
                                  const ObjectDecisionType& decision) {
  DCHECK(IsLateralDecision(decision))
      << "Decision: " << decision.ShortDebugString()
      << " is not a lateral decision";

  lateral_decision_ = MergeLateralDecision(lateral_decision_, decision);

  ADEBUG << decider_tag << " added obstacle " << Id()
         << " a lateral decision: " << decision.ShortDebugString()
         << ". The merged decision is: "
         << lateral_decision_.ShortDebugString();

  decisions_.push_back(decision);
  decider_tags_.push_back(decider_tag);
}

/**
 * @brief 生成调试字符串
 */
std::string Obstacle::DebugString() const {
  std::stringstream ss;
  ss << "Obstacle id: " << id_;
  for (size_t i = 0; i < decisions_.size(); ++i) {
    ss << " decision: " << decisions_[i].DebugString() << ", made by "
       << decider_tags_[i];
  }
  if (lateral_decision_.object_tag_case() !=
      ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    ss << "lateral decision: " << lateral_decision_.ShortDebugString();
  }
  if (longitudinal_decision_.object_tag_case() !=
      ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    ss << "longitudinal decision: "
       << longitudinal_decision_.ShortDebugString();
  }
  return ss.str();
}

/**
 * @brief 获取感知SL边界
 */
const SLBoundary& Obstacle::PerceptionSLBoundary() const {
  return sl_boundary_;
}

/**
 * @brief 设置路径ST边界
 */
void Obstacle::set_path_st_boundary(const STBoundary& boundary) {
  path_st_boundary_ = boundary;
  path_st_boundary_initialized_ = true;
}

/**
 * @brief 设置ST边界类型
 */
void Obstacle::SetStBoundaryType(const STBoundary::BoundaryType type) {
  path_st_boundary_.SetBoundaryType(type);
}

/**
 * @brief 清除ST边界
 */
void Obstacle::EraseStBoundary() { path_st_boundary_ = STBoundary(); }

/**
 * @brief 清除决策
 */
void Obstacle::EraseDecision() {
  lateral_decision_.Clear();
  longitudinal_decision_.Clear();
  decider_tags_.clear();
}

/**
 * @brief 设置参考线ST边界
 */
void Obstacle::SetReferenceLineStBoundary(const STBoundary& boundary) {
  reference_line_st_boundary_ = boundary;
}

/**
 * @brief 设置参考线ST边界类型
 */
void Obstacle::SetReferenceLineStBoundaryType(
    const STBoundary::BoundaryType type) {
  reference_line_st_boundary_.SetBoundaryType(type);
}

/**
 * @brief 清除参考线ST边界
 */
void Obstacle::EraseReferenceLineStBoundary() {
  reference_line_st_boundary_ = STBoundary();
}

/**
 * @brief 验证障碍物是否有效
 */
bool Obstacle::IsValidObstacle(
    const perception::PerceptionObstacle& perception_obstacle) {
  const double object_width = perception_obstacle.width();
  const double object_length = perception_obstacle.length();

  const double kMinObjectDimension = 1.0e-6;
  return !std::isnan(object_width) && !std::isnan(object_length) &&
         object_width > kMinObjectDimension &&
         object_length > kMinObjectDimension;
}

/**
 * @brief 检查是否阻塞车道
 */
void Obstacle::CheckLaneBlocking(const ReferenceLine& reference_line) {
  if (!IsStatic()) {
    is_lane_blocking_ = false;
    return;
  }
  DCHECK(sl_boundary_.has_start_s());
  DCHECK(sl_boundary_.has_end_s());
  DCHECK(sl_boundary_.has_start_l());
  DCHECK(sl_boundary_.has_end_l());

  /**
   * @brief 障碍物跨越车道中心
   */
  if (sl_boundary_.start_l() * sl_boundary_.end_l() < 0.0) {
    is_lane_blocking_ = true;
    return;
  }

  const double driving_width = reference_line.GetDrivingWidth(sl_boundary_);
  auto vehicle_param = common::VehicleConfigHelper::GetConfig().vehicle_param();

  /**
   * @brief 可行驶宽度小于车辆宽度+缓冲区
   */
  if (reference_line.IsOnLane(sl_boundary_) &&
      driving_width <
          vehicle_param.width() + FLAGS_static_obstacle_nudge_l_buffer) {
    is_lane_blocking_ = true;
    return;
  }

  is_lane_blocking_ = false;
}

/**
 * @brief 设置换道阻塞标志
 */
void Obstacle::SetLaneChangeBlocking(const bool is_distance_clear) {
  is_lane_change_blocking_ = is_distance_clear;
}

/**
 * @brief 获取轨迹时刻的障碍物多边形
 *
 * @param point 轨迹点
 * @return Polygon2d 变换后的多边形
 *
 * 旋转变换：
 * 1. 计算航向角差
 * 2. 相对于感知位置计算相对坐标
 * 3. 旋转变换
 * 4. 加上轨迹点位置
 */
common::math::Polygon2d Obstacle::GetObstacleTrajectoryPolygon(
    const common::TrajectoryPoint& point) const {
  /**
   * @brief 计算航向角差
   */
  double delta_heading =
      point.path_point().theta() - perception_obstacle_.theta();
  double cos_delta_heading = cos(delta_heading);
  double sin_delta_heading = sin(delta_heading);

  std::vector<common::math::Vec2d> polygon_point;
  polygon_point.reserve(perception_polygon_.points().size());

  /**
   * @brief 遍历原始多边形顶点进行变换
   */
  for (auto& iter : perception_polygon_.points()) {
    /**
     * @brief 计算相对坐标
     */
    double relative_x = iter.x() - perception_obstacle_.position().x();
    double relative_y = iter.y() - perception_obstacle_.position().y();

    /**
     * @brief 旋转+平移变换
     * x' = rx*cos - ry*sin + tx
     * y' = rx*sin + ry*cos + ty
     */
    double x = relative_x * cos_delta_heading - relative_y * sin_delta_heading +
               point.path_point().x();
    double y = relative_x * sin_delta_heading + relative_y * cos_delta_heading +
               point.path_point().y();
    polygon_point.emplace_back(x, y);
  }

  common::math::Polygon2d trajectory_point_polygon(polygon_point);
  return trajectory_point_polygon;
}

/**
 * @brief 打印多边形曲线调试信息
 */
void Obstacle::PrintPolygonCurve() const {
  if (perception_polygon_.points().size() < 1) {
    return;
  }

  PrintCurves print_curve;
  for (const auto& p : perception_polygon_.points()) {
    print_curve.AddPoint(id_ + "_ObsPolygon", p.x(), p.y());
  }
  print_curve.AddPoint(id_ + "_ObsPolygon", perception_polygon_.points()[0].x(),
                       perception_polygon_.points()[0].y());
  print_curve.PrintToLog();
}

/**
 * @brief 命名空间结束标记
 */
}  // namespace planning
}  // namespace apollo
