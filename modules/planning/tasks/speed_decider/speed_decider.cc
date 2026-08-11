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
 * @file
 * @brief 速度决策器实现文件
 *
 * 本文件实现了速度决策器(SpeedDecider)，用于在速度规划后为所有障碍物生成纵向决策
 *
 * 功能说明：
 * 1. 根据速度曲线判断自车与障碍物的时空关系（BELOW/ABOVE/CROSS）
 * 2. 为障碍物生成纵向决策（STOP/FOLLOW/YIELD/OVERTAKE/IGNORE）
 * 3. 处理特殊场景：行人停车、KEEP_CLEAR区域、跟车过近等
 *
 * 决策类型说明：
 * - STOP: 停车等待障碍物通过
 * - FOLLOW: 跟车行驶，保持安全距离
 * - YIELD: 减速让行，等待合适的超车时机
 * - OVERTAKE: 加速超车
 * - IGNORE: 忽略障碍物
 *
 * ST位置关系说明：
 * - BELOW: 自车速度曲线在ST边界下方（自车落后于障碍物）
 * - ABOVE: 自车速度曲线在ST边界上方（自车领先于障碍物）
 * - CROSS: 自车速度曲线与ST边界相交（发生碰撞）
 *
 * 相关C++语法说明：
 * - std::shared_ptr<T>: 智能指针，共享所有权
 * - switch-case: 多分支选择语句
 * - std::unordered_map: 哈希表，用于高效查找
 **/
/**
 * @brief 速度决策器头文件
 *
 * 包含SpeedDecider类的完整定义
 */
#include "modules/planning/tasks/speed_decider/speed_decider.h"

/**
 * @brief 标准库头文件
 * <algorithm>: 提供std::min, std::max, std::fmax, std::fabs等算法
 * <memory>: 提供std::shared_ptr智能指针
 */
#include <algorithm>
#include <memory>

/**
 * @brief 感知障碍物protobuf消息头文件
 * PerceptionObstacle: 感知障碍物消息类型
 * 包含障碍物的位置、速度、类型等信息
 */
#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"

/**
 * @brief 决策protobuf消息头文件
 * decision.pb.h: 包含STOP、FOLLOW、YIELD、OVERTAKE等决策消息定义
 */
#include "modules/common_msgs/planning_msgs/decision.pb.h"

/**
 * @brief Cyber RT日志和时间系统头文件
 * cyber/common/log.h: 日志系统
 * cyber/time/clock.h: 时间获取
 */
#include "cyber/common/log.h"
#include "cyber/time/clock.h"

/**
 * @brief 车辆配置助手头文件
 * VehicleConfigHelper: 获取车辆参数（长度、宽度等）
 */
#include "modules/common/configs/vehicle_config_helper.h"

/**
 * @brief 通用工具头文件
 */
#include "modules/common/util/util.h"

/**
 * @brief 规划上下文头文件
 * PlanningContext: 存储规划过程中的状态信息
 */
#include "modules/planning/planning_base/common/planning_context.h"

/**
 * @brief 规划模块GFlags头文件
 * FLAGS_*: 从配置文件获取的参数
 */
#include "modules/planning/planning_base/gflags/planning_gflags.h"

/**
 * @brief ST间隙估算器头文件
 * StGapEstimator: 安全距离估算工具
 */
#include "modules/planning/planning_interface_base/task_base/utils/st_gap_estimator.h"

/**
 * @brief Apollo命名空间开始
 */
namespace apollo {

/**
 * @brief 规划模块命名空间
 */
namespace planning {

/**
 * @brief 使用别名声明，简化类型引用
 *
 * C++语法说明：
 * using声明：引入其他命名空间的类型到当前作用域
 * 类似于typedef，但更现代
 */
using apollo::common::ErrorCode;           /**< 错误码类型 */
using apollo::common::Status;              /**< 状态类型 */
using apollo::common::VehicleConfigHelper; /**< 车辆配置助手 */
using apollo::common::math::Vec2d;         /**< 二维向量 */
using apollo::cyber::Clock;               /**< Cyber RT时钟 */
using apollo::perception::PerceptionObstacle; /**< 感知障碍物类型 */

/**
 * @brief 速度决策器初始化函数
 *
 * @param config_dir 配置文件目录路径
 * @param name 任务名称
 * @param injector 依赖注入器指针
 * @return bool 初始化是否成功
 *
 * 功能说明：
 * 1. 调用父类Task的初始化函数
 * 2. 加载SpeedDeciderConfig配置
 * 3. 初始化跟车距离函数
 *
 * 算法流程：
 * 1. 调用Task::Init进行基础初始化
 * 2. 加载SpeedDeciderConfig配置
 * 3. 从配置中读取跟车距离函数并排序
 * 4. 打印调试信息
 *
 * C++语法说明：
 * - std::shared_ptr<DependencyInjector>: 共享所有权的智能指针
 * - Task::Init<TaskConfig>: 模板方法，从配置文件加载配置
 * - std::make_pair: 创建pair对象
 * - std::sort: 标准库排序算法
 */
bool SpeedDecider::Init(const std::string& config_dir, const std::string& name,
                        const std::shared_ptr<DependencyInjector>& injector) {
  /**
   * @brief 调用父类Task的初始化函数
   *
   * Task::Init():
   *   执行基础初始化，如保存配置目录、名称、依赖注入器等
   */
  if (!Task::Init(config_dir, name, injector)) {
    return false;  /**< 初始化失败，返回false */
  }

  /**
   * @brief 加载本任务的配置
   *
   * Task::LoadConfig<T>():
   *   模板方法，从配置文件加载SpeedDeciderConfig类型配置
   *   配置内容可能包括：跟车距离参数、决策阈值等
   */
  if (!Task::LoadConfig<SpeedDeciderConfig>(&config_)) {
    return false;  /**< 配置加载失败 */
  }

  /**
   * @brief 从配置中读取跟车距离函数
   *
   * config_.follow_distance_scheduler().follow_distance():
   *   获取跟车距离调度器中的跟车距离函数列表
   *   每个函数由(speed, slope)对定义
   *
   * C++语法说明：
   * for (const auto& follow_function : ...):
   *   范围for循环，遍历follow_distance列表
   * emplace_back: 直接构造pair并添加到向量，避免拷贝
   */
  for (const auto& follow_function :
       config_.follow_distance_scheduler().follow_distance()) {
    follow_distance_function_.emplace_back(
        std::make_pair(follow_function.speed(), follow_function.slope()));
  }

  /**
   * @brief 按速度排序跟车距离函数
   *
   * std::sort:
   *   对follow_distance_function_按speed升序排序
   *   这样在查询时可以二分查找
   */
  std::sort(follow_distance_function_.begin(), follow_distance_function_.end());

  /**
   * @brief 打印排序后的跟车距离函数
   * AINFO: Apollo信息级别日志
   */
  for (const auto& iter : follow_distance_function_) {
    AINFO << "speed: " << iter.first << ", slope: " << iter.second;
  }

  return true;  /**< 初始化成功 */
}

/**
 * @brief 执行速度决策
 *
 * @param frame 当前规划帧
 * @param reference_line_info 参考线信息
 * @return Status 执行状态
 *
 * 功能说明：
 * 速度决策的主入口函数
 * 1. 调用Task::Execute执行基类逻辑
 * 2. 保存规划起点和自车SL边界
 * 3. 调用MakeObjectDecision生成所有障碍物的决策
 *
 * 算法流程：
 * 1. 调用父类Execute，保存frame_和reference_line_info_
 * 2. 获取规划起点init_point_
 * 3. 获取自车SL边界adc_sl_boundary_
 * 4. 获取参考线引用reference_line_
 * 5. 调用MakeObjectDecision生成决策
 *
 * C++语法说明：
 * - Frame*: 原始指针，表示当前规划帧
 * - ReferenceLineInfo*: 参考线信息指针
 * - reference_line_info_->reference_line():
 *   先调用reference_line_info_->获取指针，再调用reference_line()
 */
common::Status SpeedDecider::Execute(Frame* frame,
                                     ReferenceLineInfo* reference_line_info) {
  /**
   * @brief 调用父类Execute执行基础逻辑
   *
   * Task::Execute():
   *   保存frame和reference_line_info到成员变量
   */
  Task::Execute(frame, reference_line_info);

  /**
   * @brief 保存规划起点
   *
   * frame_->PlanningStartPoint():
   *   获取当前帧的规划起点（上一帧轨迹的终点）
   * 用于速度决策的初始条件
   */
  init_point_ = frame_->PlanningStartPoint();

  /**
   * @brief 保存自车SL边界
   *
   * reference_line_info_->AdcSlBoundary():
   *   获取自车在SL坐标系下的边界框
   * 包含start_s, end_s, start_l, end_l
   */
  adc_sl_boundary_ = reference_line_info_->AdcSlBoundary();

  /**
   * @brief 获取参考线引用
   *
   * reference_line_:
   *   指向reference_line_info_->reference_line()的指针
   * 用于后续查询参考线信息
   */
  reference_line_ = &reference_line_info_->reference_line();

  /**
   * @brief 生成所有障碍物的决策
   *
   * MakeObjectDecision():
   *   根据速度曲线为所有障碍物生成纵向决策
   * 参数：
   * - reference_line_info->speed_data(): 速度曲线
   * - reference_line_info->path_decision(): 路径决策（包含障碍物列表）
   */
  if (!MakeObjectDecision(reference_line_info->speed_data(),
                          reference_line_info->path_decision())
           .ok()) {
    /**
     * @brief 决策生成失败，记录错误
     */
    const std::string msg = "Get object decision by speed profile failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);  /**< 返回规划错误状态 */
  }

  return Status::OK();  /**< 执行成功 */
}

/**
 * @brief 获取ST位置
 *
 * @param path_decision 路径决策
 * @param speed_profile 速度曲线
 * @param st_boundary ST边界
 * @return STLocation 位置类型（BELOW/ABOVE/CROSS）
 *
 * 功能说明：
 * 判断自车速度曲线与障碍物ST边界的时空位置关系
 *
 * 算法说明：
 * 1. 遍历速度曲线的每个点
 * 2. 检查速度曲线是否与ST边界多边形相交
 * 3. 根据叉积判断点在边界的哪一侧
 *
 * C++语法说明：
 * - const PathDecision* const: 指向常量的指针，指针本身是常量
 * - const SpeedData&: 常量引用
 * - const STBoundary&: 常量引用
 */
SpeedDecider::STLocation SpeedDecider::GetSTLocation(
    const PathDecision* const path_decision, const SpeedData& speed_profile,
    const STBoundary& st_boundary) const {
  /**
   * @brief 空边界检查
   * 如果ST边界为空，直接返回BELOW
   */
  if (st_boundary.IsEmpty()) {
    return BELOW;
  }

  /**
   * @brief 初始化位置为BELOW
   * 初始化标志位为false
   */
  STLocation st_location = BELOW;
  bool st_position_set = false;

  /**
   * @brief 获取ST边界的时间范围
   * min_t/max_t: 边界在时间轴上的起止点
   */
  const double start_t = st_boundary.min_t();
  const double end_t = st_boundary.max_t();

  /**
   * @brief 遍历速度曲线的每个点
   *
   * for循环：
   * - i + 1 < speed_profile.size() 确保i+1有效
   * - ++i: 前置递增
   */
  for (size_t i = 0; i + 1 < speed_profile.size(); ++i) {
    /**
     * @brief 创建当前点和下一个点的ST坐标
     *
     * STPoint(s, t):
     *   创建一个ST点，s是路径距离，t是时间
     */
    const STPoint curr_st(speed_profile[i].s(), speed_profile[i].t());
    const STPoint next_st(speed_profile[i + 1].s(), speed_profile[i + 1].t());

    /**
     * @brief 检查是否在时间范围之前
     *
     * 如果当前点和下一个点都在start_t之前
     * 说明还没有进入障碍物的时间范围，继续
     */
    if (curr_st.t() < start_t && next_st.t() < start_t) {
      continue;  /**< 跳过，继续检查下一个时间段 */
    }

    /**
     * @brief 检查是否超出时间范围
     *
     * 如果当前点已经超出end_t
     * 说明已经通过了障碍物的全部时间范围，停止检查
     */
    if (curr_st.t() > end_t) {
      break;  /**< 超出范围，停止检查 */
    }

    /**
     * @brief 检查是否与ST边界相交
     *
     * FLAGS_use_st_drivable_boundary:
     *   配置开关，是否使用ST可行驶边界
     * 如果不使用，则进行相交检查
     */
    if (!FLAGS_use_st_drivable_boundary) {
      /**
       * @brief 创建速度曲线的线段
       *
       * LineSegment2d:
       *   二维线段类，由两个端点定义
       * 表示速度曲线的一段
       */
      common::math::LineSegment2d speed_line(curr_st, next_st);

      /**
       * @brief 检查线段与边界是否相交
       *
       * HasOverlap:
       *   检查两个多边形/线段是否重叠
       * 如果相交，说明速度曲线穿过ST边界
       */
      if (st_boundary.HasOverlap(speed_line)) {
        ADEBUG << "speed profile cross st_boundaries.";
        st_location = CROSS;  /**< 标记为CROSS（相交） */

        /**
         * @brief 特殊处理KEEP_CLEAR类型边界
         *
         * KEEP_CLEAR: 禁行区（如斑马线）
         * 需要检查是否可以穿越
         */
        if (!FLAGS_use_st_drivable_boundary) {
          if (st_boundary.boundary_type() ==
              STBoundary::BoundaryType::KEEP_CLEAR) {
            /**
             * @brief 检查KEEP_CLEAR是否可穿越
             *
             * CheckKeepClearCrossable:
             *   根据末点速度和位置判断是否可以穿越
             */
            if (!CheckKeepClearCrossable(path_decision, speed_profile,
                                         st_boundary)) {
              st_location = BELOW;  /**< 不可穿越，视为在下方 */
            }
          }
        }
        break;  /**< 已经确定位置，停止检查 */
      }
    }

    /**
     * @brief 确定位置关系（点在边界的哪一侧）
     *
     * 注意：需要遍历所有点确保没有CROSS
     * 只有在没有CROSS的情况下才判断BELOW/ABOVE
     *
     * C++语法说明：
     * if (!st_position_set):
     *   条件为真时执行，只有第一次会进入
     */
    if (!st_position_set) {
      /**
       * @brief 检查时间重叠
       *
       * start_t < next_st.t() && curr_st.t() < end_t:
       *   确保时间段与边界有时间重叠
       */
      if (start_t < next_st.t() && curr_st.t() < end_t) {
        /**
         * @brief 获取边界的上边界前点
         *
         * upper_points().front():
         *   获取上边界点的第一个点
         */
        STPoint bd_point_front = st_boundary.upper_points().front();

        /**
         * @brief 计算叉积判断位置
         *
         * CrossProd(p0, p1, p2):
         *   计算向量(p1-p0)和(p2-p0)的叉积
         *   叉积 > 0: p2在(p1-p0)的左侧
         *   叉积 < 0: p2在(p1-p0)的右侧
         *
         * side < 0: curr_st在边界线的左侧（速度曲线在边界下方）
         * side >= 0: curr_st在边界线的右侧（速度曲线在边界上方）
         */
        double side = common::math::CrossProd(bd_point_front, curr_st, next_st);
        st_location = side < 0.0 ? ABOVE : BELOW;
        st_position_set = true;  /**< 标记已确定位置 */
      }
    }
  }

  return st_location;  /**< 返回位置关系 */
}

/**
 * @brief 检查KEEP_CLEAR是否可穿越
 *
 * @param path_decision 路径决策
 * @param speed_profile 速度曲线
 * @param keep_clear_st_boundary KEEP_CLEAR的ST边界
 * @return bool 是否可穿越
 *
 * 功能说明：
 * 根据速度曲线的末点位置和速度
 * 判断是否可以安全穿越KEEP_CLEAR区域
 *
 * 判断条件：
 * - 末点位置 > KEEP_CLEAR最大S（已经通过）
 * - 或者末点速度 >= 阈值（可以快速通过）
 *
 * C++语法说明：
 * const SpeedData&: 常量引用，避免拷贝
 */
bool SpeedDecider::CheckKeepClearCrossable(
    const PathDecision* const path_decision, const SpeedData& speed_profile,
    const STBoundary& keep_clear_st_boundary) const {
  /**
   * @brief 初始化为可穿越
   */
  bool keep_clear_crossable = true;

  /**
   * @brief 获取速度曲线的末点
   */
  const auto& last_speed_point = speed_profile.back();

  /**
   * @brief 获取末点速度
   *
   * 首先尝试从has_v()获取
   * 如果没有速度信息，则从位置差分计算
   */
  double last_speed_point_v = 0.0;
  if (last_speed_point.has_v()) {
    /**
     * @brief 直接使用速度字段
     */
    last_speed_point_v = last_speed_point.v();
  } else {
    /**
     * @brief 从位置差分计算速度
     *
     * v = ds / dt
     */
    const size_t len = speed_profile.size();
    if (len > 1) {
      /**
       * @brief 获取倒数第二个点
       */
      const auto& last_2nd_speed_point = speed_profile[len - 2];

      /**
       * @brief 计算速度
       */
      last_speed_point_v = (last_speed_point.s() - last_2nd_speed_point.s()) /
                           (last_speed_point.t() - last_2nd_speed_point.t());
    }
  }

  /**
   * @brief 打印调试信息
   */
  ADEBUG << "last_speed_point_s[" << last_speed_point.s()
         << "] st_boundary.max_s[" << keep_clear_st_boundary.max_s()
         << "] last_speed_point_v[" << last_speed_point_v << "]";

  /**
   * @brief 判断是否可穿越
   *
   * 不可穿越条件（两者同时满足）：
   * 1. 末点位置 <= KEEP_CLEAR最大S（还没有通过）
   * 2. 末点速度 < 配置的阈值速度
   */
  if (last_speed_point.s() <= keep_clear_st_boundary.max_s() &&
      last_speed_point_v < config_.keep_clear_last_point_speed()) {
    keep_clear_crossable = false;  /**< 不可穿越 */
  }

  return keep_clear_crossable;
}

/**
 * @brief 检查KEEP_CLEAR是否被阻塞
 *
 * @param path_decision 路径决策
 * @param keep_clear_obstacle KEEP_CLEAR障碍物
 * @return bool 是否被阻塞
 *
 * 功能说明：
 * 检查KEEP_CLEAR区域是否被其他停车障碍物阻塞
 * 如果被阻塞，则不能穿越
 *
 * 算法说明：
 * 遍历所有障碍物
 * 如果障碍物是阻塞性的且距离KEEP_CLEAR足够近
 * 则KEEP_CLEAR被阻塞
 *
 * C++语法说明：
 * const Obstacle&: 常量引用
 */
bool SpeedDecider::CheckKeepClearBlocked(
    const PathDecision* const path_decision,
    const Obstacle& keep_clear_obstacle) const {
  /**
   * @brief 初始化为未阻塞
   */
  bool keep_clear_blocked = false;

  /**
   * @brief 遍历所有障碍物
   *
   * path_decision->obstacles().Items():
   *   获取路径决策中的所有障碍物
   */
  for (const auto* obstacle : path_decision->obstacles().Items()) {
    /**
     * @brief 跳过自身
     */
    if (obstacle->Id() == keep_clear_obstacle.Id()) {
      continue;  /**< 跳过 */
    }

    /**
     * @brief 获取障碍物的起始S坐标
     */
    const double obstacle_start_s = obstacle->PerceptionSLBoundary().start_s();

    /**
     * @brief 获取车辆长度
     */
    const double adc_length =
        VehicleConfigHelper::GetConfig().vehicle_param().length();

    /**
     * @brief 计算与KEEP_CLEAR边界的距离
     *
     * distance = 障碍物起点 - KEEP_CLEAR终点
     */
    const double distance =
        obstacle_start_s - keep_clear_obstacle.PerceptionSLBoundary().end_s();

    /**
     * @brief 判断是否阻塞
     *
     * 阻塞条件：
     * 1. 障碍物是阻塞性的
     * 2. 距离 > 0（在KEEP_CLEAR后方）
     * 3. 距离 < 车辆长度的一半
     */
    if (obstacle->IsBlockingObstacle() && distance > 0 &&
        distance < (adc_length / 2)) {
      keep_clear_blocked = true;  /**< 被阻塞 */
      break;  /**< 已确定被阻塞，停止检查 */
    }
  }

  return keep_clear_blocked;
}

/**
 * @brief 判断是否跟车过近
 *
 * @param obstacle 障碍物
 * @return bool 是否过近
 *
 * 功能说明：
 * 判断自车与前方的低速障碍物是否跟车距离过近
 * 需要紧急制动
 *
 * 算法说明：
 * 使用运动学公式计算安全停车距离：
 * d = (v_ego - v_obs)² / (2 * a_max)
 * 如果实际距离 < 安全距离，则过近
 *
 * C++语法说明：
 * const Obstacle&: 常量引用
 */
bool SpeedDecider::IsFollowTooClose(const Obstacle& obstacle) const {
  /**
   * @brief 非阻塞障碍物直接返回false
   */
  if (!obstacle.IsBlockingObstacle()) {
    return false;
  }

  /**
   * @brief 检查障碍物是否在自车前方
   *
   * min_t() > 0.0:
   *   障碍物的最早出现时间 > 0
   *   说明障碍物在自车前方（时间轴上）
   */
  if (obstacle.path_st_boundary().min_t() > 0.0) {
    return false;
  }

  /**
   * @brief 获取速度和自车速度
   */
  const double obs_speed = obstacle.speed();
  const double ego_speed = init_point_.v();

  /**
   * @brief 如果障碍物速度 > 自车速度，不算跟车过近
   */
  if (obs_speed > ego_speed) {
    return false;
  }

  /**
   * @brief 计算实际距离
   *
   * min_s - FLAGS_min_stop_distance_obstacle:
   *   障碍物最小S距离减去最小停车距离
   */
  const double distance =
      obstacle.path_st_boundary().min_s() - FLAGS_min_stop_distance_obstacle;

  /**
   * @brief 定义减速度常量
   */
  static constexpr double lane_follow_max_decel = 3.0;   /**< 车道跟随最大减速度 */
  static constexpr double lane_change_max_decel = 3.0;     /**< 换道最大减速度 */

  /**
   * @brief 获取当前规划状态
   *
   * injector_->planning_context()->...:
   *   通过依赖注入器访问规划上下文
   * mutable_change_lane():
   *   获取可修改的换道状态
   */
  auto* planning_status = injector_->planning_context()
                              ->mutable_planning_status()
                              ->mutable_change_lane();

  /**
   * @brief 计算安全停车距离的分子
   *
   * numerator = (v_ego - v_obs)² / 2
   * 使用动能公式：d = v² / (2a)
   */
  double distance_numerator = std::pow((ego_speed - obs_speed), 2) * 0.5;

  /**
   * @brief 根据是否换道选择减速度
   */
  double distance_denominator = lane_follow_max_decel;
  if (planning_status->has_status() &&
      planning_status->status() == ChangeLaneStatus::IN_CHANGE_LANE) {
    distance_denominator = lane_change_max_decel;  /**< 换道时使用更大减速度 */
  }

  /**
   * @brief 判断是否过近
   *
   * d < v² / (2 * a)
   * 如果实际距离小于安全距离，则过近
   */
  return distance < distance_numerator / distance_denominator;
}

/**
 * @brief 生成所有障碍物的决策
 *
 * @param speed_profile 速度曲线
 * @param path_decision 路径决策
 * @return Status 生成状态
 *
 * 功能说明：
 * 遍历所有障碍物，为每个障碍物生成纵向决策
 *
 * 决策逻辑：
 * 1. 获取障碍物的ST位置关系（BELOW/ABOVE/CROSS）
 * 2. 根据位置关系和障碍物类型生成相应决策
 * 3. STOP/FOLLOW/YIELD/OVERTAKE/IGNORE
 *
 * C++语法说明：
 * - SpeedData: 速度曲线数据结构
 * - PathDecision* const: 指向常量的指针，可修改指向的对象
 */
Status SpeedDecider::MakeObjectDecision(
    const SpeedData& speed_profile, PathDecision* const path_decision) const {
  /**
   * @brief 检查速度曲线有效性
   *
   * 速度曲线至少需要2个点
   */
  if (speed_profile.size() < 2) {
    const std::string msg = "dp_st_graph failed to get speed profile.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  /**
   * @brief 遍历所有障碍物
   */
  for (const auto* obstacle : path_decision->obstacles().Items()) {
    /**
     * @brief 获取可修改的障碍物指针
     *
     * path_decision->Find():
     *   根据ID查找障碍物
     * 返回可修改的指针
     */
    auto* mutable_obstacle = path_decision->Find(obstacle->Id());

    /**
     * @brief 获取障碍物的ST边界
     */
    const auto& boundary = mutable_obstacle->path_st_boundary();

    /**
     * @brief 检查是否应该忽略障碍物
     *
     * 忽略条件：
     * 1. ST边界为空
     * 2. 最大S < 0
     * 3. 最大T < 0
     * 4. 最早时间 >= 速度曲线末端时间（障碍物在自车后方）
     */
    if (boundary.IsEmpty() || boundary.max_s() < 0.0 ||
        boundary.max_t() < 0.0 ||
        boundary.min_t() >= speed_profile.back().t()) {
      AppendIgnoreDecision(mutable_obstacle);  /**< 添加IGNORE决策 */
      continue;  /**< 继续下一个障碍物 */
    }

    /**
     * @brief 如果已有纵向决策，跳过
     *
     * HasLongitudinalDecision():
     *   检查障碍物是否已有纵向决策
     * 如果有，则不覆盖原有决策
     */
    if (obstacle->HasLongitudinalDecision()) {
      AppendIgnoreDecision(mutable_obstacle);
      continue;
    }

    /**
     * @brief 虚拟障碍物特殊处理
     *
     * Virtual障碍物：人为创建的障碍物（如停车墙）
     * 如果中心点不在车道上，则跳过
     */
    if (obstacle->IsVirtual()) {
      const auto& obstacle_box = obstacle->PerceptionBoundingBox();
      if (!reference_line_->IsOnLane(obstacle_box.center())) {
        continue;  /**< 不在车道上，跳过 */
      }
    }

    /**
     * @brief 行人停车检查
     *
     * 如果配置为停车让行人
     * 且障碍物是行人类型
     */
    if (config_.is_stop_for_pedestrain() &&
        CheckStopForPedestrian(*mutable_obstacle)) {
      /**
       * @brief 创建停车决策
       */
      ObjectDecisionType stop_decision;
      if (CreateStopDecision(*mutable_obstacle, &stop_decision,
                             -FLAGS_min_stop_distance_obstacle)) {
        mutable_obstacle->AddLongitudinalDecision("dp_st_graph/pedestrian",
                                                  stop_decision);
      }
      continue;
    }

    /**
     * @brief 获取ST位置关系
     *
     * GetSTLocation():
     *   判断速度曲线与ST边界的位置关系
     */
    auto location = GetSTLocation(path_decision, speed_profile, boundary);

    /**
     * @brief KEEP_CLEAR阻塞检查
     */
    if (!FLAGS_use_st_drivable_boundary) {
      if (boundary.boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
        if (CheckKeepClearBlocked(path_decision, *obstacle)) {
          location = BELOW;  /**< 被阻塞，视为在下方 */
        }
      }
    }

    /**
     * @brief 根据位置关系生成决策
     *
     * switch-case:
     *   多分支选择语句
     *   根据location的值执行不同分支
     */
    switch (location) {
      /**
       * @brief BELOW情况：自车在障碍物后方
       */
      case BELOW:
        /**
         * @brief KEEP_CLEAR类型：必须停车
         */
        if (boundary.boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
          ObjectDecisionType stop_decision;
          if (CreateStopDecision(*mutable_obstacle, &stop_decision, 0.0)) {
            mutable_obstacle->AddLongitudinalDecision("dp_st_graph/keep_clear",
                                                      stop_decision);
          }
        }
        /**
         * @brief 静态障碍物：停车
         */
        else if (obstacle->IsStatic()) {
          ObjectDecisionType stop_decision;
          if (CreateStopDecision(*mutable_obstacle, &stop_decision,
                                 -FLAGS_min_stop_distance_obstacle)) {
            mutable_obstacle->AddLongitudinalDecision("dp_st_graph",
                                                      stop_decision);
          }
        }
        /**
         * @brief 动态障碍物：判断是否应该跟车
         */
        else if (CheckIsFollow(*obstacle, boundary)) {
          /**
           * @brief 跟车过近：紧急停车
           */
          if (IsFollowTooClose(*mutable_obstacle)) {
            ObjectDecisionType stop_decision;
            if (CreateStopDecision(*mutable_obstacle, &stop_decision,
                                   -FLAGS_min_stop_distance_obstacle)) {
              mutable_obstacle->AddLongitudinalDecision("dp_st_graph/too_close",
                                                        stop_decision);
            }
          }
          /**
           * @brief 正常跟车：生成FOLLOW决策
           */
          else {
            ObjectDecisionType follow_decision;
            if (CreateFollowDecision(*mutable_obstacle, &follow_decision)) {
              mutable_obstacle->AddLongitudinalDecision("dp_st_graph",
                                                        follow_decision);
            }
          }
        }
        /**
         * @brief 其他情况：YIELD决策
         */
        else {
          ObjectDecisionType yield_decision;
          if (CreateYieldDecision(*mutable_obstacle, &yield_decision)) {
            mutable_obstacle->AddLongitudinalDecision("dp_st_graph",
                                                      yield_decision);
          }
        }
        break;  /**< 结束BELOW分支 */

      /**
       * @brief ABOVE情况：自车在障碍物前方
       */
      case ABOVE:
        /**
         * @brief KEEP_CLEAR类型：忽略
         */
        if (boundary.boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
          ObjectDecisionType ignore;
          ignore.mutable_ignore();
          mutable_obstacle->AddLongitudinalDecision("dp_st_graph", ignore);
        }
        /**
         * @brief 其他类型：OVERTAKE决策
         */
        else {
          ObjectDecisionType overtake_decision;
          if (CreateOvertakeDecision(*mutable_obstacle, &overtake_decision)) {
            mutable_obstacle->AddLongitudinalDecision("dp_st_graph/overtake",
                                                      overtake_decision);
          }
        }
        break;

      /**
       * @brief CROSS情况：速度曲线与边界相交
       */
      case CROSS:
        /**
         * @brief 如果是阻塞障碍物，生成停车决策
         *
         * 注意：CROSS表示规划失败
         */
        if (mutable_obstacle->IsBlockingObstacle()) {
          ObjectDecisionType stop_decision;
          if (CreateStopDecision(*mutable_obstacle, &stop_decision,
                                 -FLAGS_min_stop_distance_obstacle)) {
            mutable_obstacle->AddLongitudinalDecision("dp_st_graph/cross",
                                                      stop_decision);
          }
          /**
           * @brief 记录错误
           *
           * CROSS表示无法找到可行的速度曲线
           * 这是规划失败的标志
           */
          const std::string msg =
              absl::StrCat("Failed to find a solution for crossing obstacle: ",
                           mutable_obstacle->Id());
          AERROR << msg;
          return Status(ErrorCode::PLANNING_ERROR, msg);
        }
        break;

      /**
       * @brief default情况：未知位置
       */
      default:
        AERROR << "Unknown position:" << location;
    }

    /**
     * @brief 添加忽略决策
     *
     * AppendIgnoreDecision():
     *   如果还没有纵向决策，添加IGNORE
     *   处理Lateral决策
     */
    AppendIgnoreDecision(mutable_obstacle);
  }

  return Status::OK();  /**< 全部完成 */
}

/**
 * @brief 添加忽略决策
 *
 * @param obstacle 障碍物指针
 *
 * 功能说明：
 * 为障碍物添加IGNORE决策（如果还没有决策）
 * 同时处理纵向和横向决策
 *
 * C++语法说明：
 * - Obstacle*: 可修改的障碍物指针
 * - mutable_xxx(): protobuf的可修改访问器
 */
void SpeedDecider::AppendIgnoreDecision(Obstacle* obstacle) const {
  /**
   * @brief 创建IGNORE决策
   */
  ObjectDecisionType ignore_decision;
  ignore_decision.mutable_ignore();  /**< 设置为IGNORE类型 */

  /**
   * @brief 如果没有纵向决策，添加IGNORE
   */
  if (!obstacle->HasLongitudinalDecision()) {
    obstacle->AddLongitudinalDecision("dp_st_graph", ignore_decision);
  }

  /**
   * @brief 如果没有横向决策，添加IGNORE
   */
  if (!obstacle->HasLateralDecision()) {
    obstacle->AddLateralDecision("dp_st_graph", ignore_decision);
  }
}

/**
 * @brief 创建停车决策
 *
 * @param obstacle 障碍物
 * @param stop_decision 输出：停车决策
 * @param stop_distance 停车距离（负值表示在障碍物后方）
 * @return bool 是否创建成功
 *
 * 功能说明：
 * 为障碍物创建STOP决策
 * 计算停车点的位置和朝向
 *
 * 算法说明：
 * fence_s = adc_SL_end + boundary_min_s + stop_distance
 * 停车点位于参考线上fence_s位置
 *
 * C++语法说明：
 * - const Obstacle&: 常量引用
 * - ObjectDecisionType* const: 指向常量的指针（指针本身是常量）
 */
bool SpeedDecider::CreateStopDecision(const Obstacle& obstacle,
                                      ObjectDecisionType* const stop_decision,
                                      double stop_distance) const {
  /**
   * @brief 获取障碍物的ST边界
   */
  const auto& boundary = obstacle.path_st_boundary();

  /**
   * @brief TODO注释：这是一个bug
   *
   * 不能混用参考线S和路径S
   * 应该用计算得到的参考线S替代boundary.min_s()
   */
  /**
   * @brief 计算停车 fence 的 S 坐标
   *
   * fence_s = adc_SL_end + boundary_min_s + stop_distance
   *
   * 公式解释：
   * - adc_sl_boundary_.end_s(): 自车在后方的SL边界
   * - boundary.min_s(): 障碍物边界的最小S
   * - stop_distance: 停车距离（负值表示在障碍物后方）
   */
  double fence_s = adc_sl_boundary_.end_s() + boundary.min_s() + stop_distance;

  /**
   * @brief KEEP_CLEAR类型特殊处理
   *
   * KEEP_CLEAR的停车位置直接使用感知边界
   */
  if (boundary.boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
    fence_s = obstacle.PerceptionSLBoundary().start_s();
  }

  /**
   * @brief 获取主停车点的S坐标
   *
   * stop_reference_line_s():
   *   路径决策中的主停车点
   * 如果主停车点更远，则忽略当前决策
   */
  const double main_stop_s =
      reference_line_info_->path_decision()->stop_reference_line_s();
  if (main_stop_s < fence_s) {
    ADEBUG << "Stop fence is further away, ignore.";
    return false;  /**< 主停车点更远，忽略 */
  }

  /**
   * @brief 获取fence点的参考线信息
   *
   * GetReferencePoint(fence_s):
   *   获取参考线上fence_s位置的点
   * 包含x, y, heading等信息
   */
  const auto fence_point = reference_line_->GetReferencePoint(fence_s);

  /**
   * @brief 设置STOP决策
   *
   * mutable_stop():
   *   获取stop子消息的可修改访问器
   */
  auto* stop = stop_decision->mutable_stop();

  /**
   * @brief 设置停车距离
   *
   * distance_s:
   *   负值表示在障碍物后方
   */
  stop->set_distance_s(stop_distance);

  /**
   * @brief 设置停车点坐标
   */
  auto* stop_point = stop->mutable_stop_point();
  stop_point->set_x(fence_point.x());
  stop_point->set_y(fence_point.y());
  stop_point->set_z(0.0);

  /**
   * @brief 设置停车朝向
   */
  stop->set_stop_heading(fence_point.heading());

  /**
   * @brief KEEP_CLEAR设置停车原因
   */
  if (boundary.boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
    stop->set_reason_code(StopReasonCode::STOP_REASON_CLEAR_ZONE);
  }

  /**
   * @brief 打印调试信息
   */
  PerceptionObstacle::Type obstacle_type = obstacle.Perception().type();
  ADEBUG << "STOP: obstacle_id[" << obstacle.Id() << "] obstacle_type["
         << PerceptionObstacle_Type_Name(obstacle_type) << "]";

  return true;  /**< 创建成功 */
}

/**
 * @brief 创建跟车决策
 *
 * @param obstacle 障碍物
 * @param follow_decision 输出：跟车决策
 * @return bool 是否创建成功
 *
 * 功能说明：
 * 为障碍物创建FOLLOW决策
 * 保持与障碍物的安全跟车距离
 *
 * 算法说明：
 * follow_distance_s = -EstimateProperFollowGap(follow_speed)
 * 负值表示在障碍物后方
 *
 * C++语法说明：
 * EstimateProperFollowGap():
 *   根据自车速度估算合适的跟车距离
 */
bool SpeedDecider::CreateFollowDecision(
    const Obstacle& obstacle, ObjectDecisionType* const follow_decision) const {
  /**
   * @brief 获取当前速度作为跟车速度
   */
  const double follow_speed = init_point_.v();

  /**
   * @brief 估算合适的跟车距离
   *
   * EstimateProperFollowGap():
   *   根据速度查询跟车距离表
   * 返回值取负表示在障碍物后方
   */
  const double follow_distance_s = -EstimateProperFollowGap(follow_speed);

  /**
   * @brief 获取障碍物ST边界
   */
  const auto& boundary = obstacle.path_st_boundary();

  /**
   * @brief 计算参考线fence S坐标
   */
  const double reference_s =
      adc_sl_boundary_.end_s() + boundary.min_s() + follow_distance_s;

  /**
   * @brief 检查主停车点
   */
  const double main_stop_s =
      reference_line_info_->path_decision()->stop_reference_line_s();
  if (main_stop_s < reference_s) {
    ADEBUG << "Follow reference_s is further away, ignore.";
    return false;
  }

  /**
   * @brief 获取参考点
   */
  auto ref_point = reference_line_->GetReferencePoint(reference_s);

  /**
   * @brief 设置FOLLOW决策
   */
  auto* follow = follow_decision->mutable_follow();
  follow->set_distance_s(follow_distance_s);

  /**
   * @brief 设置fence点坐标
   */
  auto* fence_point = follow->mutable_fence_point();
  fence_point->set_x(ref_point.x());
  fence_point->set_y(ref_point.y());
  fence_point->set_z(0.0);
  follow->set_fence_heading(ref_point.heading());

  /**
   * @brief 打印调试信息
   */
  PerceptionObstacle::Type obstacle_type = obstacle.Perception().type();
  ADEBUG << "FOLLOW: obstacle_id[" << obstacle.Id() << "] obstacle_type["
         << PerceptionObstacle_Type_Name(obstacle_type) << "]";

  return true;
}

/**
 * @brief 创建让行决策
 *
 * @param obstacle 障碍物
 * @param yield_decision 输出：让行决策
 * @return bool 是否创建成功
 *
 * 功能说明：
 * 为障碍物创建YIELD决策
 * 减速等待合适的超车时机
 *
 * 算法说明：
 * yield_distance_s = max(-obstacle_boundary.min_s(), -yield_distance_buffer)
 * 取障碍物距离和配置缓冲区的较大值
 */
bool SpeedDecider::CreateYieldDecision(
    const Obstacle& obstacle, ObjectDecisionType* const yield_decision) const {
  /**
   * @brief 获取障碍物类型
   */
  PerceptionObstacle::Type obstacle_type = obstacle.Perception().type();

  /**
   * @brief 获取配置的死区缓冲距离
   */
  double yield_distance = config_.yield_distance_buffer();

  /**
   * @brief 获取障碍物边界
   */
  const auto& obstacle_boundary = obstacle.path_st_boundary();

  /**
   * @brief 计算让行距离
   *
   * yield_distance_s = max(-obstacle.min_s, -yield_distance)
   * -obstacle.min_s: 障碍物后方距离
   * -yield_distance: 配置的死区
   */
  const double yield_distance_s =
      std::max(-obstacle_boundary.min_s(), -yield_distance);

  /**
   * @brief 计算参考线fence S坐标
   */
  const double reference_line_fence_s =
      adc_sl_boundary_.end_s() + obstacle_boundary.min_s() + yield_distance_s;

  /**
   * @brief 检查主停车点
   */
  const double main_stop_s =
      reference_line_info_->path_decision()->stop_reference_line_s();
  if (main_stop_s < reference_line_fence_s) {
    ADEBUG << "Yield reference_s is further away, ignore.";
    return false;
  }

  /**
   * @brief 获取参考点
   */
  auto ref_point = reference_line_->GetReferencePoint(reference_line_fence_s);

  /**
   * @brief 设置YIELD决策
   */
  auto* yield = yield_decision->mutable_yield();
  yield->set_distance_s(yield_distance_s);
  yield->mutable_fence_point()->set_x(ref_point.x());
  yield->mutable_fence_point()->set_y(ref_point.y());
  yield->mutable_fence_point()->set_z(0.0);
  yield->set_fence_heading(ref_point.heading());

  /**
   * @brief 打印调试信息
   */
  ADEBUG << "YIELD: obstacle_id[" << obstacle.Id() << "] obstacle_type["
         << PerceptionObstacle_Type_Name(obstacle_type) << "]";

  return true;
}

/**
 * @brief 创建超车决策
 *
 * @param obstacle 障碍物
 * @param overtake_decision 输出：超车决策
 * @return bool 是否创建成功
 *
 * 功能说明：
 * 为障碍物创建OVERTAKE决策
 * 加速超越障碍物
 *
 * 算法说明：
 * overtake_distance_s = EstimateProperOvertakingGap(obstacle_speed, adc_speed)
 * 根据障碍物速度和自车速度估算安全超车距离
 *
 * C++语法说明：
 * Vec2d::CreateUnitVec2d(theta):
 *   根据角度创建单位向量
 * InnerProd():
 *   计算两个向量的内积
 */
bool SpeedDecider::CreateOvertakeDecision(
    const Obstacle& obstacle,
    ObjectDecisionType* const overtake_decision) const {
  /**
   * @brief 获取障碍物速度
   *
   * Perception().velocity():
   *   获取感知模块输出的速度向量
   * 需要投影到自车行驶方向
   */
  const auto& velocity = obstacle.Perception().velocity();

  /**
   * @brief 计算障碍物在自车方向上的速度分量
   *
   * 步骤：
   * 1. 创建自车行驶方向的单位向量
   * 2. 计算与速度向量的内积
   * 3. 得到沿自车方向的速度标量
   */
  const double obstacle_speed =
      common::math::Vec2d::CreateUnitVec2d(init_point_.path_point().theta())
          .InnerProd(Vec2d(velocity.x(), velocity.y()));

  /**
   * @brief 估算合适的超车距离
   *
   * EstimateProperOvertakingGap():
   *   根据障碍物速度和自车速度计算安全超车距离
   */
  double overtake_distance_s =
      EstimateProperOvertakingGap(obstacle_speed, init_point_.v());

  /**
   * @brief 获取障碍物边界
   */
  const auto& boundary = obstacle.path_st_boundary();

  /**
   * @brief 计算参考线fence S坐标
   */
  const double reference_line_fence_s =
      adc_sl_boundary_.end_s() + boundary.min_s() + overtake_distance_s;

  /**
   * @brief 检查主停车点
   */
  const double main_stop_s =
      reference_line_info_->path_decision()->stop_reference_line_s();
  if (main_stop_s < reference_line_fence_s) {
    ADEBUG << "Overtake reference_s is further away, ignore.";
    return false;
  }

  /**
   * @brief 获取参考点
   */
  auto ref_point = reference_line_->GetReferencePoint(reference_line_fence_s);

  /**
   * @brief 设置OVERTAKE决策
   */
  auto* overtake = overtake_decision->mutable_overtake();
  overtake->set_distance_s(overtake_distance_s);
  overtake->mutable_fence_point()->set_x(ref_point.x());
  overtake->mutable_fence_point()->set_y(ref_point.y());
  overtake->mutable_fence_point()->set_z(0.0);
  overtake->set_fence_heading(ref_point.heading());

  /**
   * @brief 打印调试信息
   */
  PerceptionObstacle::Type obstacle_type = obstacle.Perception().type();
  ADEBUG << "OVERTAKE: obstacle_id[" << obstacle.Id() << "] obstacle_type["
         << PerceptionObstacle_Type_Name(obstacle_type) << "]";

  return true;
}

/**
 * @brief 检查是否应该跟车
 *
 * @param obstacle 障碍物
 * @param boundary ST边界
 * @return bool 是否应该跟车
 *
 * 功能说明：
 * 判断自车是否应该对障碍物采取跟车策略
 *
 * 判断条件：
 * 1. 障碍物在相邻车道（横向距离较小）
 * 2. 障碍物向自车方向移动
 * 3. 时间范围内有重叠
 *
 * C++语法说明：
 * std::fabs: 浮点数绝对值
 */
bool SpeedDecider::CheckIsFollow(const Obstacle& obstacle,
                                 const STBoundary& boundary) const {
  /**
   * @brief 检查横向距离
   *
   * 障碍物与自车的最小横向距离
   * 如果过大，说明不在同一车道
   */
  const double obstacle_l_distance =
      std::min(std::fabs(obstacle.PerceptionSLBoundary().start_l()),
               std::fabs(obstacle.PerceptionSLBoundary().end_l()));

  /**
   * @brief 横向距离过大，不是跟随场景
   */
  if (obstacle_l_distance > config_.follow_min_obs_lateral_distance()) {
    return false;
  }

  /**
   * @brief 检查障碍物是否向自车移动
   *
   * bottom_left.s() > bottom_right.s():
   *   左边界S > 右边界S，表示障碍物向后移动（相对于自车向前）
   * 这种情况不需要跟随
   */
  if (boundary.bottom_left_point().s() > boundary.bottom_right_point().s()) {
    return false;
  }

  /**
   * @brief 定义时间和截止时间常量
   */
  static constexpr double kFollowTimeEpsilon = 1e-3;    /**< 时间精度阈值 */
  static constexpr double kFollowCutOffTime = 0.5;      /**< 截止时间 */

  /**
   * @brief 检查时间范围
   *
   * min_t > kFollowCutOffTime:
   *   障碍物出现时间太晚
   * max_t < kFollowTimeEpsilon:
   *   障碍物存在时间太短
   */
  if (boundary.min_t() > kFollowCutOffTime ||
      boundary.max_t() < kFollowTimeEpsilon) {
    return false;
  }

  /**
   * @brief 检查是否跨车道且方向相反
   *
   * 如果障碍物存在时间过短
   * 可能是跨车道场景
   */
  if (boundary.max_t() - boundary.min_t() < config_.follow_min_time_sec()) {
    return false;
  }

  return true;  /**< 满足跟车条件 */
}

/**
 * @brief 检查是否为行人停车
 *
 * @param obstacle 障碍物
 * @return bool 是否应该为行人停车
 *
 * 功能说明：
 * 行人具有最高优先级，必须完全停车等待
 * 使用计时器机制：看到行人后开始计时
 * 如果行人保持静止超过阈值时间，则可以通过
 *
 * 算法说明：
 * 1. 检查是否是行人类型
 * 2. 检查行人是否在自车前方
 * 3. 使用计时器：10米内开始计时
 * 4. 行人速度>阈值，重置计时器
 * 5. 行人速度<阈值且计时超过4秒，停止等待
 *
 * C++语法说明：
 * std::hypot: 计算平方和的平方根（欧几里得距离）
 * Clock::NowInSeconds(): 获取当前时间戳
 * std::unordered_map: 哈希表，O(1)查找
 */
bool SpeedDecider::CheckStopForPedestrian(const Obstacle& obstacle) const {
  /**
   * @brief 检查障碍物类型
   *
   * PerceptionObstacle::PEDESTRIAN:
   *   行人类型
   */
  const auto& perception_obstacle = obstacle.Perception();
  if (perception_obstacle.type() != PerceptionObstacle::PEDESTRIAN) {
    return false;
  }

  /**
   * @brief 检查行人是否在自车前方
   *
   *行人SL边界的end_s与自车SL边界的start_s比较
   */
  const auto& obstacle_sl_boundary = obstacle.PerceptionSLBoundary();
  if (obstacle_sl_boundary.end_s() < adc_sl_boundary_.start_s()) {
    return false;  /**< 行人在自车后方 */
  }

  /**
   * @brief 从PlanningContext读取行人停车时间
   *
   * 用于跨帧持续跟踪同一行人
   */
  auto* mutable_speed_decider_status = injector_->planning_context()
                                           ->mutable_planning_status()
                                           ->mutable_speed_decider();

  /**
   * @brief 创建行人ID到停车时间的映射
   */
  std::unordered_map<std::string, double> stop_time_map;
  for (const auto& pedestrian_stop_time :
       mutable_speed_decider_status->pedestrian_stop_time()) {
    stop_time_map[pedestrian_stop_time.obstacle_id()] =
        pedestrian_stop_time.stop_timestamp_sec();
  }

  /**
   * @brief 获取障碍物ID
   */
  const std::string& obstacle_id = obstacle.Id();

  /**
   * @brief 定义计时器相关常量
   */
  static constexpr double kSDistanceStartTimer = 10.0;   /**< 开始计时的距离阈值 */
  static constexpr double kMaxStopSpeed = 0.3;             /**< 视为停止的速度阈值 */
  static constexpr double kPedestrianStopTimeout = 4.0;    /**< 停车等待超时时间 */

  /**
   * @brief 初始化结果为需要停车
   */
  bool result = true;

  /**
   * @brief 如果行人在10米内，开始计时器逻辑
   */
  if (obstacle.path_st_boundary().min_s() < kSDistanceStartTimer) {
    /**
     * @brief 计算行人速度
     *
     * std::hypot:
     *   计算速度向量的模（欧几里得距离）
     */
    const auto obstacle_speed = std::hypot(perception_obstacle.velocity().x(),
                                           perception_obstacle.velocity().y());

    /**
     * @brief 如果行人还在移动，重置计时器
     */
    if (obstacle_speed > kMaxStopSpeed) {
      stop_time_map.erase(obstacle_id);  /**< 从计时器映射中移除 */
    } else {
      /**
       * @brief 如果行人已停止
       */
      if (stop_time_map.count(obstacle_id) == 0) {
        /**
         * @brief 第一次看到停止的行人，添加时间戳
         */
        stop_time_map[obstacle_id] = Clock::NowInSeconds();
        ADEBUG << "add timestamp: obstacle_id[" << obstacle_id << "] timestamp["
               << Clock::NowInSeconds() << "]";
      } else {
        /**
         * @brief 检查超时
         */
        double stop_timer = Clock::NowInSeconds() - stop_time_map[obstacle_id];
        ADEBUG << "stop_timer: obstacle_id[" << obstacle_id << "] stop_timer["
               << stop_timer << "]";
        if (stop_timer >= kPedestrianStopTimeout) {
          result = false;  /**< 超时，不再等待 */
        }
      }
    }
  }

  /**
   * @brief 将计时器状态写回PlanningContext
   *
   * 用于跨帧传递状态
   */
  mutable_speed_decider_status->mutable_pedestrian_stop_time()->Clear();
  for (const auto& stop_time : stop_time_map) {
    auto pedestrian_stop_time =
        mutable_speed_decider_status->add_pedestrian_stop_time();
    pedestrian_stop_time->set_obstacle_id(stop_time.first);
    pedestrian_stop_time->set_stop_timestamp_sec(stop_time.second);
  }

  return result;
}

/**
 * @brief 估算合适的超车距离
 *
 * @param target_obs_speed 目标障碍物速度
 * @param adc_speed 自车速度
 * @return double 超车距离
 *
 * 功能说明：
 * 根据障碍物速度和自车速度计算安全超车距离
 *
 * 算法公式：
 * overtake_distance = max( max(adc_speed, target_obs_speed) * overtake_time_buffer,
 *                           overtake_min_distance )
 *
 * C++语法说明：
 * std::fmax: 浮点数取较大值
 */
double SpeedDecider::EstimateProperOvertakingGap(const double target_obs_speed,
                                                 const double adc_speed) const {
  /**
   * @brief 计算超车距离
   *
   * 取以下两者的较大值：
   * 1. 根据速度和时间缓冲计算的距离
   * 2. 最小超车距离
   */
  const double overtake_distance_s = std::fmax(
      std::fmax(adc_speed, target_obs_speed) * config_.overtake_time_buffer(),
      config_.overtake_min_distance());

  return overtake_distance_s;
}

/**
 * @brief 估算合适的跟车距离
 *
 * @param adc_speed 自车速度
 * @return double 跟车距离
 *
 * 功能说明：
 * 根据自车速度查表计算合适的跟车距离
 *
 * 算法说明：
 * 使用分段线性函数近似跟车距离
 * follow_distance = base + Σ(slope_i * speed_i)
 *
 * C++语法说明：
 * for循环中的break语句
 * 二分查找逻辑
 */
double SpeedDecider::EstimateProperFollowGap(const double& adc_speed) const {
  /**
   * @brief 获取基础跟车距离
   */
  double follow_distance = config_.stop_follow_distance();

  /**
   * @brief 遍历跟车距离函数表
   *
   * follow_distance_function_:
   *   按速度排序的(speed, slope)对列表
   * 使用分段线性插值
   */
  for (int i = 1; i < follow_distance_function_.size(); i++) {
    /**
     * @brief 如果当前速度 <= 第i个速度阈值
     * 使用第i-1个区间的斜率
     */
    if (adc_speed <= follow_distance_function_[i].first) {
      follow_distance += follow_distance_function_[i - 1].second *
                         (adc_speed - follow_distance_function_[i - 1].first);
      break;  /**< 找到合适的区间，停止搜索 */
    } else {
      /**
       * @brief 累加完整区间的距离
       */
      follow_distance += follow_distance_function_[i - 1].second *
                         (follow_distance_function_[i].first -
                          follow_distance_function_[i - 1].first);
    }
  }

  /**
   * @brief 处理速度超出表范围的情况
   *
   * 如果速度大于最后一个阈值
   * 使用最后一个区间的斜率外推
   */
  if (adc_speed > follow_distance_function_.back().first) {
    follow_distance += follow_distance_function_.back().second *
                       (adc_speed - follow_distance_function_.back().first);
  }

  /**
   * @brief 打印调试信息
   */
  AINFO << "follow_distance: " << follow_distance;

  return follow_distance;
}

}  // namespace planning
}  // namespace apollo
