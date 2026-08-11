/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
 * @file obstacle_blocking_analyzer.cc
 * @brief 障碍物阻塞分析器实现文件
 *
 * 功能说明：
 * 分析障碍物是否阻塞自车行驶路径
 * 支持借道超车场景的障碍物判断
 *
 * 应用场景：
 * - 车道借道超车（Lane Borrow）
 * - 开放空间规划
 * - 路径决策
 *
 * 核心判断逻辑：
 * 1. 非移动障碍物判断
 * 2. 阻塞障碍物判断
 * 3. 停车障碍物判断
 * 4. 障碍物与路口距离判断
 */

/**
 * @brief 当前文件的实现
 *
 * C++语法说明：
 * - #include "modules/planning/planning_base/common/obstacle_blocking_analyzer.h"：
 *   包含对应头文件
 *   定义了类的接口和声明
 */
#include "modules/planning/planning_base/common/obstacle_blocking_analyzer.h"

/**
 * @brief C++标准库算法头文件
 *
 * C++语法说明：
 * - #include <algorithm>：
 *   标准库算法头文件
 *   提供了std::max、std::min、std::sort等
 *   本文件中使用了std::max计算最大值
 */
#include <algorithm>

/**
 * @brief C++标准库智能指针头文件
 *
 * C++语法说明：
 * - #include <memory>：
 *   智能指针头文件
 *   提供了std::shared_ptr、std::unique_ptr
 */
#include <memory>

/**
 * @brief C++标准库向量容器头文件
 *
 * C++语法说明：
 * - #include <vector>：
 *   动态数组容器头文件
 *   std::vector是C++中最常用的容器之一
 */
#include <vector>

/**
 * @brief C++标准库数值限制头文件
 *
 * C++语法说明：
 * - #include <limits>：
 *   数值极限头文件
 *   提供了std::numeric_limits模板类
 *   用于获取类型的最大值、最小值等
 */
#include <limits>

/**
 * @brief Apollo车辆配置助手头文件
 *
 * 功能说明：
 * - VehicleConfigHelper类：
 *   提供车辆参数的访问接口
 *   如车辆宽度、高度等
 *
 * C++语法说明：
 * - Instance()：
 *   单例模式获取实例
 */
#include "modules/common/configs/vehicle_config_helper.h"

/**
 * @brief Apollo点工厂头文件
 *
 * 功能说明：
 * - PointFactory类：
 *   提供便捷创建各种类型点的工厂函数
 *
 * C++语法说明：
 * - PointFactory::ToPointENU：
 *   静态工厂方法创建ENU坐标点
 */
#include "modules/common/util/point_factory.h"

/**
 * @brief Apollo HD地图工具头文件
 *
 * 功能说明：
 * - HDMapUtil类：
 *   提供HD地图的访问接口
 *
 * C++语法说明：
 * - HDMapUtil::BaseMapPtr()：
 *   静态方法获取地图指针
 */
#include "modules/map/hdmap/hdmap_util.h"

/**
 * @brief 规划帧头文件
 *
 * 功能说明：
 * - Frame类：
 *   包含当前帧的所有规划相关信息
 */
#include "modules/planning/planning_base/common/frame.h"

/**
 * @brief 规划全局配置flags头文件
 *
 * 功能说明：
 * - planning_gflags.h：
 *   定义规划模块使用的gflags全局变量
 *   如FLAGS_enable_scenario_side_pass_multiple_parked_obstacles
 */
#include "modules/planning/planning_base/gflags/planning_gflags.h"

/**
 * @namespace apollo::planning
 * @brief Apollo规划模块命名空间
 */
namespace apollo {
namespace planning {

/**
 * @brief 类型别名声明
 *
 * 功能说明：
 * 为常用类型创建别名，简化代码书写
 *
 * C++语法说明：
 * - using XXX = YYY：
 *   类型别名声明，与typedef等价但更直观
 */
using apollo::common::VehicleConfigHelper;
using apollo::hdmap::HDMapUtil;

/**
 * @brief 自车距离阈值常量
 *
 * 功能说明：
 * 定义障碍物被认为是"前方"的距离阈值
 * 单位：米
 *
 * C++语法说明：
 * - constexpr：
 *   编译时常量
 *   与const的区别：
 *   - const：运行时常量
 *   - constexpr：编译时可确定值
 *   - 更适合用于性能关键的常量
 *
 * - double kAdcDistanceThreshold = 35.0：
 *   全局常量，命名约定：k开头 + 大驼峰
 *   35.0米，表示自车前方35米内的障碍物才需要考虑
 */
constexpr double kAdcDistanceThreshold = 35.0;  // unit: m

/**
 * @brief 障碍物间距离阈值
 *
 * 功能说明：
 * 判断两个障碍物是否相互阻塞的距离阈值
 * 单位：米
 */
constexpr double kObstaclesDistanceThreshold = 15.0;

/**
 * @brief 路口清除距离阈值
 *
 * 功能说明：
 * 判断障碍物是否离路口太近的距离阈值
 * 如果太近，不建议借道
 * 单位：米
 */
constexpr double kIntersectionClearanceDist = 20.0;

/**
 * @brief 环岛清除距离阈值
 *
 * 功能说明：
 * 判断障碍物是否离环岛太近的距离阈值
 * 单位：米
 */
constexpr double kJunctionClearanceDist = 15.0;

/**
 * @brief 判断障碍物是否为非移动障碍物
 *
 * @param reference_line_info 参考线信息
 * @param obstacle 要判断的障碍物
 * @return bool 如果是非移动障碍物返回true
 *
 * 功能说明：
 * 判断一个障碍物是否被认为是"非移动的"
 * 包括：停车车辆、被其他障碍物阻挡的车辆
 *
 * 算法流程：
 * 1. 检查障碍物是否在前方（太远直接返回false）
 * 2. 检查是否是停车车辆
 * 3. 检查是否被其他障碍物阻挡
 *
 * C++语法说明：
 * - const ReferenceLineInfo& reference_line_info：
 *   常量引用，传入参考线信息
 *   const保证不会被修改，引用避免拷贝
 *
 * - const Obstacle& obstacle：
 *   常量引用，传入障碍物
 *
 * - const SLBoundary& adc_sl_boundary = reference_line_info.AdcSlBoundary()：
 *   创建常量引用获取自车的SL边界
 *   AdcSlBoundary返回自车在SL坐标系下的边界框
 *
 * - obstacle.PerceptionSLBoundary().start_s()：
 *   点运算符调用成员函数
 *   PerceptionSLBoundary返回障碍物的感知SL边界
 *   start_s()获取边界在s方向的起始值
 *
 * - adc_sl_boundary.end_s() + kAdcDistanceThreshold：
 *   自车后边缘s坐标 + 距离阈值
 *   end_s()获取边界在s方向的最大值
 *
 * - ADEBUG << ...：
 *   Apollo调试日志宏
 *   输出调试信息
 *
 * - return false / return true：
 *   返回bool值表示判断结果
 */
bool IsNonmovableObstacle(const ReferenceLineInfo& reference_line_info,
                          const Obstacle& obstacle) {
  /**
   * 检查障碍物是否太远
   *
   * 如果障碍物的起始s坐标 > 自车后边缘s + 阈值
   * 说明障碍物在35米之外，不确定其状态
   */
  const SLBoundary& adc_sl_boundary = reference_line_info.AdcSlBoundary();
  if (obstacle.PerceptionSLBoundary().start_s() >
      adc_sl_boundary.end_s() + kAdcDistanceThreshold) {
    ADEBUG << " - It is too far ahead and we are not so sure of its status.";
    return false;
  }

  /**
   * 检查是否是停车车辆
   *
   * IsParkedVehicle函数判断是否是停车车辆
   * 停车车辆被认为是不可移动的
   */
  if (IsParkedVehicle(reference_line_info.reference_line(), &obstacle)) {
    ADEBUG << "It is Parked and NON-MOVABLE.";
    return true;
  }

  /**
   * 检查障碍物是否被其他障碍物阻挡
   *
   * 遍历所有其他障碍物
   * 检查是否有障碍物在自车道上阻挡了该障碍物
   */
  for (const auto* other_obstacle :
       reference_line_info.path_decision().obstacles().Items()) {
    // 跳过自身
    if (other_obstacle->Id() == obstacle.Id()) {
      continue;
    }
    // 跳过虚拟障碍物
    if (other_obstacle->IsVirtual()) {
      continue;
    }
    // 只考虑车辆类型障碍物
    if (other_obstacle->Perception().type() !=
        apollo::perception::PerceptionObstacle::VEHICLE) {
      continue;
    }
    // 获取边界框
    const auto& other_boundary = other_obstacle->PerceptionSLBoundary();
    const auto& this_boundary = obstacle.PerceptionSLBoundary();

    /**
     * 检查是否有横向重叠
     *
     * 如果other在this的侧面（l方向不重叠）
     * 则不构成阻挡
     *
     * C++语法说明：
     * - other_boundary.start_l() > this_boundary.end_l()：
     *   other在this的右侧（l值更大）
     * - other_boundary.end_l() < this_boundary.start_l()：
     *   other在this的左侧（l值更小）
     */
    if (other_boundary.start_l() > this_boundary.end_l() ||
        other_boundary.end_l() < this_boundary.start_l()) {
      // not blocking the backside vehicle
      continue;
    }
    /**
     * 计算障碍物之间的距离
     *
     * delta_s = other起始s - this结束s
     * 如果delta_s > 0，说明other在this前方
     * 如果delta_s太小（< 0），说明other在this后方
     * 如果delta_s太大（> 15m），说明距离太远
     */
    double delta_s = other_boundary.start_s() - this_boundary.end_s();
    if (delta_s < 0.0 || delta_s > kObstaclesDistanceThreshold) {
      continue;
    }
    // 如果有障碍物阻挡，则不是非移动障碍物
    return false;
  }
  ADEBUG << "IT IS NON-MOVABLE!";
  return true;
}

/**
 * @brief 判断障碍物是否为需要借道的阻塞障碍物
 *
 * @param frame 规划帧数据
 * @param obstacle 要判断的障碍物指针
 * @param block_obstacle_min_speed 阻塞障碍物最小速度阈值
 * @param min_front_sidepass_distance 最小借道前方距离
 * @param enable_obstacle_blocked_check 是否启用障碍物阻挡检查
 * @return bool 如果是阻塞障碍物返回true
 *
 * 功能说明：
 * 这是借道场景中判断每个障碍物是否需要借道的核心函数
 *
 * 阻塞条件（全部满足才算阻塞）：
 * 1. 不是虚拟障碍物
 * 2. 是静态障碍物（或速度很低）
 * 3. 在自车前方
 * 4. 在一定距离内（不太远也不太近）
 * 5. 阻塞了自车的行驶路径
 * 6. 没有被其他障碍物阻挡
 *
 * C++语法说明：
 * - const Frame& frame：
 *   常量引用，传入规划帧
 *
 * - const Obstacle* obstacle：
 *   指向常量障碍物的指针
 *   指针允许空值，需要检查
 *
 * - double block_obstacle_min_speed：
 *   速度阈值，低于此速度被认为是"静止"
 *
 * - bool enable_obstacle_blocked_check：
 *   是否执行障碍物相互阻挡检查
 */
bool IsBlockingObstacleToSidePass(const Frame& frame, const Obstacle* obstacle,
                                  double block_obstacle_min_speed,
                                  double min_front_sidepass_distance,
                                  bool enable_obstacle_blocked_check) {
  /**
   * 获取必要信息
   *
   * C++语法说明：
   * - const auto& reference_line_info = frame.reference_line_info().front()：
   *   auto自动类型推导
   *   const auto&常量引用，避免拷贝
   *   front()获取第一条参考线
   *
   * - const SLBoundary& adc_sl_boundary = reference_line_info.AdcSlBoundary()：
   *   获取自车SL边界
   */
  const auto& reference_line_info = frame.reference_line_info().front();
  const auto& reference_line = reference_line_info.reference_line();
  const SLBoundary& adc_sl_boundary = reference_line_info.AdcSlBoundary();
  const PathDecision& path_decision = reference_line_info.path_decision();
  ADEBUG << "Evaluating Obstacle: " << obstacle->Id();

  /**
   * 条件1：障碍物是虚拟障碍物
   *
   * 虚拟障碍物通常是临时创建的，不应该阻塞
   */
  if (obstacle->IsVirtual()) {
    ADEBUG << " - It is virtual.";
    return false;
  }

  /**
   * 条件2：障碍物是移动的
   *
   * IsStatic()检查是否是静态
   * speed() > block_obstacle_min_speed检查速度是否太慢
   * 移动的或速度太快的障碍物不应该被借道绕过
   */
  if (!obstacle->IsStatic() || obstacle->speed() > block_obstacle_min_speed) {
    ADEBUG << " - It is non-static.";
    return false;
  }

  /**
   * 条件3：障碍物在自车后方
   *
   * start_s是障碍物的起始s坐标
   * end_s是自车的后边缘s坐标
   * 如果障碍物起始s <= 自车后边缘s，说明在后方
   */
  if (obstacle->PerceptionSLBoundary().start_s() <= adc_sl_boundary.end_s()) {
    ADEBUG << " - It is behind ADC.";
    return false;
  }

  /**
   * 条件4：障碍物太远
   *
   * 静态常量，表示借道场景的距离阈值
   * 超过此距离的障碍物不需要考虑
   */
  static constexpr double kAdcDistanceSidePassThreshold = 15.0;
  if (obstacle->PerceptionSLBoundary().start_s() >
      adc_sl_boundary.end_s() + kAdcDistanceSidePassThreshold) {
    ADEBUG << " - It is too far ahead.";
    return false;
  }

  /**
   * 条件5：障碍物太近
   *
   * 如果距离小于最小借道距离，不适合借道
   * 需要保持安全距离
   */
  if (adc_sl_boundary.end_s() + min_front_sidepass_distance >
      obstacle->PerceptionSLBoundary().start_s()) {
    ADEBUG << " - It is too close to side-pass.";
    return false;
  }

  /**
   * 条件6：障碍物没有阻塞行驶路径
   *
   * IsBlockingDrivingPathObstacle判断是否有足够行驶宽度
   */
  if (!IsBlockingDrivingPathObstacle(reference_line, obstacle)) {
    ADEBUG << " - It is not blocking our way.";
    return false;
  }

  /**
   * 条件7：障碍物被其他障碍物阻挡
   *
   * 如果启用阻挡检查，且障碍物不是停车车辆
   * 需要检查是否有其他障碍物在前方阻挡了该障碍物
   */
  if (enable_obstacle_blocked_check &&
      !IsParkedVehicle(reference_line, obstacle)) {
    for (const auto* other_obstacle : path_decision.obstacles().Items()) {
      if (other_obstacle->Id() == obstacle->Id()) {
        continue;
      }
      if (other_obstacle->IsVirtual()) {
        continue;
      }
      /**
       * 检查l方向是否有重叠
       * 如果没有重叠，说明other在侧面，不构成阻挡
       */
      if (other_obstacle->PerceptionSLBoundary().start_l() >
              obstacle->PerceptionSLBoundary().end_l() ||
          other_obstacle->PerceptionSLBoundary().end_l() <
              obstacle->PerceptionSLBoundary().start_l()) {
        // not blocking the backside vehicle
        continue;
      }
      /**
       * 计算距离
       * other起始s - obstacle结束s
       */
      double delta_s = other_obstacle->PerceptionSLBoundary().start_s() -
                       obstacle->PerceptionSLBoundary().end_s();
      if (delta_s < 0.0 || delta_s > kAdcDistanceThreshold) {
        continue;
      }

      ADEBUG << " - It is blocked by others, too.";
      return false;
    }
  }

  ADEBUG << "IT IS BLOCKING!";
  return true;
}

/**
 * @brief 获取自车与障碍物之间的距离
 *
 * @param frame 规划帧数据
 * @param obstacle 障碍物指针
 * @return double 距离值（米）
 *
 * 功能说明：
 * 计算自车前端与障碍物后端之间的距离
 *
 * C++语法说明：
 * - const Frame& frame：
 *   常量引用
 *
 * - const Obstacle* obstacle：
 *   指向常量的指针
 *
 * - double distance_between_adc_and_obstacle =
 *     obstacle->PerceptionSLBoundary().start_s() - adc_sl_boundary.end_s()：
 *   计算距离：障碍物起始s - 自车后边缘s
 */
double GetDistanceBetweenADCAndObstacle(const Frame& frame,
                                        const Obstacle* obstacle) {
  const auto& reference_line_info = frame.reference_line_info().front();
  const SLBoundary& adc_sl_boundary = reference_line_info.AdcSlBoundary();
  double distance_between_adc_and_obstacle =
      obstacle->PerceptionSLBoundary().start_s() - adc_sl_boundary.end_s();
  return distance_between_adc_and_obstacle;
}

/**
 * @brief 判断障碍物是否阻塞行驶路径
 *
 * @param reference_line 参考线
 * @param obstacle 障碍物指针
 * @return bool 如果阻塞返回true
 *
 * 功能说明：
 * 根据障碍物的SL边界和道路宽度
 * 判断障碍物是否占据了足够的道路空间导致无法通行
 *
 * 算法流程：
 * 1. 获取障碍物处的可行驶宽度
 * 2. 获取自车宽度
 * 3. 比较可行驶宽度与（自车宽度+缓冲）
 *
 * C++语法说明：
 * - reference_line.GetDrivingWidth(obstacle->PerceptionSLBoundary())：
 *   获取指定位置的可行驶宽度
 *
 * - VehicleConfigHelper::GetConfig().vehicle_param().width()：
 *   单例获取车辆配置
 *   链式调用获取车辆宽度
 */
bool IsBlockingDrivingPathObstacle(const ReferenceLine& reference_line,
                                   const Obstacle* obstacle) {
  /**
   * 获取可行驶宽度
   *
   * GetDrivingWidth方法计算障碍物占据后的可行驶宽度
   */
  const double driving_width =
      reference_line.GetDrivingWidth(obstacle->PerceptionSLBoundary());

  /**
   * 获取自车宽度
   */
  const double adc_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width();

  ADEBUG << " (driving width = " << driving_width
         << ", adc_width = " << adc_width << ")";

  /**
   * 判断可行驶宽度是否足够
   *
   * 如果 driving_width > adc_width + 缓冲值
   * 说明有足够空间通行，不是阻塞障碍物
   *
   * C++语法说明：
   * - FLAGS_static_obstacle_nudge_l_buffer：
   *   gflags全局变量
   *   静态障碍物绕行时的l方向缓冲
   * - FLAGS_side_pass_driving_width_l_buffer：
   *   借道时的额外缓冲
   */
  if (driving_width > adc_width + FLAGS_static_obstacle_nudge_l_buffer +
                          FLAGS_side_pass_driving_width_l_buffer) {
    ADEBUG << "It is NOT blocking our path.";
    return false;
  }

  ADEBUG << "It is blocking our path.";
  return true;
}

/**
 * @brief 判断障碍物是否是停车车辆
 *
 * @param reference_line 参考线
 * @param obstacle 障碍物指针
 * @return bool 如果是停车车辆返回true
 *
 * 功能说明：
 * 判断一个障碍物是否是停在路边的车辆
 * 包括：在停车场车道上的车、在道路边缘的车
 *
 * C++语法说明：
 * - FLAGS_enable_scenario_side_pass_multiple_parked_obstacles：
 *   全局flag，控制是否启用多停车障碍物借道
 */
bool IsParkedVehicle(const ReferenceLine& reference_line,
                     const Obstacle* obstacle) {
  // 检查是否启用了多停车障碍物借道
  if (!FLAGS_enable_scenario_side_pass_multiple_parked_obstacles) {
    return false;
  }

  /**
   * 获取道路宽度
   *
   * GetRoadWidth方法返回指定s位置的道路左右宽度
   */
  double road_left_width = 0.0;
  double road_right_width = 0.0;
  double max_road_right_width = 0.0;

  // 在障碍物起始s位置获取道路宽度
  reference_line.GetRoadWidth(obstacle->PerceptionSLBoundary().start_s(),
                              &road_left_width, &road_right_width);
  max_road_right_width = road_right_width;

  // 在障碍物结束s位置获取道路宽度
  reference_line.GetRoadWidth(obstacle->PerceptionSLBoundary().end_s(),
                              &road_left_width, &road_right_width);
  max_road_right_width = std::max(max_road_right_width, road_right_width);

  /**
   * 判断是否在道路边缘
   *
   * std::abs计算绝对值
   * 如果障碍物的start_l接近道路右边缘（max_road_right_width）
   * 则认为是在道路边缘停车
   */
  bool is_at_road_edge = std::abs(obstacle->PerceptionSLBoundary().start_l()) >
                         max_road_right_width - 0.1;

  /**
   * 检查是否在停车道上
   *
   * 获取障碍物所在位置的车道信息
   */
  std::vector<std::shared_ptr<const hdmap::LaneInfo>> lanes;
  auto obstacle_box = obstacle->PerceptionBoundingBox();

  /**
   * PointFactory::ToPointENU：
   * 创建ENU坐标点
   * GetLanes获取该点附近的车道列表
   */
  HDMapUtil::BaseMapPtr()->GetLanes(
      common::util::PointFactory::ToPointENU(obstacle_box.center().x(),
                                             obstacle_box.center().y()),
      std::min(obstacle_box.width(), obstacle_box.length()), &lanes);

  bool is_on_parking_lane = false;

  /**
   * 判断是否在停车场车道上
   *
   * lanes.size() == 1：只有一个匹配车道
   * lanes.front()->lane().type() == PARKING：车道类型是停车场
   */
  if (lanes.size() == 1 &&
      lanes.front()->lane().type() == apollo::hdmap::Lane::PARKING) {
    is_on_parking_lane = true;
  }

  /**
   * 综合判断
   *
   * 停车车辆条件：
   * - 在停车道上 或者 在道路边缘
   * - 同时是静态障碍物
   */
  bool is_parked = is_on_parking_lane || is_at_road_edge;
  return is_parked && obstacle->IsStatic();
}

/**
 * @brief 判断阻塞障碍物是否远离路口
 *
 * @param reference_line_info 参考线信息
 * @param blocking_obstacle_id 阻塞障碍物ID
 * @return bool 如果远离路口返回true
 *
 * 功能说明：
 * 检查阻塞障碍物是否离交通信号灯或停车标志足够远
 * 如果太近，不建议借道
 *
 * C++语法说明：
 * - const std::string& blocking_obstacle_id：
 *   常量引用，障碍物ID字符串
 *
 * - blocking_obstacle_id.empty()：
 *   检查字符串是否为空
 */
bool IsBlockingObstacleFarFromIntersection(
    const ReferenceLineInfo& reference_line_info,
    const std::string& blocking_obstacle_id) {
  // 检查ID是否为空
  if (blocking_obstacle_id.empty()) {
    ADEBUG << "There is no blocking obstacle.";
    return true;
  }

  /**
   * 在路径决策中查找障碍物
   *
   * obstacles().Find(id)根据ID查找障碍物
   * 返回指针，如果未找到返回nullptr
   */
  const Obstacle* blocking_obstacle =
      reference_line_info.path_decision().obstacles().Find(
          blocking_obstacle_id);
  if (blocking_obstacle == nullptr) {
    ADEBUG << "Blocking obstacle is no longer there.";
    return true;
  }

  /**
   * 获取阻塞障碍物的s坐标
   *
   * PerceptionSLBoundary().end_s()获取障碍物后边缘的s坐标
   */
  double blocking_obstacle_s =
      blocking_obstacle->PerceptionSLBoundary().end_s();
  ADEBUG << "Blocking obstacle is at s = " << blocking_obstacle_s;

  /**
   * 获取遇到的第一个重叠区域
   *
   * FirstEncounteredOverlaps返回路径上遇到的重叠区域列表
   * 包括：信号灯、停车标志、路口等
   */
  const auto& first_encountered_overlaps =
      reference_line_info.FirstEncounteredOverlaps();

  /**
   * 遍历所有重叠区域
   *
   * for (const auto& overlap : first_encountered_overlaps)：
   *   范围for循环遍历重叠区域
   *   overlap.first是重叠类型
   *   overlap.second是重叠区域数据
   */
  for (const auto& overlap : first_encountered_overlaps) {
    ADEBUG << overlap.first << ", " << overlap.second.DebugString();

    /**
     * 只考虑信号灯和停车标志
     *
     * C++语法说明：
     * - overlap.first != ReferenceLineInfo::SIGNAL：
     *   比较重叠类型
     *   如果不是信号灯也不是停车标志，跳过
     */
    if (overlap.first != ReferenceLineInfo::SIGNAL &&
        overlap.first != ReferenceLineInfo::STOP_SIGN) {
      continue;
    }

    /**
     * 计算距离
     *
     * overlap.second.start_s是重叠区域的起始s
     * blocking_obstacle_s是障碍物的结束s
     */
    auto distance = overlap.second.start_s - blocking_obstacle_s;

    /**
     * 判断距离是否太近
     *
     * 信号灯/停车标志：需要20米距离
     * 其他重叠区域：需要15米距离
     */
    if (overlap.first == ReferenceLineInfo::SIGNAL ||
        overlap.first == ReferenceLineInfo::STOP_SIGN) {
      if (distance < kIntersectionClearanceDist) {
        ADEBUG << "Too close to signal intersection (" << distance
               << "m); don't SIDE_PASS.";
        return false;
      }
    } else {
      if (distance < kJunctionClearanceDist) {
        ADEBUG << "Too close to overlap_type[" << overlap.first << "] ("
               << distance << "m); don't SIDE_PASS";
        return false;
      }
    }
  }

  return true;
}

/**
 * @brief 计算阻塞障碍物到路口的距离
 *
 * @param reference_line_info 参考线信息
 * @param blocking_obstacle_id 阻塞障碍物ID
 * @return double 到路口的最小距离（米）
 *
 * 功能说明：
 * 计算阻塞障碍物到第一个遇到的信号灯或停车标志的距离
 *
 * C++语法说明：
 * - std::numeric_limits<double>::max()：
 *   获取double类型的最大值
 *   用于初始化最小距离
 */
double DistanceBlockingObstacleToIntersection(
    const ReferenceLineInfo& reference_line_info,
    const std::string& blocking_obstacle_id) {
  if (blocking_obstacle_id.empty()) {
    ADEBUG << "There is no blocking obstacle.";
    return true;
  }

  const Obstacle* blocking_obstacle =
      reference_line_info.path_decision().obstacles().Find(
          blocking_obstacle_id);
  if (blocking_obstacle == nullptr) {
    ADEBUG << "Blocking obstacle is no longer there.";
    return true;
  }

  // 获取障碍物结束s坐标
  double blocking_obstacle_s =
      blocking_obstacle->PerceptionSLBoundary().end_s();

  // 初始化最小距离为double最大值
  double min_distance = std::numeric_limits<double>::max();

  ADEBUG << "Blocking obstacle is at s = " << blocking_obstacle_s;

  const auto& first_encountered_overlaps =
      reference_line_info.FirstEncounteredOverlaps();

  for (const auto& overlap : first_encountered_overlaps) {
    ADEBUG << overlap.first << ", " << overlap.second.DebugString();

    // 只考虑信号灯和停车标志
    if (overlap.first != ReferenceLineInfo::SIGNAL &&
        overlap.first != ReferenceLineInfo::STOP_SIGN) {
      continue;
    }

    /**
     * 计算距离并更新最小值
     *
     * std::min比较两个值，返回较小的
     */
    min_distance =
        std::min(min_distance, overlap.second.start_s - blocking_obstacle_s);
  }

  return min_distance;
}

/**
 * @brief 计算阻塞障碍物到环岛的距离
 *
 * @param reference_line_info 参考线信息
 * @param blocking_obstacle_id 阻塞障碍物ID
 * @return double 到环岛的最小距离（米）
 *
 * 功能说明：
 * 计算阻塞障碍物到最近环岛的距离
 *
 * 算法说明：
 * 1. 如果障碍物在环岛内部，距离为0
 * 2. 如果障碍物在环岛前方，距离 = 环岛起始 - 障碍物结束
 * 3. 如果障碍物在环岛后方，距离 = 障碍物起始 - 环岛结束
 */
double DistanceBlockingObstacleToJunction(
    const ReferenceLineInfo& reference_line_info,
    const std::string& blocking_obstacle_id) {
  if (blocking_obstacle_id.empty()) {
    ADEBUG << "There is no blocking obstacle.";
    return true;
  }

  const Obstacle* blocking_obstacle =
      reference_line_info.path_decision().obstacles().Find(
          blocking_obstacle_id);
  if (blocking_obstacle == nullptr) {
    ADEBUG << "Blocking obstacle is no longer there.";
    return true;
  }

  // 获取障碍物的起始和结束s坐标
  double blocking_obstacle_start_s =
      blocking_obstacle->PerceptionSLBoundary().start_s();
  double blocking_obstacle_end_s =
      blocking_obstacle->PerceptionSLBoundary().end_s();

  // 初始化最小距离为double最大值
  double min_distance = std::numeric_limits<double>::max();

  AINFO << "Blocking obstacle start s = " << blocking_obstacle_start_s
        << ", end_s: " << blocking_obstacle_end_s;

  // 获取路径上的环岛重叠区域
  const auto& first_encountered_overlaps =
      reference_line_info.FirstEncounteredOverlaps();

  /**
   * 遍历所有环岛重叠区域
   *
   * junction_overlaps()返回环岛重叠区域列表
   */
  for (const auto& overlap :
       reference_line_info.reference_line().map_path().junction_overlaps()) {
    AINFO << overlap.DebugString();

    double distance = std::numeric_limits<double>::max();

    /**
     * 判断障碍物与环岛的位置关系
     *
     * 三种情况：
     * 1. 障碍物在环岛内部：distance = 0
     * 2. 障碍物在环岛前方：distance = 环岛起始 - 障碍物结束
     * 3. 障碍物在环岛后方：distance = 障碍物起始 - 环岛结束
     */
    if ((blocking_obstacle_start_s >= overlap.start_s &&
         blocking_obstacle_start_s <= overlap.end_s) ||
        (blocking_obstacle_end_s >= overlap.start_s &&
         blocking_obstacle_end_s <= overlap.end_s)) {
      // 障碍物在环岛内部
      distance = 0.0;
    } else if (blocking_obstacle_end_s < overlap.start_s) {
      // 障碍物在环岛前方
      distance = overlap.start_s - blocking_obstacle_end_s;
    } else {
      // 障碍物在环岛后方
      distance = blocking_obstacle_start_s - overlap.end_s;
    }

    // 更新最小距离
    min_distance = std::min(min_distance, distance);
  }

  return min_distance;
}

/**
 * @brief 判断阻塞障碍物是否在目的地范围内
 *
 * @param reference_line_info 参考线信息
 * @param blocking_obstacle_id 阻塞障碍物ID
 * @param threshold 距离阈值
 * @return bool 如果在目的地范围内返回true
 *
 * 功能说明：
 * 判断阻塞障碍物是否在目的地附近
 * 如果障碍物已经非常接近目的地，借道可能不值得
 *
 * 算法流程：
 * 1. 检查障碍物是否在目的地之前足够远的位置
 * 2. 如果障碍物到目的地的距离大于阈值，不值得借道
 */
bool IsBlockingObstacleWithinDestination(
    const ReferenceLineInfo& reference_line_info,
    const std::string& blocking_obstacle_id, const double threshold) {
  if (blocking_obstacle_id.empty()) {
    ADEBUG << "There is no blocking obstacle.";
    return true;
  }

  const Obstacle* blocking_obstacle =
      reference_line_info.path_decision().obstacles().Find(
          blocking_obstacle_id);
  if (blocking_obstacle == nullptr) {
    ADEBUG << "Blocking obstacle is no longer there.";
    return true;
  }

  /**
   * 获取相关s坐标
   *
   * blocking_obstacle_s：障碍物起始s
   * adc_end_s：自车后边缘s
   */
  double blocking_obstacle_s =
      blocking_obstacle->PerceptionSLBoundary().start_s();
  double adc_end_s = reference_line_info.AdcSlBoundary().end_s();

  ADEBUG << "Blocking obstacle is at s = " << blocking_obstacle_s;
  ADEBUG << "ADC is at s = " << adc_end_s;
  ADEBUG << "Destination is at s = "
         << reference_line_info.SDistanceToDestination() + adc_end_s;

  /**
   * 判断条件
   *
   * 如果：(障碍物s - 自车后边缘s + 阈值) > 到目的地的距离
   * 说明障碍物离目的地太近，不值得借道
   */
  if (blocking_obstacle_s - adc_end_s + threshold >
      reference_line_info.SDistanceToDestination()) {
    return false;
  }
  return true;
}

/**
 * @namespace命名空间结束
 *
 * C++语法说明：
 * - }  // namespace planning：
 *   结束planning命名空间
 * - }  // namespace apollo：
 *   结束apollo命名空间
 */
}  // namespace planning
}  // namespace apollo