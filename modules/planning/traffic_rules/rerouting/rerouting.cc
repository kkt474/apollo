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
 * @brief 重新路由交通规则实现文件
 *
 * 功能说明：
 * 实现了重新路由(Rerouting)交通规则，用于处理车道变换失败后的路由重新规划
 * 当车辆无法完成车道变换且无法沿当前路径继续行驶时，触发重新路由请求
 *
 * 核心概念：
 * - Rerouting：重新路由，当当前路径无法继续时的路由重新规划
 * - Lane Change Fail：车道变换失败，无法完成变道
 * - Routing：路由，由路由模块计算的从起点到终点的路径
 * - RouteEndWaypoint：路由终点路点
 *
 * C++语法说明：
 * - cyber::Clock：Cyber RT的时间服务，用于获取当前时间
 * - std::shared_ptr：智能指针，管理对象生命周期
 * - routing::FORWARD：路由模块定义的前进方向枚举
 **/
#include "modules/planning/traffic_rules/rerouting/rerouting.h"

#include <memory>
#include <string>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "cyber/time/clock.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/planning_base/common/planning_context.h"

namespace apollo {
/**
 * @brief Apollo项目主命名空间
 *
 * 命名空间说明：
 * apollo是百度自动驾驶项目的顶级命名空间
 * 包含common、hdmap、planning、control、cyber等子命名空间
 */
namespace planning {
/**
 * @brief 规划模块的命名空间
 *
 * 包含内容：
 * - 交通规则实现（traffic_rules）
 * - 场景管理（scenarios）
 * - 路径规划任务（tasks）
 * - 规划器实现（planners）
 */

using apollo::common::Status;
/**
 * @brief 使用apollo::common::Status类型
 *
 * 语法说明：
 * using声明将其他命名空间中的类型引入当前作用域
 * Status是Apollo中用于表示操作结果的状态类
 * 包含OK（成功）、ERROR（错误）等状态
 */

using apollo::cyber::Clock;
/**
 * @brief 使用apollo::cyber::Clock类型
 *
 * Clock类说明：
 * Cyber RT的时间服务类
 * 提供获取当前时间的方法
 * - Clock::NowInSeconds()：获取当前时间（秒）
 * - Clock::NowInMs()：获取当前时间（毫秒）
 */

/**
 * @brief 初始化重新路由规则
 *
 * @param name 规则名称
 * @param injector 依赖注入器指针
 * @return bool 初始化是否成功
 *
 * 功能说明：
 * 初始化Rerouting交通规则实例
 * 加载规则配置并准备处理重新路由
 *
 * 算法流程：
 * 1. 调用基类TrafficRule的Init方法进行基础初始化
 * 2. 如果基础初始化失败，返回false
 * 3. 从配置文件加载ReroutingConfig配置
 * 4. 返回配置加载结果
 */
bool Rerouting::Init(const std::string& name,
                     const std::shared_ptr<DependencyInjector>& injector) {
  if (!TrafficRule::Init(name, injector)) {
    return false;  /**< 基类初始化失败，返回false */
  }
  return TrafficRule::LoadConfig<ReroutingConfig>(&config_);
  /**
   * @brief 加载重新路由配置
   *
   * TrafficRule::LoadConfig<T>模板函数：
   * 从protobuf配置文件加载ReroutingConfig
   * &config_将配置存储到成员变量中
   */
}

/**
 * @brief 车道变换失败时触发重新路由
 *
 * @return bool 是否成功执行重新路由
 *
 * 功能说明：
 * 当车道变换失败时，评估是否需要触发重新路由
 * 如果当前路径无法继续行驶，则请求重新路由
 *
 * 算法流程：
 * 1. 检查是否已到达目的地或距离目的地很近（< 20米）
 * 2. 检查当前车道是否为前进方向
 * 3. 检查车辆是否在当前车道上
 * 4. 检查当前车道是否可以驶出
 * 5. 检查路由终点是否存在
 * 6. 检查是否需要重新路由（根据距离和时间）
 * 7. 检查冷却时间（避免频繁重新路由）
 * 8. 发送重新路由请求
 *
 * C++语法说明：
 * - static constexpr：编译时常量
 * - routing::FORWARD：路由模块定义的枚举值
 * - reference_line_info_：成员变量，存储参考线信息
 */
bool Rerouting::ChangeLaneFailRerouting() {
  /**
   * @brief 重新路由到终点的距离阈值
   *
   * 如果距离目的地小于此值，不需要重新路由
   * static constexpr：编译时常量，效率更高
   */
  static constexpr double kRerouteThresholdToEnd = 20.0;
  /**< @brief 20米，距离目的地很近的阈值 */

  /**
   * @brief 检查所有参考线是否到达目的地
   *
   * frame_->reference_line_info()：获取所有参考线信息
   * 遍历每条参考线检查是否已到达目的地
   */
  for (const auto& ref_line_info : frame_->reference_line_info()) {
    /**
     * @brief 检查是否到达目的地
     *
     * ReachedDestination()：是否已到达目的地
     * SDistanceToDestination()：到目的地的距离
     */
    if (ref_line_info.ReachedDestination() ||
        ref_line_info.SDistanceToDestination() < kRerouteThresholdToEnd) {
      return true;  /**< 已到达或接近目的地，不需要重新路由 */
    }
  }

  /**
   * @brief 获取当前车道段信息
   *
   * reference_line_info_->Lanes()：获取关联的车道段信息
   * 用于判断车辆的路由前进方向和位置
   */
  const auto& segments = reference_line_info_->Lanes();

  /**
   * @brief 条件1：如果当前参考线是前进方向，不需要重新路由
   *
   * routing::FORWARD：路由模块定义的前进方向枚举
   * NextAction()：获取下一个动作（FORWARD/LEFT/RIGHT）
   *
   * 如果不在当前车道上（IsOnSegment()返回false），也不检查重新路由
   */
  if (segments.NextAction() == routing::FORWARD) {
    return true;  /**< 当前是前进方向，不需要重新路由 */
  }

  /**
   * @brief 条件2：如果车辆还不在当前车道上，不需要重新路由
   *
   * IsOnSegment()：判断车辆是否在当前车道段上
   */
  if (!segments.IsOnSegment()) {
    return true;  /**< 车辆不在当前车道上，不需要重新路由 */
  }

  /**
   * @brief 条件3：如果当前车道可以驶出（连接到下一车道），不需要重新路由
   *
   * CanExit()：判断当前车道是否可以正常驶出
   */
  if (segments.CanExit()) {
    return true;  /**< 可以驶出当前车道，不需要重新路由 */
  }

  /**
   * @brief 条件4：检查路由终点是否存在
   *
   * RouteEndWaypoint()：获取路由终点路点
   * 如果没有有效的路由终点，不进行重新路由
   */
  const auto& route_end_waypoint = segments.RouteEndWaypoint();
  if (!route_end_waypoint.lane) {
    return true;  /**< 没有有效的路由终点，不重新路由 */
  }

  /**
   * @brief 获取路由终点坐标并转换为SL坐标
   *
   * GetSmoothPoint：获取车道上指定s坐标的平滑点
   * XYToSL：将XY坐标转换为SL坐标
   */
  auto point = route_end_waypoint.lane->GetSmoothPoint(route_end_waypoint.s);
  /**< @brief 路由终点的XY坐标 */
  const auto& reference_line = reference_line_info_->reference_line();
  /**< @brief 获取参考线 */
  common::SLPoint sl_point;
  /**< @brief SL坐标点 */
  if (!reference_line.XYToSL(point, &sl_point)) {
    /**
     * @brief 坐标转换失败
     *
     * AERROR：错误日志宏
     * ShortDebugString()：protobuf消息的简短调试字符串
     */
    AERROR << "Failed to project point: " << point.ShortDebugString();
    return false;  /**< 坐标转换失败 */
  }

  /**
   * @brief 检查路由终点是否在参考线上
   *
   * IsOnLane：判断点是否在车道上
   */
  if (!reference_line.IsOnLane(sl_point)) {
    return true;  /**< 路由终点不在参考线上，不重新路由 */
  }

  /**
   * @brief 条件5：检查当前是否需要重新路由
   *
   * 根据当前速度和准备时间计算继续行驶的距离
   * 如果路由终点距离足够远，不需要立即重新路由
   *
   * sl_point.s()：路由终点的s坐标
   * adc_s：自车前边缘s坐标
   */
  double adc_s = reference_line_info_->AdcSlBoundary().end_s();
  /**< @brief 自车前边缘s坐标 */
  const auto vehicle_state = injector_->vehicle_state();
  /**< @brief 获取车辆状态 */
  double speed = vehicle_state->linear_velocity();
  /**< @brief 当前速度（m/s） */
  const double prepare_rerouting_time = config_.prepare_rerouting_time();
  /**< @brief 准备重新路由时间（秒） */
  const double prepare_distance = speed * prepare_rerouting_time;
  /**< @brief 在准备时间内可以行驶的距离 */

  /**
   * @brief 判断是否需要重新路由
   *
   * 条件：sl_point.s() > adc_s + prepare_distance
   * 含义：路由终点距离 > 自车位置 + 准备距离
   * 如果终点足够远，可以继续行驶，不立即重新路由
   */
  if (sl_point.s() > adc_s + prepare_distance) {
    ADEBUG << "No need rerouting now because still can drive for time: "
           << prepare_rerouting_time << " seconds";
    return true;  /**< 距离充足，不需要立即重新路由 */
  }

  /**
   * @brief 条件6：检查是否在冷却时间内
   *
   * 功能：避免频繁发送重新路由请求
   * 冷却时间内只能发送一次重新路由请求
   *
   * mutable_rerouting()：获取可修改的rerouting状态
   */
  auto* rerouting = injector_->planning_context()
                        ->mutable_planning_status()
                        ->mutable_rerouting();
  /**< @brief 可修改的重新路由状态 */

  if (rerouting == nullptr) {
    /**
     * @brief 重新路由状态指针为空
     */
    AERROR << "rerouting is nullptr.";
    return false;  /**< 状态指针无效 */
  }

  /**
   * @brief 检查冷却时间
   *
   * Clock::NowInSeconds()：获取当前时间（秒）
   * has_last_rerouting_time()：是否已设置上次重新路由时间
   * last_rerouting_time()：上次重新路由时间
   * cooldown_time()：冷却时间配置
   */
  double current_time = Clock::NowInSeconds();
  if (rerouting->has_last_rerouting_time() &&
      (current_time - rerouting->last_rerouting_time() <
       config_.cooldown_time())) {
    ADEBUG << "Skip rerouting and wait for previous rerouting result";
    return true;  /**< 在冷却时间内，跳过本次重新路由 */
  }

  /**
   * @brief 发送重新路由请求
   *
   * frame_->Rerouting：调用frame的重新路由方法
   * 内部会向路由模块发送重新路由请求
   */
  if (!frame_->Rerouting(injector_->planning_context())) {
    AERROR << "Failed to send rerouting request";
    return false;  /**< 发送重新路由请求失败 */
  }

  /**
   * @brief 记录本次重新路由时间
   *
   * set_last_rerouting_time：设置上次重新路由时间
   * 用于下次判断冷却时间
   */
  rerouting->set_last_rerouting_time(current_time);
  return true;  /**< 重新路由请求发送成功 */
}

/**
 * @brief 应用重新路由规则
 *
 * @param frame 当前规划帧
 * @param reference_line_info 参考线信息
 * @return Status 应用结果状态
 *
 * 功能说明：
 * 重新路由规则的主入口函数
 * 评估是否需要触发重新路由
 *
 * 算法流程：
 * 1. 保存frame和reference_line_info到成员变量
 * 2. 调用ChangeLaneFailRerouting评估是否需要重新路由
 * 3. 返回成功或错误状态
 */
Status Rerouting::ApplyRule(Frame* const frame,
                            ReferenceLineInfo* const reference_line_info) {
  /**
   * @brief 保存到成员变量
   *
   * 注意：成员变量frame_和reference_line_info_在ChangeLaneFailRerouting中使用
   * 因此需要先保存
   */
  frame_ = frame;
  /**< @brief 保存当前规划帧 */
  reference_line_info_ = reference_line_info;
  /**< @brief 保存当前参考线信息 */

  /**
   * @brief 评估并执行重新路由
   */
  if (!ChangeLaneFailRerouting()) {
    /**
     * @brief 重新路由失败
     *
     * Status(common::PLANNING_ERROR, ...)：创建错误状态
     * common::PLANNING_ERROR：规划模块的错误码
     */
    return Status(common::PLANNING_ERROR,
                  "In un-successful lane change case, rerouting failed");
    /**
     * @brief 返回错误状态，消息描述失败原因
     */
  }
  return Status::OK();
  /**
   * @brief 返回成功状态
   */
}

}  // namespace planning
/**
 * @brief 命名空间结束标记
 */
}  // namespace apollo
/**
 * @brief Apollo命名空间结束标记
 */
