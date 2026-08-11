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
 * @file destination.cc
 * @brief 目的地停车交通规则实现文件
 *
 * 本文件实现了目的地停车交通规则(Destination)
 * 用于在车辆接近目的地时生成停车决策
 *
 * 功能说明：
 * 1. 检查是否接近目的地
 * 2. 计算目的地在参考线上的S坐标
 * 3. 判断是否已经通过目的地
 * 4. 处理Pull Over（靠边停车）场景
 * 5. 构建停车决策
 *
 * 场景说明：
 * 当车辆接近路由目的地时：
 * - 如果在目的地前方，创建停车墙
 * - 车辆在停车墙处完全停止
 * - 支持Pull Over靠边停车场景
 *
 * 停车决策流程：
 * 1. 检查是否接近目的地 (is_near_destination)
 * 2. 获取目的地的SL坐标
 * 3. 如果目的地在自车后方但未到达，返回0
 * 4. 计算停车线位置
 * 5. 调用BuildStopDecision创建停车墙
 *
 * 相关C++语法说明：
 * - std::shared_ptr<T>: 智能指针，共享所有权
 * - std::vector<std::string>: 字符串向量
 * - FLAGS_xxx: GFlags配置参数
 * - protobuf消息类型: 路由、地图、规划等消息
 **/

/**
 * @brief 本类的头文件
 *
 * 包含Destination类的完整定义
 */
#include "modules/planning/traffic_rules/destination/destination.h"

/**
 * @brief 标准库头文件
 *
 * <memory>: 提供std::shared_ptr智能指针
 * <vector>: 提供std::vector动态数组
 */
#include <memory>
#include <vector>

/**
 * @brief 地图消息protobuf头文件
 *
 * map_lane.pb.h:
 *   包含地图车道相关信息
 *   如LaneWaypoint、Pose等
 */
#include "modules/common_msgs/map_msgs/map_lane.pb.h"

/**
 * @brief 规划上下文头文件
 *
 * planning_context.h:
 *   规划上下文，存储跨帧状态
 *   如目的地状态、Pull Over状态等
 */
#include "modules/planning/planning_base/common/planning_context.h"

/**
 * @brief 规划模块GFlags头文件
 *
 * FLAGS_destination_obstacle_id:
 *   目的地障碍物ID（如"__DESTINATION_WALL__"）
 * FLAGS_virtual_stop_wall_length:
 *   虚拟停车墙长度
 */
#include "modules/planning/planning_base/gflags/planning_gflags.h"

/**
 * @brief 通用工具头文件
 *
 * common.h:
 *   包含BuildStopDecision等工具函数
 *   用于构建停车决策
 */
#include "modules/planning/planning_base/common/util/common.h"

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
 */
using apollo::common::Status;               /**< Apollo通用状态类型 */
using apollo::common::VehicleConfigHelper;   /**< 车辆配置助手 */

/**
 * @brief 目的地规则初始化函数
 *
 * @param name 规则名称
 * @param injector 依赖注入器智能指针
 * @return bool 初始化是否成功
 *
 * 功能说明：
 * 1. 调用父类TrafficRule的Init进行基础初始化
 * 2. 加载本规则特有的配置DestinationConfig
 *
 * 算法流程：
 * 1. 调用TrafficRule::Init进行基础初始化
 * 2. 如果基础初始化失败，返回false
 * 3. 调用TrafficRule::LoadConfig加载本规则配置
 * 4. 返回加载结果
 *
 * C++语法说明：
 * - const std::string& name:
 *   常量引用，规则名称用于标识和日志
 * - const std::shared_ptr<DependencyInjector>& injector:
 *   共享所有权的智能指针，依赖注入器
 */
bool Destination::Init(const std::string& name,
                      const std::shared_ptr<DependencyInjector>& injector) {
  /**
   * @brief 调用父类TrafficRule的Init进行基础初始化
   *
   * TrafficRule::Init():
   *   执行基础初始化，如保存名称和依赖注入器
   */
  if (!TrafficRule::Init(name, injector)) {
    return false;  /**< 父类初始化失败，返回false */
  }

  /**
   * @brief 加载本规则的配置
   *
   * TrafficRule::LoadConfig<T>():
   *   模板方法，从配置文件加载DestinationConfig类型配置
   *   配置内容可能包括：停车距离等参数
   */
  return TrafficRule::LoadConfig<DestinationConfig>(&config_);
}

/**
 * @brief 应用交通规则
 *
 * @param frame 当前规划帧
 * @param reference_line_info 参考线信息
 * @return Status 应用状态
 *
 * 功能说明：
 * 交通规则的入口函数
 * 由TrafficDecider::Execute调用
 *
 * 执行流程：
 * 1. 检查输入参数有效性
 * 2. 调用MakeDecisions生成停车决策
 * 3. 返回执行状态
 *
 * C++语法说明：
 * - Frame* frame:
 *   原始指针，表示当前规划帧
 * - ReferenceLineInfo* const reference_line_info:
 *   指向常量的指针，指针本身是常量
 * - CHECK_NOTNULL:
 *   Apollo断言宏，检查指针是否为空
 */
Status Destination::ApplyRule(Frame* frame,
                              ReferenceLineInfo* const reference_line_info) {
  /**
   * @brief 检查输入参数有效性
   *
   * CHECK_NOTNULL:
   *   Apollo断言宏
   *   如果指针为nullptr，程序终止
   *   仅在调试模式下生效
   */
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  /**
   * @brief 生成目的地停车决策
   *
   * MakeDecisions():
   *   检查是否接近目的地
   *   计算停车位置
   *   创建停车墙
   */
  MakeDecisions(frame, reference_line_info);

  return Status::OK();  /**< 执行成功 */
}

/**
 * @brief 生成目的地决策
 *
 * @param frame 当前规划帧
 * @param reference_line_info 参考线信息
 * @return int 0表示成功，-1表示失败
 *
 * 功能说明：
 * 生成目的地停车决策的核心函数
 *
 * 算法流程：
 * 1. 检查是否接近目的地
 * 2. 获取目的地的路由终点信息
 * 3. 将目的地XY坐标转换为SL坐标
 * 4. 检查是否已经通过目的地
 * 5. 如果是Pull Over场景，在Pull Over位置停车
 * 6. 否则在目的地位置停车
 *
 * C++语法说明：
 * - Frame* frame:
 *   原始指针，可修改
 * - ReferenceLineInfo* const reference_line_info:
 *   指向常量的指针
 */
int Destination::MakeDecisions(Frame* frame,
                              ReferenceLineInfo* const reference_line_info) {
  /**
   * @brief 检查输入参数有效性
   */
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  /**
   * @brief 检查是否接近目的地
   *
   * frame->is_near_destination():
   *   检查当前帧是否接近目的地
   *   这是规划模块根据距离阈值判断的
   * 如果不接近，直接返回0（不需要处理）
   */
  if (!frame->is_near_destination()) {
    return 0;  /**< 不接近目的地，无需处理 */
  }

  /**
   * @brief 获取路由终点
   *
   * frame->local_view().end_lane_way_point:
   *   获取本地视图中的终点车道waypoint
   *   存储了目的地的位置信息
   *
   * C++语法说明：
   * - frame->local_view():
   *   ->调用成员函数获取LocalView
   *   LocalView包含各种输入数据
   */
  const auto routing_end = frame->local_view().end_lane_way_point;

  /**
   * @brief 检查路由终点是否有效
   *
   * nullptr == routing_end:
   *   检查指针是否为空
   *   如果为空，说明没有有效的路由终点
   */
  if (nullptr == routing_end) {
    AERROR << "routing_request has no end";  /**< 记录错误日志 */
    return -1;  /**< 返回失败 */
  }

  /**
   * @brief 创建目的地的SL坐标变量
   *
   * common::SLPoint:
   *   SL坐标系下的点
   *   包含s（沿参考线距离）和l（横向偏移）
   */
  common::SLPoint dest_sl;

  /**
   * @brief 获取参考线引用
   *
   * reference_line_info->reference_line():
   *   获取参考线引用
   *   用于XY到SL坐标转换
   */
  const auto& reference_line = reference_line_info->reference_line();

  /**
   * @brief 将目的地XY坐标转换为SL坐标
   *
   * reference_line.XYToSL(routing_end->pose(), &dest_sl):
   *   将世界坐标系下的点转换为SL坐标系
   *   routing_end->pose(): 获取终点的位姿（位置和朝向）
   *   &dest_sl: 输出参数，转换后的SL坐标
   *
   * XYToSL原理：
   * - 计算点在参考线上的投影
   * - s = 沿参考线的累积距离
   * - l = 到参考线的垂直距离
   */
  reference_line.XYToSL(routing_end->pose(), &dest_sl);

  /**
   * @brief 获取自车SL边界
   *
   * reference_line_info->AdcSlBoundary():
   *   获取自车在SL坐标系下的边界框
   *   包含start_s、end_s、start_l、end_l
   */
  const auto& adc_sl = reference_line_info->AdcSlBoundary();

  /**
   * @brief 获取车辆配置
   *
   * VehicleConfigHelper::Instance()->GetConfig():
   *   单例模式获取车辆配置
   * vehicle_param():
   *   获取车辆参数子结构
   */
  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();

  /**
   * @brief 获取前边缘到几何中心的距离
   *
   * front_edge_to_center:
   *   车辆前边缘到几何中心的距离
   *   用于计算停车位置
   */
  const double ego_front_to_center =
      vehicle_config.vehicle_param().front_edge_to_center();

  /**
   * @brief 检查目的地是否超出参考线
   *
   * dest_sl.s() + ego_front_to_center > reference_line.Length():
   *   目的地S坐标 + 前边缘到中心距离 > 参考线总长度
   *   说明目的地在参考线范围之外
   *
   * 这可能是因为：
   * 1. 路由终点在下一条参考线上
   * 2. 地图数据不准确
   */
  if (dest_sl.s() + ego_front_to_center > reference_line.Length()) {
    AWARN << "dest_sl.s() + ego_front_to_center > reference_line->length()"
            <<"may cause ego is stoped by PATH_END fence not by destination";
    /**
     * @brief 打印警告：
     *   车辆可能停在PATH_END fence而不是目的地
     *   这是边界情况，不影响继续处理
     */
  }

  /**
   * @brief 获取目的地状态
   *
   * injector_->planning_context():
   *   获取规划上下文
   * mutable_planning_status():
   *   获取可修改的规划状态
   * destination():
   *   获取目的地状态
   */
  const auto& dest =
      injector_->planning_context()->mutable_planning_status()->destination();

  /**
   * @brief 检查是否已经通过目的地
   *
   * adc_sl.start_s() > dest_sl.s():
   *   自车当前位置 > 目的地位置
   *   说明自车已经驶过目的地
   *
   * && !dest.has_passed_destination():
   *   但状态标志显示还没有通过目的地
   *   这是自相矛盾的情况
   *
   * 如果目的地在自车后方但标记未到达，返回0
   */
  if (adc_sl.start_s() > dest_sl.s() && !dest.has_passed_destination()) {
    ADEBUG << "Destination at back, but we have not reached destination yet";
    return 0;  /**< 目的地在后方，无需处理 */
  }

  /**
   * @brief 定义停车墙ID
   *
   * FLAGS_destination_obstacle_id:
   *   GFlags配置的目的地障碍物ID
   *   通常是"__DESTINATION_WALL__"
   */
  const std::string stop_wall_id = FLAGS_destination_obstacle_id;

  /**
   * @brief 定义等待避让的障碍物列表
   *
   * std::vector<std::string>:
   *   字符串向量，存储障碍物ID列表
   *   用于停车时等待行人等动态障碍物通过
   *   目的地场景通常为空
   */
  const std::vector<std::string> wait_for_obstacle_ids;

  /**
   * @brief 检查是否有Pull Over状态
   *
   * pull_over_status.has_position():
   *   检查是否有Pull Over位置信息
   * pull_over_status.position().has_x() && .has_y():
   *   检查位置坐标是否有效
   *
   * Pull Over场景：
   *   靠边停车，如公交车进站、载人上下车等
   */
  const auto& pull_over_status =
      injector_->planning_context()->planning_status().pull_over();
  if (pull_over_status.has_position() && pull_over_status.position().has_x() &&
      pull_over_status.position().has_y()) {
    /**
     * @brief 打印调试信息
     */
    ADEBUG << "BuildStopDecision: pull-over position";

    /**
     * @brief 创建Pull Over位置的SL坐标
     */
    common::SLPoint pull_over_sl;
    reference_line.XYToSL(pull_over_status.position(), &pull_over_sl);

    /**
     * @brief 计算停车线S坐标
     *
     * 公式：stop_line_s = pull_over_sl.s() + front_edge_to_center + stop_distance
     *
     * 解释：
     * - pull_over_sl.s(): Pull Over位置的S坐标
     * - front_edge_to_center: 前边缘到中心的距离
     *   加上这个值，得到前边缘应该到达的位置
     * - config_.stop_distance(): 额外停车距离
     *   在Pull Over位置前方额外预留一段距离
     */
    const double stop_line_s = pull_over_sl.s() +
                               VehicleConfigHelper::GetConfig()
                                   .vehicle_param()
                                   .front_edge_to_center() +
                               config_.stop_distance();

    /**
     * @brief 构建Pull Over停车决策
     *
     * util::BuildStopDecision():
     *   工具函数，用于构建停车墙和停车决策
     * 参数：
     * - stop_wall_id: 停车墙ID
     * - stop_line_s: 停车线S坐标
     * - config_.stop_distance(): 停车距离
     * - StopReasonCode::STOP_REASON_PULL_OVER: 停车原因（靠边停车）
     * - wait_for_obstacle_ids: 等待避让的障碍物列表
     * - Getname(): 规则名称
     * - frame: 当前规划帧
     * - reference_line_info: 参考线信息
     */
    util::BuildStopDecision(stop_wall_id, stop_line_s, config_.stop_distance(),
                           StopReasonCode::STOP_REASON_PULL_OVER,
                           wait_for_obstacle_ids, Getname(), frame,
                           reference_line_info);
    return 0;  /**< Pull Over停车决策创建成功 */
  }

  /**
   * @brief 构建普通目的地停车决策
   *
   * ADEBUG:
   *   调试级别日志
   */
  ADEBUG << "BuildStopDecision: destination";

  /**
   * @brief 计算目的地车道的S坐标
   *
   * 公式：dest_lane_s = max(0, routing_end->s() - virtual_stop_wall_length/2 - stop_distance)
   *
   * 解释：
   * - routing_end->s(): 路由终点的S坐标
   * - FLAGS_virtual_stop_wall_length: 虚拟停车墙长度
   *   -FLAGS_virtual_stop_wall_length/2: 减去一半长度，使停车墙中心对准目的地
   * - config_.stop_distance(): 额外停车距离
   * - std::fmax(0.0, ...): 确保不为负
   *
   * C++语法说明：
   * std::fmax:
   *   取两个浮点数的较大值
   *   这里确保计算结果至少为0
   */
  const double dest_lane_s =
      std::fmax(0.0, routing_end->s() - FLAGS_virtual_stop_wall_length -
                         config_.stop_distance());

  /**
   * @brief 构建目的地停车决策
   *
   * util::BuildStopDecision():
   *   创建停车墙和停车决策
   * 参数：
   * - stop_wall_id: 停车墙ID
   * - routing_end->id(): 路由终点ID
   * - dest_lane_s: 目的地S坐标
   * - config_.stop_distance(): 停车距离
   * - StopReasonCode::STOP_REASON_DESTINATION: 停车原因（目的地）
   * - wait_for_obstacle_ids: 等待避让的障碍物列表
   * - Getname(): 规则名称
   * - frame: 当前规划帧
   * - reference_line_info: 参考线信息
   */
  util::BuildStopDecision(
      stop_wall_id, routing_end->id(), dest_lane_s, config_.stop_distance(),
      StopReasonCode::STOP_REASON_DESTINATION, wait_for_obstacle_ids, Getname(),
      frame, reference_line_info);

  return 0;  /**< 目的地停车决策创建成功 */
}

}  // namespace planning
}  // namespace apollo
