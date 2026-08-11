/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, messages
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 * @brief 交通信号灯交通规则实现文件
 *
 * 功能说明：
 * 实现了交通信号灯(Traffic Light)交通规则，用于处理交通灯的识别和停车决策
 * 根据交通灯颜色（红、黄、绿）决定车辆是否需要停车
 *
 * 核心概念：
 * - Traffic Light：交通信号灯，用于控制交通流
 * - Signal Color：信号灯颜色（GREEN/RED/YELLOW/BLACK/UNKNOWN）
 * - PathOverlap：路径重叠区，表示交通灯与路径的交叉区域
 * - Stop Deceleration：停车减速度，计算安全停车所需的减速度
 *
 * C++语法说明：
 * - std::vector：动态数组容器
 * - protobuf：Google Protocol Buffers，用于结构化数据序列化
 * - mutable：protobuf消息的可变成员访问器
 **/
#include "modules/planning/traffic_rules/traffic_light/traffic_light.h"

#include <memory>
#include <string>
#include <vector>

#include "modules/common_msgs/planning_msgs/planning_internal.pb.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/common.h"
#include "modules/planning/planning_base/common/util/util.h"

namespace apollo {
/**
 * @brief Apollo项目主命名空间
 *
 * 命名空间说明：
 * apollo是百度自动驾驶项目的顶级命名空间
 * 包含common、hdmap、planning、control等多个子命名空间
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

using apollo::hdmap::PathOverlap;
/**
 * @brief 使用apollo::hdmap::PathOverlap类型
 *
 * PathOverlap结构体说明：
 * 表示地图中路标与路径的交叉区域
 * 主要成员：
 * - object_id：对象ID，标识交通灯
 * - start_s：重叠区域起始点沿路径的s坐标
 * - end_s：重叠区域结束点沿路径的s坐标
 */

/**
 * @brief 初始化交通灯规则
 *
 * @param name 规则名称
 * @param injector 依赖注入器指针
 * @return bool 初始化是否成功
 *
 * 功能说明：
 * 初始化TrafficLight交通规则实例
 * 加载规则配置并准备处理交通灯
 *
 * 算法流程：
 * 1. 调用基类TrafficRule的Init方法进行基础初始化
 * 2. 如果基础初始化失败，返回false
 * 3. 从配置文件加载TrafficLightConfig配置
 * 4. 返回配置加载结果
 */
bool TrafficLight::Init(const std::string& name,
                        const std::shared_ptr<DependencyInjector>& injector) {
  if (!TrafficRule::Init(name, injector)) {
    return false;  /**< 基类初始化失败，返回false */
  }
  return TrafficRule::LoadConfig<TrafficLightConfig>(&config_);
  /**
   * @brief 加载交通灯配置
   *
   * TrafficRule::LoadConfig<T>模板函数：
   * 从protobuf配置文件加载TrafficLightConfig
   * &config_将配置存储到成员变量中
   */
}

/**
 * @brief 应用交通灯规则
 *
 * @param frame 当前规划帧
 * @param reference_line_info 参考线信息
 * @return Status 应用结果状态
 *
 * 功能说明：
 * 交通灯规则的主入口函数
 * 遍历所有交通灯重叠区，为需要停车的交通灯创建停车决策
 *
 * 算法流程：
 * 1. 调用MakeDecisions生成停车决策
 * 2. 返回成功状态
 */
Status TrafficLight::ApplyRule(Frame* const frame,
                               ReferenceLineInfo* const reference_line_info) {
  MakeDecisions(frame, reference_line_info);
  /**
   * @brief 生成交通灯决策
   *
   * 调用MakeDecisions进行实际的决策逻辑处理
   */
  return Status::OK();
  /**
   * @brief 返回成功状态
   */
}

/**
 * @brief 生成交通灯决策
 *
 * @param frame 当前规划帧
 * @param reference_line_info 参考线信息
 *
 * 功能说明：
 * 遍历参考线上的所有交通灯重叠区
 * 根据交通灯颜色和距离决定是否需要停车
 *
 * 算法流程：
 * 1. 检查规则是否启用
 * 2. 获取交通灯状态和自车位置
 * 3. 获取参考线上的所有交通灯重叠区
 * 4. 对每个交通灯：
 *    - 检查是否已经处理过
 *    - 检查S坐标投影是否正确（处理环路线时的偏差）
 *    - 获取交通灯颜色
 *    - 如果是红灯/黄灯/未知，计算停车减速度
 *    - 如果减速度合理，创建停车决策
 *
 * C++语法说明：
 * - CHECK_NOTNULL：Apollo宏，用于空指针检查
 * - injector_：成员变量，通过依赖注入获取
 * - mutable_xxx：protobuf消息的可变成员访问器
 */
void TrafficLight::MakeDecisions(Frame* const frame,
                                 ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  /**
   * @brief 空指针检查
   *
   * CHECK_NOTNULL是Apollo定义的宏：
   * 如果frame为nullptr，程序会终止并输出错误信息
   */
  CHECK_NOTNULL(reference_line_info);
  /**
   * @brief 参考线信息空指针检查
   */

  if (!config_.enabled()) {
    return;  /**< 规则未启用，不进行处理 */
  }

  /**
   * @brief 获取交通灯状态
   *
   * injector_：依赖注入器，提供对各子系统的访问
   * planning_context()：返回PlanningContext智能指针
   * mutable_planning_status()：获取可修改的规划状态
   * traffic_light()：获取traffic_light字段的引用
   */
  const auto& traffic_light_status =
      injector_->planning_context()->planning_status().traffic_light();

  /**
   * @brief 获取自车前边缘和后边缘的s坐标
   *
   * end_s()：前边缘s坐标（车辆最前端）
   * start_s()：后边缘s坐标（车辆最后端）
   *
   * 使用两个边缘坐标用于判断：
   * - 前边缘判断交通灯是否在车辆前方
   * - 后边缘判断是否已通过交通灯
   */
  const double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
  /**< @brief 自车前边缘s坐标 */
  const double adc_back_edge_s = reference_line_info->AdcSlBoundary().start_s();
  /**< @brief 自车后边缘s坐标 */

  /**
   * @brief 设置调试信息
   *
   * planning_internal::SignalLightDebug：
   * 用于调试和可视化的信号灯调试信息
   *
   * mutable_xxx是protobuf消息的成员访问器：
   * - mutable_debug()：获取可修改的debug对象
   * - mutable_planning_data()：获取可修改的规划数据
   * - mutable_signal_light()：获取可修改的信号灯调试信息
   */
  planning_internal::SignalLightDebug* signal_light_debug =
      reference_line_info->mutable_debug()
          ->mutable_planning_data()
          ->mutable_signal_light();

  /**
   * @brief 记录调试数据
   *
   * set_xxx：protobuf消息的设置方法
   * 用于记录自车前边缘s坐标和速度，供后续分析使用
   */
  signal_light_debug->set_adc_front_s(adc_front_edge_s);
  /**< @brief 设置自车前边缘s坐标 */
  signal_light_debug->set_adc_speed(
      injector_->vehicle_state()->linear_velocity());
  /**< @brief 设置自车当前速度 */

  /**
   * @brief 获取参考线上的所有交通灯重叠区
   *
   * reference_line_info->reference_line().map_path()：获取地图路径
   * .signal_overlaps()：获取所有交通灯重叠区
   *
   * 返回类型：std::vector<PathOverlap>
   */
  const std::vector<PathOverlap>& traffic_light_overlaps =
      reference_line_info->reference_line().map_path().signal_overlaps();

  /**
   * @brief 遍历所有交通灯重叠区
   */
  for (const auto& traffic_light_overlap : traffic_light_overlaps) {
    /**
     * @brief 检查交通灯是否在自车后方
     *
     * 如果交通灯结束点在自车后边缘之后，说明已经通过
     */
    if (traffic_light_overlap.end_s <= adc_back_edge_s) {
      continue;  /**< 已通过此交通灯，继续下一个 */
    }

    /**
     * @brief 检查交通灯是否已完成
     *
     * 功能：判断此交通灯是否已被场景/阶段处理过
     * 避免重复处理同一个交通灯
     *
     * done_traffic_light_overlap_id：已完成的交通灯ID列表
     */
    bool traffic_light_done = false;
    for (const auto& done_traffic_light_overlap_id :
         traffic_light_status.done_traffic_light_overlap_id()) {
      if (traffic_light_overlap.object_id == done_traffic_light_overlap_id) {
        traffic_light_done = true;  /**< 标记为已完成 */
        break;  /**< 找到匹配，跳出循环 */
      }
    }
    if (traffic_light_done) {
      continue;  /**< 已完成，继续下一个交通灯 */
    }

    /**
     * @brief 处理环路线S投影偏差问题
     *
     * 问题描述：在环路线等复杂路由中，S坐标投影可能不准确
     * 解决方式：通过比较S距离和实际XY距离来判断
     *
     * kSDiscrepanceTolerance：S坐标差异容差（10米）
     */
    static constexpr double kSDiscrepanceTolerance = 10.0;
    /**< @brief S坐标差异容差 */

    const auto& reference_line = reference_line_info->reference_line();
    /**< @brief 获取参考线 */

    /**
     * @brief 将交通灯位置从SL坐标转换为XY坐标
     *
     * SL坐标：沿路径距离(s)和垂直路径距离(l)
     * XY坐标：绝对地理坐标
     *
     * 步骤：
     * 1. 创建SL点，设置s为交通灯start_s，l=0
     * 2. 调用SLToXY转换为XY坐标
     */
    common::SLPoint traffic_light_sl;
    traffic_light_sl.set_s(traffic_light_overlap.start_s);
    traffic_light_sl.set_l(0);
    common::math::Vec2d traffic_light_point;
    reference_line.SLToXY(traffic_light_sl, &traffic_light_point);
    /**
     * @brief traffic_light_point：交通灯的XY坐标
     */

    /**
     * @brief 获取自车当前位置
     *
     * injector_->vehicle_state()：获取车辆状态
     * x()和y()：自车位置的XY坐标
     */
    common::math::Vec2d adc_position = {injector_->vehicle_state()->x(),
                                        injector_->vehicle_state()->y()};

    /**
     * @brief 计算实际距离和S距离
     *
     * DistanceXY：计算两点间的XY距离
     * s_distance：沿路径的S距离
     */
    const double distance =
        common::util::DistanceXY(traffic_light_point, adc_position);
    /**< @brief 自车到交通灯的实际XY距离 */
    const double s_distance = traffic_light_overlap.start_s - adc_front_edge_s;
    /**< @brief 自车前边缘到交通灯的S距离 */

    ADEBUG << "traffic_light[" << traffic_light_overlap.object_id
           << "] start_s[" << traffic_light_overlap.start_s << "] s_distance["
           << s_distance << "] actual_distance[" << distance << "]";

    /**
     * @brief 检查S距离是否异常
     *
     * 条件：s_distance >= 0（交通灯在前方）
     * 且 fabs(s_distance - distance) > kSDiscrepanceTolerance（S距离与实际距离差异过大）
     *
     * 如果差异过大，说明S投影有问题，跳过此交通灯
     */
    if (s_distance >= 0 &&
        fabs(s_distance - distance) > kSDiscrepanceTolerance) {
      ADEBUG << "SKIP traffic_light[" << traffic_light_overlap.object_id
             << "] close in position, but far away along reference line";
      continue;  /**< S投影异常，跳过此交通灯 */
    }

    /**
     * @brief 获取交通灯颜色
     *
     * frame->GetSignal：获取交通灯信号
     * .color()：获取信号颜色
     *
     * 返回类型：perception::TrafficLight::Color
     * 可能值：GREEN/RED/YELLOW/BLACK/UNKNOWN
     */
    auto signal_color =
        frame->GetSignal(traffic_light_overlap.object_id).color();

    /**
     * @brief 计算停车减速度
     *
     * util::GetADCStopDeceleration：
     * 计算从当前位置停车到目标位置所需的减速度
     *
     * 参数：
     * - injector_->vehicle_state()：车辆状态（包含当前位置和速度）
     * - adc_front_edge_s：自车前边缘s坐标
     * - traffic_light_overlap.start_s：交通灯停止线s坐标
     *
     * 返回值：double，需要的减速度（正值）
     */
    const double stop_deceleration = util::GetADCStopDeceleration(
        injector_->vehicle_state(), adc_front_edge_s,
        traffic_light_overlap.start_s);

    ADEBUG << "traffic_light_id[" << traffic_light_overlap.object_id
           << "] start_s[" << traffic_light_overlap.start_s << "] color["
           << signal_color << "] stop_deceleration[" << stop_deceleration
           << "]";

    /**
     * @brief 记录信号灯调试信息
     *
     * add_signal：向repeated字段添加新元素
     * set_xxx：设置各字段的值
     */
    planning_internal::SignalLightDebug::SignalDebug* signal_debug =
        signal_light_debug->add_signal();
    /**< @brief 添加新信号调试信息 */
    signal_debug->set_adc_stop_deceleration(stop_deceleration);
    /**< @brief 设置停车减速度 */
    signal_debug->set_color(signal_color);
    /**< @brief 设置信号颜色 */
    signal_debug->set_light_id(traffic_light_overlap.object_id);
    /**< @brief 设置交通灯ID */
    signal_debug->set_light_stop_s(traffic_light_overlap.start_s);
    /**< @brief 设置交通灯停止线s坐标 */

    /**
     * @brief mayaochang添加：处理绿灯和黑灯
     *
     * 绿灯或黑灯：可以直接通行，无需停车
     * 注意：BLACK可能是指交通灯不亮的情况
     */
    if (signal_color == perception::TrafficLight::GREEN ||
        signal_color == perception::TrafficLight::BLACK) {
      continue;  /**< 绿灯或黑灯，继续前进 */
    }

    /**
     * @brief 红灯/黄灯/未知：检查减速度是否合理
     *
     * config_.max_stop_deceleration()：最大允许停车减速度
     * 如果所需减速度过大，说明无法安全停车
     * 例如：在高速行驶时距离红灯太近
     */
    if (stop_deceleration > config_.max_stop_deceleration()) {
      AWARN << "stop_deceleration too big to achieve.  SKIP red light";
      continue;  /**< 减速度过大，无法安全停车，跳过 */
    }

    /**
     * @brief 构建停车决策
     *
     * ADEBUG：调试日志宏
     */
    ADEBUG << "BuildStopDecision: traffic_light["
           << traffic_light_overlap.object_id << "] start_s["
           << traffic_light_overlap.start_s << "]"
           << "stop_distance: " << config_.stop_distance();

    /**
     * @brief 创建虚拟障碍物ID
     *
     * TRAFFIC_LIGHT_VO_ID_PREFIX：交通灯虚拟障碍物ID前缀
     */
    std::string virtual_obstacle_id =
        TRAFFIC_LIGHT_VO_ID_PREFIX + traffic_light_overlap.object_id;

    /**
     * @brief 等待让行的障碍物列表
     *
     * 对于交通灯，初始为空列表
     * 后续场景可能会更新此列表，添加需要等待的障碍物
     */
    const std::vector<std::string> wait_for_obstacles;
    /**< @brief 空列表，暂无等待的障碍物 */

    /**
     * @brief 调用BuildStopDecision创建停车决策
     *
     * 参数说明：
     * - virtual_obstacle_id：虚拟障碍物ID
     * - traffic_light_overlap.start_s：停车点s坐标
     * - config_.stop_distance()：停车距离
     * - StopReasonCode::STOP_REASON_SIGNAL：停车原因（信号灯）
     * - wait_for_obstacles：等待让行的障碍物列表
     * - Getname()：获取当前规则名称
     * - frame：当前规划帧
     * - reference_line_info：参考线信息
     */
    util::BuildStopDecision(
        virtual_obstacle_id, traffic_light_overlap.start_s,
        config_.stop_distance(), StopReasonCode::STOP_REASON_SIGNAL,
        wait_for_obstacles, Getname(), frame, reference_line_info);
  }
  /**
   * @brief 遍历结束
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
