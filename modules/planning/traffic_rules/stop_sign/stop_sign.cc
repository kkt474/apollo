/*****************************************************************************
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
 * @brief 停车让行交通规则实现文件（停车标志）
 *
 * 功能说明：
 * 实现了停车标志(Stop Sign)交通规则，用于处理停车路口的停车决策
 * 与让行标志不同，停车标志要求车辆必须完全停止
 *
 * 与YieldSign的区别：
 * - Stop Sign：停车标志，必须完全停车后等待
 * - Yield Sign：让行标志，减速观察后通过
 * - Stop Sign的停车强制性强于Yield Sign
 *
 * 核心概念：
 * - Stop Sign：停车标志，八边形红色标志，要求车辆完全停止
 * - PathOverlap：路径重叠区，表示地图上路标与路径的交叉区域
 * - Virtual Obstacle：虚拟障碍物，用于生成停车决策
 *
 * C++语法说明：
 * - apollo命名空间：百度Apollo自动驾驶项目的命名空间
 * - planning模块：规划模块，负责轨迹规划和决策
 * - std::shared_ptr：智能指针，管理对象生命周期
 **/
#include "modules/planning/traffic_rules/stop_sign/stop_sign.h"

#include <memory>

#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/common.h"

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
 * 表示地图中路标（如停车标志、让行标志）与路径的交叉区域
 * 主要成员：
 * - object_id：对象ID，标识停车标志
 * - start_s：重叠区域起始点沿路径的s坐标
 * - end_s：重叠区域结束点沿路径的s坐标
 */

/**
 * @brief 初始化停车标志规则
 *
 * @param name 规则名称
 * @param injector 依赖注入器指针
 * @return bool 初始化是否成功
 *
 * 功能说明：
 * 初始化StopSign交通规则实例
 * 加载规则配置并准备处理停车标志
 *
 * 算法流程：
 * 1. 调用基类TrafficRule的Init方法进行基础初始化
 * 2. 如果基础初始化失败，返回false
 * 3. 从配置文件加载StopSignConfig配置
 * 4. 返回配置加载结果
 *
 * C++语法说明：
 * - const std::string& name：常量引用参数，避免拷贝
 * - std::shared_ptr<DependencyInjector>：智能指针，管理依赖注入器生命周期
 * - std::shared_ptr<T>：C++11智能指针，引用计数，线程安全
 * - &config_：成员变量地址，用于存储加载的配置
 * - TrafficRule::LoadConfig<T>：模板函数，泛型编程
 */
bool StopSign::Init(const std::string& name,
                    const std::shared_ptr<DependencyInjector>& injector) {
  if (!TrafficRule::Init(name, injector)) {
    /**
     * @brief 检查基类初始化结果
     *
     * 逻辑：先调用基类的Init方法，如果返回false则初始化失败
     * 语法：!是逻辑非运算符
     */
    return false;  /**< 基类初始化失败，返回false */
  }
  /**
   * @brief 加载停车标志配置
   *
   * TrafficRule::LoadConfig<T>模板函数说明：
   * - 从配置文件读取对应类型的配置
   * - T必须是protobuf消息类型（如StopSignConfig）
   * - &config_将配置存储到成员变量中
   */
  return TrafficRule::LoadConfig<StopSignConfig>(&config_);
}

/**
 * @brief 应用停车标志规则
 *
 * @param frame 当前规划帧
 * @param reference_line_info 参考线信息
 * @return Status 应用结果状态
 *
 * 功能说明：
 * 停车标志规则的主入口函数
 * 遍历所有停车标志重叠区，为未完成的停车标志创建停车决策
 *
 * 算法流程：
 * 1. 调用MakeDecisions生成停车决策
 * 2. 返回成功状态
 *
 * C++语法说明：
 * - Frame* const frame：指向常量的指针（指针本身不可变，但指向的内容可变）
 * - ReferenceLineInfo* const reference_line_info：指向常量的指针
 * - const修饰符的位置规则：const靠近类型表示指针指向常量数据
 */
Status StopSign::ApplyRule(Frame* const frame,
                           ReferenceLineInfo* const reference_line_info) {
  MakeDecisions(frame, reference_line_info);
  /**
   * @brief 生成停车决策
   *
   * 调用MakeDecisions进行实际的决策逻辑处理
   */
  return Status::OK();
  /**
   * @brief 返回成功状态
   *
   * Status::OK()是Status类的静态方法，返回成功状态对象
   */
}

/**
 * @brief 生成停车决策
 *
 * @param frame 当前规划帧
 * @param reference_line_info 参考线信息
 *
 * 功能说明：
 * 遍历参考线上的所有停车标志重叠区
 * 为尚未完成的停车标志创建停车决策
 *
 * 算法流程：
 * 1. 检查规则是否启用
 * 2. 获取当前已完成的停车标志ID
 * 3. 获取参考线上的所有停车标志重叠区
 * 4. 对每个停车标志：
 *    - 检查是否已经处理过
 *    - 如果未处理，创建虚拟停车障碍物
 *    - 调用BuildStopDecision生成停车决策
 *
 * 与YieldSign的区别：
 * - StopSign使用adc_back_edge_s（后边缘）判断位置
 * - YieldSign使用adc_front_edge_s（前边缘）判断位置
 * - 这是因为停车标志需要更早开始减速
 *
 * C++语法说明：
 * - CHECK_NOTNULL：Apollo宏，用于空指针检查
 * - injector_：成员变量，通过依赖注入获取
 * - planning_context()：获取规划上下文，存储规划过程中的状态
 */
void StopSign::MakeDecisions(Frame* const frame,
                             ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  /**
   * @brief 空指针检查
   *
   * CHECK_NOTNULL是Apollo定义的宏：
   * - 如果frame为nullptr，程序会终止并输出错误信息
   * - 用于预防编程错误，确保指针有效
   */
  CHECK_NOTNULL(reference_line_info);
  /**
   * @brief 参考线信息空指针检查
   */

  if (!config_.enabled()) {
    /**
     * @brief 检查规则是否启用
     *
     * config_是成员变量，存储StopSignConfig配置
     * enabled()返回bool，表示规则是否启用
     * !是逻辑非，如果规则未启用则直接返回
     */
    return;  /**< 规则未启用，不进行处理 */
  }

  /**
   * @brief 获取停车标志状态
   *
   * injector_：依赖注入器，提供对各子系统的访问
   * planning_context()：返回PlanningContext智能指针
   * mutable_planning_status()：获取可修改的规划状态
   * stop_sign()：获取stop_sign字段的引用
   *
   * 链式调用语法：
   * injector_->planning_context()->mutable_planning_status()->stop_sign()
   */
  const auto& stop_sign_status =
      injector_->planning_context()->planning_status().stop_sign();

  /**
   * @brief 获取自车后边缘的s坐标
   *
   * 与YieldSign的区别：这里使用start_s（后边缘）而非end_s（前边缘）
   * 原因：停车标志必须完全停止，需要更早开始减速
   * 使用后边缘可以留出更多减速距离
   *
   * reference_line_info->AdcSlBoundary()：获取自车的SL边界
   * .start_s()：获取沿路径方向的后边缘s坐标
   * adc_back_edge_s：自车后边缘s坐标
   *
   * ADC = Autonomous Driving Computer（自动驾驶计算机）
   */
  const double adc_back_edge_s = reference_line_info->AdcSlBoundary().start_s();

  /**
   * @brief 获取参考线上的所有停车标志重叠区
   *
   * reference_line_info->reference_line()：获取参考线对象
   * .map_path()：获取地图路径（MapPath）
   * .stop_sign_overlaps()：获取所有停车标志重叠区
   *
   * 返回类型：std::vector<PathOverlap>
   * 每个PathOverlap包含停车标志的位置信息（start_s, end_s）
   */
  const std::vector<PathOverlap>& stop_sign_overlaps =
      reference_line_info->reference_line().map_path().stop_sign_overlaps();

  /**
   * @brief 遍历所有停车标志重叠区
   *
   * 范围for循环（C++11特性）：
   * - const auto&：常量引用，避免拷贝
   * - stop_sign_overlap：当前遍历的停车标志
   */
  for (const auto& stop_sign_overlap : stop_sign_overlaps) {
    /**
     * @brief 检查停车标志是否在自车后方
     *
     * 与YieldSign的区别：这里判断end_s <= adc_back_edge_s
     * 使用后边缘判断，确保车辆完全通过标志后再标记为已完成
     *
     * stop_sign_overlap.end_s：停车标志重叠区结束点s坐标
     * adc_back_edge_s：自车后边缘s坐标
     * 如果停车标志结束点在自车后方，说明已经通过，跳过
     */
    if (stop_sign_overlap.end_s <= adc_back_edge_s) {
      continue;  /**< 已通过此停车标志，继续下一个 */
    }

    /**
     * @brief 检查停车标志是否已完成
     *
     * 与YieldSign的区别：这里是直接比较单个ID而非遍历列表
     * stop_sign_status.done_stop_sign_overlap_id()返回单个已完成的ID
     *
     * 功能：判断此停车标志是否已被处理过
     * 场景/阶段会设置已完成标志，避免重复处理
     */
    if (stop_sign_overlap.object_id ==
        stop_sign_status.done_stop_sign_overlap_id()) {
      /**
       * @brief 如果已处理过，跳过此停车标志
       */
      continue;  /**< 已完成，继续下一个停车标志 */
    }

    /**
     * @brief 构建停车决策
     *
     * ADEBUG：调试日志宏，仅在调试模式输出
     * <<：流输出运算符，用于拼接日志字符串
     */
    ADEBUG << "BuildStopDecision: stop_sign[" << stop_sign_overlap.object_id
           << "] start_s[" << stop_sign_overlap.start_s << "]";

    /**
     * @brief 创建虚拟障碍物ID
     *
     * STOP_SIGN_VO_ID_PREFIX：停车标志虚拟障碍物ID前缀
     * 格式：STOP_SIGN_VO_ID_PREFIX + 停车标志object_id
     * VO = Virtual Obstacle（虚拟障碍物）
     */
    const std::string virtual_obstacle_id =
        STOP_SIGN_VO_ID_PREFIX + stop_sign_overlap.object_id;

    /**
     * @brief 获取等待的障碍物ID列表
     *
     * stop_sign_status.wait_for_obstacle_id()：
     * 返回需要等待让行的障碍物ID列表
     * std::vector<std::string>：字符串向量，存储障碍物ID
     *
     * 使用迭代器构造向量：
     * - begin()：返回容器起始迭代器
     * - end()：返回容器结束迭代器
     */
    const std::vector<std::string> wait_for_obstacle_ids(
        stop_sign_status.wait_for_obstacle_id().begin(),
        stop_sign_status.wait_for_obstacle_id().end());

    /**
     * @brief 调用BuildStopDecision创建停车决策
     *
     * 参数说明：
     * - virtual_obstacle_id：虚拟障碍物ID
     * - stop_sign_overlap.start_s：停车点s坐标（停车标志起始位置）
     * - config_.stop_distance()：停车距离（距障碍物的距离）
     * - StopReasonCode::STOP_REASON_STOP_SIGN：停车原因（停车标志）
     * - wait_for_obstacle_ids：等待让行的障碍物列表
     * - Getname()：获取当前规则名称
     * - frame：当前规划帧
     * - reference_line_info：参考线信息
     *
     * util::BuildStopDecision：
     * 是规划模块提供的工具函数，用于创建标准化的停车决策
     */
    util::BuildStopDecision(
        virtual_obstacle_id, stop_sign_overlap.start_s, config_.stop_distance(),
        StopReasonCode::STOP_REASON_STOP_SIGN, wait_for_obstacle_ids, Getname(),
        frame, reference_line_info);
    /**
     * @brief 停车决策创建完成
     *
     * 函数内部会：
     * 1. 在frame中创建虚拟停车障碍物
     * 2. 为障碍物添加停车决策
     * 3. 将障碍物添加到参考线信息中
     */
  }
  /**
   * @brief 遍历结束
   */
}

}  // namespace planning
/**
 * @brief 命名空间结束标记
 *
 * 语法说明：
 * }  // namespace planning
 * 是命名空间闭合的注释习惯，提高代码可读性
 */
}  // namespace apollo
/**
 * @brief Apollo命名空间结束标记
 */
