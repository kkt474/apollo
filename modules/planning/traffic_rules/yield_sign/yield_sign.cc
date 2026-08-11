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
 * @brief 停车让行交通规则实现文件
 *
 * 功能说明：
 * 实现了让行标志(Yield Sign)交通规则，用于处理让行路口的停车和让行决策
 * 当车辆接近让行标志时，需要减速并在必要时停车等待
 *
 * 核心概念：
 * - Yield Sign：让行标志，要求车辆减速并让行其他道路使用者
 * - PathOverlap：路径重叠区，表示地图上路标与路径的交叉区域
 * - Virtual Obstacle：虚拟障碍物，用于生成停车决策
 *
 * C++语法说明：
 * - apollo命名空间：百度Apollo自动驾驶项目的命名空间
 * - planning模块：规划模块，负责轨迹规划和决策
 **/

#include "modules/planning/traffic_rules/yield_sign/yield_sign.h"

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
 * 表示地图中路标（如让行标志、停车标志）与路径的交叉区域
 * 主要成员：
 * - object_id：对象ID，标识让行标志
 * - start_s：重叠区域起始点沿路径的s坐标
 * - end_s：重叠区域结束点沿路径的s坐标
 */

/**
 * @brief 初始化让行标志规则
 *
 * @param name 规则名称
 * @param injector 依赖注入器指针
 * @return bool 初始化是否成功
 *
 * 功能说明：
 * 初始化YieldSign交通规则实例
 * 加载规则配置并准备处理让行标志
 *
 * 算法流程：
 * 1. 调用基类TrafficRule的Init方法进行基础初始化
 * 2. 如果基础初始化失败，返回false
 * 3. 从配置文件加载YieldSignConfig配置
 * 4. 返回配置加载结果
 *
 * C++语法说明：
 * - const std::string& name：常量引用参数，避免拷贝
 * - std::shared_ptr<DependencyInjector>：智能指针，管理依赖注入器生命周期
 * - std::shared_ptr<T>：C++11智能指针，引用计数，线程安全
 * - &config_：成员变量地址，用于存储加载的配置
 * - TrafficRule::LoadConfig<T>：模板函数，泛型编程
 */
bool YieldSign::Init(const std::string& name,
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
   * @brief 加载让行标志配置
   *
   * TrafficRule::LoadConfig<T>模板函数说明：
   * - 从配置文件读取对应类型的配置
   * - T必须是protobuf消息类型（如YieldSignConfig）
   * - &config_将配置存储到成员变量中
   */
  return TrafficRule::LoadConfig<YieldSignConfig>(&config_);
  /**
   * &config_是取地址操作，获取成员变量的地址
   * LoadConfig是模板方法，<YieldSignConfig>指定模板参数
   */
}

/**
 * @brief 应用让行标志规则
 *
 * @param frame 当前规划帧
 * @param reference_line_info 参考线信息
 * @return Status 应用结果状态
 *
 * 功能说明：
 * 让行标志规则的主入口函数
 * 遍历所有让行标志重叠区，为未完成的让行标志创建停车决策
 *
 * 算法流程：
 * 1. 调用MakeDecisions生成让行决策
 * 2. 返回成功状态
 *
 * C++语法说明：
 * - Frame* const frame：指向常量的指针（指针本身不可变，但指向的内容可变）
 * - ReferenceLineInfo* const reference_line_info：指向常量的指针
 * - const修饰符的位置规则：const靠近类型表示指针指向常量数据
 */
Status YieldSign::ApplyRule(Frame* const frame,
                            ReferenceLineInfo* const reference_line_info) {
  MakeDecisions(frame, reference_line_info);
  /**
   * @brief 生成让行决策
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
 * @brief 生成让行决策
 *
 * @param frame 当前规划帧
 * @param reference_line_info 参考线信息
 *
 * 功能说明：
 * 遍历参考线上的所有让行标志重叠区
 * 为尚未完成的让行标志创建停车决策
 *
 * 算法流程：
 * 1. 检查规则是否启用
 * 2. 获取当前已完成的让行标志列表
 * 3. 获取参考线上的所有让行标志重叠区
 * 4. 对每个让行标志：
 *    - 检查是否已经处理过
 *    - 如果未处理，创建虚拟停车障碍物
 *    - 调用BuildStopDecision生成停车决策
 *
 * C++语法说明：
 * - CHECK_NOTNULL：Apollo宏，用于空指针检查
 * - injector_：成员变量，通过依赖注入获取
 * - planning_context()：获取规划上下文，存储规划过程中的状态
 */
void YieldSign::MakeDecisions(Frame* const frame,
                              ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  /**
   * @brief 空指针检查
   *
   * CHECK_NOTNULL是Apollo定义的宏：
   * - 如果frame为nullptr，程序会终止并输出错误信息
   * - 用于预防编程错误，确保指针有效
   * 语法：CHECK_NOTNULL(ptr)展开为assert或类似检查
   */
  CHECK_NOTNULL(reference_line_info);
  /**
   * @brief 参考线信息空指针检查
   */

  if (!config_.enabled()) {
    /**
     * @brief 检查规则是否启用
     *
     * config_是成员变量，存储YieldSignConfig配置
     * enabled()返回bool，表示规则是否启用
     * !是逻辑非，如果规则未启用则直接返回
     */
    return;  /**< 规则未启用，不进行处理 */
  }

  /**
   * @brief 获取让行标志状态
   *
   * injector_：依赖注入器，提供对各子系统的访问
   * planning_context()：返回PlanningContext智能指针
   * mutable_planning_status()：获取可修改的规划状态
   * yield_sign()：获取yield_sign字段的引用
   *
   * 链式调用语法：
   * injector_->planning_context()->mutable_planning_status()->yield_sign()
   * 等价于：
   * auto context = injector_->planning_context();
   * auto status = context->mutable_planning_status();
   * auto yield_sign_status = status->yield_sign();
   */
  const auto& yield_sign_status =
      injector_->planning_context()->planning_status().yield_sign();

  /**
   * @brief 获取自车前边缘的s坐标
   *
   * reference_line_info->AdcSlBoundary()：获取自车的SL边界
   * .end_s()：获取沿路径方向的后边缘s坐标
   * adc_front_edge_s：自车前边缘s坐标，用于判断让行标志位置
   *
   * ADC = Autonomous Driving Computer（自动驾驶计算机）
   * 自车坐标系：s沿路径方向，l垂直于路径方向
   */
  const double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();

  /**
   * @brief 获取参考线上的所有让行标志重叠区
   *
   * reference_line_info->reference_line()：获取参考线对象
   * .map_path()：获取地图路径（MapPath）
   * .yield_sign_overlaps()：获取所有让行标志重叠区
   *
   * 返回类型：std::vector<PathOverlap>
   * 每个PathOverlap包含让行标志的位置信息（start_s, end_s）
   */
  const std::vector<PathOverlap>& yield_sign_overlaps =
      reference_line_info->reference_line().map_path().yield_sign_overlaps();

  /**
   * @brief 遍历所有让行标志重叠区
   *
   * 范围for循环（C++11特性）：
   * - const auto&：常量引用，避免拷贝
   * - yield_sign_overlap：当前遍历的让行标志
   */
  for (const auto& yield_sign_overlap : yield_sign_overlaps) {
    /**
     * @brief 检查让行标志是否在自车前方
     *
     * yield_sign_overlap.end_s：让行标志重叠区结束点s坐标
     * adc_front_edge_s：自车前边缘s坐标
     * 如果让行标志结束点在自车后方，说明已经通过，跳过
     */
    if (yield_sign_overlap.end_s <= adc_front_edge_s) {
      continue;  /**< 已通过此让行标志，继续下一个 */
    }

    /**
     * @brief 检查让行标志是否已完成
     *
     * 功能：判断此让行标志是否已被处理过
     * 场景/阶段会设置已完成标志，避免重复处理
     *
     * yield_sign_done：标记是否已完成
     * wait_for_obstacle_ids：等待让行的障碍物ID列表
     */
    bool yield_sign_done = false;
    /**
     * @brief 遍历已完成的让行标志ID列表
     *
     * yield_sign_status.done_yield_sign_overlap_id()：
     * 返回已完成的让行标志ID列表（repeated字段）
     * 使用范围for循环遍历
     */
    for (const auto& done_yield_sign_overlap_id :
         yield_sign_status.done_yield_sign_overlap_id()) {
      /**
       * @brief 比较让行标志ID
       *
       * 判断当前让行标志是否在已完成列表中
       * object_id：让行标志的唯一标识符
       */
      if (yield_sign_overlap.object_id == done_yield_sign_overlap_id) {
        yield_sign_done = true;  /**< 标记为已完成 */
        break;  /**< 找到匹配，跳出内层循环 */
      }
    }
    if (yield_sign_done) {
      /**
       * @brief 如果已处理过，跳过此让行标志
       */
      continue;  /**< 已完成，继续下一个让行标志 */
    }

    /**
     * @brief 构建停车决策
     *
     * ADEBUG：调试日志宏，仅在调试模式输出
     * <<：流输出运算符，用于拼接日志字符串
     */
    ADEBUG << "BuildStopDecision: yield_sign[" << yield_sign_overlap.object_id
           << "] start_s[" << yield_sign_overlap.start_s << "]";

    /**
     * @brief 创建虚拟障碍物ID
     *
     * YIELD_SIGN_VO_ID_PREFIX：让行标志虚拟障碍物ID前缀
     * 格式：YIELD_SIGN_VO_ID_PREFIX + 让行标志object_id
     * VO = Virtual Obstacle（虚拟障碍物）
     */
    const std::string virtual_obstacle_id =
        YIELD_SIGN_VO_ID_PREFIX + yield_sign_overlap.object_id;

    /**
     * @brief 获取等待让行的障碍物ID列表
     *
     * yield_sign_status.wait_for_obstacle_id()：
     * 返回需要等待让行的障碍物ID列表
     * std::vector<std::string>：字符串向量，存储障碍物ID
     */
    const std::vector<std::string> wait_for_obstacle_ids(
        yield_sign_status.wait_for_obstacle_id().begin(),
        yield_sign_status.wait_for_obstacle_id().end());

    /**
     * @brief 调用BuildStopDecision创建停车决策
     *
     * 参数说明：
     * - virtual_obstacle_id：虚拟障碍物ID
     * - yield_sign_overlap.start_s：停车点s坐标（让行标志起始位置）
     * - config_.stop_distance()：停车距离（距障碍物的距离）
     * - StopReasonCode::STOP_REASON_YIELD_SIGN：停车原因（让行标志）
     * - wait_for_obstacle_ids：等待让行的障碍物列表
     * - Getname()：获取当前规则名称
     * - frame：当前规划帧
     * - reference_line_info：参考线信息
     *
     * util::BuildStopDecision：
     * 是规划模块提供的工具函数，用于创建标准化的停车决策
     */
    util::BuildStopDecision(
        virtual_obstacle_id, yield_sign_overlap.start_s,
        config_.stop_distance(), StopReasonCode::STOP_REASON_YIELD_SIGN,
        wait_for_obstacle_ids, Getname(), frame, reference_line_info);
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
