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
 * @file traffic_decider.cc
 * @brief 交通规则决策器实现文件
 *
 * 本文件实现了交通规则决策器(TrafficDecider)
 * 用于在规划过程中执行各种交通规则（信号灯、停车标志、让行标志等）
 *
 * 功能说明：
 * 1. 加载交通规则插件列表
 * 2. 按顺序执行各交通规则
 * 3. 构建规划目标点（停车点）
 *
 * 架构说明：
 * TrafficDecider采用插件式架构
 * - 每种交通规则作为一个插件（TrafficRule）
 * - 通过配置文件定义规则执行顺序
 * - 支持扩展新的交通规则
 *
 * 交通规则类型：
 * - 信号灯 (Signal)
 * - 停车标志 (Stop Sign)
 * - 让行标志 (Yield Sign)
 * - 人行横道 (Crosswalk)
 * - 限速标志 (Speed Limit)
 * - 目的地 (Destination)
 *
 * 相关C++语法说明：
 * - std::shared_ptr<T>: 智能指针，共享所有权
 * - std::numeric_limits<double>::infinity(): 获取double类型的正无穷大
 * - plugin机制: 通过类名动态创建实例
 **/

/**
 * @brief 本类的头文件
 *
 * 包含TrafficDecider类的完整定义
 */
#include "modules/planning/planning_interface_base/traffic_rules_base/traffic_decider.h"

/**
 * @brief 标准库头文件
 *
 * <limits>: 提供std::numeric_limits获取类型极值
 * <memory>: 提供std::shared_ptr智能指针
 */
#include <limits>
#include <memory>

/**
 * @brief Cyber RT插件管理器头文件
 *
 * cyber/plugin_manager/plugin_manager.h:
 *   插件管理器，用于动态加载交通规则插件
 *   支持运行时加载不同的规则实现
 */
#include "cyber/plugin_manager/plugin_manager.h"

/**
 * @brief 车辆配置助手头文件
 *
 * VehicleConfigHelper:
 *   车辆配置助手，用于获取车辆参数
 *   如前后边缘到几何中心的距离等
 */
#include "modules/common/configs/vehicle_config_helper.h"

/**
 * @brief 配置工具头文件
 *
 * config_util.h:
 *   配置工具函数
 *   用于获取完整的交通规则类名
 */
#include "modules/planning/planning_base/common/util/config_util.h"

/**
 * @brief 规划模块GFlags头文件
 *
 * FLAGS_traffic_rule_config_filename:
 *   交通规则配置文件路径
 */
#include "modules/planning/planning_base/gflags/planning_gflags.h"

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
using apollo::common::Status;  /**< Apollo通用状态类型 */

/**
 * @brief 交通规则决策器初始化函数
 *
 * @param injector 依赖注入器智能指针
 * @return bool 初始化是否成功
 *
 * 功能说明：
 * 1. 加载交通规则配置文件
 * 2. 创建并初始化各交通规则插件
 * 3. 将插件添加到规则列表
 *
 * 算法流程：
 * 1. 检查是否已初始化，如果是则直接返回
 * 2. 加载配置文件获取规则管道(pipeline)
 * 3. 遍历配置中的每个规则
 * 4. 使用插件管理器创建规则实例
 * 5. 调用规则的Init进行初始化
 * 6. 将规则添加到rule_list_
 *
 * C++语法说明：
 * - const std::shared_ptr<DependencyInjector>& injector:
 *   常量引用，避免拷贝
 *   shared_ptr是引用计数智能指针
 * - if (init_) return true:
 *   惰性初始化模式，避免重复初始化
 * - apollo::cyber::common::LoadConfig<T>():
 *   从配置文件加载protobuf消息
 */
bool TrafficDecider::Init(const std::shared_ptr<DependencyInjector> &injector) {
  /**
   * @brief 惰性初始化检查
   *
   * init_是类的成员变量，初始为false
   * 如果已经初始化，直接返回true
   * 避免重复加载配置和创建插件
   */
  if (init_) return true;

  /**
   * @brief 加载配置文件路径
   *
   * FLAGS_traffic_rule_config_filename:
   *   GFlags变量，从配置文件或命令行获取
   *   通常位于modules/planning/planning_component/conf/traffic_rule_config.pb.txt
   *
   * AINFO:
   *   Apollo信息级别日志
   */
  AINFO << "Load config path:" << FLAGS_traffic_rule_config_filename;

  /**
   * @brief 加载交通规则管道配置
   *
   * apollo::cyber::common::LoadConfig():
   *   Cyber RT的配置加载函数
   *   从文件加载protobuf消息
   *
   * rule_pipeline_:
   *   成员变量，存储规则管道配置
   *   类型是TrafficRulePipeline，包含多个规则的配置
   *
   * C++语法说明：
   * - &rule_pipeline_:
   *   取地址运算符，获取成员的指针
   *   LoadConfig将数据写入该地址
   */
  if (!apollo::cyber::common::LoadConfig(FLAGS_traffic_rule_config_filename,
                                         &rule_pipeline_)) {
    /**
     * @brief 配置加载失败
     */
    AERROR << "Load pipeline of Traffic decider"
           << " failed!";
    return false;  /**< 返回false表示初始化失败 */
  }

  /**
   * @brief 遍历规则管道中的每个规则
   *
   * for循环：
   * - 初始化：int i = 0
   * - 条件：i < rule_pipeline_.rule_size()
   * - 增量：i++
   *
   * rule_pipeline_.rule_size():
   *   获取配置中规则的个数
   *
   * C++语法说明：
   * for (int i = 0; i < rule_pipeline_.rule_size(); i++):
   *   传统for循环，与范围for的区别是可以知道索引
   */
  for (int i = 0; i < rule_pipeline_.rule_size(); i++) {
    /**
     * @brief 创建交通规则插件实例
     *
     * apollo::cyber::plugin_manager::PluginManager::Instance():
     *   获取插件管理器单例
     * CreateInstance<TrafficRule>():
     *   模板方法，根据类名创建TrafficRule实例
     *   返回shared_ptr<TrafficRule>
     *
     * ConfigUtil::GetFullPlanningClassName():
     *   工具函数，将短类名转换为完整类名
     *   如 "Signal" -> "apollo::planning::traffic_rules::Signal"
     *
     * rule_pipeline_.rule(i).type():
     *   获取第i个规则配置的类型名称
     */
    auto rule =
        apollo::cyber::plugin_manager::PluginManager::Instance()
            ->CreateInstance<TrafficRule>(ConfigUtil::GetFullPlanningClassName(
                rule_pipeline_.rule(i).type()));

    /**
     * @brief 检查规则创建是否成功
     */
    if (!rule) {
      /**
       * @brief 规则创建失败
       *
       * rule_pipeline_.rule(i).name():
       *   获取第i个规则的名称
       */
      AERROR << "Init of Traffic rule" << rule_pipeline_.rule(i).name()
             << " failed!";
      return false;  /**< 初始化失败 */
    }

    /**
     * @brief 初始化单个交通规则
     *
     * rule->Init():
     *   调用规则的Init方法
     *   参数：
     *   - rule_pipeline_.rule(i).name(): 规则名称
     *   - injector: 依赖注入器
     */
    rule->Init(rule_pipeline_.rule(i).name(), injector);

    /**
     * @brief 将规则添加到规则列表
     *
     * rule_list_.push_back(rule):
     *   将创建的规则添加到vector
     *   rule是shared_ptr< TrafficRule>
     *
     * C++语法说明：
     * std::vector::push_back():
     *   将元素添加到vector末尾
     *   会触发vector的容量扩张
     */
    rule_list_.push_back(rule);
  }

  /**
   * @brief 标记初始化完成
   */
  init_ = true;

  return true;  /**< 初始化成功 */
}

/**
 * @brief 构建规划目标点（停车点）
 *
 * @param reference_line_info 参考线信息指针
 *
 * 功能说明：
 * 遍历所有障碍物，找出最近的虚拟停车墙
 * 根据停车原因设置停车点的类型（HARD/SOFT）
 * 将停车点设置到参考线信息中
 *
 * 停车点类型说明：
 * - HARD: 必须在该点停车（如目的地、停车标志、人行横道）
 * - SOFT: 建议停车（如黄灯）
 *
 * 算法流程：
 * 1. 初始化最小距离为无穷大
 * 2. 遍历所有障碍物
 * 3. 找出虚拟障碍物中有STOP决策的
 * 4. 比较start_s，找出最近的停车墙
 * 5. 根据停车原因设置停车点类型
 * 6. 计算停车点的参考线S坐标
 * 7. 设置到参考线信息中
 *
 * C++语法说明：
 * - ReferenceLineInfo *:
 *   原始指针，可修改指向的对象
 * - std::numeric_limits<double>::infinity():
 *   获取double类型的正无穷大
 * - const auto *obstacle:
 *   常量指针，指向常量对象
 */
void TrafficDecider::BuildPlanningTarget(
    ReferenceLineInfo *reference_line_info) {
  /**
   * @brief 初始化最小S距离为无穷大
   *
   * 用于找出最近的停车墙
   * 初始值为正无穷，表示还没有找到任何停车墙
   */
  double min_s = std::numeric_limits<double>::infinity();

  /**
   * @brief 定义停车点
   *
   * StopPoint:
   *   停车点数据结构
   *   包含s坐标和停车类型
   */
  StopPoint stop_point;

  /**
   * @brief 遍历路径决策中的所有障碍物
   *
   * for (const auto *obstacle : ...):
   *   范围for循环
   *   obstacle是const Obstacle*类型
   *
   * reference_line_info->path_decision()->obstacles().Items():
   *   获取路径决策中的所有障碍物列表
   */
  for (const auto *obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    /**
     * @brief 检查是否是虚拟障碍物且有停车决策
     *
     * 条件链式调用：
     * - obstacle->IsVirtual(): 是否是虚拟障碍物（人为创建的）
     * - obstacle->HasLongitudinalDecision(): 是否有纵向决策
     * - obstacle->LongitudinalDecision().has_stop(): 纵向决策是否是STOP类型
     * - obstacle->PerceptionSLBoundary().start_s() < min_s:
     *   障碍物的起始S是否小于当前最小值
     *
     * 所有条件都满足时才处理该障碍物
     */
    if (obstacle->IsVirtual() && obstacle->HasLongitudinalDecision() &&
        obstacle->LongitudinalDecision().has_stop() &&
        obstacle->PerceptionSLBoundary().start_s() < min_s) {
      /**
       * @brief 更新最小距离
       */
      min_s = obstacle->PerceptionSLBoundary().start_s();

      /**
       * @brief 获取停车原因代码
       *
       * obstacle->LongitudinalDecision().stop().reason_code():
       *   获取STOP决策中的原因代码
       *   如STOP_REASON_DESTINATION、STOP_REASON_SIGNAL等
       */
      const auto &stop_code =
          obstacle->LongitudinalDecision().stop().reason_code();

      /**
       * @brief 判断停车原因并设置停车点类型
       *
       * switch-case:
       *   多分支选择语句
       *   根据stop_code设置不同的停车类型
       */

      /**
       * @brief HARD停车类型
       *
       * 必须停车的场景：
       * - 目的地 (DESTINATION)
       * - 人行横道 (CROSSWALK)
       * - 停车标志 (STOP_SIGN)
       * - 让行标志 (YIELD_SIGN)
       * - 缓慢通过 (CREEPER)
       * - 参考线终点 (REFERENCE_END)
       * - 信号灯 (SIGNAL)
       */
      if (stop_code == StopReasonCode::STOP_REASON_DESTINATION ||
          stop_code == StopReasonCode::STOP_REASON_CROSSWALK ||
          stop_code == StopReasonCode::STOP_REASON_STOP_SIGN ||
          stop_code == StopReasonCode::STOP_REASON_YIELD_SIGN ||
          stop_code == StopReasonCode::STOP_REASON_CREEPER ||
          stop_code == StopReasonCode::STOP_REASON_REFERENCE_END ||
          stop_code == StopReasonCode::STOP_REASON_SIGNAL) {
        /**
         * @brief 设置为硬停车点
         *
         * StopPoint::HARD:
         *   表示必须完全停止
         */
        stop_point.set_type(StopPoint::HARD);

        /**
         * @brief 打印调试信息
         *
         * StopReasonCode_Name():
         *   将枚举值转换为字符串
         *   方便日志输出和调试
         */
        ADEBUG << "Hard stop at: " << min_s
               << "REASON: " << StopReasonCode_Name(stop_code);
      }
      /**
       * @brief SOFT停车类型
       *
       * 建议停车但不强制
       * 黄灯场景通常设置为SOFT
       */
      else if (stop_code == StopReasonCode::STOP_REASON_YELLOW_SIGNAL) {
        stop_point.set_type(StopPoint::SOFT);
        ADEBUG << "Soft stop at: " << min_s << "  STOP_REASON_YELLOW_SIGNAL";
      }
      /**
       * @brief 其他原因
       */
      else {
        ADEBUG << "No planning target found at reference line.";
      }
    }
  }

  /**
   * @brief 检查是否找到了停车点
   *
   * min_s != std::numeric_limits<double>::infinity():
   *   如果min_s被更新，说明找到了停车墙
   */
  if (min_s != std::numeric_limits<double>::infinity()) {
    /**
     * @brief 获取车辆配置
     *
     * common::VehicleConfigHelper::Instance()->GetConfig():
     *   单例模式获取车辆配置
     * vehicle_param():
     *   获取车辆参数子结构
     */
    const auto &vehicle_config =
        common::VehicleConfigHelper::Instance()->GetConfig();

    /**
     * @brief 获取前边缘到几何中心的距离
     *
     * front_edge_to_center:
     *   车辆前边缘到几何中心的距离
     *   用于将停车墙位置转换为停车点位置
     */
    double front_edge_to_center =
        vehicle_config.vehicle_param().front_edge_to_center();

    /**
     * @brief 计算停车点的S坐标
     *
     * 公式：stop_s = min_s - front_edge_to_center + virtual_stop_wall_length / 2
     *
     * 解释：
     * - min_s: 停车墙的起始S坐标
     * - front_edge_to_center: 前边缘到中心的距离
     *   减去这个值，得到后轴应该停的位置
     * - FLAGS_virtual_stop_wall_length / 2: 停车墙长度的一半
     *   加上这个值，停在停车墙中间
     *
     * 这样计算出的位置是自车后轴应该到达的S坐标
     */
    stop_point.set_s(min_s - front_edge_to_center +
                     FLAGS_virtual_stop_wall_length / 2.0);

    /**
     * @brief 设置到参考线信息
     *
     * reference_line_info->SetLatticeStopPoint(stop_point):
     *   将停车点设置到参考线信息中
     *   供后续的Lattice规划器使用
     */
    reference_line_info->SetLatticeStopPoint(stop_point);
  }
}

/**
 * @brief 执行交通规则决策
 *
 * @param frame 当前规划帧
 * @param reference_line_info 参考线信息
 * @return Status 执行状态
 *
 * 功能说明：
 * 按顺序执行所有交通规则
 * 并在最后构建规划目标点
 *
 * 算法流程：
 * 1. 检查输入参数有效性
 * 2. 遍历规则列表
 * 3. 对每个规则：
 *    a. 重置规则状态
 *    b. 应用规则
 * 4. 构建规划目标点
 * 5. 返回执行状态
 *
 * 规则执行顺序：
 * 由配置文件中的rule_pipeline_定义
 * 通常顺序为：信号灯 -> 停车标志 -> 让行标志 -> ...
 *
 * C++语法说明：
 * - Frame *frame:
 *   原始指针，表示当前规划帧
 * - CHECK_NOTNULL:
 *   Apollo断言宏，检查指针是否为空
 * - const auto &rule:
 *   常量引用，避免拷贝
 */
Status TrafficDecider::Execute(Frame *frame,
                               ReferenceLineInfo *reference_line_info) {
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
   * @brief 遍历规则列表并执行
   *
   * for (const auto &rule : rule_list_):
   *   范围for循环，遍历所有交通规则
   */
  for (const auto &rule : rule_list_) {
    /**
     * @brief 检查规则是否有效
     */
    if (!rule) {
      /**
       * @brief 规则无效，记录警告并继续
       */
      AERROR << "Could not find rule ";
      continue;  /**< 跳过当前规则，继续下一个 */
    }

    /**
     * @brief 重置规则状态
     *
     * rule->Reset():
     *   重置规则的内部状态
     *   确保规则从一致的状态开始执行
     */
    rule->Reset();

    /**
     * @brief 应用规则
     *
     * rule->ApplyRule():
     *   执行规则的核心逻辑
     *   参数：
     *   - frame: 当前规划帧
     *   - reference_line_info: 参考线信息
     *
     * ApplyRule可能修改：
     * - frame中的数据（如障碍物决策）
     * - reference_line_info中的数据（如限速）
     */
    rule->ApplyRule(frame, reference_line_info);

    /**
     * @brief 打印调试信息
     *
     * rule->Getname():
     *   获取规则名称
     */
    ADEBUG << "Applied rule " << rule->Getname();
  }

  /**
   * @brief 构建规划目标点
   *
   * BuildPlanningTarget():
   *   遍历所有障碍物，找出最近的停车墙
   *   并将停车点设置到reference_line_info中
   */
  BuildPlanningTarget(reference_line_info);

  return Status::OK();  /**< 执行成功 */
}

}  // namespace planning
}  // namespace apollo
