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
 * @file public_road_planner.cc
 * @brief PublicRoad规划器实现文件
 *
 * 本文件实现PublicRoadPlanner类，它是Apollo规划模块的主要规划器之一。
 * PublicRoadPlanner采用基于场景（Scenario）的规划架构，支持多种驾驶场景。
 *
 * 主要功能：
 * 1. 初始化规划器配置和依赖注入
 * 2. 管理驾驶场景（Scenario）的更新和切换
 * 3. 调用场景处理函数进行轨迹规划
 * 4. 输出调试信息和规划结果
 *
 * 设计特点：
 * - 基于插件架构，可动态加载不同规划器
 * - 场景管理器（ScenarioManager）负责场景切换
 * - 委托模式：具体规划逻辑委托给Scenario处理
 *
 * C++语法说明：
 * - std::shared_ptr<T>: 共享所有权智能指针，引用计数管理
 * - std::shared_ptr<T>&: 常量引用，避免拷贝
 * - auto关键字: 编译器自动推断变量类型
 * - if (!ptr): 智能指针的布尔转换，检查是否为空
 * - Status类: Apollo通用状态类，包含error_code和message
 */
#include "modules/planning/planners/public_road/public_road_planner.h"

#include "modules/planning/planning_base/gflags/planning_gflags.h" /**< 规划配置标志 */
#include "modules/planning/planning_interface_base/scenario_base/scenario.h" /**< 场景基类 */

namespace apollo {
/**
 * apollo:: - Apollo最外层命名空间
 */
namespace planning {

/**
 * using声明 - 将其他命名空间的类型引入当前作用域
 * 语法：using 命名空间::类型名;
 * 这样可以直接使用Status和TrajectoryPoint而不需要完整前缀
 */
using apollo::common::Status;        /**< Apollo通用状态类型 */
using apollo::common::TrajectoryPoint; /**< 轨迹点类型 */

/**
 * @brief 规划器初始化函数
 *
 * 初始化PublicRoadPlanner，包括：
 * 1. 调用基类Planner的初始化方法
 * 2. 加载规划器配置参数
 * 3. 初始化场景管理器
 *
 * @param injector 依赖注入器指针，用于获取各种服务
 * @param config_path 配置文件路径
 * @return Status 初始化状态，成功返回OK
 *
 * 语法说明：
 * - const std::shared_ptr<DependencyInjector>&: 常量引用智能指针
 *   - shared_ptr: 引用计数智能指针，多个指针可共享所有权
 *   - const: 引用本身是const，不能修改指针指向的对象
 *   - &: 引用传递，避免拷贝指针
 * - const std::string&: 常量引用字符串，函数内只读
 * - LoadConfig<T>(path, &config): 模板函数，从文件加载配置到config
 * - scenario_manager_.Init(injector, config_): 初始化场景管理器
 * - return Status::OK(): 返回成功状态
 */
Status PublicRoadPlanner::Init(
    const std::shared_ptr<DependencyInjector>& injector,
    const std::string& config_path) {
  /**
   * 调用基类Planner的Init方法
   * 这是委托调用的典型模式：先调用基类初始化，再做子类特定初始化
   * Planner::Init(injector, config_path) - 静态调用基类方法
   */
  Planner::Init(injector, config_path);

  /**
   * 加载规划器配置
   * LoadConfig<PlannerPublicRoadConfig>(config_path, &config_)
   *   - 模板函数，T指定配置类型
   *   - config_path: 配置文件路径
   *   - &config_: 输出参数，存储加载的配置
   *   - 使用&获取config_的地址传递给函数
   */
  LoadConfig<PlannerPublicRoadConfig>(config_path, &config_);

  /**
   * 初始化场景管理器
   * scenario_manager_是PublicRoadPlanner的成员变量
   * .Init(injector, config_): 调用场景管理器的Init方法
   *   - injector: 依赖注入器，用于获取各种服务和数据
   *   - config_: 规划器配置
   */
  scenario_manager_.Init(injector, config_);

  /**
   * 返回成功状态
   * Status::OK()是Status类的静态方法，返回一个表示成功的Status对象
   */
  return Status::OK();
}

/**
 * @brief 轨迹规划主函数
 *
 * PublicRoadPlanner的核心函数，在每个规划周期被调用。
 * 负责：
 * 1. 更新当前驾驶场景
 * 2. 调用场景处理进行轨迹规划
 * 3. 记录调试信息
 * 4. 返回规划结果
 *
 * @param planning_start_point 规划起始点（当前车辆状态）
 * @param frame 规划帧数据，包含所有输入信息
 * @param ptr_computed_trajectory 输出的规划轨迹指针
 * @return Status 规划状态
 *
 * 语法说明：
 * - TrajectoryPoint: 轨迹点类型，包含位置、速度、加速度等信息
 * - Frame*: 原始指针，指向规划帧数据
 *   - 使用原始指针而非智能指针，因为调用者负责生命周期管理
 *   - Frame对象在调用者那边创建和销毁
 * - ADCTrajectory*: 输出参数指针，用于返回规划轨迹
 *   - ADC = Autonomous Driving Computer，自动驾驶计算机
 *   - ADCTrajectory是protobuf消息类型
 * - auto result = ...: 自动推断result的类型
 */
Status PublicRoadPlanner::Plan(const TrajectoryPoint& planning_start_point,
                               Frame* frame,
                               ADCTrajectory* ptr_computed_trajectory) {
  /**
   * 更新场景管理器
   * scenario_manager_.Update(planning_start_point, frame)
   *   - 根据当前车辆状态和规划帧数据更新场景
   *   - 决定是否需要切换到新的驾驶场景
   *   - 例如：从车道保持切换到换道
   */
  scenario_manager_.Update(planning_start_point, frame);

  /**
   * 获取当前场景指针
   * scenario_manager_.mutable_scenario()
   *   - mutable_前缀表示可写访问器
   *   - 返回Scenario*指针
   *   - scenario_是成员变量，存储当前场景
   */
  scenario_ = scenario_manager_.mutable_scenario();

  /**
   * 检查场景是否有效
   * if (!scenario_) 等价于 if (scenario_ == nullptr)
   * 智能指针的operator bool()支持隐式转换为bool
   */
  if (!scenario_) {
    /**
     * 场景无效，返回错误状态
     * Status(error_code, message): 构造函数
     *   - apollo::common::ErrorCode::PLANNING_ERROR: 错误码
     *   - "Unknown Scenario": 错误消息
     */
    return Status(apollo::common::ErrorCode::PLANNING_ERROR,
                  "Unknown Scenario");
  }

  /**
   * 调用场景处理函数进行实际规划
   * scenario_->Process(planning_start_point, frame)
   *   - 将规划工作委托给当前场景处理
   *   - Process是Scenario的虚函数，由具体场景实现
   *   - 返回TaskStatus，表示处理结果
   */
  auto result = scenario_->Process(planning_start_point, frame);

  /**
   * 记录调试信息（可选）
   * FLAGS_enable_record_debug: gflags配置，控制是否记录调试信息
   * 使用if包裹调试代码，便于生产环境禁用
   */
  if (FLAGS_enable_record_debug) {
    /**
     * mutable_debug(): 获取可写的调试信息指针
     * ->mutable_planning_data(): 继续获取嵌套结构的可写指针
     *   这种链式调用是protobuf的常见模式
     */
    auto scenario_debug = ptr_computed_trajectory->mutable_debug()
                              ->mutable_planning_data()
                              ->mutable_scenario();

    /**
     * 设置场景调试信息
     * scenario_->Name(): 获取场景名称（如"LANE_KEEP"、"CHANGE_LANE"）
     * set_xxx(): protobuf的setter方法
     */
    scenario_debug->set_scenario_plugin_type(scenario_->Name());   /**< 设置场景类型 */
    scenario_debug->set_stage_plugin_type(scenario_->GetStage()); /**< 设置当前阶段 */
    scenario_debug->set_msg(scenario_->GetMsg());                 /**< 设置场景消息 */
  }

  /**
   * 检查场景处理结果
   * result.GetScenarioStatus(): 获取场景状态
   *   - STATUS_DONE: 场景处理完成
   *   - STATUS_UNKNOWN: 状态未知（可能出错）
   */
  if (result.GetScenarioStatus() == ScenarioStatusType::STATUS_DONE) {
    /**
     * 场景处理完成，更新场景管理器
     *
     * 注释说明：只有当前场景状态为STATUS_DONE时才更新场景管理器
     * 这是为了避免在场景处理过程中过早切换场景
     */
    // only updates scenario manager when previous scenario's status is
    // STATUS_DONE
    scenario_manager_.Update(planning_start_point, frame);

  } else if (result.GetScenarioStatus() == ScenarioStatusType::STATUS_UNKNOWN) {
    /**
     * 场景状态未知，表示处理出错
     * 返回错误状态
     *
     * result.GetTaskStatus().error_message(): 获取错误消息
     * common::PLANNING_ERROR: 规划模块的错误码
     */
    return Status(common::PLANNING_ERROR,
                  result.GetTaskStatus().error_message());
  }

  /**
   * 正常返回
   * common::OK: 成功状态码
   * result.GetTaskStatus().error_message(): 获取错误消息（成功时为空字符串）
   */
  return Status(common::OK, result.GetTaskStatus().error_message());
}

}  // namespace planning
}  // namespace apollo
