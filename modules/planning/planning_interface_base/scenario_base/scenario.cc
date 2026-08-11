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
 * @file scenario.cc
 * @brief 场景基类实现文件
 *
 * 本文件实现Scenario类，是所有驾驶场景的基类。
 * Scenario定义了场景的通用接口和生命周期管理。
 *
 * 主要功能：
 * 1. 场景初始化（加载配置、创建阶段）
 * 2. 场景处理（执行当前阶段、管理阶段切换）
 * 3. 场景状态管理（Enter/Exit/Reset）
 *
 * 设计模式：
 * - 模板方法模式：Process定义了处理流程，子类实现具体逻辑
 * - 插件模式：通过PluginManager动态加载
 * - 状态机模式：阶段之间可以相互切换
 *
 * C++语法说明：
 * - 初始化列表: : member_(value){} 构造函数初始化
 * - mutable_xxx(): protobuf可写访问器
 * - abi::__cxa_demangle(): C++ ABI名称解码
 * - typeid(*this).name(): 获取运行时类型名称
 * - switch-case: 分支语句处理不同状态
 * - std::shared_ptr<T>: 引用计数智能指针
 */
#include "modules/planning/planning_interface_base/scenario_base/scenario.h"

#include <cxxabi.h>  /**< C++ ABI名称解码库，用于获取类型名称 */

#include "cyber/class_loader/class_loader_manager.h" /**< Cyber RT类加载管理器 */
#include "cyber/plugin_manager/plugin_manager.h"    /**< Cyber RT插件管理器 */
#include "modules/planning/planning_base/common/frame.h" /**< 规划帧 */
#include "modules/planning/planning_base/common/util/config_util.h" /**< 配置工具 */
#include "modules/planning/planning_interface_base/scenario_base/stage.h" /**< 阶段基类 */

namespace apollo {
/**
 * apollo:: - Apollo最外层命名空间
 */
namespace planning {

/**
 * @brief Scenario默认构造函数
 *
 * 使用初始化列表初始化所有成员变量。
 *
 * 语法说明：
 * - : member_(value){} 初始化列表语法
 * - ScenarioResult(...): 委托构造函数，初始化场景结果
 * - nullptr: C++11空指针
 * - "": 空字符串初始化
 */
Scenario::Scenario()
    : scenario_result_(ScenarioResult(ScenarioStatusType::STATUS_UNKNOWN)),
      current_stage_(nullptr),    /**< 当前阶段指针，初始为空 */
      msg_(""),                   /**< 场景消息，空字符串 */
      injector_(nullptr),         /**< 依赖注入器，初始为空 */
      config_path_(""),           /**< 配置路径，空字符串 */
      config_dir_(""),            /**< 配置目录，空字符串 */
      name_("") {}                /**< 场景名称，空字符串 */

/**
 * @brief 场景初始化函数
 *
 * 初始化场景，包括：
 * 1. 保存场景名称和依赖注入器
 * 2. 在规划上下文中设置场景类型
 * 3. 获取配置文件路径
 * 4. 加载阶段管道配置
 *
 * @param injector 依赖注入器指针
 * @param name 场景名称
 * @return bool 初始化是否成功
 *
 * 语法说明：
 * - std::shared_ptr<DependencyInjector>: 共享指针，管理依赖注入器
 * - const std::string&: 常量引用字符串
 * - mutable_xxx(): protobuf可写访问器
 * - auto*: 自动推断指针类型
 * - abi::__cxa_demangle(): 解码C++ mangled名称为可读名称
 * - typeid(*this).name(): 获取当前对象类型名
 * - PluginManager::Instance()->GetPluginxxxPath(): 获取插件路径
 */
bool Scenario::Init(std::shared_ptr<DependencyInjector> injector,
                    const std::string& name) {
  name_ = name;   /**< 保存场景名称 */
  injector_ = injector;  /**< 保存依赖注入器 */

  /**
   * 在规划上下文中设置场景类型
   * injector_->planning_context() 获取规划上下文
   * ->mutable_planning_status() 获取可写的规划状态
   * ->mutable_scenario() 获取可写的场景信息
   */
  auto* scenario = injector_->planning_context()
                       ->mutable_planning_status()
                       ->mutable_scenario();
  scenario->Clear();   /**< 清空之前的场景信息 */
  scenario->set_scenario_type(name_);  /**< 设置新场景类型 */

  /**
   * 获取类名用于加载配置
   * abi::__cxa_demangle(typeid(*this).name(), 0, 0, &status)
   *   - typeid(*this).name(): 获取当前类的mangled名称
   *   - abi::__cxa_demangle(): 将mangled名称解码为可读名称
   *   - status: 输出参数，解码状态
   */
  int status;
  std::string class_name =
      abi::__cxa_demangle(typeid(*this).name(), 0, 0, &status);

  /**
   * 获取插件配置目录
   * PluginManager::Instance()->GetPluginClassHomePath<Scenario>(class_name)
   *   - 获取Scenario类型插件的主目录
   */
  config_dir_ = apollo::cyber::plugin_manager::PluginManager::Instance()
                    ->GetPluginClassHomePath<Scenario>(class_name);
  config_dir_ += "/conf";  /**< 添加conf子目录 */
  AINFO << "config_dir : " << config_dir_;

  /**
   * 获取场景配置文件路径
   * GetPluginConfPath<Scenario>(class_name, "conf/scenario_conf.pb.txt")
   *   - 获取特定插件的配置文件路径
   */
  config_path_ = apollo::cyber::plugin_manager::PluginManager::Instance()
                     ->GetPluginConfPath<Scenario>(class_name,
                                                   "conf/scenario_conf.pb.txt");

  /**
   * 获取阶段管道配置文件路径
   * pipeline.pb.txt定义了场景包含的阶段列表
   */
  std::string pipeline_config_path =
      apollo::cyber::plugin_manager::PluginManager::Instance()
          ->GetPluginConfPath<Scenario>(class_name, "conf/pipeline.pb.txt");
  AINFO << "Load config path:" << pipeline_config_path;

  /**
   * 从文件加载阶段管道配置
   * GetProtoFromFile(&scenario_pipeline_config_)
   *   - 将protobuf消息从文件加载到scenario_pipeline_config_
   */
  if (!apollo::cyber::common::GetProtoFromFile(pipeline_config_path,
                                               &scenario_pipeline_config_)) {
    AERROR << "Load pipeline of " << name_ << " failed!";
    return false;  /**< 加载失败返回false */
  }

  /**
   * 构建阶段名称到配置的映射
   * for (const auto& stage : scenario_pipeline_config_.stage())
   *   - 遍历配置中的所有阶段
   * stage_pipeline_map_[stage.name()] = &stage
   *   - 用阶段名作为key，阶段配置指针作为value
   */
  for (const auto& stage : scenario_pipeline_config_.stage()) {
    stage_pipeline_map_[stage.name()] = &stage;  /**< 建立映射表 */
  }

  return true;  /**< 初始化成功 */
}

/**
 * @brief 场景处理主函数
 *
 * 场景的核心处理函数，在每个规划周期被调用。
 * 负责管理阶段的执行和切换。
 *
 * 处理流程：
 * 1. 如果没有当前阶段，创建第一个阶段
 * 2. 调用当前阶段的Process方法
 * 3. 根据阶段返回值决定后续动作
 * 4. 管理阶段切换
 *
 * @param planning_init_point 规划起始点
 * @param frame 规划帧数据
 * @return ScenarioResult 场景处理结果
 *
 * 语法说明：
 * - const common::TrajectoryPoint&: 常量引用轨迹点
 * - Frame*: 原始指针指向规划帧
 * - switch-case: 分支语句处理不同阶段状态
 * - ->: 指针调用成员方法
 */
ScenarioResult Scenario::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  /**
   * 情况1：没有当前阶段，需要创建
   * if (current_stage_ == nullptr)
   */
  if (current_stage_ == nullptr) {
    /**
     * 创建第一个阶段
     * CreateStage(stage_pipeline_config_.stage(0))
     *   - 获取配置中第一个阶段的配置
     *   - 调用CreateStage创建阶段实例
     */
    // stage init
    current_stage_ = CreateStage(
        *stage_pipeline_map_[scenario_pipeline_config_.stage(0).name()]);

    /**
     * 检查阶段创建是否成功
     */
    if (nullptr == current_stage_) {
      AERROR << "Create stage " << scenario_pipeline_config_.stage(0).name()
             << " failed!";
      scenario_result_.SetStageResult(StageStatusType::ERROR);  /**< 设置阶段错误 */
      return scenario_result_;  /**< 返回错误结果 */
    }
    AINFO << "Create stage " << current_stage_->Name();
  }

  /**
   * 情况2：阶段名称为空
   * if (current_stage_->Name().empty())
   * 表示场景处理完成
   */
  if (current_stage_->Name().empty()) {
    scenario_result_.SetScenarioStatus(ScenarioStatusType::STATUS_DONE); /**< 设置场景完成 */
    return scenario_result_;
  }

  /**
   * 调用当前阶段的Process方法
   * auto ret = current_stage_->Process(planning_init_point, frame)
   *   - 执行阶段处理
   *   - 返回StageResult表示处理结果
   */
  // stage Process
  auto ret = current_stage_->Process(planning_init_point, frame);
  scenario_result_.SetStageResult(ret);  /**< 保存阶段结果 */

  /**
   * 根据阶段返回值处理不同情况
   * switch (ret.GetStageStatus())
   *   - ERROR: 阶段出错
   *   - RUNNING: 阶段运行中
   *   - FINISHED: 阶段完成
   */
  switch (ret.GetStageStatus()) {
    /**
     * ERROR: 阶段执行出错
     */
    case StageStatusType::ERROR: {
      AERROR << "Stage '" << current_stage_->Name() << "' returns error";
      scenario_result_.SetScenarioStatus(ScenarioStatusType::STATUS_UNKNOWN); /**< 场景状态未知 */
      break;  /**< 跳出switch */
    }

    /**
     * RUNNING: 阶段正在运行
     */
    case StageStatusType::RUNNING: {
      scenario_result_.SetScenarioStatus(ScenarioStatusType::STATUS_PROCESSING); /**< 设置处理中状态 */
      break;
    }

    /**
     * FINISHED: 阶段完成，需要切换到下一阶段
     */
    case StageStatusType::FINISHED: {
      /**
       * 获取下一阶段名称
       * current_stage_->NextStage()
       */
      auto next_stage = current_stage_->NextStage();

      /**
       * 检查是否需要切换阶段
       * if (next_stage != current_stage_->Name())
       */
      if (next_stage != current_stage_->Name()) {
        AINFO << "switch stage from " << current_stage_->Name() << " to "
              << next_stage;

        /**
         * 如果下一阶段名称为空，场景完成
         */
        if (next_stage.empty()) {
          scenario_result_.SetScenarioStatus(ScenarioStatusType::STATUS_DONE); /**< 场景完成 */
          return scenario_result_;
        }

        /**
         * 检查下一阶段是否在映射表中
         * if (stage_pipeline_map_.find(next_stage) == stage_pipeline_map_.end())
         */
        if (stage_pipeline_map_.find(next_stage) == stage_pipeline_map_.end()) {
          AERROR << "Failed to find config for stage: " << next_stage;
          scenario_result_.SetScenarioStatus(
              ScenarioStatusType::STATUS_UNKNOWN);  /**< 状态未知 */
          return scenario_result_;
        }

        /**
         * 创建下一阶段
         */
        current_stage_ = CreateStage(*stage_pipeline_map_[next_stage]);

        /**
         * 检查阶段创建是否成功
         */
        if (current_stage_ == nullptr) {
          AWARN << "Current stage is a null pointer.";
          scenario_result_.SetScenarioStatus(
              ScenarioStatusType::STATUS_UNKNOWN);
          return scenario_result_;
        }
      }

      /**
       * 设置场景状态
       * 根据当前阶段是否有效决定是处理中还是完成
       */
      if (current_stage_ != nullptr && !current_stage_->Name().empty()) {
        scenario_result_.SetScenarioStatus(
            ScenarioStatusType::STATUS_PROCESSING);  /**< 继续处理 */
      } else {
        scenario_result_.SetScenarioStatus(ScenarioStatusType::STATUS_DONE); /**< 场景完成 */
      }
      break;
    }

    /**
     * default: 未知状态
     */
    default: {
      AWARN << "Unexpected Stage return value: "
            << static_cast<int>(ret.GetStageStatus());  /**< 转换为整数输出 */
      scenario_result_.SetScenarioStatus(ScenarioStatusType::STATUS_UNKNOWN);
    }
  }

  return scenario_result_;  /**< 返回场景结果 */
}

/**
 * @brief 创建阶段实例
 *
 * 使用插件管理器创建阶段的实例。
 *
 * @param stage_pipeline 阶段管道配置
 * @return std::shared_ptr<Stage> 创建的阶段指针
 *
 * 语法说明：
 * - const StagePipeline&: 常量引用阶段配置
 * - std::shared_ptr<Stage>: 共享指针，管理阶段生命周期
 * - PluginManager::Instance()->CreateInstance<Stage>(): 创建Stage插件实例
 * - ConfigUtil::GetFullPlanningClassName(): 获取完整类名
 */
std::shared_ptr<Stage> Scenario::CreateStage(
    const StagePipeline& stage_pipeline) {
  /**
   * 创建阶段实例
   * PluginManager::Instance()->CreateInstance<Stage>(class_name)
   *   - 根据阶段类型名创建Stage实例
   */
  auto stage_ptr =
      apollo::cyber::plugin_manager::PluginManager::Instance()
          ->CreateInstance<Stage>(
              ConfigUtil::GetFullPlanningClassName(stage_pipeline.type()));

  /**
   * 检查创建是否成功并初始化
   * nullptr == stage_ptr: 检查是否为空
   * &&: 逻辑与
   * !stage_ptr->Init(...): 调用Init方法
   */
  if (nullptr == stage_ptr ||
      !stage_ptr->Init(stage_pipeline, injector_, config_dir_, GetContext())) {
    AERROR << "Create stage " << stage_pipeline.name() << " of " << name_
           << " failed!";
    return nullptr;  /**< 创建或初始化失败返回空指针 */
  }

  return stage_ptr;  /**< 返回创建的阶段指针 */
}

/**
 * @brief 获取当前阶段名称
 *
 * @return std::string 当前阶段名称，如果无阶段则返回空字符串
 *
 * 语法说明：
 * - const成员函数，不能修改成员变量
 * - current_stage_ ? current_stage_->Name() : ""
 *   - 三元运算符，如果current_stage_有效返回名称，否则返回空字符串
 */
const std::string Scenario::GetStage() const {
  return current_stage_ ? current_stage_->Name() : "";
}

/**
 * @brief 重置场景
 *
 * 重置场景状态，清空当前阶段和结果。
 */
void Scenario::Reset() {
  scenario_result_ = ScenarioResult(ScenarioStatusType::STATUS_UNKNOWN); /**< 重置结果为未知 */
  current_stage_ = nullptr;  /**< 清空当前阶段 */
}

}  // namespace planning
}  // namespace apollo
