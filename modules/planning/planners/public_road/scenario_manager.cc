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
 * @file scenario_manager.cc
 * @brief 场景管理器实现文件
 *
 * 本文件实现ScenarioManager类，负责管理自动驾驶中的各种驾驶场景。
 * 场景管理器根据当前环境状态决定使用哪个场景，并在适当时候进行场景切换。
 *
 * 主要功能：
 * 1. 加载和初始化所有配置的场景
 * 2. 根据条件判断是否需要切换场景
 * 3. 管理场景的生命周期（Enter/Exit/Reset）
 * 4. 维护当前场景和默认场景
 *
 * 设计模式：
 * - 插件模式：通过PluginManager动态加载场景
 * - 状态机模式：场景之间可以相互转换
 * - 单例模式：PluginManager使用单例
 *
 * C++语法说明：
 * - std::shared_ptr<T>: 引用计数智能指针
 * - std::vector<T>: 动态数组容器
 * - for循环遍历容器
 * - CHECK_NOTNULL(ptr): 断言检查指针非空
 * - PluginManager::Instance(): 获取单例实例
 * - .get(): 获取智能指针管理的原始指针
 */
#include "modules/planning/planners/public_road/scenario_manager.h"

#include <algorithm>  /**< C++标准算法库 */
#include <string>     /**< C++标准字符串库 */
#include <vector>     /**< C++标准向量容器 */

#include "cyber/plugin_manager/plugin_manager.h" /**< Cyber RT插件管理器 */
#include "modules/common/status/status.h"       /**< Apollo状态类 */
#include "modules/planning/planning_base/common/util/config_util.h" /**< 配置工具 */
#include "modules/planning/planning_interface_base/scenario_base/scenario.h" /**< 场景基类 */

namespace apollo {
/**
 * apollo:: - Apollo最外层命名空间
 */
namespace planning {

/**
 * using声明 - 将其他命名空间的类型引入当前作用域
 */
using apollo::cyber::plugin_manager::PluginManager; /**< Cyber RT插件管理器类 */

/**
 * @brief 场景管理器初始化函数
 *
 * 初始化场景管理器，包括：
 * 1. 检查是否已初始化（避免重复初始化）
 * 2. 保存依赖注入器
 * 3. 通过插件管理器加载所有配置的场景
 * 4. 设置默认场景
 *
 * @param injector 依赖注入器指针，用于获取各种服务
 * @param planner_config 规划器配置，包含场景列表
 * @return bool 初始化是否成功
 *
 * 语法说明：
 * - const std::shared_ptr<DependencyInjector>&: 常量引用智能指针
 * - const PlannerPublicRoadConfig&: 常量引用配置
 * - if (init_): 检查是否已初始化标志
 * - planner_config.scenario_size(): 获取配置中场景的数量
 * - PluginManager::Instance()->CreateInstance<T>(): 创建插件实例
 * - scenario_list_.push_back(scenario): 将场景添加到列表
 * - .get(): shared_ptr的方法，返回原始指针
 */
bool ScenarioManager::Init(const std::shared_ptr<DependencyInjector>& injector,
                           const PlannerPublicRoadConfig& planner_config) {
  /**
   * 检查是否已初始化
   * if (init_) 如果已经初始化，直接返回true
   * 避免重复初始化
   */
  if (init_) {
    return true;  /**< 已初始化，直接返回成功 */
  }

  /**
   * 保存依赖注入器
   * injector_ = injector: 智能指针赋值，引用计数+1
   */
  injector_ = injector;

  /**
   * 遍历配置中的所有场景
   * for (int i = 0; i < planner_config.scenario_size(); i++)
   *   - scenario_size(): protobuf的repeated字段大小
   *   - i从0到scenario_size-1
   */
  for (int i = 0; i < planner_config.scenario_size(); i++) {
    /**
     * 使用插件管理器创建场景实例
     * PluginManager::Instance()->CreateInstance<Scenario>(class_name)
     *   - Instance(): 获取PluginManager单例
     *   - CreateInstance<T>: 模板方法，创建类型T的实例
     *   - ConfigUtil::GetFullPlanningClassName(...): 获取类的完整名称
     *     将简单类型名转换为完整的类名字符串
     *   - planner_config.scenario(i).type(): 获取第i个场景的类型
     */
    auto scenario = PluginManager::Instance()->CreateInstance<Scenario>(
        ConfigUtil::GetFullPlanningClassName(
            planner_config.scenario(i).type()));

    /**
     * ACHECK - Apollo断言宏
     * 检查场景是否成功初始化
     * scenario->Init(...) 调用场景的初始化方法
     * << 操作符连接错误消息
     */
    ACHECK(scenario->Init(injector_, planner_config.scenario(i).name()))
        << "Can not init scenario" << planner_config.scenario(i).name();

    /**
     * 将场景添加到场景列表
     * scenario_list_.push_back(scenario)
     *   - push_back: vector的方法，在末尾添加元素
     *   - scenario是shared_ptr<Scenario>类型
     */
    scenario_list_.push_back(scenario);

    /**
     * 查找并保存默认场景
     * if (planner_config.scenario(i).name() == "LANE_FOLLOW")
     *   - .name(): 获取场景名称
     *   - "LANE_FOLLOW" 是车道跟随场景的名称
     */
    if (planner_config.scenario(i).name() == "LANE_FOLLOW") {
      default_scenario_type_ = scenario; /**< 保存默认场景指针 */
    }
  }

  /**
   * AINFO - Apollo信息级别日志
   * 输出加载的场景列表
   * planner_config.DebugString(): protobuf的调试字符串表示
   */
  AINFO << "Load scenario list:" << planner_config.DebugString();

  /**
   * 设置当前场景为默认场景
   * current_scenario_ = default_scenario_type_
   *   - current_scenario_: 成员变量，当前激活的场景
   *   - default_scenario_type_: 默认场景（车道跟随）
   */
  current_scenario_ = default_scenario_type_;

  /**
   * 设置初始化标志
   * init_ = true: 标记为已初始化
   */
  init_ = true;

  return true;  /**< 初始化成功 */
}

/**
 * @brief 更新当前场景
 *
 * 根据当前车辆状态和规划帧数据，判断是否需要切换到其他场景。
 * 这是场景管理器的核心逻辑。
 *
 * 切换逻辑：
 * 1. 如果当前场景正在处理中，不切换（高优先级）
 * 2. 否则检查是否有可转移的场景
 * 3. 如果有，执行场景切换
 *
 * @param ego_point 自车轨迹点（当前位置）
 * @param frame 规划帧数据
 *
 * 语法说明：
 * - const common::TrajectoryPoint&: 常量引用轨迹点
 * - Frame*: 原始指针，指向规划帧
 * - CHECK_NOTNULL(ptr): 断言检查指针非空，失败终止程序
 * - for (auto scenario : scenario_list_): 范围for循环遍历
 * - .get(): shared_ptr获取原始指针
 * - current_scenario_.get() == scenario.get(): 比较原始指针是否相等
 */
void ScenarioManager::Update(const common::TrajectoryPoint& ego_point,
                             Frame* frame) {
  /**
   * CHECK_NOTNULL - 断言检查frame指针非空
   * 如果frame为nullptr，程序会终止并输出错误
   * 这是防御性编程，确保后续代码不会访问空指针
   */
  CHECK_NOTNULL(frame);

  /**
   * 遍历所有场景，检查是否可以切换
   * for (auto scenario : scenario_list_)
   *   - auto: 自动推断scenario类型为shared_ptr<Scenario>
   *   - : 范围for循环语法，遍历容器所有元素
   */
  for (auto scenario : scenario_list_) {
    /**
     * 情况1：当前场景正在处理中，不切换
     *
     * 条件：
     * - current_scenario_.get() == scenario.get(): 当前遍历的场景就是当前场景
     * - current_scenario_->GetStatus() == STATUS_PROCESSING: 场景状态为处理中
     *
     * 原因：场景正在处理中时有最高优先级，不能被打断
     */
    if (current_scenario_.get() == scenario.get() &&
        current_scenario_->GetStatus() ==
            ScenarioStatusType::STATUS_PROCESSING) {
      /**
       * 直接返回，不进行场景切换
       * 当前场景继续处理
       */
      return;  /**< 不切换，保持当前场景 */
    }

    /**
     * 情况2：检查当前遍历的场景是否可以转移
     *
     * scenario->IsTransferable(current_scenario_.get(), *frame)
     *   - IsTransferable: 场景的方法，判断是否可以切换到该场景
     *   - 参数1：当前场景指针
     *   - 参数2：规划帧数据引用
     *   - 返回bool：是否可以切换
     */
    if (scenario->IsTransferable(current_scenario_.get(), *frame)) {
      /**
       * 可以切换，执行场景切换
       */

      /**
       * 调用当前场景的Exit方法
       * current_scenario_->Exit(frame)
       *   - Exit: 场景退出时调用的方法
       *   - 用于清理状态、保存数据等
       */
      current_scenario_->Exit(frame);

      /**
       * AINFO - 记录场景切换日志
       * 输出从哪个场景切换到哪个场景
       * current_scenario_->Name(): 获取当前场景名称
       * scenario->Name(): 获取目标场景名称
       */
      AINFO << "switch scenario from " << current_scenario_->Name() << " to "
            << scenario->Name();

      /**
       * 更新当前场景指针
       * current_scenario_ = scenario
       *   - 智能指针赋值，引用计数会自动管理
       */
      current_scenario_ = scenario;

      /**
       * 重置新场景状态
       * current_scenario_->Reset()
       *   - Reset: 重置场景内部状态
       */
      current_scenario_->Reset();

      /**
       * 调用新场景的Enter方法
       * current_scenario_->Enter(frame)
       *   - Enter: 场景进入时调用的方法
       *   - 用于初始化状态、分配资源等
       */
      current_scenario_->Enter(frame);

      /**
       * 切换完成，直接返回
       * 注意：一次Update只切换一次场景
       */
      return;
    }
  }
}

/**
 * @brief 重置场景管理器
 *
 * 将当前场景重置为默认场景（车道跟随）。
 * 通常在规划出错或需要完全重启时调用。
 *
 * @param frame 规划帧数据
 *
 * 语法说明：
 * - if (current_scenario_): 检查智能指针是否有效
 *   - shared_ptr的operator bool()检查是否拥有对象
 */
void ScenarioManager::Reset(Frame* frame) {
  /**
   * 检查当前场景是否存在
   * if (current_scenario_) 等价于 if (current_scenario_ != nullptr)
   */
  if (current_scenario_) {
    /**
     * 调用当前场景的Exit方法
     * 清理当前场景的状态
     */
    current_scenario_->Exit(frame);
  }

  /**
   * AINFO - 记录重置日志
   */
  AINFO << "Reset to default scenario:" << default_scenario_type_->Name();

  /**
   * 重置默认场景
   * default_scenario_type_->Reset()
   */
  default_scenario_type_->Reset();

  /**
   * 设置当前场景为默认场景
   * current_scenario_ = default_scenario_type_
   */
  current_scenario_ = default_scenario_type_;
}

}  // namespace planning
}  // namespace apollo
