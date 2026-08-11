/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
 * @file dependency_injector.h
 * @brief 依赖注入器头文件
 *
 * 功能说明：
 * 提供规划模块中各种共享服务的统一访问点
 * 实现依赖注入模式，解耦各组件间的依赖关系
 *
 * 设计模式：
 * - 依赖注入（Dependency Injection）：通过构造函数或setter注入依赖
 * - 服务定位器（Service Locator）：提供全局访问点
 * - 单例模式（各服务本身）：各服务在规划过程中唯一存在
 */

#pragma once

#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/planning_base/common/ego_info.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/history.h"
#include "modules/planning/planning_base/common/learning_based_data.h"
#include "modules/planning/planning_base/common/planning_context.h"

namespace apollo {
/**
 * @brief Apollo最外层命名空间
 */
namespace planning {

/**
 * @class DependencyInjector
 * @brief 依赖注入器类
 *
 * 功能说明：
 * 规划模块的依赖注入容器
 * 集中管理所有规划过程中需要共享的服务对象
 * 各组件通过注入器获取依赖，而非直接创建或持有依赖
 *
 * 设计目的：
 * 1. 解耦组件依赖：各组件不需要知道依赖的具体实现
 * 2. 统一生命周期管理：所有服务由注入器统一管理
 * 3. 方便单元测试：可以注入mock对象进行测试
 * 4. 避免循环依赖：通过注入器中转依赖关系
 *
 * 包含的服务：
 * - PlanningContext：规划上下文（跨帧状态）
 * - FrameHistory：历史帧记录
 * - History：历史规划结果
 * - EgoInfo：自车信息
 * - VehicleStateProvider：车辆状态提供者
 * - LearningBasedData：基于学习的数据
 *
 * C++语法说明：
 * - class：类声明关键字
 * - public/private：访问限定符
 * - = default：使用编译器默认生成的函数实现
 */
class DependencyInjector {
 public:
  /**
   * @brief 默认构造函数
   *
   * 功能说明：
   * 使用编译器生成的默认构造函数
   * 简单的空实现，让编译器生成
   *
   * C++语法说明：
   * - = default：
   *   显式请求编译器生成默认实现
   *   等价于空函数体{}
   *   但更明确，且保留了 defaulted 函数的特殊属性
   */
  DependencyInjector() = default;

  /**
   * @brief 析构函数
   *
   * 功能说明：
   * 使用编译器生成的默认析构函数
   * 负责清理成员对象
   *
   * C++语法说明：
   * - ~DependencyInjector()：
   *   析构函数命名规则：~ + 类名
   *   在对象生命周期结束时自动调用
   * - virtual：
   *   此处未加virtual，意味着不期望被继承
   *   如果需要被继承，应设为virtual
   */
  ~DependencyInjector() = default;

  /**
   * @brief 获取规划上下文
   *
   * @return PlanningContext* 指向规划上下文的指针
   *
   * 功能说明：
   * 返回规划上下文的指针，用于访问跨帧的规划状态
   * PlanningContext存储规划过程中的持久化状态
   *
   * C++语法说明：
   * - PlanningContext*：
   *   返回原始指针
   *   允许调用者修改对象内容
   * - return &planning_context_：
   *   返回成员变量的地址
   *   & 是取地址运算符
   */
  PlanningContext* planning_context() { return &planning_context_; }

  /**
   * @brief 获取帧历史
   *
   * @return FrameHistory* 指向帧历史的指针
   *
   * 功能说明：
   * 返回帧历史的指针，用于访问历史帧数据
   * FrameHistory记录了最近多帧的规划结果
   */
  FrameHistory* frame_history() { return &frame_history_; }

  /**
   * @brief 获取历史记录
   *
   * @return History* 指向历史记录的指针
   *
   * 功能说明：
   * 返回历史记录的指针
   * History记录了更长期的历史规划数据
   */
  History* history() { return &history_; }

  /**
   * @brief 获取自车信息
   *
   * @return EgoInfo* 指向自车信息的指针
   *
   * 功能说明：
   * 返回自车信息的指针
   * EgoInfo包含当前车辆的所有状态信息
   */
  EgoInfo* ego_info() { return &ego_info_; }

  /**
   * @brief 获取车辆状态提供者
   *
   * @return apollo::common::VehicleStateProvider* 指向车辆状态提供者的指针
   *
   * 功能说明：
   * 返回车辆状态提供者的指针
   * VehicleStateProvider从底层获取实时车辆状态
   *
   * C++语法说明：
   * - apollo::common::VehicleStateProvider：
   *   使用完整命名空间限定的类型名
   *   VehicleStateProvider定义在common子命名空间中
   */
  apollo::common::VehicleStateProvider* vehicle_state() {
    return &vehicle_state_;
  }

  /**
   * @brief 获取基于学习的数据
   *
   * @return LearningBasedData* 指向基于学习的数据的指针
   *
   * 功能说明：
   * 返回基于学习的数据的指针
   * LearningBasedData存储机器学习模型的预测结果
   */
  LearningBasedData* learning_based_data() { return &learning_based_data_; }

 private:
  /**
   * @brief 规划上下文成员
   *
   * PlanningContext：
   * - Protobuf消息类型的包装类
   * - 存储跨帧的规划状态
   * - 例如：上一次的决策结果、场景状态等
   *
   * C++语法说明：
   * - private:
   *   私有成员访问限定符
   *   只有类内部成员函数可以访问
   *   外部代码必须通过公有的getter方法访问
   */
  PlanningContext planning_context_;

  /**
   * @brief 帧历史成员
   *
   * FrameHistory：
   * - IndexedQueue实现的历史帧存储
   * - 记录最近N帧的规划结果
   * - 用于轨迹拼接和历史参考
   */
  FrameHistory frame_history_;

  /**
   * @brief 历史记录成员
   *
   * History：
   * - 更长期的历史规划记录
   * - 可以追溯更早的规划决策
   */
  History history_;

  /**
   * @brief 自车信息成员
   *
   * EgoInfo：
   * - 存储自车的实时状态
   * - 包括位置、速度、加速度、朝向等
   */
  EgoInfo ego_info_;

  /**
   * @brief 车辆状态提供者成员
   *
   * apollo::common::VehicleStateProvider：
   * - 从CAN总线或定位系统获取车辆状态
   * - 提供统一的车辆状态访问接口
   *
   * C++语法说明：
   * - apollo::common::VehicleStateProvider：
   *   外部定义的类，作为成员变量使用
   *   使用完整命名空间避免歧义
   */
  apollo::common::VehicleStateProvider vehicle_state_;

  /**
   * @brief 基于学习的数据成员
   *
   * LearningBasedData：
   * - 存储来自机器学习模型的数据
   * - 例如：障碍物预测、轨迹预测等
   */
  LearningBasedData learning_based_data_;
};

}  // namespace planning
}  // namespace apollo
