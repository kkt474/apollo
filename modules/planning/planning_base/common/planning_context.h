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
 * @file planning_context.h
 * @brief 规划上下文头文件
 *
 * 功能说明：
 * 定义了规划模块的运行时上下文（PlanningContext）
 * 该上下文在多个规划帧之间持久化存储
 * 用于保存跨帧的状态信息
 *
 * 核心概念：
 * - PlanningContext：规划上下文，存储规划模块的运行时状态
 * - PlanningStatus：规划状态，包含各种交通规则的运行时状态
 * - Frame：规划帧，单个规划周期的数据容器
 * - 跨帧状态：需要在多个规划周期之间保持的状态
 *
 * 设计原则：
 * - 所有状态信息应放在PlanningStatus中，便于维护
 * - 不要在此层级创建新的结构体
 * - 通过const/non-const方法提供安全的读写访问
 *
 * C++语法说明：
 * - #pragma once：头文件保护，避免重复包含
 * - class xxx = default：使用默认实现
 * - const T&：常量引用，只读访问
 * - T*：裸指针，非const访问
 * - {}：空函数体
 **/

/**
 * @brief 头文件保护
 *
 * #pragma once：
 * 防止头文件被重复包含的预处理指令
 * 优点：简洁，跨平台
 * 缺点：不能防止非物理拷贝的重复包含
 *
 * 替代方案：传统的#ifndef/#define/#endif
 */
#pragma once

#include "modules/planning/planning_base/proto/planning_status.pb.h"
/**
 * @brief 规划状态Protobuf消息头文件
 *
 * PlanningStatus：
 * - Protobuf消息类型，定义规划模块的运行时状态
 * - 包含各种交通规则的运行时信息
 * - 如：crosswalk、yield_sign、stop_sign等的状态
 *
 * Protobuf特点：
 * - 结构化数据序列化协议
 * - 支持跨语言、跨平台
 * - 定义在.proto文件中，通过protoc编译生成
 */

#include "cyber/common/macros.h"
/**
 * @brief Cyber RT通用宏定义头文件
 *
 * 包含常用的宏定义：
 * - DISALLOW_COPY_AND_ASSIGN：禁止拷贝和赋值
 * - AINFO/ADEBUG/AERROR：日志宏
 * - CHECK_NOTNULL：空指针检查
 */

namespace apollo {
/**
 * @brief Apollo项目主命名空间
 */

namespace planning {
/**
 * @brief 规划模块命名空间
 */

/**
 * @brief 规划上下文类
 *
 * 功能说明：
 * 规划模块的运行时上下文
 * 在多个规划帧之间持久化存储
 * 保存跨帧需要的状怸信息
 *
 * 设计目的：
 * - 提供规划状态的集中管理
 * - 支持跨帧状态共享
 * - 简化状态传递
 *
 * 使用方式：
 * - 通过DependencyInjector获取单例实例
 * - 在每个Frame中访问和更新状态
 *
 * C++语法说明：
 * - class PlanningContext：类声明
 * - public/private：访问控制符
 */
class PlanningContext {
 public:
  /**
   * @brief 构造函数
   *
   * = default：
   * 使用编译器生成的默认实现
   * 等价于空函数体{}，但更明确
   *
   * 特点：
   * - 默认构造函数
   * - 不进行任何初始化
   * - planning_status_会调用其默认构造函数
   */
  PlanningContext() = default;

  /**
   * @brief 清除上下文状态
   *
   * 功能说明：
   * 重置规划上下文到初始状态
   * 通常在重新规划或出错时调用
   *
   * 实现位置：
   * 在planning_context.cc中定义
   */
  void Clear();

  /**
   * @brief 初始化上下文
   *
   * 功能说明：
   * 初始化规划上下文
   * 设置初始状态和默认值
   *
   * 实现位置：
   * 在planning_context.cc中定义
   */
  void Init();

  /**
   * @brief 获取规划状态（只读）
   *
   * @return const PlanningStatus& 规划状态的常量引用
   *
   * 功能说明：
   * 提供规划状态的只读访问
   * 用于需要读取状态但不需要修改的场景
   *
   * C++语法说明：
   * - const PlanningStatus&：
   *   - PlanningStatus：Protobuf消息类型
   *   - const：返回常量引用，不可修改
   *   - &：引用，避免拷贝
   *
   * - const PlanningStatus& planning_status() const：
   *   - 第一个const：返回类型是常量引用
   *   - 第二个const：成员函数承诺不修改对象状态
   *
   * @code
   *   const PlanningStatus& status = context.planning_status();
   *   // 可以读取status，但不能修改
   * @endcode
   */
  const PlanningStatus& planning_status() const { return planning_status_; }

  /**
   * @brief 获取规划状态（可修改）
   *
   * @return PlanningStatus* 指向规划状态的指针
   *
   * 功能说明：
   * 提供规划状态的可修改访问
   * 用于需要修改状态的场景
   *
   * 返回值：
   * - 返回裸指针而非引用
   * - 调用者负责正确使用指针
   * - 可以通过指针修改状态
   *
   * C++语法说明：
   * - PlanningStatus*：
   *   - 返回原始指针
   *   - 非const，可以修改对象
   *   - nullptr：空指针字面量
   *
   * @code
   *   PlanningStatus* status = context.mutable_planning_status();
   *   status->mutable_crosswalk()->set_xxx();
   * @endcode
   */
  PlanningStatus* mutable_planning_status() { return &planning_status_; }
  /**
   * @note 注意：
   * 虽然返回指针，但&planning_status_保证不为nullptr
   * Protobuf消息类的特性：总是有效
   */

 private:
  /**
   * @brief 私有成员变量
   *
   * protected/private区别：
   * - protected：子类可访问
   * - private：只有本类可访问
   *
   * 规划上下文只需要本类操作状态
   */

  /**
   * @brief 规划状态成员变量
   *
   * @brief PlanningStatus：
   * - Protobuf消息类型的成员变量
   * - 存储所有跨帧的规划状态
   *
   * 初始化：
   * - 默认构造
   * - PlanningStatus的默认构造函数会初始化所有字段为默认值
   *
   * 访问控制：
   * - private：只有PlanningContext类的方法可以访问
   * - 外部通过public的getter/setter访问
   *
   * 设计优点：
   * - 封装状态，统一访问入口
   * - 可以添加额外的访问逻辑
   * - 便于调试和日志
   */
  PlanningStatus planning_status_;
};

}  // namespace planning
/**
 * @brief 规划命名空间结束标记
 *
 * 注释习惯：// namespace xxx
 * 用于明确命名空间的作用域结束位置
 */

}  // namespace apollo
/**
 * @brief Apollo命名空间结束标记
 */
