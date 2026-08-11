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
 * @file scenario.h
 * @brief 场景(Scenario)基类头文件
 *
 * 功能说明：
 * 定义了Apollo规划模块中所有场景的基类
 * 场景是规划模块进行任务级决策和执行的基本单元
 *
 * 设计理念：
 * 1. 场景管理器根据当前环境状态选择合适的场景
 * 2. 每个场景由多个阶段(Stage)组成
 * 3. 场景之间可以相互转换(IsTransferable)
 * 4. 使用模板方法模式定义算法骨架，具体实现由子类完成
 *
 * 应用场景：
 * - 车道跟随(Lane Follow)
 * - 车道变更(Lane Change)
 * - 车辆借道(Lane Borrow)
 * - 自主泊车(Valet Parking)
 * - 红绿灯处理(Traffic Light)
 * - 停车让行(Yield Sign)
 * - 等等
 *
 * C++语法说明：
 * - class：类声明关键字
 * - virtual：虚函数，支持运行时多态
 * - = default：使用编译器默认实现
 * - = 0：纯虚函数，强制子类实现
 * - std::shared_ptr：共享所有权智能指针
 * - template：模板编程，支持泛型
 * - std::unordered_map：基于哈希表的键值对容器
 * - namespace：命名空间，避免命名冲突
 * - initializer_list：初始化列表
 */

/**
 * @brief 头文件保护指令
 *
 * C++语法说明：
 * - #pragma once：
 *   预处理指令，防止头文件被多次包含
 *   现代编译器支持，比#ifndef...#define...#endif更简洁
 *   优点：无需手动写保护宏，编译器自动处理
 *   注意：不是标准C++的一部分，但主流编译器都支持
 */
#pragma once

/**
 * @brief C++标准库头文件
 *
 * C++语法说明：
 * - #include <memory>：
 *   智能指针头文件
 *   提供std::shared_ptr、std::unique_ptr等智能指针
 *   智能指针可以自动管理内存，避免内存泄漏
 *
 * - #include <string>：
 *   标准库字符串类
 *   std::string是处理字符串的常用类型
 *   提供字符串拼接、查找、比较等功能
 *
 * - #include <unordered_map>：
 *   哈希表容器头文件
 *   std::unordered_map提供O(1)平均时间复杂度的查找
 *   键值对存储，适合需要快速查找的场景
 */
#include <memory>
#include <string>
#include <unordered_map>

/**
 * @brief Apollo场景配置protobuf头文件
 *
 * 功能说明：
 * - scenario_pipeline.pb.h：
 *   Protobuf编译生成的代码
 *   定义了StagePipeline等配置结构
 *   用于描述场景的阶段配置
 *
 * C++语法说明：
 * - "modules/..."：
 *   使用引号而非尖括号，表示自定义头文件
 *   编译器从当前文件目录开始搜索
 */
#include "modules/planning/planning_interface_base/scenario_base/proto/scenario_pipeline.pb.h"

/**
 * @brief Cyber RT文件系统工具头文件
 *
 * 功能说明：
 * - cyber/common/file.h：
 *   Apollo Cyber RT框架的文件操作工具
 *   提供GetProtoFromFile、LoadConfig等函数
 *   用于加载protobuf配置文件
 *
 * C++语法说明：
 * - "cyber/..."：
 *   Cyber RT框架的头文件路径约定
 *   cyber是Apollo的实时计算框架
 */
#include "cyber/common/file.h"

/**
 * @brief 依赖注入器头文件
 *
 * 功能说明：
 * - dependency_injector.h：
 *   依赖注入器类
 *   集中管理规划模块所需的各种服务
 *   解耦组件依赖，便于单元测试
 */
#include "modules/planning/planning_base/common/dependency_injector.h"

/**
 * @brief 场景处理结果头文件
 *
 * 功能说明：
 * - process_result.h：
 *   定义了ScenarioResult结构
 *   描述场景执行的结果状态
 *   包括成功、失败、继续执行等状态
 */
#include "modules/planning/planning_interface_base/scenario_base/process_result.h"

/**
 * @namespace apollo::common
 * @brief Apollo通用数据类型命名空间
 *
 * C++语法说明：
 * - namespace apollo：
 *   Apollo最外层命名空间
 *   所有Apollo代码都在此命名空间下
 * - namespace common：
 *   通用数据类型的子命名空间
 *
 * 前向声明：
 * 这里只声明了TrajectoryPoint类名，没有包含完整定义
 * 目的是让后续代码知道这个类型存在
 */
namespace apollo {
namespace common {
class TrajectoryPoint;  /**< 前向声明：轨迹点类 */
}  // namespace common
}  // namespace apollo

/**
 * @namespace apollo::planning
 * @brief Apollo规划模块命名空间
 *
 * C++语法说明：
 * - 所有规划相关的类都在此命名空间
 * - 命名空间可以嵌套使用
 */
namespace apollo {
namespace planning {

/**
 * @class Frame
 * @brief 规划帧类前向声明
 *
 * C++语法说明：
 * - class Frame：
 *   前向声明，让编译器知道Frame是一个类名
 *   在实际使用时会包含完整定义
 *   作用：避免循环包含头文件
 */
class Frame;

/**
 * @struct ScenarioContext
 * @brief 场景上下文数据结构
 *
 * 功能说明：
 * 定义场景之间共享的上下文信息
 * 存储当前场景需要传递给其他场景的数据
 *
 * 设计目的：
 * - 跨场景数据传递
 * - 保存场景执行状态
 * - 子类可以扩展此结构添加特定数据
 *
 * C++语法说明：
 * - struct vs class：
 *   struct默认访问控制是public
 *   class默认访问控制是private
 *   这里使用struct表示这是一个纯数据容器
 */
struct ScenarioContext {
 public:
  /**
   * @brief 默认构造函数
   *
   * C++语法说明：
   * - ScenarioContext() {}：
   *   空函数体，使用默认实现
   *   花括号{}表示空的构造函数体
   */
  ScenarioContext() {}
};

/**
 * @class Stage
 * @brief 阶段类前向声明
 *
 * 功能说明：
 * Stage是场景的子单元
 * 每个场景由多个Stage组成
 *
 * C++语法说明：
 * - 只声明类名，不包含完整定义
 * - 前向声明足以用于创建智能指针
 */
class Stage;

/**
 * @class DependencyInjector
 * @brief 依赖注入器类前向声明
 *
 * 功能说明：
 * 依赖注入器用于解耦组件依赖
 * 提供各种服务的访问接口
 */
class DependencyInjector;

/**
 * @class Scenario
 * @brief 场景基类
 *
 * 功能说明：
 * 所有具体场景的基类
 * 定义了场景的通用接口和生命周期
 *
 * 设计模式：
 * - 模板方法模式：定义算法骨架
 * - 工厂模式：CreateStage创建阶段
 * - 策略模式：不同场景有不同行为
 *
 * 生命周期：
 * 1. 构造：创建场景对象
 * 2. 初始化：Init()加载配置
 * 3. 进入：Enter()准备资源
 * 4. 执行：Process()执行业务逻辑
 * 5. 退出：Exit()清理资源
 *
 * C++语法说明：
 * - class Scenario：
 *   类声明，定义一个名为Scenario的类
 * - public：
 *   公有访问控制符，后续成员可被外部访问
 */
class Scenario {
 public:
  /**
   * @brief 默认构造函数
   *
   * 功能说明：
   * 创建场景对象
   * 不进行任何初始化，初始化在Init()中完成
   *
   * C++语法说明：
   * - Scenario()：
   *   无参构造函数，与类名同名
   *   没有= default或= delete时，使用编译器隐式生成
   */
  Scenario();

  /**
   * @brief 虚析构函数
   *
   * 功能说明：
   * 销毁场景对象
   * 必须为虚函数以支持多态删除
   *
   * C++语法说明：
   * - virtual ~Scenario()：
   *   virtual：虚函数关键字
   *   析构函数设置为virtual使得通过基类指针删除派生类对象时
   *   能正确调用派生类的析构函数
   *
   * - = default：
   *   使用编译器默认生成的析构函数实现
   *   相当于~Scenario() {}
   *   但= default更明确，且保留了 defaulted 函数的特殊属性
   *
   * 为什么析构函数需要virtual？
   * 当通过基类指针删除派生类对象时：
   * - 如果析构函数不是virtual，只会调用基类析构函数
   * - 如果析构函数是virtual，会先调用派生类析构函数，再调用基类析构函数
   */
  virtual ~Scenario() = default;

  /**
   * @brief 初始化场景
   *
   * @param injector 依赖注入器指针，用于访问各种服务
   * @param name 场景名称，用于标识和日志输出
   * @return bool 初始化是否成功
   *
   * 功能说明：
   * 初始化场景对象，包括：
   * 1. 保存依赖注入器
   * 2. 保存场景名称
   * 3. 加载场景配置
   * 4. 创建阶段对象
   *
   * C++语法说明：
   * - virtual bool Init(...)：
   *   virtual：虚函数，子类可以重写
   *   bool：返回布尔值表示成功/失败
   *
   * - std::shared_ptr<DependencyInjector> injector：
   *   shared_ptr：共享所有权智能指针
   *   - 多个对象可以共享同一个指针
   *   - 最后一个持有者负责销毁对象
   *   - 相比unique_ptr更灵活，但有额外开销
   *
   * - const std::string& name：
   *   const：输入参数不可修改
   *   std::string&：字符串引用，避免拷贝
   *   组合起来是常量引用，效率和安全性兼备
   */
  virtual bool Init(std::shared_ptr<DependencyInjector> injector,
                    const std::string& name);

  /**
   * @brief 获取场景上下文
   *
   * @return ScenarioContext* 指向场景上下文的指针
   *
   * 功能说明：
   * 返回当前场景的上下文数据
   * 子类需要重写此方法返回具体的上下文类型
   *
   * C++语法说明：
   * - virtual ScenarioContext* GetContext() = 0：
   *   = 0表示这是纯虚函数
   *   纯虚函数没有实现，要求子类必须重写
   *   包含纯虚函数的类是抽象类，不能直接实例化
   *
   * - ScenarioContext*：
   *   返回原始指针
   *   指针可以被delete，但通常由调用者管理生命周期
   */
  virtual ScenarioContext* GetContext() = 0;

  /**
   * @brief 判断是否可以转换到当前场景
   *
   * @param other_scenario 当前场景指针
   * @param frame 规划帧数据
   * @return bool 是否可以转换
   *
   * 功能说明：
   * 场景管理器的核心接口之一
   * 判断是否应该从other_scenario转换到当前场景
   *
   * 判断条件（具体由子类实现）：
   * - 当前场景的前置条件是否满足
   * - 目标停车位是否存在
   * - 道路条件是否适合
   * - 传感器数据是否有效
   *
   * 默认实现：
   * - 返回false，表示默认不支持场景转换
   * - 子类需要重写实现具体的转换逻辑
   *
   * C++语法说明：
   * - const Scenario* other_scenario：
   *   const：指针指向的内容不可修改
   *   Scenario*：指向Scenario类的指针
   *
   * - const Frame& frame：
   *   const：引用不可修改
   *   Frame&：Frame类的引用，避免拷贝
   *
   * - { return false; }：
   *   花括号内是默认实现
   *   虽然是虚函数，但提供了默认实现
   *   子类可以不重写使用默认行为
   */
  virtual bool IsTransferable(const Scenario* other_scenario,
                              const Frame& frame) {
    return false;
  }

  /**
   * @brief 处理场景执行
   *
   * @param planning_init_point 规划起始点（车辆当前位置）
   * @param frame 规划帧数据
   * @return ScenarioResult 场景执行结果
   *
   * 功能说明：
   * 场景的核心业务逻辑
   * 决定当前场景如何执行规划任务
   *
   * 执行流程：
   * 1. 检查是否需要场景转换
   * 2. 执行当前阶段
   * 3. 根据阶段返回值决定后续操作
   *
   * C++语法说明：
   * - virtual ScenarioResult Process(...)：
   *   virtual：虚函数，子类可以重写
   *   ScenarioResult：返回场景执行结果类型
   *
   * - const common::TrajectoryPoint& planning_init_point：
   *   common::TrajectoryPoint：完整类型名
   *   const&：常量引用，避免拷贝
   */
  virtual ScenarioResult Process(
      const common::TrajectoryPoint& planning_init_point, Frame* frame);

  /**
   * @brief 退出场景
   *
   * @param frame 规划帧数据
   * @return bool 退出是否成功
   *
   * 功能说明：
   * 场景退出时的清理工作
   * 如：保存状态、释放资源等
   *
   * 默认实现：
   * - 返回true，表示默认总是可以退出
   *
   * C++语法说明：
   * - virtual bool Exit(Frame* frame) { return true; }：
   *   虚函数带有默认实现
   *   子类可以重写，也可以使用默认实现
   */
  virtual bool Exit(Frame* frame) { return true; }

  /**
   * @brief 进入场景
   *
   * @param frame 规划帧数据
   * @return bool 进入是否成功
   *
   * 功能说明：
   * 场景进入时的准备工作
   * 如：初始化状态、加载数据等
   *
   * 默认实现：
   * - 返回true，表示默认总是可以进入
   */
  virtual bool Enter(Frame* frame) { return true; }

  /**
   * @brief 创建阶段对象
   *
   * @param stage_pipeline 阶段配置
   * @return std::shared_ptr<Stage> 创建的阶段智能指针
   *
   * 功能说明：
   * 根据配置创建具体的阶段对象
   * 使用工厂模式实现阶段对象的创建
   *
   * C++语法说明：
   * - std::shared_ptr<Stage>：
   *   返回智能指针，管理Stage对象的生命周期
   *   避免了手动delete
   *
   * - const StagePipeline& stage_pipeline：
   *   阶段的配置信息
   *   包含阶段的类型、参数等
   */
  std::shared_ptr<Stage> CreateStage(const StagePipeline& stage_pipeline);

  /**
   * @brief 获取场景状态
   *
   * @return const ScenarioStatusType& 场景状态的常量引用
   *
   * 功能说明：
   * 返回当前场景的执行状态
   *
   * C++语法说明：
   * - const ScenarioStatusType&：
   *   返回常量引用，避免拷贝
   *   const保证返回的状态不可修改
   *
   * - scenario_result_.GetScenarioStatus()：
   *   调用成员对象的成员函数
   *   scenario_result_是ScenarioResult类型
   */
  const ScenarioStatusType& GetStatus() const {
    return scenario_result_.GetScenarioStatus();
  }

  /**
   * @brief 获取当前阶段名称
   *
   * @return const std::string 当前阶段名称
   */
  const std::string GetStage() const;

  /**
   * @brief 获取场景消息
   *
   * @return const std::string& 消息的常量引用
   *
   * 功能说明：
   * 返回场景执行过程中的调试信息或错误消息
   *
   * C++语法说明：
   * - const std::string&：
   *   返回常量引用，避免拷贝
   */
  const std::string& GetMsg() const { return msg_; }

  /**
   * @brief 获取场景名称
   *
   * @return const std::string& 场景名称的常量引用
   */
  const std::string& Name() const { return name_; }

  /**
   * @brief 重置场景
   *
   * 功能说明：
   * 在进入场景前重置所有状态
   * 清空当前阶段、状态消息等
   */
  void Reset();

 protected:
  /**
   * @brief 加载配置模板方法
   *
   * @tparam T 配置类型模板参数
   * @param config 输出：配置对象指针
   * @return bool 加载是否成功
   *
   * 功能说明：
   * 通用的配置文件加载方法
   * 使用模板支持不同类型的配置
   *
   * C++语法说明：
   * - template <typename T>：
   *   函数模板声明
   *   T是泛型类型参数，具体类型在调用时确定
   *
   * - template <typename T> bool LoadConfig(T* config)：
   *   模板函数定义
   *   可以接受任何类型的配置指针
   */
  template <typename T>
  bool LoadConfig(T* config);

  /**
   * @brief 场景执行结果
   *
   * C++语法说明：
   * - ScenarioResult：
   *   场景结果结构体类型
   *   包含状态码、消息等信息
   */
  ScenarioResult scenario_result_;

  /**
   * @brief 当前阶段智能指针
   *
   * 功能说明：
   * 指向当前正在执行的阶段
   *
   * C++语法说明：
   * - std::shared_ptr<Stage>：
   *   共享指针，多个对象可以共享同一个阶段
   *   最后一个持有者负责销毁
   */
  std::shared_ptr<Stage> current_stage_;

  /**
   * @brief 阶段配置映射表
   *
   * 功能说明：
   * 存储场景中所有阶段的配置
   * 使用阶段名称作为键快速查找
   *
   * C++语法说明：
   * - std::unordered_map<std::string, const StagePipeline*>：
   *   unordered_map：基于哈希表的容器
   *   - 键：std::string（阶段名称）
   *   - 值：const StagePipeline*（阶段配置指针）
   *   - const StagePipeline*：指向常量配置的指针
   *
   * 优点：
   * - O(1)平均查找时间复杂度
   * - 适合需要频繁查找的配置场景
   */
  std::unordered_map<std::string, const StagePipeline*> stage_pipeline_map_;

  /**
   * @brief 调试消息
   *
   * 功能说明：
   * 存储场景执行过程中的调试或错误信息
   * 可通过GetMsg()获取
   */
  std::string msg_;  // debug msg

  /**
   * @brief 依赖注入器智能指针
   *
   * 功能说明：
   * 指向依赖注入器的共享指针
   * 用于访问规划模块所需的各种服务
   */
  std::shared_ptr<DependencyInjector> injector_;

  /**
   * @brief 配置文件路径
   *
   * 功能说明：
   * 当前场景的配置文件路径
   */
  std::string config_path_;

  /**
   * @brief 配置目录路径
   *
   * 功能说明：
   * 当前场景的配置目录
   * 用于查找配置文件
   */
  std::string config_dir_;

  /**
   * @brief 场景名称
   *
   * 功能说明：
   * 当前场景的名称标识
   * 用于日志输出和调试
   */
  std::string name_;

  /**
   * @brief 场景管道配置
   *
   * 功能说明：
   * Protobuf消息类型
   * 包含场景的完整配置信息
   */
  ScenarioPipeline scenario_pipeline_config_;
};

/**
 * @brief 加载配置模板函数实现
 *
 * @tparam T 配置类型
 * @param config 输出：配置对象指针
 * @return bool 加载是否成功
 *
 * 功能说明：
 * 模板函数的实现
 * 使用Apollo的文件工具加载配置
 *
 * C++语法说明：
 * - template <typename T> bool Scenario::LoadConfig(T* config)：
 *   模板函数定义，需要在头文件中实现
 *   ::作用域解析符表示这是Scenario类的成员
 *
 * - apollo::cyber::common::LoadConfig<T>(...)：
 *   Apollo的通用配置加载函数
 *   - cyber::common::：完整命名空间路径
 *   - LoadConfig<T>：模板函数，T是类型参数
 *
 * - config_path_：
 *   类的成员变量，存储配置文件路径
 *   在Init()中设置
 */
template <typename T>
bool Scenario::LoadConfig(T* config) {
  return apollo::cyber::common::LoadConfig<T>(config_path_, config);
}

/**
 * @namespace命名空间结束
 *
 * C++语法说明：
 * - }  // namespace planning：
 *   结束planning命名空间
 *   注释方便阅读
 *
 * - }  // namespace apollo：
 *   结束apollo命名空间
 */
}  // namespace planning
}  // namespace apollo