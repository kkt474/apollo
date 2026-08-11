/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
 * @file stage_approaching_parking_spot.h
 * @brief 接近停车位阶段头文件
 *
 * 功能说明：
 * 定义了接近停车位阶段的类接口
 * 该阶段是自主泊车场景的第二个阶段
 *
 * 工作流程：
 * 1. 初始化阶段配置和上下文
 * 2. 设置目标停车位信息到规划帧
 * 3. 处理目的障碍物（忽略其决策）
 * 4. 在参考线上执行任务
 * 5. 检查车辆是否已停在合适位置
 * 6. 决定是否进入下一阶段（PARKING）
 *
 * 继承关系：
 * StageApproachingParkingSpot 继承自 Stage
 * Stage 是所有规划阶段的基类
 *
 * C++语法说明：
 * - class：类声明关键字
 * - public：公有继承
 * - virtual：虚函数，支持运行时多态
 * - override：重写基类虚函数
 * - std::shared_ptr：共享所有权智能指针
 * - std::string：标准库字符串
 * - #pragma once：头文件保护
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
 */
#include <memory>
#include <string>

/**
 * @brief Cyber RT插件管理器头文件
 *
 * 功能说明：
 * - cyber/plugin_manager/plugin_manager.h：
 *   Apollo Cyber RT的插件系统
 *   允许动态加载和卸载模块
 *   支持运行时扩展功能
 *
 * C++语法说明：
 * - CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN：
 *   宏，用于注册插件到插件管理器
 *   这是一个类级别的注册机制
 *   允许系统在运行时发现和加载插件
 */
#include "cyber/plugin_manager/plugin_manager.h"

/**
 * @brief Stage基类头文件
 *
 * 功能说明：
 * - modules/planning/planning_interface_base/scenario_base/stage.h：
 *   定义了Stage基类
 *   Stage是所有规划阶段的抽象基类
 *   提供阶段的通用接口和生命周期
 *
 * C++语法说明：
 * - "modules/..."：
 *   使用引号表示自定义头文件路径
 *   编译器从当前文件目录开始搜索
 *
 * 继承说明：
 * StageApproachingParkingSpot 公有继承自 Stage
 * public继承表示：
 * - 基类的public成员仍是public
 * - 基类的protected成员仍是protected
 * - 基类的private成员不可直接访问
 */
#include "modules/planning/planning_interface_base/scenario_base/stage.h"

/**
 * @brief 泊车场景头文件
 *
 * 功能说明：
 * - valet_parking_scenario.h：
 *   定义了ValetParkingContext结构
 *   ValetParkingContext包含泊车场景的上下文数据
 *   如：目标停车位ID、预停车标志等
 *
 * C++语法说明：
 * - 这是场景特定的头文件
 *   用于访问场景上下文类型
 *   StageApproachingParkingSpot是ValetParking场景的一个阶段
 */
#include "modules/planning/scenarios/valet_parking/valet_parking_scenario.h"

/**
 * @namespace apollo::planning
 * @brief Apollo规划模块命名空间
 *
 * C++语法说明：
 * - namespace apollo：
 *   Apollo最外层命名空间
 *   所有Apollo代码都在此命名空间下
 * - namespace planning：
 *   规划子命名空间
 */
namespace apollo {
namespace planning {

/**
 * @class StageApproachingParkingSpot
 * @brief 接近停车位阶段类
 *
 * 功能说明：
 * 自主泊车场景中的"接近停车位"阶段
 * 负责将车辆引导到停车位前的合适位置
 *
 * 继承关系：
 * StageApproachingParkingSpot 公有继承自 Stage
 *
 * 设计模式：
 * - 模板方法模式：基类定义算法骨架
 * - 策略模式：不同阶段有不同行为
 *
 * 生命周期：
 * 1. 构造：创建阶段对象
 * 2. 初始化：Init()加载配置
 * 3. 执行：Process()执行业务逻辑
 * 4. 重复执行Process直到完成
 *
 * C++语法说明：
 * - class StageApproachingParkingSpot :
 *   类声明，定义一个名为StageApproachingParkingSpot的类
 *
 * - public Stage：
 *   公有继承自Stage类
 *   Stage是阶段基类，定义了阶段的通用接口
 */
class StageApproachingParkingSpot : public Stage {
 public:
  /**
   * @brief 初始化阶段
   *
   * @param config 阶段管道配置，包含阶段的执行流程和参数
   * @param injector 依赖注入器，提供各种服务的访问接口
   * @param config_dir 配置文件目录，用于加载配置文件
   * @param context 场景上下文，包含跨阶段共享的数据
   * @return bool 初始化是否成功
   *
   * 功能说明：
   * 初始化阶段的内部状态和配置
   * 这是进入阶段时调用的第一个方法
   *
   * 重写说明：
   * 该方法重写了基类Stage的Init方法
   *
   * C++语法说明：
   * - bool Init(...)：
   *   返回布尔值表示成功/失败
   *
   * - const StagePipeline& config：
   *   常量引用，传入阶段配置
   *   const保证配置不会被修改
   *   引用避免拷贝大型结构
   *
   * - const std::shared_ptr<DependencyInjector>& injector：
   *   共享指针的常量引用
   *   - shared_ptr：多个所有者共享对象
   *   - const引用：既不拷贝也不获得所有权
   *
   * - const std::string& config_dir：
   *   常量引用，传入配置目录路径
   *
   * - void* context：
   *   通用指针类型，可以指向任何类型
   *   用于传递场景上下文数据
   */
  bool Init(const StagePipeline& config,
            const std::shared_ptr<DependencyInjector>& injector,
            const std::string& config_dir, void* context);

  /**
   * @brief 处理阶段执行
   *
   * @param planning_init_point 规划起始点（车辆当前位置）
   * @param frame 规划帧数据，包含当前帧的所有规划信息
   * @return StageResult 阶段执行结果
   *
   * 功能说明：
   * 这是阶段的主要执行逻辑
   * 负责任务调度、状态检查和流程控制
   *
   * 重写说明：
   * 该方法重写了基类Stage的Process方法
   * 使用override关键字显式声明
   *
   * C++语法说明：
   * - StageResult Process(...)：
   *   StageResult是阶段执行结果类型
   *   包含状态码、错误信息等
   *
   * - const common::TrajectoryPoint& planning_init_point：
   *   common::TrajectoryPoint：Apollo的轨迹点类型
   *   const&：常量引用，避免拷贝
   *
   * - Frame* frame：
   *   原始指针，用于修改规划帧数据
   *   使用指针而非引用，因为frame可能需要重新赋值
   *
   * - override：
   *   C++11关键字，显式表示重写基类虚函数
   *   编译器会检查基类是否有可重写的对应虚函数
   *   如果没有，编译错误
   *   作用：防止拼写错误、签名不匹配等
   */
  StageResult Process(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame) override;

 private:
  /**
   * @brief 检查自车是否已停止
   *
   * @param frame 规划帧数据
   * @return bool 如果自车已合适地停止返回true
   *
   * 功能说明：
   * 判断自车是否已经停在停车位前合适的位置
   *
   * 停止条件：
   * 1. 车辆速度低于最大停止速度阈值
   * 2. 车辆前边缘与停车线的距离在有效范围内
   *
   * C++语法说明：
   * - bool CheckADCStop(const Frame& frame)：
   *   private成员函数，仅类内部可调用
   *   const Frame&：常量引用，frame不会被修改
   */
  bool CheckADCStop(const Frame& frame);

  /**
   * @brief 泊车场景配置
   *
   * 功能说明：
   * 存储泊车场景的特定配置参数
   * 从ValetParkingContext中复制而来
   *
   * C++语法说明：
   * - ScenarioValetParkingConfig：
   *   泊车场景配置的Protobuf消息类型
   *   包含停车距离、速度限制等参数
   *
   * - scenario_config_：
   *   成员变量命名约定：xxx_后缀
   *   表示这是类的成员变量
   */
  ScenarioValetParkingConfig scenario_config_;
};

/**
 * @brief Cyber插件注册宏
 *
 * 功能说明：
 * 将StageApproachingParkingSpot类注册为Stage类型的插件
 * 允许系统在运行时发现和加载这个阶段
 *
 * C++语法说明：
 * - CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN：
 *   Apollo Cyber RT的插件注册宏
 *   这是一个类级别的注册机制
 *   展开后会在全局插件管理器中注册此类
 *
 * - apollo::planning::StageApproachingParkingSpot：
 *   要注册的类名（完整命名空间）
 *
 * - Stage：
 *   注册的插件基类型
 *   StageApproachingParkingSpot是Stage的派生类
 *
 * 使用方式：
 * 通过插件管理器可以动态创建StageApproachingParkingSpot实例
 * 而无需直接引用具体的类名
 * 这实现了类创建的解耦
 *
 * 示例：
 * @code
 *   auto plugin_manager = cyber::plugin_manager::PluginManager::Instance();
 *   auto stage = plugin_manager->CreateInstance<Stage>("StageApproachingParkingSpot");
 * @endcode
 */
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
    apollo::planning::StageApproachingParkingSpot, Stage)

/**
 * @namespace命名空间结束
 *
 * C++语法说明：
 * - }  // namespace planning：
 *   结束planning命名空间
 *   注释方便阅读和导航
 *
 * - }  // namespace apollo：
 *   结束apollo命名空间
 */
}  // namespace planning
}  // namespace apollo