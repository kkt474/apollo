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
 * @file open_space_pre_stop_decider.h
 * @brief 开放空间预停车决策器头文件
 *
 * 功能说明：
 * 定义了开放空间预停车决策器的类接口
 * 继承自Decider基类，是Task的派生类
 *
 * 应用场景：
 * - 自主泊车(Valet Parking)
 * - 靠边停车(Pull Over)
 *
 * 设计理念：
 * 1. 支持两种停车类型：PARKING和PULL_OVER
 * 2. 根据停车类型选择不同的处理逻辑
 * 3. 使用插件机制动态加载
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
 * @brief Protobuf配置头文件
 *
 * 功能说明：
 * - open_space_pre_stop_decider.pb.h：
 *   Protobuf编译生成的代码
 *   定义了OpenSpacePreStopDeciderConfig配置结构
 *   包含预停车决策器的所有配置参数
 *
 * C++语法说明：
 * - "modules/planning/tasks/open_space_pre_stop_decider/proto/..."：
 *   使用引号表示自定义头文件路径
 *   protobuf生成的代码通常放在proto子目录
 */
#include "modules/planning/tasks/open_space_pre_stop_decider/proto/open_space_pre_stop_decider.pb.h"

/**
 * @brief Cyber RT宏定义头文件
 *
 * 功能说明：
 * - cyber/common/macros.h：
 *   定义了Apollo常用的宏
 *   如DISALLOW_COPY_AND_ASSIGN禁用拷贝构造和赋值运算符
 *   CHECK_NOTNULL断言宏等
 *
 * C++语法说明：
 * - DISALLOW_COPY_AND_ASSIGN：
 *   宏定义，禁止拷贝构造和赋值
 *   通常用于单例模式或资源管理类
 */
#include "cyber/common/macros.h"

/**
 * @brief Cyber RT插件管理器头文件
 *
 * 功能说明：
 * - cyber/plugin_manager/plugin_manager.h：
 *   Apollo Cyber RT的插件系统
 *   允许动态加载和创建Task实例
 *
 * C++语法说明：
 * - CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN：
 *   宏，用于注册插件到插件管理器
 */
#include "cyber/plugin_manager/plugin_manager.h"

/**
 * @brief 规划帧头文件
 *
 * 功能说明：
 * - modules/planning/planning_base/common/frame.h：
 *   定义了Frame类
 *   包含当前帧的所有规划相关信息
 *
 * C++语法说明：
 * - Frame：
 *   规划帧类，包含车辆状态、参考线、障碍物等信息
 */
#include "modules/planning/planning_base/common/frame.h"

/**
 * @brief 参考线信息头文件
 *
 * 功能说明：
 * - reference_line_info.h：
 *   定义了ReferenceLineInfo类
 *   包含参考线的路径、速度限制等信息
 *
 * C++语法说明：
 * - ReferenceLineInfo：
 *   参考线信息类，用于描述一条规划路径
 */
#include "modules/planning/planning_base/common/reference_line_info.h"

/**
 * @brief 决策器基类头文件
 *
 * 功能说明：
 * - common/decider.h：
 *   定义了Decider类
 *   所有决策器继承自此类
 *
 * C++语法说明：
 * - Decider：
 *   决策器基类，提供通用接口
 *   Task继承自Decider
 */
#include "modules/planning/planning_interface_base/task_base/common/decider.h"

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
 * @class OpenSpacePreStopDecider
 * @brief 开放空间预停车决策器类
 *
 * 功能说明：
 * 在开放空间规划（如自主泊车）场景中，计算预停车位置
 * 继承自Decider类，是Task的派生类
 *
 * 继承关系：
 * OpenSpacePreStopDecider -> Decider -> Task
 *
 * 停车类型：
 * - PARKING：停车位停车
 * - PULL_OVER：靠边停车
 *
 * 设计模式：
 * - 策略模式：根据停车类型选择不同策略
 * - 插件模式：动态加载
 *
 * C++语法说明：
 * - class OpenSpacePreStopDecider :
 *   类声明，定义一个名为OpenSpacePreStopDecider的类
 *
 * - public Decider：
 *   公有继承自Decider类
 *   public继承表示：
 *   - 基类的public成员仍是public
 *   - 基类的protected成员仍是protected
 *   - 基类的private成员不可直接访问
 */
class OpenSpacePreStopDecider : public Decider {
 public:
  /**
   * @brief 初始化开放空间预停车决策器
   *
   * @param config_dir 配置文件目录
   * @param name 任务名称
   * @param injector 依赖注入器指针
   * @return bool 初始化是否成功
   *
   * 功能说明：
   * 初始化决策器的内部状态和配置
   *
   * 重写说明：
   * 该方法重写了基类Decider的Init方法
   *
   * C++语法说明：
   * - bool Init(...)：
   *   返回布尔值表示成功/失败
   *
   * - const std::string& config_dir：
   *   常量引用，传入配置文件目录路径
   *   const保证配置不会被修改
   *   引用避免拷贝大型字符串
   *
   * - const std::string& name：
   *   常量引用，传入任务名称
   *
   * - const std::shared_ptr<DependencyInjector>& injector：
   *   共享指针的常量引用
   *   - shared_ptr：多个所有者共享对象
   *   - const引用：既不拷贝也不获得所有权
   *
   * - override：
   *   C++11关键字，显式表示重写基类虚函数
   *   编译器会检查基类是否有可重写的对应虚函数
   */
  bool Init(const std::string& config_dir, const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;

 private:
  /**
   * @brief 处理预停车决策
   *
   * @param frame 规划帧数据
   * @param reference_line_info 参考线信息
   * @return apollo::common::Status 处理状态
   *
   * 功能说明：
   * 根据配置中的停车类型，选择相应的处理函数
   *
   * 重写说明：
   * 该方法重写了基类Decider的Process方法
   *
   * C++语法说明：
   * - apollo::common::Status：
   *   Apollo通用状态类，包含错误码和消息
   *
   * - Frame*：
   *   原始指针，指向规划帧
   *   使用指针因为frame需要被修改
   *
   * - ReferenceLineInfo*：
   *   参考线信息指针
   *   用于访问和修改参考线相关信息
   *
   * - override：
   *   重写基类虚函数
   */
  apollo::common::Status Process(
      Frame* frame, ReferenceLineInfo* reference_line_info) override;

  /**
   * @brief 检查停车位预停车条件
   *
   * @param frame 规划帧数据
   * @param reference_line_info 参考线信息
   * @param target_s 输出：目标s坐标
   * @return bool 检查是否成功
   *
   * 功能说明：
   * 检查停车位的预停车条件
   * 从地图路径中找到目标停车位并计算其中心点
   *
   * C++语法说明：
   * - Frame* const frame：
   *   指向常量的指针
   *   - Frame*：指针类型
   *   - const：指针指向的内容不可修改
   *   这表示函数不会通过此指针修改frame指向的内容
   *
   * - ReferenceLineInfo* const reference_line_info：
   *   同样是指向常量的指针
   *
   * - double* target_s：
   *   指针输出参数
   *   函数内通过解引用赋值输出结果
   */
  bool CheckParkingSpotPreStop(Frame* const frame,
                               ReferenceLineInfo* const reference_line_info,
                               double* target_s);

  /**
   * @brief 检查靠边停车预停车条件
   *
   * @param frame 规划帧数据
   * @param reference_line_info 参考线信息
   * @param target_s 输出：目标s坐标
   * @return bool 检查是否成功
   *
   * 功能说明：
   * 检查靠边停车的预停车条件
   * 从规划上下文中获取靠边停车目标位置
   *
   * C++语法说明：
   * - Frame* const frame：
   *   指向常量的指针
   *
   * - ReferenceLineInfo* const reference_line_info：
   *   指向常量的指针
   *
   * - double* target_s：
   *   指针输出参数
   */
  bool CheckPullOverPreStop(Frame* const frame,
                            ReferenceLineInfo* const reference_line_info,
                            double* target_s);

  /**
   * @brief 设置停车位停车围栏
   *
   * @param target_s 目标s坐标
   * @param frame 规划帧数据
   * @param reference_line_info 参考线信息
   *
   * 功能说明：
   * 计算并设置停车位的预停车围栏位置
   *
   * C++语法说明：
   * - const double target_s：
   *   const double类型，传入的目标s坐标
   *   const表示函数内不会修改这个值
   *
   * - Frame* const frame：
   *   指向常量的指针
   *
   * - ReferenceLineInfo* const reference_line_info：
   *   指向常量的指针
   */
  void SetParkingSpotStopFence(const double target_s, Frame* const frame,
                               ReferenceLineInfo* const reference_line_info);

  /**
   * @brief 设置靠边停车围栏
   *
   * @param target_s 目标s坐标
   * @param frame 规划帧数据
   * @param reference_line_info 参考线信息
   *
   * 功能说明：
   * 计算并设置靠边停车的预停车围栏位置
   *
   * C++语法说明：
   * - const double target_s：
   *   const double类型
   *
   * - Frame* const frame：
   *   指向常量的指针
   *
   * - ReferenceLineInfo* const reference_line_info：
   *   指向常量的指针
   */
  void SetPullOverStopFence(const double target_s, Frame* const frame,
                            ReferenceLineInfo* const reference_line_info);

 private:
  /**
   * @brief 预停车决策器配置
   *
   * 功能说明：
   * 存储预停车决策器的配置参数
   * 从protobuf配置文件中加载
   *
   * C++语法说明：
   * - OpenSpacePreStopDeciderConfig：
   *   Protobuf消息类型
   *   包含预停车决策器的所有配置参数
   *
   * - config_：
   *   成员变量命名约定：xxx_后缀
   *   表示这是类的成员变量
   */
  OpenSpacePreStopDeciderConfig config_;

  /**
   * @brief 开放空间预停车墙ID常量
   *
   * 功能说明：
   * 定义预停车围栏的唯一标识符
   * 用于在决策中标识这个特定的停止墙
   *
   * C++语法说明：
   * - static：
   *   静态成员，类内共享
   *   不需要创建类实例即可访问
   *
   * - constexpr：
   *   编译时常量
   *   在编译时确定值，不能在运行时修改
   *
   * - const char*：
   *   指向常量字符的指针
   *   字符数组表示字符串
   *
   * - OPEN_SPACE_STOP_ID：
   *   常量名称命名约定：全大写+下划线
   *
   * 示例：
   * - OPEN_SPACE_STOP_ID = "OPEN_SPACE_PRE_STOP"
   *   这是一个字符串字面量
   */
  static constexpr const char* OPEN_SPACE_STOP_ID = "OPEN_SPACE_PRE_STOP";
};

/**
 * @brief Cyber插件注册宏
 *
 * 功能说明：
 * 将OpenSpacePreStopDecider类注册为Task类型的插件
 * 允许系统在运行时发现和加载这个决策器
 *
 * C++语法说明：
 * - CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN：
 *   Apollo Cyber RT的插件注册宏
 *   这是一个类级别的注册机制
 *   展开后会在全局插件管理器中注册此类
 *
 * - apollo::planning::OpenSpacePreStopDecider：
 *   要注册的类名（完整命名空间）
 *
 * - Task：
 *   注册的插件基类型
 *   OpenSpacePreStopDecider是Task的派生类
 *
 * 使用方式：
 * 通过插件管理器可以动态创建OpenSpacePreStopDecider实例
 * 而无需直接引用具体的类名
 * 这实现了类创建的解耦
 */
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::OpenSpacePreStopDecider,
                                     Task)

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