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
 ******************************************************************************/

/**
 * @file control_component.h
 *
 * @brief 控制组件头文件
 *
 * 功能说明：
 * 本头文件声明了ControlComponent类
 * 是Apollo控制模块的核心组件
 * 继承自TimerComponent，实现定时轮询机制
 *
 * 主要功能：
 * 1. 订阅底盘、定位、规划等话题数据
 * 2. 检查输入数据有效性和时间戳
 * 3. 调用控制算法计算控制命令
 * 4. 发布控制命令到下游执行器
 *
 * 类的继承关系：
 * ControlComponent -> TimerComponent -> ComponentBase
 * TimerComponent是Cyber RT的定时组件基类
 *
 * 数据流：
 * Localization(定位) + Chassis(底盘) + Trajectory(轨迹) -> ControlComponent -> ControlCommand(控制命令)
 *
 * C++语法说明：
 * - #pragma once：预处理器指令，防止头文件被重复包含
 * - class：类声明，用户定义类型
 * - final：限制类不能被继承
 * - public：公有继承
 * - virtual/override：C++多态支持
 * - std::shared_ptr：智能指针，引用计数管理对象生命周期
 * - std::mutex：互斥锁，用于线程同步
 * - friend class：友元类声明
 **/

#pragma once

/**
 * @brief 标准库智能指针和字符串头文件
 *
 * C++语法说明：
 * - <memory>：提供std::shared_ptr、std::unique_ptr等智能指针
 * - <string>：提供std::string字符串类
 */
#include <memory>
#include <string>

/**
 * @brief Apollo底盘消息
 *
 * C++语法说明：
 * - chassis.pb.h：protobuf生成的头文件
 * - 定义了Chassis消息类型
 *   包含车速、档位、方向盘角度等车辆状态信息
 */
#include "modules/common_msgs/chassis_msgs/chassis.pb.h"

/**
 * @brief Apollo控制命令消息
 *
 * C++语法说明：
 * - control_cmd.pb.h：控制命令protobuf消息
 * - ControlCommand：控制命令消息类型
 *   包含油门、刹车、转向等控制量
 */
#include "modules/common_msgs/control_msgs/control_cmd.pb.h"

/**
 * @brief Apollo控制交互消息
 *
 * C++语法说明：
 * - control_interactive_msg.pb.h：
 *   控制交互消息，用于控制模块与其他模块的信息交换
 */
#include "modules/common_msgs/control_msgs/control_interactive_msg.pb.h"

/**
 * @brief Apollo Pad消息
 *
 * C++语法说明：
 * - pad_msg.pb.h：
 *   Pad消息，用于人工操作指令
 *   如启动、停车、切换模式等
 */
#include "modules/common_msgs/control_msgs/pad_msg.pb.h"

/**
 * @brief Apollo外部命令状态消息
 *
 * C++语法说明：
 * - command_status.pb.h：
 *   外部命令的状态反馈消息
 */
#include "modules/common_msgs/external_command_msgs/command_status.pb.h"

/**
 * @brief Apollo定位消息
 *
 * C++语法说明：
 * - localization.pb.h：
 *   定位估计消息
 *   包含车辆当前位置、姿态、速度等信息
 */
#include "modules/common_msgs/localization_msgs/localization.pb.h"

/**
 * @brief Apollo规划消息
 *
 * C++语法说明：
 * - planning.pb.h：
 *   规划轨迹消息
 *   包含ADCTrajectory（自动驾驶轨迹）
 */
#include "modules/common_msgs/planning_msgs/planning.pb.h"

/**
 * @brief 控制组件预处理配置
 *
 * C++语法说明：
 * - preprocessor.pb.h：
 *   预处理模块的配置消息
 */
#include "modules/control/control_component/proto/preprocessor.pb.h"

/**
 * @brief Cyber RT类加载器
 *
 * C++语法说明：
 * - cyber/class_loader/class_loader.h：
 *   Cyber RT的类加载器
 *   支持运行时动态加载组件
 */
#include "cyber/class_loader/class_loader.h"

/**
 * @brief Cyber RT定时组件
 *
 * C++语法说明：
 * - cyber/component/timer_component.h：
 *   TimerComponent是定时组件基类
 *   提供定时循环调用Proc()的能力
 *   ControlComponent继承自此类实现周期性控制
 */
#include "cyber/component/timer_component.h"

/**
 * @brief Cyber RT时间系统
 *
 * C++语法说明：
 * - cyber/time/time.h：
 *   提供时间相关的类和函数
 */
#include "cyber/time/time.h"

/**
 * @brief Apollo监控日志缓冲区
 *
 * C++语法说明：
 * - MonitorLogBuffer：
 *   监控日志缓冲区
 *   用于记录模块运行状态和错误
 */
#include "modules/common/monitor_log/monitor_log_buffer.h"

/**
 * @brief Apollo通用工具函数
 *
 * C++语法说明：
 * - util.h：
 *   提供通用工具函数
 *   如FillHeader填充消息头
 */
#include "modules/common/util/util.h"

/**
 * @brief 依赖注入器
 *
 * C++语法说明：
 * - dependency_injector.h：
 *   DependencyInjector类
 *   用于解耦组件依赖
 *   提供各模块间的数据共享
 */
#include "modules/control/control_component/controller_task_base/common/dependency_injector.h"

/**
 * @brief 控制任务代理
 *
 * C++语法说明：
 * - control_task_agent.h：
 *   ControlTaskAgent类
 *   管理控制器的执行
 */
#include "modules/control/control_component/controller_task_base/control_task_agent.h"

/**
 * @brief 预处理子模块
 *
 * C++语法说明：
 * - preprocessor_submodule.h：
 *   预处理子模块头文件
 */
#include "modules/control/control_component/submodules/preprocessor_submodule.h"

namespace apollo {
/**
 * @brief Apollo主命名空间
 */
namespace control {

/**
 * @class ControlComponent
 * @brief 控制组件类
 *
 * 功能说明：
 * ControlComponent是控制模块的核心组件
 * 继承自apollo::cyber::TimerComponent
 * 实现定时轮询的控制循环
 *
 * 主要职责：
 * 1. 订阅并处理定位、底盘、规划轨迹等消息
 * 2. 检查输入数据有效性和时间戳
 * 3. 调用控制器计算控制命令
 * 4. 发布控制命令到底盘执行
 *
 * 使用场景：
 * - 自动驾驶系统的控制模块
 * - 接收规划轨迹，输出车辆控制命令
 * - 紧急停止检测和保护
 *
 * C++语法说明：
 * - class ControlComponent final : public apollo::cyber::TimerComponent：
 *   final关键字限制此类不能被继承
 *   公有继承自TimerComponent
 *
 * - friend class ControlTestBase：
 *   友元类声明
 *   ControlTestBase可以访问ControlComponent的私有成员
 *   用于测试目的
 */
class ControlComponent final : public apollo::cyber::TimerComponent {
  /**
   * @brief 友元类声明
   *
   * C++语法说明：
   * - friend class：
   *   友元关键字，允许指定类访问本类的私有成员
   *   友元关系是单向的，不传递
   */
  friend class ControlTestBase;

 public:
  /**
   * @brief 默认构造函数
   *
   * 功能说明：
   * 初始化控制组件
   * 设置监控日志缓冲区
   */
  ControlComponent();

  /**
   * @brief 初始化函数
   *
   * @return bool 初始化成功返回true
   *
   * 功能说明：
   * 组件初始化入口
   * 执行以下初始化操作：
   * 1. 创建依赖注入器
   * 2. 加载控制流程配置
   * 3. 创建话题读者和作者
   * 4. 等待车辆状态就绪
   *
   * C++语法说明：
   * - bool Init() override：
   *   重写基类的Init虚函数
   *   override关键字确保正确重写
   */
  bool Init() override;

  /**
   * @brief 处理函数（主循环）
   *
   * @return bool 处理成功返回true
   *
   * 功能说明：
   * 控制组件的主处理循环
   * 被TimerComponent定时调度
   * 实现周期性控制
   *
   * C++语法说明：
   * - bool Proc() override：
   *   重写基类的Proc虚函数
   *   Proc是TimerComponent的主入口
   */
  bool Proc() override;

 private:
  /**
   * @brief Pad消息回调
   *
   * @param pad Pad消息的共享指针
   *
   * 功能说明：
   * 处理接收到的Pad消息（人工操作指令）
   *
   * C++语法说明：
   * - const std::shared_ptr<PadMessage> &pad：
   *   shared_ptr：智能指针，引用计数管理对象生命周期
   *   const引用避免拷贝
   *   PadMessage是protobuf消息类型
   */
  void OnPad(const std::shared_ptr<PadMessage> &pad);

  /**
   * @brief 底盘消息回调
   *
   * @param chassis 底盘消息的共享指针
   *
   * 功能说明：
   * 处理接收到的底盘数据
   * 更新最新底盘状态
   */
  void OnChassis(const std::shared_ptr<apollo::canbus::Chassis> &chassis);

  /**
   * @brief 规划轨迹回调
   *
   * @param trajectory 轨迹消息的共享指针
   *
   * 功能说明：
   * 处理接收到的规划轨迹数据
   * 更新最新轨迹
   */
  void OnPlanning(
      const std::shared_ptr<apollo::planning::ADCTrajectory> &trajectory);

  /**
   * @brief 规划命令状态回调
   *
   * @param planning_command_status 规划命令状态的共享指针
   *
   * 功能说明：
   * 处理接收到的规划命令状态
   */
  void OnPlanningCommandStatus(
      const std::shared_ptr<external_command::CommandStatus>
          &planning_command_status);

  /**
   * @brief 定位消息回调
   *
   * @param localization 定位消息的共享指针
   *
   * 功能说明：
   * 处理接收到的定位数据
   * 更新最新定位状态
   */
  void OnLocalization(
      const std::shared_ptr<apollo::localization::LocalizationEstimate>
          &localization);

  /**
   * @brief 监控消息回调
   *
   * @param monitor_message 监控消息引用
   *
   * 功能说明：
   * 处理监控消息
   * 如果有FATAL级别消息，设置紧急停止标志
   *
   * C++语法说明：
   * - const apollo::common::monitor::MonitorMessage &：
   *   常量引用输入参数
   *   监控消息不需要拷贝
   */
  void OnMonitor(
      const apollo::common::monitor::MonitorMessage &monitor_message);

  /**
   * @brief 生产控制命令
   *
   * @param control_command 输出参数，生成的控制命令
   * @return common::Status 处理状态
   *
   * 功能说明：
   * 控制组件的核心函数
   * 完整的控制流程：
   * 1. 检查输入数据有效性
   * 2. 检查时间戳
   * 3. 检查紧急停止条件
   * 4. 调用控制器计算控制命令
   * 5. 处理紧急停止
   */
  common::Status ProduceControlCommand(ControlCommand *control_command);

  /**
   * @brief 检查输入数据有效性
   *
   * @param local_view LocalView指针
   * @return common::Status 检查状态
   *
   * 功能说明：
   * 检查所有输入数据是否有效
   * 包括轨迹数据有效性、低速轨迹点处理
   */
  common::Status CheckInput(LocalView *local_view);

  /**
   * @brief 检查时间戳有效性
   *
   * @param local_view LocalView常量引用
   * @return common::Status 检查状态
   *
   * 功能说明：
   * 检查定位、底盘、轨迹消息的时间戳
   * 判断是否有消息超时
   */
  common::Status CheckTimestamp(const LocalView &local_view);

  /**
   * @brief 检查Pad消息有效性
   *
   * @return common::Status 检查状态
   *
   * 功能说明：
   * 检查Pad消息是否有效
   */
  common::Status CheckPad();

  /**
   * @brief 重置并生成零控制命令
   *
   * @param chassis 底盘常量指针
   * @param control_command 控制命令指针
   *
   * 功能说明：
   * 在非自动模式下调用
   * 重置控制器状态并输出零控制命令
   */
  void ResetAndProduceZeroControlCommand(const canbus::Chassis *chassis,
                                         ControlCommand *control_command);

  /**
   * @brief 获取车辆俯仰角
   *
   * @param control_command 控制命令指针
   *
   * 功能说明：
   * 计算并设置车辆俯仰角到调试信息中
   */
  void GetVehiclePitchAngle(ControlCommand *control_command);

  /**
   * @brief 检查自动模式
   *
   * @param chassis 底盘常量指针
   *
   * 功能说明：
   * 检查驾驶模式是否切换到自动模式
   * 设置相关标志位
   */
  void CheckAutoMode(const canbus::Chassis *chassis);

  /**
   * @brief 发布控制交互消息
   *
   * 功能说明：
   * 发布控制交互消息到交互话题
   */
  void PublishControlInteractiveMsg();

 private:
  /**
   * @brief 初始化时间
   *
   * 功能说明：
   * 记录组件初始化时的时间戳
   * 用于测试模式超时检测
   *
   * C++语法说明：
   * - apollo::cyber::Time：
   *   Cyber RT的时间类型
   *   支持时间运算和转换
   */
  apollo::cyber::Time init_time_;

  /**
   * @brief 最新定位数据
   *
   * 功能说明：
   * 存储最近一次接收到的定位消息
   *
   * C++语法说明：
   * - localization::LocalizationEstimate：
   *   定位估计消息类型
   */
  localization::LocalizationEstimate latest_localization_;

  /**
   * @brief 最新底盘数据
   *
   * 功能说明：
   * 存储最近一次接收到的底盘消息
   *
   * C++语法说明：
   * - canbus::Chassis：
   *   底盘消息类型
   */
  canbus::Chassis latest_chassis_;

  /**
   * @brief 最新轨迹数据
   *
   * 功能说明：
   * 存储最近一次接收到的规划轨迹
   *
   * C++语法说明：
   * - planning::ADCTrajectory：
   *   自动驾驶轨迹消息类型
   */
  planning::ADCTrajectory latest_trajectory_;

  /**
   * @brief 规划命令状态
   *
   * 功能说明：
   * 存储规划命令的执行状态
   */
  external_command::CommandStatus planning_command_status_;

  /**
   * @brief Pad消息
   *
   * 功能说明：
   * 存储人工操作指令
   */
  PadMessage pad_msg_;

  /**
   * @brief 最新重规划轨迹头
   *
   * 功能说明：
   * 存储最新重规划的轨迹头信息
   * 用于调试和状态追踪
   */
  common::Header latest_replan_trajectory_header_;

  /**
   * @brief 控制任务代理
   *
   * 功能说明：
   * 管理控制器的执行
   * 调用具体的控制算法
   */
  ControlTaskAgent control_task_agent_;

  /**
   * @brief 紧急停止标志
   *
   * 功能说明：
   * 标记是否触发紧急停止
   */
  bool estop_ = false;

  /**
   * @brief 紧急停止原因
   *
   * 功能说明：
   * 存储紧急停止的详细原因
   */
  std::string estop_reason_;

  /**
   * @brief Pad消息接收标志
   *
   * 功能说明：
   * 标记是否接收到新的Pad消息
   */
  bool pad_received_ = false;

  /**
   * @brief 状态丢失计数
   */
  unsigned int status_lost_ = 0;

  /**
   * @brief 状态完整性检查失败计数
   */
  unsigned int status_sanity_check_failed_ = 0;

  /**
   * @brief 总状态丢失计数
   */
  unsigned int total_status_lost_ = 0;

  /**
   * @brief 总状态完整性检查失败计数
   */
  unsigned int total_status_sanity_check_failed_ = 0;

  /**
   * @brief 控制流程配置
   *
   * 功能说明：
   * 存储控制器的配置信息
   *
   * C++语法说明：
   * - ControlPipeline：
   *   控制流程配置消息类型
   */
  ControlPipeline control_pipeline_;

  /**
   * @brief 互斥锁
   *
   * 功能说明：
   * 保护共享数据的线程安全访问
   *
   * C++语法说明：
   * - std::mutex：
   *   互斥锁，用于线程同步
   *   保证同一时刻只有一个线程访问共享数据
   */
  std::mutex mutex_;

  /**
   * @brief 底盘话题读者
   *
   * 功能说明：
   * 订阅底盘话题，接收车辆状态数据
   *
   * C++语法说明：
   * - std::shared_ptr<cyber::Reader<apollo::canbus::Chassis>>：
   *   shared_ptr智能指针管理Reader对象
   *   Reader是Cyber RT的读者类型
   *   <Chassis>指定消息类型
   */
  std::shared_ptr<cyber::Reader<apollo::canbus::Chassis>> chassis_reader_;

  /**
   * @brief Pad消息读者
   */
  std::shared_ptr<cyber::Reader<PadMessage>> pad_msg_reader_;

  /**
   * @brief 定位话题读者
   */
  std::shared_ptr<cyber::Reader<apollo::localization::LocalizationEstimate>>
      localization_reader_;

  /**
   * @brief 轨迹话题读者
   */
  std::shared_ptr<cyber::Reader<apollo::planning::ADCTrajectory>>
      trajectory_reader_;

  /**
   * @brief 规划命令状态读者
   */
  std::shared_ptr<cyber::Reader<apollo::external_command::CommandStatus>>
      planning_command_status_reader_;

  /**
   * @brief 控制命令写入者
   *
   * 功能说明：
   * 发布控制命令到底盘执行
   *
   * C++语法说明：
   * - std::shared_ptr<cyber::Writer<ControlCommand>>：
   *   shared_ptr智能指针管理Writer对象
   *   Writer是Cyber RT的写者类型
   */
  std::shared_ptr<cyber::Writer<ControlCommand>> control_cmd_writer_;

  /**
   * @brief 本地视图写入者（使用子模块时）
   *
   * 功能说明：
   * 当使用控制子模块时，发布LocalView
   */
  std::shared_ptr<cyber::Writer<LocalView>> local_view_writer_;

  /**
   * @brief 控制交互消息写入者
   */
  std::shared_ptr<cyber::Writer<ControlInteractiveMsg>>
      control_interactive_writer_;

  /**
   * @brief 监控日志缓冲区
   *
   * 功能说明：
   * 用于记录控制模块的运行状态和错误
   *
   * C++语法说明：
   * - common::monitor::MonitorLogBuffer：
   *   监控日志缓冲区
   */
  common::monitor::MonitorLogBuffer monitor_logger_buffer_;

  /**
   * @brief 本地视图
   *
   * 功能说明：
   * 存储所有输入数据的整合视图
   */
  LocalView local_view_;

  /**
   * @brief 依赖注入器
   *
   * 功能说明：
   * 提供各模块间的数据共享和解耦
   *
   * C++语法说明：
   * - std::shared_ptr<DependencyInjector>：
   *   智能指针管理DependencyInjector对象
   */
  std::shared_ptr<DependencyInjector> injector_;

  /**
   * @brief 上一次转向命令
   *
   * 功能说明：
   * 存储上一次的转向目标值
   * 用于紧急停止时保持转向
   */
  double previous_steering_command_ = 0.0;

  /**
   * @brief 是否自动模式标志
   */
  bool is_auto_ = false;

  /**
   * @brief 是否从其他模式切换到自动模式
   */
  bool from_else_to_auto_ = false;

  /**
   * @brief 定位超时计数
   */
  uint16_t localization_timeout_count_ = 0;

  /**
   * @brief 轨迹超时计数
   */
  uint16_t trajectory_timeout_count_ = 0;

  /**
   * @brief 底盘超时计数
   */
  uint16_t chassis_timeout_count_ = 0;
};

/**
 * @brief Cyber RT组件注册宏
 *
 * 功能说明：
 * 将ControlComponent类注册为Cyber RT的组件
 * 使得可以通过Cyber RT框架进行加载和调度
 *
 * C++语法说明：
 * - CYBER_REGISTER_COMPONENT：
 *   Cyber RT的组件注册宏
 *   展开后在全局注册表中添加条目
 *   允许框架动态创建此组件实例
 *
 * 使用场景：
 * - 允许在配置文件中声明组件
 * - 支持组件的动态加载
 * - 与Cyber RT调度器集成
 */
CYBER_REGISTER_COMPONENT(ControlComponent)

}  // namespace control
}  // namespace apollo