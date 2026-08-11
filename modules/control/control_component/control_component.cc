/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License atis_control_test_mode
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ******************************************************************************/

/**
 * @file control_component.cc
 *
 * @brief 控制组件实现文件
 *
 * 功能说明：
 * 本文件实现了ControlComponent类，是Apollo控制模块的核心组件
 * 负责读取传感器数据、接收规划轨迹、输出控制命令
 *
 * 主要功能：
 * 1. 订阅底盘、定位、规划等话题数据
 * 2. 检查输入数据有效性和时间戳
 * 3. 调用控制算法计算控制命令
 * 4. 发布控制命令到下游执行器
 *
 * 数据流：
 * Localization(定位) + Chassis(底盘) + Trajectory(轨迹) -> ControlComponent -> ControlCommand(控制命令)
 *
 * C++语法说明：
 * - #include：预处理指令，包含头文件
 * - namespace：命名空间，避免命名冲突
 * - class：类声明
 * - ::运算符：作用域解析，类名::函数名表示成员函数
 * - std::shared_ptr：智能指针，引用计数管理对象生命周期
 * - std::mutex：互斥锁，用于线程同步
 * - std::lock_guard：RAII锁管理类
 **/

#include "modules/control/control_component/control_component.h"

/**
 * @brief Abseil字符串处理库
 *
 * C++语法说明：
 * - absl/strings/str_cat.h：提供字符串拼接功能
 *   比std::string + 操作更高效
 */
#include "absl/strings/str_cat.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
/**
 * @brief Cyber RT时间系统
 *
 * C++语法说明：
 * - cyber/time/clock.h：提供时间获取功能
 * - Clock::Now()：获取当前时间
 */
#include "cyber/time/clock.h"

/**
 * @brief Apollo适配器配置
 *
 * C++语法说明：
 * - adapter_gflags.h：定义话题名称等配置
 * - FLAGS_xxx：gflags全局配置变量
 */
#include "modules/common/adapters/adapter_gflags.h"

/**
 * @brief 延迟记录器
 *
 * C++语法说明：
 * - LatencyRecorder：记录模块处理延迟
 */
#include "modules/common/latency_recorder/latency_recorder.h"

/**
 * @brief 车辆状态提供者
 *
 * C++语法说明：
 * - VehicleStateProvider：提供车辆状态信息
 */
#include "modules/common/vehicle_state/vehicle_state_provider.h"

/**
 * @brief 控制模块配置参数
 *
 * C++语法说明：
 * - control_gflags.h：定义控制模块的配置参数
 */
#include "modules/control/control_component/common/control_gflags.h"

namespace apollo {
/**
 * @brief Apollo主命名空间
 */
namespace control {

/**
 * @brief 类型别名声明
 *
 * C++语法说明：
 * using别名声明，将长类型名简化为短名
 * 类似于typedef，但语法更直观
 */
using apollo::canbus::Chassis;                     ///< 底盘消息类型
using apollo::common::ErrorCode;                  ///< 错误码类型
using apollo::common::Status;                     ///< 状态类型
using apollo::common::VehicleStateProvider;        ///< 车辆状态提供者
using apollo::cyber::Clock;                       ///< Cyber RT时钟
using apollo::localization::LocalizationEstimate; ///< 定位估计消息
using apollo::planning::ADCTrajectory;           ///< 自动驾驶轨迹消息

/**
 * @brief 双重精度epsilon
 *
 * 功能说明：
 * 用于浮点数比较的极小值
 * 避免直接比较浮点数相等
 *
 * C++语法说明：
 * - const double：
 *   常量double类型
 * - kDoubleEpsilon：
 *   k开头是Apollo的常量命名惯例
 * - 1e-6：
 *   科学计数法，表示1×10⁻⁶
 */
const double kDoubleEpsilon = 1e-6;

/**
 * @brief 默认构造函数
 *
 * 功能说明：
 * 初始化控制组件
 * 初始化监控日志缓冲区
 *
 * C++语法说明：
 * - ControlComponent()：
 *   构造函数，没有返回类型
 *
 * - : monitor_logger_buffer_(common::monitor::MonitorMessageItem::CONTROL)：
 *   初始化列表
 *   在构造函数体执行前初始化成员变量
 *   将monitor_logger_buffer_初始化为CONTROL类型的监控消息
 */
ControlComponent::ControlComponent()
    : monitor_logger_buffer_(common::monitor::MonitorMessageItem::CONTROL) {}

/**
 * @brief 初始化函数
 *
 * @return bool 初始化成功返回true
 *
 * 功能说明：
 * 控制组件的初始化入口
 * 执行以下初始化操作：
 * 1. 创建依赖注入器
 * 2. 加载控制流程配置
 * 3. 创建话题读者和作者
 * 4. 等待车辆状态就绪
 *
 * C++语法说明：
 * - bool Init()：
 *   初始化函数，返回bool表示成功失败
 *
 * - std::make_shared<DependencyInjector>()：
 *   创建共享指针
 *   make_shared比直接用new更高效
 *   在堆上分配DependencyInjector对象
 *
 * - Clock::Now()：
 *   静态方法调用
 *   ::作用域限定符
 *   获取当前时间戳
 *
 * - cyber::common::GetProtoFromFile(FLAGS_pipeline_file, &control_pipeline_)：
 *   从文件加载protobuf配置
 *   &获取地址作为输出参数
 */
bool ControlComponent::Init() {
  injector_ = std::make_shared<DependencyInjector>();
  init_time_ = Clock::Now();

  AINFO << "Control init, starting ...";

  /**
   * @brief 加载控制流程配置文件
   *
   * C++语法说明：
   * - ACHECK(...)：
   *   Apollo断言宏
   *   如果条件为false，输出错误并终止程序
   *
   * - cyber::common::GetProtoFromFile：
   *   从配置文件加载protobuf消息
   *   FLAGS_pipeline_file是配置文件路径
   *   &control_pipeline_是输出参数
   */
  // /apollo/modules/control/control_component/conf/pipeline.pb.txt
  ACHECK(
      cyber::common::GetProtoFromFile(FLAGS_pipeline_file, &control_pipeline_))
      << "Unable to load control pipeline file: " + FLAGS_pipeline_file;

  AINFO << "ControlTask pipeline config file: " << FLAGS_pipeline_file
        << " is loaded.";

  /**
   * @brief 初始化控制器代理
   *
   * C++语法说明：
   * - if (!FLAGS_use_control_submodules && ...)：
   *   !逻辑非，检查是否不使用子模块
   * - control_task_agent_.Init(injector_, control_pipeline_).ok()：
   *   调用Init方法，返回Status对象
   *   .ok()检查是否成功
   */
  ADEBUG << "FLAGS_use_control_submodules: " << FLAGS_use_control_submodules;
  if (!FLAGS_is_control_ut_test_mode) {
    if (!FLAGS_use_control_submodules &&
        !control_task_agent_.Init(injector_, control_pipeline_).ok()) {
      ADEBUG << "original control";
      monitor_logger_buffer_.ERROR(
          "Control init controller failed! Stopping...");
      return false;
    }
  }

  /**
   * @brief 创建底盘话题读者
   *
   * C++语法说明：
   * - cyber::ReaderConfig：
   *   Cyber RT读者配置结构体
   * - chassis_reader_config.channel_name = FLAGS_chassis_topic：
   *   设置要订阅的话题名称
   * - chassis_reader_config.pending_queue_size = FLAGS_chassis_pending_queue_size：
   *   设置待处理队列大小
   * - node_->CreateReader<Chassis>(...)：
   *   模板函数，创建指定类型的读者
   *   <Chassis>指定消息类型
   */
  cyber::ReaderConfig chassis_reader_config;
  chassis_reader_config.channel_name = FLAGS_chassis_topic;
  chassis_reader_config.pending_queue_size = FLAGS_chassis_pending_queue_size;

  chassis_reader_ =
      node_->CreateReader<Chassis>(chassis_reader_config, nullptr);
  ACHECK(chassis_reader_ != nullptr);

  /**
   * @brief 创建规划轨迹话题读者
   *
   * C++语法说明：
   * - ADCTrajectory：
   *   自动驾驶轨迹消息类型
   *   包含轨迹点序列
   */
  cyber::ReaderConfig planning_reader_config;
  planning_reader_config.channel_name = FLAGS_planning_trajectory_topic;
  planning_reader_config.pending_queue_size = FLAGS_planning_pending_queue_size;

  trajectory_reader_ =
      node_->CreateReader<ADCTrajectory>(planning_reader_config, nullptr);
  ACHECK(trajectory_reader_ != nullptr);

  /**
   * @brief 创建规划命令状态读者
   */
  cyber::ReaderConfig planning_command_status_reader_config;
  planning_command_status_reader_config.channel_name =
      FLAGS_planning_command_status;
  planning_command_status_reader_config.pending_queue_size =
      FLAGS_planning_status_msg_pending_queue_size;
  planning_command_status_reader_ =
      node_->CreateReader<external_command::CommandStatus>(
          planning_command_status_reader_config, nullptr);
  ACHECK(planning_command_status_reader_ != nullptr);

  /**
   * @brief 创建定位话题读者
   */
  cyber::ReaderConfig localization_reader_config;
  localization_reader_config.channel_name = FLAGS_localization_topic;
  localization_reader_config.pending_queue_size =
      FLAGS_localization_pending_queue_size;

  localization_reader_ = node_->CreateReader<LocalizationEstimate>(
      localization_reader_config, nullptr);
  ACHECK(localization_reader_ != nullptr);

  /**
   * @brief 创建Pad消息读者
   */
  cyber::ReaderConfig pad_msg_reader_config;
  pad_msg_reader_config.channel_name = FLAGS_pad_topic;
  pad_msg_reader_config.pending_queue_size = FLAGS_pad_msg_pending_queue_size;

  pad_msg_reader_ =
      node_->CreateReader<PadMessage>(pad_msg_reader_config, nullptr);
  ACHECK(pad_msg_reader_ != nullptr);

  /**
   * @brief 创建控制命令写入者
   *
   * C++语法说明：
   * - if (!FLAGS_use_control_submodules)：
   *   条件分支：是否使用子模块
   * - node_->CreateWriter<ControlCommand>(FLAGS_control_command_topic)：
   *   创建控制命令写入者
   *   发布ControlCommand消息到指定话题
   */
  if (!FLAGS_use_control_submodules) {
    control_cmd_writer_ =
        node_->CreateWriter<ControlCommand>(FLAGS_control_command_topic);
    ACHECK(control_cmd_writer_ != nullptr);
  } else {
    local_view_writer_ =
        node_->CreateWriter<LocalView>(FLAGS_control_local_view_topic);
    ACHECK(local_view_writer_ != nullptr);
  }
  control_interactive_writer_ = node_->CreateWriter<ControlInteractiveMsg>(
      FLAGS_control_interative_topic);
  ACHECK(control_interactive_writer_ != nullptr);

  /**
   * @brief 等待车辆状态就绪
   *
   * 功能说明：
   * 因为广告频道不会立即就绪
   * 需要短暂等待
   *
   * C++语法说明：
   * - std::this_thread::sleep_for(std::chrono::milliseconds(1000))：
   *   线程休眠1000毫秒
   *   this_thread命名空间提供线程控制函数
   *   chrono是时间库，milliseconds是毫秒单位
   */
  AINFO << "Control resetting vehicle state, sleeping for 1000 ms ...";
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  /**
   * @brief 设置默认驾驶动作
   *
   * C++语法说明：
   * - DrivingAction_Name((enum DrivingAction)FLAGS_action)：
   *   枚举类型转换 + 获取枚举名称
   *   将枚举值转换为字符串
   *
   * - pad_msg_.set_action((enum DrivingAction)FLAGS_action)：
   *   set方法设置protobuf消息字段
   *   显式类型转换
   */
  AINFO << "Control default driving action is "
        << DrivingAction_Name((enum DrivingAction)FLAGS_action);
  pad_msg_.set_action((enum DrivingAction)FLAGS_action);

  return true;
}

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
 *   shared_ptr：智能指针，引用计数管理对象
 *   const引用避免拷贝
 *
 * - std::lock_guard<std::mutex> lock(mutex_)：
 *   RAII风格的互斥锁
 *   构造时加锁，析构时解锁
 *   自动管理锁的生命周期
 *
 * - pad_msg_.CopyFrom(*pad)：
 *   CopyFrom：protobuf消息拷贝方法
 *   *pad解引用获取对象
 */
void ControlComponent::OnPad(const std::shared_ptr<PadMessage> &pad) {
  std::lock_guard<std::mutex> lock(mutex_);
  pad_msg_.CopyFrom(*pad);
  ADEBUG << "Received Pad Msg:" << pad_msg_.DebugString();
  AERROR_IF(!pad_msg_.has_action()) << "pad message check failed!";
}

/**
 * @brief 底盘消息回调
 *
 * @param chassis 底盘消息的共享指针
 *
 * 功能说明：
 * 处理接收到的底盘数据
 * 更新最新底盘状态
 */
void ControlComponent::OnChassis(const std::shared_ptr<Chassis> &chassis) {
  ADEBUG << "Received chassis data: run chassis callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  latest_chassis_.CopyFrom(*chassis);
}

/**
 * @brief 规划轨迹回调
 *
 * @param trajectory 轨迹消息的共享指针
 *
 * 功能说明：
 * 处理接收到的规划轨迹数据
 * 更新最新轨迹
 */
void ControlComponent::OnPlanning(
    const std::shared_ptr<ADCTrajectory> &trajectory) {
  ADEBUG << "Received chassis data: run trajectory callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  latest_trajectory_.CopyFrom(*trajectory);
}

/**
 * @brief 规划命令状态回调
 *
 * @param planning_command_status 规划命令状态的共享指针
 */
void ControlComponent::OnPlanningCommandStatus(
    const std::shared_ptr<external_command::CommandStatus>
        &planning_command_status) {
  ADEBUG << "Received plannning command status data: run planning command "
            "status callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  planning_command_status_.CopyFrom(*planning_command_status);
}

/**
 * @brief 定位消息回调
 *
 * @param localization 定位消息的共享指针
 *
 * 功能说明：
 * 处理接收到的定位数据
 * 更新最新定位状态
 */
void ControlComponent::OnLocalization(
    const std::shared_ptr<LocalizationEstimate> &localization) {
  ADEBUG << "Received control data: run localization message callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  latest_localization_.CopyFrom(*localization);
}

/**
 * @brief 监控消息回调
 *
 * @param monitor_message 监控消息
 *
 * 功能说明：
 * 处理监控消息
 * 如果有FATAL级别消息，设置紧急停止标志
 *
 * C++语法说明：
 * - for (const auto &item : monitor_message.item())：
 *   范围for循环遍历
 *   item()返回repeated字段的迭代器范围
 *
 * - common::monitor::MonitorMessageItem::FATAL：
 *   监控消息的FATAL级别
 */
void ControlComponent::OnMonitor(
    const common::monitor::MonitorMessage &monitor_message) {
  for (const auto &item : monitor_message.item()) {
    if (item.log_level() == common::monitor::MonitorMessageItem::FATAL) {
      estop_ = true;
      return;
    }
  }
}

/**
 * @brief 生产控制命令
 *
 * @param control_command 输出参数，生成的控制命令
 * @return Status 处理状态
 *
 * 功能说明：
 * 控制组件的核心函数
 * 完整的控制流程：
 * 1. 检查输入数据有效性
 * 2. 检查时间戳
 * 3. 检查紧急停止条件
 * 4. 调用控制器计算控制命令
 * 5. 处理紧急停止
 *
 * C++语法说明：
 * - Status ControlComponent::ProduceControlCommand(...)：
 *   返回Status类型表示处理结果
 *
 * - ControlCommand *control_command：
 *   原始指针作为输出参数
 *   指向调用者提供的ControlCommand对象
 */
Status ControlComponent::ProduceControlCommand(
    ControlCommand *control_command) {
  Status status = CheckInput(&local_view_);

  /**
   * @brief 输入数据检查失败处理
   */
  if (!status.ok()) {
    AERROR_EVERY(100) << "Control input data failed: "
                      << status.error_message();
    /**
     * @brief 设置不允许接管建议
     *
     * C++语法说明：
     * - mutable_engage_advice()：
     *   mutable方法获取可修改的子消息
     *   即使外层对象是const也能修改
     *
     * - set_advice/set_reason：
     *   protobuf的set方法设置字段值
     */
    control_command->mutable_engage_advice()->set_advice(
        apollo::common::EngageAdvice::DISALLOW_ENGAGE);
    control_command->mutable_engage_advice()->set_reason(
        status.error_message());
    estop_ = true;
    estop_reason_ = status.error_message();
  } else {
    estop_ = false;
    Status status_ts = CheckTimestamp(local_view_);
    if (!status_ts.ok()) {
      AERROR << "Input messages timeout";
      estop_ = true;
      status = status_ts;
      if (local_view_.chassis().driving_mode() !=
          apollo::canbus::Chassis::COMPLETE_AUTO_DRIVE) {
        control_command->mutable_engage_advice()->set_advice(
            apollo::common::EngageAdvice::DISALLOW_ENGAGE);
        control_command->mutable_engage_advice()->set_reason(
            status.error_message());
      }
    } else {
      control_command->mutable_engage_advice()->set_advice(
          apollo::common::EngageAdvice::READY_TO_ENGAGE);
      estop_ = false;
    }
  }

  /**
   * @brief 检查紧急停止标志
   *
   * C++语法说明：
   * - estop_ = FLAGS_enable_persistent_estop ? ... : ...：
   *   三元运算符
   *   条件 ? 值1 : 值2
   * - local_view_.trajectory().estop().is_estop()：
   *   链式调用获取嵌套消息
   */
  estop_ = FLAGS_enable_persistent_estop
               ? estop_ || local_view_.trajectory().estop().is_estop()
               : local_view_.trajectory().estop().is_estop();

  if (local_view_.trajectory().estop().is_estop()) {
    estop_ = true;
    estop_reason_ = "estop from planning : ";
    estop_reason_ += local_view_.trajectory().estop().reason();
  }

  /**
   * @brief 检查轨迹是否为空
   */
  if (local_view_.trajectory().trajectory_point().empty()) {
    AWARN_EVERY(100) << "planning has no trajectory point. ";
    estop_ = true;
    estop_reason_ = "estop for empty planning trajectory, planning headers: " +
                    local_view_.trajectory().header().ShortDebugString();
  }

  /**
   * @brief 检查前进挡负速度保护
   */
  if (FLAGS_enable_gear_drive_negative_speed_protection) {
    const double kEpsilon = 0.001;
    auto first_trajectory_point = local_view_.trajectory().trajectory_point(0);
    if (local_view_.chassis().gear_location() == Chassis::GEAR_DRIVE &&
        first_trajectory_point.v() < -1 * kEpsilon) {
      estop_ = true;
      estop_reason_ = "estop for negative speed when gear_drive";
    }
  }

  /**
   * @brief 非紧急停止时的处理
   */
  if (!estop_) {
    if (local_view_.chassis().driving_mode() == Chassis::COMPLETE_MANUAL) {
      control_task_agent_.Reset();
      AINFO_EVERY(100) << "No estop. Reset Controllers in Manual Mode";
    }

    /**
     * @brief 填充调试信息
     *
     * C++语法说明：
     * - control_command->mutable_debug()->mutable_input_debug()：
     *   链式调用mutable方法
     *   mutable_debug()获取调试子消息
     *   mutable_input_debug()获取输入调试子消息
     *
     * - CopyFrom：
     *   protobuf消息拷贝
     */
    auto debug = control_command->mutable_debug()->mutable_input_debug();
    debug->mutable_localization_header()->CopyFrom(
        local_view_.localization().header());
    debug->mutable_canbus_header()->CopyFrom(local_view_.chassis().header());
    debug->mutable_trajectory_header()->CopyFrom(
        local_view_.trajectory().header());

    if (local_view_.trajectory().is_replan()) {
      latest_replan_trajectory_header_ = local_view_.trajectory().header();
    }

    if (latest_replan_trajectory_header_.has_sequence_num()) {
      debug->mutable_latest_replan_trajectory_header()->CopyFrom(
          latest_replan_trajectory_header_);
    }
  }

  /**
   * @brief 调用控制器计算控制命令
   */
  if (!estop_) {
    if (!local_view_.trajectory().trajectory_point().empty()) {
      /**
       * @brief 控制器代理计算控制命令
       *
       * C++语法说明：
       * - control_task_agent_.ComputeControlCommand(...):
       *   调用控制器计算函数
       *   传入定位、底盘、轨迹信息
       *   输出控制命令
       */
      Status status_compute = control_task_agent_.ComputeControlCommand(
          &local_view_.localization(), &local_view_.chassis(),
          &local_view_.trajectory(), control_command);
      ADEBUG << "status_compute is " << status_compute;

      if (!status_compute.ok()) {
        AERROR << "Control main function failed" << " with localization: "
               << local_view_.localization().ShortDebugString()
               << " with chassis: " << local_view_.chassis().ShortDebugString()
               << " with trajectory: "
               << local_view_.trajectory().ShortDebugString()
               << " with cmd: " << control_command->ShortDebugString()
               << " status:" << status_compute.error_message();
        estop_ = true;
        estop_reason_ = status_compute.error_message();
        status = status_compute;
      }
    }
  } else {
    control_task_agent_.Reset();
    AINFO_EVERY(10) << "Estop trigger, Reset Controllers in Auto Mode";
  }

  /**
   * @brief 紧急停止时的命令设置
   */
  if (estop_) {
    AWARN_EVERY(100) << "Estop triggered! No control core method executed!";
    control_command->set_speed(0.0);
    control_command->set_throttle(0.0);
    control_command->set_brake(FLAGS_soft_estop_brake);
    control_command->set_acceleration(FLAGS_soft_estop_acceleration);
    control_command->set_gear_location(latest_chassis_.gear_location());
    control_command->set_parking_brake(latest_chassis_.parking_brake());
    previous_steering_command_ =
        injector_->previous_control_command_mutable()->steering_target();
    control_command->set_steering_target(previous_steering_command_);
  }

  /**
   * @brief 设置车辆信号
   */
  if (local_view_.trajectory().decision().has_vehicle_signal()) {
    control_command->mutable_signal()->CopyFrom(
        local_view_.trajectory().decision().vehicle_signal());
  }
  return status;
}

/**
 * @brief 处理函数（主循环）
 *
 * @return bool 处理成功返回true
 *
 * 功能说明：
 * 控制组件的主处理循环
 * 被Cyber RT调度器循环调用
 *
 * 处理流程：
 * 1. 读取各话题最新数据
 * 2. 组装LocalView
 * 3. 调用ProduceControlCommand
 * 4. 发布控制命令
 */
bool ControlComponent::Proc() {
  AINFO << "control proc start.";
  const auto start_time = Clock::Now();

  injector_->control_debug_info_clear();

  /**
   * @brief 读取底盘数据
   *
   * C++语法说明：
   * - chassis_reader_->Observe()：
   *   通知读者有新消息
   * - chassis_reader_->GetLatestObserved()：
   *   获取最新收到的消息
   */
  chassis_reader_->Observe();
  const auto &chassis_msg = chassis_reader_->GetLatestObserved();
  if (chassis_msg == nullptr) {
    AERROR << "Chassis msg is not ready!";
    injector_->set_control_process(false);
    return false;
  }
  OnChassis(chassis_msg);

  /**
   * @brief 读取规划轨迹数据
   */
  trajectory_reader_->Observe();
  const auto &trajectory_msg = trajectory_reader_->GetLatestObserved();
  if (trajectory_msg == nullptr) {
    AERROR << "planning msg is not ready!";
  } else {
    /**
     * @brief 检查是否收到新的规划数据
     *
     * C++语法说明：
     * - latest_trajectory_.header().sequence_num()：
     *   获取消息序列号
     * - trajectory_msg->header().sequence_num()：
     *   ->调用指针的成员函数
     */
    if (latest_trajectory_.header().sequence_num() !=
        trajectory_msg->header().sequence_num()) {
      OnPlanning(trajectory_msg);
    }
  }

  /**
   * @brief 读取规划命令状态
   */
  planning_command_status_reader_->Observe();
  const auto &planning_status_msg =
      planning_command_status_reader_->GetLatestObserved();
  if (planning_status_msg != nullptr) {
    OnPlanningCommandStatus(planning_status_msg);
    ADEBUG << "Planning command status msg is \n"
           << planning_command_status_.ShortDebugString();
  }
  injector_->set_planning_command_status(planning_command_status_);

  /**
   * @brief 读取定位数据
   */
  localization_reader_->Observe();
  const auto &localization_msg = localization_reader_->GetLatestObserved();
  if (localization_msg == nullptr) {
    AERROR << "localization msg is not ready!";
    injector_->set_control_process(false);
    return false;
  }
  OnLocalization(localization_msg);

  /**
   * @brief 读取Pad消息
   */
  pad_msg_reader_->Observe();
  const auto &pad_msg = pad_msg_reader_->GetLatestObserved();
  if (pad_msg != nullptr) {
    OnPad(pad_msg);
  }

  /**
   * @brief 组装LocalView
   *
   * C++语法说明：
   * - std::lock_guard<std::mutex> lock(mutex_)：
   *   作用域锁，在块结束时自动解锁
   *
   * - local_view_.mutable_chassis()->CopyFrom(...)：
   *   mutable_chassis()获取可修改的底盘消息
   *   CopyFrom拷贝数据
   */
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.mutable_chassis()->CopyFrom(latest_chassis_);
    local_view_.mutable_trajectory()->CopyFrom(latest_trajectory_);
    local_view_.mutable_localization()->CopyFrom(latest_localization_);
    if (pad_msg != nullptr) {
      local_view_.mutable_pad_msg()->CopyFrom(pad_msg_);
    }
  }

  /**
   * @brief 使用控制子模块
   */
  if (FLAGS_use_control_submodules) {
    local_view_.mutable_header()->set_lidar_timestamp(
        local_view_.trajectory().header().lidar_timestamp());
    local_view_.mutable_header()->set_camera_timestamp(
        local_view_.trajectory().header().camera_timestamp());
    local_view_.mutable_header()->set_radar_timestamp(
        local_view_.trajectory().header().radar_timestamp());
    common::util::FillHeader(FLAGS_control_local_view_topic, &local_view_);

    const auto end_time = Clock::Now();

    /**
     * @brief 记录延迟
     *
     * C++语法说明：
     * - static apollo::common::LatencyRecorder latency_recorder(...)：
     *   static局部变量
     *   只初始化一次
     *   在函数调用间保持存在
     */
    static apollo::common::LatencyRecorder latency_recorder(
        FLAGS_control_local_view_topic);
    latency_recorder.AppendLatencyRecord(
        local_view_.trajectory().header().lidar_timestamp(), start_time,
        end_time);

    local_view_writer_->Write(local_view_);
    return true;
  }

  /**
   * @brief 处理Pad消息
   */
  if (pad_msg != nullptr) {
    ADEBUG << "pad_msg: " << pad_msg_.ShortDebugString();
    ADEBUG << "pad_msg is not nullptr";
    if (pad_msg_.action() == DrivingAction::RESET) {
      AINFO << "Control received RESET action!";
      estop_ = false;
      estop_reason_.clear();
    }
    pad_received_ = true;
  }

  /**
   * @brief 测试模式处理
   */
  if (FLAGS_is_control_test_mode && FLAGS_control_test_duration > 0 &&
      (start_time - init_time_).ToSecond() > FLAGS_control_test_duration) {
    AERROR << "Control finished testing. exit";
    injector_->set_control_process(false);
    return false;
  }

  injector_->set_control_process(true);

  injector_->mutable_control_debug_info()
      ->mutable_control_component_debug()
      ->Clear();
  CheckAutoMode(&local_view_.chassis());

  ControlCommand control_command;

  Status status;
  if (local_view_.chassis().driving_mode() ==
      apollo::canbus::Chassis::COMPLETE_AUTO_DRIVE) {
    status = ProduceControlCommand(&control_command);
    ADEBUG << "Produce control command normal.";
  } else {
    ADEBUG << "Into reset control command.";
    ResetAndProduceZeroControlCommand(&latest_chassis_, &control_command);
  }

  AERROR_IF(!status.ok()) << "Failed to produce control command:"
                          << status.error_message();

  if (pad_received_) {
    control_command.mutable_pad_msg()->CopyFrom(pad_msg_);
    pad_received_ = false;
  }

  /**
   * @brief 传递紧急停止原因
   */
  if (estop_) {
    control_command.mutable_header()->mutable_status()->set_msg(estop_reason_);
  }

  /**
   * @brief 设置时间戳头
   */
  control_command.mutable_header()->set_lidar_timestamp(
      local_view_.trajectory().header().lidar_timestamp());
  control_command.mutable_header()->set_camera_timestamp(
      local_view_.trajectory().header().camera_timestamp());
  control_command.mutable_header()->set_radar_timestamp(
      local_view_.trajectory().header().radar_timestamp());

  if (FLAGS_is_control_test_mode) {
    ADEBUG << "Skip publish control command in test mode";
    return true;
  }

  /**
   * @brief 获取车辆俯仰角
   */
  if (fabs(control_command.debug().simple_lon_debug().vehicle_pitch()) <
      kDoubleEpsilon) {
    injector_->vehicle_state()->Update(local_view_.localization(),
                                       local_view_.chassis());
    GetVehiclePitchAngle(&control_command);
  }

  const auto end_time = Clock::Now();
  const double time_diff_ms = (end_time - start_time).ToSecond() * 1e3;
  ADEBUG << "total control time spend: " << time_diff_ms << " ms.";

  /**
   * @brief 设置延迟统计
   *
   * C++语法说明：
   * - set_total_time_ms：
   *   设置总时间（毫秒）
   * - set_total_time_exceeded：
   *   设置是否超过周期
   */
  control_command.mutable_latency_stats()->set_total_time_ms(time_diff_ms);
  control_command.mutable_latency_stats()->set_total_time_exceeded(
      time_diff_ms > FLAGS_control_period * 1e3);
  if (control_command.mutable_latency_stats()->total_time_exceeded()) {
    AINFO << "total control cycle time is exceeded: " << time_diff_ms << " ms.";
  }
  status.Save(control_command.mutable_header()->mutable_status());

  /**
   * @brief 记录延迟到延迟记录器
   */
  if (local_view_.trajectory().header().has_lidar_timestamp()) {
    static apollo::common::LatencyRecorder latency_recorder(
        FLAGS_control_command_topic);
    latency_recorder.AppendLatencyRecord(
        local_view_.trajectory().header().lidar_timestamp(), start_time,
        end_time);
  }

  /**
   * @brief 填充消息头
   *
   * C++语法说明：
   * - common::util::FillHeader：
   *   填充消息头部的通用函数
   */
  common::util::FillHeader(node_->Name(), &control_command);
  if (FLAGS_sim_by_record) {
    control_command.mutable_header()->set_timestamp_sec(
        latest_chassis_.header().timestamp_sec());
  }
  ADEBUG << control_command.ShortDebugString();

  /**
   * @brief 发布控制命令
   */
  control_cmd_writer_->Write(control_command);

  /**
   * @brief 保存当前控制命令
   */
  injector_->Set_pervious_control_command(&control_command);
  injector_->previous_control_command_mutable()->CopyFrom(control_command);
  injector_->previous_control_debug_mutable()->CopyFrom(
      injector_->control_debug_info());

  PublishControlInteractiveMsg();
  const auto end_process_control_time = Clock::Now();
  const double process_control_time_diff =
      (end_process_control_time - start_time).ToSecond() * 1e3;
  if (control_command.mutable_latency_stats()->total_time_exceeded()) {
    AINFO << "control all spend time is exceeded.";
  }
  AINFO << "control proc finished, total time spend: "
        << process_control_time_diff << " ms.";
  return true;
}

/**
 * @brief 检查输入数据有效性
 *
 * @param local_view LocalView指针
 * @return Status 检查状态
 *
 * 功能说明：
 * 检查所有输入数据是否有效
 * 包括轨迹数据有效性、低速轨迹点处理
 *
 * C++语法说明：
 * - Status CheckInput(LocalView *local_view)：
 *   返回Status类型
 *   指针作为输入输出参数
 */
Status ControlComponent::CheckInput(LocalView *local_view) {
  ADEBUG << "Received localization:"
         << local_view->localization().ShortDebugString();
  ADEBUG << "Received chassis:" << local_view->chassis().ShortDebugString();

  if (!local_view->trajectory().estop().is_estop() &&
      local_view->trajectory().trajectory_point().empty()) {
    AWARN_EVERY(100) << "planning has no trajectory point. ";
    const std::string msg =
        absl::StrCat("planning has no trajectory point. planning_seq_num:",
                     local_view->trajectory().header().sequence_num());
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, msg);
  }

  /**
   * @brief 处理低速轨迹点
   *
   * 功能说明：
   * 如果轨迹点的速度和加速度都接近0
   * 将速度精确设置为0
   *
   * C++语法说明：
   * - for (auto &trajectory_point : *local_view->mutable_trajectory()->mutable_trajectory_point())：
   *   mutable_trajectory()获取可变轨迹消息
   *   mutable_trajectory_point()获取可变轨迹点列表
   *   *解引用获取列表引用
   *
   * - std::abs(...):
   *   绝对值函数
   */
  for (auto &trajectory_point :
       *local_view->mutable_trajectory()->mutable_trajectory_point()) {
    if (std::abs(trajectory_point.v()) < FLAGS_minimum_speed_resolution &&
        std::abs(trajectory_point.a()) < FLAGS_max_acceleration_when_stopped) {
      trajectory_point.set_v(0.0);
      trajectory_point.set_a(0.0);
    }
  }

  injector_->vehicle_state()->Update(local_view->localization(),
                                     local_view->chassis());

  return Status::OK();
}

/**
 * @brief 检查时间戳有效性
 *
 * @param local_view LocalView引用
 * @return Status 检查状态
 *
 * 功能说明：
 * 检查定位、底盘、轨迹消息的时间戳
 * 判断是否有消息超时
 *
 * C++语法说明：
 * - const LocalView &local_view：
 *   常量引用输入参数
 */
Status ControlComponent::CheckTimestamp(const LocalView &local_view) {
  if (!FLAGS_enable_input_timestamp_check || FLAGS_is_control_test_mode) {
    ADEBUG << "Skip input timestamp check by gflags.";
    return Status::OK();
  }
  std::string err_msg = "";
  double current_timestamp = FLAGS_sim_by_record
                                 ? latest_chassis_.header().timestamp_sec()
                                 : Clock::NowInSeconds();

  /**
   * @brief 检查定位时间戳
   */
  double localization_diff =
      current_timestamp - local_view.localization().header().timestamp_sec();
  bool localization_consist_timeout = false;
  if (localization_diff >
      (FLAGS_max_localization_miss_num * FLAGS_localization_period)) {
    localization_consist_timeout = true;
    localization_timeout_count_++;
    AERROR << "Localization msg lost for " << std::to_string(localization_diff)
           << "s";
    AERROR << "current_timestamp: " << std::to_string(current_timestamp)
           << ", localization_timestamp: "
           << std::to_string(
                  local_view.localization().header().timestamp_sec());
    err_msg = err_msg + " Localization msg timeout. ";
  } else {
    localization_consist_timeout = false;
    localization_timeout_count_ = 0;
  }
  if (localization_consist_timeout &&
      (localization_timeout_count_ >= FLAGS_max_localization_miss_num)) {
    localization_timeout_count_ = FLAGS_max_localization_miss_num;
    AERROR << "write the monitor logger, localization msg lost "
           << localization_timeout_count_ << " times.";
    monitor_logger_buffer_.ERROR("Localization msg lost");
  }

  /**
   * @brief 检查底盘时间戳
   */
  double chassis_diff =
      current_timestamp - local_view.chassis().header().timestamp_sec();
  bool chassis_consist_timeout = false;
  if (chassis_diff > (FLAGS_max_chassis_miss_num * FLAGS_chassis_period)) {
    chassis_consist_timeout = true;
    chassis_timeout_count_++;
    AERROR << "Chassis msg lost for " << std::to_string(chassis_diff) << "s";
    AERROR << "current_timestamp: " << std::to_string(current_timestamp)
           << ", chassis_timestamp: "
           << std::to_string(local_view.chassis().header().timestamp_sec());
    err_msg = err_msg + " Chassis msg timeout. ";
  } else {
    chassis_consist_timeout = false;
    chassis_timeout_count_ = 0;
  }
  if (chassis_consist_timeout &&
      (chassis_timeout_count_ >= FLAGS_max_chassis_miss_num)) {
    chassis_timeout_count_ = FLAGS_max_chassis_miss_num;
    AERROR << "write the monitor logger, chassis msg lost "
           << chassis_timeout_count_ << " times.";
    monitor_logger_buffer_.ERROR("Chassis msg lost");
  }

  /**
   * @brief 检查轨迹时间戳
   */
  double trajectory_diff =
      current_timestamp - local_view.trajectory().header().timestamp_sec();
  bool trajectory_consist_timeout = false;
  if (trajectory_diff >
      (FLAGS_max_planning_miss_num * FLAGS_trajectory_period)) {
    trajectory_consist_timeout = true;
    trajectory_timeout_count_++;
    AERROR << "Trajectory msg lost for " << std::to_string(trajectory_diff)
           << "s";
    AERROR << "current_timestamp: " << std::to_string(current_timestamp)
           << ", trajectory_timestamp: "
           << std::to_string(local_view.trajectory().header().timestamp_sec());
    err_msg = err_msg + " Trajectory msg lost. ";
  } else {
    trajectory_consist_timeout = false;
    trajectory_timeout_count_ = 0;
  }
  if (trajectory_consist_timeout &&
      (trajectory_timeout_count_ >= FLAGS_max_localization_miss_num)) {
    trajectory_timeout_count_ = FLAGS_max_localization_miss_num;
    AERROR << "write the monitor logger, trajectory msg lost "
           << trajectory_timeout_count_ << " times.";
    monitor_logger_buffer_.ERROR("Trajectory msg lost");
  }

  if (!err_msg.empty()) {
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, err_msg);
  } else {
    return Status::OK();
  }
}

/**
 * @brief 重置并生成零控制命令
 *
 * @param chassis 底盘指针
 * @param control_command 控制命令指针
 *
 * 功能说明：
 * 在非自动模式下调用
 * 重置控制器状态并输出零控制命令
 */
void ControlComponent::ResetAndProduceZeroControlCommand(
    const canbus::Chassis *chassis, ControlCommand *control_command) {
  control_command->set_throttle(0.0);
  control_command->set_steering_target(0.0);
  control_command->set_steering_rate(0.0);
  control_command->set_speed(0.0);
  control_command->set_brake(0.0);
  control_command->set_gear_location(chassis->gear_location());
  control_command->set_parking_brake(chassis->parking_brake());
  control_task_agent_.Reset();
  latest_trajectory_.mutable_trajectory_point()->Clear();
  latest_trajectory_.mutable_path_point()->Clear();
  trajectory_reader_->ClearData();
}

/**
 * @brief 获取车辆俯仰角
 *
 * @param control_command 控制命令指针
 *
 * 功能说明：
 * 计算并设置车辆俯仰角到调试信息中
 */
void ControlComponent::GetVehiclePitchAngle(ControlCommand *control_command) {
  double vehicle_pitch = injector_->vehicle_state()->pitch() * 180 / M_PI;
  control_command->mutable_debug()
      ->mutable_simple_lon_debug()
      ->set_vehicle_pitch(vehicle_pitch + FLAGS_pitch_offset_deg);
}

/**
 * @brief 检查自动模式
 *
 * @param chassis 底盘指针
 *
 * 功能说明：
 * 检查驾驶模式是否切换到自动模式
 * 设置相关标志位
 */
void ControlComponent::CheckAutoMode(const canbus::Chassis *chassis) {
  if (!injector_->previous_control_debug_mutable()
           ->mutable_control_component_debug()
           ->is_auto() &&
      chassis->driving_mode() == apollo::canbus::Chassis::COMPLETE_AUTO_DRIVE) {
    from_else_to_auto_ = true;
    AINFO << "From else to auto!!!";
  } else {
    from_else_to_auto_ = false;
  }
  ADEBUG << "from_else_to_auto_: " << from_else_to_auto_;
  injector_->mutable_control_debug_info()
      ->mutable_control_component_debug()
      ->set_from_else_to_auto(from_else_to_auto_);

  if (chassis->driving_mode() == apollo::canbus::Chassis::COMPLETE_AUTO_DRIVE) {
    is_auto_ = true;
  } else {
    is_auto_ = false;
  }
  injector_->mutable_control_debug_info()
      ->mutable_control_component_debug()
      ->set_is_auto(is_auto_);
}

/**
 * @brief 发布控制交互消息
 *
 * 功能说明：
 * 发布控制交互消息到交互话题
 */
void ControlComponent::PublishControlInteractiveMsg() {
  auto control_interactive_msg = injector_->control_interactive_info();
  common::util::FillHeader(node_->Name(), &control_interactive_msg);
  ADEBUG << "control interactive msg is: "
         << control_interactive_msg.ShortDebugString();
  control_interactive_writer_->Write(control_interactive_msg);
}

}  // namespace control
}  // namespace apollo