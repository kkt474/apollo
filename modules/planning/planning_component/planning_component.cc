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
 * @file planning_component.cc
 * @brief 规划组件实现文件
 *
 * 本文件是Apollo自动驾驶规划模块的核心组件实现。
 * 规划组件负责：
 * 1. 接收感知、定位、底盘等输入数据
 * 2. 根据路由请求计算安全高效的行驶轨迹
 * 3. 向控制模块发布轨迹数据
 *
 * 语法说明：
 * - apollo::common::math::Vec2d: Apollo自定义的二维向量类，用于表示位置坐标
 * - std::shared_ptr<T>: C++11智能指针，表示共享所有权指针，引用计数为0时自动释放
 * - std::unique_ptr<T>: C++11独占所有权的智能指针，不可复制只能移动
 * - std::mutex: C++11线程互斥锁，用于保护多线程共享数据
 * - std::lock_guard<std::mutex>: RAII风格的锁守卫，构造时加锁，析构时自动解锁
 * - lambda表达式: [this](const std::shared_ptr<T>& x) {...} 用于创建回调函数
 * - auto: C++11关键字，编译器自动推断变量类型
 */

#include "modules/planning/planning_component/planning_component.h"

/**
 * cyber::common::file - Apollo Cyber RT框架的文件操作工具
 * 用于读取配置文件、日历等操作
 */
#include "cyber/common/file.h"

/**
 * common::adapters::adapter_gflags - Apollo的适配器配置标志
 * 定义了各模块间的通信话题名称等配置
 */
#include "modules/common/adapters/adapter_gflags.h"

/**
 * common::configs::config_gflags - Apollo全局配置标志
 * 包含各种运行时配置选项，如FLAGS_use_navigation_mode等
 */
#include "modules/common/configs/config_gflags.h"

/**
 * common::math::Vec2d - 二维向量类
 * 用于表示二维平面上的点或向量，提供x、y坐标以及向量运算功能
 */
#include "modules/common/math/vec2d.h"

/**
 * common::util::message_util - 消息工具类
 * 提供FillHeader等函数用于填充消息头部的公共字段（如时间戳、序列号等）
 */
#include "modules/common/util/message_util.h"

/**
 * common::util::util - 通用工具函数集合
 * 包含IsProtoEqual等实用工具函数
 */
#include "modules/common/util/util.h"

/**
 * map::hdmap::hdmap_util - 高精地图工具类
 * 提供访问高精地图的接口，如HDMapUtil::BaseMapPtr()获取地图指针
 */
#include "modules/map/hdmap/hdmap_util.h"

/**
 * planning_base::common::history - 历史轨迹记录器
 * 用于记录规划的历史轨迹数据，支持回放和分析
 */
#include "modules/planning/planning_base/common/history.h"

/**
 * planning_base::common::planning_context - 规划上下文
 * 存储规划的内部状态信息，如重路由状态等
 */
#include "modules/planning/planning_base/common/planning_context.h"

/**
 * planning_base::common::util - 规划基础工具函数
 */
#include "modules/planning/planning_base/common/util/util.h"

/**
 * planning_component::navi_planning - 导航模式规划类
 * 用于导航模式下的轨迹规划（不基于车道线的规划方式）
 */
#include "modules/planning/planning_component/navi_planning.h"

/**
 * planning_component::on_lane_planning - 车道线模式规划类
 * 用于基于车道线的规划模式，是主要的规划方式
 */
#include "modules/planning/planning_component/on_lane_planning.h"

namespace apollo {
/**
 * apollo:: - Apollo项目的最外层命名空间
 * 所有Apollo相关代码都位于此命名空间下
 */
namespace planning {

/**
 * using声明 - 将其他命名空间中的类型引入当前作用域
 * 语法：using 命名空间::类型名;
 * 之后可以直接使用Vec2d而不是apollo::common::math::Vec2d
 */
using apollo::common::math::Vec2d;          /**< 二维向量类型，用于坐标表示 */
using apollo::cyber::ComponentBase;        /**< Cyber RT组件基类，提供组件通用功能 */

/**
 * HDMapUtil::BaseMapPtr() - 高精地图单例访问接口
 * 返回指向高精地图的指针，用于查询道路、车道等信息
 */
using apollo::hdmap::HDMapUtil;

/**
 * control::ControlInteractiveMsg - 控制交互消息
 * 用于规划模块与控制模块之间的交互通信
 */
using apollo::control::ControlInteractiveMsg;

/**
 * perception::TrafficLightDetection - 感知模块的红绿灯检测结果
 * 包含红绿灯的位置、颜色、置信度等信息
 */
using apollo::perception::TrafficLightDetection;

/**
 * relative_map::MapMsg - 相对地图消息
 * 在导航模式下使用，提供车辆周围的相对地图信息
 */
using apollo::relative_map::MapMsg;

/**
 * routing::RoutingRequest - 路由请求
 * 用户或系统发出的路径规划请求，包含起点、终点、途经点等
 */
using apollo::routing::RoutingRequest;

/**
 * routing::RoutingResponse - 路由响应
 * 路由模块返回的路径规划结果，包含一系列导航车道信息
 */
using apollo::routing::RoutingResponse;

/**
 * storytelling::Stories - 故事/场景描述消息
 * 包含当前驾驶场景的描述信息，如换道、让行等行为
 */
using apollo::storytelling::Stories;

/**
 * @brief 规划组件初始化函数
 *
 * 这是Cyber RT组件的Init入口函数，在组件启动时调用。
 * 主要完成以下初始化工作：
 * 1. 创建依赖注入器(DependencyInjector)
 * 2. 根据配置选择NaviPlanning或OnLanePlanning
 * 3. 加载规划配置文件
 * 4. 初始化消息处理器（学习模式）
 * 5. 创建各话题的读者(Reader)用于接收输入数据
 * 6. 创建轨迹发布者(Writer)用于输出规划结果
 *
 * @return bool 初始化是否成功
 *
 * 语法说明：
 * - std::make_shared<T>(args): C++11创建shared_ptr的工厂函数
 * - std::make_unique<T>(args): C++14创建unique_ptr的工厂函数
 * - node_->CreateReader<T>(topic, callback): Cyber RT创建话题读者的API
 * - node_->CreateWriter<T>(topic): Cyber RT创建话题发布者的API
 * - [this](const std::shared_ptr<T>& x) {...}: lambda表达式作为回调函数，捕获this指针
 * - std::lock_guard<std::mutex>: 线程安全锁，保证多线程下数据访问安全
 */
bool PlanningComponent::Init() {
  /**
   * injector_ - 依赖注入器
   * 创建规划模块所需的各种依赖对象，如车辆状态、参考线生成器等
   * std::make_shared<T>(injector_)使用完美转发传递injector构造DependencyInjector
   */
  injector_ = std::make_shared<DependencyInjector>();

  /**
   * FLAGS_use_navigation_mode - 配置标志，决定使用哪种规划模式
   * true: 使用NaviPlanning（导航模式），不依赖固定车道线
   * false: 使用OnLanePlanning（车道线模式），基于高清地图车道线规划
   *
   * std::make_unique<T>(injector_)创建独占所有权的智能指针
   * unique_ptr不允许复制，确保对象唯一性
   */
  if (FLAGS_use_navigation_mode) {
    planning_base_ = std::make_unique<NaviPlanning>(injector_);
  } else {
    planning_base_ = std::make_unique<OnLanePlanning>(injector_);
  }

  /**
   * ACHECK宏 - Apollo的断言检查宏，类似assert但会输出更详细的错误信息
   * ACHECK(ComponentBase::GetProtoConfig(&config_))表示：
   *   尝试从配置文件加载PlanningConfig到config_变量
   *   如果失败则输出错误并终止程序
   *
   * ComponentBase::ConfigFilePath() - 获取组件配置文件的路径
   * 配置文件通常为*.conf文件，位于modules/planning/conf/目录下
   */  //反序列化
  ACHECK(ComponentBase::GetProtoConfig(&config_))
      << "failed to load planning config file "
      << ComponentBase::ConfigFilePath();

  /**
   * FLAGS_planning_offline_learning - 是否使用离线学习模式
   * config_.learning_mode() - 获取配置的在线学习模式
   * PlanningConfig::NO_LEARNING - 不使用学习功能
   *
   * message_process_.Init() - 初始化消息处理器
   * 在学习模式下需要处理更多的数据采集和预处理
   */
  if (FLAGS_planning_offline_learning ||
      config_.learning_mode() != PlanningConfig::NO_LEARNING) {
    if (!message_process_.Init(config_, injector_)) {
      AERROR << "failed to init MessageProcess";  /**< AERROR输出错误日志 */
      return false;
    }
  }

  /**
   * planning_base_->Init(config_) - 调用具体规划器的初始化
   * 根据前面选择的NaviPlanning或OnLanePlanning执行对应的初始化
   */
  planning_base_->Init(config_);

  /**
   * CreateReader<PlanningCommand> - 创建规划命令读者
   * PlanningCommand包含用户或外部系统发出的驾驶命令
   *
   * 回调函数lambda表达式说明：
   * [this](const std::shared_ptr<PlanningCommand>& planning_command) {...}
   * - [this]: 捕获列表，表示捕获当前对象的this指针
   * - const std::shared_ptr<PlanningCommand>&: 常量引用，避免不必要的拷贝
   * - 函数体中使用std::lock_guard加锁，保证线程安全
   * - planning_command_.CopyFrom(*planning_command): 深拷贝消息内容
   */
  planning_command_reader_ = node_->CreateReader<PlanningCommand>(
      config_.topic_config().planning_command_topic(),
      [this](const std::shared_ptr<PlanningCommand>& planning_command) {
        AINFO << "Received planning data: run planning callback."
              << planning_command->header().DebugString();
        std::lock_guard<std::mutex> lock(mutex_);  /**< RAII锁，构造加锁析构解锁 */
        planning_command_.CopyFrom(*planning_command);
      });

  /**
   * CreateReader<TrafficLightDetection> - 创建红绿灯检测结果读者
   * 接收来自感知模块的红绿灯识别结果
   *
   * ADEBUG - Apollo调试级别日志，仅在调试模式输出
   */
  traffic_light_reader_ = node_->CreateReader<TrafficLightDetection>(
      config_.topic_config().traffic_light_detection_topic(),
      [this](const std::shared_ptr<TrafficLightDetection>& traffic_light) {
        ADEBUG << "Received traffic light data: run traffic light callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        traffic_light_.CopyFrom(*traffic_light);
      });

  /**
   * CreateReader<PadMessage> - 创建Pad消息读者
   * PadMessage通常来自车载交互设备（如方向盘旁边的控制pad）
   * 用于接收驾驶员的即时控制命令
   */
  pad_msg_reader_ = node_->CreateReader<PadMessage>(
      config_.topic_config().planning_pad_topic(),
      [this](const std::shared_ptr<PadMessage>& pad_msg) {
        ADEBUG << "Received pad data: run pad callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        pad_msg_.CopyFrom(*pad_msg);
      });

  /**
   * CreateReader<Stories> - 创建故事/场景消息读者
   * Stories包含当前驾驶场景的描述和状态
   */
  story_telling_reader_ = node_->CreateReader<Stories>(
      config_.topic_config().story_telling_topic(),
      [this](const std::shared_ptr<Stories>& stories) {
        ADEBUG << "Received story_telling data: run story_telling callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        stories_.CopyFrom(*stories);
      });

  /**
   * CreateReader<ControlInteractiveMsg> - 创建控制交互消息读者
   * 接收控制模块反馈的交互信息
   */
  control_interactive_reader_ = node_->CreateReader<ControlInteractiveMsg>(
      config_.topic_config().control_interative_topic(),
      [this](const std::shared_ptr<ControlInteractiveMsg>&
                 control_interactive_msg) {
        ADEBUG << "Received story_telling data: run story_telling callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        control_interactive_msg_.CopyFrom(*control_interactive_msg);
      });

  /**
   * relative_map_reader_ - 相对地图读者
   * 仅在导航模式下创建，因为导航模式不依赖高清地图而使用相对地图
   */
  if (FLAGS_use_navigation_mode) {
    relative_map_reader_ = node_->CreateReader<MapMsg>(
        config_.topic_config().relative_map_topic(),
        [this](const std::shared_ptr<MapMsg>& map_message) {
          ADEBUG << "Received relative map data: run relative map callback.";
          std::lock_guard<std::mutex> lock(mutex_);
          relative_map_.CopyFrom(*map_message);
        });
  }

  /**
   * CreateWriter<ADCTrajectory> - 创建轨迹发布者
   * ADCTrajectory (Autonomous Driving Computer Trajectory) 自动驾驶轨迹
   * 包含轨迹点序列、速度曲线、决策信息等
   * 这是规划模块最主要输出，供给控制模块执行
   */
  planning_writer_ = node_->CreateWriter<ADCTrajectory>(
      config_.topic_config().planning_trajectory_topic());

  /**
   * CreateClient<TRequest, TResponse> - 创建RPC客户端
   * 用于向路由模块发送重路由请求
   * LaneFollowCommand: 车道跟随命令类型
   * CommandStatus: 命令执行状态响应
   */
  rerouting_client_ =
      node_->CreateClient<apollo::external_command::LaneFollowCommand,
                          external_command::CommandStatus>(
          config_.topic_config().routing_request_topic());

  /**
   * CreateWriter<PlanningLearningData> - 创建学习数据发布者
   * 用于发布训练数据到学习系统
   */
  planning_learning_data_writer_ = node_->CreateWriter<PlanningLearningData>(
      config_.topic_config().planning_learning_data_topic());

  /**
   * CreateWriter<CommandStatus> - 创建命令状态发布者
   * 向外部系统报告命令执行状态（RUNNING/FINISHED/ERROR）
   * FLAGS_planning_command_status: 命令状态话题的配置标志
   */
  command_status_writer_ = node_->CreateWriter<external_command::CommandStatus>(
      FLAGS_planning_command_status);

  return true;  /**< 初始化成功返回 */
}

/**
 * @brief 规划组件的主处理函数（Proc）
 *
 * 这是Cyber RT组件的主回调函数，每个规划周期被调用一次。
 * 典型调用频率为10Hz（每100ms一次）。
 *
 * @param prediction_obstacles 感知模块输出的障碍物预测结果
 * @param chassis 车辆底盘状态（速度、加速度、方向盘角度等）
 * @param localization_estimate 定位模块输出的自车位置和姿态
 * @return bool 处理是否成功
 *
 * 语法说明：
 * - const std::shared_ptr<T>&: 常量引用参数，避免拷贝
 * - std::shared_ptr<T>：共享指针，引用计数管理生命周期
 * - mutable_xxx(): Protobuf消息的mutable访问器，返回可写指针
 * - CopyFrom(): Protobuf消息的深拷贝方法
 */
bool PlanningComponent::Proc(
    const std::shared_ptr<prediction::PredictionObstacles>&
        prediction_obstacles,
    const std::shared_ptr<canbus::Chassis>& chassis,
    const std::shared_ptr<localization::LocalizationEstimate>&
        localization_estimate) {

  /**
   * ACHECK断言检查 - 确保prediction_obstacles指针有效
   * 如果为空会记录错误并返回false
   */
  ACHECK(prediction_obstacles != nullptr);

  /**
   * CheckRerouting() - 检查是否需要重路由
   * 当车辆偏离原定路线或遇到无法通行的情况时触发
   */
  // check and process possible rerouting request
  CheckRerouting();

  /**
   * local_view_ - 本地视图结构体
   * 整合本周期所有输入数据，便于传递给RunOnce
   * 包含：prediction_obstacles, chassis, localization_estimate, traffic_light等
   */
  // process fused input data
  local_view_.prediction_obstacles = prediction_obstacles;
  local_view_.chassis = chassis;
  local_view_.localization_estimate = localization_estimate;

  /**
   * std::lock_guard<std::mutex> - 线程安全访问保护
   * 使用大括号限定作用域，析构时自动释放锁
   *
   * planning_command_是共享变量，需要互斥保护 因为 planning_command_ 由 Reader 回调在 I/O 线程写入
   * IsProtoEqual()比较两个消息的header是否相同，避免重复处理
   */
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!local_view_.planning_command ||
        !common::util::IsProtoEqual(local_view_.planning_command->header(),
                                    planning_command_.header())) {
      local_view_.planning_command =
          std::make_shared<PlanningCommand>(planning_command_);
    }
  }

  /**
   * traffic_light_和relative_map_的更新
   * 使用make_shared创建新的共享指针副本
   */
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.traffic_light =
        std::make_shared<TrafficLightDetection>(traffic_light_);
    local_view_.relative_map = std::make_shared<MapMsg>(relative_map_);
  }

  /**
   * PadMessage处理逻辑
   * CLEAR_PLANNING是一个特殊命令，用于清除当前规划
   * 例如驾驶员接管控制时需要清除原规划
   */
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!local_view_.pad_msg ||
        !common::util::IsProtoEqual(local_view_.pad_msg->header(),
                                    pad_msg_.header())) {
      // Check if "CLEAR_PLANNING" PadMessage is received and process.
      if (pad_msg_.action() == PadMessage::CLEAR_PLANNING) {
        local_view_.planning_command = nullptr;   /**< 清除规划命令 */
        planning_command_.Clear();                /**< 重置存储的命令 */
      }
      local_view_.pad_msg = std::make_shared<PadMessage>(pad_msg_);
    }
  }

  /**
   * Stories消息更新
   */
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.stories = std::make_shared<Stories>(stories_);
  }

  /**
   * ControlInteractiveMsg处理
   * 检查消息头部是否有变化，有变化才更新
   */
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!local_view_.control_interactive_msg ||
        !common::util::IsProtoEqual(
            local_view_.control_interactive_msg->header(),
            control_interactive_msg_.header())) {
      local_view_.control_interactive_msg =
          std::make_shared<ControlInteractiveMsg>(control_interactive_msg_);
    }
  }

  /**
   * CheckInput() - 输入数据有效性检查
   * 检查定位、底盘、地图等关键数据是否就绪
   */
  if (!CheckInput()) {
    AINFO << "Input check failed";
    return false;
  }

  /**
   * 在线学习模式的数据处理
   * 当config_.learning_mode()不是NO_LEARNING时执行
   * message_process_.OnXxx()系列函数处理并存储训练数据
   */
  if (config_.learning_mode() != PlanningConfig::NO_LEARNING) {
    // data process for online training
    message_process_.OnChassis(*local_view_.chassis);          /**< 处理底盘数据 */
    message_process_.OnPrediction(*local_view_.prediction_obstacles); /**< 处理预测数据 */
    if (local_view_.planning_command->has_lane_follow_command()) {
      message_process_.OnRoutingResponse(
          local_view_.planning_command->lane_follow_command());
    }
    message_process_.OnStoryTelling(*local_view_.stories);    /**< 处理场景故事数据 */
    message_process_.OnTrafficLightDetection(*local_view_.traffic_light); /**< 处理红绿灯数据 */
    message_process_.OnLocalization(*local_view_.localization_estimate); /**< 处理定位数据 */
  }

  /**
   * 强化学习测试模式（RL_TEST）
   * 生成学习数据帧并发布，不执行实际规划
   */
  // publish learning data frame for RL test
  if (config_.learning_mode() == PlanningConfig::RL_TEST) {
    PlanningLearningData planning_learning_data;
    LearningDataFrame* learning_data_frame =
        injector_->learning_based_data()->GetLatestLearningDataFrame();
    if (learning_data_frame) {
      planning_learning_data.mutable_learning_data_frame()->CopyFrom(
          *learning_data_frame);
      common::util::FillHeader(node_->Name(), &planning_learning_data);
      planning_learning_data_writer_->Write(planning_learning_data);
    } else {
      AERROR << "fail to generate learning data frame";
      return false;
    }
    return true;  /**< RL_TEST模式直接返回，不执行后续规划 */
  }

  /**
   * ADCTrajectory - 自动驾驶轨迹消息
   * 这是规划的核心输出数据结构
   *
   * planning_base_->RunOnce() - 执行单次轨迹规划
   * 输入：local_view_（所有输入数据）
   * 输出：adc_trajectory_pb（计算得到的轨迹）
   */
  ADCTrajectory adc_trajectory_pb;
  planning_base_->RunOnce(local_view_, &adc_trajectory_pb);

  /**
   * auto关键字 - 编译器自动推断start_time类型为double
   * timestamp_sec()返回时间戳（秒）
   * 记录规划开始时间，用于后续计算相对时间
   */
  auto start_time = adc_trajectory_pb.header().timestamp_sec();  // 保存原始时间戳

  /**
   * FillHeader() - 填充消息公共头部
   * node_->Name()获取本节点名称
   * 填充内容包括：时间戳、序列号、模块名等
   */
  common::util::FillHeader(node_->Name(), &adc_trajectory_pb);
  // 用当前时间覆盖 header 中的时间戳，因此需要先保存原始值
  /**
   * SetLocation() - 设置轨迹的位置信息
   * 将自车位置和车道边界信息写入轨迹消息
   */
  SetLocation(&adc_trajectory_pb);

  /**
   * 时间戳修正
   * 由于FillHeader可能修改了timestamp_sec，需要调整轨迹点的relative_time
   *
   * mutable_trajectory_point() - 返回轨迹点列表的可写指针
   * for (auto& p : ...) - 范围for循环，auto&表示引用避免拷贝
   * p.set_relative_time() - 设置每个轨迹点的相对时间
   */
  // modify trajectory relative time due to the timestamp change in header
  // RunOnce() 中规划的轨迹点时间基准 = start_time（规划开始时刻）
  // FillHeader() 后消息时间基准 = 当前时刻（可能晚了若干ms）
  // dt = start_time - 新时间戳  （通常为负值，因为新时间戳更大）
  const double dt = start_time - adc_trajectory_pb.header().timestamp_sec();
  for (auto& p : *adc_trajectory_pb.mutable_trajectory_point()) {
    p.set_relative_time(p.relative_time() + dt);
  }

  /**
   * Write() - 发布轨迹消息到话题
   * 订阅者（主要是控制模块）会接收到这条轨迹
   */
  planning_writer_->Write(adc_trajectory_pb);

  /**
   * 命令执行状态反馈
   * 向外部系统报告命令执行状态
   */
  // Send command execution feedback.
  // Error occured while executing the command.
  external_command::CommandStatus command_status;
  common::util::FillHeader(node_->Name(), &command_status);

  /**
   * 设置命令ID（如果存在规划命令）
   */
  if (nullptr != local_view_.planning_command) {
    command_status.set_command_id(local_view_.planning_command->command_id());
  }

  /**
   * TrajectoryType - 轨迹类型枚举
   * 用于区分不同类型的轨迹（如正常车道跟随、换道、泊车等）
   */
  ADCTrajectory::TrajectoryType current_trajectory_type =
      adc_trajectory_pb.trajectory_type();

  // 三种状态判定
  /**
   * 判断轨迹状态并设置相应的命令状态
   * - error_code != OK: 发生错误
   * - IsPlanningFinished(): 规划已完成（如到达目的地）
   * - 否则：规划正在执行中
   */
  if (adc_trajectory_pb.header().status().error_code() !=
      common::ErrorCode::OK) {
    command_status.set_status(external_command::CommandStatusType::ERROR);
    command_status.set_message(adc_trajectory_pb.header().status().msg());
  } else if (planning_base_->IsPlanningFinished(current_trajectory_type)) {
    AINFO << "Set the external_command: FINISHED";
    command_status.set_status(external_command::CommandStatusType::FINISHED);
  } else {
    AINFO << "Set the external_command: RUNNING";
    command_status.set_status(external_command::CommandStatusType::RUNNING);
  }
  
  // 发布状态 & 记录历史
  /**
   * 发布命令状态
   */
  command_status_writer_->Write(command_status);

  /**
   * 记录到历史记录
   * injector_->history()获取历史记录器
   * Add()添加当前轨迹到历史
   */
  // record in history
  auto* history = injector_->history();
  history->Add(adc_trajectory_pb);

  return true;  /**< 本周期处理成功 */
}

/**
 * @brief 检查并执行重路由
 *
 * 当车辆偏离规划路径或遇到障碍时，需要发起重路由请求
 * 重路由状态存储在planning_context中
 *
 * 语法说明：
 * - auto* : 原始指针，不拥有对象所有权
 * - mutable_xxx() : 返回可修改的嵌套消息指针
 * - std::make_shared<T>(xxx) : 创建共享指针并拷贝构造
 * - SendRequest() : RPC调用，向路由服务发送请求
 */
void PlanningComponent::CheckRerouting() {
  auto* rerouting = injector_->planning_context()
                        ->mutable_planning_status()
                        ->mutable_rerouting();

  /**
   * need_rerouting() - 检查是否需要重路由的标志
   * 如果不需要，直接返回
   */
  if (!rerouting->need_rerouting()) {
    return;
  }

  /**
   * 填充重路由命令的头部信息
   * FillHeader()添加时间戳、模块名等
   */
  AINFO << "node_->Name(): " << node_->Name();
  common::util::FillHeader(node_->Name(),
                           rerouting->mutable_lane_follow_command());

  /**
   * 创建共享指针并发送RPC请求
   * rerouting_client_->SendRequest()是异步调用
   */
  auto lane_follow_command_ptr =
      std::make_shared<apollo::external_command::LaneFollowCommand>(
          rerouting->lane_follow_command());
  rerouting_client_->SendRequest(lane_follow_command_ptr);

  /**
   * 重置重路由标志，避免重复发送请求
   */
  rerouting->set_need_rerouting(false);
}

/**
 * @brief 检查输入数据有效性
 *
 * 在每个规划周期开始时调用，检查必要的输入数据是否就绪
 * 如果有数据未就绪，会填充decision中的not_ready信息并返回false
 *
 * @return bool 所有检查是否通过
 *
 * 语法说明：
 * - mutable_decision()->mutable_main_decision()->mutable_not_ready()
 *   一系列mutable调用获取嵌套消息的写指针
 * - set_reason() - 设置未就绪的具体原因
 * - nullptr - C++11空指针常量
 */
bool PlanningComponent::CheckInput() {
  ADCTrajectory trajectory_pb;  /**< 创建轨迹消息用于存储检查结果 */

  /**
   * SetLocation()会填充部分轨迹信息
   */
  SetLocation(&trajectory_pb);

  /**
   * 获取not_ready决策的指针
   * 用于填充哪个数据未就绪以及原因
   */
  auto* not_ready = trajectory_pb.mutable_decision()
                        ->mutable_main_decision()
                        ->mutable_not_ready();

  /**
   * 逐一检查关键输入数据
   * - localization_estimate: 定位数据
   * - chassis: 底盘数据
   * - BaseMapPtr(): 高精地图指针
   */
  if (local_view_.localization_estimate == nullptr) {
    not_ready->set_reason("localization not ready");  /**< 定位未就绪 */
  } else if (local_view_.chassis == nullptr) {
    not_ready->set_reason("chassis not ready");       /**< 底盘未就绪 */
  } else if (HDMapUtil::BaseMapPtr() == nullptr) {
    not_ready->set_reason("map not ready");           /**< 地图未就绪 */
  } else {
    // nothing - 所有基础数据就绪
  }

  /**
   * 导航模式 vs 车道线模式的额外检查
   * - 导航模式：需要检查relative_map
   * - 车道线模式：需要检查planning_command
   */
  if (FLAGS_use_navigation_mode) {
    if (!local_view_.relative_map->has_header()) {
      not_ready->set_reason("relative map not ready");
    }
  } else {
    if (!local_view_.planning_command ||
        !local_view_.planning_command->has_header()) {
      not_ready->set_reason("planning_command not ready");
    }
  }

  /**
   * 如果有任何数据未就绪
   * 发布一个空的轨迹并返回false
   */
  if (not_ready->has_reason()) {
    AINFO << not_ready->reason() << "; skip the planning cycle.";
    common::util::FillHeader(node_->Name(), &trajectory_pb);
    planning_writer_->Write(trajectory_pb);
    return false;
  }

  return true;  /**< 所有检查通过 */
}

/**
 * @brief 设置轨迹的位置信息
 *
 * 将自车位置和车道边界信息写入轨迹消息
 *
 * @param ptr_trajectory_pb 指向轨迹消息的指针
 *
 * 语法说明：
 * - mutable_xxx() : 返回可写指针
 * - set_x()/set_y() : 设置坐标值
 * - Vec2d{x, y} : C++11列表初始化构造二维向量
 */
void PlanningComponent::SetLocation(ADCTrajectory* const ptr_trajectory_pb) {
  /**
   * mutable_location_pose() - 获取位置pose的可写指针
   * 然后设置车辆当前位置x、y坐标
   */
  auto p = ptr_trajectory_pb->mutable_location_pose();
  p->mutable_vehice_location()->set_x(
      local_view_.localization_estimate->pose().position().x());
  p->mutable_vehice_location()->set_y(
      local_view_.localization_estimate->pose().position().y());

  /**
   * Vec2d类 - Apollo的二维向量实现
   * {x, y}使用花括号初始化
   */
  const Vec2d& adc_position = {
      local_view_.localization_estimate->pose().position().x(),
      local_view_.localization_estimate->pose().position().y()};
  Vec2d left_point, right_point;

  /**
   * GenerateWidthOfLane() - 生成车道边界宽度
   * 根据自车位置计算左右车道边界点
   */
  if (planning_base_->GenerateWidthOfLane(adc_position, left_point,
                                          right_point)) {
    /**
     * 设置左右车道边界点坐标
     */
    p->mutable_left_lane_boundary_point()->set_x(left_point.x());
    p->mutable_left_lane_boundary_point()->set_y(left_point.y());
    p->mutable_right_lane_boundary_point()->set_x(right_point.x());
    p->mutable_right_lane_boundary_point()->set_y(right_point.y());
  }
}

}  // namespace planning
}  // namespace apollo
