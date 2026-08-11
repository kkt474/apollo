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
 * @file stage_approaching_parking_spot.cc
 * @brief 接近停车位阶段实现文件
 *
 * 功能说明：
 * 自主泊车场景中的"接近停车位"阶段
 * 这是泊车流程的第二个阶段
 *
 * 工作流程：
 * 1. 初始化阶段配置和上下文
 * 2. 设置目标停车位信息到规划帧
 * 3. 处理目的障碍物（忽略其决策）
 * 4. 在参考线上执行任务
 * 5. 检查车辆是否已停在合适位置
 * 6. 决定是否进入下一阶段（PARKING）
 *
 * 应用场景：
 * - 自主泊车（Valet Parking）
 * - 停车场自动泊入
 * - 召唤车辆到指定停车位
 */

/**
 * @brief 头文件包含
 *
 * C++标准库头文件：
 * - <string>：标准库字符串类，提供std::string类型
 *
 * Apollo组件头文件：
 * - vehicle_config_helper.h：车辆配置助手，用于获取车辆参数
 * - vehicle_state_provider.h：车辆状态提供者，实时获取车辆状态
 * - stage_approaching_parking_spot.h：当前阶段的定义
 */
#include <string>

/**
 * @brief Apollo车辆配置助手头文件
 *
 * 功能说明：
 * - VehicleConfigHelper类：
 *   车辆配置的单一实例管理器
 *   提供对车辆参数的访问接口
 *   如：车辆尺寸、最大速度、加速度等
 *
 * C++语法说明：
 * - Instance()：
 *   单例模式的获取实例方法
 *   返回全局唯一的VehicleConfigHelper实例
 */
#include "modules/common/configs/vehicle_config_helper.h"

/**
 * @brief Apollo车辆状态提供者头文件
 *
 * 功能说明：
 * - VehicleStateProvider类：
 *   从底层系统获取实时车辆状态
 *   如：位置、速度、加速度、航向角等
 *
 * C++语法说明：
 * - injector_->vehicle_state()：
 *   通过依赖注入器获取车辆状态服务
 *   这是服务定位器模式的实现
 */
#include "modules/common/vehicle_state/vehicle_state_provider.h"

/**
 * @brief 当前阶段的头文件
 *
 * 功能说明：
 * - StageApproachingParkingSpot类定义
 * - ValetParkingContext结构定义
 * - StagePipeline配置定义
 *
 * C++语法说明：
 * - "modules/planning/..."：
 *   使用引号表示自定义头文件路径
 *   编译器从当前文件目录开始搜索
 */
#include "modules/planning/scenarios/valet_parking/stage_approaching_parking_spot.h"

/**
 * @namespace apollo::planning
 * @brief Apollo规划模块命名空间
 *
 * C++语法说明：
 * - namespace apollo：
 *   Apollo最外层命名空间
 * - namespace planning：
 *   规划子命名空间
 * - 可以使用缩进格式或单行格式
 */
namespace apollo {
namespace planning {

/**
 * @brief 初始化接近停车位阶段
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
 * 算法流程：
 * 1. 调用基类Stage的Init进行基础初始化
 * 2. 从上下文中获取泊车场景配置
 * 3. 复制配置到成员变量
 *
 * C++语法说明：
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
 * - if (!Stage::Init(...))：
 *   作用域解析运算符::调用基类方法
 *   逻辑非运算符!判断返回值
 *
 * - GetContextAs<ValetParkingContext>()：
 *   模板方法，将上下文转换为具体类型
 *   - GetContext()返回基类Context指针
 *   - GetContextAs<T>()进行类型转换
 *   - 编译时类型安全检查
 *
 * - scenario_config_.CopyFrom(...)：
 *   Protobuf消息的CopyFrom方法
 *   深拷贝另一个消息的所有字段
 *
 * - return false / return true：
 *   返回bool值表示初始化成功/失败
 */
bool StageApproachingParkingSpot::Init(
    const StagePipeline& config,
    const std::shared_ptr<DependencyInjector>& injector,
    const std::string& config_dir, void* context) {
  // 调用基类Stage的Init进行基础初始化
  // 包括：保存配置、注入器、配置目录等
  if (!Stage::Init(config, injector, config_dir, context)) {
    return false;  // 基类初始化失败，返回false
  }

  // 从上下文中获取泊车场景配置并复制到成员变量
  // GetContextAs<ValetParkingContext>()：
  //   将通用上下文指针转换为ValetParkingContext类型
  //   -> 访问箭头运算符，先解引用GetContextAs()的返回值，再访问scenario_config
  //   .CopyFrom(...)：深拷贝配置
  scenario_config_.CopyFrom(
      GetContextAs<ValetParkingContext>()->scenario_config);

  return true;  // 初始化成功
}

/**
 * @brief 处理接近停车位阶段的核心逻辑
 *
 * @param planning_init_point 规划起始点（车辆当前位置）
 * @param frame 规划帧数据，包含当前帧的所有规划信息
 * @return StageResult 阶段执行结果
 *
 * 功能说明：
 * 这是阶段的主要执行逻辑
 * 负责任务调度、状态检查和流程控制
 *
 * 算法流程：
 * 1. 检查目标停车位ID是否有效
 * 2. 设置目标停车位信息到规划帧
 * 3. 设置预停车标志和点
 * 4. 处理目的障碍物（忽略其决策）
 * 5. 在参考线上执行任务
 * 6. 更新上下文的预停车状态
 * 7. 检查是否完成（车辆已停止）
 * 8. 返回执行结果
 *
 * C++语法说明：
 * - const common::TrajectoryPoint& planning_init_point：
 *   common::TrajectoryPoint：Apollo的轨迹点类型
 *   const&：常量引用，避免拷贝
 *
 * - Frame* frame：
 *   原始指针，用于修改规划帧数据
 *   使用指针而非引用，因为frame可能需要重新赋值
 *
 * - ADEBUG << "stage: StageApproachingParkingSpot"：
 *   Apollo调试日志宏
 *   输出阶段名称便于调试追踪
 *
 * - CHECK_NOTNULL(frame)：
 *   断言宏，确保指针非空
 *   如果为空，程序会报错并终止
 *
 * - auto scenario_context = GetContextAs<ValetParkingContext>()：
 *   auto：自动类型推导
 *   GetContextAs<>()：模板方法，转换上下文类型
 *
 * - scenario_context->target_parking_spot_id.empty()：
 *   ->：访问智能指针或迭代器指向的对象
 *   .empty()：检查字符串是否为空
 *
 * - return result.SetStageStatus(StageStatusType::ERROR)：
 *   设置阶段状态为错误并返回
 *   链式调用风格
 */
StageResult StageApproachingParkingSpot::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  // 输出调试日志，标记阶段名称
  ADEBUG << "stage: StageApproachingParkingSpot";

  // 断言检查frame指针非空
  // 这是一个防御性编程技巧
  CHECK_NOTNULL(frame);

  // 创建阶段结果对象
  StageResult result;

  // 获取泊车场景上下文
  // auto自动推导为ValetParkingContext*类型
  auto scenario_context = GetContextAs<ValetParkingContext>();

  /**
   * 检查目标停车位ID是否有效
   * 如果为空，说明没有有效的停车目标
   * 这是严重错误，返回ERROR状态
   */
  if (scenario_context->target_parking_spot_id.empty()) {
    // 设置阶段状态为错误
    return result.SetStageStatus(StageStatusType::ERROR);
  }

  /**
   * 设置目标停车位ID到规划帧
   *
   * frame->mutable_open_space_info()：
   *   mutable_前缀：返回可修改的指针
   *   open_space_info：开放空间信息
   *
   * mutable_target_parking_spot_id()：
   *   mutable_前缀：返回可修改的字段指针
   *   用于设置目标停车位ID
   *
   * scenario_context->target_parking_spot_id：
   *   从上下文获取目标停车位ID
   */
  *(frame->mutable_open_space_info()->mutable_target_parking_spot_id()) =
      scenario_context->target_parking_spot_id;

  /**
   * 设置预停车立即停止标志
   *
   * set_pre_stop_rightaway_flag：
   *   设置是否需要预停车
   *   用于在到达停车位前减速
   *
   * scenario_context->pre_stop_rightaway_flag：
   *   从上下文获取预停车标志
   */
  frame->mutable_open_space_info()->set_pre_stop_rightaway_flag(
      scenario_context->pre_stop_rightaway_flag);

  /**
   * 设置预停车点
   *
   * mutable_pre_stop_rightaway_point()：
   *   获取可修改的预停车点指针
   *
   * scenario_context->pre_stop_rightaway_point：
   *   从上下文获取预停车点的位置
   */
  *(frame->mutable_open_space_info()->mutable_pre_stop_rightaway_point()) =
      scenario_context->pre_stop_rightaway_point;

  /**
   * 处理参考线上的目的障碍物
   *
   * 目的障碍物是起点/终点处的虚拟障碍物
   * 在泊车场景中，我们需要忽略其对规划的影响
   *
   * 遍历所有参考线进行处理
   */
  auto* reference_lines = frame->mutable_reference_line_info();
  for (auto& reference_line : *reference_lines) {
    // 获取当前参考线的路径决策
    auto* path_decision = reference_line.path_decision();

    // 检查路径决策是否有效
    if (nullptr == path_decision) {
      continue;  // 无效，跳过当前参考线
    }

    // 在路径决策中查找目的障碍物
    // FLAGS_destination_obstacle_id：
    //   全局flag，存储目的障碍物的ID
    auto* dest_obstacle = path_decision->Find(FLAGS_destination_obstacle_id);

    // 检查目的障碍物是否存在
    if (nullptr == dest_obstacle) {
      continue;  // 不存在，跳过
    }

    /**
     * 忽略目的障碍物的决策
     *
     * mutable_ignore()：
     *   获取ignore决策的可修改指针
     *   ignore决策表示完全忽略该障碍物
     *
     * EraseDecision()：
     *   清除该障碍物上的所有现有决策
     *
     * AddLongitudinalDecision(...)：
     *   添加纵向决策
     *   参数1：决策ID（字符串描述）
     *   参数2：决策类型
     */
    ObjectDecisionType decision;
    decision.mutable_ignore();  // 设置为ignore类型
    dest_obstacle->EraseDecision();  // 清除现有决策
    dest_obstacle->AddLongitudinalDecision("ignore-dest-in-valet-parking",
                                           decision);  // 添加忽略决策
  }

  /**
   * 在参考线上执行任务
   *
   * ExecuteTaskOnReferenceLine：
   *   Stage类的核心方法
   *   在所有参考线上执行规划任务
   *   返回执行结果
   */
  result = ExecuteTaskOnReferenceLine(planning_init_point, frame);

  /**
   * 更新上下文的预停车状态
   *
   * 将当前帧的预停车状态保存回上下文
   * 这样下一个阶段/帧可以访问这些信息
   */
  scenario_context->pre_stop_rightaway_flag =
      frame->open_space_info().pre_stop_rightaway_flag();
  scenario_context->pre_stop_rightaway_point =
      frame->open_space_info().pre_stop_rightaway_point();

  /**
   * 检查车辆是否已停止
   *
   * CheckADCStop(*frame)：
   *   检查自车是否停在合适位置
   *   如果返回true，说明已完成当前阶段
   */
  if (CheckADCStop(*frame)) {
    // 设置下一阶段为"VALET_PARKING_PARKING"（泊入阶段）
    next_stage_ = "VALET_PARKING_PARKING";
    // 设置阶段状态为完成
    return StageResult(StageStatusType::FINISHED);
  }

  /**
   * 检查执行结果是否有错误
   *
   * result.HasError()：
   *   检查结果对象是否包含错误
   *   包括：规划失败、超时、碰撞等
   */
  if (result.HasError()) {
    // 输出错误日志
    AERROR << "StopSignUnprotectedStagePreStop planning error";
    // 设置阶段状态为错误
    return result.SetStageStatus(StageStatusType::ERROR);
  }

  /**
   * 阶段正在运行中
   *
   * 返回RUNNING状态
   * 表示阶段尚未完成，需要继续执行
   */
  return result.SetStageStatus(StageStatusType::RUNNING);
}

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
 * - const Frame& frame：
 *   常量引用，传入规划帧
 *   使用引用避免拷贝大型结构
 *   const保证不会修改frame
 */
bool StageApproachingParkingSpot::CheckADCStop(const Frame& frame) {
  /**
   * 获取参考线信息
   *
   * frame.reference_line_info()：
   *   返回参考线信息列表的引用
   *   .front()：获取第一个参考线
   *   这是在没有其他参考线时的简化处理
   */
  const auto& reference_line_info = frame.reference_line_info().front();

  /**
   * 获取自车当前速度
   *
   * injector_->vehicle_state()：
   *   injector_是依赖注入器指针
   *   -> 调用get()返回的原始指针的vehicle_state()方法
   *   .linear_velocity()：获取线速度
   */
  const double adc_speed = injector_->vehicle_state()->linear_velocity();

  /**
   * 获取最大停止速度阈值
   *
   * common::VehicleConfigHelper::Instance()：
   *   获取车辆配置助手的单例实例
   *
   * .GetConfig()：
   *   获取车辆配置消息
   *
   * .vehicle_param()：
   *   获取车辆参数子消息
   *
   * .max_abs_speed_when_stopped()：
   *   获取车辆静止时的最大绝对速度
   *   通常是一个很小的值，如0.1 m/s
   *
   * C++语法说明：
   * - const double：
   *   const double类型，表示速度值不会被修改
   */
  const double max_adc_stop_speed = common::VehicleConfigHelper::Instance()
                                        ->GetConfig()
                                        .vehicle_param()
                                        .max_abs_speed_when_stopped();

  /**
   * 检查车辆是否静止
   *
   * 如果当前速度大于最大停止速度阈值
   * 说明车辆仍在移动，未停止
   */
  if (adc_speed > max_adc_stop_speed) {
    ADEBUG << "ADC not stopped: speed[" << adc_speed << "]";
    return false;  // 车辆未停止
  }

  /**
   * 检查停止位置是否合适
   *
   * 获取车辆前边缘在参考线上的s坐标
   * AdcSlBoundary()：
   *   自车的SL边界框
   * .end_s()：
   *   边界框在s方向的最大值（前端）
   */
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();

  /**
   * 获取停车线（停车围栏）的起始s坐标
   *
   * frame.open_space_info()：
   *   开放空间信息
   *
   * .open_space_pre_stop_fence_s()：
   *   预停车围栏的s坐标
   *   这是规划算法计算的理想停车位置
   */
  const double stop_fence_start_s =
      frame.open_space_info().open_space_pre_stop_fence_s();

  /**
   * 计算车辆前边缘到停车线的距离
   *
   * 距离 = 停车线位置 - 车辆前端位置
   * 正值表示车辆在停车线后方（未到达）
   * 负值表示车辆已越过停车线（已超过）
   */
  const double distance_stop_line_to_adc_front_edge =
      stop_fence_start_s - adc_front_edge_s;

  /**
   * 检查距离是否在有效范围内
   *
   * scenario_config_.max_valid_stop_distance()：
   *   配置中定义的最大有效停车距离
   *   如果距离超过此值，说明停车位置不准确
   *
   * 有效停车条件：
   * - 车辆必须在停车线后方（distance >= 0理论上，但有容差）
   * - 车辆不能离停车线太远（距离 < max_valid_stop_distance）
   */
  if (distance_stop_line_to_adc_front_edge >
      scenario_config_.max_valid_stop_distance()) {
    ADEBUG << "not a valid stop. too far from stop line.";
    return false;  // 停车位置无效，距离太远
  }

  /**
   * 所有检查通过
   *
   * - 车辆速度足够低（已停止）
   * - 停车位置在有效范围内
   *
   * 返回true，表示自车已合适地停止
   */
  return true;
}

/**
 * @namespace命名空间结束
 *
 * C++语法说明：
 * - }  // namespace planning：
 *   结束planning命名空间
 * - }  // namespace apollo：
 *   结束apollo命名空间
 *
 * 注释中的命名空间名称便于阅读和导航
 */
}  // namespace planning
}  // namespace apollo