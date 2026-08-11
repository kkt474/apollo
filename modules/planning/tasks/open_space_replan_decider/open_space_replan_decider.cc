/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
 * @file open_space_replan_decider.cc
 *
 * @brief 开源空间重规划决策器实现文件
 *
 * 功能说明：
 * 本文件实现了 OpenSpaceReplanDecider 类
 * 负责决定开放空间场景下是否需要进行轨迹重规划
 * 继承自 Decider 基类
 *
 * 核心概念：
 * - 重规划标志（replan_flag）：指示是否需要重新规划轨迹
 * - 开放空间（Open Space）：停车场、狭窄道路等非结构化场景
 * - 停车判断：基于速度判断车辆是否处于静止状态
 *
 * C++语法说明：
 * - #include：预处理指令，包含头文件
 * - namespace：命名空间，避免命名冲突
 * - class：类声明
 * - ::运算符：作用域解析，类名::函数名表示成员函数
 * - const引用参数：避免拷贝，提高性能
 * - shared_ptr：智能指针，管理对象生命周期
 **/

#include "modules/planning/tasks/open_space_replan_decider/open_space_replan_decider.h"

namespace apollo {
/**
 * @brief Apollo主命名空间
 */
namespace planning {

/**
 * @brief 类型别名声明
 *
 * C++语法说明：
 * - using：类型别名声明，等价于typedef
 * - apollo::common::ErrorCode：Apollo通用错误码枚举
 * - apollo::common::Status：Apollo通用状态类型
 */
using apollo::common::ErrorCode;
using apollo::common::Status;

/**
 * @brief 初始化函数
 *
 * @param config_dir 配置目录路径
 * @param name 任务名称
 * @param injector 依赖注入器指针
 * @return bool 初始化成功返回true
 *
 * 功能说明：
 * 1. 调用基类Decider的Init方法进行基础初始化
 * 2. 加载开源空间重规划决策器配置
 *
 * C++语法说明：
 * - const std::string &config_dir：
 *   常量引用参数，承诺不修改，避免拷贝
 *
 * - const std::shared_ptr<DependencyInjector> &injector：
 *   shared_ptr智能指针的常量引用
 *   shared_ptr：引用计数智能指针
 *   共享所有权，引用计数管理生命周期
 *
 * - Decider::Init(config_dir, name, injector)：
 *   调用基类Decider的Init方法
 *   ::作用域限定符
 *
 * - Decider::LoadConfig<OpenSpaceReplanDeciderConfig>(&config_)：
 *   模板方法调用，加载特定类型的protobuf配置
 *   LoadConfig<T>是模板函数，T是配置类型
 *   &config_：输出参数，存储加载的配置
 */
bool OpenSpaceReplanDecider::Init(
    const std::string &config_dir, const std::string &name,
    const std::shared_ptr<DependencyInjector> &injector) {
  /**
   * @brief 调用基类初始化
   *
   * Decider::Init(...)：
   *   调用父类Decider的Init方法
   *   进行通用的初始化工作
   *   返回false表示初始化失败
   */
  if (!Decider::Init(config_dir, name, injector)) {
    return false;
  }
  /**
   * @brief 加载开源空间重规划决策器配置
   *
   * Decider::LoadConfig<OpenSpaceReplanDeciderConfig>：
   *   模板方法，加载指定类型的protobuf配置
   *   &config_：输出参数，存储加载的配置
   *   返回false表示加载失败
   */
  return Decider::LoadConfig<OpenSpaceReplanDeciderConfig>(&config_);
}

/**
 * @brief 主处理函数
 *
 * @param frame 当前规划帧指针
 * @return Status 处理状态
 *
 * 功能说明：
 * 决策是否需要进行开放空间轨迹重规划
 *
 * 重规划条件（满足任一则重规划）：
 * 1. 上一帧没有优化轨迹数据
 * 2. 车辆处于静止状态（速度小于阈值）
 *
 * C++语法说明：
 * - Frame *frame：
 *   裸指针参数，指向当前规划帧
 *   Frame是规划帧数据结构
 *
 * - const auto&：
 *   常量引用，自动类型推导
 *   用于避免拷贝，提高性能
 *
 * - injector_->frame_history()->Latest()：
 *   链式调用
 *   injector_：成员变量，DependencyInjector指针
 *   ->frame_history()：调用injector的方法
 *   ->Latest()：获取最新的历史帧
 *
 * - if (frame == nullptr || previous_frame == nullptr)：
 *   nullptr检查，防止空指针解引用
 *   || 逻辑或运算符
 *
 * - AERROR << msg：
 *   AERROR：Apollo错误级别日志宏
 *   << 运算符连接日志内容
 *
 * - Status(ErrorCode::PLANNING_ERROR, msg)：
 *   构造函数创建错误状态
 *   包含错误码和错误消息
 *
 * - common::VehicleConfigHelper::Instance()->GetConfig()：
 *   单例模式获取配置
 *   VehicleConfigHelper：车辆配置帮助类
 *   Instance()：获取单例实例
 *   GetConfig()：获取配置数据
 *
 * - vehicle_param().max_abs_speed_when_stopped()：
 *   获取车辆参数
 *   max_abs_speed_when_stopped：停车时的最大绝对速度阈值
 *
 * - frame->mutable_open_space_info()：
 *   mutable_前缀方法
 *   protobuf可变成员访问
 *   返回可修改的OpenSpaceInfo引用
 *
 * - set_replan_flag(true/false)：
 *   protobuf的setter方法
 *   设置重规划标志
 *
 * - injector_->vehicle_state()->linear_velocity()：
 *   获取车辆状态
 *   linear_velocity()：获取线速度
 */
Status OpenSpaceReplanDecider::Process(Frame *frame) {
  /**
   * @brief 获取上一帧
   *
   * injector_->frame_history()：
   *   获取帧历史记录管理器
   *   用于访问历史规划帧数据
   *
   * ->Latest()：
   *   获取最新的历史帧
   *   返回const Frame*类型
   */
  const auto& previous_frame = injector_->frame_history()->Latest();
  /**
   * @brief 空指针检查
   *
   * frame == nullptr || previous_frame == nullptr：
   *   检查当前帧和历史帧是否有效
   *   nullptr：空指针字面量
   *   ||：逻辑或，只要有一个为真就进入
   *
   * 防御性编程：
   *   确保后续访问不会导致空指针解引用
   */
  if (frame == nullptr || previous_frame == nullptr) {
    /**
     * @brief 构造错误消息
     *
     * const std::string：
     *   const限定字符串内容不可变
     *   std::string：标准库字符串类
     *
     * msg = "..."：
     *   字符串赋值
     */
    const std::string msg =
        "Invalid frame, fail to process the OpenSpaceReplanDecider.";
    /**
     * @brief 记录错误日志
     *
     * AERROR：
     *   Apollo错误级别日志宏
     *   << 运算符重载，连接日志内容
     */
    AERROR << msg;
    /**
     * @brief 返回错误状态
     *
     * Status(ErrorCode::PLANNING_ERROR, msg)：
     *   构造错误状态
     *   ErrorCode::PLANNING_ERROR：规划错误码
     *   msg：错误消息
     */
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  /**
   * @brief 判断是否需要重规划
   *
   * 重规划条件：
   * 1. 上一帧没有优化轨迹数据
   * 2. 车辆速度小于停车速度阈值
   *
   * previous_frame->open_space_info().optimizer_trajectory_data().empty()：
   *   optimizer_trajectory_data()：优化后的轨迹数据
   *   .empty()：检查容器是否为空
   *   如果为空，说明没有已规划的轨迹
   *
   * injector_->vehicle_state()->linear_velocity()：
   *   获取车辆当前线速度
   *
   * common::VehicleConfigHelper::GetConfig().vehicle_param().max_abs_speed_when_stopped()：
   *   获取停车时的最大速度阈值
   *   用于判断车辆是否处于静止状态
   *
   * < 比较运算符：
   *   判断当前速度是否小于阈值
   *
   * && 逻辑与运算符：
   *   两个条件都需要满足才重规划
   *   上一帧无轨迹 且 车辆静止
   */
  if (previous_frame->open_space_info().optimizer_trajectory_data().empty() &&
      injector_->vehicle_state()->linear_velocity() <
      common::VehicleConfigHelper::GetConfig().
          vehicle_param().max_abs_speed_when_stopped()) {
    /**
     * @brief 设置重规划标志为true
     *
     * frame->mutable_open_space_info()：
     *   获取当前帧的可变OpenSpaceInfo引用
     *   mutable_前缀表示可修改
     *
     * ->set_replan_flag(true)：
     *   设置重规划标志为true
     *   表示需要进行轨迹重规划
     */
    frame->mutable_open_space_info()->set_replan_flag(true);
  } else {
    /**
     * @brief 设置重规划标志为false
     *
     * 不满足重规划条件：
     * - 上一帧已有轨迹 且/或 车辆在移动
     * - 复用现有轨迹即可
     */
    frame->mutable_open_space_info()->set_replan_flag(false);
  }
  /**
   * @brief 返回成功状态
   *
   * Status::OK()：
   *   Apollo成功状态
   *   表示处理成功完成
   */
  return Status::OK();
}

/**
 * @brief 命名空间结束标记
 *
 * C++语法说明：
 * }  // namespace planning：
 *   单行注释说明命名空间结束
 *   便于代码阅读和导航
 */
}  // namespace planning
}  // namespace apollo
