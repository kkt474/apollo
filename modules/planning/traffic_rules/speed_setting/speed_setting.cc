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
 * @file speed_setting.cc
 * @brief 速度设置交通规则实现文件
 *
 * 本文件实现了速度设置交通规则(SpeedSetting)
 * 用于处理外部速度命令，设置巡航速度
 *
 * 功能说明：
 * 1. 处理速度命令(SpeedCommand)
 * 2. 解析目标速度或速度因子
 * 3. 设置参考线的巡航速度
 * 4. 维护上次速度的持久化状态
 *
 * 命令类型说明：
 * 1. MotionCommand: 运动命令，触发使用基础巡航速度
 * 2. SpeedCommand: 自定义速度命令，支持设置目标速度或速度因子
 *    - target_speed: 直接设置目标速度
 *    - target_speed_factor: 基于上次速度的倍数因子
 *    - is_restore_target_speed: 恢复默认速度
 *
 * 状态维护：
 * - last_cruise_speed_: 上次设置的巡航速度
 * - last_sequence_num_: 上次处理的命令序列号
 * 用于判断是否为新命令，避免重复处理
 *
 * 相关C++语法说明：
 * - std::shared_ptr<T>: 智能指针，共享所有权
 * - protobuf消息: custom_command.Is<T>()、custom_command.UnpackTo<T>()
 * - 初始化列表: 构造函数中使用
 **/

/**
 * @brief 本类的头文件
 *
 * 包含SpeedSetting类的完整定义
 */
#include "modules/planning/traffic_rules/speed_setting/speed_setting.h"

/**
 * @brief 外部命令消息protobuf头文件
 *
 * speed_command.pb.h:
 *   包含SpeedCommand消息定义
 *   用于解析外部速度命令
 */
#include "modules/common_msgs/external_command_msgs/speed_command.pb.h"

/**
 * @brief Apollo命名空间开始
 */
namespace apollo {

/**
 * @brief 规划模块命名空间
 */
namespace planning {

/**
 * @brief 使用别名声明，简化类型引用
 *
 * C++语法说明：
 * using声明：引入其他命名空间的类型到当前作用域
 */
using apollo::common::Status;  /**< Apollo通用状态类型 */

/**
 * @brief SpeedSetting构造函数
 *
 * 功能说明：
 * 初始化速度设置规则
 * 使用初始化列表设置初始状态
 *
 * 初始化值说明：
 * - last_cruise_speed_ = -1.0: 初始巡航速度为负值，表示未设置
 * - last_sequence_num_ = -1: 初始序列号为-1，表示无历史命令
 *
 * C++语法说明：
 * : last_cruise_speed_(-1.0), last_sequence_num_(-1)
 * 初始化列表语法，在构造函数体执行之前初始化成员变量
 * 这是初始化成员变量的推荐方式，效率高于在函数体内赋值
 */
SpeedSetting::SpeedSetting() : last_cruise_speed_(-1.0), last_sequence_num_(-1) {
    /**
     * @brief Apollo License验证钩子
     *
     * 这是一个占位注释，用于Apollo License验证
     * 不影响实际功能
     */
}

/**
 * @brief 应用速度设置规则
 *
 * @param frame 当前规划帧
 * @param reference_line_info 参考线信息
 * @return Status 应用状态
 *
 * 功能说明：
 * 处理外部速度命令，设置巡航速度
 *
 * 算法流程：
 * 1. 检查是否有新的速度命令
 * 2. 如果没有新命令，使用上次设置的速度
 * 3. 如果是MotionCommand，使用基础巡航速度
 * 4. 如果是SpeedCommand，解析并设置目标速度
 * 5. 处理速度因子或直接设置速度
 *
 * C++语法说明：
 * - Frame* const frame:
 *   指向常量的指针（指针本身是常量）
 *   函数内不能改变frame的指向
 * - ReferenceLineInfo* const reference_line_info:
 *   指向常量的指针
 */
Status SpeedSetting::ApplyRule(Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  /**
   * @brief 获取本地视图中的规划命令
   *
   * frame->local_view().planning_command:
   *   LocalView包含规划所需的各种输入数据
   *   planning_command是外部发送给规划模块的命令
   *
   * C++语法说明：
   * - frame->local_view():
   *   ->运算符，通过指针调用成员函数获取local_view
   * - .planning_command:
   *   .运算符，访问local_view的成员
   */
  const auto& planning_command = frame->local_view().planning_command;

  /**
   * @brief 检查命令是否有效
   *
   * 条件链式调用（所有条件都满足才处理新命令）：
   * 1. !planning_command->has_header():
   *    命令没有header
   * 2. !planning_command->header().has_sequence_num():
   *    header没有序列号
   * 3. planning_command->header().sequence_num() == last_sequence_num_:
   *    序列号与上次相同（重复命令）
   *
   * 如果满足任一条件，说明没有新的速度命令
   */
  if (!planning_command->has_header() || !planning_command->header().has_sequence_num()
      || planning_command->header().sequence_num() == last_sequence_num_) {
      /**
       * @brief 使用上次的巡航速度
       *
       * if (last_cruise_speed_ > 0.0):
       *   检查上次速度是否有效（大于0）
       *
       * reference_line_info->SetCruiseSpeed(last_cruise_speed_):
       *   设置参考线的巡航速度
       */
      if (last_cruise_speed_ > 0.0) {
          reference_line_info->SetCruiseSpeed(last_cruise_speed_);
      }
      return Status::OK();  /**< 返回成功 */
  }

  /**
   * @brief 检查是否是MotionCommand
   *
   * planning_command->is_motion_command():
   *   检查命令类型是否为运动命令
   * MotionCommand通常用于自动驾驶的启动/停止等
   *
   * 如果是MotionCommand，使用基础巡航速度
   */
  if (planning_command->is_motion_command()) {
      /**
       * @brief 使用基础巡航速度
       *
       * reference_line_info->GetBaseCruiseSpeed():
       *   获取参考线的基础巡航速度
       *   这是从地图或配置中获取的默认速度
       */
      last_cruise_speed_ = reference_line_info->GetBaseCruiseSpeed();
      return Status::OK();  /**< 返回成功 */
  }

  /**
   * @brief 检查是否有自定义命令且类型为SpeedCommand
   *
   * 条件链式调用：
   * 1. !planning_command->has_custom_command():
   *    没有自定义命令
   * 2. !planning_command->custom_command().Is<external_command::SpeedCommand>():
   *    自定义命令不是SpeedCommand类型
   *
   * 如果满足任一条件，说明不是速度命令
   */
  if (!planning_command->has_custom_command()
      || !planning_command->custom_command().Is<external_command::SpeedCommand>()) {
      /**
       * @brief 使用上次的巡航速度
       */
      if (last_cruise_speed_ > 0.0) {
          reference_line_info->SetCruiseSpeed(last_cruise_speed_);
      }
      return Status::OK();  /**< 返回成功 */
  }

  /**
   * @brief 获取自定义命令引用
   *
   * custom_command():
   *   获取CustomCommand消息的引用
   *   包含序列化的命令数据
   */
  const auto& custom_command = planning_command->custom_command();

  /**
   * @brief 解包SpeedCommand
   *
   * external_command::SpeedCommand:
   *   SpeedCommand消息类型
   *   包含目标速度、速度因子等
   *
   * custom_command.UnpackTo(&speed_command):
   *   将序列化的CustomCommand数据反解包为SpeedCommand
   *   返回bool表示是否成功
   */
  external_command::SpeedCommand speed_command;
  if (!custom_command.UnpackTo(&speed_command)) {
      /**
       * @brief 解包失败，使用上次的巡航速度
       */
      if (last_cruise_speed_ > 0.0) {
          reference_line_info->SetCruiseSpeed(last_cruise_speed_);
      }
      AERROR << "Unpack speed command failed!";  /**< 记录错误日志 */
      return Status(common::PLANNING_ERROR);  /**< 返回错误状态 */
  }

  /**
   * @brief 检查是否是恢复目标速度命令
   *
   * speed_command.has_is_restore_target_speed():
   *   检查是否有恢复标志字段
   * speed_command.is_restore_target_speed():
   *   获取恢复标志的值
   *
   * 如果是恢复命令，使用基础巡航速度
   */
  if (speed_command.has_is_restore_target_speed() && speed_command.is_restore_target_speed()) {
      last_cruise_speed_ = reference_line_info->GetBaseCruiseSpeed();
      return Status::OK();  /**< 返回成功 */
  }

  /**
   * @brief 更新命令序列号
   *
   * 保存当前命令的序列号
   * 用于下次判断是否有新命令
   */
  last_sequence_num_ = planning_command->header().sequence_num();

  /**
   * @brief 首次设置巡航速度
   *
   * if (last_cruise_speed_ < 0.0):
   *   如果上次速度为负值（未设置）
   *   使用基础巡航速度初始化
   */
  if (last_cruise_speed_ < 0.0) {
      last_cruise_speed_ = reference_line_info->GetBaseCruiseSpeed();
  }

  /**
   * @brief 处理速度调整命令
   *
   * 根据命令内容设置目标速度
   * 两种方式：
   * 1. 直接设置目标速度
   * 2. 基于上次速度计算目标速度
   */

  /**
   * @brief 方式1：直接设置目标速度
   *
   * speed_command.has_target_speed():
   *   检查是否有target_speed字段
   *
   * reference_line_info->SetCruiseSpeed(speed_command.target_speed()):
   *   直接将命令中的目标速度设置为巡航速度
   */
  if (speed_command.has_target_speed()) {
      reference_line_info->SetCruiseSpeed(speed_command.target_speed());
  }
  /**
   * @brief 方式2：基于速度因子计算
   *
   * else if (speed_command.has_target_speed_factor()):
   *   如果没有直接目标速度，检查是否有速度因子
   *
   * target_speed = last_cruise_speed_ * target_speed_factor:
   *   新速度 = 上次速度 × 速度因子
   *   例如：上次100km/h × 0.8 = 80km/h
   */
  else if (speed_command.has_target_speed_factor()) {
      double target_speed = last_cruise_speed_ * speed_command.target_speed_factor();
      reference_line_info->SetCruiseSpeed(target_speed);
  }

  /**
   * @brief 更新上次巡航速度
   *
   * 保存当前设置的基础巡航速度
   * 用于下次命令的速度因子计算
   */
  last_cruise_speed_ = reference_line_info->GetBaseCruiseSpeed();

  return Status::OK();  /**< 设置成功 */
}

}  // namespace planning
}  // namespace apollo
