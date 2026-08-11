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
 * @file common.cc
 * @brief 规划通用工具函数实现文件
 *
 * 本文件实现了规划模块的通用工具函数，主要用于构建停车决策。
 *
 * 核心功能：
 * 1. 创建虚拟停车墙障碍物
 * 2. 添加停车决策到路径决策中
 *
 * 设计模式：
 * - 工厂模式：通过Frame::CreateStopObstacle创建虚拟障碍物
 * - 装饰模式：停车决策作为障碍物的属性添加
 *
 * C++语法说明：
 * - namespace嵌套：apollo -> planning -> util三层命名空间
 * - const引用：避免参数拷贝
 * - 返回值：返回int表示成功/失败状态
 */

#include "modules/planning/planning_base/common/util/common.h"

namespace apollo {
/**
 * @brief Apollo顶层命名空间
 */
namespace planning {

/**
 * @brief planning子命名空间
 */
namespace util {

/**
 * @brief 类型别名定义
 * WithinBound: 判断值是否在范围内的工具函数
 */
using apollo::common::util::WithinBound;

/**
 * @brief 构建停车决策
 *
 * @param stop_wall_id 停车墙ID
 * @param stop_line_s 停车线s坐标（沿参考线）
 * @param stop_distance 停车距离（停车线前的距离）
 * @param stop_reason_code 停车原因代码
 * @param wait_for_obstacles 等待避让的障碍物ID列表
 * @param decision_tag 决策标签
 * @param frame 当前规划帧
 * @param reference_line_info 参考线信息
 * @param stop_wall_width 停车墙宽度（可选，默认值）
 * @return int 0表示成功，-1表示失败
 *
 * 功能流程：
 * 1. 验证停车线坐标在参考线范围内
 * 2. 创建虚拟停车墙障碍物
 * 3. 将障碍物添加到参考线信息中
 * 4. 构建停车决策
 * 5. 添加纵向决策到路径决策
 *
 * 算法说明：
 * - 停车点s坐标 = stop_line_s - stop_distance
 * - 停车距离为负值表示在停车线后方
 *
 * C++语法说明：
 * - const std::string&: 常量引用，避免字符串拷贝
 * - const double: 常量参数，按值传递
 * - Frame* const: 指向常量的指针
 * - CHECK_NOTNULL(pointer): Apollo断言宏，检查指针非空
 * - mutable_xxx(): protobuf消息的可修改访问器
 */
int BuildStopDecision(const std::string& stop_wall_id,
                      const double stop_line_s,
                      const double stop_distance,
                      const StopReasonCode& stop_reason_code,
                      const std::vector<std::string>& wait_for_obstacles,
                      const std::string& decision_tag,
                      Frame* const frame,
                      ReferenceLineInfo* const reference_line_info,
                      double stop_wall_width) {
  /**
   * @brief 断言检查
   * 确保frame和reference_line_info指针非空
   */
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  /**
   * @brief 获取参考线引用
   */
  const auto& reference_line = reference_line_info->reference_line();

  /**
   * @brief 验证停车线坐标是否在参考线范围内
   * WithinBound(0.0, reference_line.Length(), stop_line_s):
   *   检查stop_line_s是否在[0, Length]范围内
   */
  if (!WithinBound(0.0, reference_line.Length(), stop_line_s)) {
    AERROR << "stop_line_s[" << stop_line_s << "] is not on reference line";
    return 0;  /**< 返回0表示无效 */
  }

  /**
   * @brief 创建虚拟停车墙障碍物
   * frame->CreateStopObstacle():
   *   工厂方法，创建虚拟障碍物
   * 参数：
   * - reference_line_info: 参考线信息
   * - stop_wall_id: 停车墙ID
   * - stop_line_s: 停车线s坐标
   * - stop_wall_width: 停车墙宽度
   */
  const auto* obstacle =
      frame->CreateStopObstacle(reference_line_info, stop_wall_id,
                                stop_line_s, stop_wall_width);

  /**
   * @brief 检查障碍物创建是否成功
   */
  if (!obstacle) {
    AERROR << "Failed to create obstacle [" << stop_wall_id << "]";
    return -1;  /**< 返回-1表示失败 */
  }

  /**
   * @brief 将障碍物添加到参考线信息中
   * reference_line_info->AddObstacle():
   *   添加障碍物到路径决策的障碍物列表
   */
  const Obstacle* stop_wall = reference_line_info->AddObstacle(obstacle);
  if (!stop_wall) {
    AERROR << "Failed to add obstacle[" << stop_wall_id << "]";
    return -1;
  }

  /**
   * @brief 计算实际停车点的s坐标
   * 公式：stop_s = stop_line_s - stop_distance
   * - stop_line_s: 停车线位置
   * - stop_distance: 停车距离（正值表示在停车线前方，负值表示后方）
   *
   * 在Apollo中：
   * - stop_distance > 0: 停车点在停车线前方
   * - stop_distance < 0: 停车点在停车线后方
   * - stop_distance = 0: 刚好停在停车线
   */
  const double stop_s = stop_line_s - stop_distance;

  /**
   * @brief 获取停车点的参考线信息
   * reference_line.GetReferencePoint(stop_s):
   *   获取参考线上指定s坐标处的点
   * 包含x, y坐标和航向角heading
   */
  const auto& stop_point = reference_line.GetReferencePoint(stop_s);

  /**
   * @brief 获取停车点的航向角
   * 停车时车辆应该朝向的方向
   */
  const double stop_heading =
      reference_line.GetReferencePoint(stop_s).heading();

  /**
   * @brief 创建停车决策对象
   * ObjectDecisionType: 障碍物决策类型protobuf消息
   */
  ObjectDecisionType stop;

  /**
   * @brief 获取可修改的stop决策
   * mutable_stop(): 获取stop子消息的可修改访问器
   */
  auto* stop_decision = stop.mutable_stop();

  /**
   * @brief 设置停车原因代码
   * set_reason_code():
   *   设置停车原因的枚举值
   * 如：STOP_REASON_OBSTACLE, STOP_REASON_SIGNAL等
   */
  stop_decision->set_reason_code(stop_reason_code);

  /**
   * @brief 设置停车距离
   * set_distance_s():
   *   负值表示在障碍物后方
   * 这里直接使用传入的stop_distance
   */
  stop_decision->set_distance_s(-stop_distance);

  /**
   * @brief 设置停车航向角
   */
  stop_decision->set_stop_heading(stop_heading);

  /**
   * @brief 设置停车点坐标
   * mutable_stop_point(): 获取停车点消息的可修改访问器
   * set_x/set_y/set_z(): 设置坐标值
   * 停车点z坐标设为0.0（地面高度）
   */
  stop_decision->mutable_stop_point()->set_x(stop_point.x());
  stop_decision->mutable_stop_point()->set_y(stop_point.y());
  stop_decision->mutable_stop_point()->set_z(0.0);

  /**
   * @brief 添加需要等待避让的障碍物列表
   * add_wait_for_obstacle():
   *   向停车决策添加一个需要等待的障碍物ID
   * 用于停车时等待行人等动态障碍物通过
   *
   * C++语法说明：
   * - for循环遍历wait_for_obstacles向量
   * - size_t: 无符号整数类型，用于表示大小和索引
   * - ++i: 前置递增运算符
   */
  for (size_t i = 0; i < wait_for_obstacles.size(); ++i) {
    stop_decision->add_wait_for_obstacle(wait_for_obstacles[i]);
  }

  /**
   * @brief 获取路径决策
   * reference_line_info->path_decision():
   *   获取该参考线的路径决策对象
   */
  auto* path_decision = reference_line_info->path_decision();

  /**
   * @brief 添加纵向停车决策
   * path_decision->AddLongitudinalDecision():
   *   向指定障碍物添加纵向决策
   * 参数：
   * - decision_tag: 决策者标签（如"RuleBasedStopDecider"）
   * - stop_wall->Id(): 停车墙障碍物ID
   * - stop: 停车决策对象
   */
  path_decision->AddLongitudinalDecision(decision_tag, stop_wall->Id(), stop);

  return 0;  /**< 成功返回0 */
}

/**
 * @brief 构建停车决策（重载版本，基于车道ID）
 *
 * @param stop_wall_id 停车墙ID
 * @param lane_id 车道ID
 * @param lane_s 车道s坐标
 * @param stop_distance 停车距离
 * @param stop_reason_code 停车原因代码
 * @param wait_for_obstacles 等待避让的障碍物列表
 * @param decision_tag 决策标签
 * @param frame 当前规划帧
 * @param reference_line_info 参考线信息
 * @return int 0表示成功，-1表示失败
 *
 * 与上一版本的区别：
 * - 使用车道ID而非参考线坐标
 * - 停车墙位置由车道ID和s坐标确定
 * - 停车墙必须在车道上（检查IsOnLane）
 *
 * C++语法说明：
 * - 函数重载：与上一函数名相同但参数列表不同
 * - 编译器会根据参数类型自动选择调用哪个版本
 */
int BuildStopDecision(const std::string& stop_wall_id,
                      const std::string& lane_id,
                      const double lane_s,
                      const double stop_distance,
                      const StopReasonCode& stop_reason_code,
                      const std::vector<std::string>& wait_for_obstacles,
                      const std::string& decision_tag,
                      Frame* const frame,
                      ReferenceLineInfo* const reference_line_info) {
  /**
   * @brief 断言检查
   */
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  /**
   * @brief 获取参考线引用
   */
  const auto& reference_line = reference_line_info->reference_line();

  /**
   * @brief 创建虚拟停车墙障碍物
   * frame->CreateStopObstacle(stop_wall_id, lane_id, lane_s):
   *   工厂方法，根据车道ID和s坐标创建障碍物
   */
  const auto* obstacle =
      frame->CreateStopObstacle(stop_wall_id, lane_id, lane_s);

  /**
   * @brief 检查障碍物创建是否成功
   */
  if (!obstacle) {
    AERROR << "Failed to create obstacle [" << stop_wall_id << "]";
    return -1;
  }

  /**
   * @brief 将障碍物添加到参考线信息
   */
  const Obstacle* stop_wall = reference_line_info->AddObstacle(obstacle);
  if (!stop_wall) {
    AERROR << "Failed to create obstacle for: " << stop_wall_id;
    return -1;
  }

  /**
   * @brief 获取停车墙边界框
   * stop_wall->PerceptionBoundingBox():
   *   获取障碍物的感知边界框
   */
  const auto& stop_wall_box = stop_wall->PerceptionBoundingBox();

  /**
   * @brief 检查停车墙中心是否在车道上
   * reference_line.IsOnLane():
   *   判断点是否在参考线的可行驶车道范围内
   * 如果停车墙不在车道上，跳过停车决策
   */
  if (!reference_line.IsOnLane(stop_wall_box.center())) {
    ADEBUG << "stop point is not on lane. SKIP STOP decision";
    return 0;  /**< 返回0表示跳过 */
  }

  /**
   * @brief 计算停车点坐标
   * stop_wall->PerceptionSLBoundary().start_s():
   *   获取停车墙的起始s坐标
   * 停车点 = 停车墙起点 - stop_distance
   */
  auto stop_point = reference_line.GetReferencePoint(
      stop_wall->PerceptionSLBoundary().start_s() - stop_distance);

  /**
   * @brief 创建停车决策
   */
  ObjectDecisionType stop;
  auto* stop_decision = stop.mutable_stop();

  /**
   * @brief 设置停车决策参数
   */
  stop_decision->set_reason_code(stop_reason_code);
  stop_decision->set_distance_s(-stop_distance);
  stop_decision->set_stop_heading(stop_point.heading());

  /**
   * @brief 设置停车点坐标
   */
  stop_decision->mutable_stop_point()->set_x(stop_point.x());
  stop_decision->mutable_stop_point()->set_y(stop_point.y());
  stop_decision->mutable_stop_point()->set_z(0.0);

  /**
   * @brief 获取路径决策并添加纵向决策
   */
  auto* path_decision = reference_line_info->path_decision();
  path_decision->AddLongitudinalDecision(decision_tag, stop_wall->Id(), stop);

  return 0;
}

/**
 * @brief 命名空间结束标记
 */
}  // namespace util
}  // namespace planning
}  // namespace apollo
