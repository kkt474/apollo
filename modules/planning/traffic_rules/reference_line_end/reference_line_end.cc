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
 * @file reference_line_end.cc
 * @brief 参考线终点交通规则实现文件
 *
 * 本文件实现了参考线终点交通规则(ReferenceLineEnd)
 * 用于在车辆接近参考线终点时生成停车决策
 *
 * 功能说明：
 * 1. 检查车辆是否接近参考线终点
 * 2. 计算剩余路径长度
 * 3. 创建虚拟停车墙
 * 4. 构建停车决策
 *
 * 场景说明：
 * 当车辆在参考线上行驶时：
 * - 如果剩余路径长度小于阈值，创建停车墙
 * - 确保车辆在参考线终点前完全停止
 * - 这发生在：
 *   1. 换道失败，需要返回原车道
 *   2. 路由终点在下一条参考线
 *   3. 当前参考线结束（如道路尽头）
 *
 * 决策流程：
 * 1. 计算剩余路径长度 = 参考线长度 - 自车后边缘S
 * 2. 如果剩余长度 > 阈值，直接返回（不需要处理）
 * 3. 如果剩余长度 <= 阈值，创建虚拟停车墙
 * 4. 在停车墙处构建停车决策
 *
 * 相关C++语法说明：
 * - std::shared_ptr<T>: 智能指针，共享所有权
 * - FLAGS_xxx: GFlags配置参数
 * - protobuf消息类型: 停车决策、障碍物等
 **/

/**
 * @brief 本类的头文件
 *
 * 包含ReferenceLineEnd类的完整定义
 */
#include "modules/planning/traffic_rules/reference_line_end/reference_line_end.h"

/**
 * @brief 标准库头文件
 *
 * <memory>: 提供std::shared_ptr智能指针
 */
#include <memory>

/**
 * @brief PNC点protobuf消息头文件
 *
 * pnc_point.pb.h:
 *   包含PathPoint、SpeedPoint、TrajectoryPoint等
 */
#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"

/**
 * @brief 车辆配置助手头文件
 *
 * VehicleConfigHelper:
 *   车辆配置助手，用于获取车辆参数
 */
#include "modules/common/configs/vehicle_config_helper.h"

/**
 * @brief 规划模块GFlags头文件
 *
 * FLAGS_virtual_stop_wall_length:
 *   虚拟停车墙长度
 */
#include "modules/planning/planning_base/gflags/planning_gflags.h"

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
 * @brief 参考线终点规则初始化函数
 *
 * @param name 规则名称
 * @param injector 依赖注入器智能指针
 * @return bool 初始化是否成功
 *
 * 功能说明：
 * 1. 调用父类TrafficRule的Init进行基础初始化
 * 2. 加载本规则特有的配置ReferenceLineEndConfig
 *
 * C++语法说明：
 * - const std::string& name:
 *   常量引用，规则名称用于标识和日志
 * - const std::shared_ptr<DependencyInjector>& injector:
 *   共享所有权的智能指针，依赖注入器
 */
bool ReferenceLineEnd::Init(
    const std::string& name,
    const std::shared_ptr<DependencyInjector>& injector) {
  /**
   * @brief 调用父类TrafficRule的Init进行基础初始化
   *
   * TrafficRule::Init():
   *   执行基础初始化，如保存名称和依赖注入器
   */
  if (!TrafficRule::Init(name, injector)) {
    return false;  /**< 父类初始化失败，返回false */
  }

  /**
   * @brief 加载本规则的配置
   *
   * TrafficRule::LoadConfig<T>():
   *   模板方法，从配置文件加载ReferenceLineEndConfig类型配置
   *   配置内容可能包括：最小剩余长度、停车距离等
   */
  return TrafficRule::LoadConfig<ReferenceLineEndConfig>(&config_);
}

/**
 * @brief 应用交通规则
 *
 * @param frame 当前规划帧
 * @param reference_line_info 参考线信息
 * @return Status 应用状态
 *
 * 功能说明：
 * 交通规则的入口函数
 * 由TrafficDecider::Execute调用
 *
 * 执行流程：
 * 1. 获取参考线和自车SL边界
 * 2. 计算剩余路径长度
 * 3. 如果剩余长度足够，直接返回
 * 4. 如果剩余长度不足，创建虚拟停车墙
 * 5. 构建停车决策
 *
 * C++语法说明：
 * - Frame* frame:
 *   原始指针，表示当前规划帧
 * - ReferenceLineInfo* const reference_line_info:
 *   指向常量的指针（指针本身是常量）
 */
Status ReferenceLineEnd::ApplyRule(
    Frame* frame, ReferenceLineInfo* const reference_line_info) {
  /**
   * @brief 获取参考线引用
   *
   * reference_line_info->reference_line():
   *   获取参考线引用
   *   用于查询参考线属性和计算
   */
  const auto& reference_line = reference_line_info->reference_line();

  /**
   * @brief 打印调试信息
   *
   * ADEBUG:
   *   Apollo调试级别日志
   *   仅在调试模式下输出
   */
  ADEBUG << "ReferenceLineEnd length[" << reference_line.Length() << "]";

  /**
   * @brief 遍历并打印参考线上的所有车道段
   *
   * for (const auto& segment : reference_line_info->Lanes()):
   *   范围for循环，遍历所有车道段
   *
   * reference_line_info->Lanes():
   *   获取参考线上的车道段列表
   * segment.lane->lane().id().id():
   *   获取车道的ID字符串
   */
  for (const auto& segment : reference_line_info->Lanes()) {
    ADEBUG << "   lane[" << segment.lane->lane().id().id() << "]";
  }

  /**
   * @brief 计算剩余路径长度
   *
   * remain_s = 参考线长度 - 自车后边缘S坐标
   *
   * 公式解释：
   * - reference_line.Length(): 参考线的总长度
   * - reference_line_info->AdcSlBoundary().end_s(): 自车在后方的SL边界S坐标
   * - remain_s: 从自车后边缘到参考线终点的距离
   *
   * C++语法说明：
   * - reference_line_info->AdcSlBoundary().end_s():
   *   链式调用，先获取AdcSlBoundary，再获取其end_s成员
   */
  double remain_s =
      reference_line.Length() - reference_line_info->AdcSlBoundary().end_s();

  /**
   * @brief 检查剩余长度是否足够
   *
   * config_.min_reference_line_remain_length():
   *   配置中的最小剩余长度阈值
   * 如果剩余长度大于阈值，说明还不需要处理
   *
   * 条件：remain_s > min_reference_line_remain_length
   * 如果为true，直接返回OK（不需要创建停车墙）
   */
  if (remain_s > config_.min_reference_line_remain_length()) {
    return Status::OK();  /**< 剩余长度足够，无需处理 */
  }

  /**
   * @brief 创建虚拟停车墙ID
   *
   * REF_LINE_END_VO_ID_PREFIX:
   *   参考线终点虚拟障碍物ID前缀
   *   通常是"__REFERENCE_LINE_END_WALL__"
   *
   * reference_line_info->Lanes().Id():
   *   获取当前参考线的车道ID
   * 完整ID = 前缀 + 车道ID，确保唯一性
   */
  std::string virtual_obstacle_id =
      REF_LINE_END_VO_ID_PREFIX + reference_line_info->Lanes().Id();

  /**
   * @brief 计算障碍物起始S坐标
   *
   * 公式：obstacle_start_s = reference_line.Length() - 2 * virtual_stop_wall_length
   *
   * 解释：
   * - 在参考线终点前两个停车墙长度处创建停车墙
   * - 这样停车墙的中心在终点前一个墙长
   * - 确保车辆能在终点前完全停止
   *
   * C++语法说明：
   * FLAGS_virtual_stop_wall_length:
   *   GFlags配置的虚拟停车墙长度
   */
  double obstacle_start_s =
      reference_line.Length() - 2 * FLAGS_virtual_stop_wall_length;

  /**
   * @brief 创建停车障碍物
   *
   * frame->CreateStopObstacle():
   *   在帧中创建一个虚拟停车障碍物
   * 参数：
   * - reference_line_info: 参考线信息
   * - virtual_obstacle_id: 停车墙ID
   * - obstacle_start_s: 停车墙起始S坐标
   *
   * 返回值：
   * - 成功：返回创建的障碍物指针
   * - 失败：返回nullptr
   */
  auto* obstacle = frame->CreateStopObstacle(
      reference_line_info, virtual_obstacle_id, obstacle_start_s);

  /**
   * @brief 检查障碍物创建是否成功
   */
  if (!obstacle) {
    /**
     * @brief 创建失败，记录错误
     */
    return Status(common::PLANNING_ERROR,
                 "Failed to create reference line end obstacle");
  }

  /**
   * @brief 将障碍物添加到参考线信息
   *
   * reference_line_info->AddObstacle(obstacle):
   *   将障碍物添加到路径决策中
   * 参数：障碍物指针
   * 返回值：成功返回障碍物指针，失败返回nullptr
   */
  Obstacle* stop_wall = reference_line_info->AddObstacle(obstacle);

  /**
   * @brief 检查添加是否成功
   */
  if (!stop_wall) {
    /**
     * @brief 添加失败，记录错误
     */
    return Status(
        common::PLANNING_ERROR,
        "Failed to create path obstacle for reference line end obstacle");
  }

  /**
   * @brief 计算停车线S坐标
   *
   * 公式：stop_line_s = obstacle_start_s - config_.stop_distance()
   *
   * 解释：
   * - obstacle_start_s: 障碍物起始位置
   * - config_.stop_distance(): 配置的停车距离
   * - stop_line_s: 实际停车线位置
   *
   * 停车线在障碍物起始位置前方一定距离
   * 确保车辆停在障碍物之前
   */
  const double stop_line_s = obstacle_start_s - config_.stop_distance();

  /**
   * @brief 获取停车点的参考线信息
   *
   * reference_line.GetReferencePoint(stop_line_s):
   *   获取参考线上指定S坐标位置的点
   *   包含x, y坐标和航向角heading
   */
  auto stop_point = reference_line.GetReferencePoint(stop_line_s);

  /**
   * @brief 创建停车决策
   *
   * ObjectDecisionType:
   *   障碍物决策类型
   *   用于STOP/FOLLOW/YIELD/OVERTAKE/IGNORE等决策
   */
  ObjectDecisionType stop;

  /**
   * @brief 获取停车决策的可修改指针
   *
   * mutable_stop():
   *   protobuf的可修改访问器
   *   返回stop子消息的指针
   */
  auto stop_decision = stop.mutable_stop();

  /**
   * @brief 设置停车原因代码
   *
   * set_reason_code():
   *   设置停车原因的枚举值
   * STOP_REASON_DESTINATION:
   *   使用目的地原因代码
   *   因为参考线终点被视为一种"目的地"
   */
  stop_decision->set_reason_code(StopReasonCode::STOP_REASON_DESTINATION);

  /**
   * @brief 设置停车距离
   *
   * set_distance_s(-config_.stop_distance()):
   *   负值表示在停车点后方
   *   停车距离用于计算精确的停车位置
   */
  stop_decision->set_distance_s(-config_.stop_distance());

  /**
   * @brief 设置停车朝向
   *
   * set_stop_heading():
   *   设置停车时车辆的航向角
   *   stop_point.heading():
   *     获取参考点在停车位置的航向角
   */
  stop_decision->set_stop_heading(stop_point.heading());

  /**
   * @brief 设置停车点坐标
   *
   * mutable_stop_point():
   *   获取停车点消息的可修改访问器
   * set_x(), set_y(), set_z():
   *   设置停车点的世界坐标
   * z坐标设为0.0表示地面高度
   */
  stop_decision->mutable_stop_point()->set_x(stop_point.x());
  stop_decision->mutable_stop_point()->set_y(stop_point.y());
  stop_decision->mutable_stop_point()->set_z(0.0);

  /**
   * @brief 获取路径决策指针
   *
   * reference_line_info->path_decision():
   *   获取该参考线的路径决策对象
   *   包含所有障碍物的决策列表
   */
  auto* path_decision = reference_line_info->path_decision();

  /**
   * @brief 添加纵向停车决策
   *
   * path_decision->AddLongitudinalDecision():
   *   向指定障碍物添加纵向决策
   * 参数：
   * - Getname(): 规则名称（"ReferenceLineEnd"）
   * - stop_wall->Id(): 停车墙障碍物ID
   * - stop: 停车决策对象
   *
   * 这会创建STOP决策，使车辆在该位置停止
   */
  path_decision->AddLongitudinalDecision(Getname(), stop_wall->Id(), stop);

  return Status::OK();  /**< 停车决策创建成功 */
}

}  // namespace planning
}  // namespace apollo
