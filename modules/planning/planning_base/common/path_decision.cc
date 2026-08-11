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
 * @file path_decision.cc
 * @brief 路径决策类实现文件
 *
 * 本文件实现了PathDecision类，负责管理路径上的所有障碍物决策。
 *
 * 核心功能：
 * 1. 障碍物管理（添加、查找）
 * 2. ST边界管理
 * 3. 纵向/横向决策管理
 * 4. 主停车点管理
 *
 * 设计模式：
 * - 使用IndexedObstacles存储障碍物，支持快速查找
 * - 决策分为纵向和横向两类
 *
 * C++语法说明：
 * - std::string: 字符串类型
 * - const引用: 输入参数传递，避免拷贝
 * - 指针: 返回可修改的障碍物指针
 * - nullptr: C++11空指针
 */

#include "modules/planning/planning_base/common/path_decision.h"

#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
/**
 * @brief 感知障碍物消息protobuf定义
 */

#include "modules/common/configs/vehicle_config_helper.h"
/**
 * @brief 车辆配置辅助类
 */
#include "modules/common/util/util.h"
/**
 * @brief 通用工具函数
 */

namespace apollo {
/**
 * @brief Apollo顶层命名空间
 */
namespace planning {

/**
 * @brief 添加障碍物到路径决策
 *
 * @param obstacle 障碍物对象（常量引用，避免拷贝）
 * @return Obstacle* 添加的障碍物指针
 *
 * 功能说明：
 * - 将障碍物添加到障碍物索引表中
 * - IndexedObstacles::Add使用ID作为键存储障碍物
 * - 返回添加的障碍物指针，允许外部修改
 *
 * C++语法说明：
 * - const Obstacle& obstacle: 常量引用参数
 *   比值传递更高效（避免拷贝整个Obstacle对象）
 * - Obstacle*: 返回原始指针
 *   指向内部存储的障碍物，允许调用者修改
 * - obstacles_.Add(obstacle.Id(), obstacle):
 *   调用障碍物索引表的Add方法
 *   obstacle.Id()获取障碍物唯一ID
 */
Obstacle *PathDecision::AddObstacle(const Obstacle &obstacle) {
  return obstacles_.Add(obstacle.Id(), obstacle);
}

/**
 * @brief 获取障碍物索引表引用
 *
 * @return const IndexedObstacles& 障碍物索引表的常量引用
 *
 * 功能说明：
 * - 提供对障碍物集合的只读访问
 * - const引用保证数据不被修改
 *
 * C++语法说明：
 * - const IndexedObstacles&:
 *   常量引用返回，只读访问
 *   不能修改内部数据
 */
const IndexedObstacles &PathDecision::obstacles() const { return obstacles_; }

/**
 * @brief 根据ID查找障碍物（可修改版本）
 *
 * @param object_id 障碍物ID
 * @return Obstacle* 找到返回指针，未找到返回nullptr
 *
 * 功能说明：
 * - 在障碍物索引表中根据ID查找
 * - 返回可修改的指针，允许调用者修改障碍物
 *
 * C++语法说明：
 * - const std::string& object_id:
 *   常量引用字符串参数
 * - nullptr: C++11空指针常量
 *   相比NULL更类型安全
 */
Obstacle *PathDecision::Find(const std::string &object_id) {
  return obstacles_.Find(object_id);
}

/**
 * @brief 根据ID查找障碍物（只读版本）
 *
 * @param object_id 障碍物ID
 * @return const Obstacle* 找到返回常量指针，未找到返回nullptr
 *
 * 功能说明：
 * - 在障碍物索引表中根据ID查找
 * - 返回常量指针，只能读取不能修改
 *
 * C++语法说明：
 * - const Obstacle*:
 *   返回指向常量的指针
 *   确保返回的对象不被修改
 */
const Obstacle *PathDecision::Find(const std::string &object_id) const {
  return obstacles_.Find(object_id);
}

/**
 * @brief 根据感知障碍物ID查找感知消息
 *
 * @param perception_obstacle_id 感知障碍物ID（字符串形式）
 * @return const perception::PerceptionObstacle* 找到返回常量指针
 *
 * 功能说明：
 * - 遍历所有障碍物，查找感知ID匹配的对象
 * - 返回感知消息的常量指针
 *
 * C++语法说明：
 * - for (const auto *obstacle : obstacles_.Items()):
 *   范围for循环遍历障碍物列表
 *   const auto*: 指向常量的指针
 * - std::to_string():
 *   将数值转换为字符串
 * - return nullptr:
 *   未找到时返回空指针
 */
const perception::PerceptionObstacle *PathDecision::FindPerceptionObstacle(
    const std::string &perception_obstacle_id) const {
  /**
   * @brief 遍历所有障碍物
   */
  for (const auto *obstacle : obstacles_.Items()) {
    /**
     * @brief 比较感知ID是否匹配
     */
    if (std::to_string(obstacle->Perception().id()) == perception_obstacle_id) {
      return &(obstacle->Perception());
    }
  }
  return nullptr;
}

/**
 * @brief 设置障碍物的路径ST边界
 *
 * @param id 障碍物ID
 * @param boundary ST边界
 *
 * 功能说明：
 * - 根据ID查找障碍物
 * - 设置障碍物的路径ST边界
 * - ST边界定义了障碍物在时空上的占用区域
 *
 * C++语法说明：
 * - auto* obstacle = obstacles_.Find(id):
 *   auto自动类型推导
 *   pointer: 获取可修改的障碍物指针
 * - if (!obstacle):
 *   检查指针是否为空
 *   !运算符对指针有效，空指针为true
 * - obstacle->set_path_st_boundary(boundary):
 *   箭头运算符调用指针成员方法
 */
void PathDecision::SetSTBoundary(const std::string &id,
                                 const STBoundary &boundary) {
  auto *obstacle = obstacles_.Find(id);

  if (!obstacle) {
    AERROR << "Failed to find obstacle : " << id;
    return;
  } else {
    obstacle->set_path_st_boundary(boundary);
  }
}

/**
 * @brief 添加横向决策
 *
 * @param tag 决策者标签（如"PathDecider/left-nudge"）
 * @param object_id 障碍物ID
 * @param decision 横向决策类型
 * @return bool 是否添加成功
 *
 * 功能说明：
 * - 为指定障碍物添加横向决策
 * - 横向决策包括：IGNORE、NUDGE
 * - 决策会被合并到障碍物的决策列表中
 *
 * C++语法说明：
 * - const std::string&: 常量引用，避免字符串拷贝
 * - const ObjectDecisionType&:
 *   protobuf消息的常量引用
 *   避免拷贝提高效率
 * - obstacle->AddLateralDecision(tag, decision):
 *   调用障碍物方法添加决策
 */
bool PathDecision::AddLateralDecision(const std::string &tag,
                                      const std::string &object_id,
                                      const ObjectDecisionType &decision) {
  auto *obstacle = obstacles_.Find(object_id);
  if (!obstacle) {
    AERROR << "failed to find obstacle";
    return false;
  }
  obstacle->AddLateralDecision(tag, decision);
  return true;
}

/**
 * @brief 清除所有障碍物的ST边界
 *
 * 功能说明：
 * - 遍历所有障碍物
 * - 清除每个障碍物的路径ST边界
 * - 用于重置规划状态
 *
 * C++语法说明：
 * - for (const auto *obstacle : obstacles_.Items()):
 *   范围for循环遍历
 *   obstacles_.Items()返回障碍物指针列表
 * - auto* obstacle_ptr = obstacles_.Find(obstacle->Id()):
 *   重新查找获取可修改指针
 *   因为原obstacle是const指针
 * - obstacle_ptr->EraseStBoundary():
 *   调用清除方法
 */
void PathDecision::EraseStBoundaries() {
  for (const auto *obstacle : obstacles_.Items()) {
    auto *obstacle_ptr = obstacles_.Find(obstacle->Id());
    obstacle_ptr->EraseStBoundary();
  }
}

/**
 * @brief 添加纵向决策
 *
 * @param tag 决策者标签
 * @param object_id 障碍物ID
 * @param decision 纵向决策类型
 * @return bool 是否添加成功
 *
 * 功能说明：
 * - 为指定障碍物添加纵向决策
 * - 纵向决策包括：IGNORE、STOP、YIELD、FOLLOW、OVERTAKE
 * - 决策会被合并到障碍物的决策列表中
 */
bool PathDecision::AddLongitudinalDecision(const std::string &tag,
                                           const std::string &object_id,
                                           const ObjectDecisionType &decision) {
  auto *obstacle = obstacles_.Find(object_id);
  if (!obstacle) {
    AERROR << "failed to find obstacle";
    return false;
  }
  obstacle->AddLongitudinalDecision(tag, decision);
  return true;
}

/**
 * @brief 与主停车点合并
 *
 * @param obj_stop 停车决策
 * @param obj_id 障碍物ID
 * @param reference_line 参考线
 * @param adc_sl_boundary 自车SL边界
 * @return bool 是否更新主停车点
 *
 * 功能说明：
 * - 比较新停车点与当前主停车点的距离
 * - 如果新停车点更近（更安全），更新主停车点
 * - 用于确保选择最近的停车点
 *
 * 算法流程：
 * 1. 将停车点坐标转换为SL坐标
 * 2. 检查停车点是否在参考线范围内
 * 3. 比较停车距离与当前主停车点
 * 4. 如果更近，更新主停车点
 *
 * C++语法说明：
 * - common::PointENU: ENU坐标系下的点
 * - common::SLPoint: SL坐标系下的点
 * - reference_line.XYToSL(...):
 *   参考线的坐标转换方法
 * - std::fmax(...):
 *   浮点数取较大值
 *   确保停车点不会比自车前缘更近
 */
bool PathDecision::MergeWithMainStop(const ObjectStop &obj_stop,
                                     const std::string &obj_id,
                                     const ReferenceLine &reference_line,
                                     const SLBoundary &adc_sl_boundary) {
  /**
   * @brief 获取停车点坐标
   */
  common::PointENU stop_point = obj_stop.stop_point();
  common::SLPoint stop_line_sl;

  /**
   * @brief 将XY坐标转换为SL坐标
   */
  reference_line.XYToSL(stop_point, &stop_line_sl);

  /**
   * @brief 获取停车点的s坐标
   */
  double stop_line_s = stop_line_sl.s();

  /**
   * @brief 检查停车点是否在参考线范围内
   * 如果不在有效范围内，忽略此停车决策
   */
  if (stop_line_s < 0.0 || stop_line_s > reference_line.Length()) {
    AERROR << "Ignore object:" << obj_id << " fence route_s[" << stop_line_s
           << "] not in range[0, " << reference_line.Length() << "]";
    return false;
  }

  /**
   * @brief 获取车辆配置
   */
  const auto &vehicle_config = common::VehicleConfigHelper::GetConfig();

  /**
   * @brief 确保停车距离不会比自车前缘更近
   *
   * 公式：stop_line_s = max(stop_line_s, adc_end_s - front_edge_to_center)
   *
   * 原因：
   * - stop_line_s是停车点相对于参考线起点的距离
   * - 自车当前位置是adc_sl_boundary.end_s()
   * - 如果停车点比自车前缘还近，没有意义
   * - 需要留出前轴到车辆中心的距离
   */
  stop_line_s = std::fmax(
      stop_line_s, adc_sl_boundary.end_s() -
                       vehicle_config.vehicle_param().front_edge_to_center());

  /**
   * @brief 比较与当前主停车点的距离
   *
   * 如果新停车点比当前主停车点更远，不更新
   */
  if (stop_line_s >= stop_reference_line_s_) {
    ADEBUG << "stop point is farther than current main stop point.";
    return false;
  }

  /**
   * @brief 更新主停车点
   * main_stop_: 存储当前选择的主停车决策
   * stop_reference_line_s_: 主停车点的s坐标
   */

  /**
   * @brief 清除旧的主停车信息
   */
  main_stop_.Clear();

  /**
   * @brief 设置停车原因
   */
  main_stop_.set_reason_code(obj_stop.reason_code());
  main_stop_.set_reason("stop by " + obj_id);

  /**
   * @brief 设置停车点坐标
   * mutable_stop_point(): 获取可修改的停车点
   */
  main_stop_.mutable_stop_point()->set_x(obj_stop.stop_point().x());
  main_stop_.mutable_stop_point()->set_y(obj_stop.stop_point().y());

  /**
   * @brief 设置停车航向角
   */
  main_stop_.set_stop_heading(obj_stop.stop_heading());

  /**
   * @brief 更新主停车点的s坐标
   */
  stop_reference_line_s_ = stop_line_s;

  /**
   * @brief 打印调试信息
   */
  ADEBUG << " main stop obstacle id:" << obj_id
         << " stop_line_s:" << stop_line_s << " stop_point: ("
         << obj_stop.stop_point().x() << obj_stop.stop_point().y()
         << " ) stop_heading: " << obj_stop.stop_heading();
  return true;
}

/**
 * @brief 命名空间结束标记
 */
}  // namespace planning
}  // namespace apollo
