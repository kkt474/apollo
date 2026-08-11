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
 * @file ego_info.cc
 * @brief 自车信息实现文件
 *
 * 本文件实现EgoInfo类，用于存储和管理自车（自动驾驶车辆）的相关信息。
 * EgoInfo包含自车的几何形状（包围盒）、当前位置、速度等状态信息。
 *
 * 主要功能：
 * 1. 存储和更新自车状态
 * 2. 计算自车包围盒（用于碰撞检测）
 * 3. 计算前方障碍物安全距离
 * 4. 获取自车到目的地的距离
 *
 * C++语法说明：
 * - const auto&: const引用，避免拷贝
 * - Vec2d::rotate(): 二维向量旋转
 * - Vec2d::CreateUnitVec2d(): 创建单位向量
 * - Box2d: 二维包围盒类
 * - static constexpr: 静态编译时常量
 */
#include "modules/planning/planning_base/common/ego_info.h"

#include "cyber/common/log.h"  /**< Cyber RT日志系统 */
#include "modules/common/configs/vehicle_config_helper.h" /**< 车辆配置助手 */

namespace apollo {
/**
 * apollo:: - Apollo最外层命名空间
 */
namespace planning {

/**
 * using声明 - 将其他命名空间的类型引入当前作用域
 * 语法：using 命名空间::类型名;
 */
using apollo::common::math::Box2d;  /**< 二维包围盒类，用于碰撞检测 */
using apollo::common::math::Vec2d;  /**< 二维向量类 */

/**
 * @brief EgoInfo默认构造函数
 *
 * 初始化EgoInfo对象，获取车辆配置参数。
 *
 * 语法说明：
 * - EgoInfo(): 构造函数，与类名同名，无返回类型
 * - common::VehicleConfigHelper::GetConfig(): 静态方法，获取车辆配置单例
 * - ego_vehicle_config_: 类成员变量，存储车辆配置
 */
EgoInfo::EgoInfo() {
  /**
   * VehicleConfigHelper::GetConfig() - 获取车辆配置
   * 这是一个静态单例方法，返回车辆配置对象
   * 配置包含：车辆尺寸（长、宽）、边缘到中心的距离等
   */
  ego_vehicle_config_ = common::VehicleConfigHelper::GetConfig();
}

/**
 * @brief 更新自车信息
 *
 * 根据给定的轨迹起始点和车辆状态更新自车信息。
 * 这是EgoInfo的主要更新接口，在每个规划周期调用。
 *
 * @param start_point 规划起始点（轨迹点格式）
 * @param vehicle_state 当前车辆状态
 * @return bool 更新是否成功（始终返回true）
 *
 * 语法说明：
 * - const common::TrajectoryPoint&: 常量引用参数，只读
 * - set_start_point(): 设置起始点
 * - set_vehicle_state(): 设置车辆状态
 * - CalculateEgoBox(): 计算自车包围盒
 */
bool EgoInfo::Update(const common::TrajectoryPoint& start_point,
                     const common::VehicleState& vehicle_state) {
  set_start_point(start_point);   /**< 保存规划起始点 */
  set_vehicle_state(vehicle_state); /**< 保存车辆状态 */
  CalculateEgoBox(vehicle_state); /**< 计算自车包围盒 */
  return true;  /**< 更新成功 */
}

/**
 * @brief 计算自车包围盒
 *
 * 根据车辆状态和车辆参数计算自车的二维包围盒。
 * 包围盒用于后续的碰撞检测计算。
 *
 * 车辆坐标系：
 * - 以车辆后轴中心为原点
 * - X轴指向前方，Y轴指向左侧
 * - heading为航向角（相对于世界坐标系）
 *
 * @param vehicle_state 车辆状态
 *
 * 语法说明：
 * - const auto& param: const引用，避免拷贝
 * - ego_vehicle_config_.vehicle_param(): 获取车辆参数子配置
 * - param.front_edge_to_center(): 车辆前边缘到中心距离
 * - param.back_edge_to_center(): 车辆后边缘到中心距离
 * - Vec2d(x, y): 二维向量构造函数
 */
void EgoInfo::CalculateEgoBox(const common::VehicleState& vehicle_state) {
  /**
   * 获取车辆参数引用
   * ego_vehicle_config_.vehicle_param() - 返回车辆参数配置
   * const auto&: const引用，提高效率
   */
  const auto& param = ego_vehicle_config_.vehicle_param();
  ADEBUG << "param: " << param.DebugString(); /**< 调试日志输出参数 */

  /**
   * 计算从后轴中心到几何中心的向量
   *
   * vec_to_center = ((前边缘-后边缘)/2, (左边缘-右边缘)/2)
   *
   * 由于车辆参数给出的是边缘到中心的距离：
   * - front_edge_to_center: 前边缘到中心
   * - back_edge_to_center: 后边缘到中心
   * - left_edge_to_center: 左边缘到中心
   * - right_edge_to_center: 右边缘到中心
   *
   * 几何中心 = 后轴中心 + vec_to_center
   *
   * 示例：
   * - 如果前边缘距中心2米，后边缘距中心1米
   * - 则几何中心距后轴中心 = (2 - 1) / 2 = 0.5米
   */
  Vec2d vec_to_center(
      (param.front_edge_to_center() - param.back_edge_to_center()) / 2.0,
      (param.left_edge_to_center() - param.right_edge_to_center()) / 2.0);

  /**
   * 创建自车当前位置向量
   * Vec2d(x, y): 使用车辆状态的X、Y坐标构造向量
   */
  Vec2d position(vehicle_state.x(), vehicle_state.y());

  /**
   * 计算自车几何中心
   * 几何中心 = 当前位置 + vec_to_center.rotate(heading)
   *
   * vec_to_center.rotate(heading):
   *   - 将vec_to_center绕原点旋转heading角度
   *   - 因为vec_to_center是在车辆局部坐标系下计算的
   *   - 需要旋转到世界坐标系
   */
  Vec2d center(position + vec_to_center.rotate(vehicle_state.heading()));

  /**
   * 创建自车包围盒
   * Box2d(center, heading, length, width)
   *   - center: 包围盒中心点
   *   - heading: 航向角
   *   - length: 车辆长度
   *   - width: 车辆宽度
   */
  ego_box_ =
      Box2d(center, vehicle_state.heading(), param.length(), param.width());
}

/**
 * @brief 清空自车信息
 *
 * 重置EgoInfo的所有成员变量到初始状态。
 *
 * 语法说明：
 * - .Clear(): Protobuf消息的方法，清空所有字段
 * - front_clear_distance_: 前方清空距离
 * - FLAGS_default_front_clear_distance: 默认前方清空距离配置
 */
void EgoInfo::Clear() {
  start_point_.Clear();   /**< 清空起始点 */
  vehicle_state_.Clear(); /**< 清空车辆状态 */
  front_clear_distance_ = FLAGS_default_front_clear_distance; /**< 重置前方清空距离 */
}

/**
 * @brief 计算前方障碍物清空距离
 *
 * 计算自车到前方障碍物的距离。
 * 注意：此函数已被标记为TODO，计划在未来移除。
 *
 * 原因：
 * 1. 航向角不一定与道路对齐
 * 2. 道路不一定是直的
 *
 * @param obstacles 障碍物列表
 *
 * 语法说明：
 * - const std::vector<const Obstacle*>&: 常量引用，障碍物指针向量
 * - IsVirtual(): 判断是否为虚拟障碍物
 * - HasOverlap(): 判断两个包围盒是否有重叠
 * - static constexpr: 静态编译时常量
 */
void EgoInfo::CalculateFrontObstacleClearDistance(
    const std::vector<const Obstacle*>& obstacles) {
  /**
   * 创建自车位置向量
   */
  Vec2d position(vehicle_state_.x(), vehicle_state_.y());

  /**
   * 获取车辆参数（同CalculateEgoBox）
   */
  const auto& param = ego_vehicle_config_.vehicle_param();

  /**
   * 计算几何中心向量（同CalculateEgoBox）
   */
  Vec2d vec_to_center(
      (param.front_edge_to_center() - param.back_edge_to_center()) / 2.0,
      (param.left_edge_to_center() - param.right_edge_to_center()) / 2.0);

  /**
   * 计算几何中心（同CalculateEgoBox）
   */
  Vec2d center(position + vec_to_center.rotate(vehicle_state_.heading()));

  /**
   * 创建航向单位向量
   * Vec2d::CreateUnitVec2d(angle): 创建指定角度的单位向量
   * 单位向量 = (cos(angle), sin(angle))
   */
  Vec2d unit_vec_heading = Vec2d::CreateUnitVec2d(vehicle_state_.heading());

  /**
   * static constexpr: 静态编译时常量
   * kDistanceThreshold: 有效距离阈值（50米）
   *   - 由于自车航向误差，超过此距离的测量不太可靠
   */
  // Due to the error of ego heading, only short range distance is meaningful
  static constexpr double kDistanceThreshold = 50.0;  /**< 有效距离阈值：50米 */
  static constexpr double buffer = 0.1;  /**< 缓冲区：0.1米 */

  /**
   * 计算影响区域长度
   * impact_region_length = 车辆长度 + 缓冲区 + 阈值
   */
  const double impact_region_length =
      param.length() + buffer + kDistanceThreshold;

  /**
   * 创建自车前方影响区域包围盒
   * center + unit_vec_heading * kDistanceThreshold / 2.0
   *   - 将中心点沿航向方向前移阈值的一半
   *   - 作为前方区域包围盒的中心
   *
   * Box2d: 前方区域（比自车更长，用于检测前方障碍物）
   */
  Box2d ego_front_region(center + unit_vec_heading * kDistanceThreshold / 2.0,
                         vehicle_state_.heading(), impact_region_length,
                         param.width() + buffer);

  /**
   * 遍历所有障碍物
   * for (const auto& obstacle : obstacles)
   *   - range-based for循环，C++11
   *   - const auto&: const引用，避免拷贝
   */
  for (const auto& obstacle : obstacles) {
    /**
     * 跳过虚拟障碍物和不与前方区域重叠的障碍物
     *
     * IsVirtual(): 判断是否为虚拟障碍物（如停车墙）
     * HasOverlap(): 判断两个Box2d是否有重叠区域
     * PerceptionBoundingBox(): 获取障碍物的感知包围盒
     */
    if (obstacle->IsVirtual() ||
        !ego_front_region.HasOverlap(obstacle->PerceptionBoundingBox())) {
      continue;  /**< 跳过不相关的障碍物 */
    }

    /**
     * 计算到障碍物的距离
     * dist = 自车中心到障碍物中心的距离 - 自车对角线的一半
     *
     * ego_box_.center(): 获取自车包围盒中心
     * .DistanceTo(): 计算到另一点的欧氏距离
     * obstacle->PerceptionBoundingBox().center(): 障碍物包围盒中心
     * ego_box_.diagonal() / 2.0: 自车对角线的一半（近似为等效半径）
     */
    double dist = ego_box_.center().DistanceTo(
                      obstacle->PerceptionBoundingBox().center()) -
                  ego_box_.diagonal() / 2.0;

    /**
     * 更新最小清空距离
     * front_clear_distance_: 前方清空距离
     * 初始值为负数，表示未设置
     * 如果dist < front_clear_distance_，更新为更小的值
     */
    if (front_clear_distance_ < 0.0 || dist < front_clear_distance_) {
      front_clear_distance_ = dist;  /**< 更新最小清空距离 */
    }
  }
}

/**
 * @brief 计算当前路线信息
 *
 * 从参考线提供者获取自车当前所在车道和到目的地的距离。
 *
 * @param reference_line_provider 参考线提供者指针
 *
 * 语法说明：
 * - const ReferenceLineProvider*: 原始指针，可为空
 * - GetAdcWaypoint(): 获取自车在参考线上的位置（Waypoint）
 * - GetAdcDis2Destination(): 获取自车到目的地的距离
 * - adc_waypoint_: 自车所在车道的导航点
 * - distance_to_destination_: 到目的地的距离
 */
void EgoInfo::CalculateCurrentRouteInfo(
    const ReferenceLineProvider* reference_line_provider) {
  /**
   * GetAdcWaypoint: 获取自车导航点
   * 将结果写入adc_waypoint_成员变量
   * ADC = Autonomous Driving Computer（自动驾驶计算机）
   */
  reference_line_provider->GetAdcWaypoint(&adc_waypoint_);

  /**
   * GetAdcDis2Destination: 获取到目的地距离
   * 将结果写入distance_to_destination_成员变量
   */
  reference_line_provider->GetAdcDis2Destination(&distance_to_destination_);

  /**
   * AINFO - 信息级别日志
   * DebugString() - Protobuf消息的调试字符串表示
   */
  AINFO << adc_waypoint_.DebugString();
  AINFO << "distance_to_destination: " << distance_to_destination_;
}
}  // namespace planning
}  // namespace apollo
