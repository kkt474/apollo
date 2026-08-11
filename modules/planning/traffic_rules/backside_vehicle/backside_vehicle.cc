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
 * @file backside_vehicle.cc
 * @brief 后方来车交通规则实现文件
 *
 * 本文件实现了后方来车交通规则(BacksideVehicle)
 * 用于处理自车后方的障碍物（超车场景）
 *
 * 功能说明：
 * 1. 判断障碍物是否在自车后方
 * 2. 决定是否忽略后方来车
 * 3. 处理预测轨迹与自车的重叠检测
 *
 * 场景说明：
 * 当车辆从后方超车时，自车规划系统需要正确判断：
 * - 后方来车不应影响自车的车道保持决策
 * - 但如果超车轨迹会与自车碰撞，则不能忽略
 *
 * 忽略条件：
 * 1. 障碍物完全在自车后方（end_s < adc.start_s）
 * 2. 障碍物与自车没有ST重叠区域
 * 3. 障碍物预测轨迹的最小S < -adc_length
 * 4. 障碍物在相邻车道且不在自车行驶路径上
 * 5. 预测轨迹与自车没有碰撞
 *
 * 相关C++语法说明：
 * - std::shared_ptr<T>: 智能指针，共享所有权
 * - const成员函数: 承诺不修改成员变量
 * - Polygon2d/Box2d: 用于碰撞检测的几何图形
 **/

/**
 * @brief 本类的头文件
 *
 * 包含BacksideVehicle类的完整定义
 */
#include "modules/planning/traffic_rules/backside_vehicle/backside_vehicle.h"

/**
 * @brief 标准库头文件
 *
 * <memory>: 提供std::shared_ptr智能指针
 * <string>: 提供std::string字符串类型
 */
#include <memory>
#include <string>

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
using apollo::common::Status;              /**< Apollo通用状态类型 */
using apollo::common::VehicleState;        /**< 车辆状态类型 */
using apollo::common::math::Box2d;          /**< 二维边界框类型 */
using apollo::common::math::Polygon2d;      /**< 二维多边形类型 */

/**
 * @brief 后方来车规则初始化函数
 *
 * @param name 规则名称
 * @param injector 依赖注入器智能指针
 * @return bool 初始化是否成功
 *
 * 功能说明：
 * 1. 调用父类TrafficRule的Init进行基础初始化
 * 2. 加载本规则特有的配置BacksideVehicleConfig
 *
 * 算法流程：
 * 1. 调用TrafficRule::Init进行基础初始化
 * 2. 如果基础初始化失败，返回false
 * 3. 调用TrafficRule::LoadConfig加载本规则配置
 * 4. 返回加载结果
 *
 * C++语法说明：
 * - const std::string& name:
 *   常量引用，规则名称用于标识和日志
 * - const std::shared_ptr<DependencyInjector>& injector:
 *   共享所有权的智能指针，依赖注入器
 * - if (!TrafficRule::Init(...)):
 *   条件语句，调用父类初始化并检查结果
 */
bool BacksideVehicle::Init(
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
   *   模板方法，从配置文件加载BacksideVehicleConfig类型配置
   *   配置内容可能包括：后方车道宽度等参数
   *
   * C++语法说明：
   * - LoadConfig<BacksideVehicleConfig>:
   *   模板实例化，指定配置类型
   * - &config_:
   *   取地址运算符，将配置写入成员变量
   */
  return TrafficRule::LoadConfig<BacksideVehicleConfig>(&config_);
}

/**
 * @brief 制作车道保持障碍物决策
 *
 * @param adc_sl_boundary 自车SL边界
 * @param path_decision 路径决策指针
 * @param vehicle_state 车辆状态
 *
 * 功能说明：
 * 判断自车道保持过程中，哪些后方障碍物可以忽略
 * 这是BacksideVehicle规则的核心逻辑
 *
 * 忽略逻辑：
 * 遍历所有障碍物，判断是否满足忽略条件
 * 如果满足，则添加IGNORE决策
 *
 * C++语法说明：
 * - const SLBoundary& adc_sl_boundary:
 *   自车SL边界，常量引用
 * - PathDecision* path_decision:
 *   原始指针，可修改路径决策
 * - const VehicleState& vehicle_state:
 *   车辆状态，常量引用
 */
void BacksideVehicle::MakeLaneKeepingObstacleDecision(
    const SLBoundary& adc_sl_boundary, PathDecision* path_decision,
    const VehicleState& vehicle_state) {
  /**
   * @brief 创建IGNORE决策
   *
   * ObjectDecisionType:
   *   障碍物决策类型，包含STOP/FOLLOW/YIELD/OVERTAKE/IGNORE等
   *
   * C++语法说明：
   * - ObjectDecisionType ignore:
   *   创建空的决策对象
   * - ignore.mutable_ignore():
   *   mutable_前缀是protobuf的特殊方法
   *   用于在const对象上获取可修改的成员
   *   将决策类型设置为IGNORE
   */
  ObjectDecisionType ignore;
  ignore.mutable_ignore();

  /**
   * @brief 计算自车长度
   *
   * adc_length_s = end_s - start_s:
   *   自车在S方向的长度
   *   end_s是后边缘，start_s是前边缘
   *
   * SLBoundary说明：
   * - start_s: S方向的起始位置（前边缘）
   * - end_s: S方向的结束位置（后边缘）
   * - start_l/end_l: L方向的边界
   */
  const double adc_length_s =
      adc_sl_boundary.end_s() - adc_sl_boundary.start_s();

  /**
   * @brief 遍历路径决策中的所有障碍物
   *
   * for (const auto* obstacle : path_decision->obstacles().Items()):
   *   范围for循环，遍历障碍物列表
   *   obstacle是const Obstacle*类型
   *
   * C++语法说明：
   * - path_decision->obstacles().Items():
   *   ->调用成员函数，.获取成员
   *   Items()返回所有障碍物的列表
   */
  for (const auto* obstacle : path_decision->obstacles().Items()) {
    /**
     * @brief 条件1：障碍物不在自车后方
     *
     * obstacle->PerceptionSLBoundary().end_s() >= adc_sl_boundary.start_s():
     *   障碍物的后边缘 >= 自车的前边缘
     *   说明障碍物至少与自车有部分重叠
     *   不能忽略
     *
     * 示意图：
     * |<--adc-->|
     *           |<--obstacle-->|  (不忽略)
     * |<--obstacle-->|          (不忽略)
     *              |<--obstacle-->|  (需要继续判断)
     */
    if (obstacle->PerceptionSLBoundary().end_s() >= adc_sl_boundary.start_s()) {
      // don't ignore such vehicles.
      continue;  /**< 不忽略，继续下一个障碍物 */
    }

    /**
     * @brief 条件2：障碍物不是危险级别
     *
     * IsCautionLevelObstacle():
     *   检查障碍物是否需要特别注意
     *   如紧急车辆、异常行为车辆等
     *
     * && 第二个条件：
     *   障碍物的起始S > 自车的起始S
     *   排除掉完全在自车后方的障碍物
     *   这两个条件确保只处理特定范围内的障碍物
     */
    if (obstacle->IsCautionLevelObstacle() &&
          obstacle->PerceptionSLBoundary().start_s() >=
              adc_sl_boundary.start_s()) {
      // don't ignore such vehicles.
      continue;  /**< 不忽略，继续下一个 */
    }

    /**
     * @brief 条件3：障碍物没有ST边界
     *
     * reference_line_st_boundary().IsEmpty():
     *   检查障碍物在参考线上的ST边界是否为空
     *   如果为空，说明障碍物不在考虑范围内
     *
     * 添加IGNORE决策：
     * - AddLongitudinalDecision: 添加纵向决策
     * - AddLateralDecision: 添加横向决策
     * - "backside_vehicle/no-st-region": 决策标签/来源
     */
    if (obstacle->reference_line_st_boundary().IsEmpty()) {
      path_decision->AddLongitudinalDecision("backside_vehicle/no-st-region",
                                             obstacle->Id(), ignore);
      path_decision->AddLateralDecision("backside_vehicle/no-st-region",
                                        obstacle->Id(), ignore);
      continue;  /**< 已添加IGNORE决策，继续下一个 */
    }

    /**
     * @brief 条件4：障碍物在自车后方足够远
     *
     * reference_line_st_boundary().min_s() < -adc_length_s:
     *   障碍物的最小S < -自车长度
     *   说明障碍物的预测轨迹完全在自车后方
     *   可以安全忽略
     *
     * 示意图：
     * |<--adc-->|
     *             |<--obstacle-->|  (min_s < -adc_length_s，可忽略)
     */
    // Ignore the car comes from back of ADC
    if (obstacle->reference_line_st_boundary().min_s() < -adc_length_s) {
      path_decision->AddLongitudinalDecision("backside_vehicle/st-min-s < adc",
                                             obstacle->Id(), ignore);
      path_decision->AddLateralDecision("backside_vehicle/st-min-s < adc",
                                        obstacle->Id(), ignore);
      continue;  /**< 已添加IGNORE决策 */
    }

    /**
     * @brief 条件5：检查车道边界
     *
     * 获取配置的后方车道宽度
     * lane_boundary = config_.backside_lane_width()
     */
    const double lane_boundary = config_.backside_lane_width();

    /**
     * @brief 条件5.1：障碍物起始S < 自车结束S
     *
     * 障碍物在自车后方，但在可触及范围内
     */
    if (obstacle->PerceptionSLBoundary().start_s() < adc_sl_boundary.end_s()) {
      /**
       * @brief 检查障碍物是否在相邻车道
       *
       * start_l() > lane_boundary:
       *   障碍物左侧 > 车道边界
       *   说明在自车左侧车道
       *
       * end_l() < -lane_boundary:
       *   障碍物右侧 < -车道边界
       *   说明在自车右侧车道
       *
       * 如果满足任一条件，说明不在自车行驶路径上
       */
      if (obstacle->PerceptionSLBoundary().start_l() > lane_boundary ||
          obstacle->PerceptionSLBoundary().end_l() < -lane_boundary) {
        continue;  /**< 在相邻车道，不忽略当前障碍物 */
      }

      /**
       * @brief 添加IGNORE决策
       *
       * 虽然障碍物在自车后方
       * 但由于在相邻车道且不在行驶路径上，可以忽略
       */
      path_decision->AddLongitudinalDecision("backside_vehicle/sl < adc.end_s",
                                             obstacle->Id(), ignore);
      path_decision->AddLateralDecision("backside_vehicle/sl < adc.end_s",
                                        obstacle->Id(), ignore);
      continue;  /**< 已添加IGNORE决策 */
    }

    /**
     * @brief 条件6：预测轨迹与自车碰撞检测
     *
     * PredictionLineOverlapEgo():
     *   检查障碍物的预测轨迹是否与自车有重叠
     *   如果有重叠，不能忽略（可能发生碰撞）
     */
    if (PredictionLineOverlapEgo(*obstacle, vehicle_state)) {
      ADEBUG << "Prediction Line Overlap Ego Obstacle " << obstacle->Id();

      /**
       * @brief 预测轨迹与自车重叠，不忽略
       *
       * 虽然障碍物在后方
       * 但如果超车轨迹会与自车碰撞，需要保持警惕
       */
      path_decision->AddLongitudinalDecision("backside_vehicle/"
                                              "prediction line overlap ego",
                                              obstacle->Id(), ignore);
      path_decision->AddLateralDecision("backside_vehicle/"
                                        "prediction line overlap ego",
                                        obstacle->Id(), ignore);
      continue;  /**< 不忽略，继续下一个 */
    }
  }
}

/**
 * @brief 应用交通规则
 *
 * @param frame 当前规划帧（未使用）
 * @param reference_line_info 参考线信息
 * @return Status 应用状态
 *
 * 功能说明：
 * 交通规则的入口函数
 * 由TrafficDecider::Execute调用
 *
 * 执行流程：
 * 1. 获取路径决策
 * 2. 获取自车SL边界
 * 3. 获取车辆状态
 * 4. 如果在车道保持状态，调用MakeLaneKeepingObstacleDecision
 *
 * C++语法说明：
 * - Frame* const:
 *   指向常量的指针（指针本身是常量）
 *   参数frame在函数内不会被修改指向
 * - ReferenceLineInfo* const reference_line_info:
 *   指向常量的指针
 */
Status BacksideVehicle::ApplyRule(
    Frame* const, ReferenceLineInfo* const reference_line_info) {
  /**
   * @brief 获取路径决策
   *
   * reference_line_info->path_decision():
   *   获取参考线信息中的路径决策指针
   */
  auto* path_decision = reference_line_info->path_decision();

  /**
   * @brief 获取自车SL边界
   *
   * reference_line_info->AdcSlBoundary():
   *   获取自车在SL坐标系下的边界框
   */
  const auto& adc_sl_boundary = reference_line_info->AdcSlBoundary();

  /**
   * @brief 获取车辆状态
   *
   * reference_line_info->vehicle_state():
   *   获取当前帧的车辆状态
   */
  const VehicleState& vehicle_state = reference_line_info->vehicle_state();

  /**
   * @brief 检查是否在车道保持状态
   *
   * reference_line_info->Lanes().IsOnSegment():
   *   检查自车是否在参考线段上
   *   IsOnSegment()返回true表示在车道保持状态
   *
   * Lanes:
   *   参考线的车道信息
   *   IsOnSegment():
   *     判断是否在正常的车道保持模式
   */
  // The lane keeping reference line.
  if (reference_line_info->Lanes().IsOnSegment()) {
    /**
     * @brief 调用车道保持障碍物决策函数
     *
     * MakeLaneKeepingObstacleDecision():
     *   判断哪些后方障碍物可以忽略
     */
    MakeLaneKeepingObstacleDecision(adc_sl_boundary, path_decision,
                                    vehicle_state);
  }

  return Status::OK();  /**< 执行成功 */
}

/**
 * @brief 检查预测轨迹是否与自车重叠
 *
 * @param obstacle 障碍物
 * @param vehicle_state 车辆状态
 * @return bool 是否重叠（碰撞）
 *
 * 功能说明：
 * 检查障碍物的预测轨迹是否与自车有重叠
 * 用于判断超车时是否会与自车发生碰撞
 *
 * 算法流程：
 * 1. 获取障碍物的预测轨迹
 * 2. 构建自车的多边形
 * 3. 遍历轨迹的每个点
 * 4. 获取该时刻障碍物的多边形
 * 5. 检查两个多边形是否重叠
 *
 * C++语法说明：
 * - const Obstacle& obstacle:
 *   常量引用，障碍物引用
 * - Polygon2d::HasOverlap():
 *   检查两个多边形是否重叠/相交
 */
bool BacksideVehicle::PredictionLineOverlapEgo(
    const Obstacle& obstacle, const VehicleState& vehicle_state) {
  /**
   * @brief 获取障碍物的预测轨迹
   *
   * obstacle.Trajectory().trajectory_point():
   *   获取轨迹点列表
   *   Trajectory是预测轨迹消息类型
   *   trajectory_point()返回轨迹点向量
   */
  const auto& trajectory = obstacle.Trajectory().trajectory_point();

  /**
   * @brief 检查轨迹是否为空
   *
   * 如果轨迹为空（静态障碍物），返回false
   * 静态障碍物不需要检查预测轨迹重叠
   */
  if (trajectory.empty()) return false;

  /**
   * @brief 获取车辆参数
   *
   * common::VehicleConfigHelper::GetConfig().vehicle_param():
   *   获取车辆配置单例，然后获取车辆参数
   */
  const auto vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();

  /**
   * @brief 构建自车边界框
   *
   * Box2d构造函数参数：
   * 1. center: 中心点坐标 {x, y}
   * 2. heading: 朝向角（弧度）
   * 3. length: 长度
   * 4. width: 宽度
   *
   * C++语法说明：
   * - vehicle_state.x(), vehicle_state.y():
   *   获取车辆状态的x, y坐标
   * - vehicle_state.heading():
   *   获取航向角
   */
  Box2d adc_box({vehicle_state.x(), vehicle_state.y()}, vehicle_state.heading(),
                vehicle_param.length(), vehicle_param.width());

  /**
   * @brief 将边界框转换为多边形
   *
   * Polygon2d(adc_box):
   *   从Box2d创建Polygon2d
   *   多边形用于更精确的碰撞检测
   */
  Polygon2d adc_polygon(adc_box);

  /**
   * @brief 遍历预测轨迹的每个点
   *
   * for (auto& point : trajectory):
   *   范围for循环
   *   point是轨迹中的每个时刻点
   */
  for (auto& point : trajectory) {
    /**
     * @brief 获取障碍物在该时刻的多边形
     *
     * obstacle.GetObstacleTrajectoryPolygon(point):
     *   根据轨迹点计算障碍物的旋转后的边界框
     *   返回Polygon2d类型
     */
    Polygon2d obs_polygon = obstacle.GetObstacleTrajectoryPolygon(point);

    /**
     * @brief 检查多边形是否重叠
     *
     * adc_polygon.HasOverlap(obs_polygon):
     *   检查自车多边形和障碍物多边形是否相交/重叠
     *   返回true表示有重叠（碰撞）
     */
    if (adc_polygon.HasOverlap(obs_polygon)) {
      return true;  /**< 发生重叠/碰撞 */
    }
  }

  return false;  /**< 没有重叠 */
}

}  // namespace planning
}  // namespace apollo
