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
 * @file trajectory_stitcher.cc
 * @brief 轨迹拼接器实现文件
 *
 * 本文件实现TrajectoryStitcher类，负责将上一轮规划的轨迹与新一轮规划进行拼接。
 * 轨迹拼接是连续规划的关键技术，确保规划轨迹的平滑性和连续性。
 *
 * 主要功能：
 * 1. 从车辆状态计算轨迹点
 * 2. 判断是否需要重规划
 * 3. 计算轨迹拼接点
 * 4. 处理导航模式下的轨迹变换
 *
 * C++语法说明：
 * - std::vector<T>: 动态数组，支持随机访问
 * - std::pair<T1, T2>: 模板类，表示一对值
 * - std::for_each(begin, end, func): 对范围内每个元素执行函数
 * - std::abs/fabs: 绝对值函数（整数/浮点数）
 * - static_cast<T>: 编译时类型转换
 * - lambda表达式: [capture](params){body} 匿名函数
 * - constexpr: 编译时常量表达式
 */
#include "modules/planning/planning_base/common/trajectory_stitcher.h"

#include <algorithm>              /**< C++标准算法库，包含std::for_each, std::min, std::max等 */

#include "absl/strings/str_cat.h" /**< Abseil字符串拼接库 */

#include "cyber/common/log.h"     /**< Cyber RT日志系统 */
#include "modules/common/configs/config_gflags.h" /**< Apollo配置标志 */
#include "modules/common/math/angle.h"   /**< 角度数学库，包含NormalizeAngle等 */
#include "modules/common/math/quaternion.h" /**< 四元数数学库 */
#include "modules/common/util/util.h"    /**< 通用工具函数 */
#include "modules/common/vehicle_model/vehicle_model.h" /**< 车辆模型 */
#include "modules/planning/planning_base/gflags/planning_gflags.h" /**< 规划配置标志 */

namespace apollo {
/**
 * apollo:: - Apollo最外层命名空间
 */
namespace planning {

/**
 * using声明 - 将其他命名空间的类型引入当前作用域
 * 语法：using 命名空间::类型名;
 * 之后可以直接使用类型名而不需要完整命名空间前缀
 */
using apollo::common::TrajectoryPoint;  /**< 轨迹点类型，包含位置、速度、加速度等信息 */
using apollo::common::VehicleModel;    /**< 车辆模型类，提供车辆状态预测功能 */
using apollo::common::VehicleState;    /**< 车辆状态类型 */
using apollo::common::math::Vec2d;     /**< 二维向量类 */

/**
 * @brief 从车辆状态计算轨迹点
 *
 * 将车辆状态（位置、速度、加速度等）转换为一个轨迹点。
 * 用于从当前车辆状态初始化轨迹。
 *
 * @param planning_cycle_time 规划周期时间（秒）
 * @param vehicle_state 车辆状态
 * @return TrajectoryPoint 转换后的轨迹点
 *
 * 语法说明：
 * - mutable_path_point(): 获取PathPoint的可写指针
 * - set_xxx(): Protobuf设置方法，设置对应字段值
 * - set_theta(): 设置航向角（弧度）
 * - set_kappa(): 设置曲率（1/半径）
 * - set_v(): 设置线速度（m/s）
 * - set_a(): 设置线加速度（m/s²）
 * - set_relative_time(): 设置相对时间（相对于规划起始点）
 */
TrajectoryPoint TrajectoryStitcher::ComputeTrajectoryPointFromVehicleState(
    const double planning_cycle_time, const VehicleState& vehicle_state) {
  TrajectoryPoint point;  /**< 创建空轨迹点 */

  /**
   * 设置路径点信息
   * mutable_path_point() - 获取可写的PathPoint指针
   * set_s(0.0) - s坐标设为0（起始点）
   * set_x/y/z() - 设置笛卡尔坐标
   */
  point.mutable_path_point()->set_s(0.0);
  point.mutable_path_point()->set_x(vehicle_state.x());     /**< 设置X坐标 */
  point.mutable_path_point()->set_y(vehicle_state.y());     /**< 设置Y坐标 */
  point.mutable_path_point()->set_z(vehicle_state.z());     /**< 设置Z坐标（高度） */
  point.mutable_path_point()->set_theta(vehicle_state.heading()); /**< 设置航向角 */
  point.mutable_path_point()->set_kappa(vehicle_state.kappa());   /**< 设置曲率 */

  /**
   * 设置动力学参数
   */
  point.set_v(vehicle_state.linear_velocity());              /**< 设置线速度 */
  point.set_a(vehicle_state.linear_acceleration());          /**< 设置线加速度 */
  point.set_relative_time(planning_cycle_time);              /**< 设置相对时间 */
  return point;  /**< 返回构造好的轨迹点 */
}

/**
 * @brief 计算重规划拼接轨迹
 *
 * 当需要完全重规划时，计算新的拼接轨迹起点。
 * 如果车辆基本静止（速度、加速度都很小），直接使用当前车辆状态；
 * 否则使用车辆模型预测一个周期后的状态。
 *
 * @param planning_cycle_time 规划周期时间（秒）
 * @param vehicle_state 车辆状态
 * @return std::vector<TrajectoryPoint> 包含单个重规划点的向量
 *
 * 语法说明：
 * - static constexpr double: 静态编译时常量
 *   - constexpr表示编译时可求值
 *   - static表示类级别共享
 * - std::abs() / std::fabs(): 绝对值函数（整数abs，浮点数fabs）
 * - VehicleModel::Predict(): 静态方法，使用车辆模型预测未来状态
 * - std::vector<T>(n, value): 构造包含n个value的向量
 */
std::vector<TrajectoryPoint>
TrajectoryStitcher::ComputeReinitStitchingTrajectory(
    const double planning_cycle_time, const VehicleState& vehicle_state) {
  TrajectoryPoint reinit_point;  /**< 重规划点 */

  /**
   * kEpsilon_v/kEpsilon_a - 速度/加速度阈值常量
   * static constexpr: 编译时常量，类级别共享
   * 用于判断车辆是否基本静止
   */
  static constexpr double kEpsilon_v = 0.1;   /**< 速度阈值：0.1 m/s */
  static constexpr double kEpsilon_a = 0.4;   /**< 加速度阈值：0.4 m/s² */
  // TODO(Jinyun/Yu): adjust kEpsilon if corrected IMU acceleration provided

  /**
   * 判断车辆是否基本静止
   * std::abs() - 整数/浮点数绝对值
   * 条件：|速度| < 0.1 且 |加速度| < 0.4
   */
  if (std::abs(vehicle_state.linear_velocity()) < kEpsilon_v &&
      std::abs(vehicle_state.linear_acceleration()) < kEpsilon_a) {
    /**
     * 车辆基本静止，直接使用当前状态创建轨迹点
     */
    reinit_point = ComputeTrajectoryPointFromVehicleState(planning_cycle_time,
                                                          vehicle_state);
  } else {
    /**
     * 车辆在运动，使用车辆模型预测
     * VehicleModel::Predict() - 静态方法
     * 根据运动模型预测planning_cycle_time后的车辆状态
     */
    VehicleState predicted_vehicle_state;
    predicted_vehicle_state =
        VehicleModel::Predict(planning_cycle_time, vehicle_state);
    reinit_point = ComputeTrajectoryPointFromVehicleState(
        planning_cycle_time, predicted_vehicle_state);
  }

  /**
   * 返回包含单个重规划点的向量
   * std::vector<T>(n, value) - 构造大小为n的向量，所有元素为value的拷贝
   */
  return std::vector<TrajectoryPoint>(1, reinit_point);
}

/**
 * @brief 变换上一轮发布的轨迹（仅用于导航模式）
 *
 * 导航模式下，由于没有精确的参考线定位，需要对上一轮轨迹进行坐标变换。
 * 使用旋转矩阵R和平移向量t进行变换：
 *   P_new = R⁻¹ * (P_old - t)
 *
 * @param x_diff X方向平移量
 * @param y_diff Y方向平移量
 * @param theta_diff 航向角差值
 * @param prev_trajectory 上一轮轨迹指针
 *
 * 语法说明：
 * - std::cos/sin: 标准库三角函数
 * - std::for_each(begin, end, func): 对[begin, end)范围内每个元素执行func
 * - lambda表达式: [&cos_theta, &sin_theta, &tx, &ty, &theta_diff](common::TrajectoryPoint& p){...}
 *   - [&...] 表示捕获列表，以引用方式捕获外部变量
 *   - (common::TrajectoryPoint& p) 为参数
 * - common::math::NormalizeAngle(): 将角度归一化到[-π, π]范围
 */
void TrajectoryStitcher::TransformLastPublishedTrajectory(
    const double x_diff, const double y_diff, const double theta_diff,
    PublishableTrajectory* prev_trajectory) {
  /**
   * 空指针检查
   */
  if (!prev_trajectory) {
    return;  /**< 无轨迹直接返回 */
  }

  /**
   * 计算旋转矩阵的逆矩阵元素
   * R = [cosθ, -sinθ]
   *     [sinθ,  cosθ]
   * R⁻¹ = [cosθ, sinθ]
   *       [-sinθ, cosθ]
   *
   * cos_theta = cos(θ_diff)
   * sin_theta = -sin(θ_diff)  (R⁻¹的第二行第一列)
   */
  // R^-1
  double cos_theta = std::cos(theta_diff);   /**< cos(θ_diff) */
  double sin_theta = -std::sin(theta_diff);  /**< -sin(θ_diff) */

  /**
   * 计算平移向量的逆变换
   * -R⁻¹ * t = -[cosθ, sinθ; -sinθ, cosθ] * [x_diff; y_diff]
   *
   * tx = -(cosθ * x_diff - sinθ * y_diff)
   * ty = -(sinθ * x_diff + cosθ * y_diff)
   */
  // -R^-1 * t
  auto tx = -(cos_theta * x_diff - sin_theta * y_diff); /**< 逆平移X分量 */
  auto ty = -(sin_theta * x_diff + cos_theta * y_diff); /**< 逆平移Y分量 */

  /**
   * std::for_each - 遍历并修改每个轨迹点
   *
   * prev_trajectory->begin()/end() - 获取迭代器
   * lambda表达式 [&cos_theta, &sin_theta, &tx, &ty, &theta_diff](common::TrajectoryPoint& p){...}
   *   - 捕获列表[&cos_theta, &sin_theta, &tx, &ty, &theta_diff]
   *   - 以引用方式捕获所有外部变量
   *   - p: 当前遍历的轨迹点引用
   */
  std::for_each(prev_trajectory->begin(), prev_trajectory->end(),
                [&cos_theta, &sin_theta, &tx, &ty,
                 &theta_diff](common::TrajectoryPoint& p) {
                  /**
                   * 获取当前点的坐标和航向
                   */
                  auto x = p.path_point().x();       /**< 获取X坐标 */
                  auto y = p.path_point().y();       /**< 获取Y坐标 */
                  auto theta = p.path_point().theta(); /**< 获取航向角 */

                  /**
                   * 应用坐标变换
                   * x_new = cosθ * x - sinθ * y + tx
                   * y_new = sinθ * x + cosθ * y + ty
                   */
                  auto x_new = cos_theta * x - sin_theta * y + tx;
                  auto y_new = sin_theta * x + cos_theta * y + ty;

                  /**
                   * 归一化航向角差
                   * NormalizeAngle() - 将角度归一化到[-π, π]范围
                   */
                  auto theta_new =
                      common::math::NormalizeAngle(theta - theta_diff);

                  /**
                   * 更新轨迹点坐标
                   * mutable_path_point() - 获取可写指针
                   * set_x/set_y/set_theta() - 设置新坐标
                   */
                  p.mutable_path_point()->set_x(x_new);    /**< 更新X坐标 */
                  p.mutable_path_point()->set_y(y_new);    /**< 更新Y坐标 */
                  p.mutable_path_point()->set_theta(theta_new); /**< 更新航向角 */
                });
}

/**
 * @brief 计算需要保留的拼接轨迹
 *
 * 轨迹拼接的主函数。根据多种条件判断是否需要重规划，并计算保留的轨迹段。
 *
 * 重规划条件（满足任一即重规划）：
 * 1. 自动驾驶模式关闭
 * 2. 没有上一轮轨迹
 * 3. 位置偏差过大（横向/纵向/时间）
 * 4. 档位从N切换到D
 * 5. 收到控制模块的重规划请求
 *
 * @param vehicle_chassis 车辆底盘状态
 * @param vehicle_state 车辆状态
 * @param current_timestamp 当前时间戳（秒）
 * @param planning_cycle_time 规划周期时间（秒）
 * @param preserved_points_num 保留的历史轨迹点数量
 * @param replan_by_offset 是否按偏移量判断重规划
 * @param prev_trajectory 上一轮轨迹指针
 * @param replan_reason 输出：重规划原因
 * @param control_interactive_msg 控制交互消息
 * @return std::vector<TrajectoryPoint> 拼接后的轨迹点向量
 *
 * 语法说明：
 * - const T&: 常量引用，函数内只读
 * - size_t: 无符号整数类型，用于表示大小和索引
 * - std::string*: 输出参数指针
 * - static_cast<T>: 编译时类型转换
 */
std::vector<TrajectoryPoint> TrajectoryStitcher::ComputeStitchingTrajectory(
    const canbus::Chassis& vehicle_chassis, const VehicleState& vehicle_state,
    const double current_timestamp, const double planning_cycle_time,
    const size_t preserved_points_num, const bool replan_by_offset,
    const PublishableTrajectory* prev_trajectory, std::string* replan_reason,
    const control::ControlInteractiveMsg& control_interactive_msg) {
  /**
   * 1. 检查是否需要重规划（基础检查）
   * need_replan_by_necessary_check() - 判断必要条件
   * 如果返回true表示需要重规划
   */
  // 1.check replan not by offset
  size_t time_matched_index = 0;  /**< 时间匹配点的索引 */
  if (need_replan_by_necessary_check(vehicle_state, current_timestamp,
                                     prev_trajectory, replan_reason,
                                     &time_matched_index)) {
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  /**
   * 2. 检查档位切换重规划
   * 从空档(N)切换到前进档(D)时触发重规划
   * 原因：避免大的位置误差
   */
  // 2. check replan by GEAR switch
  if (vehicle_chassis.has_gear_location()) {  /**< 检查是否有档位信息 */
    static canbus::Chassis::GearPosition gear_pos = canbus::Chassis::GEAR_NEUTRAL; /**< 静态变量记录上次档位 */
    if (gear_pos == canbus::Chassis::GEAR_NEUTRAL &&
        vehicle_chassis.gear_location() == canbus::Chassis::GEAR_DRIVE) {
      gear_pos = vehicle_chassis.gear_location();  /**< 更新档位状态 */
      const std::string msg =
          "gear change from n to d, replan to avoid large station error";
      AERROR << msg;
      *replan_reason = msg;  /**< 设置重规划原因 */
      return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                              vehicle_state);
    }
    gear_pos = vehicle_chassis.gear_location();  /**< 更新当前档位 */
  }

  /**
   * 获取时间匹配点
   * prev_trajectory->TrajectoryPointAt(index) - 获取指定索引的轨迹点
   * static_cast<uint32_t>(time_matched_index) - size_t转uint32_t
   */
  auto time_matched_point = prev_trajectory->TrajectoryPointAt(
      static_cast<uint32_t>(time_matched_index));

  /**
   * 3. 检查控制交互重规划
   * 收到控制模块的重规划请求时触发
   */
  // 3.check replan by control_interactive_msg
  if (need_replan_by_control_interactive(current_timestamp, replan_reason,
                                         control_interactive_msg)) {
    return ComputeControlInteractiveStitchingTrajectory(
        planning_cycle_time, vehicle_state, time_matched_point,
        control_interactive_msg);
  }

  /**
   * 4. 查找位置匹配点
   * QueryNearestPointWithBuffer() - 查找最近点（带缓冲区）
   * 参数：位置坐标{ x, y }，缓冲区大小
   * 返回最近点在轨迹中的索引
   */
  size_t position_matched_index = prev_trajectory->QueryNearestPointWithBuffer(
      {vehicle_state.x(), vehicle_state.y()}, 1.0e-6); /**< 1e-6为搜索缓冲区 */

  /**
   * 计算当前位置在轨迹上的Frenet坐标投影
   * ComputePositionProjection() - 计算Frenet坐标系下的s和l值
   * 返回pair: first=s(沿轨迹距离), second=l(横向偏移)
   *
   * Frenet坐标系：
   * - s: 沿轨迹方向的距离
   * - l: 垂直于轨迹方向的横向偏移（正为左，负为右）
   */
  auto frenet_sd = ComputePositionProjection(
      vehicle_state.x(), vehicle_state.y(),
      prev_trajectory->TrajectoryPointAt(
          static_cast<uint32_t>(position_matched_index)));

  /**
   * 4. 按偏移量检查是否需要重规划（可选项）
   * replan_by_offset: 是否启用偏移量检查
   */
  // 4.check replan by offset
  if (replan_by_offset) {
    /**
     * 检查驻车制动
     * 从驻车状态切换到行驶状态时触发重规划
     */
    if (vehicle_chassis.has_parking_brake()) {  /**< 检查是否有驻车制动状态 */
      static bool parking_brake = true;  /**< 静态变量记录上次状态 */
      if (parking_brake && !vehicle_chassis.parking_brake()) { /**< 状态从开到关 */
        parking_brake = vehicle_chassis.parking_brake();
        const std::string msg =
            "parking brake off, ego move, replan to avoid large station error";
        AERROR << msg;
        *replan_reason = msg;
        return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                                vehicle_state);
      }
      parking_brake = vehicle_chassis.parking_brake();
    }

    /**
     * 计算纵向和横向偏差
     * lon_diff: 纵向偏差（沿轨迹方向）
     * lat_diff: 横向偏差（垂直轨迹方向）
     *
     * frenet_sd.first: s坐标
     * frenet_sd.second: l坐标
     */
    auto lon_diff = time_matched_point.path_point().s() - frenet_sd.first; /**< 纵向距离差 */
    auto lat_diff = frenet_sd.second; /**< 横向偏移 */

    /**
     * 计算时间偏差
     * 当前车辆时间对应点的相对时间 - 位置匹配点的相对时间
     */
    double time_diff =
        time_matched_point.relative_time() -
        prev_trajectory
            ->TrajectoryPointAt(static_cast<uint32_t>(position_matched_index))
            .relative_time();

    /**
     * ADEBUG - Apollo调试级别日志
     */
    ADEBUG << "Control lateral diff: " << lat_diff
           << ", longitudinal diff: " << lon_diff
           << ", time diff: " << time_diff;

    /**
     * 检查横向偏差是否超过阈值
     * std::fabs() - 浮点数绝对值
     * FLAGS_replan_lateral_distance_threshold - 横向重规划阈值  0.5
     */
    if (std::fabs(lat_diff) > FLAGS_replan_lateral_distance_threshold) {
      const std::string msg = absl::StrCat(
          "the distance between matched point and actual position is too "
          "large. Replan is triggered. lat_diff = ",
          lat_diff);  /**< absl::StrCat字符串拼接 */
      AERROR << msg;
      *replan_reason = msg;
      return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                              vehicle_state);
    }

    /**
     * 检查纵向偏差是否超过阈值
     * FLAGS_replan_longitudinal_distance_threshold - 纵向重规划阈值  2.5
     */
    if (std::fabs(lon_diff) > FLAGS_replan_longitudinal_distance_threshold) {
      const std::string msg = absl::StrCat(
          "the distance between matched point and actual position is too "
          "large. Replan is triggered. lon_diff = ",
          lon_diff);
      AERROR << msg;
      *replan_reason = msg;
      return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                              vehicle_state);
    }

    /**
     * 检查时间偏差是否超过阈值
     * FLAGS_replan_time_threshold - 时间重规划阈值
     */
    if (std::fabs(time_diff) > FLAGS_replan_time_threshold) {
      const std::string msg = absl::StrCat(
          "the difference between time matched point relative time and "
          "actual position corresponding relative time is too "
          "large. Replan is triggered. time_diff = ",
          time_diff);
      AERROR << msg;
      *replan_reason = msg;
      return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                              vehicle_state);
    }
  } else {
    /**
     * 偏移量检查被禁用时的日志
     */
    ADEBUG << "replan according to certain amount of "
           << "lat、lon and time offset is disabled";
  }

  /**
   * 5. 计算拼接轨迹
   * 从上一轮轨迹中保留从匹配点到前向时间点的轨迹段
   */
  // 4.stitching_trajectory

  /**
   * 计算前向时间点
   * forward_rel_time = current_timestamp - prev_trajectory.header_time() + planning_cycle_time
   * 意义：从轨迹起点到当前时间点的时间长度 + 一个规划周期
   */
  double forward_rel_time =
      current_timestamp - prev_trajectory->header_time() + planning_cycle_time;

  /**
   * 查找前向时间点对应的轨迹索引
   * QueryLowerBoundPoint() - 查找第一个时间>=forward_rel_time的点
   */
  size_t forward_time_index =
      prev_trajectory->QueryLowerBoundPoint(forward_rel_time);

  /**
   * ADEBUG - 调试日志输出匹配索引
   */
  ADEBUG << "Position matched index:\t" << position_matched_index;
  ADEBUG << "Time matched index:\t" << time_matched_index;

  /**
   * 取时间和位置匹配索引的较小值
   * 确保拼接起点的一致性
   */
  auto matched_index = std::min(time_matched_index, position_matched_index);

  /**
   * 构造拼接轨迹向量
   * 从轨迹中提取[保留起点, 前向终点]范围的轨迹点
   *
   * prev_trajectory->begin() + 起点索引
   * std::max(0, static_cast<int>(matched_index - preserved_points_num))
   *   - 确保索引不小于0
   *   - 保留preserved_points_num个历史点
   * prev_trajectory->begin() + forward_time_index + 1
   *   - 前向终点（包含该点）
   */
  std::vector<TrajectoryPoint> stitching_trajectory(
      prev_trajectory->begin() +
          std::max(0, static_cast<int>(matched_index - preserved_points_num)),
      prev_trajectory->begin() + forward_time_index + 1);
  ADEBUG << "stitching_trajectory size: " << stitching_trajectory.size();

  /**
   * 调整拼接轨迹的相对时间和s坐标
   */
  const double zero_s = stitching_trajectory.back().path_point().s(); /**< 最后一个点的s值作为零点 */

  /**
   * 遍历所有拼接点
   * for (auto& tp : stitching_trajectory) - 范围for循环，auto&避免拷贝
   */
  for (auto& tp : stitching_trajectory) {
    /**
     * 检查轨迹点是否有效
     * has_path_point() - Protobuf方法，检查字段是否存在
     */
    if (!tp.has_path_point()) {
      *replan_reason = "replan for previous trajectory missed path point";
      return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                              vehicle_state);
    }

    /**
     * 调整相对时间
     * 原相对时间 + 轨迹头时间 - 当前时间
     * 将时间基准从旧轨迹头转换到新轨迹头
     */
    tp.set_relative_time(tp.relative_time() + prev_trajectory->header_time() -
                         current_timestamp);

    /**
     * 调整s坐标
     * 新s = 旧s - zero_s
     * 使起点s为0
     */
    tp.mutable_path_point()->set_s(tp.path_point().s() - zero_s);
  }

  return stitching_trajectory;  /**< 返回拼接后的轨迹 */
}

/**
 * @brief 计算位置在轨迹点上的Frenet坐标投影
 *
 * 将笛卡尔坐标(x, y)投影到轨迹点上，得到Frenet坐标(s, l)。
 *
 * Frenet坐标系：
 * - s: 沿轨迹切线方向的距离
 * - l: 垂直于轨迹切线方向的横向偏移
 *
 * @param x 目标点X坐标
 * @param y 目标点Y坐标
 * @param p 轨迹参考点
 * @return std::pair<double, double> Frenet坐标(first=s, second=l)
 *
 * 语法说明：
 * - Vec2d: Apollo二维向量类
 * - v.InnerProd(n): 向量点积
 * - v.CrossProd(n): 向量叉积（返回标量，在2D中为z分量）
 */
std::pair<double, double> TrajectoryStitcher::ComputePositionProjection(
    const double x, const double y, const TrajectoryPoint& p) {
  /**
   * 计算从轨迹点到目标点的向量
   * Vec2d(x1, y1) - 二维向量构造函数
   */
  Vec2d v(x - p.path_point().x(), y - p.path_point().y()); /**< 向量v = 目标点 - 轨迹点 */

  /**
   * 计算轨迹点的单位切向量
   * n = [cos(θ), sin(θ)]，其中θ为轨迹点航向角
   */
  Vec2d n(std::cos(p.path_point().theta()), std::sin(p.path_point().theta()));

  std::pair<double, double> frenet_sd;  /**< Frenet坐标对 */

  /**
   * 计算s坐标
   * s = v · n + p.s()
   * v·n是向量v在切线方向n上的投影（点积）
   * 加上参考点的s值
   */
  frenet_sd.first = v.InnerProd(n) + p.path_point().s(); /**< s = 投影 + 参考s */

  /**
   * 计算l坐标（横向偏移）
   * l = v × n (2D叉积的标量结果)
   * 正值表示在轨迹左侧，负值表示在右侧
   */
  frenet_sd.second = v.CrossProd(n); /**< l = 叉积 */

  return frenet_sd;  /**< 返回Frenet坐标 */
}

/**
 * @brief 必要的重规划检查
 *
 * 检查是否需要从当前车辆状态开始重规划。
 * 基础检查包括：轨迹拼接开关、上一轮轨迹存在性、驾驶模式等。
 *
 * @param vehicle_state 车辆状态
 * @param current_timestamp 当前时间戳
 * @param prev_trajectory 上一轮轨迹指针
 * @param replan_reason 输出：重规划原因
 * @param time_matched_index 输出：时间匹配点的索引
 * @return bool 是否需要重规划
 *
 * 语法说明：
 * - bool: 布尔类型，true或false
 * - std::string*: 字符串输出参数指针
 * - size_t*: 无符号整数输出参数指针
 * - FLAGS_xxx: gflags配置标志变量
 */
bool TrajectoryStitcher::need_replan_by_necessary_check(
    const common::VehicleState& vehicle_state, const double current_timestamp,
    const PublishableTrajectory* prev_trajectory, std::string* replan_reason,
    size_t* time_matched_index) {
  /**
   * 检查轨迹拼接是否启用
   * FLAGS_enable_trajectory_stitcher - 配置标志
   */
  if (!FLAGS_enable_trajectory_stitcher) {
    *replan_reason = "stitch is disabled by gflag.";
    return true;  /**< 禁用则必须重规划 */
  }

  /**
   * 检查上一轮轨迹是否存在
   */
  if (!prev_trajectory) {
    *replan_reason = "replan for no previous trajectory.";
    return true;  /**< 无轨迹则重规划 */
  }

  /**
   * 检查驾驶模式
   * COMPLETE_AUTO_DRIVE - 完全自动驾驶模式
   * 非自动驾驶模式需要重规划
   */
  if (vehicle_state.driving_mode() != canbus::Chassis::COMPLETE_AUTO_DRIVE) {
    *replan_reason = "replan for manual mode.";
    return true;  /**< 手动模式则重规划 */
  }

  /**
   * 获取上一轮轨迹的点数量
   * prev_trajectory->NumOfPoints() - 返回轨迹点数量
   */
  size_t prev_trajectory_size = prev_trajectory->NumOfPoints();

  /**
   * 检查轨迹是否为空
   */
  if (prev_trajectory_size == 0) {
    ADEBUG << "Projected trajectory at time [" << prev_trajectory->header_time()
           << "] size is zero! Previous planning not exist or failed. Use "
              "origin car status instead.";
    *replan_reason = "replan for empty previous trajectory.";
    return true;  /**< 空轨迹则重规划 */
  }

  /**
   * 计算车辆相对时间
   * veh_rel_time = current_timestamp - prev_trajectory.header_time()
   * 表示从上一轮轨迹时刻到现在经过的时间
   */
  const double veh_rel_time =
      current_timestamp - prev_trajectory->header_time();

  /**
   * 查找时间匹配点
   * QueryLowerBoundPoint() - 查找第一个relative_time >= veh_rel_time的点
   * 返回该点在轨迹中的索引
   */
  *time_matched_index = prev_trajectory->QueryLowerBoundPoint(veh_rel_time);

  /**
   * 检查当前时间是否小于轨迹起始时间
   * 条件：匹配索引为0 且 车辆相对时间 < 轨迹第一个点的相对时间
   */
  if (*time_matched_index == 0 &&
      veh_rel_time < prev_trajectory->StartPoint().relative_time()) {
    AWARN << "current time smaller than the previous trajectory's first time";
    *replan_reason =
        "replan for current time smaller than the previous trajectory's first "
        "time.";
    return true;  /**< 时间早于轨迹起点则重规划 */
  }

  /**
   * 检查当前时间是否超出轨迹结束时间
   * 条件：匹配索引+1 >= 轨迹大小（越界检查）
   */
  if (*time_matched_index + 1 >= prev_trajectory_size) {
    AWARN << "current time beyond the previous trajectory's last time";
    *replan_reason =
        "replan for current time beyond the previous trajectory's last time";
    return true;  /**< 时间超出轨迹终点则重规划 */
  }

  /**
   * 获取时间匹配点
   * TrajectoryPointAt() - 根据索引获取轨迹点
   * static_cast<uint32_t> - 类型转换，确保索引类型正确
   */
  auto time_matched_point = prev_trajectory->TrajectoryPointAt(
      static_cast<uint32_t>(*time_matched_index));

  /**
   * 检查匹配点是否有效（是否有路径点数据）
   * has_path_point() - Protobuf方法，检查字段是否存在
   */
  if (!time_matched_point.has_path_point()) {
    *replan_reason = "replan for previous trajectory missed path point";
    return true;  /**< 轨迹点数据缺失则重规划 */
  }

  return false;  /**< 所有检查通过，不需要重规划 */
}

/**
 * @brief 检查控制交互重规划
 *
 * 检查是否收到控制模块的重规划请求。
 *
 * @param current_timestamp 当前时间戳
 * @param replan_reason 输出：重规划原因
 * @param control_interactive_msg 控制交互消息
 * @return bool 是否需要重规划
 *
 * 语法说明：
 * - control_interactive_msg.header().timestamp_sec() - 获取消息头的时间戳
 * - rel_time > 0.5: 如果消息延迟超过0.5秒，认为超时忽略
 * - has_replan_request(): 检查是否有重规划请求字段
 */
bool TrajectoryStitcher::need_replan_by_control_interactive(
    const double current_timestamp, std::string* replan_reason,
    const control::ControlInteractiveMsg& control_interactive_msg) {
  /**
   * 计算控制交互消息的延迟
   */
  const double rel_time =
      current_timestamp - control_interactive_msg.header().timestamp_sec();

  /**
   * 检查是否超时
   * rel_time > 0.5秒 认为消息已过期
   */
  if (rel_time > 0.5) {
    AINFO << "control_interactive_msg time out, skip replay by control "
             "interactive";
    return false;  /**< 超时则不重规划 */
  }

  /**
   * 检查是否有重规划请求
   * has_replan_request() - 检查字段是否存在
   * replan_request() == true - 检查请求值
   */
  if (control_interactive_msg.has_replan_request() &&
      control_interactive_msg.replan_request() == true) {
    *replan_reason = "replan for control_interactive_msg, " +
                     control_interactive_msg.replan_request_reason(); /**< 拼接原因字符串 */
    return true;  /**< 有重规划请求则重规划 */
  }

  return false;  /**< 无重规划请求 */
}

/**
 * @brief 计算控制交互拼接轨迹
 *
 * 根据控制交互消息的类型计算拼接轨迹。
 * 支持全部重规划或仅速度重规划。
 *
 * @param planning_cycle_time 规划周期时间
 * @param vehicle_state 车辆状态
 * @param time_match_point 时间匹配点
 * @param control_interactive_msg 控制交互消息
 * @return std::vector<TrajectoryPoint> 拼接轨迹
 *
 * 语法说明：
 * - control::ReplanRequestReasonCode::REPLAN_REQ_ALL_REPLAN - 全部重规划
 * - control::ReplanRequestReasonCode::REPLAN_REQ_STATION_REPLAN - 纵向重规划
 * - VehicleState::set_xxx() - 设置车辆状态的各个字段
 */
std::vector<common::TrajectoryPoint>
TrajectoryStitcher::ComputeControlInteractiveStitchingTrajectory(
    const double planning_cycle_time, const common::VehicleState& vehicle_state,
    const common::TrajectoryPoint& time_match_point,
    const control::ControlInteractiveMsg& control_interactive_msg) {
  /**
   * 检查重规划原因代码
   * replan_req_reason_code() - 获取重规划原因代码
   * REPLAN_REQ_ALL_REPLAN: 全部重规划
   * REPLAN_REQ_STATION_REPLAN: 纵向（station）重规划
   */
  if (control_interactive_msg.replan_req_reason_code() ==
          control::ReplanRequestReasonCode::REPLAN_REQ_ALL_REPLAN ||
      control_interactive_msg.replan_req_reason_code() ==
          control::ReplanRequestReasonCode::REPLAN_REQ_STATION_REPLAN) {
    AINFO << "control_interactive_msg replan, all replan";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  } else {
    /**
     * 速度重规划
     * 位置使用时间匹配点的位置，但速度使用当前车辆状态的速度
     */
    AINFO << "control_interactive_msg replan, speed replan";

    /**
     * 创建临时车辆状态拷贝
     * VehicleState: 包含x, y, z, heading, kappa等
     */
    VehicleState vehicle_state_tmp = vehicle_state; /**< 拷贝当前状态 */

    /**
     * 使用时间匹配点的位置信息覆盖临时状态
     * time_match_point.path_point().x/y/z/theta/kappa() - 获取路径点属性
     */
    vehicle_state_tmp.set_x(time_match_point.path_point().x());     /**< 设置X坐标 */
    vehicle_state_tmp.set_y(time_match_point.path_point().y());     /**< 设置Y坐标 */
    vehicle_state_tmp.set_z(time_match_point.path_point().z());     /**< 设置Z坐标 */
    vehicle_state_tmp.set_heading(time_match_point.path_point().theta()); /**< 设置航向角 */
    vehicle_state_tmp.set_kappa(time_match_point.path_point().kappa());   /**< 设置曲率 */

    /**
     * 返回基于临时状态的重新初始化轨迹
     */
    return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                            vehicle_state_tmp);
  }
}

}  // namespace planning
}  // namespace apollo
