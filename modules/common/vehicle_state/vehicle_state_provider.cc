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
 * @file vehicle_state_provider.cc
 * @brief 车辆状态提供者实现文件
 *
 * 本文件实现了VehicleStateProvider类，负责：
 * 1. 整合定位模块（Localization）和底盘模块（Chassis）的数据
 * 2. 提供统一的车辆状态访问接口
 * 3. 计算车辆运动学参数（如曲率kappa）
 * 4. 估算车辆未来位置
 *
 * 核心概念：
 * - VehicleState: 包含车辆位置(x,y,z)、航向角、速度、加速度、档位等完整状态
 * - localization::LocalizationEstimate: 定位模块输出的位姿估计
 * - canbus::Chassis: 底盘模块输出的车辆底盘状态
 *
 * 语法说明：
 * - const Reference &: 常量引用，避免拷贝
 * - std::abs/fabs: 绝对值函数（C标准库）
 * - Eigen::Vector3d/Quaternion: Eigen库向量和四元数类型
 * - mutable_pose(): Protobuf可变访问器
 * - constexpr: 编译时常量表达式
 */

#include "modules/common/vehicle_state/vehicle_state_provider.h"

/**
 * #include <cmath> - C标准库数学函数
 * 提供std::abs, std::fabs, std::sin, std::cos等数学函数
 */
#include <cmath>

/**
 * Eigen/Core - Eigen线性代数库头文件
 * Eigen是一个C++模板库，用于矩阵向量运算
 * Vector3d: 三维double向量
 * Quaternion<double>: 四元数模板类
 * toRotationMatrix(): 四元数转旋转矩阵
 */
#include "Eigen/Core"

/**
 * absl/strings/str_cat.h - Abseil字符串拼接
 * absl::StrCat用于高效拼接错误消息字符串
 */
#include "absl/strings/str_cat.h"

/**
 * cyber/common/log.h - Cyber RT日志系统
 * 提供AERROR, ADEBUG等日志宏
 */
#include "cyber/common/log.h"

/**
 * modules/common/configs/config_gflags.h
 * 全局配置标志，包含FLAGS_reverse_heading_vehicle_state等
 */
#include "modules/common/configs/config_gflags.h"

/**
 * modules/common/math/euler_angles_zxy.h
 * 欧拉角库，用于从四元数计算欧拉角
 * EulerAnglesZXYd: Z-X-Y顺序的欧拉角结构
 */
#include "modules/common/math/euler_angles_zxy.h"

/**
 * modules/common/math/quaternion.h
 * 四元数库，提供QuaternionToHeading等函数
 */
#include "modules/common/math/quaternion.h"

namespace apollo {

/**
 * apollo:: - Apollo最外层命名空间
 */
namespace common {

/**
 * @brief 更新车辆状态
 *
 * 这是核心函数，接收定位和底盘数据，更新内部vehicle_state_
 *
 * @param localization 定位模块输出的位姿估计
 * @param chassis 底盘模块输出的车辆底盘状态
 * @return Status 更新是否成功
 *
 * 更新流程：
 * 1. 保存原始定位数据
 * 2. 构建除线性速度外的所有状态（位置、姿态、角速度、加速度）
 * 3. 设置时间戳（优先使用定位的测量时间）
 * 4. 设置档位
 * 5. 设置线性速度（考虑倒档反转）
 * 6. 设置方向盘百分比
 * 7. 计算曲率kappa
 * 8. 设置驾驶模式
 *
 * 语法说明：
 * - const LocalizationEstimate &: 常量引用，避免拷贝
 * - original_localization_: 成员变量，保存原始定位数据
 * - mutable_pose(): 返回pose字段的可写指针
 * - has_xxx(): Protobuf的has判断方法，检查字段是否存在
 * - set_xxx(): Protobuf的setter方法，设置字段值
 * - std::abs: C标准库的绝对值函数（重载了int/double/float等版本）
 */
Status VehicleStateProvider::Update(
    const localization::LocalizationEstimate &localization,
    const canbus::Chassis &chassis) {

  /**
   * original_localization_ = localization
   * 保存原始定位数据，用于后续可能需要访问原始数据
   * 这是一个拷贝赋值操作
   */
  original_localization_ = localization;

  /**
   * ConstructExceptLinearVelocity() - 构建除线性速度外的所有状态
   * 位置(x,y,z)、航向角(heading)、角速度、线加速度、欧拉角等
   * 这些信息主要来自定位模块
   */
  if (!ConstructExceptLinearVelocity(localization)) {
    /**
     * absl::StrCat - Abseil字符串拼接
     * 高效地拼接错误消息
     */
    std::string msg = absl::StrCat(
        "Fail to update because ConstructExceptLinearVelocity error.",
        "localization:\n", localization.DebugString());
    return Status(ErrorCode::LOCALIZATION_ERROR, msg);
  }

  /**
   * 设置时间戳
   * 优先使用定位的测量时间(has_measurement_time())
   * 其次使用定位头部的timestamp_sec
   * 最后使用底盘的时间戳
   */
  if (localization.has_measurement_time()) {
    /**
     * set_timestamp() - 设置车辆状态的时间戳
     * 来自定位的测量时间，更准确
     */
    vehicle_state_.set_timestamp(localization.measurement_time());
  } else if (localization.header().has_timestamp_sec()) {
    vehicle_state_.set_timestamp(localization.header().timestamp_sec());
  } else if (chassis.has_header() && chassis.header().has_timestamp_sec()) {
    AERROR << "Unable to use location timestamp for vehicle state. Use chassis "
              "time instead.";
    vehicle_state_.set_timestamp(chassis.header().timestamp_sec());
  }

  /**
   * 设置档位
   * 优先使用底盘的gear_location
   * 如果底盘没有提供，默认为GEAR_NONE（无档位）
   *
   * canbus::Chassis::GEAR_NONE/GEAR_DRIVE/GEAR_REVERSE等是档位枚举值
   */
  if (chassis.has_gear_location()) {
    vehicle_state_.set_gear(chassis.gear_location());
  } else {
    vehicle_state_.set_gear(canbus::Chassis::GEAR_NONE);
  }

  /**
   * 设置线性速度
   * chassis.speed_mps()返回速度（米/秒）
   *
   * FLAGS_reverse_heading_vehicle_state - 是否在倒档时反转航向
   * 如果设置了此标志且当前为倒档，速度取反
   */
  if (chassis.has_speed_mps()) {
    vehicle_state_.set_linear_velocity(chassis.speed_mps());
    if (!FLAGS_reverse_heading_vehicle_state &&
        vehicle_state_.gear() == canbus::Chassis::GEAR_REVERSE) {
      /**
       * set_linear_velocity() - 设置线性速度
       * 如果是倒档且不需要反转航向，则速度取反
       */
      vehicle_state_.set_linear_velocity(-vehicle_state_.linear_velocity());
    }
  }

  /**
   * 设置方向盘转角百分比
   * steering_percentage()范围通常是-1.0到1.0
   * 表示方向盘转动角度占最大角度的比例
   */
  if (chassis.has_steering_percentage()) {
    vehicle_state_.set_steering_percentage(chassis.steering_percentage());
  }

  /**
   * 计算曲率kappa
   * kappa = angular_velocity / linear_velocity
   * 表示车辆轨迹的弯曲程度
   *
   * static constexpr double kEpsilon = 0.1
   * - static: 文件内全局唯一
   * - constexpr: 编译时常量，编译器会在编译时求值
   * - kEpsilon: 惯例用k前缀表示常量
   *
   * std::abs()处理速度接近零的情况，避免除零
   */
  static constexpr double kEpsilon = 0.1;
  if (std::abs(vehicle_state_.linear_velocity()) < kEpsilon) {
    /**
     * 速度很小时，曲率设为0（车辆几乎直线行驶）
     */
    vehicle_state_.set_kappa(0.0);
  } else {
    /**
     * kappa = angular_velocity / linear_velocity
     * 例如：角速度1.0 rad/s，速度10 m/s，则kappa=0.1
     * kappa > 0表示左转，kappa < 0表示右转
     */
    vehicle_state_.set_kappa(vehicle_state_.angular_velocity() /
                             vehicle_state_.linear_velocity());
  }

  /**
   * 设置驾驶模式
   * Chassis::DrivingMode枚举值：
   * - COMPLETE_AUTO_DRIVE: 完全自动驾驶
   * - AUTO_DRIVE_ONLY: 仅自动模式
   * - MANUAL_DRIVE: 人工驾驶
   */
  vehicle_state_.set_driving_mode(chassis.driving_mode());

  return Status::OK();
}

/**
 * @brief 构建除线性速度外的所有车辆状态
 *
 * 从定位数据中提取并设置：
 * - 位置 (x, y, z)
 * - 航向角 (heading)
 * - 角速度 (angular_velocity)
 * - 线加速度 (linear_acceleration)
 * - 欧拉角 (roll, pitch, yaw)
 *
 * @param localization 定位估计数据
 * @return bool 构建是否成功
 *
 * 语法说明：
 * - const LocalizationEstimate &: 常量引用
 * - has_pose(): 检查pose字段是否存在
 * - mutable_pose()->CopyFrom(): 拷贝赋值pose
 * - set_x()/set_y()/set_z(): 设置位置坐标
 * - math::QuaternionToHeading(): 四元数转航向角
 * - math::EulerAnglesZXYd: Z-X-Y欧拉角结构
 */
bool VehicleStateProvider::ConstructExceptLinearVelocity(
    const localization::LocalizationEstimate &localization) {

  /**
   * 检查定位数据是否有有效的pose
   * has_pose()是Protobuf的has方法
   */
  if (!localization.has_pose()) {
    AERROR << "Invalid localization input.";
    return false;
  }

  /**
   * 在导航模式下，跳过定位更新
   * FLAGS_use_navigation_mode是配置标志
   * 导航模式可能使用相对定位，不依赖绝对定位
   */
  // skip localization update when it is in use_navigation_mode.
  if (FLAGS_use_navigation_mode) {
    ADEBUG << "Skip localization update when it is in use_navigation_mode.";
    return true;
  }

  /**
   * mutable_pose()->CopyFrom() - 拷贝整个pose到vehicle_state_
   * mutable_pose()返回Pose消息的可写指针
   * CopyFrom()执行深拷贝
   */
  vehicle_state_.mutable_pose()->CopyFrom(localization.pose());

  /**
   * 设置位置坐标
   * localization.pose().position()获取位置消息
   * x(), y(), z()获取各轴坐标
   */
  if (localization.pose().has_position()) {
    vehicle_state_.set_x(localization.pose().position().x());
    vehicle_state_.set_y(localization.pose().position().y());
    vehicle_state_.set_z(localization.pose().position().z());
  }

  /**
   * 获取航向角（heading）
   * 航向角是车辆纵轴与正北方向的夹角，单位弧度
   * 范围通常是[-PI, PI]或[0, 2*PI]
   *
   * const auto &orientation - 常量引用，避免拷贝
   */
  const auto &orientation = localization.pose().orientation();

  /**
   * 航向角来源优先级：
   * 1. 直接使用pose.heading()（如果存在）
   * 2. 从四元数计算（QuaternionToHeading）
   *
   * 四元数(qw, qx, qy, qz)描述了车辆的旋转姿态
   * QuaternionToHeading将其转换为航向角
   */
  if (localization.pose().has_heading()) {
    vehicle_state_.set_heading(localization.pose().heading());
  } else {
    /**
     * QuaternionToHeading - 四元数转航向角函数
     * 参数：qw(实部), qx, qy, qz(虚部)
     * 这是四元数的标准表示方式
     */
    vehicle_state_.set_heading(
        math::QuaternionToHeading(orientation.qw(), orientation.qx(),
                                  orientation.qy(), orientation.qz()));
  }

  /**
   * FLAGS_enable_map_reference_unify - 地图参考系统一标志
   * 当启用时，使用带VRF（Vehicle Reference Frame）标记的速度
   * VRF表示速度已经转换到车身坐标系
   */
  if (FLAGS_enable_map_reference_unify) {
    /**
     * angular_velocity_vrf - 车身参考系下的角速度
     * 只使用z轴分量，因为车辆主要在二维平面运动
     */
    if (!localization.pose().has_angular_velocity_vrf()) {
      AERROR << "localization.pose().has_angular_velocity_vrf() must be true "
                "when FLAGS_enable_map_reference_unify is true.";
      return false;
    }
    vehicle_state_.set_angular_velocity(
        localization.pose().angular_velocity_vrf().z());

    /**
     * linear_acceleration_vrf - 车身参考系下的线加速度
     * y轴分量是纵向加速度（前进/后退方向）
     */
    if (!localization.pose().has_linear_acceleration_vrf()) {
      AERROR << "localization.pose().has_linear_acceleration_vrf() must be "
                "true when FLAGS_enable_map_reference_unify is true.";
      return false;
    }
    vehicle_state_.set_linear_acceleration(
        localization.pose().linear_acceleration_vrf().y());
  } else {
    /**
     * 未启用参考系统一，使用原始的角速度和线加速度
     * 同样是z轴角速度和y轴线加速度
     */
    if (!localization.pose().has_angular_velocity()) {
      AERROR << "localization.pose() has no angular velocity.";
      return false;
    }
    vehicle_state_.set_angular_velocity(
        localization.pose().angular_velocity().z());

    if (!localization.pose().has_linear_acceleration()) {
      AERROR << "localization.pose() has no linear acceleration.";
      return false;
    }
    vehicle_state_.set_linear_acceleration(
        localization.pose().linear_acceleration().y());
  }

  /**
   * 设置欧拉角（roll, pitch, yaw）
   * 欧拉角描述车辆的旋转姿态：
   * - roll: 侧倾角（绕x轴）
   * - pitch: 俯仰角（绕y轴）
   * - yaw: 偏航角（绕z轴）
   *
   * 优先使用pose中已有的euler_angles
   * 否则从四元数计算
   */
  if (localization.pose().has_euler_angles()) {
    /**
     * euler_angles()返回包含roll, pitch, yaw的结构
     * 注意：Apollo中欧拉角的xyz对应关系可能有特定约定
     */
    vehicle_state_.set_roll(localization.pose().euler_angles().y());
    vehicle_state_.set_pitch(localization.pose().euler_angles().x());
    vehicle_state_.set_yaw(localization.pose().euler_angles().z());
  } else {
    /**
     * 从四元数计算欧拉角
     * EulerAnglesZXYd构造函数接收四元数的四个分量
     * 然后调用roll(), pitch(), yaw()获取各角度
     */
    math::EulerAnglesZXYd euler_angle(orientation.qw(), orientation.qx(),
                                      orientation.qy(), orientation.qz());
    vehicle_state_.set_roll(euler_angle.roll());
    vehicle_state_.set_pitch(euler_angle.pitch());
    vehicle_state_.set_yaw(euler_angle.yaw());
  }

  return true;
}

/**
 * @brief 获取车辆X坐标
 *
 * const成员函数，不会修改对象状态
 * 直接返回vehicle_state_中存储的x值
 *
 * @return double 车辆X坐标（地图坐标系）
 */
double VehicleStateProvider::x() const { return vehicle_state_.x(); }

/**
 * @brief 获取车辆Y坐标
 * @return double 车辆Y坐标（地图坐标系）
 */
double VehicleStateProvider::y() const { return vehicle_state_.y(); }

/**
 * @brief 获取车辆Z坐标（高度）
 * @return double 车辆Z坐标（地图坐标系）
 */
double VehicleStateProvider::z() const { return vehicle_state_.z(); }

/**
 * @brief 获取侧倾角（roll）
 *
 * 车辆绕纵轴（行驶方向）的旋转角度
 * 左倾为正，右倾为负
 *
 * @return double 侧倾角（弧度）
 */
double VehicleStateProvider::roll() const { return vehicle_state_.roll(); }

/**
 * @brief 获取俯仰角（pitch）
 *
 * 车辆绕横轴的旋转角度
 * 抬头为正，低头为负
 *
 * @return double 俯仰角（弧度）
 */
double VehicleStateProvider::pitch() const { return vehicle_state_.pitch(); }

/**
 * @brief 获取偏航角（yaw）
 *
 * 车辆绕垂直轴的旋转角度
 * 等同于heading航向角
 *
 * @return double 偏航角（弧度）
 */
double VehicleStateProvider::yaw() const { return vehicle_state_.yaw(); }

/**
 * @brief 获取航向角
 *
 * 车辆纵轴与正北方向的夹角
 * 范围通常是[-PI, PI]或[0, 2*PI]
 *
 * @return double 航向角（弧度）
 */
double VehicleStateProvider::heading() const {
  return vehicle_state_.heading();
}

/**
 * @brief 获取曲率kappa
 *
 * kappa = angular_velocity / linear_velocity
 * 描述车辆轨迹的弯曲程度
 *
 * @return double 曲率（1/米）
 */
double VehicleStateProvider::kappa() const { return vehicle_state_.kappa(); }

/**
 * @brief 获取线性速度
 *
 * 车辆当前的行驶速度（标量）
 * 正值表示前进，负值表示后退
 *
 * @return double 线性速度（米/秒）
 */
double VehicleStateProvider::linear_velocity() const {
  return vehicle_state_.linear_velocity();
}

/**
 * @brief 获取角速度
 *
 * 车辆绕垂直轴的旋转角速度
 *
 * @return double 角速度（弧度/秒）
 */
double VehicleStateProvider::angular_velocity() const {
  return vehicle_state_.angular_velocity();
}

/**
 * @brief 获取线加速度
 *
 * 车辆纵向加速度
 *
 * @return double 线加速度（米/秒²）
 */
double VehicleStateProvider::linear_acceleration() const {
  return vehicle_state_.linear_acceleration();
}

/**
 * @brief 获取档位
 *
 * canbus::Chassis::GEAR_DRIVE/GEAR_REVERSE/GEAR_NEUTRAL等
 *
 * @return double 档位枚举值（转换为double）
 */
double VehicleStateProvider::gear() const { return vehicle_state_.gear(); }

/**
 * @brief 获取方向盘转角百分比
 *
 * 范围-1.0到1.0，表示占最大转角的比例
 *
 * @return double 方向盘转角百分比
 */
double VehicleStateProvider::steering_percentage() const {
  return vehicle_state_.steering_percentage();
}

/**
 * @brief 获取时间戳
 *
 * 车辆状态的采集时间
 *
 * @return double 时间戳（秒）
 */
double VehicleStateProvider::timestamp() const {
  return vehicle_state_.timestamp();
}

/**
 * @brief 获取车辆姿态（Pose）
 *
 * @return const localization::Pose& 车辆姿态引用
 */
const localization::Pose &VehicleStateProvider::pose() const {
  return vehicle_state_.pose();
}

/**
 * @brief 获取原始定位姿态
 *
 * 返回Update时保存的原始定位数据中的姿态
 *
 * @return const localization::Pose& 原始定位姿态引用
 */
const localization::Pose &VehicleStateProvider::original_pose() const {
  return original_localization_.pose();
}

/**
 * @brief 设置线性速度
 *
 * 允许外部直接设置线性速度值
 *
 * @param linear_velocity 线性速度值（米/秒）
 */
void VehicleStateProvider::set_linear_velocity(const double linear_velocity) {
  vehicle_state_.set_linear_velocity(linear_velocity);
}

/**
 * @brief 获取完整车辆状态
 *
 * 返回内部的VehicleState消息引用
 *
 * @return const VehicleState& 车辆状态引用
 */
const VehicleState &VehicleStateProvider::vehicle_state() const {
  return vehicle_state_;
}

/**
 * @brief 估算未来位置
 *
 * 根据当前车辆状态（速度、角速度、姿态）估算t秒后的位置
 * 考虑圆弧运动模型
 *
 * @param t 时间间隔（秒）
 * @return math::Vec2d 估算的二维位置(x, y)
 *
 * 算法说明：
 * 当角速度≈0时，车辆近似直线运动：
 *   dx = 0, dy = v * t
 *
 * 当角速度≠0时，车辆做圆弧运动：
 *   r = v / omega (转弯半径)
 *   dx = -r * (1 - cos(omega * t))
 *   dy = r * sin(omega * t)
 *
 * 如果有姿态信息（旋转矩阵），将运动向量转换到世界坐标系
 *
 * 语法说明：
 * - Eigen::Vector3d: 三维double向量类
 * - Eigen::Quaternion<double>: double型四元数类
 * - quaternion.toRotationMatrix(): 四元数转3x3旋转矩阵
 * - std::fabs: C标准库浮点数绝对值函数
 * - std::sin/std::cos: C标准库三角函数
 * - math::Vec2d: Apollo二维向量类
 */
math::Vec2d VehicleStateProvider::EstimateFuturePosition(const double t) const {
  /**
   * vec_distance - 相对于当前位置的位移向量
   * [0]对应x，[1]对应y，[2]对应z
   * 初始化为(0, 0, 0)
   */
  Eigen::Vector3d vec_distance(0.0, 0.0, 0.0);

  /**
   * v - 当前线性速度
   */
  double v = vehicle_state_.linear_velocity();

  /**
   * 判断角速度是否足够小（近似直线运动）
   * std::fabs是C标准库的浮点数绝对值函数
   * 阈值0.0001 rad/s
   */
  // Predict distance travel vector
  if (std::fabs(vehicle_state_.angular_velocity()) < 0.0001) {
    /**
     * 角速度很小，直线运动模型
     * x方向无位移，y方向移动v*t
     */
    vec_distance[0] = 0.0;
    vec_distance[1] = v * t;
  } else {
    /**
     * 圆弧运动模型
     * omega = angular_velocity
     * r = v / omega (转弯半径)
     *
     * x方向位移 = -r * (1 - cos(omega * t))
     * 这考虑了起点不在圆心正上方的偏移
     */
    vec_distance[0] = -v / vehicle_state_.angular_velocity() *
                      (1.0 - std::cos(vehicle_state_.angular_velocity() * t));
    /**
     * y方向位移 = r * sin(omega * t)
     */
    vec_distance[1] = std::sin(vehicle_state_.angular_velocity() * t) * v /
                      vehicle_state_.angular_velocity();
  }

  /**
   * 如果有旋转信息（姿态四元数），考虑旋转影响
   * 将局部坐标系下的位移转换到世界坐标系
   */
  // If we have rotation information, take it into consideration.
  if (vehicle_state_.pose().has_orientation()) {
    /**
     * 获取四元数
     * 格式：(qw, qx, qy, qz)
     */
    const auto &orientation = vehicle_state_.pose().orientation();
    Eigen::Quaternion<double> quaternion(orientation.qw(), orientation.qx(),
                                         orientation.qy(), orientation.qz());

    /**
     * pos_vec - 当前位置向量
     */
    Eigen::Vector3d pos_vec(vehicle_state_.x(), vehicle_state_.y(),
                            vehicle_state_.z());

    /**
     * future_pos_3d = R * vec_distance + pos_vec
     * R是四元数对应的旋转矩阵
     * 将局部运动向量转换到世界坐标系
     */
    const Eigen::Vector3d future_pos_3d =
        quaternion.toRotationMatrix() * vec_distance + pos_vec;

    /**
     * 返回二维位置
     * 取结果向量的x和y分量
     */
    return math::Vec2d(future_pos_3d[0], future_pos_3d[1]);
  }

  /**
   * 如果没有有效的旋转信息
   * 直接将位移加到当前位置（不考虑旋转）
   * 这假设车辆始终朝当前heading方向运动
   */
  // If no valid rotation information provided from localization,
  // return the estimated future position without rotation.
  return math::Vec2d(vec_distance[0] + vehicle_state_.x(),
                     vec_distance[1] + vehicle_state_.y());
}

/**
 * @brief 计算质心位置
 *
 * 计算车辆质心（Center of Mass）在地图坐标系中的位置
 *
 * @param rear_to_com_distance 后轮到质心的距离
 * @return math::Vec2d 质心二维位置
 *
 * 算法说明：
 * 1. 确定偏移向量v（从后轴中心到质心）
 * 2. 根据档位和FLAGS决定是否应用偏移
 * 3. 如果有姿态信息，应用旋转变换
 * 4. 返回质心位置
 *
 * 语法说明：
 * - Eigen::Vector3d v: 三维向量
 * - v << 0.0, rear_to_com_distance, 0.0: Eigen向量初始化语法
 * - quaternion.toRotationMatrix() * v: 旋转向量
 */
math::Vec2d VehicleStateProvider::ComputeCOMPosition(
    const double rear_to_com_distance) const {
  /**
   * v - 从后轴中心到质心的偏移向量
   * 在车身坐标系下，假设质心在后轴中心的前方（沿y轴）
   */
  // set length as distance between rear wheel and center of mass.
  Eigen::Vector3d v;

  /**
   * 根据档位和配置标志决定是否应用偏移
   *
   * FLAGS_state_transform_to_com_reverse:
   *   倒档时是否将状态转换到质心坐标系
   * FLAGS_state_transform_to_com_drive:
   *   前进档时是否将状态转换到质心坐标系
   *
   * 只有在启用对应标志且处于对应档位时，才应用偏移
   */
  if ((FLAGS_state_transform_to_com_reverse &&
       vehicle_state_.gear() == canbus::Chassis::GEAR_REVERSE) ||
      (FLAGS_state_transform_to_com_drive &&
       vehicle_state_.gear() == canbus::Chassis::GEAR_DRIVE)) {
    /**
     * Eigen向量的<<运算符初始化
     * v = (0, rear_to_com_distance, 0)
     */
    v << 0.0, rear_to_com_distance, 0.0;
  } else {
    v << 0.0, 0.0, 0.0;
  }

  /**
   * pos_vec - 当前位置向量（后轴中心）
   */
  Eigen::Vector3d pos_vec(vehicle_state_.x(), vehicle_state_.y(),
                          vehicle_state_.z());

  /**
   * 初始化质心位置（不考虑旋转）
   * com_pos_3d = v + pos_vec
   */
  // Initialize the COM position without rotation
  Eigen::Vector3d com_pos_3d = v + pos_vec;

  /**
   * 如果有姿态信息，应用旋转变换
   * 将车身坐标系下的偏移转换到世界坐标系
   */
  // If we have rotation information, take it into consideration.
  if (vehicle_state_.pose().has_orientation()) {
    const auto &orientation = vehicle_state_.pose().orientation();
    Eigen::Quaternion<double> quaternion(orientation.qw(), orientation.qx(),
                                         orientation.qy(), orientation.qz());
    /**
     * 重新计算：R * v + pos_vec
     * R是旋转矩阵，将车身坐标系转换到世界坐标系
     */
    // Update the COM position with rotation
    com_pos_3d = quaternion.toRotationMatrix() * v + pos_vec;
  }

  /**
   * 返回质心的二维位置
   */
  return math::Vec2d(com_pos_3d[0], com_pos_3d[1]);
}

}  // namespace common
}  // namespace apollo
