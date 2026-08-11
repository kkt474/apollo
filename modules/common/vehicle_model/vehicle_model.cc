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
 * @file vehicle_model.cc
 * @brief 车辆模型实现文件
 *
 * 本文件实现VehicleModel类，提供车辆运动预测功能。
 * 车辆模型用于根据当前状态预测未来一段时间的车辆状态。
 *
 * 当前实现的模型：
 * - 自行车模型（Bicycle Model）：将车辆简化为两个轮子的自行车
 * - 后轴中心居中（rear-centered）：以车辆后轴中心为参考点
 * - 运动学模型（Kinematic）：不考虑力的因素，只考虑运动几何
 *
 * C++语法说明：
 * - CHECK_GT(a, b): 断言宏，a > b，否则终止程序
 * - std::cos/sin: 标准库三角函数
 * - static constexpr: 静态编译时常量
 * - while循环: 迭代计算预测轨迹
 */
#include "modules/common/vehicle_model/vehicle_model.h"

#include "cyber/common/file.h"  /**< Cyber RT文件操作工具 */
#include "modules/common/configs/config_gflags.h" /**< Apollo配置标志 */

namespace apollo {
/**
 * apollo:: - Apollo最外层命名空间
 */
namespace common {

/**
 * @brief 后轴中心自行车运动学模型
 *
 * 使用后轴中心为参考点的自行车运动学模型进行车辆状态预测。
 * 采用欧拉前向法进行离散化，假设控制命令恒定，z轴位置恒定。
 *
 * 自行车模型将四轮车辆简化为两轮自行车：
 * - 前轮负责转向
 * - 后轮驱动（但在后轴中心参考点下可视为纯滚动）
 *
 * 运动学方程（连续形式）：
 *   φ̇ = v * κ  （航向角变化率 = 速度 × 曲率）
 *   ẋ = v * cos(φ)  （X方向速度 = 速度 × cos(航向角)）
 *   ẏ = v * sin(φ)  （Y方向速度 = 速度 × sin(航向角)）
 *   v̇ = a  （速度变化率 = 加速度）
 *
 * @param vehicle_model_config 车辆模型配置（包含时间步长dt等参数）
 * @param predicted_time_horizon 预测时间范围（秒）
 * @param cur_vehicle_state 当前车辆状态
 * @param predicted_vehicle_state 输出的预测车辆状态指针
 *
 * 语法说明：
 * - const T&: 常量引用，函数内只读
 * - T*: 原始指针作为输出参数
 * - VehicleState: 包含x, y, z, heading, kappa, velocity, acceleration等
 * - κ (kappa): 曲率，1/转弯半径，无人车转向模型核心参数
 */
void VehicleModel::RearCenteredKinematicBicycleModel(
    const VehicleModelConfig& vehicle_model_config,
    const double predicted_time_horizon, const VehicleState& cur_vehicle_state,
    VehicleState* predicted_vehicle_state) {
  /**
   * CHECK_GT - 断言检查
   * GT = Greater Than
   * 确保预测时间范围大于0
   */
  // Kinematic bicycle model centered at rear axis center by Euler forward
  // discretization
  // Assume constant control command and constant z axis position
  CHECK_GT(predicted_time_horizon, 0.0);

  /**
   * 获取模型配置参数
   * dt: 时间步长（秒），每次迭代的时间增量
   * vehicle_model_config.rc_kinematic_bicycle_model().dt()
   *   - 先获取rc_kinematic_bicycle_model子配置
   *   - 再调用dt()获取时间步长
   */
  double dt = vehicle_model_config.rc_kinematic_bicycle_model().dt();

  /**
   * 保存当前状态值到局部变量
   * 避免重复访问getter方法，提高效率
   * cur_vehicle_state.x() - 获取当前X坐标
   */
  double cur_x = cur_vehicle_state.x();
  double cur_y = cur_vehicle_state.y();
  double cur_z = cur_vehicle_state.z();    /**< Z坐标保持不变 */
  double cur_phi = cur_vehicle_state.heading(); /**< 当前航向角（弧度） */
  double cur_v = cur_vehicle_state.linear_velocity(); /**< 当前线速度（m/s） */
  double cur_a = cur_vehicle_state.linear_acceleration(); /**< 当前线加速度（m/s²） */

  /**
   * 初始化下一时刻状态
   * 初始值设为当前状态，随着迭代更新
   */
  double next_x = cur_x;
  double next_y = cur_y;
  double next_phi = cur_phi;
  double next_v = cur_v;

  /**
   * 确保时间步长不超过预测范围
   * 如果dt >= predicted_time_horizon，直接设置为预测时间范围
   * 这样可以一步完成预测
   */
  if (dt >= predicted_time_horizon) {
    dt = predicted_time_horizon;
  }

  /**
   * countdown_time: 倒计时器，记录剩余未预测的时间
   * 初始值为预测时间范围，每次迭代减去dt
   */
  double countdown_time = predicted_time_horizon;
  bool finish_flag = false;  /**< 完成标志，标识最后一次迭代 */

  /**
   * static constexpr: 静态编译时常量
   * 用于浮点数比较，避免精度问题
   * 1e-8 = 0.00000001
   */
  static constexpr double kepsilon = 1e-8;

  /**
   * while循环：迭代计算预测状态
   * 条件：
   *   - countdown_time > kepsilon: 剩余时间足够进行迭代
   *   - !finish_flag: 未完成最后一次迭代
   */
  while (countdown_time > kepsilon && !finish_flag) {
    /**
     * 每次迭代减去时间步长
     * countdown_time -= dt 等价于 countdown_time = countdown_time - dt
     */
    countdown_time -= dt;

    /**
     * 检测是否为最后一次迭代
     * 如果减去dt后剩余时间小于epsilon，需要调整最后一次dt
     */
    if (countdown_time < kepsilon) {
      /**
       * dt = countdown_time + dt
       * 含义：将最后一次dt调整为刚好消耗完剩余时间
       * 相当于：dt = 原始countdown_time（调整后）
       */
      dt = countdown_time + dt;
      finish_flag = true;  /**< 设置完成标志 */
    }

    /**
     * ========== 欧拉前向离散化运动学方程 ==========
     *
     * 中点法则（Midpoint method）用于航向角更新：
     * 比纯欧拉法更稳定
     *
     * 中间航向角：
     * intermidiate_phi = cur_phi + 0.5 * dt * cur_v * cur_vehicle_state.kappa()
     *
     * 解释：
     *   - cur_v * kappa = cur_v / R = ω（角速度）
     *   - 0.5 * dt * ω = 半步长的角度变化
     *   - cur_phi + 0.5 * dt * ω = 中点航向角
     */
    double intermidiate_phi =
        cur_phi + 0.5 * dt * cur_v * cur_vehicle_state.kappa();

    /**
     * 更新航向角（完整一步）
     * 使用中点法则
     *
     * next_phi = cur_phi + dt * (cur_v + 0.5 * dt * cur_a) * kappa
     *
     * 解释：
     *   - (cur_v + 0.5 * dt * cur_a) = 中点速度（考虑加速度影响）
     *   - dt * 中点速度 * kappa = 一步航向角变化
     */
    next_phi =
        cur_phi + dt * (cur_v + 0.5 * dt * cur_a) * cur_vehicle_state.kappa();

    /**
     * 更新X坐标
     * 使用中点航向角计算位移
     *
     * next_x = cur_x + dt * (cur_v + 0.5 * dt * cur_a) * cos(intermidiate_phi)
     *
     * 解释：
     *   - cos(intermidiate_phi) = X方向单位向量
     *   - (cur_v + 0.5 * dt * cur_a) = 中点速度
     *   - dt * 中点速度 * cos() = X方向位移
     */
    next_x =
        cur_x + dt * (cur_v + 0.5 * dt * cur_a) * std::cos(intermidiate_phi);

    /**
     * 更新Y坐标
     * 使用中点航向角计算位移
     *
     * next_y = cur_y + dt * (cur_v + 0.5 * dt * cur_a) * sin(intermidiate_phi)
     *
     * 解释：
     *   - sin(intermidiate_phi) = Y方向单位向量
     *   - dt * 中点速度 * sin() = Y方向位移
     */
    next_y =
        cur_y + dt * (cur_v + 0.5 * dt * cur_a) * std::sin(intermidiate_phi);

    /**
     * 更新速度
     * 简单欧拉积分：v = v0 + a * dt
     */
    next_v = cur_v + dt * cur_a;

    /**
     * 将计算结果存回当前状态变量
     * 为下一次迭代做准备
     */
    cur_x = next_x;   /**< 更新当前X */
    cur_y = next_y;   /**< 更新当前Y */
    cur_phi = next_phi; /**< 更新当前航向角 */
    cur_v = next_v;   /**< 更新当前速度 */
  }

  /**
   * 将预测结果写入输出参数
   * set_x/set_y/set_z/set_heading: 设置车辆状态
   */
  predicted_vehicle_state->set_x(next_x); /**< 设置预测X坐标 */
  predicted_vehicle_state->set_y(next_y); /**< 设置预测Y坐标 */
  predicted_vehicle_state->set_z(cur_z);  /**< Z坐标保持不变 */

  /**
   * 设置航向角
   * 曲率kappa保持不变（假设匀速转向）
   */
  predicted_vehicle_state->set_heading(next_phi);
  predicted_vehicle_state->set_kappa(cur_vehicle_state.kappa());

  /**
   * 设置速度和加速度
   */
  predicted_vehicle_state->set_linear_velocity(next_v); /**< 设置预测速度 */
  predicted_vehicle_state->set_linear_acceleration(
      cur_vehicle_state.linear_acceleration()); /**< 加速度保持不变 */
}

/**
 * @brief 预测未来车辆状态
 *
 * 车辆状态预测的主入口函数。
 * 根据配置选择合适的车辆模型进行预测。
 *
 * @param predicted_time_horizon 预测时间范围（秒）
 * @param cur_vehicle_state 当前车辆状态
 * @return VehicleState 预测的车辆状态（按值返回）
 *
 * 语法说明：
 * - VehicleState: 返回值类型，按值返回会触发拷贝构造
 * - FLAGS_vehicle_model_config_filename: gflags配置标志，模型配置文件路径
 * - cyber::common::GetProtoFromFile(): 从文件加载protobuf配置
 */
VehicleState VehicleModel::Predict(const double predicted_time_horizon,
                                   const VehicleState& cur_vehicle_state) {
  /**
   * 创建车辆模型配置对象
   * VehicleModelConfig: 包含模型类型和具体模型参数
   */
  VehicleModelConfig vehicle_model_config;

  /**
   * 从配置文件加载模型配置
   * cyber::common::GetProtoFromFile(filename, &config)
   *   - 第一个参数：配置文件路径（从FLAGS获取）
   *   - 第二个参数：输出配置对象指针
   *   - 返回bool表示是否成功加载
   *
   * ACHECK - Apollo断言宏
   * 如果加载失败，输出错误消息并终止程序
   */
  ACHECK(cyber::common::GetProtoFromFile(FLAGS_vehicle_model_config_filename,
                                         &vehicle_model_config))
      << "Failed to load vehicle model config file "
      << FLAGS_vehicle_model_config_filename;

  /**
   * ACHECK - 检查模型类型是否支持
   * 当前不支持的模型：
   *   - COM_CENTERED_DYNAMIC_BICYCLE_MODEL: 质心中心动态自行车模型（未实现）
   *   - MLP_MODEL: 多层感知机模型（未实现）
   *
   * != 表示不等于
   * VehicleModelConfig::COM_CENTERED_DYNAMIC_BICYCLE_MODEL: 枚举值，动态模型类型
   */
  // Some models not supported for now
  ACHECK(vehicle_model_config.model_type() !=
         VehicleModelConfig::COM_CENTERED_DYNAMIC_BICYCLE_MODEL);
  ACHECK(vehicle_model_config.model_type() != VehicleModelConfig::MLP_MODEL);

  /**
   * 创建预测状态对象
   * 用于存储预测结果
   */
  VehicleState predicted_vehicle_state;

  /**
   * 根据模型类型调用对应的预测函数
   * 目前只支持REAR_CENTERED_KINEMATIC_BICYCLE_MODEL
   *
   * VehicleModelConfig::REAR_CENTERED_KINEMATIC_BICYCLE_MODEL: 枚举值
   * == 比较枚举值
   */
  if (vehicle_model_config.model_type() ==
      VehicleModelConfig::REAR_CENTERED_KINEMATIC_BICYCLE_MODEL) {
    /**
     * 调用后轴中心自行车运动学模型
     * 参数：
     *   - vehicle_model_config: 模型配置
     *   - predicted_time_horizon: 预测时间范围
     *   - cur_vehicle_state: 当前状态
     *   - &predicted_vehicle_state: 输出参数指针
     */
    RearCenteredKinematicBicycleModel(vehicle_model_config,
                                      predicted_time_horizon, cur_vehicle_state,
                                      &predicted_vehicle_state);
  }

  /**
   * 返回预测结果
   * 按值返回，会调用VehicleState的拷贝构造函数
   */
  return predicted_vehicle_state;
}

}  // namespace common
}  // namespace apollo
