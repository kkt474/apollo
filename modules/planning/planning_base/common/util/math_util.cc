/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
 * @file math_util.cc
 * @brief 数学工具函数实现文件
 *
 * 功能说明：
 * 实现坐标转换和角度计算的工具函数
 * 主要用于世界坐标系与障碍物局部坐标系之间的转换
 */

#include "modules/planning/planning_base/common/util/math_util.h"

#include <utility>

#include "modules/common/math/math_utils.h"

namespace apollo {
namespace planning {
namespace util {

/**
 * @brief 世界坐标到障碍物相对坐标的转换实现
 *
 * @param input_world_coord 输入的世界坐标
 * @param obj_world_coord 障碍物的世界坐标
 * @param obj_world_angle 障碍物的世界角度(弧度)
 * @return std::pair<double, double> 相对坐标
 *
 * 算法详解：
 * 1. 计算输入点相对于障碍物中心的向量差
 *    - x_diff = input_world_coord.first - obj_world_coord.first
 *    - y_diff = input_world_coord.second - obj_world_coord.second
 *
 * 2. 将直角坐标转换为极坐标
 *    - rho = sqrt(x_diff^2 + y_diff^2)，表示距离
 *    - theta = atan2(y_diff, x_diff)，表示角度
 *
 * 3. 角度校正
 *    - theta_relative = theta - obj_world_angle
 *    - 将角度转换到障碍物的局部坐标系中
 *
 * 4. 转换回笛卡尔坐标（相对于障碍物）
 *    - local_x = cos(theta_relative) * rho
 *    - local_y = sin(theta_relative) * rho
 *
 * C++语法说明：
 * - std::pair<double, double>::first/second：
 *   std::pair的两个元素的访问方式
 *   - first：第一个元素，通常是x坐标
 *   - second：第二个元素，通常是y坐标
 *
 * - std::sqrt：标准库函数，计算平方根
 *   #include <cmath>
 *
 * - std::atan2：标准库函数，计算反正切
 *   返回值范围是[-PI, PI]
 *   atan2(y, x) 等价于 atan(y/x)，但能处理所有象限
 *
 * - std::cos/std::sin：标准库三角函数
 *   接受弧度值作为参数
 *
 * - std::make_pair：创建std::pair的辅助函数
 *   返回 std::pair<double, double>(cos(theta)*rho, sin(theta)*rho)
 */
std::pair<double, double> WorldCoordToObjCoord(
    std::pair<double, double> input_world_coord,
    std::pair<double, double> obj_world_coord, double obj_world_angle) {
  // 第一步：计算输入点与障碍物中心的坐标差值
  // 这是将坐标原点移动到障碍物位置的过程
  double x_diff = input_world_coord.first - obj_world_coord.first;
  double y_diff = input_world_coord.second - obj_world_coord.second;

  // 第二步：计算极坐标表示
  // rho是两点之间的欧几里得距离
  // 使用std::sqrt而不是直接计算sqrt(x_diff*x_diff)以获得更好的数值稳定性
  double rho = std::sqrt(x_diff * x_diff + y_diff * y_diff);

  // 计算从障碍物位置到输入点的角度
  // std::atan2能正确处理所有象限的情况，避免除零错误
  // 结果范围是[-PI, PI]
  double theta = std::atan2(y_diff, x_diff) - obj_world_angle;

  // 第三步：构建相对坐标
  // 使用三角函数将极坐标转换回笛卡尔坐标
  // local_x 相当于在障碍物前进方向上的分量
  // local_y 相当于在障碍物侧向方向上的分量
  // 注意：这里的坐标轴方向是由obj_world_angle决定的
  return std::make_pair(std::cos(theta) * rho, std::sin(theta) * rho);
}

/**
 * @brief 世界角度到障碍物相对角度的转换实现
 *
 * @param input_world_angle 输入的世界角度(弧度)
 * @param obj_world_angle 障碍物的世界角度(弧度)
 * @return double 相对角度，已归一化到[-PI, PI]
 *
 * 算法详解：
 * 1. 计算两个角度的差值：delta = input_world_angle - obj_world_angle
 * 2. 使用NormalizeAngle函数将差值归一化到[-PI, PI]范围
 *
 * C++语法说明：
 * - common::math::NormalizeAngle：
 *   Apollo数学库中的角度归一化函数
 *   将任意角度转换到[-PI, PI]范围内
 *   实现原理：angle = angle - floor(angle / (2*PI) + 0.5) * 2*PI
 *
 * - 函数调用中的命名空间限定：
 *   common::math:: 表示在common命名空间下的math子命名空间
 *   这是由于Apollo代码使用嵌套命名空间组织
 *
 * 角度归一化的意义：
 * - 角度是周期性变量，PI和-P1表示同一个方向
 * - 归一化可以简化角度比较和计算
 * - 例如：270度和-90度实际上是等价的
 */
double WorldAngleToObjAngle(double input_world_angle, double obj_world_angle) {
  // 直接返回两个角度的差值，并自动归一化
  // 这比手动计算后再归一化更简洁，且不容易出错
  return common::math::NormalizeAngle(input_world_angle - obj_world_angle);
}

}  // namespace util
}  // namespace planning
}  // namespace apollo
