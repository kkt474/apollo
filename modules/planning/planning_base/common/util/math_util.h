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
#pragma once

#include <utility>

namespace apollo {
/**
 * @brief Apollo规划模块命名空间
 *
 * 功能说明：
 * - apollo：最外层命名空间，代表Apollo自动驾驶平台
 * - planning：规划模块命名空间，包含所有规划相关的类和函数
 * - util：工具函数命名空间，包含各种辅助功能
 *
 * C++语法说明：
 * - namespace：命名空间关键字，用于组织代码、避免命名冲突
 * - 嵌套命名空间可以在C++17中简写为 namespace apollo::planning::util
 * - 旧版C++需要使用嵌套的namespace声明
 */
namespace planning {

/**
 * @brief 工具函数命名空间
 *
 * 功能说明：
 * 封装规划模块中使用的各种数学和坐标转换工具函数
 * 这些函数是纯函数，不持有状态，可在多处复用
 */
namespace util {

/**
 * @brief 将世界坐标转换为障碍物相对坐标
 *
 * @param input_world_coord 输入的世界坐标(std::pair<double, double>)
 *        第一个元素为x坐标，第二个元素为y坐标
 * @param obj_world_coord 障碍物的世界坐标(std::pair<double, double>)
 *        作为坐标转换的参考原点
 * @param obj_world_angle 障碍物的世界角度(double,弧度)
 *        障碍物的朝向角，用于旋转坐标系统
 * @return std::pair<double, double> 转换后的相对坐标
 *         相对于障碍物的局部坐标系坐标
 *
 * 功能说明：
 * 将一个点从世界坐标系转换到以障碍物为中心的局部坐标系
 * 这个转换在障碍物规避和碰撞检测中非常有用
 *
 * 算法流程：
 * 1. 计算输入点与障碍物中心的差值(delta_x, delta_y)
 * 2. 计算该差值的距离rho和平面角theta
 * 3. 将theta减去障碍物角度，得到相对于障碍物朝向的角度
 * 4. 使用三角函数计算新的相对坐标
 *
 * C++语法说明：
 * - std::pair<double, double>：
 *   标准库模板类，表示两个元素的元组
 *   - first：第一个元素（通常是x坐标）
 *   - second：第二个元素（通常是y坐标）
 * - double：双精度浮点数类型
 *
 * 示例：
 * @code
 *   std::pair<double, double> vehicle_pos = {100.0, 50.0};
 *   std::pair<double, double> obstacle_pos = {105.0, 52.0};
 *   double obstacle_angle = M_PI / 4;  // 45度
 *
 *   auto local_pos = WorldCoordToObjCoord(vehicle_pos, obstacle_pos, obstacle_angle);
 *   // local_pos.first 是在障碍物前方的距离
 *   // local_pos.second 是在障碍物侧方的距离
 * @endcode
 */
std::pair<double, double> WorldCoordToObjCoord(
    std::pair<double, double> input_world_coord,
    std::pair<double, double> obj_world_coord, double obj_world_angle);

/**
 * @brief 将世界角度转换为障碍物相对角度
 *
 * @param input_world_angle 输入的世界角度(double,弧度)
 * @param obj_world_angle 障碍物的世界角度(double,弧度)
 * @return double 转换后的相对角度(弧度)
 *
 * 功能说明：
 * 计算输入角度相对于障碍物朝向的角度差
 * 结果会被归一化到[-PI, PI]范围内
 *
 * 算法流程：
 * 1. 计算两个角度的差值
 * 2. 使用NormalizeAngle函数将结果归一化到[-PI, PI]
 *
 * C++语法说明：
 * - double：双精度浮点数类型，用于表示角度（弧度制）
 * - std::atan2返回的角度范围是[-PI, PI]
 *
 * 示例：
 * @code
 *   double vehicle_heading = M_PI / 3;  // 车辆航向60度
 *   double obstacle_heading = M_PI / 6;  // 障碍物朝向30度
 *
 *   double relative_angle = WorldAngleToObjAngle(vehicle_heading, obstacle_heading);
 *   // relative_angle = PI/6 (30度)，表示车辆相对于障碍物的偏转角度
 * @endcode
 */
double WorldAngleToObjAngle(double input_world_angle, double obj_world_angle);

}  // namespace util
}  // namespace planning
}  // namespace apollo
