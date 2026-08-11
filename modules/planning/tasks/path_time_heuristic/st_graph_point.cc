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
 * @file: st_graph_point.cc
 * @brief ST图点实现文件
 *
 * 本文件实现了ST图上的网格点(StGraphPoint)数据结构
 * 用于在动态规划速度优化算法中存储每个网格点的状态和代价信息
 *
 * 功能说明：
 * 1. 存储ST图上每个网格点的索引和坐标
 * 2. 存储动态规划中的各类代价（总代价、障碍物代价、空间势能代价）
 * 3. 存储前驱点指针，用于回溯最优路径
 * 4. 存储最优速度
 *
 * 数据成员说明：
 * - index_t_: 时间维度索引
 * - index_s_: 空间维度索引
 * - point_: ST坐标点 (s, t)
 * - pre_point_: 指向前驱点的指针
 * - reference_cost_: 参考代价
 * - obstacle_cost_: 障碍物代价
 * - spatial_potential_cost_: 空间势能代价
 * - total_cost_: 总代价
 * - optimal_speed_: 最优速度
 *
 * 相关C++语法说明：
 * - std::uint32_t: 无符号32位整数类型
 * - const成员函数: 承诺不修改成员变量
 * - 指针存储: 使用原始指针存储前驱点引用
 **/

/**
 * @brief ST图点头文件
 *
 * 包含StGraphPoint类的完整定义
 */
#include "modules/planning/tasks/path_time_heuristic/st_graph_point.h"

/**
 * @brief 规划模块GFlags头文件
 * FLAGS_*: 从配置文件获取的参数
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
 * @brief 获取空间维度索引
 *
 * @return std::uint32_t 空间索引
 *
 * C++语法说明：
 * - std::uint32_t: 无符号32位整数类型，用于表示非负索引
 * - const成员函数: 在函数后加const，承诺不修改成员变量
 *   允许在const对象上调用const成员函数
 */
std::uint32_t StGraphPoint::index_s() const { return index_s_; }

/**
 * @brief 获取时间维度索引
 *
 * @return std::uint32_t 时间索引
 */
std::uint32_t StGraphPoint::index_t() const { return index_t_; }

/**
 * @brief 获取ST坐标点
 *
 * @return const STPoint& ST坐标点的常量引用
 *
 * C++语法说明：
 * - const STPoint&: 返回常量引用，避免拷贝
 *   调用者不能修改返回的点，但可以读取其数据
 */
const STPoint& StGraphPoint::point() const { return point_; }

/**
 * @brief 获取前驱点指针
 *
 * @return const StGraphPoint* 指向前驱点的常量指针
 *
 * 功能说明：
 * 在动态规划中，每个点需要记录其前驱点
 * 用于最后回溯最优路径
 *
 * C++语法说明：
 * - const StGraphPoint*: 指向常量的指针
 *   指针指向的对象不能被修改
 * - 返回nullptr表示没有前驱（起点）
 */
const StGraphPoint* StGraphPoint::pre_point() const { return pre_point_; }

/**
 * @brief 获取参考代价
 *
 * @return double 参考代价
 *
 * 功能说明：
 * 参考代价通常是沿参考线行驶的基准代价
 * 用于归一化总代价计算
 */
double StGraphPoint::reference_cost() const { return reference_cost_; }

/**
 * @brief 获取障碍物代价
 *
 * @return double 障碍物代价
 *
 * 功能说明：
 * 障碍物代价表示在该点位置与障碍物的接近程度
 * 距离障碍物越近，代价越高
 * 如果与障碍物重叠，代价为无穷大（不可达）
 */
double StGraphPoint::obstacle_cost() const { return obstacle_cost_; }

/**
 * @brief 获取空间势能代价
 *
 * @return double 空间势能代价
 *
 * 功能说明：
 * 空间势能代价与道路曲率、车道边界等因素相关
 * 用于引导轨迹沿平滑的路径行驶
 */
double StGraphPoint::spatial_potential_cost() const {
  return spatial_potential_cost_;
}

/**
 * @brief 获取总代价
 *
 * @return double 总代价
 *
 * 功能说明：
 * 总代价 = obstacle_cost + spatial_potential_cost + 前驱点总代价 + 边代价
 * 动态规划的目标是找到从起点到终点总代价最小的路径
 */
double StGraphPoint::total_cost() const { return total_cost_; }

/**
 * @brief 初始化ST图点
 *
 * @param index_t 时间维度索引
 * @param index_s 空间维度索引
 * @param st_point ST坐标点
 *
 * 功能说明：
 * 初始化网格点的基本属性
 * 在创建代价表时调用
 *
 * C++语法说明：
 * - void: 无返回值
 * - 参数中的const表示参数是输入参数，不会被修改
 * - 赋值给成员变量：index_t_ = index_t
 */
void StGraphPoint::Init(const std::uint32_t index_t,
                        const std::uint32_t index_s, const STPoint& st_point) {
  index_t_ = index_t;   /**< 设置时间索引 */
  index_s_ = index_s;   /**< 设置空间索引 */
  point_ = st_point;    /**< 设置ST坐标点 */
}

/**
 * @brief 设置参考代价
 *
 * @param reference_cost 参考代价值
 *
 * 功能说明：
 * 设置该点的参考代价
 */
void StGraphPoint::SetReferenceCost(const double reference_cost) {
  reference_cost_ = reference_cost;
}

/**
 * @brief 设置障碍物代价
 *
 * @param obs_cost 障碍物代价值
 *
 * 功能说明：
 * 设置该点的障碍物代价
 * 在DP代价计算中调用
 */
void StGraphPoint::SetObstacleCost(const double obs_cost) {
  obstacle_cost_ = obs_cost;
}

/**
 * @brief 设置空间势能代价
 *
 * @param spatial_potential_cost 空间势能代价值
 *
 * 功能说明：
 * 设置该点的空间势能代价
 * 与道路曲率、车道边界等相关
 */
void StGraphPoint::SetSpatialPotentialCost(
    const double spatial_potential_cost) {
  spatial_potential_cost_ = spatial_potential_cost;
}

/**
 * @brief 设置总代价
 *
 * @param total_cost 总代价值
 *
 * 功能说明：
 * 设置该点的总代价
 * 包括：障碍物代价 + 空间势能代价 + 边代价
 */
void StGraphPoint::SetTotalCost(const double total_cost) {
  total_cost_ = total_cost;
}

/**
 * @brief 设置前驱点
 *
 * @param pre_point 前驱点引用
 *
 * 功能说明：
 * 记录到达该点的最优前驱点
 * 用于最后回溯最优路径
 *
 * C++语法说明：
 * - const StGraphPoint& pre_point: 前驱点的常量引用
 * - &pre_point: 取地址运算符，获取前驱点的指针
 * - pre_point_: 存储前驱点的原始指针
 *
 * 注意：这里存储的是指针而不是引用
 * 指针可以在不确定对象生命周期的情况下使用
 * 但需要确保前驱对象的生命周期长于当前对象
 */
void StGraphPoint::SetPrePoint(const StGraphPoint& pre_point) {
  pre_point_ = &pre_point;
}

/**
 * @brief 获取最优速度
 *
 * @return double 最优速度值
 *
 * 功能说明：
 * 获取通过该点的最优速度
 * 这个速度是在最小化总代价时计算得到的
 */
double StGraphPoint::GetOptimalSpeed() const { return optimal_speed_; }

/**
 * @brief 设置最优速度
 *
 * @param optimal_speed 最优速度值
 *
 * 功能说明：
 * 设置该点的最优速度
 * 在DP代价计算中更新
 */
void StGraphPoint::SetOptimalSpeed(const double optimal_speed) {
  optimal_speed_ = optimal_speed;
}

}  // namespace planning
}  // namespace apollo
