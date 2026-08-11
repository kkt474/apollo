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
 * @file
 * @brief DP-ST代价计算器实现文件
 *
 * 本文件实现了动态规划速度优化中的代价计算器(DpStCost)
 * 用于计算ST图上每个网格点的各类代价
 *
 * 功能说明：
 * 1. 障碍物代价计算（obstacle_cost）
 * 2. 空间势能代价计算（spatial_potential_cost）
 * 3. 速度代价计算（speed_cost）
 * 4. 加速度代价计算（accel_cost）
 * 5. Jerk代价计算（jerk_cost）
 *
 * 代价函数说明：
 * - obstacle_cost: 基于与障碍物的安全距离，惩罚过近的轨迹
 * - spatial_potential_cost: 惩罚远离终点的状态
 * - speed_cost: 惩罚超速、低速、偏离巡航速度
 * - accel_cost: 惩罚过大的加速度/减速度
 * - jerk_cost: 惩罚过大的加速度变化率（舒适性）
 *
 * 相关C++语法说明：
 * - std::numeric_limits<double>::infinity(): 无穷大常量
 * - std::array<T, N>: 固定大小数组，用于代价缓存
 * - std::vector<std::pair<T1, T2>>: 向量中存储pair，用于边界缓存
 **/
/**
 * @brief DP-ST代价计算器头文件
 *
 * 包含DpStCost类的完整定义
 */
#include "modules/planning/tasks/path_time_heuristic/dp_st_cost.h"

/**
 * @brief 标准库头文件
 * <algorithm>: 提供std::min, std::max, std::sort, std::ceil等算法
 * <limits>: 提供std::numeric_limits获取类型极值
 */
#include <algorithm>
#include <limits>

/**
 * @brief 车辆配置助手头文件
 * VehicleConfigHelper: 车辆配置助手
 */
#include "modules/common/configs/vehicle_config_helper.h"

/**
 * @brief ST图数据结构头文件
 * st_point.h: STPoint数据结构
 */
#include "modules/planning/planning_base/common/speed/st_point.h"

/**
 * @brief 规划模块GFlags头文件
 * FLAGS_*: 从配置文件获取的参数
 */
#include "modules/planning/planning_base/gflags/planning_gflags.h"

/**
 * @brief ST间隙估算器头文件
 * StGapEstimator: 安全距离估算工具
 */
#include "modules/planning/planning_interface_base/task_base/utils/st_gap_estimator.h"

/**
 * @brief Apollo命名空间开始
 */
namespace apollo {

/**
 * @brief 规划模块命名空间
 */
namespace planning {

/**
 * @brief 匿名命名空间定义无穷大常量
 *
 * C++语法说明：
 * namespace {}: 匿名命名空间，内部定义的符号仅当前文件可见
 * static constexpr: 静态常量，在整个程序运行期间保持不变
 */
namespace {
/**
 * @brief 无穷大常量
 *
 * 用于表示不可达状态或极大代价
 * 当代价等于kInf时，表示该路径不可行
 */
constexpr double kInf = std::numeric_limits<double>::infinity();
}

/**
 * @brief DpStCost构造函数
 *
 * @param config DP速度优化器配置
 * @param total_t 总时间
 * @param total_s 总路径长度
 * @param obstacles 障碍物列表
 * @param st_drivable_boundary ST可行驶边界
 * @param init_point 规划起点
 *
 * 功能说明：
 * 初始化代价计算器，建立障碍物ID到索引的映射
 * 初始化代价缓存数据结构
 *
 * 初始化列表说明：
 * : config_(config), obstacles_(obstacles), ...
 * 初始化列表在函数体执行之前初始化成员变量
 * 效率比在函数体内赋值更高
 */
DpStCost::DpStCost(const DpStSpeedOptimizerConfig& config, const double total_t,
                   const double total_s,
                   const std::vector<const Obstacle*>& obstacles,
                   const STDrivableBoundary& st_drivable_boundary,
                   const common::TrajectoryPoint& init_point)
    : config_(config),                        /**< 初始化配置 */
      obstacles_(obstacles),                  /**< 初始化障碍物列表 */
      st_drivable_boundary_(st_drivable_boundary),  /**< 初始化ST可行驶边界 */
      init_point_(init_point),               /**< 初始化规划起点 */
      unit_t_(config.unit_t()),              /**< 初始化时间单元 */
      total_s_(total_s) {                    /**< 初始化总路径长度 */
  /**
   * @brief 建立障碍物ID到索引的映射
   *
   * boundary_map_: std::unordered_map<std::string, int>
   * 用于快速查找障碍物在obstacles_向量中的索引
   */
  int index = 0;
  for (const auto& obstacle : obstacles) {
    boundary_map_[obstacle->path_st_boundary().id()] = index++;
  }

  /**
   * @brief 添加KEEP_CLEAR区域
   *
   * KEEP_CLEAR区域如斑马线等，是禁止停车的区域
   * 在这些区域内低速行驶会有额外惩罚
   */
  AddToKeepClearRange(obstacles);

  /**
   * @brief 计算时间维度大小
   *
   * std::ceil(total_t / unit_t_): 向上取整，确保覆盖整个时间范围
   * +1: 加上起点时刻
   * static_cast<uint32_t>: 显式类型转换
   */
  const auto dimension_t =
      static_cast<uint32_t>(std::ceil(total_t / static_cast<double>(unit_t_))) +
      1;

  /**
   * @brief 初始化障碍物边界代价缓存
   *
   * boundary_cost_: 二维向量
   * 外层：每个障碍物
   * 内层：每个时间点的(s_upper, s_lower)边界对
   * 缓存避免重复计算
   */
  boundary_cost_.resize(obstacles_.size());
  for (auto& vec : boundary_cost_) {
    vec.resize(dimension_t, std::make_pair(-1.0, -1.0));
  }

  /**
   * @brief 初始化加速度代价缓存
   * accel_cost_: 固定大小数组，存储不同加速度值的代价
   * fill(-1.0): 初始值-1表示未计算
   */
  accel_cost_.fill(-1.0);

  /**
   * @brief 初始化Jerk代价缓存
   * jerk_cost_: 固定大小数组，存储不同Jerk值的代价
   */
  jerk_cost_.fill(-1.0);
}

/**
 * @brief 添加KEEP_CLEAR区域到列表
 *
 * @param obstacles 障碍物列表
 *
 * 功能说明：
 * 遍历所有障碍物，找出KEEP_CLEAR类型的障碍物
 * 将其S方向的覆盖范围添加到keep_clear_range_列表
 *
 * KEEP_CLEAR边界类型：
 * - 人行横道（斑马线）
 * - 路口
 * - 其他禁止停车的区域
 */
void DpStCost::AddToKeepClearRange(
    const std::vector<const Obstacle*>& obstacles) {
  /**
   * @brief 遍历所有障碍物
   */
  for (const auto& obstacle : obstacles) {
    /**
     * @brief 跳过空边界
     */
    if (obstacle->path_st_boundary().IsEmpty()) {
      continue;
    }

    /**
     * @brief 只处理KEEP_CLEAR类型
     */
    if (obstacle->path_st_boundary().boundary_type() !=
        STBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }

    /**
     * @brief 获取S方向范围
     * min_s(): 障碍物的最小S坐标
     * max_s(): 障碍物的最大S坐标
     */
    double start_s = obstacle->path_st_boundary().min_s();
    double end_s = obstacle->path_st_boundary().max_s();

    /**
     * @brief 添加到范围列表
     * emplace_back: 直接构造，避免拷贝
     */
    keep_clear_range_.emplace_back(start_s, end_s);
  }

  /**
   * @brief 排序并合并重叠的区域
   */
  SortAndMergeRange(&keep_clear_range_);
}

/**
 * @brief 排序并合并重叠的范围
 *
 * @param keep_clear_range 输入/输出：范围列表
 *
 * 功能说明：
 * 1. 按起点坐标排序
 * 2. 合并重叠的区域
 *
 * 算法说明：
 * 使用双指针算法
 * - i指向当前合并区域的起点
 * - j向后扫描
 * - 如果j的起点 > i的终点，说明不重叠，i=j
 * - 否则说明重叠，合并（更新i的终点为较大值）
 */
void DpStCost::SortAndMergeRange(
    std::vector<std::pair<double, double>>* keep_clear_range) {
  /**
   * @brief 空列表检查
   */
  if (!keep_clear_range || keep_clear_range->empty()) {
    return;
  }

  /**
   * @brief 按起点排序
   * std::sort: 默认按first升序，first相同时按second升序
   */
  std::sort(keep_clear_range->begin(), keep_clear_range->end());

  /**
   * @brief 双指针扫描合并
   */
  size_t i = 0;
  size_t j = i + 1;
  while (j < keep_clear_range->size()) {
    /**
     * @brief 如果i的终点 < j的起点，不重叠
     * 移动到下一个区域
     */
    if (keep_clear_range->at(i).second < keep_clear_range->at(j).first) {
      ++i;
      ++j;
    } else {
      /**
       * @brief 重叠，合并
       * 更新i的终点为两者中较大的值
       */
      keep_clear_range->at(i).second = std::max(keep_clear_range->at(i).second,
                                                keep_clear_range->at(j).second);
      ++j;
    }
  }

  /**
   * @brief 调整向量大小
   * 保留合并后的区域数量
   */
  keep_clear_range->resize(i + 1);
}

/**
 * @brief 检查点是否在KEEP_CLEAR区域内
 *
 * @param s S坐标
 * @return bool 是否在KEEP_CLEAR区域
 *
 * 功能说明：
 * 检查给定的S坐标是否在任何KEEP_CLEAR区域内
 * 用于判断是否需要应用低速惩罚
 */
bool DpStCost::InKeepClearRange(double s) const {
  /**
   * @brief 遍历所有KEEP_CLEAR区域
   */
  for (const auto& p : keep_clear_range_) {
    /**
     * @brief 检查s是否在[p.first, p.second]范围内
     */
    if (p.first <= s && p.second >= s) {
      return true;  /**< 在区域内 */
    }
  }
  return false;  /**< 不在任何区域内 */
}

/**
 * @brief 获取障碍物代价
 *
 * @param st_graph_point ST图点
 * @return double 障碍物代价
 *
 * 功能说明：
 * 计算在该ST点位置与所有障碍物的接近程度代价
 *
 * 算法说明：
 * 1. 如果启用ST可行驶边界，检查点是否在边界内
 * 2. 遍历所有障碍物：
 *    a. 跳过虚拟障碍物
 *    b. 跳过STOP决策的障碍物
 *    c. 检查时间范围
 *    d. 如果点在障碍物边界内，返回无穷大
 *    e. 计算跟随/超车距离代价
 *
 * 代价计算：
 * - s < s_lower（跟随）：代价与(s_lower - s - safe_distance)成正比
 * - s > s_upper（超车）：代价与(s - s_upper - safe_distance)成正比
 */
double DpStCost::GetObstacleCost(const StGraphPoint& st_graph_point) {
  /**
   * @brief 获取当前点的S和T坐标
   */
  const double s = st_graph_point.point().s();
  const double t = st_graph_point.point().t();

  /**
   * @brief 初始化代价
   */
  double cost = 0.0;

  /**
   * @brief 检查ST可行驶边界
   *
   * FLAGS_use_st_drivable_boundary: 配置开关
   * 如果启用，检查点是否在可行驶边界内
   */
  if (FLAGS_use_st_drivable_boundary) {
    /**
     * @brief TODO(Jiancheng): move to configs
     * 边界分辨率，目前硬编码为0.1
     */
    static constexpr double boundary_resolution = 0.1;

    /**
     * @brief 计算时间索引
     */
    int index = static_cast<int>(t / boundary_resolution);

    /**
     * @brief 获取该时刻的S边界
     */
    const double lower_bound =
        st_drivable_boundary_.st_boundary(index).s_lower();
    const double upper_bound =
        st_drivable_boundary_.st_boundary(index).s_upper();

    /**
     * @brief 如果超出边界，返回无穷大代价
     */
    if (s > upper_bound || s < lower_bound) {
      return kInf;  /**< 不可达 */
    }
  }

  /**
   * @brief 遍历所有障碍物
   */
  for (const auto* obstacle : obstacles_) {
    /**
     * @brief 跳过虚拟障碍物
     *
     * 虚拟障碍物如创建的停车墙等
     * 不应用接近惩罚
     */
    if (obstacle->IsVirtual()) {
      continue;
    }

    /**
     * @brief 跳过STOP决策的障碍物
     *
     * 停车障碍物在映射时已经有安全边距
     * 不需要额外的排斥力
     */
    if (obstacle->LongitudinalDecision().has_stop()) {
      continue;
    }

    /**
     * @brief 获取障碍物的ST边界
     */
    auto boundary = obstacle->path_st_boundary();

    /**
     * @brief 跳过超出水平线的障碍物
     *
     * FLAGS_speed_lon_decision_horizon:
     *   纵向决策水平线（前方多远的障碍物需要考虑）
     */
    if (boundary.min_s() > FLAGS_speed_lon_decision_horizon) {
      continue;
    }

    /**
     * @brief 检查时间范围
     * 如果当前时刻不在障碍物的时间范围内，跳过
     */
    if (t < boundary.min_t() || t > boundary.max_t()) {
      continue;
    }

    /**
     * @brief 如果点在障碍物边界内，返回无穷大代价
     * 这意味着发生碰撞
     */
    if (boundary.IsPointInBoundary(st_graph_point.point())) {
      return kInf;  /**< 碰撞，不可达 */
    }

    /**
     * @brief 初始化边界值
     */
    double s_upper = 0.0;
    double s_lower = 0.0;

    /**
     * @brief 获取障碍物边界
     *
     * boundary_map_: 障碍物ID到索引的映射
     * boundary_cost_: 缓存的边界值
     *
     * 如果缓存命中，使用缓存值
     * 否则计算并缓存
     */
    int boundary_index = boundary_map_[boundary.id()];
    if (boundary_cost_[boundary_index][st_graph_point.index_t()].first < 0.0) {
      /**
       * @brief 缓存未命中，计算边界
       * GetBoundarySRange(t, &s_upper, &s_lower):
       *   根据时间t获取S方向的边界
       */
      boundary.GetBoundarySRange(t, &s_upper, &s_lower);
      boundary_cost_[boundary_index][st_graph_point.index_t()] =
          std::make_pair(s_upper, s_lower);
    } else {
      /**
       * @brief 缓存命中，直接使用
       */
      s_upper = boundary_cost_[boundary_index][st_graph_point.index_t()].first;
      s_lower = boundary_cost_[boundary_index][st_graph_point.index_t()].second;
    }

    /**
     * @brief 情况1：s < s_lower，跟随场景
     *
     * 自车在障碍物后方
     * 计算与安全距离的偏差代价
     */
    if (s < s_lower) {
      /**
       * @brief 安全跟车距离
       */
      const double follow_distance_s = config_.safe_distance();

      /**
       * @brief 如果距离大于安全距离，无惩罚
       */
      if (s + follow_distance_s < s_lower) {
        continue;  /**< 安全距离足够 */
      } else {
        /**
         * @brief 计算距离偏差
         * s_diff = safe_distance - (s_lower - s)
         * 当s越接近s_lower，s_diff越小
         */
        auto s_diff = follow_distance_s - s_lower + s;

        /**
         * @brief 代价 = 权重 × 默认代价 × s_diff²
         *
         * 使用平方使近距离的惩罚急剧增加
         * 引导轨迹保持安全距离
         */
        cost += config_.obstacle_weight() * config_.default_obstacle_cost() *
                s_diff * s_diff;
      }
    }
    /**
     * @brief 情况2：s > s_upper，超车场景
     *
     * 自车在障碍物前方
     */
    else if (s > s_upper) {
      /**
       * @brief 安全超车距离
       * StGapEstimator::EstimateSafeOvertakingGap():
       *   估算安全超车间隙
       */
      const double overtake_distance_s =
          StGapEstimator::EstimateSafeOvertakingGap();

      /**
       * @brief 如果距离大于安全超车距离，无惩罚
       */
      if (s > s_upper + overtake_distance_s) {
        continue;  /**< 安全距离足够 */
      } else {
        /**
         * @brief 计算距离偏差
         */
        auto s_diff = overtake_distance_s + s_upper - s;

        /**
         * @brief 代价计算
         */
        cost += config_.obstacle_weight() * config_.default_obstacle_cost() *
                s_diff * s_diff;
      }
    }
  }

  /**
   * @brief 乘以时间单元，归一化代价
   */
  return cost * unit_t_;
}

/**
 * @brief 获取空间势能代价
 *
 * @param point ST点
 * @return double 空间势能代价
 *
 * 功能说明：
 * 惩罚远离终点的状态
 * 引导轨迹向终点方向移动
 *
 * 代价公式：
 * cost = (total_s_ - point.s()) × spatial_potential_penalty
 *
 * 解释：
 * - 越接近终点(total_s_)，代价越低
 * - 越远离起点(0)，代价越高
 */
double DpStCost::GetSpatialPotentialCost(const StGraphPoint& point) {
  return (total_s_ - point.point().s()) * config_.spatial_potential_penalty();
}

/**
 * @brief 获取参考代价
 *
 * @param point 当前点
 * @param reference_point 参考点
 * @return double 参考代价
 *
 * 功能说明：
 * 惩罚偏离参考线的程度
 *
 * 代价公式：
 * cost = reference_weight × (s - ref_s)² × unit_t_
 */
double DpStCost::GetReferenceCost(const STPoint& point,
                                  const STPoint& reference_point) const {
  return config_.reference_weight() * (point.s() - reference_point.s()) *
         (point.s() - reference_point.s()) * unit_t_;
}

/**
 * @brief 获取速度代价
 *
 * @param first 前一个ST点
 * @param second 当前ST点
 * @param speed_limit 速度限制
 * @param cruise_speed 巡航速度
 * @return double 速度代价
 *
 * 功能说明：
 * 综合考虑以下因素：
 * 1. 速度为负（倒车）：无穷大惩罚
 * 2. 在KEEP_CLEAR区域低速行驶：额外惩罚
 * 3. 超速：惩罚超过程度
 * 4. 低速：惩罚低速程度
 * 5. 偏离巡航速度：惩罚偏离程度
 */
double DpStCost::GetSpeedCost(const STPoint& first, const STPoint& second,
                              const double speed_limit,
                              const double cruise_speed) const {
  /**
   * @brief 初始化代价
   */
  double cost = 0.0;

  /**
   * @brief 计算当前段速度
   * v = ds / dt = (s2 - s1) / unit_t_
   */
  const double speed = (second.s() - first.s()) / unit_t_;

  /**
   * @brief 速度为负，返回无穷大代价
   * 负速度表示倒车，不允许
   */
  if (speed < 0) {
    return kInf;  /**< 不可达 */
  }

  /**
   * @brief 获取最大停止速度阈值
   *
   * 车辆在完全停止时的速度会有一个微小值
   * 低于这个阈值认为是停止状态
   */
  const double max_adc_stop_speed = common::VehicleConfigHelper::Instance()
                                        ->GetConfig()
                                        .vehicle_param()
                                        .max_abs_speed_when_stopped();

  /**
   * @brief 在KEEP_CLEAR区域低速行驶惩罚
   *
   * InKeepClearRange(second.s()):
   *   检查当前点是否在KEEP_CLEAR区域内
   * second.s(): 当前点的S坐标
   */
  if (speed < max_adc_stop_speed && InKeepClearRange(second.s())) {
    /**
     * @brief 低速且在禁停区，添加惩罚
     * keep_clear_low_speed_penalty × default_speed_cost × unit_t_
     */
    cost += config_.keep_clear_low_speed_penalty() * unit_t_ *
            config_.default_speed_cost();
  }

  /**
   * @brief 速度偏差计算
   * det_speed = (v - limit) / limit
   * 正值表示超速，负值表示低速
   */
  double det_speed = (speed - speed_limit) / speed_limit;

  /**
   * @brief 超速惩罚
   * 惩罚与超速程度的平方成正比
   */
  if (det_speed > 0) {
    cost += config_.exceed_speed_penalty() * config_.default_speed_cost() *
            (det_speed * det_speed) * unit_t_;
  }
  /**
   * @brief 低速惩罚
   * 惩罚与低速程度的绝对值成正比
   */
  else if (det_speed < 0) {
    cost += config_.low_speed_penalty() * config_.default_speed_cost() *
            -det_speed * unit_t_;
  }

  /**
   * @brief 巡航速度惩罚
   *
   * enable_dp_reference_speed():
   *   配置开关，是否启用巡航速度惩罚
   * diff_speed = v - cruise_speed
   * 代价与速度偏差的绝对值成正比
   */
  if (config_.enable_dp_reference_speed()) {
    double diff_speed = speed - cruise_speed;
    cost += config_.reference_speed_penalty() * config_.default_speed_cost() *
            fabs(diff_speed) * unit_t_;
  }

  return cost;
}

/**
 * @brief 获取加速度代价（缓存版本）
 *
 * @param accel 加速度值
 * @return double 加速度代价
 *
 * 功能说明：
 * 计算加速度的代价
 * 使用数组缓存不同加速度值的代价，加速计算
 *
 * 代价公式：
 * - 正加速度：accel_penalty × accel²
 * - 负加速度：decel_penalty × accel² + sigmoid项
 *
 * Sigmoid项说明：
 * - 当accel接近max_decel时，sigmoid项趋于0
 * - 当accel接近max_accel时，sigmoid项趋于1
 * 用于平滑地限制加速度在允许范围内
 */
double DpStCost::GetAccelCost(const double accel) {
  /**
   * @brief 初始化代价
   */
  double cost = 0.0;

  /**
   * @brief 离散化参数
   * kEpsilon: 离散化精度，0.1
   * kShift: 偏移量，100（用于处理负值索引）
   */
  static constexpr double kEpsilon = 0.1;
  static constexpr size_t kShift = 100;

  /**
   * @brief 计算加速度的离散化索引
   * accel / kEpsilon: 将加速度映射到数组索引
   * + 0.5: 四舍五入
   * + kShift: 偏移，处理负加速度
   */
  const size_t accel_key = static_cast<size_t>(accel / kEpsilon + 0.5 + kShift);

  /**
   * @brief 边界检查
   */
  DCHECK_LT(accel_key, accel_cost_.size());
  if (accel_key >= accel_cost_.size()) {
    return kInf;  /**< 索引超出范围，不可达 */
  }

  /**
   * @brief 检查缓存
   * 初始值为-1.0，表示未计算
   */
  if (accel_cost_.at(accel_key) < 0.0) {
    /**
     * @brief 未缓存，计算代价
     */
    const double accel_sq = accel * accel;

    /**
     * @brief 获取配置参数
     */
    double max_acc = config_.max_acceleration();
    double max_dec = config_.max_deceleration();
    double accel_penalty = config_.accel_penalty();
    double decel_penalty = config_.decel_penalty();

    /**
     * @brief 根据正负应用不同惩罚系数
     */
    if (accel > 0.0) {
      cost = accel_penalty * accel_sq;
    } else {
      cost = decel_penalty * accel_sq;
    }

    /**
     * @brief 添加Sigmoid平滑项
     *
     * std::exp(x): e^x
     * 第一项：当accel接近max_dec时，sigmoid趋于0
     * 第二项：当accel接近max_acc时，sigmoid趋于1
     *
     * 作用：平滑地限制加速度在[max_dec, max_acc]范围内
     */
    cost += accel_sq * decel_penalty * decel_penalty /
                (1 + std::exp(1.0 * (accel - max_dec))) +
            accel_sq * accel_penalty * accel_penalty /
                (1 + std::exp(-1.0 * (accel - max_acc)));

    /**
     * @brief 缓存计算结果
     */
    accel_cost_.at(accel_key) = cost;
  } else {
    /**
     * @brief 缓存命中，直接使用
     */
    cost = accel_cost_.at(accel_key);
  }

  /**
   * @brief 乘以时间单元归一化
   */
  return cost * unit_t_;
}

/**
 * @brief 通过三个点计算加速度代价
 *
 * @param first 第一个点
 * @param second 第二个点（当前点的前驱）
 * @param third 第三个点（当前点）
 * @return double 加速度代价
 *
 * 算法说明：
 * 使用二阶中心差分近似加速度：
 * a = (s1 - 2*s2 + s3) / dt²
 *
 * 其中：
 * - first是前前驱点
 * - second是前驱点
 * - third是当前点
 *
 * 时间间隔均为unit_t_
 */
double DpStCost::GetAccelCostByThreePoints(const STPoint& first,
                                           const STPoint& second,
                                           const STPoint& third) {
  /**
   * @brief 计算加速度
   * a = (s_first - 2*s_second + s_third) / unit_t_²
   */
  double accel = (first.s() + third.s() - 2 * second.s()) / (unit_t_ * unit_t_);

  /**
   * @brief 调用通用加速度代价计算
   */
  return GetAccelCost(accel);
}

/**
 * @brief 通过两个点和前速度计算加速度代价
 *
 * @param pre_speed 前一时刻的速度
 * @param pre_point 前一时刻的位置
 * @param curr_point 当前时刻的位置
 * @return double 加速度代价
 *
 * 算法说明：
 * 1. 计算当前速度：v = (s_curr - s_pre) / unit_t_
 * 2. 计算加速度：a = (v - v_pre) / unit_t_
 */
double DpStCost::GetAccelCostByTwoPoints(const double pre_speed,
                                         const STPoint& pre_point,
                                         const STPoint& curr_point) {
  /**
   * @brief 计算当前速度
   */
  double current_speed = (curr_point.s() - pre_point.s()) / unit_t_;

  /**
   * @brief 计算加速度
   */
  double accel = (current_speed - pre_speed) / unit_t_;

  /**
   * @brief 调用通用加速度代价计算
   */
  return GetAccelCost(accel);
}

/**
 * @brief Jerk代价计算（缓存版本）
 *
 * @param jerk Jerk值（加速度变化率）
 * @return double Jerk代价
 *
 * 功能说明：
 * Jerk是加速度对时间的导数，反映加速度变化的剧烈程度
 * 高Jerk会导致乘客不舒适
 *
 * 代价公式：
 * - 正Jerk：positive_jerk_coeff × jerk² × unit_t_
 * - 负Jerk：negative_jerk_coeff × jerk² × unit_t_
 *
 * C++语法说明：
 * std::array<T, N>: 固定大小数组，存储预计算的Jerk代价
 * fill(-1.0): 初始化所有元素为-1，表示未计算
 */
double DpStCost::JerkCost(const double jerk) {
  /**
   * @brief 初始化代价
   */
  double cost = 0.0;

  /**
   * @brief 离散化参数
   */
  static constexpr double kEpsilon = 0.1;
  static constexpr size_t kShift = 200;  /**< 偏移量增加到200，适配更大的jerk范围 */

  /**
   * @brief 计算Jerk的离散化索引
   */
  const size_t jerk_key = static_cast<size_t>(jerk / kEpsilon + 0.5 + kShift);

  /**
   * @brief 边界检查
   */
  if (jerk_key >= jerk_cost_.size()) {
    return kInf;  /**< 超出范围 */
  }

  /**
   * @brief 检查缓存
   */
  if (jerk_cost_.at(jerk_key) < 0.0) {
    /**
     * @brief 未缓存，计算代价
     */
    double jerk_sq = jerk * jerk;

    /**
     * @brief 根据正负应用不同系数
     * 正负Jerk可能对应不同的舒适性权重
     */
    if (jerk > 0) {
      cost = config_.positive_jerk_coeff() * jerk_sq * unit_t_;
    } else {
      cost = config_.negative_jerk_coeff() * jerk_sq * unit_t_;
    }

    /**
     * @brief 缓存结果
     */
    jerk_cost_.at(jerk_key) = cost;
  } else {
    /**
     * @brief 缓存命中
     */
    cost = jerk_cost_.at(jerk_key);
  }

  /**
   * @brief TODO(All): normalize to unit_t_
   * 注释说明：当前没有按unit_t_归一化
   */
  return cost;
}

/**
 * @brief 通过四个点计算Jerk代价
 *
 * @param first 前前驱点
 * @param second 前驱点
 * @param third 当前点的前驱
 * @param fourth 当前点
 * @return double Jerk代价
 *
 * 算法说明：
 * 使用四阶中心差分近似Jerk：
 * jerk = (s1 - 3*s2 + 3*s3 - s4) / dt³
 *
 * 推导：
 * - a1 = (s1 - 2*s2 + s3) / dt²（前三点的加速度）
 * - a2 = (s2 - 2*s3 + s4) / dt²（后三点的加速度）
 * - jerk = (a2 - a1) / dt = (s1 - 3*s2 + 3*s3 - s4) / dt³
 */
double DpStCost::GetJerkCostByFourPoints(const STPoint& first,
                                         const STPoint& second,
                                         const STPoint& third,
                                         const STPoint& fourth) {
  /**
   * @brief 计算Jerk
   */
  double jerk = (fourth.s() - 3 * third.s() + 3 * second.s() - first.s()) /
                (unit_t_ * unit_t_ * unit_t_);

  /**
   * @brief 调用通用Jerk代价计算
   */
  return JerkCost(jerk);
}

/**
 * @brief 通过两个点和前速度、前加速度计算Jerk代价
 *
 * @param pre_speed 前一时刻的速度
 * @param pre_acc 前一时刻的加速度
 * @param pre_point 前一时刻的位置
 * @param curr_point 当前时刻的位置
 * @return double Jerk代价
 *
 * 算法说明：
 * 1. curr_speed = (s_curr - s_pre) / unit_t_
 * 2. curr_acc = (v_curr - v_pre) / unit_t_
 * 3. jerk = (a_curr - a_pre) / unit_t_
 */
double DpStCost::GetJerkCostByTwoPoints(const double pre_speed,
                                        const double pre_acc,
                                        const STPoint& pre_point,
                                        const STPoint& curr_point) {
  /**
   * @brief 计算当前速度
   */
  const double curr_speed = (curr_point.s() - pre_point.s()) / unit_t_;

  /**
   * @brief 计算当前加速度
   */
  const double curr_accel = (curr_speed - pre_speed) / unit_t_;

  /**
   * @brief 计算Jerk
   */
  const double jerk = (curr_accel - pre_acc) / unit_t_;

  /**
   * @brief 调用通用Jerk代价计算
   */
  return JerkCost(jerk);
}

/**
 * @brief 通过三个点和初始速度计算Jerk代价
 *
 * @param first_speed 初始速度
 * @param first 第一个点
 * @param second 第二个点
 * @param third 第三个点
 * @return double Jerk代价
 *
 * 算法说明：
 * 1. pre_speed = (s2 - s1) / unit_t_
 * 2. pre_acc = (v_pre - v0) / unit_t_ = (s2 - s1 - v0*unit_t_) / unit_t_²
 * 3. curr_speed = (s3 - s2) / unit_t_
 * 4. curr_acc = (v_curr - v_pre) / unit_t_
 * 5. jerk = (a_curr - a_pre) / unit_t_
 */
double DpStCost::GetJerkCostByThreePoints(const double first_speed,
                                          const STPoint& first,
                                          const STPoint& second,
                                          const STPoint& third) {
  /**
   * @brief 计算前速度和加速度
   */
  const double pre_speed = (second.s() - first.s()) / unit_t_;
  const double pre_acc = (pre_speed - first_speed) / unit_t_;

  /**
   * @brief 计算当前速度和加速度
   */
  const double curr_speed = (third.s() - second.s()) / unit_t_;
  const double curr_acc = (curr_speed - pre_speed) / unit_t_;

  /**
   * @brief 计算Jerk
   */
  const double jerk = (curr_acc - pre_acc) / unit_t_;

  /**
   * @brief 调用通用Jerk代价计算
   */
  return JerkCost(jerk);
}

}  // namespace planning
}  // namespace apollo
