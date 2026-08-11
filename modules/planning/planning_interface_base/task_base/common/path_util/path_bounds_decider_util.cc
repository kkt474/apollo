/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
 * @file path_bounds_decider_util.cc
 * @brief 路径边界决定工具实现文件
 *
 * 本文件实现PathBoundsDeciderUtil类，是Apollo规划模块中决定路径边界的核心工具类。
 * 路径边界定义了自车可以安全行驶的横向(l)范围。
 *
 * 主要功能：
 * 1. 路径边界初始化
 * 2. 根据车道信息限制边界
 * 3. 根据静态障碍物调整边界
 * 4. 边界松弛处理
 * 5. 绕行决策（Nudge）
 *
 * 设计特点：
 * - 基于Frenet坐标系进行边界计算
 * - 支持动态边界更新
 * - 障碍物避让决策
 * - 边界角点约束
 *
 * C++语法说明：
 * - std::numeric_limits<T>::max()/lowest(): 类型极限值
 * - std::fmin/fmax: 浮点比较函数
 * - std::vector<T>::emplace_back(): 就地构造元素
 * - lambda表达式: 匿名函数用于排序和比较
 * - std::sort: 排序算法
 * - std::max_element: 最大值查找
 * - std::tuple: 元组容器
 * - for (size_t i = 0; i < size; ++i): 无符号整数遍历
 */
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_bounds_decider_util.h"

#include <algorithm>        /**< C++标准算法库：std::sort, std::max_element等 */
#include <functional>      /**< std::function函数包装器 */
#include <limits>          /**< 类型极限值：std::numeric_limits */
#include <set>            /**< 有序集合：红黑树实现 */
#include <string>         /**< 字符串类型 */
#include <tuple>          /**< 元组容器 */
#include <unordered_map>  /**< 无序映射：哈希表实现 */
#include <vector>         /**< 动态数组容器 */

#include "modules/common/configs/vehicle_config_helper.h" /**< 车辆配置助手 */
#include "modules/common/math/linear_interpolation.h" /**< 线性插值 */
#include "modules/common/util/util.h"    /**< 通用工具函数 */
#include "modules/planning/planning_base/common/sl_polygon.h" /**< SL多边形 */
#include "modules/planning/planning_base/common/util/util.h"   /**< 规划工具函数 */
#include "modules/planning/planning_base/gflags/planning_gflags.h" /**< GFLAGS配置 */

namespace apollo {
/**
 * apollo:: - Apollo最外层命名空间
 */
namespace planning {

/**
 * using类型别名声明
 */
using apollo::common::VehicleConfigHelper;  /**< 车辆配置助手类型别名 */

/**
 * @brief 初始化路径边界
 *
 * 根据参考线和初始状态，创建初始路径边界。
 * 边界初始化为[-∞, +∞]，后续会根据车道和障碍物进一步限制。
 *
 * @param reference_line_info 参考线信息
 * @param path_bound 输出：路径边界指针
 * @param init_sl_state 初始SL状态 [s, l, ds, dl, dds, ddl]
 * @return bool 是否初始化成功
 *
 * 语法说明：
 * - PathBoundary* const path_bound: 指针本身是const，不能改变指向
 * - std::numeric_limits<double>::lowest(): double类型的最小值
 * - std::numeric_limits<double>::max(): double类型的最大值
 * - std::fmin/fmax: 比较两个浮点数返回较小/较大值
 * - FLAGS_path_bounds_decider_resolution: 路径边界分辨率配置
 */
bool PathBoundsDeciderUtil::InitPathBoundary(
    const ReferenceLineInfo& reference_line_info,
    PathBoundary* const path_bound, SLState init_sl_state) {
  /**
   * 断言检查：确保输出参数非空
   * CHECK_NOTNULL: 运行时断言，验证指针不为空
   */
  CHECK_NOTNULL(path_bound);

  /**
   * 清空现有边界
   * path_bound->clear(): 清空边界容器
   */
  path_bound->clear();

  /**
   * 获取参考线引用
   * reference_line_info.reference_line(): 获取参考线
   */
  const auto& reference_line = reference_line_info.reference_line();

  /**
   * 设置路径边界分辨率
   * path_bound->set_delta_s(): 设置相邻边界点的s间隔
   * FLAGS_path_bounds_decider_resolution: 配置参数
   */
  path_bound->set_delta_s(FLAGS_path_bounds_decider_resolution);

  /**
   * 获取车辆配置
   * VehicleConfigHelper::Instance()->GetConfig():
   *   - Instance(): 获取单例实例
   *   - GetConfig(): 获取配置
   */
  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();

  double index = 0;  /**< 索引用途的计数器 */

  /**
   * 获取参考线的towing l值
   * reference_line_info.reference_line_towing_l():
   *   - 返回拖车偏移l值向量
   */
  const auto& reference_line_towing_l =
      reference_line_info.reference_line_towing_l();

  /**
   * 循环创建路径边界点
   * for (double curr_s = init_sl_state.first[0]; ...; curr_s += ...)
   *   - curr_s: 当前累积距离坐标
   *   - init_sl_state.first[0]: 初始s坐标
   *
   * 终止条件：std::fmin(A, B)取较小值
   *   - A: init_s + max(视距, 巡航速度 × 时间长度)
   *   - B: 参考线长度
   */
  for (double curr_s = init_sl_state.first[0];
       curr_s < std::fmin(init_sl_state.first[0] +
                              std::fmax(FLAGS_path_bounds_horizon,         // 100m
                                        reference_line_info.GetCruiseSpeed() *
                                            FLAGS_trajectory_time_length),    //  8s
                          reference_line.Length());
       curr_s += FLAGS_path_bounds_decider_resolution) {   // 0.5m
    /**
     * 添加新的边界点
     * emplace_back(curr_s, std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max()):
     *   - curr_s: 当前s坐标
     *   - l_lower = lowest(): 初始下界为负无穷
     *   - l_upper = max(): 初始上界为正无穷
     */
    path_bound->emplace_back(curr_s, std::numeric_limits<double>::lowest(),
                             std::numeric_limits<double>::max());

    /**
     * 设置towing_l值
     * 如果有对应的towing值，则设置
     */
    if (index < reference_line_towing_l.size()) {
      path_bound->back().towing_l = reference_line_towing_l.at(index);
    }
    index++;
  }

  /**
   * 返回检查
   */
  if (path_bound->empty()) {
    ADEBUG << "Empty path boundary in InitPathBoundary";
    return false;  /**< 边界为空，返回失败 */
  }
  return true;  /**< 初始化成功 */
}

/**
 * @brief 获取起始点
 *
 * 将规划起始点转换到Frenet坐标系。
 * 如果使用前轴中心规划，会从前轴中心反推后轴中心。
 *
 * @param planning_start_point 规划起始轨迹点
 * @param reference_line 参考线
 * @param init_sl_state 输出：初始SL状态
 *
 * 语法说明：
 * - FLAGS_use_front_axe_center_in_path_planning: 配置标志
 * - InferFrontAxeCenterFromRearAxeCenter(): 从后轴中心推断前轴中心
 */
void PathBoundsDeciderUtil::GetStartPoint(
    common::TrajectoryPoint planning_start_point,
    const ReferenceLine& reference_line, SLState* init_sl_state) {
  /**
   * 检查是否使用前轴中心进行路径规划
   */
  if (FLAGS_use_front_axe_center_in_path_planning) {
    /**
     * 从后轴中心推断前轴中心
     */
    planning_start_point =
        InferFrontAxeCenterFromRearAxeCenter(planning_start_point);
  }

  /**
   * 打印调试信息
   * std::fixed: 固定小数点格式
   */
  AINFO << std::fixed << "Plan at the starting point: x = "
        << planning_start_point.path_point().x()
        << ", y = " << planning_start_point.path_point().y()
        << ", and angle = " << planning_start_point.path_point().theta();

  /**
   * 将轨迹点转换到Frenet坐标系
   * reference_line.ToFrenetFrame():
   *   - 将XY坐标系的轨迹点转换为SL坐标系
   *   - 返回pair<s_condition, l_condition>
   */
  *init_sl_state = reference_line.ToFrenetFrame(planning_start_point);
}

/**
 * @brief 获取自车道宽度
 *
 * @param reference_line 参考线
 * @param adc_s 自车累积s坐标
 * @return double 车道宽度（左右宽度之和）
 *
 * 语法说明：
 * - constexpr double kDefaultLaneWidth = 5.0: 编译时常量
 */
double PathBoundsDeciderUtil::GetADCLaneWidth(
    const ReferenceLine& reference_line, const double adc_s) {
  double lane_left_width = 0.0;   /**< 左侧车道宽度 */
  double lane_right_width = 0.0;  /**< 右侧车道宽度 */

  /**
   * 获取指定s处的车道宽度
   * reference_line.GetLaneWidth(s, &left, &right):
   *   - 返回是否成功获取
   */
  if (!reference_line.GetLaneWidth(adc_s, &lane_left_width,
                                   &lane_right_width)) {
    /**
     * 获取失败，使用默认宽度
     * constexpr: 编译时常量，存储在程序只读内存
     */
    constexpr double kDefaultLaneWidth = 5.0;
    AWARN << "Failed to get lane width at planning start point.";
    return kDefaultLaneWidth;
  } else {
    /**
     * 返回总宽度
     */
    return lane_left_width + lane_right_width;
  }
}

/**
 * @brief 使用缓冲区更新路径边界
 *
 * 同时更新左右边界。
 *
 * @param left_bound 左侧边界值
 * @param right_bound 右侧边界值
 * @param left_type 左边界类型
 * @param right_type 右边界类型
 * @param left_id 左边界ID
 * @param right_id 右边界ID
 * @param bound_point 输出：边界点指针
 * @return bool 是否更新成功
 */
bool PathBoundsDeciderUtil::UpdatePathBoundaryWithBuffer(
    double left_bound, double right_bound, BoundType left_type,
    BoundType right_type, std::string left_id, std::string right_id,
    PathBoundPoint* const bound_point) {
  /**
   * 分别更新左右边界
   */
  if (!UpdateLeftPathBoundaryWithBuffer(left_bound, left_type, left_id,
                                        bound_point)) {
    return false;
  }
  if (!UpdateRightPathBoundaryWithBuffer(right_bound, right_type, right_id,
                                         bound_point)) {
    return false;
  }
  return true;
}

/**
 * @brief 使用缓冲区更新左侧路径边界
 *
 * 将原始边界减去自车半宽，得到车辆可行驶的左边界。
 *
 * @param left_bound 原始左边界
 * @param left_type 左边界类型
 * @param left_id 左边界ID
 * @param bound_point 输出：边界点指针
 * @return bool 是否更新成功
 *
 * 核心逻辑：
 * 1. left_bound = left_bound - adc_half_width
 * 2. 如果新左边界比当前更严格（更小），则更新
 * 3. 检查是否导致路径被阻塞
 */
bool PathBoundsDeciderUtil::UpdateLeftPathBoundaryWithBuffer(
    double left_bound, BoundType left_type, std::string left_id,
    PathBoundPoint* const bound_point) {
  /**
   * 计算自车半宽
   * VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0:
   *   - 获取车辆宽度的一半
   */
  double adc_half_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;

  /**
   * 减去自车半宽，得到车辆外边缘的左边界
   */
  left_bound = left_bound - adc_half_width;

  /**
   * 拷贝当前边界点
   * new_point = *bound_point: 拷贝构造
   */
  PathBoundPoint new_point = *bound_point;

  /**
   * 如果新左边界比当前更严格（更小），则更新
   */
  if (new_point.l_upper.l > left_bound) {
    new_point.l_upper.l = left_bound;   /**< 更新上界l值 */
    new_point.l_upper.type = left_type;  /**< 更新边界类型 */
    new_point.l_upper.id = left_id;       /**< 更新边界ID */
  }

  /**
   * 检查路径是否被阻塞
   * 如果下界大于上界，说明无有效路径
   */
  if (new_point.l_lower.l > new_point.l_upper.l) {
    ADEBUG << "Path is blocked at" << new_point.l_lower.l << " "
           << new_point.l_upper.l;
    return false;  /**< 路径阻塞，返回失败 */
  }

  /**
   * 更新边界点
   * *bound_point = new_point: 拷贝赋值
   */
  *bound_point = new_point;
  return true;  /**< 更新成功 */
}

/**
 * @brief 使用缓冲区更新右侧路径边界
 *
 * 将原始边界加上自车半宽，得到车辆可行驶的右边界。
 *
 * @param right_bound 原始右边界
 * @param right_type 右边界类型
 * @param right_id 右边界ID
 * @param bound_point 输出：边界点指针
 * @return bool 是否更新成功
 */
bool PathBoundsDeciderUtil::UpdateRightPathBoundaryWithBuffer(
    double right_bound, BoundType right_type, std::string right_id,
    PathBoundPoint* const bound_point) {
  /**
   * 计算自车半宽
   */
  double adc_half_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;

  /**
   * 加上自车半宽，得到车辆外边缘的右边界
   */
  right_bound = right_bound + adc_half_width;

  /**
   * 拷贝当前边界点
   */
  PathBoundPoint new_point = *bound_point;

  /**
   * 如果新右边界比当前更严格（更大），则更新
   */
  if (new_point.l_lower.l < right_bound) {
    new_point.l_lower.l = right_bound;    /**< 更新下界l值 */
    new_point.l_lower.type = right_type;   /**< 更新边界类型 */
    new_point.l_lower.id = right_id;       /**< 更新边界ID */
  }

  /**
   * 检查路径是否被阻塞
   */
  if (new_point.l_lower.l > new_point.l_upper.l) {
    ADEBUG << "Path is blocked at";
    return false;  /**< 路径阻塞，返回失败 */
  }

  /**
   * 更新边界点
   */
  *bound_point = new_point;
  return true;  /**< 更新成功 */
}

/**
 * @brief 裁剪路径边界
 *
如果某点被阻塞（l_lower > l_upper），裁剪掉该点及之后的所有点。
 * @param path_blocked_idx 阻塞点索引
 * @param path_boundaries 输入/输出：路径边界
 *
 * 语法说明：
 * - while (condition) { ... }: 循环直到条件不满足
 * - path_boundaries->pop_back(): 移除最后一个元素
 */
void PathBoundsDeciderUtil::TrimPathBounds(
    const int path_blocked_idx, PathBoundary* const path_boundaries) {
  if (path_blocked_idx != -1) {  /**< -1表示无阻塞 */
    if (path_blocked_idx == 0) {
      AINFO << "Completely blocked. Cannot move at all.";
    }

    /**
     * 计算裁剪后的末端s坐标
     * front_edge_to_center: 前边缘到后轴中心的距离
     */
    double front_edge_to_center =
        VehicleConfigHelper::GetConfig().vehicle_param().front_edge_to_center();
    double trimmed_s =
        path_boundaries->at(path_blocked_idx).s - front_edge_to_center;

    AINFO << "Trimmed from " << path_boundaries->back().s << " to "
          << path_boundaries->at(path_blocked_idx).s;

    /**
     * 移除s值大于trimmed_s的边界点
     * while循环 + pop_back(): 持续移除末尾元素
     */
    while (path_boundaries->size() > 1 &&
           path_boundaries->back().s > trimmed_s) {
      path_boundaries->pop_back();  /**< 移除最后一个元素 */
    }
  }
}

/**
 * @brief 获取SL多边形列表
 *
 * 从参考线信息中提取静态障碍物的SL多边形。
 *
 * @param reference_line_info 参考线信息
 * @param polygons 输出：SL多边形向量
 * @param init_sl_state 初始SL状态
 *
 * 语法说明：
 * - obstacles.Items(): 获取障碍物指针列表
 * - polygons->emplace_back(): 在容器末尾就地构造
 * - sort(...): 标准排序算法
 * - lambda表达式作为比较函数
 */
void PathBoundsDeciderUtil::GetSLPolygons(
    const ReferenceLineInfo& reference_line_info,
    std::vector<SLPolygon>* polygons, const SLState& init_sl_state) {
  polygons->clear();  /**< 清空多边形容器 */

  /**
   * 获取路径决策中的障碍物
   * reference_line_info.path_decision().obstacles():
   *   - path_decision(): 获取路径决策引用
   *   - obstacles(): 获取障碍物集合
   */
  auto obstacles = reference_line_info.path_decision().obstacles();

  /**
   * 计算自车后边缘的s坐标
   * reference_line_info.AdcSlBoundary().start_s():
   *   - AdcSlBoundary(): 自车的SL边界
   *   - start_s(): 边界起点s坐标
   */
  const double adc_back_edge_s = reference_line_info.AdcSlBoundary().start_s();

  /**
   * 遍历所有障碍物
   * for (const auto* obstacle : obstacles.Items())
   *   - range-based for循环（C++11）
   */
  for (const auto* obstacle : obstacles.Items()) {
    /**
     * 检查障碍物是否在路径决定范围内
     * IsWithinPathDeciderScopeObstacle():
     *   - 过滤虚拟障碍物、有忽略决策的障碍物、动态障碍物
     */
    if (!IsWithinPathDeciderScopeObstacle(*obstacle)) {
      continue;  /**< 跳过不在范围内的障碍物 */
    }

    /**
     * 获取障碍物的感知多边形
     * obstacle->PerceptionPolygon(): 返回XY多边形
     */
    auto xy_poly = obstacle->PerceptionPolygon();

    /**
     * 检查障碍物是否在自车后边缘之后
     * PerceptionSLBoundary().end_s(): 障碍物SL边界终点s坐标
     * 如果在自车后方，则跳过
     */
    if (obstacle->PerceptionSLBoundary().end_s() < adc_back_edge_s) {
      continue;  /**< 障碍物在自车后方，跳过 */
    }

    /**
     * 获取障碍物的SL边界
     */
    const auto obstacle_sl = obstacle->PerceptionSLBoundary();

    /**
     * 创建SLPolygon并添加到列表
     * emplace_back(obstacle_sl, obstacle->Id(), obstacle->Perception().type()):
     *   - 就地构造SLPolygon对象
     *   - 参数：SL边界、障碍物ID、感知类型
     */
    polygons->emplace_back(obstacle_sl, obstacle->Id(),
                           obstacle->Perception().type());
  }

  /**
   * 按最小s坐标排序
   * sort(begin, end, compare):
   *   - 使用lambda表达式作为比较函数
   *   - a.MinS() < b.MinS(): 按MinS升序
   */
  sort(polygons->begin(), polygons->end(),
       [](const SLPolygon& a, const SLPolygon& b) {
         return a.MinS() < b.MinS();
       });
}

/**
 * @brief 根据SL多边形更新路径边界
 *
 * 遍历所有SL多边形（障碍物），更新路径边界。
 * 处理绕行（Nudge）决策和阻塞检测。
 *
 * @param reference_line_info 参考线信息
 * @param sl_polygon 输入/输出：SL多边形向量
 * @param init_sl_state 初始SL状态
 * @param path_boundary 输入/输出：路径边界
 * @param blocked_id 输出：阻塞障碍物ID
 * @param narrowest_width 输出：最窄路径宽度
 * @return bool 是否成功（有阻塞时返回false）
 *
 * 核心逻辑：
 * 1. 遍历路径边界的每个点
 * 2. 对每个SL多边形，计算与路径边界的交集
 * 3. 确定绕行方向（左绕或右绕）
 * 4. 更新边界或检测阻塞
 */
bool PathBoundsDeciderUtil::UpdatePathBoundaryBySLPolygon(
    const ReferenceLineInfo& reference_line_info,
    std::vector<SLPolygon>* const sl_polygon, const SLState& init_sl_state,
    PathBoundary* const path_boundary, std::string* const blocked_id,
    double* const narrowest_width) {
  std::vector<double> center_l;   /**< 路径中心线l值列表 */
  double max_nudge_check_distance;  /**< 最大绕行检查距离 */

  /**
   * 根据路径类型确定中心线
   * IsChangeLanePath(): 是否为换道路径
   * path_boundary->label().find("regular/left/right"): 检查路径标签
   */
  if (reference_line_info.IsChangeLanePath() ||
      path_boundary->label().find("regular/left") != std::string::npos ||
      path_boundary->label().find("regular/right") != std::string::npos) {
    /**
     * 换道路径：使用路径边界中心作为参考
     * (l_upper + l_lower) / 2: 计算中心l值
     */
    center_l.push_back(
        (path_boundary->front().l_upper.l + path_boundary->front().l_lower.l) *
        0.5);
    max_nudge_check_distance =
        std::max(FLAGS_max_nudge_check_distance_in_lk,
                 2 * VehicleConfigHelper::GetConfig().vehicle_param().length());
  } else {
    /**
     * 普通车道跟随路径：使用l=0（参考线中心）
     */
    center_l.push_back(0.0);
    max_nudge_check_distance =
        std::max(FLAGS_max_nudge_check_distance_in_lc,
                 2 * VehicleConfigHelper::GetConfig().vehicle_param().length());
  }

  /**
   * 初始化最窄宽度
   * path_boundary->front().l_upper.l - path_boundary->front().l_lower.l:
   *   - 初始宽度 = 第一个边界点的上下界差值
   */
  *narrowest_width =
      path_boundary->front().l_upper.l - path_boundary->front().l_lower.l;

  /**
   * 计算路径中心l值
   */
  double mid_l =
      (path_boundary->front().l_upper.l + path_boundary->front().l_lower.l) / 2;

  /**
   * 计算绕行检查的点数
   * max_nudge_check_distance / 分辨率 = 距离内的点数
   */
  size_t nudge_check_count =
      size_t(max_nudge_check_distance / FLAGS_path_bounds_decider_resolution);

  double last_max_nudge_l = center_l.front();  /**< 上次最大绕行l值 */

  /**
   * 获取自车SL边界
   */
  const double adc_end_s = reference_line_info.AdcSlBoundary().end_s() + 1.0;
  const double adc_start_s = reference_line_info.AdcSlBoundary().start_s();
  const double adc_end_l = reference_line_info.AdcSlBoundary().end_l();

  /**
   * 遍历路径边界的所有点（从1开始，跳过起始点）
   * for (size_t i = 1; i < path_boundary->size(); ++i)
   */
  for (size_t i = 1; i < path_boundary->size(); ++i) {
    double path_boundary_s = path_boundary->at(i).s;  /**< 当前s坐标 */
    auto& left_bound = path_boundary->at(i).l_upper;   /**< 左边界引用 */
    auto& right_bound = path_boundary->at(i).l_lower;  /**< 右边界引用 */

    /**
     * 记录默认宽度
     */
    double default_width = right_bound.l - left_bound.l;

    /**
     * 计算滑动窗口的起始迭代器
     * center_l.end() - std::min(count, size):
     *   - 取最后n个元素
     */
    auto begin_it =
        center_l.end() - std::min(nudge_check_count, center_l.size());

    /**
     * 找最大绕行l值
     * std::max_element: 查找最大元素
     * lambda: 比较绝对值大小
     */
    last_max_nudge_l = *std::max_element(
        begin_it, center_l.end(),
        [](double a, double b) { return std::fabs(a) < std::fabs(b); });

    AINFO << "last max nudge l: " << last_max_nudge_l;

    bool is_left_width = false;

    /**
     * 计算等效自车宽度
     * util::CalculateEquivalentEgoWidth():
     *   - 根据障碍物位置和道路结构计算等效宽度
     */
    double eq_width = util::CalculateEquivalentEgoWidth(
        reference_line_info, path_boundary_s, &is_left_width);

    /**
     * 计算左右半宽
     * is_left_width: 障碍物是否在自车左侧
     */
    double left_half_width, right_half_width;
    if (is_left_width) {
      left_half_width = eq_width + FLAGS_obstacle_lat_buffer; // 0.4
      right_half_width =
          VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0 +
          FLAGS_obstacle_lat_buffer;
    } else {
      left_half_width =
          VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0 +
          FLAGS_obstacle_lat_buffer;
      right_half_width = eq_width + FLAGS_obstacle_lat_buffer;
    }

    /**
     * 遍历所有SL多边形
     */
    for (size_t j = 0; j < sl_polygon->size(); j++) {
      /**
       * 检查是否忽略此障碍物
       */
      if (sl_polygon->at(j).NudgeInfo() == SLPolygon::IGNORE) {
        AINFO << "UpdatePathBoundaryBySLPolygon, ignore obs: "
              << sl_polygon->at(j).id();
        continue;
      }

      /**
       * 获取障碍物的s范围
       */
      double min_s = sl_polygon->at(j).MinS();
      double max_s =
          sl_polygon->at(j).MaxS() + FLAGS_obstacle_lon_end_buffer_park;  // 0.6

      /**
       * 确保障碍物s范围足够大
       */
      if (max_s - min_s < FLAGS_path_bounds_decider_resolution) {
        max_s += FLAGS_path_bounds_decider_resolution;
        min_s -= FLAGS_path_bounds_decider_resolution;
      }

      /**
       * s范围检查
       * 如果障碍物完全在当前点之前，跳过
       * 如果障碍物完全在当前点之后，停止遍历（已排序）
       */
      if (max_s < path_boundary_s) {   // 障碍物已过，跳过
        continue;
      }
      if (min_s > path_boundary_s) { // 障碍物未到 → 后面更远，退出内层循环
        break;
      }

      /**
       * 更新障碍物的可通行信息
       */
      sl_polygon->at(j).UpdatePassableInfo(left_bound.l, right_bound.l,
                                           right_half_width, left_half_width,
                                           path_boundary_s);

      /**
       * 获取障碍物在当前s处的左右边界
       * GetRightBoundaryByS/GetLeftBoundaryByS():
       *   - 在SL多边形上插值获取指定s处的l值
       */
      double l_lower = sl_polygon->at(j).GetRightBoundaryByS(path_boundary_s);
      double l_upper = sl_polygon->at(j).GetLeftBoundaryByS(path_boundary_s);

      /**
       * 计算绕行边界
       */
      PathBoundPoint obs_left_nudge_bound(
          path_boundary_s, l_upper + right_half_width, left_bound.l);
      obs_left_nudge_bound.towing_l = path_boundary->at(i).towing_l;

      PathBoundPoint obs_right_nudge_bound(path_boundary_s, right_bound.l,
                                           l_lower - left_half_width);
      obs_right_nudge_bound.towing_l = path_boundary->at(i).towing_l;

      /**
       * 确定绕行方向
       */
      if (sl_polygon->at(j).NudgeInfo() == SLPolygon::UNDEFINED) {
        AINFO << "last_max_nudge_l: " << last_max_nudge_l
              << ", obs id: " << sl_polygon->at(j).id()
              << ", obs l: " << l_lower << ", " << l_upper;

        double obs_l = (l_lower + l_upper) / 2;  /**< 障碍物中心l值 */

        /**
         * 判断绕行方向
         */
        if (sl_polygon->at(j).is_passable()[RIGHT_INDEX]) {
          if (sl_polygon->at(j).is_passable()[LEFT_INDEX]) {
            /**
             * 两边都可通行
             * 根据相对位置和历史绕行确定方向
             */
            if (std::fabs(obs_l - mid_l) < 0.4 &&
                std::fabs(path_boundary_s - init_sl_state.first[0]) < 5.0) {
              /**
               * 障碍物靠近路径中心且在起点附近
               * 根据自车初始l位置决定绕行方向
               */
              if (init_sl_state.second[0] < obs_l) {
                sl_polygon->at(j).SetNudgeInfo(SLPolygon::RIGHT_NUDGE);
                AINFO << sl_polygon->at(j).id()
                      << " right nudge with init_sl_state";
              } else {
                sl_polygon->at(j).SetNudgeInfo(SLPolygon::LEFT_NUDGE);
                AINFO << sl_polygon->at(j).id()
                      << " left nudge width init_sl_state";
              }
            } else if (path_boundary_s - adc_start_s > 0 &&
                       path_boundary_s - adc_end_s < 0) {
              /**
               * 障碍物在自车范围内
               */
              if (adc_end_l < obs_l) {
                sl_polygon->at(j).SetNudgeInfo(SLPolygon::RIGHT_NUDGE);
              } else {
                sl_polygon->at(j).SetNudgeInfo(SLPolygon::LEFT_NUDGE);
              }
            } else {
              /**
               * 根据历史最大绕行方向决定
               */
              if (last_max_nudge_l < obs_l) {
                sl_polygon->at(j).SetNudgeInfo(SLPolygon::RIGHT_NUDGE);
              } else {
                sl_polygon->at(j).SetNudgeInfo(SLPolygon::LEFT_NUDGE);
              }
            }
          } else {
            /**
             * 只有右边可通行
             */
            sl_polygon->at(j).SetNudgeInfo(SLPolygon::RIGHT_NUDGE);
            AINFO << sl_polygon->at(j).id()
                  << " right nudge, left is not passable";
          }
        } else {
          /**
           * 只有左边可通行
           */
          sl_polygon->at(j).SetNudgeInfo(SLPolygon::LEFT_NUDGE);
          AINFO << sl_polygon->at(j).id()
                << " left nudge, right is not passable";
        }
      }

      /**
       * 应用绕行边界到路径
       */
      if (sl_polygon->at(j).NudgeInfo() == SLPolygon::RIGHT_NUDGE) {
        /**
         * 右绕行：更新左边界
         */
        if (obs_right_nudge_bound.l_upper.l < path_boundary->at(i).towing_l) {
          sl_polygon->at(j).SetOverlapeWithReferCenter(true);
          sl_polygon->at(j).SetOverlapeSizeWithReference(
              path_boundary->at(i).towing_l - obs_right_nudge_bound.l_upper.l);
        }

        /**
         * 检查是否可通行
         */
        if (!sl_polygon->at(j).is_passable()[RIGHT_INDEX]) {
          /**
           * 路径被阻塞
           */
          *blocked_id = sl_polygon->at(j).id();
          AINFO << "blocked at " << *blocked_id << ", s: " << path_boundary_s
                << ", left bound: " << left_bound.l
                << ", right bound: " << right_bound.l;
          sl_polygon->at(j).SetNudgeInfo(SLPolygon::BLOCKED);
          break;  /**< 跳出障碍物遍历 */
        }

        /**
         * 更新左边界
         */
        if (obs_right_nudge_bound.l_upper.l < left_bound.l) {
          AINFO << "update left_bound: s " << path_boundary_s << ", l "
                << left_bound.l << " -> " << obs_right_nudge_bound.l_upper.l;
          left_bound.l = obs_right_nudge_bound.l_upper.l;
          left_bound.type = BoundType::OBSTACLE;
          left_bound.id = sl_polygon->at(j).id();
          *narrowest_width =
              std::min(*narrowest_width, left_bound.l - right_bound.l);
        }
      } else if (sl_polygon->at(j).NudgeInfo() == SLPolygon::LEFT_NUDGE) {
        /**
         * 左绕行：更新右边界
         */
        if (obs_left_nudge_bound.l_lower.l > path_boundary->at(i).towing_l) {
          sl_polygon->at(j).SetOverlapeWithReferCenter(true);
          sl_polygon->at(j).SetOverlapeSizeWithReference(
              obs_left_nudge_bound.l_lower.l - path_boundary->at(i).towing_l);
        }

        /**
         * 检查是否可通行
         */
        if (!sl_polygon->at(j).is_passable()[LEFT_INDEX]) {
          /**
           * 路径被阻塞
           */
          *blocked_id = sl_polygon->at(j).id();
          AINFO << "blocked at " << *blocked_id << ", s: " << path_boundary_s
                << ", left bound: " << left_bound.l
                << ", right bound: " << right_bound.l;
          sl_polygon->at(j).SetNudgeInfo(SLPolygon::BLOCKED);
          break;
        }

        /**
         * 更新右边界
         */
        if (obs_left_nudge_bound.l_lower.l > right_bound.l) {
          AINFO << "update right_bound: s " << path_boundary_s << ", l "
                << right_bound.l << " -> " << obs_left_nudge_bound.l_lower.l;
          right_bound.l = obs_left_nudge_bound.l_lower.l;
          right_bound.type = BoundType::OBSTACLE;
          right_bound.id = sl_polygon->at(j).id();
          *narrowest_width =
              std::min(*narrowest_width, left_bound.l - right_bound.l);
        }
      }

      /**
       * 更新最大绕行l值
       */
      last_max_nudge_l = std::fabs((left_bound.l + right_bound.l) / 2.0 -
                                   mid_l) > std::fabs(last_max_nudge_l - mid_l)
                             ? (left_bound.l + right_bound.l) / 2.0
                             : last_max_nudge_l;
    }

    /**
     * 检查是否阻塞
     */
    if (!blocked_id->empty()) {
      TrimPathBounds(i, path_boundary);
      *narrowest_width = default_width;
      return false;  /**< 路径被阻塞 */
    }

    /**
     * 添加中心线l值
     */
    center_l.push_back((left_bound.l + right_bound.l) / 2.0);
    AINFO << "update s: " << path_boundary_s
          << ", center_l: " << center_l.back();
  }
  return true;  /**< 边界更新成功 */
}

/**
 * @brief 添加角点约束
 *
 * 将障碍物的角点作为额外约束添加到路径边界。
 *
 * @param s 角点s坐标
 * @param l_lower 角点下界l值
 * @param l_upper 角点上界l值
 * @param path_boundary 路径边界
 * @param extra_constraints 输出：额外约束
 * @return bool 是否添加成功
 */
bool PathBoundsDeciderUtil::AddCornerPoint(
    double s, double l_lower, double l_upper, const PathBoundary& path_boundary,
    ObsCornerConstraints* extra_constraints) {
  size_t left_index = 0;   /**< 左侧插值索引 */
  size_t right_index = 0;   /**< 右侧插值索引 */
  double left_weight = 0.0;  /**< 左侧权重 */
  double right_weight = 0.0; /**< 右侧权重 */

  /**
   * 获取插值权重和索引
   * path_boundary.get_interpolated_s_weight():
   *   - 根据s值找到在边界中的位置和权重
   */
  if (!path_boundary.get_interpolated_s_weight(s, &left_weight, &right_weight,
                                               &left_index, &right_index)) {
    AERROR << "Fail to find extra path bound point in path boundary: " << s
           << ", path boundary start s: " << path_boundary.front().s
           << ", path boundary end s: " << path_boundary.back().s;
    return false;
  }

  /**
   * 过滤过近的约束点
   */
  if (left_weight < 0.05 || right_weight < 0.05) {
    return false;
  }

  ADEBUG << "corner" << s << "left_weight" << left_weight << "right_weight"
         << right_weight << "left_index" << left_index << "right_index"
         << right_index << "l_lower" << l_lower << "l_upper" << l_upper;

  /**
   * 添加约束
   * emplace_back(left_weight, right_weight, l_lower, l_upper, left_index, right_index, s):
   *   - 就地构造约束对象
   */
  extra_constraints->emplace_back(left_weight, right_weight, l_lower, l_upper,
                                  left_index, right_index, s);
  return true;
}

/**
 * @brief 添加角点约束（SLPoint版本）
 *
 * @param sl_pt SL坐标点
 * @param path_boundary 路径边界
 * @param extra_constraints 输出：额外约束
 * @param is_left 是否为左侧角点
 * @param obs_id 障碍物ID
 * @param is_front_pt 是否为前方点
 * @return bool 是否添加成功
 */
bool PathBoundsDeciderUtil::AddCornerPoint(
    SLPoint sl_pt, const PathBoundary& path_boundary,
    ObsCornerConstraints* extra_constraints, bool is_left, std::string obs_id,
    bool is_front_pt) {
  size_t left_index = 0;
  size_t right_index = 0;
  double left_weight = 0.0;
  double right_weight = 0.0;

  if (!path_boundary.get_interpolated_s_weight(
          sl_pt.s(), &left_weight, &right_weight, &left_index, &right_index)) {
    AERROR << "Fail to find extra path bound point in path boundary: "
           << sl_pt.s()
           << ", path boundary start s: " << path_boundary.front().s
           << ", path boundary end s: " << path_boundary.back().s;
    return true;  /**< 返回true避免中断流程 */
  }

  /**
   * 获取边界值
   */
  double bound_l_upper = path_boundary.get_upper_bound_by_interpolated_index(
      left_weight, right_weight, left_index, right_index);
  double bound_l_lower = path_boundary.get_lower_bound_by_interpolated_index(
      left_weight, right_weight, left_index, right_index);

  /**
   * 计算角点l值
   * is_left ? A : B: 三元条件运算符
   */
  double corner_l = is_left ? sl_pt.l() - GetBufferBetweenADCCenterAndEdge()
                            : sl_pt.l() + GetBufferBetweenADCCenterAndEdge();

  /**
   * 检查并更新边界
   */
  if ((is_left && corner_l < bound_l_upper) ||
      (!is_left && corner_l > bound_l_lower)) {
    if (is_left) {
      bound_l_upper = corner_l;
    } else {
      bound_l_lower = corner_l;
    }

    extra_constraints->emplace_back(left_weight, right_weight, bound_l_lower,
                                    bound_l_upper, left_index, right_index,
                                    sl_pt.s(), obs_id);

    /**
     * 检查是否阻塞
     */
    if (bound_l_upper < bound_l_lower) {
      extra_constraints->blocked_id = obs_id;
      extra_constraints->block_left_index = left_index;
      extra_constraints->block_right_index = right_index;
      AINFO << "AddCornerPoint blocked id: " << obs_id << ", index ["
            << left_index << ", " << right_index << "]";
      return false;  /**< 返回false表示阻塞 */
    }
  }

  /**
   * 启用扩展障碍物角点
   */
  if (FLAGS_enable_expand_obs_corner) {
    /**
     * 计算扩展s坐标
     */
    double add_s = is_front_pt ? sl_pt.s() - FLAGS_expand_obs_corner_lon_buffer
                               : sl_pt.s() + FLAGS_expand_obs_corner_lon_buffer;

    if (!path_boundary.get_interpolated_s_weight(
            add_s, &left_weight, &right_weight, &left_index, &right_index)) {
      return true;
    }

    bound_l_upper = path_boundary.get_upper_bound_by_interpolated_index(
        left_weight, right_weight, left_index, right_index);
    bound_l_lower = path_boundary.get_lower_bound_by_interpolated_index(
        left_weight, right_weight, left_index, right_index);

    if ((is_left && corner_l < bound_l_upper) ||
        (!is_left && corner_l > bound_l_lower)) {
      if (is_left) {
        bound_l_upper = corner_l;
      } else {
        bound_l_lower = corner_l;
      }

      extra_constraints->emplace_back(left_weight, right_weight, bound_l_lower,
                                      bound_l_upper, left_index, right_index,
                                      add_s, obs_id);

      if (bound_l_upper < bound_l_lower) {
        extra_constraints->blocked_id = obs_id;
        extra_constraints->block_left_index = left_index;
        extra_constraints->block_right_index = right_index;
        AINFO << "AddCornerPoint blocked id: " << obs_id << ", index ["
              << left_index << ", " << right_index << "]";
        return false;
      }
    }
  }
  return true;
}

/**
 * @brief 添加角点边界
 *
 * 遍历所有SL多边形，添加角点约束。
 *
 * @param sl_polygons SL多边形列表
 * @param path_boundary 输入/输出：路径边界
 */
void PathBoundsDeciderUtil::AddCornerBounds(
    const std::vector<SLPolygon>& sl_polygons,
    PathBoundary* const path_boundary) {
  /**
   * 获取可写的额外路径边界
   */
  auto* extra_path_bound = path_boundary->mutable_extra_path_bound();

  /**
   * 遍历所有SL多边形
   */
  for (const auto& obs_polygon : sl_polygons) {
    /**
     * 检查障碍物s范围
     */
    if (obs_polygon.MinS() > path_boundary->back().s) {
      ADEBUG << "obs_polygon.MinS()" << obs_polygon.MinS()
             << "path_boundary->back().s" << path_boundary->back().s;
      break;  /**< 障碍物在路径末端之后，停止遍历 */
    }

    if (obs_polygon.MaxS() < path_boundary->front().s) {
      continue;  /**< 障碍物在路径起始点之前，跳过 */
    }

    /**
     * 根据绕行信息添加角点
     */
    if (obs_polygon.NudgeInfo() == SLPolygon::LEFT_NUDGE) {
      /**
       * 左绕行：添加右侧角点
       */
      for (size_t i = 0; i < obs_polygon.RightBoundary().size(); i++) {
        auto pt = obs_polygon.RightBoundary().at(i);
        bool is_front_pt = i < (obs_polygon.RightBoundary().size() * 0.5);
        if (!AddCornerPoint(pt, *path_boundary, extra_path_bound, false,
                            obs_polygon.id(), is_front_pt)) {
          break;  /**< 被阻塞，停止添加 */
        }
      }
    } else if (obs_polygon.NudgeInfo() == SLPolygon::RIGHT_NUDGE) {
      /**
       * 右绕行：添加左侧角点
       */
      for (size_t i = 0; i < obs_polygon.LeftBoundary().size(); i++) {
        auto pt = obs_polygon.LeftBoundary().at(i);
        bool is_front_pt = i > (obs_polygon.LeftBoundary().size() * 0.5);
        if (!AddCornerPoint(pt, *path_boundary, extra_path_bound, true,
                            obs_polygon.id(), is_front_pt)) {
          break;
        }
      }
    } else {
      AINFO << "no nugde info, ignore obs: " << obs_polygon.id();
    }

    /**
     * 检查是否阻塞
     */
    if (!extra_path_bound->blocked_id.empty()) {
      break;  /**< 阻塞，停止添加 */
    }
  }
}

/**
 * @brief 添加自车顶点边界
 *
 * 添加自车四个角点的边界约束。
 *
 * @param path_boundary 输入/输出：路径边界
 */
void PathBoundsDeciderUtil::AddAdcVertexBounds(
    PathBoundary* const path_boundary) {
  auto* adc_vertex_bound = path_boundary->mutable_adc_vertex_bound();

  /**
   * 获取前边缘到后轴中心的距离
   */
  double front_edge_to_center = apollo::common::VehicleConfigHelper::GetConfig()
                                    .vehicle_param()
                                    .front_edge_to_center();

  /**
   * 遍历路径边界
   */
  for (size_t i = 0; i < path_boundary->size(); i++) {
    /**
     * 计算后轴s坐标
     */
    double rear_axle_s = path_boundary->at(i).s - front_edge_to_center;

    if (rear_axle_s <= path_boundary->start_s()) {
      continue;  /**< 在起始点之前，跳过 */
    }

    size_t left_index = 0;
    size_t right_index = 0;
    double left_weight = 0.0;
    double right_weight = 0.0;

    if (!path_boundary->get_interpolated_s_weight(rear_axle_s, &left_weight,
                                                  &right_weight, &left_index,
                                                  &right_index)) {
      AERROR << "Fail to find vertex path bound point in path boundary: "
             << path_boundary->at(i).s
             << "path boundary start s: " << path_boundary->front().s
             << ", path boundary end s: " << path_boundary->back().s;
      continue;
    }

    /**
     * 添加自车顶点边界
     */
    adc_vertex_bound->emplace_back(
        left_weight, right_weight, path_boundary->at(i).l_lower.l,
        path_boundary->at(i).l_upper.l, left_index, right_index, rear_axle_s);
  }

  adc_vertex_bound->front_edge_to_center = front_edge_to_center;
}

/**
 * @brief 根据静态障碍物获取边界
 *
 * 更新路径边界，处理静态障碍物。
 *
 * @param reference_line_info 参考线信息
 * @param sl_polygons SL多边形列表
 * @param init_sl_state 初始SL状态
 * @param path_boundary 输入/输出：路径边界
 * @param blocking_obstacle_id 输出：阻塞障碍物ID
 * @param narrowest_width 输出：最窄宽度
 * @return bool 是否成功
 */
bool PathBoundsDeciderUtil::GetBoundaryFromStaticObstacles(
    const ReferenceLineInfo& reference_line_info,
    std::vector<SLPolygon>* const sl_polygons, const SLState& init_sl_state,
    PathBoundary* const path_boundary, std::string* const blocking_obstacle_id,
    double* const narrowest_width) {
  // Step 1: 根据 SLPolygon 挤压边界 + 确定NUDGE方向
  UpdatePathBoundaryBySLPolygon(reference_line_info, sl_polygons, init_sl_state,
                                path_boundary, blocking_obstacle_id,
                                narrowest_width);

  // Step 2: 添加额外约束（角点约束、自车顶点约束）
  AddExtraPathBound(*sl_polygons, path_boundary, init_sl_state,
                    blocking_obstacle_id);
  return true;
}

/**
 * @brief 获取自车中心到边缘的缓冲区
 *
 * @return double 缓冲区大小
 */
double PathBoundsDeciderUtil::GetBufferBetweenADCCenterAndEdge() {
  /**
   * 自车半宽 + 障碍物横向缓冲区
   */
  double adc_half_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;

  return (adc_half_width + FLAGS_obstacle_lat_buffer);
}

/**
 * @brief 检查障碍物是否在路径决定范围内
 *
 * 过滤条件：
 * 1. 非虚拟障碍物
 * 2. 非忽略决策
 * 3. 静态障碍物
 *
 * @param obstacle 障碍物引用
 * @return bool 是否在范围内
 */
bool PathBoundsDeciderUtil::IsWithinPathDeciderScopeObstacle(
    const Obstacle& obstacle) {
  /**
   * 障碍物应该非虚拟
   */
  if (obstacle.IsVirtual()) {
    return false;
  }

  /**
   * 障碍物不应该有决策
   */
  if (obstacle.HasLongitudinalDecision() && obstacle.HasLateralDecision() &&
      obstacle.IsIgnore()) {
    return false;
  }

  /**
   * 障碍物应该静态且低速
   */
  if (!obstacle.IsStatic() ||
      obstacle.speed() > FLAGS_static_obstacle_speed_threshold) {
    return false;
  }

  return true;  /**< 在范围内 */
}

/**
 * @brief 从后轴中心推断前轴中心
 *
 * 根据后轴中心位置和航向角，计算前轴中心位置。
 *
 * @param traj_point 后轴中心的轨迹点
 * @return common::TrajectoryPoint 前轴中心的轨迹点
 *
 * 公式：
 * x_new = x + wheel_base * cos(theta)
 * y_new = y + wheel_base * sin(theta)
 */
common::TrajectoryPoint
PathBoundsDeciderUtil::InferFrontAxeCenterFromRearAxeCenter(
    const common::TrajectoryPoint& traj_point) {
  /**
   * 获取前后轴距离
   */
  double front_to_rear_axe_distance =
      VehicleConfigHelper::GetConfig().vehicle_param().wheel_base();

  /**
   * 拷贝轨迹点
   */
  common::TrajectoryPoint ret = traj_point;

  /**
   * 计算新的x坐标
   * mutable_path_point(): 获取可写的PathPoint
   */
  ret.mutable_path_point()->set_x(
      traj_point.path_point().x() +
      front_to_rear_axe_distance * std::cos(traj_point.path_point().theta()));

  /**
   * 计算新的y坐标
   */
  ret.mutable_path_point()->set_y(
      traj_point.path_point().y() +
      front_to_rear_axe_distance * std::sin(traj_point.path_point().theta()));

  return ret;
}

/**
 * @brief 根据自车道获取边界
 *
 * 使用车道边界限制路径边界。
 *
 * @param reference_line_info 参考线信息
 * @param init_sl_state 初始SL状态
 * @param path_bound 输入/输出：路径边界
 * @return bool 是否成功
 */
 // 根据自车所在车道信息，将路径边界从初始的无限大范围收窄为车道边界范围，同时减去自车半宽，确保车辆不越出车道
bool PathBoundsDeciderUtil::GetBoundaryFromSelfLane(
    const ReferenceLineInfo& reference_line_info, const SLState& init_sl_state,
    PathBoundary* const path_bound) {
  /**
   * 健全性检查
   */
  CHECK_NOTNULL(path_bound);
  ACHECK(!path_bound->empty());

  const ReferenceLine& reference_line = reference_line_info.reference_line();

  /**
   * 获取自车道的宽度
   */
  double adc_lane_width =
      GetADCLaneWidth(reference_line, init_sl_state.first[0]);

  /**
   * 初始化历史车道宽度
   */
  double past_lane_left_width = adc_lane_width / 2.0;
  double past_lane_right_width = adc_lane_width / 2.0;
  int path_blocked_idx = -1;  /**< 阻塞索引，-1表示无阻塞 */

  /**
   * 遍历所有路径边界点
   */
  for (size_t i = 0; i < path_bound->size(); ++i) {
    double curr_s = (*path_bound)[i].s;

    // ① 从HDMap查询车道宽度（相对于地图车道中心线）
    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;
    double offset_to_lane_center = 0.0;
    // 计算的宽度是相对于地图中心线
    if (!reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                     &curr_lane_right_width)) {
      AWARN << "Failed to get lane width at s = " << curr_s;
      curr_lane_left_width = past_lane_left_width;
      curr_lane_right_width = past_lane_right_width;
    } else {
      /**
       * 获取道路中心偏移：reference line 相对 HDMap lane center 的横向偏移
       */
      // offset_to_lane_center>0:参考线在地图中心线左侧
      reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);

      /**
       * 应用偏移：参考中线线由地图中心线转到参考线
       */
      curr_lane_left_width += offset_to_lane_center;
      curr_lane_right_width -= offset_to_lane_center;

      /**
       * 更新历史宽度
       */
      past_lane_left_width = curr_lane_left_width;
      past_lane_right_width = curr_lane_right_width;
    }

    /**
     * 计算边界  存在问题
     */
    double offset_to_map = 0.0;
    reference_line.GetOffsetToMap(curr_s, &offset_to_map);

    double curr_left_bound = 0.0;
    double curr_right_bound = 0.0;
    curr_left_bound = curr_lane_left_width - offset_to_map;
    curr_right_bound = -curr_lane_right_width - offset_to_map;

    /**
     * 更新路径边界
     */
    if (!UpdatePathBoundaryWithBuffer(curr_left_bound, curr_right_bound,
                                      BoundType::LANE, BoundType::LANE, "", "",
                                      &path_bound->at(i))) {
      path_blocked_idx = static_cast<int>(i);
    }

    if (path_blocked_idx != -1) {
      break;  /**< 路径被阻塞，停止更新 */
    }
  }

  /**
   * 裁剪路径边界
   */
  PathBoundsDeciderUtil::TrimPathBounds(path_blocked_idx, path_bound);

  return true;
}

/**
 * @brief 根据道路获取边界
 *
 * 使用道路边界限制路径边界。
 *
 * @param reference_line_info 参考线信息
 * @param init_sl_state 初始SL状态
 * @param path_bound 输入/输出：路径边界
 * @return bool 是否成功
 */
bool PathBoundsDeciderUtil::GetBoundaryFromRoad(
    const ReferenceLineInfo& reference_line_info, const SLState& init_sl_state,
    PathBoundary* const path_bound) {
  CHECK_NOTNULL(path_bound);
  ACHECK(!path_bound->empty());

  const ReferenceLine& reference_line = reference_line_info.reference_line();

  double adc_lane_width =
      GetADCLaneWidth(reference_line, init_sl_state.first[0]);

  /**
   * 初始化历史道路宽度
   */
  double past_road_left_width = adc_lane_width / 2.0;
  double past_road_right_width = adc_lane_width / 2.0;
  int path_blocked_idx = -1;

  /**
   * 遍历路径边界
   */
  for (size_t i = 0; i < path_bound->size(); ++i) {
    /**
     * 获取道路边界
     */
    double curr_s = (*path_bound)[i].s;
    double curr_road_left_width = 0.0;
    double curr_road_right_width = 0.0;

    if (!reference_line.GetRoadWidth(curr_s, &curr_road_left_width,
                                     &curr_road_right_width)) {
      AWARN << "Failed to get lane width at s = " << curr_s;
      curr_road_left_width = past_road_left_width;
      curr_road_right_width = past_road_right_width;
    }

    past_road_left_width = curr_road_left_width;
    past_road_right_width = curr_road_right_width;

    double curr_left_bound = curr_road_left_width;
    double curr_right_bound = -curr_road_right_width;

    ADEBUG << "At s = " << curr_s
           << ", left road bound = " << curr_road_left_width
           << ", right road bound = " << curr_road_right_width;

    /**
     * 更新到路径边界
     */
    if (!UpdatePathBoundaryWithBuffer(curr_left_bound, curr_right_bound,
                                      BoundType::ROAD, BoundType::ROAD, "", "",
                                      &path_bound->at(i))) {
      path_blocked_idx = static_cast<int>(i);
    }

    if (path_blocked_idx != -1) {
      break;
    }
  }

  AINFO << "path_blocked_idx: " << path_blocked_idx;
  TrimPathBounds(path_blocked_idx, path_bound);
  return true;
}

/**
 * @brief 根据自车扩展边界
 *
 * 使用自车位置扩展边界以包含自车。
 *
 * @param reference_line_info 参考线信息
 * @param init_sl_state 初始SL状态
 * @param extend_buffer 扩展缓冲区
 * @param path_bound 输入/输出：路径边界
 * @return bool 是否成功
 */
bool PathBoundsDeciderUtil::ExtendBoundaryByADC(
    const ReferenceLineInfo& reference_line_info, const SLState& init_sl_state,
    const double extend_buffer, PathBoundary* const path_bound) {
  /**
   * 自车相对于车道中心的l值
   */
  double adc_l_to_lane_center = init_sl_state.second[0];

  /**
   * 最大横向加速度
   * static constexpr: 静态编译时常量
   */
  static constexpr double kMaxLateralAccelerations = 1.5;

  /**
   * 根据速度计算速度缓冲区:横向偏移
   * 公式：v^2 / (2 * a_max)
   */
  double ADC_speed_buffer = (init_sl_state.second[1] > 0 ? 1.0 : -1.0) *
                            init_sl_state.second[1] * init_sl_state.second[1] /
                            kMaxLateralAccelerations / 2.0;

  /**
   * 计算自车半宽
   */
  double adc_half_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;

  /**
   * 计算左右边界
   * std::fmax/fmin: 取较大/较小值
   */
  double left_bound_adc =
      std::fmax(adc_l_to_lane_center, adc_l_to_lane_center + ADC_speed_buffer) +
      adc_half_width + extend_buffer;

  double right_bound_adc =
      std::fmin(adc_l_to_lane_center, adc_l_to_lane_center + ADC_speed_buffer) -
      adc_half_width - extend_buffer;

  /**
   * 容差值
   */
  static constexpr double kEpsilon = 0.05;

  /**
   * 遍历路径边界
   */
  for (size_t i = 0; i < path_bound->size(); ++i) {
    /**
     * 获取道路宽度
     */
    double road_left_width = std::fabs(left_bound_adc) + kEpsilon;
    double road_right_width = std::fabs(right_bound_adc) + kEpsilon;

    reference_line_info.reference_line().GetRoadWidth(
        (*path_bound)[i].s, &road_left_width, &road_right_width);

    /**
     * 计算道路边界
     */
    double left_bound_road = road_left_width - adc_half_width;
    double right_bound_road = -road_right_width + adc_half_width;

    /**
     * 更新左边界
     */
    if (left_bound_adc > (*path_bound)[i].l_upper.l) {
      (*path_bound)[i].l_upper.l =
          std::max(std::min(left_bound_adc, left_bound_road),
                   (*path_bound)[i].l_upper.l);
      (*path_bound)[i].l_upper.type = BoundType::ADC;
      (*path_bound)[i].l_upper.id = "adc";
    }

    /**
     * 更新右边界
     */
    if (right_bound_adc < (*path_bound)[i].l_lower.l) {
      (*path_bound)[i].l_lower.l =
          std::min(std::max(right_bound_adc, right_bound_road),
                   (*path_bound)[i].l_lower.l);
      (*path_bound)[i].l_lower.type = BoundType::ADC;
      (*path_bound)[i].l_lower.id = "adc";
    }
  }
  return true;
}

/**
 * @brief 将边界从车道中心转换到参考线
 *
 * @param reference_line_info 参考线信息
 * @param path_bound 输入/输出：路径边界
 */
void PathBoundsDeciderUtil::ConvertBoundarySAxisFromLaneCenterToRefLine(
    const ReferenceLineInfo& reference_line_info,
    PathBoundary* const path_bound) {
  const ReferenceLine& reference_line = reference_line_info.reference_line();

  for (size_t i = 0; i < path_bound->size(); ++i) {
    double curr_s = (*path_bound)[i].s;

    /**
     * 获取参考线相对车道中心的偏移
     */
    double refline_offset_to_lane_center = 0.0;
    reference_line.GetOffsetToMap(curr_s, &refline_offset_to_lane_center);

    /**
     * 调整边界l值
     */
    (*path_bound)[i].l_lower.l -= refline_offset_to_lane_center;
    (*path_bound)[i].l_upper.l -= refline_offset_to_lane_center;
  }
}

/**
 * @brief 检查点是否在路径边界内
 *
 * @param reference_line_info 参考线信息
 * @param x x坐标
 * @param y y坐标
 * @param path_bound 路径边界
 * @return int 匹配的边界索引，-1表示不在边界内
 */
int PathBoundsDeciderUtil::IsPointWithinPathBound(
    const ReferenceLineInfo& reference_line_info, const double x,
    const double y, const PathBound& path_bound) {
  common::SLPoint point_sl;

  /**
   * 将XY点转换到SL坐标系
   */
  reference_line_info.reference_line().XYToSL({x, y}, &point_sl);

  /**
   * 检查纵向是否在边界内
   */
  if (point_sl.s() > path_bound.back().s ||
      point_sl.s() <
          path_bound.front().s - FLAGS_path_bounds_decider_resolution * 2) {
    ADEBUG << "Longitudinally outside the boundary.";
    return -1;
  }

  /**
   * 二分查找定位
   */
  int idx_after = 0;
  while (idx_after < static_cast<int>(path_bound.size()) &&
         path_bound[idx_after].s < point_sl.s()) {
    ++idx_after;
  }

  ADEBUG << "The idx_after = " << idx_after;
  ADEBUG << "The boundary is: "
         << "[" << path_bound[idx_after].l_lower.l << ", "
         << path_bound[idx_after].l_upper.l << "].";
  ADEBUG << "The point is at: " << point_sl.l();

  /**
   * 检查横向是否在边界内
   */
  int idx_before = idx_after - 1;
  if (path_bound[idx_before].l_lower.l <= point_sl.l() &&
      path_bound[idx_before].l_upper.l >= point_sl.l() &&
      path_bound[idx_after].l_lower.l <= point_sl.l() &&
      path_bound[idx_after].l_upper.l >= point_sl.l()) {
    return idx_after;  /**< 在边界内 */
  }

  ADEBUG << "Laterally outside the boundary.";
  return -1;  /**< 不在边界内 */
}

/**
 * @brief 松弛边界点
 *
 * 根据车辆动力学约束松弛边界点。
 *
 * @param path_bound_point 输入/输出：边界点
 * @param is_left 是否为左侧
 * @param init_l 初始l值
 * @param heading 航向角
 * @param delta_s s差值
 * @param init_frenet_kappa 初始Frenet曲率
 * @param min_radius 最小转弯半径
 * @return bool 是否成功
 *
 * 语法说明：
 * - std::fabs(): 浮点数绝对值
 * - std::pow(x, y): x的y次方
 * - std::tan(): 正切函数
 */
bool PathBoundsDeciderUtil::RelaxBoundaryPoint(
    PathBoundPoint* const path_bound_point, bool is_left, double init_l,
    double heading, double delta_s, double init_frenet_kappa,
    double min_radius) {
  bool is_success = false;
  double protective_restrict = 0.0;  /**< 保护性约束 */
  double relax_constraint = 0.0;      /**< 松弛约束 */

  /**
   * 计算转弯半径
   */
  double radius = 1.0 / std::fabs(init_frenet_kappa);

  /**
   * 保存旧缓冲区
   */
  double old_buffer = FLAGS_obstacle_lat_buffer;

  /**
   * 计算新缓冲区
   */
  double new_buffer = std::max(FLAGS_ego_front_slack_buffer,
                               FLAGS_nonstatic_obstacle_nudge_l_buffer);

  if (is_left) {
    /**
     * 计算左侧弧边界
     */
    if (init_frenet_kappa > 0 && heading < 0) {
      is_success = util::left_arc_bound_with_heading_with_reverse_kappa(
          delta_s, min_radius, heading, init_frenet_kappa,
          &protective_restrict);
    } else {
      is_success = util::left_arc_bound_with_heading(delta_s, radius, heading,
                                                     &protective_restrict);
    }

    /**
     * 计算约束
     */
    relax_constraint =
        std::max(path_bound_point->l_upper.l, init_l + protective_restrict);

    AINFO << "init_pt_l: " << init_l
          << ", left_bound: " << path_bound_point->l_upper.l
          << ",  diff s: " << delta_s << ", radius: " << radius
          << ", protective_restrict: " << protective_restrict
          << ", left_obs_constraint: " << relax_constraint;

    if (path_bound_point->is_nudge_bound[LEFT_INDEX]) {
      old_buffer = std::max(FLAGS_obstacle_lat_buffer,
                            FLAGS_static_obstacle_nudge_l_buffer);
    }

    relax_constraint =
        std::min(path_bound_point->l_upper.l + old_buffer - new_buffer,
                 relax_constraint);

    AINFO << "left_obs_constraint: " << relax_constraint;
    path_bound_point->l_upper.l = relax_constraint;
  } else {
    /**
     * 计算右侧弧边界
     */
    if (init_frenet_kappa < 0 && heading > 0) {
      is_success = util::right_arc_bound_with_heading_with_reverse_kappa(
          delta_s, min_radius, heading, init_frenet_kappa,
          &protective_restrict);
    } else {
      is_success = util::right_arc_bound_with_heading(delta_s, radius, heading,
                                                      &protective_restrict);
    }

    relax_constraint =
        std::min(path_bound_point->l_lower.l, init_l + protective_restrict);

    AINFO << "init_pt_l: " << init_l
          << ", right_bound: " << path_bound_point->l_lower.l
          << ",  diff s: " << delta_s << ", radius: " << radius
          << ", protective_restrict: " << protective_restrict
          << ", right_obs_constraint: " << relax_constraint;

    if (path_bound_point->is_nudge_bound[RIGHT_INDEX]) {
      old_buffer = std::max(FLAGS_obstacle_lat_buffer,
                            FLAGS_static_obstacle_nudge_l_buffer);
    }

    relax_constraint =
        std::max(path_bound_point->l_lower.l - old_buffer + new_buffer,
                 relax_constraint);

    AINFO << "right_obs_constraint: " << relax_constraint;
    path_bound_point->l_lower.l = relax_constraint;
  }
  return is_success;
}

/**
 * @brief 松弛自车路径边界
 *
 * 根据自车初始状态和车辆动力学约束松弛边界。
 *
 * @param path_boundary 输入/输出：路径边界
 * @param init_sl_state 初始SL状态
 * @return bool 是否成功
 */
bool PathBoundsDeciderUtil::RelaxEgoPathBoundary(
    PathBoundary* const path_boundary, const SLState& init_sl_state) {
  if (path_boundary->size() < 2) {
    AINFO << "path_boundary size = 0, return.";
    return false;
  }

  /**
   * 获取车辆参数
   */
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();

  /**
   * 计算最小转弯半径
   * max(物理最小半径, 转向机构限制的半径)
   * std::tan(max_steer_angle / steer_ratio) / wheel_base:
   *   - 根据最大转向角和传动比计算
   */
  double min_radius =
      std::max(veh_param.min_turn_radius(),
               std::tan(veh_param.max_steer_angle() / veh_param.steer_ratio()) /
                   veh_param.wheel_base());

  /**
   * 计算初始Frenet曲率
   * ddl / (1 + dl^2)^1.5: 考虑横向速度的曲率
   */
  double init_frenet_kappa =
      init_sl_state.second[2] /
      std::pow(1 + std::pow(init_sl_state.second[1], 2), 1.5);

  /**
   * 限制曲率在安全范围内
   */
  if (init_frenet_kappa < 0) {
    init_frenet_kappa = std::min(
        -1.0 / (min_radius + FLAGS_relax_ego_radius_buffer), init_frenet_kappa);
  } else {
    init_frenet_kappa = std::max(
        1.0 / (min_radius + FLAGS_relax_ego_radius_buffer), init_frenet_kappa);
  }

  /**
   * 计算初始Frenet航向角
   * Vec2d(1.0, init_sl_state.second[1]).Angle():
   *   - 根据横向速度计算航向角
   */
  const auto& init_pt = path_boundary->at(0);
  double init_frenet_heading =
      common::math::Vec2d(1.0, init_sl_state.second[1]).Angle();
  double init_pt_l = init_sl_state.second[0];

  bool left_calculate_success = true;
  bool right_calculate_success = true;

  /**
   * 遍历路径边界进行松弛
   */
  for (size_t i = 1; i < path_boundary->size(); ++i) {
    auto& left_bound = path_boundary->at(i).l_upper;
    auto& right_bound = path_boundary->at(i).l_lower;

    double delta_s = path_boundary->at(i).s - init_pt.s;

    if (delta_s > FLAGS_relax_path_s_threshold) {
      left_calculate_success = false;
      right_calculate_success = false;
      break;
    }

    /**
     * 松弛左侧边界
     */
    if (left_calculate_success &&
        (left_bound.type == BoundType::OBSTACLE ||
         path_boundary->at(i).is_nudge_bound[LEFT_INDEX])) {
      left_calculate_success = RelaxBoundaryPoint(
          &path_boundary->at(i), true, init_pt_l, init_frenet_heading, delta_s,
          init_frenet_kappa, min_radius);
    }

    /**
     * 松弛右侧边界
     */
    if (right_calculate_success &&
        (right_bound.type == BoundType::OBSTACLE ||
         path_boundary->at(i).is_nudge_bound[RIGHT_INDEX])) {
      right_calculate_success = RelaxBoundaryPoint(
          &path_boundary->at(i), false, init_pt_l, init_frenet_heading, delta_s,
          init_frenet_kappa, min_radius);
    }

    if (!left_calculate_success && !right_calculate_success) {
      break;
    }
  }
  return true;
}

/**
 * @brief 松弛障碍物角点边界
 *
 * @param path_boundary 输入/输出：路径边界
 * @param init_sl_state 初始SL状态
 * @return bool 是否成功
 */
bool PathBoundsDeciderUtil::RelaxObsCornerBoundary(
    PathBoundary* const path_boundary, const SLState& init_sl_state) {
  if (path_boundary->size() < 2) {
    AINFO << "path_boundary size = 0, return.";
    return false;
  }

  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();

  double min_radius =
      std::max(veh_param.min_turn_radius(),
               std::tan(veh_param.max_steer_angle() / veh_param.steer_ratio()) /
                   veh_param.wheel_base());

  double init_frenet_kappa =
      std::fabs(init_sl_state.second[2] /
                std::pow(1 + std::pow(init_sl_state.second[1], 2), 1.5));

  if (init_frenet_kappa < 0) {
    init_frenet_kappa = std::min(
        -1.0 / (min_radius + FLAGS_relax_ego_radius_buffer), init_frenet_kappa);
  } else {
    init_frenet_kappa = std::max(
        1.0 / (min_radius + FLAGS_relax_ego_radius_buffer), init_frenet_kappa);
  }

  double kappa_radius = 1.0 / std::fabs(init_frenet_kappa);

  const auto& init_pt = path_boundary->at(0);
  double init_frenet_heading =
      common::math::Vec2d(1.0, init_sl_state.second[1]).Angle();
  double init_pt_l = init_sl_state.second[0];

  bool left_calculate_success = true;
  bool right_calculate_success = true;

  double new_buffer = std::max(FLAGS_ego_front_slack_buffer,
                               FLAGS_nonstatic_obstacle_nudge_l_buffer);

  /**
   * 遍历额外路径边界
   */
  for (auto& extra_path_bound : *(path_boundary->mutable_extra_path_bound())) {
    double delta_s = extra_path_bound.rear_axle_s - init_pt.s;

    if (delta_s > FLAGS_relax_path_s_threshold) {
      AINFO << "RelaxObsCornerBoundary delta_s: " << delta_s << " break";
      break;
    }

    AINFO << "RelaxObsCornerBoundary check delta_s: " << delta_s;

    /**
     * 计算左侧
     */
    if (left_calculate_success) {
      double left_protective_restrict = 0.0;

      if (init_frenet_kappa > 0 && init_frenet_heading < 0) {
        left_calculate_success =
            util::left_arc_bound_with_heading_with_reverse_kappa(
                delta_s, min_radius, init_frenet_heading, init_frenet_kappa,
                &left_protective_restrict);
      } else {
        left_calculate_success = util::left_arc_bound_with_heading(
            delta_s, kappa_radius, init_frenet_heading,
            &left_protective_restrict);
      }

      double left_obs_constraint = std::max(
          extra_path_bound.upper_bound, init_pt_l + left_protective_restrict);

      AINFO << "extra_path_bound, init_pt_l: " << init_pt_l
            << ", left_bound: " << extra_path_bound.upper_bound
            << ",  diff s: " << delta_s << ", min_radius: " << min_radius
            << ", init_frenet_heading: " << init_frenet_heading
            << ", protective_restrict: " << left_protective_restrict
            << ", left_obs_constraint: " << left_obs_constraint;

      left_obs_constraint = std::min(
          extra_path_bound.upper_bound + FLAGS_obstacle_lat_buffer - new_buffer,
          left_obs_constraint);

      AINFO << "extra_path_bound left_obs_constraint: " << left_obs_constraint;
      extra_path_bound.upper_bound = left_obs_constraint;
    }

    /**
     * 计算右侧
     */
    if (right_calculate_success) {
      double right_protective_restrict = 0.0;

      if (init_frenet_kappa < 0 && init_frenet_heading > 0) {
        right_calculate_success =
            util::right_arc_bound_with_heading_with_reverse_kappa(
                delta_s, min_radius, init_frenet_heading, init_frenet_kappa,
                &right_protective_restrict);
      } else {
        right_calculate_success = util::right_arc_bound_with_heading(
            delta_s, kappa_radius, init_frenet_heading,
            &right_protective_restrict);
      }

      double right_obs_constraint = std::min(
          extra_path_bound.lower_bound, init_pt_l + right_protective_restrict);

      AINFO << "extra_path_bound, init_pt_l: " << init_pt_l
            << ", right_bound: " << extra_path_bound.lower_bound
            << ",  diff s: " << delta_s << ", min_radius: " << min_radius
            << ", init_frenet_heading: " << init_frenet_heading
            << ", protective_restrict: " << right_protective_restrict
            << ", right_obs_constraint: " << right_obs_constraint;

      right_obs_constraint = std::max(
          extra_path_bound.lower_bound - FLAGS_obstacle_lat_buffer + new_buffer,
          right_obs_constraint);

      AINFO << "extra_path_bound, right_obs_constraint: "
            << right_obs_constraint;
      extra_path_bound.lower_bound = right_obs_constraint;
    }

    if (!left_calculate_success && !right_calculate_success) {
      break;
    }
  }
  return true;
}

/**
 * @brief 使用障碍物角点边界更新阻塞信息
 *
 * @param path_boundary 输入/输出：路径边界
 * @param blocked_id 输入/输出：阻塞ID
 * @return bool 是否成功
 */
bool PathBoundsDeciderUtil::UpdateBlockInfoWithObsCornerBoundary(
    PathBoundary* const path_boundary, std::string* const blocked_id) {
  if (path_boundary->extra_path_bound().blocked_id.empty()) {
    AINFO << "UpdateBlockInfoWithObsCornerBoundary, block id empty";
    return true;
  }

  auto* extra_path_bound = path_boundary->mutable_extra_path_bound();
  size_t path_boundary_block_index = extra_path_bound->block_right_index;

  /**
   * 更新阻塞ID
   */
  *blocked_id = extra_path_bound->blocked_id;
  TrimPathBounds(path_boundary_block_index, path_boundary);

  AINFO << "update block id: " << *blocked_id
        << ", path_boundary size: " << path_boundary->size();

  if (path_boundary->size() < 1) {
    extra_path_bound->clear();
    AERROR << "UpdateBlockInfoWithObsCornerBoundary, new_path_index < 1";
    return false;
  }

  /**
   * 移除超范围约束
   */
  size_t new_path_index = path_boundary->size() - 1;
  while (extra_path_bound->size() > 0 &&
         (extra_path_bound->back().id == *blocked_id ||
          extra_path_bound->back().right_index > new_path_index)) {
    AINFO << "remove extra_path_bound: s "
          << extra_path_bound->back().rear_axle_s << ", index ["
          << extra_path_bound->back().left_index << ", "
          << extra_path_bound->back().right_index << "]";
    extra_path_bound->pop_back();
  }
  return false;
}

/**
 * @brief 添加额外路径约束
 *
 * 依次调用：
 * 1. 松弛自车路径边界
 * 2. 添加角点边界
 * 3. 松弛障碍物角点边界
 * 4. 更新阻塞信息
 *
 * @param sl_polygons SL多边形列表
 * @param path_boundary 输入/输出：路径边界
 * @param init_sl_state 初始SL状态
 * @param blocked_id 输入/输出：阻塞ID
 * @return bool 是否成功
 */
bool PathBoundsDeciderUtil::AddExtraPathBound(
    const std::vector<SLPolygon>& sl_polygons,
    PathBoundary* const path_boundary, const SLState& init_sl_state,
    std::string* const blocked_id) {
  /**
   * 松弛自车路径边界
   */
  RelaxEgoPathBoundary(path_boundary, init_sl_state);

  /**
   * 如果启用角点约束
   */
  if (FLAGS_enable_corner_constraint) {
    AddCornerBounds(sl_polygons, path_boundary);
    RelaxObsCornerBoundary(path_boundary, init_sl_state);
    UpdateBlockInfoWithObsCornerBoundary(path_boundary, blocked_id);
  }

  /**
   * 如果启用自车顶点约束
   */
  if (FLAGS_enable_adc_vertex_constraint) {
    AddAdcVertexBounds(path_boundary);
  }

  return true;
}

}  // namespace planning
}  // namespace apollo
