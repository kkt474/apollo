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
 * @file sl_polygon.cc
 * @brief SL多边形实现文件
 *
 * 功能说明：
 * 实现了SL多边形(SLPolygon)类，用于在Frenet坐标系下表示障碍物的边界
 * SL坐标系：s为沿参考线的距离，l为垂直于参考线的距离
 *
 * 核心概念：
 * - SL坐标系：Frenet坐标系，沿路径方向(s)和垂直方向(l)
 * - SLBoundary：SL边界，描述障碍物在SL坐标系下的占据区域
 * - Left/Right Boundary：左右边界，组成多边形
 * - NudgeType：绕行类型（LEFT_NUDGE/RIGHT_NUDGE）
 * - 可通行性检查：判断车辆是否能绕过障碍物
 *
 * 数据结构：
 * - sl_boundary_：原始SL边界protobuf消息
 * - left_boundary_：左边界点列表（按s升序）
 * - right_boundary_：右边界点列表（按s升序）
 * - min_s_point_/max_s_point_：s方向极值点
 * - min_l_point_/max_l_point_：l方向极值点
 *
 * C++语法说明：
 * - std::numeric_limits<double>::max()：double类型的最大值
 * - std::numeric_limits<double>::lowest()：double类型的最小值
 * - std::lower_bound：二分查找
 * - std::reverse：反转容器元素
 * - std::swap：交换两个对象
 * - std::prev/next：迭代器导航
 **/
#include "modules/planning/planning_base/common/sl_polygon.h"

#include <limits>
/**
 * @brief 数值限制头文件
 *
 * 提供数值类型限制：
 * - std::numeric_limits<T>::max()：类型T的最大值
 * - std::numeric_limits<T>::lowest()：类型T的最小（负数最大）
 */

#include <string>
/**
 * @brief 字符串头文件
 *
 * 提供std::string类型
 */

#include <utility>
/**
 * @brief 工具类头文件
 *
 * 提供：
 * - std::pair：模板类
 * - std::swap：交换函数
 */

#include "modules/common/configs/vehicle_config_helper.h"
/**
 * @brief 车辆配置辅助类头文件
 *
 * VehicleConfigHelper：
 * - 获取车辆配置参数
 * - MinSafeTurnRadius()：最小安全转弯半径
 */

#include "modules/planning/planning_base/gflags/planning_gflags.h"
/**
 * @brief 规划模块配置标志头文件
 *
 * FLAGS_xxx：
 * - nonstatic_obstacle_nudge_l_buffer：非静态障碍物绕行l缓冲区
 * - max_stop_distance_obstacle：障碍物最大停车距离
 * - min_stop_distance_obstacle：障碍物最小停车距离
 */

namespace apollo {
/**
 * @brief Apollo项目主命名空间
 */

namespace planning {
/**
 * @brief 规划模块命名空间
 */

using apollo::common::VehicleConfigHelper;
/**
 * @brief 使用VehicleConfigHelper类型
 */

/**
 * @brief 从边界插值获取指定s处的L值
 *
 * @param boundary 边界点列表（按s升序）
 * @param s 沿路径的位置坐标
 * @return double 插值得到的L值
 *
 * 功能说明：
 * 使用线性插值从边界点列表中获取指定s位置的L值
 * 边界点列表按s坐标升序排列
 *
 * 算法流程：
 * 1. 边界检查：s在范围内
 * 2. 使用二分查找lower_bound找到合适的区间
 * 3. 线性插值计算L值
 *
 * C++语法说明：
 * - ACHECK：断言宏，调试模式下检查条件
 * - std::numeric_limits<double>::max()：double最大值
 * - std::lower_bound：二分查找，返回第一个不小于目标的迭代器
 * - lambda表达式：自定义比较函数
 */
double SLPolygon::GetInterpolatedLFromBoundary(
    const std::vector<SLPoint>& boundary, const double s) {
  ACHECK(!boundary.empty());
  /**
   * @brief 断言：边界列表非空
   *
   * ACHECK宏：
   * - 调试模式下，如果条件为false，程序终止
   * - 发布模式下不进行检查
   */

  if (s <= boundary.front().s()) {
    /**
     * @brief s小于等于第一个点的s
     *
     * 返回第一个点的L值
     */
    return boundary.front().l();
    /**
     * @brief .front()：获取容器第一个元素
     */
  } else if (s >= boundary.back().s()) {
    /**
     * @brief s大于等于最后一个点的s
     *
     * 返回最后一个点的L值
     */
    return boundary.back().l();
    /**
     * @brief .back()：获取容器最后一个元素
     */
  }

  SLPoint sl_point;
  /**< @brief 临时SL点 */
  sl_point.set_s(s);
  /**< @brief 设置s坐标 */
  sl_point.set_l(0.0);
  /**< @brief 初始化l为0 */

  auto cmp = [](const SLPoint& sl_point, const double s) {
    /**
     * @brief lambda比较函数
     *
     * 用于std::lower_bound的自定义比较
     * 比较SLPoint的s与给定s值
     */
    return sl_point.s() < s;
  };

  auto iter = std::lower_bound(boundary.begin(), boundary.end(), s, cmp);
  /**
   * @brief 二分查找下界
   *
   * 返回第一个s' >= s的迭代器
   */

  auto last_iter = *(iter - 1);
  /**
   * @brief 获取前一个迭代器
   *
   * *(iter - 1)：解引用获取前一个点
   */

  /**
   * @brief 线性插值计算L值
   *
   * 公式：L = L1 + (s - s1) * (L2 - L1) / (s2 - s1)
   * 其中：(s1, L1)是前一个点，(s2, L2)是当前点
   */
  return last_iter.l() + (s - last_iter.s()) * (iter->l() - last_iter.l()) /
                             (iter->s() - last_iter.s());
  /**
   * @brief iter->l()：访问当前点的l值
   * @brief iter->s()：访问当前点的s值
   */
}

/**
 * @brief 打印边界到日志
 *
 * @param prefix 前缀字符串，用于日志标识
 *
 * 功能说明：
 * 将SL边界点打印到日志，用于调试和可视化
 *
 * C++语法说明：
 * - PrintCurves：日志打印辅助类
 * - sl_boundary_.boundary_point_size()：protobuf repeated字段大小
 * - sl_boundary_.boundary_point(i)：获取第i个边界点
 */
void SLPolygon::PrintToLog(std::string prefix) const {
  PrintCurves print_curve;
  /**< @brief 曲线打印辅助对象 */
  std::string key = id_ + prefix + "_sl_boundary";
  /**< @brief 日志key，组合ID和前缀 */

  for (int i = 0; i < sl_boundary_.boundary_point_size(); i++) {
    /**
     * @brief 遍历所有边界点
     *
     * boundary_point_size()：repeated字段的元素数量
     * boundary_point(i)：获取第i个元素
     */
    print_curve.AddPoint(key, sl_boundary_.boundary_point(i).s(),
                         sl_boundary_.boundary_point(i).l());
    /**
     * @brief 添加点到曲线
     *
     * AddPoint(key, s, l)：添加名为key的(s, l)点
     */
  }
  print_curve.PrintToLog();
  /**< @brief 打印所有曲线到日志 */
}

/**
 * @brief 打印边界块到日志
 *
 * 功能说明：
 * 将SL边界作为块打印，用于调试和可视化
 * 与PrintToLog的区别：使用不同的日志key
 */
void SLPolygon::PrintToLogBlock() const {
  PrintCurves print_curve;
  std::string key = id_ + "_BlockSLPolygons";
  /**< @brief 日志key */

  for (int i = 0; i < sl_boundary_.boundary_point_size(); i++) {
    print_curve.AddPoint(key, sl_boundary_.boundary_point(i).s(),
                         sl_boundary_.boundary_point(i).l());
  }
  print_curve.PrintToLog();
}

/**
 * @brief SLPolygon构造函数
 *
 * @param sl_boundary SL边界protobuf消息
 * @param id 障碍物ID
 * @param type 障碍物类型
 * @param print_log 是否打印日志
 *
 * 功能说明：
 * 从SL边界构造SL多边形
 * 计算左右边界、极值点，并确保左边界在右边界上方
 *
 * 算法流程：
 * 1. 遍历边界点，找到s和l方向的极值点
 * 2. 提取右边界（从min_s_index到max_s_index）
 * 3. 提取左边界（从max_s_index到min_s_index）
 * 4. 确保边界按s升序排列
 * 5. 确保左边界在右边界上方
 *
 * C++语法说明：
 * - 初始化列表：: sl_boundary_(sl_boundary), id_(id), obstacle_type_(type)
 * - std::numeric_limits<double>::max()：最大值
 * - std::numeric_limits<double>::lowest()：最小值
 * - std::reverse：反转容器
 * - 模运算(t + 1) % size：循环遍历
 */
SLPolygon::SLPolygon(SLBoundary sl_boundary, std::string id,
                     PerceptionObstacle::Type type, bool print_log)
    : sl_boundary_(sl_boundary), id_(id), obstacle_type_(type) {
  /**
   * @brief 初始化列表
   *
   * 语法：Constructor() : member1(val1), member2(val2) { }
   * 在函数体执行前初始化成员变量
   */

  int min_s_index = -1;
  /**< @brief 最小s值的索引 */
  int max_s_index = -1;
  /**< @brief 最大s值的索引 */
  int min_l_index = -1;
  /**< @brief 最小l值的索引 */
  int max_l_index = -1;
  /**< @brief 最大l值的索引 */

  double min_s = std::numeric_limits<double>::max();
  /**< @brief 最小s值初始化为double最大 */
  double min_l = std::numeric_limits<double>::max();
  /**< @brief 最小l值初始化为double最大 */
  double max_s = std::numeric_limits<double>::lowest();
  /**< @brief 最大s值初始化为double最小（负数最大） */
  double max_l = std::numeric_limits<double>::lowest();
  /**< @brief 最大l值初始化为double最小 */

  /**
   * @brief 第一步：遍历边界点找极值
   */
  for (int i = 0; i < sl_boundary.boundary_point_size(); i++) {
    /**
     * @brief 遍历所有边界点
     */
    const auto& sl_point = sl_boundary.boundary_point(i);
    /**< @brief 获取第i个边界点的引用 */

    if (sl_point.s() < min_s) {
      /**
       * @brief 更新最小s值
       */
      min_s = sl_point.s();
      min_s_index = i;
    }
    if (sl_point.s() > max_s) {
      /**
       * @brief 更新最大s值
       */
      max_s = sl_point.s();
      max_s_index = i;
    }
    if (sl_point.l() < min_l) {
      /**
       * @brief 更新最小l值
       */
      min_l = sl_point.l();
      min_l_index = i;
    }
    if (sl_point.l() > max_l) {
      /**
       * @brief 更新最大l值
       */
      max_l = sl_point.l();
      max_l_index = i;
    }
  }

  /**
   * @brief 保存极值点
   */
  min_s_point_ = sl_boundary.boundary_point(min_s_index);
  /**< @brief 最小s点 */
  min_l_point_ = sl_boundary.boundary_point(min_l_index);
  /**< @brief 最小l点 */
  max_s_point_ = sl_boundary.boundary_point(max_s_index);
  /**< @brief 最大s点 */
  max_l_point_ = sl_boundary.boundary_point(max_l_index);
  /**< @brief 最大l点 */

  /**
   * @brief 第二步：提取右边界
   *
   * 从min_s_index开始，顺时针遍历到max_s_index
   * 使用模运算实现循环
   */
  int t = min_s_index;
  /**< @brief 当前索引 */
  SLPoint sl_point;
  /**< @brief 临时SL点 */

  while (t != max_s_index) {
    /**
     * @brief 循环直到达到max_s_index
     */
    sl_point.set_s(sl_boundary.boundary_point(t).s());
    sl_point.set_l(sl_boundary.boundary_point(t).l());
    right_boundary_.push_back(sl_point);
    /**
     * @brief 添加点到右边界列表
     */
    t = (t + 1) % sl_boundary.boundary_point_size();
    /**
     * @brief 模运算实现循环索引
     * 如果t+1超出范围，回到0
     */
  }

  sl_point.set_s(sl_boundary.boundary_point(t).s());
  sl_point.set_l(sl_boundary.boundary_point(t).l());
  right_boundary_.push_back(sl_point);
  /**< @brief 添加最后一个点 */

  /**
   * @brief 确保右边界按s升序
   *
   * 如果第一个点的s大于最后一个点的s，反转列表
   */
  if (right_boundary_.front().s() > right_boundary_.back().s()) {
    std::reverse(right_boundary_.begin(), right_boundary_.end());
    /**
     * @brief std::reverse：反转容器元素顺序
     */
  }

  /**
   * @brief 第三步：提取左边界
   *
   * 从max_s_index开始，顺时针遍历到min_s_index
   */
  t = max_s_index;
  while (t != min_s_index) {
    sl_point.set_s(sl_boundary.boundary_point(t).s());
    sl_point.set_l(sl_boundary.boundary_point(t).l());
    left_boundary_.push_back(sl_point);
    t = (t + 1) % sl_boundary.boundary_point_size();
  }

  sl_point.set_s(sl_boundary.boundary_point(t).s());
  sl_point.set_l(sl_boundary.boundary_point(t).l());
  left_boundary_.push_back(sl_point);
  /**< @brief 添加最后一个点 */

  /**
   * @brief 确保左边界按s升序
   */
  if (left_boundary_.front().s() > left_boundary_.back().s()) {
    std::reverse(left_boundary_.begin(), left_boundary_.end());
  }

  /**
   * @brief 第四步：确保左边界在右边界上方
   *
   * 在s的中点位置比较左右边界的L值
   * 如果左边界L值小于右边界L值，交换左右边界
   */
  double mid_s = (sl_boundary.boundary_point(min_s_index).s() +
                  sl_boundary.boundary_point(max_s_index).s()) /
                 2.0;
  /**< @brief s坐标中点 */

  if (GetInterpolatedLFromBoundary(left_boundary_, mid_s) <
      GetInterpolatedLFromBoundary(right_boundary_, mid_s)) {
    /**
     * @brief 如果左边界在右边界下方（l值更小），交换
     *
     * 交换后，左边界l值 > 右边界l值
     * 这是SL多边形的正确拓扑结构
     */
    std::swap(left_boundary_, right_boundary_);
    /**
     * @brief std::swap：交换两个容器的所有元素
     */
  }

  /**
   * @brief 第五步：可选日志打印
   */
  if (print_log) {
    PrintCurves print_curve;
    for (auto pt : right_boundary_) {
      print_curve.AddPoint("right_boundary", pt.s(), pt.l());
    }
    for (auto pt : left_boundary_) {
      print_curve.AddPoint("left_boundary", pt.s(), pt.l());
    }
    print_curve.PrintToLog();
  }
}

/**
 * @brief 从边界插值获取指定s处的L值（重载版本）
 *
 * @param boundary 边界点列表
 * @param s 沿路径的位置坐标
 * @return double 插值得到的L值
 *
 * 功能说明：
 * 与GetInterpolatedLFromBoundary类似，但实现略有不同
 * 使用std::prev获取前一个迭代器
 *
 * C++语法说明：
 * - std::prev：返回迭代器的前一个位置
 * - 返回类型是L值而非SLPoint
 */
double SLPolygon::GetInterpolatedSFromBoundary(
    const std::vector<SLPoint>& boundary, double s) {
  if (s <= boundary.front().s()) {
    return boundary.front().l();
    /**
     * @brief 返回第一个点的l值
     */
  }
  if (s >= boundary.back().s()) {
    return boundary.back().l();
    /**
     * @brief 返回最后一个点的l值
     */
  }

  /**
   * @brief 二分查找
   *
   * lambda表达式内联在参数中
   */
  auto iter = std::lower_bound(
      boundary.begin(), boundary.end(), s,
      [](const SLPoint& sl_point, const double s) { return sl_point.s() < s; });

  auto last_iter = std::prev(iter);
  /**
   * @brief std::prev：返回迭代器的前一个位置
   * 等价于 *(iter - 1)
   */

  /**
   * @brief 线性插值
   */
  double ret = last_iter->l() + (s - last_iter->s()) *
                                    (iter->l() - last_iter->l()) /
                                    (iter->s() - last_iter->s());
  return ret;
}

/**
 * @brief 更新可通行信息
 *
 * @param left_bound 左侧边界
 * @param right_bound 右侧边界
 * @param left_buffer 左侧缓冲区
 * @param right_buffer 右侧缓冲区
 * @param check_s 检查的s位置
 *
 * 功能说明：
 * 判断在给定s位置，车辆是否能从左侧或右侧绕过障碍物
 * 基于左右边界的l值和缓冲区判断
 *
 * 算法流程：
 * 1. 获取在check_s处障碍物的左右边界l值
 * 2. 判断左侧是否足够宽敞
 * 3. 判断右侧是否足够宽敞
 *
 * C++语法说明：
 * - is_passable_：bool数组，[0]左侧，[1]右侧
 */
void SLPolygon::UpdatePassableInfo(double left_bound, double right_bound,
                                   double left_buffer, double right_buffer,
                                   double check_s) {
  if (!is_passable_[0] && !is_passable_[1]) {
    /**
     * @brief 如果两侧都不可通行，直接返回
     */
    return;
  }

  double l_lower = GetRightBoundaryByS(check_s);
  /**< @brief 获取右边界在check_s处的L值 */
  double l_upper = GetLeftBoundaryByS(check_s);
  /**< @brief 获取左边界在check_s处的L值 */

  is_passable_[0] = left_bound > l_upper + left_buffer;
  /**
   * @brief 判断左侧是否可通行
   *
   * 条件：left_bound > l_upper + left_buffer
   * 含义：自车左侧边界 > 障碍物左边界 + 缓冲区
   */
  is_passable_[1] = right_bound < l_lower - right_buffer;
  /**
   * @brief 判断右侧是否可通行
   *
   * 条件：right_bound < l_lower - right_buffer
   * 含义：自车右侧边界 < 障碍物右边界 - 缓冲区
   */
}

/**
 * @brief 计算最小转弯半径停车距离
 *
 * @param adc_min_l 自车最小l值（左边缘）
 * @param adc_max_l 自车最大l值（右边缘）
 * @param ego_half_width 自车半宽
 * @return double 停车距离
 *
 * 功能说明：
 * 计算在最小转弯半径约束下，车辆能够停车的最近距离
 * 基于勾股定理计算
 *
 * 算法流程：
 * 1. 获取最小安全转弯半径
 * 2. 根据绕行类型计算横向差值
 * 3. 使用勾股定理计算停车距离
 * 4. 应用距离限制
 *
 * C++语法说明：
 * - static constexpr：编译时常量
 * - VehicleConfigHelper::MinSafeTurnRadius()：静态方法调用
 * - std::sqrt：平方根函数
 * - std::atan2：二维反正切
 * - std::fabs：浮点数绝对值
 */
double SLPolygon::MinRadiusStopDistance(double adc_min_l, double adc_max_l,
                                        double ego_half_width) {
  if (min_radius_stop_distance_ > 0) {
    /**
     * @brief 如果已计算过，直接返回缓存值
     */
    return min_radius_stop_distance_;
  }

  static constexpr double stop_distance_buffer = 0.4;
  /**< @brief 停车距离缓冲（米） */

  double min_turn_radius = VehicleConfigHelper::MinSafeTurnRadius();
  /**< @brief 获取最小安全转弯半径 */
  AINFO << "min_turn_radius: " << min_turn_radius;

  const auto& adc_param =
      VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
  /**< @brief 获取车辆参数 */

  double lateral_diff = 0.0;
  /**< @brief 横向差值 */

  double expand_adc_half_width =
      ego_half_width + FLAGS_nonstatic_obstacle_nudge_l_buffer;
  /**< @brief 扩展后的自车半宽 */
  min_turn_radius += expand_adc_half_width;
  /**< @brief 扩展最小转弯半径 */
  AINFO << "expand min_turn_radius: " << min_turn_radius;

  /**
   * @brief 根据绕行类型计算横向差值
   */
  if (nudge_type_ == NudgeType::LEFT_NUDGE) {
    /**
     * @brief 向左绕行
     *
     * 横向差值 = 障碍物最大l - 自车最小l + 缓冲区
     */
    lateral_diff =
        max_l_point_.l() - adc_min_l + FLAGS_nonstatic_obstacle_nudge_l_buffer;
  } else if (nudge_type_ == NudgeType::RIGHT_NUDGE) {
    /**
     * @brief 向右绕行
     *
     * 横向差值 = 自车最大l + 缓冲区 - 障碍物最小l
     */
    lateral_diff =
        adc_max_l + FLAGS_nonstatic_obstacle_nudge_l_buffer - min_l_point_.l();
  }

  lateral_diff = std::max(0.0, lateral_diff);
  /**< @brief 确保横向差值非负 */

  const double kEpison = 1e-5;
  /**< @brief 很小的时间常数，用于避免除零 */
  lateral_diff = std::min(lateral_diff, min_turn_radius - kEpison);
  /**< @brief 限制横向差值 */

  AINFO << "obs: " << id_ << ", lateral_diff: " << lateral_diff;

  /**
   * @brief 使用勾股定理计算停车距离
   *
   * 公式：sqrt(r² - (r-d)²) = sqrt(2rd - d²)
   * 其中：r是最小转弯半径，d是横向差值
   *
   * 推导：
   * - 车辆需要绕过障碍物
   * - 转弯半径为r，横向移动d
   * - 直角三角形勾股定理
   */
  double min_radius_stop_distance_ =
      std::sqrt(std::fabs(min_turn_radius * min_turn_radius -
                          (min_turn_radius - lateral_diff) *
                              (min_turn_radius - lateral_diff))) +
      stop_distance_buffer;

  /**
   * @brief 计算转弯heading角
   */
  double turn_heading =
      std::atan2(min_radius_stop_distance_, min_turn_radius - lateral_diff);
  /**
   * @brief std::atan2(y, x)：返回y/x的反正切
   */

  /**
   * @brief 加上前轴到中心的距离
   *
   * 停车距离从前轴计算，需要转换到后轴
   */
  min_radius_stop_distance_ +=
      adc_param.front_edge_to_center() * std::cos(turn_heading);

  /**
   * @brief 应用距离限制
   */
  min_radius_stop_distance_ =
      std::min(min_radius_stop_distance_, FLAGS_max_stop_distance_obstacle);
  /**< @brief 不超过最大停车距离 */
  min_radius_stop_distance_ =
      std::max(min_radius_stop_distance_, FLAGS_min_stop_distance_obstacle +
                                              adc_param.front_edge_to_center());
  /**< @brief 不小于最小停车距离 */

  AINFO << "obs: " << id_
        << ", min_radius_stop_distance: " << min_radius_stop_distance_;
  return min_radius_stop_distance_;
  /**< @brief 返回计算得到的停车距离 */
}

}  // namespace planning
/**
 * @brief 命名空间结束标记
 */
}  // namespace apollo
/**
 * @brief Apollo命名空间结束标记
 */
