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
 * @file reference_line.cc
 * @brief 参考线实现文件
 *
 * 本文件实现ReferenceLine类，是Apollo规划模块中参考线的核心数据结构。
 * 参考线是规划的基础，定义了车辆应该遵循的路径。
 *
 * 主要功能：
 * 1. 参考线构建和拼接
 * 2. XY坐标系与SL坐标系转换
 * 3. 参考点插值与查询
 * 4. 障碍物SL边界计算
 * 5. 道路和车道信息查询
 * 6. 速度限制管理
 *
 * 设计特点：
 * - 基于Frenet坐标系进行规划
 * - 支持路径插值和平滑
 * - 高效的最近点查找算法
 * - 使用累积s坐标加速查询
 *
 * C++语法说明：
 * - std::vector<T>: 动态数组容器
 * - std::pair<T1, T2>: 键值对容器
 * - std::array<T, N>: 固定大小数组
 * - std::move(): 移动语义
 * - std::numeric_limits<T>: 类型极限值
 * - lambda表达式: 匿名函数
 * - auto: 自动类型推导
 * - override: 覆盖基类虚函数
 */
#include "modules/planning/planning_base/reference_line/reference_line.h"

#include <algorithm>        /**< C++标准算法库：std::sort, std::lower_bound等 */
#include <limits>          /**< 类型极限值：std::numeric_limits */
#include <unordered_set>    /**< 无序集合：基于哈希表的集合 */

#include "absl/strings/str_cat.h"     /**< Abseil字符串拼接 */
#include "absl/strings/str_join.h"    /**< Abseil字符串连接 */
#include "boost/math/tools/minima.hpp" /**< Boost数学库：Brent最小值查找 */

#include "cyber/common/log.h"    /**< Cyber日志宏 */
#include "modules/common/math/angle.h"    /**< 角度计算 */
#include "modules/common/math/cartesian_frenet_conversion.h" /**< 坐标转换 */
#include "modules/common/math/linear_interpolation.h" /**< 线性插值 */
#include "modules/common/math/vec2d.h"   /**< 二维向量 */
#include "modules/common/util/map_util.h"    /**< 地图工具 */
#include "modules/common/util/string_util.h"  /**< 字符串工具 */
#include "modules/common/util/util.h"     /**< 通用工具 */
#include "modules/planning/planning_base/gflags/planning_gflags.h" /**< GFLAGS配置 */

namespace apollo {
/**
 * apollo:: - Apollo最外层命名空间
 */
namespace planning {

/**
 * using类型别名声明 - 将复杂类型简化为简短名称
 */
using MapPath = hdmap::Path;  /**< MapPath是hdmap::Path的别名 */
using apollo::common::SLPoint;  /**< SLPoint: SL坐标系下的点 */
using apollo::common::math::CartesianFrenetConverter; /**< 笛卡尔与Frenet转换器 */
using apollo::common::math::Vec2d;   /**< 二维向量类型 */
using apollo::common::util::DistanceXY;  /**< XY距离计算函数 */
using apollo::hdmap::InterpolatedIndex;  /**< 插值索引类型 */

/**
 * @brief 构造函数：从参考点向量构造参考线
 *
 * 使用参考点列表初始化参考线，同时构建地图路径。
 *
 * @param reference_points 参考点向量
 *
 * 语法说明：
 * - : reference_points_(reference_points) 初始化列表，直接初始化成员
 * - std::move(std::vector<hdmap::MapPathPoint>(...)): 移动语义构造map_path_
 * - reference_points.begin()/end(): 迭代器范围构造
 * - CHECK_EQ(a, b): 断言a==b，不相等则程序终止
 */
ReferenceLine::ReferenceLine(
    const std::vector<ReferencePoint>& reference_points)
    : reference_points_(reference_points),  /**< 直接拷贝构造 */
      map_path_(std::move(std::vector<hdmap::MapPathPoint>(  /**< 移动语义 */
          reference_points.begin(), reference_points.end()))) {
  /**
   * 断言检查：确保点数一致
   * CHECK_EQ: 运行时断言，验证两值相等
   * static_cast<size_t>: 类型转换，将int转为size_t
   * map_path_.num_points(): 获取地图路径的点数
   */
  CHECK_EQ(static_cast<size_t>(map_path_.num_points()),
           reference_points_.size());
}

/**
 * @brief 构造函数：从地图路径构造参考线
 *
 * 从hdmap::Path构造参考线，为每个路径点创建参考点。
 *
 * @param hdmap_path 地图路径
 *
 * 语法说明：
 * - for (const auto& point : hdmap_path.path_points()): range-based for
 * - DCHECK(!point.lane_waypoints().empty()): 开发时断言
 * - const auto& lane_waypoint = point.lane_waypoints()[0]: 引用声明
 * - emplace_back(): 在容器末尾就地构造元素
 */
ReferenceLine::ReferenceLine(const MapPath& hdmap_path)
    : map_path_(hdmap_path) {  /**< 直接拷贝构造地图路径 */
  /**
   * 遍历地图路径的所有路径点
   */
  for (const auto& point : hdmap_path.path_points()) {
    /**
     * 开发时断言：确保路径点有车道航路点信息
     * DCHECK: Debug断言，生产环境不执行
     */
    DCHECK(!point.lane_waypoints().empty());

    /**
     * 获取第一个车道航路点
     * point.lane_waypoints()[0]: 获取第一个车道航路点
     */
    const auto& lane_waypoint = point.lane_waypoints()[0];

    /**
     * 创建参考点并添加到列表
     * emplace_back参数：MapPathPoint, heading, kappa(0.0), dkappa(0.0)
     */
    reference_points_.emplace_back(
        hdmap::MapPathPoint(point, point.heading(), lane_waypoint), 0.0, 0.0);
  }
  CHECK_EQ(static_cast<size_t>(map_path_.num_points()),
           reference_points_.size());
}

/**
 * @brief 拼接两条参考线
 *
 * 将另一条参考线拼接到当前参考线的起点或终点。
 * 用于轨迹拼接场景。
 *
 * @param other 要拼接的参考线
 * @return bool 是否拼接成功
 *
 * 核心逻辑：
 * 1. 检查两端点是否能在另一条参考线上找到对应点
 * 2. 计算横向误差，确保在阈值内
 * 3. 使用二分查找确定拼接位置
 * 4. 插入对应的参考点
 *
 * 语法说明：
 * - std::lower_bound: 二分查找，返回第一个>=值的迭代器
 * - std::upper_bound: 二分查找，返回第一个>值的迭代器
 * - std::distance: 计算两个迭代器之间的距离
 * - std::numeric_limits<double>::max(): double类型的最大值
 */
bool ReferenceLine::Stitch(const ReferenceLine& other) {
  /**
   * 检查另一条参考线是否为空
   */
  if (other.reference_points().empty()) {
    AWARN << "The other reference line is empty.";
    return true;  /**< 空参考线视为拼接成功 */
  }

  /**
   * 获取当前参考线的首尾点
   * reference_points_.front(): 返回第一个元素引用
   * reference_points_.back(): 返回最后一个元素引用
   */
  auto first_point = reference_points_.front();
  common::SLPoint first_sl;
  /**
   * 将XY点转换到SL坐标系
   * other.XYToSL(): 在另一条参考线上投影
   */
  if (!other.XYToSL(first_point, &first_sl)) {
    AWARN << "Failed to project the first point to the other reference line.";
    return false;
  }

  /**
   * 检查首点是否在另一条参考线范围内
   * first_sl.s() > 0: 在起点之后
   * first_sl.s() < other.Length(): 在终点之前
   */
  bool first_join = first_sl.s() > 0 && first_sl.s() < other.Length();

  /**
   * 同样的逻辑检查尾点
   */
  auto last_point = reference_points_.back();
  common::SLPoint last_sl;
  if (!other.XYToSL(last_point, &last_sl)) {
    AWARN << "Failed to project the last point to the other reference line.";
    return false;
  }
  bool last_join = last_sl.s() > 0 && last_sl.s() < other.Length();

  /**
   * 两端都不能拼接，返回失败
   */
  if (!first_join && !last_join) {
    AERROR << "These reference lines are not connected.";
    return false;
  }

  /**
   * 获取累积s坐标向量
   * other.map_path().accumulated_s(): 返回累积距离向量
   */
  const auto& accumulated_s = other.map_path().accumulated_s();
  const auto& other_points = other.reference_points();
  auto lower = accumulated_s.begin();

  /**
   * 静态常量：拼接误差阈值（0.1米）
   * static constexpr: 编译时常量
   */
  static constexpr double kStitchingError = 1e-1;

  /**
   * 如果首点可以拼接
   */
  if (first_join) {
    /**
     * 检查横向误差
     * first_sl.l(): 横向偏移，理想情况下应为0
     */
    if (first_sl.l() > kStitchingError) {
      AERROR << "lateral stitching error on first join of reference line too "
                "big, stitching fails";
      return false;
    }

    /**
     * 使用二分查找定位插入位置
     * std::lower_bound: 找第一个>=first_sl.s()的位置
     */
    lower = std::lower_bound(accumulated_s.begin(), accumulated_s.end(),
                             first_sl.s());

    /**
     * 计算要插入的点数
     * std::distance: 计算迭代器距离
     */
    size_t start_i = std::distance(accumulated_s.begin(), lower);

    /**
     * 在参考线开头插入另一条线的点
     * reference_points_.insert(pos, begin, end):
     *   在pos位置插入[begin, end)范围的元素
     */
    reference_points_.insert(reference_points_.begin(), other_points.begin(),
                             other_points.begin() + start_i);
  }

  /**
   * 如果尾点可以拼接
   */
  if (last_join) {
    if (last_sl.l() > kStitchingError) {
      AERROR << "lateral stitching error on first join of reference line too "
                "big, stitching fails";
      return false;
    }

    /**
     * std::upper_bound: 找第一个>last_sl.s()的位置
     */
    auto upper = std::upper_bound(lower, accumulated_s.end(), last_sl.s());
    auto end_i = std::distance(accumulated_s.begin(), upper);

    /**
     * 在参考线末尾插入另一条线的点
     */
    reference_points_.insert(reference_points_.end(),
                             other_points.begin() + end_i, other_points.end());
  }

  /**
   * 重建地图路径
   */
  map_path_ = MapPath(std::move(std::vector<hdmap::MapPathPoint>(
      reference_points_.begin(), reference_points_.end())));
  return true;
}

/**
 * @brief 获取最近参考点（XY坐标）
 *
 * 遍历所有参考点，找到距离给定XY点最近的点。
 * 使用线性搜索，时间复杂度O(n)。
 *
 * @param xy 输入的XY坐标点
 * @return ReferencePoint 最近的参考点
 *
 * 语法说明：
 * - std::numeric_limits<double>::max(): double最大值作为初始最小值
 * - size_t: 无符号整数，用于索引
 * - DistanceXY(): XY坐标系距离计算
 */
ReferencePoint ReferenceLine::GetNearestReferencePoint(
    const common::math::Vec2d& xy) const {
  /**
   * 初始化最小距离为double最大值
   * 确保第一个点会被更新
   */
  double min_dist = std::numeric_limits<double>::max();
  size_t min_index = 0;

  /**
   * 遍历所有参考点找最近点
   * for (size_t i = 0; i < size; ++i): 无符号整数的遍历
   */
  for (size_t i = 0; i < reference_points_.size(); ++i) {
    /**
     * 计算当前点的距离
     * DistanceXY(xy, reference_points_[i]):
     *   计算xy到参考点i的距离
     */
    const double distance = DistanceXY(xy, reference_points_[i]);
    if (distance < min_dist) {
      min_dist = distance;
      min_index = i;
    }
  }
  return reference_points_[min_index];
}

/**
 * @brief 根据XY点截取参考线段
 *
 * @param point 任意XY坐标点
 * @param look_backward 向后看的距离
 * @param look_forward 向前看的距离
 * @return bool 是否成功
 */
bool ReferenceLine::Segment(const common::math::Vec2d& point,
                            const double look_backward,
                            const double look_forward) {
  common::SLPoint sl;
  if (!XYToSL(point, &sl)) {
    AERROR << "Failed to project point: " << point.DebugString();
    return false;
  }
  /**
   * 重载调用：使用s坐标进行截取
   */
  return Segment(sl.s(), look_backward, look_forward);
}

/**
 * @brief 根据s坐标截取参考线段
 *
 * 根据累积s坐标确定起止索引，截取子段。
 *
 * @param s 累积距离坐标
 * @param look_backward 向后看的距离
 * @param look_forward 向前看的距离
 * @return bool 是否成功
 *
 * 语法说明：
 * - std::lower_bound/upper_bound: 二分查找
 * - std::distance: 计算迭代器距离
 * - std::vector::insert: 在指定位置插入范围
 */
bool ReferenceLine::Segment(const double s, const double look_backward,
                            const double look_forward) {
  const auto& accumulated_s = map_path_.accumulated_s();

  /**
   * inclusive: 包含起始点
   * std::lower_bound: 找第一个 >= (s - look_backward)
   */
  auto start_index =
      std::distance(accumulated_s.begin(),
                    std::lower_bound(accumulated_s.begin(), accumulated_s.end(),
                                     s - look_backward));

  /**
   * exclusive: 不包含结束点
   * std::upper_bound: 找第一个 > (s + look_forward)
   */
  auto end_index =
      std::distance(accumulated_s.begin(),
                    std::upper_bound(accumulated_s.begin(), accumulated_s.end(),
                                     s + look_forward));

  /**
   * 检查点数是否足够
   */
  if (end_index - start_index < 2) {
    AERROR << "Too few reference points after shrinking.";
    return false;
  }

  /**
   * 更新参考点向量
   * std::vector<ReferencePoint>(begin + start, begin + end):
   *   构造新的向量，包含[start, end)范围的点
   */
  reference_points_ =
      std::vector<ReferencePoint>(reference_points_.begin() + start_index,
                                  reference_points_.begin() + end_index);

  /**
   * 重建地图路径
   */
  map_path_ = MapPath(std::vector<hdmap::MapPathPoint>(
      reference_points_.begin(), reference_points_.end()));
  return true;
}

/**
 * @brief 将路径点转换到Frenet坐标系
 *
 * 将XY坐标系的路径点转换为Frenet坐标系的点，
 * 包括s坐标、l坐标以及它们的导数。
 *
 * @param path_point XY坐标系的路径点
 * @return common::FrenetFramePoint Frenet坐标系的点
 *
 * 核心公式：
 * - s, l: 基础坐标转换
 * - dl = CalculateLateralDerivative(): 横向速度
 * - ddl = CalculateSecondOrderLateralDerivative(): 横向加速度
 */
common::FrenetFramePoint ReferenceLine::GetFrenetPoint(
    const common::PathPoint& path_point) const {
  if (reference_points_.empty()) {
    return common::FrenetFramePoint();  /**< 空参考线返回默认点 */
  }

  common::SLPoint sl_point;
  XYToSL(path_point, &sl_point);
  common::FrenetFramePoint frenet_frame_point;
  frenet_frame_point.set_s(sl_point.s());
  frenet_frame_point.set_l(sl_point.l());

  const double theta = path_point.theta();    /**< 路径点航向角 */
  const double kappa = path_point.kappa();     /**< 路径点曲率 */
  const double l = frenet_frame_point.l();    /**< 横向偏移 */

  /**
   * 获取参考点
   */
  ReferencePoint ref_point = GetReferencePoint(frenet_frame_point.s());

  const double theta_ref = ref_point.heading();  /**< 参考线航向角 */
  const double kappa_ref = ref_point.kappa();     /**< 参考线曲率 */
  const double dkappa_ref = ref_point.dkappa();   /**< 参考线曲率导数 */

  /**
   * 计算横向一阶导数（横向速度）
   * CartesianFrenetConverter::CalculateLateralDerivative()
   */
  const double dl = CartesianFrenetConverter::CalculateLateralDerivative(
      theta_ref, theta, l, kappa_ref);

  /**
   * 计算横向二阶导数（横向加速度）
   * CartesianFrenetConverter::CalculateSecondOrderLateralDerivative()
   */
  const double ddl =
      CartesianFrenetConverter::CalculateSecondOrderLateralDerivative(
          theta_ref, theta, kappa_ref, kappa, dkappa_ref, l);

  frenet_frame_point.set_dl(dl);
  frenet_frame_point.set_ddl(ddl);
  return frenet_frame_point;
}

/**
 * @brief 将轨迹点转换到Frenet坐标系
 *
 * 用于规划起始点的坐标转换。
 *
 * @param traj_point 轨迹点（含速度、加速度信息）
 * @return std::pair<std::array<double,3>, std::array<double,3>>
 *         第一对：s状态的[s, ds, dds]
 *         第二对：l状态的[l, dl, ddl]
 *
 * 语法说明：
 * - std::pair<std::array<double,3>, ...>: 嵌套容器
 * - std::array<double, 3>: 固定大小数组[值, 一阶导, 二阶导]
 * - CartesianFrenetConverter::cartesian_to_frenet(): 笛卡尔转Frenet
 */
std::pair<std::array<double, 3>, std::array<double, 3>>
ReferenceLine::ToFrenetFrame(const common::TrajectoryPoint& traj_point) const {
  ACHECK(!reference_points_.empty());  /**< 断言参考线非空 */

  common::SLPoint sl_point;
  /**
   * XYToSL重载版本：直接传入heading和xy坐标
   */
  XYToSL(traj_point.path_point().theta(),
         {traj_point.path_point().x(), traj_point.path_point().y()}, &sl_point);

  std::array<double, 3> s_condition;  /**< s状态: [s, ds, dds] */
  std::array<double, 3> l_condition;  /**< l状态: [l, dl, ddl] */

  ReferencePoint ref_point = GetReferencePoint(sl_point.s());

  /**
   * 笛卡尔坐标系转Frenet坐标系
   * 输入：
   *   - 参考点信息(x, y, heading, kappa, dkappa)
   *   - 轨迹点信息(x, y, v, a, theta, kappa)
   * 输出：
   *   - s_condition: [s, ds, dds]
   *   - l_condition: [l, dl, ddl]
   */
  CartesianFrenetConverter::cartesian_to_frenet(
      sl_point.s(), ref_point.x(), ref_point.y(), ref_point.heading(),
      ref_point.kappa(), ref_point.dkappa(), traj_point.path_point().x(),
      traj_point.path_point().y(), traj_point.v(), traj_point.a(),
      traj_point.path_point().theta(), traj_point.path_point().kappa(),
      &s_condition, &l_condition);

  /**
   * 打印调试信息
   * std::fixed: 固定小数点格式
   */
  AINFO << "planning_start_point x,y,the,k: " << std::fixed
        << traj_point.path_point().x() << ", y: " << traj_point.path_point().y()
        << "," << traj_point.path_point().theta() << ","
        << traj_point.path_point().kappa() << "," << traj_point.v() << ","
        << traj_point.a();
  AINFO << "ref point x y the ka dka" << std::fixed << ref_point.x() << ","
        << ref_point.y() << "," << ref_point.heading() << ","
        << ref_point.kappa() << "," << ref_point.dkappa();

  return std::make_pair(s_condition, l_condition);  /**< 构造并返回pair */
}

/**
 * @brief 根据累积s坐标获取最近参考点
 *
 * 利用累积s坐标进行二分查找，比线性搜索更快。
 *
 * @param s 累积距离坐标
 * @return ReferencePoint 最近的参考点
 *
 * 语法说明：
 * - accumulated_s.front()/back(): 获取首尾元素
 * - std::fabs(): 浮点数绝对值
 * - 1e-2: 容差值（0.01米）
 */
ReferencePoint ReferenceLine::GetNearestReferencePoint(const double s) const {
  const auto& accumulated_s = map_path_.accumulated_s();

  /**
   * s小于起点：返回首点
   * 1e-2容差处理浮点误差
   */
  if (s < accumulated_s.front() - 1e-2) {
    AWARN << "The requested s: " << s << " < 0.";
    return reference_points_.front();
  }

  /**
   * s大于终点：返回尾点
   */
  if (s > accumulated_s.back() + 1e-2) {
    AWARN << "The requested s: " << s
          << " > reference line length: " << accumulated_s.back();
    return reference_points_.back();
  }

  /**
   * 二分查找
   * std::lower_bound: 返回第一个>=s的迭代器
   */
  auto it_lower =
      std::lower_bound(accumulated_s.begin(), accumulated_s.end(), s);

  if (it_lower == accumulated_s.begin()) {
    return reference_points_.front();
  }

  /**
   * 比较前后两个点的距离
   */
  auto index = std::distance(accumulated_s.begin(), it_lower);
  if (std::fabs(accumulated_s[index - 1] - s) <
      std::fabs(accumulated_s[index] - s)) {
    return reference_points_[index - 1];
  }
  return reference_points_[index];
}

/**
 * @brief 获取最近参考点索引
 *
 * @param s 累积距离坐标
 * @return size_t 最近参考点的索引
 */
size_t ReferenceLine::GetNearestReferenceIndex(const double s) const {
  const auto& accumulated_s = map_path_.accumulated_s();
  if (s < accumulated_s.front() - 1e-2) {
    AWARN << "The requested s: " << s << " < 0.";
    return 0;
  }
  if (s > accumulated_s.back() + 1e-2) {
    AWARN << "The requested s: " << s << " > reference line length "
          << accumulated_s.back();
    return reference_points_.size() - 1;
  }
  auto it_lower =
      std::lower_bound(accumulated_s.begin(), accumulated_s.end(), s);
  return std::distance(accumulated_s.begin(), it_lower);
}

/**
 * @brief 获取指定s范围的参考点
 *
 * @param start_s 起始s坐标
 * @param end_s 结束s坐标
 * @return std::vector<ReferencePoint> 指定范围内的参考点
 */
std::vector<ReferencePoint> ReferenceLine::GetReferencePoints(
    double start_s, double end_s) const {
  if (start_s < 0.0) {
    start_s = 0.0;
  }
  if (end_s > Length()) {
    end_s = Length();
  }
  std::vector<ReferencePoint> ref_points;
  auto start_index = GetNearestReferenceIndex(start_s);
  auto end_index = GetNearestReferenceIndex(end_s);
  if (start_index < end_index) {
    ref_points.assign(reference_points_.begin() + start_index,
                      reference_points_.begin() + end_index);
  }
  return ref_points;
}

/**
 * @brief 根据s坐标获取参考点（支持插值）
 *
 * 如果s正好对应某个参考点，返回该点；
 * 否则在相邻点之间进行线性插值。
 *
 * @param s 累积距离坐标
 * @return ReferencePoint 插值后的参考点
 *
 * 语法说明：
 * - map_path_.GetIndexFromS(): 获取插值索引
 * - InterpolateWithMatchedIndex(): 基于索引的插值
 */
ReferencePoint ReferenceLine::GetReferencePoint(const double s) const {
  const auto& accumulated_s = map_path_.accumulated_s();
  if (s < accumulated_s.front() - 1e-2) {
    ADEBUG << "The requested s: " << s << " < 0.";
    return reference_points_.front();
  }
  if (s > accumulated_s.back() + 1e-2) {
    ADEBUG << "The requested s: " << s
          << " > reference line length: " << accumulated_s.back();
    return reference_points_.back();
  }

  /**
   * 获取插值索引
   * InterpolatedIndex包含：
   *   - id: 基准点索引
   *   - offset: 在[id, id+1]之间的偏移比例
   */
  auto interpolate_index = map_path_.GetIndexFromS(s);

  size_t index = interpolate_index.id;
  size_t next_index = index + 1;
  if (next_index >= reference_points_.size()) {
    next_index = reference_points_.size() - 1;
  }

  const auto& p0 = reference_points_[index];
  const auto& p1 = reference_points_[next_index];

  const double s0 = accumulated_s[index];
  const double s1 = accumulated_s[next_index];

  /**
   * 使用匹配索引进行插值
   */
  return InterpolateWithMatchedIndex(p0, s0, p1, s1, interpolate_index);
}

/**
 * @brief 找最优点（使用Brent方法）
 *
 * 在[p0, p1]区间内找到使得到指定点(x,y)距离最小的s值。
 *
 * @param p0 起点参考点
 * @param s0 起点s坐标
 * @param p1 终点参考点
 * @param s1 终点s坐标
 * @param x 目标点x坐标
 * @param y 目标点y坐标
 * @return double 最小距离对应的s值
 *
 * 语法说明：
 * - lambda表达式: [&p0, &p1, &s0, &s1, &x, &y](const double s) { ... }
 * - ::boost::math::tools::brent_find_minima: Brent黄金分割法求最小值
 */
double ReferenceLine::FindMinDistancePoint(const ReferencePoint& p0,
                                           const double s0,
                                           const ReferencePoint& p1,
                                           const double s1, const double x,
                                           const double y) {
  /**
   * 定义距离平方函数
   * lambda表达式捕获外部变量
   */
  auto func_dist_square = [&p0, &p1, &s0, &s1, &x, &y](const double s) {
    auto p = Interpolate(p0, s0, p1, s1, s);
    double dx = p.x() - x;
    double dy = p.y() - y;
    return dx * dx + dy * dy;
  };

  /**
   * Brent方法求最小值
   * 参数：函数, 下界, 上界, 迭代次数
   * 返回.first为最小值点，.second为最小值
   */
  return ::boost::math::tools::brent_find_minima(func_dist_square, s0, s1, 8)
      .first;
}

/**
 * @brief 根据XY坐标获取参考点（带优化）
 *
 * 使用先搜索后Brent优化的方法找最近点。
 *
 * @param x x坐标
 * @param y y坐标
 * @return ReferencePoint 最近的参考点
 */
ReferencePoint ReferenceLine::GetReferencePoint(const double x,
                                                const double y) const {
  CHECK_GE(reference_points_.size(), 0U);  /**< 断言非空 */

  /**
   * 定义距离平方函数
   */
  auto func_distance_square = [](const ReferencePoint& point, const double x,
                                 const double y) {
    double dx = point.x() - x;
    double dy = point.y() - y;
    return dx * dx + dy * dy;
  };

  double d_min = func_distance_square(reference_points_.front(), x, y);
  size_t index_min = 0;

  /**
   * 第一遍：线性搜索找初始最近点
   */
  for (size_t i = 1; i < reference_points_.size(); ++i) {
    double d_temp = func_distance_square(reference_points_[i], x, y);
    if (d_temp < d_min) {
      d_min = d_temp;
      index_min = i;
    }
  }

  /**
   * 确定搜索范围：前后各一个点
   */
  size_t index_start = index_min == 0 ? index_min : index_min - 1;
  size_t index_end =
      index_min + 1 == reference_points_.size() ? index_min : index_min + 1;

  if (index_start == index_end) {
    return reference_points_[index_start];
  }

  double s0 = map_path_.accumulated_s()[index_start];
  double s1 = map_path_.accumulated_s()[index_end];

  /**
   * 第二遍：Brent优化找精确最优点
   */
  double s = ReferenceLine::FindMinDistancePoint(
      reference_points_[index_start], s0, reference_points_[index_end], s1, x,
      y);

  return Interpolate(reference_points_[index_start], s0,
                     reference_points_[index_end], s1, s);
}

/**
 * @brief SL坐标转XY坐标
 *
 * 根据s坐标找到参考点，然后根据l偏移计算XY坐标。
 *
 * @param sl_point SL坐标点
 * @param xy_point 输出：XY坐标点
 * @return bool 是否转换成功
 *
 * 公式：
 * x = x_ref - l * sin(heading)
 * y = y_ref + l * cos(heading)
 */
bool ReferenceLine::SLToXY(const SLPoint& sl_point,
                           common::math::Vec2d* const xy_point) const {
  if (map_path_.num_points() < 2) {
    AERROR << "The reference line has too few points.";
    return false;
  }

  const auto matched_point = GetReferencePoint(sl_point.s());

  /**
   * 获取角度的16位表示（提高精度和兼容性）
   * common::math::Angle16::from_rad: 从弧度创建角度
   */
  const auto angle = common::math::Angle16::from_rad(matched_point.heading());

  /**
   * SL到XY的转换
   * sin/cos: 三角函数计算垂直方向偏移
   */
  xy_point->set_x(matched_point.x() - common::math::sin(angle) * sl_point.l());
  xy_point->set_y(matched_point.y() + common::math::cos(angle) * sl_point.l());
  return true;
}

/**
 * @brief XY坐标转SL坐标（基础版本）
 *
 * @param xy_point XY坐标点
 * @param sl_point 输出：SL坐标点
 * @param warm_start_s 起始s猜测值（加速）
 * @return bool 是否转换成功
 */
bool ReferenceLine::XYToSL(const common::math::Vec2d& xy_point,
                           common::SLPoint* const sl_point,
                           double warm_start_s) const {
  double s = warm_start_s;
  double l = 0.0;

  /**
   * 根据warm_start_s选择不同投影方法
   */
  if (warm_start_s < 0.0) {
    if (!map_path_.GetProjection(xy_point, &s, &l)) {
      AERROR << "Cannot get nearest point from path.";
      return false;
    }
  } else {
    if (!map_path_.GetProjectionWithWarmStartS(xy_point, &s, &l)) {
      AERROR << "Cannot get nearest point from path with warm_start_s: "
             << warm_start_s;
      return false;
    }
  }

  sl_point->set_s(s);
  sl_point->set_l(l);
  return true;
}

/**
 * @brief XY坐标转SL坐标（带航向版本）
 *
 * @param heading 航向角
 * @param xy_point XY坐标点
 * @param sl_point 输出：SL坐标点
 * @param warm_start_s 起始s猜测值
 * @return bool 是否转换成功
 */
bool ReferenceLine::XYToSL(const double heading,
                           const common::math::Vec2d& xy_point,
                           common::SLPoint* const sl_point,
                           double warm_start_s) const {
  double s = warm_start_s;
  double l = 0.0;
  if (warm_start_s < 0.0) {
    if (!map_path_.GetProjection(heading, xy_point, &s, &l)) {
      AERROR << "Cannot get nearest point from path.";
      return false;
    }
  } else {
    if (!map_path_.GetProjectionWithWarmStartS(xy_point, &s, &l)) {
      AERROR << "Cannot get nearest point from path with warm_start_s: "
             << warm_start_s;
      return false;
    }
  }

  sl_point->set_s(s);
  sl_point->set_l(l);
  return true;
}

/**
 * @brief XY坐标转SL坐标（启发式版本）
 *
 * @param xy_point XY坐标点
 * @param sl_point 输出：SL坐标点
 * @param hueristic_start_s 启发式搜索起始s
 * @param hueristic_end_s 启发式搜索结束s
 * @return bool 是否转换成功
 */
bool ReferenceLine::XYToSL(const common::math::Vec2d& xy_point,
                           common::SLPoint* const sl_point,
                           double hueristic_start_s,
                           double hueristic_end_s) const {
  double s = 0.0;
  double l = 0.0;
  double min_distance = 0.0;
  if (!map_path_.GetProjectionWithHueristicParams(xy_point, hueristic_start_s,
                                                  hueristic_end_s, &s, &l,
                                                  &min_distance)) {
    AERROR << "Cannot get nearest point from path with hueristic_start_s: "
           << hueristic_start_s << " hueristic_end_s: " << hueristic_end_s;
    return false;
  }
  sl_point->set_s(s);
  sl_point->set_l(l);
  return true;
}

/**
 * @brief 基于匹配索引的参考点插值
 *
 * @param p0 起点参考点
 * @param s0 起点s坐标
 * @param p1 终点参考点
 * @param s1 终点s坐标
 * @param index 插值索引
 * @return ReferencePoint 插值结果
 */
ReferencePoint ReferenceLine::InterpolateWithMatchedIndex(
    const ReferencePoint& p0, const double s0, const ReferencePoint& p1,
    const double s1, const InterpolatedIndex& index) const {
  if (std::fabs(s0 - s1) < common::math::kMathEpsilon) {
    return p0;  /**< 起点终点重合，返回起点 */
  }

  double s = s0 + index.offset;  /**< 计算目标s坐标 */

  /**
   * 开发时断言：确保s在有效范围内
   * DCHECK_LE: 断言第一个参数<=第二个参数
   */
  DCHECK_LE(s0 - 1.0e-6, s) << "s: " << s << " is less than s0 : " << s0;
  DCHECK_LE(s, s1 + 1.0e-6) << "s: " << s << " is larger than s1: " << s1;

  /**
   * 获取平滑路径点
   */
  auto map_path_point = map_path_.GetSmoothPoint(index);

  /**
   * 设置航向
   * map_path_.unit_directions()[index.id].Angle():
   *   获取指定索引的单位方向向量，转为角度
   */
  map_path_point.set_heading(map_path_.unit_directions()[index.id].Angle());

  /**
   * 线性插值曲率和曲率导数
   * common::math::lerp(): 线性插值函数
   */
  const double kappa = common::math::lerp(p0.kappa(), s0, p1.kappa(), s1, s);
  const double dkappa = common::math::lerp(p0.dkappa(), s0, p1.dkappa(), s1, s);

  return ReferencePoint(map_path_point, kappa, dkappa);
}

/**
 * @brief 通用参考点插值函数
 *
 * 在两个参考点之间进行线性插值。
 *
 * @param p0 起点参考点
 * @param s0 起点s坐标
 * @param p1 终点参考点
 * @param s1 终点s坐标
 * @param s 目标s坐标
 * @return ReferencePoint 插值结果
 *
 * 插值内容：x, y, heading, kappa, dkappa
 */
ReferencePoint ReferenceLine::Interpolate(const ReferencePoint& p0,
                                          const double s0,
                                          const ReferencePoint& p1,
                                          const double s1, const double s) {
  if (std::fabs(s0 - s1) < common::math::kMathEpsilon) {
    return p0;
  }
  DCHECK_LE(s0 - 1.0e-6, s) << " s: " << s << " is less than s0 :" << s0;
  DCHECK_LE(s, s1 + 1.0e-6) << "s: " << s << " is larger than s1: " << s1;

  /**
   * 线性插值x, y坐标
   */
  const double x = common::math::lerp(p0.x(), s0, p1.x(), s1, s);
  const double y = common::math::lerp(p0.y(), s0, p1.y(), s1, s);

  /**
   * 球面线性插值航向角
   * slerp vs lerp: 航向角需要考虑角度回绕问题
   */
  const double heading =
      common::math::slerp(p0.heading(), s0, p1.heading(), s1, s);

  /**
   * 线性插值曲率和曲率导数
   */
  const double kappa = common::math::lerp(p0.kappa(), s0, p1.kappa(), s1, s);
  const double dkappa = common::math::lerp(p0.dkappa(), s0, p1.dkappa(), s1, s);

  /**
   * 处理车道航路点信息
   */
  std::vector<hdmap::LaneWaypoint> waypoints;
  if (!p0.lane_waypoints().empty() && !p1.lane_waypoints().empty()) {
    const auto& p0_waypoint = p0.lane_waypoints()[0];
    if ((s - s0) + p0_waypoint.s <= p0_waypoint.lane->total_length()) {
      const double lane_s = p0_waypoint.s + s - s0;
      waypoints.emplace_back(p0_waypoint.lane, lane_s);
    }
    const auto& p1_waypoint = p1.lane_waypoints()[0];
    if (p1_waypoint.lane->id().id() != p0_waypoint.lane->id().id() &&
        p1_waypoint.s - (s1 - s) >= 0) {
      const double lane_s = p1_waypoint.s - (s1 - s);
      waypoints.emplace_back(p1_waypoint.lane, lane_s);
    }
    if (waypoints.empty()) {
      const double lane_s = p0_waypoint.s;
      waypoints.emplace_back(p0_waypoint.lane, lane_s);
    }
  }

  /**
   * 构造并返回插值后的参考点
   */
  return ReferencePoint(hdmap::MapPathPoint({x, y}, heading, waypoints), kappa,
                        dkappa);
}

/**
 * @brief 获取参考点向量引用
 */
const std::vector<ReferencePoint>& ReferenceLine::reference_points() const {
  return reference_points_;
}

/**
 * @brief 获取地图路径引用
 */
const MapPath& ReferenceLine::map_path() const { return map_path_; }

/**
 * @brief 获取指定s处的车道宽度
 *
 * @param s 累积距离坐标
 * @param lane_left_width 输出：左侧宽度
 * @param lane_right_width 输出：右侧宽度
 * @return bool 是否获取成功
 */
bool ReferenceLine::GetLaneWidth(const double s, double* const lane_left_width,
                                 double* const lane_right_width) const {
  if (map_path_.path_points().empty()) {
    return false;
  }

  if (!map_path_.GetLaneWidth(s, lane_left_width, lane_right_width)) {
    return false;
  }
  return true;
}

/**
 * @brief 获取道路偏移
 *
 * @param s 累积距离坐标
 * @param l_offset 输出：偏移值
 * @return bool 是否获取成功
 */
bool ReferenceLine::GetOffsetToMap(const double s, double* l_offset) const {
  if (map_path_.path_points().empty()) {
    return false;
  }

  auto ref_point = GetNearestReferencePoint(s);
  if (ref_point.lane_waypoints().empty()) {
    return false;
  }
  *l_offset = ref_point.lane_waypoints().front().l;
  return true;
}

/**
 * @brief 获取指定s处的道路宽度
 */
bool ReferenceLine::GetRoadWidth(const double s, double* const road_left_width,
                                 double* const road_right_width) const {
  if (map_path_.path_points().empty()) {
    return false;
  }
  return map_path_.GetRoadWidth(s, road_left_width, road_right_width);
}

/**
 * @brief 获取道路类型
 *
 * @param s 累积距离坐标
 * @return hdmap::Road::Type 道路类型
 */
hdmap::Road::Type ReferenceLine::GetRoadType(const double s) const {
  const hdmap::HDMap* hdmap = hdmap::HDMapUtil::BaseMapPtr();
  CHECK_NOTNULL(hdmap);

  hdmap::Road::Type road_type = hdmap::Road::UNKNOWN;

  SLPoint sl_point;
  sl_point.set_s(s);
  sl_point.set_l(0.0);
  common::math::Vec2d pt;
  SLToXY(sl_point, &pt);

  common::PointENU point;
  point.set_x(pt.x());
  point.set_y(pt.y());
  point.set_z(0.0);
  std::vector<hdmap::RoadInfoConstPtr> roads;

  /**
   * 查询给定点的道路信息
   * 搜索半径4.0米
   */
  hdmap->GetRoads(point, 4.0, &roads);
  for (auto road : roads) {
    if (road->type() != hdmap::Road::UNKNOWN) {
      road_type = road->type();
      break;
    }
  }
  return road_type;
}

/**
 * @brief 获取车道边界类型
 */
void ReferenceLine::GetLaneBoundaryType(
    const double s, hdmap::LaneBoundaryType::Type* const left_boundary_type,
    hdmap::LaneBoundaryType::Type* const right_boundary_type) const {
  auto ref_point = GetReferencePoint(s);
  const auto waypoint = ref_point.lane_waypoints().front();
  *left_boundary_type = hdmap::LeftBoundaryType(waypoint);
  *right_boundary_type = hdmap::RightBoundaryType(waypoint);
}

/**
 * @brief 根据s坐标获取经过的车道列表
 *
 * @param s 累积距离坐标
 * @param lanes 输出：车道信息指针向量
 */
void ReferenceLine::GetLaneFromS(
    const double s, std::vector<hdmap::LaneInfoConstPtr>* lanes) const {
  CHECK_NOTNULL(lanes);
  auto ref_point = GetReferencePoint(s);
  std::unordered_set<hdmap::LaneInfoConstPtr> lane_set;

  /**
   * 使用unordered_set去重
   */
  for (const auto& lane_waypoint : ref_point.lane_waypoints()) {
    if (common::util::InsertIfNotPresent(&lane_set, lane_waypoint.lane)) {
      lanes->push_back(lane_waypoint.lane);
    }
  }
}

/**
 * @brief 获取可行驶宽度
 *
 * 根据SL边界和车道宽度计算可行驶区域宽度。
 *
 * @param sl_boundary SL边界
 * @return double 可行驶宽度
 */
double ReferenceLine::GetDrivingWidth(const SLBoundary& sl_boundary) const {
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  GetLaneWidth(sl_boundary.start_s(), &lane_left_width, &lane_right_width);

  /**
   * 计算左右两侧的可行驶宽度
   */
  double driving_width = std::max(lane_left_width - sl_boundary.end_l(),
                                  lane_right_width + sl_boundary.start_l());
  driving_width = std::min(lane_left_width + lane_right_width, driving_width);
  ADEBUG << "Driving width [" << driving_width << "].";
  return driving_width;
}

/**
 * @brief 检查点是否在车道上
 */
bool ReferenceLine::IsOnLane(const common::math::Vec2d& vec2d_point) const {
  common::SLPoint sl_point;
  if (!XYToSL(vec2d_point, &sl_point)) {
    return false;
  }
  return IsOnLane(sl_point);
}

/**
 * @brief 检查SL边界是否在车道上
 */
bool ReferenceLine::IsOnLane(const SLBoundary& sl_boundary) const {
  if (sl_boundary.end_s() < 0 || sl_boundary.start_s() > Length()) {
    return false;
  }
  double middle_s = (sl_boundary.start_s() + sl_boundary.end_s()) / 2.0;
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  map_path_.GetLaneWidth(middle_s, &lane_left_width, &lane_right_width);
  return sl_boundary.start_l() <= lane_left_width &&
         sl_boundary.end_l() >= -lane_right_width;
}

/**
 * @brief 检查SL点是否在车道上
 */
bool ReferenceLine::IsOnLane(const SLPoint& sl_point) const {
  if (sl_point.s() <= 0 || sl_point.s() > map_path_.length()) {
    return false;
  }
  double left_width = 0.0;
  double right_width = 0.0;

  if (!GetLaneWidth(sl_point.s(), &left_width, &right_width)) {
    return false;
  }

  return sl_point.l() >= -right_width && sl_point.l() <= left_width;
}

/**
 * @brief 检查包围盒是否阻塞道路
 */
bool ReferenceLine::IsBlockRoad(const common::math::Box2d& box2d,
                                double gap) const {
  return map_path_.OverlapWith(box2d, gap);
}

/**
 * @brief 检查点是否在道路上
 */
bool ReferenceLine::IsOnRoad(const common::math::Vec2d& vec2d_point) const {
  common::SLPoint sl_point;
  return XYToSL(vec2d_point, &sl_point) && IsOnRoad(sl_point);
}

/**
 * @brief 检查SL边界是否在道路上
 */
bool ReferenceLine::IsOnRoad(const SLBoundary& sl_boundary) const {
  if (sl_boundary.end_s() < 0 || sl_boundary.start_s() > Length()) {
    return false;
  }
  double middle_s = (sl_boundary.start_s() + sl_boundary.end_s()) / 2.0;
  double road_left_width = 0.0;
  double road_right_width = 0.0;
  map_path_.GetRoadWidth(middle_s, &road_left_width, &road_right_width);
  return sl_boundary.start_l() <= road_left_width &&
         sl_boundary.end_l() >= -road_right_width;
}

/**
 * @brief 检查SL点是否在道路上
 */
bool ReferenceLine::IsOnRoad(const SLPoint& sl_point) const {
  if (sl_point.s() <= 0 || sl_point.s() > map_path_.length()) {
    return false;
  }
  double road_left_width = 0.0;
  double road_right_width = 0.0;

  if (!GetRoadWidth(sl_point.s(), &road_left_width, &road_right_width)) {
    return false;
  }

  return sl_point.l() >= -road_right_width && sl_point.l() <= road_left_width;
}

/**
 * @brief 获取近似SL边界（使用包围盒长度）
 *
 * 这是一个近似算法，保证结果比精确边界更大。
 *
 * @param box 车辆包围盒
 * @param start_s 起始s坐标
 * @param end_s 结束s坐标
 * @param sl_boundary 输出：SL边界
 * @return bool 是否成功
 */
bool ReferenceLine::GetApproximateSLBoundary(
    const common::math::Box2d& box, const double start_s, const double end_s,
    SLBoundary* const sl_boundary) const {
  double s = 0.0;
  double l = 0.0;
  double distance = 0.0;
  if (!map_path_.GetProjectionWithHueristicParams(box.center(), start_s, end_s,
                                                  &s, &l, &distance)) {
    AERROR << "Cannot get projection point from path.";
    return false;
  }

  auto projected_point = map_path_.GetSmoothPoint(s);

  /**
   * 旋转包围盒到与参考线对齐
   * RotatedFromCenter: 以中心点为轴旋转
   */
  auto rotated_box = box;
  rotated_box.RotateFromCenter(-projected_point.heading());

  std::vector<common::math::Vec2d> corners;
  rotated_box.GetAllCorners(&corners);

  /**
   * 初始化边界值
   * std::numeric_limits<double>::max(): double最大值
   * std::numeric_limits<double>::lowest(): double最小值
   */
  double min_s(std::numeric_limits<double>::max());
  double max_s(std::numeric_limits<double>::lowest());
  double min_l(std::numeric_limits<double>::max());
  double max_l(std::numeric_limits<double>::lowest());

  /**
   * 遍历所有角点找边界
   */
  for (const auto& point : corners) {
    /**
     * x <--> s, y <--> l
     * 因为包围盒已旋转到与参考线对齐
     */
    min_s = std::fmin(min_s, point.x() - rotated_box.center().x() + s);
    max_s = std::fmax(max_s, point.x() - rotated_box.center().x() + s);
    min_l = std::fmin(min_l, point.y() - rotated_box.center().y() + l);
    max_l = std::fmax(max_l, point.y() - rotated_box.center().y() + l);
  }
  sl_boundary->set_start_s(min_s);
  sl_boundary->set_end_s(max_s);
  sl_boundary->set_start_l(min_l);
  sl_boundary->set_end_l(max_l);
  return true;
}

/**
 * @brief 获取包围盒的SL边界
 */
bool ReferenceLine::GetSLBoundary(const common::math::Box2d& box,
                                  SLBoundary* const sl_boundary,
                                  double warm_start_s) const {
  std::vector<common::math::Vec2d> corners;
  box.GetAllCorners(&corners);
  return GetSLBoundary(corners, sl_boundary, warm_start_s);
}

/**
 * @brief 获取多边形的SL边界
 */
bool ReferenceLine::GetSLBoundary(const common::math::Polygon2d& polygon,
                                  SLBoundary* const sl_boundary,
                                  double warm_start_s) const {
  std::vector<common::math::Vec2d> corners = polygon.points();
  return GetSLBoundary(corners, sl_boundary, warm_start_s);
}

/**
 * @brief 获取角点列表的SL边界
 *
 * 核心SL边界计算算法：
 * 1. 将所有角点转换到SL坐标系
 * 2. 逆时针遍历边，找凸包顶点
 * 3. 计算s和l的边界
 *
 * @param corners 角点列表
 * @param sl_boundary 输出：SL边界
 * @param warm_start_s 起始s猜测值
 * @return bool 是否成功
 */
bool ReferenceLine::GetSLBoundary(
    const std::vector<common::math::Vec2d>& corners,
    SLBoundary* const sl_boundary, double warm_start_s) const {
  double start_s(std::numeric_limits<double>::max());
  double end_s(std::numeric_limits<double>::lowest());
  double start_l(std::numeric_limits<double>::max());
  double end_l(std::numeric_limits<double>::lowest());

  /**
   * The order must be counter-clockwise
   * 角点必须按逆时针顺序
   */
  std::vector<SLPoint> sl_corners;
  std::vector<common::math::Vec2d> obs_corners = corners;

  /**
   * get first point which is closest to ego position
   * 找距离自车最近的角点作为起点
   */
  {
    int first_index = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (int i = 0; i < obs_corners.size(); ++i) {
      double ego_dist = ego_position_.DistanceTo(obs_corners[i]);
      if (ego_dist < min_dist) {
        min_dist = ego_dist;
        first_index = i;
      }
    }

    /**
     * std::rotate: 旋转向量，使first_index成为新起点
     */
    std::rotate(obs_corners.begin(), obs_corners.begin() + first_index,
                obs_corners.end());

    const common::math::Vec2d& first_point = obs_corners.front();
    ADEBUG << "first_point: " << std::setprecision(9) << first_point.x() << ", "
           << first_point.y();

    SLPoint first_sl_point;
    if (!XYToSL(first_point, &first_sl_point, warm_start_s)) {
      AERROR << "Failed to get projection for point: "
             << first_point.DebugString() << " on reference line.";
      return false;
    }
    sl_corners.push_back(std::move(first_sl_point));
  }

  /**
   * 转换其余角点到SL坐标系
   */
  double hueristic_start_s = 0.0;
  double hueristic_end_s = 0.0;
  double distance = 0.0;
  SLPoint sl_point;
  for (size_t i = 1; i < obs_corners.size(); ++i) {
    distance = obs_corners[i].DistanceTo(obs_corners[i - 1]);
    hueristic_start_s = sl_corners.back().s() - 2.0 * distance;
    hueristic_end_s = sl_corners.back().s() + 2.0 * distance;
    if (!XYToSL(obs_corners[i], &sl_point, hueristic_start_s,
                hueristic_end_s)) {
      AERROR << "Failed to get projection for point: "
             << obs_corners[i].DebugString() << " on reference line.";
      return false;
    }
    sl_corners.push_back(std::move(sl_point));
  }

  /**
   * 逆时针遍历边，添加凸包顶点
   */
  for (size_t i = 0; i < obs_corners.size(); ++i) {
    auto index0 = i;
    auto index1 = (i + 1) % obs_corners.size();  /**< 环形索引 */
    const auto& p0 = obs_corners[index0];
    const auto& p1 = obs_corners[index1];

    const auto p_mid = (p0 + p1) * 0.5;  /**< 边中点 */
    distance = obs_corners[index0].DistanceTo(p_mid);
    hueristic_start_s = sl_corners[index0].s() - 2.0 * distance;
    hueristic_end_s = sl_corners[index0].s() + 2.0 * distance;
    SLPoint sl_point_mid;
    if (!XYToSL(p_mid, &sl_point_mid, hueristic_start_s, hueristic_end_s)) {
      AERROR << "Failed to get projection for point: " << p_mid.DebugString()
             << " on reference line.";
      return false;
    }

    /**
     * 构建SL坐标系下的边向量
     */
    Vec2d v0(sl_corners[index1].s() - sl_corners[index0].s(),
             sl_corners[index1].l() - sl_corners[index0].l());

    Vec2d v1(sl_point_mid.s() - sl_corners[index0].s(),
             sl_point_mid.l() - sl_corners[index0].l());

    *sl_boundary->add_boundary_point() = sl_corners[index0];

    /**
     * 叉积判断凸性
     * v0.CrossProd(v1) < 0: 凹点，需要添加
     */
    if (v0.CrossProd(v1) < 0.0) {
      *sl_boundary->add_boundary_point() = sl_point_mid;
    }
  }

  /**
   * 计算s和l的边界值
   */
  for (const auto& sl_point : sl_boundary->boundary_point()) {
    start_s = std::fmin(start_s, sl_point.s());
    end_s = std::fmax(end_s, sl_point.s());
    start_l = std::fmin(start_l, sl_point.l());
    end_l = std::fmax(end_l, sl_point.l());
  }

  sl_boundary->set_start_s(start_s);
  sl_boundary->set_end_s(end_s);
  sl_boundary->set_start_l(start_l);
  sl_boundary->set_end_l(end_l);
  return true;
}

/**
 * @brief 获取指定s范围的车道段
 */
std::vector<hdmap::LaneSegment> ReferenceLine::GetLaneSegments(
    const double start_s, const double end_s) const {
  return map_path_.GetLaneSegments(start_s, end_s);
}

/**
 * @brief 获取多边形的SL边界（简化版）
 */
bool ReferenceLine::GetSLBoundary(const hdmap::Polygon& polygon,
                                  SLBoundary* const sl_boundary) const {
  double start_s(std::numeric_limits<double>::max());
  double end_s(std::numeric_limits<double>::lowest());
  double start_l(std::numeric_limits<double>::max());
  double end_l(std::numeric_limits<double>::lowest());

  for (const auto& point : polygon.point()) {
    SLPoint sl_point;
    if (!XYToSL(point, &sl_point)) {
      AERROR << "Failed to get projection for point: " << point.DebugString()
             << " on reference line.";
      return false;
    }
    start_s = std::fmin(start_s, sl_point.s());
    end_s = std::fmax(end_s, sl_point.s());
    start_l = std::fmin(start_l, sl_point.l());
    end_l = std::fmax(end_l, sl_point.l());
  }
  sl_boundary->set_start_s(start_s);
  sl_boundary->set_end_s(end_s);
  sl_boundary->set_start_l(start_l);
  sl_boundary->set_end_l(end_l);
  return true;
}

/**
 * @brief 检查是否有重叠
 */
bool ReferenceLine::HasOverlap(const common::math::Box2d& box) const {
  SLBoundary sl_boundary;
  if (!GetSLBoundary(box, &sl_boundary)) {
    AERROR << "Failed to get sl boundary for box: " << box.DebugString();
    return false;
  }
  if (sl_boundary.end_s() < 0 || sl_boundary.start_s() > Length()) {
    return false;
  }
  if (sl_boundary.start_l() * sl_boundary.end_l() < 0) {
    return false;
  }

  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  const double mid_s = (sl_boundary.start_s() + sl_boundary.end_s()) / 2.0;
  if (mid_s < 0 || mid_s > Length()) {
    ADEBUG << "ref_s is out of range: " << mid_s;
    return false;
  }
  if (!map_path_.GetLaneWidth(mid_s, &lane_left_width, &lane_right_width)) {
    AERROR << "Failed to get width at s = " << mid_s;
    return false;
  }
  if (sl_boundary.start_l() > 0) {
    return sl_boundary.start_l() < lane_left_width;
  } else {
    return sl_boundary.end_l() > -lane_right_width;
  }
}

/**
 * @brief 调试信息字符串
 */
std::string ReferenceLine::DebugString() const {
  const auto limit =
      std::min(reference_points_.size(),
               static_cast<size_t>(FLAGS_trajectory_point_num_for_debug));
  return absl::StrCat(
      "point num:", reference_points_.size(),
      absl::StrJoin(reference_points_.begin(),
                    reference_points_.begin() + limit, "",
                    apollo::common::util::DebugStringFormatter()));
}

/**
 * @brief 获取指定s处的速度限制
 *
 * 综合考虑地图速度限制和自定义速度限制。
 *
 * @param s 累积距离坐标
 * @return double 速度限制值
 */
double ReferenceLine::GetSpeedLimitFromS(const double s) const {
  /**
   * 首先检查是否有自定义速度限制
   */
  for (const auto& speed_limit : speed_limit_) {
    if (s >= speed_limit.start_s && s <= speed_limit.end_s) {
      return speed_limit.speed_limit;
    }
  }

  /**
   * 从地图获取速度限制
   */
  const auto& map_path_point = GetReferencePoint(s);

  double speed_limit = FLAGS_planning_upper_speed_limit;  /**< 默认上限 */
  bool speed_limit_found = false;

  for (const auto& lane_waypoint : map_path_point.lane_waypoints()) {
    if (lane_waypoint.lane == nullptr) {
      AWARN << "lane_waypoint.lane is nullptr.";
      continue;
    }
    speed_limit_found = true;
    speed_limit =
        std::fmin(lane_waypoint.lane->lane().speed_limit(), speed_limit);
  }

  if (!speed_limit_found) {
    /**
     * 使用默认速度限制
     * 根据道路类型选择
     */
    speed_limit = FLAGS_default_city_road_speed_limit;
    hdmap::Road::Type road_type = GetRoadType(s);
    if (road_type == hdmap::Road::HIGHWAY) {
      speed_limit = FLAGS_default_highway_speed_limit;
    }
  }

  return speed_limit;
}

/**
 * @brief 添加速度限制段
 *
 * 合并重叠的速度限制区间。
 *
 * @param start_s 起始s坐标
 * @param end_s 结束s坐标
 * @param speed_limit 速度限制值
 */
void ReferenceLine::AddSpeedLimit(double start_s, double end_s,
                                  double speed_limit) {
  std::vector<SpeedLimit> new_speed_limit;

  /**
   * 处理与现有速度限制的合并
   */
  for (const auto& limit : speed_limit_) {
    if (start_s >= limit.end_s || end_s <= limit.start_s) {
      /**
       * 完全不重叠，保留原区间
       */
      new_speed_limit.emplace_back(limit);
    } else {
      /**
       * 有重叠，需要分割合并
       */
      double min_speed = std::min(limit.speed_limit, speed_limit);
      if (start_s >= limit.start_s) {
        new_speed_limit.emplace_back(limit.start_s, start_s, min_speed);
        if (end_s <= limit.end_s) {
          new_speed_limit.emplace_back(start_s, end_s, min_speed);
          new_speed_limit.emplace_back(end_s, limit.end_s, limit.speed_limit);
        } else {
          new_speed_limit.emplace_back(start_s, limit.end_s, min_speed);
        }
      } else {
        new_speed_limit.emplace_back(start_s, limit.start_s, speed_limit);
        if (end_s <= limit.end_s) {
          new_speed_limit.emplace_back(limit.start_s, end_s, min_speed);
          new_speed_limit.emplace_back(end_s, limit.end_s, limit.speed_limit);
        } else {
          new_speed_limit.emplace_back(limit.start_s, limit.end_s, min_speed);
        }
      }
      start_s = limit.end_s;
      end_s = std::max(end_s, limit.end_s);
    }
  }

  speed_limit_.clear();

  /**
   * 添加新区间
   */
  if (end_s > start_s) {
    new_speed_limit.emplace_back(start_s, end_s, speed_limit);
  }

  /**
   * 排序并去重
   */
  for (const auto& limit : new_speed_limit) {
    if (limit.start_s < limit.end_s) {
      speed_limit_.emplace_back(limit);
    }
  }

  /**
   * 按起始s坐标排序
   */
  std::sort(speed_limit_.begin(), speed_limit_.end(),
            [](const SpeedLimit& a, const SpeedLimit& b) {
              if (a.start_s != b.start_s) {
                return a.start_s < b.start_s;
              }
              if (a.end_s != b.end_s) {
                return a.end_s < b.end_s;
              }
              return a.speed_limit < b.speed_limit;
            });
}

}  // namespace planning
}  // namespace apollo
