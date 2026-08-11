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
 * @file st_boundary.cc
 * @brief ST边界（时空边界）实现文件
 *
 * 功能说明：
 * ST边界是Apollo规划模块中用于描述障碍物在时空中的占用区域
 * - S轴：沿参考线的累积距离
 * - T轴：时间
 * ST边界表示在某个时间段内，障碍物在S方向上占据的范围
 *
 * 应用场景：
 * - 速度规划：在ST图上搜索可行驶区域
 * - 碰撞检测：确保自车轨迹不与任何ST边界重叠
 * - 决策制定：根据边界类型（STOP/FOLLOW/YIELD等）做出相应决策
 */

#include "modules/planning/planning_base/common/speed/st_boundary.h"

/**
 * @brief Cyber RT日志系统头文件
 *
 * C++语法说明：
 * - cyber/common/log.h：
 *   Apollo Cyber RT框架的日志系统
 *   提供AERROR、ACHECK、ADEBUG等日志宏
 */

#include "cyber/common/log.h"

/**
 * @brief 数学工具函数头文件
 *
 * 功能说明：
 * - modules/common/math/math_utils.h：
 *   提供通用数学工具函数
 *
 * C++语法说明：
 * - std::fmin/std::fmax：
 *   标准库函数，返回两个浮点数的最小/最大值
 *   比三元运算符更能处理NaN情况
 */

#include "modules/common/math/math_utils.h"

/**
 * @brief 调试信息打印工具头文件
 *
 * 功能说明：
 * - PrintPoints类：用于将点集数据打印到日志
 * - 便于在调试时可视化ST边界
 */

#include "modules/planning/planning_base/common/util/print_debug_info.h"

/**
 * @brief 规划模块全局flags头文件
 *
 * 功能说明：
 * - planning_gflags.h：
 *   定义规划模块使用的全局配置变量
 *   如FLAGS_speed_lon_decision_horizon（速度纵向决策范围）
 */

#include "modules/planning/planning_base/gflags/planning_gflags.h"

/**
 * @brief Apollo命名空间
 *
 * C++语法说明：
 * - namespace apollo：
 *   Apollo项目的最外层命名空间
 * - namespace planning：
 *   规划模块的子命名空间
 */
namespace apollo {
namespace planning {

/**
 * @brief 类型别名声明
 *
 * 功能说明：
 * 为常用类型创建别名简化代码书写
 *
 * C++语法说明：
 * - using LineSegment2d = apollo::common::math::LineSegment2d：
 *   2D线段类，用于几何计算
 *   在ST边界中用于判断点是否接近线段
 *
 * - using Vec2d = apollo::common::math::Vec2d：
 *   2D向量类，提供几何运算
 *   如距离计算、叉积等
 */
using apollo::common::math::LineSegment2d;
using apollo::common::math::Vec2d;

/**
 * @brief ST边界构造函数
 *
 * @param point_pairs ST点对向量，每对包含(下边界点, 上边界点)
 * @param is_accurate_boundary 是否精确边界（不做点精简）
 *
 * 功能说明：
 * 从ST点对构造ST边界
 * 每个点对表示在某一时刻t，s方向的上下边界
 *
 * 算法流程：
 * 1. 验证输入点对的有效性
 * 2. 如果不是精确模式，移除冗余点
 * 3. 分离上下边界点
 * 4. 构建多边形点序列（逆时针）
 * 5. 计算边界框（min_s, max_s, min_t, max_t）
 *
 * C++语法说明：
 * - const std::vector<std::pair<STPoint, STPoint>>& point_pairs：
 *   输入参数为ST点对的常量引用
 *   每个pair的第一个元素是下边界，第二个是上边界
 *
 * - std::move(reduced_pairs)：
 *   移动语义，避免不必要的数据拷贝
 */
STBoundary::STBoundary(
    const std::vector<std::pair<STPoint, STPoint>>& point_pairs,
    bool is_accurate_boundary) {
  // 验证输入点对的有效性
  // ACHECK在debug模式下会检查条件，如果不满足则输出错误并终止
  ACHECK(IsValid(point_pairs)) << "The input point_pairs are NOT valid";

  // 创建点对副本，用于可能的精简操作
  std::vector<std::pair<STPoint, STPoint>> reduced_pairs(point_pairs);

  // 如果不是精确模式，执行点精简算法
  // 移除在同一直线上的中间点，减少计算量
  if (!is_accurate_boundary) {
    RemoveRedundantPoints(&reduced_pairs);
  }

  // 分离上下边界点
  // 遍历每个点对，分别提取上下边界点
  for (const auto& item : reduced_pairs) {
    const double t = item.first.t();  // 提取时间t（上下边界点使用相同的t）
    // 在下边界点向量的末尾构造新点(s, t)
    lower_points_.emplace_back(item.first.s(), t);
    // 在上边界点向量的末尾构造新点(s, t)
    upper_points_.emplace_back(item.second.s(), t);
  }

  // 构建多边形点序列
  // Polygon2d需要按逆时针顺序排列点
  // 首先添加下边界点（从左到右）
  for (const auto& point : lower_points_) {
    // 注意：这里交换了t和s的顺序
    // Polygon2d使用(t, s)作为坐标，但STPoint使用(s, t)
    points_.emplace_back(point.t(), point.s());
  }

  // 添加上边界点（从右到左，逆时针顺序）
  // rbegin/rend：反向迭代器，从最后一个元素到第一个
  for (auto rit = upper_points_.rbegin(); rit != upper_points_.rend(); ++rit) {
    points_.emplace_back(rit->t(), rit->s());
  }

  // 调用Polygon2d的BuildFromPoints构建多边形
  // 这是基类的初始化函数，基于points_构建2D多边形
  BuildFromPoints();

  // 计算边界的s方向范围（下边界最小s，上边界最大s）
  for (const auto& point : lower_points_) {
    // std::fmin：浮点数最小值函数
    // 比较当前min_s和新点的s，取较小值
    min_s_ = std::fmin(min_s_, point.s());
  }
  for (const auto& point : upper_points_) {
    // std::fmax：浮点数最大值函数
    // 比较当前max_s和新点的s，取较大值
    max_s_ = std::fmax(max_s_, point.s());
  }

  // 计算边界的时间范围
  // 下边界的第一点和最后一点的时间即min_t和max_t
  min_t_ = lower_points_.front().t();
  max_t_ = lower_points_.back().t();

  // 初始化障碍物道路右侧结束时间为最小值
  // 这个值用于某些特殊场景，如车辆右转时的处理
  obstacle_road_right_ending_t_ = std::numeric_limits<double>::lowest();
}

/**
 * @brief 创建ST边界的工厂方法（旧版本）
 *
 * @param lower_points 下边界点序列
 * @param upper_points 上边界点序列
 * @return STBoundary 创建的ST边界对象
 *
 * 功能说明：
 * 通过上下边界点序列创建ST边界实例
 * 这是CreateInstance的简化版本，会执行点精简
 *
 * 算法流程：
 * 1. 检查上下边界点数是否一致且至少为2
 * 2. 将点序列转换为点对格式
 * 3. 调用构造函数创建STBoundary
 *
 * C++语法说明：
 * - std::vector<STPoint>&：
 *   向量的引用类型，用于输入
 *
 * - lower_points.at(i)：
 *   at()方法会进行边界检查，比operator[]更安全
 *
 * - STPoint(lower_points.at(i).s(), lower_points.at(i).t())：
 *   显式构造STPoint，指定s和t坐标
 */
STBoundary STBoundary::CreateInstance(
    const std::vector<STPoint>& lower_points,
    const std::vector<STPoint>& upper_points) {
  // 检查边界点数组有效性
  if (lower_points.size() != upper_points.size() || lower_points.size() < 2) {
    // 返回空边界（默认构造函数）
    return STBoundary();
  }

  // 创建点对向量
  std::vector<std::pair<STPoint, STPoint>> point_pairs;
  for (size_t i = 0; i < lower_points.size(); ++i) {
    // emplace_back直接构造pair对象
    // 避免拷贝或移动操作
    point_pairs.emplace_back(
        STPoint(lower_points.at(i).s(), lower_points.at(i).t()),
        STPoint(upper_points.at(i).s(), upper_points.at(i).t()));
  }
  return STBoundary(point_pairs);
}

/**
 * @brief 创建精确ST边界的工厂方法
 *
 * @param lower_points 下边界点序列
 * @param upper_points 上边界点序列
 * @return STBoundary 创建的ST边界对象
 *
 * 功能说明：
 * 通过上下边界点序列创建ST边界实例
 * 与CreateInstance的区别是不会移除冗余点
 * 用于需要精确边界的高精度场景
 *
 * C++语法说明：
 * - STBoundary(point_pairs, true)：
 *   第二个参数true表示is_accurate_boundary
 *   告诉构造函数不做点精简
 */
STBoundary STBoundary::CreateInstanceAccurate(
    const std::vector<STPoint>& lower_points,
    const std::vector<STPoint>& upper_points) {
  // 检查边界点数组有效性
  if (lower_points.size() != upper_points.size() || lower_points.size() < 2) {
    return STBoundary();
  }

  // 创建点对向量
  std::vector<std::pair<STPoint, STPoint>> point_pairs;
  for (size_t i = 0; i < lower_points.size(); ++i) {
    point_pairs.emplace_back(
        STPoint(lower_points.at(i).s(), lower_points.at(i).t()),
        STPoint(upper_points.at(i).s(), upper_points.at(i).t()));
  }
  // 传入true表示精确模式，不做点精简
  return STBoundary(point_pairs, true);
}

/**
 * @brief 获取边界类型的字符串名称
 *
 * @param type 边界类型枚举值
 * @return std::string 类型的字符串表示
 *
 * 功能说明：
 * 将BoundaryType枚举值转换为可读字符串
 * 用于日志输出和调试显示
 *
 * C++语法说明：
 * - static_cast<int>(type)：
 *   显式类型转换，将枚举值转换为整数
 *   用于日志输出，因为枚举的operator<<可能被重载
 */
std::string STBoundary::TypeName(BoundaryType type) {
  if (type == BoundaryType::FOLLOW) {
    return "FOLLOW";
  } else if (type == BoundaryType::KEEP_CLEAR) {
    return "KEEP_CLEAR";
  } else if (type == BoundaryType::OVERTAKE) {
    return "OVERTAKE";
  } else if (type == BoundaryType::STOP) {
    return "STOP";
  } else if (type == BoundaryType::YIELD) {
    return "YIELD";
  } else if (type == BoundaryType::UNKNOWN) {
    return "UNKNOWN";
  }
  // 如果是未知类型，输出警告并返回"UNKNOWN"
  AWARN << "Unknown boundary type " << static_cast<int>(type)
        << ", treated as UNKNOWN";
  return "UNKNOWN";
}

/**
 * @brief 获取不阻塞的S范围
 *
 * @param curr_time 当前时间
 * @param s_upper 输出：上边界S值
 * @param s_lower 输出：下边界S值
 * @return bool 是否成功获取
 *
 * 功能说明：
 * 获取在当前时间不自车被阻塞的S范围
 * 这个函数用于速度规划中确定可行驶区域
 *
 * 决策逻辑：
 * - STOP/YIELD/FOLLOW边界：自车必须保持在边界下方
 *   因此s_upper设置为lower_cross_s（下边界s值）
 * - OVERTAKE边界：自车可以超过障碍物
 *   因此s_lower设置为upper_cross_s（上边界s值）
 *
 * 算法流程：
 * 1. 初始化s_upper为决策范围，s_lower为0
 * 2. 如果时间在边界外，直接返回（不阻塞）
 * 3. 在边界上查找当前时间所在的区间
 * 4. 线性插值计算该时刻的s边界值
 * 5. 根据边界类型设置s_upper或s_lower
 *
 * C++语法说明：
 * - CHECK_NOTNULL(s_upper)：
 *   断言指针非空，如果为空则程序终止
 *   不同于nullptr检查，CHECK_NOTNULL在release模式也会检查
 *
 * - FLAGS_speed_lon_decision_horizon：
 *   全局flag变量，表示速度规划的纵向决策范围
 */
bool STBoundary::GetUnblockSRange(const double curr_time, double* s_upper,
                                  double* s_lower) const {
  CHECK_NOTNULL(s_upper);  // 断言s_upper指针非空
  CHECK_NOTNULL(s_lower);  // 断言s_lower指针非空

  // 初始化为默认值：s_upper为决策范围，s_lower为0
  *s_upper = FLAGS_speed_lon_decision_horizon;
  *s_lower = 0.0;

  // 如果当前时间在边界时间范围之外，不阻塞
  if (curr_time < min_t_ || curr_time > max_t_) {
    return true;
  }

  // 在下边界点序列中查找curr_time所在的区间
  size_t left = 0;
  size_t right = 0;
  if (!GetIndexRange(lower_points_, curr_time, &left, &right)) {
    AERROR << "Fail to get index range.";
    return false;
  }

  // 如果当前时间超过上边界的起始时间，不阻塞
  if (curr_time > upper_points_[right].t()) {
    return true;
  }

  // 计算在线段[left, right]中的相对位置
  // r = 0表示在left点，r = 1表示在right点
  const double r =
      (left == right
           ? 0.0  // 如果left==right，r为0
           : (curr_time - upper_points_[left].t()) /
                 (upper_points_[right].t() - upper_points_[left].t()));

  // 线性插值计算上边界在当前时间的s值
  double upper_cross_s =
      upper_points_[left].s() +
      r * (upper_points_[right].s() - upper_points_[left].s());

  // 线性插值计算下边界在当前时间的s值
  double lower_cross_s =
      lower_points_[left].s() +
      r * (lower_points_[right].s() - lower_points_[left].s());

  // 根据边界类型设置s范围
  if (boundary_type_ == BoundaryType::STOP ||
      boundary_type_ == BoundaryType::YIELD ||
      boundary_type_ == BoundaryType::FOLLOW) {
    // 这些情况下，自车必须在边界下方行驶
    *s_upper = lower_cross_s;  // 上边界设为下边界的s值
  } else if (boundary_type_ == BoundaryType::OVERTAKE) {
    // 超车情况下，自车可以在边界上方行驶
    // std::fmax确保s_lower不会小于0
    *s_lower = std::fmax(*s_lower, upper_cross_s);
  } else {
    // 不支持的边界类型
    ADEBUG << "boundary_type is not supported. boundary_type: "
           << static_cast<int>(boundary_type_);
    return false;
  }
  return true;
}

/**
 * @brief 获取边界的S范围
 *
 * @param curr_time 当前时间
 * @param s_upper 输出：上边界S值
 * @param s_lower 输出：下边界S值
 * @return bool 是否成功获取
 *
 * 功能说明：
 * 获取在当前时间边界本身的S范围
 * 与GetUnblockSRange的区别是直接返回边界值，不做决策处理
 *
 * 算法流程：
 * 1. 检查时间是否在边界范围内
 * 2. 查找当前时间所在的点区间
 * 3. 线性插值计算边界的s值
 */
bool STBoundary::GetBoundarySRange(const double curr_time, double* s_upper,
                                   double* s_lower) const {
  CHECK_NOTNULL(s_upper);  // 断言s_upper指针非空
  CHECK_NOTNULL(s_lower);  // 断言s_lower指针非空

  // 检查时间是否在边界范围内
  if (curr_time < min_t_ || curr_time > max_t_) {
    return false;
  }

  // 在下边界点序列中查找curr_time所在的区间
  size_t left = 0;
  size_t right = 0;
  if (!GetIndexRange(lower_points_, curr_time, &left, &right)) {
    AERROR << "Fail to get index range.";
    return false;
  }

  // 计算在线段中的相对位置
  const double r =
      (left == right
           ? 0.0
           : (curr_time - upper_points_[left].t()) /
                 (upper_points_[right].t() - upper_points_[left].t()));

  // 线性插值计算上边界的s值
  *s_upper = upper_points_[left].s() +
             r * (upper_points_[right].s() - upper_points_[left].s());

  // 线性插值计算下边界的s值
  *s_lower = lower_points_[left].s() +
             r * (lower_points_[right].s() - lower_points_[left].s());

  // 限制s值在有效范围内
  // std::fmin限制上界，std::fmax限制下界
  *s_upper = std::fmin(*s_upper, FLAGS_speed_lon_decision_horizon);
  *s_lower = std::fmax(*s_lower, 0.0);
  return true;
}

/**
 * @brief 获取边界的斜率（ds/dt，即速度）
 *
 * @param curr_time 当前时间
 * @param ds_upper 输出：上边界斜率
 * @param ds_lower 输出：下边界斜率
 * @return bool 是否成功获取
 *
 * 功能说明：
 * 计算ST边界在当前时间的斜率
 * 上边界斜率表示障碍物的前进速度
 * 下边界斜率用于判断是否可以跟随
 *
 * 算法流程：
 * 使用中心差分法计算斜率：
 * 1. 获取curr_time前后的边界s值
 * 2. 分别计算前向和后向差分
 * 3. 取平均值作为最终斜率
 *
 * C++语法说明：
 * - static constexpr double kTimeIncrement = 0.05：
 *   constexpr表示编译时常量
 *   时间增量，用于差分计算
 *
 * - (next_s_upper - curr_s_upper) / kTimeIncrement：
 *   前向差分公式
 *   表示单位时间内s的变化量
 */
bool STBoundary::GetBoundarySlopes(const double curr_time, double* ds_upper,
                                   double* ds_lower) const {
  // 检查输出指针是否为空
  if (ds_upper == nullptr || ds_lower == nullptr) {
    return false;
  }

  // 检查时间是否在边界范围内
  if (curr_time < min_t_ || curr_time > max_t_) {
    return false;
  }

  // 时间增量，用于差分计算
  static constexpr double kTimeIncrement = 0.05;

  // 获取前一个时刻的边界s值
  double t_prev = curr_time - kTimeIncrement;
  double prev_s_upper = 0.0;
  double prev_s_lower = 0.0;
  bool has_prev = GetBoundarySRange(t_prev, &prev_s_upper, &prev_s_lower);

  // 获取后一个时刻的边界s值
  double t_next = curr_time + kTimeIncrement;
  double next_s_upper = 0.0;
  double next_s_lower = 0.0;
  bool has_next = GetBoundarySRange(t_next, &next_s_upper, &next_s_lower);

  // 获取当前时刻的边界s值
  double curr_s_upper = 0.0;
  double curr_s_lower = 0.0;
  GetBoundarySRange(curr_time, &curr_s_upper, &curr_s_lower);

  // 如果前后都没有有效数据，返回失败
  if (!has_prev && !has_next) {
    return false;
  }

  // 如果前后都有数据，使用中心差分
  if (has_prev && has_next) {
    // 上边界斜率：前向差分和后向差分的平均值
    *ds_upper = ((next_s_upper - curr_s_upper) / kTimeIncrement +
                 (curr_s_upper - prev_s_upper) / kTimeIncrement) *
                0.5;
    // 下边界斜率：同样使用中心差分
    *ds_lower = ((next_s_lower - curr_s_lower) / kTimeIncrement +
                 (curr_s_lower - prev_s_lower) / kTimeIncrement) *
                0.5;
    return true;
  }

  // 如果只有前向数据，使用后向差分
  if (has_prev) {
    *ds_upper = (curr_s_upper - prev_s_upper) / kTimeIncrement;
    *ds_lower = (curr_s_lower - prev_s_lower) / kTimeIncrement;
  } else {
    // 如果只有后向数据，使用前向差分
    *ds_upper = (next_s_upper - curr_s_upper) / kTimeIncrement;
    *ds_lower = (next_s_lower - curr_s_lower) / kTimeIncrement;
  }
  return true;
}

/**
 * @brief 检查ST点是否在边界内部
 *
 * @param st_point ST点
 * @return bool 如果点在边界内部返回true
 *
 * 功能说明：
 * 判断一个ST点是否落在ST边界所围成的区域内
 * 用于碰撞检测和轨迹验证
 *
 * 算法流程：
 * 使用叉积判断点是否在上下边界之间：
 * 1. 找到ST点所在的时间区间
 * 2. 计算ST点与上下边界线段的叉积
 * 3. 如果叉积符号相反，说明点在边界内部
 *
 * C++语法说明：
 * - common::math::CrossProd：
 *   叉积函数，判断点在线段的哪一侧
 *   返回值符号表示方向
 *
 * - (check_upper * check_lower < 0)：
 *   如果两个叉积符号相反，说明点在上下边界之间
 */
STBoundary::IsPointInBoundary(const STPoint& st_point) const {
  // 检查时间是否在边界范围内
  if (st_point.t() <= min_t_ || st_point.t() >= max_t_) {
    return false;
  }

  // 找到时间所在的区间
  size_t left = 0;
  size_t right = 0;
  if (!GetIndexRange(lower_points_, st_point.t(), &left, &right)) {
    AERROR << "failed to get index range.";
    return false;
  }

  // 计算ST点与上边界线段的叉积
  const double check_upper = common::math::CrossProd(
      st_point, upper_points_[left], upper_points_[right]);

  // 计算ST点与下边界线段的叉积
  const double check_lower = common::math::CrossProd(
      st_point, lower_points_[left], lower_points_[right]);

  // 如果叉积符号相反，说明点在边界内部
  return (check_upper * check_lower < 0);
}

/**
 * @brief 沿S方向扩展边界
 *
 * @param s 扩展的距离
 * @return STBoundary 扩展后的新边界
 *
 * 功能说明：
 * 沿着S方向扩展边界的上下界
 * - 下边界向上移动s距离
 * - 上边界向下移动s距离
 * 用于在规划时预留安全裕量
 *
 * 算法流程：
 * 对每个时间点，将上下边界分别向对方移动s距离
 */
STBoundary STBoundary::ExpandByS(const double s) const {
  // 如果边界为空，返回空边界
  if (lower_points_.empty()) {
    return STBoundary();
  }

  // 创建扩展后的点对向量
  std::vector<std::pair<STPoint, STPoint>> point_pairs;
  for (size_t i = 0; i < lower_points_.size(); ++i) {
    // 下边界向上移动s距离
    // 上边界向下移动s距离
    point_pairs.emplace_back(
        STPoint(lower_points_[i].s() - s, lower_points_[i].t()),
        STPoint(upper_points_[i].s() + s, upper_points_[i].t()));
  }
  return STBoundary(std::move(point_pairs));
}

/**
 * @brief 沿T方向扩展边界
 *
 * @param t 扩展的时间
 * @return STBoundary 扩展后的新边界
 *
 * 功能说明：
 * 沿着T方向（时间方向）扩展边界
 * - 在起始时间前扩展
 * - 在结束时间后扩展
 * 用于预测障碍物的运动趋势
 *
 * 算法流程：
 * 1. 在起始点前添加新点（基于起始点的速度趋势外推）
 * 2. 保留原始点
 * 3. 在结束点后添加新点（基于结束点的速度趋势外推）
 *
 * C++语法说明：
 * - point_pairs.front().first.set_s(...)：
 *   front()返回第一个元素的引用
 *   first是pair的第一个元素
 *   set_s()设置STPoint的s值
 */
STBoundary STBoundary::ExpandByT(const double t) const {
  // 检查边界是否为空
  if (lower_points_.empty()) {
    AERROR << "The current st_boundary has NO points.";
    return STBoundary();
  }

  std::vector<std::pair<STPoint, STPoint>> point_pairs;

  // 计算起始点的速度趋势（s方向的变化率）
  const double left_delta_t = lower_points_[1].t() - lower_points_[0].t();
  const double lower_left_delta_s = lower_points_[1].s() - lower_points_[0].s();
  const double upper_left_delta_s = upper_points_[1].s() - upper_points_[0].s();

  // 在起始时间前添加新点
  // 基于起始点的速度趋势向时间减小方向外推
  point_pairs.emplace_back(
      STPoint(lower_points_[0].s() - t * lower_left_delta_s / left_delta_t,
              lower_points_[0].t() - t),
      STPoint(upper_points_[0].s() - t * upper_left_delta_s / left_delta_t,
              upper_points_.front().t() - t));

  // 确保扩展后的下边界不超过上边界
  const double kMinSEpsilon = 1e-3;  // 最小s差值
  point_pairs.front().first.set_s(
      std::fmin(point_pairs.front().second.s() - kMinSEpsilon,
                point_pairs.front().first.s()));

  // 保留原始点
  for (size_t i = 0; i < lower_points_.size(); ++i) {
    point_pairs.emplace_back(lower_points_[i], upper_points_[i]);
  }

  // 计算结束点的速度趋势
  size_t length = lower_points_.size();
  DCHECK_GE(length, 2U);  // 断言长度至少为2

  const double right_delta_t =
      lower_points_[length - 1].t() - lower_points_[length - 2].t();
  const double lower_right_delta_s =
      lower_points_[length - 1].s() - lower_points_[length - 2].s();
  const double upper_right_delta_s =
      upper_points_[length - 1].s() - upper_points_[length - 2].s();

  // 在结束时间后添加新点
  // 基于结束点的速度趋势向时间增加方向外推
  point_pairs.emplace_back(STPoint(lower_points_.back().s() +
                                       t * lower_right_delta_s / right_delta_t,
                                   lower_points_.back().t() + t),
                           STPoint(upper_points_.back().s() +
                                       t * upper_right_delta_s / right_delta_t,
                                   upper_points_.back().t() + t));

  // 确保扩展后的上边界不低于下边界
  point_pairs.back().second.set_s(
      std::fmax(point_pairs.back().second.s(),
                point_pairs.back().first.s() + kMinSEpsilon));

  return STBoundary(std::move(point_pairs));
}

/**
 * @brief 获取边界类型
 */
STBoundary::BoundaryType STBoundary::boundary_type() const {
  return boundary_type_;
}

/**
 * @brief 设置边界类型
 *
 * @param boundary_type 新的边界类型
 */
void STBoundary::SetBoundaryType(const BoundaryType& boundary_type) {
  boundary_type_ = boundary_type;
}

/**
 * @brief 获取障碍物ID
 */
const std::string& STBoundary::id() const { return id_; }

/**
 * @brief 设置障碍物ID
 */
void STBoundary::set_id(const std::string& id) { id_ = id; }

/**
 * @brief 获取特征长度
 */
double STBoundary::characteristic_length() const {
  return characteristic_length_;
}

/**
 * @brief 设置特征长度
 *
 * @param characteristic_length 特征长度
 *
 * 功能说明：
 * 设置边界的特征长度
 * 特征长度用于某些决策算法中的距离计算
 */
void STBoundary::SetCharacteristicLength(const double characteristic_length) {
  characteristic_length_ = characteristic_length;
}

/**
 * @brief 获取边界的最小S值
 */
double STBoundary::min_s() const { return min_s_; }

/**
 * @brief 获取边界的最小时间
 */
double STBoundary::min_t() const { return min_t_; }

/**
 * @brief 获取边界的最大S值
 */
double STBoundary::max_s() const { return max_s_; }

/**
 * @brief 获取边界的最大时间
 */
double STBoundary::max_t() const { return max_t_; }

/**
 * @brief 按时间裁剪边界
 *
 * @param t 裁剪时间点
 * @return STBoundary 裁剪后的边界
 *
 * 功能说明：
 * 保留时间大于等于t的边界部分
 * 用于去除已经过去的时间段
 *
 * 算法流程：
 * 遍历所有点，只保留t值大于等于指定时间的点
 */
STBoundary STBoundary::CutOffByT(const double t) const {
  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;
  for (size_t i = 0; i < lower_points_.size() && i < upper_points_.size();
       ++i) {
    // 跳过时间小于t的点
    if (lower_points_[i].t() < t) {
      continue;
    }
    lower_points.push_back(lower_points_[i]);
    upper_points.push_back(upper_points_[i]);
  }
  return CreateInstance(lower_points, upper_points);
}

/**
 * @brief 获取边界框的四个角点
 *
 * 功能说明：
 * ST边界被当作2D多边形处理
 * 这四个函数返回多边形的四个角点
 *
 * C++语法说明：
 * - upper_points_.front()：
 *   front()返回第一个元素的引用
 *   对应左边界（t值最小）
 *
 * - upper_points_.back()：
 *   back()返回最后一个元素的引用
 *   对应右边界（t值最大）
 */
STPoint STBoundary::upper_left_point() const {
  DCHECK(!upper_points_.empty()) << "StBoundary has zero points.";
  return upper_points_.front();
}

STPoint STBoundary::upper_right_point() const {
  DCHECK(!upper_points_.empty()) << "StBoundary has zero points.";
  return upper_points_.back();
}

STPoint STBoundary::bottom_left_point() const {
  DCHECK(!lower_points_.empty()) << "StBoundary has zero points.";
  return lower_points_.front();
}

STPoint STBoundary::bottom_right_point() const {
  DCHECK(!lower_points_.empty()) << "StBoundary has zero points.";
  return lower_points_.back();
}

/**
 * @brief 设置边界框的四个角点
 */
void STBoundary::set_upper_left_point(STPoint st_point) {
  upper_left_point_ = std::move(st_point);
}

void STBoundary::set_upper_right_point(STPoint st_point) {
  upper_right_point_ = std::move(st_point);
}

void STBoundary::set_bottom_left_point(STPoint st_point) {
  bottom_left_point_ = std::move(st_point);
}

void STBoundary::set_bottom_right_point(STPoint st_point) {
  bottom_right_point_ = std::move(st_point);
}

///////////////////////////////////////////////////////////////////////////////
// 私有成员函数

/**
 * @brief 验证ST点对的有效性
 *
 * @param point_pairs 要验证的点对
 * @return bool 如果有效返回true
 *
 * 功能说明：
 * 检查ST点对是否满足以下条件：
 * 1. 至少有两个点对
 * 2. 每对中上边界的s >= 下边界的s
 * 3. 每对中两个点的时间t必须相同
 * 4. 时间必须严格递增
 *
 * C++语法说明：
 * - std::fabs(...) > kStBoundaryEpsilon：
 *   fabs计算浮点数绝对值
 *   用于判断两个浮点数是否相等（考虑误差）
 */
bool STBoundary::IsValid(
    const std::vector<std::pair<STPoint, STPoint>>& point_pairs) const {
  // 检查点对数量
  if (point_pairs.size() < 2) {
    AERROR << "point_pairs.size() must >= 2, but current point_pairs.size() = "
           << point_pairs.size();
    return false;
  }

  // 定义容差常量
  static constexpr double kStBoundaryEpsilon = 1e-9;  // 边界容差
  static constexpr double kMinDeltaT = 1e-6;           // 最小时间增量

  // 遍历所有点对进行验证
  for (size_t i = 0; i < point_pairs.size(); ++i) {
    const auto& curr_lower = point_pairs[i].first;   // 当前下边界点
    const auto& curr_upper = point_pairs[i].second;   // 当前上边界点

    // 检查上边界s是否大于等于下边界s
    if (curr_upper.s() < curr_lower.s()) {
      AERROR << "ST-boundary's upper-s must >= lower-s";
      return false;
    }

    // 检查上下边界点的时间是否相同
    if (std::fabs(curr_lower.t() - curr_upper.t()) > kStBoundaryEpsilon) {
      AERROR << "Points in every ST-point pair should be at the same time.";
      return false;
    }

    // 如果不是最后一个点对，检查时间是否递增
    if (i + 1 != point_pairs.size()) {
      const auto& next_lower = point_pairs[i + 1].first;
      const auto& next_upper = point_pairs[i + 1].second;

      // 检查时间是否严格递增
      // 使用fmax/fmin处理可能的浮点数误差
      if (std::fmax(curr_lower.t(), curr_upper.t()) + kMinDeltaT >=
          std::fmin(next_lower.t(), next_upper.t())) {
        AERROR << "Latter points should have larger t: "
               << "curr_lower[" << curr_lower.DebugString() << "] curr_upper["
               << curr_upper.DebugString() << "] next_lower["
               << next_lower.DebugString() << "] next_upper["
               << next_upper.DebugString() << "]";
        return false;
      }
    }
  }
  return true;
}

/**
 * @brief 检查点是否接近线段
 *
 * @param seg 线段
 * @param point 点
 * @param max_dist 最大距离
 * @return bool 如果点在线段max_dist范围内返回true
 *
 * C++语法说明：
 * - seg.DistanceSquareTo(point)：
 *   LineSegment2d的方法，计算点到线段的距离平方
 *   使用距离平方避免开方运算，提高效率
 */
bool STBoundary::IsPointNear(const common::math::LineSegment2d& seg,
                             const Vec2d& point, const double max_dist) {
  return seg.DistanceSquareTo(point) < max_dist * max_dist;
}

/**
 * @brief 移除冗余点
 *
 * @param point_pairs 输入/输出的点对向量
 *
 * 功能说明：
 * 当上下边界点都在近似直线上时，移除中间的点
 * 只保留端点，减少计算量
 *
 * 算法流程（Douglas-Peucker算法的简化版本）：
 * 1. 从第一个点开始
 * 2. 尝试跳过中间点，直接连接更远的点
 * 3. 如果中间点离这条直线太远，则保留
 * 4. 重复直到处理完所有点
 *
 * C++语法说明：
 * - while (i < point_pairs->size() && j + 1 < point_pairs->size())：
 *   双指针算法，i和j用于遍历
 *
 * - LineSegment2d(...)：
 *   直接用两个点构造线段
 */
void STBoundary::RemoveRedundantPoints(
    std::vector<std::pair<STPoint, STPoint>>* point_pairs) {
  // 如果点对数量小于等于2，无需精简
  if (!point_pairs || point_pairs->size() <= 2) {
    return;
  }

  const double kMaxDist = 0.1;  // 最大允许距离
  size_t i = 0;
  size_t j = 1;

  // 双指针遍历
  while (i < point_pairs->size() && j + 1 < point_pairs->size()) {
    // 创建从点i到点j+1的线段
    LineSegment2d lower_seg(point_pairs->at(i).first,
                            point_pairs->at(j + 1).first);
    LineSegment2d upper_seg(point_pairs->at(i).second,
                            point_pairs->at(j + 1).second);

    // 检查中间点j是否接近这条线段
    // 如果不接近，说明需要保留中间点
    if (!IsPointNear(lower_seg, point_pairs->at(j).first, kMaxDist) ||
        !IsPointNear(upper_seg, point_pairs->at(j).second, kMaxDist)) {
      ++i;  // 移动起始点
      if (i != j) {
        point_pairs->at(i) = point_pairs->at(j);  // 保留点j
      }
    }
    ++j;  // 移动结束点
  }

  // 添加最后一个点并调整向量大小
  point_pairs->at(++i) = point_pairs->back();
  point_pairs->resize(i + 1);
}

/**
 * @brief 获取时间t所在的点索引区间
 *
 * @param points ST点序列
 * @param t 时间值
 * @param left 输出：左边界索引
 * @param right 输出：右边界索引
 * @return bool 是否成功找到区间
 *
 * 功能说明：
 * 在有序的ST点序列中，找到时间t所在的区间[left, right]
 * - left：第一个时间大于t的点的索引减1
 * - right：第一个时间大于等于t的点的索引
 *
 * 算法流程：
 * 使用二分查找（std::lower_bound）找到第一个时间大于等于t的点
 *
 * C++语法说明：
 * - auto comp = [](const STPoint& p, const double t) { return p.t() < t; }：
 *   Lambda表达式，用于自定义比较函数
 *   std::lower_bound需要用这个来判断"小于"
 *
 * - std::lower_bound(points.begin(), points.end(), t, comp)：
 *   二分查找，找到第一个满足comp(point, t) == false的元素
 *   即第一个时间不小于t的点的迭代器
 *
 * - std::distance(points.begin(), first_ge)：
 *   计算两个迭代器之间的距离
 */
bool STBoundary::GetIndexRange(const std::vector<STPoint>& points,
                               const double t, size_t* left,
                               size_t* right) const {
  CHECK_NOTNULL(left);   // 断言left指针非空
  CHECK_NOTNULL(right);  // 断言right指针非空

  // 检查t是否在点序列的时间范围内
  if (t < points.front().t() || t > points.back().t()) {
    AERROR << "t is out of range. t = " << t;
    return false;
  }

  // Lambda表达式：比较STPoint的时间与t的大小
  auto comp = [](const STPoint& p, const double t) { return p.t() < t; };

  // 使用lower_bound进行二分查找
  // 找到第一个时间大于等于t的点的迭代器
  auto first_ge = std::lower_bound(points.begin(), points.end(), t, comp);

  // 计算索引
  size_t index = std::distance(points.begin(), first_ge);

  // 设置左右边界索引
  if (index == 0) {
    // t小于等于第一个点的时间
    *left = *right = 0;
  } else if (first_ge == points.end()) {
    // t大于所有点的时间
    *left = *right = points.size() - 1;
  } else {
    // t在中间位置
    *left = index - 1;   // 前一个点
    *right = index;       // 当前点
  }
  return true;
}

/**
 * @brief 打印调试信息
 *
 * @param suffix 后缀字符串
 *
 * 功能说明：
 * 将ST边界数据打印到日志
 * 用于调试和可视化分析
 *
 * 算法流程：
 * 1. 生成调试键名（类型+ID+后缀）
 * 2. 添加下边界点（从左到右）
 * 3. 添加上边界点（从右到左，形成闭合多边形）
 * 4. 打印到日志
 *
 * C++语法说明：
 * - std::string type_name = TypeName(boundary_type_)：
 *   调用TypeName获取类型的字符串表示
 *
 * - for (auto iter = upper_points_.rbegin(); iter != upper_points_.rend(); iter++)：
 *   rbegin/rend：反向迭代器
 *   从最后一个元素遍历到第一个
 *
 * - iter->t() / iter->s()：
 *   迭代器的箭头运算符访问成员
 */
void STBoundary::PrintDebug(std::string suffix) const {
  std::string type_name = TypeName(boundary_type_);
  std::string key_name = type_name + id_ + suffix;
  PrintPoints debug(key_name);

  // 检查边界是否为空
  if (lower_points_.empty() || upper_points_.empty()) {
    return;
  }

  // 添加下边界点
  for (auto pt : lower_points_) {
    debug.AddPoint(pt.t(), pt.s());
  }

  // 添加上边界点（反向遍历）
  for (auto iter = upper_points_.rbegin(); iter != upper_points_.rend();
       iter++) {
    debug.AddPoint(iter->t(), iter->s());
  }

  // 添加第一个下边界点闭合多边形
  debug.AddPoint(lower_points_.front().t(), lower_points_.front().s());
  debug.PrintToLog();
}

}  // namespace planning
}  // namespace apollo