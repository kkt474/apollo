/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
 * @file path_matcher.cc
 * @brief 路径匹配器实现文件
 *
 * 功能说明：
 * 路径匹配器用于将笛卡尔坐标系中的点匹配到参考路径上
 * 主要功能：
 * 1. MatchToPath：基于(x,y)坐标找到路径上最近的点
 * 2. GetPathFrenetCoordinate：计算点在Frenet坐标系中的坐标
 * 3. FindProjectionPoint：计算点到线段的投影点
 *
 * 算法原理：
 * - MatchToPath使用暴力搜索找到距离最近的路径点
 * - 使用线性插值获得更精确的匹配位置
 * - GetPathFrenetCoordinate计算相对坐标和侧向偏移
 *
 * C++语法说明：
 * - namespace嵌套：apollo::common::math三层命名空间
 * - lambda表达式：匿名函数对象用于距离计算和比较
 * - std::pair：标准库键值对容器
 * - std::lower_bound：二分查找下界
 * - std::hypot：欧几里得距离函数
 * - std::copysign：符号复制函数
 */

#include "modules/common/math/path_matcher.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "glog/logging.h"

#include "modules/common/math/linear_interpolation.h"

namespace apollo {
/**
 * @brief Apollo外层命名空间
 */
namespace common {
/**
 * @brief common模块命名空间
 */
namespace math {
/**
 * @brief math工具函数命名空间
 */

/**
 * @brief 将(x,y)点匹配到参考路径上
 *
 * @param reference_line 参考路径点序列
 * @param x 目标点的X坐标
 * @param y 目标点的Y坐标
 * @return PathPoint 匹配到参考路径上的点
 *
 * 功能说明：
 * 在参考路径上找到距离给定(x,y)点最近的点
 * 如果最近的两个点不相同，还会在它们之间进行线性插值
 *
 * 算法流程：
 * 1. 定义距离平方函数（lambda表达式）
 * 2. 找到距离最小的点索引
 * 3. 确定插值区间[index_start, index_end]
 * 4. 调用FindProjectionPoint计算投影点
 *
 * C++语法说明：
 * - CHECK_GT(reference_line.size(), 0U)：
 *   glog的CHECK宏，检查条件是否满足
 *   如果不满足，程序终止并输出错误
 *   GT表示Greater Than，0U是无符号整数0
 *
 * - auto func_distance_square = [](const PathPoint& point, const double x,
 *                                   const double y) { ... }：
 *   lambda表达式定义匿名函数
 *   [capture](params) { body }
 *   - []空捕获列表：不捕获外部变量
 *   - const PathPoint& point：常量引用参数
 *   - 返回dx*dx + dy*dy（距离平方）
 *
 * - reference_line.front() / reference_line.back()：
 *   获取容器的第一个/最后一个元素
 *
 * - std::size_t：
 *   无符号整数类型，用于表示大小和索引
 *   size()返回的就是这个类型
 *
 * - for (std::size_t i = 1; ...)：
 *   从索引1开始遍历，避免与front重复比较
 *
 * - std::pair<double, double>：
 *   存储两个double值的容器
 *   first存储s坐标，second存储d坐标（侧向偏移）
 *
 * - ternary operator ? :：
 *   条件运算符，简化if-else
 *   (condition) ? value_if_true : value_if_false
 */
PathPoint PathMatcher::MatchToPath(const std::vector<PathPoint>& reference_line,
                                   const double x, const double y) {
  /**
   * @brief 检查参考线是否为空
   *
   * C++语法说明：
   * CHECK_GT是glog宏，要求第一个参数大于第二个
   * reference_line.size()返回std::size_t类型的元素个数
   * 0U是unsigned int类型的0
   */
  CHECK_GT(reference_line.size(), 0U);

  /**
   * @brief 定义距离平方计算函数
   *
   * C++语法说明：
   * lambda表达式创建匿名函数对象
   * - []空捕获列表：不需要捕获外部变量
   * - (const PathPoint& point, const double x, const double y)：
   *   参数列表，两个const引用
   * - { return dx * dx + dy * dy; }：
   *   函数体，计算欧几里得距离的平方
   *   避免开方运算，提高效率
   */
  auto func_distance_square = [](const PathPoint& point, const double x,
                                 const double y) {
    double dx = point.x() - x;
    double dy = point.y() - y;
    return dx * dx + dy * dy;
  };

  /**
   * @brief 初始化最小距离和对应索引
   *
   * C++语法说明：
   * - reference_line.front()：
   *   返回第一个元素的引用
   *   获取路径起始点
   *
   * - std::size_t index_min = 0：
   *   最小距离对应索引，初始化为0
   */
  double distance_min = func_distance_square(reference_line.front(), x, y);
  std::size_t index_min = 0;

  /**
   * @brief 遍历参考线找到距离最小的点
   *
   * C++语法说明：
   * - for循环从i=1开始
   *   因为front已经作为初始值，不需要再比较
   *
   * - ++i：前置递增运算符
   *   先递增再使用i的值
   *
   * - if (distance_temp < distance_min)：
   *   更新最小距离和对应索引
   */
  for (std::size_t i = 1; i < reference_line.size(); ++i) {
    double distance_temp = func_distance_square(reference_line[i], x, y);
    if (distance_temp < distance_min) {
      distance_min = distance_temp;
      index_min = i;
    }
  }

  /**
   * @brief 确定插值区间
   *
   * 功能说明：
   * 插值区间是[max(0, index_min-1), min(size-1, index_min+1)]
   * 确保区间有效且包含最近点
   *
   * C++语法说明：
   * - (index_min == 0) ? index_min : index_min - 1：
   *   三元运算符
   *   如果index_min为0，index_start也为0
   *   否则为index_min - 1
   *
   * - (index_min + 1 == reference_line.size()) ? index_min : index_min + 1：
   *   如果index_min+1超出范围，index_end等于index_min
   *   否则为index_min + 1
   *
   * - ==：比较运算符
   * - reference_line.size()：返回元素个数
   */
  std::size_t index_start = (index_min == 0) ? index_min : index_min - 1;
  std::size_t index_end =
      (index_min + 1 == reference_line.size()) ? index_min : index_min + 1;

  /**
   * @brief 如果区间只有一个点，直接返回
   *
   * C++语法说明：
   * 当reference_line只有一个元素时
   * index_start == index_end
   */
  if (index_start == index_end) {
    return reference_line[index_start];
  }

  /**
   * @brief 计算投影点
   *
   * 调用FindProjectionPoint在两个最近点之间进行插值
   * 获得更精确的匹配位置
   */
  return FindProjectionPoint(reference_line[index_start],
                             reference_line[index_end], x, y);
}

/**
 * @brief 获取路径Frenet坐标系坐标
 *
 * @param reference_line 参考路径点序列
 * @param x 目标点的X坐标
 * @param y 目标点的Y坐标
 * @return std::pair<double, double> (s, d) Frenet坐标
 *   - first: 沿路径的累计距离s
 *   - second: 垂直于路径的侧向偏移d
 *
 * 功能说明：
 * 将笛卡尔坐标(x,y)转换为Frenet坐标系(s,d)
 * - s: 沿参考线累计距离
 * - d: 垂直于参考线的偏移，左正右负
 *
 * 算法流程：
 * 1. 先匹配到参考线上获得matched_point
 * 2. 计算与matched_point的差值delta_x, delta_y
 * 3. 计算侧向偏移side（利用叉积判断左右）
 * 4. 返回(s, d)坐标对
 *
 * C++语法说明：
 * - auto matched_path_point = MatchToPath(...)：
 *   auto自动类型推导
 *   匹配到参考线上的点
 *
 * - std::pair<double, double>：
 *   标准库模板类，存储两个相关值
 *   .first访问第一个元素，.second访问第二个
 *
 * - std::cos(rtheta) * delta_y - std::sin(rtheta) * delta_x：
 *   计算叉积判断点在线的哪一侧
 *   这等价于计算(dx, dy)与(cos, sin)的2D叉积
 *
 * - std::hypot(delta_x, delta_y)：
 *   C++11函数，计算sqrt(x*x + y*y)
 *   比直接计算更数值稳定
 *
 * - std::copysign(hypot, side)：
 *   将hypot的绝对值，符号由side决定
 *   side > 0返回正值，side < 0返回负值
 */
std::pair<double, double> PathMatcher::GetPathFrenetCoordinate(
    const std::vector<PathPoint>& reference_line, const double x,
    const double y) {
  /**
   * @brief 匹配点到参考线上
   */
  auto matched_path_point = MatchToPath(reference_line, x, y);

  /**
   * @brief 获取匹配点的参数
   */
  double rtheta = matched_path_point.theta();  // 匹配点朝向角
  double rx = matched_path_point.x();          // 匹配点x坐标
  double ry = matched_path_point.y();          // 匹配点y坐标

  /**
   * @brief 计算到匹配点的差值
   */
  double delta_x = x - rx;  // x方向差值
  double delta_y = y - ry;  // y方向差值

  /**
   * @brief 计算侧向偏移判断值
   *
   * 通过叉积判断点在线的左侧还是右侧
   * - 结果 > 0：点在路径左侧
   * - 结果 < 0：点在路径右侧
   */
  double side = std::cos(rtheta) * delta_y - std::sin(rtheta) * delta_x;

  /**
   * @brief 构建Frenet坐标对
   */
  std::pair<double, double> relative_coordinate;

  /**
   * @brief 设置s坐标（沿路径累计距离）
   */
  relative_coordinate.first = matched_path_point.s();

  /**
   * @brief 设置d坐标（侧向偏移，带符号）
   *
   * std::copysign确保：
   * - side > 0时，d为正（左侧）
   * - side < 0时，d为负（右侧）
   * std::hypot计算欧几里得距离
   */
  relative_coordinate.second =
      std::copysign(std::hypot(delta_x, delta_y), side);

  return relative_coordinate;
}

/**
 * @brief 根据s坐标匹配到参考路径
 *
 * @param reference_line 参考路径点序列
 * @param s 目标s坐标
 * @return PathPoint 匹配到的路径点
 *
 * 功能说明：
 * 根据给定的累计距离s，在参考路径上找到对应位置的点
 * 如果s在路径范围内，进行线性插值
 *
 * 算法流程：
 * 1. 使用std::lower_bound二分查找第一个s >= 目标值的点
 * 2. 根据位置分为三种情况处理：
 *    - it_lower == begin：返回第一个点
 *    - it_lower == end：返回最后一个点
 *    - 其他：插值计算
 *
 * C++语法说明：
 * - auto comp = [](const PathPoint& point, const double s) { ... }：
 *   lambda比较函数
 *   用于std::lower_bound自定义比较规则
 *
 * - std::lower_bound(begin, end, value, comp)：
 *   二分查找第一个不小于value的元素
 *   返回迭代器
 *   要求范围内元素已按comp排序
 *
 * - it_lower == reference_line.begin()：
 *   比较迭代器是否到达边界
 *   使用==或!=运算符
 *
 * - *(it_lower - 1)：
 *   解引用迭代器获取元素
 *   it_lower - 1是前一个元素的迭代器
 */
PathPoint PathMatcher::MatchToPath(const std::vector<PathPoint>& reference_line,
                                   const double s) {
  /**
   * @brief 定义比较函数
   *
   * 功能说明：
   * 比较PathPoint的s值与目标s
   * 用于二分查找
   *
   * C++语法说明：
   * - lambda表达式：[](const PathPoint& point, const double s)
   * - return point.s() < s：point.s() < s时返回true
   */
  auto comp = [](const PathPoint& point, const double s) {
    return point.s() < s;
  };

  /**
   * @brief 二分查找下界
   *
   * C++语法说明：
   * - std::lower_bound：
   *   在[begin, end)范围内查找第一个不小于s的元素
   *   comp定义了"小于"的关系
   *
   * - reference_line.begin() / reference_line.end()：
   *   容器的起始和结束迭代器
   */
  auto it_lower =
      std::lower_bound(reference_line.begin(), reference_line.end(), s, comp);

  /**
   * @brief 处理边界情况
   *
   * - it_lower == begin：s小于所有点的s，返回第一个点
   * - it_lower == end：s大于所有点的s，返回最后一个点
   */
  if (it_lower == reference_line.begin()) {
    return reference_line.front();
  } else if (it_lower == reference_line.end()) {
    return reference_line.back();
  }

  /**
   * @brief 插值计算
   *
   * 在it_lower-1和it_lower之间进行线性插值
   * 使用InterpolateUsingLinearApproximation函数
   *
   * C++语法说明：
   * - *(it_lower - 1)：获取前一个元素
   * - *it_lower：获取当前元素
   * - p0.s() + delta_s：计算目标s在区间中的位置
   */
  // interpolate between it_lower - 1 and it_lower
  // return interpolate(*(it_lower - 1), *it_lower, s);
  return InterpolateUsingLinearApproximation(*(it_lower - 1), *it_lower, s);
}

/**
 * @brief 找到点到线段的投影点
 *
 * @param p0 线段起点
 * @param p1 线段终点
 * @param x 目标点X坐标
 * @param y 目标点Y坐标
 * @return PathPoint 投影点
 *
 * 功能说明：
 * 计算点(x,y)到线段(p0,p1)的投影点
 * 返回投影在线段上的位置（通过插值得到）
 *
 * 算法流程：
 * 1. 计算v0 = (x,y) - p0
 * 2. 计算v1 = p1 - p0
 * 3. 计算v1的模长v1_norm
 * 4. 计算点积dot = v0 · v1
 * 5. 计算delta_s = dot / v1_norm（投影在v1上的距离）
 * 6. 在p0和p1之间插值得到投影点
 *
 * C++语法说明：
 * - double v0x = x - p0.x()：
 *   计算向量分量
 *   p0.x()调用PathPoint的x()方法
 *
 * - std::sqrt(v1x * v1x + v1y * v1y)：
 *   计算向量v1的欧几里得模长
 *   也可以使用std::hypot(v1x, v1y)
 *
 * - v0x * v1x + v0y * v1y：
 *   向量点积（内积）
 *   几何意义：v0在v1上的投影长度 × |v1|
 *
 * - delta_s = dot / v1_norm：
 *   投影距离 = 点积 / 模长
 *   这就是从p0沿p0->p1方向应该前进的距离
 */
PathPoint PathMatcher::FindProjectionPoint(const PathPoint& p0,
                                           const PathPoint& p1, const double x,
                                           const double y) {
  /**
   * @brief 计算向量v0 = (x,y) - p0
   */
  double v0x = x - p0.x();
  double v0y = y - p0.y();

  /**
   * @brief 计算向量v1 = p1 - p0
   */
  double v1x = p1.x() - p0.x();
  double v1y = p1.y() - p0.y();

  /**
   * @brief 计算v1的模长
   *
   * C++语法说明：
   * std::sqrt：平方根函数
   * 也可以使用std::hypot(v1x, v1y)
   */
  double v1_norm = std::sqrt(v1x * v1x + v1y * v1y);

  /**
   * @brief 计算点积v0 · v1
   */
  double dot = v0x * v1x + v0y * v1y;

  /**
   * @brief 计算投影距离
   *
   * dot / v1_norm得到v0在v1上的有符号投影距离
   */
  double delta_s = dot / v1_norm;

  /**
   * @brief 插值得到投影点
   *
   * p0.s() + delta_s是在p0到p1路径上的累计距离
   * InterpolateUsingLinearApproximation进行线性插值
   */
  return InterpolateUsingLinearApproximation(p0, p1, p0.s() + delta_s);
}

/**
 * @brief 命名空间结束标记
 *
 * C++语法说明：
 * 三层命名空间的闭合注释
 * }  // namespace math
 * }  // namespace common
 * }  // namespace apollo
 */
}  // namespace math
}  // namespace common
}  // namespace apollo
