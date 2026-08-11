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
 * @file discretized_trajectory.cc
 * @brief 离散轨迹实现文件
 *
 * 本文件实现DiscretizedTrajectory类，它是Apollo规划模块中轨迹数据的核心表示。
 * 轨迹由一系列离散的TrajectoryPoint组成，支持时间查询、空间查询、插值等操作。
 *
 * C++语法说明：
 * - std::vector<T>: C++动态数组模板类，继承自STL容器
 * - std::lower_bound(): 二分查找，返回第一个不小于给定值的迭代器
 * - std::numeric_limits<T>::max(): 类型T能表示的最大值
 * - lambda表达式: [](params){body} 匿名函数对象
 * - CHECK_GT/CHECK_LT: 断言宏，检查条件是否满足
 * - ACHECK: Apollo断言宏，运行时检查条件
 * - ACHECK_NE/GE/LE: 断言不等于/大于等于/小于等于
 */
#include "modules/planning/planning_base/common/trajectory/discretized_trajectory.h"

#include <limits>                  /**< C++数值极限库，包含std::numeric_limits等 */

#include "cyber/common/log.h"     /**< Cyber RT日志系统 */
#include "modules/common/math/linear_interpolation.h" /**< 线性插值数学库 */
#include "modules/planning/planning_base/common/planning_context.h" /**< 规划上下文 */

namespace apollo {
/**
 * apollo:: - Apollo最外层命名空间
 * 所有Apollo相关代码都位于此命名空间下
 */
namespace planning {

/**
 * using声明 - 将其他命名空间的类型引入当前作用域
 * 语法：using 命名空间::类型名;
 */
using apollo::common::TrajectoryPoint;  /**< 轨迹点类型 */

/**
 * @brief 离散轨迹构造函数（从轨迹点向量构造）
 *
 * 使用轨迹点向量初始化离散轨迹。
 * 注意：轨迹点向量不能为空，否则程序会终止。
 *
 * @param trajectory_points 轨迹点向量（包含位置、速度、加速度等信息）
 *
 * 语法说明：
 * - : std::vector<T>(trajectory_points) - 初始化列表语法，调用基类构造函数
 *   这里DiscretizedTrajectory继承自std::vector<TrajectoryPoint>
 * - ACHECK(): Apollo断言宏，类似assert但输出更详细的错误信息
 * - << 操作符: 用于连接错误消息字符串
 * - !trajectory_points.empty(): 检查向量是否为空
 *   - empty()是std::vector的方法，返回容器是否为空
 * - "trajectory_points should NOT be empty()": 断言失败时输出的错误消息
 */
DiscretizedTrajectory::DiscretizedTrajectory(
    const std::vector<TrajectoryPoint>& trajectory_points)
    : std::vector<TrajectoryPoint>(trajectory_points) {
  /**
   * ACHECK - Apollo断言宏
   * ACHECK(condition) << "error message"
   * 如果condition为false，程序会输出错误消息并终止
   */
  ACHECK(!trajectory_points.empty())
      << "trajectory_points should NOT be empty()";
}

/**
 * @brief 离散轨迹构造函数（从ADCTrajectory构造）
 *
 * 从Apollo的ADCTrajectory消息类型构造离散轨迹。
 * ADCTrajectory是规划模块输出的protobuf消息格式。
 *
 * @param trajectory ADCTrajectory消息，包含trajectory_point()字段
 *
 * 语法说明：
 * - assign(begin, end): std::vector的方法，用[begin, end)范围内的元素替换容器内容
 * - trajectory.trajectory_point().begin/end(): 
 *   - trajectory_point()返回repeated字段（类似动态数组）
 *   - begin()/end()返回迭代器
 * - this调用基类assign方法，将protobuf消息中的轨迹点拷贝到当前容器
 */
DiscretizedTrajectory::DiscretizedTrajectory(const ADCTrajectory& trajectory) {
  /**
   * assign - 赋值方法
   * 将trajectory中的所有轨迹点拷贝到当前向量
   */
  assign(trajectory.trajectory_point().begin(),
         trajectory.trajectory_point().end());
}

/**
 * @brief 根据相对时间评估轨迹点
 *
 * 使用线性插值计算指定相对时间对应的轨迹点。
 * 这是轨迹时间查询的核心方法，用于轨迹拼接和轨迹跟踪。
 *
 * @param relative_time 相对时间（秒），相对于轨迹起始点
 * @return TrajectoryPoint 插值计算得到的轨迹点
 *
 * 语法说明：
 * - auto: C++11关键字，编译器自动推断变量类型
 * - lambda表达式: [](const TrajectoryPoint& p, const double relative_time){...}
 *   - 用于std::lower_bound的自定义比较函数
 *   - 返回p.relative_time() < relative_time
 * - std::lower_bound(begin, end, value, comp):
 *   - 二分查找算法，在有序范围内查找第一个不小于value的元素
 *   - comp是自定义比较函数对象
 *   - 返回满足!comp(element, value)的第一个元素迭代器
 * - common::math::InterpolateUsingLinearApproximation():
 *   - 线性近似插值，在两个已知点之间线性插值
 *   - 参数：前一点，后一点，目标时间
 * - *(it_lower - 1): 解引用迭代器前一个位置的值
 * - front()/back(): std::vector方法，返回首/尾元素引用
 */
TrajectoryPoint DiscretizedTrajectory::Evaluate(
    const double relative_time) const {
  /**
   * 定义比较函数对象
   * comp(p, relative_time) 返回 p.relative_time() < relative_time
   * 这是一个lambda表达式，作为函数对象传递给std::lower_bound
   */
  auto comp = [](const TrajectoryPoint& p, const double relative_time) {
    return p.relative_time() < relative_time;
  };

  /**
   * std::lower_bound - 二分查找
   * 在[begin(), end())范围内查找第一个relative_time >= relative_time的元素
   * 使用comp作为比较函数
   */
  auto it_lower = std::lower_bound(begin(), end(), relative_time, comp);

  /**
   * 边界情况1：it_lower == begin()
   * 说明relative_time小于第一个点的时间，返回第一个点
   */
  if (it_lower == begin()) {
    return front();  /**< front()返回首元素引用 */
  } 
  /**
   * 边界情况2：it_lower == end()
   * 说明relative_time大于所有点的时间，返回最后一个点
   */
  else if (it_lower == end()) {
    AWARN << "When evaluate trajectory, relative_time(" << relative_time
          << ") is too large";
    return back();  /**< back()返回尾元素引用 */
  }

  /**
   * 正常情况：进行线性插值
   * 在(it_lower-1)和it_lower两个点之间插值
   * InterpolateUsingLinearApproculation: 线性近似插值函数
   * 返回新创建的TrajectoryPoint（按值返回）
   */
  return common::math::InterpolateUsingLinearApproximation(
      *(it_lower - 1), *it_lower, relative_time);
}

/**
 * @brief 查询下界时间点索引
 *
 * 使用二分查找找到第一个relative_time >= given_relative_time的点。
 * 用于轨迹时间对齐和拼接。
 *
 * @param relative_time 目标相对时间
 * @param epsilon 时间缓冲区（秒），允许的小误差范围
 * @return size_t 满足条件的点的索引
 *
 * 语法说明：
 * - size_t: 无符号整数类型，用于表示大小和索引
 * - const double epsilon: 常量引用参数，函数内只读
 * - std::numeric_limits<double>::max(): double类型的最大值
 *   - 用于初始化最小距离平方
 * - std::lower_bound()与自定义函数对象
 * - std::distance(begin, it): 返回两个迭代器之间的距离（元素个数）
 * - CHECK_GT(a, b): 断言宏，a > b，否则终止程序
 */
size_t DiscretizedTrajectory::QueryLowerBoundPoint(const double relative_time,
                                                   const double epsilon) const {
  /**
   * ACHECK - 断言检查轨迹非空
   * empty()是std::vector的方法，检查容器是否为空
   */
  ACHECK(!empty());

  /**
   * 快速路径：如果relative_time >= 最后一个点的时间
   * 直接返回最后一个点的索引(size-1)
   * back().relative_time() - 获取尾元素的相对时间
   */
  if (relative_time >= back().relative_time()) {
    return size() - 1;  /**< size()返回元素个数，索引从0开始 */
  }

  /**
   * 自定义比较函数
   * lambda表达式: 检查tp.relative_time() + epsilon < relative_time
   * 即 tp.relative_time() < relative_time - epsilon
   * epsilon作为缓冲区，允许小的误差
   */
  auto func = [&epsilon](const TrajectoryPoint& tp,
                         const double relative_time) {
    return tp.relative_time() + epsilon < relative_time;
  };

  /**
   * std::lower_bound - 二分查找
   * 查找第一个满足 func(point, relative_time) == false 的点
   * 即第一个 relative_time >= tp.relative_time() - epsilon 的点
   */
  auto it_lower = std::lower_bound(begin(), end(), relative_time, func);

  /**
   * std::distance - 计算迭代器之间的距离
   * 返回从begin()到it_lower之间的元素个数
   * 这就是我们要找的索引
   */
  return std::distance(begin(), it_lower);
}

/**
 * @brief 查询距离最近的轨迹点索引
 *
 * 遍历所有轨迹点，找到与给定位置欧氏距离最近的点。
 * 使用距离平方进行比较，避免开方运算提高效率。
 *
 * @param position 二维位置向量（Vec2d类型）
 * @return size_t 最近点的索引
 *
 * 语法说明：
 * - const common::math::Vec2d&: 常量引用参数，避免拷贝
 * - std::numeric_limits<double>::max(): 初始化最小距离平方为最大值
 * - data()[i]: 直接访问底层数组，比operator[]少边界检查
 * - Vec2d::DistanceSquareTo(): 计算到另一个点的距离平方
 * - for循环: 遍历所有轨迹点
 */
size_t DiscretizedTrajectory::QueryNearestPoint(
    const common::math::Vec2d& position) const {
  /**
   * dist_sqr_min - 到目前为止的最小距离平方
   * 初始化为double类型的最大值
   */
  double dist_sqr_min = std::numeric_limits<double>::max();
  size_t index_min = 0;  /**< 最小距离对应索引，初始化为0 */

  /**
   * 遍历所有轨迹点
   * for (size_t i = 0; i < size(); ++i)
   *   - size_t: 无符号整数类型
   *   - size(): 返回元素个数
   *   - ++i: 前置递增
   */
  for (size_t i = 0; i < size(); ++i) {
    /**
     * 创建当前位置的Vec2d对象
     * data()[i].path_point().x/y(): 获取第i个轨迹点的X和Y坐标
     * data(): std::vector的方法，返回底层数组的指针
     */
    const common::math::Vec2d curr_point(data()[i].path_point().x(),
                                         data()[i].path_point().y());

    /**
     * 计算到目标位置的距离平方
     * DistanceSquareTo() - Vec2d类的方法，计算欧氏距离的平方
     * 使用平方比较避免开方运算，更高效
     */
    const double dist_sqr = curr_point.DistanceSquareTo(position);

    /**
     * 更新最小距离和对应索引
     * if (dist_sqr < dist_sqr_min)
     */
    if (dist_sqr < dist_sqr_min) {
      dist_sqr_min = dist_sqr;  /**< 更新最小距离平方 */
      index_min = i;            /**< 更新最小距离对应的索引 */
    }
  }
  return index_min;  /**< 返回最近点的索引 */
}

/**
 * @brief 带缓冲区的最近点查询
 *
 * 在QueryNearestPoint基础上增加缓冲区概念，
 * 如果找到的点距离大于缓冲区，则返回缓冲区内的最优解。
 * 用于处理边界情况，提高搜索鲁棒性。
 *
 * @param position 二维位置向量
 * @param buffer 缓冲区大小，距离平方加上此值
 * @return size_t 最近点的索引
 *
 * 语法说明：
 * - dist_sqr < dist_sqr_min + buffer
 *   只要找到的点比"当前最小+缓冲区"小，就更新
 *   这样即使没有找到非常近的点，也会返回一个相对最优的索引
 */
size_t DiscretizedTrajectory::QueryNearestPointWithBuffer(
    const common::math::Vec2d& position, const double buffer) const {
  double dist_sqr_min = std::numeric_limits<double>::max(); /**< 最小距离平方 */
  size_t index_min = 0;  /**< 最小距离对应索引 */

  /**
   * 遍历所有轨迹点
   * 与QueryNearestPoint类似，但比较条件不同
   */
  for (size_t i = 0; i < size(); ++i) {
    /**
     * 创建当前位置Vec2d对象
     */
    const common::math::Vec2d curr_point(data()[i].path_point().x(),
                                         data()[i].path_point().y());

    /**
     * 计算距离平方
     */
    const double dist_sqr = curr_point.DistanceSquareTo(position);

    /**
     * 带缓冲区的比较
     * dist_sqr < dist_sqr_min + buffer
     * 只要比"当前最小+缓冲区"小就更新
     * 这样可以接受略大于最小值但仍在缓冲区内的点
     */
    if (dist_sqr < dist_sqr_min + buffer) {
      dist_sqr_min = dist_sqr;
      index_min = i;
    }
  }
  return index_min;
}

/**
 * @brief 追加轨迹点
 *
 * 在轨迹末尾追加一个新的轨迹点。
 * 断言检查新点的时间必须大于最后一个点的时间。
 *
 * @param trajectory_point 要追加的轨迹点
 *
 * 语法说明：
 * - CHECK_GT(a, b): 断言宏，a > b，否则终止程序并输出错误
 *   GT = Greater Than
 * - back().relative_time(): 获取当前最后一个点的相对时间
 * - push_back(item): std::vector方法，在末尾添加元素
 */
void DiscretizedTrajectory::AppendTrajectoryPoint(
    const TrajectoryPoint& trajectory_point) {
  /**
   * 如果轨迹非空，检查时间递增
   * !empty(): 检查容器是否非空
   */
  if (!empty()) {
    /**
     * CHECK_GT - 断言检查
     * 确保新点的时间大于最后一个点的时间
     * 这是轨迹的时间单调性要求
     */
    CHECK_GT(trajectory_point.relative_time(), back().relative_time());
  }

  /**
   * push_back - 在末尾追加元素
   * 从此DiscretizedTrajectory继承自std::vector
   */
  push_back(trajectory_point);
}

/**
 * @brief 获取指定索引的轨迹点
 *
 * @param index 轨迹点索引
 * @return const TrajectoryPoint& 轨迹点的常量引用
 *
 * 语法说明：
 * - const TrajectoryPoint&: 常量引用返回，不能修改返回的轨迹点
 * - CHECK_LT(a, b): 断言宏，a < b，否则终止程序
 *   LT = Less Than
 * - NumOfPoints(): 返回轨迹点数量
 * - data()[index]: 直接访问底层数组
 *   等价于operator[]但少边界检查，更高效
 */
const TrajectoryPoint& DiscretizedTrajectory::TrajectoryPointAt(
    const size_t index) const {
  /**
   * CHECK_LT - 断言检查索引有效
   * index < NumOfPoints()
   * 确保索引在有效范围内[0, NumOfPoints())
   */
  CHECK_LT(index, NumOfPoints());
  return data()[index];  /**< 返回指定索引的轨迹点引用 */
}

/**
 * @brief 获取轨迹起点
 *
 * @return TrajectoryPoint 轨迹的第一个点（按值返回）
 *
 * 语法说明：
 * - front(): std::vector方法，返回首元素引用
 * - ACHECK(!empty()): 确保轨迹非空
 */
TrajectoryPoint DiscretizedTrajectory::StartPoint() const {
  ACHECK(!empty());  /**< 断言检查轨迹非空 */
  return front();    /**< 返回首元素（按值返回） */
}

/**
 * @brief 获取轨迹的时间长度
 *
 * @return double 时间长度（秒），即最后一个点和第一个点的相对时间差
 *
 * 语法说明：
 * - back().relative_time() - 尾元素相对时间
 * - front().relative_time() - 首元素相对时间
 * - return 0.0: 空轨迹返回0.0
 */
double DiscretizedTrajectory::GetTemporalLength() const {
  /**
   * 空轨迹检查
   */
  if (empty()) {
    return 0.0;  /**< 空轨迹时间长度为0 */
  }
  /**
   * 时间长度 = 最后一个点的时间 - 第一个点的时间
   */
  return back().relative_time() - front().relative_time();
}

/**
 * @brief 获取轨迹的空间长度
 *
 * @return double 空间长度（米），即最后一个点和第一个点的s坐标差
 *
 * 语法说明：
 * - back().path_point().s() - 尾元素的路径长度s
 * - front().path_point().s() - 首元素的路径长度s
 * - s坐标表示沿轨迹的累积距离
 */
double DiscretizedTrajectory::GetSpatialLength() const {
  /**
   * 空轨迹检查
   */
  if (empty()) {
    return 0.0;  /**< 空轨迹空间长度为0 */
  }
  /**
   * 空间长度 = 最后一个点的s - 第一个点的s
   */
  return back().path_point().s() - front().path_point().s();
}

}  // namespace planning
}  // namespace apollo
