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
 * @file util.h
 * @brief 通用工具函数头文件
 *
 * 功能说明：
 * 提供Apollo通用的工具函数
 * 包括：Protobuf比较、哈希、距离计算、浮点数比较等
 *
 * 核心概念：
 * - 模板函数：支持多种类型的通用实现
 * - SFINAE：模板类型选择技术
 * - ULPI误差：最后位置单位，用于浮点数比较
 * - 成员函数指针：指向类成员函数的指针
 *
 * C++语法说明：
 * - #pragma once：头文件保护
 * - template：模板声明
 * - std::enable_if：类型选择
 * - std::numeric_limits：数值类型限制
 * - std::hash：哈希函数对象
 * - std::pair：模板类，表示一对值
 * - std::mem_fun：成员函数适配器
 * - operator overload：运算符重载
 * - #define：宏定义
 **/

/**
 * @brief 头文件保护
 *
 * #pragma once：
 * 防止头文件被重复包含
 * 现代编译器支持，简洁高效
 */
#pragma once

#include <algorithm>
/**
 * @brief 标准算法头文件
 *
 * 提供常用算法：
 * - std::max/min：极值
 * - std::sort：排序
 * - std::find：查找
 */

#include <iostream>
/**
 * @brief 输入输出流头文件
 *
 * 提供：
 * - std::ostream：输出流
 * - std::cout：标准输出
 */

#include <limits>
/**
 * @brief 数值限制头文件
 *
 * 提供std::numeric_limits<T>：
 * - max()：类型最大值
 * - min()：类型最小值
 * - epsilon()：机器epsilon
 * - is_integer：是否整数类型
 */

#include <memory>
/**
 * @brief 智能指针头文件
 *
 * 提供：
 * - std::shared_ptr：引用计数智能指针
 * - std::unique_ptr：独占所有权智能指针
 */

#include <string>
/**
 * @brief 字符串头文件
 *
 * 提供std::string字符串类
 */

#include <utility>
/**
 * @brief 工具类头文件
 *
 * 提供：
 * - std::pair：模板类，表示一对值
 * - std::move：移动语义
 */

#include <vector>
/**
 * @brief 动态数组头文件
 *
 * 提供std::vector容器
 */

#include "cyber/common/log.h"
/**
 * @brief Cyber RT日志头文件
 *
 * 提供：
 * - AINFO/ADEBUG：信息/调试日志
 * - AERROR/AWARN：错误/警告日志
 */

#include "cyber/common/types.h"
/**
 * @brief Cyber RT类型定义头文件
 *
 * 提供：
 * - SUCC/ERROR：状态码
 */

#include "modules/common/configs/config_gflags.h"
/**
 * @brief 规划配置参数头文件
 *
 * 提供FLAGS_xxx配置参数访问
 */

#include "modules/common/math/vec2d.h"
/**
 * @brief 二维向量数学头文件
 *
 * 提供Vec2d类：
 * - 向量运算：加减乘除
 * - 点乘、叉乘
 * - 长度、距离计算
 */

#include "modules/common_msgs/basic_msgs/geometry.pb.h"
/**
 * @brief 几何消息头文件
 *
 * 提供几何相关消息类型：
 * - PointENU：ENU坐标点
 */

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
/**
 * @brief PNC点消息头文件
 *
 * 提供路径点消息类型：
 * - PathPoint：路径点
 */

namespace apollo {
/**
 * @brief Apollo项目主命名空间
 */

namespace common {
/**
 * @brief 通用工具命名空间
 */

namespace util {
/**
 * @brief 工具函数命名空间
 *
 * apollo::common::util：
 * 通用工具函数集合
 */

/**
 * @brief 判断两个Protobuf消息是否相等
 *
 * @tparam ProtoA 第一个Protobuf消息类型模板参数
 * @tparam ProtoB 第二个Protobuf消息类型模板参数
 * @param a 第一个Protobuf消息常量引用
 * @param b 第二个Protobuf消息常量引用
 * @return bool 两消息是否相等
 *
 * 功能说明：
 * 比较两个Protobuf消息是否相等
 * 通过比较类型名和序列化字符串判断
 *
 * 算法流程：
 * 1. 比较消息类型名
 * 2. 比较序列化后的字符串
 *
 * C++语法说明：
 * - template <typename ProtoA, typename ProtoB>：
 *   函数模板，支持两种不同类型
 * - const ProtoA& a：常量引用参数，避免拷贝
 * - a.GetTypeName()：获取Protobuf消息类型名
 * - a.SerializeAsString()：序列化消息为字符串
 *
 * @code
 *   Chassis c1, c2;
 *   if (IsProtoEqual(c1, c2)) {
 *     // 两消息相等
 *   }
 * @endcode
 */
template <typename ProtoA, typename ProtoB>
/**
 * @brief 模板函数定义
 *
 * template语法：
 * template <typename T1, typename T2>
 * 声明两个类型模板参数
 */
bool IsProtoEqual(const ProtoA& a, const ProtoB& b) {
  /**
   * @brief 比较类型名和序列化字符串
   *
   * a.GetTypeName() == b.GetTypeName()：
   * - 获取两个消息的类型名
   * - 比较是否相同
   *
   * a.SerializeAsString() == b.SerializeAsString()：
   * - 将消息序列化为字符串
   * - 比较字符串是否相等
   *
   * 注意：SerializeAsString是二进制序列化
   */

  return a.GetTypeName() == b.GetTypeName() &&
         a.SerializeAsString() == b.SerializeAsString();
  /**
   * @brief 返回逻辑与结果
   *
   * &&：逻辑与运算符
   * 只有类型名相同且序列化字符串相同，才算相等
   */

  // Test shows that the above method is 5 times faster than the
  // API: google::protobuf::util::MessageDifferencer::Equals(a, b);
  /**
   * @brief 性能注释
   *
   * 测试显示此方法比MessageDifferencer::Equals快5倍
   * MessageDifferencer是Protobuf提供的深度比较工具
   */
}

/**
 * @struct PairHash
 * @brief pair的哈希函数对象
 *
 * 功能说明：
 * 为std::pair提供哈希计算
 * 用于unordered_map等容器的键类型
 *
 * C++语法说明：
 * - struct：结构体声明
 * - operator()：函数调用运算符重载
 * - std::hash<T>：标准库哈希函数对象
 */
struct PairHash {
  /**
   * @brief 重载函数调用运算符
   *
   * @tparam T pair第一个元素类型
   * @tparam U pair第二个元素类型
   * @param pair 输入的pair
   * @return size_t 哈希值
   *
   * 功能说明：
   * 将pair的两个元素分别哈希后异或
   *
   * 算法：
   * hash = hash(first) ^ hash(second)
   *
   * C++语法说明：
   * - template <typename T, typename U>：
   *   结构体模板，两个类型参数
   * - size_t operator()(const std::pair<T, U>& pair) const：
   *   重载()运算符，使对象可像函数一样调用
   *   const承诺不修改状态
   */
  template <typename T, typename U>
  /**
   * @brief 模板成员函数
   *
   * typename T, typename U：
   * 两个模板参数，分别对应pair的两个类型
   */
  size_t operator()(const std::pair<T, U>& pair) const {
    /**
     * @brief 哈希计算实现
     *
     * std::hash<T>()(pair.first)：
     * - 创建std::hash<T>对象
     * - 调用其operator()计算哈希
     *
     * ^：按位异或运算符
     * - 将两个哈希值异或
     * - 结合两元素的哈希
     */
    return std::hash<T>()(pair.first) ^ std::hash<U>()(pair.second);
    /**
     * @brief 返回组合哈希值
     *
     * 异或的特点：
     * - a ^ a = 0
     * - 可结合、可交换
     * - 保留两元素的信息
     */
  }
};

/**
 * @brief 判断值是否在范围内
 *
 * @tparam T 值类型模板参数
 * @param start 范围起始值
 * @param end 范围结束值
 * @param value 要检查的值
 * @return bool 值是否在[start, end]范围内
 *
 * 功能说明：
 * 检查value是否在闭区间[start, end]内
 *
 * C++语法说明：
 * - template <typename T>：
 *   单类型参数模板
 * - return value >= start && value <= end：
 *   逻辑与组合两个比较
 *
 * @code
 *   if (WithinBound(0, 10, 5)) {
 *     // 5在[0,10]范围内
 *   }
 * @endcode
 */
template <typename T>
/**
 * @brief 模板函数
 *
 * typename T：
 * 类型模板参数，可以是int、double等
 */
bool WithinBound(T start, T end, T value) {
  /**
   * @brief 范围检查
   *
   * value >= start：
   * - 检查值不小于起始值
   *
   * value <= end：
   * - 检查值不大于结束值
   *
   * &&：逻辑与
   * - 两个条件都满足才返回true
   */
  return value >= start && value <= end;
  /**
   * @brief 返回布尔结果
   *
   * 闭区间：[start, end]
   * 包括start和end本身
   */
}

PointENU operator+(const PointENU enu, const math::Vec2d& xy);
/**
 * @brief PointENU加法运算符声明
 *
 * @param enu ENU坐标点
 * @param xy 二维向量
 * @return PointENU 运算结果
 *
 * 功能说明：
 * 将ENU坐标点与二维向量相加
 *
 * C++语法说明：
 * - operator+：运算符重载
 * - const PointENU：返回值类型
 * - const PointENU enu：常量引用参数
 * - const math::Vec2d& xy：常量引用参数
 */

/**
 * @brief 均匀切片函数
 *
 * @tparam T 数值类型模板参数
 * @param start 切片起始值
 * @param end 切片结束值
 * @param num 切片数量
 * @param sliced 输出参数，切片点向量
 *
 * 功能说明：
 * 将区间[start, end]均匀切分为num+1个点
 * 返回包含所有切片点的向量
 *
 * 算法流程：
 * 1. 计算步长delta = (end - start) / num
 * 2. 调整向量大小为num+1
 * 3. 循环计算并填充每个切片点
 * 4. 最后一个点设置为end
 *
 * C++语法说明：
 * - std::vector<T>*：指向向量的指针
 * - sliced->resize(num + 1)：调整向量大小
 * - sliced->at(i)：访问向量元素
 * - for循环的逗号表达式：i++和s+=delta
 *
 * @code
 *   std::vector<double> points;
 *   uniform_slice(0.0, 10.0, 4, &points);
 *   // points = {0, 2.5, 5.0, 7.5, 10.0}
 * @endcode
 */
/**
 * uniformly slice a segment [start, end] to num + 1 pieces
 * the result sliced will contain the n + 1 points that slices the provided
 * segment. `start` and `end` will be the first and last element in `sliced`.
 */
/**
 * @brief 注释说明
 *
 * 将线段[start, end]均匀切分为num+1段
 * 结果包含n+1个切片点
 * start和end分别是第一个和最后一个元素
 */
template <typename T>
/**
 * @brief 模板函数
 *
 * typename T：
 * 数值类型，可以是int、double等
 */
void uniform_slice(const T start, const T end, uint32_t num,
                   /**
                    * @brief 切片数量
                    *
                    * uint32_t：
                    * 无符号32位整数
                    * 非负整数
                    */
                   std::vector<T>* sliced) {
                   /**
                    * @brief 输出参数
                    *
                    * std::vector<T>*：
                    * - 指向向量的指针
                    * - 非const，可以修改
                    */

  if (!sliced || num == 0) {
    /**
     * @brief 空指针或零切片检查
     *
     * !sliced：
     * - 检查指针是否为空
     * - nullptr转换bool为false
     *
     * num == 0：
     * - 检查切片数是否为0
     */
    return;  /**< 无效参数，直接返回 */
  }

  const T delta = (end - start) / num;
  /**
   * @brief 计算切片步长
   *
   * (end - start) / num：
   * - 总长度除以切片数
   * - 得到每个切片的宽度
   *
   * const T delta：
   * - const：常量，不可修改
   * - T：与模板参数相同类型
   */

  sliced->resize(num + 1);
  /**
   * @brief 调整向量大小
   *
   * resize(num + 1)：
   * - 调整向量容量
   * - num个切片产生num+1个点
   * - 元素会重新分配
   */

  T s = start;
  /**
   * @brief 初始化当前点值
   */
  for (uint32_t i = 0; i < num; ++i, s += delta) {
    /**
     * @brief 循环填充切片点
     *
     * for循环三表达式：
     * - 初始化：uint32_t i = 0
     * - 条件：i < num
     * - 更新：++i, s += delta
     *
     * 逗号表达式：
     * - i++先返回i再加1
     * - s += delta同时更新s
     *
     * 注意：++i和i++的区别
     * - ++i：先加1后返回
     * - i++：先返回后加1
     */
    sliced->at(i) = s;
    /**
     * @brief 设置第i个切片点
     *
     * at(i)：
     * - 访问向量第i个元素
     * - 带边界检查
     */
  }

  sliced->at(num) = end;
  /**
   * @brief 设置最后一个点为end
   *
   * 确保最后一个点正好是end
   * 避免浮点误差累积
   */
}

/**
 * @brief 计算两点间的XY平面距离
 *
 * @tparam U 第一个点类型模板参数
 * @tparam V 第二个点类型模板参数
 * @param u 第一个点常量引用
 * @param v 第二个点常量引用
 * @return double 两点间的欧几里得距离
 *
 * 功能说明：
 * 计算两点在XY平面上的欧几里得距离
 * 要求类型U和V都有x()和y()成员函数
 *
 * 算法：
 * distance = sqrt((u.x-v.x)^2 + (u.y-v.y)^2)
 *
 * C++语法说明：
 * - template <typename U, typename V>：
 *   两个独立的类型参数
 *   U和V可以是不同类型，只要都有x()和y()
 * - const U& u：常量引用
 * - std::hypot：计算sqrt(x*x + y*y)的数学函数
 *
 * @code
 *   Vec2d p1(1.0, 2.0);
 *   Vec2d p2(4.0, 6.0);
 *   double d = DistanceXY(p1, p2);  // d = 5.0
 * @endcode
 */
/**
 * calculate the distance beteween Point u and Point v, which are all have
 * member function x() and y() in XY dimension.
 * @param u one point that has member function x() and y().
 * @param b one point that has member function x() and y().
 * @return sqrt((u.x-v.x)^2 + (u.y-v.y)^2), i.e., the Euclid distance on XY
 * dimension.
 */
/**
 * @brief 注释说明
 *
 * 计算两个点的XY距离
 * 要求点有x()和y()成员函数
 */
template <typename U, typename V>
/**
 * @brief 双类型模板
 */
double DistanceXY(const U& u, const V& v) {
  /**
   * @brief 计算欧几里得距离
   *
   * std::hypot(u.x() - v.x(), u.y() - v.y())：
   * - hypot(a, b) = sqrt(a*a + b*b)
   * - 更稳定，避免中间溢出
   *
   * u.x()和v.x()：
   * - 调用x()成员函数
   * - 模板不限制具体类型
   */
  return std::hypot(u.x() - v.x(), u.y() - v.y());
  /**
   * @brief 返回计算结果
   */
}

/**
 * @brief 判断两点是否相同
 *
 * @tparam U 第一个点类型模板参数
 * @tparam V 第二个点类型模板参数
 * @param u 第一个点常量引用
 * @param v 第二个点常量引用
 * @return bool 两点是否相同
 *
 * 功能说明：
 * 判断两点在XY平面上是否是同一点
 * 使用误差阈值判断浮点数相等
 *
 * 算法：
 * (u.x-v.x)^2 < epsilon^2 && (u.y-v.y)^2 < epsilon^2
 *
 * C++语法说明：
 * - static constexpr double：
 *   静态编译时常量
 *   比#define有类型安全
 * - 1e-8 * 1e-8：
 *   科学计数法
 *   epsilon的平方
 *
 * @code
 *   Vec2d p1(1.0, 2.0);
 *   Vec2d p2(1.0 + 1e-10, 2.0);
 *   if (SamePointXY(p1, p2)) {
 *     // 被认为是同一点
 *   }
 * @endcode
 */
/**
 * Check if two points u and v are the same point on XY dimension.
 * @param u one point that has member function x() and y().
 * @param v one point that has member function x() and y().
 * @return sqrt((u.x-v.x)^2 + (u.y-v.y)^2) < epsilon, i.e., the Euclid distance
 * on XY dimension.
 */
/**
 * @brief 注释说明
 *
 * 检查XY维度上两点是否相同
 */
template <typename U, typename V>
/**
 * @brief 双类型模板
 */
bool SamePointXY(const U& u, const V& v) {
  /**
   * @brief 定义误差阈值
   *
   * static constexpr double：
   * - static：程序唯一实例
   * - constexpr：编译时可确定值
   * - double：双精度浮点类型
   *
   * kMathEpsilonSqr：
   * - k前缀：常量命名约定
   * - EpsilonSqr：epsilon平方
   * - 1e-8 * 1e-8 = 1e-16
   */
  static constexpr double kMathEpsilonSqr = 1e-8 * 1e-8;

  /**
   * @brief 比较X坐标差平方
   *
   * (u.x() - v.x())：
   * - 获取两点的x坐标差
   *
   * (u.x() - v.x()) * (u.x() - v.x())：
   * - 计算差值的平方
   * - 避免开方运算
   *
   * < kMathEpsilonSqr：
   * - 与阈值比较
   * - 小于阈值认为相等
   */
  return (u.x() - v.x()) * (u.x() - v.x()) < kMathEpsilonSqr &&
         /**
          * @brief 逻辑与：两个维度都要满足
          */
         (u.y() - v.y()) * (u.y() - v.y()) < kMathEpsilonSqr;
  /**
   * @brief 返回布尔结果
   *
   * &&：逻辑与
   * 两个维度都在阈值内才算同一点
   */
}

PathPoint GetWeightedAverageOfTwoPathPoints(const PathPoint& p1,
                                            const PathPoint& p2,
                                            const double w1, const double w2);
/**
 * @brief 计算两个路径点的加权平均声明
 *
 * @param p1 第一个路径点
 * @param p2 第二个路径点
 * @param w1 第一个点权重
 * @param w2 第二个点权重
 * @return PathPoint 加权平均结果
 *
 * 功能说明：
 * 计算两个路径点的加权平均值
 * 用于轨迹平滑等场景
 */

/**
 * @brief 判断两个浮点数是否相等
 *
 * @tparam T 数值类型模板参数
 * @param x 第一个浮点数
 * @param y 第二个浮点数
 * @param ulp 比较精度（最后位置单位）
 * @return bool 两数是否相等
 *
 * 功能说明：
 * 基于ULP(Units in Last Place)的浮点数比较
 * 更适合处理浮点精度问题
 *
 * 算法原理：
 * 1. 计算两数之差的绝对值
 * 2. 与基于epsilon和ULP的阈值比较
 * 3. 或者检查结果是否小于最小正规数
 *
 * C++语法说明：
 * - typename std::enable_if<...>::type：
 *   SFINAE技术，限制只处理浮点类型
 * - std::numeric_limits<T>::is_integer：
 *   检查类型是否为整数
 * - std::numeric_limits<T>::epsilon()：
 *   机器epsilon
 * - std::fabs：浮点数绝对值
 *
 * @code
 *   if (IsFloatEqual(0.1 + 0.2, 0.3)) {
 *     // 考虑浮点误差，认为相等
 *   }
 * @endcode
 */
// Test whether two float or double numbers are equal.
// ulp: units in the last place.
/**
 * @brief 注释说明
 *
 * 测试两个浮点数是否相等
 * ulp：最后位置单位
 */
template <typename T>
/**
 * @brief 模板函数
 *
 * typename T：
 * 数值类型模板参数
 */
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type
/**
 * @brief SFINAE类型约束
 *
 * typename std::enable_if<Cond, int>::type：
 * - Cond为true时有type成员
 * - !std::numeric_limits<T>::is_integer：
 *   取反，检查T不是整数类型
 *
 * 效果：
 * 只有浮点类型(double, float)才能使用此函数
 * 整数类型会触发SFINAE，匹配到其他重载
 */
IsFloatEqual(T x, T y, int ulp = 2) {
  /**
   * @brief 默认参数ulp=2
   *
   * 允许最后2位有差异
   */

  // the machine epsilon has to be scaled to the magnitude of the values used
  // and multiplied by the desired precision in ULPs (units in the last place)
  /**
   * @brief 算法注释
   *
   * 机器epsilon需要根据值的量级缩放
   * 并乘以ULP指定的精度
   */

  return std::fabs(x - y) <
             std::numeric_limits<T>::epsilon() * std::fabs(x + y) * ulp
         /**
          * @brief 第一部分比较
          *
          * std::fabs(x - y)：
          * - 差值的绝对值
          *
          * std::numeric_limits<T>::epsilon()：
          * - 机器epsilon
          * - float: ~1.19e-7
          * - double: ~2.22e-16
          *
          * std::fabs(x + y)：
          * - 两数之和的绝对值
          * - 用于缩放epsilon到当前量级
          *
          * ulp：
          * - 精度单位数
          * - 控制比较的严格程度
          */
         // unless the result is subnormal
         /**
          * @brief 第二部分检查
          *
          * 处理次正规数情况
          */
         || std::fabs(x - y) < std::numeric_limits<T>::min();
         /**
          * @brief 或者差值小于最小正规数
          *
          * ||：逻辑或
          * - 如果第一条件不满足
          * - 检查是否是次正规数情况
          *
          * std::numeric_limits<T>::min()：
          * - 最小正规正浮点数
          * - 比此值更小的数是次正规数
          */
}
}  // namespace util
/**
 * @brief util命名空间结束标记
 */

}  // namespace common
/**
 * @brief common命名空间结束标记
 */

}  // namespace apollo
/**
 * @brief apollo命名空间结束标记
 */

/**
 * @brief 函数信息模板类
 *
 * @tparam T 包含成员函数的类类型
 *
 * 功能说明：
 * 封装成员函数及其名称
 * 用于批量执行函数的工具类
 *
 * C++语法说明：
 * - template <typename T>：
 *   类模板声明
 * - typedef int (T::*Function)()：
 *   成员函数指针类型声明
 *   - T::*：指向T类成员的指针
 *   - Function：类型别名
 */
template <typename T>
/**
 * @brief 类模板定义
 */
class FunctionInfo {
 public:
  /**
   * @brief 成员函数指针类型别名
   *
   * typedef int (T::*Function)()：
   * - typedef：类型别名定义
   * - T::*：指向类T成员的指针
   * - Function：别名的名字
   *
   * 含义：
   * - Function是指向T类成员函数的指针
   * - 该成员函数返回int，无参数
   *
   * @code
   *   typedef int (MyClass::*Handler)();
   *   Handler h = &MyClass::my_method;
   * @endcode
   */
  typedef int (T::*Function)();
  /**
   * @brief 存储成员函数指针
   *
   * Function function_：
   * - 成员函数指针变量
   * - 可以指向T类的任何返回int无参成员函数
   */
  Function function_;

  /**
   * @brief 存储函数名称
   *
   * std::string fun_name_：
   * - 函数名称字符串
   * - 用于日志输出
   */
  std::string fun_name_;
};

/**
 * @brief 执行所有函数
 *
 * @tparam T 类类型模板参数
 * @tparam count 函数列表长度
 * @param obj 对象指针
 * @param fun_list 函数信息数组
 * @return bool 是否所有函数都执行成功
 *
 * 功能说明：
 * 批量执行对象的一组成员函数
 * 如果任何函数返回非SUCC，立即返回false
 *
 * 算法流程：
 * 1. 遍历函数列表
 * 2. 调用每个成员函数
 * 3. 检查返回值是否为SUCC
 * 4. 如果失败，输出错误日志并返回false
 *
 * C++语法说明：
 * - template <typename T, size_t count>：
 *   两个模板参数：类类型和数组长度
 * - size_t：无符号大小类型
 * - (obj->*(fun_list[i].function_))()：
 *   成员函数指针调用语法
 * - apollo::cyber::SUCC：成功状态码
 *
 * @code
 *   FunctionInfo<MyClass> funs[] = {
 *     {&MyClass::init, "init"},
 *     {&MyClass::start, "start"},
 *   };
 *   if (ExcuteAllFunctions(obj, funs)) {
 *     // 所有函数执行成功
 *   }
 * @endcode
 */
template <typename T, size_t count>
/**
 * @brief 双模板参数
 *
 * typename T：类类型
 * size_t count：常量表达式，表示数组长度
 */
bool ExcuteAllFunctions(T* obj, FunctionInfo<T> fun_list[]) {
  /**
   * @brief 遍历函数列表
   *
   * for (size_t i = 0; i < count; i++)：
   * - size_t：无符号大小类型
   * - i < count：数组边界检查
   */
  for (size_t i = 0; i < count; i++) {
    /**
     * @brief 调用成员函数
     *
     * (obj->*(fun_list[i].function_))()：
     * - obj->*：成员指针解引用运算符
     * - fun_list[i].function_：获取函数指针
     * - ()：调用函数
     *
     * 分解：
     * - obj->*：对obj使用成员指针
     * - (fun_list[i].function_)：获取第i个函数指针
     * - ()：调用该成员函数
     */
    if ((obj->*(fun_list[i].function_))() != apollo::cyber::SUCC) {
      /**
       * @brief 检查返回值
       *
       * != apollo::cyber::SUCC：
       * - 如果返回值不是成功状态
       * - SUCC是Cyber RT的成功状态码
       */
      AERROR << fun_list[i].fun_name_ << " failed.";
      /**
       * @brief 输出错误日志
       *
       * fun_list[i].fun_name_：
       * - 获取第i个函数的名称
       * - 用于标识哪个函数失败
       */
      return false;  /**< 函数执行失败 */
    }
  }
  return true;  /**< 所有函数都执行成功 */
}

/**
 * @brief 执行所有函数的宏
 *
 * @param type 类类型
 * @param obj 对象指针
 * @param list 函数信息数组
 *
 * 功能说明：
 * 便捷宏，包装ExcuteAllFunctions调用
 * 自动计算数组长度
 *
 * C++语法说明：
 * - #define：宏定义
 * - sizeof(list) / sizeof(FunctionInfo<type>)：
 *   计算数组元素个数
 *   - sizeof(list)：整个数组大小
 *   - sizeof(FunctionInfo<type>)：单个元素大小
 *
 * @code
 *   EXEC_ALL_FUNS(MyClass, obj, funs)
 *   // 等价于：
 *   // ExcuteAllFunctions<MyClass, sizeof(funs)/sizeof(FunctionInfo<MyClass>)>(obj, funs)
 * @endcode
 */
#define EXEC_ALL_FUNS(type, obj, list) \
  /**
   * @brief 宏定义
   *
   * \：行继续符
   * 将多行宏写在一行
   */
  ExcuteAllFunctions<type, sizeof(list) / sizeof(FunctionInfo<type>)>(obj, list)
  /**
   * @brief 展开后的代码
   *
   * ExcuteAllFunctions<type, N>(obj, list)
   * - type：类类型模板参数
   * - N：数组长度常量
   */

/**
 * @brief pair的流输出运算符重载
 *
 * @tparam A pair第一个元素类型
 * @tparam B pair第二个元素类型
 * @param os 输出流引用
 * @param p 要输出的pair
 * @return std::ostream& 输出流引用
 *
 * 功能说明：
 * 重载<<运算符，使pair可以直接输出到流
 * 格式："first: xxx, second: xxx"
 *
 * C++语法说明：
 * - template <typename A, typename B>：
 *   双类型模板参数
 * - std::ostream& operator<<：
 *   返回ostream引用的运算符重载
 * - return os << ...：
 *   链式输出
 *
 * @code
 *   std::pair<int, std::string> p = {1, "hello"};
 *   std::cout << p << std::endl;
 *   // 输出：first: 1, second: hello
 * @endcode
 */
template <typename A, typename B>
/**
 * @brief 双类型模板
 */
std::ostream& operator<<(std::ostream& os, std::pair<A, B>& p) {
  /**
   * @brief 重载运算符实现
   *
   * os << "first: " << p.first：
   * - 输出"first: "和pair的第一个元素
   *
   * << ", second: " << p.second：
   * - 继续输出", second: "和第二个元素
   *
   * return os：
   * - 返回流引用，支持链式调用
   */
  return os << "first: " << p.first << ", second: " << p.second;
}

/**
 * @brief 多线程安全锁宏
 *
 * @param mutex_type 互斥量类型
 *
 * 功能说明：
 * 如果启用多线程模式，创建锁
 * 否则创建一个空指针
 *
 * 算法流程：
 * 1. 检查FLAGS_multithread_run标志
 * 2. 如果启用多线程，创建unique_lock
 * 3. 否则保持lock_ptr为nullptr
 *
 * C++语法说明：
 * - #define：宏定义
 * - std::unique_ptr<std::unique_lock<std::mutex>>：
 *   智能指针，指向unique_lock
 * - if (FLAGS_multithread_run)：
 *   检查多线程标志
 * - lock_ptr.reset(new std::unique_lock<std::mutex>(mutex_type))：
 *   创建并管理lock对象
 *
 * @code
 *   std::mutex my_mutex;
 *   UNIQUE_LOCK_MULTITHREAD(my_mutex);
 *   // 如果启用多线程，lock_ptr现在持有锁
 *   // 否则lock_ptr是nullptr
 * @endcode
 */
#define UNIQUE_LOCK_MULTITHREAD(mutex_type)                         \
  /**
   * @brief 宏定义
   *
   * \：行继续符
   */
  std::unique_ptr<std::unique_lock<std::mutex>> lock_ptr = nullptr; \
  /**
   * @brief 初始化空指针
   *
   * std::unique_ptr<std::unique_lock<std::mutex>>：
   * - 智能指针，管理unique_lock对象
   * - unique_lock比lock_guard更灵活
   *
   * = nullptr：
   * - 默认初始化为空
   */
  if (FLAGS_multithread_run) {                                      \
    /**
     * @brief 检查多线程标志
     *
     * FLAGS_multithread_run：
     * - GFlags配置参数
     * - 在config_gflags.cc中定义
     */
    lock_ptr.reset(new std::unique_lock<std::mutex>(mutex_type));   \
    /**
     * @brief 创建锁
     *
     * lock_ptr.reset(new ...):
     * - reset替换管理的对象
     * - 如果已有对象，先释放
     *
     * new std::unique_lock<std::mutex>(mutex_type):
     * - 创建unique_lock对象
     * - 构造时自动加锁mutex_type
     * - 析构时自动解锁
     */
  }
