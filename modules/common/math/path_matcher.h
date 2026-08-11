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
 * @file path_matcher.h
 * @brief 路径匹配器类声明头文件
 *
 * 功能说明：
 * 路径匹配器类用于将笛卡尔坐标系中的点匹配到参考路径上
 * 主要功能：
 * 1. MatchToPath(x,y)：基于(x,y)坐标找到路径上最近的点
 * 2. GetPathFrenetCoordinate：计算点在Frenet坐标系中的坐标
 * 3. MatchToPath(s)：基于s坐标找到路径上的点
 *
 * 设计特点：
 * - 工具类，所有方法均为静态方法
 * - 禁止实例化（构造函数被delete）
 * - 提供坐标转换功能，用于规划模块
 *
 * C++语法说明：
 * - #pragma once：编译指示符，防止头文件重复包含
 * - class PathMatcher：路径匹配器类
 * - = delete：禁用特定的成员函数
 * - static关键字：静态成员函数，无需实例化即可调用
 */

/**
 * @brief 确保头文件只被包含一次
 *
 * C++语法说明：
 * #pragma once是编译指示符
 * 告诉编译器这个头文件只处理一次
 * 作用类似于传统的#ifndef宏保护
 * 优点：更简洁，编译器直接处理
 */
#pragma once

/**
 * @brief 标准库头文件
 *
 * C++语法说明：
 * - #include <utility>：标准库工具类
 *   - std::pair：键值对容器
 *
 * - #include <vector>：动态数组容器
 *   - std::vector：可变大小的数组
 */
#include <utility>
#include <vector>

/**
 * @brief Apollo PNC点消息头文件
 *
 * C++语法说明：
 * pnc_point.pb.h由pnc_point.proto编译生成
 * 包含PathPoint、TrajectoryPoint等消息定义
 * PathPoint包含x, y, z, theta, kappa等路径点信息
 */
#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"

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
 * @class PathMatcher
 * @brief 路径匹配器类
 *
 * 功能说明：
 * 路径匹配器用于将笛卡尔坐标系中的点匹配到参考路径上
 * 这是规划模块中的基础工具类
 *
 * 使用场景：
 * - 车辆定位：将GPS位置匹配到参考路径
 * - 坐标转换：笛卡尔坐标与Frenet坐标的转换
 * - 轨迹评估：计算实际轨迹与参考路径的偏差
 *
 * 设计特点：
 * 1. 工具类：所有方法都是静态方法
 * 2. 不可实例化：构造函数被delete
 * 3. 无状态：不存储任何成员变量
 *
 * C++语法说明：
 * - class PathMatcher：
 *   类声明，默认访问限定符为private
 *
 * - public: / private:：
 *   访问限定符
 *   public：成员可以从类外部访问
 *   private：成员只能在类内部访问
 */
class PathMatcher {
 public:
  /**
   * @brief 禁用默认构造函数
   *
   * C++语法说明：
   * - PathMatcher() = delete：
   *   delete是C++11的关键字
   *   表示禁用此函数
   *   任何尝试构造PathMatcher对象的代码都会编译错误
   *
   * 设计意图：
   * PathMatcher是工具类，不需要实例化
   * 所有方法都是静态方法，直接通过类名调用
   * 删除构造函数可以防止意外的实例化
   */
  PathMatcher() = delete;

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
   * 算法流程（详见实现文件）：
   * 1. 定义距离平方函数
   * 2. 找到距离最小的点索引
   * 3. 确定插值区间
   * 4. 计算投影点
   *
   * C++语法说明：
   * - static PathPoint：
   *   静态成员函数，属于类而非对象
   *   可以通过PathMatcher::MatchToPath()调用
   *
   * - const std::vector<PathPoint>&：
   *   常量引用输入参数
   *   const：承诺不修改
   *   引用：避免拷贝
   *
   * - const double x, const double y：
   *   const值参数
   *   承诺不修改参数值
   *   按值传递
   */
  static PathPoint MatchToPath(const std::vector<PathPoint>& reference_line,
                               const double x, const double y);

  /**
   * @brief 获取路径Frenet坐标系坐标
   *
   * @param reference_line 参考路径点序列
   * @param x 目标点的X坐标
   * @param y 目标点的Y坐标
   * @return std::pair<double, double> (s, d) Frenet坐标
   *   - first: 沿路径的累计距离s
   *   - second: 垂直于路径的侧向偏移d（左正右负）
   *
   * 功能说明：
   * 将笛卡尔坐标(x,y)转换为Frenet坐标系(s,d)
   * - s: 沿参考线累计距离
   * - d: 垂直于参考线的偏移，左正右负
   *
   * C++语法说明：
   * - static std::pair<double, double>：
   *   静态函数返回std::pair
   *   pair存储两个相关值
   *
   * - const std::vector<PathPoint>& reference_line：
   *   常量引用输入参数
   */
  static std::pair<double, double> GetPathFrenetCoordinate(
      const std::vector<PathPoint>& reference_line, const double x,
      const double y);

  /**
   * @brief 根据s坐标匹配到参考路径
   *
   * @param reference_line 参考路径点序列
   * @param s 目标s坐标（沿路径的累计距离）
   * @return PathPoint 匹配到的路径点
   *
   * 功能说明：
   * 根据给定的累计距离s，在参考路径上找到对应位置的点
   * 如果s在路径范围内，进行线性插值
   *
   * C++语法说明：
   * - static PathPoint MatchToPath(...)：
   *   静态函数重载
   *   根据参数类型不同调用不同实现
   *   第一个版本接收(x,y)，第二个版本接收(s)
   */
  static PathPoint MatchToPath(const std::vector<PathPoint>& reference_line,
                               const double s);

 private:
  /**
   * @brief 找到点到线段的投影点（私有方法）
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
   * 算法流程（详见实现文件）：
   * 1. 计算v0 = (x,y) - p0
   * 2. 计算v1 = p1 - p0
   * 3. 计算v1的模长
   * 4. 计算点积dot = v0 · v1
   * 5. 计算delta_s = dot / v1_norm
   * 6. 在p0和p1之间插值得到投影点
   *
   * C++语法说明：
   * - private:：
   *   私有成员，只能在类内部访问
   *   此方法是内部实现，不对外公开
   *
   * - static PathPoint FindProjectionPoint(...)：
   *   静态私有方法
   *   只能被类内部的静态方法调用
   */
  static PathPoint FindProjectionPoint(const PathPoint& p0, const PathPoint& p1,
                                       const double x, const double y);
};

/**
 * @brief 命名空间结束标记
 *
 * C++语法说明：
 * // 注释用于说明命名空间结束
 * 三层命名空间的闭合：
 * }  // namespace math
 * }  // namespace common
 * }  // namespace apollo
 */
}  // namespace math
}  // namespace common
}  // namespace apollo
