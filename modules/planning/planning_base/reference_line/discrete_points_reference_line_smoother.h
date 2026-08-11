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
 ******************************************************************************/

/**
 * @file discrete_points_reference_line_smoother.h
 *
 * @brief 离散点参考线平滑器头文件
 *
 * 功能说明：
 * 本头文件声明了DiscretePointsReferenceLineSmoother类
 * 是Apollo规划模块中基于离散点进行参考线平滑的核心算法类
 *
 * 主要功能：
 * 1. 对离散点组成的原始参考线进行平滑处理
 * 2. 支持两种平滑算法：Cos Theta平滑和Fem Pos平滑
 * 3. 生成平滑后的参考线及其曲率信息
 *
 * 类的继承关系：
 * DiscretePointsReferenceLineSmoother -> ReferenceLineSmoother（父类）
 * ReferenceLineSmoother是参考线平滑器的基类
 *
 * 核心概念：
 * - 参考线（Reference Line）：自动驾驶的期望行驶路径
 * - 锚点（Anchor Point）：必须经过的约束点
 * - 离散点平滑：将离散的航点序列平滑为连续曲线
 * - Cos Theta平滑：基于余弦角度约束的平滑算法
 * - Fem Pos平滑：基于有限元位置偏差的平滑算法
 *
 * C++语法说明：
 * - #pragma once：预处理器指令，防止头文件被重复包含
 * - class：类声明，用户定义类型
 * - public/private：访问限定符
 * - virtual/override：C++多态支持
 * - explicit：explicit关键字，防止隐式转换
 * - = default：默认实现关键字
 * - std::vector：标准库动态数组容器
 * - std::pair：标准库pair结构体
 **/

#pragma once

/**
 * @brief 标准库工具头文件
 *
 * C++语法说明：
 * - <utility>：提供std::pair等工具
 *   pair是二元组结构，用于存储两个相关联的值
 */
#include <utility>

/**
 * @brief 标准库动态数组容器
 *
 * C++语法说明：
 * - <vector>：提供std::vector模板类
 *   vector是动态数组，可自动扩展
 *   支持随机访问和迭代器遍历
 */
#include <vector>

/**
 * @brief 参考线平滑器配置proto
 *
 * C++语法说明：
 * - "modules/planning/planning_base/proto/reference_line_smoother_config.pb.h"：
 *   protobuf生成的头文件
 *   定义了ReferenceLineSmootherConfig配置结构
 *   包含平滑算法的参数配置
 */
#include "modules/planning/planning_base/proto/reference_line_smoother_config.pb.h"

/**
 * @brief 参考线类
 *
 * C++语法说明：
 * - ReferenceLine：参考线类
 *   表示自动驾驶的期望行驶路径
 *   包含路径点序列和曲率信息
 */
#include "modules/planning/planning_base/reference_line/reference_line.h"

/**
 * @brief 参考线平滑器基类
 *
 * C++语法说明：
 * - ReferenceLineSmoother：参考线平滑器基类
 *   定义了平滑器的接口
 *   DiscretePointsReferenceLineSmoother继承自此基类
 */
#include "modules/planning/planning_base/reference_line/reference_line_smoother.h"

/**
 * @brief 参考点类
 *
 * C++语法说明：
 * - ReferencePoint：参考点类
 *   包含位置、航向、曲率等信息
 *   是ReferenceLine的基本组成单元
 */
#include "modules/planning/planning_base/reference_line/reference_point.h"

namespace apollo {
/**
 * @brief Apollo主命名空间
 *
 * 功能说明：
 * 所有Apollo模块的代码都在apollo命名空间下
 * 避免与其他库的命名冲突
 */
namespace planning {
/**
 * @brief 规划模块命名空间
 *
 * 功能说明：
 * planning命名空间包含所有规划相关的类和函数
 */

/**
 * @class DiscretePointsReferenceLineSmoother
 * @brief 离散点参考线平滑器类
 *
 * 功能说明：
 * DiscretePointsReferenceLineSmoother是处理参考线平滑的核心类
 * 继承自ReferenceLineSmoother基类
 * 负责管理与路由相关的地图数据
 *
 * 主要职责：
 * 1. 设置锚点约束
 * 2. 执行坐标归一化/反归一化
 * 3. 调用平滑算法（Cos Theta或Fem Pos）
 * 4. 生成平滑后的参考点profile
 *
 * 使用场景：
 * - 车道跟随场景下的参考线平滑
 * - 路径规划前的数据预处理
 * - 生成平滑且曲率连续的轨迹
 *
 * C++语法说明：
 * - class DiscretePointsReferenceLineSmoother : public ReferenceLineSmoother：
 *   类继承声明
 *   DiscretePointsReferenceLineSmoother公有继承自ReferenceLineSmoother
 *   public继承意味着基类的public成员仍是public
 */
class DiscretePointsReferenceLineSmoother : public ReferenceLineSmoother {
 public:
  /**
   * @brief 构造函数
   *
   * @param config 参考线平滑器配置
   *
   * 功能说明：
   * 初始化离散点参考线平滑器
   * 调用基类构造函数并传入配置
   *
   * C++语法说明：
   * - explicit DiscretePointsReferenceLineSmoother(...)：
   *   explicit关键字
   *   防止隐式类型转换
   *   只允许显式调用构造函数
   *   避免 DiscretePointsReferenceLineSmoother x = config 这样的隐式转换
   *
   * - const ReferenceLineSmootherConfig& config：
   *   const：承诺不修改参数
   *   &：引用，避免参数拷贝
   *   ReferenceLineSmootherConfig：平滑器配置类型
   */
  explicit DiscretePointsReferenceLineSmoother(
      const ReferenceLineSmootherConfig& config);

  /**
   * @brief 虚析构函数
   *
   * 功能说明：
   * 确保通过基类指针删除派生类对象时
   * 能正确调用派生类的析构函数
   *
   * C++语法说明：
   * - virtual：
   *   虚函数关键字，支持运行时多态
   *
   * - ~DiscretePointsReferenceLineSmoother()：
   *   析构函数，类名前加~
   *
   * - = default：
   *   默认实现关键字
   *   使用编译器自动生成的析构函数
   */
  virtual ~DiscretePointsReferenceLineSmoother() = default;

  /**
   * @brief 平滑参考线主函数
   *
   * @param raw_reference_line 原始（未平滑）参考线
   * @param smoothed_reference_line 输出参数，平滑后的参考线
   * @return bool 平滑成功返回true
   *
   * 功能说明：
   * 对原始参考线进行平滑处理
   * 是该类的核心公共接口
   *
   * C++语法说明：
   * - bool Smooth(...)：
   *   返回bool类型表示成功或失败
   *
   * - const ReferenceLine& raw_reference_line：
   *   const引用输入参数
   *   承诺不修改原始参考线
   *
   * - ReferenceLine* const smoothed_reference_line：
   *   指向常量的指针
   *   指针本身是const（不能改变指向）
   *   但可以通过它修改所指对象
   *   作为输出参数使用
   *
   * - override：
   *   显式声明重写基类虚函数
   *   编译器会检查基类是否有同名虚函数
   */
  bool Smooth(const ReferenceLine& raw_reference_line,
              ReferenceLine* const smoothed_reference_line) override;

  /**
   * @brief 设置锚点
   *
   * @param 锚点向量引用
   *
   * 功能说明：
   * 设置用于平滑的锚点约束
   * 锚点是路径上必须经过的点
   *
   * C++语法说明：
   * - const std::vector<AnchorPoint>&：
   *   const引用避免拷贝
   *   std::vector存储AnchorPoint类型
   *   AnchorPoint是锚点结构体类型
   *
   * - override：
   *   重写基类的纯虚函数
   */
  void SetAnchorPoints(const std::vector<AnchorPoint>&) override;

 private:
  /**
   * @brief Cos Theta平滑算法
   *
   * @param raw_point2d 原始2D坐标点
   * @param bounds 横向约束边界
   * @param ptr_smoothed_point2d 输出参数，平滑后的2D坐标点
   * @return bool 平滑成功返回true
   *
   * 功能说明：
   * Cos Theta平滑是一种角度约束平滑方法
   * 核心思想是最小化路径方向变化的平方和
   *
   * C++语法说明：
   * - std::vector<std::pair<double, double>>：
   *   动态数组，存储pair（二元组）
   *   pair<double, double>表示2D坐标点(x, y)
   *
   * - std::vector<std::pair<double, double>>* ptr_smoothed_point2d：
   *   指针类型的输出参数
   *   可以是nullptr
   */
  bool CosThetaSmooth(
      const std::vector<std::pair<double, double>>& raw_point2d,
      const std::vector<double>& bounds,
      std::vector<std::pair<double, double>>* ptr_smoothed_point2d);

  /**
   * @brief Fem Pos平滑算法
   *
   * @param raw_point2d 原始2D坐标点
   * @param bounds 横向约束边界
   * @param ptr_smoothed_point2d 输出参数，平滑后的2D坐标点
   * @return bool 平滑成功返回true
   *
   * 功能说明：
   * Fem Pos平滑是一种基于位置偏差的平滑方法
   * 使用有限元思想最小化位置偏差和曲线平滑度
   *
   * C++语法说明：
   * - 设计与CosThetaSmooth类似
   *   参数和返回值类型相同
   */
  bool FemPosSmooth(
      const std::vector<std::pair<double, double>>& raw_point2d,
      const std::vector<double>& bounds,
      std::vector<std::pair<double, double>>* ptr_smoothed_point2d);

  /**
   * @brief 坐标归一化
   *
   * @param xy_points 输入输出参数，2D坐标点向量
   *
   * 功能说明：
   * 将所有坐标点平移到以第一个点为原点的局部坐标系
   * 有助于减少数值计算误差和改善优化收敛性
   *
   * C++语法说明：
   * - std::vector<std::pair<double, double>>* xy_points：
   *   指针类型的输入输出参数
   *   函数内直接修改原容器
   */
  void NormalizePoints(std::vector<std::pair<double, double>>* xy_points);

  /**
   * @brief 坐标反归一化
   *
   * @param xy_points 输入输出参数，2D坐标点向量
   *
   * 功能说明：
   * 将归一化坐标还原到原始坐标系
   * 是NormalizePoints的逆操作
   *
   * C++语法说明：
   * - 与NormalizePoints参数类型相同
   */
  void DeNormalizePoints(std::vector<std::pair<double, double>>* xy_points);

  /**
   * @brief 生成参考点profile
   *
   * @param raw_reference_line 原始参考线
   * @param xy_points 平滑后的2D坐标点
   * @param reference_points 输出参数，生成的参考点列表
   * @return bool 生成成功返回true
   *
   * 功能说明：
   * 将平滑后的2D坐标点转换为完整的参考点
   * 包括航向角、曲率等信息
   *
   * C++语法说明：
   * - std::vector<ReferencePoint>* reference_points：
   *   指针类型的输出参数
   *   ReferencePoint是参考线的点类型
   */
  bool GenerateRefPointProfile(
      const ReferenceLine& raw_reference_line,
      const std::vector<std::pair<double, double>>& xy_points,
      std::vector<ReferencePoint>* reference_points);

  /**
   * @brief 锚点列表
   *
   * 功能说明：
   * 存储用于平滑的锚点约束
   * 锚点是路径上必须经过的点
   *
   * C++语法说明：
   * - std::vector<AnchorPoint>：
   *   动态数组容器
   *   存储AnchorPoint类型的锚点
   *
   * - anchor_points_：
   *   成员变量使用_后缀
   *   Apollo的成员变量命名惯例
   */
  std::vector<AnchorPoint> anchor_points_;

  /**
   * @brief 归一化原点X坐标
   *
   * 功能说明：
   * 存储归一化时使用的原点X坐标
   * 在反归一化时需要使用
   *
   * C++语法说明：
   * - double zero_x_ = 0.0：
   *   double类型
   *   默认初始化为0.0
   *   _后缀表示成员变量
   */
  double zero_x_ = 0.0;

  /**
   * @brief 归一化原点Y坐标
   *
   * 功能说明：
   * 存储归一化时使用的原点Y坐标
   * 在反归一化时需要使用
   */
  double zero_y_ = 0.0;
};

}  // namespace planning
}  // namespace apollo