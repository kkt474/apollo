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
 * @file discrete_points_reference_line_smoother.cc
 *
 * @brief 离散点参考线平滑器实现文件
 *
 * 功能说明：
 * 本文件实现了DiscretePointsReferenceLineSmoother类
 * 是Apollo规划模块中基于离散点进行参考线平滑的核心算法类
 *
 * 主要功能：
 * 1. 对离散点组成的原始参考线进行平滑处理
 * 2. 支持两种平滑算法：Cos Theta平滑和Fem Pos平滑
 * 3. 生成平滑后的参考线及其曲率信息
 *
 * 核心概念：
 * - 参考线（Reference Line）：自动驾驶的期望行驶路径
 * - 锚点（Anchor Point）：必须经过的约束点
 * - 离散点平滑：将离散的航点序列平滑为连续曲线
 * - 曲率（Kappa）：描述曲线弯曲程度
 *
 * C++语法说明：
 * - #include：预处理指令，包含头文件
 * - namespace：命名空间，避免命名冲突
 * - class：类声明
 * - ::运算符：作用域解析，类名::函数名表示成员函数
 * - std::vector：标准库动态数组容器
 * - std::pair：标准库 pair 结构体
 **/

#include "modules/planning/planning_base/reference_line/discrete_points_reference_line_smoother.h"

/**
 * @brief 标准库算法头文件
 *
 * C++语法说明：
 * - <algorithm>：提供std::max、std::min、std::for_each等算法函数
 * - std::for_each：遍历容器并对每个元素执行操作
 */
#include <algorithm>

/**
 * @brief Cyber RT文件操作和日志系统
 *
 * C++语法说明：
 * - cyber/common/file.h：Cyber RT框架的文件操作工具
 * - cyber/common/log.h：Cyber RT的日志系统
 * - AERROR：Apollo错误级别日志宏
 * - CHECK_GT/CHECK_EQ：断言宏，检查条件
 */
#include "cyber/common/file.h"
#include "cyber/common/log.h"

/**
 * @brief Apollo通用工具类
 *
 * C++语法说明：
 * - modules/common/util/util.h：通用工具函数
 */
#include "modules/common/util/util.h"

/**
 * @brief 规划模块配置参数
 *
 * C++语法说明：
 * - gflags/planning_gflags.h：gflags配置变量
 * - FLAGS_xxx：全局配置标志
 */
#include "modules/planning/planning_base/gflags/planning_gflags.h"

/**
 * @brief 离散点数学工具
 *
 * C++语法说明：
 * - DiscretePointsMath：离散点数学计算类
 * - ComputePathProfile：计算路径的航向、曲率等 profile 信息
 */
#include "modules/planning/planning_base/math/discrete_points_math.h"

/**
 * @brief Cos Theta平滑器
 *
 * 功能说明：
 * Cos Theta平滑是一种基于余弦角度约束的平滑算法
 * 目标是最小化路径的方向变化
 */
#include "modules/planning/planning_base/math/discretized_points_smoothing/cos_theta_smoother.h"

/**
 * @brief Fem Pos平滑器
 *
 * 功能说明：
 * Fem Pos平滑是一种基于位置偏差的平滑算法
 * 使用有限元方法最小化位置偏差和曲线平滑度
 */
#include "modules/planning/planning_base/math/discretized_points_smoothing/fem_pos_deviation_smoother.h"

namespace apollo {
/**
 * @brief Apollo主命名空间
 */
namespace planning {

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
 * - DiscretePointsReferenceLineSmoother(...) :
 *   构造函数声明和实现
 *
 * - const ReferenceLineSmootherConfig& config：
 *   常量引用参数
 *   config是平滑器的配置参数结构体
 *
 * - : ReferenceLineSmoother(config)：
 *   初始化列表，调用基类构造函数
 *   将配置传递给基类
 *
 * - ReferenceLineSmoother(config)：
 *   基类构造函数的调用
 *   父类 ReferenceLineSmoother 的构造函数
 */
DiscretePointsReferenceLineSmoother::DiscretePointsReferenceLineSmoother(
    const ReferenceLineSmootherConfig& config)
    : ReferenceLineSmoother(config) {}

/**
 * @brief 平滑参考线主函数
 *
 * @param raw_reference_line 原始（未平滑）参考线
 * @param smoothed_reference_line 输出参数，平滑后的参考线
 * @return bool 平滑成功返回true
 *
 * 功能说明：
 * 这是离散点参考线平滑的主入口函数
 * 完整的平滑流程：
 * 1. 提取锚点坐标和横向约束
 * 2. 固定首尾锚点的横向约束为0
 * 3. 坐标归一化
 * 4. 根据配置选择平滑算法
 * 5. 坐标反归一化
 * 6. 生成参考点profile（航向、曲率等）
 * 7. 构建平滑后的参考线
 *
 * C++语法说明：
 * - const ReferenceLine& raw_reference_line：
 *   常量引用输入参数
 *   ReferenceLine是参考线类
 *
 * - ReferenceLine* const smoothed_reference_line：
 *   指向常量的指针
 *   指针本身是const（不能改变指向）
 *   但可以通过它修改所指对象
 *
 * - std::vector<std::pair<double, double>>：
 *   动态数组容器，存储pair（二元组）
 *   pair<double, double> 表示2D坐标点(x, y)
 *
 * - std::vector<double>：
 *   动态数组容器，存储double类型
 *   用于存储边界约束
 */
bool DiscretePointsReferenceLineSmoother::Smooth(
    const ReferenceLine& raw_reference_line,
    ReferenceLine* const smoothed_reference_line) {
  /**
   * @brief 准备原始点和约束
   *
   * raw_point2d：存储锚点的2D坐标
   * anchorpoints_lateralbound：存储每个锚点的横向约束
   */
  std::vector<std::pair<double, double>> raw_point2d;
  std::vector<double> anchorpoints_lateralbound;

  /**
   * @brief 遍历锚点提取坐标和约束
   *
   * C++语法说明：
   * - for (const auto& anchor_point : anchor_points_)：
   *   范围for循环
   *   const引用避免拷贝
   *   anchor_points_是类成员变量
   *
   * - raw_point2d.emplace_back(...)：
   *   emplace_back在容器末尾直接构造元素
   *   比push_back更高效
   *   避免拷贝或移动
   *
   * - anchor_point.path_point.x() / .y()：
   *   访问PathPoint的x和y坐标
   *   path_point是锚点内的路径点
   *
   * - anchor_point.lateral_bound：
   *   锚点的横向约束边界
   */
  for (const auto& anchor_point : anchor_points_) {
    raw_point2d.emplace_back(anchor_point.path_point.x(),
                             anchor_point.path_point.y());
    anchorpoints_lateralbound.emplace_back(anchor_point.lateral_bound);
  }

  /**
   * @brief 固定首尾锚点横向约束为0
   *
   * 功能说明：
   * 将起始点和终点的横向约束设为0
   * 确保车辆在道路中心附近开始和结束
   * 避免末端状态偏离道路中心
   *
   * C++语法说明：
   * - anchorpoints_lateralbound.front()：
   *   front()返回容器首元素的引用
   *
   * - anchorpoints_lateralbound.back()：
   *   back()返回容器末尾元素的引用
   *
   * - = 0.0：
   *   直接赋值，将横向约束设为0
   */
  anchorpoints_lateralbound.front() = 0.0;
  anchorpoints_lateralbound.back() = 0.0;

  /**
   * @brief 坐标归一化
   *
   * 功能说明：
   * 将所有坐标点平移到以第一个点为原点的坐标系
   * 减少数值计算中的精度问题
   * 改善优化算法的收敛性
   *
   * C++语法说明：
   * - NormalizePoints(&raw_point2d)：
   *   &raw_point2d获取vector的地址
   *   传递给函数进行原位修改
   */
  NormalizePoints(&raw_point2d);

  bool status = false;

  /**
   * @brief 根据配置选择平滑方法
   *
   * C++语法说明：
   * - config_.discrete_points().smoothing_method()：
   *   config_是类成员变量，存储配置
   *   链式调用获取平滑方法枚举值
   *
   * - std::vector<std::pair<double, double>> smoothed_point2d：
   *   创建空vector存储平滑后的2D点
   */
  const auto& smoothing_method = config_.discrete_points().smoothing_method();
  std::vector<std::pair<double, double>> smoothed_point2d;

  /**
   * @brief switch语句选择平滑算法
   *
   * C++语法说明：
   * - switch (smoothing_method)：
   *   switch语句根据枚举值选择分支
   *
   * - case DiscretePointsSmootherConfig::COS_THETA_SMOOTHING：
   *   case标签，必须是常量表达式
   *   使用作用域限定符::
   *
   * - status = CosThetaSmooth(...)：
   *   调用Cos Theta平滑函数
   *
   * - &smoothed_point2d：
   *   &获取vector地址作为输出参数
   *
   * - break：
   *   跳出switch语句
   *
   * - default:
   *   默认分支，当所有case都不匹配时执行
   *
   * - AERROR << "Smoother type not defined"：
   *   AERROR是Apollo错误日志宏
   *   <<运算符连接日志内容
   */
  switch (smoothing_method) {
    case DiscretePointsSmootherConfig::COS_THETA_SMOOTHING:
      status = CosThetaSmooth(raw_point2d, anchorpoints_lateralbound,
                              &smoothed_point2d);
      break;
    case DiscretePointsSmootherConfig::FEM_POS_DEVIATION_SMOOTHING:
      status = FemPosSmooth(raw_point2d, anchorpoints_lateralbound,
                            &smoothed_point2d);
      break;
    default:
      AERROR << "Smoother type not defined";
      return false;
  }

  /**
   * @brief 检查平滑是否成功
   */
  if (!status) {
    AERROR << "discrete_points reference line smoother fails";
    return false;
  }

  /**
   * @brief 坐标反归一化
   *
   * 功能说明：
   * 将归一化坐标转换回原始坐标系
   * 还原到真实的地理坐标
   *
   * C++语法说明：
   * - DeNormalizePoints(&smoothed_point2d)：
   *   原位修改vector
   */
  DeNormalizePoints(&smoothed_point2d);

  /**
   * @brief 生成参考点profile
   *
   * 功能说明：
   * 根据平滑后的2D点生成完整的参考点信息
   * 包括航向角、曲率等
   *
   * C++语法说明：
   * - std::vector<ReferencePoint> ref_points：
   *   ReferencePoint是参考线的点类型
   *   包含位置、航向、曲率等信息
   *
   * - GenerateRefPointProfile(...)：
   *   生成参考点profile的函数
   *   输出参数是reference_points
   */
  std::vector<ReferencePoint> ref_points;
  GenerateRefPointProfile(raw_reference_line, smoothed_point2d, &ref_points);

  /**
   * @brief 移除重复参考点
   *
   * 功能说明：
   * 去除过于接近的重复参考点
   * 减少数据冗余
   *
   * C++语法说明：
   * - ReferencePoint::RemoveDuplicates(&ref_points)：
   *   静态成员函数调用
   *   ::作用域限定符
   *   &ref_points传递地址
   */
  ReferencePoint::RemoveDuplicates(&ref_points);

  /**
   * @brief 检查参考点数量是否足够
   *
   * C++语法说明：
   * - ref_points.size() < 2：
   *   size()返回容器元素个数
   *   参考线至少需要2个点
   */
  if (ref_points.size() < 2) {
    AERROR << "Fail to generate smoothed reference line.";
    return false;
  }

  /**
   * @brief 构建平滑后的参考线
   *
   * 功能说明：
   * 使用平滑后的参考点构造参考线对象
   *
   * C++语法说明：
   * - *smoothed_reference_line = ReferenceLine(ref_points)：
   *   解引用指针赋值
   *   调用ReferenceLine构造函数
   *   传入参考点vector
   */
  *smoothed_reference_line = ReferenceLine(ref_points);
  return true;
}

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
 * 生成平滑且自然的曲率连续路径
 *
 * 算法原理：
 * 目标函数 = Σ cos(θ_i) 或类似的角度惩罚项
 * 约束条件 = 位置约束 + 边界约束
 *
 * C++语法说明：
 * - const std::vector<std::pair<double, double>>& raw_point2d：
 *   常量引用输入参数
 *   二维坐标点vector
 *
 * - std::vector<std::pair<double, double>>* ptr_smoothed_point2d：
 *   指针类型的输出参数
 *   可以是nullptr
 */
bool DiscretePointsReferenceLineSmoother::CosThetaSmooth(
    const std::vector<std::pair<double, double>>& raw_point2d,
    const std::vector<double>& bounds,
    std::vector<std::pair<double, double>>* ptr_smoothed_point2d) {
  /**
   * @brief 获取Cos Theta平滑配置
   *
   * C++语法说明：
   * - config_.discrete_points().cos_theta_smoothing()：
   *   链式调用获取嵌套配置
   *   返回CosThetaSmoothingConfig结构体
   */
  const auto& cos_theta_config =
      config_.discrete_points().cos_theta_smoothing();

  /**
   * @brief 创建Cos Theta平滑器实例
   *
   * C++语法说明：
   * - CosThetaSmoother smoother(cos_theta_config)：
   *   在栈上创建平滑器对象
   *   传入配置进行初始化
   */
  CosThetaSmoother smoother(cos_theta_config);

  /**
   * @brief 调整边界约束
   *
   * 功能说明：
   * Cos Theta平滑器使用box约束（矩形约束）
   * 需要将横向约束缩放为对角线约束
   * 因为box约束同时约束x和y两个方向
   *
   * 缩放比例 = 1 / √2 ≈ 0.707
   *
   * C++语法说明：
   * - std::vector<double> box_bounds = bounds：
   *   拷贝构造
   *   box_bounds是新vector
   *
   * - const double box_ratio = 1.0 / std::sqrt(2.0)：
   *   const常量
   *   std::sqrt是平方根函数
   *
   * - for (auto& bound : box_bounds)：
   *   范围for循环
   *   auto& 自动推导类型并引用
   *   可以修改元素值
   *
   * - bound *= box_ratio：
   *   复合赋值运算符
   *   等价于 bound = bound * box_ratio
   */
  std::vector<double> box_bounds = bounds;
  const double box_ratio = 1.0 / std::sqrt(2.0);
  for (auto& bound : box_bounds) {
    bound *= box_ratio;
  }

  /**
   * @brief 调用平滑器求解
   *
   * C++语法说明：
   * - std::vector<double> opt_x, opt_y：
   *   声明两个vector存储优化结果
   *   opt_x存储优化后的x坐标
   *   opt_y存储优化后的y坐标
   *
   * - smoother.Solve(raw_point2d, box_bounds, &opt_x, &opt_y)：
   *   调用Solve执行优化
   *   传入输入点和约束
   *   通过指针返回优化结果
   */
  std::vector<double> opt_x;
  std::vector<double> opt_y;
  bool status = smoother.Solve(raw_point2d, box_bounds, &opt_x, &opt_y);

  /**
   * @brief 检查求解是否成功
   */
  if (!status) {
    AERROR << "Costheta reference line smoothing failed";
    return false;
  }

  /**
   * @brief 检查结果有效性
   *
   * C++语法说明：
   * - opt_x.size() < 2 || opt_y.size() < 2：
   *   || 逻辑或运算符
   *   结果至少需要2个点才能构成路径
   */
  if (opt_x.size() < 2 || opt_y.size() < 2) {
    AERROR << "Return by Costheta smoother is wrong. Size smaller than 2 ";
    return false;
  }

  /**
   * @brief 断言x和y结果数量相等
   *
   * C++语法说明：
   * - CHECK_EQ(opt_x.size(), opt_y.size())：
   *   Apollo断言宏
   *   CHECK_EQ检查两个值是否相等
   *   不相等时打印错误并终止程序
   *
   * - << "x and y result size not equal"：
   *   << 运算符连接错误消息
   */
  CHECK_EQ(opt_x.size(), opt_y.size()) << "x and y result size not equal";

  /**
   * @brief 将优化结果转换为pair格式
   *
   * C++语法说明：
   * - size_t point_size = opt_x.size()：
   *   size_t是无符号整数类型
   *   用于表示容器大小和索引
   *
   * - for (size_t i = 0; i < point_size; ++i)：
   *   传统for循环
   *   ++i 前置递增运算符
   *
   * - ptr_smoothed_point2d->emplace_back(opt_x[i], opt_y[i])：
   *   -> 调用指针的成员函数
   *   emplace_back构造pair并添加到vector
   */
  size_t point_size = opt_x.size();
  for (size_t i = 0; i < point_size; ++i) {
    ptr_smoothed_point2d->emplace_back(opt_x[i], opt_y[i]);
  }

  return true;
}

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
 * 使用有限元思想最小化：
 * 1. 与原始点位置的偏差
 * 2. 路径的平滑度（曲率变化）
 *
 * 算法特点：
 * - 更好的局部路径保持能力
 * - 更平滑的曲率分布
 * - 适合复杂场景的路径平滑
 *
 * C++语法说明：
 * - FemPosDeviationSmoother：
 *   有限元位置偏差平滑器类
 *   设计与CosThetaSmoother类似
 */
bool DiscretePointsReferenceLineSmoother::FemPosSmooth(
    const std::vector<std::pair<double, double>>& raw_point2d,
    const std::vector<double>& bounds,
    std::vector<std::pair<double, double>>* ptr_smoothed_point2d) {
  /**
   * @brief 获取Fem Pos平滑配置
   */
  const auto& fem_pos_config =
      config_.discrete_points().fem_pos_deviation_smoothing();

  /**
   * @brief 创建Fem Pos平滑器实例
   */
  FemPosDeviationSmoother smoother(fem_pos_config);

  /**
   * @brief 调整边界约束（与Cos Theta相同）
   */
  std::vector<double> box_bounds = bounds;
  const double box_ratio = 1.0 / std::sqrt(2.0);
  for (auto& bound : box_bounds) {
    bound *= box_ratio;
  }

  /**
   * @brief 调用平滑器求解
   */
  std::vector<double> opt_x;
  std::vector<double> opt_y;
  bool status = smoother.Solve(raw_point2d, box_bounds, &opt_x, &opt_y);

  if (!status) {
    AERROR << "Fem Pos reference line smoothing failed";
    return false;
  }

  if (opt_x.size() < 2 || opt_y.size() < 2) {
    AERROR << "Return by fem pos smoother is wrong. Size smaller than 2 ";
    return false;
  }

  CHECK_EQ(opt_x.size(), opt_y.size()) << "x and y result size not equal";

  size_t point_size = opt_x.size();
  for (size_t i = 0; i < point_size; ++i) {
    ptr_smoothed_point2d->emplace_back(opt_x[i], opt_y[i]);
  }

  return true;
}

/**
 * @brief 设置锚点
 *
 * @param anchor_points 锚点向量
 *
 * 功能说明：
 * 设置用于平滑的锚点约束
 * 锚点是路径上必须经过的点
 *
 * C++语法说明：
 * - CHECK_GT(anchor_points.size(), 1U)：
 *   断言宏
 *   CHECK_GT检查第一个参数是否大于第二个
 *   1U是unsigned int类型的1
 *   确保至少有2个锚点
 *
 * - anchor_points_ = anchor_points：
 *   赋值运算符
 *   将参数拷贝给成员变量
 */
void DiscretePointsReferenceLineSmoother::SetAnchorPoints(
    const std::vector<AnchorPoint>& anchor_points) {
  CHECK_GT(anchor_points.size(), 1U);
  anchor_points_ = anchor_points;
}

/**
 * @brief 坐标归一化
 *
 * @param xy_points 输入输出参数，2D坐标点向量
 *
 * 功能说明：
 * 将所有坐标点平移到以第一个点为原点的局部坐标系
 * 这有助于：
 * 1. 减少大数值计算误差
 * 2. 改善优化算法收敛性
 * 3. 提高数值稳定性
 *
 * 变换公式：
 * x' = x - x₀
 * y' = y - y₀
 * 其中(x₀, y₀)是第一个点的坐标
 *
 * C++语法说明：
 * - std::vector<std::pair<double, double>>* xy_points：
 *   指针类型的输入输出参数
 *   函数内直接修改原容器
 *
 * - zero_x_ = xy_points->front().first：
 *   front()返回首元素引用
 *   .first获取pair的第一个元素(x坐标)
 *   赋值给成员变量zero_x_
 *
 * - std::for_each(xy_points->begin(), xy_points->end(), [this](...))：
 *   for_each是算法函数
 *   第一个参数：遍历起始迭代器
 *   第二个参数：遍历结束迭代器
 *   第三个参数：lambda表达式（仿函数）
 *
 * - [this](std::pair<double, double>& point) { ... }：
 *   lambda表达式
 *   [this]捕获列表，允许访问this指针
 *   (std::pair<double, double>& point)参数声明
 *
 * - auto curr_x = point.first：
 *   auto自动类型推导
 *   .first获取pair的x坐标
 *
 * - std::pair<double, double> xy(curr_x - zero_x_, curr_y - zero_y_)：
 *   构造新的pair
 *   计算相对坐标
 *
 * - point = std::move(xy)：
 *   std::move将左值转为右值引用
 *   避免拷贝，提高效率
 *   移动语义，将xy的资源转移给point
 */
void DiscretePointsReferenceLineSmoother::NormalizePoints(
    std::vector<std::pair<double, double>>* xy_points) {
  zero_x_ = xy_points->front().first;
  zero_y_ = xy_points->front().second;
  std::for_each(xy_points->begin(), xy_points->end(),
                [this](std::pair<double, double>& point) {
                  auto curr_x = point.first;
                  auto curr_y = point.second;
                  std::pair<double, double> xy(curr_x - zero_x_,
                                               curr_y - zero_y_);
                  point = std::move(xy);
                });
}

/**
 * @brief 坐标反归一化
 *
 * @param xy_points 输入输出参数，2D坐标点向量
 *
 * 功能说明：
 * 将归一化坐标还原到原始坐标系
 * 是NormalizePoints的逆操作
 *
 * 变换公式：
 * x = x' + x₀
 * y = y' + y₀
 *
 * C++语法说明：
 * - [this](std::pair<double, double>& point)：
 *   lambda表达式
 *   [this]捕获列表
 *   可以访问类的成员变量zero_x_和zero_y_
 *
 * - curr_x + zero_x_：
 *   加法运算，恢复原始坐标
 */
void DiscretePointsReferenceLineSmoother::DeNormalizePoints(
    std::vector<std::pair<double, double>>* xy_points) {
  std::for_each(xy_points->begin(), xy_points->end(),
                [this](std::pair<double, double>& point) {
                  auto curr_x = point.first;
                  auto curr_y = point.second;
                  std::pair<double, double> xy(curr_x + zero_x_,
                                               curr_y + zero_y_);
                  point = std::move(xy);
                });
}

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
 * 包括：
 * 1. 计算每个点的航向角
 * 2. 计算每个点的曲率
 * 3. 计算累积距离
 * 4. 转换到SL坐标系
 * 5. 获取车道信息
 *
 * C++语法说明：
 * - std::vector<double> headings：
 *   存储每个点的航向角（弧度）
 *
 * - std::vector<double> kappas：
 *   存储每个点的曲率
 *
 * - std::vector<double> dkappas：
 *   存储每个点的曲率变化率
 *
 * - std::vector<double> accumulated_s：
 *   存储累积距离
 */
bool DiscretePointsReferenceLineSmoother::GenerateRefPointProfile(
    const ReferenceLine& raw_reference_line,
    const std::vector<std::pair<double, double>>& xy_points,
    std::vector<ReferencePoint>* reference_points) {
  /**
   * @brief 计算路径profile
   *
   * 功能说明：
   * 使用DiscretePointsMath工具类计算：
   * - headings：每个点的航向角
   * - accumulated_s：累积距离
   * - kappas：曲率
   * - dkappas：曲率导数
   *
   * C++语法说明：
   * - DiscretePointsMath::ComputePathProfile(...)：
   *   静态成员函数调用
   *   ::作用域限定符
   *
   * - &headings, &accumulated_s, &kappas, &dkappas：
   *   指针作为输出参数
   */
  std::vector<double> headings;
  std::vector<double> kappas;
  std::vector<double> dkappas;
  std::vector<double> accumulated_s;
  if (!DiscretePointsMath::ComputePathProfile(
          xy_points, &headings, &accumulated_s, &kappas, &dkappas)) {
    return false;
  }

  /**
   * @brief 遍历所有点构建参考点
   *
   * C++语法说明：
   * - size_t points_size = xy_points.size()：
   *   获取点数量
   *   size_t适合存储容器大小
   */
  size_t points_size = xy_points.size();
  for (size_t i = 0; i < points_size; ++i) {
    /**
     * @brief 坐标转换XY->SL
     *
     * 功能说明：
     * 将全局XY坐标转换为参考线SL坐标系
     * S：沿参考线的累积距离
     * L：到参考线的横向偏移
     */
    common::SLPoint ref_sl_point;
    if (!raw_reference_line.XYToSL({xy_points[i].first, xy_points[i].second},
                                   &ref_sl_point)) {
      return false;
    }

    /**
     * @brief 检查SL点有效性
     *
     * C++语法说明：
     * - const double kEpsilon = 1e-6：
     *   非常小的容差值
     *   用于浮点数比较
     *
     * - ref_sl_point.s() < -kEpsilon：
     *   S值过小（在参考线起点之前）
     *
     * - ref_sl_point.s() > raw_reference_line.Length()：
     *   S值过大（超过参考线终点）
     *
     * - continue：
     *   跳过本次循环，继续下一次
     */
    const double kEpsilon = 1e-6;
    if (ref_sl_point.s() < -kEpsilon ||
        ref_sl_point.s() > raw_reference_line.Length()) {
      continue;
    }

    /**
     * @brief 确保S值非负
     *
     * C++语法说明：
     * - std::max(ref_sl_point.s(), 0.0)：
     *   std::max返回较大值
     *   确保S不小于0
     *
     * - ref_sl_point.set_s(...)：
     *   调用setter方法设置S值
     */
    ref_sl_point.set_s(std::max(ref_sl_point.s(), 0.0));

    /**
     * @brief 获取参考点信息
     *
     * C++语法说明：
     * - raw_reference_line.GetReferencePoint(ref_sl_point.s())：
     *   根据S值获取参考线上的参考点
     *   返回ReferencePoint类型
     */
    ReferencePoint rlp = raw_reference_line.GetReferencePoint(ref_sl_point.s());

    /**
     * @brief 更新车道航点横向偏移
     *
     * 功能说明：
     * 将SL点的L值赋给所有相关的车道航点
     *
     * C++语法说明：
     * - auto new_lane_waypoints = rlp.lane_waypoints()：
     *   auto自动推导类型
     *   获取参考点的车道航点列表
     *
     * - for (auto& lane_waypoint : new_lane_waypoints)：
     *   范围for循环
     *   引用可以直接修改元素
     *
     * - lane_waypoint.l = ref_sl_point.l()：
     *   设置横向偏移L值
     */
    auto new_lane_waypoints = rlp.lane_waypoints();
    for (auto& lane_waypoint : new_lane_waypoints) {
      lane_waypoint.l = ref_sl_point.l();
    }

    /**
     * @brief 构造参考点并添加
     *
     * C++语法说明：
     * - hdmap::MapPathPoint(...)：
     *   构造地图路径点
     *   参数：坐标点、航向角、车道航点列表
     *
     * - common::math::Vec2d(xy_points[i].first, xy_points[i].second)：
     *   构造2D向量
     *   使用坐标值初始化
     *
     * - ReferencePoint(..., kappas[i], dkappas[i])：
     *   构造参考点
     *   参数：地图路径点、曲率、曲率导数
     *
     * - reference_points->emplace_back(...)：
     *   通过指针调用emplace_back
     *   直接构造ReferencePoint并添加到vector
     */
    reference_points->emplace_back(ReferencePoint(
        hdmap::MapPathPoint(
            common::math::Vec2d(xy_points[i].first, xy_points[i].second),
            headings[i], new_lane_waypoints),
        kappas[i], dkappas[i]));
  }
  return true;
}

}  // namespace planning
}  // namespace apollo