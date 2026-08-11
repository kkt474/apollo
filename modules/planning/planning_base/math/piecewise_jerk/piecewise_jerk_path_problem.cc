/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
 * @file piecewise_jerk_path_problem.cc
 * @brief 分段jerk路径优化问题实现文件
 *
 * 本文件实现了PiecewiseJerkPathProblem类，继承自PiecewiseJerkProblem基类。
 * 专门用于路径（而非速度）的分段jerk优化。
 *
 * 与基类的区别：
 * - 计算核矩阵P时考虑jerk项的权重
 * - 增加了额外约束（extra_constraints）和顶点约束（vertex_constraints）
 * - 增加了拖拽点参考线（towing_x_ref）支持
 *
 * 优化目标：
 * min sum(w_x * x² + w_dx * dx² + w_ddx * ddx² + w_dddx * dddx²)
 * + sum(w_x_ref * (x - x_ref)² + w_towing * (x - towing_x)²)
 *
 * C++语法说明：
 * - : PiecewiseJerkProblem(...): 构造函数初始化列表，调用基类构造函数
 * - override: C++11 override关键字，明确表示重写基类虚函数
 */

#include "modules/planning/planning_base/math/piecewise_jerk/piecewise_jerk_path_problem.h"

/**
 * @brief Cyber日志系统头文件
 * 提供AINFO, AERROR, ADEBUG等日志宏
 */
#include "cyber/common/log.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
/**
 * @brief Planning模块GFlags配置
 * FLAGS_enable_osqp_debug: OSQP调试开关
 * FLAGS_path_speed_osqp_setting_time_limit: OSQP时间限制
 */

namespace apollo {
/**
 * @brief Apollo顶层命名空间
 */
namespace planning {

/**
 * @brief PiecewiseJerkPathProblem构造函数
 *
 * @param num_of_knots 路径结点数量
 * @param delta_s 路径点间距（米）
 * @param x_init 初始状态 [x, dx, ddx]
 *
 * 使用构造函数初始化列表调用基类构造函数
 * 这是C++委托构造的常用方式
 *
 * C++语法说明：
 * : PiecewiseJerkProblem(num_of_knots, delta_s, x_init)
 *   - 初始化列表语法
 *   - 在构造函数的参数列表后、函数体前使用冒号
 *   - 用于调用基类构造函数进行初始化
 */
PiecewiseJerkPathProblem::PiecewiseJerkPathProblem(
    const size_t num_of_knots, const double delta_s,
    const std::array<double, 3>& x_init)
    : PiecewiseJerkProblem(num_of_knots, delta_s, x_init) {}

/**
 * @brief 计算核矩阵P（目标函数的Hessian矩阵）
 *
 * @param P_data 输出：稀疏矩阵非零元素值
 * @param P_indices 输出：非零元素列索引
 * @param P_indptr 输出：每列非零元素起始索引
 *
 * 核矩阵P是目标函数二次项系数的矩阵形式
 * 对于问题：min (1/2) x^T P x + q^T x
 * P是对称正定矩阵
 *
 * 目标函数展开：
 * P矩阵结构（对角元素）：
 * - 对角线前半部分：weight_x + weight_x_ref[i]（位置权重+参考权重）
 * - 中间部分：weight_dx（速度权重）
 * - 后半部分：weight_ddx + weight_dddx/delta_s²（加速度+jerk权重）
 *
 * 特别处理：
 * - 最后一个点额外增加weight_end_state权重
 * - jerk项在相邻点之间有耦合（-2 * weight_dddx / delta_s²）
 *
 * C++语法说明：
 * - std::vector<std::vector<std::pair<c_int, c_float>>> columns:
 *   二维稀疏向量，存储P矩阵的列格式
 *   外层vector是列索引，内层vector是该列的非零元素
 *   pair<列索引, 系数值>
 */
void PiecewiseJerkPathProblem::CalculateKernel(std::vector<c_float>* P_data,
                                               std::vector<c_int>* P_indices,
                                               std::vector<c_int>* P_indptr) {
  /**
   * @brief 转换为int类型用于索引计算
   */
  const int n = static_cast<int>(num_of_knots_);

  /**
   * @brief 计算变量总数
   * 3 * n: 每个结点有x, dx, ddx三个变量
   */
  const int num_of_variables = 3 * n;

  /**
   * @brief 计算非零元素个数
   * num_of_variables: 对角线元素（3N个）
   * (n - 1): jerk耦合项（N-1个）
   * 总计：3N + (N-1) = 4N - 1
   */
  const int num_of_nonzeros = num_of_variables + (n - 1);

  /**
   * @brief 创建稀疏列结构
   * columns[i]: 第i列的非零元素列表
   */
  std::vector<std::vector<std::pair<c_int, c_float>>> columns(num_of_variables);

  /**
   * @brief 当前非零元素索引
   */
  int value_index = 0;

  /**
   * @brief 第1部分：x（位置）变量的对角线元素
   *
   * 目标函数中x²项的系数：
   * weight_x + weight_x_ref_vec_[i]
   * - weight_x: 统一位置权重
   * - weight_x_ref_vec_[i]: 第i点的参考位置权重
   *
   * 除以scale_factor²用于反缩放
   */
  for (int i = 0; i < n - 1; ++i) {
    columns[i].emplace_back(i, (weight_x_ + weight_x_ref_vec_[i]) /
                                   (scale_factor_[0] * scale_factor_[0]));
    ++value_index;
  }

  /**
   * @brief 最后一个x变量的处理（额外增加终点权重）
   * weight_end_state_[0]: 终点位置权重
   */
  columns[n - 1].emplace_back(
      n - 1, (weight_x_ + weight_x_ref_vec_[n - 1] + weight_end_state_[0]) /
                 (scale_factor_[0] * scale_factor_[0]));
  ++value_index;

  /**
   * @brief 第2部分：dx（速度）变量的对角线元素
   *
   * 目标函数中dx²项的系数：weight_dx
   */
  for (int i = 0; i < n - 1; ++i) {
    columns[n + i].emplace_back(
        n + i, weight_dx_ / (scale_factor_[1] * scale_factor_[1]));
    ++value_index;
  }

  /**
   * @brief 最后一个dx变量的处理（额外增加终点权重）
   */
  columns[2 * n - 1].emplace_back(2 * n - 1,
                                  (weight_dx_ + weight_end_state_[1]) /
                                      (scale_factor_[1] * scale_factor_[1]));
  ++value_index;

  /**
   * @brief 第3部分：ddx（加速度）变量的对角线元素
   *
   * 公式：(weight_ddx + 2 * weight_dddx / delta_s²) / scale²
   *
   * 物理意义：
   * - weight_ddx: 加速度平滑权重
   * - weight_dddx / delta_s²: jerk项等效权重
   * - 系数2是因为相邻点jerk项会产生两个ddx项
   */
  auto delta_s_square = delta_s_ * delta_s_;  /**< delta_s的平方缓存 */
  columns[2 * n].emplace_back(2 * n,
                              (weight_ddx_ + weight_dddx_ / delta_s_square) /
                                  (scale_factor_[2] * scale_factor_[2]));
  ++value_index;

  /**
   * @brief 中间结点的ddx权重
   * 公式：(weight_ddx + 2.0 * weight_dddx / delta_s²) / scale²
   */
  for (int i = 1; i < n - 1; ++i) {
    columns[2 * n + i].emplace_back(
        2 * n + i, (weight_ddx_ + 2.0 * weight_dddx_ / delta_s_square) /
                       (scale_factor_[2] * scale_factor_[2]));
    ++value_index;
  }

  /**
   * @brief 最后一个ddx变量（额外增加终点权重）
   */
  columns[3 * n - 1].emplace_back(
      3 * n - 1,
      (weight_ddx_ + weight_dddx_ / delta_s_square + weight_end_state_[2]) /
          (scale_factor_[2] * scale_factor_[2]));
  ++value_index;

  /**
   * @brief 第4部分：jerk耦合项（非对角元素）
   *
   * jerk项展开：
   * (ddx[i+1] - ddx[i])² / delta_s²
   * = (ddx[i+1]² - 2*ddx[i+1]*ddx[i] + ddx[i]²) / delta_s²
   *
   * 交叉项系数：-2 * weight_dddx / delta_s²
   * 分配到ddx[i]和ddx[i+1]的对角线上各一半
   */
  for (int i = 0; i < n - 1; ++i) {
    columns[2 * n + i].emplace_back(2 * n + i + 1,
                                    (-2.0 * weight_dddx_ / delta_s_square) /
                                        (scale_factor_[2] * scale_factor_[2]));
    ++value_index;
  }

  /**
   * @brief 检查非零元素数量是否正确
   */
  CHECK_EQ(value_index, num_of_nonzeros);

  /**
   * @brief 将列格式转换为CSC格式
   *
   * CSC (Compressed Sparse Column) 格式：
   * - P_indptr: 每列的起始索引（长度=num_variables+1）
   * - P_indices: 非零元素的行索引
   * - P_data: 非零元素值
   *
   * 注意：OSQP需要P矩阵是上三角形式
   * 乘以2.0是因为OSQP的目标函数是(1/2)x^T P x
   */
  int ind_p = 0;  /**< 当前处理位置索引 */
  for (int i = 0; i < num_of_variables; ++i) {
    P_indptr->push_back(ind_p);  /**< 记录第i列的起始位置 */

    /**
     * @brief 遍历第i列的所有非零元素
     */
    for (const auto& row_data_pair : columns[i]) {
      P_data->push_back(row_data_pair.second * 2.0);  /**< 乘以2.0 */
      P_indices->push_back(row_data_pair.first);  /**< 行索引 */
      ++ind_p;
    }
  }
  P_indptr->push_back(ind_p);  /**< 添加最后一列的结束索引 */
}

/**
 * @brief 计算仿射约束矩阵A
 *
 * @param A_data 输出：稀疏矩阵非零元素值
 * @param A_indices 输出：非零元素列索引
 * @param A_indptr 输出：每列非零元素起始索引
 * @param lower_bounds 输出：约束下界
 * @param upper_bounds 输出：约束上界
 *
 * 约束类型：
 * 1. 变量边界约束（3N个）
 * 2. 额外约束（extra_constraints）
 * 3. 顶点约束（vertex_constraints）
 * 4. jerk约束（2*(N-1)个）
 * 5. 速度连续性约束（2*(N-1)个）
 * 6. 位移连续性约束（3*(N-1)个）
 * 7. 初始状态约束（3个）
 *
 * C++语法说明：
 * - extra_constraints_: 额外约束向量，用于自定义约束
 * - vertex_constraints_: 顶点约束，用于车辆顶点边界
 */
void PiecewiseJerkPathProblem::CalculateAffineConstraint(
    std::vector<c_float>* A_data, std::vector<c_int>* A_indices,
    std::vector<c_int>* A_indptr, std::vector<c_float>* lower_bounds,
    std::vector<c_float>* upper_bounds) {
  /**
   * @brief 获取结点数
   */
  const int n = static_cast<int>(num_of_knots_);

  /**
   * @brief 获取额外约束数量
   * extra_constraints_: 向量，存储用户自定义的线性约束
   */
  const int num_of_extra_constraints =
      static_cast<int>(extra_constraints_.size());

  /**
   * @brief 获取顶点约束数量
   * vertex_constraints_: 车辆顶点边界约束
   */
  const int num_of_vertex_constraints =
      static_cast<int>(vertex_constraints_.size());

  /**
   * @brief 计算变量和约束总数
   */
  const int num_of_variables = 3 * n;
  const int num_of_constraints = num_of_variables + 3 * (n - 1) + 3 +
                                 num_of_extra_constraints +
                                 num_of_vertex_constraints;

  /**
   * @brief 调整边界向量大小
   */
  lower_bounds->resize(num_of_constraints);
  upper_bounds->resize(num_of_constraints);

  /**
   * @brief 创建稀疏变量结构
   * variables[i]: 第i个变量涉及的约束列表
   */
  std::vector<std::vector<std::pair<c_int, c_float>>> variables(
      num_of_variables);

  int constraint_index = 0;

  /**
   * @brief 第1部分：设置变量边界约束（3N个）
   * x_bounds_, dx_bounds_, ddx_bounds_
   */
  for (int i = 0; i < num_of_variables; ++i) {
    if (i < n) {
      /**
       * @brief x变量的边界
       */
      variables[i].emplace_back(constraint_index, 1.0);
      lower_bounds->at(constraint_index) =
          x_bounds_[i].first * scale_factor_[0];
      upper_bounds->at(constraint_index) =
          x_bounds_[i].second * scale_factor_[0];
    } else if (i < 2 * n) {
      /**
       * @brief dx变量的边界
       */
      variables[i].emplace_back(constraint_index, 1.0);

      lower_bounds->at(constraint_index) =
          dx_bounds_[i - n].first * scale_factor_[1];
      upper_bounds->at(constraint_index) =
          dx_bounds_[i - n].second * scale_factor_[1];
    } else {
      /**
       * @brief ddx变量的边界
       */
      variables[i].emplace_back(constraint_index, 1.0);
      lower_bounds->at(constraint_index) =
          ddx_bounds_[i - 2 * n].first * scale_factor_[2];
      upper_bounds->at(constraint_index) =
          ddx_bounds_[i - 2 * n].second * scale_factor_[2];
    }
    ++constraint_index;
  }

  /**
   * @brief 检查约束索引
   */
  CHECK_EQ(constraint_index, num_of_variables);

  /**
   * @brief 第2部分：设置额外约束
   *
   * extra_constraints_格式：
   * left_index, right_index: 约束涉及的变量索引
   * left_weight, right_weight: 对应系数
   * lower_bound, upper_bound: 约束边界
   *
   * 约束形式：left_weight * x[left_index] + right_weight * x[right_index] in [lower, upper]
   */
  for (int i = 0; i < num_of_extra_constraints; ++i) {
    auto& left_index = extra_constraints_[i].left_index;
    auto& right_index = extra_constraints_[i].right_index;
    auto& left_weight = extra_constraints_[i].left_weight;
    auto& right_weight = extra_constraints_[i].right_weight;

    /**
     * @brief 添加两个变量项
     */
    variables[left_index].emplace_back(constraint_index,
                                       left_weight / scale_factor_[0]);
    variables[right_index].emplace_back(constraint_index,
                                        right_weight / scale_factor_[0]);

    /**
     * @brief 设置约束边界
     */
    lower_bounds->at(constraint_index) = extra_constraints_[i].lower_bound;
    upper_bounds->at(constraint_index) = extra_constraints_[i].upper_bound;
    ++constraint_index;
  }

  /**
   * @brief 第3部分：设置顶点约束
   *
   * vertex_constraints_用于车辆顶点边界约束
   * 需要同时约束顶点的位置和方向
   *
   * front_edge_to_center: 前顶点到中心点的距离
   */
  for (int i = 0; i < num_of_vertex_constraints; ++i) {
    auto& left_index = vertex_constraints_[i].left_index;
    auto& right_index = vertex_constraints_[i].right_index;
    auto& left_weight = vertex_constraints_[i].left_weight;
    auto& right_weight = vertex_constraints_[i].right_weight;

    /**
     * @brief 位置约束项
     */
    variables[left_index].emplace_back(constraint_index,
                                       left_weight / scale_factor_[0]);
    variables[right_index].emplace_back(constraint_index,
                                        right_weight / scale_factor_[0]);

    /**
     * @brief 方向约束项（考虑前顶点偏移）
     * 顶点方向与中心点方向的差异由front_edge_to_center产生
     */
    variables[n + left_index].emplace_back(
        constraint_index, vertex_constraints_.front_edge_to_center *
                              left_weight / scale_factor_[1]);
    variables[n + right_index].emplace_back(
        constraint_index, vertex_constraints_.front_edge_to_center *
                              right_weight / scale_factor_[1]);

    /**
     * @brief 设置约束边界
     */
    lower_bounds->at(constraint_index) = vertex_constraints_[i].lower_bound;
    upper_bounds->at(constraint_index) = vertex_constraints_[i].upper_bound;
    ++constraint_index;
  }

  /**
   * @brief 第4部分：设置jerk约束（2*(N-1)个）
   *
   * jerk = (ddx[i+1] - ddx[i]) / delta_s
   * 约束：dddx_bound.first <= jerk <= dddx_bound.second
   *
   * 矩阵形式：-1 * ddx[i] + 1 * ddx[i+1] in [lower, upper]
   */
  for (int i = 0; i + 1 < n; ++i) {
    variables[2 * n + i].emplace_back(constraint_index, -1.0);
    variables[2 * n + i + 1].emplace_back(constraint_index, 1.0);
    lower_bounds->at(constraint_index) =
        dddx_bound_.first * delta_s_ * scale_factor_[2];
    upper_bounds->at(constraint_index) =
        dddx_bound_.second * delta_s_ * scale_factor_[2];
    ++constraint_index;
  }

  /**
   * @brief 第5部分：设置速度连续性约束（2*(N-1)个）
   *
   * 公式：x(i+1)' - x(i)' - 0.5*delta_s*x(i)'' - 0.5*delta_s*x(i+1)'' = 0
   *
   * 物理意义：梯形积分近似位移
   */
  for (int i = 0; i + 1 < n; ++i) {
    variables[n + i].emplace_back(constraint_index, -1.0 * scale_factor_[2]);
    variables[n + i + 1].emplace_back(constraint_index, 1.0 * scale_factor_[2]);
    variables[2 * n + i].emplace_back(constraint_index,
                                      -0.5 * delta_s_ * scale_factor_[1]);
    variables[2 * n + i + 1].emplace_back(constraint_index,
                                          -0.5 * delta_s_ * scale_factor_[1]);
    lower_bounds->at(constraint_index) = 0.0;
    upper_bounds->at(constraint_index) = 0.0;
    ++constraint_index;
  }

  /**
   * @brief 第6部分：设置位移连续性约束（3*(N-1)个）
   *
   * 公式：x(i+1) - x(i) - delta_s*x(i)' - 1/3*delta_s²*x(i)'' - 1/6*delta_s²*x(i+1)'' = 0
   *
   * 基于Simpson积分公式
   */
  auto delta_s_sq_ = delta_s_ * delta_s_;  /**< delta_s平方缓存 */
  for (int i = 0; i + 1 < n; ++i) {
    variables[i].emplace_back(constraint_index,
                              -1.0 * scale_factor_[1] * scale_factor_[2]);
    variables[i + 1].emplace_back(constraint_index,
                                  1.0 * scale_factor_[1] * scale_factor_[2]);
    variables[n + i].emplace_back(
        constraint_index, -delta_s_ * scale_factor_[0] * scale_factor_[2]);
    variables[2 * n + i].emplace_back(
        constraint_index,
        -delta_s_sq_ / 3.0 * scale_factor_[0] * scale_factor_[1]);
    variables[2 * n + i + 1].emplace_back(
        constraint_index,
        -delta_s_sq_ / 6.0 * scale_factor_[0] * scale_factor_[1]);

    lower_bounds->at(constraint_index) = 0.0;
    upper_bounds->at(constraint_index) = 0.0;
    ++constraint_index;
  }

  /**
   * @brief 第7部分：设置初始状态约束（3个）
   *
   * x(0) = x_init[0], dx(0) = x_init[1], ddx(0) = x_init[2]
   */
  variables[0].emplace_back(constraint_index, 1.0);
  lower_bounds->at(constraint_index) = x_init_[0] * scale_factor_[0];
  upper_bounds->at(constraint_index) = x_init_[0] * scale_factor_[0];
  ++constraint_index;

  variables[n].emplace_back(constraint_index, 1.0);
  lower_bounds->at(constraint_index) = x_init_[1] * scale_factor_[1];
  upper_bounds->at(constraint_index) = x_init_[1] * scale_factor_[1];
  ++constraint_index;

  variables[2 * n].emplace_back(constraint_index, 1.0);
  lower_bounds->at(constraint_index) = x_init_[2] * scale_factor_[2];
  upper_bounds->at(constraint_index) = x_init_[2] * scale_factor_[2];
  ++constraint_index;

  /**
   * @brief 最终检查约束总数
   */
  CHECK_EQ(constraint_index, num_of_constraints);

  /**
   * @brief 转换为CSC格式
   */
  int ind_p = 0;
  for (int i = 0; i < num_of_variables; ++i) {
    A_indptr->push_back(ind_p);
    for (const auto& variable_nz : variables[i]) {
      A_data->push_back(variable_nz.second);
      A_indices->push_back(variable_nz.first);
      ++ind_p;
    }
  }

  /**
   * @brief 添加最后一列的结束索引
   * OSQP CSC格式要求
   */
  A_indptr->push_back(ind_p);
}

/**
 * @brief 计算目标函数的线性偏移向量q
 *
 * @param q 输出：线性项向量
 *
 * 目标函数形式：min (1/2) x^T P x + q^T x
 * q向量包含所有关于x的线性项
 *
 * 对于参考线项：(x - x_ref)² = x² - 2*x_ref*x + x_ref²
 * 展开后线性项为：-2 * x_ref * x
 * 所以q项为：-2 * weight_x_ref * x_ref
 *
 * C++语法说明：
 * - CHECK_NOTNULL(q): 断言q指针非空
 * - q->resize(): 调整向量大小
 * - q->at(i): 访问向量第i个元素
 */
void PiecewiseJerkPathProblem::CalculateOffset(std::vector<c_float>* q) {
  CHECK_NOTNULL(q);  /**< 断言检查q非空，避免空指针解引用 */

  const int n = static_cast<int>(num_of_knots_);
  const int kNumParam = 3 * n;  /**< 变量总数 */

  /**
   * @brief 初始化q向量为0
   */
  q->resize(kNumParam, 0.0);

  /**
   * @brief 如果存在参考线，添加参考线对q的贡献
   *
   * 对于项：weight_x_ref * (x - x_ref)²
   * 展开：weight_x_ref * x² - 2 * weight_x_ref * x_ref * x + weight_x_ref * x_ref²
   * 线性项系数：-2 * weight_x_ref * x_ref
   */
  if (has_x_ref_) {
    for (int i = 0; i < n; ++i) {
      q->at(i) += -2.0 * weight_x_ref_vec_.at(i) * x_ref_[i] / scale_factor_[0];
    }
  }

  /**
   * @brief 添加终点状态参考对q的贡献
   *
   * 只影响最后一个点
   */
  if (has_end_state_ref_) {
    /**
     * @brief 终点x贡献
     */
    q->at(n - 1) +=
        -2.0 * weight_end_state_[0] * end_state_ref_[0] / scale_factor_[0];

    /**
     * @brief 终点dx贡献
     */
    q->at(2 * n - 1) +=
        -2.0 * weight_end_state_[1] * end_state_ref_[1] / scale_factor_[1];

    /**
     * @brief 终点ddx贡献
     */
    q->at(3 * n - 1) +=
        -2.0 * weight_end_state_[2] * end_state_ref_[2] / scale_factor_[2];
  }

  /**
   * @brief 如果存在拖拽点参考线，添加贡献
   *
   * towing_x_ref用于路径形状控制
   * 类似于x_ref，但权重分开存储
   */
  if (has_towing_x_ref_) {
    for (int i = 0; i < n; ++i) {
      q->at(i) += -2.0 * weight_towing_x_ref_vec_.at(i) * towing_x_ref_[i] /
                  scale_factor_[0];
    }
  }
}

/**
 * @brief 获取OSQP求解器默认设置（重载版本）
 *
 * @return OSQPSettings* 求解器设置指针
 *
 * 与基类区别：
 * - 多了一个time_limit设置
 * - 限制优化时间，防止单次求解耗时过长
 *
 * C++语法说明：
 * override关键字明确表示重写基类虚函数
 */
OSQPSettings* PiecewiseJerkPathProblem::SolverDefaultSettings() {
  /**
   * @brief 分配OSQPSettings结构内存
   */
  OSQPSettings* settings =
      reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));

  /**
   * @brief 设置OSQP默认参数
   */
  osqp_set_default_settings(settings);

  /**
   * @brief 启用增强优化（polish）
   * 提高解的精度但增加计算时间
   */
  settings->polish = true;

  /**
   * @brief 设置详细输出标志
   */
  settings->verbose = FLAGS_enable_osqp_debug;

  /**
   * @brief 启用缩放终止条件
   */
  settings->scaled_termination = true;

  /**
   * @brief 设置时间限制
   * FLAGS_path_speed_osqp_setting_time_limit:
   * 路径/速度优化的最大求解时间（秒）
   * 防止单个优化问题占用过多时间
   */
  settings->time_limit = FLAGS_path_speed_osqp_setting_time_limit;

  return settings;
}

/**
 * @brief 命名空间结束标记
 */
}  // namespace planning
}  // namespace apollo
