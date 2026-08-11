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
 * @file piecewise_jerk_problem.cc
 * @brief 分段jerk路径优化问题实现文件
 *
 * 本文件实现了基于OSQP求解器的分段jerk路径优化问题。
 * 分段jerk优化是Apollo路径优化的核心算法，将路径规划问题转化为QP（二次规划）问题。
 *
 * 数学模型：
 * 优化变量：x = [x, dx, ddx]^T，包含位置、速度、加速度
 * 目标函数：min sum(weight_x * x² + weight_dx * dx² + weight_ddx * ddx² + weight_dddx * dddx²)
 * 约束条件：
 *   1. 变量边界约束：x_bounds, dx_bounds, ddx_bounds
 *   2. jerk约束：|dddx| <= dddx_bound
 *   3. 运动学约束：x(i+1) - x(i) = delta_s * dx(i) + ...
 *   4. 初始状态约束：x(0) = x_init, dx(0) = dx_init, ddx(0) = ddx_init
 *
 * OSQP求解器：
 * - OSQP是一个高效的稀疏QP求解器
 * - 使用CSC稀疏矩阵格式存储
 * - 支持等式和不等式约束
 */

#include "modules/planning/planning_base/math/piecewise_jerk/piecewise_jerk_problem.h"

/**
 * @brief Cyber日志系统头文件
 * cyber::common::Log相关宏：AINFO, AERROR, ADEBUG等
 */
#include "cyber/common/log.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
/**
 * @brief GFlags配置参数
 * FLAGS_enable_osqp_debug: OSQP调试开关
 */

namespace apollo {
/**
 * @brief Apollo顶层命名空间
 */
namespace planning {

/**
 * @brief 匿名命名空间
 * 用于定义文件范围内的常量
 * kMaxVariableRange: 变量取值范围上限，防止数值溢出
 */
namespace {
constexpr double kMaxVariableRange = 1.0e10;  /**< 1e10 = 100亿，作为默认边界值 */
}  // namespace

/**
 * @brief PiecewiseJerkProblem构造函数
 *
 * @param num_of_knots 路径结点数量
 * @param delta_s 路径点间距（米）
 * @param x_init 初始状态 [x, dx, ddx]
 *
 * 初始化过程：
 * 1. 检查结点数量的合法性（必须>=2）
 * 2. 初始化各变量边界为默认值（-kMaxVariableRange, kMaxVariableRange）
 * 3. 初始化参考线权重为0
 *
 * C++语法说明：
 * - size_t: 无符号整数类型，用于表示大小和索引
 * - std::array<double, 3>: 固定大小数组，存储初始状态
 * - std::make_pair(): 创建pair对象
 * - resize(): 调整vector大小
 */
PiecewiseJerkProblem::PiecewiseJerkProblem(
    const size_t num_of_knots, const double delta_s,
    const std::array<double, 3>& x_init) {
  /**
   * @brief 检查结点数是否合法
   * CHECK_GE: 运行时断言，确保num_of_knots >= 2
   * _GE后缀表示Greater or Equal
   */
  CHECK_GE(num_of_knots, 2U);

  /**
   * @brief 保存结点数
   */
  num_of_knots_ = num_of_knots;

  /**
   * @brief 保存初始状态
   * x_init_ = x_init: 数组拷贝赋值
   */
  x_init_ = x_init;

  /**
   * @brief 保存路径间距
   */
  delta_s_ = delta_s;

  /**
   * @brief 初始化x（位置）边界
   * resize(num, value): 将vector调整为num个元素，每个元素为value
   * std::make_pair(-kMaxVariableRange, kMaxVariableRange):
   *   创建pair，第一个元素是下界，第二个是上界
   */
  x_bounds_.resize(num_of_knots_,
                   std::make_pair(-kMaxVariableRange, kMaxVariableRange));

  /**
   * @brief 初始化dx（速度）边界
   */
  dx_bounds_.resize(num_of_knots_,
                    std::make_pair(-kMaxVariableRange, kMaxVariableRange));

  /**
   * @brief 初始化ddx（加速度）边界
   */
  ddx_bounds_.resize(num_of_knots_,
                     std::make_pair(-kMaxVariableRange, kMaxVariableRange));

  /**
   * @brief 初始化参考线权重向量
   * std::vector<double>(num_of_knots_, 0.0):
   *   创建num_of_knots_个double元素，初始化为0.0
   */
  weight_x_ref_vec_ = std::vector<double>(num_of_knots_, 0.0);
}

/**
 * @brief 构建QP问题的数据结构
 *
 * @param data 输出：OSQP问题数据指针
 * @return bool 是否成功构建
 *
 * OSQP问题标准形式：
 * min (1/2) x^T P x + q^T x
 * s.t. l <= A x <= u
 *
 * 其中：
 * - P: 核矩阵（ Hessian 矩阵）
 * - q: 线性项向量
 * - A: 约束矩阵
 * - l, u: 约束上下界
 *
 * C++语法说明：
 * - std::vector<c_float>: OSQP使用的float类型向量
 * - std::vector<c_int>: OSQP使用的int类型向量
 * - CopyData(): 拷贝数据到OSQP格式
 */
bool PiecewiseJerkProblem::FormulateProblem(OSQPData* data) {
  /**
   * @brief 计算核矩阵P（目标函数的二次项）
   * P_data: 稀疏矩阵的非零元素值
   * P_indices: 非零元素的列索引
   * P_indptr: 每行非零元素的起始索引（CSC格式）
   */
  std::vector<c_float> P_data;
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;
  CalculateKernel(&P_data, &P_indices, &P_indptr);

  /**
   * @brief 计算约束矩阵A和约束边界
   * A_data: 约束矩阵的非零元素值
   * A_indices: 非零元素的列索引
   * A_indptr: 每行非零元素的起始索引
   * lower_bounds/upper_bounds: 约束的下界和上界
   */
  std::vector<c_float> A_data;
  std::vector<c_int> A_indices;
  std::vector<c_int> A_indptr;
  std::vector<c_float> lower_bounds;
  std::vector<c_float> upper_bounds;
  CalculateAffineConstraint(&A_data, &A_indices, &A_indptr, &lower_bounds,
                            &upper_bounds);

  /**
   * @brief 计算偏移向量q（目标函数的线性项）
   */
  std::vector<c_float> q;
  CalculateOffset(&q);

  /**
   * @brief 检查上下界维度一致性
   * CHECK_EQ: 断言两个值相等
   */
  CHECK_EQ(lower_bounds.size(), upper_bounds.size());

  /**
   * @brief 计算优化变量的总维度
   * 3 * num_of_knots_: 每个结点有x, dx, ddx三个变量
   */
  size_t kernel_dim = 3 * num_of_knots_;

  /**
   * @brief 计算约束的数量
   * = 3N（变量边界）+ 3(N-1)（jerk约束+运动学约束）+ 3（初始状态）
   */
  size_t num_affine_constraint = lower_bounds.size();

  /**
   * @brief 填充OSQPData结构
   * data->n: 优化变量个数
   * data->m: 约束个数
   */
  data->n = kernel_dim;
  data->m = num_affine_constraint;

  /**
   * @brief 创建核矩阵P的CSC格式稀疏矩阵
   * csc_matrix(rows, cols, nnz, data, indices, indptr):
   * - rows/cols: 矩阵行数和列数
   * - nnz: 非零元素个数
   * - CopyData(): 将vector数据拷贝到OSQP所需格式
   */
  data->P = csc_matrix(kernel_dim, kernel_dim, P_data.size(), CopyData(P_data),
                       CopyData(P_indices), CopyData(P_indptr));

  /**
   * @brief 创建目标函数线性项向量q
   */
  data->q = CopyData(q);

  /**
   * @brief 创建约束矩阵A的CSC格式稀疏矩阵
   */
  data->A =
      csc_matrix(num_affine_constraint, kernel_dim, A_data.size(),
                 CopyData(A_data), CopyData(A_indices), CopyData(A_indptr));

  /**
   * @brief 设置约束边界
   * data->l: 约束下界
   * data->u: 约束上界
   */
  data->l = CopyData(lower_bounds);
  data->u = CopyData(upper_bounds);

  /**
   * @brief 检查上下界是否有效
   * lower[i] <= upper[i] 必须成立
   */
  return CheckLowUpperBound(lower_bounds, upper_bounds);
}

/**
 * @brief 执行QP优化
 *
 * @param max_iter 最大迭代次数
 * @return bool 优化是否成功
 *
 * 优化流程：
 * 1. 分配OSQP数据和工作空间
 * 2. 调用FormulateProblem构建问题
 * 3. 设置求解器参数
 * 4. 调用osqp_setup初始化求解器
 * 5. 调用osqp_solve求解
 * 6. 提取结果并缩放
 * 7. 清理资源
 *
 * C++语法说明：
 * - reinterpret_cast<T>: 编译期类型转换，将一种指针类型转换为另一种
 * - c_malloc/c_free: C语言内存分配/释放函数
 * - auto: 自动类型推导
 * - nullptr: C++11空指针常量
 */
bool PiecewiseJerkProblem::Optimize(const int max_iter) {
  /**
   * @brief 分配OSQPData结构内存
   * c_malloc(sizeof(OSQPData)): C风格内存分配
   * reinterpret_cast<OSQPData*>: 将void*转换为OSQPData*
   */
  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));

  /**
   * @brief 构建QP问题
   * 如果失败则清理并返回false
   */
  if (FormulateProblem(data)) {
    FreeData(data);
    return false;
  }

  /**
   * @brief 获取OSQP默认设置
   */
  OSQPSettings* settings = SolverDefaultSettings();

  /**
   * @brief 设置最大迭代次数
   * settings->max_iter: OSQP求解器参数
   */
  settings->max_iter = max_iter;

  /**
   * @brief 创建OSQP工作空间指针
   */
  OSQPWorkspace* osqp_work = nullptr;

  /**
   * @brief 初始化OSQP求解器
   * osqp_setup(): 设置并验证问题数据
   */
  osqp_work = osqp_setup(data, settings);

  /**
   * @brief 执行优化求解
   */
  osqp_solve(osqp_work);

  /**
   * @brief 获取求解状态
   * osqp_work->info->status_val:
   * - 1: 求解成功，已收敛
   * - 2: 求解成功，已达到迭代上限
   * - 负值: 求解失败
   */
  auto status = osqp_work->info->status_val;

  /**
   * @brief 检查求解状态
   * status < 0: 求解器内部失败
   * status != 1 && status != 2: 未成功收敛
   */
  if (status < 0 || (status != 1 && status != 2)) {
    AERROR << "failed optimization status:\t" << osqp_work->info->status;
    osqp_cleanup(osqp_work);
    FreeData(data);
    c_free(settings);  /**< 释放C语言分配的内存 */
    return false;
  }

  /**
   * @brief 检查解是否为空
   */
  if (osqp_work->solution == nullptr) {
    AERROR << "The solution from OSQP is nullptr";
    osqp_cleanup(osqp_work);
    FreeData(data);
    c_free(settings);
    return false;
  }

  /**
   * @brief 调整输出向量大小
   * resize(num_of_knots_): 将x_, dx_, ddx_调整为num_of_knots_个元素
   */
  x_.resize(num_of_knots_);
  dx_.resize(num_of_knots_);
  ddx_.resize(num_of_knots_);

  /**
   * @brief 提取并缩放优化结果
   * osqp_work->solution->x: 原始解向量
   * 变量排列：[x_0, x_1, ..., x_{N-1}, dx_0, dx_1, ..., dx_{N-1}, ddx_0, ddx_1, ..., ddx_{N-1}]
   * scale_factor_: 缩放因子，用于改善数值稳定性
   *
   * 除以scale_factor_是反缩放操作
   */
  for (size_t i = 0; i < num_of_knots_; ++i) {
    /**
     * @brief 提取x（位置）
     * solution->x[i]: 第i个结点的位置
     */
    x_.at(i) = osqp_work->solution->x[i] / scale_factor_[0];

    /**
     * @brief 提取dx（速度）
     * solution->x[i + num_of_knots_]: 第i个结点的速度
     */
    dx_.at(i) = osqp_work->solution->x[i + num_of_knots_] / scale_factor_[1];

    /**
     * @brief 提取ddx（加速度）
     * solution->x[i + 2 * num_of_knots_]: 第i个结点的加速度
     */
    ddx_.at(i) =
        osqp_work->solution->x[i + 2 * num_of_knots_] / scale_factor_[2];
  }

  /**
   * @brief 清理OSQP资源
   * osqp_cleanup(): 释放OSQP工作空间
   * FreeData(): 释放问题数据
   * c_free(): 释放设置结构
   */
  osqp_cleanup(osqp_work);
  FreeData(data);
  c_free(settings);
  return true;
}

/**
 * @brief 计算仿射约束矩阵A
 *
 * @param A_data 输出：稀疏矩阵非零元素值
 * @param A_indices 输出：非零元素列索引
 * @param A_indptr 输出：每行非零元素起始索引
 * @param lower_bounds 输出：约束下界
 * @param upper_bounds 输出：约束上界
 *
 * 约束类型：
 * 1. 变量边界约束（3N个）
 *    x_lower <= x <= x_upper
 *    dx_lower <= dx <= dx_upper
 *    ddx_lower <= ddx <= ddx_upper
 *
 * 2. jerk约束（2*(N-1)个）
 *    dddx_bound.first <= (ddx[i+1] - ddx[i]) / delta_s <= dddx_bound.second
 *
 * 3. 运动学约束（3*(N-1)个）
 *    x(i+1)' - x(i)' - 0.5*delta_s*x(i)'' - 0.5*delta_s*x(i+1)'' = 0
 *    x(i+1) - x(i) - delta_s*x(i)' - ... = 0
 *
 * 4. 初始状态约束（3个）
 *    x(0) = x_init[0], dx(0) = x_init[1], ddx(0) = x_init[2]
 *
 * C++语法说明：
 * - std::vector<std::vector<std::pair<c_int, c_float>>>:
 *   二维稀疏向量，外层vector是变量，内层vector是该变量涉及的约束
 *   pair<index, coefficient>: 约束索引和系数
 * - emplace_back(): 原位构造并添加元素
 */
void PiecewiseJerkProblem::CalculateAffineConstraint(
    std::vector<c_float>* A_data, std::vector<c_int>* A_indices,
    std::vector<c_int>* A_indptr, std::vector<c_float>* lower_bounds,
    std::vector<c_float>* upper_bounds) {
  /**
   * @brief 将num_of_knots_转换为int类型
   * static_cast<T>: 编译期类型转换
   */
  const int n = static_cast<int>(num_of_knots_);

  /**
   * @brief 计算变量总数
   * 3 * n: 每个结点有x, dx, ddx三个变量
   */
  const int num_of_variables = 3 * n;

  /**
   * @brief 计算约束总数
   * = 3N（变量边界）+ 3(N-1)（jerk约束+运动学约束）+ 3（初始状态）
   */
  const int num_of_constraints = num_of_variables + 3 * (n - 1) + 3;

  /**
   * @brief 调整约束边界向量大小
   */
  lower_bounds->resize(num_of_constraints);
  upper_bounds->resize(num_of_constraints);

  /**
   * @brief 创建稀疏约束矩阵的二维向量结构
   * variables[i]: 第i个变量涉及的约束列表
   * 每个约束表示为pair<约束索引, 系数>
   */
  std::vector<std::vector<std::pair<c_int, c_float>>> variables(
      num_of_variables);

  /**
   * @brief 当前约束索引
   */
  int constraint_index = 0;

  /**
   * @brief 第1部分：设置变量边界约束（3N个）
   * x_bounds_, dx_bounds_, ddx_bounds_分别约束x, dx, ddx
   */
  for (int i = 0; i < num_of_variables; ++i) {
    if (i < n) {
      /**
       * @brief x变量的边界约束
       * 变量i对应x[i]
       * 系数为1.0，表示 x[i] 在下界和上界之间
       */
      variables[i].emplace_back(constraint_index, 1.0);
      lower_bounds->at(constraint_index) =
          x_bounds_[i].first * scale_factor_[0];
      upper_bounds->at(constraint_index) =
          x_bounds_[i].second * scale_factor_[0];
    } else if (i < 2 * n) {
      /**
       * @brief dx变量的边界约束
       * 变量i对应dx[i-n]
       */
      variables[i].emplace_back(constraint_index, 1.0);

      lower_bounds->at(constraint_index) =
          dx_bounds_[i - n].first * scale_factor_[1];
      upper_bounds->at(constraint_index) =
          dx_bounds_[i - n].second * scale_factor_[1];
    } else {
      /**
       * @brief ddx变量的边界约束
       * 变量i对应ddx[i-2n]
       */
      variables[i].emplace_back(constraint_index, 1.0);

      lower_bounds->at(constraint_index) =
          ddx_bounds_[i - 2 * n].first * scale_factor_[2];
      upper_bounds->at(constraint_index) =
          ddx_bounds_[i - 2 * n].second * scale_factor_[2];
    }
    ++constraint_index;  /**< 前置递增，先加1再使用 */
  }

  /**
   * @brief 检查约束索引是否正确
   */
  CHECK_EQ(constraint_index, num_of_variables);

  /**
   * @brief 第2部分：设置jerk约束（2*(N-1)个）
   * jerk = (ddx[i+1] - ddx[i]) / delta_s
   * 约束：dddx_bound.first <= jerk <= dddx_bound.second
   *
   * 矩阵形式：
   * -1 * ddx[i] + 1 * ddx[i+1] 在 [ddx_bound.first * delta_s, ddx_bound.second * delta_s] 之间
   */
  for (int i = 0; i + 1 < n; ++i) {
    /**
     * @brief ddx[i]的系数为-1
     */
    variables[2 * n + i].emplace_back(constraint_index, -1.0);

    /**
     * @brief ddx[i+1]的系数为+1
     */
    variables[2 * n + i + 1].emplace_back(constraint_index, 1.0);

    /**
     * @brief 设置约束边界
     * 乘以delta_s_是因为约束是关于ddx而非jerk
     */
    lower_bounds->at(constraint_index) =
        dddx_bound_.first * delta_s_ * scale_factor_[2];
    upper_bounds->at(constraint_index) =
        dddx_bound_.second * delta_s_ * scale_factor_[2];
    ++constraint_index;
  }

  /**
   * @brief 第3部分：设置速度连续性约束（2*(N-1)个）
   * 公式：x(i+1)' - x(i)' - 0.5*delta_s*x(i)'' - 0.5*delta_s*x(i+1)'' = 0
   *
   * 物理意义：速度的积分等于位移
   * x(i+1) - x(i) = ∫_{s_i}^{s_{i+1}} dx ds
   *              ≈ 0.5*delta_s*dx(i) + 0.5*delta_s*dx(i+1)  (梯形积分)
   *
   * 重新整理得到上述等式
   */
  for (int i = 0; i + 1 < n; ++i) {
    /**
     * @brief dx[i]的系数
     * 注意这里乘以了scale_factor_[2]以统一量纲
     */
    variables[n + i].emplace_back(constraint_index, -1.0 * scale_factor_[2]);

    /**
     * @brief dx[i+1]的系数
     */
    variables[n + i + 1].emplace_back(constraint_index, 1.0 * scale_factor_[2]);

    /**
     * @brief ddx[i]的系数
     * -0.5 * delta_s * scale_factor_[1]
     */
    variables[2 * n + i].emplace_back(constraint_index,
                                      -0.5 * delta_s_ * scale_factor_[1]);

    /**
     * @brief ddx[i+1]的系数
     */
    variables[2 * n + i + 1].emplace_back(constraint_index,
                                          -0.5 * delta_s_ * scale_factor_[1]);

    /**
     * @brief 等式约束，上下界相等
     */
    lower_bounds->at(constraint_index) = 0.0;
    upper_bounds->at(constraint_index) = 0.0;
    ++constraint_index;
  }

  /**
   * @brief 第4部分：设置位移连续性约束（3*(N-1)个）
   * 公式：x(i+1) - x(i) - delta_s*x(i)' - 1/3*delta_s²*x(i)'' - 1/6*delta_s²*x(i+1)'' = 0
   *
   * 物理意义：位移的积分
   * x(i+1) - x(i) = ∫_{s_i}^{s_{i+1}} x ds
   * 利用Simpson公式或积分平均近似
   *
   * @note 代码中的系数可能需要核实，这里使用二次近似
   */
  auto delta_s_sq_ = delta_s_ * delta_s_;  /**< delta_s的平方 */
  for (int i = 0; i + 1 < n; ++i) {
    /**
     * @brief x[i]的系数
     */
    variables[i].emplace_back(constraint_index,
                              -1.0 * scale_factor_[1] * scale_factor_[2]);

    /**
     * @brief x[i+1]的系数
     */
    variables[i + 1].emplace_back(constraint_index,
                                  1.0 * scale_factor_[1] * scale_factor_[2]);

    /**
     * @brief dx[i]的系数
     * -delta_s * scale_factor_[0] * scale_factor_[2]
     */
    variables[n + i].emplace_back(
        constraint_index, -delta_s_ * scale_factor_[0] * scale_factor_[2]);

    /**
     * @brief ddx[i]的系数
     * -delta_s²/3 * scale_factor_[0] * scale_factor_[1]
     */
    variables[2 * n + i].emplace_back(
        constraint_index,
        -delta_s_sq_ / 3.0 * scale_factor_[0] * scale_factor_[1]);

    /**
     * @brief ddx[i+1]的系数
     * -delta_s²/6 * scale_factor_[0] * scale_factor_[1]
     */
    variables[2 * n + i + 1].emplace_back(
        constraint_index,
        -delta_s_sq_ / 6.0 * scale_factor_[0] * scale_factor_[1]);

    /**
     * @brief 等式约束
     */
    lower_bounds->at(constraint_index) = 0.0;
    upper_bounds->at(constraint_index) = 0.0;
    ++constraint_index;
  }

  /**
   * @brief 第5部分：设置初始状态约束（3个）
   * 约束x(0) = x_init[0], dx(0) = x_init[1], ddx(0) = x_init[2]
   */

  /**
   * @brief x[0] = x_init[0]
   */
  variables[0].emplace_back(constraint_index, 1.0);
  lower_bounds->at(constraint_index) = x_init_[0] * scale_factor_[0];
  upper_bounds->at(constraint_index) = x_init_[0] * scale_factor_[0];
  ++constraint_index;

  /**
   * @brief dx[0] = x_init[1]
   */
  variables[n].emplace_back(constraint_index, 1.0);
  lower_bounds->at(constraint_index) = x_init_[1] * scale_factor_[1];
  upper_bounds->at(constraint_index) = x_init_[1] * scale_factor_[1];
  ++constraint_index;

  /**
   * @brief ddx[0] = x_init[2]
   */
  variables[2 * n].emplace_back(constraint_index, 1.0);
  lower_bounds->at(constraint_index) = x_init_[2] * scale_factor_[2];
  upper_bounds->at(constraint_index) = x_init_[2] * scale_factor_[2];
  ++constraint_index;

  /**
   * @brief 最终检查约束总数
   */
  CHECK_EQ(constraint_index, num_of_constraints);

  /**
   * @brief 将稀疏约束结构转换为CSC格式
   * CSC (Compressed Sparse Column) 格式：
   * - A_data: 非零元素值，按列存储
   * - A_indices: 非零元素行索引
   * - A_indptr: 每列非零元素起始索引
   */
  int ind_p = 0;  /**< 当前处理的位置索引 */
  for (int i = 0; i < num_of_variables; ++i) {
    /**
     * @brief 记录第i列的起始索引
     */
    A_indptr->push_back(ind_p);

    /**
     * @brief 遍历第i个变量的所有非零约束
     * const auto& variable_nz : variables[i]:
     *   范围for循环，遍历variables[i]中的每个元素
     */
    for (const auto& variable_nz : variables[i]) {
      /**
       * @brief 添加系数值
       */
      A_data->push_back(variable_nz.second);

      /**
       * @brief 添加约束索引（行索引）
       */
      A_indices->push_back(variable_nz.first);
      ++ind_p;  /**< 更新位置索引 */
    }
  }

  /**
   * @brief 添加最后一列的结束索引
   * 这是CSC格式的要求
   * 参考OSQP源码：https://github.com/oxfordcontrol/osqp/blob/master/src/cs.c#L255
   */
  A_indptr->push_back(ind_p);
}

/**
 * @brief 获取OSQP求解器默认设置
 *
 * @return OSQPSettings* 求解器设置指针
 *
 * 主要设置项：
 * - polish: 是否使用增强优化（提高解的精度）
 * - verbose: 是否输出详细信息
 * - scaled_termination: 是否使用缩放终止条件
 *
 * C++语法说明：
 * - reinterpret_cast<T>: 类型转换，用于将一种指针类型转换为另一种
 * - c_malloc: C语言内存分配
 */
OSQPSettings* PiecewiseJerkProblem::SolverDefaultSettings() {
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
   * polish可以提高解的精度，但会增加计算时间
   */
  settings->polish = true;

  /**
   * @brief 设置详细输出标志
   * FLAGS_enable_osqp_debug: 从GFlags读取的调试开关
   */
  settings->verbose = FLAGS_enable_osqp_debug;

  /**
   * @brief 启用缩放终止条件
   * 使用缩放后的残差判断收敛
   */
  settings->scaled_termination = true;

  return settings;
}

/**
 * @brief 设置x（位置）边界
 *
 * @param x_bounds 边界向量，每个元素是pair<下界, 上界>
 *
 * C++语法说明：
 * - std::move(x_bounds): 移动语义，避免不必要的数据拷贝
 *   将x_bounds的资源转移给x_bounds_，之后x_bounds变为空
 */
void PiecewiseJerkProblem::set_x_bounds(
    std::vector<std::pair<double, double>> x_bounds) {
  CHECK_EQ(x_bounds.size(), num_of_knots_);  /**< 检查边界数量 */
  x_bounds_ = std::move(x_bounds);  /**< 移动赋值 */
}

/**
 * @brief 设置dx（速度）边界
 */
void PiecewiseJerkProblem::set_dx_bounds(
    std::vector<std::pair<double, double>> dx_bounds) {
  CHECK_EQ(dx_bounds.size(), num_of_knots_);
  dx_bounds_ = std::move(dx_bounds);
}

/**
 * @brief 设置ddx（加速度）边界
 */
void PiecewiseJerkProblem::set_ddx_bounds(
    std::vector<std::pair<double, double>> ddx_bounds) {
  CHECK_EQ(ddx_bounds.size(), num_of_knots_);
  ddx_bounds_ = std::move(ddx_bounds);
}

/**
 * @brief 设置统一的x边界（所有结点相同）
 *
 * @param x_lower_bound 统一下界
 * @param x_upper_bound 统一上界
 *
 * for循环遍历所有边界，将每个边界设置为相同的值
 */
void PiecewiseJerkProblem::set_x_bounds(const double x_lower_bound,
                                        const double x_upper_bound) {
  for (auto& x : x_bounds_) {  /**< auto&: 引用，避免拷贝 */
    x.first = x_lower_bound;
    x.second = x_upper_bound;
  }
}

/**
 * @brief 设置统一的dx边界
 */
void PiecewiseJerkProblem::set_dx_bounds(const double dx_lower_bound,
                                         const double dx_upper_bound) {
  for (auto& x : dx_bounds_) {
    x.first = dx_lower_bound;
    x.second = dx_upper_bound;
  }
}

/**
 * @brief 设置统一的ddx边界
 */
void PiecewiseJerkProblem::set_ddx_bounds(const double ddx_lower_bound,
                                          const double ddx_upper_bound) {
  for (auto& x : ddx_bounds_) {
    x.first = ddx_lower_bound;
    x.second = ddx_upper_bound;
  }
}

/**
 * @brief 设置统一的参考线权重
 *
 * @param weight_x_ref 统一权重值
 * @param x_ref 参考线位置序列
 *
 * 功能说明：
 * 所有结点使用相同的参考线权重
 */
void PiecewiseJerkProblem::set_x_ref(const double weight_x_ref,
                                     std::vector<double> x_ref) {
  CHECK_EQ(x_ref.size(), num_of_knots_);

  /**
   * @brief 保存权重（标量）
   */
  weight_x_ref_ = weight_x_ref;

  /**
   * @brief 创建均匀权重向量
   * std::vector<double>(num_of_knots_, weight_x_ref):
   *   创建num_of_knots_个元素，每个初始化为weight_x_ref
   */
  weight_x_ref_vec_ = std::vector<double>(num_of_knots_, weight_x_ref);

  /**
   * @brief 移动保存参考线
   */
  x_ref_ = std::move(x_ref);

  /**
   * @brief 标记存在参考线
   */
  has_x_ref_ = true;
}

/**
 * @brief 设置分段的参考线权重（每个结点权重可以不同）
 *
 * @param weight_x_ref_vec 每个结点的权重向量
 * @param x_ref 参考线位置序列
 *
 * 与上一函数的区别：
 * - 权重向量：每个结点可以有不同权重
 * - 用于更精细的优化控制
 */
void PiecewiseJerkProblem::set_x_ref(std::vector<double> weight_x_ref_vec,
                                     std::vector<double> x_ref) {
  CHECK_EQ(x_ref.size(), num_of_knots_);
  CHECK_EQ(weight_x_ref_vec.size(), num_of_knots_);

  /**
   * @brief 移动赋值，避免拷贝
   */
  weight_x_ref_vec_ = std::move(weight_x_ref_vec);
  x_ref_ = std::move(x_ref);
  has_x_ref_ = true;
}

/**
 * @brief 设置统一的拖拽点参考线
 *
 * @param weight_towing_x_ref 统一权重
 * @param towing_x_ref 拖拽点参考序列
 *
 * 拖拽点参考线用于路径形状控制
 */
void PiecewiseJerkProblem::set_towing_x_ref(const double weight_towing_x_ref,
                                            std::vector<double> towing_x_ref) {
  CHECK_EQ(towing_x_ref.size(), num_of_knots_);

  /**
   * @brief 创建统一权重向量
   */
  weight_towing_x_ref_vec_ =
      std::vector<double>(num_of_knots_, weight_towing_x_ref);

  /**
   * @brief 移动保存参考线
   */
  towing_x_ref_ = std::move(towing_x_ref);

  /**
   * @brief 标记存在拖拽参考
   */
  has_towing_x_ref_ = true;
}

/**
 * @brief 设置分段的拖拽点参考线权重
 */
void PiecewiseJerkProblem::set_towing_x_ref(
    std::vector<double> weight_towing_x_ref_vec,
    std::vector<double> towing_x_ref) {
  CHECK_EQ(towing_x_ref.size(), num_of_knots_);
  CHECK_EQ(weight_towing_x_ref_vec.size(), num_of_knots_);

  weight_towing_x_ref_vec_ = std::move(weight_towing_x_ref_vec);
  towing_x_ref_ = std::move(towing_x_ref);
  has_towing_x_ref_ = true;
}

/**
 * @brief 设置终点状态约束
 *
 * @param weight_end_state 终点状态权重 [w_x, w_dx, w_ddx]
 * @param end_state_ref 终点状态目标值 [x, dx, ddx]
 *
 * 功能说明：
 * 对终点状态添加软约束，使其接近目标值
 */
void PiecewiseJerkProblem::set_end_state_ref(
    const std::array<double, 3>& weight_end_state,
    const std::array<double, 3>& end_state_ref) {
  weight_end_state_ = weight_end_state;
  end_state_ref_ = end_state_ref;
  has_end_state_ref_ = true;
}

/**
 * @brief 释放OSQP数据内存
 *
 * @param data OSQP数据指针
 *
 * 注意：
 * - delete[] 用于释放数组
 * - 需要分别释放P、q、A、l、u等数组
 *
 * C++语法说明：
 * - delete[]: 释放数组内存
 * - data->P->i: 结构体指针的成员访问
 */
void PiecewiseJerkProblem::FreeData(OSQPData* data) {
  /**
   * @brief 释放向量q, l, u
   */
  delete[] data->q;
  delete[] data->l;
  delete[] data->u;

  /**
   * @brief 释放核矩阵P的 CSC 格式数据
   * CSC格式使用三个数组：i（行索引）, p（列指针）, x（数据值）
   */
  delete[] data->P->i;
  delete[] data->P->p;
  delete[] data->P->x;

  /**
   * @brief 释放约束矩阵A的 CSC 格式数据
   */
  delete[] data->A->i;
  delete[] data->A->p;
  delete[] data->A->x;
}

/**
 * @brief 检查约束上下界是否有效
 *
 * @param lower 约束下界向量
 * @param upper 约束上界向量
 * @return bool 如果存在lower[i] > upper[i]返回true，否则返回false
 *
 * 有效约束必须满足：lower <= upper
 */
bool PiecewiseJerkProblem::CheckLowUpperBound(
    const std::vector<c_float>& lower, const std::vector<c_float>& upper) {
  /**
   * @brief 遍历检查每个约束边界
   */
  for (size_t i = 0; i < lower.size(); i++) {
    if (lower[i] > upper[i]) {
      return true;  /**< 发现无效边界 */
    }
  }
  return false;  /**< 所有边界都有效 */
}

/**
 * @brief 命名空间结束标记
 */
}  // namespace planning
}  // namespace apollo
