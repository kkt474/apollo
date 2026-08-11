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
 * @file piecewise_jerk_fallback_speed.cc
 * @brief 分段加加速度速度优化器实现文件
 *
 * 本文件实现了基于分段加加速度(jerk)优化的速度规划算法
 * 使用数值优化方法生成平滑的速度曲线
 *
 * 功能说明：
 * 1. 基于二次规划(QP)的速度曲线优化
 * 2. 考虑ST边界约束（障碍物避让）
 * 3. 考虑速度、加速度、jerk约束
 * 4. 支持回退(fallback)机制
 *
 * 算法原理：
 * - 将速度规划问题建模为分段加加速度优化问题
 * - 目标函数：最小化jerk的平方和，保证乘坐舒适性
 * - 约束条件：ST边界、速度限制、加减速限制
 * - 使用OSQP或其他QP求解器求解
 *
 * 变量定义：
 * - s: 沿路径的距离
 * - ds/dx: 速度 (v = ds/dt)
 * - dds/ddx: 加速度 (a = dv/dt)
 * - ddds/dddx: 加加速度 (jerk = da/dt)
 *
 * 相关C++语法说明：
 * - std::array<T, N>: 固定大小数组，存储[s, v, a]状态
 * - std::vector: 动态数组，存储优化变量序列
 * - std::move: 移动语义，避免大数据拷贝
 **/

/**
 * @brief 标准库头文件
 *
 * <algorithm>: 提供std::min, std::max, std::fmin, std::fabs等算法
 * <string>: 提供std::string字符串类型
 * <utility>: 提供std::pair
 * <vector>: 提供std::vector动态数组
 */
#include <algorithm>

#include <string>
#include <utility>
#include <vector>

/**
 * @brief PNC点protobuf消息头文件
 * 包含PathPoint、SpeedPoint、TrajectoryPoint等数据结构
 */
#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"

/**
 * @brief 车辆状态提供者头文件
 * VehicleStateProvider: 获取当前车辆状态（速度、加速度、档位等）
 */
#include "modules/common/vehicle_state/vehicle_state_provider.h"

/**
 * @brief 速度剖面生成器头文件
 * SpeedProfileGenerator: 速度剖面生成工具
 */
#include "modules/planning/planning_base/common/speed_profile_generator.h"

/**
 * @brief ST图数据头文件
 * StGraphData: 存储ST图相关数据的数据结构
 */
#include "modules/planning/planning_base/common/st_graph_data.h"

/**
 * @brief 调试信息打印工具头文件
 * PrintCurves: 用于记录曲线数据，方便调试
 */
#include "modules/planning/planning_base/common/util/print_debug_info.h"

/**
 * @brief 规划模块GFlags头文件
 * FLAGS_*: 从配置文件获取的参数
 */
#include "modules/planning/planning_base/gflags/planning_gflags.h"

/**
 * @brief 分段加加速度速度问题头文件
 * PiecewiseJerkSpeedProblem: 速度优化问题的数学建模
 */
#include "modules/planning/planning_base/math/piecewise_jerk/piecewise_jerk_speed_problem.h"

/**
 * @brief 本任务头文件
 */
#include "modules/planning/tasks/piecewise_jerk_speed/piecewise_jerk_speed_optimizer.h"

/**
 * @brief Apollo命名空间开始
 */
namespace apollo {

/**
 * @brief 规划模块命名空间
 */
namespace planning {

/**
 * @brief 使用别名声明，简化类型引用
 *
 * C++语法说明：
 * using声明：引入其他命名空间的类型到当前作用域
 */
using apollo::common::ErrorCode;     /**< 错误码类型 */
using apollo::common::PathPoint;      /**< 路径点类型 */
using apollo::common::SpeedPoint;      /**< 速度点类型 */
using apollo::common::Status;         /**< 状态类型 */
using apollo::common::TrajectoryPoint;  /**< 轨迹点类型 */

/**
 * @brief 分段加加速度速度优化器初始化函数
 *
 * @param config_dir 配置文件目录路径
 * @param name 任务名称
 * @param injector 依赖注入器指针
 * @return bool 初始化是否成功
 *
 * 功能说明：
 * 1. 调用父类SpeedOptimizer的初始化函数
 * 2. 加载PiecewiseJerkSpeedOptimizerConfig配置
 *
 * C++语法说明：
 * - std::shared_ptr<DependencyInjector>: 共享所有权的智能指针
 * - SpeedOptimizer::Init: 父类的初始化函数
 * - SpeedOptimizer::LoadConfig<T>: 模板函数，从配置文件加载配置
 */
bool PiecewiseJerkSpeedOptimizer::Init(
    const std::string& config_dir, const std::string& name,
    const std::shared_ptr<DependencyInjector>& injector) {
  /**
   * @brief 调用父类SpeedOptimizer的Init进行基础初始化
   *
   * 执行基础初始化操作：
   * - 保存配置目录、名称、依赖注入器
   * - 初始化任务基类成员
   */
  if (!SpeedOptimizer::Init(config_dir, name, injector)) {
    return false;  /**< 父类初始化失败，返回false */
  }

  /**
   * @brief 加载本任务的配置
   *
   * SpeedOptimizer::LoadConfig<T>():
   *   模板成员函数，用于从配置目录加载配置
   *   T指定了配置的类型，这里是PiecewiseJerkSpeedOptimizerConfig
   *
   * 配置内容可能包括：
   * - 加速度权重 (acc_weight)
   * - jerk权重 (jerk_weight)
   * - 曲率惩罚权重 (kappa_penalty_weight)
   * - 参考速度权重 (ref_v_weight)
   */
  return SpeedOptimizer::LoadConfig<PiecewiseJerkSpeedOptimizerConfig>(
      &config_);
}

/**
 * @brief 速度优化主函数
 *
 * @param path_data 路径数据
 * @param init_point 规划起点
 * @param speed_data 输出：优化后的速度数据
 * @return Status 处理状态
 *
 * 功能说明：
 * 分段加加速度速度优化的主入口
 * 1. 检查是否到达目的地
 * 2. 更新ST边界约束
 * 3. 更新速度边界和参考速度
 * 4. 构建并求解QP问题
 * 5. 提取优化结果
 *
 * 算法流程：
 * 1. 检查是否到达终点，如果是则直接返回
 * 2. 获取初始状态 [s, v, a]
 * 3. 处理倒车档位特殊情况
 * 4. 构建时间网格
 * 5. 根据ST边界计算每个时刻的s上下界
 * 6. 计算速度边界和参考速度
 * 7. 调整初始状态确保满足约束
 * 8. 构建PiecewiseJerkSpeedProblem
 * 9. 求解QP问题
 * 10. 提取并返回速度曲线
 *
 * C++语法说明：
 * - const PathData&: 常量引用
 * - const TrajectoryPoint&: 常量引用
 * - SpeedData* const speed_data: 指向常量的指针（指针本身是常量）
 */
Status PiecewiseJerkSpeedOptimizer::Process(const PathData& path_data,
                                            const TrajectoryPoint& init_point,
                                            SpeedData* const speed_data) {
  /**
   * @brief 检查是否到达目的地
   *
   * reference_line_info_->ReachedDestination():
   *   判断规划是否已经到达目的地
   * 如果到达，则不需要继续优化
   */
  if (reference_line_info_->ReachedDestination()) {
    return Status::OK();  /**< 已到达目的地，直接返回成功 */
  }

  /**
   * @brief 断言检查：确保输出参数非空
   *
   * ACHECK: Apollo断言，仅在调试模式下生效
   * 检查speed_data指针是否有效
   */
  ACHECK(speed_data != nullptr);

  /**
   * @brief 保存参考速度数据副本
   *
   * SpeedData reference_speed_data = *speed_data:
   *   复制一份原始速度数据作为参考
   * 用于后续计算速度限制和参考轨迹
   */
  SpeedData reference_speed_data = *speed_data;

  /**
   * @brief 检查路径数据是否为空
   *
   * path_data.discretized_path().empty():
   *   检查离散化路径是否为空
   * 如果为空，说明规划失败
   */
  if (path_data.discretized_path().empty()) {
    const std::string msg = "Empty path data";  /**< 错误信息 */
    AERROR << msg;  /**< 记录错误日志 */
    return Status(ErrorCode::PLANNING_ERROR, msg);  /**< 返回规划错误状态 */
  }

  /**
   * @brief 获取可修改的ST图数据引用
   *
   * reference_line_info_->mutable_st_graph_data():
   *   获取ST图数据的可修改指针
   * 用于后续修改ST图数据
   */
  StGraphData& st_graph_data = *reference_line_info_->mutable_st_graph_data();

  /**
   * @brief 创建调试曲线记录器
   *
   * PrintCurves: 用于记录优化过程中的各种曲线数据
   * 方便后续分析和调试
   */
  PrintCurves print_debug;

  /**
   * @brief 获取车辆参数
   *
   * common::VehicleConfigHelper::GetConfig().vehicle_param():
   *   获取车辆配置单例，然后获取车辆参数
   * 包含：max_acceleration, max_deceleration, length, width等
   */
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();

  /**
   * @brief 初始化状态向量
   *
   * std::array<double, 3>:
   *   固定大小数组，存储三个元素
   * init_s = {s, v, a} = {0.0, 初始速度, 初始加速度}
   *
   * st_graph_data.init_point():
   *   获取ST图的初始点
   * .v() 和 .a():
   *   获取该点的速度和加速度
   */
  std::array<double, 3> init_s = {0.0, st_graph_data.init_point().v(),
                                  st_graph_data.init_point().a()};

  /**
   * @brief 获取车辆状态
   *
   * frame_->vehicle_state():
   *   获取当前帧的车辆状态
   * 包含：位置、速度、加速度、档位等信息
   */
  const auto& vehicle_state = frame_->vehicle_state();

  /**
   * @brief 处理倒车档位
   *
   * canbus::Chassis::GEAR_REVERSE:
   *   倒车档位
   * 如果当前是倒车模式，需要特殊处理
   */
  if (vehicle_state.gear() == canbus::Chassis::GEAR_REVERSE) {
    /**
     * @brief 倒车时速度取反并与0比较
     *
     * std::max(-init_s[1], 0.0):
     *   如果原速度是负的（向后），取绝对值
     *   同时确保速度不为负
     */
    init_s[1] = std::max(-init_s[1], 0.0);

    /**
     * @brief 倒车时加速度也取反
     * 确保加速度方向与速度方向一致
     */
    init_s[2] = -init_s[2];

    /**
     * @brief 打印调试信息
     */
    AINFO << "transfer reverse speed" << init_s[0] << "," << init_s[1] << ","
          << init_s[2];
  }

  /**
   * @brief 设置时间参数
   *
   * delta_t: 时间步长，0.1秒
   * 用于将连续时间离散化为网格点
   */
  double delta_t = 0.1;

  /**
   * @brief 获取路径长度和时间范围
   *
   * st_graph_data.path_length():
   *   获取路径的总长度
   * st_graph_data.total_time_by_conf():
   *   获取配置的总规划时间
   */
  double total_length = st_graph_data.path_length();
  double total_time = st_graph_data.total_time_by_conf();

  /**
   * @brief 计算网格点数量
   *
   * num_of_knots = total_time / delta_t + 1
   * +1是因为包括时间0点
   * static_cast<int>: 显式类型转换
   */
  int num_of_knots = static_cast<int>(total_time / delta_t) + 1;

  /**
   * @brief 记录初始状态到调试曲线
   */
  print_debug.AddPoint("optimize_st_curve", 0, init_s[0]);  /**< 初始位置s=0 */
  print_debug.AddPoint("optimize_vt_curve", 0, init_s[1]);  /**< 初始速度 */
  print_debug.AddPoint("optimize_at_curve", 0, init_s[2]);   /**< 初始加速度 */

  /**
   * @brief 更新ST边界
   *
   * 根据障碍物的ST边界，计算每个时刻s的上下界
   * s_bounds: 存储每个时刻的[s_lower, s_upper]
   */
  const double kEpsilon = 0.01;  /**< 极小值，用于避免数值问题 */

  /**
   * @brief 初始化s边界向量
   *
   * std::vector<std::pair<double, double>>:
   *   存储每个时刻的(s下界, s上界)
   */
  std::vector<std::pair<double, double>> s_bounds;

  /**
   * @brief 遍历所有时刻
   *
   * for循环：
   * - i从0到num_of_knots-1
   * - 计算每个时刻的s边界
   */
  for (int i = 0; i < num_of_knots; ++i) {
    /**
     * @brief 计算当前时刻
     *
     * curr_t = i * delta_t
     */
    double curr_t = i * delta_t;

    /**
     * @brief 初始化s边界
     *
     * s_lower_bound: 下界，初始为0（路径起点）
     * s_upper_bound: 上界，初始为总长度（路径终点）
     */
    double s_lower_bound = 0.0;
    double s_upper_bound = total_length;

    /**
     * @brief 遍历所有ST边界（障碍物）
     *
     * st_graph_data.st_boundaries():
     *   获取所有障碍物的ST边界列表
     */
    for (const STBoundary* boundary : st_graph_data.st_boundaries()) {
      /**
       * @brief 初始化边界值
       */
      double s_lower = 0.0;
      double s_upper = 0.0;

      /**
       * @brief 获取当前时刻的S边界
       *
       * GetUnblockSRange(curr_t, &s_upper, &s_lower):
       *   获取时刻t时不被阻塞的S范围
       *   s_upper: 可通过的最大S
       *   s_lower: 可通过的最小S
       * 返回false表示该时刻障碍物不在考虑范围内
       */
      if (!boundary->GetUnblockSRange(curr_t, &s_upper, &s_lower)) {
        continue;  /**< 障碍物不在范围内，跳过 */
      }

      /**
       * @brief 根据边界类型更新s边界
       *
       * switch-case:
       *   多分支选择，根据障碍物决策类型调整边界
       *
       * C++语法说明：
       * boundary->boundary_type():
       *   ->运算符，通过指针访问成员函数
       */
      switch (boundary->boundary_type()) {
        /**
         * @brief STOP和YIELD类型：限制上界
         *
         * 自车必须在障碍物后方
         * s_upper取当前值和障碍物边界的较小值
         */
        case STBoundary::BoundaryType::STOP:
        case STBoundary::BoundaryType::YIELD:
          s_upper_bound = std::fmin(s_upper_bound, s_upper);
          break;

        /**
         * @brief FOLLOW类型：限制上界
         *
         * 跟车场景，同样需要限制上界
         * TODO注释：决策端的跟车缓冲区需要统一
         */
        case STBoundary::BoundaryType::FOLLOW:
          s_upper_bound = std::fmin(s_upper_bound, s_upper);
          break;

        /**
         * @brief OVERTAKE类型：限制下界
         *
         * 超车场景，自车必须在障碍物前方
         * s_lower取当前值和障碍物边界的较大值
         */
        case STBoundary::BoundaryType::OVERTAKE:
          s_lower_bound = std::fmax(s_lower_bound, s_lower);
          break;

        /**
         * @brief 其他类型：不做处理
         */
        default:
          break;
      }
    }

    /**
     * @brief 确保下界小于上界（加一个极小值）
     *
     * s_upper_bound = max(s_upper_bound, s_lower_bound + epsilon)
     * 确保上界至少比下界大一个极小值
     */
    s_upper_bound = std::fmax(s_upper_bound, s_lower_bound + kEpsilon);

    /**
     * @brief 记录调试曲线
     */
    print_debug.AddPoint("st_bounds_lower", curr_t, s_lower_bound);
    print_debug.AddPoint("st_bounds_upper", curr_t, s_upper_bound);

    /**
     * @brief 检查边界有效性
     *
     * 如果下界大于上界，说明约束冲突
     * 这是规划失败的标志
     */
    if (s_lower_bound > s_upper_bound) {
      const std::string msg =
          "s_lower_bound larger than s_upper_bound on STGraph";
      AERROR << msg;  /**< 记录错误日志 */

      /**
       * @brief 清理并返回错误
       */
      speed_data->clear();  /**< 清空速度数据 */
      print_debug.PrintToLog();  /**< 打印调试曲线 */
      return Status(ErrorCode::PLANNING_ERROR, msg);  /**< 返回错误状态 */
    }

    /**
     * @brief 添加当前时刻的s边界
     *
     * emplace_back:
     *   直接构造pair并添加到向量
     *   避免拷贝构造
     */
    s_bounds.emplace_back(s_lower_bound, s_upper_bound);
  }

  /**
   * @brief 更新速度边界和参考s
   *
   * x_ref: 参考s轨迹
   * dx_ref: 参考速度
   * dx_ref_weight: 参考速度权重
   * penalty_dx: 曲率惩罚
   * s_dot_bounds: 速度边界
   */
  std::vector<double> x_ref(num_of_knots, total_length);  /**< 初始化为总长度 */
  std::vector<double> dx_ref(num_of_knots,
                             reference_line_info_->GetCruiseSpeed());  /**< 初始化为巡航速度 */
  std::vector<double> dx_ref_weight(num_of_knots, config_.ref_v_weight());  /**< 初始化为配置的权重 */
  std::vector<double> penalty_dx;  /**< 曲率惩罚向量 */
  std::vector<std::pair<double, double>> s_dot_bounds;  /**< 速度边界向量 */

  /**
   * @brief 获取速度限制
   *
   * st_graph_data.speed_limit():
   *   获取沿路径的速度限制曲线
   */
  const SpeedLimit& speed_limit = st_graph_data.speed_limit();

  /**
   * @brief 遍历所有时刻
   */
  for (int i = 0; i < num_of_knots; ++i) {
    /**
     * @brief 计算当前时刻
     */
    double curr_t = i * delta_t;

    /**
     * @brief 获取路径s
     *
     * reference_speed_data.EvaluateByTime(curr_t, &sp):
     *   根据时间获取参考速度曲线上对应的点
     * sp.s(): 获取该点的s坐标
     */
    SpeedPoint sp;
    reference_speed_data.EvaluateByTime(curr_t, &sp);
    const double path_s = sp.s();

    /**
     * @brief 更新参考s
     */
    x_ref[i] = path_s;

    /**
     * @brief 获取曲率并计算惩罚
     *
     * path_data.GetPathPointWithPathS(path_s):
     *   根据s坐标获取路径上的点
     * .kappa():
     *   获取该点的曲率
     *
     * penalty_dx = |kappa| * kappa_penalty_weight
     * 曲率越大，惩罚越大（鼓励小曲率行驶）
     */
    PathPoint path_point = path_data.GetPathPointWithPathS(path_s);
    penalty_dx.push_back(std::fabs(path_point.kappa()) *
                         config_.kappa_penalty_weight());

    /**
     * @brief 计算速度边界
     *
     * v_lower_bound: 下界为0（不能倒车）
     * v_upper_bound: 上界为全局最大速度限制
     */
    const double v_lower_bound = 0.0;
    double v_upper_bound = FLAGS_planning_upper_speed_limit;

    /**
     * @brief 限制速度上界
     *
     * speed_limit.GetSpeedLimitByS(path_s):
     *   根据s坐标获取该点的限速
     * 取全局限速和道路限速的较小值
     */
    v_upper_bound =
        std::fmin(speed_limit.GetSpeedLimitByS(path_s), v_upper_bound);

    /**
     * @brief 更新参考速度
     *
     * dx_ref[i] = min(v_upper_bound, dx_ref[i])
     * 确保参考速度不超过限速
     */
    dx_ref[i] = std::fmin(v_upper_bound, dx_ref[i]);

    /**
     * @brief 添加速度边界
     */
    s_dot_bounds.emplace_back(v_lower_bound, std::fmax(v_upper_bound, 0.0));

    /**
     * @brief 记录调试曲线
     */
    print_debug.AddPoint("st_reference_line", curr_t, x_ref[i]);
    print_debug.AddPoint("st_penalty_dx", curr_t, penalty_dx.back());
    print_debug.AddPoint("vt_reference_line", curr_t, dx_ref[i]);
    print_debug.AddPoint("vt_weighting", curr_t, dx_ref_weight[i]);
    print_debug.AddPoint("vt_boundary_lower", curr_t, v_lower_bound);
    print_debug.AddPoint("sv_boundary_lower", path_s, v_lower_bound);
    print_debug.AddPoint("sk_curve", path_s, path_point.kappa());
    print_debug.AddPoint("vt_boundary_upper", curr_t, v_upper_bound);
    print_debug.AddPoint("sv_boundary_upper", path_s, v_upper_bound);
  }

  /**
   * @brief 调整初始状态
   *
   * AdjustInitStatus():
   *   检查初始状态是否满足约束
   *   如果不满足，调整初始加速度为0
   */
  AdjustInitStatus(s_dot_bounds, delta_t, init_s);

  /**
   * @brief 创建分段加加速度优化问题
   *
   * PiecewiseJerkSpeedProblem:
   *   速度优化问题的数学建模类
   *   参数：
   *   - num_of_knots: 网格点数量
   *   - delta_t: 时间步长
   *   - init_s: 初始状态 [s, v, a]
   */
  PiecewiseJerkSpeedProblem piecewise_jerk_problem(num_of_knots, delta_t,
                                                   init_s);

  /**
   * @brief 设置优化权重
   *
   * set_weight_ddx:
   *   设置加速度权重（越大越希望加速度小）
   * set_weight_dddx:
   *   设置加加速度权重（越大越希望加加速度小，即更平滑）
   */
  piecewise_jerk_problem.set_weight_ddx(config_.acc_weight());
  piecewise_jerk_problem.set_weight_dddx(config_.jerk_weight());

  /**
   * @brief 设置缩放因子
   *
   * set_scale_factor:
   *   设置变量的缩放因子
   *   {1.0, 10.0, 100.0} 分别对应 s, ds, dds
   *   用于改善优化问题的数值稳定性
   */
  piecewise_jerk_problem.set_scale_factor({1.0, 10.0, 100.0});

  /**
   * @brief 设置S边界
   *
   * set_x_bounds:
   *   设置位置s的可行域
   *   - 整体边界：0到total_length
   *   - 网格边界：每个时刻的s_bounds
   */
  piecewise_jerk_problem.set_x_bounds(0.0, total_length);

  /**
   * @brief 设置加速度边界
   *
   * set_ddx_bounds:
   *   设置加速度的可行域
   *   - 下界：max_deceleration（负值）
   *   - 上界：max_acceleration（正值）
   */
  piecewise_jerk_problem.set_ddx_bounds(veh_param.max_deceleration(),
                                        veh_param.max_acceleration());

  /**
   * @brief 设置加加速度边界
   *
   * set_dddx_bound:
   *   设置加加速度的可行域
   *   - 下界：longitudinal_jerk_lower_bound（负值）
   *   - 上界：longitudinal_jerk_upper_bound（正值）
   */
  piecewise_jerk_problem.set_dddx_bound(FLAGS_longitudinal_jerk_lower_bound,
                                        FLAGS_longitudinal_jerk_upper_bound);

  /**
   * @brief 设置网格边界
   *
   * set_x_bounds(std::move(s_bounds)):
   *   使用移动语义，将s_bounds转移到优化问题中
   *   std::move避免数据拷贝
   */
  piecewise_jerk_problem.set_x_bounds(std::move(s_bounds));

  /**
   * @brief 设置参考速度
   *
   * set_dx_ref:
   *   设置参考速度及其权重
   */
  piecewise_jerk_problem.set_dx_ref(dx_ref_weight, dx_ref);

  /**
   * @brief 设置参考位置
   *
   * set_x_ref:
   *   设置参考s轨迹及其权重
   */
  piecewise_jerk_problem.set_x_ref(config_.ref_s_weight(), std::move(x_ref));

  /**
   * @brief 设置曲率惩罚
   *
   * set_penalty_dx:
   *   设置速度惩罚向量
   */
  piecewise_jerk_problem.set_penalty_dx(penalty_dx);

  /**
   * @brief 设置速度边界
   */
  piecewise_jerk_problem.set_dx_bounds(std::move(s_dot_bounds));

  /**
   * @brief 求解优化问题
   *
   * Optimize():
   *   调用QP求解器求解优化问题
   * 如果求解失败，尝试回退机制
   */
  if (!piecewise_jerk_problem.Optimize()) {
    /**
     * @brief 优化失败，记录警告
     */
    const std::string msg = "Piecewise jerk speed optimizer failed!";
    AERROR << msg << ".try to fallback.";

    /**
     * @brief 回退机制：放宽速度约束
     *
     * set_dx_bounds:
     *   将速度上界设为初始速度和全局最大速度的较大值
     *   这样可以确保初始状态是可行的
     */
    piecewise_jerk_problem.set_dx_bounds(
        0.0, std::fmax(FLAGS_planning_upper_speed_limit,
                       st_graph_data.init_point().v()));

    /**
     * @brief 再次尝试求解
     *
     * 如果配置允许放宽约束且第一次失败
     * 或者放宽约束后仍然失败
     */
    if (!FLAGS_speed_optimize_fail_relax_velocity_constraint ||
        !piecewise_jerk_problem.Optimize()) {
      /**
       * @brief 回退也失败，记录错误并返回
       */
      speed_data->clear();  /**< 清空速度数据 */

      /**
       * @brief 记录调试曲线
       */
      print_debug.AddPoint("optimize_st_curve", 0, init_s[0]);
      print_debug.AddPoint("optimize_vt_curve", 0, init_s[1]);
      print_debug.AddPoint("optimize_at_curve", 0, init_s[2]);

      /**
       * @brief 打印优化参数
       */
      AINFO << "jerk_bound: " << FLAGS_longitudinal_jerk_lower_bound << ","
            << FLAGS_longitudinal_jerk_upper_bound;
      AINFO << "acc bound: " << veh_param.max_deceleration() << ","
            << veh_param.max_acceleration();

      print_debug.PrintToLog();  /**< 打印调试曲线 */
      return Status(ErrorCode::PLANNING_ERROR, msg);  /**< 返回错误状态 */
    }
  }

  /**
   * @brief 提取优化结果
   *
   * opt_x(): 获取最优s序列
   * opt_dx(): 获取最优ds序列（速度）
   * opt_ddx(): 获取最优dds序列（加速度）
   */
  const std::vector<double>& s = piecewise_jerk_problem.opt_x();
  const std::vector<double>& ds = piecewise_jerk_problem.opt_dx();
  const std::vector<double>& dds = piecewise_jerk_problem.opt_ddx();

  /**
   * @brief 记录优化结果到调试曲线
   */
  for (int i = 0; i < num_of_knots; ++i) {
    /**
     * @brief 打印调试信息
     */
    ADEBUG << "For t[" << i * delta_t << "], s = " << s[i] << ", v = " << ds[i]
           << ", a = " << dds[i];

    /**
     * @brief 记录曲线数据
     */
    print_debug.AddPoint("optimize_st_curve", i * delta_t, s[i]);
    print_debug.AddPoint("optimize_vt_curve", i * delta_t, ds[i]);
    print_debug.AddPoint("optimize_at_curve", i * delta_t, dds[i]);
  }

  /**
   * @brief 清空并构建输出速度数据
   *
   * speed_data->clear():
   *   清空原有的速度数据
   */
  speed_data->clear();

  /**
   * @brief 添加第一个速度点
   *
   * AppendSpeedPoint(s, t, v, a, jerk):
   *   添加一个速度点到速度曲线
   */
  speed_data->AppendSpeedPoint(s[0], 0.0, ds[0], dds[0], 0.0);

  /**
   * @brief 添加后续速度点
   *
   * for循环：
   * - 从第二个点开始（i=1）
   * - 跳过已经停止的点
   */
  for (int i = 1; i < num_of_knots; ++i) {
    /**
     * @brief 避免添加已经停止的点
     *
     * if (ds[i] <= 0.0):
     *   如果速度小于等于0，说明已经停止
     *   停止添加点
     */
    if (ds[i] <= 0.0) {
      break;  /**< 速度为负或零，停止 */
    }

    /**
     * @brief 计算jerk并添加速度点
     *
     * jerk = (a[i] - a[i-1]) / delta_t
     * 加加速度 = 加速度差分 / 时间步长
     */
    speed_data->AppendSpeedPoint(s[i], delta_t * i, ds[i], dds[i],
                                 (dds[i] - dds[i - 1]) / delta_t);
  }

  /**
   * @brief 填充足够的速度点
   *
   * SpeedProfileGenerator::FillEnoughSpeedPoints():
   *   确保速度曲线有足够的数据点
   *   用于生成平滑的轨迹
   */
  SpeedProfileGenerator::FillEnoughSpeedPoints(speed_data);

  /**
   * @brief 记录调试信息
   *
   * RecordDebugInfo():
   *   将速度数据和ST图调试信息记录到debug结构
   */
  RecordDebugInfo(*speed_data, st_graph_data.mutable_st_graph_debug());

  /**
   * @brief 打印调试曲线
   */
  print_debug.PrintToLog();

  return Status::OK();  /**< 优化成功 */
}

/**
 * @brief 调整初始状态
 *
 * @param s_dot_bound 速度边界
 * @param delta_t 时间步长
 * @param init_s 输入/输出：初始状态 [s, v, a]
 *
 * 功能说明：
 * 检查初始状态是否满足速度边界约束
 * 如果不满足，调整初始加速度为0
 *
 * 算法说明：
 * 1. 从初始状态出发，模拟几秒后的状态
 * 2. 检查模拟的状态是否在速度边界内
 * 3. 如果超出边界，说明初始加速度不合理
 * 4. 将初始加速度设为0，使用更保守的规划
 *
 * C++语法说明：
 * - std::array<double, 3>&: 引用，可以修改传入的数组
 */
void PiecewiseJerkSpeedOptimizer::AdjustInitStatus(
    const std::vector<std::pair<double, double>> s_dot_bound, double delta_t,
    std::array<double, 3>& init_s) {
  /**
   * @brief 初始化速度边界
   */
  double v_min = init_s[1];  /**< 最小速度 = 初始速度 */
  double v_max = init_s[1];  /**< 最大速度 = 初始速度 */
  double a_min = init_s[2];  /**< 最小加速度 = 初始加速度 */
  double a_max = init_s[2];  /**< 最大加速度 = 初始加速度 */
  double last_a_min = 0;     /**< 上一步的最小加速度 */
  double last_a_max = 0;     /**< 上一步的最大加速度 */

  /**
   * @brief 模拟前几个时刻的状态
   *
   * for循环：
   * - 从第1个时刻到最后一个
   * - 使用jerk极限模拟可能的加速度变化
   */
  for (size_t i = 1; i < s_dot_bound.size(); i++) {
    /**
     * @brief 保存上一步的加速度
     */
    last_a_min = a_min;
    last_a_max = a_max;

    /**
     * @brief 更新加速度边界
     *
     * a_min = a_min + delta_t * jerk_upper_bound
     * 使用最大jerk上界增加最小加速度
     */
    a_min = a_min + delta_t * FLAGS_longitudinal_jerk_upper_bound;

    /**
     * @brief 更新最大加速度
     *
     * a_max = a_max + delta_t * jerk_lower_bound
     * jerk_lower_bound是负值，所以实际是减少
     */
    a_max = a_max + delta_t * FLAGS_longitudinal_jerk_lower_bound;

    /**
     * @brief 更新速度边界
     *
     * v = v + 0.5 * delta_t * (a_current + a_last)
     * 使用梯形积分计算速度变化
     */
    v_min = v_min + 0.5 * delta_t * (a_min + last_a_min);
    v_max = v_max + 0.5 * delta_t * (a_max + last_a_max);

    /**
     * @brief 检查速度是否超出边界
     *
     * 如果 v_min < 下界 或 v_max > 上界
     * 说明初始加速度不合理
     */
    if (v_min < s_dot_bound[i].first || v_max > s_dot_bound[i].second) {
      /**
       * @brief 记录警告
       */
      AWARN << "init state not appropriate in" << i << "," << v_min << ","
            << v_max << "adjust acc to 0 in init state " << init_s[0] << ","
            << init_s[1] << "," << init_s[2];

      /**
       * @brief 调整初始加速度为0
       */
      init_s[2] = 0;

      /**
       * @brief 直接返回
       */
      return;
    }
  }
}

}  // namespace planning
}  // namespace apollo
