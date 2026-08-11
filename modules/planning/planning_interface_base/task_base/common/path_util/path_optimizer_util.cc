/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
 * @file path_optimizer_util.cc
 * @brief 路径优化工具实现文件
 *
 * 本文件实现了路径优化的核心工具函数，包括：
 * 1. PiecewiseJerk路径生成：将优化结果转换为Frenet帧路径
 * 2. jerk边界估计：根据车速计算最大允许的jerk值
 * 3. 路径点坐标转换：前轴参考点到后轴参考点的转换
 * 4. 路径优化主函数：使用分段jerk优化算法求解最优路径
 * 5. 参考线更新函数：根据边界约束更新路径参考线
 *
 * 核心算法：分段jerk路径优化（Piecewise Jerk Path Optimization）
 * - 将路径问题转化为QP（二次规划）问题
 * - 目标函数：最小化jerk（三阶导数）的平方和
 * - 约束条件：边界约束、顶点约束、终点状态约束
 */

#include "modules/planning/planning_interface_base/task_base/common/path_util/path_optimizer_util.h"

/**
 * @brief 标准库头文件
 * <utility>: 提供std::pair等工具
 * <vector>: 提供std::vector动态数组容器
 */
#include <utility>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
/**
 * @brief 车辆配置辅助类
 * VehicleConfigHelper::GetConfig(): 获取车辆配置参数
 * 用于获取车辆物理参数（轴距、宽度、转向比等）
 */
#include "modules/common/math/math_utils.h"
/**
 * @brief 数学工具库
 * 提供常用的数学函数和工具
 */
#include "modules/planning/planning_base/common/speed/speed_data.h"
/**
 * @brief 速度数据结构
 * SpeedData: 存储时间-速度序列
 */
#include "modules/planning/planning_base/common/trajectory1d/piecewise_jerk_trajectory1d.h"
/**
 * @brief 一维分段jerk轨迹类
 * PiecewiseJerkTrajectory1d: 用于生成一维分段jerk轨迹
 * 支持轨迹的拼接和求值
 */
#include "modules/planning/planning_base/common/util/print_debug_info.h"
/**
 * @brief 调试信息打印工具
 * PrintCurves: 用于将曲线数据打印到日志
 * PrintBox: 用于打印ADC边界框数据
 */
#include "modules/planning/planning_base/gflags/planning_gflags.h"
/**
 * @brief Planning模块的GFlags配置参数
 * 通过命令行标志配置优化器行为
 */
#include "modules/planning/planning_base/math/piecewise_jerk/piecewise_jerk_path_problem.h"
/**
 * @brief 分段jerk路径优化问题类
 * PiecewiseJerkPathProblem: QP求解器，用于解决分段jerk路径优化问题
 * 核心优化算法实现
 */

namespace apollo {
/**
 * @brief Apollo顶层命名空间
 * 包含自动驾驶所有模块
 */
namespace planning {

/**
 * @brief 将PiecewiseJerk轨迹转换为Frenet帧路径
 *
 * @param x 优化结果：横向位置序列 (l值)
 * @param dx 优化结果：横向一阶导数序列 (dl/dt)
 * @param ddx 优化结果：横向二阶导数序列 (ddl/dt²)
 * @param delta_s 路径点间距 (米)
 * @param start_s 起始s坐标 (米)
 * @return FrenetFramePath 转换后的Frenet帧路径
 *
 * 算法流程：
 * 1. 创建PiecewiseJerkTrajectory1d对象，用初始状态初始化
 * 2. 遍历所有点，计算jerk (三阶导数)，追加到轨迹段
 * 3. 按分辨率采样轨迹，生成FrenetFramePoint序列
 * 4. 返回FrenetFramePath对象
 *
 * C++语法说明：
 * - std::vector<double>: 动态数组模板类
 * - std::size_t: 无符号整数类型，用于表示大小和索引
 * - std::move(): 移动语义，避免不必要的拷贝
 * - .front(): 返回容器第一个元素的引用
 * - .emplace_back(): 原位构造并添加元素，比push_back更高效
 */
FrenetFramePath PathOptimizerUtil::ToPiecewiseJerkPath(
    const std::vector<double>& x, const std::vector<double>& dx,
    const std::vector<double>& ddx, const double delta_s,
    const double start_s) {
  /**
   * @brief 断言检查输入非空
   * ACHECK: Apollo自定义断言，运行时检查
   * 确保传入的向量非空，避免后续访问空指针
   */
  ACHECK(!x.empty());
  ACHECK(!dx.empty());
  ACHECK(!ddx.empty());

  /**
   * @brief 创建分段jerk轨迹对象
   * PiecewiseJerkTrajectory1d构造函数参数：
   * - x.front(): 初始位置 (l值)
   * - dx.front(): 初始一阶导数 (dl/dt)
   * - ddx.front(): 初始二阶导数 (ddl/dt²)
   */
  PiecewiseJerkTrajectory1d piecewise_jerk_traj(x.front(), dx.front(),
                                                ddx.front());

  /**
   * @brief 遍历所有点，追加轨迹段
   * for循环：从i=1开始，因为i=0是初始状态
   * std::size_t i = 1: 使用无符号类型，避免负数索引
   * ++i: 前置递增，先自增再使用
   *
   * jerk计算公式：dddl = (ddx[i] - ddx[i-1]) / delta_s
   * - ddx[i]: 当前点的二阶导数
   * - ddx[i-1]: 前一个点的二阶导数
   * - delta_s: 距离间隔
   * - jerk是三阶导数，表示曲率变化率
   */
  for (std::size_t i = 1; i < x.size(); ++i) {
    const auto dddl = (ddx[i] - ddx[i - 1]) / delta_s;
    /**
     * @brief 追加轨迹段
     * AppendSegment参数：
     * - dddl: 当前段的jerk值（三阶导数）
     * - delta_s: 段的长度
     */
    piecewise_jerk_traj.AppendSegment(dddl, delta_s);
  }

  /**
   * @brief 创建Frenet帧路径点容器
   * common::FrenetFramePoint: Frenet坐标系下的路径点
   * 包含s、l、dl、ddl等信息
   */
  std::vector<common::FrenetFramePoint> frenet_frame_path;
  double accumulated_s = 0.0;  /**< 累积距离，初始化为0 */

  /**
   * @brief 按分辨率采样轨迹
   * while循环：累积距离小于轨迹总长度时继续
   * piecewise_jerk_traj.ParamLength(): 获取轨迹参数长度
   */
  while (accumulated_s < piecewise_jerk_traj.ParamLength()) {
    /**
     * @brief 求值轨迹得到当前点的状态
     * Evaluate(param, order):
     * - param: 参数值（这里用accumulated_s距离）
     * - order: 求导阶数
     *   - 0: 位置 l
     *   - 1: 一阶导数 dl
     *   - 2: 二阶导数 ddl
     */
    double l = piecewise_jerk_traj.Evaluate(0, accumulated_s);
    double dl = piecewise_jerk_traj.Evaluate(1, accumulated_s);
    double ddl = piecewise_jerk_traj.Evaluate(2, accumulated_s);

    /**
     * @brief 创建Frenet帧点并设置属性
     * common::FrenetFramePoint:
     * - set_s(): 设置沿参考线的累积距离
     * - set_l(): 设置横向偏移
     * - set_dl(): 设置横向速度
     * - set_ddl(): 设置横向加速度
     */
    common::FrenetFramePoint frenet_frame_point;
    frenet_frame_point.set_s(accumulated_s + start_s);
    frenet_frame_point.set_l(l);
    frenet_frame_point.set_dl(dl);
    frenet_frame_point.set_ddl(ddl);

    /**
     * @brief 将点添加到路径容器
     * std::move(frenet_frame_point): 移动语义，将对象所有权转移给容器
     * 避免拷贝，提高效率
     */
    frenet_frame_path.push_back(std::move(frenet_frame_point));

    /**
     * @brief 累积距离增加
     * FLAGS_trajectory_space_resolution: 轨迹空间分辨率（全局配置）
     * 控制采样点的密度
     */
    accumulated_s += FLAGS_trajectory_space_resolution;
  }

  /**
   * @brief 返回Frenet帧路径
   * std::move(frenet_frame_path): 移动语义返回，避免拷贝
   */
  return FrenetFramePath(std::move(frenet_frame_path));
}

/**
 * @brief 根据车速估计jerk边界
 *
 * @param vehicle_speed 当前车速 (m/s)
 * @return double 最大允许的jerk值 (1/s²)
 *
 * 算法原理：
 * 基于车辆转向特性，计算保证车辆稳定行驶的最大jerk
 *
 * 公式：max_yaw_rate / wheel_base / vehicle_speed
 * - max_yaw_rate: 最大横摆角速度 = max_steer_angle_rate / steer_ratio
 * - wheel_base: 轴距
 * - vehicle_speed: 车速
 *
 * 物理意义：
 * - 最大转向角速度 / 轴距 = 最大横摆角速度
 * - 横摆角速度 / 车速 = 最大曲率kappa
 * - 曲率变化率 = jerk
 *
 * C++语法说明：
 * - const auto& veh_param: 常量引用，避免拷贝
 * - veh_param.wheel_base(): 车辆配置参数获取方法
 */
double PathOptimizerUtil::EstimateJerkBoundary(const double vehicle_speed) {
  /**
   * @brief 获取车辆配置参数引用
   * VehicleConfigHelper::GetConfig(): 获取全局车辆配置单例
   * .vehicle_param(): 获取车辆参数结构体
   */
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();

  /**
   * @brief 获取轴距参数
   * wheel_base(): 前后轴中心之间的距离
   */
  const double axis_distance = veh_param.wheel_base();

  /**
   * @brief 计算最大横摆角速度
   * max_steer_angle_rate(): 最大转向角速度 (rad/s)
   * steer_ratio(): 转向比（转向盘转角/车轮转角）
   * max_yaw_rate: 车辆横摆角速度上限
   */
  const double max_yaw_rate =
      veh_param.max_steer_angle_rate() / veh_param.steer_ratio();

  /**
   * @brief 计算jerk边界
   * max_yaw_rate / axis_distance / vehicle_speed
   *
   * 推导：
   * - 曲率kappa = tan(theta) / wheel_base ≈ theta / wheel_base (小角度近似)
   * - yaw_rate = kappa * speed = theta * speed / wheel_base
   * - jerk = dkappa/ds * speed = d(yaw_rate/speed)/ds * speed
   */
  return max_yaw_rate / axis_distance / vehicle_speed;
}

/**
 * @brief 将路径点从前轴参考转换为后轴参考
 *
 * @param path_data 输入的路径数据（基于前轴）
 * @return std::vector<common::PathPoint> 转换后的路径点（基于后轴）
 *
 * 坐标转换公式：
 * x_rear = x_front - wheel_base * cos(theta)
 * y_rear = y_front - wheel_base * sin(theta)
 *
 * 为什么需要转换？
 * - 规划时通常使用前轴作为参考点
 * - 控制和定位通常使用后轴作为参考点
 * - 需要统一到同一参考点进行控制
 *
 * C++语法说明：
 * - auto path_point : path_data.discretized_path():
 *   范围for循环，遍历discretized_path()返回的每一个PathPoint
 * - std::cos/std::sin: C标准库三角函数
 */
std::vector<common::PathPoint>
PathOptimizerUtil::ConvertPathPointRefFromFrontAxeToRearAxe(
    const PathData& path_data) {
  std::vector<common::PathPoint> ret;  /**< 返回的路径点容器 */

  /**
   * @brief 获取轴距
   * 前轴到后轴的距离，用于坐标转换
   */
  double front_to_rear_axe_distance =
      apollo::common::VehicleConfigHelper::GetConfig()
          .vehicle_param()
          .wheel_base();

  /**
   * @brief 遍历原始路径的每个点
   * path_data.discretized_path(): 返回路径点的vector引用
   * for (auto path_point : ...): 范围for循环遍历
   */
  for (auto path_point : path_data.discretized_path()) {
    /**
     * @brief 拷贝当前路径点
     * 避免修改原始数据
     */
    common::PathPoint new_path_point = path_point;

    /**
     * @brief 计算后轴x坐标
     * x_rear = x_front - wheel_base * cos(theta)
     * path_point.x(): 前轴位置的x坐标
     * std::cos(path_point.theta()): 前进方向的余弦值
     */
    new_path_point.set_x(path_point.x() - front_to_rear_axe_distance *
                                              std::cos(path_point.theta()));

    /**
     * @brief 计算后轴y坐标
     * y_rear = y_front - wheel_base * sin(theta)
     * path_point.y(): 前轴位置的y坐标
     * std::sin(path_point.theta()): 前进方向的正弦值
     */
    new_path_point.set_y(path_point.y() - front_to_rear_axe_distance *
                                              std::sin(path_point.theta()));

    /**
     * @brief 添加转换后的点到结果容器
     */
    ret.push_back(new_path_point);
  }
  return ret;
}

/**
 * @brief 主路径优化函数
 *
 * @param init_state 初始状态 [s, l, dl, ddl] 或 [x, dx, ddx]
 * @param end_state 终点状态 [l, dl, ddl]
 * @param l_ref 横向位置参考线
 * @param l_ref_weight 参考线权重
 * @param path_boundary 路径边界约束
 * @param ddl_bounds 二阶导数约束 (ddl下界、上界)
 * @param dddl_bound 三阶导数约束 (jerk边界)
 * @param config 优化器配置参数
 * @param x 输出：优化后的横向位置序列
 * @param dx 输出：优化后的横向一阶导数序列
 * @param ddx 输出：优化后的横向二阶导数序列
 * @return bool 优化是否成功
 *
 * 算法流程：
 * 1. 创建PiecewiseJerkPathProblem优化问题
 * 2. 设置目标函数权重（位置、速度、加速度、jerk）
 * 3. 设置边界约束（位置边界、速度边界、加速度边界）
 * 4. 调用QP求解器Optimize()
 * 5. 提取优化结果
 *
 * C++语法说明：
 * - std::array<double, 3>: 固定大小数组模板
 * - const std::vector<std::pair<double, double>>&:
 *   常量引用，避免拷贝
 * - std::vector<double>*:
 *   指针参数，用于输出结果
 */
bool PathOptimizerUtil::OptimizePath(
    const SLState& init_state, const std::array<double, 3>& end_state,
    std::vector<double> l_ref, std::vector<double> l_ref_weight,
    const PathBoundary& path_boundary,
    const std::vector<std::pair<double, double>>& ddl_bounds, double dddl_bound,
    const PiecewiseJerkPathConfig& config, std::vector<double>* x,
    std::vector<double>* dx, std::vector<double>* ddx) {
  /**
   * @brief 获取路径边界数量（结点数）
   * boundary(): 返回边界的二维vector [num_points][2(上下界)]
   * kNumKnots: 优化问题的结点数
   */
  const auto& lat_boundaries = path_boundary.boundary();
  const size_t kNumKnots = lat_boundaries.size();

  /**
   * @brief 打印调试信息
   * AINFO: Apollo信息日志宏
   */
  AINFO << "kNumKnots: " << kNumKnots;

  /**
   * @brief 获取路径点间距
   * delta_s(): 路径边界点之间的距离
   */
  double delta_s = path_boundary.delta_s();

  /**
   * @brief 创建分段jerk路径优化问题
   * PiecewiseJerkPathProblem构造函数：
   * - kNumKnots: 结点数量
   * - delta_s: 结点间距
   * - init_state.second: 初始状态 [l, dl, ddl]
   */
  PiecewiseJerkPathProblem piecewise_jerk_problem(kNumKnots, delta_s,
                                                  init_state.second);

  /**
   * @brief 获取ADC顶点约束
   * adc_vertex_bound(): 自车顶点边界
   * 包含每个结点的上下界
   */
  const auto& adc_vertex_constraints = path_boundary.adc_vertex_bound();

  /**
   * @brief 获取额外路径边界
   * extra_path_bound(): 额外约束边界
   */
  const auto& extra_bound = path_boundary.extra_path_bound();

  /**
   * @brief 提取参考拖拽点l值
   * towing_l: 路径中心线/参考线的横向偏移
   */
  std::vector<double> towing_l_ref;
  for (auto& path_boundary_pt : path_boundary) {
    towing_l_ref.emplace_back(path_boundary_pt.towing_l);
  }

  /**
   * @brief 创建曲线打印对象
   * 用于调试时输出优化过程的曲线数据
   */
  PrintCurves print_curve;

  /**
   * @brief 打印ADC顶点约束曲线
   * for循环遍历所有顶点约束
   * AddPoint(label, x, y): 添加曲线点
   */
  for (size_t i = 0; i < adc_vertex_constraints.size(); i++) {
    print_curve.AddPoint(path_boundary.label() + "_vertex_l_lower",
                         adc_vertex_constraints[i].rear_axle_s,
                         adc_vertex_constraints[i].lower_bound);
    print_curve.AddPoint(path_boundary.label() + "_vertex_l_upper",
                         adc_vertex_constraints[i].rear_axle_s,
                         adc_vertex_constraints[i].upper_bound);
  }

  /**
   * @brief 打印额外边界曲线
   */
  for (size_t i = 0; i < extra_bound.size(); i++) {
    print_curve.AddPoint(path_boundary.label() + "_conner_l_lower",
                         extra_bound[i].rear_axle_s,
                         extra_bound[i].lower_bound);
    print_curve.AddPoint(path_boundary.label() + "_conner_l_upper",
                         extra_bound[i].rear_axle_s,
                         extra_bound[i].upper_bound);
  }

  /**
   * @brief 打印参考线和边界曲线
   * for循环遍历所有结点
   */
  for (size_t i = 0; i < kNumKnots; i++) {
    double s = i * path_boundary.delta_s() + path_boundary.start_s();
    print_curve.AddPoint(path_boundary.label() + "_ref_l", s, l_ref[i]);
    print_curve.AddPoint(path_boundary.label() + "_towing_ref_l", s,
                         towing_l_ref[i]);
    print_curve.AddPoint(path_boundary.label() + "_ref_l_weight", s,
                         l_ref_weight[i]);
    print_curve.AddPoint(path_boundary.label() + "_l_lower", s,
                         lat_boundaries[i].first);
    print_curve.AddPoint(path_boundary.label() + "_l_upper", s,
                         lat_boundaries[i].second);
    print_curve.AddPoint(path_boundary.label() + "_ddl_lower", s,
                         ddl_bounds[i].first);
    print_curve.AddPoint(path_boundary.label() + "_ddl_upper", s,
                         ddl_bounds[i].second);
  }

  /**
   * @brief 打印初始状态
   * path_boundary.label(): 路径边界标签
   * init_state.second: 初始横向状态 [l, dl, ddl]
   */
  print_curve.AddPoint(path_boundary.label() + "_opt_l",
                       path_boundary.start_s(), init_state.second[0]);
  print_curve.AddPoint(path_boundary.label() + "_opt_dl",
                       path_boundary.start_s(), init_state.second[1]);
  print_curve.AddPoint(path_boundary.label() + "_opt_ddl",
                       path_boundary.start_s(), init_state.second[2]);

  /**
   * @brief 设置终点状态权重
   * std::array<double, 3U>: 固定大小数组
   * {config.weight_end_state_l(), ...}: 初始化列表
   */
  std::array<double, 3U> end_state_weight = {config.weight_end_state_l(),
                                             config.weight_end_state_dl(),
                                             config.weight_end_state_ddl()};

  /**
   * @brief 设置终点状态约束
   * set_end_state_ref(weight, state):
   * - weight: 各分量权重
   * - state: 终点状态目标值
   */
  piecewise_jerk_problem.set_end_state_ref(end_state_weight, end_state);

  /**
   * @brief 设置横向位置参考线
   * set_x_ref(weight, reference):
   * - std::move(l_ref_weight): 移动权重向量，避免拷贝
   * - l_ref: 引用传递参考线
   */
  piecewise_jerk_problem.set_x_ref(std::move(l_ref_weight), l_ref);

  /**
   * @brief 设置拖拽参考线
   * set_towing_x_ref(weight, reference):
   * towing_l_ref: 拖拽点参考线（路径中心线）
   */
  piecewise_jerk_problem.set_towing_x_ref(config.l_weight(), towing_l_ref);

  /**
   * @brief 设置目标函数权重
   * - set_weight_x(): 横向位置权重
   * - set_weight_dx(): 横向速度权重
   * - set_weight_ddx(): 横向加速度权重
   * - set_weight_dddx(): 横向jerk权重
   *
   * 优化目标：min w_l*l² + w_dl*dl² + w_ddl*dddl² + w_dddl*dddl²
   */
  piecewise_jerk_problem.set_weight_x(config.l_weight());
  piecewise_jerk_problem.set_weight_dx(config.dl_weight());
  piecewise_jerk_problem.set_weight_ddx(config.ddl_weight());
  piecewise_jerk_problem.set_weight_dddx(config.dddl_weight());

  /**
   * @brief 设置变量缩放因子
   * {1.0, 10.0, 100.0}: [l, dl, ddl]的缩放因子
   * 用于改善QP求解器的数值稳定性
   */
  piecewise_jerk_problem.set_scale_factor({1.0, 10.0, 100.0});

  /**
   * @brief 设置额外约束和顶点约束
   */
  piecewise_jerk_problem.set_extra_constraints(extra_bound);
  piecewise_jerk_problem.set_vertex_constraints(adc_vertex_constraints);

  /**
   * @brief 记录优化开始时间
   * std::chrono::system_clock::now(): 获取当前时间点
   */
  auto start_time = std::chrono::system_clock::now();

  /**
   * @brief 设置优化边界
   * - set_x_bounds(): 横向位置边界 [l_lower, l_upper]
   * - set_dx_bounds(): 横向速度边界 [-v_max, v_max]
   * - set_ddx_bounds(): 横向加速度边界
   */
  piecewise_jerk_problem.set_x_bounds(lat_boundaries);
  piecewise_jerk_problem.set_dx_bounds(
      -config.lateral_derivative_bound_default(),
      config.lateral_derivative_bound_default());
  piecewise_jerk_problem.set_ddx_bounds(ddl_bounds);

  /**
   * @brief 设置jerk（三阶导数）边界
   * dddl_bound: jerk的绝对值上限
   */
  piecewise_jerk_problem.set_dddx_bound(dddl_bound);

  /**
   * @brief 执行优化
   * Optimize(max_iteration): 调用QP求解器
   * 返回是否成功收敛
   */
  bool success = piecewise_jerk_problem.Optimize(config.max_iteration());

  /**
   * @brief 计算优化耗时
   */
  auto end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_time - start_time;

  /**
   * @brief 打印优化耗时
   * ADEBUG: Apollo调试日志宏
   * diff.count(): duration转换为秒
   * *1000: 转换为毫秒
   */
  ADEBUG << "Path Optimizer used time: " << diff.count() * 1000 << " ms.";

  /**
   * @brief 优化失败处理
   */
  if (!success) {
    /**
     * @brief 打印错误信息
     * AERROR: Apollo错误日志宏
     */
    AERROR << path_boundary.label() << "piecewise jerk path optimizer failed";

    /**
     * @brief 打印初始状态和参数用于调试
     */
    AINFO << "init s(" << init_state.first[0] << "," << init_state.first[1]
          << "," << init_state.first[2] << ") l (" << init_state.second[0]
          << "," << init_state.second[1] << "," << init_state.second[2];
    AINFO << "dx bound" << config.lateral_derivative_bound_default();
    AINFO << "jerk bound" << dddl_bound;

    /**
     * @brief 打印所有曲线到日志
     * 用于离线分析优化失败原因
     */
    print_curve.PrintToLog();
    return false;
  }

  /**
   * @brief 提取优化结果
   * opt_x/opt_dx/opt_ddx():
   * 返回优化后的横向位置、速度、加速度序列
   */
  *x = piecewise_jerk_problem.opt_x();
  *dx = piecewise_jerk_problem.opt_dx();
  *ddx = piecewise_jerk_problem.opt_ddx();

  /**
   * @brief 创建ADC边界框打印对象
   */
  PrintBox print_box("opt_l_box");

  /**
   * @brief 打印优化结果曲线
   */
  for (size_t i = 0; i < kNumKnots; i++) {
    double s = i * path_boundary.delta_s() + path_boundary.start_s();
    print_curve.AddPoint(path_boundary.label() + "_opt_l", s, (*x)[i]);
    print_curve.AddPoint(path_boundary.label() + "_opt_dl", s, (*dx)[i]);
    print_curve.AddPoint(path_boundary.label() + "_opt_ddl", s, (*ddx)[i]);

    /**
     * @brief 添加ADC边界框
     * std::atan((*dx)[i]): atan(dl) ≈ 航向角变化
     * true: 是否有效
     */
    print_box.AddAdcBox(s, (*x)[i], std::atan((*dx)[i]), true);
  }

  /**
   * @brief 打印所有曲线
   */
  print_curve.PrintToLog();
  return true;
}

/**
 * @brief 带拖拽点的路径优化函数（重载版本）
 *
 * 与OptimizePath的区别：
 * - 额外参数towing_l_ref和towing_l_ref_weight
 * - 用于更精细的路径形状控制
 *
 * @param towing_l_ref 拖拽点横向位置参考
 * @param towing_l_ref_weight 拖拽点参考权重
 */
bool PathOptimizerUtil::OptimizePathWithTowingPoints(
    const SLState& init_state, const std::array<double, 3>& end_state,
    std::vector<double> l_ref, std::vector<double> l_ref_weight,
    std::vector<double> towing_l_ref, std::vector<double> towing_l_ref_weight,
    const PathBoundary& path_boundary,
    const std::vector<std::pair<double, double>>& ddl_bounds, double dddl_bound,
    const PiecewiseJerkPathConfig& config, std::vector<double>* x,
    std::vector<double>* dx, std::vector<double>* ddx) {
  /**
   * @brief 获取边界结点数
   */
  const auto& lat_boundaries = path_boundary.boundary();
  const size_t kNumKnots = lat_boundaries.size();

  /**
   * @brief 获取路径间距
   */
  double delta_s = path_boundary.delta_s();

  /**
   * @brief 创建优化问题
   */
  PiecewiseJerkPathProblem piecewise_jerk_problem(kNumKnots, delta_s,
                                                  init_state.second);

  /**
   * @brief 获取约束边界
   */
  const auto& adc_vertex_constraints = path_boundary.adc_vertex_bound();
  const auto& extra_bound = path_boundary.extra_path_bound();

  /**
   * @brief 创建调试曲线打印对象
   */
  PrintCurves print_curve;

  /**
   * @brief 打印顶点约束曲线
   */
  for (size_t i = 0; i < adc_vertex_constraints.size(); i++) {
    print_curve.AddPoint(path_boundary.label() + "_vertex_l_lower",
                         adc_vertex_constraints[i].rear_axle_s,
                         adc_vertex_constraints[i].lower_bound);
    print_curve.AddPoint(path_boundary.label() + "_vertex_l_upper",
                         adc_vertex_constraints[i].rear_axle_s,
                         adc_vertex_constraints[i].upper_bound);
  }

  /**
   * @brief 打印额外边界曲线
   */
  for (size_t i = 0; i < extra_bound.size(); i++) {
    print_curve.AddPoint(path_boundary.label() + "_conner_l_lower",
                         extra_bound[i].rear_axle_s,
                         extra_bound[i].lower_bound);
    print_curve.AddPoint(path_boundary.label() + "_conner_l_upper",
                         extra_bound[i].rear_axle_s,
                         extra_bound[i].upper_bound);
  }

  /**
   * @brief 打印所有边界和参考曲线
   */
  for (size_t i = 0; i < kNumKnots; i++) {
    double s = i * path_boundary.delta_s() + path_boundary.start_s();
    print_curve.AddPoint(path_boundary.label() + "_ref_l", s, l_ref[i]);
    print_curve.AddPoint(path_boundary.label() + "_towing_ref_l", s,
                         towing_l_ref[i]);
    print_curve.AddPoint(path_boundary.label() + "_ref_l_weight", s,
                         l_ref_weight[i]);
    print_curve.AddPoint(path_boundary.label() + "_l_lower", s,
                         lat_boundaries[i].first);
    print_curve.AddPoint(path_boundary.label() + "_l_upper", s,
                         lat_boundaries[i].second);
    print_curve.AddPoint(path_boundary.label() + "_dl_lower", s,
                         -config.lateral_derivative_bound_default());
    print_curve.AddPoint(path_boundary.label() + "_dl_upper", s,
                         config.lateral_derivative_bound_default());
    print_curve.AddPoint(path_boundary.label() + "_ddl_lower", s,
                         ddl_bounds[i].first);
    print_curve.AddPoint(path_boundary.label() + "_ddl_upper", s,
                         ddl_bounds[i].second);
  }

  /**
   * @brief 打印初始状态
   */
  print_curve.AddPoint(path_boundary.label() + "_opt_l",
                       path_boundary.start_s(), init_state.second[0]);
  print_curve.AddPoint(path_boundary.label() + "_opt_dl",
                       path_boundary.start_s(), init_state.second[1]);
  print_curve.AddPoint(path_boundary.label() + "_opt_ddl",
                       path_boundary.start_s(), init_state.second[2]);

  /**
   * @brief 设置参考线
   * std::move(): 移动语义，避免不必要的数据拷贝
   */
  piecewise_jerk_problem.set_x_ref(std::move(l_ref_weight), l_ref);
  piecewise_jerk_problem.set_towing_x_ref(std::move(towing_l_ref_weight),
                                          towing_l_ref);

  /**
   * @brief 设置目标函数权重
   */
  piecewise_jerk_problem.set_weight_x(config.l_weight());
  piecewise_jerk_problem.set_weight_dx(config.dl_weight());
  piecewise_jerk_problem.set_weight_ddx(config.ddl_weight());
  piecewise_jerk_problem.set_weight_dddx(config.dddl_weight());

  /**
   * @brief 设置缩放因子和约束
   */
  piecewise_jerk_problem.set_scale_factor({1.0, 10.0, 100.0});
  piecewise_jerk_problem.set_extra_constraints(extra_bound);
  piecewise_jerk_problem.set_vertex_constraints(adc_vertex_constraints);

  /**
   * @brief 记录优化开始时间
   */
  auto start_time = std::chrono::system_clock::now();

  /**
   * @brief 设置边界和执行优化
   */
  piecewise_jerk_problem.set_x_bounds(lat_boundaries);
  piecewise_jerk_problem.set_dx_bounds(
      -config.lateral_derivative_bound_default(),
      config.lateral_derivative_bound_default());
  piecewise_jerk_problem.set_ddx_bounds(ddl_bounds);
  piecewise_jerk_problem.set_dddx_bound(dddl_bound);

  /**
   * @brief 执行优化
   */
  bool success = piecewise_jerk_problem.Optimize(config.max_iteration());

  /**
   * @brief 计算优化耗时
   */
  auto end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_time - start_time;
  ADEBUG << "Path Optimizer used time: " << diff.count() * 1000 << " ms.";

  /**
   * @brief 优化失败处理
   */
  if (!success) {
    AERROR << path_boundary.label() << "piecewise jerk path optimizer failed";
    AINFO << "init s(" << init_state.first[0] << "," << init_state.first[1]
          << "," << init_state.first[2] << ") l (" << init_state.second[0]
          << "," << init_state.second[1] << "," << init_state.second[2];
    AINFO << "dx bound" << config.lateral_derivative_bound_default();
    AINFO << "jerk bound" << dddl_bound;
    print_curve.PrintToLog();
    return false;
  }

  /**
   * @brief 提取优化结果
   */
  *x = piecewise_jerk_problem.opt_x();
  *dx = piecewise_jerk_problem.opt_dx();
  *ddx = piecewise_jerk_problem.opt_ddx();

  /**
   * @brief 打印优化结果曲线
   */
  for (size_t i = 0; i < kNumKnots; i++) {
    double s = i * path_boundary.delta_s() + path_boundary.start_s();
    print_curve.AddPoint(path_boundary.label() + "_opt_l", s, (*x)[i]);
    print_curve.AddPoint(path_boundary.label() + "_opt_dl", s, (*dx)[i]);
    print_curve.AddPoint(path_boundary.label() + "_opt_ddl", s, (*ddx)[i]);
  }
  print_curve.PrintToLog();
  return true;
}

/**
 * @brief 根据边界更新路径参考线
 *
 * @param path_boundary 路径边界
 * @param weight 更新权重
 * @param ref_l 输入/输出：参考线l值
 * @param weight_ref_l 输入/输出：参考线权重
 *
 * 功能说明：
 * 当障碍物靠近路径时，需要更新参考线使其偏向另一侧
 * 算法检测左右边界是否有障碍物，并调整参考线位置
 *
 * C++语法说明：
 * - std::vector<double>*: 指针参数，用于输出
 * - resize(): 调整容器大小
 */
void PathOptimizerUtil::UpdatePathRefWithBound(
    const PathBoundary& path_boundary, double weight,
    std::vector<double>* ref_l, std::vector<double>* weight_ref_l) {
  /**
   * @brief 调整输出向量大小
   * resize(size): 将向量调整为指定大小
   */
  ref_l->resize(path_boundary.size());
  weight_ref_l->resize(path_boundary.size());

  /**
   * @brief 极小值常量
   * 用于浮点数比较，避免精度问题
   */
  const double kEpison = 1e-2;

  /**
   * @brief 标志位：是否需要根据左/右边界更新
   */
  bool need_update_by_left_boundary = false;
  bool need_update_by_right_boundary = false;

  /**
   * @brief 遍历所有路径边界点
   */
  for (size_t i = 0; i < ref_l->size(); i++) {
    /**
     * @brief 检查左边界是否有障碍物
     * 条件：
     * - 左边界类型是OBSTACLE
     * - 左边界l值 < 拖拽点l值 + epsilon
     *   (障碍物在拖拽点左侧且靠近)
     */
    need_update_by_left_boundary =
        path_boundary[i].l_upper.type == BoundType::OBSTACLE &&
        path_boundary[i].l_upper.l < path_boundary[i].towing_l + kEpison;

    /**
     * @brief 检查右边界是否有障碍物
     * 条件：
     * - 右边界类型是OBSTACLE
     * - 右边界l值 > 拖拽点l值 - epsilon
     *   (障碍物在拖拽点右侧且靠近)
     */
    need_update_by_right_boundary =
        path_boundary[i].l_lower.type == BoundType::OBSTACLE &&
        path_boundary[i].l_lower.l > path_boundary[i].towing_l - kEpison;

    /**
     * @brief 如果左右都不需要更新，设置权重为0
     * 0权重表示该点不参与参考线优化
     */
    if (!need_update_by_left_boundary && !need_update_by_right_boundary) {
      weight_ref_l->at(i) = 0;  /**< at(i): 访问索引i的元素 */
      continue;  /**< 跳过本次循环，进入下一次 */
    }

    /**
     * @brief 计算当前边界中心作为候选参考点
     */
    double center_ref_l =
        (path_boundary[i].l_lower.l + path_boundary[i].l_upper.l) / 2.0;
    weight_ref_l->at(i) = weight;  /**< 设置更新权重 */

    /**
     * @brief 检查边界宽度是否足够
     * 如果边界宽度 < 2 * FLAGS_path_obs_ref_shift_distance + epsilon
     * 说明空间太小，直接使用中心作为参考
     */
    if ((path_boundary[i].l_upper.l - path_boundary[i].l_lower.l) <
        (2 * FLAGS_path_obs_ref_shift_distance + kEpison)) {
      ref_l->at(i) = center_ref_l;
      continue;
    }

    /**
     * @brief 根据左边界更新参考点
     * 如果障碍物在左前方：
     * - new_center = l_upper - FLAGS_path_obs_ref_shift_distance
     * - 选择使参考点移动距离最小的方案
     */
    if (need_update_by_left_boundary &&
        std::fabs(path_boundary[i].l_upper.l -
                  FLAGS_path_obs_ref_shift_distance - ref_l->at(i)) <
            std::fabs(center_ref_l - ref_l->at(i))) {
      center_ref_l =
          path_boundary[i].l_upper.l - FLAGS_path_obs_ref_shift_distance;
    }

    /**
     * @brief 根据右边界更新参考点
     * 如果障碍物在右前方：
     * - new_center = l_lower + FLAGS_path_obs_ref_shift_distance
     */
    if (need_update_by_right_boundary &&
        std::fabs(path_boundary[i].l_lower.l +
                  FLAGS_path_obs_ref_shift_distance - ref_l->at(i)) <
            std::fabs(center_ref_l - ref_l->at(i))) {
      center_ref_l =
          path_boundary[i].l_lower.l + FLAGS_path_obs_ref_shift_distance;
    }

    /**
     * @brief 更新参考点位置
     */
    ref_l->at(i) = center_ref_l;

    /**
     * @brief 打印调试信息
     */
    AINFO << "need_update_path_ref: s: " << path_boundary[i].s
          << ", l: " << ref_l->at(i);
  }
}

/**
 * @brief 根据边界更新路径参考线（重载版本，带拖拽参考）
 *
 * @param towing_ref_l 拖拽点参考线（输入）
 *
 * 与上一版本区别：
 * - 使用towing_ref_l作为输入参考，而非之前的ref_l
 * - 更新逻辑略有不同
 */
void PathOptimizerUtil::UpdatePathRefWithBound(
    const PathBoundary& path_boundary, double weight,
    const std::vector<double>& towing_ref_l, std::vector<double>* ref_l,
    std::vector<double>* weight_ref_l) {
  /**
   * @brief 调整输出向量大小
   */
  ref_l->resize(path_boundary.size());
  weight_ref_l->resize(path_boundary.size());

  /**
   * @brief 极小值常量
   */
  const double kEpison = 1e-2;

  /**
   * @brief 遍历所有路径边界点
   */
  for (size_t i = 0; i < ref_l->size(); i++) {
    /**
     * @brief 判断是否需要更新参考线
     * 条件：
     * - 边界类型是OBSTACLE
     * - 且障碍物边界超出拖拽点
     */
    bool is_need_update_path_ref =
        (path_boundary[i].l_lower.type == BoundType::OBSTACLE ||
         path_boundary[i].l_upper.type == BoundType::OBSTACLE) &&
        (path_boundary[i].l_lower.l > towing_ref_l[i] - kEpison ||
         path_boundary[i].l_upper.l < towing_ref_l[i] + kEpison);

    /**
     * @brief 根据判断结果更新参考线
     */
    if (is_need_update_path_ref) {
      /**
       * @brief 使用边界中心作为新参考
       */
      ref_l->at(i) =
          (path_boundary[i].l_lower.l + path_boundary[i].l_upper.l) / 2.0;
      weight_ref_l->at(i) = weight;
      AINFO << "need_update_path_ref: s: " << path_boundary[i].s
            << ", l: " << ref_l->at(i);
    } else {
      /**
       * @brief 不需要更新，权重设为0
       */
      weight_ref_l->at(i) = 0;
    }
  }
}

/**
 * @brief 在绕行方向上更新路径参考线
 *
 * @param path_boundary 路径边界
 * @param weight 更新权重
 * @param ref_l 输入/输出：参考线l值
 * @param weight_ref_l 输入/输出：参考线权重
 * @param is_left_side_pass 是否左侧绕行
 *
 * 功能说明：
 * 当需要绕行时，根据绕行方向更新参考线
 * - 左侧绕行：只考虑左边界（l_upper）
 * - 右侧绕行：只考虑右边界（l_lower）
 */
void PathOptimizerUtil::UpdatePathRefWithBoundInSidePassDirection(
    const PathBoundary& path_boundary, double weight,
    std::vector<double>* ref_l, std::vector<double>* weight_ref_l,
    bool is_left_side_pass) {
  /**
   * @brief 调整输出向量大小
   */
  ref_l->resize(path_boundary.size());
  weight_ref_l->resize(path_boundary.size());

  /**
   * @brief 极小值常量
   */
  const double kEpison = 1e-2;

  /**
   * @brief 遍历所有路径边界点
   */
  for (size_t i = 0; i < ref_l->size(); i++) {
    /**
     * @brief 判断是否需要更新
     * - is_left_side_pass为true时：检查l_upper（左侧边界）
     * - is_left_side_pass为false时：检查l_lower（右侧边界）
     */
    bool is_need_update_path_ref =
        ((path_boundary[i].l_lower.type == BoundType::OBSTACLE &&
          is_left_side_pass) ||
         (path_boundary[i].l_upper.type == BoundType::OBSTACLE &&
          !is_left_side_pass)) &&
        (path_boundary[i].l_lower.l > path_boundary[i].towing_l - kEpison ||
         path_boundary[i].l_upper.l < path_boundary[i].towing_l + kEpison);

    /**
     * @brief 执行更新
     */
    if (is_need_update_path_ref) {
      ref_l->at(i) =
          (path_boundary[i].l_lower.l + path_boundary[i].l_upper.l) / 2.0;
      weight_ref_l->at(i) = weight;
      AINFO << "need_update_path_ref: s: " << path_boundary[i].s
            << ", l: " << ref_l->at(i);
    } else {
      weight_ref_l->at(i) = 0;
    }
  }
}

/**
 * @brief 计算加速度边界
 *
 * @param path_boundary 路径边界
 * @param reference_line 参考线
 * @param ddl_bounds 输出：二阶导数（加速度）边界
 *
 * 功能说明：
 * 根据车辆物理限制和道路曲率，计算允许的横向加速度边界
 *
 * 公式：ddl_bound = ±(tan(max_steer_angle) / wheel_base - kappa)
 * - tan(max_steer_angle) / wheel_base: 最大可能曲率
 * - kappa: 当前道路曲率
 * - 减去kappa是因为车辆沿曲线行驶时已有向心加速度
 *
 * C++语法说明：
 * - std::vector<std::pair<double, double>>*:
 *   指向pair向量的指针，pair.first是下界，pair.second是上界
 * - emplace_back(): 原位构造并添加元素
 */
void PathOptimizerUtil::CalculateAccBound(
    const PathBoundary& path_boundary, const ReferenceLine& reference_line,
    std::vector<std::pair<double, double>>* ddl_bounds) {
  /**
   * @brief 获取车辆参数
   */
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();

  /**
   * @brief 计算最大横向加速度边界
   * 公式：tan(max_steer_angle) / steer_ratio / wheel_base
   *
   * max_steer_angle(): 最大转向角
   * steer_ratio(): 转向比
   * wheel_base(): 轴距
   *
   * 物理意义：最大转向产生的横向加速度
   */
  const double lat_acc_bound =
      std::tan(veh_param.max_steer_angle() / veh_param.steer_ratio()) /
      veh_param.wheel_base();

  /**
   * @brief 获取路径边界点数
   */
  size_t path_boundary_size = path_boundary.boundary().size();

  /**
   * @brief 遍历每个边界点，计算对应的加速度边界
   */
  for (size_t i = 0; i < path_boundary_size; ++i) {
    /**
     * @brief 计算当前点的s坐标
     * i * delta_s + start_s: 第i个点到起点的距离
     */
    double s = static_cast<double>(i) * path_boundary.delta_s() +
               path_boundary.start_s();

    /**
     * @brief 获取参考线上最近点的曲率
     * GetNearestReferencePoint(s): 获取s处最近的参考点
     * .kappa(): 获取该点的曲率
     */
    double kappa = reference_line.GetNearestReferencePoint(s).kappa();

    /**
     * @brief 添加加速度边界
     * 下界: -lat_acc_bound - kappa
     * 上界: lat_acc_bound - kappa
     * 减去kappa考虑了已存在的向心加速度
     */
    // 车辆转向能力的剩余部分可用于曲率变化
    ddl_bounds->emplace_back(-lat_acc_bound - kappa, lat_acc_bound - kappa);
  }
}

/**
 * @brief 命名空间结束标记
 */
}  // namespace planning
}  // namespace apollo
