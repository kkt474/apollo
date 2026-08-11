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
 * @file lattice_planner.cc
 * @brief Lattice规划器实现文件
 *
 * 功能说明：
 * Lattice规划器是Apollo规划模块中基于 Frenet 坐标系的双层纵向-横向解耦规划算法
 * 主要特点：
 * 1. 在Frenet坐标系下进行规划，将三维问题分解为纵向和横向两个一维问题
 * 2. 纵向规划：生成速度曲线 (s, v, a, jerk)
 * 3. 横向规划：生成横向偏移曲线 (d, d', d'')
 * 4. 通过轨迹评估器选择最优的纵向-横向轨迹对
 * 5. 支持轨迹约束检查（速度、加速度、jerk、曲率）
 * 6. 支持碰撞检测
 * 7. 支持备用轨迹生成
 *
 * 算法流程：
 * 1. 获取参考线并转换为PathPoint格式
 * 2. 匹配初始点到参考线
 * 3. 转换为Frenet坐标系初始状态
 * 4. 解析决策获取规划目标
 * 5. 生成纵向和横向1维轨迹束
 * 6. 评估轨迹对的可行性和成本
 * 7. 选择最优无碰撞轨迹
 *
 * C++语法说明：
 * - namespace嵌套：apollo::planning两层命名空间
 * - using声明：引入常用类型简化代码
 * - std::make_shared：创建shared_ptr智能指针
 * - std::array：固定大小数组容器
 * - auto关键字：自动类型推导
 * - std::dynamic_pointer_cast：智能指针的动态类型转换
 * - 范围for循环：遍历容器元素
 */

#include "modules/planning/planners/lattice/lattice_planner.h"

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "cyber/common/log.h"
#include "cyber/common/macros.h"
#include "cyber/time/clock.h"
#include "modules/common/math/cartesian_frenet_conversion.h"
#include "modules/common/math/path_matcher.h"
#include "modules/planning/planners/lattice/behavior/collision_checker.h"
#include "modules/planning/planners/lattice/behavior/path_time_graph.h"
#include "modules/planning/planners/lattice/behavior/prediction_querier.h"
#include "modules/planning/planners/lattice/trajectory_generation/backup_trajectory_generator.h"
#include "modules/planning/planners/lattice/trajectory_generation/lattice_trajectory1d.h"
#include "modules/planning/planners/lattice/trajectory_generation/trajectory1d_generator.h"
#include "modules/planning/planners/lattice/trajectory_generation/trajectory_combiner.h"
#include "modules/planning/planners/lattice/trajectory_generation/trajectory_evaluator.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_base/math/constraint_checker/constraint_checker.h"

namespace apollo {
/**
 * @brief Apollo外层命名空间
 *
 * C++语法说明：
 * namespace关键字用于声明命名空间
 * 所有Apollo相关代码都位于apollo命名空间下
 */
namespace planning {

/**
 * @brief 使用using声明引入常用类型
 *
 * C++语法说明：
 * using声明：using apollo::common::ErrorCode;
 * 使类型可以直接使用，不必写完整限定名
 * 放在.cpp文件顶部，简化代码书写
 *
 * - ErrorCode：错误码枚举
 * - PathPoint：路径点结构
 * - Status：操作状态类
 * - TrajectoryPoint：轨迹点结构
 * - CartesianFrenetConverter：笛卡尔- Frenet坐标系转换工具
 * - PathMatcher：路径匹配工具
 * - Clock：Cyber RT时间获取工具
 */
using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::math::CartesianFrenetConverter;
using apollo::common::math::PathMatcher;
using apollo::cyber::Clock;

namespace {

/**
 * @brief 将参考点转换为离散化的路径点格式
 *
 * @param ref_points 参考点向量
 * @return std::vector<PathPoint> 离散化的路径点序列
 *
 * 功能说明：
 * 将参考线上的参考点转换为PathPoint格式
 * 同时计算每个点的累计距离s
 *
 * C++语法说明：
 * - const std::vector<ReferencePoint>&：
 *   常量引用输入参数，避免拷贝
 *
 * - std::vector<PathPoint>：
 *   返回值，通过移动语义返回
 *
 * - for (const auto& ref_point : ref_points)：
 *   范围for循环遍历参考点
 *   const auto&避免拷贝
 *
 * - PathPoint path_point：
 *   值拷贝创建局部变量
 *
 * - path_point.set_x(ref_point.x())：
 *   protobuf的setter方法设置字段值
 *
 * - std::move(path_point)：
 *   移动语义，将path_point的所有权转移给vector
 *   避免拷贝，提高效率
 *
 * - std::sqrt(dx * dx + dy * dy)：
 *   std::sqrt：平方根函数
 *   计算欧几里得距离
 *
 * 算法流程：
 * 1. 遍历每个参考点
 * 2. 复制位置、朝向、曲率等信息到PathPoint
 * 3. 计算与前一点的距离，累加到s
 * 4. 将PathPoint移动到返回向量
 */
std::vector<PathPoint> ToDiscretizedReferenceLine(
    const std::vector<ReferencePoint>& ref_points) {
  double s = 0.0;  // 累计距离初始化为0
  std::vector<PathPoint> path_points;  // 存储转换后的路径点

  /**
   * @brief 遍历每个参考点
   */
  for (const auto& ref_point : ref_points) {
    PathPoint path_point;  // 创建PathPoint对象
    path_point.set_x(ref_point.x());  // 设置x坐标
    path_point.set_y(ref_point.y());  // 设置y坐标
    path_point.set_theta(ref_point.heading());  // 设置朝向角
    path_point.set_kappa(ref_point.kappa());  // 设置曲率
    path_point.set_dkappa(ref_point.dkappa());  // 设置曲率变化率

    /**
     * @brief 计算累计距离s
     *
     * 如果不是第一个点，计算与前一点的欧几里得距离
     * 并累加到总距离s
     */
    if (!path_points.empty()) {
      double dx = path_point.x() - path_points.back().x();  // x方向差分
      double dy = path_point.y() - path_points.back().y();  // y方向差分
      s += std::sqrt(dx * dx + dy * dy);  // 累加欧几里得距离
    }
    path_point.set_s(s);  // 设置累计距离
    path_points.push_back(std::move(path_point));  // 移动到结果向量
  }
  return path_points;  // 返回结果
}

/**
 * @brief 计算Frenet坐标系的初始状态
 *
 * @param matched_point 匹配到参考线的点
 * @param cartesian_state 笛卡尔坐标系的轨迹点状态
 * @param ptr_s 输出参数：Frenet坐标系s方向状态[s, ds, dds]
 * @param ptr_d 输出参数：Frenet坐标系d方向状态[d, dd, ddd]
 *
 * 功能说明：
 * 将车辆在笛卡尔坐标系的状态转换为Frenet坐标系
 * Frenet坐标系以参考线为基准，更适合道路场景的规划
 *
 * C++语法说明：
 * - std::array<double, 3>：
 *   固定大小数组容器
 *   包含3个元素：[位置, 速度, 加速度]或[位移, 速度, 加速度]
 *
 * - std::array<double, 3>*：
 *   指向数组的指针作为输出参数
 *   函数通过解引用修改数组内容
 *
 * - CartesianFrenetConverter::cartesian_to_frenet()：
 *   静态成员函数调用
 *   ::作用域解析运算符访问命名空间内的类
 *
 * 算法流程：
 * 1. 获取匹配点的信息（s, x, y, theta, kappa, dkappa）
 * 2. 获取当前Cartesian状态（x, y, v, a, theta, kappa）
 * 3. 调用转换函数计算Frenet状态
 */
void ComputeInitFrenetState(const PathPoint& matched_point,
                            const TrajectoryPoint& cartesian_state,
                            std::array<double, 3>* ptr_s,
                            std::array<double, 3>* ptr_d) {
  /**
   * @brief 调用坐标系转换函数
   *
   * 参数说明：
   * - matched_point.s()：匹配点的累计距离
   * - matched_point.x/y/theta/kappa/dkappa()：匹配点的位姿信息
   * - cartesian_state.path_point().x/y/theta/kappa()：当前位置信息
   * - cartesian_state.v()/a()：当前速度、加速度
   * - ptr_s, ptr_d：输出参数，存储转换后的Frenet状态
   */
  CartesianFrenetConverter::cartesian_to_frenet(
      matched_point.s(), matched_point.x(), matched_point.y(),
      matched_point.theta(), matched_point.kappa(), matched_point.dkappa(),
      cartesian_state.path_point().x(), cartesian_state.path_point().y(),
      cartesian_state.v(), cartesian_state.a(),
      cartesian_state.path_point().theta(),
      cartesian_state.path_point().kappa(), ptr_s, ptr_d);
}

}  // namespace

/**
 * @brief Lattice规划器的主入口函数
 *
 * @param planning_start_point 规划起始点
 * @param frame 规划帧数据
 * @param ptr_computed_trajectory 输出参数：计算出的轨迹
 * @return Status 操作状态
 *
 * 功能说明：
 * 对所有参考线进行规划，返回成功的规划数量
 * 如果有任何参考线规划成功，返回OK状态
 *
 * C++语法说明：
 * - Status LatticePlanner::Plan(...)：
 *   类外定义的成员函数
 *   ::作用域解析运算符表示属于LatticePlanner类
 *
 * - Frame* frame：
 *   裸指针指向Frame对象
 *   Frame包含当前帧的所有规划数据
 *
 * - ADCTrajectory* ptr_computed_trajectory：
 *   指向轨迹的指针作为输出参数
 *   ptr_前缀表示这是输出参数
 *
 * - size_t：
 *   无符号整数类型
 *   用于表示大小、索引等非负值
 */
Status LatticePlanner::Plan(const TrajectoryPoint& planning_start_point,
                            Frame* frame,
                            ADCTrajectory* ptr_computed_trajectory) {
  size_t success_line_count = 0;  /**< 成功规划的参考线数量 */
  size_t index = 0;  /**< 当前参考线索引 */

  /**
   * @brief 遍历所有参考线进行规划
   *
   * C++语法说明：
   * - for (auto& reference_line_info : *frame->mutable_reference_line_info())：
   *   mutable_reference_line_info()返回可变指针
   *   *解引用获取容器引用
   *   auto&遍历并保持引用
   */
  for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
    /**
     * @brief 设置参考线优先级成本
     *
     * 第一条参考线优先级最高，成本为0
     * 其他参考线成本较高
     *
     * C++语法说明：
     * - index != 0：比较运算符
     * - FLAGS_cost_non_priority_reference_line：
     *   GFlags定义的全局标志变量
     */
    if (index != 0) {
      reference_line_info.SetPriorityCost(
          FLAGS_cost_non_priority_reference_line);
    } else {
      reference_line_info.SetPriorityCost(0.0);
    }

    /**
     * @brief 调用单条参考线的规划函数
     *
     * C++语法说明：
     * auto status = ...：
     *   自动类型推导
     *   Status类型的变量status
     */
    auto status =
        PlanOnReferenceLine(planning_start_point, frame, &reference_line_info);

    /**
     * @brief 处理规划失败的情况
     *
     * C++语法说明：
     * - status != Status::OK()：
     *   比较Status对象与OK状态
     *
     * - reference_line_info.IsChangeLanePath()：
     *   判断是否为变道路径
     *
     * - AERROR << ...：
     *   Apollo ERROR级别日志宏
     *   <<运算符用于拼接日志内容
     */
    if (status != Status::OK()) {
      if (reference_line_info.IsChangeLanePath()) {
        AERROR << "Planner failed to change lane to "
               << reference_line_info.Lanes().Id();
      } else {
        AERROR << "Planner failed to " << reference_line_info.Lanes().Id();
      }
    } else {
      success_line_count += 1;  /**< 成功计数加1 */
    }
    ++index;  /**< 索引递增 */
  }

  /**
   * @brief 返回规划结果
   *
   * 如果有至少一条参考线规划成功，返回OK
   * 否则返回错误状态
   */
  if (success_line_count > 0) {
    return Status::OK();
  }
  return Status(ErrorCode::PLANNING_ERROR,
                "Failed to plan on any reference line.");
}

/**
 * @brief 在单条参考线上进行Lattice规划
 *
 * @param planning_init_point 规划初始点
 * @param frame 规划帧数据
 * @param reference_line_info 参考线信息
 * @return Status 操作状态
 *
 * 功能说明：
 * Lattice规划的核心算法实现
 * 采用Frenet坐标系下的纵向-横向解耦规划
 *
 * 算法流程（带编号的步骤）：
 * 1. 获取参考线并转换为PathPoint格式
 * 2. 匹配初始点到参考线
 * 3. 计算Frenet坐标系的初始状态
 * 4. 解析决策获取规划目标
 * 5. 生成纵向和横向1维轨迹束
 * 6. 评估轨迹对的可行性和成本
 * 7. 选择最优无碰撞轨迹
 *
 * C++语法说明：
 * - std::make_shared<std::vector<PathPoint>>：
 *   创建shared_ptr智能指针
 *   管理动态分配的vector对象
 *   shared_ptr引用计数，多个指针可以共享所有权
 *
 * - static size_t：
 *   static修饰局部变量
 *   跨调用保持状态
 *   在函数作用域外不可见
 */
Status LatticePlanner::PlanOnReferenceLine(
    const TrajectoryPoint& planning_init_point, Frame* frame,
    ReferenceLineInfo* reference_line_info) {
  static size_t num_planning_cycles = 0;  /**< 累计规划周期数（静态变量） */
  static size_t num_planning_succeeded_cycles = 0;  /**< 累计成功规划周期数 */

  /**
   * @brief 记录时间戳用于性能分析
   *
   * C++语法说明：
   * - double start_time = Clock::NowInSeconds()：
   *   Clock::NowInSeconds()获取当前时间（秒）
   *   返回double类型的时间戳
   */
  double start_time = Clock::NowInSeconds();
  double current_time = start_time;

  /**
   * @brief 调试日志输出规划周期信息
   *
   * C++语法说明：
   * - ADEBUG << ...：
   *   Apollo DEBUG级别日志宏
   *   仅在DEBUG模式下输出
   *   <<运算符拼接字符串和变量
   */
  ADEBUG << "Number of planning cycles: " << num_planning_cycles << " "
         << num_planning_succeeded_cycles;
  ++num_planning_cycles;

  /**
   * @brief 设置是否在参考线上标志
   *
   * C++语法说明：
   * reference_line_info->set_is_on_reference_line()：
   *   指针->方法()调用成员函数
   */
  reference_line_info->set_is_on_reference_line();

  /**
   * @brief 步骤1：获取参考线并转换为PathPoint格式
   *
   * std::make_shared创建shared_ptr
   * ToDiscretizedReferenceLine将参考点转换为路径点格式
   * reference_line_info->reference_line().reference_points()获取参考点序列
   *
   * C++语法说明：
   * - auto ptr_reference_line：
   *   auto自动推导类型为std::shared_ptr<std::vector<PathPoint>>
   *   ptr_前缀表示这是指针类型
   */
  // 1. obtain a reference line and transform it to the PathPoint format.
  auto ptr_reference_line =
      std::make_shared<std::vector<PathPoint>>(ToDiscretizedReferenceLine(
          reference_line_info->reference_line().reference_points()));

  /**
   * @brief 步骤2：匹配初始点到参考线
   *
   * 使用PathMatcher将车辆当前位置匹配到参考线上
   * 得到参考线上与车辆位置最接近的点
   *
   * C++语法说明：
   * - PathMatcher::MatchToPath()：
   *   静态成员函数调用
   *   根据x, y坐标在路径上查找匹配点
   *
   * - planning_init_point.path_point().x()：
   *   链式成员访问
   *   path_point()返回PathPoint引用
   *   .x()获取x坐标
   */
  // 2. compute the matched point of the init planning point on the reference
  // line.
  PathPoint matched_point = PathMatcher::MatchToPath(
      *ptr_reference_line, planning_init_point.path_point().x(),
      planning_init_point.path_point().y());

  /**
   * @brief 步骤3：计算Frenet坐标系的初始状态
   *
   * 使用CartesianFrenetConverter进行坐标系转换
   * 得到初始的[s, ds, dds]和[d, dd, ddd]
   *
   * C++语法说明：
   * - std::array<double, 3> init_s：
   *   固定大小数组
   *   init_s[0]=s位置, init_s[1]=ds速度, init_s[2]=dds加速度
   *
   * - &init_s作为输出参数：
   *   指针传递给函数，函数内部修改数组内容
   */
  // 3. according to the matched point, compute the init state in Frenet frame.
  std::array<double, 3> init_s;
  std::array<double, 3> init_d;
  ComputeInitFrenetState(matched_point, planning_init_point, &init_s, &init_d);

  /**
   * @brief 输出时间戳日志
   */
  ADEBUG << "ReferenceLine and Frenet Conversion Time = "
         << (Clock::NowInSeconds() - current_time) * 1000;
  current_time = Clock::NowInSeconds();

  /**
   * @brief 创建预测查询器
   *
   * 用于获取障碍物的预测轨迹信息
   *
   * C++语法说明：
   * - std::make_shared<PredictionQuerier>(...)：
   *   创建PredictionQuerier的shared_ptr
   *   参数传递给构造函数
   *
   * - frame->obstacles()：
   *   获取当前帧的障碍物列表
   */
  auto ptr_prediction_querier = std::make_shared<PredictionQuerier>(
      frame->obstacles(), ptr_reference_line);

  /**
   * @brief 步骤4：创建路径-时间图并解析决策
   *
   * PathTimeGraph管理ST图的构建和查询
   * 用于处理动态障碍物的时空关系
   *
   * C++语法说明：
   * - FLAGS_speed_lon_decision_horizon：
   *   纵向决策视野（距离）
   *   GFlags全局标志变量
   *
   * - FLAGS_trajectory_time_length：
   *   轨迹时间长度
   *   规划的时间范围
   */
  // 4. parse the decision and get the planning target.
  auto ptr_path_time_graph = std::make_shared<PathTimeGraph>(
      ptr_prediction_querier->GetObstacles(), *ptr_reference_line,
      reference_line_info, init_s[0],
      init_s[0] + FLAGS_speed_lon_decision_horizon, 0.0,
      FLAGS_trajectory_time_length, init_d);

  /**
   * @brief 获取速度限制
   *
   * 从参考线获取当前速度限制
   *
   * C++语法说明：
   * - reference_line_info->reference_line()：
   *   获取参考线引用
   * - GetSpeedLimitFromS(init_s[0])：
   *   根据初始s位置获取速度限制
   */
  double speed_limit =
      reference_line_info->reference_line().GetSpeedLimitFromS(init_s[0]);
  reference_line_info->SetLatticeCruiseSpeed(speed_limit);

  /**
   * @brief 获取规划目标
   *
   * 包含停车点、跟车目标等信息
   *
   * C++语法说明：
   * - planning_target.has_stop_point()：
   *   protobuf的has_方法检查字段是否存在
   */
  PlanningTarget planning_target = reference_line_info->planning_target();
  if (planning_target.has_stop_point()) {
    ADEBUG << "Planning target stop s: " << planning_target.stop_point().s()
           << "Current ego s: " << init_s[0];
  }

  ADEBUG << "Decision_Time = " << (Clock::NowInSeconds() - current_time) * 1000;
  current_time = Clock::NowInSeconds();

  /**
   * @brief 步骤5：生成纵向和横向1维轨迹束
   *
   * Trajectory1dGenerator生成多条候选轨迹
   * lon_trajectory1d_bundle：纵向轨迹束
   * lat_trajectory1d_bundle：横向轨迹束
   *
   * C++语法说明：
   * - std::vector<std::shared_ptr<Curve1d>>：
   *   存储Curve1d智能指针的向量
   *   Curve1d是1维曲线的基类
   */
  // 5. generate 1d trajectory bundle for longitudinal and lateral respectively.
  Trajectory1dGenerator trajectory1d_generator(
      init_s, init_d, ptr_path_time_graph, ptr_prediction_querier);
  std::vector<std::shared_ptr<Curve1d>> lon_trajectory1d_bundle;
  std::vector<std::shared_ptr<Curve1d>> lat_trajectory1d_bundle;
  trajectory1d_generator.GenerateTrajectoryBundles(
      planning_target, &lon_trajectory1d_bundle, &lat_trajectory1d_bundle);

  ADEBUG << "Trajectory_Generation_Time = "
         << (Clock::NowInSeconds() - current_time) * 1000;
  current_time = Clock::NowInSeconds();

  /**
   * @brief 步骤6：创建轨迹评估器
   *
   * TrajectoryEvaluator评估所有轨迹对的可行性
   * 并按成本排序
   *
   * C++语法说明：
   * - TrajectoryEvaluator：
   *   评估器类，计算轨迹成本
   *   包括安全性、舒适性、效率等指标
   */
  // 6. first, evaluate the feasibility of the 1d trajectories according to
  // dynamic constraints.
  //   second, evaluate the feasible longitudinal and lateral trajectory pairs
  //   and sort them according to the cost.
  TrajectoryEvaluator trajectory_evaluator(
      init_s, planning_target, lon_trajectory1d_bundle, lat_trajectory1d_bundle,
      ptr_path_time_graph, ptr_reference_line);

  ADEBUG << "Trajectory_Evaluator_Construction_Time = "
         << (Clock::NowInSeconds() - current_time) * 1000;
  current_time = Clock::NowInSeconds();

  /**
   * @brief 输出轨迹数量统计日志
   *
   * C++语法说明：
   * - trajectory_evaluator.num_of_trajectory_pairs()：
   *   获取轨迹对数量
   * - lon_trajectory1d_bundle.size()：
   *   获取向量大小
   */
  ADEBUG << "number of trajectory pairs = "
         << trajectory_evaluator.num_of_trajectory_pairs()
         << "  number_lon_traj = " << lon_trajectory1d_bundle.size()
         << "  number_lat_traj = " << lat_trajectory1d_bundle.size();

  /**
   * @brief 创建碰撞检查器和约束检查器
   *
   * CollisionChecker检查轨迹是否与障碍物碰撞
   *
   * C++语法说明：
   * - CollisionChecker collision_checker(...)：
   *   值拷贝创建局部对象
   *   参数包括障碍物、初始状态、参考线等
   */
  // Get instance of collision checker and constraint checker
  CollisionChecker collision_checker(frame->obstacles(), init_s[0], init_d[0],
                                     *ptr_reference_line, reference_line_info,
                                     ptr_path_time_graph);

  /**
   * @brief 步骤7：选择最优无碰撞轨迹
   *
   * 遍历所有轨迹对，选择第一个满足约束且无碰撞的轨迹
   *
   * C++语法说明：
   * - size_t constraint_failure_count = 0：
   *   约束失败计数
   *
   * - while (trajectory_evaluator.has_more_trajectory_pairs())：
   *   循环直到没有更多轨迹对
   *   轨迹评估器按成本排序，每次返回最优的
   */
  // 7. always get the best pair of trajectories to combine; return the first
  // collision-free trajectory.
  size_t constraint_failure_count = 0;  /**< 约束失败次数 */
  size_t collision_failure_count = 0;  /**< 碰撞失败次数 */
  size_t combined_constraint_failure_count = 0;  /**< 组合约束失败次数 */

  size_t lon_vel_failure_count = 0;  /**< 纵向速度超限次数 */
  size_t lon_acc_failure_count = 0;  /**< 纵向加速度超限次数 */
  size_t lon_jerk_failure_count = 0;  /**< 纵向jerk超限次数 */
  size_t curvature_failure_count = 0;  /**< 曲率超限次数 */
  size_t lat_acc_failure_count = 0;  /**< 横向加速度超限次数 */
  size_t lat_jerk_failure_count = 0;  /**< 横向jerk超限次数 */

  size_t num_lattice_traj = 0;  /**< 有效轨迹数量 */

  /**
   * @brief 主循环：评估轨迹对
   */
  while (trajectory_evaluator.has_more_trajectory_pairs()) {
    /**
     * @brief 获取最优轨迹对的成本
     */
    double trajectory_pair_cost =
        trajectory_evaluator.top_trajectory_pair_cost();

    /**
     * @brief 获取最优轨迹对
     *
     * C++语法说明：
     * - trajectory_evaluator.next_top_trajectory_pair()：
     *   返回pair<shared_ptr<Curve1d>, shared_ptr<Curve1d>>
     *   first是纵向轨迹，second是横向轨迹
     */
    auto trajectory_pair = trajectory_evaluator.next_top_trajectory_pair();

    /**
     * @brief 合并两条1维轨迹为一条2维轨迹
     *
     * TrajectoryCombiner::Combine将纵向和横向轨迹组合
     *
     * C++语法说明：
     * - TrajectoryCombiner::Combine(...)：
     *   静态函数调用
     *   参数包括参考线、纵向轨迹、横向轨迹
     */
    // combine two 1d trajectories to one 2d trajectory
    auto combined_trajectory = TrajectoryCombiner::Combine(
        *ptr_reference_line, *trajectory_pair.first, *trajectory_pair.second,
        planning_init_point.relative_time());

    /**
     * @brief 检查轨迹约束
     *
     * ConstraintChecker::ValidTrajectory检查：
     * - 纵向速度、加速度、jerk
     * - 横向加速度、jerk
     * - 曲率
     *
     * C++语法说明：
     * - auto result = ConstraintChecker::ValidTrajectory(...)：
     *   自动类型推导
     *   Result是约束检查结果的枚举类型
     */
    // check longitudinal and lateral acceleration
    // considering trajectory curvatures
    auto result = ConstraintChecker::ValidTrajectory(combined_trajectory);

    /**
     * @brief 处理约束检查失败
     *
     * C++语法说明：
     * - switch (result)：
     *   switch语句处理不同约束类型
     *
     * - case ConstraintChecker::Result::LON_VELOCITY_OUT_OF_BOUND:
     *   枚举类型成员访问
     *   ::作用域解析运算符
     *
     * - ++lon_vel_failure_count：
     *   前置自增运算符
     */
    if (result != ConstraintChecker::Result::VALID) {
      ++combined_constraint_failure_count;

      switch (result) {
        case ConstraintChecker::Result::LON_VELOCITY_OUT_OF_BOUND:
          lon_vel_failure_count += 1;
          break;
        case ConstraintChecker::Result::LON_ACCELERATION_OUT_OF_BOUND:
          lon_acc_failure_count += 1;
          break;
        case ConstraintChecker::Result::LON_JERK_OUT_OF_BOUND:
          lon_jerk_failure_count += 1;
          break;
        case ConstraintChecker::Result::CURVATURE_OUT_OF_BOUND:
          curvature_failure_count += 1;
          break;
        case ConstraintChecker::Result::LAT_ACCELERATION_OUT_OF_BOUND:
          lat_acc_failure_count += 1;
          break;
        case ConstraintChecker::Result::LAT_JERK_OUT_OF_BOUND:
          lat_jerk_failure_count += 1;
          break;
        case ConstraintChecker::Result::VALID:
        default:
          // Intentional empty
          break;
      }
      continue;  /**< 跳过本次循环，继续评估下一个轨迹对 */
    }

    /**
     * @brief 检查碰撞
     *
     * CollisionChecker::InCollision判断轨迹是否与障碍物碰撞
     *
     * C++语法说明：
     * - if (collision_checker.InCollision(combined_trajectory))：
     *   true表示有碰撞
     */
    // check collision with other obstacles
    if (collision_checker.InCollision(combined_trajectory)) {
      ++collision_failure_count;
      continue;  /**< 有碰撞，跳过 */
    }

    /**
     * @brief 找到有效轨迹，设置输出
     *
     * C++语法说明：
     * - reference_line_info->SetTrajectory(...)：
     *   设置规划轨迹
     *
     * - SetCost(... + ...)：
     *   设置轨迹成本 = 优先级成本 + 轨迹对成本
     *
     * - SetDrivable(true)：
     *   设置为可行驶
     */
    // put combine trajectory into debug data
    const auto& combined_trajectory_points = combined_trajectory;
    num_lattice_traj += 1;
    reference_line_info->SetTrajectory(combined_trajectory);
    reference_line_info->SetCost(reference_line_info->PriorityCost() +
                                 trajectory_pair_cost);
    reference_line_info->SetDrivable(true);

    /**
     * @brief 输出调试信息
     *
     * 打印选择的轨迹的起止状态
     *
     * C++语法说明：
     * - std::dynamic_pointer_cast<LatticeTrajectory1d>(...)：
     *   智能指针的动态类型转换
     *   将Curve1d指针转换为LatticeTrajectory1d指针
     *   用于访问特定的派生类方法
     */
    // Print the chosen end condition and start condition
    ADEBUG << "Starting Lon. State: s = " << init_s[0] << " ds = " << init_s[1]
           << " dds = " << init_s[2];
    // cast
    auto lattice_traj_ptr =
        std::dynamic_pointer_cast<LatticeTrajectory1d>(trajectory_pair.first);
    if (!lattice_traj_ptr) {
      ADEBUG << "Dynamically casting trajectory1d ptr. failed.";
    }

    if (lattice_traj_ptr->has_target_position()) {
      ADEBUG << "Ending Lon. State s = " << lattice_traj_ptr->target_position()
             << " ds = " << lattice_traj_ptr->target_velocity()
             << " t = " << lattice_traj_ptr->target_time();
    }

    ADEBUG << "InputPose";
    ADEBUG << "XY: " << planning_init_point.ShortDebugString();
    ADEBUG << "S: (" << init_s[0] << ", " << init_s[1] << "," << init_s[2]
           << ")";
    ADEBUG << "L: (" << init_d[0] << ", " << init_d[1] << "," << init_d[2]
           << ")";

    ADEBUG << "Reference_line_priority_cost = "
           << reference_line_info->PriorityCost();
    ADEBUG << "Total_Trajectory_Cost = " << trajectory_pair_cost;
    ADEBUG << "OutputTrajectory";
    for (uint i = 0; i < 10; ++i) {
      ADEBUG << combined_trajectory_points[i].ShortDebugString();
    }

    break;  /**< 找到有效轨迹，退出循环 */
    /*
    auto combined_trajectory_path =
        ptr_debug->mutable_planning_data()->add_trajectory_path();
    for (uint i = 0; i < combined_trajectory_points.size(); ++i) {
      combined_trajectory_path->add_trajectory_point()->CopyFrom(
          combined_trajectory_points[i]);
    }
    combined_trajectory_path->set_lattice_trajectory_cost(trajectory_pair_cost);
    */
  }

  /**
   * @brief 输出性能统计日志
   */
  ADEBUG << "Trajectory_Evaluation_Time = "
         << (Clock::NowInSeconds() - current_time) * 1000;

  ADEBUG << "Step CombineTrajectory Succeeded";

  ADEBUG << "1d trajectory not valid for constraint ["
         << constraint_failure_count << "] times";
  ADEBUG << "Combined trajectory not valid for ["
         << combined_constraint_failure_count << "] times";
  ADEBUG << "Trajectory not valid for collision [" << collision_failure_count
         << "] times";
  ADEBUG << "Total_Lattice_Planning_Frame_Time = "
         << (Clock::NowInSeconds() - start_time) * 1000;

  /**
   * @brief 返回规划结果
   *
   * 如果找到有效轨迹，返回OK
   * 否则尝试生成备用轨迹或返回错误
   */
  if (num_lattice_traj > 0) {
    ADEBUG << "Planning succeeded";
    num_planning_succeeded_cycles += 1;
    reference_line_info->SetDrivable(true);
    return Status::OK();
  } else {
    AERROR << "Planning failed";

    /**
     * @brief 尝试生成备用轨迹
     *
     * 当没有找到可行轨迹时
     * BackupTrajectoryGenerator生成安全但可能不最优的轨迹
     *
     * C++语法说明：
     * - FLAGS_enable_backup_trajectory：
     *   GFlags布尔标志
     *   控制是否启用备用轨迹
     */
    if (FLAGS_enable_backup_trajectory) {
      AERROR << "Use backup trajectory";
      BackupTrajectoryGenerator backup_trajectory_generator(
          init_s, init_d, planning_init_point.relative_time(),
          std::make_shared<CollisionChecker>(collision_checker),
          &trajectory1d_generator);

      /**
       * @brief 生成并设置备用轨迹
       */
      DiscretizedTrajectory trajectory =
          backup_trajectory_generator.GenerateTrajectory(*ptr_reference_line);

      reference_line_info->AddCost(FLAGS_backup_trajectory_cost);
      reference_line_info->SetTrajectory(trajectory);
      reference_line_info->SetDrivable(true);
      return Status::OK();

    } else {
      /**
       * @brief 无法生成备用轨迹，返回错误
       *
       * C++语法说明：
       * - std::numeric_limits<double>::infinity()：
       *   double类型的正无穷大
       *   表示最高的成本
       */
      reference_line_info->SetCost(std::numeric_limits<double>::infinity());
      return Status(ErrorCode::PLANNING_ERROR, "No feasible trajectories");
    }
  }
}

}  // namespace planning
}  // namespace apollo
