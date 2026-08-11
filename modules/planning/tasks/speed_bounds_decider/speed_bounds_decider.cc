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
 * @file speed_bounds_decider.cc
 * @brief 速度边界决策器实现文件
 *
 * 本文件实现了SpeedBoundsDecider类，负责生成速度规划的边界约束。
 *
 * 核心功能：
 * 1. 将障碍物映射到ST图（时空图）
 * 2. 生成沿路径的速度限制
 * 3. 计算速度回退距离
 *
 * ST图概念：
 * - S轴：沿路径的累积距离
 * - T轴：时间
 * - ST边界：障碍物在时空上的占用区域
 *
 * 设计理念：
 * - 速度规划前需要知道障碍物的时空占用
 * - 速度必须满足ST边界的约束
 * - 速度限制来自道路曲率、限速标志等
 *
 * C++语法说明：
 * - std::chrono: 时间处理
 * - std::vector: 动态数组容器
 * - std::shared_ptr: 引用计数智能指针
 * - const引用: 避免拷贝提高效率
 */

#include "modules/planning/tasks/speed_bounds_decider/speed_bounds_decider.h"

/**
 * @brief 标准库头文件
 * <algorithm>: 提供std::min, std::max, std::fmax等算法
 * <limits>: 提供std::numeric_limits获取数值极限
 * <memory>: 提供std::shared_ptr智能指针
 * <string>: 提供std::string字符串
 * <vector>: 提供std::vector动态数组
 */
#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "modules/common/vehicle_state/vehicle_state_provider.h"
/**
 * @brief 车辆状态提供者
 * 用于获取车辆当前状态
 */
#include "modules/planning/planning_base/common/path/path_data.h"
/**
 * @brief 路径数据结构
 * PathData: 存储路径点序列
 */
#include "modules/planning/planning_base/common/planning_context.h"
/**
 * @brief 规划上下文
 * 存储规划过程中的共享状态
 */
#include "modules/planning/planning_base/common/st_graph_data.h"
/**
 * @brief ST图数据
 * 存储速度规划的时空边界数据
 */
#include "modules/planning/planning_base/common/util/common.h"
/**
 * @brief 通用工具函数
 */
#include "modules/planning/planning_base/gflags/planning_gflags.h"
/**
 * @brief Planning模块GFlags配置
 */
#include "modules/planning/tasks/speed_bounds_decider/speed_limit_decider.h"
/**
 * @brief 速度限制决策器
 * 计算道路曲率和限速标志产生的速度限制
 */
#include "modules/planning/tasks/speed_bounds_decider/st_boundary_mapper.h"
/**
 * @brief ST边界映射器
 * 将障碍物映射到ST图
 */

namespace apollo {
/**
 * @brief Apollo顶层命名空间
 */
namespace planning {

/**
 * @brief 类型别名定义
 * using: 类型别名，简化代码书写
 */
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::planning_internal::StGraphBoundaryDebug;
using apollo::planning_internal::STGraphDebug;

/**
 * @brief 初始化速度边界决策器
 *
 * @param config_dir 配置文件目录
 * @param name 任务名称
 * @param injector 依赖注入器
 * @return bool 初始化是否成功
 *
 * 初始化流程：
 * 1. 调用基类Decider::Init进行基础初始化
 * 2. 加载SpeedBoundsDeciderConfig配置
 */
bool SpeedBoundsDecider::Init(
    const std::string &config_dir, const std::string &name,
    const std::shared_ptr<DependencyInjector> &injector) {
  /**
   * @brief 调用基类初始化
   */
  if (!Decider::Init(config_dir, name, injector)) {
    return false;
  }

  /**
   * @brief 加载任务特定配置
   */
  return Decider::LoadConfig<SpeedBoundsDeciderConfig>(&config_);
}

/**
 * @brief 执行速度边界决策
 *
 * @param frame 当前规划帧
 * @param reference_line_info 参考线信息
 * @return Status 执行状态
 *
 * 主要流程：
 * 1. 映射障碍物到ST图
 * 2. 创建速度限制
 * 3. 保存ST图数据到frame
 *
 * C++语法说明：
 * - Frame *const frame: 指向常量的指针
 * - ReferenceLineInfo *const reference_line_info: 参考线信息指针
 */
Status SpeedBoundsDecider::Process(
    Frame *const frame, ReferenceLineInfo *const reference_line_info) {
  /**
   * @brief 获取路径数据
   * const PathData &: 常量引用，避免拷贝
   */
  const PathData &path_data = reference_line_info->path_data();

  /**
   * @brief 获取规划起点
   * TrajectoryPoint: 包含位置、速度、加速度等
   */
  const TrajectoryPoint &init_point = frame->PlanningStartPoint();

  /**
   * @brief 获取参考线
   */
  const ReferenceLine &reference_line = reference_line_info->reference_line();

  /**
   * @brief 获取路径决策
   * PathDecision *const: 指向常量的指针，可修改指向的对象
   */
  PathDecision *const path_decision = reference_line_info->path_decision();

  /**
   * @brief 步骤1：映射障碍物到ST图
   *
   * 使用chrono获取当前时间用于性能测量
   * std::chrono::system_clock::now(): 获取当前时间点
   */
  auto time1 = std::chrono::system_clock::now();

  /**
   * @brief 创建ST边界映射器
   * STBoundaryMapper: 将障碍物映射到ST（时空）图
   * 参数：
   * - config_: 配置参数
   * - reference_line: 参考线
   * - path_data: 路径数据
   * - path_data.discretized_path().Length(): 路径长度作为S轴范围
   * - config_.total_time(): 总时间作为T轴范围
   * - injector_: 依赖注入器
   */
  STBoundaryMapper boundary_mapper(config_, reference_line, path_data,
                                   path_data.discretized_path().Length(),
                                   config_.total_time(), injector_);

  /**
   * @brief 检查是否使用ST可行驶边界
   * FLAGS_use_st_drivable_boundary: 配置开关
   */
  if (!FLAGS_use_st_drivable_boundary) {
    /**
     * @brief 清除现有的ST边界
     * 用于重新计算
     */
    path_decision->EraseStBoundaries();
  }

  /**
   * @brief 计算ST边界
   * boundary_mapper.ComputeSTBoundary():
   *   核心算法：将障碍物映射为ST图中的多边形区域
   * 返回ErrorCode判断是否成功
   *
   * C++语法说明：
   * - .code(): 获取错误码
   * - ErrorCode::PLANNING_ERROR: 错误码枚举值
   */
  if (boundary_mapper.ComputeSTBoundary(path_decision).code() ==
      ErrorCode::PLANNING_ERROR) {
    const std::string msg = "Mapping obstacle failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  /**
   * @brief 计算ST边界映射耗时
   */
  auto time2 = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = time2 - time1;

  /**
   * @brief 打印耗时调试信息
   * diff.count(): 将duration转换为秒
   * *1000: 转换为毫秒
   */
  ADEBUG << "Time for ST Boundary Mapping = " << diff.count() * 1000
         << " msec.";

  /**
   * @brief 收集所有有效的ST边界
   * std::vector<const STBoundary *>: 存储ST边界指针的向量
   */
  std::vector<const STBoundary *> boundaries;

  /**
   * @brief 遍历所有障碍物
   * for (auto *obstacle : path_decision->obstacles().Items()):
   *   范围for循环遍历障碍物列表
   */
  for (auto *obstacle : path_decision->obstacles().Items()) {
    /**
     * @brief 获取障碍物ID
     */
    const auto &id = obstacle->Id();

    /**
     * @brief 获取障碍物的ST边界
     */
    const auto &st_boundary = obstacle->path_st_boundary();

    /**
     * @brief 检查ST边界是否为空
     * IsEmpty(): 判断边界是否有效
     */
    if (!st_boundary.IsEmpty()) {
      /**
       * @brief 判断边界类型
       * KEEP_CLEAR: 禁行区（如斑马线）
       * 设置为非阻塞障碍物
       */
      if (st_boundary.boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
        path_decision->Find(id)->SetBlockingObstacle(false);
      } else {
        /**
         * @brief 其他类型设为阻塞障碍物
         */
        path_decision->Find(id)->SetBlockingObstacle(true);
      }

      /**
       * @brief 打印调试信息
       */
      st_boundary.PrintDebug("_obs_st_bounds");

      /**
       * @brief 添加到边界列表
       */
      boundaries.push_back(&st_boundary);
    }
  }

  /**
   * @brief 设置速度回退距离
   * 用于速度规划失败时的回退策略
   */
  const double min_s_on_st_boundaries = SetSpeedFallbackDistance(path_decision);

  /**
   * @brief 步骤2：创建速度限制
   * SpeedLimitDecider: 根据道路曲率、限速标志等计算速度限制
   */
  SpeedLimitDecider speed_limit_decider(config_, reference_line, path_data);

  /**
   * @brief 速度限制数据结构
   * SpeedLimit: 存储沿路径各点的速度上限
   */
  SpeedLimit speed_limit;

  /**
   * @brief 获取速度限制
   * GetSpeedLimits():
   *   遍历所有障碍物，计算综合速度限制
   */
  if (!speed_limit_decider
           .GetSpeedLimits(path_decision->obstacles(), &speed_limit)
           .ok()) {
    const std::string msg = "Getting speed limits failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  /**
   * @brief 步骤3：获取路径长度作为S轴搜索范围
   * path_data.discretized_path().Length():
   *   离散路径的总长度
   */
  const double path_data_length = path_data.discretized_path().Length();

  /**
   * @brief 步骤4：获取时间范围作为T轴搜索范围
   * config_.total_time():
   *   从配置获取总规划时间
   */
  const double total_time_by_conf = config_.total_time();

  /**
   * @brief 将生成的ST图数据加载回frame
   * reference_line_info_->mutable_st_graph_data():
   *   获取可修改的ST图数据指针
   */
  StGraphData *st_graph_data = reference_line_info_->mutable_st_graph_data();

  /**
   * @brief 创建ST图调试信息
   * mutable_debug(): 获取可修改的调试信息
   * add_st_graph(): 添加一个ST图调试条目
   */
  auto *debug = reference_line_info_->mutable_debug();
  STGraphDebug *st_graph_debug = debug->mutable_planning_data()->add_st_graph();

  /**
   * @brief 加载ST图数据
   * st_graph_data->LoadData():
   *   将所有边界、速度限制等数据存储到ST图数据对象
   */
  st_graph_data->LoadData(boundaries, min_s_on_st_boundaries, init_point,
                          speed_limit, reference_line_info->GetCruiseSpeed(),
                          path_data_length, total_time_by_conf, st_graph_debug);

  /**
   * @brief 记录ST图调试信息
   * RecordSTGraphDebug():
   *   将边界和速度限制数据保存到调试结构
   */
  RecordSTGraphDebug(*st_graph_data, st_graph_debug);

  return Status::OK();
}

/**
 * @brief 设置速度回退距离
 *
 * @param path_decision 路径决策
 * @return double 速度回退的最小s坐标
 *
 * 功能说明：
 * - 计算所有障碍物ST边界的最小s坐标
 * - 用于速度规划失败时的回退策略
 * - 区分同向和逆向障碍物
 *
 * 算法流程：
 * 1. 遍历所有障碍物的ST边界
 * 2. 找到边界最低点（时间=0时刻）的s坐标
 * 3. 区分同向(min_s_non_reverse)和逆向(min_s_reverse)
 * 4. 返回较小的非逆向障碍物s坐标
 *
 * C++语法说明：
 * - static constexpr: 编译期常量
 * - std::numeric_limits<double>::infinity():
 *   获取double类型的正无穷大
 * - std::min/std::max: 极值函数
 */
double SpeedBoundsDecider::SetSpeedFallbackDistance(
    PathDecision *const path_decision) {
  /**
   * @brief 极小值常量
   * 用于浮点数比较，避免精度问题
   */
  static constexpr double kEpsilon = 1.0e-6;

  /**
   * @brief 初始化最小s坐标为无穷大
   * min_s_non_reverse: 同向障碍物的最小s坐标
   * min_s_reverse: 逆向障碍物的最小s坐标
   */
  double min_s_non_reverse = std::numeric_limits<double>::infinity();
  double min_s_reverse = std::numeric_limits<double>::infinity();

  /**
   * @brief 遍历所有障碍物
   */
  for (auto *obstacle : path_decision->obstacles().Items()) {
    /**
     * @brief 获取障碍物的ST边界
     */
    const auto &st_boundary = obstacle->path_st_boundary();

    /**
     * @brief 跳过空的ST边界
     */
    if (st_boundary.IsEmpty()) {
      continue;
    }

    /**
     * @brief 获取ST边界底部点的s坐标
     * bottom_left_point(): ST边界左下角点（t=0时刻左边界）
     * bottom_right_point(): ST边界右下角点（t=0时刻右边界）
     */
    const auto left_bottom_point_s = st_boundary.bottom_left_point().s();
    const auto right_bottom_point_s = st_boundary.bottom_right_point().s();

    /**
     * @brief 计算边界最低点的s坐标
     * 取左右边界中较小的s值
     */
    const auto lowest_s = std::min(left_bottom_point_s, right_bottom_point_s);

    /**
     * @brief 判断是同向还是逆向障碍物
     * left_bottom_point_s - right_bottom_point_s > kEpsilon:
     *   如果差值大于0，说明是逆向（从后向前行驶）
     */
    if (left_bottom_point_s - right_bottom_point_s > kEpsilon) {
      /**
       * @brief 逆向障碍物
       */
      if (min_s_reverse > lowest_s) {
        min_s_reverse = lowest_s;
      }
    } else {
      /**
       * @brief 同向障碍物
       */
      if (min_s_non_reverse > lowest_s) {
        min_s_non_reverse = lowest_s;
      }
    }
  }

  /**
   * @brief 确保值不为负
   * std::max(value, 0.0): 不小于0
   */
  min_s_reverse = std::max(min_s_reverse, 0.0);
  min_s_non_reverse = std::max(min_s_non_reverse, 0.0);

  /**
   * @brief 返回速度回退距离
   * 如果同向障碍物更近，返回其s坐标
   * 否则返回0.0（使用逆向障碍物的距离）
   */
  return min_s_non_reverse > min_s_reverse ? 0.0 : min_s_non_reverse;
}

/**
 * @brief 记录ST图调试信息
 *
 * @param st_graph_data ST图数据
 * @param st_graph_debug ST图调试信息输出
 *
 * 功能说明：
 * - 将ST边界信息记录到调试结构
 * - 将速度限制信息记录到调试结构
 * - 用于可视化调试和日志分析
 *
 * C++语法说明：
 * - const StGraphData &: 常量引用，输入参数
 * - STGraphDebug *: 指针参数，输出参数
 * - switch-case: 分支语句处理不同边界类型
 */
void SpeedBoundsDecider::RecordSTGraphDebug(
    const StGraphData &st_graph_data, STGraphDebug *st_graph_debug) const {
  /**
   * @brief 检查是否启用调试信息记录
   * FLAGS_enable_record_debug: 配置开关
   */
  if (!FLAGS_enable_record_debug || !st_graph_debug) {
    ADEBUG << "Skip record debug info";
    return;
  }

  /**
   * @brief 遍历所有ST边界
   * st_graph_data.st_boundaries():
   *   获取ST边界的常量引用
   */
  for (const auto &boundary : st_graph_data.st_boundaries()) {
    /**
     * @brief 添加边界调试信息
     * add_boundary(): 添加一个边界条目
     */
    auto boundary_debug = st_graph_debug->add_boundary();

    /**
     * @brief 设置边界名称（障碍物ID）
     */
    boundary_debug->set_name(boundary->id());

    /**
     * @brief 根据边界类型设置调试类型
     * switch-case: 分支语句
     * 每个case设置对应的调试枚举值
     */
    switch (boundary->boundary_type()) {
      /**
       * @brief FOLLOW: 跟随边界
       */
      case STBoundary::BoundaryType::FOLLOW:
        boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_FOLLOW);
        break;

      /**
       * @brief OVERTAKE: 超车边界
       */
      case STBoundary::BoundaryType::OVERTAKE:
        boundary_debug->set_type(
            StGraphBoundaryDebug::ST_BOUNDARY_TYPE_OVERTAKE);
        break;

      /**
       * @brief STOP: 停车边界
       */
      case STBoundary::BoundaryType::STOP:
        boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_STOP);
        break;

      /**
       * @brief UNKNOWN: 未知类型
       */
      case STBoundary::BoundaryType::UNKNOWN:
        boundary_debug->set_type(
            StGraphBoundaryDebug::ST_BOUNDARY_TYPE_UNKNOWN);
        break;

      /**
       * @brief YIELD: 让行边界
       */
      case STBoundary::BoundaryType::YIELD:
        boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_YIELD);
        break;

      /**
       * @brief KEEP_CLEAR: 禁行区边界
       */
      case STBoundary::BoundaryType::KEEP_CLEAR:
        boundary_debug->set_type(
            StGraphBoundaryDebug::ST_BOUNDARY_TYPE_KEEP_CLEAR);
        break;
    }

    /**
     * @brief 记录边界的点序列
     * boundary->points():
     *   获取ST边界的多边形顶点
     * 每个点包含(t, s)坐标
     */
    for (const auto &point : boundary->points()) {
      auto point_debug = boundary_debug->add_point();

      /**
       * @brief 设置点的t和s坐标
       * point.x()是t，point.y()是s
       * 这是因为在ST图中x轴是时间，y轴是距离
       */
      point_debug->set_t(point.x());
      point_debug->set_s(point.y());
    }
  }

  /**
   * @brief 记录速度限制曲线
   * st_graph_data.speed_limit().speed_limit_points():
   *   获取速度限制点序列，每个点是(s, v)对
   */
  for (const auto &point : st_graph_data.speed_limit().speed_limit_points()) {
    /**
     * @brief 添加速度限制点
     */
    common::SpeedPoint *speed_point = st_graph_debug->add_speed_limit();

    /**
     * @brief 设置s坐标和速度值
     * point.first是s，point.second是v
     */
    speed_point->set_s(point.first);
    speed_point->set_v(point.second);
  }
}

/**
 * @brief 命名空间结束标记
 */
}  // namespace planning
}  // namespace apollo
