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
 * @file lane_change_path.cc
 * @brief 换道路径规划任务实现文件
 *
 * 本文件实现了LaneChangePath类，负责换道场景的路径规划。
 *
 * 换道流程：
 * 1. 更新换道状态（检查是否满足换道条件）
 * 2. 决定路径边界（根据车道信息、自车位置、障碍物）
 * 3. 优化路径（使用分段jerk优化算法）
 * 4. 评估路径（选择有效路径）
 *
 * 核心概念：
 * - 换道状态机：CHANGE_LANE_FINISHED, IN_CHANGE_LANE, CHANGE_LANE_FAILED
 * - 路径边界：定义可行驶区域的左右边界
 * - Frenet坐标系：将路径规划问题简化为沿参考线的优化问题
 *
 * C++语法说明：
 * - std::shared_ptr<T>: 智能指针，引用计数，线程安全
 * - using apollo::common::math::Vec2d: 类型别名，简化代码
 */

#include "modules/planning/tasks/lane_change_path/lane_change_path.h"

/**
 * @brief 标准库头文件
 * <algorithm>: 提供std::max, std::min, std::fmax, std::fmin等算法
 * <limits>: 提供std::numeric_limits获取数值极限
 * <memory>: 提供智能指针std::shared_ptr
 * <string>: 提供std::string字符串类型
 * <utility>: 提供std::pair, std::move等工具
 * <vector>: 提供std::vector动态数组
 */
#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "cyber/time/clock.h"
/**
 * @brief Cyber RT时间系统
 * Clock::NowInSeconds(): 获取当前时间戳（秒）
 */
#include "modules/common/configs/vehicle_config_helper.h"
/**
 * @brief 车辆配置辅助类
 * VehicleConfigHelper::GetConfig(): 获取车辆配置
 */
#include "modules/planning/planning_base/common/planning_context.h"
/**
 * @brief 规划上下文
 * 存储规划过程中的共享状态，如换道状态
 */
#include "modules/planning/planning_interface_base/task_base/common/path_generation.h"
/**
 * @brief 路径生成基类
 * Task: 所有规划任务的基类
 */
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_assessment_decider_util.h"
/**
 * @brief 路径评估工具
 * 评估路径是否有效、安全
 */
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_bounds_decider_util.h"
/**
 * @brief 路径边界决策工具
 * 决定路径的左右边界
 */
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_optimizer_util.h"
/**
 * @brief 路径优化工具
 * 使用QP算法优化路径
 */

namespace apollo {
/**
 * @brief Apollo顶层命名空间
 */
namespace planning {

/**
 * @brief 类型别名定义
 * using: 类型别名，简化代码书写
 *
 * 作用：
 * - 减少代码冗余
 * - 提高代码可读性
 * - 方便类型更改
 */
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleConfigHelper;
using apollo::common::math::Box2d;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;
using apollo::cyber::Clock;

/**
 * @brief 常量定义
 * constexpr: 编译期常量，比#define更类型安全
 *
 * kIntersectionClearanceDist: 交叉路口清除距离（米）
 * kJunctionClearanceDist: 汇入区清除距离（米）
 */
constexpr double kIntersectionClearanceDist = 20.0;
constexpr double kJunctionClearanceDist = 15.0;

/**
 * @brief 初始化换道路径任务
 *
 * @param config_dir 配置文件目录
 * @param name 任务名称
 * @param injector 依赖注入器
 * @return bool 初始化是否成功
 *
 * 初始化流程：
 * 1. 调用基类Task::Init进行基础初始化
 * 2. 加载LaneChangePathConfig配置
 *
 * C++语法说明：
 * - std::shared_ptr<DependencyInjector>: 智能指针，管理依赖注入
 * - &config_: 成员变量初始化
 */
bool LaneChangePath::Init(const std::string& config_dir,
                          const std::string& name,
                          const std::shared_ptr<DependencyInjector>& injector) {
  /**
   * @brief 调用基类初始化
   * Task::Init: 基类Task的初始化函数
   * 如果失败则返回false
   */
  if (!Task::Init(config_dir, name, injector)) {
    return false;
  }

  /**
   * @brief 加载任务特定配置
   * Task::LoadConfig<T>: 模板函数，从配置文件加载配置
   * &config_: 将配置存储到成员变量config_
   */
  return Task::LoadConfig<LaneChangePathConfig>(&config_);
}

/**
 * @brief 执行换道路径规划
 *
 * @param frame 当前规划帧
 * @param reference_line_info 参考线信息
 * @return apollo::common::Status 执行状态
 *
 * 主要流程：
 * 1. 更新换道状态
 * 2. 检查是否需要换道
 * 3. 决定路径边界
 * 4. 优化路径
 * 5. 评估并选择最优路径
 *
 * C++语法说明：
 * - Frame*: 原始指针，需要保证frame生命周期有效
 * - ReferenceLineInfo*: 参考线信息，包含车道、障碍物等
 * - Status: Apollo状态类型，包含错误码和消息
 */
apollo::common::Status LaneChangePath::Process(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  /**
   * @brief 更新换道状态
   * 检查当前是否满足换道条件
   */
  UpdateLaneChangeStatus();

  /**
   * @brief 获取当前换道状态
   * injector_: 依赖注入器，访问规划上下文
   * mutable_planning_status(): 获取可修改的规划状态
   * mutable_change_lane(): 获取可修改的换道状态
   * status(): 获取当前换道状态枚举值
   *
   * C++语法说明：
   * - injector_->xxx: 箭头运算符，访问智能指针指向对象的成员
   * - chained call: 链式调用，可读性高
   */
  const auto& status = injector_->planning_context()
                           ->mutable_planning_status()
                           ->mutable_change_lane()
                           ->status();

  /**
   * @brief 检查是否是换道路径且路径不可复用
   * IsChangeLanePath(): 判断当前是否在换道过程中
   * path_reusable(): 判断路径是否可以复用
   */
  if (!reference_line_info->IsChangeLanePath() ||
      reference_line_info->path_reusable()) {
    ADEBUG << "Skip this time" << reference_line_info->IsChangeLanePath()
           << "path reusable" << reference_line_info->path_reusable();
    return Status::OK();  /**< 返回成功状态 */
  }

  /**
   * @brief 检查是否正在进行换道
   * ChangeLaneStatus::IN_CHANGE_LANE: 换道中状态
   */
  if (status != ChangeLaneStatus::IN_CHANGE_LANE) {
    ADEBUG << injector_->planning_context()
                  ->mutable_planning_status()
                  ->mutable_change_lane()
                  ->DebugString();
    return Status(ErrorCode::PLANNING_ERROR,
                  "Not satisfy lane change  conditions");
  }

  /**
   * @brief 创建候选路径容器
   * std::vector<PathBoundary>: 路径边界数组
   * std::vector<PathData>: 路径数据数组
   */
  std::vector<PathBoundary> candidate_path_boundaries;
  std::vector<PathData> candidate_path_data;

  /**
   * @brief 获取起始点SL状态
   * init_sl_state_: 存储初始状态的成员变量
   */
  GetStartPointSLState();

  /**
   * @brief 决定路径边界
   * 输入：参考线信息、初始SL状态
   * 输出：候选路径边界数组
   */
  if (!DecidePathBounds(&candidate_path_boundaries)) {
    return Status(ErrorCode::PLANNING_ERROR, "lane change path bounds failed");
  }

  /**
   * @brief 优化路径
   * 输入：路径边界
   * 输出：候选路径数据数组
   */
  if (!OptimizePath(candidate_path_boundaries, &candidate_path_data)) {
    return Status(ErrorCode::PLARNING_ERROR,
                  "lane change path optimize failed");
  }

  /**
   * @brief 评估路径并选择最优
   * 输入：候选路径数组
   * 输出：最终路径数据
   */
  if (!AssessPath(&candidate_path_data,
                  reference_line_info->mutable_path_data())) {
    return Status(ErrorCode::PLANNING_ERROR, "No valid lane change path");
  }

  return Status::OK();
}

/**
 * @brief 决定路径边界
 *
 * @param boundary 输出：路径边界数组
 * @return bool 是否成功
 *
 * 路径边界决定流程：
 * 1. 初始化为无限大区域
 * 2. 根据自车道信息收窄边界
 * 3. 根据自车位置扩展边界
 * 4. 移除换道禁行区的边界
 * 5. 根据静态障碍物调整边界
 *
 * C++语法说明：
 * - std::vector<PathBoundary>*: 指针参数，用于输出
 * - emplace_back(): 原位构造并添加元素
 * - boundary->back(): 获取最后一个元素的引用
 */
bool LaneChangePath::DecidePathBounds(std::vector<PathBoundary>* boundary) {
  /**
   * @brief 添加一个空的路径边界
   * emplace_back(): 在vector末尾构造一个空对象
   */
  boundary->emplace_back();

  /**
   * @brief 获取边界引用
   * auto&: 自动类型推导+引用，避免拷贝
   */
  auto& path_bound = boundary->back();

  /**
   * @brief 路径最窄宽度
   * 初始化为0，后续计算实际最窄宽度
   */
  double path_narrowest_width = 0;

  /**
   * @brief 第1步：初始化路径边界为无限大区域
   * PathBoundsDeciderUtil::InitPathBoundary():
   *   将边界初始化为足够大的区域
   * reference_line_info_: 参考线信息引用
   * init_sl_state_: 初始SL状态
   */
  if (!PathBoundsDeciderUtil::InitPathBoundary(*reference_line_info_,
                                               &path_bound, init_sl_state_)) {
    const std::string msg = "Failed to initialize path boundaries.";
    AERROR << msg;
    return false;
  }

  /**
   * @brief 第2步：根据自车道信息决定粗糙边界
   * GetBoundaryFromSelfLane():
   *   根据当前车道宽度、位置收窄边界
   */
  if (!PathBoundsDeciderUtil::GetBoundaryFromSelfLane(
          *reference_line_info_, init_sl_state_, &path_bound)) {
    AERROR << "Failed to decide a rough boundary based on self lane.";
    return false;
  }

  /**
   * @brief 第3步：根据自车位置扩展边界
   * GetBoundaryFromSelfLane():
   *   考虑自车几何尺寸，确保自车在边界内
   * config_.extend_adc_buffer(): 自车扩展缓冲区大小
   */
  if (!PathBoundsDeciderUtil::ExtendBoundaryByADC(
          *reference_line_info_, init_sl_state_, config_.extend_adc_buffer(),
          &path_bound)) {
    AERROR << "Failed to decide a rough boundary based on adc.";
    return false;
  }

  /**
   * @brief 第4步：移除换道禁行区的边界
   * GetBoundaryFromLaneChangeForbiddenZone():
   *   在禁行区域（十字路口、汇入区等）不允许换道
   */
  GetBoundaryFromLaneChangeForbiddenZone(&path_bound);

  /**
   * @brief 设置路径标签
   * 用于标识路径类型，便于调试和日志分析
   */
  path_bound.set_label("regular/lane_change");

  /**
   * @brief 保存临时边界用于后续追加
   * 当路径被阻塞时，可能需要追加尾部边界点
   */
  PathBound temp_path_bound = path_bound;
  std::string blocking_obstacle_id;  /**< 阻塞障碍物ID */
  obs_sl_polygons_.clear();  /**< 清空之前的SL多边形 */

  /**
   * @brief 获取SL多边形（障碍物）
   * GetSLPolygons():
   *   将障碍物转换到SL坐标系
   */
  PathBoundsDeciderUtil::GetSLPolygons(*reference_line_info_, &obs_sl_polygons_,
                                       init_sl_state_);

  /**
   * @brief 第5步：根据静态障碍物调整边界
   * GetBoundaryFromStaticObstacles():
   *   将障碍物占用区域从边界中移除
   * &blocking_obstacle_id: 输出阻塞障碍物ID
   * &path_narrowest_width: 输出最窄路径宽度
   */
  if (!PathBoundsDeciderUtil::GetBoundaryFromStaticObstacles(
          *reference_line_info_, &obs_sl_polygons_, init_sl_state_, &path_bound,
          &blocking_obstacle_id, &path_narrowest_width)) {
    AERROR << "Failed to decide fine tune the boundaries after "
              "taking into consideration all static obstacles.";
    return false;
  }

  /**
   * @brief 追加尾部边界点，避免零长度路径
   * 当存在阻塞障碍物时，追加尾部点以保证路径有效
   *
   * while循环条件：
   * - blocking_obstacle_id非空（存在阻塞）
   * - path_bound.size() < temp_path_bound.size()（可以追加）
   * - counter < FLAGS_num_extra_tail_bound_point（未超过最大追加数）
   */
  int counter = 0;
  while (!blocking_obstacle_id.empty() &&
         path_bound.size() < temp_path_bound.size() &&
         counter < FLAGS_num_extra_tail_bound_point) {
    path_bound.push_back(temp_path_bound[path_bound.size()]);
    counter++;
  }

  /**
   * @brief 设置阻塞障碍物ID
   */
  path_bound.set_blocking_obstacle_id(blocking_obstacle_id);

  /**
   * @brief 记录调试信息
   */
  RecordDebugInfo(path_bound, path_bound.label(), reference_line_info_);
  return true;
}

/**
 * @brief 优化路径
 *
 * @param path_boundaries 输入：路径边界数组
 * @param candidate_path_data 输出：候选路径数据数组
 * @return bool 是否成功
 *
 * 优化流程：
 * 1. 对每个路径边界进行优化
 * 2. 计算加速度边界
 * 3. 估计jerk边界
 * 4. 调用QP优化器
 * 5. 转换结果为PathData格式
 *
 * C++语法说明：
 * - const std::vector<PathBoundary>&: 常量引用，输入参数
 * - std::vector<PathData>*: 指针参数，输出结果
 * - std::array<double, 3>: 固定大小数组，存储状态[x, dx, ddx]
 */
bool LaneChangePath::OptimizePath(
    const std::vector<PathBoundary>& path_boundaries,
    std::vector<PathData>* candidate_path_data) {
  /**
   * @brief 获取路径优化配置
   */
  const auto& config = config_.path_optimizer_config();

  /**
   * @brief 获取参考线引用
   */
  const ReferenceLine& reference_line = reference_line_info_->reference_line();

  /**
   * @brief 设置终点状态
   * end_state = {0.0, 0.0, 0.0}:
   * - 终点横向位置为0（沿参考线）
   * - 终点横向速度为0
   * - 终点横向加速度为0
   */
  std::array<double, 3> end_state = {0.0, 0.0, 0.0};

  /**
   * @brief 遍历每个路径边界
   * for (const auto& path_boundary : path_boundaries):
   *   范围for循环，遍历容器所有元素
   */
  for (const auto& path_boundary : path_boundaries) {
    /**
     * @brief 获取边界大小
     * path_boundary.boundary().size(): 获取边界点数量
     */
    size_t path_boundary_size = path_boundary.boundary().size();

    /**
     * @brief 检查边界有效性
     * 必须至少有两个点才能进行优化
     */
    if (path_boundary_size <= 1U) {
      AERROR << "Get invalid path boundary with size: " << path_boundary_size;
      return false;
    }

    /**
     * @brief 创建优化结果容器
     * opt_l: 优化后的横向位置
     * opt_dl: 优化后的横向速度
     * opt_ddl: 优化后的横向加速度
     * ddl_bounds: 加速度边界
     */
    std::vector<double> opt_l, opt_dl, opt_ddl;
    std::vector<std::pair<double, double>> ddl_bounds;

    /**
     * @brief 计算加速度边界
     * CalculateAccBound():
     *   根据车辆物理限制和道路曲率计算
     */
    PathOptimizerUtil::CalculateAccBound(path_boundary, reference_line,
                                         &ddl_bounds);

    /**
     * @brief 估计jerk边界
     * EstimateJerkBoundary():
     *   根据车速计算最大允许jerk
     * std::fmax(): 取较大值，确保分母不为零
     */
    const double jerk_bound = PathOptimizerUtil::EstimateJerkBoundary(
        std::fmax(init_sl_state_.first[1], 1e-12));

    /**
     * @brief 初始化参考线和权重
     * ref_l: 参考横向位置（初始化为0）
     * weight_ref_l: 参考线权重（初始化为0）
     */
    std::vector<double> ref_l(path_boundary_size, 0);
    std::vector<double> weight_ref_l(path_boundary_size, 0);

    /**
     * @brief 执行路径优化
     * OptimizePath():
     *   使用分段jerk优化算法
     */
    bool res_opt = PathOptimizerUtil::OptimizePath(
        init_sl_state_, end_state, ref_l, weight_ref_l, path_boundary,
        ddl_bounds, jerk_bound, config, &opt_l, &opt_dl, &opt_ddl);

    /**
     * @brief 如果优化成功，转换结果
     */
    if (res_opt) {
      /**
       * @brief 转换为Frenet帧路径
       * ToPiecewiseJerkPath():
       *   将优化结果(x, dx, ddx)转换为FrenetFramePath
       */
      auto frenet_frame_path = PathOptimizerUtil::ToPiecewiseJerkPath(
          opt_l, opt_dl, opt_ddl, path_boundary.delta_s(),
          path_boundary.start_s());

      /**
       * @brief 创建PathData并设置属性
       */
      PathData path_data;
      path_data.SetReferenceLine(&reference_line);
      path_data.SetFrenetPath(std::move(frenet_frame_path));

      /**
       * @brief 如果使用前轴中心作为规划参考
       * 需要转换为后轴中心
       */
      if (FLAGS_use_front_axe_center_in_path_planning) {
        auto discretized_path = DiscretizedPath(
            PathOptimizerUtil::ConvertPathPointRefFromFrontAxeToRearAxe(
                path_data));
        path_data.SetDiscretizedPath(discretized_path);
      }

      /**
       * @brief 设置路径标签和阻塞障碍物ID
       */
      path_data.set_path_label(path_boundary.label());
      path_data.set_blocking_obstacle_id(path_boundary.blocking_obstacle_id());

      /**
       * @brief 添加到候选路径数组
       * std::move(path_data): 移动语义，避免拷贝
       */
      candidate_path_data->push_back(std::move(path_data));
    }
  }

  /**
   * @brief 检查是否有有效路径
   */
  if (candidate_path_data->empty()) {
    return false;
  }
  return true;
}

/**
 * @brief 评估路径并选择最优
 *
 * @param candidate_path_data 输入/输出：候选路径数组
 * @param final_path 输出：最终路径
 * @return bool 是否找到有效路径
 *
 * 评估流程：
 * 1. 遍历所有候选路径
 * 2. 检查路径是否有效
 * 3. 设置路径信息
 * 4. 裁剪尾部超出车道点
 * 5. 选择第一个有效路径
 *
 * C++语法说明：
 * - std::vector<PathData>*: 指针参数，输入输出
 * - *final_path = xxx: 解引用赋值
 */
bool LaneChangePath::AssessPath(std::vector<PathData>* candidate_path_data,
                                PathData* final_path) {
  /**
   * @brief 有效路径容器
   */
  std::vector<PathData> valid_path_data;

  /**
   * @brief 遍历候选路径
   */
  for (auto& curr_path_data : *candidate_path_data) {
    /**
     * @brief 检查路径是否有效
     * IsValidRegularPath():
     *   检查路径是否满足安全性和可行性要求
     */
    if (PathAssessmentDeciderUtil::IsValidRegularPath(*reference_line_info_,
                                                      curr_path_data)) {
      /**
       * @brief 设置路径信息
       */
      SetPathInfo(&curr_path_data);

      /**
       * @brief 如果接近目的地，裁剪尾部点
       * SDistanceToDestination(): 到目的地的纵向距离
       */
      if (reference_line_info_->SDistanceToDestination() <
          FLAGS_path_trim_destination_threshold) {
        PathAssessmentDeciderUtil::TrimTailingOutLanePoints(&curr_path_data);
      }

      /**
       * @brief 检查裁剪后是否为空
       */
      if (curr_path_data.Empty()) {
        AINFO << "lane change path is empty after trimed";
        continue;  /**< 跳过本次循环 */
      }

      /**
       * @brief 添加到有效路径数组
       */
      valid_path_data.push_back(curr_path_data);
    }
  }

  /**
   * @brief 如果没有有效路径
   */
  if (valid_path_data.empty()) {
    AINFO << "All lane change path are not valid";
    return false;
  }

  /**
   * @brief 选择第一个有效路径
   * 这里简单选择第一个，实际可以更复杂的选择逻辑
   */
  *final_path = valid_path_data[0];

  /**
   * @brief 移动障碍物SL多边形到参考线信息
   * std::move(obs_sl_polygons_):
   *   移动语义，将所有权转移给reference_line_info
   * 移动后obs_sl_polygons_变为空
   */
  *(reference_line_info_->mutable_obs_sl_polygons()) = std::move(obs_sl_polygons_);

  /**
   * @brief 记录调试信息
   */
  RecordDebugInfo(*final_path, final_path->path_label(), reference_line_info_);
  return true;
}

/**
 * @brief 更新换道状态
 *
 * 更新逻辑：
 * 1. 如果没有历史状态，初始化为CHANGE_LANE_FINISHED
 * 2. 如果没有多条参考线，说明换道结束
 * 3. 如果在换道中，检查是否换道成功/失败
 * 4. 处理失败后的冻结时间
 * 5. 处理成功后再次换道
 *
 * C++语法说明：
 * - auto* prev_status = xxx:
   *   auto自动类型推导，*表示原始指针
 * - Clock::NowInSeconds(): 获取当前时间
 */
void LaneChangePath::UpdateLaneChangeStatus() {
  std::string change_lane_id;  /**< 换道ID */

  /**
   * @brief 获取上一个换道状态
   */
  auto* prev_status = injector_->planning_context()
                          ->mutable_planning_status()
                          ->mutable_change_lane();

  /**
   * @brief 获取当前时间戳（秒）
   */
  double now = Clock::NowInSeconds();

  /**
   * @brief 初始化换道状态
   * 如果没有历史状态
   */
  if (!prev_status->has_status()) {
    UpdateStatus(now, ChangeLaneStatus::CHANGE_LANE_FINISHED, "");
    return;
  }

  /**
   * @brief 检查是否有换道需求
   * frame_->reference_line_info().size() > 1:
   *   如果有多条参考线，说明需要换道
   */
  bool has_change_lane = frame_->reference_line_info().size() > 1;

  /**
   * @brief 如果没有换道需求
   */
  if (!has_change_lane) {
    if (prev_status->status() == ChangeLaneStatus::IN_CHANGE_LANE) {
      /**
       * @brief 之前在换道，现在没有换道需求，说明换道完成
       */
      UpdateStatus(now, ChangeLaneStatus::CHANGE_LANE_FINISHED,
                   prev_status->path_id());
    }
    return;
  }

  /**
   * @brief 有换道需求，正在换道中
   */
  if (reference_line_info_->IsChangeLanePath()) {
    /**
     * @brief 获取上一帧
     */
    const auto* history_frame = injector_->frame_history()->Latest();

    /**
     * @brief 检查上一帧是否成功
     */
    if (!CheckLastFrameSucceed(history_frame)) {
      UpdateStatus(now, ChangeLaneStatus::CHANGE_LANE_FAILED, change_lane_id);
      is_exist_lane_change_start_position_ = false;
      return;
    }

    /**
     * @brief 检查是否可以安全换道
     */
    is_clear_to_change_lane_ = IsClearToChangeLane(reference_line_info_);
    change_lane_id = reference_line_info_->Lanes().Id();

    ADEBUG << "change_lane_id" << change_lane_id;

    /**
     * @brief 处理换道失败后的状态
     */
    if (prev_status->status() == ChangeLaneStatus::CHANGE_LANE_FAILED) {
      /**
       * @brief 检查冻结时间是否已过
       * change_lane_fail_freeze_time():
       *   失败后需要等待的时间才能再次尝试
       */
      if (now - prev_status->timestamp() >
          config_.change_lane_fail_freeze_time()) {
        UpdateStatus(now, ChangeLaneStatus::IN_CHANGE_LANE, change_lane_id);
        ADEBUG << "change lane again after failed";
      }
      return;
    }

    /**
     * @brief 处理换道成功后的状态
     */
    else if (prev_status->status() ==
               ChangeLaneStatus::CHANGE_LANE_FINISHED) {
      /**
       * @brief 检查成功冻结时间
       */
      if (now - prev_status->timestamp() >
          config_.change_lane_success_freeze_time()) {
        UpdateStatus(now, ChangeLaneStatus::IN_CHANGE_LANE, change_lane_id);
        AINFO << "change lane again after success";
      }
    }

    /**
     * @brief 处理正在换道中的状态
     */
    else if (prev_status->status() == ChangeLaneStatus::IN_CHANGE_LANE) {
      /**
       * @brief 如果换道ID改变，说明换道完成
       */
      if (prev_status->path_id() != change_lane_id) {
        AINFO << "change_lane_id" << change_lane_id << "prev"
              << prev_status->path_id();
        UpdateStatus(now, ChangeLaneStatus::CHANGE_LANE_FINISHED,
                     prev_status->path_id());
      }
    }
  }
}

/**
 * @brief 检查是否可以安全换道
 *
 * @param reference_line_info 参考线信息
 * @return bool 是否可以安全换道
 *
 * 安全检查：
 * 1. 获取本车位置和速度
 * 2. 遍历所有动态障碍物
 * 3. 计算安全距离（根据方向、同向/逆向）
 * 4. 检查障碍物是否在安全距离内
 *
 * C++语法说明：
 * - std::numeric_limits<double>::max():
   *   获取double类型的最大值
 * - const auto* obstacle: 指向常量的指针
 */
bool LaneChangePath::IsClearToChangeLane(
    ReferenceLineInfo* reference_line_info) {
  /**
   * @brief 获取本车SL边界
   */
  double ego_start_s = reference_line_info->AdcSlBoundary().start_s();
  double ego_end_s = reference_line_info->AdcSlBoundary().end_s();

  /**
   * @brief 获取本车速度
   * std::abs(): 取绝对值
   */
  double ego_v =
      std::abs(reference_line_info->vehicle_state().linear_velocity());

  /**
   * @brief 遍历所有障碍物
   * path_decision()->obstacles().Items():
   *   获取障碍物列表
   */
  for (const auto* obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    /**
     * @brief 跳过虚拟障碍物和静态障碍物
     * 虚拟障碍物用于测试，静态障碍物已由边界处理
     */
    if (obstacle->IsVirtual() || obstacle->IsStatic()) {
      ADEBUG << "skip one virtual or static obstacle";
      continue;
    }

    /**
     * @brief 初始化障碍物SL边界
     */
    double start_s = std::numeric_limits<double>::max();
    double end_s = -std::numeric_limits<double>::max();
    double start_l = std::numeric_limits<double>::max();
    double end_l = -std::numeric_limits<double>::max();

    /**
     * @brief 遍历障碍物感知多边形的所有点
     * 计算障碍物的SL边界
     */
    for (const auto& p : obstacle->PerceptionPolygon().points()) {
      apollo::common::SLPoint sl_point;

      /**
       * @brief 将XY坐标转换为SL坐标
       * XYToSL(): 参考线坐标转换
       */
      reference_line_info->reference_line().XYToSL(p, &sl_point);

      /**
       * @brief 更新边界
       * std::fmin/std::fmax: 浮点数的min/max
       */
      start_s = std::fmin(start_s, sl_point.s());
      end_s = std::fmax(end_s, sl_point.s());

      start_l = std::fmin(start_l, sl_point.l());
      end_l = std::fmax(end_l, sl_point.l());
    }

    /**
     * @brief 检查障碍物是否在目标车道内
     */
    if (reference_line_info->IsChangeLanePath()) {
      double left_width(0), right_width(0);

      /**
       * @brief 获取目标车道宽度
       */
      reference_line_info->mutable_reference_line()->GetLaneWidth(
          (start_s + end_s) * 0.5, &left_width, &right_width);

      /**
       * @brief 如果障碍物完全不在目标车道内，跳过
       */
      if (end_l < -right_width || start_l > left_width) {
        continue;
      }
    }

    /**
     * @brief 判断障碍物是否与本车同向
     * 基于预测轨迹判断
     */
    bool same_direction = true;
    if (obstacle->HasTrajectory()) {
      /**
       * @brief 获取障碍物第一时刻的航向角
       */
      double obstacle_moving_direction =
          obstacle->Trajectory().trajectory_point(0).path_point().theta();

      /**
       * @brief 获取本车航向角
       */
      const auto& vehicle_state = reference_line_info->vehicle_state();
      double vehicle_moving_direction = vehicle_state.heading();

      /**
       * @brief 如果是倒档，调整航向角
       */
      if (vehicle_state.gear() == canbus::Chassis::GEAR_REVERSE) {
        vehicle_moving_direction =
            common::math::NormalizeAngle(vehicle_moving_direction + M_PI);
      }

      /**
       * @brief 计算航向角差异
       * NormalizeAngle(): 将角度标准化到[-π, π]
       */
      double heading_difference = std::abs(common::math::NormalizeAngle(
          obstacle_moving_direction - vehicle_moving_direction));

      /**
       * @brief 如果角度差小于90度，认为同向
       */
      same_direction = heading_difference < (M_PI / 2.0);
    }

    /**
     * @brief 定义安全距离常量
     * 这些值应该移到配置文件中
     */
    static constexpr double kSafeTimeOnSameDirection = 3.0;
    static constexpr double kSafeTimeOnOppositeDirection = 5.0;
    static constexpr double kForwardMinSafeDistanceOnSameDirection = 10.0;
    static constexpr double kBackwardMinSafeDistanceOnSameDirection = 10.0;
    static constexpr double kForwardMinSafeDistanceOnOppositeDirection = 50.0;
    static constexpr double kBackwardMinSafeDistanceOnOppositeDirection = 1.0;
    static constexpr double kDistanceBuffer = 0.5;

    double kForwardSafeDistance = 0.0;
    double kBackwardSafeDistance = 0.0;

    /**
     * @brief 根据方向计算安全距离
     */
    if (same_direction) {
      /**
       * @brief 同向行驶
       * 安全距离 = 速度差 * 时间 + 最小距离
       */
      kForwardSafeDistance =
          std::fmax(kForwardMinSafeDistanceOnSameDirection,
                    (ego_v - obstacle->speed()) * kSafeTimeOnSameDirection);
      kBackwardSafeDistance =
          std::fmax(kBackwardMinSafeDistanceOnSameDirection,
                    (obstacle->speed() - ego_v) * kSafeTimeOnSameDirection);
    } else {
      /**
       * @brief 逆向行驶
       * 安全距离 = (ego_v + obstacle_speed) * 时间 + 最小距离
       */
      kForwardSafeDistance =
          std::fmax(kForwardMinSafeDistanceOnOppositeDirection,
                    (ego_v + obstacle->speed()) * kSafeTimeOnOppositeDirection);
      kBackwardSafeDistance = kBackwardMinSafeDistanceOnOppositeDirection;
    }

    /**
     * @brief 使用滞回滤波器检查安全性
     * HysteresisFilter():
     *   考虑障碍物是否正在阻塞，使用不同的阈值
     */
    if (HysteresisFilter(ego_start_s - end_s, kBackwardSafeDistance,
                         kDistanceBuffer, obstacle->IsLaneChangeBlocking()) &&
        HysteresisFilter(start_s - ego_end_s, kForwardSafeDistance,
                         kDistanceBuffer, obstacle->IsLaneChangeBlocking())) {
      /**
       * @brief 标记障碍物为换道阻塞
       */
      reference_line_info->path_decision()
          ->Find(obstacle->Id())
          ->SetLaneChangeBlocking(true);
      ADEBUG << "Lane Change is blocked by obstacle" << obstacle->Id();
      return false;
    } else {
      reference_line_info->path_decision()
          ->Find(obstacle->Id())
          ->SetLaneChangeBlocking(false);
    }
  }
  return true;
}

/**
 * @brief 获取换道起始点
 *
 * @param reference_line 参考线
 * @param adc_frenet_s 自车Frenet纵向坐标
 * @param start_xy 输出：起始点XY坐标
 *
 * 计算逻辑：
 * 1. 计算换道起始s坐标 = 准备长度 + 自车当前s
 * 2. 设置换道起始l坐标为0（参考线上）
 * 3. 转换为XY坐标
 *
 * C++语法说明：
 * - common::math::Vec2d*: 输出参数指针
 */
void LaneChangePath::GetLaneChangeStartPoint(
    const ReferenceLine& reference_line, double adc_frenet_s,
    common::math::Vec2d* start_xy) {
  /**
   * @brief 计算换道起始s坐标
   * lane_change_prepare_length():
   *   换道准备长度，给予足够的提前量
   */
  double lane_change_start_s =
      config_.lane_change_prepare_length() + adc_frenet_s;

  /**
   * @brief 创建SL点
   */
  common::SLPoint lane_change_start_sl;
  lane_change_start_sl.set_s(lane_change_start_s);
  lane_change_start_sl.set_l(0.0);

  /**
   * @brief 转换为XY坐标
   * SLToXY(): SL坐标转XY坐标
   */
  reference_line.SLToXY(lane_change_start_sl, start_xy);
}

/**
 * @brief 获取换道禁行区边界
 *
 * @param path_bound 输入/输出：路径边界
 *
 * 禁行区域：
 * - 交叉路口
 * - 汇入区
 * - 其他不允许换道的区域
 *
 * 处理逻辑：
 * 1. 如果可以安全换道，清除起始位置标记
 * 2. 否则，确定换道起始位置
 * 3. 移除起始位置之前目标车道的边界
 *
 * C++语法说明：
 * - CHECK_NOTNULL(path_bound): 断言检查非空
 */
void LaneChangePath::GetBoundaryFromLaneChangeForbiddenZone(
    PathBoundary* const path_bound) {
  /**
   * @brief 断言检查
   */
  CHECK_NOTNULL(path_bound);

  /**
   * @brief 如果可以安全换道，清除起始位置
   */
  if (is_clear_to_change_lane_) {
    is_exist_lane_change_start_position_ = false;
    return;
  }

  double lane_change_start_s = 0.0;
  const ReferenceLine& reference_line = reference_line_info_->reference_line();

  /**
   * @brief 如果已存在预设的换道起始位置，使用它
   */
  if (is_exist_lane_change_start_position_) {
    common::SLPoint point_sl;
    reference_line.XYToSL(lane_change_start_xy_, &point_sl);
    lane_change_start_s = point_sl.s();
  } else {
    /**
     * @brief 计算换道起始位置
     * TODO(jiacheng): 使用ML模型学习最佳位置
     */
    lane_change_start_s =
        config_.lane_change_prepare_length() + init_sl_state_.first[0];

    /**
     * @brief 更新预设的换道起始XY位置
     */
    GetLaneChangeStartPoint(reference_line, init_sl_state_.first[0],
                            &lane_change_start_xy_);
  }

  /**
   * @brief 如果已通过换道起始位置，直接返回
   */
  if (lane_change_start_s < init_sl_state_.first[0]) {
    return;
  }

  /**
   * @brief 获取自车半宽
   */
  double adc_half_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;

  /**
   * @brief 遍历边界点，移除禁行区边界
   */
  for (size_t i = 0; i < path_bound->size(); ++i) {
    double curr_s = (*path_bound)[i].s;

    /**
     * @brief 如果到达换道起始位置，停止处理
     */
    if (curr_s > lane_change_start_s) {
      break;
    }

    /**
     * @brief 获取当前点的车道宽度
     */
    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;
    double offset_to_map = 0.0;
    reference_line.GetOffsetToMap(curr_s, &offset_to_map);

    if (reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                    &curr_lane_right_width)) {
      double offset_to_lane_center = 0.0;
      reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
      curr_lane_left_width += offset_to_lane_center;
      curr_lane_right_width -= offset_to_lane_center;
    }

    /**
     * @brief 应用偏移
     */
    curr_lane_left_width -= offset_to_map;
    curr_lane_right_width += offset_to_map;

    /**
     * @brief 更新右边界
     * 如果自车在车道左边，右边界收紧
     */
    (*path_bound)[i].l_lower.l = init_sl_state_.second[0] > curr_lane_left_width
                                     ? curr_lane_left_width + adc_half_width
                                     : (*path_bound)[i].l_lower.l;
    (*path_bound)[i].l_lower.l =
        std::fmin((*path_bound)[i].l_lower.l, init_sl_state_.second[0] - 0.1);

    /**
     * @brief 更新左边界
     * 如果自车在车道右边，左边界收紧
     */
    (*path_bound)[i].l_upper.l =
        init_sl_state_.second[0] < -curr_lane_right_width
            ? -curr_lane_right_width - adc_half_width
            : (*path_bound)[i].l_upper.l;
    (*path_bound)[i].l_upper.l =
        std::fmax((*path_bound)[i].l_upper.l, init_sl_state_.second[0] + 0.1);
  }
}

/**
 * @brief 更新换道状态
 *
 * @param timestamp 时间戳
 * @param status_code 状态码
 * @param path_id 路径ID
 *
 * C++语法说明：
 * - auto* lane_change_status = xxx:
   *   获取可修改的状态指针
 * - set_xxx(): protobuf消息的setter方法
 */
void LaneChangePath::UpdateStatus(double timestamp,
                                  ChangeLaneStatus::Status status_code,
                                  const std::string& path_id) {
  auto* lane_change_status = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_change_lane();
  AINFO << "lane change update from" << lane_change_status->DebugString()
        << "to";
  lane_change_status->set_timestamp(timestamp);
  lane_change_status->set_path_id(path_id);
  lane_change_status->set_status(status_code);
  AINFO << lane_change_status->DebugString();
}

/**
 * @brief 滞回滤波器
 *
 * @param obstacle_distance 障碍物距离
 * @param safe_distance 安全距离
 * @param distance_buffer 距离缓冲区
 * @param is_obstacle_blocking 障碍物是否正在阻塞
 * @return bool 是否安全
 *
 * 滞回逻辑：
 * - 如果正在阻塞：obstacle_distance < safe_distance + buffer
 * - 如果不在阻塞：obstacle_distance < safe_distance - buffer
 *
 * 这样可以避免频繁的状态切换
 *
 * C++语法说明：
 * - const bool is_obstacle_blocking: 常量引用参数
 */
bool LaneChangePath::HysteresisFilter(const double obstacle_distance,
                                      const double safe_distance,
                                      const double distance_buffer,
                                      const bool is_obstacle_blocking) {
  if (is_obstacle_blocking) {
    /**
     * @brief 阻塞状态使用较宽松的阈值
     */
    return obstacle_distance < safe_distance + distance_buffer;
  } else {
    /**
     * @brief 非阻塞状态使用较严格的阈值
     */
    return obstacle_distance < safe_distance - distance_buffer;
  }
}

/**
 * @brief 设置路径信息
 *
 * @param path_data 输入/输出：路径数据
 *
 * 功能：
 * 1. 初始化路径点决策
 * 2. 遍历每个路径点，判断是否在车道内
 * 3. 设置路径点的类型（IN_LANE/OUT_ON_FORWARD_LANE等）
 *
 * C++语法说明：
 * - std::get<1>(tuple): 获取tuple的第二个元素
 * - std::move(path_decision): 移动语义
 */
void LaneChangePath::SetPathInfo(PathData* const path_data) {
  /**
   * @brief 创建路径点决策数组
   */
  std::vector<PathPointDecision> path_decision;

  /**
   * @brief 初始化路径点决策
   */
  PathAssessmentDeciderUtil::InitPathPointDecision(
      *path_data, PathData::PathPointType::IN_LANE, &path_decision);

  /**
   * @brief 获取离散路径引用
   */
  const auto& discrete_path = path_data->discretized_path();

  /**
   * @brief 自车SL边界
   */
  SLBoundary ego_sl_boundary;

  /**
   * @brief 遍历每个路径点
   */
  for (size_t i = 0; i < discrete_path.size(); ++i) {
    /**
     * @brief 获取当前点的SL边界
     */
    if (!GetSLBoundary(*path_data, i, reference_line_info_, &ego_sl_boundary)) {
      ADEBUG << "Unable to get SL-boundary of ego-vehicle.";
      continue;
    }

    /**
     * @brief 获取车道宽度
     */
    double lane_left_width = 0.0;
    double lane_right_width = 0.0;
    double middle_s =
        (ego_sl_boundary.start_s() + ego_sl_boundary.end_s()) / 2.0;

    if (reference_line_info_->reference_line().GetLaneWidth(
            middle_s, &lane_left_width, &lane_right_width)) {
      /**
       * @brief 车道内额外的缓冲区
       */
      double back_to_inlane_extra_buffer = 0.2;

      /**
       * @brief 判断路径点类型
       */
      if (ego_sl_boundary.start_l() > lane_left_width ||
          ego_sl_boundary.end_l() < -lane_right_width) {
        /**
         * @brief 自车还未开始换道
         */
        std::get<1>((path_decision)[i]) = PathData::PathPointType::IN_LANE;
      } else if (ego_sl_boundary.start_l() >
                     -lane_right_width + back_to_inlane_extra_buffer &&
                 ego_sl_boundary.end_l() <
                     lane_left_width - back_to_inlane_extra_buffer) {
        /**
         * @brief 自车已安全完成换道
         */
        std::get<1>((path_decision)[i]) = PathData::PathPointType::IN_LANE;
      } else {
        /**
         * @brief 自车正在跨越两个车道
         */
        std::get<1>((path_decision)[i]) =
            PathData::PathPointType::OUT_ON_FORWARD_LANE;
      }
    } else {
      AERROR << "reference line not ready when setting path point guide";
      return;
    }
  }

  /**
   * @brief 设置路径点决策指南
   */
  path_data->SetPathPointDecisionGuide(std::move(path_decision));
}

/**
 * @brief 检查上一帧是否成功
 *
 * @param last_frame 上一帧
 * @return bool 是否成功
 *
 * 检查逻辑：
 * 遍历上一帧的所有参考线信息
 * 如果是换道路径，检查轨迹类型
 * 如果是SPEED_FALLBACK（速度回退），说明失败
 *
 * C++语法说明：
 * - const apollo::planning::Frame* const:
   *   指向常量的常量指针
 *   第一个const: 指针指向的内容不可修改
 *   第二个const: 指针本身不可修改
 */
bool LaneChangePath::CheckLastFrameSucceed(
    const apollo::planning::Frame* const last_frame) {
  if (last_frame) {
    for (const auto& reference_line_info : last_frame->reference_line_info()) {
      /**
       * @brief 只检查换道路径
       */
      if (!reference_line_info.IsChangeLanePath()) {
        continue;
      }

      /**
       * @brief 获取轨迹类型
       */
      const auto history_trajectory_type =
          reference_line_info.trajectory_type();

      /**
       * @brief 如果是速度回退，说明规划失败
       */
      if (history_trajectory_type == ADCTrajectory::SPEED_FALLBACK) {
        return false;
      }
    }
  }
  return true;
}

/**
 * @brief 命名空间结束标记
 */
}  // namespace planning
}  // namespace apollo
