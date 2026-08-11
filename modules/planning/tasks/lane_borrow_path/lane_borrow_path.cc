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
 * @file lane_borrow_path.cc
 * @brief 借道绕行路径规划任务实现文件
 *
 * 本文件实现了LaneBorrowPath类，负责在车道被阻塞时借道绕行的路径规划。
 *
 * 与换道(LaneChange)的区别：
 * - 换道(LaneChange)：永久切换到目标车道
 * - 借道(LaneBorrow)：临时借用相邻车道，绕过障碍物后返回原车道
 *
 * 借道场景：
 * - 前方有静止障碍物阻塞
 * - 自车速度较低（< lane_borrow_max_speed）
 * - 障碍物不在路口内
 * - 障碍物是长期存在的
 * - 相邻车道可借道（不是实线）
 *
 * 核心概念：
 * - SidePassDirection::LEFT_BORROW: 向左借道
 * - SidePassDirection::RIGHT_BORROW: 向右借道
 * - forward lane: 前向车道（与本车同向）
 * - reverse lane: 逆向车道（与本车逆向）
 *
 * C++语法说明：
 * - std::shared_ptr<T>: 智能指针，引用计数
 * - std::tuple: 元组，存储固定数量的异构值
 * - std::function: 函数包装器
 */

#include "modules/planning/tasks/lane_borrow_path/lane_borrow_path.h"

/**
 * @brief 标准库头文件
 * <algorithm>: 提供std::min, std::max, std::fmax, std::fabs等算法
 * <functional>: 提供std::function函数包装器
 * <memory>: 提供std::shared_ptr智能指针
 * <string>: 提供std::string字符串
 * <tuple>: 提供std::tuple元组
 * <utility>: 提供std::pair, std::move等
 * <vector>: 提供std::vector动态数组
 */
#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
/**
 * @brief 车辆配置辅助类
 * VehicleConfigHelper::GetConfig(): 获取车辆配置参数
 */
#include "modules/planning/planning_base/common/obstacle_blocking_analyzer.h"
/**
 * @brief 障碍物阻塞分析器
 * 用于分析障碍物是否阻塞规划路径
 */
#include "modules/planning/planning_base/common/planning_context.h"
/**
 * @brief 规划上下文
 * 存储规划过程中的共享状态
 */
#include "modules/planning/planning_interface_base/task_base/common/path_generation.h"
/**
 * @brief 路径生成基类
 * Task: 所有规划任务的基类
 */
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_assessment_decider_util.h"
/**
 * @brief 路径评估工具
 * 评估路径是否有效
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
 * using: 类型别名，简化代码
 *
 * 作用：
 * - 减少代码冗余
 * - 提高可读性
 */
using apollo::common::Status;
using apollo::common::VehicleConfigHelper;
using apollo::common::math::Box2d;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;

/**
 * @brief 常量定义
 * constexpr: 编译期常量，比#define更类型安全
 *
 * kIntersectionClearanceDist: 交叉路口清除距离（米）
 *   - 信号灯/停车标志前需要保持的安全距离
 * kJunctionClearanceDist: 汇入区清除距离（米）
 *   - 路口前需要保持的安全距离
 */
constexpr double kIntersectionClearanceDist = 20.0;
constexpr double kJunctionClearanceDist = 15.0;

/**
 * @brief 初始化借道绕行路径任务
 *
 * @param config_dir 配置文件目录
 * @param name 任务名称
 * @param injector 依赖注入器
 * @return bool 初始化是否成功
 *
 * 初始化流程：
 * 1. 调用基类Task::Init进行基础初始化
 * 2. 加载LaneBorrowPathConfig配置
 */
bool LaneBorrowPath::Init(const std::string& config_dir,
                          const std::string& name,
                          const std::shared_ptr<DependencyInjector>& injector) {
  /**
   * @brief 调用基类初始化
   * Task::Init: 基类Task的初始化函数
   */
  if (!Task::Init(config_dir, name, injector)) {
    return false;
  }

  /**
   * @brief 加载任务特定配置
   * Task::LoadConfig<T>: 模板函数，从配置文件加载配置
   */
  return Task::LoadConfig<LaneBorrowPathConfig>(&config_);
}

/**
 * @brief 执行借道绕行路径规划
 *
 * @param frame 当前规划帧
 * @param reference_line_info 参考线信息
 * @return apollo::common::Status 执行状态
 *
 * 主要流程：
 * 1. 检查是否允许借道
 * 2. 检查是否需要借道
 * 3. 决定路径边界
 * 4. 优化路径
 * 5. 评估并选择最优路径
 *
 * 与LaneChangePath的区别：
 * - 即使评估失败也返回OK（允许不执行借道）
 * - 支持多条候选路径（左右借道）
 */
apollo::common::Status LaneBorrowPath::Process(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  /**
   * @brief 检查是否允许借道
   * config_.is_allow_lane_borrowing(): 配置中的借道开关
   * path_reusable(): 路径是否可复用
   */
  if (!config_.is_allow_lane_borrowing() ||
      reference_line_info->path_reusable()) {
    ADEBUG << "path reusable" << reference_line_info->path_reusable()
           << ",skip";
    return Status::OK();
  }

  /**
   * @brief 检查是否需要借道
   * IsNecessaryToBorrowLane():
   *   检查障碍物、速度、距离等条件
   */
  if (!IsNecessaryToBorrowLane()) {
    ADEBUG << "No need to borrow lane";
    return Status::OK();
  }

  /**
   * @brief 创建候选路径容器
   */
  std::vector<PathBoundary> candidate_path_boundaries;
  std::vector<PathData> candidate_path_data;

  /**
   * @brief 获取起始点SL状态
   */
  GetStartPointSLState();

  /**
   * @brief 决定路径边界
   */
  if (!DecidePathBounds(&candidate_path_boundaries)) {
    return Status::OK();  /**< 注意：这里返回OK，不是错误 */
  }

  /**
   * @brief 优化路径
   */
  if (!OptimizePath(candidate_path_boundaries, &candidate_path_data)) {
    return Status::OK();  /**< 注意：这里返回OK，不是错误 */
  }

  /**
   * @brief 评估路径并更新参考线信息
   * 即使失败也返回OK，因为借道是可选的
   */
  if (AssessPath(&candidate_path_data,
                 reference_line_info->mutable_path_data())) {
    ADEBUG << "lane borrow path success";
  }

  return Status::OK();
}

/**
 * @brief 决定路径边界
 *
 * @param boundary 输出：路径边界数组
 * @return bool 是否成功生成至少一条边界
 *
 * 路径边界决定流程：
 * 1. 对每个借道方向（左右）分别生成边界
 * 2. 初始化边界为无限大区域
 * 3. 根据相邻车道信息收窄边界
 * 4. 根据静态障碍物调整边界
 *
 * C++语法说明：
 * - for循环遍历decided_side_pass_direction_：
 *   支持同时生成左右两侧的借道路径
 * - boundary->emplace_back(): 在vector末尾构造空对象
 * - boundary->pop_back(): 移除最后一个元素（当某方向失败时）
 */
bool LaneBorrowPath::DecidePathBounds(std::vector<PathBoundary>* boundary) {
  /**
   * @brief 遍历每个借道方向
   * decided_side_pass_direction_: 存储决定的借道方向列表
   * 可能包含LEFT_BORROW和/或RIGHT_BORROW
   */
  for (size_t i = 0; i < decided_side_pass_direction_.size(); i++) {
    /**
     * @brief 添加空路径边界
     */
    boundary->emplace_back();
    auto& path_bound = boundary->back();

    /**
     * @brief 阻塞障碍物ID和借道类型
     */
    std::string blocking_obstacle_id = "";
    std::string borrow_lane_type = "";
    double path_narrowest_width = 0;

    /**
     * @brief 第1步：初始化路径边界为无限大区域
     */
    if (!PathBoundsDeciderUtil::InitPathBoundary(*reference_line_info_,
                                                 &path_bound, init_sl_state_)) {
      const std::string msg = "Failed to initialize path boundaries.";
      AERROR << msg;
      boundary->pop_back();  /**< 移除失败的边界 */
      continue;  /**< 继续下一个方向 */
    }

    /**
     * @brief 第2步：根据相邻车道信息决定粗糙边界
     * GetBoundaryFromNeighborLane():
     *   根据借道方向（左右）和相邻车道宽度设置边界
     */
    if (!GetBoundaryFromNeighborLane(decided_side_pass_direction_[i],
                                     &path_bound, &borrow_lane_type)) {
      AERROR << "Failed to decide a rough boundary based on lane and adc.";
      boundary->pop_back();
      continue;
    }

    /**
     * @brief 设置路径标签
     * 用于标识路径类型："regular/leftforward", "regular/rightreverse"等
     */
    std::string label;
    if (decided_side_pass_direction_[i] == SidePassDirection::LEFT_BORROW) {
      label = "regular/left" + borrow_lane_type;
    } else {
      label = "regular/right" + borrow_lane_type;
    }
    path_bound.set_label(label);

    /**
     * @brief 第3步：根据静态障碍物调整边界
     */
    PathBound temp_path_bound = path_bound;
    obs_sl_polygons_.clear();
    PathBoundsDeciderUtil::GetSLPolygons(*reference_line_info_,
                                         &obs_sl_polygons_, init_sl_state_);

    /**
     * @brief GetBoundaryFromStaticObstacles():
     *   将障碍物占用区域从边界中移除
     */
    if (!PathBoundsDeciderUtil::GetBoundaryFromStaticObstacles(
            *reference_line_info_, &obs_sl_polygons_, init_sl_state_,
            &path_bound, &blocking_obstacle_id, &path_narrowest_width)) {
      const std::string msg =
          "Failed to decide fine tune the boundaries after "
          "taking into consideration all static obstacles.";
      AERROR << msg;
      boundary->pop_back();
      continue;
    }

    /**
     * @brief 第4步：追加尾部边界点，避免零长度路径
     */
    int counter = 0;
    while (!blocking_obstacle_id.empty() &&
           path_bound.size() < temp_path_bound.size() &&
           counter < FLAGS_num_extra_tail_bound_point) {    // 20
      path_bound.push_back(temp_path_bound[path_bound.size()]);
      counter++;
    }

    ADEBUG << "Completed generating path boundaries.";

    /**
     * @brief 设置阻塞障碍物ID并记录调试信息
     */
    path_bound.set_blocking_obstacle_id(blocking_obstacle_id);
    RecordDebugInfo(path_bound, path_bound.label(), reference_line_info_);
  }

  /**
   * @brief 返回是否有至少一条有效边界
   */
  return !boundary->empty();
}

/**
 * @brief 优化路径
 *
 * @param path_boundaries 输入：路径边界数组
 * @param candidate_path_data 输出：候选路径数据数组
 * @return bool 是否成功
 *
 * 与LaneChangePath::OptimizePath的区别：
 * - 使用UpdatePathRefWithBound更新参考线
 * - 参考线会根据边界约束动态调整
 */
bool LaneBorrowPath::OptimizePath(
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
   * {0.0, 0.0, 0.0}: 终点横向位置、速度、加速度都为0
   */
  std::array<double, 3> end_state = {0.0, 0.0, 0.0};

  /**
   * @brief 遍历每个路径边界
   */
  for (const auto& path_boundary : path_boundaries) {
    /**
     * @brief 创建优化结果容器
     */
    std::vector<double> opt_l, opt_dl, opt_ddl;
    std::vector<std::pair<double, double>> ddl_bounds;

    /**
     * @brief 计算加速度边界
     */
    PathOptimizerUtil::CalculateAccBound(path_boundary, reference_line,
                                         &ddl_bounds);

    /**
     * @brief 估计jerk边界
     * std::fmax(init_sl_state_.first[1], 1e-12):
     *   确保速度不为零，避免除零
     */
    const double jerk_bound = PathOptimizerUtil::EstimateJerkBoundary(
        std::fmax(init_sl_state_.first[1], 1e-12));

    /**
     * @brief 更新参考线和权重
     * UpdatePathRefWithBound():
     *   根据边界约束更新参考线位置
     */
    std::vector<double> ref_l;
    std::vector<double> weight_ref_l;
    PathOptimizerUtil::UpdatePathRefWithBound(
        path_boundary, config.path_reference_l_weight(), &ref_l, &weight_ref_l);

    /**
     * @brief 执行路径优化
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
 * 与LaneChangePath::AssessPath的区别：
 * - 支持多候选路径比较
 * - 选择最优的借道方向
 */
bool LaneBorrowPath::AssessPath(std::vector<PathData>* candidate_path_data,
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
     */
    if (PathAssessmentDeciderUtil::IsValidRegularPath(*reference_line_info_,
                                                      curr_path_data)) {
      /**
       * @brief 设置路径信息
       */
      SetPathInfo(&curr_path_data);

      /**
       * @brief 如果接近目的地，裁剪尾部点
       */
      if (reference_line_info_->SDistanceToDestination() <
          FLAGS_path_trim_destination_threshold) {
        PathAssessmentDeciderUtil::TrimTailingOutLanePoints(&curr_path_data);
      }

      /**
       * @brief 检查裁剪后是否为空
       */
      if (curr_path_data.Empty()) {
        AINFO << "lane borrow path is empty after trimed";
        continue;
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
    AINFO << "All lane borrow path are not valid";
    return false;
  }

  /**
   * @brief 获取阻塞障碍物信息
   */
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  const std::string blocking_obstacle_id =
      mutable_path_decider_status->front_static_obstacle_id();
  const Obstacle* blocking_obstacle =
      reference_line_info_->path_decision()->obstacles().Find(
          blocking_obstacle_id);

  /**
   * @brief 如果有多条有效路径，进行比较选择
   */
  if (valid_path_data.size() > 1) {
    if (ComparePathData(valid_path_data[0], valid_path_data[1],
                        blocking_obstacle)) {
      *final_path = valid_path_data[0];
    } else {
      *final_path = valid_path_data[1];
    }
  } else {
    *final_path = valid_path_data[0];
  }

  /**
   * @brief 移动障碍物SL多边形到参考线信息
   */
  *(reference_line_info_->mutable_obs_sl_polygons()) = std::move(obs_sl_polygons_);

  /**
   * @brief 记录调试信息
   */
  RecordDebugInfo(*final_path, final_path->path_label(), reference_line_info_);
  return true;
}

/**
 * @brief 根据相邻车道信息获取边界
 *
 * @param pass_direction 借道方向（LEFT_BORROW/RIGHT_BORROW）
 * @param path_bound 输入/输出：路径边界
 * @param borrow_lane_type 输出：借道类型（forward/reverse）
 * @return bool 是否成功
 *
 * 处理流程：
 * 1. 获取当前车道宽度
 * 2. 获取相邻车道宽度
 * 3. 计算左右边界
 * 4. 更新边界
 *
 * C++语法说明：
 * - CHECK_NOTNULL(path_bound): 断言检查非空
 * - ACHECK: Apollo断言，仅在DEBUG模式生效
 */
bool LaneBorrowPath::GetBoundaryFromNeighborLane(
    const SidePassDirection pass_direction, PathBoundary* const path_bound,
    std::string* borrow_lane_type) {
  /**
   * @brief 断言检查
   */
  CHECK_NOTNULL(path_bound);
  ACHECK(!path_bound->empty());

  const ReferenceLine& reference_line = reference_line_info_->reference_line();

  /**
   * @brief 获取自车所在车道宽度
   * GetADCLaneWidth(): 根据参考线和s坐标获取车道宽度
   */
  double adc_lane_width = PathBoundsDeciderUtil::GetADCLaneWidth(
      reference_line, init_sl_state_.first[0]);

  double offset_to_map = 0;
  bool borrowing_reverse_lane = false;

  /**
   * @brief 获取参考线相对地图的偏移
   */
  reference_line.GetOffsetToMap(init_sl_state_.first[0], &offset_to_map);

  /**
   * @brief 初始化历史宽度
   * 用于当获取车道宽度失败时使用历史值
   */
  double past_lane_left_width = adc_lane_width / 2.0;
  double past_lane_right_width = adc_lane_width / 2.0;
  int path_blocked_idx = -1;  /**< 路径被阻塞的索引 */

  /**
   * @brief 遍历所有边界点
   */
  for (size_t i = 0; i < path_bound->size(); ++i) {
    double curr_s = (*path_bound)[i].s;

    /**
     * @brief 第1步：获取当前点的车道宽度
     */
    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;
    double offset_to_lane_center = 0.0;
    // 计算的宽度是相对于地图中心线
    if (!reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                     &curr_lane_right_width)) {
      AWARN << "Failed to get lane width at s = " << curr_s;
      /**
       * @brief 使用历史宽度作为默认值
       */
      curr_lane_left_width = past_lane_left_width;
      curr_lane_right_width = past_lane_right_width;
    } else {
      /**
       * @brief 应用道路中心偏移
       */
      reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
      curr_lane_left_width += offset_to_lane_center;
      curr_lane_right_width -= offset_to_lane_center;

      /**
       * @brief 更新历史宽度
       */
      past_lane_left_width = curr_lane_left_width;
      past_lane_right_width = curr_lane_right_width;
    }

    /**
     * @brief 第2步：获取相邻车道宽度
     * CheckLaneBoundaryType(): 检查车道边界类型（是否是实线）
     */
    double curr_neighbor_lane_width = 0.0;
    if (CheckLaneBoundaryType(*reference_line_info_, curr_s, pass_direction)) {
      hdmap::Id neighbor_lane_id;

      if (pass_direction == SidePassDirection::LEFT_BORROW) {
        /**
         * @brief 借用左侧车道
         * LeftForward: 左前向车道（与本车同向）
         * LeftReverse: 左逆向车道（与本车逆向）
         */
        if (reference_line_info_->GetNeighborLaneInfo(
                ReferenceLineInfo::LaneType::LeftForward, curr_s,
                &neighbor_lane_id, &curr_neighbor_lane_width)) {
          ADEBUG << "Borrow left forward neighbor lane."
                 << neighbor_lane_id.id();
        } else if (reference_line_info_->GetNeighborLaneInfo(
                       ReferenceLineInfo::LaneType::LeftReverse, curr_s,
                       &neighbor_lane_id, &curr_neighbor_lane_width)) {
          borrowing_reverse_lane = true;  /**< 标记为逆向车道 */
          ADEBUG << "Borrow left reverse neighbor lane."
                 << neighbor_lane_id.id();
        } else {
          ADEBUG << "There is no left neighbor lane.";
        }
      } else if (pass_direction == SidePassDirection::RIGHT_BORROW) {
        /**
         * @brief 借用右侧车道
         */
        if (reference_line_info_->GetNeighborLaneInfo(
                ReferenceLineInfo::LaneType::RightForward, curr_s,
                &neighbor_lane_id, &curr_neighbor_lane_width)) {
          ADEBUG << "Borrow right forward neighbor lane."
                 << neighbor_lane_id.id();
        } else if (reference_line_info_->GetNeighborLaneInfo(
                       ReferenceLineInfo::LaneType::RightReverse, curr_s,
                       &neighbor_lane_id, &curr_neighbor_lane_width)) {
          borrowing_reverse_lane = true;
          ADEBUG << "Borrow right reverse neighbor lane."
                 << neighbor_lane_id.id();
        } else {
          ADEBUG << "There is no right neighbor lane.";
        }
      }
    }

    /**
     * @brief 第3步：计算边界
     */
    double offset_to_map = 0.0;
    reference_line.GetOffsetToMap(curr_s, &offset_to_map);

    /**
     * @brief 左边界
     * 如果是向左借道，加上相邻车道宽度
     */
    double curr_left_bound_lane =
        curr_lane_left_width + (pass_direction == SidePassDirection::LEFT_BORROW
                                    ? curr_neighbor_lane_width
                                    : 0.0);

    /**
     * @brief 右边界
     * 如果是向右借道，加上相邻车道宽度
     */
    double curr_right_bound_lane =
        -curr_lane_right_width -
        (pass_direction == SidePassDirection::RIGHT_BORROW
             ? curr_neighbor_lane_width
             : 0.0);

    double curr_left_bound = 0.0;
    double curr_right_bound = 0.0;

    /**
     * @brief 应用地图偏移
     */
    curr_left_bound = curr_left_bound_lane - offset_to_map;
    curr_right_bound = curr_right_bound_lane - offset_to_map;
    // 为啥都是减去地图偏移？因为边界是相对于地图中心线的，而地图偏移是从车道中心线到地图中心线的距离，所以需要减去这个偏移来得到相对于车道中心线的边界位置。

    /**
     * @brief 第4步：更新边界
     * UpdatePathBoundaryWithBuffer():
     *   使用缓冲区更新边界
     */
    if (!PathBoundsDeciderUtil::UpdatePathBoundaryWithBuffer(
            curr_left_bound, curr_right_bound, BoundType::LANE, BoundType::LANE,
            "", "", &path_bound->at(i))) {
      path_blocked_idx = static_cast<int>(i);
    }

    /**
     * @brief 如果路径被阻塞，停止处理
     */
    if (path_blocked_idx != -1) {
      break;
    }
  }

  /**
   * @brief 裁剪边界
   */
  PathBoundsDeciderUtil::TrimPathBounds(path_blocked_idx, path_bound);

  /**
   * @brief 设置借道类型
   */
  *borrow_lane_type = borrowing_reverse_lane ? "reverse" : "forward";
  return true;
}

/**
 * @brief 更新自车道路径信息
 *
 * 更新逻辑：
 * - 如果当前路径是自车道且无阻塞，增加计数
 * - 否则重置计数
 */
void LaneBorrowPath::UpdateSelfPathInfo() {
  auto cur_path = reference_line_info_->path_data();

  /**
   * @brief 检查当前路径是否满足条件
   * - 非空
   * - 标签包含"self"
   * - 无阻塞障碍物
   */
  if (!cur_path.Empty() &&
      cur_path.path_label().find("self") != std::string::npos &&
      cur_path.blocking_obstacle_id().empty()) {
    /**
     * @brief 增加自车道使用计数
     * std::min(use_self_lane_ + 1, 10):
     *   最大值为10，防止溢出
     */
    use_self_lane_ = std::min(use_self_lane_ + 1, 10);
  } else {
    use_self_lane_ = 0;
  }

  /**
   * @brief 更新阻塞障碍物ID
   */
  blocking_obstacle_id_ = cur_path.blocking_obstacle_id();
}

/**
 * @brief 检查是否需要借道
 *
 * @return bool 是否需要借道
 *
 * 检查条件：
 * 1. ADC要求：
 *    - 只有单条参考线
 *    - 速度 < lane_borrow_max_speed
 * 2. 障碍物条件：
 *    - 障碍物远离交叉路口/汇入区
 *    - 是长期阻塞障碍物
 *    - 在目的地范围内
 *    - 是可绕行的静止障碍物
 *
 * 状态机逻辑：
 * - 如果当前是借道状态：检查是否可以切回自车道
 * - 如果当前是自车道状态：检查是否需要借道
 */
bool LaneBorrowPath::IsNecessaryToBorrowLane() {
  /**
   * @brief 获取路径决策状态
   */
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();

  /**
   * @brief 如果当前是借道状态
   */
  if (mutable_path_decider_status->is_in_path_lane_borrow_scenario()) {
    UpdateSelfPathInfo();

    /**
     * @brief 如果已经能使用自车道一段时间
     * 切换回非借道状态
     */
    if (use_self_lane_ >= 6) {
      mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(false);
      decided_side_pass_direction_.clear();
      AINFO << "Switch from LANE-BORROW path to SELF-LANE path.";
    }
  } else {
    /**
     * @brief 如果当前是自车道状态
     * 检查是否需要借道
     */
    AINFO << "Blocking obstacle ID["
          << mutable_path_decider_status->front_static_obstacle_id() << "]";

    /**
     * @brief ADC条件检查
     */
    if (!HasSingleReferenceLine(*frame_)) {
      return false;
    }
    if (!IsWithinSidePassingSpeedADC(*frame_)) {
      return false;
    }

    /**
     * @brief 障碍物条件检查
     */
    if (!IsBlockingObstacleFarFromIntersection(*reference_line_info_)) {
      return false;
    }
    if (!IsLongTermBlockingObstacle()) {
      return false;
    }
    if (!IsBlockingObstacleWithinDestination(*reference_line_info_)) {
      return false;
    }
    if (!IsSidePassableObstacle(*reference_line_info_)) {
      return false;
    }

    /**
     * @brief 切换到借道状态
     */
    if (decided_side_pass_direction_.empty()) {
      /**
       * @brief 第一次初始化借道方向
       */
      bool left_borrowable;
      bool right_borrowable;

      /**
       * @brief 检查左右车道是否可借
       */
      CheckLaneBorrow(*reference_line_info_, &left_borrowable,
                      &right_borrowable);

      if (!left_borrowable && !right_borrowable) {
        mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(false);
        AINFO << "LEFT AND RIGHT LANE CAN NOT BORROW";
        return false;
      } else {
        mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(true);

        /**
         * @brief 添加可行的借道方向
         */
        if (left_borrowable) {
          decided_side_pass_direction_.push_back(
              SidePassDirection::LEFT_BORROW);
        }
        if (right_borrowable) {
          decided_side_pass_direction_.push_back(
              SidePassDirection::RIGHT_BORROW);
        }
      }
    }
    use_self_lane_ = 0;
    AINFO << "Switch from SELF-LANE path to LANE-BORROW path.";
  }
  return mutable_path_decider_status->is_in_path_lane_borrow_scenario();
}

/**
 * @brief 检查是否只有单条参考线
 *
 * 借道只能在单参考线条件下进行
 * 多参考线时由其他模块处理
 */
bool LaneBorrowPath::HasSingleReferenceLine(const Frame& frame) {
  return frame.reference_line_info().size() == 1;
}

/**
 * @brief 检查自车速度是否在借道允许范围内
 */
bool LaneBorrowPath::IsWithinSidePassingSpeedADC(const Frame& frame) {
  return frame.PlanningStartPoint().v() < config_.lane_borrow_max_speed();
}

/**
 * @brief 检查是否是长期阻塞障碍物
 *
 * 只有长期存在的障碍物才值得借道绕行
 * 避免因为短暂障碍物而频繁切换
 */
bool LaneBorrowPath::IsLongTermBlockingObstacle() {
  if (injector_->planning_context()
          ->planning_status()
          .path_decider()
          .front_static_obstacle_cycle_counter() >=
      config_.long_term_blocking_obstacle_cycle_threshold()) {
    ADEBUG << "The blocking obstacle is long-term existing.";
    return true;
  } else {
    ADEBUG << "The blocking obstacle is not long-term existing.";
    return false;
  }
}

/**
 * @brief 检查阻塞障碍物是否在目的地范围内
 *
 * 如果障碍物在目的地之外，不需要借道
 */
bool LaneBorrowPath::IsBlockingObstacleWithinDestination(
    const ReferenceLineInfo& reference_line_info) {
  const auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  const std::string blocking_obstacle_id =
      path_decider_status.front_static_obstacle_id();

  if (blocking_obstacle_id.empty()) {
    ADEBUG << "There is no blocking obstacle.";
    return true;
  }

  const Obstacle* blocking_obstacle =
      reference_line_info.path_decision()->obstacles().Find(
          blocking_obstacle_id);

  if (blocking_obstacle == nullptr) {
    ADEBUG << "Blocking obstacle is no longer there.";
    return true;
  }

  /**
   * @brief 获取障碍物和自车的s坐标
   */
  double blocking_obstacle_s =
      blocking_obstacle->PerceptionSLBoundary().start_s();
  double adc_end_s = reference_line_info.AdcSlBoundary().end_s();

  ADEBUG << "Blocking obstacle is at s = " << blocking_obstacle_s;
  ADEBUG << "ADC is at s = " << adc_end_s;
  ADEBUG << "Destination is at s = "
         << reference_line_info.SDistanceToDestination() + adc_end_s;

  /**
   * @brief 如果障碍物在目的地之外，返回false
   */
  if (blocking_obstacle_s - adc_end_s >
      reference_line_info.SDistanceToDestination()) {
    return false;
  }
  return true;
}

/**
 * @brief 检查阻塞障碍物是否远离交叉路口
 *
 * 在交叉路口附近不允许借道
 */
bool LaneBorrowPath::IsBlockingObstacleFarFromIntersection(
    const ReferenceLineInfo& reference_line_info) {
  const auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  const std::string blocking_obstacle_id =
      path_decider_status.front_static_obstacle_id();

  if (blocking_obstacle_id.empty()) {
    ADEBUG << "There is no blocking obstacle.";
    return true;
  }

  const Obstacle* blocking_obstacle =
      reference_line_info.path_decision()->obstacles().Find(
          blocking_obstacle_id);

  if (blocking_obstacle == nullptr) {
    ADEBUG << "Blocking obstacle is no longer there.";
    return true;
  }

  /**
   * @brief 获取障碍物的s坐标
   */
  double blocking_obstacle_s =
      blocking_obstacle->PerceptionSLBoundary().end_s();

  ADEBUG << "Blocking obstacle is at s = " << blocking_obstacle_s;

  /**
   * @brief 获取第一个遇到的交叉路口
   */
  const auto& first_encountered_overlaps =
      reference_line_info.FirstEncounteredOverlaps();

  /**
   * @brief 遍历所有遇到的交叉路口
   */
  for (const auto& overlap : first_encountered_overlaps) {
    ADEBUG << overlap.first << ", " << overlap.second.DebugString();

    /**
     * @brief 只检查信号灯和停车标志
     */
    if (overlap.first != ReferenceLineInfo::SIGNAL &&
        overlap.first != ReferenceLineInfo::STOP_SIGN) {
      continue;
    }

    auto distance = overlap.second.start_s - blocking_obstacle_s;

    /**
     * @brief 信号灯/停车标志：距离 < kIntersectionClearanceDist
     * 汇入区：距离 < kJunctionClearanceDist
     */
    if (overlap.first == ReferenceLineInfo::SIGNAL ||
        overlap.first == ReferenceLineInfo::STOP_SIGN) {
      if (distance < kIntersectionClearanceDist) {
        ADEBUG << "Too close to signal intersection (" << distance
               << "m); don't SIDE_PASS.";
        return false;
      }
    } else {
      if (distance < kJunctionClearanceDist) {
        ADEBUG << "Too close to overlap_type[" << overlap.first << "] ("
               << distance << "m); don't SIDE_PASS";
        return false;
      }
    }
  }

  return true;
}

/**
 * @brief 检查阻塞障碍物是否可绕行
 */
bool LaneBorrowPath::IsSidePassableObstacle(
    const ReferenceLineInfo& reference_line_info) {
  const auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  const std::string blocking_obstacle_id =
      path_decider_status.front_static_obstacle_id();

  if (blocking_obstacle_id.empty()) {
    ADEBUG << "There is no blocking obstacle.";
    return false;
  }

  const Obstacle* blocking_obstacle =
      reference_line_info.path_decision()->obstacles().Find(
          blocking_obstacle_id);

  if (blocking_obstacle == nullptr) {
    ADEBUG << "Blocking obstacle is no longer there.";
    return false;
  }

  /**
   * @brief 检查是否是可移动障碍物
   */
  return IsNonmovableObstacle(reference_line_info, *blocking_obstacle);
}

/**
 * @brief 检查左右车道是否可借
 *
 * @param reference_line_info 参考线信息
 * @param left_neighbor_lane_borrowable 输出：左侧是否可借
 * @param right_neighbor_lane_borrowable 输出：右侧是否可借
 *
 * 检查内容：
 * - 是否有相邻车道
 * - 车道边界是否是实线（SOLID_YELLOW, SOLID_WHITE, DOUBLE_YELLOW）
 */
void LaneBorrowPath::CheckLaneBorrow(
    const ReferenceLineInfo& reference_line_info,
    bool* left_neighbor_lane_borrowable, bool* right_neighbor_lane_borrowable) {
  const ReferenceLine& reference_line = reference_line_info.reference_line();

  /**
   * @brief 初始化为可借
   */
  *left_neighbor_lane_borrowable = true;
  *right_neighbor_lane_borrowable = true;

  /**
   * @brief 前视距离
   */
  static constexpr double kLookforwardDistance = 100.0;
  double check_s = reference_line_info.AdcSlBoundary().end_s();

  /**
   * @brief 计算前视距离上限
   */
  const double lookforward_distance =
      std::min(check_s + kLookforwardDistance, reference_line.Length());

  /**
   * @brief 沿路径向前检查
   */
  while (check_s < lookforward_distance) {
    auto ref_point = reference_line.GetNearestReferencePoint(check_s);

    /**
     * @brief 如果没有车道信息，两侧都不可借
     */
    if (ref_point.lane_waypoints().empty()) {
      *left_neighbor_lane_borrowable = false;
      *right_neighbor_lane_borrowable = false;
      return;
    }

    /**
     * @brief 获取车道信息
     */
    auto ptr_lane_info = reference_line_info.LocateLaneInfo(check_s);

    /**
     * @brief 检查是否有相邻车道
     */
    if (ptr_lane_info->lane().left_neighbor_forward_lane_id().empty() &&
        ptr_lane_info->lane().left_neighbor_reverse_lane_id().empty()) {
      *left_neighbor_lane_borrowable = false;
    }
    if (ptr_lane_info->lane().right_neighbor_forward_lane_id().empty() &&
        ptr_lane_info->lane().right_neighbor_reverse_lane_id().empty()) {
      *right_neighbor_lane_borrowable = false;
    }

    const auto waypoint = ref_point.lane_waypoints().front();

    /**
     * @brief 车道边界类型
     */
    hdmap::LaneBoundaryType::Type lane_boundary_type =
        hdmap::LaneBoundaryType::UNKNOWN;

    /**
     * @brief 检查左侧车道边界
     */
    if (*left_neighbor_lane_borrowable) {
      lane_boundary_type = hdmap::LeftBoundaryType(waypoint);

      /**
       * @brief 实线不可借
       */
      if (lane_boundary_type == hdmap::LaneBoundaryType::SOLID_YELLOW ||
          lane_boundary_type == hdmap::LaneBoundaryType::DOUBLE_YELLOW ||
          lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE) {
        *left_neighbor_lane_borrowable = false;
      }
      ADEBUG << "s[" << check_s << "] left_lane_boundary_type["
             << LaneBoundaryType_Type_Name(lane_boundary_type) << "]";
    }

    /**
     * @brief 检查右侧车道边界
     */
    if (*right_neighbor_lane_borrowable) {
      lane_boundary_type = hdmap::RightBoundaryType(waypoint);

      if (lane_boundary_type == hdmap::LaneBoundaryType::SOLID_YELLOW ||
          lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE) {
        *right_neighbor_lane_borrowable = false;
      }
      ADEBUG << "s[" << check_s << "] right_neighbor_lane_borrowable["
             << LaneBoundaryType_Type_Name(lane_boundary_type) << "]";
    }

    /**
     * @brief 前进一步
     */
    check_s += 2.0;
  }
}

/**
 * @brief 检查车道边界类型
 *
 * @param reference_line_info 参考线信息
 * @param check_s 检查点的s坐标
 * @param lane_borrow_info 借道方向
 * @return bool 是否可以通过（不是实线）
 *
 * 用于判断特定点的车道边界是否是实线
 */
bool LaneBorrowPath::CheckLaneBoundaryType(
    const ReferenceLineInfo& reference_line_info, const double check_s,
    const SidePassDirection& lane_borrow_info) {
  const ReferenceLine& reference_line = reference_line_info.reference_line();

  auto ref_point = reference_line.GetNearestReferencePoint(check_s);

  if (ref_point.lane_waypoints().empty()) {
    return false;
  }

  const auto waypoint = ref_point.lane_waypoints().front();

  /**
   * @brief 车道边界类型
   */
  hdmap::LaneBoundaryType::Type lane_boundary_type =
      hdmap::LaneBoundaryType::UNKNOWN;

  /**
   * @brief 根据借道方向获取边界类型
   */
  if (lane_borrow_info == SidePassDirection::LEFT_BORROW) {
    lane_boundary_type = hdmap::LeftBoundaryType(waypoint);
  } else if (lane_borrow_info == SidePassDirection::RIGHT_BORROW) {
    lane_boundary_type = hdmap::RightBoundaryType(waypoint);
  }

  /**
   * @brief 实线返回false（不可借）
   */
  if (lane_boundary_type == hdmap::LaneBoundaryType::SOLID_YELLOW ||
      lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE) {
    return false;
  }
  return true;
}

/**
 * @brief 设置路径信息
 *
 * @param path_data 输入/输出：路径数据
 *
 * 功能：
 * - 初始化路径点决策
 * - 判断每个点是在本车道、借道前向车道还是借道逆向车道
 */
void LaneBorrowPath::SetPathInfo(PathData* const path_data) {
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

  bool is_prev_point_out_lane = false;  /**< 上一个点是否在车道外 */
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
       * @brief 缓冲区
       */
      double back_to_inlane_extra_buffer = 0.2;

      /**
       * @brief 滞回缓冲区
       * 如果上一个点在车道外，使用更大的缓冲区
       */
      double in_and_out_lane_hysteresis_buffer =
          is_prev_point_out_lane ? back_to_inlane_extra_buffer : 0.0;

      /**
       * @brief 判断是否在借道状态
       */
      if (ego_sl_boundary.end_l() >
              lane_left_width + in_and_out_lane_hysteresis_buffer ||
          ego_sl_boundary.start_l() <
              -lane_right_width - in_and_out_lane_hysteresis_buffer) {
        /**
         * @brief 根据路径标签判断是前向还是逆向借道
         */
        if (path_data->path_label().find("reverse") != std::string::npos) {
          std::get<1>((path_decision)[i]) =
              PathData::PathPointType::OUT_ON_REVERSE_LANE;
        } else if (path_data->path_label().find("forward") !=
                   std::string::npos) {
          std::get<1>((path_decision)[i]) =
              PathData::PathPointType::OUT_ON_FORWARD_LANE;
        } else {
          std::get<1>((path_decision)[i]) = PathData::PathPointType::UNKNOWN;
        }

        /**
         * @brief 更新状态
         */
        if (!is_prev_point_out_lane) {
          if (ego_sl_boundary.end_l() >
                  lane_left_width + back_to_inlane_extra_buffer ||
              ego_sl_boundary.start_l() <
                  -lane_right_width - back_to_inlane_extra_buffer) {
            is_prev_point_out_lane = true;
          }
        }
      } else {
        /**
         * @brief 在本车道内
         */
        std::get<1>((path_decision)[i]) = PathData::PathPointType::IN_LANE;

        if (is_prev_point_out_lane) {
          is_prev_point_out_lane = false;
        }
      }
    } else {
      AERROR
          << "reference line not ready when setting path point guide, middle_s"
          << middle_s << ",index" << i << "path point"
          << discrete_path[i].DebugString();
      break;
    }
  }

  /**
   * @brief 设置路径点决策指南
   */
  path_data->SetPathPointDecisionGuide(std::move(path_decision));
}

/**
 * @brief 比较两条路径数据，选择最优
 *
 * @param lhs 左路径
 * @param rhs 右路径
 * @param blocking_obstacle 阻塞障碍物
 * @return bool lhs是否优于rhs
 *
 * 选择策略：
 * 1. 选择更长的路径
 * 2. 如果长度相近，选择逆向车道更少的
 * 3. 根据障碍物位置选择更方便的绕行方向
 * 4. 如果自车偏离车道较远，选择能让自车更快回到车道中心的
 * 5. 如果以上都相近，选择左侧绕行
 */
bool ComparePathData(const PathData& lhs, const PathData& rhs,
                     const Obstacle* blocking_obstacle) {
  ADEBUG << "Comparing " << lhs.path_label() << " and " << rhs.path_label();

  /**
   * @brief 路径长度比较容差
   */
  static constexpr double kNeighborPathLengthComparisonTolerance = 25.0;

  /**
   * @brief 获取路径长度
   */
  double lhs_path_length = lhs.frenet_frame_path().back().s();
  double rhs_path_length = rhs.frenet_frame_path().back().s();

  /**
   * @brief 第1步：选择更长的路径
   */
  if (std::fabs(lhs_path_length - rhs_path_length) >
      kNeighborPathLengthComparisonTolerance) {
    return lhs_path_length > rhs_path_length;
  }

  /**
   * @brief 第2步：如果长度相近，选择逆向车道更少的
   */
  int lhs_on_reverse =
      ContainsOutOnReverseLane(lhs.path_point_decision_guide());
  int rhs_on_reverse =
      ContainsOutOnReverseLane(rhs.path_point_decision_guide());

  /**
   * @brief 如果逆向车道数差异大于6，选择逆向少的
   */
  if (std::abs(lhs_on_reverse - rhs_on_reverse) > 6) {
    return lhs_on_reverse < rhs_on_reverse;
  }

  /**
   * @brief 第3步：根据障碍物位置选择
   */
  if (blocking_obstacle) {
    /**
     * @brief 获取障碍物中心l坐标
     */
    const double obstacle_l =
        (blocking_obstacle->PerceptionSLBoundary().start_l() +
         blocking_obstacle->PerceptionSLBoundary().end_l()) /
        2;

    ADEBUG << "obstacle[" << blocking_obstacle->Id() << "] l[" << obstacle_l
           << "]";

    /**
     * @brief 障碍物在右侧，选择向左绕行
     * 障碍物在左侧，选择向右绕行
     */
    return (obstacle_l > 0.0
                ? (lhs.path_label().find("right") != std::string::npos)
                : (lhs.path_label().find("left") != std::string::npos));
  } else {
    /**
     * @brief 第4步：根据自车位置选择
     * 自车在左侧，选择向右绕行
     * 自车在右侧，选择向左绕行
     */
    double adc_l = lhs.frenet_frame_path().front().l();
    if (adc_l < -1.0) {
      return lhs.path_label().find("right") != std::string::npos;
    } else if (adc_l > 1.0) {
      return lhs.path_label().find("left") != std::string::npos;
    }
  }

  /**
   * @brief 第5步：选择更早回到车道的
   */
  static constexpr double kBackToSelfLaneComparisonTolerance = 20.0;

  int lhs_back_idx = GetBackToInLaneIndex(lhs.path_point_decision_guide());
  int rhs_back_idx = GetBackToInLaneIndex(rhs.path_point_decision_guide());

  double lhs_back_s = lhs.frenet_frame_path()[lhs_back_idx].s();
  double rhs_back_s = rhs.frenet_frame_path()[rhs_back_idx].s();

  if (std::fabs(lhs_back_s - rhs_back_s) > kBackToSelfLaneComparisonTolerance) {
    return lhs_back_idx < rhs_back_idx;
  }

  /**
   * @brief 第6步：最后选择左侧绕行
   */
  bool lhs_on_leftlane = lhs.path_label().find("left") != std::string::npos;
  return lhs_on_leftlane;
}

/**
 * @brief 计算路径中在逆向车道上的点数
 *
 * @param path_point_decision 路径点决策数组
 * @return int 逆向车道点数
 */
int ContainsOutOnReverseLane(
    const std::vector<PathPointDecision>& path_point_decision) {
  int ret = 0;

  for (const auto& curr_decision : path_point_decision) {
    if (std::get<1>(curr_decision) ==
        PathData::PathPointType::OUT_ON_REVERSE_LANE) {
      ++ret;
    }
  }
  return ret;
}

/**
 * @brief 获取回到车道的索引
 *
 * @param path_point_decision 路径点决策数组
 * @return int 最后一个不在车道内的点的索引
 */
int GetBackToInLaneIndex(
    const std::vector<PathPointDecision>& path_point_decision) {
  /**
   * @brief 从后向前查找第一个不在车道内的点
   */
  for (int i = static_cast<int>(path_point_decision.size()) - 1; i >= 0; --i) {
    if (std::get<1>(path_point_decision[i]) !=
        PathData::PathPointType::IN_LANE) {
      return i;
    }
  }
  return 0;
}

/**
 * @brief 命名空间结束标记
 */
}  // namespace planning
}  // namespace apollo
