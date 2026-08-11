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
 * @file open_space_trajectory_post_process.cc
 *
 * @brief 开放空间轨迹后处理实现文件
 *
 * 功能说明：
 * 本文件实现了 OpenSpaceTrajectoryPostProcess 类
 * 负责开放空间轨迹的后处理工作
 * 主要功能包括：
 * 1. 轨迹插值（Interpolation）
 * 2. 轨迹分段（Partition）
 * 3. 档位切换处理（Gear Shift）
 * 4. 最近点追踪（Closest Point Tracking）
 * 5. 轨迹拼接与时间调整
 *
 * 核心概念：
 * - Trajectory Partitioning：根据档位变化将连续轨迹分段
 * - Gear Shift Trajectory：档位切换时的过渡轨迹
 * - IOU（Intersection over Union）：用于匹配车辆和轨迹点的重叠度
 *
 * C++语法说明：
 * - namespace：命名空间，避免命名冲突
 * - std::priority_queue：优先队列，用于选择最优轨迹点
 * - std::shared_ptr：智能指针，引用计数管理
 * - protobuf消息操作：mutable_xxx(), set_xxx(), xxx_size()
 **/

#include "modules/planning/tasks/open_space_trajectory_post_process/open_space_trajectory_post_process.h"

/**
 * @brief 标准库头文件
 *
 * C++语法说明：
 * - <algorithm>：标准算法（std::min, std::abs等）
 * - <memory>：智能指针
 * - <queue>：队列容器
 */
#include <algorithm>
#include <memory>
#include <queue>

/**
 * @brief Abseil字符串工具
 *
 * 功能：字符串拼接等操作
 */
#include "absl/strings/str_cat.h"

#include "cyber/time/clock.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/status/status.h"
#include "modules/planning/planning_base/common/planning_context.h"

/**
 * @brief Apollo命名空间开始
 */
namespace apollo {
/**
 * @brief 规划模块命名空间
 */
namespace planning {

/**
 * @brief 类型别名声明
 *
 * C++语法说明：
 * - using：类型别名声明，等价于typedef
 */
using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::math::Box2d;
using apollo::common::math::NormalizeAngle;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;
using apollo::cyber::Clock;

/**
 * @brief 初始化函数
 *
 * @param config_dir 配置目录路径
 * @param name 任务名称
 * @param injector 依赖注入器指针
 * @return bool 初始化成功返回true
 *
 * 功能说明：
 * 1. 调用基类Task的Init方法进行基础初始化
 * 2. 加载开放空间轨迹后处理配置
 * 3. 初始化车辆参数
 *
 * C++语法说明：
 * - const std::string& config_dir：
 *   常量引用参数，避免拷贝
 *
 * - const std::shared_ptr<DependencyInjector>& injector：
 *   shared_ptr智能指针的常量引用
 *
 * - Task::LoadConfig<T>(&config_)：
 *   模板方法加载protobuf配置
 *
 * - VehicleConfigHelper::Instance()->GetConfig()：
 *   单例模式获取车辆配置
 */
bool OpenSpaceTrajectoryPostProcess::Init(
    const std::string& config_dir, const std::string& name,
    const std::shared_ptr<DependencyInjector>& injector) {
  /**
   * @brief 调用基类初始化
   */
  if (!Task::Init(config_dir, name, injector)) {
    return false;
  }
  /**
   * @brief 加载后处理配置
   */
  bool res = Task::LoadConfig<OpenSpaceTrajectoryPostProcessConfig>(&config_);
  /**
   * @brief 加载搜索范围参数
   */
  heading_search_range_ = config_.heading_search_range();       /**< 航向搜索范围 */
  heading_track_range_ = config_.heading_track_range();         /**< 航向追踪范围 */
  distance_search_range_ = config_.distance_search_range();       /**< 距离搜索范围 */
  heading_offset_to_midpoint_ = config_.heading_offset_to_midpoint();     /**< 到中点航向偏移 */
  lateral_offset_to_midpoint_ = config_.lateral_offset_to_midpoint();     /**< 到中点横向偏移 */
  longitudinal_offset_to_midpoint_ = config_.longitudinal_offset_to_midpoint();  /**< 到中点纵向偏移 */
  vehicle_box_iou_threshold_to_midpoint_ =
      config_.vehicle_box_iou_threshold_to_midpoint();   /**< IOU阈值 */
  scale_destination_ = config_.scale_destination();     /**< 目的地缩放因子 */
  stop_check_window_ = config_.stop_check_window();     /**< 停车检查窗口 */
  /**
   * @brief 获取车辆参数
   */
  vehicle_param_ =
      common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
  ego_length_ = vehicle_param_.length();                 /**< 车辆长度 */
  ego_width_ = vehicle_param_.width();                   /**< 车辆宽度 */
  shift_distance_ = ego_length_ / 2.0 - vehicle_param_.back_edge_to_center();  /**< 后轴到后边缘距离 */
  wheel_base_ = vehicle_param_.wheel_base();             /**< 轴距 */
  AINFO << config_.DebugString();
  return res;
}

/**
 * @brief 主处理函数
 *
 * @return Status 处理状态
 *
 * 功能说明：
 * 1. 检查优化轨迹是否为空
 * 2. 如果有上一帧结果则复用
 * 3. 否则进行轨迹插值、分段
 * 4. 选择最近轨迹点进行追踪
 *
 * C++语法说明：
 * - frame_->open_space_info()：
 *   获取开放空间信息
 *
 * - mutable_open_space_info()：
 *   protobuf可变字段访问
 *
 * - std::priority_queue：
 *   优先队列，存储候选轨迹点及其IOU
 *
 * - Polygon2d::ComputeIoU()：
 *   计算两个多边形的IOU（重叠度）
 */
Status OpenSpaceTrajectoryPostProcess::Process() {
  /**
   * @brief 获取开放空间信息
   */
  const auto& open_space_info = frame_->open_space_info();
  auto open_space_info_ptr = frame_->mutable_open_space_info();
  /**
   * @brief 检查优化轨迹是否为空
   */
  if (open_space_info_ptr->optimizer_trajectory_data().empty()) {
    AINFO << "optimize trajectory empty";
    return Status::OK();
  }

  /**
   * @brief 复制上一帧的轨迹数据
   *
   * partitioned_trajectories：分段后的轨迹列表
   * interpolated_trajectory_result：插值后的轨迹
   */
  const Frame* previous_frame = injector_->frame_history()->Latest();
  if (previous_frame != nullptr) {
    *(open_space_info_ptr->mutable_partitioned_trajectories()) =
        previous_frame->open_space_info().partitioned_trajectories();
    *(open_space_info_ptr->mutable_interpolated_trajectory_result()) =
        previous_frame->open_space_info().interpolated_trajectory_result();
  }

  /**
   * @brief 检查是否已有分段轨迹
   */
  if (open_space_info_ptr->partitioned_trajectories().empty()) {
    const auto& optimized_trajectory_result =
        open_space_info.optimizer_trajectory_data();

    auto* interpolated_trajectory_result_ptr =
        open_space_info_ptr->mutable_interpolated_trajectory_result();

    /**
     * @brief 轨迹插值
     *
     * InterpolateTrajectory：
     *   将稀疏的优化轨迹插值为更密的轨迹点
     */
    InterpolateTrajectory(optimized_trajectory_result,
                          interpolated_trajectory_result_ptr);

    auto* partitioned_trajectories =
        open_space_info_ptr->mutable_partitioned_trajectories();

    /**
     * @brief 轨迹分段
     *
     * PartitionTrajectory：
     *   根据档位变化将轨迹分为多段
     */
    PartitionTrajectory(*interpolated_trajectory_result_ptr,
                        partitioned_trajectories);

    /**
     * @brief 设置初始化标志
     */
    auto* open_space_status = injector_->planning_context()
                                  ->mutable_planning_status()
                                  ->mutable_open_space();
    open_space_status->set_position_init(true);
    auto* chosen_partitioned_trajectory =
        open_space_info_ptr->mutable_chosen_partitioned_trajectory();
    /**
     * @brief 调整相对时间和距离
     */
    AdjustRelativeTimeAndS(open_space_info.partitioned_trajectories(), 0, 0,
                           chosen_partitioned_trajectory);
    current_trajectory_index_ = 0;
    fail_search_fallback_ = false;
    last_index_ = -1;
    return Status::OK();
  }

  auto* partitioned_trajectories =
        open_space_info_ptr->mutable_partitioned_trajectories();
  /**
   * @brief 获取分段轨迹数量
   */
  size_t trajectories_size = partitioned_trajectories->size();
  /**
   * @brief 轨迹索引边界检查
   */
  current_trajectory_index_ =
      std::min<size_t>(trajectories_size - 1, current_trajectory_index_);
  size_t current_trajectory_point_index = 0;
  bool flag_change_to_next = false;

  /**
   * @brief 更新车辆信息
   */
  UpdateVehicleInfo();
  AINFO << "current_trajectory_index_" << current_trajectory_index_;
  /**
   * @brief 获取当前轨迹的档位和轨迹点
   */
  const auto& gear =
      partitioned_trajectories->at(current_trajectory_index_).second;
  const auto& cur_trajectory =
      partitioned_trajectories->at(current_trajectory_index_).first;
  size_t trajectory_size = cur_trajectory.size();
  CHECK_GT(trajectory_size, 0U);
  /**
   * @brief 检查是否到达轨迹终点
   */
  flag_change_to_next = CheckReachTrajectoryEnd(
      cur_trajectory, gear, trajectories_size, &current_trajectory_point_index);
  AINFO << "current_trajectory_index_" << current_trajectory_index_ << ","
        << current_trajectory_point_index << "time index" << last_index_
        << "flag_change_to_next" << flag_change_to_next;
  auto* chosen_partitioned_trajectory =
      open_space_info_ptr->mutable_chosen_partitioned_trajectory();

  /**
   * @brief 检查是否到达目的地
   */
  if (current_trajectory_index_ == partitioned_trajectories->size() - 1
      && current_trajectory_point_index == cur_trajectory.size() - 1) {
    AINFO << "reach destination";
    frame_->mutable_open_space_info()->set_destination_reached(true);
    /**
     * @brief 生成停车轨迹
     */
    GenerateStopTrajectory(
        frame_->local_view().chassis->gear_location(),
        chosen_partitioned_trajectory);
    return Status::OK();
  }

  auto chosen_trajectory = &(chosen_partitioned_trajectory->first);
  AINFO << "Before InsertGearShiftTrajectory [" << chosen_trajectory->size();
  /**
   * @brief 插入档位切换轨迹
   */
  if (config_.use_gear_shift_trajectory()) {
    if (InsertGearShiftTrajectory(flag_change_to_next,
                                  current_trajectory_index_,
                                  open_space_info.partitioned_trajectories(),
                                  chosen_partitioned_trajectory) &&
        chosen_partitioned_trajectory->first.size() != 0) {
      chosen_trajectory = &(chosen_partitioned_trajectory->first);
      ADEBUG << "After InsertGearShiftTrajectory [" << chosen_trajectory->size()
             << "]";
      last_index_ = -1;
      return Status::OK();
    }
  }
  AINFO << "Before InsertStopTrajectory [" << chosen_trajectory->size();
  /**
   * @brief 选择最近的轨迹点
   *
   * std::priority_queue：
   *   优先队列，按IOU值排序
   *   comp_是自定义比较函数
   */
  std::priority_queue<std::pair<size_t, double>,
                      std::vector<std::pair<size_t, double>>, comp_>
      closest_point;
  AINFO << "ego:" << std::fixed << ego_x_ << "," << ego_y_ << ","
        << vehicle_moving_direction_;
  /**
   * @brief 遍历所有轨迹点，寻找最近点
   */
  for (size_t j = 0; j < trajectory_size; ++j) {
    const TrajectoryPoint& trajectory_point = cur_trajectory.at(j);
    const PathPoint& path_point = trajectory_point.path_point();
    const double path_point_x = path_point.x();
    const double path_point_y = path_point.y();
    const double path_point_theta = path_point.theta();
    /**
     * @brief 计算从车辆到轨迹点的向量
     */
    const Vec2d tracking_vector(path_point_x - ego_x_, path_point_y - ego_y_);
    const double distance = tracking_vector.Length();
    /**
     * @brief 计算轨迹点的运动方向
     *
     * 根据档位调整航向角：
     * - 前进档：直接使用path_point_theta
     * - 倒档：加上π（方向反转）
     */
    const double traj_point_moving_direction =
        gear == canbus::Chassis::GEAR_REVERSE
            ? NormalizeAngle(path_point_theta + M_PI)
            : path_point_theta;
    /**
     * @brief 计算航向差异
     */
    const double heading_search_difference = std::abs(NormalizeAngle(
        traj_point_moving_direction - vehicle_moving_direction_));
    /**
     * @brief 过滤条件：距离和航向都在搜索范围内
     */
    if (distance < distance_search_range_ &&
        heading_search_difference < heading_search_range_) {
      /**
       * @brief 计算车辆和轨迹点的IOU
       */
      Box2d path_point_box({path_point_x, path_point_y}, path_point_theta,
                           ego_length_, ego_width_);
      /**
       * @brief 偏移向量（考虑后轴中心）
       */
      Vec2d shift_vec{shift_distance_ * std::cos(path_point_theta),
                      shift_distance_ * std::sin(path_point_theta)};
      path_point_box.Shift(shift_vec);
      /**
       * @brief 计算IOU
       *
       * Polygon2d::ComputeIoU：
       *   计算两个多边形重叠度
       *   IOU = Intersection / Union
       */
      double iou_ratio =
          Polygon2d(ego_box_).ComputeIoU(Polygon2d(path_point_box));
      AINFO << std::fixed << "get closetst point" << path_point_x << ","
            << path_point_y << " distance" << distance << "iou" << iou_ratio
            << "pt theta" << traj_point_moving_direction;
      /**
       * @brief 加入优先队列
       */
      closest_point.emplace(j, iou_ratio);
    }
  }

  AINFO << "closest_point size" << closest_point.size() << " stop_check_count_: " << stop_check_count_;
  static constexpr int stop_count_replan_threshold = 300;
  /**
   * @brief 检查是否需要重规划
   *
   * 条件：
   * 1. 没有找到最近点
   * 2. 搜索失败回退
   * 3. 停车检查超过阈值
   */
  if (closest_point.empty() || fail_search_fallback_ || stop_check_count_ > stop_count_replan_threshold) {
    frame_->mutable_open_space_info()->
        mutable_optimizer_trajectory_data()->clear();
    frame_->mutable_open_space_info()->
        mutable_path_planning_trajectory_result()->clear();
    frame_->mutable_open_space_info()->
        mutable_interpolated_trajectory_result()->clear();
    frame_->mutable_open_space_info()->
        mutable_partitioned_trajectories()->clear();
    frame_->mutable_open_space_info()->
        mutable_chosen_partitioned_trajectory()->first.clear();
    const std::string msg =
        "Fail to find nearest trajectory point to follow stop to fallback "
        "replan";
    AERROR << msg;
    stop_check_count_ = 0;
    return Status::OK();
  }
  /**
   * @brief 获取IOU最大的轨迹点
   */
  current_trajectory_point_index = closest_point.top().first;
  if (!flag_change_to_next) {
    /**
     * @brief 时间匹配
     */
    double veh_rel_time;
    size_t time_match_index;
    double now_time = Clock::Instance()->NowInSeconds();
    auto& traj = partitioned_trajectories->at(current_trajectory_index_).first;
    if (last_index_ != -1) {
      /**
       * @brief 根据上一帧的时间推算当前时间
       */
      veh_rel_time = traj[last_index_].relative_time() + now_time - last_time_;
      AINFO << std::fixed << now_time << "," << last_time_;
      time_match_index = traj.QueryLowerBoundPoint(veh_rel_time);
    } else {
      time_match_index = current_trajectory_point_index;
      last_index_ = time_match_index;
      last_time_ = now_time;
    }

    AINFO << "time_match_index" << time_match_index << "pos match index"
          << current_trajectory_point_index;
    AINFO << "TRAJ CLOSEST" << std::fixed
          << traj.at(current_trajectory_point_index).path_point().x() << ","
          << traj.at(current_trajectory_point_index).path_point().y();
    /**
     * @brief 检查位置匹配是否合理
     */
    if (std::abs(traj[time_match_index].path_point().s() -
                 traj.at(current_trajectory_point_index).path_point().s()) <
        config_.speed_replan_distance()) {
      current_trajectory_point_index = time_match_index;
    } else {
      AINFO << "reset speed because matched point too far";
      last_index_ = current_trajectory_point_index;
      last_time_ = now_time;
    }
  } else {
    last_index_ = -1;
  }

  AINFO << "current_trajectory_point_index" << current_trajectory_point_index;
  AdjustRelativeTimeAndS(open_space_info.partitioned_trajectories(),
                         current_trajectory_index_,
                         current_trajectory_point_index,
                         chosen_partitioned_trajectory);
  return Status::OK();
}

/**
 * @brief 轨迹插值函数
 *
 * @param optimized_trajectory_result 优化后的稀疏轨迹
 * @param interpolated_trajectory 输出参数，插值后的密轨迹
 *
 * 功能说明：
 * 对稀疏的优化轨迹点进行线性插值
 * 生成更密集的轨迹点用于控制
 *
 * C++语法说明：
 * - FLAGS_use_iterative_anchoring_smoother：
 *   gflags布尔配置，控制是否使用迭代锚定平滑器
 *
 * - interpolated_pieces_num：
 *   每两个优化点之间插入的点数
 *
 * - static_cast<double>(...)：
 *   显式类型转换
 *
 * - emplace_back：
 *   直接在容器末尾构造元素
 *
 * - common::math::InterpolateUsingLinearApproximation：
 *   线性插值函数
 */
void OpenSpaceTrajectoryPostProcess::InterpolateTrajectory(
    const DiscretizedTrajectory& optimized_trajectory_result,
    DiscretizedTrajectory* interpolated_trajectory) {
  /**
   * @brief 检查是否使用迭代锚定平滑器
   *
   * 如果使用，则跳过插值
   */
  if (FLAGS_use_iterative_anchoring_smoother) {
    *interpolated_trajectory = optimized_trajectory_result;
    return;
  }
  interpolated_trajectory->clear();
  size_t interpolated_pieces_num = config_.interpolated_pieces_num();
  CHECK_GT(optimized_trajectory_result.size(), 0U);
  CHECK_GT(interpolated_pieces_num, 0U);
  /**
   * @brief 计算插值参数
   */
  size_t trajectory_to_be_partitioned_intervals_num =
      optimized_trajectory_result.size() - 1;
  size_t interpolated_points_num = interpolated_pieces_num - 1;
  /**
   * @brief 遍历所有区间进行插值
   */
  for (size_t i = 0; i < trajectory_to_be_partitioned_intervals_num; ++i) {
    /**
     * @brief 计算每个区间的时间间隔
     */
    double relative_time_interval =
        (optimized_trajectory_result.at(i + 1).relative_time() -
         optimized_trajectory_result.at(i).relative_time()) /
        static_cast<double>(interpolated_pieces_num);
    /**
     * @brief 添加起始点
     */
    interpolated_trajectory->push_back(optimized_trajectory_result.at(i));
    /**
     * @brief 在区间内插入点
     */
    for (size_t j = 0; j < interpolated_points_num; ++j) {
      double relative_time =
          optimized_trajectory_result.at(i).relative_time() +
          (static_cast<double>(j) + 1.0) * relative_time_interval;
      /**
       * @brief 线性插值
       */
      interpolated_trajectory->emplace_back(
          common::math::InterpolateUsingLinearApproximation(
              optimized_trajectory_result.at(i),
              optimized_trajectory_result.at(i + 1), relative_time));
    }
  }
  /**
   * @brief 添加终点
   */
  interpolated_trajectory->push_back(optimized_trajectory_result.back());
}

/**
 * @brief 更新车辆信息
 *
 * 功能说明：
 * 从车辆状态更新当前位置、速度、档位等信息
 * 更新车辆边界框
 *
 * C++语法说明：
 * - Box2d box({x, y}, heading, length, width)：
 *   构造2D边界框
 *
 * - std::move(box)：
 *   移动语义，避免拷贝
 *
 * - vehicle_state.gear()：
 *   获取当前档位
 *
 * - absl/fabs：
 *   绝对值函数
 */
void OpenSpaceTrajectoryPostProcess::UpdateVehicleInfo() {
  const common::VehicleState& vehicle_state = frame_->vehicle_state();
  ego_theta_ = vehicle_state.heading();                  /**< 车辆航向角 */
  ego_x_ = vehicle_state.x();                               /**< 车辆x坐标 */
  ego_y_ = vehicle_state.y();                               /**< 车辆y坐标 */
  ego_v_ = vehicle_state.linear_velocity();                 /**< 车辆线速度 */
  ego_gear_ = vehicle_state.gear();                        /**< 车辆档位 */
  /**
   * @brief 创建车辆边界框
   */
  Box2d box({ego_x_, ego_y_}, ego_theta_, ego_length_, ego_width_);
  ego_box_ = std::move(box);
  /**
   * @brief 计算后轴中心偏移
   */
  Vec2d ego_shift_vec{shift_distance_ * std::cos(ego_theta_),
                      shift_distance_ * std::sin(ego_theta_)};
  ego_box_.Shift(ego_shift_vec);
  /**
   * @brief 计算车辆运动方向
   */
  vehicle_moving_direction_ =
      vehicle_state.gear() == canbus::Chassis::GEAR_REVERSE
          ? NormalizeAngle(ego_theta_ + M_PI)
          : ego_theta_;
  /**
   * @brief 更新停车计数
   */
  AINFO << fabs(ego_v_) << "speed" << vehicle_param_.max_abs_speed_when_stopped() << " " << stop_check_count_;
  stop_check_count_ = fabs(ego_v_) < vehicle_param_.max_abs_speed_when_stopped() ?
                          stop_check_count_ + 1 : 0;
}

/**
 * @brief 轨迹编码函数
 *
 * @param trajectory 要编码的轨迹
 * @param encoding 输出参数，编码字符串
 * @return bool 编码成功返回true
 *
 * 功能说明：
 * 将轨迹的起点和终点信息编码为字符串
 * 用于轨迹历史记录和去重
 *
 * C++语法说明：
 * - static_cast<int>(...)：
 *   显式转换为整型
 * - absl::StrCat：
 *   字符串拼接
 */
bool OpenSpaceTrajectoryPostProcess::EncodeTrajectory(
    const DiscretizedTrajectory& trajectory, std::string* const encoding) {
  if (trajectory.empty()) {
    AERROR << "Fail to encode trajectory because it is empty";
    return false;
  }
  /**
   * @brief 编码原点（地图坐标偏移）
   */
  static constexpr double encoding_origin_x = 58700.0;
  static constexpr double encoding_origin_y = 4141000.0;
  const auto& init_path_point = trajectory.front().path_point();
  const auto& last_path_point = trajectory.back().path_point();

  /**
   * @brief 计算起点坐标（整数化）
   */
  const int init_point_x =
      static_cast<int>((init_path_point.x() - encoding_origin_x) * 1000.0);
  const int init_point_y =
      static_cast<int>((init_path_point.y() - encoding_origin_y) * 1000.0);
  const int init_point_heading =
      static_cast<int>(init_path_point.theta() * 10000.0);
  /**
   * @brief 计算终点坐标（整数化）
   */
  const int last_point_x =
      static_cast<int>((last_path_point.x() - encoding_origin_x) * 1000.0);
  const int last_point_y =
      static_cast<int>((last_path_point.y() - encoding_origin_y) * 1000.0);
  const int last_point_heading =
      static_cast<int>(last_path_point.theta() * 10000.0);

  /**
   * @brief 拼接编码字符串
   */
  *encoding = absl::StrCat(
      // init point
      init_point_x, "_", init_point_y, "_", init_point_heading, "/",
      // last point
      last_point_x, "_", last_point_y, "_", last_point_heading);
  return true;
}

/**
 * @brief 检查轨迹是否已被遍历
 *
 * @param trajectory_encoding_to_check 要检查的轨迹编码
 * @return bool 如果已遍历返回true
 *
 * 功能说明：
 * 检查轨迹历史记录
 * 避免重复选择同一轨迹
 */
bool OpenSpaceTrajectoryPostProcess::CheckTrajTraversed(
    const std::string& trajectory_encoding_to_check) {
  const auto& open_space_status =
      injector_->planning_context()->planning_status().open_space();
  const int index_history_size =
      open_space_status.partitioned_trajectories_index_history_size();

  if (index_history_size <= 1) {
    return false;
  }
  for (int i = 0; i < index_history_size - 1; i++) {
    const auto& index_history =
        open_space_status.partitioned_trajectories_index_history(i);
    if (index_history == trajectory_encoding_to_check) {
      return true;
    }
  }
  return false;
}

/**
 * @brief 更新轨迹历史记录
 *
 * @param chosen_trajectory_encoding 选择的轨迹编码
 */
void OpenSpaceTrajectoryPostProcess::UpdateTrajHistory(
    const std::string& chosen_trajectory_encoding) {
  auto* open_space_status = injector_->planning_context()
                                ->mutable_planning_status()
                                ->mutable_open_space();

  const auto& trajectory_history =
      injector_->planning_context()
          ->planning_status()
          .open_space()
          .partitioned_trajectories_index_history();
  if (trajectory_history.empty()) {
    open_space_status->add_partitioned_trajectories_index_history(
        chosen_trajectory_encoding);
    return;
  }
  /**
   * @brief 检查是否与上一个轨迹相同
   */
  if (*(trajectory_history.rbegin()) == chosen_trajectory_encoding) {
    return;
  }
  open_space_status->add_partitioned_trajectories_index_history(
      chosen_trajectory_encoding);
}

/**
 * @brief 轨迹分段函数
 *
 * @param raw_trajectory 原始轨迹
 * @param partitioned_trajectories 输出参数，分段后的轨迹列表
 *
 * 功能说明：
 * 根据档位变化将连续轨迹分为多段
 * 每段有独立的档位（前进/倒档）
 *
 * C++语法说明：
 * - TrajGearPair：
 *   (DiscretizedTrajectory, GearPosition) 的别名
 *   轨迹和档位的配对
 *
 * - CHECK_NOTNULL：
 *   Apollo断言宏，检查指针非空
 *
 * - emplace_back：
 *   直接构造并添加元素
 *
 * - Vec2d::DistanceTo()：
 *   计算两点间距离
 */
void OpenSpaceTrajectoryPostProcess::PartitionTrajectory(
    const DiscretizedTrajectory& raw_trajectory,
    std::vector<TrajGearPair>* partitioned_trajectories) {
  CHECK_NOTNULL(partitioned_trajectories);

  size_t horizon = raw_trajectory.size();

  partitioned_trajectories->clear();
  partitioned_trajectories->emplace_back();
  TrajGearPair* current_trajectory_gear = &(partitioned_trajectories->back());

  auto* trajectory = &(current_trajectory_gear->first);
  auto* gear = &(current_trajectory_gear->second);

  /**
   * @brief 确定初始档位
   *
   * 比较轨迹方向和航向角：
   * - 角度差 < 90度：前进档
   * - 角度差 >= 90度：倒档
   */
  const auto& first_path_point = raw_trajectory.front().path_point();
  const auto& second_path_point = raw_trajectory[1].path_point();
  double heading_angle = first_path_point.theta();
  const Vec2d init_tracking_vector(
      second_path_point.x() - first_path_point.x(),
      second_path_point.y() - first_path_point.y());
  double tracking_angle = init_tracking_vector.Angle();
  *gear =
      std::abs(common::math::NormalizeAngle(tracking_angle - heading_angle)) <
              (M_PI_2)
          ? canbus::Chassis::GEAR_DRIVE
          : canbus::Chassis::GEAR_REVERSE;

  /**
   * @brief 初始化累计距离
   */
  Vec2d last_pos_vec(first_path_point.x(), first_path_point.y());
  double distance_s = 0.0;
  bool is_trajectory_last_point = false;

  /**
   * @brief 遍历轨迹点，检查档位变化
   */
  for (size_t i = 0; i < horizon - 1; ++i) {
    const TrajectoryPoint& trajectory_point = raw_trajectory.at(i);
    const TrajectoryPoint& next_trajectory_point = raw_trajectory.at(i + 1);

    /**
     * @brief 检查档位变化
     */
    heading_angle = trajectory_point.path_point().theta();
    const Vec2d tracking_vector(next_trajectory_point.path_point().x() -
                                    trajectory_point.path_point().x(),
                                next_trajectory_point.path_point().y() -
                                    trajectory_point.path_point().y());
    tracking_angle = tracking_vector.Angle();
    auto cur_gear =
        std::abs(common::math::NormalizeAngle(tracking_angle - heading_angle)) <
                (M_PI_2)
            ? canbus::Chassis::GEAR_DRIVE
            : canbus::Chassis::GEAR_REVERSE;

    /**
     * @brief 检测到档位变化，创建新段
     */
    if (cur_gear != *gear) {
      is_trajectory_last_point = true;
      LoadTrajectoryPoint(trajectory_point, is_trajectory_last_point, *gear,
                          &last_pos_vec, &distance_s, trajectory);
      partitioned_trajectories->emplace_back();
      current_trajectory_gear = &(partitioned_trajectories->back());
      current_trajectory_gear->second = cur_gear;
      distance_s = 0.0;
      is_trajectory_last_point = false;
    }

    trajectory = &(current_trajectory_gear->first);
    gear = &(current_trajectory_gear->second);

    LoadTrajectoryPoint(trajectory_point, is_trajectory_last_point, *gear,
                        &last_pos_vec, &distance_s, trajectory);
  }
  is_trajectory_last_point = true;
  const TrajectoryPoint& last_trajectory_point = raw_trajectory.back();
  LoadTrajectoryPoint(last_trajectory_point, is_trajectory_last_point, *gear,
                      &last_pos_vec, &distance_s, trajectory);
}

/**
 * @brief 加载轨迹点
 *
 * @param trajectory_point 源轨迹点
 * @param is_trajectory_last_point 是否为段终点
 * @param gear 当前档位
 * @param last_pos_vec 输入输出：上一位置向量
 * @param distance_s 输入输出：累计距离
 * @param current_trajectory 输出：当前轨迹
 *
 * 功能说明：
 * 将轨迹点添加到当前段
 * 计算累计距离s
 *
 * C++语法说明：
 * - current_trajectory->emplace_back()：
 *   在末尾添加空元素
 *
 * - std::tan(steer) / wheel_base：
 *   根据前轮转角计算曲率kappa
 */
void OpenSpaceTrajectoryPostProcess::LoadTrajectoryPoint(
    const TrajectoryPoint& trajectory_point,
    const bool is_trajectory_last_point,
    const canbus::Chassis::GearPosition& gear, Vec2d* last_pos_vec,
    double* distance_s, DiscretizedTrajectory* current_trajectory) {
  current_trajectory->emplace_back();
  TrajectoryPoint* point = &(current_trajectory->back());
  point->set_relative_time(trajectory_point.relative_time());
  point->mutable_path_point()->set_x(trajectory_point.path_point().x());
  point->mutable_path_point()->set_y(trajectory_point.path_point().y());
  point->mutable_path_point()->set_theta(trajectory_point.path_point().theta());
  point->set_v(trajectory_point.v());
  point->mutable_path_point()->set_s(*distance_s);
  Vec2d cur_pos_vec(trajectory_point.path_point().x(),
                    trajectory_point.path_point().y());
  /**
   * @brief 计算累计距离
   *
   * 前进为正，倒档为负
   */
  *distance_s += (gear == canbus::Chassis::GEAR_REVERSE ? -1.0 : 1.0) *
                 (cur_pos_vec.DistanceTo(*last_pos_vec));
  *last_pos_vec = cur_pos_vec;
  /**
   * @brief 计算曲率
   *
   * kappa = tan(steer) / wheel_base
   * 终点曲率为负（准备换档）
   */
  point->mutable_path_point()->set_kappa((is_trajectory_last_point ? -1 : 1) *
                                         std::tan(trajectory_point.steer()) /
                                         wheel_base_);
  point->set_a(trajectory_point.a());

  AINFO << "Load trajectory point: " << point->DebugString();
}

/**
 * @brief 检查是否到达轨迹终点
 *
 * @param trajectory 当前轨迹
 * @param gear 当前档位
 * @param trajectories_size 轨迹总数
 * @param current_trajectory_point_index 输出：轨迹点索引
 * @return bool 是否需要切换到下一段
 */
bool OpenSpaceTrajectoryPostProcess::CheckReachTrajectoryEnd(
    const DiscretizedTrajectory& trajectory,
    const canbus::Chassis::GearPosition& gear, const size_t trajectories_size,
    size_t* current_trajectory_point_index) {
  const TrajectoryPoint& trajectory_end_point = trajectory.back();
  const size_t trajectory_size = trajectory.size();
  const PathPoint& path_end_point = trajectory_end_point.path_point();
  AINFO << "scale_destination_:" << scale_destination_;
  /**
   * @brief 计算目的地缩放因子
   *
   * 最后一段使用缩放因子
   */
  double scale = current_trajectory_index_ == frame_->open_space_info().partitioned_trajectories().size() - 1
          ? scale_destination_
          : 1.0;
  AINFO << "Scale: " << scale;
  /**
   * @brief 检查是否到达目标点
   */
  if (CheckArrivePoint(
        gear,
        path_end_point,
        scale * lateral_offset_to_midpoint_,
        scale * longitudinal_offset_to_midpoint_,
        scale * heading_offset_to_midpoint_)) {
    if (current_trajectory_index_ + 1 >= trajectories_size) {
      current_trajectory_index_ = trajectories_size - 1;
      *current_trajectory_point_index = trajectory_size - 1;
    } else {
      current_trajectory_index_ += 1;
      *current_trajectory_point_index = 0;
    }
    stop_check_count_ = 0;
    AINFO << "Reach the end of a trajectory, switching to next one";
    return true;
  } else {
    return false;
  }
}

/**
 * @brief 失败安全搜索
 *
 * @param partitioned_trajectories 分段轨迹
 * @param trajectories_encodings 轨迹编码列表
 * @param current_trajectory_index 输出：选择的轨迹索引
 * @param current_trajectory_point_index 输出：选择的点索引
 * @return bool 搜索成功返回true
 *
 * 功能说明：
 * 当正常搜索失败时
 * 搜索所有轨迹找最近的有效点
 */
bool OpenSpaceTrajectoryPostProcess::UseFailSafeSearch(
    const std::vector<TrajGearPair>& partitioned_trajectories,
    const std::vector<std::string>& trajectories_encodings,
    size_t* current_trajectory_index, size_t* current_trajectory_point_index) {
  AERROR << "Trajectory partition fail, using failsafe search";
  const size_t trajectories_size = partitioned_trajectories.size();
  /**
   * @brief 存储所有轨迹的最近点
   */
  std::priority_queue<std::pair<std::pair<size_t, size_t>, double>,
                      std::vector<std::pair<std::pair<size_t, size_t>, double>>,
                      pair_comp_>
      failsafe_closest_point_on_trajs;
  /**
   * @brief 遍历所有轨迹
   */
  for (size_t i = 0; i < trajectories_size; ++i) {
    const auto& trajectory = partitioned_trajectories.at(i).first;
    size_t trajectory_size = trajectory.size();
    CHECK_GT(trajectory_size, 0U);
    std::priority_queue<std::pair<size_t, double>,
                        std::vector<std::pair<size_t, double>>, comp_>
        failsafe_closest_point;

    /**
     * @brief 遍历轨迹点找最近点
     */
    for (size_t j = 0; j < trajectory_size; ++j) {
      const TrajectoryPoint& trajectory_point = trajectory.at(j);
      const PathPoint& path_point = trajectory_point.path_point();
      const double path_point_x = path_point.x();
      const double path_point_y = path_point.y();
      const double path_point_theta = path_point.theta();
      const Vec2d tracking_vector(path_point_x - ego_x_, path_point_y - ego_y_);
      const double distance = tracking_vector.Length();
      if (distance < distance_search_range_) {
        Box2d path_point_box({path_point_x, path_point_y}, path_point_theta,
                             ego_length_, ego_width_);
        Vec2d shift_vec{shift_distance_ * std::cos(path_point_theta),
                        shift_distance_ * std::sin(path_point_theta)};
        path_point_box.Shift(shift_vec);
        double iou_ratio =
            Polygon2d(ego_box_).ComputeIoU(Polygon2d(path_point_box));
        failsafe_closest_point.emplace(j, iou_ratio);
      }
    }
    if (!failsafe_closest_point.empty()) {
      size_t closest_point_index = failsafe_closest_point.top().first;
      double max_iou_ratio = failsafe_closest_point.top().second;
      failsafe_closest_point_on_trajs.emplace(
          std::make_pair(i, closest_point_index), max_iou_ratio);
    }
  }
  if (failsafe_closest_point_on_trajs.empty()) {
    return false;
  } else {
    bool closest_and_not_repeated_traj_found = false;
    /**
     * @brief 选择IOU最大且未遍历的轨迹
     */
    while (!failsafe_closest_point_on_trajs.empty()) {
      *current_trajectory_index =
          failsafe_closest_point_on_trajs.top().first.first;
      *current_trajectory_point_index =
          failsafe_closest_point_on_trajs.top().first.second;
      if (CheckTrajTraversed(
              trajectories_encodings[*current_trajectory_index])) {
        failsafe_closest_point_on_trajs.pop();
      } else {
        closest_and_not_repeated_traj_found = true;
        UpdateTrajHistory(trajectories_encodings[*current_trajectory_index]);
        return true;
      }
    }
    if (!closest_and_not_repeated_traj_found) {
      return false;
    }

    return true;
  }
}

/**
 * @brief 插入档位切换轨迹
 *
 * @param flag_change_to_next 是否切换到下一段
 * @param current_trajectory_index 当前轨迹索引
 * @param partitioned_trajectories 分段轨迹列表
 * @param gear_switch_idle_time_trajectory 输出：档位切换轨迹
 * @return bool 处理成功返回true
 */
bool OpenSpaceTrajectoryPostProcess::InsertGearShiftTrajectory(
    const bool flag_change_to_next, const size_t current_trajectory_index,
    const std::vector<TrajGearPair>& partitioned_trajectories,
    TrajGearPair* gear_switch_idle_time_trajectory) {
  const auto* last_frame = injector_->frame_history()->Latest();
  auto* current_gear_status =
      frame_->mutable_open_space_info()->mutable_gear_switch_states();
  if (last_frame) {
    const auto& last_gear_status =
        last_frame->open_space_info().gear_switch_states();
    *(current_gear_status) = last_gear_status;
  } else {
    AERROR << "Lost last frame";
  }
  const auto& curr_gear =
      partitioned_trajectories.at(current_trajectory_index).second;
  /**
   * @brief 检查是否需要档位切换
   */
  if (flag_change_to_next || !current_gear_status->gear_shift_period_finished ||
      curr_gear != ego_gear_) {
    current_gear_status->gear_shift_period_finished = false;
    if (current_gear_status->gear_shift_period_started) {
      current_gear_status->gear_shift_start_time =
          Clock::Instance()->NowInSeconds();
      current_gear_status->gear_shift_position =
          partitioned_trajectories.at(current_trajectory_index).second;
      current_gear_status->gear_shift_period_started = false;
      current_gear_status->gear_shift_period_time = 0.0;
    }
    /**
     * @brief 检查档位切换是否完成
     */
    if (current_gear_status->gear_shift_period_time >
            config_.gear_shift_period_duration() &&
        current_gear_status->gear_shift_position == ego_gear_) {
      current_gear_status->gear_shift_period_finished = true;
      current_gear_status->gear_shift_period_started = true;
      stop_check_count_ = 0;
      AINFO << "finished gear shift";
    } else {
      double init_kappa = partitioned_trajectories.at(current_trajectory_index)
                              .first[0]
                              .path_point()
                              .kappa();
      GenerateGearShiftTrajectory(current_gear_status->gear_shift_position,
                                  init_kappa, gear_switch_idle_time_trajectory);
      AINFO << "change gear: " << current_gear_status->gear_shift_position;
      current_gear_status->gear_shift_period_time =
          Clock::Instance()->NowInSeconds() -
          current_gear_status->gear_shift_start_time;
      return true;
    }
  }

  return true;
}

/**
 * @brief 生成档位切换轨迹
 *
 * @param gear_position 目标档位
 * @param init_kappa 初始曲率
 * @param gear_switch_idle_time_trajectory 输出：生成的轨迹
 */
void OpenSpaceTrajectoryPostProcess::GenerateGearShiftTrajectory(
    const canbus::Chassis::GearPosition& gear_position, double init_kappa,
    TrajGearPair* gear_switch_idle_time_trajectory) {
  gear_switch_idle_time_trajectory->first.clear();
  const double gear_shift_max_t = config_.gear_shift_max_t();
  const double gear_shift_unit_t = config_.gear_shift_unit_t();
  /**
   * @brief 生成静止的过渡轨迹
   */
  for (double t = 0.0; t < gear_shift_max_t; t += gear_shift_unit_t) {
    TrajectoryPoint point;
    point.mutable_path_point()->set_x(frame_->vehicle_state().x());
    point.mutable_path_point()->set_y(frame_->vehicle_state().y());
    point.mutable_path_point()->set_theta(frame_->vehicle_state().heading());
    point.mutable_path_point()->set_s(0.0);
    point.mutable_path_point()->set_kappa(init_kappa);
    point.set_relative_time(t);
    point.set_v(0.0);
    point.set_a(0.0);
    gear_switch_idle_time_trajectory->first.emplace_back(point);
  }
  ADEBUG << "gear_switch_idle_time_trajectory"
         << gear_switch_idle_time_trajectory->first.size();
  gear_switch_idle_time_trajectory->second = gear_position;
}

/**
 * @brief 生成停车轨迹
 *
 * @param gear_position 当前档位
 * @param stop_trajectory 输出：生成的停车轨迹
 */
void OpenSpaceTrajectoryPostProcess::GenerateStopTrajectory(
    const canbus::Chassis::GearPosition& gear_position,
    TrajGearPair* stop_trajectory) {
  stop_trajectory->first.clear();
  /**
   * @brief 根据档位决定加速度方向
   */
  int alpha = gear_position == canbus::Chassis::GEAR_DRIVE ? 1 : -1;
  /**
   * @brief 生成2秒的停车轨迹
   */
  for (double t = 0.0; t < 2.0; t += 0.1) {
    TrajectoryPoint point;
    point.mutable_path_point()->set_x(frame_->vehicle_state().x());
    point.mutable_path_point()->set_y(frame_->vehicle_state().y());
    point.mutable_path_point()->set_theta(frame_->vehicle_state().heading());
    point.mutable_path_point()->set_s(0.0);
    point.mutable_path_point()->set_kappa(0.0);
    point.set_relative_time(t);
    point.set_v(0.0);
    point.set_a(alpha * -0.2);
    stop_trajectory->first.emplace_back(point);
  }
  ADEBUG << "stop_trajectory"
         << stop_trajectory->first.size();
  stop_trajectory->second = gear_position;
}

/**
 * @brief 调整相对时间和距离
 *
 * @param partitioned_trajectories 分段轨迹列表
 * @param current_trajectory_index 当前轨迹索引
 * @param closest_trajectory_point_index 最近点索引
 * @param current_partitioned_trajectory 输出：调整后的轨迹
 *
 * 功能说明：
 * 将最近点设为原点
 * 调整所有轨迹点的相对时间和距离
 */
void OpenSpaceTrajectoryPostProcess::AdjustRelativeTimeAndS(
    const std::vector<TrajGearPair>& partitioned_trajectories,
    const size_t current_trajectory_index,
    const size_t closest_trajectory_point_index,
    TrajGearPair* current_partitioned_trajectory) {
  const size_t partitioned_trajectories_size = partitioned_trajectories.size();
  CHECK_GT(partitioned_trajectories_size, current_trajectory_index);

  /**
   * @brief 复制当前轨迹段
   */
  *(current_partitioned_trajectory) =
      partitioned_trajectories.at(current_trajectory_index);
  auto trajectory = &(current_partitioned_trajectory->first);

  for (int i = 0; i < trajectory->size(); i++) {
    ADEBUG << "trajectory: " << i << " " << std::setprecision(9) <<
    trajectory->at(i).path_point().x() << " " <<
    trajectory->at(i).path_point().y();
  }

  /**
   * @brief 计算偏移量
   */
  double time_shift =
      trajectory->at(closest_trajectory_point_index).relative_time();
  double s_shift =
      trajectory->at(closest_trajectory_point_index).path_point().s();
  const size_t trajectory_size = trajectory->size();
  /**
   * @brief 调整所有点的相对时间和距离
   */
  for (size_t i = 0; i < trajectory_size; ++i) {
    TrajectoryPoint* trajectory_point = &(trajectory->at(i));
    trajectory_point->set_relative_time(trajectory_point->relative_time() -
                                        time_shift);
    trajectory_point->mutable_path_point()->set_s(
        trajectory_point->path_point().s() - s_shift);
  }
}

/**
 * @brief 检查是否到达目标点
 *
 * @param gear 当前档位
 * @param path_end_point 目标点
 * @param lateral_offset_to_midpoint 横向容差
 * @param longitudinal_offset_to_midpoint 纵向容差
 * @param heading_offset_to_midpoint 航向容差
 * @return bool 到达返回true
 */
bool OpenSpaceTrajectoryPostProcess::CheckArrivePoint(
        const canbus::Chassis::GearPosition& gear,
        const PathPoint& path_end_point,
        const double& lateral_offset_to_midpoint,
        const double& longitudinal_offset_to_midpoint,
        const double& heading_offset_to_midpoint) {
  AINFO << "path_end_point " << path_end_point.DebugString();
  AINFO << "ego_x_: " << ego_x_
      << ", ego_y_: " << ego_y_
      << " heading: " << vehicle_moving_direction_;
  const double path_end_point_x = path_end_point.x();
  const double path_end_point_y = path_end_point.y();
  const Vec2d tracking_vector(ego_x_ - path_end_point_x, ego_y_ - path_end_point_y);
  const double path_end_point_theta = path_end_point.theta();
  const double included_angle = NormalizeAngle(path_end_point_theta - tracking_vector.Angle());
  /**
   * @brief 计算到终点的距离
   */
  const double distance_to_trajs_end = std::sqrt(
          (path_end_point_x - ego_x_) * (path_end_point_x - ego_x_)
          + (path_end_point_y - ego_y_) * (path_end_point_y - ego_y_));
  /**
   * @brief 计算横向和纵向偏移
   */
  const double lateral_offset = std::abs(distance_to_trajs_end * std::sin(included_angle));
  const double longitudinal_offset = std::abs(distance_to_trajs_end * std::cos(included_angle));
  const double traj_end_point_moving_direction
          = ego_gear_ == canbus::Chassis::GEAR_REVERSE ? NormalizeAngle(path_end_point_theta + M_PI) : path_end_point_theta;

  const double heading_search_to_trajs_end
          = std::abs(NormalizeAngle(traj_end_point_moving_direction - vehicle_moving_direction_));

  /**
   * @brief 检查所有到达条件
   */
  double end_point_iou_ratio = 0.0;
  if (lateral_offset < lateral_offset_to_midpoint && longitudinal_offset < longitudinal_offset_to_midpoint
      && heading_search_to_trajs_end < heading_offset_to_midpoint
      && stop_check_count_ > stop_check_window_) {
    /**
     * @brief 计算IOU
     */
    Box2d path_end_point_box({path_end_point_x, path_end_point_y}, path_end_point_theta, ego_length_, ego_width_);
    Vec2d shift_vec{shift_distance_ * std::cos(path_end_point_theta), shift_distance_ * std::sin(path_end_point_theta)};
    path_end_point_box.Shift(shift_vec);
    end_point_iou_ratio = Polygon2d(ego_box_).ComputeIoU(Polygon2d(path_end_point_box));

    if (end_point_iou_ratio > vehicle_box_iou_threshold_to_midpoint_) {
      AINFO << "ego reach point";
      return true;
    }
  }

  AINFO << "Vehicle did not reach end of a trajectory with conditions for "
           "lateral distance_check: "
        << (lateral_offset < lateral_offset_to_midpoint) << " and actual lateral distance: " << lateral_offset
        << "; longitudinal distance_check: " << (longitudinal_offset < longitudinal_offset_to_midpoint)
        << " and actual longitudinal distance: " << longitudinal_offset
        << "; heading_check: " << (heading_search_to_trajs_end < heading_offset_to_midpoint)
        << " with actual heading: " << heading_search_to_trajs_end
        << "; stop_check: " << (stop_check_count_ > stop_check_window_)
        << " with actual stop count: " << stop_check_count_
        << "; velocity_check: " << (std::abs(ego_v_) < vehicle_param_.max_abs_speed_when_stopped())
        << " with actual linear velocity: " << ego_v_
        << "; iou_check: " << (end_point_iou_ratio > vehicle_box_iou_threshold_to_midpoint_)
        << " with actual iou: " << end_point_iou_ratio;
  return false;
}

}  // namespace planning
}  // namespace apollo
