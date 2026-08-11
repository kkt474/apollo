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
 * @file path_decider.cc
 * @brief 路径决策任务实现文件
 *
 * 本文件实现了PathDecider类，负责对路径上的障碍物进行决策。
 *
 * 决策类型：
 * 1. IGNORE - 忽略障碍物（不在路径上或距离足够远）
 * 2. STOP - 停车决策（障碍物在路径上且无法绕行）
 * 3. NUDGE - 绕行决策（障碍物在路径上但可以绕行）
 *    - LEFT_NUDGE: 向左绕行
 *    - RIGHT_NUDGE: 向右绕行
 *
 * 核心逻辑：
 * - 遍历所有障碍物
 * - 判断障碍物是否在路径的S范围内
 * - 判断障碍物是否在路径的L范围内
 * - 根据相对位置生成决策
 *
 * C++语法说明：
 * - std::shared_ptr<T>: 智能指针，引用计数
 * - protobuf消息: ObjectDecisionType, ObjectStop, ObjectNudge等
 * - mutable_xxx(): protobuf消息的可修改访问器
 */

#include "modules/planning/tasks/path_decider/path_decider.h"

/**
 * @brief 标准库头文件
 * <algorithm>: 提供std::max, std::min等算法
 * <memory>: 提供std::shared_ptr智能指针
 */
#include <algorithm>
#include <memory>

#include "modules/common_msgs/planning_msgs/decision.pb.h"
/**
 * @brief 规划决策消息protobuf定义
 * ObjectDecisionType: 障碍物决策类型
 * ObjectStop: 停车决策
 * ObjectNudge: 绕行决策
 */

#include "modules/common/configs/vehicle_config_helper.h"
/**
 * @brief 车辆配置辅助类
 * VehicleConfigHelper::GetConfig(): 获取车辆配置参数
 */
#include "modules/common/util/util.h"
/**
 * @brief 通用工具函数
 */
#include "modules/planning/planning_base/common/planning_context.h"
/**
 * @brief 规划上下文
 * 存储规划过程中的共享状态
 */
#include "modules/planning/planning_base/common/util/print_debug_info.h"
/**
 * @brief 调试信息打印工具
 */
#include "modules/planning/planning_base/gflags/planning_gflags.h"
/**
 * @brief Planning模块的GFlags配置参数
 */

namespace apollo {
/**
 * @brief Apollo顶层命名空间
 */
namespace planning {

/**
 * @brief 类型别名定义
 * using: 类型别名，简化代码
 */
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleConfigHelper;

/**
 * @brief 初始化路径决策任务
 *
 * @param config_dir 配置文件目录
 * @param name 任务名称
 * @param injector 依赖注入器
 * @return bool 初始化是否成功
 *
 * 初始化流程：
 * 1. 调用基类Task::Init进行基础初始化
 * 2. 加载PathDeciderConfig配置
 */
bool PathDecider::Init(const std::string &config_dir, const std::string &name,
                       const std::shared_ptr<DependencyInjector> &injector) {
  /**
   * @brief 调用基类初始化
   */
  if (!Task::Init(config_dir, name, injector)) {
    return false;
  }

  /**
   * @brief 加载任务特定配置
   */
  return Task::LoadConfig<PathDeciderConfig>(&config_);
}

/**
 * @brief 执行路径决策
 *
 * @param frame 当前规划帧
 * @param reference_line_info 参考线信息
 * @return Status 执行状态
 *
 * 执行流程：
 * 1. 调用基类Task::Execute
 * 2. 调用Process进行实际决策
 *
 * C++语法说明：
 * - Task::Execute(): 调用基类的执行方法
 * - 返回值使用apollo::common::Status
 */
Status PathDecider::Execute(Frame *frame,
                            ReferenceLineInfo *reference_line_info) {
  /**
   * @brief 调用基类执行
   */
  Task::Execute(frame, reference_line_info);

  /**
   * @brief 调用Process进行路径决策
   * 参数：
   * - reference_line_info: 参考线信息
   * - reference_line_info->path_data(): 当前路径数据
   * - reference_line_info->path_decision(): 路径决策
   */
  return Process(reference_line_info, reference_line_info->path_data(),
                 reference_line_info->path_decision());
}

/**
 * @brief 处理路径决策
 *
 * @param reference_line_info 参考线信息
 * @param path_data 路径数据
 * @param path_decision 路径决策（输出）
 * @return Status 执行状态
 *
 * 主要流程：
 * 1. 检查路径是否可复用
 * 2. 打印调试信息
 * 3. 更新阻塞障碍物状态
 * 4. 生成障碍物决策
 *
 * C++语法说明：
 * - const ReferenceLineInfo*: 常量指针，输入参数
 * - const PathData&: 常量引用，输入参数
 * - PathDecision*const: 指向常量的指针，输出参数
 */
Status PathDecider::Process(const ReferenceLineInfo *reference_line_info,
                            const PathData &path_data,
                            PathDecision *const path_decision) {
  /**
   * @brief 检查路径是否可复用
   * FLAGS_enable_skip_path_tasks: 是否跳过路径任务的标志
   * path_reusable(): 判断路径是否可以复用
   * 如果路径可复用，直接返回OK跳过决策
   */
  if (FLAGS_enable_skip_path_tasks && reference_line_info->path_reusable()) {
    return Status::OK();
  }

  /**
   * @brief 创建调试曲线打印对象
   */
  PrintCurves debug_info;

  /**
   * @brief 获取离散路径
   */
  const auto &path = path_data.discretized_path();

  /**
   * @brief 如果路径非空，添加起始点边界框
   */
  if (!path.empty()) {
    /**
     * @brief 获取车辆边界框
     * VehicleConfigHelper::Instance()->GetBoundingBox():
     *   根据路径点获取车辆的边界框
     */
    const auto &vehicle_box =
        common::VehicleConfigHelper::Instance()->GetBoundingBox(path[0]);

    /**
     * @brief 添加调试信息
     */
    debug_info.AddPoint("start_point_box", vehicle_box.GetAllCorners());
  }

  /**
   * @brief 添加路径点调试信息
   */
  for (const auto &path_pt : path) {
    debug_info.AddPoint("output_path", path_pt.x(), path_pt.y());
  }

  /**
   * @brief 打印调试信息到日志
   */
  debug_info.PrintToLog();

  /**
   * @brief 阻塞障碍物ID
   */
  std::string blocking_obstacle_id;

  /**
   * @brief 获取路径决策状态
   * injector_->planning_context(): 获取规划上下文
   * mutable_planning_status(): 获取可修改的规划状态
   * mutable_path_decider(): 获取路径决策状态
   */
  auto *mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();

  /**
   * @brief 如果存在阻塞障碍物
   * GetBlockingObstacle(): 获取阻塞障碍物指针
   */
  if (reference_line_info->GetBlockingObstacle() != nullptr) {
    /**
     * @brief 获取阻塞障碍物ID
     */
    blocking_obstacle_id = reference_line_info->GetBlockingObstacle()->Id();

    /**
     * @brief 获取当前计数器值
     * front_static_obstacle_cycle_counter:
     *   前方静止障碍物存在的帧数计数器
     */
    int front_static_obstacle_cycle_counter =
        mutable_path_decider_status->front_static_obstacle_cycle_counter();

    /**
     * @brief 确保计数器非负
     * std::max(value, 0): 取较大值
     */
    mutable_path_decider_status->set_front_static_obstacle_cycle_counter(
        std::max(front_static_obstacle_cycle_counter, 0));

    /**
     * @brief 增加计数器
     * std::min(value + 1, 10): 最大值为10
     */
    mutable_path_decider_status->set_front_static_obstacle_cycle_counter(
        std::min(front_static_obstacle_cycle_counter + 1, 10));

    /**
     * @brief 更新阻塞障碍物ID
     */
    mutable_path_decider_status->set_front_static_obstacle_id(
        reference_line_info->GetBlockingObstacle()->Id());
  } else {
    /**
     * @brief 没有阻塞障碍物，减少计数器
     */
    int front_static_obstacle_cycle_counter =
        mutable_path_decider_status->front_static_obstacle_cycle_counter();

    /**
     * @brief 确保计数器不超过0
     */
    mutable_path_decider_status->set_front_static_obstacle_cycle_counter(
        std::min(front_static_obstacle_cycle_counter, 0));

    /**
     * @brief 减少计数器
     * std::max(value - 1, -10): 最小值为-10
     */
    mutable_path_decider_status->set_front_static_obstacle_cycle_counter(
        std::max(front_static_obstacle_cycle_counter - 1, -10));

    /**
     * @brief 如果计数器小于-2，清除阻塞障碍物ID
     * 说明障碍物已经不在前方足够长时间
     */
    if (mutable_path_decider_status->front_static_obstacle_cycle_counter() <
        -2) {
      std::string id = " ";  /**< 设置为空格 */
      mutable_path_decider_status->set_front_static_obstacle_id(id);
    }
  }

  /**
   * @brief 生成障碍物决策
   */
  if (!MakeObjectDecision(path_data, blocking_obstacle_id, path_decision)) {
    const std::string msg = "Failed to make decision based on tunnel";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  return Status::OK();
}

/**
 * @brief 生成障碍物决策
 *
 * @param path_data 路径数据
 * @param blocking_obstacle_id 阻塞障碍物ID
 * @param path_decision 路径决策（输出）
 * @return bool 是否成功
 *
 * 决策流程：
 * 1. 生成静态障碍物决策
 * 2. 如果启用，忽略后方障碍物
 */
bool PathDecider::MakeObjectDecision(const PathData &path_data,
                                     const std::string &blocking_obstacle_id,
                                     PathDecision *const path_decision) {
  /**
   * @brief 生成静态障碍物决策
   */
  if (!MakeStaticObstacleDecision(path_data, blocking_obstacle_id,
                                  path_decision)) {
    AERROR << "Failed to make decisions for static obstacles";
    return false;
  }

  /**
   * @brief 忽略后方障碍物
   * config_.ignore_backward_obstacle():
   *   是否忽略后方障碍物的配置
   */
  if (config_.ignore_backward_obstacle()) {
    IgnoreBackwardObstacle(path_decision);
  }

  return true;
}

/**
 * @brief 生成静态障碍物决策
 *
 * @param path_data 路径数据
 * @param blocking_obstacle_id 阻塞障碍物ID
 * @param path_decision 路径决策（输出）
 * @return bool 是否成功
 *
 * 决策逻辑：
 * 1. IGNORE: 障碍物不在路径S范围内
 * 2. IGNORE: 障碍物横向距离足够远
 * 3. STOP: 障碍物与自车横向重叠且无法绕行
 * 4. NUDGE: 障碍物靠近但可以绕行
 *    - LEFT_NUDGE: 向左绕行
 *    - RIGHT_NUDGE: 向右绕行
 *
 * C++语法说明：
 * - ACHECK: Apollo断言，仅在DEBUG模式生效
 * - mutable_xxx(): protobuf消息的可修改访问器
 */
bool PathDecider::MakeStaticObstacleDecision(
    const PathData &path_data, const std::string &blocking_obstacle_id,
    PathDecision *const path_decision) {
  /**
   * @brief 断言检查
   */
  ACHECK(path_decision);

  /**
   * @brief 获取Frenet路径
   */
  const auto &frenet_path = path_data.frenet_frame_path();

  /**
   * @brief 检查路径是否为空
   */
  if (frenet_path.empty()) {
    AERROR << "Path is empty.";
    return false;
  }

  /**
   * @brief 获取车辆参数
   * half_width: 车辆半宽
   */
  const double half_width =
      common::VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;

  /**
   * @brief 横向忽略缓冲区
   * lateral_radius: 用于判断是否忽略障碍物的横向距离阈值
   */
  const double lateral_radius = half_width + FLAGS_lateral_ignore_buffer;  // 3.0

  /**
   * @brief 遍历所有障碍物
   */
  for (const auto *obstacle : path_decision->obstacles().Items()) {
    /**
     * @brief 获取障碍物ID和类型名称
     */
    const std::string &obstacle_id = obstacle->Id();
    const std::string obstacle_type_name =
        PerceptionObstacle_Type_Name(obstacle->Perception().type());

    ADEBUG << "obstacle_id[<< " << obstacle_id << "] type["
           << obstacle_type_name << "]";

    /**
     * @brief 跳过非静态障碍物和虚拟障碍物
     * 静态障碍物是需要决策的主要目标
     * 虚拟障碍物用于测试，不参与决策
     */
    if (!obstacle->IsStatic() || obstacle->IsVirtual()) {
      continue;
    }

    /**
     * @brief 跳过已有IGNORE决策的障碍物
     * HasLongitudinalDecision(): 是否有纵向决策
     * LongitudinalDecision().has_ignore(): 纵向决策是否是IGNORE
     */
    if (obstacle->HasLongitudinalDecision() &&
        obstacle->LongitudinalDecision().has_ignore() &&
        obstacle->HasLateralDecision() &&
        obstacle->LateralDecision().has_ignore()) {
      continue;
    }

    /**
     * @brief 跳过已有STOP决策的障碍物
     */
    if (obstacle->HasLongitudinalDecision() &&
        obstacle->LongitudinalDecision().has_stop()) {
      continue;
    }

    /**
     * @brief 如果是阻塞障碍物且不在借道场景，添加STOP决策
     *
     * 阻塞障碍物：
     * - 导致路径规划失败的障碍物
     * - 需要自车完全停车等待
     */
    if (obstacle->Id() == blocking_obstacle_id &&
        !injector_->planning_context()
             ->planning_status()
             .path_decider()
             .is_in_path_lane_borrow_scenario()) {
      ADEBUG << "Blocking obstacle = " << blocking_obstacle_id;

      /**
       * @brief 创建停车决策
       * ObjectDecisionType: 障碍物决策类型
       * mutable_stop(): 获取可修改的stop决策
       */
      ObjectDecisionType object_decision;
      *object_decision.mutable_stop() = GenerateObjectStopDecision(*obstacle);

      /**
       * @brief 添加纵向决策
       * AddLongitudinalDecision(tag, obstacle_id, decision):
       *   添加指定障碍物的纵向决策
       */
      path_decision->AddLongitudinalDecision("PathDecider/blocking_obstacle",
                                             obstacle->Id(), object_decision);
      continue;
    }

    /**
     * @brief 跳过clear-zone障碍物
     * clear-zone是特殊区域，不需要决策
     */
    if (obstacle->reference_line_st_boundary().boundary_type() ==
        STBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }

    /**
     * @brief 默认决策为IGNORE
     * mutable_ignore(): 获取可修改的ignore决策
     */
    ObjectDecisionType object_decision;
    object_decision.mutable_ignore();

    /**
     * @brief 获取障碍物的SL边界
     */
    const auto &sl_boundary = obstacle->PerceptionSLBoundary();

    /**
     * @brief 检查障碍物是否在路径S范围内
     * frenet_path.front().s(): 路径起始点S坐标
     * frenet_path.back().s(): 路径终点S坐标
     *
     * 条件1：sl_boundary.end_s() < frenet_path.front().s()
     *   障碍物在路径起点之前
     * 条件2：sl_boundary.start_s() > frenet_path.back().s()
     *   障碍物在路径终点之后
     */
    if (sl_boundary.end_s() < frenet_path.front().s() ||
        sl_boundary.start_s() > frenet_path.back().s()) {
      /**
       * @brief 添加IGNORE决策
       */
      path_decision->AddLongitudinalDecision("PathDecider/not-in-s",
                                             obstacle->Id(), object_decision);
      path_decision->AddLateralDecision("PathDecider/not-in-s", obstacle->Id(),
                                        object_decision);
      continue;
    }

    /**
     * @brief 获取路径上最近的Frenet点
     * GetNearestPoint(): 获取距离障碍物最近的路径点
     */
    const auto frenet_point = frenet_path.GetNearestPoint(sl_boundary);

    /**
     * @brief 获取最近点的L坐标
     */
    const double curr_l = frenet_point.l();

    /**
     * @brief 计算最小绕行距离
     * min_nudge_l: 车辆半宽 + 静态障碍物缓冲区/2
     */
    double min_nudge_l = half_width + config_.static_obstacle_buffer() / 2.0;   // 0.3

    /**
     * @brief 检查障碍物横向距离
     *
     * curr_l: 自车当前位置的L值
     * lateral_radius: 忽略缓冲区
     *
     * 条件1：curr_l - lateral_radius > sl_boundary.end_l()
     *   障碍物完全在自车左侧
     * 条件2：curr_l + lateral_radius < sl_boundary.start_l()
     *   障碍物完全在自车右侧
     */
    if (curr_l - lateral_radius > sl_boundary.end_l() ||
        curr_l + lateral_radius < sl_boundary.start_l()) {
      /**
       * @brief 横向距离足够远，IGNORE
       */
      path_decision->AddLateralDecision("PathDecider/not-in-l", obstacle->Id(),
                                        object_decision);
    }
    /**
     * @brief 检查是否需要STOP
     *
     * sl_boundary.end_l() >= curr_l - min_nudge_l:
     *   障碍物下边界不低于自车绕行线
     * sl_boundary.start_l() <= curr_l + min_nudge_l:
     *   障碍物上边界不超过自车绕行线
     */
    else if (sl_boundary.end_l() >= curr_l - min_nudge_l &&
               sl_boundary.start_l() <= curr_l + min_nudge_l) {
      /**
       * @brief 如果启用跳过重叠停车检查
       */
      if (config_.skip_overlap_stop_check()) {
        AINFO << "skip_overlap_stop_check";
      } else {
        /**
         * @brief 横向重叠，需要STOP
         * GenerateObjectStopDecision():
         *   生成停车决策，包含停车点和距离
         */
        *object_decision.mutable_stop() = GenerateObjectStopDecision(*obstacle);

        /**
         * @brief 合并主停车决策
         * MergeWithMainStop():
         *   如果这是更近的停车点，更新主停车决策
         */
        if (path_decision->MergeWithMainStop(
                object_decision.stop(), obstacle->Id(),
                reference_line_info_->reference_line(),
                reference_line_info_->AdcSlBoundary())) {
          path_decision->AddLongitudinalDecision(
              "PathDecider/nearest-stop", obstacle->Id(), object_decision);
        } else {
          /**
           * @brief 不是最近的停车点，IGNORE
           */
          ObjectDecisionType object_decision;
          object_decision.mutable_ignore();
          path_decision->AddLongitudinalDecision(
              "PathDecider/not-nearest-stop", obstacle->Id(), object_decision);
        }
        AINFO << "Add stop decision for static obs " << obstacle->Id()
              << "start l" << sl_boundary.start_l() << "end l"
              << sl_boundary.end_l() << "curr_l" << curr_l << "min_nudge_l"
              << min_nudge_l;
      }
    }
    /**
     * @brief 需要NUDGE绕行
     */
    else {
      /**
       * @brief 判断绕行方向
       *
       * LEFT_NUDGE条件：
       * sl_boundary.end_l() < curr_l - min_nudge_l
       *   障碍物下边界低于绕行线，需要向左绕
       */
      if (sl_boundary.end_l() < curr_l - min_nudge_l) {
        /**
         * @brief 创建向左绕行决策
         * ObjectNudge: 绕行决策
         * set_type(ObjectNudge::LEFT_NUDGE): 设置为向左绕行
         * set_distance_l(): 设置绕行距离
         */
        ObjectNudge *object_nudge_ptr = object_decision.mutable_nudge();
        object_nudge_ptr->set_type(ObjectNudge::LEFT_NUDGE);
        object_nudge_ptr->set_distance_l(config_.static_obstacle_buffer());
        path_decision->AddLateralDecision("PathDecider/left-nudge",
                                          obstacle->Id(), object_decision);
      }
      /**
       * @brief RIGHT_NUDGE条件
       * sl_boundary.start_l() > curr_l + min_nudge_l
       *   障碍物上边界高于绕行线，需要向右绕
       */
      else if (sl_boundary.start_l() > curr_l + min_nudge_l) {
        /**
         * @brief 创建向右绕行决策
         * distance_l为负值表示向右
         */
        ObjectNudge *object_nudge_ptr = object_decision.mutable_nudge();
        object_nudge_ptr->set_type(ObjectNudge::RIGHT_NUDGE);
        object_nudge_ptr->set_distance_l(-config_.static_obstacle_buffer());
        path_decision->AddLateralDecision("PathDecider/right-nudge",
                                          obstacle->Id(), object_decision);
      }
    }
  }

  return true;
}

/**
 * @brief 生成停车决策
 *
 * @param obstacle 障碍物
 * @return ObjectStop 停车决策
 *
 * 停车决策包含：
 * - 停车原因代码
 * - 停车距离
 * - 停车点位置
 * - 停车航向角
 *
 * C++语法说明：
 * - const Obstacle&: 常量引用，输入参数
 */
ObjectStop PathDecider::GenerateObjectStopDecision(
    const Obstacle &obstacle) const {
  ObjectStop object_stop;

  /**
   * @brief 计算最小停车距离
   * MinRadiusStopDistance():
   *   根据障碍物计算最小安全停车距离
   * 考虑车辆动力学限制
   */
  double stop_distance = obstacle.MinRadiusStopDistance(
      VehicleConfigHelper::GetConfig().vehicle_param());

  /**
   * @brief 设置停车原因
   * StopReasonCode::STOP_REASON_OBSTACLE: 障碍物导致的停车
   */
  object_stop.set_reason_code(StopReasonCode::STOP_REASON_OBSTACLE);

  /**
   * @brief 设置停车距离
   * 负值表示在障碍物后方
   */
  object_stop.set_distance_s(-stop_distance);

  /**
   * @brief 计算停车点的参考线S坐标
   * stop_ref_s = 障碍物起点S - 停车距离  
   * // 最后计算出来的是后轴应该停的位置
   */
  const double stop_ref_s =
      obstacle.PerceptionSLBoundary().start_s() - stop_distance;

  /**
   * @brief 获取停车点的参考线信息
   */
  const auto stop_ref_point =
      reference_line_info_->reference_line().GetReferencePoint(stop_ref_s);

  /**
   * @brief 设置停车点坐标
   * mutable_stop_point(): 获取可修改的停车点
   */
  object_stop.mutable_stop_point()->set_x(stop_ref_point.x());
  object_stop.mutable_stop_point()->set_y(stop_ref_point.y());

  /**
   * @brief 设置停车航向角
   */
  object_stop.set_stop_heading(stop_ref_point.heading());

  return object_stop;
}

/**
 * @brief 忽略后方障碍物
 *
 * @param path_decision 路径决策（输出）
 * @return bool 是否成功
 *
 * 功能：
 * - 对于已经完全在自车后方的动态障碍物，添加IGNORE决策
 * - 后方障碍物不需要考虑，因为自车不会后退
 *
 * C++语法说明：
 * - Obstacle::PerceptionSLBoundary():
 *   使用::明确指定作用于Obstacle类
 */
bool PathDecider::IgnoreBackwardObstacle(PathDecision *const path_decision) {
  /**
   * @brief 获取自车起点S坐标
   */
  double adc_start_s = reference_line_info_->AdcSlBoundary().start_s();

  /**
   * @brief 遍历所有障碍物
   */
  for (const auto *obstacle : path_decision->obstacles().Items()) {
    /**
     * @brief 跳过静态障碍物和虚拟障碍物
     */
    if (obstacle->IsStatic() || obstacle->IsVirtual()) {
      continue;
    }

    /**
     * @brief 检查障碍物是否在自车后方
     * Obstacle::PerceptionSLBoundary():
     *   明确使用::指定作用域，避免歧义
     * 条件：障碍物终点S < 自车起点S
     */
    if (obstacle->Obstacle::PerceptionSLBoundary().end_s() < adc_start_s) {
      /**
       * @brief 添加IGNORE决策
       */
      ObjectDecisionType object_decision;
      object_decision.mutable_ignore();
      path_decision->AddLongitudinalDecision(
          "PathDecider/ignore-backward-obstacle", obstacle->Id(),
          object_decision);
    }
  }
  return true;
}

/**
 * @brief 命名空间结束标记
 */
}  // namespace planning
}  // namespace apollo
