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
 * See the the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file rule_based_stop_decider.cc
 * @brief 基于规则的停车决策器实现文件
 *
 * 本文件实现了RuleBasedStopDecider类，基于规则而非优化来生成停车决策。
 *
 * 核心功能：
 * 1. 借道绕行时的停车决策
 * 2. 紧急换道检查
 * 3. 路径终点停车决策
 *
 * 与优化类决策器的区别：
 * - 优化类：根据代价函数优化得到停车点
 * - 规则类：根据预定义规则直接决策
 *
 * C++语法说明：
 * - std::shared_ptr<T>: 智能指针，引用计数
 * - std::tuple: 元组，存储异构数据
 * - std::string: 字符串类型
 * - static变量: 函数内静态变量，跨调用保持状态
 */

#include "modules/planning/tasks/rule_based_stop_decider/rule_based_stop_decider.h"

/**
 * @brief 标准库头文件
 * <string>: 提供std::string字符串类型
 * <tuple>: 提供std::tuple元组
 * <vector>: 提供std::vector动态数组
 */
#include <string>
#include <tuple>
#include <vector>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
/**
 * @brief PNC点消息protobuf定义
 * 包含SLPoint, PathPoint等数据结构
 */

#include "modules/common/vehicle_state/vehicle_state_provider.h"
/**
 * @brief 车辆状态提供者
 * 获取车辆当前状态（速度、位置等）
 */
#include "modules/planning/planning_base/common/planning_context.h"
/**
 * @brief 规划上下文
 * 存储规划过程中的共享状态
 */
#include "modules/planning/planning_base/common/util/common.h"
/**
 * @brief 通用工具函数
 * 包含BuildStopDecision等
 */
#include "modules/planning/planning_base/gflags/planning_gflags.h"
/**
 * @brief Planning模块GFlags配置
 */
#include "modules/planning/planning_interface_base/task_base/common/lane_change_util/lane_change_util.h"
/**
 * @brief 换道工具函数
 * 包含IsClearToChangeLane等
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
using apollo::common::SLPoint;
using apollo::common::Status;
using apollo::common::math::Vec2d;

/**
 * @brief 匿名命名空间
 * 定义文件范围内的常量
 */
namespace {
/**
 * @brief 直行代价常量
 * 用于判断是否应该执行换道
 * 如果换道代价小于此值，说明换道成功
 */
constexpr double kStraightForwardLineCost = 10.0;
}  // namespace

/**
 * @brief 初始化基于规则的停车决策器
 *
 * @param config_dir 配置文件目录
 * @param name 任务名称
 * @param injector 依赖注入器
 * @return bool 初始化是否成功
 *
 * 初始化流程：
 * 1. 调用基类Decider::Init进行基础初始化
 * 2. 加载RuleBasedStopDeciderConfig配置
 */
bool RuleBasedStopDecider::Init(
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
  return Decider::LoadConfig<RuleBasedStopDeciderConfig>(&config_);
}

/**
 * @brief 执行基于规则的停车决策
 *
 * @param frame 当前规划帧
 * @param reference_line_info 参考线信息
 * @return Status 执行状态
 *
 * 主要流程：
 * 1. 借道绕行时的停车决策
 * 2. 紧急换道检查
 * 3. 路径终点停车决策
 *
 * C++语法说明：
 * - Frame *const frame: 指向常量的指针
 *   第一个const: 指针指向的内容不可修改
 *   第二个const: 指针本身不可修改
 */
apollo::common::Status RuleBasedStopDecider::Process(
    Frame *const frame, ReferenceLineInfo *const reference_line_info) {
  /**
   * @brief 1. 借道逆向车道时的停车决策
   * config_.enable_stop_on_side_pass():
   *   配置开关，是否启用此功能
   */
  if (config_.enable_stop_on_side_pass()) {
    StopOnSidePass(frame, reference_line_info);
  }

  /**
   * @brief 2. 紧急换道检查
   * config_.enable_lane_change_urgency_checking():
   *   配置开关，是否启用紧急换道检查
   */
  if (config_.enable_lane_change_urgency_checking()) {
    CheckLaneChangeUrgency(frame);
  }

  /**
   * @brief 3. 路径终点停车决策
   * 无论是否启用配置，都会执行
   */
  AddPathEndStop(frame, reference_line_info);

  return Status::OK();
}

/**
 * @brief 检查换道紧急程度
 *
 * @param frame 当前规划帧
 *
 * 功能说明：
 * - 检查目标车道是否被阻塞
 * - 如果是换道路径，检查是否满足换道条件
 * - 如果目标车道阻塞且无法换道，设置紧急停车点等待
 *
 * C++语法说明：
 * - for (auto &reference_line_info : *frame->mutable_reference_line_info()):
 *   范围for循环遍历可修改的参考线信息
 *   mutable_reference_line_info()返回可修改的引用
 */
void RuleBasedStopDecider::CheckLaneChangeUrgency(Frame *const frame) {
  /**
   * @brief 遍历所有参考线
   */
  for (auto &reference_line_info : *frame->mutable_reference_line_info()) {
    /**
     * @brief 检查是否是换道路径
     */
    if (reference_line_info.IsChangeLanePath()) {
      /**
       * @brief 检查是否可以换道
       */
      is_clear_to_change_lane_ = IsClearToChangeLane(&reference_line_info);

      /**
       * @brief 检查换道规划是否成功
       * Cost() < kStraightForwardLineCost: 代价小于直行代价
       */
      is_change_lane_planning_succeed_ =
          reference_line_info.Cost() < kStraightForwardLineCost;
      continue;  /**< 跳过当前循环，继续下一个 */
    }

    /**
     * @brief 如果不是换道场景，或目标车道不阻塞且换道成功，跳过
     */
    if (frame->reference_line_info().size() <= 1 ||
        (is_clear_to_change_lane_ && is_change_lane_planning_succeed_)) {
      continue;
    }

    /**
     * @brief 获取路由终点
     */
    const auto &route_end_waypoint =
        reference_line_info.Lanes().RouteEndWaypoint();

    /**
     * @brief 检查是否能获取到车道信息
     */
    if (!route_end_waypoint.lane) {
      continue;
    }

    /**
     * @brief 获取路由终点的平滑坐标
     */
    auto point = route_end_waypoint.lane->GetSmoothPoint(route_end_waypoint.s);

    /**
     * @brief 获取可修改的参考线指针
     */
    auto *reference_line = reference_line_info.mutable_reference_line();

    common::SLPoint sl_point;

    /**
     * @brief 将路由终点投影到当前参考线的SL坐标
     */
    if (reference_line->XYToSL(point, &sl_point) &&
        reference_line->IsOnLane(sl_point)) {
      /**
       * @brief 计算到通道终点的距离
       */
      double distance_to_passage_end =
          sl_point.s() - reference_line_info.AdcSlBoundary().end_s();

      /**
       * @brief 如果自车离路由终点还很远，不需要停车
       */
      if (distance_to_passage_end >
          config_.approach_distance_for_lane_change()) {
        continue;
      }

      /**
       * @brief 紧急情况下，设置临时停车点等待换道
       * TODO(Jiaxuan Xu): 用更智能的动作替代停车点
       */
      const std::string stop_wall_id = "lane_change_stop";
      std::vector<std::string> wait_for_obstacles;

      /**
       * @brief 构建停车决策
       * util::BuildStopDecision():
       *   创建停车墙和停车决策
       */
      util::BuildStopDecision(
          stop_wall_id, sl_point.s(), config_.urgent_distance_for_lane_change(),
          StopReasonCode::STOP_REASON_LANE_CHANGE_URGENCY, wait_for_obstacles,
          "RuleBasedStopDecider", frame, &reference_line_info);
    }
  }
}

/**
 * @brief 添加路径终点停车决策
 *
 * @param frame 当前规划帧
 * @param reference_line_info 参考线信息
 *
 * 功能说明：
 * - 如果路径太短，设置终点停车点
 * - 用于确保车辆在路径结束时能安全停止
 *
 * C++语法说明：
 * - path_data.frenet_frame_path().back().s():
 *   frenet帧路径最后一个点的s坐标
 * - path_data.frenet_frame_path().front().s():
 *   frenet帧路径第一个点的s坐标
 * - PATH_END_VO_ID_PREFIX: 路径终点虚拟障碍物ID前缀
 */
void RuleBasedStopDecider::AddPathEndStop(
    Frame *const frame, ReferenceLineInfo *const reference_line_info) {
  /**
   * @brief 检查路径标签是否非空
   * 并且路径长度小于阈值
   */
  if (!reference_line_info->path_data().path_label().empty() &&
      reference_line_info->path_data().frenet_frame_path().back().s() -
              reference_line_info->path_data().frenet_frame_path().front().s() <
          config_.short_path_length_threshold()) {
    /**
     * @brief 创建停车墙ID
     * 格式: PATH_END_VO_ID_PREFIX + path_label
     */
    const std::string stop_wall_id =
        PATH_END_VO_ID_PREFIX + reference_line_info->path_data().path_label();
    std::vector<std::string> wait_for_obstacles;

    /**
     * @brief 构建停车决策
     * 停车点设置在路径终点前0.1米
     */
    util::BuildStopDecision(
        stop_wall_id,
        reference_line_info->path_data().frenet_frame_path().back().s() - 0.1,
        0.0, StopReasonCode::STOP_REASON_REFERENCE_END, wait_for_obstacles,
        "RuleBasedStopDecider", frame, reference_line_info);
  }
}

/**
 * @brief 借道绕行时的停车处理
 *
 * @param frame 当前规划帧
 * @param reference_line_info 参考线信息
 *
 * 状态机逻辑：
 * 1. 如果是自车道行驶，重置检查状态
 * 2. 检查是否可以完成借道
 * 3. 检查是否需要停车等待
 * 4. 决定是否应该停车
 *
 * C++语法说明：
 * - static bool check_clear:
 *   静态变量，跨函数调用保持状态
 *   用于记忆之前的检查结果
 * - static common::PathPoint change_lane_stop_path_point:
 *   静态变量，存储换道停车点
 */
void RuleBasedStopDecider::StopOnSidePass(
    Frame *const frame, ReferenceLineInfo *const reference_line_info) {
  static bool check_clear;  /**< 是否检查完成标志 */
  static common::PathPoint change_lane_stop_path_point;  /**< 换道停车点 */

  const PathData &path_data = reference_line_info->path_data();
  double stop_s_on_pathdata = 0.0;

  /**
   * @brief 如果是自车道行驶
   * 重置检查状态和停车点
   */
  if (path_data.path_label().find("self") != std::string::npos) {
    check_clear = false;
    change_lane_stop_path_point.Clear();
    return;
  }

  /**
   * @brief 如果已经检查完成且通过，检查是否可以开始行驶
   */
  if (check_clear &&
      CheckClearDone(*reference_line_info, change_lane_stop_path_point)) {
    check_clear = false;
  }

  /**
   * @brief 如果尚未检查
   */
  if (!check_clear &&
      CheckSidePassStop(path_data, *reference_line_info, &stop_s_on_pathdata)) {
    /**
     * @brief 检查视野是否被遮挡
     * IsPerceptionBlocked(): 检查感知范围内是否有障碍物
     */
    if (!IsPerceptionBlocked(*reference_line_info, config_.search_beam_length(),
                             config_.search_beam_radius_intensity(),
                             config_.search_range(),
                             config_.is_block_angle_threshold()) &&
        IsClearToChangeLane(reference_line_info)) {
      return;  /**< 可以继续行驶 */
    }

    /**
     * @brief 检查自车是否已停车
     */
    if (!CheckADCStop(path_data, *reference_line_info, stop_s_on_pathdata)) {
      /**
       * @brief 未停车，构建停车点
       */
      if (!BuildSidePassStopFence(path_data, stop_s_on_pathdata,
                                  &change_lane_stop_path_point, frame,
                                  reference_line_info)) {
        AERROR << "Set side pass stop fail";
      }
    } else {
      /**
       * @brief 已停车，检查是否可以开始行驶
       */
      if (IsClearToChangeLane(reference_line_info)) {
        check_clear = true;
      }
    }
  }
}

/**
 * @brief 检查是否需要设置借道停车点
 *
 * @param path_data 路径数据
 * @param reference_line_info 参考线信息
 * @param stop_s_on_pathdata 输出：停车点的s坐标
 * @return bool 是否需要停车
 *
 * 功能说明：
 * - 检测从自车道到借道车道的转换点
 * - 在转换点前设置停车点等待安全时机
 *
 * C++语法说明：
 * - std::tuple<double, PathData::PathData::PathPointType, double>:
 *   元组类型，包含(s坐标, 路径点类型, 某个值)
 * - std::get<1>(point_guide):
 *   获取元组第2个元素（索引从0开始）
 */
bool RuleBasedStopDecider::CheckSidePassStop(
    const PathData &path_data, const ReferenceLineInfo &reference_line_info,
    double *stop_s_on_pathdata) {
  /**
   * @brief 获取路径点决策指南
   * path_point_decision_guide:
   *   存储(path_s, point_type, other_value)的元组列表
   */
  const std::vector<std::tuple<double, PathData::PathPointType, double>>
      &path_point_decision_guide = path_data.path_point_decision_guide();

  PathData::PathPointType last_path_point_type =
      PathData::PathPointType::UNKNOWN;

  /**
   * @brief 遍历路径点决策指南
   */
  for (const auto &point_guide : path_point_decision_guide) {
    /**
     * @brief 检测从IN_LANE到OUT_ON_REVERSE_LANE的转换
     * 即从自车道进入逆向借道车道
     */
    if (last_path_point_type == PathData::PathPointType::IN_LANE &&
        std::get<1>(point_guide) ==
            PathData::PathPointType::OUT_ON_REVERSE_LANE) {
      /**
       * @brief 获取转换点的s坐标作为停车点
       */
      *stop_s_on_pathdata = std::get<0>(point_guide);

      /**
       * @brief 根据车辆位置近似停车点s
       */
      const auto &vehicle_config =
          common::VehicleConfigHelper::Instance()->GetConfig();
      const double ego_front_to_center =
          vehicle_config.vehicle_param().front_edge_to_center();

      common::PathPoint stop_pathpoint;

      /**
       * @brief 获取指定s坐标处的路径点
       */
      if (!path_data.GetPathPointWithRefS(*stop_s_on_pathdata,
                                          &stop_pathpoint)) {
        AERROR << "Can't get stop point on path data";
        return false;
      }

      /**
       * @brief 计算停车点坐标
       * 将路径点沿前进方向偏移前轴到中心的距离
       */
      const double ego_theta = stop_pathpoint.theta();
      Vec2d shift_vec{ego_front_to_center * std::cos(ego_theta),
                      ego_front_to_center * std::sin(ego_theta)};
      const Vec2d stop_fence_pose =
          shift_vec + Vec2d(stop_pathpoint.x(), stop_pathpoint.y());

      double stop_l_on_pathdata = 0.0;

      /**
       * @brief 获取最近点并转换到SL坐标
       */
      const auto &nearby_path = reference_line_info.reference_line().map_path();
      nearby_path.GetNearestPoint(stop_fence_pose, stop_s_on_pathdata,
                                  &stop_l_on_pathdata);
      return true;
    }
    last_path_point_type = std::get<1>(point_guide);
  }
  return false;
}

/**
 * @brief 构建借道停车点
 *
 * @param path_data 路径数据
 * @param stop_s_on_pathdata 停车点的s坐标
 * @param stop_point 输出：停车点
 * @param frame 当前规划帧
 * @param reference_line_info 参考线信息
 * @return bool 是否成功
 *
 * C++语法说明：
 * - CHECK_NOTNULL(pointer):
 *   Apollo断言宏，检查指针是否为空
 *   仅在DEBUG模式生效
 */
bool RuleBasedStopDecider::BuildSidePassStopFence(
    const PathData &path_data, const double stop_s_on_pathdata,
    common::PathPoint *stop_point, Frame *const frame,
    ReferenceLineInfo *const reference_line_info) {
  CHECK_NOTNULL(frame);  /**< 断言frame非空 */
  CHECK_NOTNULL(reference_line_info);  /**< 断言reference_line_info非空 */

  /**
   * @brief 获取停车点路径坐标
   */
  if (!path_data.GetPathPointWithRefS(stop_s_on_pathdata, stop_point)) {
    AERROR << "Can't get stop point on path data";
    return false;
  }

  const std::string stop_wall_id = "Side_Pass_Stop";
  std::vector<std::string> wait_for_obstacles;

  /**
   * @brief 获取参考线上的SL坐标
   */
  const auto &nearby_path = reference_line_info->reference_line().map_path();
  double stop_point_s = 0.0;
  double stop_point_l = 0.0;
  nearby_path.GetNearestPoint({stop_point->x(), stop_point->y()}, &stop_point_s,
                              &stop_point_l);

  /**
   * @brief 构建停车决策
   */
  util::BuildStopDecision(stop_wall_id, stop_point_s, 0.0,
                          StopReasonCode::STOP_REASON_SIDEPASS_SAFETY,
                          wait_for_obstacles, "RuleBasedStopDecider", frame,
                          reference_line_info);
  return true;
}

/**
 * @brief 检查自车是否停在停车点
 *
 * @param path_data 路径数据
 * @param reference_line_info 参考线信息
 * @param stop_s_on_pathdata 停车点的s坐标
 * @return bool 自车是否已停在停车点
 *
 * 检查条件：
 * 1. 车辆速度小于阈值
 * 2. 停车点距离自车前边缘足够近
 *
 * C++语法说明：
 * - injector_->vehicle_state():
 *   箭头运算符链式调用
 *   injector_是shared_ptr，->获取内部对象
 */
bool RuleBasedStopDecider::CheckADCStop(
    const PathData &path_data, const ReferenceLineInfo &reference_line_info,
    const double stop_s_on_pathdata) {
  common::PathPoint stop_point;

  /**
   * @brief 获取停车点
   */
  if (!path_data.GetPathPointWithRefS(stop_s_on_pathdata, &stop_point)) {
    AERROR << "Can't get stop point on path data";
    return false;
  }

  /**
   * @brief 获取自车速度
   * injector_->xxx: 访问智能指针指向的对象成员
   */
  const double adc_speed = injector_->vehicle_state()->linear_velocity();

  /**
   * @brief 检查速度是否足够小
   */
  if (adc_speed > config_.max_adc_stop_speed()) {
    ADEBUG << "ADC not stopped: speed[" << adc_speed << "]";
    return false;
  }

  /**
   * @brief 获取自车前边缘s坐标
   */
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();

  /**
   * @brief 获取停车点的SL坐标
   */
  const auto &nearby_path = reference_line_info.reference_line().map_path();
  double stop_point_s = 0.0;
  double stop_point_l = 0.0;
  nearby_path.GetNearestPoint({stop_point.x(), stop_point.y()}, &stop_point_s,
                              &stop_point_l);

  /**
   * @brief 计算停车点到自车前边缘的距离
   */
  const double distance_stop_line_to_adc_front_edge =
      stop_point_s - adc_front_edge_s;

  /**
   * @brief 检查距离是否有效
   */
  if (distance_stop_line_to_adc_front_edge >
      config_.max_valid_stop_distance()) {
    ADEBUG << "not a valid stop. too far from stop line.";
    return false;
  }

  return true;
}

/**
 * @brief 检查借道是否完成
 *
 * @param reference_line_info 参考线信息
 * @param stop_point 停车点
 * @return bool 借道是否完成
 *
 * 功能说明：
 * - 检查自车是否已经安全完成借道
 * - 通过判断自车是否完全在车道内来确定
 */
bool RuleBasedStopDecider::CheckClearDone(
    const ReferenceLineInfo &reference_line_info,
    const common::PathPoint &stop_point) {
  /**
   * @brief 获取自车SL边界
   */
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double adc_back_edge_s = reference_line_info.AdcSlBoundary().start_s();
  const double adc_start_l = reference_line_info.AdcSlBoundary().start_l();
  const double adc_end_l = reference_line_info.AdcSlBoundary().end_l();

  /**
   * @brief 获取车道宽度
   */
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  reference_line_info.reference_line().GetLaneWidth(
      (adc_front_edge_s + adc_back_edge_s) / 2.0, &lane_left_width,
      &lane_right_width);

  /**
   * @brief 转换停车点到SL坐标
   */
  SLPoint stop_sl_point;
  reference_line_info.reference_line().XYToSL(stop_point, &stop_sl_point);

  /**
   * @brief 使用到上一个停车点的距离判断是否需要再次检查clear
   */
  if (adc_back_edge_s > stop_sl_point.s()) {
    /**
     * @brief 自车后边缘已通过停车点
     * 检查自车是否完全在车道内
     */
    if (adc_start_l > -lane_right_width || adc_end_l < lane_left_width) {
      return true;
    }
  }
  return false;
}

/**
 * @brief 命名空间结束标记
 */
}  // namespace planning
}  // namespace apollo
