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
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/time/time.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/planner_factory.h"
#include "modules/planning/planning.h"

namespace apollo {
namespace planning {

using apollo::common::adapter::AdapterManager;
using TrajectoryPb = ADCTrajectory;

Planning::Planning() {
  ptr_planner_ = PlannerFactory::CreateInstance(PlannerType::RTK_PLANNER);
}
/// @brief 
/// @param vehicle_state 当前车辆状态的对象（包括位置、速度等）
/// @param is_on_auto_mode 车辆是否处于自动驾驶模式
/// @param publish_time 规划的发布时间，用于计算执行开始时间
/// @param planning_trajectory 存储生成的规划轨迹
/// @return 
bool Planning::Plan(const common::vehicle_state::VehicleState& vehicle_state,
                    const bool is_on_auto_mode, const double publish_time,
                    std::vector<TrajectoryPoint>* planning_trajectory) {
  double planning_cycle_time = 1.0 / FLAGS_planning_loop_rate;
  double execution_start_time = publish_time;
  // 处在自驾同时上次规划轨迹不为空
  if (is_on_auto_mode && !last_trajectory_.empty()) {
    // if the auto-driving mode is on and we have the trajectory from last
    // cycle, then
    // find the planning starting point from the last planning result.
    // this ensures the smoothness of planning output and
    // therefore the smoothness of control execution.
    // 根据上次的规划轨迹找到当前规划的起始点 matched_point 和起始点在上次轨迹中的索引 matched_index。
    // 这有助于从上次轨迹的延续点开始规划，以保持轨迹的连续性
    auto matched_info =
        ComputeStartingPointFromLastTrajectory(execution_start_time);
    // first: 当前规划起点在上一帧中的匹配点
    TrajectoryPoint matched_point = matched_info.first;
    // second：当前规划起点在上一帧中的匹配点对应的索引
    std::size_t matched_index = matched_info.second;

    // Compute the position deviation between current vehicle
    // position and target vehicle position.
    // If the deviation exceeds a specific threshold,
    // it will be unsafe to planning from the matched point.
    // 计算当前车辆位置与目标位置（即上次轨迹中的起始点）之间的偏差
    double dx = matched_point.x - vehicle_state.x();
    double dy = matched_point.y - vehicle_state.y();
    double position_deviation = std::sqrt(dx * dx + dy * dy);
    // 如果当前位置和目标位置的偏差小于一个预设的阈值（FLAGS_replanning_threshold），
    // 表示位置误差在可接受范围内，可以从上次的规划结果继续规划
    if (position_deviation < FLAGS_replanning_threshold) {  // 2
      // planned trajectory from the matched point, the matched point has
      // relative time 0.
      // 使用上次轨迹的匹配点 matched_point 作为起点进行新的规划
      // RTK规划
      bool planning_succeeded =
          ptr_planner_->Plan(matched_point, planning_trajectory);

      if (!planning_succeeded) {
        last_trajectory_.clear();
        return false;
      }

      // a segment of last trajectory to be attached to planned trajectory in
      // case controller needs.
      // 如果规划成功，获取上次轨迹的一段（GetOverheadTrajectory），并将其插入到新规划轨迹的开头。
      //这样做是为了将上次轨迹的“过渡段”与新的规划轨迹拼接起来，以便控制器在执行时平滑过渡
      auto overhead_trajectory = GetOverheadTrajectory(
          matched_index, (std::size_t)FLAGS_rtk_trajectory_backward);   // 10
      planning_trajectory->insert(planning_trajectory->begin(),
                                  overhead_trajectory.begin(),
                                  overhead_trajectory.end());

      // store the planned trajectory and header info for next planning cycle.
      // 新的规划轨迹存储到 last_trajectory_ 中，以便下一次规划时使用
      last_trajectory_ = *planning_trajectory;
      // 更新上次规划的时间戳（last_header_time_）
      last_header_time_ = execution_start_time;
      return true;
    }
  }

  // if 1. the auto-driving mode is off or
  //    2. we don't have the trajectory from last planning cycle or
  //    3. the position deviation from actual and target is too high
  // then planning from current vehicle state.
  // 如果不满足上述条件（例如自动驾驶模式关闭或位置误差过大），则从当前的车辆状态计算新的起始点（vehicle_state_point）
  // 起始点是从当前车辆状态和规划周期计算得出的
  TrajectoryPoint vehicle_state_point =
      ComputeStartingPointFromVehicleState(vehicle_state, planning_cycle_time);
  // 使用当前的车辆状态作为起始点，调用 ptr_planner_->Plan() 进行路径规划
  bool planning_succeeded =
      ptr_planner_->Plan(vehicle_state_point, planning_trajectory);
  // 如果规划失败，清空上次轨迹并返回 false 表示规划失败
  if (!planning_succeeded) {
    last_trajectory_.clear();
    return false;
  }
  // store the planned trajectory and header info for next planning cycle.
  last_trajectory_ = *planning_trajectory;
  last_header_time_ = execution_start_time;
  return true;
}
// 根据上次规划的轨迹（last_trajectory_）和给定的开始时间（start_time），计算出本次规划的起始点
/*
1.根据上次规划的轨迹 last_trajectory_ 和给定的开始时间 start_time，找到与该时间最接近的轨迹点。
2.使用二分查找（std::lower_bound）来加速查找过程，找出符合条件的起始点。
3.返回一个 std::pair，包含找到的轨迹点和该点在轨迹中的索引。
*/
std::pair<TrajectoryPoint, std::size_t>
Planning::ComputeStartingPointFromLastTrajectory(
    const double start_time) const {
  //  Lambda 函数 comp，用于比较 TrajectoryPoint 和 double 类型的时间
  // 根据 TrajectoryPoint 中的 relative_time 字段（表示轨迹点的相对时间）与给定时间 t 进行比较
  auto comp = [](const TrajectoryPoint& p, const double t) {
    return p.relative_time < t;
  };
  // 使用 std::lower_bound 查找第一个 relative_time 大于或等于 start_time - last_header_time_ 的轨迹点
  // last_header_time_ 是上次规划的时间戳，它被用来将 start_time 转换为与上次规划的相对时间（即与上次规划的开始时间相关的时间）
  // start_time - last_header_time_ ：从上次规划的时间点开始查找
  auto it_lower =
      std::lower_bound(last_trajectory_.begin(), last_trajectory_.end(),
                       start_time - last_header_time_, comp);
  if (it_lower == last_trajectory_.end()) {
    it_lower--;
  }
  // 计算出 it_lower 指向的轨迹点在 last_trajectory_ 中的索引 index
  std::size_t index = it_lower - last_trajectory_.begin();
  return std::pair<TrajectoryPoint, std::size_t>(*it_lower, index);
}

/*
1.根据车辆的当前状态（包括位置、速度、加速度、角速度等）计算轨迹的起始点。
2.将车辆的位置信息（x, y, z）和运动信息（速度、加速度等）赋值给轨迹点。
3.如果车辆有运动（速度大于 0.1），计算并赋值曲率（kappa），描述车辆的转弯程度。
4.初始化轨迹点的曲率变化率、路程和相对时间为 0
*/
/// @brief 根据当前的车辆状态（vehicle_state）生成一个初始的轨迹点（TrajectoryPoint），该点将作为规划的起始点
/// @param vehicle_state 
/// @param forward_time 
/// @return 
TrajectoryPoint Planning::ComputeStartingPointFromVehicleState(
    const common::vehicle_state::VehicleState& vehicle_state,
    const double forward_time) const {
  // Eigen::Vector2d estimated_position =
  // vehicle_state.EstimateFuturePosition(forward_time);
  TrajectoryPoint point; // 存储计算得到的轨迹起始点
  // point.x = estimated_position.x();
  // point.y = estimated_position.y();
  point.x = vehicle_state.x();
  point.y = vehicle_state.y();
  point.z = vehicle_state.z();
  point.v = vehicle_state.linear_velocity();
  point.a = vehicle_state.linear_acceleration();
  point.kappa = 0.0;
  // 车辆正在移动
  if (point.v > 0.1) {
    point.kappa =
        vehicle_state.angular_velocity() / vehicle_state.linear_velocity();
  }
  point.dkappa = 0.0; // 用于描述轨迹的转向变化，初始化时设为 0
  point.s = 0.0;
  point.relative_time = 0.0;
  return point;
}

void Planning::Reset() {
  last_header_time_ = 0.0;
  last_trajectory_.clear();
}

/*
1.从 last_trajectory_ 中提取一段与当前规划匹配的“过渡”轨迹。过渡轨迹的起始点是从 matched_index 向前获取的，长度由 buffer_size 控制。
2.将这段过渡轨迹的时间戳重置，使得轨迹的时间从 matched_index 处开始为零，确保时间连续性。
3.返回这段过渡轨迹，以便与当前规划的轨迹合并，确保控制器执行时的平滑过渡
*/
/// @brief 从上次规划的轨迹中提取出一段“过渡”轨迹，用于与新的规划轨迹连接，确保控制器在执行时能够平滑过渡
/// @param matched_index 上次规划轨迹中匹配点的索引，表示从该点开始获取“过渡”轨迹
/// @param buffer_size 从匹配点向前获取的轨迹点数，即“过渡”轨迹的长度
/// @return 
std::vector<TrajectoryPoint> Planning::GetOverheadTrajectory(
    const std::size_t matched_index, const std::size_t buffer_size) {
// 从哪一轨迹点开始提取过渡轨迹
// 确保如果轨迹较短，过渡轨迹的起点不会超出轨迹范围
  const std::size_t start_index =
      matched_index < buffer_size ? 0 : matched_index - buffer_size;
// 取从 start_index 到 matched_index 之间的轨迹段（不包括 matched_index），
// 并存储在 overhead_trajectory 中。这段轨迹用于平滑连接上次规划与当前规划
  auto overhead_trajectory =
      std::vector<TrajectoryPoint>(last_trajectory_.begin() + start_index,
                                   last_trajectory_.begin() + matched_index);
// 这个时间值将用于重置过渡轨迹的时间，使得轨迹中的时间值从零开始，确保时间连续性
  double zero_relative_time = last_trajectory_[matched_index].relative_time;
  // reset relative time
// 将这段轨迹的时间重置为相对于 matched_index 轨迹点的时间，使得这段轨迹从零时间开始
  for (auto& p : overhead_trajectory) {
    p.relative_time -= zero_relative_time;
  }
  return overhead_trajectory;
}

}  // namespace planning
}  // namespace apollo
