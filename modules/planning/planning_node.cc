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

#include "modules/planning/planning_node.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/proto/planning.pb.h"

namespace apollo {
namespace planning {

using apollo::common::vehicle_state::VehicleState;
using apollo::common::adapter::AdapterManager;
using apollo::common::time::Clock;
using TrajectoryPb = ADCTrajectory;

PlanningNode::PlanningNode() {
  AdapterManager::Init(FLAGS_adapter_config_path);
}

PlanningNode::~PlanningNode() {}
/// @brief 
void PlanningNode::Run() {
  // 控制循环的频率
  static ros::Rate loop_rate(FLAGS_planning_loop_rate);  // 5：每秒执行5次
  while (ros::ok()) {
    // 每次循环时执行 RunOnce() 方法，处理一次规划任务
    RunOnce();
    // 处理ROS消息回调，确保ROS节点能够正常与外部系统进行通信
    ros::spinOnce();
    // 根据设定的频率计算出一个适当的休眠时间，使得每次循环的时间间隔为 1 / FLAGS_planning_loop_rate
    loop_rate.sleep();
  }
}

/*
1.监听和更新外部数据（如定位、底盘数据等）。
2.检查定位和底盘数据的有效性，如果数据无效则跳过当前周期。
3.根据最新的定位和底盘信息进行路径规划。
4.如果规划成功，将规划结果转换为适当的格式并发布。如果规划失败，则输出错误信息
*/
/// @brief 执行了一次完整的规划周期，涉及从不同的数据源获取信息、执行路径规划，并发布规划结果
void PlanningNode::RunOnce() {
  // 监听或更新与外部系统（如传感器、控制器等）的数据流。
  //可能会在每次循环中检查传感器或其他数据的更新
  AdapterManager::Observe();
  if (AdapterManager::GetLocalization() == nullptr) {
    AERROR << "Localization is not available; skip the planning cycle";
    return;
  }
  if (AdapterManager::GetLocalization()->Empty()) {
    AERROR << "localization messages are missing; skip the planning cycle";
    return;
  } else {
    AINFO << "Get localization message;";
  }

  if (AdapterManager::GetChassis() == nullptr) {
    AERROR << "Chassis is not available; skip the planning cycle";
    return;
  }
  if (AdapterManager::GetChassis()->Empty()) {
    AERROR << "Chassis messages are missing; skip the planning cycle";
    return;
  } else {
    AINFO << "Get localization message;";
  }
  // 输出日志，表示规划过程开始
  AINFO << "Start planning ...";
  // 获取最新的定位数据，并将其转换为 VehicleState（车辆状态）
  const auto& localization =
      AdapterManager::GetLocalization()->GetLatestObserved();
  VehicleState vehicle_state(localization);
  // 获取最新的底盘数据，并检查车辆是否处于自动驾驶模式
  const auto& chassis = AdapterManager::GetChassis()->GetLatestObserved();
  bool is_on_auto_mode = chassis.driving_mode() == chassis.COMPLETE_AUTO_DRIVE;
  // 规划周期: 0.2s
  double planning_cycle_time = 1.0 / FLAGS_planning_loop_rate;
  // the execution_start_time is the estimated time when the planned trajectory
  // will be executed by the controller.
  // 预估执行时间，根据当前时间和规划周期时间计算得出的，它表示规划的轨迹将在多长时间后被控制器执行
  double execution_start_time =
      apollo::common::time::ToSecond(apollo::common::time::Clock::Now()) +
      planning_cycle_time;
  // 存储规划生成的轨迹点
  std::vector<TrajectoryPoint> planning_trajectory;
  // 生成规划轨迹
  bool res_planning = planning_.Plan(vehicle_state, is_on_auto_mode,
      execution_start_time, &planning_trajectory);
  // 如果规划成功，则将规划结果转换为 TrajectoryPb（一个protobuf格式的轨迹消息），
  //并通过 AdapterManager::PublishPlanningTrajectory() 发布该轨迹
  if (res_planning) {
    TrajectoryPb trajectory_pb = ToTrajectoryPb(execution_start_time, planning_trajectory);
    AdapterManager::PublishPlanningTrajectory(trajectory_pb);
    AINFO << "Planning succeeded";
  } else {
    AINFO << "Planning failed";
  }
}

void PlanningNode::Reset() {
  planning_.Reset();
}

TrajectoryPb PlanningNode::ToTrajectoryPb(
    const double header_time,
    const std::vector<TrajectoryPoint>& discretized_trajectory) {
  TrajectoryPb trajectory_pb;
  AdapterManager::FillPlanningTrajectoryHeader("planning",
                                               trajectory_pb.mutable_header());

  trajectory_pb.mutable_header()->set_timestamp_sec(header_time);

  for (const auto& trajectory_point : discretized_trajectory) {
    auto ptr_trajectory_point_pb = trajectory_pb.add_adc_trajectory_point();
    ptr_trajectory_point_pb->set_x(trajectory_point.x);
    ptr_trajectory_point_pb->set_y(trajectory_point.y);
    ptr_trajectory_point_pb->set_theta(trajectory_point.theta);
    ptr_trajectory_point_pb->set_curvature(trajectory_point.kappa);
    ptr_trajectory_point_pb->set_relative_time(trajectory_point.relative_time);
    ptr_trajectory_point_pb->set_speed(trajectory_point.v);
    ptr_trajectory_point_pb->set_acceleration_s(trajectory_point.a);
    ptr_trajectory_point_pb->set_accumulated_s(trajectory_point.s);
  }
  return std::move(trajectory_pb);
}

}  // namespace planning
}  // namespace apollo
