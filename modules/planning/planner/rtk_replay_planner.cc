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

#include "rtk_replay_planner.h"

#include <fstream>

#include "modules/common/log.h"
#include "modules/common/util/string_tokenizer.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::vehicle_state::VehicleState;

RTKReplayPlanner::RTKReplayPlanner() {
  ReadTrajectoryFile(FLAGS_rtk_trajectory_filename);
}
/// @brief 生成基于 RTK 记录的离散化轨迹，确保轨迹的时间连续性并满足后续规划的需求
/// @param start_point 在上一帧中找到当前规划的时间匹配点
/// @param ptr_discretized_trajectory 
/// @return 
bool RTKReplayPlanner::Plan(
    const TrajectoryPoint& start_point,
    std::vector<TrajectoryPoint>* ptr_discretized_trajectory) {
  // 检查RTK轨迹
  if (complete_rtk_trajectory_.empty() || complete_rtk_trajectory_.size() < 2) {
    AERROR << "RTKReplayPlanner doesn't have a recorded trajectory or "
              "the recorded trajectory doesn't have enough valid trajectory "
              "points.";
    return false;
  }
  // 找到与给定起始点（start_point）最匹配的轨迹点在 complete_rtk_trajectory_ 中的位置
  std::size_t matched_index =
      QueryPositionMatchedPoint(start_point, complete_rtk_trajectory_);

  std::size_t forward_buffer = FLAGS_rtk_trajectory_forward;  // 800
  // 计算轨迹的结束索引，确保不超出 complete_rtk_trajectory_ 的范围。
  // 如果 matched_index + forward_buffer 超过了轨迹的长度，则 end_index 被设置为轨迹的最后一个点
  std::size_t end_index =
      matched_index + forward_buffer >= complete_rtk_trajectory_.size()
          ? complete_rtk_trajectory_.size() - 1
          : matched_index + forward_buffer - 1;

  if (ptr_discretized_trajectory->size() > 0) {
    ptr_discretized_trajectory->clear();
  }

  ptr_discretized_trajectory->insert(
      ptr_discretized_trajectory->begin(),
      complete_rtk_trajectory_.begin() + matched_index,
      complete_rtk_trajectory_.begin() + end_index + 1);

  // reset relative time
  // 对轨迹进行时间重置：将所有轨迹点的 relative_time（相对时间）减去 matched_index 处的时间，
  // 使得第一个轨迹点的时间为 0，以便后续规划使用
  double zero_time = complete_rtk_trajectory_[matched_index].relative_time;;
  for (std::size_t i = 0; i < ptr_discretized_trajectory->size(); ++i) {
    (*ptr_discretized_trajectory)[i].relative_time -= zero_time;
  }

  // check if the trajectory has enough points;
  // if not, append the last points multiple times and
  // adjust their corresponding time stamps.
  while (ptr_discretized_trajectory->size() < FLAGS_rtk_trajectory_forward) {
    const auto& last_point = ptr_discretized_trajectory->back();
    ptr_discretized_trajectory->push_back(last_point);
    ptr_discretized_trajectory->back().relative_time += FLAGS_trajectory_resolution; // 0.01
  }
  return true;
}

void RTKReplayPlanner::ReadTrajectoryFile(const std::string& filename) {
  if (!complete_rtk_trajectory_.empty()) {
    complete_rtk_trajectory_.clear();
  }

  std::ifstream file_in(filename.c_str());
  if (!file_in.is_open()) {
    AERROR << "RTKReplayPlanner cannot open trajectory file: " << filename;
    return;
  }

  std::string line;
  // skip the header line.
  getline(file_in, line);

  while (true) {
    getline(file_in, line);
    if (line == "") {
      break;
    }

    auto tokens = apollo::common::util::StringTokenizer::Split(line, "\t ");
    if (tokens.size() < 11) {
      AERROR << "RTKReplayPlanner parse line failed; the data dimension does not match.";
      AERROR << line;
      continue;
    }

    TrajectoryPoint point;
    point.x = std::stod(tokens[0]);
    point.y = std::stod(tokens[1]);
    point.z = std::stod(tokens[2]);

    point.v = std::stod(tokens[3]);
    point.a = std::stod(tokens[4]);

    point.kappa = std::stod(tokens[5]);
    point.dkappa = std::stod(tokens[6]);

    point.relative_time = std::stod(tokens[7]);

    point.theta = std::stod(tokens[8]);

    point.s = std::stod(tokens[10]);
    complete_rtk_trajectory_.push_back(point);
  }

  file_in.close();
}
/*
1.遍历给定轨迹 trajectory 中的每个轨迹点，计算每个点与起始点 start_point 之间的平方距离。
2.找到与起始点距离最小的轨迹点，并返回该点在轨迹中的索引
*/
/// @brief 从给定的轨迹中查找与给定起始点（start_point）最匹配的位置点。匹配标准是找到与起始点的距离最近的轨迹点
/// @param start_point 用于匹配的起始轨迹点
/// @param trajectory 需要查找的轨迹，类型为 std::vector<TrajectoryPoint>，包含了一系列轨迹点
/// @return 
std::size_t RTKReplayPlanner::QueryPositionMatchedPoint(
    const TrajectoryPoint& start_point,
    const std::vector<TrajectoryPoint>& trajectory) const {
  auto func_distance_square = [](const PathPoint& point, const double x,
                                 const double y) {
    double dx = point.x - x;
    double dy = point.y - y;
    return dx * dx + dy * dy;
  };
  double d_min = func_distance_square(trajectory.front(), start_point.x,
                                      start_point.y);
  std::size_t index_min = 0;
  for (std::size_t i = 1; i < trajectory.size(); ++i) {
    double d_temp = func_distance_square(trajectory[i], start_point.x,
                                         start_point.y);
    if (d_temp < d_min) {
      d_min = d_temp;
      index_min = i;
    }
  }
  return index_min;
}

}  // namespace planning
}  // nameapace apollo
