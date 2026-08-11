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
 * @file reference_line_info.cc
 * @brief 参考线信息实现文件
 *
 * 本文件实现ReferenceLineInfo类，是Apollo规划模块中参考线规划的核心数据结构。
 * ReferenceLineInfo封装了一条参考线的所有信息，包括参考线本身、路径数据、速度数据、
 * 障碍物决策、轨迹等。
 *
 * 主要功能：
 * 1. 参考线初始化和SL边界计算
 * 2. 障碍物管理和决策
 * 3. 路径与速度规划结果存储
 * 4. 轨迹合成与调整
 * 5. 决策结果导出
 *
 * 设计特点：
 * - 包含完整的规划结果（路径、速度、轨迹）
 * - 管理障碍物决策和ST边界
 * - 支持多参考线规划的选择
 *
 * C++语法说明：
 * - std::unordered_map<K, V>: 基于哈希表的关联数组，O(1)查找
 * - std::ios::fixed/setprecision: IO流格式化输出
 * - cyber::common::Hash(): Cyber RT哈希函数
 * - std::move(): 移动语义，避免不必要拷贝
 * - std::for_each(): STL算法，对范围内元素执行函数
 * - lambda表达式: [](params){body} 匿名函数
 */
#include "modules/planning/planning_base/common/reference_line_info.h"

#include <algorithm>  /**< C++标准算法库，包含std::sort, std::for_each等 */

#include "absl/strings/str_cat.h" /**< Abseil字符串拼接库 */

#include "modules/common_msgs/planning_msgs/sl_boundary.pb.h" /**< SL边界protobuf */
#include "modules/planning/planning_base/proto/planning_status.pb.h" /**< 规划状态protobuf */

#include "cyber/task/task.h"  /**< Cyber RT异步任务 */
#include "modules/common/configs/vehicle_config_helper.h" /**< 车辆配置助手 */
#include "modules/common/util/point_factory.h" /**< 点工厂工具 */
#include "modules/common/util/util.h"    /**< 通用工具函数 */
#include "modules/map/hdmap/hdmap_common.h" /**< 地图通用定义 */
#include "modules/map/hdmap/hdmap_util.h" /**< 地图工具 */
#include "modules/planning/planning_base/common/util/print_debug_info.h" /**< 调试信息打印 */

namespace apollo {
/**
 * apollo:: - Apollo最外层命名空间
 */
namespace planning {

/**
 * using声明 - 将其他命名空间的类型引入当前作用域
 */
using apollo::canbus::Chassis;            /**< 底盘状态类型 */
using apollo::common::EngageAdvice;       /**< 驾驶建议类型 */
using apollo::common::TrajectoryPoint;     /**< 轨迹点类型 */
using apollo::common::VehicleConfigHelper; /**< 车辆配置助手 */
using apollo::common::VehicleSignal;       /**< 车辆信号类型 */
using apollo::common::math::Box2d;         /**< 二维包围盒 */
using apollo::common::math::Vec2d;         /**< 二维向量 */
using apollo::common::util::PointFactory;  /**< 点工厂 */

/**
 * 静态成员变量定义
 * std::unordered_map<K, V>: 基于哈希表的关联数组
 * 键为string（路口ID），值为bool（是否受保护）
 * static表示类级别共享，所有ReferenceLineInfo实例共享同一份数据
 */
std::unordered_map<std::string, bool>
    ReferenceLineInfo::junction_right_of_way_map_;

/**
 * @brief ReferenceLineInfo构造函数
 *
 * 初始化参考线信息，包括：
 * 1. 保存车辆状态、规划起点、参考线和路径段
 * 2. 生成参考线的唯一标识符
 *
 * @param vehicle_state 车辆状态
 * @param adc_planning_point 自车规划起点
 * @param reference_line 参考线
 * @param segments 路径段（路由段列表）
 *
 * 语法说明：
 * - : member_(value){} 初始化列表语法
 * - lanes_(segments) 直接初始化成员变量
 * - std::ostringstream: 字符串输出流，用于格式化字符串
 * - segs_id.setf(std::ios::fixed): 设置浮点数输出为固定小数点格式
 * - segs_id.precision(2): 设置小数精度为2位
 * - cyber::common::Hash(): 计算字符串的哈希值
 */
ReferenceLineInfo::ReferenceLineInfo(const common::VehicleState& vehicle_state,
                                     const TrajectoryPoint& adc_planning_point,
                                     const ReferenceLine& reference_line,
                                     const hdmap::RouteSegments& segments)
    : vehicle_state_(vehicle_state),             /**< 初始化车辆状态 */
      adc_planning_point_(adc_planning_point),   /**< 初始化规划起点 */
      reference_line_(reference_line),           /**< 初始化参考线 */
      lanes_(segments) {                         /**< 初始化路径段 */
  /**
   * 检查路径段是否为空
   * if (lanes_.size() == 0)
   *   - size()返回容器大小
   */
  if (lanes_.size() == 0) {
    return;  /**< 为空直接返回 */
  }

  /**
   * 创建字符串流用于格式化输出
   * std::ostringstream: 可变字符串，用于构建唯一ID
   */
  std::ostringstream segs_id;
  segs_id.setf(std::ios::fixed);  /**< 设置固定小数点格式 */
  segs_id.precision(2);           /**< 设置小数精度为2位 */
  segs_id.str("");                /**< 清空字符串 */

  /**
   * 遍历所有路径段
   * for (const auto& seg : lanes_)
   *   - range-based for循环，C++11
   *   - const auto&: const引用，避免拷贝
   */
  for (const auto& seg : lanes_) {
    /**
     * 拼接车道ID
     * seg.lane->id().id(): 获取车道ID字符串
     * << 操作符用于流输出
     */
    segs_id << "_" << seg.lane->id().id();
  }

  /**
   * 拼接起止s坐标
   * lanes_.front().start_s: 第一个路径段的起始s
   * lanes_.back().end_s: 最后一个路径段的结束s
   */
  segs_id << "_" << lanes_.front().start_s << "_" << lanes_.back().end_s;

  /**
   * 计算哈希值作为唯一标识
   * cyber::common::Hash(segs_id.str()): 计算字符串的哈希值
   */
  key_ = cyber::common::Hash(segs_id.str());
  id_ = segs_id.str();  /**< 保存原始字符串作为ID */
}

/**
 * @brief 参考线信息初始化
 *
 * 初始化参考线信息，包括：
 * 1. 计算自车SL边界
 * 2. 初始化重叠区域
 * 3. 添加障碍物
 * 4. 设置速度限制
 *
 * @param obstacles 障碍物列表
 * @param target_speed 目标速度
 * @return bool 初始化是否成功
 *
 * 语法说明：
 * - const std::vector<const Obstacle*>&: 常量引用，障碍物指针向量
 * - const auto& param = VehicleConfigHelper::GetConfig().vehicle_param():
   获取车辆参数
 * - Vec2d::rotate(): 二维向量旋转
 * - Box2d: 二维包围盒
 * - reference_line_.GetSLBoundary(): 获取SL边界
 */
bool ReferenceLineInfo::Init(const std::vector<const Obstacle*>& obstacles,
                             double target_speed) {
  /**
   * 获取车辆参数
   * VehicleConfigHelper::GetConfig(): 获取车辆配置单例
   * .vehicle_param(): 获取车辆参数子配置
   */
  const auto& param = VehicleConfigHelper::GetConfig().vehicle_param();

  /**
   * 获取规划起点的路径点
   * adc_planning_point_.path_point(): 获取路径点
   */
  const auto& path_point = adc_planning_point_.path_point();

  /**
   * 创建自车位置向量
   * Vec2d(x, y): 二维向量构造函数
   */
  Vec2d position(path_point.x(), path_point.y());

  /**
   * 计算从后轴中心到几何中心的向量
   * (前边缘-后边缘)/2, (左边缘-右边缘)/2
   */
  Vec2d vec_to_center(
      (param.front_edge_to_center() - param.back_edge_to_center()) / 2.0,
      (param.left_edge_to_center() - param.right_edge_to_center()) / 2.0);

  /**
   * 计算几何中心
   * 位置 + vec_to_center.rotate(heading)
   * rotate(): 旋转向量到世界坐标系
   */
  Vec2d center(position + vec_to_center.rotate(path_point.theta()));

  /**
   * 创建自车包围盒
   * Box2d(center, heading, length, width)
   */
  Box2d box(center, path_point.theta(), param.length(), param.width());

  /**
   * 创建实时车辆位置包围盒
   * 使用vehicle_state_而非planning_point
   */
  Vec2d vehicle_position(vehicle_state_.x(), vehicle_state_.y());
  Vec2d vehicle_center(vehicle_position +
                       vec_to_center.rotate(vehicle_state_.heading()));
  Box2d vehicle_box(vehicle_center, vehicle_state_.heading(), param.length(),
                    param.width());

  /**
   * 获取自车SL边界
   * reference_line_.GetSLBoundary(box, &adc_sl_boundary_)
   *   - 将XY坐标系下的包围盒转换为SL坐标系
   */
  if (!reference_line_.GetSLBoundary(box, &adc_sl_boundary_)) {
    AERROR << "Failed to get ADC boundary from box: " << box.DebugString();
    return false;
  }

  /**
   * 初始化第一个重叠区域
   */
  InitFirstOverlaps();

  /**
   * 检查自车SL边界是否在参考线上
   * adc_sl_boundary_.end_s() < 0: 在参考线起点之前
   * adc_sl_boundary_.start_s() > reference_line_.Length(): 在参考线终点之后
   */
  if (adc_sl_boundary_.end_s() < 0 ||
      adc_sl_boundary_.start_s() > reference_line_.Length()) {
    AWARN << "Vehicle SL " << adc_sl_boundary_.ShortDebugString()
          << " is not on reference line:[0, " << reference_line_.Length()
          << "]";
  }

  /**
   * 检查横向距离是否过大
   * static constexpr double: 静态编译时常量
   * kOutOfReferenceLineL = 14.0米
   */
  static constexpr double kOutOfReferenceLineL = 14.0;  // 单位：米
  if (adc_sl_boundary_.start_l() > kOutOfReferenceLineL ||
      adc_sl_boundary_.end_l() < -kOutOfReferenceLineL) {
    AERROR << "Ego vehicle is too far away from reference line."
           << "adc_sl_boundary_.start_l:" << adc_sl_boundary_.start_l()
           << "adc_sl_boundary_.end_l:" << adc_sl_boundary_.end_l();
    return false;
  }

  /**
   * 检查自车是否在参考线上
   * reference_line_.IsOnLane(): 判断SL边界是否在车道内
   */
  is_on_reference_line_ = reference_line_.IsOnLane(adc_sl_boundary_);

  /**
   * 添加障碍物
   * AddObstacles(): 添加所有障碍物到参考线
   */
  if (!AddObstacles(obstacles)) {
    AERROR << "Failed to add obstacles to reference line";
    return false;
  }

  /**
   * 遍历地图路径获取减速带
   * const auto& map_path = reference_line_.map_path():
   *   获取参考线对应的地图路径
   * map_path.speed_bump_overlaps(): 获取减速带重叠区域
   */
  const auto& map_path = reference_line_.map_path();
  for (const auto& speed_bump : map_path.speed_bump_overlaps()) {
    /**
     * 添加速度限制
     * -1和+1.0确保可以采样到
     * FLAGS_speed_bump_speed_limit: 减速带速度限制配置
     */
    reference_line_.AddSpeedLimit(speed_bump.start_s - 1.0,
                                  speed_bump.end_s + 1.0,
                                  FLAGS_speed_bump_speed_limit);
  }

  /**
   * 设置巡航速度
   * SetCruiseSpeed(): 设置基础巡航速度
   * SetLatticeCruiseSpeed(): 设置网格规划巡航速度
   */
  SetCruiseSpeed(target_speed);
  SetLatticeCruiseSpeed(target_speed);

  /**
   * 清空车辆信号
   * vehicle_signal_.Clear(): 清空protobuf消息
   */
  vehicle_signal_.Clear();

  return true;  /**< 初始化成功 */
}

/**
 * @brief 获取候选路径数据列表
 *
 * @return const std::vector<PathData>& 候选路径数据列表的常量引用
 *
 * 语法说明：
 * - const std::vector<PathData>&: 常量引用返回，避免拷贝
 */
const std::vector<PathData>& ReferenceLineInfo::GetCandidatePathData() const {
  return candidate_path_data_;
}

/**
 * @brief 设置候选路径数据
 *
 * 使用移动语义避免拷贝。
 *
 * @param candidate_path_data 候选路径数据（右值引用）
 *
 * 语法说明：
 * - std::vector<PathData>&&: 右值引用，用于移动语义
 * - candidate_path_data_ = std::move(candidate_path_data):
 *   移动而非拷贝，提高效率
 */
void ReferenceLineInfo::SetCandidatePathData(
    std::vector<PathData>&& candidate_path_data) {
  candidate_path_data_ = std::move(candidate_path_data);
}

/**
 * @brief 获取候选路径边界列表
 *
 * @return const std::vector<PathBoundary>& 候选路径边界列表的常量引用
 */
const std::vector<PathBoundary>& ReferenceLineInfo::GetCandidatePathBoundaries()
    const {
  return candidate_path_boundaries_;
}

/**
 * @brief 设置候选路径边界
 *
 * @param path_boundaries 候选路径边界（右值引用）
 */
void ReferenceLineInfo::SetCandidatePathBoundaries(
    std::vector<PathBoundary>&& path_boundaries) {
  candidate_path_boundaries_ = std::move(path_boundaries);
}

/**
 * @brief 限制巡航速度
 *
 * 如果当前巡航速度高于限制值，则降低到限制值。
 *
 * @param speed 限制速度
 *
 * 语法说明：
 * - if (base_cruise_speed_ <= speed): 检查当前速度是否更低
 */
void ReferenceLineInfo::LimitCruiseSpeed(double speed) {
  if (base_cruise_speed_ <= speed) {
    return;  /**< 当前速度已低于限制，直接返回 */
  }
  cruise_speed_ = speed;  /**< 更新巡航速度 */
}

/**
 * @brief 获取基础巡航速度
 *
 * @return double 基础巡航速度，如果为0则使用默认值
 *
 * 语法说明：
 * - 三元运算符: cond ? val1 : val2
 */
double ReferenceLineInfo::GetBaseCruiseSpeed() const {
  return base_cruise_speed_ > 0.0 ? base_cruise_speed_
                                  : FLAGS_default_cruise_speed;
}

/**
 * @brief 获取巡航速度
 *
 * @return double 巡航速度，如果为0则使用默认值
 */
double ReferenceLineInfo::GetCruiseSpeed() const {
  return cruise_speed_ > 0.0 ? cruise_speed_ : FLAGS_default_cruise_speed;
}

/**
 * @brief 获取指定s坐标处的车道信息
 *
 * @param s 沿参考线的距离
 * @return hdmap::LaneInfoConstPtr 车道信息指针
 *
 * 语法说明：
 * - hdmap::LaneInfoConstPtr: 车道信息常量指针
 * - reference_line_.GetLaneFromS(s, &lanes): 根据s坐标获取车道列表
 * - lanes.empty(): 检查向量是否为空
 * - lanes.front(): 返回首元素引用
 */
hdmap::LaneInfoConstPtr ReferenceLineInfo::LocateLaneInfo(
    const double s) const {
  std::vector<hdmap::LaneInfoConstPtr> lanes;
  reference_line_.GetLaneFromS(s, &lanes);
  if (lanes.empty()) {
    AWARN << "cannot get any lane using s";
    return nullptr;
  }
  return lanes.front();
}

/**
 * @brief 获取相邻车道信息
 *
 * 根据车道类型获取相邻车道的信息。
 *
 * @param lane_type 车道类型（左前/左后/右前/右后）
 * @param s 沿参考线的距离
 * @param ptr_lane_id 输出：车道ID
 * @param ptr_lane_width 输出：车道宽度
 * @return bool 是否成功获取
 *
 * 语法说明：
 * - switch-case: 分支语句，根据lane_type处理不同情况
 * - ptr_lane_info->lane().left_neighbor_forward_lane_id():
 *   获取左前方相邻车道ID列表
 * - .empty(): 检查列表是否为空
 */
bool ReferenceLineInfo::GetNeighborLaneInfo(
    const ReferenceLineInfo::LaneType lane_type, const double s,
    hdmap::Id* ptr_lane_id, double* ptr_lane_width) const {
  auto ptr_lane_info = LocateLaneInfo(s);
  if (ptr_lane_info == nullptr) {
    return false;
  }

  /**
   * switch分支处理不同的车道类型
   */
  switch (lane_type) {
    case LaneType::LeftForward: {  /**< 左前方车道 */
      if (ptr_lane_info->lane().left_neighbor_forward_lane_id().empty()) {
        return false;
      }
      *ptr_lane_id = ptr_lane_info->lane().left_neighbor_forward_lane_id(0);
      break;
    }
    case LaneType::LeftReverse: {  /**< 左后方车道 */
      if (ptr_lane_info->lane().left_neighbor_reverse_lane_id().empty()) {
        return false;
      }
      *ptr_lane_id = ptr_lane_info->lane().left_neighbor_reverse_lane_id(0);
      break;
    }
    case LaneType::RightForward: {  /**< 右前方车道 */
      if (ptr_lane_info->lane().right_neighbor_forward_lane_id().empty()) {
        return false;
      }
      *ptr_lane_id = ptr_lane_info->lane().right_neighbor_forward_lane_id(0);
      break;
    }
    case LaneType::RightReverse: {  /**< 右后方车道 */
      if (ptr_lane_info->lane().right_neighbor_reverse_lane_id().empty()) {
        return false;
      }
      *ptr_lane_id = ptr_lane_info->lane().right_neighbor_reverse_lane_id(0);
      break;
    }
    default:  /**< 默认情况 */
      ACHECK(false);  /**< 断言失败，程序终止 */
  }

  /**
   * 根据ID获取相邻车道
   */
  auto ptr_neighbor_lane =
      hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(*ptr_lane_id);
  if (ptr_neighbor_lane == nullptr) {
    return false;
  }

  /**
   * 获取参考点并计算投影
   */
  auto ref_point = reference_line_.GetReferencePoint(s);
  double neighbor_s = 0.0;
  double neighbor_l = 0.0;
  if (!ptr_neighbor_lane->GetProjection({ref_point.x(), ref_point.y()},
                                        &neighbor_s, &neighbor_l)) {
    return false;
  }

  /**
   * 获取车道宽度
   */
  *ptr_lane_width = ptr_neighbor_lane->GetWidth(neighbor_s);
  return true;
}

/**
 * @brief 获取第一个重叠区域
 *
 * 在自车前方的重叠区域中，找到s坐标最小的那个。
 *
 * @param path_overlaps 重叠区域列表
 * @param path_overlap 输出：第一个重叠区域
 * @return bool 是否找到
 *
 * 语法说明：
 * - CHECK_NOTNULL(ptr): 断言检查指针非空
 * - static constexpr double: 静态编译时常量
 * - for循环遍历迭代器
 */
bool ReferenceLineInfo::GetFirstOverlap(
    const std::vector<hdmap::PathOverlap>& path_overlaps,
    hdmap::PathOverlap* path_overlap) {
  CHECK_NOTNULL(path_overlap);  /**< 断言检查输出参数非空 */
  const double start_s = adc_sl_boundary_.end_s();  /**< 自车后边界s坐标 */
  static constexpr double kMaxOverlapRange = 500.0;  /**< 最大重叠范围 */
  double overlap_min_s = kMaxOverlapRange;  /**< 初始化最小s为最大值 */

  auto overlap_min_s_iter = path_overlaps.end();  /**< 迭代器初始化为end */
  for (auto iter = path_overlaps.begin(); iter != path_overlaps.end(); ++iter) {
    if (iter->end_s < start_s) {  /**< 重叠区域在自车后方 */
      continue;  /**< 跳过 */
    }
    if (overlap_min_s > iter->start_s) {  /**< 找到更小的起始s */
      overlap_min_s_iter = iter;
      overlap_min_s = iter->start_s;
    }
  }

  /**
   * 检查是否找到有效重叠区域
   */
  if (overlap_min_s_iter != path_overlaps.end()) {
    *path_overlap = *overlap_min_s_iter;  /**< 拷贝重叠区域 */
  }

  return overlap_min_s < kMaxOverlapRange;  /**< 返回是否在范围内 */
}

/**
 * @brief 初始化第一个重叠区域
 *
 * 遍历地图路径，找到各类重叠区域的第一个。
 *
 * 处理的重叠类型：
 * - 清除区域(clear_zone)
 * - 人行横道(crosswalk)
 * - PNC路口(pnc_junction)
 * - 交通信号灯(signal)
 * - 停车让行标志(stop_sign)
 * - 减速让行标志(yield_sign)
 * - 区域(area)
 *
 * 语法说明：
 * - emplace_back(): 在容器末尾就地构造元素
 * - std::sort(): 排序算法
 * - lambda表达式作为比较函数
 */
void ReferenceLineInfo::InitFirstOverlaps() {
  const auto& map_path = reference_line_.map_path();  /**< 获取地图路径 */

  /**
   * 清除区域
   */
  hdmap::PathOverlap clear_area_overlap;
  if (GetFirstOverlap(map_path.clear_area_overlaps(), &clear_area_overlap)) {
    first_encounter_overlaps_.emplace_back(CLEAR_AREA, clear_area_overlap);
  }

  /**
   * 人行横道
   */
  hdmap::PathOverlap crosswalk_overlap;
  if (GetFirstOverlap(map_path.crosswalk_overlaps(), &crosswalk_overlap)) {
    first_encounter_overlaps_.emplace_back(CROSSWALK, crosswalk_overlap);
  }

  /**
   * PNC路口
   */
  hdmap::PathOverlap pnc_junction_overlap;
  if (GetFirstOverlap(map_path.pnc_junction_overlaps(),
                      &pnc_junction_overlap)) {
    first_encounter_overlaps_.emplace_back(PNC_JUNCTION, pnc_junction_overlap);
  }

  /**
   * 交通信号灯
   */
  hdmap::PathOverlap signal_overlap;
  if (GetFirstOverlap(map_path.signal_overlaps(), &signal_overlap)) {
    first_encounter_overlaps_.emplace_back(SIGNAL, signal_overlap);
  }

  /**
   * 停车让行标志
   */
  hdmap::PathOverlap stop_sign_overlap;
  if (GetFirstOverlap(map_path.stop_sign_overlaps(), &stop_sign_overlap)) {
    first_encounter_overlaps_.emplace_back(STOP_SIGN, stop_sign_overlap);
  }

  /**
   * 减速让行标志
   */
  hdmap::PathOverlap yield_sign_overlap;
  if (GetFirstOverlap(map_path.yield_sign_overlaps(), &yield_sign_overlap)) {
    first_encounter_overlaps_.emplace_back(YIELD_SIGN, yield_sign_overlap);
  }

  /**
   * 区域
   */
  hdmap::PathOverlap area_overlap;
  if (GetFirstOverlap(map_path.area_overlaps(), &area_overlap)) {
    first_encounter_overlaps_.emplace_back(AREA, area_overlap);
  }

  /**
   * 按start_s排序
   */
  if (!first_encounter_overlaps_.empty()) {
    std::sort(first_encounter_overlaps_.begin(),
              first_encounter_overlaps_.end(),
              [](const std::pair<OverlapType, hdmap::PathOverlap>& a,
                 const std::pair<OverlapType, hdmap::PathOverlap>& b) {
                return a.second.start_s < b.second.start_s;
              });
  }
}

/**
 * @brief 判断s坐标是否在重叠区域内
 *
 * @param overlap 重叠区域
 * @param s 沿参考线的距离
 * @return bool 是否在重叠区域内
 *
 * 语法说明：
 * - static constexpr double kEpsilon = 1e-2: 小量容差
 */
bool WithinOverlap(const hdmap::PathOverlap& overlap, double s) {
  static constexpr double kEpsilon = 1e-2;  /**< 容差：0.01米 */
  return overlap.start_s - kEpsilon <= s && s <= overlap.end_s + kEpsilon;
}

/**
 * @brief 设置路口通行权
 *
 * @param junction_s 路口s坐标
 * @param is_protected 是否受保护
 */
void ReferenceLineInfo::SetJunctionRightOfWay(const double junction_s,
                                              const bool is_protected) const {
  for (const auto& overlap : reference_line_.map_path().junction_overlaps()) {
    if (WithinOverlap(overlap, junction_s)) {
      junction_right_of_way_map_[overlap.object_id] = is_protected;
    }
  }
}

/**
 * @brief 获取通行权状态
 *
 * @return ADCTrajectory::RightOfWayStatus 通行权状态
 *
 * 语法说明：
 * - ADCTrajectory::PROTECTED: 受保护通行
 * - ADCTrajectory::UNPROTECTED: 非保护通行
 */
ADCTrajectory::RightOfWayStatus ReferenceLineInfo::GetRightOfWayStatus() const {
  for (const auto& overlap : reference_line_.map_path().junction_overlaps()) {
    if (overlap.end_s < adc_sl_boundary_.start_s()) {
      junction_right_of_way_map_.erase(overlap.object_id);  /**< 移除已过路口 */
    } else if (WithinOverlap(overlap, adc_sl_boundary_.end_s())) {
      auto is_protected = junction_right_of_way_map_[overlap.object_id];
      if (is_protected) {
        return ADCTrajectory::PROTECTED;
      }
    }
  }
  return ADCTrajectory::UNPROTECTED;
}

/**
 * @brief 获取路径段引用
 *
 * @return const hdmap::RouteSegments& 路径段列表引用
 */
const hdmap::RouteSegments& ReferenceLineInfo::Lanes() const { return lanes_; }

/**
 * @brief 获取目标车道ID列表
 *
 * @return std::list<hdmap::Id> 车道ID列表
 *
 * 语法说明：
 * - std::list<T>: 双向链表容器
 * - lane_ids.push_back(): 在链表末尾添加元素
 */
std::list<hdmap::Id> ReferenceLineInfo::TargetLaneId() const {
  std::list<hdmap::Id> lane_ids;
  for (const auto& lane_seg : lanes_) {
    lane_ids.push_back(lane_seg.lane->id());
  }
  return lane_ids;
}

/**
 * @brief 获取自车SL边界引用
 *
 * @return const SLBoundary& SL边界引用
 */
const SLBoundary& ReferenceLineInfo::AdcSlBoundary() const {
  return adc_sl_boundary_;
}

/**
 * @brief 获取路径决策指针
 *
 * @return PathDecision* 路径决策指针
 */
PathDecision* ReferenceLineInfo::path_decision() { return &path_decision_; }

/**
 * @brief 获取路径决策常量引用
 *
 * @return const PathDecision& 路径决策引用
 */
const PathDecision& ReferenceLineInfo::path_decision() const {
  return path_decision_;
}

/**
 * @brief 获取参考线常量引用
 *
 * @return const ReferenceLine& 参考线引用
 */
const ReferenceLine& ReferenceLineInfo::reference_line() const {
  return reference_line_;
}

/**
 * @brief 获取可写参考线指针
 *
 * @return ReferenceLine* 参考线指针
 */
ReferenceLine* ReferenceLineInfo::mutable_reference_line() {
  return &reference_line_;
}

/**
 * @brief 设置轨迹
 *
 * @param trajectory 离散轨迹
 */
void ReferenceLineInfo::SetTrajectory(const DiscretizedTrajectory& trajectory) {
  discretized_trajectory_ = trajectory;
}

/**
 * @brief 障碍物添加辅助函数
 *
 * 使用共享指针版本的AddObstacle。
 *
 * @param obstacle 共享指针障碍物
 * @return bool 是否添加成功
 */
bool ReferenceLineInfo::AddObstacleHelper(
    const std::shared_ptr<Obstacle>& obstacle) {
  return AddObstacle(obstacle.get()) != nullptr;
}

/**
 * @brief 添加障碍物到参考线
 *
 * 添加障碍物并计算其在SL和ST坐标系下的边界。
 * 注意：此函数是线程安全的。
 *
 * @param obstacle 障碍物指针
 * @return Obstacle* 添加后的障碍物指针，失败返回nullptr
 *
 * 语法说明：
 * - if (!obstacle): 空指针检查
 * - auto* mutable_obstacle = path_decision_.AddObstacle(*obstacle):
 *   添加障碍物到决策并获取可写指针
 * - reference_line_.GetSLBoundary(): 计算SL边界
 * - mutable_obstacle->SetPerceptionSlBoundary(): 设置感知SL边界
 * - mutable_obstacle->BuildReferenceLineStBoundary(): 构建ST边界
 */
Obstacle* ReferenceLineInfo::AddObstacle(const Obstacle* obstacle) {
  if (!obstacle) {  /**< 空指针检查 */
    AERROR << "The provided obstacle is empty";
    return nullptr;
  }

  // 1. 添加到PathDecision
  auto* mutable_obstacle = path_decision_.AddObstacle(*obstacle);
  if (!mutable_obstacle) {
    AERROR << "failed to add obstacle " << obstacle->Id();
    return nullptr;
  }

  // 2. ★ 计算SL边界（投影到参考线Frenet坐标系）
  SLBoundary perception_sl;
  if (!reference_line_.GetSLBoundary(obstacle->PerceptionPolygon(),
                                     &perception_sl)) {
    AERROR << "Failed to get sl boundary for obstacle: " << obstacle->Id();
    return mutable_obstacle;
  }
  mutable_obstacle->SetPerceptionSlBoundary(perception_sl);

  // 3. 检查是否阻塞车道
  mutable_obstacle->CheckLaneBlocking(reference_line_);

  /**
   * 检查是否阻塞车道
   */
  if (mutable_obstacle->IsLaneBlocking()) {
    ADEBUG << "obstacle [" << obstacle->Id() << "] is lane blocking.";
  } else {
    ADEBUG << "obstacle [" << obstacle->Id() << "] is NOT lane blocking.";
  }

  // 4. 判断是否为无关障碍物
  if (IsIrrelevantObstacle(*mutable_obstacle)) {
    ObjectDecisionType ignore;
    // 无关 → 设置IGNORE决策，不建ST边界
    ignore.mutable_ignore();  
    path_decision_.AddLateralDecision("reference_line_filter", obstacle->Id(),
                                      ignore);
    path_decision_.AddLongitudinalDecision("reference_line_filter",
                                           obstacle->Id(), ignore);
    AINFO << "NO build reference line st boundary. id:" << obstacle->Id();
  } else {
    // 相关 → ★ 构建参考线ST边界
    AINFO << "build reference line st boundary. id:" << obstacle->Id();
    mutable_obstacle->BuildReferenceLineStBoundary(reference_line_,
                                                   adc_sl_boundary_.start_s());
    ADEBUG << "reference line st boundary: t["
           << mutable_obstacle->reference_line_st_boundary().min_t() << ", "
           << mutable_obstacle->reference_line_st_boundary().max_t() << "] s["
           << mutable_obstacle->reference_line_st_boundary().min_s() << ", "
           << mutable_obstacle->reference_line_st_boundary().max_s() << "]";
  }
  return mutable_obstacle;
}

/**
 * @brief 批量添加障碍物
 *
 * 支持多线程添加障碍物以提高效率。
 *
 * @param obstacles 障碍物指针列表
 * @return bool 是否全部添加成功
 *
 * 语法说明：
 * - FLAGS_use_multi_thread_to_add_obstacles: 配置标志，决定是否多线程
 * - std::vector<std::future<Obstacle*>>: 异步结果向量
 * - cyber::Async(): 异步执行函数
 * - result.get(): 获取异步结果
 */
bool ReferenceLineInfo::AddObstacles(
    const std::vector<const Obstacle*>& obstacles) {
  if (FLAGS_use_multi_thread_to_add_obstacles) {
    std::vector<std::future<Obstacle*>> results;
    for (const auto* obstacle : obstacles) {
      /**
       * 异步调用AddObstacle
       * cyber::Async(&ReferenceLineInfo::AddObstacle, this, obstacle)
       *   - &ReferenceLineInfo::AddObstacle: 成员函数指针
       *   - this: 对象指针
       *   - obstacle: 函数参数
       */
      results.push_back(
          cyber::Async(&ReferenceLineInfo::AddObstacle, this, obstacle));
    }
    for (auto& result : results) {
      if (!result.get()) {  /**< 获取异步结果 */
        AERROR << "Fail to add obstacles.";
        return false;
      }
    }
  } else {
    for (const auto* obstacle : obstacles) {
      if (!AddObstacle(obstacle)) {
        AERROR << "Failed to add obstacle " << obstacle->Id();
        return false;
      }
    }
  }
  return true;
}

/**
 * @brief 判断是否为无关障碍物
 *
 * 无关障碍物会被忽略，不参与规划计算。
 *
 * @param obstacle 障碍物
 * @return bool 是否无关
 *
 * 无关条件：
 * 1. 非危险级别障碍物
 * 2. 在参考线范围之外
 * 3. 自车后方且不在当前车道
 */
bool ReferenceLineInfo::IsIrrelevantObstacle(const Obstacle& obstacle) {
  if (obstacle.IsCautionLevelObstacle()) {
    return false;  /**< 危险障碍物不是无关的 */
  }

  const auto& obstacle_boundary = obstacle.PerceptionSLBoundary();
  if (obstacle_boundary.start_s() > reference_line_.Length()) {
    return true;  /**< 在参考线范围之外 */
  }

  /**
   * 如果自车在参考线上、不是换道路径、且障碍物在自车后方
   */
  if (is_on_reference_line_ && !IsChangeLanePath() &&
      adc_sl_boundary_.start_s() - obstacle_boundary.end_s() >
          FLAGS_obstacle_lon_ignore_buffer &&
      (reference_line_.IsOnLane(obstacle_boundary) ||
       obstacle_boundary.end_s() < 0.0)) {
    return true;  
  }
  return false;
}

/**
 * @brief 获取轨迹引用
 *
 * @return const DiscretizedTrajectory& 轨迹引用
 */
const DiscretizedTrajectory& ReferenceLineInfo::trajectory() const {
  return discretized_trajectory_;
}

/**
 * @brief 设置网格规划停车点
 */
void ReferenceLineInfo::SetLatticeStopPoint(const StopPoint& stop_point) {
  planning_target_.mutable_stop_point()->CopyFrom(stop_point);
}

/**
 * @brief 设置网格规划巡航速度
 */
void ReferenceLineInfo::SetLatticeCruiseSpeed(double speed) {
  planning_target_.set_cruise_speed(speed);
}

/**
 * @brief 判断是否从上一参考线开始
 *
 * @param previous_reference_line_info 上一参考线信息
 * @return bool 是否可以从上一参考线继续
 */
bool ReferenceLineInfo::IsStartFrom(
    const ReferenceLineInfo& previous_reference_line_info) const {
  if (reference_line_.reference_points().empty()) {
    return false;
  }
  auto start_point = reference_line_.reference_points().front();
  const auto& prev_reference_line =
      previous_reference_line_info.reference_line();
  common::SLPoint sl_point;
  prev_reference_line.XYToSL(start_point, &sl_point);
  return previous_reference_line_info.reference_line_.IsOnLane(sl_point);
}

/**
 * @brief 获取路径数据引用
 */
const PathData& ReferenceLineInfo::path_data() const { return path_data_; }

/**
 * @brief 获取备用路径数据引用
 */
const PathData& ReferenceLineInfo::fallback_path_data() const {
  return fallback_path_data_;
}

/**
 * @brief 获取速度数据引用
 */
const SpeedData& ReferenceLineInfo::speed_data() const { return speed_data_; }

/**
 * @brief 获取可写路径数据指针
 */
PathData* ReferenceLineInfo::mutable_path_data() { return &path_data_; }

/**
 * @brief 获取可写备用路径数据指针
 */
PathData* ReferenceLineInfo::mutable_fallback_path_data() {
  return &fallback_path_data_;
}

/**
 * @brief 获取可写速度数据指针
 */
SpeedData* ReferenceLineInfo::mutable_speed_data() { return &speed_data_; }

/**
 * @brief 获取RSS信息引用
 */
const RSSInfo& ReferenceLineInfo::rss_info() const { return rss_info_; }

/**
 * @brief 获取可写RSS信息指针
 */
RSSInfo* ReferenceLineInfo::mutable_rss_info() { return &rss_info_; }

/**
 * @brief 合并路径和速度剖面生成轨迹
 *
 * 将路径数据和时间参数化的速度数据合并为完整的轨迹点序列。
 * 使用可变时间分辨率以减少数据量同时保证控制模块需求。
 *
 * @param relative_time 相对时间偏移
 * @param start_s 起始s坐标
 * @param ptr_discretized_trajectory 输出：合并后的轨迹
 * @return bool 是否成功
 *
 * 语法说明：
 * - ACHECK(ptr != nullptr): 断言检查非空
 * - kDenseTimeResolution: 密集采样间隔
 * - kSparseTimeResolution: 稀疏采样间隔
 * - kDenseTimeSec: 密集采样的时间长度
 * - path_data_.discretized_path().empty(): 检查路径是否为空
 * - speed_data_.EvaluateByTime(): 在指定时间评估速度
 * - ptr_discretized_trajectory->AppendTrajectoryPoint(): 添加轨迹点
 * - std::for_each(): STL算法遍历
 * - lambda表达式处理反向路径
 */
bool ReferenceLineInfo::CombinePathAndSpeedProfile(
    const double relative_time, const double start_s,
    DiscretizedTrajectory* ptr_discretized_trajectory) {
  ACHECK(ptr_discretized_trajectory != nullptr);  /**< 断言检查 */

  /**
   * 时间分辨率配置
   */
  const double kDenseTimeResoltuion = FLAGS_trajectory_time_min_interval;  /**< 密集间隔 */
  const double kSparseTimeResolution = FLAGS_trajectory_time_max_interval;  /**< 稀疏间隔 */
  const double kDenseTimeSec = FLAGS_trajectory_time_high_density_period;  /**< 密集采样时长 */

  if (path_data_.discretized_path().empty()) {
    AERROR << "path data is empty";
    return false;
  }

  if (speed_data_.empty()) {
    AERROR << "speed profile is empty";
    return false;
  }

  /**
   * 遍历速度剖面
   * for (double cur_rel_time = 0.0; cur_rel_time < speed_data_.TotalTime(); ...)
   *   - 初始值0.0，终止条件TotalTime()，增量根据时间段选择不同分辨率
   */
  for (double cur_rel_time = 0.0; cur_rel_time < speed_data_.TotalTime();
       cur_rel_time += (cur_rel_time < kDenseTimeSec ? kDenseTimeResoltuion
                                                     : kSparseTimeResolution)) {
    common::SpeedPoint speed_point;
    if (!speed_data_.EvaluateByTime(cur_rel_time, &speed_point)) {
      AERROR << "Fail to get speed point with relative time " << cur_rel_time;
      return false;
    }

    if (speed_point.s() > path_data_.discretized_path().Length()) {
      break;  /**< 超出路径长度 */
    }

    /**
     * 获取对应s坐标的路径点
     */
    common::PathPoint path_point =
        path_data_.GetPathPointWithPathS(speed_point.s());
    path_point.set_s(path_point.s() + start_s);  /**< 加上起始偏移 */

    /**
     * 构造轨迹点
     */
    common::TrajectoryPoint trajectory_point;
    trajectory_point.mutable_path_point()->CopyFrom(path_point);
    trajectory_point.set_v(speed_point.v());
    trajectory_point.set_a(speed_point.a());
    trajectory_point.set_relative_time(speed_point.t() + relative_time);
    ptr_discretized_trajectory->AppendTrajectoryPoint(trajectory_point);
  }

  /**
   * 处理反向路径
   */
  if (path_data_.is_reverse_path()) {
    std::for_each(ptr_discretized_trajectory->begin(),
                  ptr_discretized_trajectory->end(),
                  [](common::TrajectoryPoint& trajectory_point) {
                    trajectory_point.set_v(-trajectory_point.v());
                    trajectory_point.set_a(-trajectory_point.a());
                    trajectory_point.mutable_path_point()->set_s(
                        -trajectory_point.path_point().s());
                  });
    AINFO << "reversed path";
    ptr_discretized_trajectory->SetIsReversed(true);
  }
  return true;
}

/**
 * @brief 调整从当前位置开始的轨迹
 *
 * 将规划起点插入到已有轨迹中，并调整相对时间。
 *
 * @param planning_start_point 规划起始点
 * @param trajectory 原始轨迹
 * @param adjusted_trajectory 输出：调整后的轨迹
 * @return bool 是否成功
 *
 * 语法说明：
 * - std::atan2(y, x): 计算反正切
 * - common::math::AngleDiff(): 计算角度差
 * - cut_trajectory.erase(): 删除范围内的元素
 * - cut_trajectory.insert(): 在指定位置插入元素
 * - std::sqrt(): 平方根函数
 */
bool ReferenceLineInfo::AdjustTrajectoryWhichStartsFromCurrentPos(
    const common::TrajectoryPoint& planning_start_point,
    const std::vector<common::TrajectoryPoint>& trajectory,
    DiscretizedTrajectory* adjusted_trajectory) {
  ACHECK(adjusted_trajectory != nullptr);

  static constexpr double kMaxAngleDiff = M_PI_2;  /**< 最大角度差：90度 */
  const double start_point_heading = planning_start_point.path_point().theta();
  const double start_point_x = planning_start_point.path_point().x();
  const double start_point_y = planning_start_point.path_point().y();
  const double start_point_relative_time = planning_start_point.relative_time();

  /**
   * 查找插入索引
   */
  int insert_idx = -1;
  for (size_t i = 0; i < trajectory.size(); ++i) {
    if (trajectory[i].relative_time() <= start_point_relative_time) {
      continue;  /**< 跳过规划起点之前的点 */
    }

    const double cur_point_x = trajectory[i].path_point().x();
    const double cur_point_y = trajectory[i].path_point().y();
    const double tracking_heading =
        std::atan2(cur_point_y - start_point_y, cur_point_x - start_point_x);
    if (std::fabs(common::math::AngleDiff(start_point_heading,
                                          tracking_heading)) < kMaxAngleDiff) {
      insert_idx = i;  /**< 找到插入位置 */
      break;
    }
  }

  if (insert_idx == -1) {
    AERROR << "All points are behind of planning init point";
    return false;
  }

  /**
   * 剪切并插入规划起点
   */
  DiscretizedTrajectory cut_trajectory(trajectory);
  cut_trajectory.erase(cut_trajectory.begin(),
                       cut_trajectory.begin() + insert_idx);
  cut_trajectory.insert(cut_trajectory.begin(), planning_start_point);

  /**
   * 检查相对时间有效性
   */
  if (cut_trajectory.size() > 1 && cut_trajectory.front().relative_time() >=
                                       cut_trajectory[1].relative_time()) {
    AERROR << "planning init point relative_time["
           << cut_trajectory.front().relative_time()
           << "] larger than its next point's relative_time["
           << cut_trajectory[1].relative_time() << "]";
    return false;
  }

  /**
   * 重新计算累积s坐标
   */
  double accumulated_s = 0.0;
  for (size_t i = 1; i < cut_trajectory.size(); ++i) {
    const auto& pre_path_point = cut_trajectory[i - 1].path_point();
    auto* cur_path_point = cut_trajectory[i].mutable_path_point();
    accumulated_s += std::sqrt((cur_path_point->x() - pre_path_point.x()) *
                                   (cur_path_point->x() - pre_path_point.x()) +
                               (cur_path_point->y() - pre_path_point.y()) *
                                   (cur_path_point->y() - pre_path_point.y()));
    cur_path_point->set_s(accumulated_s);
  }

  /**
   * 重新生成轨迹，使用可变时间分辨率
   */
  adjusted_trajectory->clear();
  const double kDenseTimeResoltuion = FLAGS_trajectory_time_min_interval;
  const double kSparseTimeResolution = FLAGS_trajectory_time_max_interval;
  const double kDenseTimeSec = FLAGS_trajectory_time_high_density_period;
  for (double cur_rel_time = cut_trajectory.front().relative_time();
       cur_rel_time <= cut_trajectory.back().relative_time();
       cur_rel_time += (cur_rel_time < kDenseTimeSec ? kDenseTimeResoltuion
                                                     : kSparseTimeResolution)) {
    adjusted_trajectory->AppendTrajectoryPoint(
        cut_trajectory.Evaluate(cur_rel_time));
  }
  return true;
}

/**
 * @brief 设置可行驶标志
 */
void ReferenceLineInfo::SetDrivable(bool drivable) { is_drivable_ = drivable; }

/**
 * @brief 判断是否可行驶
 */
bool ReferenceLineInfo::IsDrivable() const { return is_drivable_; }

/**
 * @brief 判断是否为换道路径
 */
bool ReferenceLineInfo::IsChangeLanePath() const {
  return !Lanes().IsOnSegment();
}

/**
 * @brief 判断是否为相邻道路径
 */
bool ReferenceLineInfo::IsNeighborLanePath() const {
  return Lanes().IsNeighborSegment();
}

/**
 * @brief 获取路径速度调试字符串
 */
std::string ReferenceLineInfo::PathSpeedDebugString() const {
  return absl::StrCat("path_data:", path_data_.DebugString(),
                      "speed_data:", speed_data_.DebugString());
}

/**
 * @brief 判断自车是否在路由车道上
 */
bool ReferenceLineInfo::IsEgoOnRoutingLane() const {
  double ego_x = vehicle_state_.x();
  double ego_y = vehicle_state_.y();
  double lane_left_width = 0;
  double lane_right_width = 0;
  common::SLPoint sl_point;
  reference_line_.XYToSL({ego_x, ego_y}, &sl_point);
  reference_line_.GetLaneWidth(sl_point.s(), &lane_left_width,
                               &lane_right_width);
  ADEBUG << "s: " << sl_point.s() << " l: " << sl_point.l()
         << " lane_left_width: " << lane_left_width
         << " lane_right_width: " << lane_right_width;
  if (lane_left_width >= sl_point.l() && -lane_right_width <= sl_point.l()) {
    return true;
  } else {
    return false;
  }
}

/**
 * @brief 根据车道类型设置转向灯
 *
 * 根据车道变换、车道借用、车道转向类型设置转向灯信号。
 *
 * @param vehicle_signal 输出：车辆信号
 *
 * 语法说明：
 * - CHECK_NOTNULL(ptr): 断言检查非空
 * - routing::ChangeLaneType::LEFT/RIGHT: 路由变道类型
 * - path_data_.path_label().find("left"): 查找路径标签
 * - std::string::npos: 字符串未找到的返回值
 */
void ReferenceLineInfo::SetTurnSignalBasedOnLaneTurnType(
    common::VehicleSignal* vehicle_signal) const {
  CHECK_NOTNULL(vehicle_signal);

  /**
   * 如果已有转向灯信号，直接返回
   */
  if (vehicle_signal->has_turn_signal() &&
      vehicle_signal->turn_signal() != VehicleSignal::TURN_NONE) {
    return;
  }
  vehicle_signal->set_turn_signal(VehicleSignal::TURN_NONE);

  /**
   * 根据换道类型设置
   */
  if (IsChangeLanePath()) {
    if (Lanes().PreviousAction() == routing::ChangeLaneType::LEFT) {
      vehicle_signal->set_turn_signal(VehicleSignal::TURN_LEFT);
    } else if (Lanes().PreviousAction() == routing::ChangeLaneType::RIGHT) {
      vehicle_signal->set_turn_signal(VehicleSignal::TURN_RIGHT);
    }
    return;
  }

  /**
   * 根据车道借用类型设置
   */
  bool is_ego_on_routing_lane = IsEgoOnRoutingLane();
  if (path_data_.path_label().find("left") != std::string::npos &&
      is_ego_on_routing_lane) {
    vehicle_signal->set_turn_signal(VehicleSignal::TURN_LEFT);
    AINFO << "Set turn signal to left";
    return;
  }
  if (path_data_.path_label().find("left") != std::string::npos &&
      !is_ego_on_routing_lane) {
    vehicle_signal->set_turn_signal(VehicleSignal::TURN_RIGHT);
    AINFO << "Set turn signal to right";
    return;
  }
  if (path_data_.path_label().find("right") != std::string::npos &&
      is_ego_on_routing_lane) {
    vehicle_signal->set_turn_signal(VehicleSignal::TURN_RIGHT);
    AINFO << "Set turn signal to right";
    return;
  }
  if (path_data_.path_label().find("right") != std::string::npos &&
      !is_ego_on_routing_lane) {
    vehicle_signal->set_turn_signal(VehicleSignal::TURN_LEFT);
    AINFO << "Set turn signal to left";
    return;
  }

  /**
   * 根据车道转向类型设置
   */
  double route_s = 0.0;
  const double adc_s = adc_sl_boundary_.end_s();
  for (const auto& seg : Lanes()) {
    if (route_s > adc_s + FLAGS_turn_signal_distance) {
      break;
    }
    route_s += seg.end_s - seg.start_s;
    if (route_s < adc_s) {
      continue;
    }
    const auto& turn = seg.lane->lane().turn();
    if (turn == hdmap::Lane::LEFT_TURN) {
      vehicle_signal->set_turn_signal(VehicleSignal::TURN_LEFT);
      AINFO << "Set turn signal to left";
      break;
    } else if (turn == hdmap::Lane::RIGHT_TURN) {
      vehicle_signal->set_turn_signal(VehicleSignal::TURN_RIGHT);
      AINFO << "Set turn signal to right";
      break;
    } else if (turn == hdmap::Lane::U_TURN) {
      /**
       * U型转弯需要根据几何形状判断左右
       * 使用叉积判断方向
       */
      auto start_xy =
          PointFactory::ToVec2d(seg.lane->GetSmoothPoint(seg.start_s));
      auto middle_xy = PointFactory::ToVec2d(
          seg.lane->GetSmoothPoint((seg.start_s + seg.end_s) / 2.0));
      auto end_xy = PointFactory::ToVec2d(seg.lane->GetSmoothPoint(seg.end_s));
      auto start_to_middle = middle_xy - start_xy;
      auto start_to_end = end_xy - start_xy;
      if (start_to_middle.CrossProd(start_to_end) < 0) {
        vehicle_signal->set_turn_signal(VehicleSignal::TURN_RIGHT);
      } else {
        vehicle_signal->set_turn_signal(VehicleSignal::TURN_LEFT);
      }
      break;
    }
  }
}

/**
 * @brief 设置转向灯信号
 */
void ReferenceLineInfo::SetTurnSignal(
    const VehicleSignal::TurnSignal& turn_signal) {
  vehicle_signal_.set_turn_signal(turn_signal);
}

/**
 * @brief 设置紧急灯
 */
void ReferenceLineInfo::SetEmergencyLight() {
  vehicle_signal_.set_emergency_light(true);
}

/**
 * @brief 导出车辆信号
 */
void ReferenceLineInfo::ExportVehicleSignal(
    common::VehicleSignal* vehicle_signal) const {
  CHECK_NOTNULL(vehicle_signal);
  *vehicle_signal = vehicle_signal_;
  SetTurnSignalBasedOnLaneTurnType(vehicle_signal);
}

/**
 * @brief 判断是否到达目的地
 */
bool ReferenceLineInfo::ReachedDestination() const {
  const double distance_destination = SDistanceToDestination();
  const double distance_ref_end = SDistanceToRefEnd();
  AINFO << "distance_destination:" << distance_destination
        << "distance_ref_end: " << distance_ref_end;
  return distance_destination <= FLAGS_passed_destination_threshold ||
         distance_ref_end <= FLAGS_passed_referenceline_end_threshold;
}

/**
 * @brief 计算到目的地的距离
 *
 * @return double 到目的地的距离
 *
 * 语法说明：
 * - std::numeric_limits<double>::max(): double最大值作为初始值
 * - path_decision_.Find(): 在决策中查找障碍物
 * - dest_ptr->LongitudinalDecision().has_stop(): 检查是否有停车决策
 */
double ReferenceLineInfo::SDistanceToDestination() const {
  double res = std::numeric_limits<double>::max();
  const auto* dest_ptr = path_decision_.Find(FLAGS_destination_obstacle_id);
  if (!dest_ptr) {
    return res;
  }
  if (!dest_ptr->LongitudinalDecision().has_stop()) {
    return res;
  }
  if (!reference_line_.IsOnLane(dest_ptr->PerceptionBoundingBox().center())) {
    return res;
  }
  const double stop_s = dest_ptr->PerceptionSLBoundary().start_s() +
                        dest_ptr->LongitudinalDecision().stop().distance_s();
  AINFO << "stop_s: " << stop_s << "end_s: " << adc_sl_boundary_.end_s();
  return stop_s - adc_sl_boundary_.end_s();
}

/**
 * @brief 计算到参考线终点的距离
 */
double ReferenceLineInfo::SDistanceToRefEnd() const {
  double res = std::numeric_limits<double>::max();

  std::string ref_end_id;
  for (const auto* obstacle : path_decision_.obstacles().Items()) {
    std::string id = obstacle->Id();
    if (id.find("REF_END") != std::string::npos) {
      ref_end_id = id;
      AINFO << "Found reference line end id: " << ref_end_id;
    } else {
      continue;
    }
  }
  if (!ref_end_id.empty()) {
    AINFO << "REF_END: " << ref_end_id;
    const auto* ref_end_ptr = path_decision_.Find(ref_end_id);
    AINFO << "ref_end_ptr:" << ref_end_ptr->DebugString();
    if (!ref_end_ptr && !ref_end_ptr->LongitudinalDecision().has_stop() &&
        !reference_line_.IsOnLane(
            ref_end_ptr->PerceptionBoundingBox().center())) {
      const double stop_s =
          ref_end_ptr->PerceptionSLBoundary().start_s() +
          ref_end_ptr->LongitudinalDecision().stop().distance_s();
      AINFO << "REF_END: stop_s: " << stop_s
            << "end_s: " << adc_sl_boundary_.end_s();
      res = stop_s - adc_sl_boundary_.end_s();
    }
  }
  return res;
}

/**
 * @brief 导出决策结果
 *
 * @param decision_result 输出：决策结果
 * @param planning_context 规划上下文
 */
void ReferenceLineInfo::ExportDecision(
    DecisionResult* decision_result, PlanningContext* planning_context) const {
  MakeDecision(decision_result, planning_context);
  ExportVehicleSignal(decision_result->mutable_vehicle_signal());
  auto* main_decision = decision_result->mutable_main_decision();
  if (main_decision->has_stop()) {
    main_decision->mutable_stop()->set_change_lane_type(
        Lanes().PreviousAction());
  } else if (main_decision->has_cruise()) {
    main_decision->mutable_cruise()->set_change_lane_type(
        Lanes().PreviousAction());
  }
}

/**
 * @brief 做出决策
 *
 * 决策流程：
 * 1. 默认巡航
 * 2. 检查停车决策
 * 3. 如有错误，做紧急停车决策
 * 4. 任务完成决策
 * 5. 设置障碍物决策
 */
void ReferenceLineInfo::MakeDecision(DecisionResult* decision_result,
                                     PlanningContext* planning_context) const {
  CHECK_NOTNULL(decision_result);
  decision_result->Clear();

  /**
   * 默认巡航决策
   */
  decision_result->mutable_main_decision()->mutable_cruise();

  /**
   * 检查停车决策
   */
  int error_code = MakeMainStopDecision(decision_result);
  if (error_code < 0) {
    MakeEStopDecision(decision_result);
  }
  MakeMainMissionCompleteDecision(decision_result, planning_context);
  SetObjectDecisions(decision_result->mutable_object_decision());
}

/**
 * @brief 任务完成决策
 */
void ReferenceLineInfo::MakeMainMissionCompleteDecision(
    DecisionResult* decision_result, PlanningContext* planning_context) const {
  if (!decision_result->main_decision().has_stop()) {
    return;
  }
  auto main_stop = decision_result->main_decision().stop();
  if (main_stop.reason_code() != STOP_REASON_DESTINATION &&
      main_stop.reason_code() != STOP_REASON_PULL_OVER &&
      main_stop.reason_code() != STOP_REASON_REFERENCE_END) {
    return;
  }
  const double distance_destination = SDistanceToDestination();
  const double distance_to_reference_end = SDistanceToRefEnd();
  if (distance_destination > FLAGS_destination_check_distance &&
      distance_to_reference_end > FLAGS_destination_check_distance) {
    return;
  }

  auto mission_complete =
      decision_result->mutable_main_decision()->mutable_mission_complete();
  if (ReachedDestination()) {
    planning_context->mutable_planning_status()
        ->mutable_destination()
        ->set_has_passed_destination(true);
  } else {
    mission_complete->mutable_stop_point()->CopyFrom(main_stop.stop_point());
    mission_complete->set_stop_heading(main_stop.stop_heading());
  }
}

/**
 * @brief 主要停车决策
 *
 * 遍历所有障碍物，找到最近的停车决策。
 *
 * @param decision_result 输出：决策结果
 * @return int 1表示有停车决策，0表示无，负数表示错误
 */
int ReferenceLineInfo::MakeMainStopDecision(
    DecisionResult* decision_result) const {
  double min_stop_line_s = std::numeric_limits<double>::infinity();
  const Obstacle* stop_obstacle = nullptr;
  const ObjectStop* stop_decision = nullptr;

  for (const auto* obstacle : path_decision_.obstacles().Items()) {
    const auto& object_decision = obstacle->LongitudinalDecision();
    if (!object_decision.has_stop()) {
      continue;
    }

    apollo::common::PointENU stop_point = object_decision.stop().stop_point();
    common::SLPoint stop_line_sl;
    reference_line_.XYToSL(stop_point, &stop_line_sl);

    double stop_line_s = stop_line_sl.s();
    if (stop_line_s < 0 || stop_line_s > reference_line_.Length()) {
      AERROR << "Ignore object:" << obstacle->Id() << " fence route_s["
             << stop_line_s << "] not in range[0, " << reference_line_.Length()
             << "]";
      continue;
    }

    if (stop_line_s < min_stop_line_s) {
      min_stop_line_s = stop_line_s;
      stop_obstacle = obstacle;
      stop_decision = &(object_decision.stop());
    }
  }

  if (stop_obstacle != nullptr) {
    MainStop* main_stop =
        decision_result->mutable_main_decision()->mutable_stop();
    main_stop->set_reason_code(stop_decision->reason_code());
    main_stop->set_reason("stop by " + stop_obstacle->Id());
    main_stop->mutable_stop_point()->set_x(stop_decision->stop_point().x());
    main_stop->mutable_stop_point()->set_y(stop_decision->stop_point().y());
    main_stop->set_stop_heading(stop_decision->stop_heading());
    ADEBUG << " main stop obstacle id:" << stop_obstacle->Id()
           << " stop_line_s:" << min_stop_line_s << " stop_point: ("
           << stop_decision->stop_point().x() << stop_decision->stop_point().y()
           << " ) stop_heading: " << stop_decision->stop_heading();
    return 1;
  }
  return 0;
}

/**
 * @brief 设置障碍物决策
 */
void ReferenceLineInfo::SetObjectDecisions(
    ObjectDecisions* object_decisions) const {
  for (const auto obstacle : path_decision_.obstacles().Items()) {
    if (!obstacle->HasNonIgnoreDecision()) {
      continue;
    }
    auto* object_decision = object_decisions->add_decision();
    object_decision->set_id(obstacle->Id());
    object_decision->set_perception_id(obstacle->PerceptionId());
    if (obstacle->HasLateralDecision() && !obstacle->IsLateralIgnore()) {
      object_decision->add_object_decision()->CopyFrom(
          obstacle->LateralDecision());
    }
    if (obstacle->HasLongitudinalDecision() &&
        !obstacle->IsLongitudinalIgnore()) {
      object_decision->add_object_decision()->CopyFrom(
          obstacle->LongitudinalDecision());
    }
  }
}

/**
 * @brief 导出驾驶建议
 *
 * @param engage_advice 输出：驾驶建议
 * @param planning_context 规划上下文
 *
 * 语法说明：
 * - static EngageAdvice prev_advice: 静态变量保持上次建议
 * - EngageAdvice::READY_TO_ENGAGE: 准备接管
 * - EngageAdvice::KEEP_ENGAGED: 保持接管
 * - EngageAdvice::PREPARE_DISENGAGE: 准备脱离
 * - EngageAdvice::DISALLOW_ENGAGE: 不允许接管
 */
void ReferenceLineInfo::ExportEngageAdvice(
    EngageAdvice* engage_advice, PlanningContext* planning_context) const {
  static EngageAdvice prev_advice;
  static constexpr double kMaxAngleDiff = M_PI / 6.0;  /**< 最大角度差：30度 */

  bool engage = false;
  if (!IsDrivable()) {
    prev_advice.set_reason("Reference line not drivable");
  } else if (!is_on_reference_line_) {
    const auto& scenario_type =
        planning_context->planning_status().scenario().scenario_type();
    if (scenario_type == "PARK_AND_GO" || IsChangeLanePath()) {
      engage = true;
    } else {
      prev_advice.set_reason("Not on reference line");
    }
  } else {
    /**
     * 检查航向对齐
     */
    auto ref_point =
        reference_line_.GetReferencePoint(adc_sl_boundary_.end_s());
    if (common::math::AngleDiff(vehicle_state_.heading(), ref_point.heading()) <
        kMaxAngleDiff) {
      engage = true;
    } else {
      prev_advice.set_reason("Vehicle heading is not aligned");
    }
  }

  if (engage) {
    if (vehicle_state_.driving_mode() !=
        Chassis::DrivingMode::Chassis_DrivingMode_COMPLETE_AUTO_DRIVE) {
      prev_advice.set_advice(EngageAdvice::READY_TO_ENGAGE);
    } else {
      prev_advice.set_advice(EngageAdvice::KEEP_ENGAGED);
    }
    prev_advice.clear_reason();
  } else {
    if (prev_advice.advice() != EngageAdvice::DISALLOW_ENGAGE) {
      prev_advice.set_advice(EngageAdvice::PREPARE_DISENGAGE);
    }
  }
  engage_advice->CopyFrom(prev_advice);
}

/**
 * @brief 紧急停车决策
 */
void ReferenceLineInfo::MakeEStopDecision(
    DecisionResult* decision_result) const {
  decision_result->Clear();

  MainEmergencyStop* main_estop =
      decision_result->mutable_main_decision()->mutable_estop();
  main_estop->set_reason_code(MainEmergencyStop::ESTOP_REASON_INTERNAL_ERR);
  main_estop->set_reason("estop reason to be added");
  main_estop->mutable_cruise_to_stop();

  ObjectDecisions* object_decisions =
      decision_result->mutable_object_decision();
  for (const auto obstacle : path_decision_.obstacles().Items()) {
    auto* object_decision = object_decisions->add_decision();
    object_decision->set_id(obstacle->Id());
    object_decision->set_perception_id(obstacle->PerceptionId());
    object_decision->add_object_decision()->mutable_avoid();
  }
}

/**
 * @brief 获取路径转向类型
 *
 * @param s 沿参考线的距离
 * @return hdmap::Lane::LaneTurn 转向类型
 */
hdmap::Lane::LaneTurn ReferenceLineInfo::GetPathTurnType(const double s) const {
  const double forward_buffer = 20.0;
  double route_s = 0.0;
  for (const auto& seg : Lanes()) {
    if (route_s > s + forward_buffer) {
      break;
    }
    route_s += seg.end_s - seg.start_s;
    if (route_s < s) {
      continue;
    }
    const auto& turn_type = seg.lane->lane().turn();
    if (turn_type == hdmap::Lane::LEFT_TURN ||
        turn_type == hdmap::Lane::RIGHT_TURN ||
        turn_type == hdmap::Lane::U_TURN) {
      return turn_type;
    }
  }
  return hdmap::Lane::NO_TURN;
}

/**
 * @brief 获取路口通行权状态
 */
bool ReferenceLineInfo::GetIntersectionRightofWayStatus(
    const hdmap::PathOverlap& pnc_junction_overlap) const {
  if (GetPathTurnType(pnc_junction_overlap.start_s) != hdmap::Lane::NO_TURN) {
    return false;
  }
  return true;
}

/**
 * @brief 获取PNC路口
 */
int ReferenceLineInfo::GetPnCJunction(
    const double s, hdmap::PathOverlap* pnc_junction_overlap) const {
  CHECK_NOTNULL(pnc_junction_overlap);
  const std::vector<hdmap::PathOverlap>& pnc_junction_overlaps =
      reference_line_.map_path().pnc_junction_overlaps();

  static constexpr double kError = 1.0;  // 米
  for (const auto& overlap : pnc_junction_overlaps) {
    if (s >= overlap.start_s - kError && s <= overlap.end_s + kError) {
      *pnc_junction_overlap = overlap;
      return 1;
    }
  }
  return 0;
}

/**
 * @brief 获取路口
 */
int ReferenceLineInfo::GetJunction(const double s,
                                   hdmap::PathOverlap* junction_overlap) const {
  CHECK_NOTNULL(junction_overlap);
  const std::vector<hdmap::PathOverlap>& junction_overlaps =
      reference_line_.map_path().junction_overlaps();

  static constexpr double kError = 1.0;  // 米
  for (const auto& overlap : junction_overlaps) {
    if (s >= overlap.start_s - kError && s <= overlap.end_s + kError) {
      *junction_overlap = overlap;
      return 1;
    }
  }
  return 0;
}

/**
 * @brief 获取区域
 */
int ReferenceLineInfo::GetArea(const double s,
                               hdmap::PathOverlap* area_overlap) const {
  CHECK_NOTNULL(area_overlap);
  const std::vector<hdmap::PathOverlap>& area_overlaps =
      reference_line_.map_path().area_overlaps();

  static constexpr double kError = 1.0;  // 米
  for (const auto& overlap : area_overlaps) {
    if (s >= overlap.start_s - kError && s <= overlap.end_s + kError) {
      *area_overlap = overlap;
      return 1;
    }
  }
  return 0;
}

/**
 * @brief 设置阻塞障碍物
 */
void ReferenceLineInfo::SetBlockingObstacle(
    const std::string& blocking_obstacle_id) {
  blocking_obstacle_ = path_decision_.Find(blocking_obstacle_id);
}

/**
 * @brief 获取所有停车决策的SL点
 */
std::vector<common::SLPoint> ReferenceLineInfo::GetAllStopDecisionSLPoint()
    const {
  std::vector<common::SLPoint> result;
  for (const auto* obstacle : path_decision_.obstacles().Items()) {
    const auto& object_decision = obstacle->LongitudinalDecision();
    if (!object_decision.has_stop()) {
      continue;
    }
    apollo::common::PointENU stop_point = object_decision.stop().stop_point();
    common::SLPoint stop_line_sl;
    reference_line_.XYToSL(stop_point, &stop_line_sl);
    if (stop_line_sl.s() <= 0 || stop_line_sl.s() >= reference_line_.Length()) {
      continue;
    }
    result.push_back(stop_line_sl);
  }

  /**
   * 按s坐标排序
   */
  if (!result.empty()) {
    std::sort(result.begin(), result.end(),
              [](const common::SLPoint& a, const common::SLPoint& b) {
                return a.s() < b.s();
              });
  }
  return result;
}

/**
 * @brief 获取参考线上的重叠区域
 */
hdmap::PathOverlap* ReferenceLineInfo::GetOverlapOnReferenceLine(
    const std::string& overlap_id, const OverlapType& overlap_type) const {
  if (overlap_type == ReferenceLineInfo::SIGNAL) {
    const auto& traffic_light_overlaps =
        reference_line_.map_path().signal_overlaps();
    for (const auto& traffic_light_overlap : traffic_light_overlaps) {
      if (traffic_light_overlap.object_id == overlap_id) {
        return const_cast<hdmap::PathOverlap*>(&traffic_light_overlap);
      }
    }
  } else if (overlap_type == ReferenceLineInfo::STOP_SIGN) {
    const auto& stop_sign_overlaps =
        reference_line_.map_path().stop_sign_overlaps();
    for (const auto& stop_sign_overlap : stop_sign_overlaps) {
      if (stop_sign_overlap.object_id == overlap_id) {
        return const_cast<hdmap::PathOverlap*>(&stop_sign_overlap);
      }
    }
  } else if (overlap_type == ReferenceLineInfo::PNC_JUNCTION) {
    const auto& pnc_junction_overlaps =
        reference_line_.map_path().pnc_junction_overlaps();
    for (const auto& pnc_junction_overlap : pnc_junction_overlaps) {
      if (pnc_junction_overlap.object_id == overlap_id) {
        return const_cast<hdmap::PathOverlap*>(&pnc_junction_overlap);
      }
    }
  } else if (overlap_type == ReferenceLineInfo::YIELD_SIGN) {
    const auto& yield_sign_overlaps =
        reference_line_.map_path().yield_sign_overlaps();
    for (const auto& yield_sign_overlap : yield_sign_overlaps) {
      if (yield_sign_overlap.object_id == overlap_id) {
        return const_cast<hdmap::PathOverlap*>(&yield_sign_overlap);
      }
    }
  } else if (overlap_type == ReferenceLineInfo::JUNCTION) {
    const auto& junction_overlaps =
        reference_line_.map_path().junction_overlaps();
    for (const auto& junction_overlap : junction_overlaps) {
      if (junction_overlap.object_id == overlap_id) {
        return const_cast<hdmap::PathOverlap*>(&junction_overlap);
      }
    }
  } else if (overlap_type == ReferenceLineInfo::AREA) {
    const auto& area_overlaps = reference_line_.map_path().area_overlaps();
    for (const auto& area_overlap : area_overlaps) {
      if (area_overlap.object_id == overlap_id) {
        return const_cast<hdmap::PathOverlap*>(&area_overlap);
      }
    }
  }
  return nullptr;
}

/**
 * @brief 获取可写候选路径数据指针
 */
std::vector<PathData>* ReferenceLineInfo::MutableCandidatePathData() {
  return &candidate_path_data_;
}

/**
 * @brief 获取指定范围内的重叠区域
 */
void ReferenceLineInfo::GetRangeOverlaps(
    std::vector<hdmap::PathOverlap>* path_overlaps, double start_s,
    double end_s) {
  CHECK_NOTNULL(path_overlaps);
  const std::vector<hdmap::PathOverlap>& junction_overlaps =
      reference_line_.map_path().junction_overlaps();

  static constexpr double kError = 0.1;  // 米
  AINFO << "GetRangeOverlaps start_s: " << start_s << ", end_s: " << end_s;
  for (const auto& overlap : junction_overlaps) {
    if (start_s >= overlap.end_s - kError) {
      continue;
    } else if (end_s < overlap.start_s - kError) {
      break;
    } else {
      AINFO << "overlap emplace_back " << overlap.start_s << ", "
            << overlap.end_s << " ]";
      path_overlaps->emplace_back(overlap);
    }
  }
}

/**
 * @brief 获取参考线拖曳l值引用
 */
const std::vector<double>& ReferenceLineInfo::reference_line_towing_l() const {
  return reference_line_towing_l_;
}

/**
 * @brief 获取可写参考线拖曳l值指针
 */
std::vector<double>* ReferenceLineInfo::mutable_reference_line_towing_l() {
  return &reference_line_towing_l_;
}

/**
 * @brief 获取参考线拖曳路径边界引用
 */
const PathBoundary& ReferenceLineInfo::reference_line_towing_path_boundary()
    const {
  return reference_line_towing_path_boundary_;
}

/**
 * @brief 获取可写参考线拖曳路径边界指针
 */
PathBoundary* ReferenceLineInfo::mutable_reference_line_towing_path_boundary() {
  return &reference_line_towing_path_boundary_;
}

/**
 * @brief 获取障碍物SL多边形列表引用
 */
const std::vector<SLPolygon>& ReferenceLineInfo::obs_sl_polygons() const {
  return obs_sl_polygons_;
}

/**
 * @brief 获取可写障碍物SL多边形列表指针
 */
std::vector<SLPolygon>* ReferenceLineInfo::mutable_obs_sl_polygons() {
  return &obs_sl_polygons_;
}

/**
 * @brief 打印参考段调试信息
 */
void ReferenceLineInfo::PrintReferenceSegmentDebugString() {
  PrintCurves print_curve;
  const auto& lane_segments = reference_line_.GetMapPath().lane_segments();
  for (size_t i = 0; i < lane_segments.size(); ++i) {
    for (const auto& seg :
         lane_segments.at(i).lane->lane().left_boundary().curve().segment()) {
      for (const auto& pt : seg.line_segment().point()) {
        print_curve.AddPoint(std::to_string(index_) + "_left_pt_print", pt.x(),
                             pt.y());
      }
    }
    for (const auto& seg :
         lane_segments.at(i).lane->lane().right_boundary().curve().segment()) {
      for (const auto& pt : seg.line_segment().point()) {
        print_curve.AddPoint(std::to_string(index_) + "_right_pt_print", pt.x(),
                             pt.y());
      }
    }
    for (const auto& pt : lane_segments.at(i).lane->points()) {
      print_curve.AddPoint(std::to_string(index_) + "_center_pt_print", pt.x(),
                           pt.y());
    }
  }
  print_curve.PrintToLog();
}

}  // namespace planning
}  // namespace apollo
