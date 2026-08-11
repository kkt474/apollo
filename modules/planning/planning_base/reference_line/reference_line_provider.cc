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
 * @file reference_line_provider.cc
 * @brief 参考线提供者实现文件
 *
 * 本文件实现了ReferenceLineProvider类，负责：
 * 1. 根据路由模块的指令生成和管理参考线（ReferenceLine）
 * 2. 提供参考线的平滑、缝合、裁剪等功能
 * 3. 支持导航模式和车道线模式两种工作模式
 * 4. 管理参考线的历史记录
 *
 * 核心概念：
 * - ReferenceLine（参考线）：由一系列参考点组成，代表车辆的理想行驶路径
 * - RouteSegments（路由段）：路由模块返回的路径段信息
 * - PncMap：路径规划和控制地图接口
 * - SL坐标系：沿车道纵向(S)和横向(L)的坐标系统
 *
 * 语法说明：
 * - std::list<T>: 双向链表容器，适合频繁的插入删除操作
 * - std::lock_guard<std::mutex>: RAII风格的互斥锁
 * - std::chrono::milliseconds: 时间duration类型
 * - std::unordered_set<T>: 无序哈希集合，用于快速查找
 * - cyber::Async(): Cyber RT的异步任务调度
 */

#include "modules/planning/planning_base/reference_line/reference_line_provider.h"

/**
 * #include <algorithm> - C++标准库算法头文件
 * 提供std::min, std::max, std::copy, std::copy_if, std::sort等算法函数
 */
#include <algorithm>

/**
 * #include <limits> - C++标准库数值极限头文件
 * 提供std::numeric_limits<T>用于获取类型的数值范围
 */
#include <limits>

/**
 * #include <utility> - C++标准库工具头文件
 * 提供std::pair等工具
 */
#include <utility>

/**
 * cyber/common/file.h - Cyber RT框架的文件操作工具
 * GetProtoFromFile用于从文件加载Protobuf配置
 */
#include "cyber/common/file.h"

/**
 * cyber/plugin_manager/plugin_manager.h - Cyber RT插件管理器
 * 用于动态加载和创建PncMap插件
 */
#include "cyber/plugin_manager/plugin_manager.h"

/**
 * cyber/task/task.h - Cyber RT异步任务头文件
 * cyber::Async用于创建异步任务
 */
#include "cyber/task/task.h"

/**
 * cyber/time/clock.h - Cyber RT时钟系统
 * Clock::NowInSeconds()获取当前时间戳
 */
#include "cyber/time/clock.h"

/**
 * modules/common/configs/vehicle_config_helper.h
 * 车辆配置辅助类，提供访问车辆参数（如宽度）的接口
 */
#include "modules/common/configs/vehicle_config_helper.h"

/**
 * modules/common/math/math_utils.h
 * 数学工具函数，如AngleDiff计算角度差
 */
#include "modules/common/math/math_utils.h"

/**
 * modules/common/util/point_factory.h
 * 点工厂类，用于创建不同类型的点（MapPoint, Vec2d等）
 */
#include "modules/common/util/point_factory.h"

/**
 * modules/common/vehicle_state/vehicle_state_provider.h
 * 车辆状态提供者类
 */
#include "modules/common/vehicle_state/vehicle_state_provider.h"

/**
 * modules/map/hdmap/hdmap_util.h
 * 高精地图工具类
 */
#include "modules/map/hdmap/hdmap_util.h"

/**
 * modules/map/pnc_map/path.h
 * PncMap路径相关类
 */
#include "modules/map/pnc_map/path.h"

/**
 * modules/planning/planning_base/common/planning_context.h
 * 规划上下文，存储全局规划状态
 */
#include "modules/planning/planning_base/common/planning_context.h"

/**
 * modules/planning/planning_base/gflags/planning_gflags.h
 * 规划模块的gflags配置
 */
#include "modules/planning/planning_base/gflags/planning_gflags.h"

/**
 * @namespace apollo::planning
 * @brief Apollo规划模块命名空间
 */
namespace apollo {
namespace planning {

/**
 * using声明 - 将其他命名空间的类型引入当前作用域
 */

/**
 * VehicleConfigHelper - 车辆配置辅助类
 * 提供访问车辆参数（长、宽、高等）的接口
 */
using apollo::common::VehicleConfigHelper;

/**
 * VehicleState - 车辆状态结构
 * 包含位置、速度、航向角等信息
 */
using apollo::common::VehicleState;

/**
 * AngleDiff - 计算两个角度之间的差异
 * 返回值范围[-PI, PI]
 */
using apollo::common::math::AngleDiff;

/**
 * Vec2d - 二维向量类
 * 用于表示二维平面坐标
 */
using apollo::common::math::Vec2d;

/**
 * Clock - Cyber RT时钟类
 * Clock::NowInSeconds()获取当前时间
 */
using apollo::cyber::Clock;

/**
 * HDMapUtil - 高精地图工具类
 * HDMapUtil::BaseMapPtr()获取地图指针
 */
using apollo::hdmap::HDMapUtil;

/**
 * LaneWaypoint - 车道上的一个路点
 * 包含所在车道、沿车道距离s等信息
 */
using apollo::hdmap::LaneWaypoint;

/**
 * MapPathPoint - 地图路径点
 * 包含位置、航向角等信息
 */
using apollo::hdmap::MapPathPoint;

/**
 * RouteSegments - 路由段列表
 * 代表一段连续的行驶路径
 */
using apollo::hdmap::RouteSegments;

/**
 * @brief 参考线提供者析构函数
 *
 * 析构时清理资源，确保线程安全停止
 */
ReferenceLineProvider::~ReferenceLineProvider() {}

/**
 * @brief 参考线提供者构造函数
 *
 * 初始化参考线提供者的各种组件：
 * 1. 加载平滑器配置（QP样条/螺旋线/离散点）
 * 2. 加载PncMap插件
 * 3. 初始化相对地图（如果是导航模式）
 *
 * @param vehicle_state_provider 车辆状态提供者指针
 * @param reference_line_config 参考线配置指针
 * @param relative_map 相对地图消息指针（导航模式使用）
 *
 * 语法说明：
 * - : vehicle_state_provider_(vehicle_state_provider)
 *   构造函数初始化列表，直接初始化成员变量
 * - std::shared_ptr<T>: 共享指针，用于共享所有权
 * - smoother_.reset(new Xxx): 释放旧平滑器并创建新的
 */
ReferenceLineProvider::ReferenceLineProvider(
    const common::VehicleStateProvider *vehicle_state_provider,
    const ReferenceLineConfig *reference_line_config,
    const std::shared_ptr<relative_map::MapMsg> &relative_map)
    : vehicle_state_provider_(vehicle_state_provider) {

  /**
   * current_pnc_map_初始化为空
   * 将在UpdatePlanningCommand时设置
   */
  current_pnc_map_ = nullptr;

  /**
   * 根据FLAGS_use_navigation_mode决定是否使用相对地图
   * 非导航模式：relative_map_设为nullptr，使用高清地图
   * 导航模式：使用传入的relative_map
   */
  if (!FLAGS_use_navigation_mode) {
    relative_map_ = nullptr;
  } else {
    relative_map_ = relative_map;
  }

  /**
   * 加载平滑器配置
   * GetProtoFromFile从配置文件加载Protobuf消息
   * FLAGS_smoother_config_filename指定配置文件路径
   */
  ACHECK(cyber::common::GetProtoFromFile(FLAGS_smoother_config_filename,
                                         &smoother_config_))
      << "Failed to load smoother config file "
      << FLAGS_smoother_config_filename;

  /**
   * 根据配置选择平滑器类型：
   * - qp_spline: QP样条平滑器（QpSplineReferenceLineSmoother）
   * - spiral: 螺旋线平滑器（SpiralReferenceLineSmoother）
   * - discrete_points: 离散点平滑器（DiscretePointsReferenceLineSmoother）
   *
   * smoother_.reset(new Xxx)创建新的平滑器实例
   * reset会释放原有对象（如果有）
   */
  if (smoother_config_.has_qp_spline()) {
    smoother_.reset(new QpSplineReferenceLineSmoother(smoother_config_));
  } else if (smoother_config_.has_spiral()) {
    smoother_.reset(new SpiralReferenceLineSmoother(smoother_config_));
  } else if (smoother_config_.has_discrete_points()) {
    smoother_.reset(new DiscretePointsReferenceLineSmoother(smoother_config_));
  } else {
    ACHECK(false) << "unknown smoother config "
                  << smoother_config_.DebugString();
  }

  /**
   * 加载PncMap插件
   * pnc_map_list_存储可用的PncMap插件列表
   */
  // Load pnc map plugins.
  pnc_map_list_.clear();

  /**
   * 如果配置为空或没有指定pnc_map_class，使用默认的LaneFollowMap
   * PluginManager::Instance()获取插件管理器单例
   * CreateInstance<T>(plugin_name)根据插件名称创建实例
   */
  // Set "apollo::planning::LaneFollowMap" as default if pnc_map_class is empty.
  if (nullptr == reference_line_config ||
      reference_line_config->pnc_map_class().empty()) {
    const auto &pnc_map =
        apollo::cyber::plugin_manager::PluginManager::Instance()
            ->CreateInstance<planning::PncMapBase>(
                "apollo::planning::LaneFollowMap");
    pnc_map_list_.emplace_back(pnc_map);
  } else {
    /**
     * 遍历配置中的pnc_map_class列表
     * 为每个名称创建对应的插件实例
     */
    const auto &pnc_map_names = reference_line_config->pnc_map_class();
    for (const auto &map_name : pnc_map_names) {
      const auto &pnc_map =
          apollo::cyber::plugin_manager::PluginManager::Instance()
              ->CreateInstance<planning::PncMapBase>(map_name);
      pnc_map_list_.emplace_back(pnc_map);
    }
  }

  /**
   * is_initialized_标志设为true，表示初始化完成
   */
  is_initialized_ = true;
}

/**
 * @brief 更新规划命令
 *
 * 当接收到新的路由请求时调用此函数
 * 查找能够处理该命令的PncMap并更新路由信息
 *
 * @param command 规划命令（包含路由请求）
 * @return bool 更新是否成功
 *
 * 语法说明：
 * - std::lock_guard<std::mutex>: RAII锁，在作用域结束时自动解锁
 * - routing_mutex_: 保护routing相关数据的互斥锁
 */
bool ReferenceLineProvider::UpdatePlanningCommand(
    const planning::PlanningCommand &command) {

  /**
   * std::lock_guard<std::mutex> - 线程安全锁
   * 构造函数自动加锁，析构函数自动解锁
   * 确保多线程下对planning_command_的安全访问
   */
  std::lock_guard<std::mutex> routing_lock(routing_mutex_);

  bool find_matched_pnc_map = false;

  /**
   * 遍历pnc_map_list_找到能处理该命令的PncMap
   * CanProcess()检查该PncMap是否能处理此命令
   */
  for (const auto &pnc_map : pnc_map_list_) {
    if (pnc_map->CanProcess(command)) {
      current_pnc_map_ = pnc_map;
      find_matched_pnc_map = true;
      break;  /**< 找到第一个匹配的PncMap就停止 */
    }
  }

  /**
   * 如果没有找到任何能处理该命令的PncMap，返回错误
   */
  if (nullptr == current_pnc_map_) {
    AERROR << "Cannot find pnc map to process input command!"
           << command.DebugString();
    return false;
  }

  /**
   * 如果找到了匹配的PncMap但不是默认的那个，记录警告
   * 旧的PncMap将继续使用
   */
  if (!find_matched_pnc_map) {
    AWARN << "Find no pnc map for the input command and the old one will be "
             "used!";
  }

  /**
   * 更新路由信息
   * pnc_map_mutex_保护current_pnc_map_的访问
   */
  // Update routing in pnc_map
  std::lock_guard<std::mutex> lock(pnc_map_mutex_);

  /**
   * IsNewPlanningCommand检查是否是新的规划命令
   * 如果是，调用UpdatePlanningCommand更新路由
   */
  if (current_pnc_map_->IsNewPlanningCommand(command)) {
    is_new_command_ = true;
    if (!current_pnc_map_->UpdatePlanningCommand(command)) {
      AERROR << "Failed to update routing in pnc map: "
             << command.DebugString();
      return false;
    }
  }

  /**
   * 保存规划命令并设置标志
   */
  planning_command_ = command;
  has_planning_command_ = true;
  return true;
}

/**
 * @brief 获取未来路线waypoints
 *
 * @return std::vector<routing::LaneWaypoint> 沿路线的waypoint列表
 *
 * 在非导航模式下，从current_pnc_map_获取未来路线waypoints
 * 在导航模式下返回空向量
 */
std::vector<routing::LaneWaypoint>
ReferenceLineProvider::FutureRouteWaypoints() {
  /**
   * !FLAGS_use_navigation_mode: 非导航模式
   * current_pnc_map_不为空：已初始化PncMap
   */
  if (!FLAGS_use_navigation_mode && nullptr != current_pnc_map_) {
    std::lock_guard<std::mutex> lock(pnc_map_mutex_);
    return current_pnc_map_->FutureRouteWaypoints();
  }

  // return an empty routing::LaneWaypoint vector in Navigation mode.
  return std::vector<routing::LaneWaypoint>();
}

/**
 * @brief 获取终点车道waypoint
 *
 * @param end_point 输出参数，指向终点waypoint的共享指针
 */
void ReferenceLineProvider::GetEndLaneWayPoint(
    std::shared_ptr<routing::LaneWaypoint> &end_point) const {
  if (nullptr == current_pnc_map_) {
    end_point = nullptr;
    return;
  }
  current_pnc_map_->GetEndLaneWayPoint(end_point);
}

/**
 * @brief 根据车道ID获取车道信息
 *
 * @param id 车道ID
 * @return hdmap::LaneInfoConstPtr 车道信息常量指针
 */
hdmap::LaneInfoConstPtr ReferenceLineProvider::GetLaneById(
    const hdmap::Id &id) const {
  if (nullptr == current_pnc_map_) {
    return nullptr;
  }
  return current_pnc_map_->GetLaneById(id);
}

/**
 * @brief 更新车辆状态
 *
 * 规划线程调用此函数更新车辆状态
 * 参考线生成时需要根据车辆位置来确定参考线范围
 *
 * @param vehicle_state 当前车辆状态
 *
 * 语法说明：
 * - std::lock_guard<std::mutex> lock(vehicle_state_mutex_)
 *   临时锁，作用域结束后自动解锁
 * - vehicle_state_ = vehicle_state
 *   拷贝赋值，将传入的状态保存到成员变量
 */
void ReferenceLineProvider::UpdateVehicleState(
    const VehicleState &vehicle_state) {
  std::lock_guard<std::mutex> lock(vehicle_state_mutex_);
  vehicle_state_ = vehicle_state;
}

/**
 * @brief 启动参考线提供者
 *
 * 在组件初始化时调用，启动后台参考线生成线程
 *
 * @return bool 启动是否成功
 *
 * 语法说明：
 * - cyber::Async(&Class::method, this)
 *   Cyber RT的异步任务调度，创建新线程执行指定方法
 * - task_future_存储异步任务 future，可用于等待任务完成
 */
bool ReferenceLineProvider::Start() {
  /**
   * 导航模式下不需要启动线程
   * 导航模式使用相对地图，不需要持续生成参考线
   */
  if (FLAGS_use_navigation_mode) {
    return true;
  }

  /**
   * 检查是否已初始化: 构造函数完成后，is_initialized_ = true
   */
  if (!is_initialized_) {
    AERROR << "ReferenceLineProvider has NOT been initiated.";
    return false;
  }

  /**
   * FLAGS_enable_reference_line_provider_thread
   * 是否启用独立的参考线生成线程
   * 如果启用，启动GenerateThread后台线程
   */
  if (FLAGS_enable_reference_line_provider_thread) {
    /**
     * cyber::Async创建异步任务
     * &ReferenceLineProvider::GenerateThread是成员函数指针
     * this作为参数传入
     * 返回future可用于获取任务结果或等待完成
     */
    task_future_ = cyber::Async(&ReferenceLineProvider::GenerateThread, this);
  }
  return true;
}

/**
 * @brief 停止参考线提供者
 *
 * 析构或组件关闭时调用，停止后台线程
 */
void ReferenceLineProvider::Stop() {
  is_stop_ = true;  /**< 设置停止标志 */

  /**
   * 如果有后台线程，等待其结束
   * task_future_.get()阻塞直到任务完成
   */
  if (FLAGS_enable_reference_line_provider_thread) {
    task_future_.get();
  }
}

/**
 * @brief 重置参考线提供者
 *
 * 清空所有参考线、路由段和历史记录
 * 用于收到新路由请求时重置状态
 */
void ReferenceLineProvider::Reset() {
  std::lock_guard<std::mutex> lock(routing_mutex_);

  /**
   * 重置各种标志和容器
   */
  has_planning_command_ = false;  /**< 无有效规划命令 */
  is_new_command_ = false;        /**< 不是新命令 */
  reference_lines_.clear();        /**< 清空参考线 */
  route_segments_.clear();         /**< 清空路由段 */
  is_reference_line_updated_ = false;  /**< 参考线未更新 */

  /**
   * planning_command_.Clear()清空规划命令内容
   */
  planning_command_.Clear();

  /**
   * 清空历史记录队列
   * 使用while循环和pop()逐个清空
   */
  while (!reference_line_history_.empty()) {
    reference_line_history_.pop();
  }
}

/**
 * @brief 更新参考线
 *
 * 当生成新的参考线时调用此函数更新内部状态
 * 同时维护历史记录
 *
 * @param reference_lines 新的参考线列表
 * @param route_segments 新的路由段列表
 *
 * 语法说明：
 * - std::list<T>: 双向链表，emplace_back在末尾构造元素
 * - reference_lines.size() != route_segments.size()
 *   两个列表大小必须一致
 */
void ReferenceLineProvider::UpdateReferenceLine(
    const std::list<ReferenceLine> &reference_lines,
    const std::list<hdmap::RouteSegments> &route_segments) {

  /**
   * 验证输入有效性
   * 大小不一致或为空都是无效输入
   */
  if (reference_lines.size() != route_segments.size()) {
    AERROR << "The calculated reference line size(" << reference_lines.size()
           << ") and route_segments size(" << route_segments.size()
           << ") are different";
    return;
  }
  if (reference_lines.empty()) {
    return;
  }

  /**
   * reference_lines_mutex_保护参考线的更新
   */
  std::lock_guard<std::mutex> lock(reference_lines_mutex_);

  /**
   * 比较新旧参考线的大小
   * 如果大小不同，直接完全替换
   */
  if (reference_lines_.size() != reference_lines.size()) {
    reference_lines_ = reference_lines;
    route_segments_ = route_segments;
  } else {
    /**
     * 大小相同时，逐个比较并更新有变化的参考线
     * 使用多迭代器同时遍历新旧数据
     */
    auto segment_iter = route_segments.begin();
    auto internal_iter = reference_lines_.begin();
    auto internal_segment_iter = route_segments_.begin();

    /**
     * 遍历所有参考线
     * for循环的多个迭代器变量用逗号分隔
     */
    for (auto iter = reference_lines.begin();
         iter != reference_lines.end() &&
         segment_iter != route_segments.end() &&
         internal_iter != reference_lines_.end() &&
         internal_segment_iter != route_segments_.end();
         ++iter, ++segment_iter, ++internal_iter, ++internal_segment_iter) {

      /**
       * 如果原始参考点为空，直接替换
       */
      if (iter->reference_points().empty()) {
        *internal_iter = *iter;
        *internal_segment_iter = *segment_iter;
        continue;
      }

      /**
       * SamePointXY检查两点是否相同（XY平面）
       * 比较首尾点和长度，判断是否有变化
       */
      if (common::util::SamePointXY(
              iter->reference_points().front(),
              internal_iter->reference_points().front()) &&
          common::util::SamePointXY(iter->reference_points().back(),
                                    internal_iter->reference_points().back()) &&
          std::fabs(iter->Length() - internal_iter->Length()) <
              common::math::kMathEpsilon) {
        /**
         * 如果完全相同，跳过（保持原样）
         */
        continue;
      }

      /**
       * 有变化，更新该参考线
       */
      *internal_iter = *iter;
      *internal_segment_iter = *segment_iter;
    }
  }

  /**
   * 更新历史记录
   * 使用栈结构存储历史（push/pop）
   */
  // update history
  reference_line_history_.push(reference_lines_);
  route_segments_history_.push(route_segments_);

  /**
   * kMaxHistoryNum = 3
   * 只保留最近3条历史记录
   */
  static constexpr int kMaxHistoryNum = 3;
  if (reference_line_history_.size() > kMaxHistoryNum) {
    reference_line_history_.pop();
    route_segments_history_.pop();
  }
}

/**
 * @brief 参考线生成线程
 *
 * 后台线程函数，持续生成参考线
 * 在FLAGS_enable_reference_line_provider_thread为true时启用
 *
 * 语法说明：
 * - while (!is_stop_): 主循环，直到收到停止标志
 * - cyber::SleepFor(): Cyber RT的睡眠函数
 * - std::chrono::milliseconds(50): 50毫秒睡眠
 */
void ReferenceLineProvider::GenerateThread() {
  while (!is_stop_) {
    static constexpr int32_t kSleepTime = 50;  // milliseconds
    cyber::SleepFor(std::chrono::milliseconds(kSleepTime));

    const double start_time = Clock::NowInSeconds();

    /**
     * 没有规划命令时继续等待
     */
    if (!has_planning_command_) {
      continue;
    }

    /**
     * 创建参考线和路由段
     */
    std::list<ReferenceLine> reference_lines;
    std::list<hdmap::RouteSegments> segments;
    if (!CreateReferenceLine(&reference_lines, &segments)) {
      is_reference_line_updated_ = false;
      AERROR << "Fail to get reference line";
      continue;
    }

    /**
     * 更新参考线
     */
    UpdateReferenceLine(reference_lines, segments);

    const double end_time = Clock::NowInSeconds();
    std::lock_guard<std::mutex> lock(reference_lines_mutex_);
    last_calculation_time_ = end_time - start_time;
    is_reference_line_updated_ = true;
  }
}

/**
 * @brief 获取上次计算的时间延迟
 *
 * @return double 上次生成参考线的时间消耗（秒）
 */
double ReferenceLineProvider::LastTimeDelay() {
  if (FLAGS_enable_reference_line_provider_thread &&
      !FLAGS_use_navigation_mode) {
    std::lock_guard<std::mutex> lock(reference_lines_mutex_);
    return last_calculation_time_;
  } else {
    return last_calculation_time_;
  }
}

/**
 * @brief 获取参考线
 *
 * 主要入口函数，供外部调用获取当前参考线
 * 在不同模式下有不同的处理逻辑
 *
 * @param reference_lines 输出参数，参考线列表
 * @param segments 输出参数，路由段列表
 * @return bool 获取是否成功
 *
 * 语法说明：
 * - CHECK_NOTNULL(ptr): 断言检查指针非空
 * - reference_lines->assign(): 赋值操作，清空并复制
 */
bool ReferenceLineProvider::GetReferenceLines(
    std::list<ReferenceLine> *reference_lines,
    std::list<hdmap::RouteSegments> *segments) {
  CHECK_NOTNULL(reference_lines);  /**< 断言：reference_lines非空 */
  CHECK_NOTNULL(segments);         /**< 断言：segments非空 */

  /**
   * 没有规划命令时返回true（使用空的参考线）
   */
  if (!has_planning_command_) {
    return true;
  }

  /**
   * 导航模式：从相对地图获取参考线
   */
  if (FLAGS_use_navigation_mode) {
    double start_time = Clock::NowInSeconds();
    bool result = GetReferenceLinesFromRelativeMap(reference_lines, segments);
    if (!result) {
      AERROR << "Failed to get reference line from relative map";
    }
    double end_time = Clock::NowInSeconds();
    last_calculation_time_ = end_time - start_time;
    return result;
  }

  /**
   * 有独立线程模式：直接从缓冲区获取
   */
  if (FLAGS_enable_reference_line_provider_thread) {
    std::lock_guard<std::mutex> lock(reference_lines_mutex_);
    if (!reference_lines_.empty()) {
      /**
       * assign()：清空目标容器并复制源容器的所有元素
       * begin()/end()：容器的迭代器
       */
      reference_lines->assign(reference_lines_.begin(), reference_lines_.end());
      segments->assign(route_segments_.begin(), route_segments_.end());
      return true;
    }
  } else {
    /**
     * 无独立线程模式：同步创建参考线
     */
    double start_time = Clock::NowInSeconds();
    if (CreateReferenceLine(reference_lines, segments)) {
      UpdateReferenceLine(*reference_lines, *segments);
      double end_time = Clock::NowInSeconds();
      last_calculation_time_ = end_time - start_time;
      return true;
    }
  }

  /**
   * 如果上述都失败，尝试使用历史记录
   */
  AINFO << "Reference line is NOT ready.";
  if (reference_line_history_.empty()) {
    AINFO << "Failed to use reference line latest history";
    return false;
  }

  /**
   * 使用最近一次的历史参考线
   * back()获取队列末尾元素的引用
   */
  reference_lines->assign(reference_line_history_.back().begin(),
                          reference_line_history_.back().end());
  segments->assign(route_segments_history_.back().begin(),
                   route_segments_history_.back().end());
  AWARN << "Use reference line from history!";
  return true;
}

/**
 * @brief 优先处理换道
 *
 * 如果有换道请求，将换道相关的路由段放到最前面
 * 确保优先考虑换道需求
 *
 * @param route_segments 路由段列表（会被修改）
 *
 * 语法说明：
 * - splice()：将元素从一个列表移动到另一个列表
 */
void ReferenceLineProvider::PrioritizeChangeLane(
    std::list<hdmap::RouteSegments> *route_segments) {
  CHECK_NOTNULL(route_segments);

  auto iter = route_segments->begin();

  /**
   * 遍历查找第一个不在当前车道上的路由段
   * IsOnSegment()检查是否在当前车道上
   */
  while (iter != route_segments->end()) {
    if (!iter->IsOnSegment()) {
      /**
       * splice()操作：
       * 将iter指向的元素移动到route_segments的开头
       */
      route_segments->splice(route_segments->begin(), *route_segments, iter);
      break;
    }
    ++iter;
  }
}

/**
 * @brief 从相对地图获取参考线
 *
 * 导航模式下使用，从相对地图消息构建参考线
 *
 * @param reference_lines 输出参考线列表
 * @param segments 输出路由段列表
 * @return bool 获取是否成功
 *
 * 核心流程：
 * 1. 获取车辆当前所在车道及其左右邻居车道
 * 2. 识别高优先级车道（可能需要换道）
 * 3. 确定目标车道和换道类型
 * 4. 构建参考线和路由段
 */
bool ReferenceLineProvider::GetReferenceLinesFromRelativeMap(
    std::list<ReferenceLine> *reference_lines,
    std::list<hdmap::RouteSegments> *segments) {
  CHECK_GE(relative_map_->navigation_path_size(), 0);  /**< 断言：navigation_path_size >= 0 */
  CHECK_NOTNULL(reference_lines);
  CHECK_NOTNULL(segments);

  /**
   * 检查是否有导航路径
   */
  if (relative_map_->navigation_path().empty()) {
    AERROR << "There isn't any navigation path in current relative map.";
    return false;
  }

  /**
   * 获取HDMap指针
   * HDMapUtil::BaseMapPtr(*relative_map_)使用相对地图创建HDMap
   */
  auto *hdmap = HDMapUtil::BaseMapPtr(*relative_map_);
  if (!hdmap) {
    AERROR << "hdmap is null";
    return false;
  }

  /**
   * 1. 获取ADC（自车）当前车道信息
   * navigation_lane_ids存储所有导航路径上的车道ID
   */
  // 1.get adc current lane info ,such as lane_id,lane_priority,neighbor lanes
  std::unordered_set<std::string> navigation_lane_ids;  /**< 无序集合，O(1)查找 */
  for (const auto &path_pair : relative_map_->navigation_path()) {
    const auto lane_id = path_pair.first;
    navigation_lane_ids.insert(lane_id);
  }
  if (navigation_lane_ids.empty()) {
    AERROR << "navigation path ids is empty";
    return false;
  }

  /**
   * 获取车辆状态
   */
  // get current adc lane info by vehicle state
  common::VehicleState vehicle_state = vehicle_state_provider_->vehicle_state();

  /**
   * 获取自车最近的车道waypoint
   */
  hdmap::LaneWaypoint adc_lane_way_point;
  if (!GetNearestWayPointFromNavigationPath(vehicle_state, navigation_lane_ids,
                                            &adc_lane_way_point)) {
    return false;
  }

  /**
   * 获取自车所在车道的ID
   */
  const std::string adc_lane_id = adc_lane_way_point.lane->id().id();

  /**
   * 获取该车道在导航路径中的优先级
   */
  auto *adc_navigation_path = apollo::common::util::FindOrNull(
      relative_map_->navigation_path(), adc_lane_id);
  if (adc_navigation_path == nullptr) {
    AERROR << "adc lane cannot be found in relative_map_->navigation_path";
    return false;
  }
  const uint32_t adc_lane_priority = adc_navigation_path->path_priority();

  /**
   * 2. 获取自车左边的邻居车道
   * 使用while循环沿左边邻居车道一直向前找
   */
  // get adc left neighbor lanes
  std::vector<std::string> left_neighbor_lane_ids;
  auto left_lane_ptr = adc_lane_way_point.lane;
  while (left_lane_ptr != nullptr &&
         left_lane_ptr->lane().left_neighbor_forward_lane_id_size() > 0) {
    /**
     * left_neighbor_forward_lane_id(0)获取第一个左邻居车道ID
     */
    auto neighbor_lane_id =
        left_lane_ptr->lane().left_neighbor_forward_lane_id(0);
    left_neighbor_lane_ids.emplace_back(neighbor_lane_id.id());
    left_lane_ptr = hdmap->GetLaneById(neighbor_lane_id);
  }
  ADEBUG << adc_lane_id
         << " left neighbor size : " << left_neighbor_lane_ids.size();
  for (const auto &neighbor : left_neighbor_lane_ids) {
    ADEBUG << adc_lane_id << " left neighbor : " << neighbor;
  }

  /**
   * 3. 获取自车右边的邻居车道（类似左边）
   */
  // get adc right neighbor lanes
  std::vector<std::string> right_neighbor_lane_ids;
  auto right_lane_ptr = adc_lane_way_point.lane;
  while (right_lane_ptr != nullptr &&
         right_lane_ptr->lane().right_neighbor_forward_lane_id_size() > 0) {
    auto neighbor_lane_id =
        right_lane_ptr->lane().right_neighbor_forward_lane_id(0);
    right_neighbor_lane_ids.emplace_back(neighbor_lane_id.id());
    right_lane_ptr = hdmap->GetLaneById(neighbor_lane_id);
  }
  ADEBUG << adc_lane_id
         << " right neighbor size : " << right_neighbor_lane_ids.size();
  for (const auto &neighbor : right_neighbor_lane_ids) {
    ADEBUG << adc_lane_id << " right neighbor : " << neighbor;
  }

  /**
   * 4. 获取高优先级车道列表
   * 优先级数值越小优先级越高
   */
  // 2.get the higher priority lane info list which priority higher
  // than current lane and get the highest one as the target lane
  using LaneIdPair = std::pair<std::string, uint32_t>;  /**< (车道ID, 优先级)对 */
  std::vector<LaneIdPair> high_priority_lane_pairs;
  ADEBUG << "relative_map_->navigation_path_size = "
         << relative_map_->navigation_path_size();
  for (const auto &path_pair : relative_map_->navigation_path()) {
    const auto lane_id = path_pair.first;
    const uint32_t priority = path_pair.second.path_priority();
    ADEBUG << "lane_id = " << lane_id << " priority = " << priority
           << " adc_lane_id = " << adc_lane_id
           << " adc_lane_priority = " << adc_lane_priority;
    /**
     * 优先级数值小于自车道优先级时为高优先级车道
     * 数值越小优先级越高
     */
    // the smaller the number, the higher the priority
    if (adc_lane_id != lane_id && priority < adc_lane_priority) {
      high_priority_lane_pairs.emplace_back(lane_id, priority);
    }
  }

  /**
   * 找到优先级最高的目标车道
   */
  // get the target lane
  bool is_lane_change_needed = false;
  LaneIdPair target_lane_pair;
  if (!high_priority_lane_pairs.empty()) {
    /**
     * std::sort按优先级排序（从小到大）
     * lambda表达式定义比较函数
     */
    std::sort(high_priority_lane_pairs.begin(), high_priority_lane_pairs.end(),
              [](const LaneIdPair &left, const LaneIdPair &right) {
                return left.second < right.second;
              });
    ADEBUG << "need to change lane";
    // the highest priority lane as the target navigation lane
    target_lane_pair = high_priority_lane_pairs.front();
    is_lane_change_needed = true;
  }

  /**
   * 5. 确定到目标车道的最近邻居车道
   * 判断目标在自车左边还是右边
   */
  // 3.get current lane's the nearest neighbor lane to the target lane
  // and make sure it position is left or right on the current lane
  routing::ChangeLaneType lane_change_type = routing::FORWARD;
  std::string nearest_neighbor_lane_id;
  if (is_lane_change_needed) {
    /**
     * std::find在容器中查找元素
     * 如果目标在左边邻居列表中
     */
    // target on the left of adc
    if (left_neighbor_lane_ids.end() !=
        std::find(left_neighbor_lane_ids.begin(), left_neighbor_lane_ids.end(),
                  target_lane_pair.first)) {
      lane_change_type = routing::LEFT;
      nearest_neighbor_lane_id =
          adc_lane_way_point.lane->lane().left_neighbor_forward_lane_id(0).id();
    } else if (right_neighbor_lane_ids.end() !=
               std::find(right_neighbor_lane_ids.begin(),
                         right_neighbor_lane_ids.end(),
                         target_lane_pair.first)) {
      /**
       * 目标在右边
       */
      // target lane on the right of adc
      lane_change_type = routing::RIGHT;
      nearest_neighbor_lane_id = adc_lane_way_point.lane->lane()
                                     .right_neighbor_forward_lane_id(0)
                                     .id();
    }
  }

  /**
   * 6. 构建参考线和路由段
   */
  for (const auto &path_pair : relative_map_->navigation_path()) {
    const auto &lane_id = path_pair.first;
    const auto &path_points = path_pair.second.path().path_point();

    /**
     * 获取车道信息
     */
    auto lane_ptr = hdmap->GetLaneById(hdmap::MakeMapId(lane_id));

    /**
     * 创建路由段
     * 从0到total_length覆盖整个车道
     */
    RouteSegments segment;
    segment.emplace_back(lane_ptr, 0.0, lane_ptr->total_length());
    segment.SetCanExit(true);
    segment.SetId(lane_id);
    segment.SetNextAction(routing::FORWARD);
    segment.SetStopForDestination(false);
    segment.SetPreviousAction(routing::FORWARD);

    /**
     * 如果需要换道，标记相关属性
     */
    if (is_lane_change_needed) {
      /**
       * 如果是最近邻居车道（换道目标）
       */
      if (lane_id == nearest_neighbor_lane_id) {
        ADEBUG << "adc lane_id = " << adc_lane_id
               << " nearest_neighbor_lane_id = " << lane_id;
        segment.SetIsNeighborSegment(true);
        segment.SetPreviousAction(lane_change_type);
      } else if (lane_id == adc_lane_id) {
        /**
         * 如果是自车当前车道
         */
        segment.SetIsOnSegment(true);
        segment.SetNextAction(lane_change_type);
      }
    }

    /**
     * 添加路由段
     */
    segments->emplace_back(segment);

    /**
     * 构建参考点列表
     */
    std::vector<ReferencePoint> ref_points;
    for (const auto &path_point : path_points) {
      /**
       * MapPathPoint包含位置和航向
       * LaneWaypoint包含车道和s值
       */
      ref_points.emplace_back(
          MapPathPoint{Vec2d{path_point.x(), path_point.y()},
                       path_point.theta(),
                       LaneWaypoint(lane_ptr, path_point.s())},
          path_point.kappa(), path_point.dkappa());
    }

    /**
     * 创建参考线
     * 使用参考点范围构造
     */
    reference_lines->emplace_back(ref_points.begin(), ref_points.end());

    /**
     * 设置参考线优先级
     */
    reference_lines->back().SetPriority(path_pair.second.path_priority());
  }
  return !segments->empty();
}

/**
 * @brief 从导航路径获取最近的车道waypoint
 *
 * @param state 车辆状态
 * @param navigation_lane_ids 导航车道ID集合
 * @param waypoint 输出最近waypoint
 * @return bool 是否成功找到
 *
 * 算法流程：
 * 1. 获取车辆周围一定范围内的所有车道（带朝向）
 * 2. 过滤出在导航路径中的有效车道
 * 3. 找到距离自车最近的车道
 */
bool ReferenceLineProvider::GetNearestWayPointFromNavigationPath(
    const common::VehicleState &state,
    const std::unordered_set<std::string> &navigation_lane_ids,
    hdmap::LaneWaypoint *waypoint) {

  /**
   * kMaxDistance = 10.0米
   * 在10米范围内搜索
   */
  const double kMaxDistance = 10.0;
  waypoint->lane = nullptr;
  std::vector<hdmap::LaneInfoConstPtr> lanes;

  /**
   * PointFactory::ToPointENU将VehicleState转换为PointENU
   */
  auto point = common::util::PointFactory::ToPointENU(state);

  /**
   * 检查坐标是否有效（非NaN）
   * std::isnan检测Not-a-Number
   */
  if (std::isnan(point.x()) || std::isnan(point.y())) {
    AERROR << "vehicle state is invalid";
    return false;
  }

  auto *hdmap = HDMapUtil::BaseMapPtr();
  if (!hdmap) {
    AERROR << "hdmap is null";
    return false;
  }

  /**
   * GetLanesWithHeading获取指定范围内带特定朝向的车道
   * point: 查询点
   * kMaxDistance: 最大距离
   * state.heading(): 查询朝向
   * M_PI / 2.0: 朝向容差（90度）
   * &lanes: 输出参数
   * 返回值<0表示失败
   */
  // get all adc direction lanes from map in kMaxDistance range
  // by vehicle point in map
  const int status = hdmap->GetLanesWithHeading(
      point, kMaxDistance, state.heading(), M_PI / 2.0, &lanes);
  if (status < 0) {
    AERROR << "failed to get lane from point " << point.ShortDebugString();
    return false;
  }

  /**
   * std::copy_if复制满足条件的元素
   * lambda表达式检查车道是否在导航路径中
   */
  // get lanes that exist in both map and navigation paths as valid lanes
  std::vector<hdmap::LaneInfoConstPtr> valid_lanes;
  std::copy_if(lanes.begin(), lanes.end(), std::back_inserter(valid_lanes),
               [&](hdmap::LaneInfoConstPtr ptr) {
                 return navigation_lane_ids.count(ptr->lane().id().id()) > 0;
               });
  if (valid_lanes.empty()) {
    AERROR << "no valid lane found within " << kMaxDistance
           << " meters with heading " << state.heading();
    return false;
  }

  /**
   * 在有效车道中找到最近的车道
   */
  // get nearest lane waypoints for current adc position
  double min_distance = std::numeric_limits<double>::infinity();
  for (const auto &lane : valid_lanes) {
    /**
     * GetProjection将点投影到车道上
     * 返回沿车道距离s和横向距离l
     */
    double s = 0.0;
    double l = 0.0;
    if (!lane->GetProjection({point.x(), point.y()}, state.heading(), &s, &l)) {
      continue;
    }

    /**
     * 检查投影点是否在车道范围内
     */
    static constexpr double kEpsilon = 1e-6;
    if (s > (lane->total_length() + kEpsilon) || (s + kEpsilon) < 0.0) {
      continue;
    }

    /**
     * GetNearestPoint获取最近点及其距离
     */
    double distance = 0.0;
    common::PointENU map_point =
        lane->GetNearestPoint({point.x(), point.y()}, &distance);

    /**
     * 记录距离最近的车道
     */
    if (distance < min_distance) {
      double s = 0.0;
      double l = 0.0;
      if (!lane->GetProjection({map_point.x(), map_point.y()}, &s, &l)) {
        AERROR << "failed to get projection for map_point "
               << map_point.DebugString();
        continue;
      }
      min_distance = distance;
      waypoint->lane = lane;
      waypoint->s = s;
    }
  }

  if (waypoint->lane == nullptr) {
    AERROR << "failed to find nearest point " << point.ShortDebugString();
  }
  return waypoint->lane != nullptr;
}

/**
 * @brief 创建路由段
 *
 * @param vehicle_state 车辆状态
 * @param segments 输出路由段列表
 * @return bool 创建是否成功
 */
bool ReferenceLineProvider::CreateRouteSegments(
    const common::VehicleState &vehicle_state,
    std::list<hdmap::RouteSegments> *segments) {
  {
    std::lock_guard<std::mutex> lock(pnc_map_mutex_);
    /**
     * GetRouteSegments从PncMap获取路由段
     */
    if (!current_pnc_map_->GetRouteSegments(vehicle_state, segments)) {
      AERROR << "Failed to extract segments from routing";
      return false;
    }
  }

  /**
   * 打印调试信息
   */
  for (auto &seg : *segments) {
    ADEBUG << seg.DebugString();
  }

  /**
   * 如果启用换道优先，调整路由段顺序
   */
  if (FLAGS_prioritize_change_lane) {
    PrioritizeChangeLane(segments);
  }
  return !segments->empty();
}

/**
 * @brief 创建参考线
 *
 * 核心函数，根据路由段生成平滑的参考线
 *
 * @param reference_lines 输出参考线列表
 * @param segments 输出路由段列表
 * @return bool 创建是否成功
 */
bool ReferenceLineProvider::CreateReferenceLine(
    std::list<ReferenceLine> *reference_lines,
    std::list<hdmap::RouteSegments> *segments) {
  CHECK_NOTNULL(reference_lines);
  CHECK_NOTNULL(segments);

  /**
   * 获取车辆状态（线程安全）
   */
  common::VehicleState vehicle_state;
  {
    std::lock_guard<std::mutex> lock(vehicle_state_mutex_);
    vehicle_state = vehicle_state_;
  }

  /**
   * 获取规划命令（线程安全）
   */
  planning::PlanningCommand command;
  {
    std::lock_guard<std::mutex> lock(routing_mutex_);
    command = planning_command_;
  }

  if (nullptr == current_pnc_map_) {
    AERROR << "Current pnc map is null! " << command.DebugString();
    return false;
  }

  /**
   * 创建路由段
   */
  if (!CreateRouteSegments(vehicle_state, segments)) {
    AERROR << "Failed to create reference line from routing";
    return false;
  }

  /**
   * 根据标志决定是全新创建还是缝合
   * is_new_command_: 新命令，需要重新平滑
   * FLAGS_enable_reference_line_stitching: 启用参考线缝合
   */
  if (is_new_command_ || !FLAGS_enable_reference_line_stitching) {
    /**
     * 遍历每个路由段，平滑生成参考线
     */
    for (auto iter = segments->begin(); iter != segments->end();) {
      /**
       * emplace_back在末尾构造空元素
       */
      reference_lines->emplace_back();

      /**
       * SmoothRouteSegment平滑路由段生成参考线
       * 如果失败，pop_back删除刚添加的空元素
       * erase删除对应路由段
       */
      if (!SmoothRouteSegment(*iter, &reference_lines->back())) {
        AERROR << "Failed to create reference line from route segments";
        reference_lines->pop_back();
        iter = segments->erase(iter);
      } else {
        /**
         * 成功时，裁剪参考线到车辆周围范围
         */
        common::SLPoint sl;
        if (!reference_lines->back().XYToSL(
                vehicle_state.heading(),
                common::math::Vec2d(vehicle_state.x(), vehicle_state.y()),
                &sl)) {
          AWARN << "Failed to project point: {" << vehicle_state.x() << ","
                << vehicle_state.y() << "} to stitched reference line";
        }
        Shrink(sl, &reference_lines->back(), &(*iter));
        ++iter;
      }
    }
    is_new_command_ = false;
    return true;
  } else {
    /**
     * 缝合模式：扩展已有参考线
     */
    // stitching reference line
    for (auto iter = segments->begin(); iter != segments->end();) {
      reference_lines->emplace_back();
      if (!ExtendReferenceLine(vehicle_state, &(*iter),
                               &reference_lines->back())) {
        AERROR << "Failed to extend reference line";
        reference_lines->pop_back();
        iter = segments->erase(iter);
      } else {
        ++iter;
      }
    }
  }
  return true;
}

/**
 * @brief 扩展参考线
 *
 * 在已有参考线的基础上扩展新的路由段
 *
 * @param state 车辆状态
 * @param segments 路由段指针
 * @param reference_line 参考线指针
 * @return bool 扩展是否成功
 */
bool ReferenceLineProvider::ExtendReferenceLine(const VehicleState &state,
                                                RouteSegments *segments,
                                                ReferenceLine *reference_line) {

  /**
   * 保存原有属性
   */
  RouteSegments segment_properties;
  segment_properties.SetProperties(*segments);

  /**
   * 查找与当前段连接的上一段
   */
  auto prev_segment = route_segments_.begin();
  auto prev_ref = reference_lines_.begin();
  while (prev_segment != route_segments_.end()) {
    if (prev_segment->IsConnectedSegment(*segments)) {
      break;
    }
    ++prev_segment;
    ++prev_ref;
  }

  /**
   * 如果没有找到连接的段，使用平滑方式创建
   */
  if (prev_segment == route_segments_.end()) {
    if (!route_segments_.empty() && segments->IsOnSegment()) {
      AWARN << "Current route segment is not connected with previous route "
               "segment";
    }
    return SmoothRouteSegment(*segments, reference_line);
  }

  /**
   * 获取车辆在上一参考线的投影
   */
  common::SLPoint sl_point;
  Vec2d vec2d(state.x(), state.y());
  LaneWaypoint waypoint;
  if (!prev_segment->GetProjection(vec2d, state.heading(), &sl_point,
                                   &waypoint)) {
    AWARN << "Vehicle current point: " << vec2d.DebugString()
          << " not on previous reference line";
    return SmoothRouteSegment(*segments, reference_line);
  }

  /**
   * 计算剩余距离
   */
  const double prev_segment_length = RouteSegments::Length(*prev_segment);
  const double remain_s = prev_segment_length - sl_point.s();

  /**
   * LookForwardDistance根据速度计算前向查看距离
   */
  const double look_forward_required_distance =
      planning::PncMapBase::LookForwardDistance(state.linear_velocity());

  /**
   * 如果剩余距离足够，不需要扩展
   */
  if (remain_s > look_forward_required_distance) {
    *segments = *prev_segment;
    segments->SetProperties(segment_properties);
    *reference_line = *prev_ref;
    ADEBUG << "Reference line remain " << remain_s
           << ", which is more than required " << look_forward_required_distance
           << " and no need to extend";
    return true;
  }

  /**
   * 计算扩展范围
   */
  double future_start_s =
      std::max(sl_point.s(), prev_segment_length -
                                 FLAGS_reference_line_stitch_overlap_distance);
  double future_end_s =
      prev_segment_length + FLAGS_look_forward_extend_distance;

  /**
   * 调用PncMap扩展段
   */
  RouteSegments shifted_segments;
  std::unique_lock<std::mutex> lock(pnc_map_mutex_);
  if (!current_pnc_map_->ExtendSegments(*prev_segment, future_start_s,
                                        future_end_s, &shifted_segments)) {
    lock.unlock();
    AERROR << "Failed to shift route segments forward";
    return SmoothRouteSegment(*segments, reference_line);
  }
  lock.unlock();

  /**
   * 检查扩展是否有效
   */
  if (prev_segment->IsWaypointOnSegment(shifted_segments.LastWaypoint())) {
    *segments = *prev_segment;
    segments->SetProperties(segment_properties);
    *reference_line = *prev_ref;
    ADEBUG << "Could not further extend reference line";
    return true;
  }

  /**
   * 创建新参考线并平滑
   */
  hdmap::Path path(shifted_segments);
  ReferenceLine new_ref(path);
  if (!SmoothPrefixedReferenceLine(*prev_ref, new_ref, reference_line)) {
    AWARN << "Failed to smooth forward shifted reference line";
    return SmoothRouteSegment(*segments, reference_line);
  }

  /**
   * 缝合新旧参考线
   */
  if (!reference_line->Stitch(*prev_ref)) {
    AWARN << "Failed to stitch reference line";
    return SmoothRouteSegment(*segments, reference_line);
  }
  if (!shifted_segments.Stitch(*prev_segment)) {
    AWARN << "Failed to stitch route segments";
    return SmoothRouteSegment(*segments, reference_line);
  }

  /**
   * 更新段和属性
   */
  *segments = shifted_segments;
  segments->SetProperties(segment_properties);

  /**
   * 裁剪到车辆周围
   */
  common::SLPoint sl;
  if (!reference_line->XYToSL(state.heading(), vec2d, &sl)) {
    AWARN << "Failed to project point: " << vec2d.DebugString()
          << " to stitched reference line";
  }
  return Shrink(sl, reference_line, segments);
}

/**
 * @brief 裁剪参考线
 *
 * 根据车辆位置裁剪参考线
 * 只保留车辆周围的部分，移除前方过远和后方已过的部分
 *
 * @param sl 车辆在参考线的SL坐标
 * @param reference_line 参考线指针
 * @param segments 路由段指针
 * @return bool 裁剪是否成功
 *
 * 语法说明：
 * - GetNearestReferenceIndex()获取最近参考点索引
 * - AngleDiff()计算角度差异
 */
bool ReferenceLineProvider::Shrink(const common::SLPoint &sl,
                                   ReferenceLine *reference_line,
                                   RouteSegments *segments) {
  // shrink reference line
  double new_backward_distance = sl.s();
  double new_forward_distance = reference_line->Length() - sl.s();
  bool need_shrink = false;

  /**
   * 如果后方距离过大，需要裁剪
   */
  if (sl.s() > planning::FLAGS_look_backward_distance * 1.5) {
    ADEBUG << "reference line back side is " << sl.s()
           << ", shrink reference line: origin length: "
           << reference_line->Length();
    new_backward_distance = planning::FLAGS_look_backward_distance;
    need_shrink = true;
  }

  /**
   * 检查前方航向变化
   */
  // check heading
  const auto index = reference_line->GetNearestReferenceIndex(sl.s());
  const auto &ref_points = reference_line->reference_points();
  const double cur_heading = ref_points[index].heading();
  auto last_index = index;

  /**
   * AngleDiff计算角度差
   * FLAGS_reference_line_max_forward_heading_diff最大前向航向差异
   */
  while (last_index < ref_points.size() &&
         std::fabs(AngleDiff(cur_heading, ref_points[last_index].heading())) <
             FLAGS_reference_line_max_forward_heading_diff) {
    ++last_index;
  }
  --last_index;
  if (last_index != ref_points.size() - 1) {
    need_shrink = true;
    common::SLPoint forward_sl;
    reference_line->XYToSL(ref_points[last_index], &forward_sl);
    new_forward_distance = forward_sl.s() - sl.s();
  }

  /**
   * 检查后方航向变化
   */
  // check backward heading
  last_index = index;
  while (last_index > 0 &&
         abs(AngleDiff(cur_heading, ref_points[last_index].heading())) <
             FLAGS_reference_line_max_backward_heading_diff) {
    --last_index;
  }
  if (last_index != 0) {
    need_shrink = true;
    common::SLPoint backward_sl;
    reference_line->XYToSL(ref_points[last_index], &backward_sl);
    new_backward_distance = sl.s() - backward_sl.s();
  }

  if (need_shrink) {
    if (!reference_line->Segment(sl.s(), new_backward_distance,
                                 new_forward_distance)) {
      AWARN << "Failed to shrink reference line";
    }
    if (!segments->Shrink(sl.s(), new_backward_distance,
                          new_forward_distance)) {
      AWARN << "Failed to shrink route segment";
    }
  }
  return true;
}

/**
 * @brief 检查平滑后的参考线是否有效
 *
 * @param raw 原始参考线
 * @param smoothed 平滑后的参考线
 * @return bool 是否有效
 *
 * 通过比较平滑前后参考线的偏差
 */
bool ReferenceLineProvider::IsReferenceLineSmoothValid(
    const ReferenceLine &raw, const ReferenceLine &smoothed) const {
  static constexpr double kReferenceLineDiffCheckStep = 10.0;

  /**
   * 沿参考线每隔10米检查一次
   */
  for (double s = 0.0; s < smoothed.Length();
       s += kReferenceLineDiffCheckStep) {
    auto xy_new = smoothed.GetReferencePoint(s);

    common::SLPoint sl_new;
    if (!raw.XYToSL(xy_new, &sl_new)) {
      AERROR << "Fail to change xy point on smoothed reference line to sl "
                "point respect to raw reference line.";
      return false;
    }

    /**
     * 计算横向偏差
     * std::fabs计算浮点数绝对值
     */
    const double diff = std::fabs(sl_new.l());

    /**
     * 如果偏差过大，平滑无效
     * FLAGS_smoothed_reference_line_max_diff最大允许偏差
     */
    if (diff > FLAGS_smoothed_reference_line_max_diff) {
      AERROR << "Fail to provide reference line because too large diff "
                "between smoothed and raw reference lines. diff: "
             << diff;
      return false;
    }
  }
  return true;
}

/**
 * @brief 获取锚点
 *
 * 锚点用于参考线平滑，定义路径约束
 *
 * @param reference_line 参考线
 * @param s 沿参考线的距离
 * @return AnchorPoint 锚点
 */
AnchorPoint ReferenceLineProvider::GetAnchorPoint(
    const ReferenceLine &reference_line, double s) const {
  AnchorPoint anchor;

  /**
   * 纵向边界
   */
  anchor.longitudinal_bound = smoother_config_.longitudinal_boundary_bound();
  auto ref_point = reference_line.GetReferencePoint(s);

  /**
   * 如果没有车道信息，使用默认横向边界
   */
  if (ref_point.lane_waypoints().empty()) {
    anchor.path_point = ref_point.ToPathPoint(s);
    anchor.lateral_bound = smoother_config_.max_lateral_boundary_bound();
    return anchor;
  }

  /**
   * 计算有效的车道宽度
   */
  const double adc_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width();
  const Vec2d left_vec =
      Vec2d::CreateUnitVec2d(ref_point.heading() + M_PI / 2.0);
  auto waypoint = ref_point.lane_waypoints().front();
  double left_width = 0.0;
  double right_width = 0.0;
  waypoint.lane->GetWidth(waypoint.s, &left_width, &right_width);
  const double kEpislon = 1e-8;
  double effective_width = 0.0;

  /**
   * 计算安全车道宽度（减去车辆宽度和路缘石偏移）
   */
  // shrink width by vehicle width, curb
  double safe_lane_width = left_width + right_width;
  safe_lane_width -= adc_width;
  bool is_lane_width_safe = true;

  if (safe_lane_width < kEpislon) {
    ADEBUG << "lane width [" << left_width + right_width << "] "
           << "is smaller than adc width [" << adc_width << "]";
    effective_width = kEpislon;
    is_lane_width_safe = false;
  }

  /**
   * 处理路缘石偏移
   */
  double center_shift = 0.0;
  if (hdmap::RightBoundaryType(waypoint) == hdmap::LaneBoundaryType::CURB) {
    safe_lane_width -= smoother_config_.curb_shift();
    if (safe_lane_width < kEpislon) {
      ADEBUG << "lane width smaller than adc width and right curb shift";
      effective_width = kEpislon;
      is_lane_width_safe = false;
    } else {
      center_shift += 0.5 * smoother_config_.curb_shift();
    }
  }
  if (hdmap::LeftBoundaryType(waypoint) == hdmap::LaneBoundaryType::CURB) {
    safe_lane_width -= smoother_config_.curb_shift();
    if (safe_lane_width < kEpislon) {
      ADEBUG << "lane width smaller than adc width and left curb shift";
      effective_width = kEpislon;
      is_lane_width_safe = false;
    } else {
      center_shift -= 0.5 * smoother_config_.curb_shift();
    }
  }

  /**
   * 应用缓冲区
   */
  //  apply buffer if possible
  const double buffered_width =
      safe_lane_width - 2.0 * smoother_config_.lateral_buffer();
  safe_lane_width =
      buffered_width < kEpislon ? safe_lane_width : buffered_width;

  /**
   * 根据车道宽度计算中心偏移
   */
  // shift center depending on the road width
  if (is_lane_width_safe) {
    effective_width = 0.5 * safe_lane_width;
  }

  ref_point += left_vec * center_shift;
  anchor.path_point = ref_point.ToPathPoint(s);
  anchor.lateral_bound = common::math::Clamp(
      effective_width, smoother_config_.min_lateral_boundary_bound(),
      smoother_config_.max_lateral_boundary_bound());
  return anchor;
}

/**
 * @brief 获取锚点列表
 *
 * @param reference_line 参考线
 * @param anchor_points 输出锚点列表
 */
void ReferenceLineProvider::GetAnchorPoints(
    const ReferenceLine &reference_line,
    std::vector<AnchorPoint> *anchor_points) const {
  CHECK_NOTNULL(anchor_points);

  /**
   * 根据配置的最大约束间隔计算锚点数量
   */
  const double interval = smoother_config_.max_constraint_interval();
  int num_of_anchors =
      std::max(2, static_cast<int>(reference_line.Length() / interval + 0.5));
  std::vector<double> anchor_s;

  /**
   * uniform_slice均匀分割区间
   */
  common::util::uniform_slice(0.0, reference_line.Length(), num_of_anchors - 1,
                              &anchor_s);

  /**
   * 为每个s值生成锚点
   */
  for (const double s : anchor_s) {
    AnchorPoint anchor = GetAnchorPoint(reference_line, s);
    anchor_points->emplace_back(anchor);
  }

  /**
   * 设置首尾锚点为强制约束
   */
  anchor_points->front().longitudinal_bound = 1e-6;
  anchor_points->front().lateral_bound = 1e-6;
  anchor_points->front().enforced = true;
  anchor_points->back().longitudinal_bound = 1e-6;
  anchor_points->back().lateral_bound = 1e-6;
  anchor_points->back().enforced = true;
}

/**
 * @brief 平滑路由段
 *
 * @param segments 路由段
 * @param reference_line 输出参考线
 * @return bool 平滑是否成功
 */
bool ReferenceLineProvider::SmoothRouteSegment(const RouteSegments &segments,
                                               ReferenceLine *reference_line) {
  hdmap::Path path(segments);
  return SmoothReferenceLine(ReferenceLine(path), reference_line);
}

/**
 * @brief 平滑带前缀的参考线
 *
 * 用于扩展时的平滑，保持与已有参考线的连接
 *
 * @param prefix_ref 前缀参考线
 * @param raw_ref 待平滑的原始参考线
 * @param reference_line 输出平滑后的参考线
 * @return bool 平滑是否成功
 */
bool ReferenceLineProvider::SmoothPrefixedReferenceLine(
    const ReferenceLine &prefix_ref, const ReferenceLine &raw_ref,
    ReferenceLine *reference_line) {

  /**
   * 如果禁用平滑，直接复制
   */
  if (!FLAGS_enable_smooth_reference_line) {
    *reference_line = raw_ref;
    return true;
  }

  /**
   * 生成锚点
   */
  // generate anchor points:
  std::vector<AnchorPoint> anchor_points;
  GetAnchorPoints(raw_ref, &anchor_points);

  /**
   * 根据前缀参考线修改锚点
   */
  // modify anchor points based on prefix_ref
  for (auto &point : anchor_points) {
    common::SLPoint sl_point;
    if (!prefix_ref.XYToSL(point.path_point, &sl_point)) {
      continue;
    }
    if (sl_point.s() < 0 || sl_point.s() > prefix_ref.Length()) {
      continue;
    }

    /**
     * 使用前缀参考线上的点
     */
    auto prefix_ref_point = prefix_ref.GetNearestReferencePoint(sl_point.s());
    point.path_point.set_x(prefix_ref_point.x());
    point.path_point.set_y(prefix_ref_point.y());
    point.path_point.set_z(0.0);
    point.path_point.set_theta(prefix_ref_point.heading());
    point.longitudinal_bound = 1e-6;
    point.lateral_bound = 1e-6;
    point.enforced = true;
    break;
  }

  /**
   * 设置锚点并平滑
   */
  smoother_->SetAnchorPoints(anchor_points);
  if (!smoother_->Smooth(raw_ref, reference_line)) {
    AERROR << "Failed to smooth prefixed reference line with anchor points";
    return false;
  }

  /**
   * 检查平滑是否有效
   */
  if (!IsReferenceLineSmoothValid(raw_ref, *reference_line)) {
    AERROR << "The smoothed reference line error is too large";
    return false;
  }
  return true;
}

/**
 * @brief 平滑参考线
 *
 * @param raw_reference_line 原始参考线
 * @param reference_line 输出平滑后的参考线
 * @return bool 平滑是否成功
 */
bool ReferenceLineProvider::SmoothReferenceLine(
    const ReferenceLine &raw_reference_line, ReferenceLine *reference_line) {
  if (!FLAGS_enable_smooth_reference_line) {
    *reference_line = raw_reference_line;
    return true;
  }

  /**
   * 生成锚点
   */
  // generate anchor points:
  std::vector<AnchorPoint> anchor_points;
  GetAnchorPoints(raw_reference_line, &anchor_points);

  /**
   * 设置锚点并平滑
   */
  smoother_->SetAnchorPoints(anchor_points);
  if (!smoother_->Smooth(raw_reference_line, reference_line)) {
    AERROR << "Failed to smooth reference line with anchor points";
    return false;
  }

  /**
   * 检查平滑是否有效
   */
  if (!IsReferenceLineSmoothValid(raw_reference_line, *reference_line)) {
    AERROR << "The smoothed reference line error is too large";
    return false;
  }
  return true;
}

/**
 * @brief 获取自车waypoint
 *
 * @param waypoint 输出waypoint
 * @return bool 获取是否成功
 */
bool ReferenceLineProvider::GetAdcWaypoint(hdmap::LaneWaypoint* waypoint) const {
  if (nullptr == current_pnc_map_) {
    AERROR << "Cannot find pnc map to get adc waypoint!";
    return false;
  }
  *waypoint = current_pnc_map_->GetAdcWaypoint();
  return true;
}

/**
 * @brief 获取到目的地的距离
 *
 * @param dis 输出距离
 * @return bool 获取是否成功
 */
bool ReferenceLineProvider::GetAdcDis2Destination(double *dis) const {
  if (nullptr == current_pnc_map_) {
    AERROR << "Cannot find pnc map to get adc distance to destination!";
    return false;
  }
  *dis = current_pnc_map_->GetDistanceToDestination();
  return true;
}

}  // namespace planning
}  // namespace apollo
