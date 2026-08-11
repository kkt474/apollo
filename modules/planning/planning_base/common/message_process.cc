/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
 * @file message_process.cc
 * @brief 消息处理实现文件
 *
 * 功能说明：
 * 处理规划模块收到的各种消息（Chassis、Localization、Prediction、Routing等）
 * 用于在线学习和离线学习的数据处理
 *
 * 主要功能：
 * 1. 消息初始化和关闭 - Init/Close
 * 2. 底盘消息处理 - OnChassis
 * 3. 定位消息处理 - OnLocalization
 * 4. 预测消息处理 - OnPrediction
 * 5. 路由消息处理 - OnRoutingResponse
 * 6. 交通灯消息处理 - OnTrafficLightDetection
 * 7. 故事消息处理 - OnStoryTelling
 * 8. 离线数据处理 - ProcessOfflineData
 * 9. 学习数据帧生成 - GenerateLearningDataFrame
 *
 * 设计模式：
 * - 回调函数模式：各OnXxx函数作为消息回调
 * - 数据积累模式：使用deque积累历史数据
 * - 相对坐标转换：将世界坐标转换为自车相对坐标
 *
 * C++语法说明：
 * - std::deque<T>：双端队列，支持前后高效插入
 * - std::shared_ptr<T>：共享所有权的智能指针
 * - std::chrono：时间处理命名空间
 * - Protobuf消息：*.ParseFromString()从字符串解析
 * - std::ios_base::out | std::ios_base::app：文件打开模式组合
 * - absl::StrCat：高效字符串拼接
 */
#include "modules/planning/planning_base/common/message_process.h"

#include <algorithm>      /**< C++标准算法库：std::max, std::min, std::reverse等 */
#include <cmath>         /**< C++数学库：std::sqrt, std::abs等 */
#include <memory>         /**< C++智能指针：std::shared_ptr */
#include <string>         /**< C++字符串库 */

#include "modules/common_msgs/map_msgs/map_lane.pb.h"

#include "cyber/common/file.h"     /**< Cyber RT文件操作 */
#include "cyber/record/record_reader.h"  /**< Cyber RT录制文件读取 */
#include "cyber/time/clock.h"     /**< Cyber RT时间获取 */
#include "modules/common/adapters/adapter_gflags.h"  /**< 适配器配置参数 */
#include "modules/common/util/point_factory.h"  /**< 位置点工厂 */
#include "modules/common/util/util.h"  /**< 通用工具函数 */
#include "modules/map/hdmap/hdmap_util.h"  /**< 高精地图工具 */
#include "modules/planning/planning_base/common/feature_output.h"  /**< 特征输出 */
#include "modules/planning/planning_base/common/util/math_util.h"  /**< 数学工具 */
#include "modules/planning/planning_base/gflags/planning_gflags.h"  /**< 规划模块配置 */

namespace apollo {
/**
 * @namespace apollo::
 * @brief Apollo最外层命名空间
 */
namespace planning {

/**
 * @brief using类型别名声明
 *
 * using是C++11的类型别名声明方式，比typedef更直观
 * 作用：简化类型名，避免重复写完整命名空间
 *
 * C++语法说明：
 * - using A = B：A是B的别名
 * - apollo::canbus::Chassis：完整命名空间路径
 * - const引用返回类型：返回常量引用，避免拷贝
 */
using apollo::canbus::Chassis;                    /**< 底盘消息类型 */
using apollo::cyber::Clock;                       /**< Cyber RT时钟类型 */
using apollo::cyber::record::RecordMessage;      /**< 录制消息类型 */
using apollo::cyber::record::RecordReader;        /**< 录制文件读取器 */
using apollo::dreamview::HMIStatus;               /**< HMI状态类型 */
using apollo::hdmap::ClearAreaInfoConstPtr;       /**< 清除区域信息常量指针 */
using apollo::hdmap::CrosswalkInfoConstPtr;       /**< 人行横道信息常量指针 */
using apollo::hdmap::HDMapUtil;                   /**< 高精地图工具类 */
using apollo::hdmap::JunctionInfoConstPtr;         /**< 交叉路口信息常量指针 */
using apollo::hdmap::LaneInfoConstPtr;            /**< 车道信息常量指针 */
using apollo::hdmap::PNCJunctionInfoConstPtr;      /**< PNC交叉路口信息常量指针 */
using apollo::hdmap::SignalInfoConstPtr;          /**< 信号灯信息常量指针 */
using apollo::hdmap::StopSignInfoConstPtr;        /**< 停车标志信息常量指针 */
using apollo::hdmap::YieldSignInfoConstPtr;       /**< 让行标志信息常量指针 */
using apollo::localization::LocalizationEstimate;  /**< 定位估计消息类型 */
using apollo::perception::TrafficLightDetection;  /**< 交通灯检测消息类型 */
using apollo::prediction::PredictionObstacle;     /**< 预测障碍物类型 */
using apollo::prediction::PredictionObstacles;    /**< 预测障碍物列表类型 */
using apollo::routing::RoutingResponse;           /**< 路由响应消息类型 */
using apollo::storytelling::CloseToJunction;     /**< 接近交叉路口故事类型 */
using apollo::storytelling::Stories;               /**< 故事列表类型 */

/**
 * @brief 消息处理器初始化
 *
 * @param planning_config 规划配置消息
 * @return bool 初始化是否成功
 *
 * 功能说明：
 * 1. 复制规划配置
 * 2. 初始化地图名称映射
 * 3. 设置离线学习日志文件
 *
 * C++语法说明：
 * - planning_config_.CopyFrom(planning_config)：
 *   Protobuf消息的CopyFrom方法，深拷贝另一个消息的内容
 *   将planning_config的所有字段复制到planning_config_
 *
 * - map_m_["Sunnyvale"] = "sunnyvale"：
 *   std::map的下标操作符[]
 *   如果key不存在，会插入新的key-value对
 *   用于初始化地图名称映射表
 *
 * - FLAGS_map_dir.substr(FLAGS_map_dir.find_last_of("/") + 1)：
 *   - FLAGS_xxx：gflags库的全局变量
 *   - substr(pos)：返回从pos开始的子字符串
 *   - find_last_of("/")：查找最后一个"/"的位置
 *   - +1：跳过"/"字符，获取地图文件夹名称
 *
 * - obstacle_history_map_.clear()：
 *   清空障碍物历史map
 *   clear()移除所有元素
 *
 * - std::ios_base::out | std::ios_base::app：
 *   文件打开模式组合
 *   - out：写模式
 *   - app：追加模式（每次写入移到文件末尾）
 *   - |：位运算符，组合多个模式
 *
 * - std::chrono::system_clock::now()：
 *   获取系统时钟的当前时间点
 *   std::chrono用于时间处理
 *
 * - std::time(nullptr)：
 *   获取当前时间（time_t类型）
 *   nullptr表示传递给time()的参数不使用
 *
 * - std::gmtime(&now) / std::localtime(&now)：
 *   将time_t转换为分解时间结构(tm)
 *   - gmtime：UTC时间
 *   - localtime：本地时间
 *
 * - std::asctime(tm*)：
 *   将tm结构转换为可读字符串
 *   格式：Thu Jan 31 00:00:00 2020\n
 */
bool MessageProcess::Init(const PlanningConfig& planning_config) {
  // 复制规划配置到成员变量
  planning_config_.CopyFrom(planning_config);

  // 初始化地图名称映射表
  // 将HMI显示的地图名映射到实际地图文件夹名
  map_m_["Sunnyvale"] = "sunnyvale";
  map_m_["Sunnyvale Big Loop"] = "sunnyvale_big_loop";
  map_m_["Sunnyvale With Two Offices"] = "sunnyvale_with_two_offices";
  map_m_["Gomentum"] = "gomentum";
  map_m_["Sunnyvale Loop"] = "sunnyvale_loop";
  map_m_["San Mateo"] = "san_mateo";

  // 从FLAGS_map_dir提取地图名称
  // find_last_of("/")找到最后一个"/"的位置
  // substr(+1)获取"/"后面的部分作为地图名
  map_name_ = FLAGS_map_dir.substr(FLAGS_map_dir.find_last_of("/") + 1);

  // 清空障碍物历史记录
  obstacle_history_map_.clear();

  // 检查是否启用离线学习模式
  if (FLAGS_planning_offline_learning) {
    // 打开日志文件用于离线学习记录
    // std::ios_base::out | std::ios_base::app：写模式+追加模式
    log_file_.open(FLAGS_planning_data_dir + "/learning_data.log",
                   std::ios_base::out | std::ios_base::app);

    // 记录开始时间
    start_time_ = std::chrono::system_clock::now();

    // 获取当前时间
    std::time_t now = std::time(nullptr);

    // 写入UTC时间和本地时间到日志
    log_file_ << "UTC date and time: " << std::asctime(std::gmtime(&now))
              << "Local date and time: " << std::asctime(std::localtime(&now));
  }
  return true;
}

/**
 * @brief 消息处理器初始化（带依赖注入器版本）
 *
 * @param planning_config 规划配置
 * @param injector 依赖注入器智能指针
 * @return bool 初始化是否成功
 *
 * C++语法说明：
 * - const std::shared_ptr<DependencyInjector>& injector：
 *   依赖注入器的常量引用智能指针
 *   - std::shared_ptr：共享所有权的智能指针
 *   - const引用：避免拷贝，提高效率
 *
 * - injector_ = injector：
 *   赋值智能指针
 *   shared_ptr的引用计数+1
 *
 * - return Init(planning_config)：
 *   调用另一个Init重载
 *   代码复用，避免重复
 */
bool MessageProcess::Init(const PlanningConfig& planning_config,
                          const std::shared_ptr<DependencyInjector>& injector) {
  // 保存依赖注入器指针
  injector_ = injector;
  // 委托给另一个Init重载完成初始化
  return Init(planning_config);
}

/**
 * @brief 消息处理器关闭
 *
 * 功能说明：
 * 1. 清除特征输出
 * 2. 关闭离线学习日志文件
 * 3. 写入统计信息
 *
 * C++语法说明：
 * - FeatureOutput::Clear()：
 *   静态方法调用
 *   ::是命名空间/作用域运算符
 *
 * - absl::StrCat(...)：
 *   Abseil库的高效字符串拼接函数
 *   比+运算符更高效，支持多种类型自动转换
 *
 * - std::chrono::duration<double>：
 *   duration表示时间间隔
 *   double表示用秒作为单位
 *
 * - elapsed_seconds.count()：
 *   count()返回duration中的时间单位数量
 *   对于duration<double>，返回秒数
 *
 * - log_file_.close()：
 *   关闭文件
 *   刷新缓冲区，确保数据写入
 */
void MessageProcess::Close() {
  // 清除特征输出
  FeatureOutput::Clear();

  // 检查是否启用了离线学习模式
  if (FLAGS_planning_offline_learning) {
    // 构造日志消息：总学习数据帧数
    const std::string msg = absl::StrCat("Total learning_data_frame number: ",
                                         total_learning_data_frame_num_);
    // 使用AINFO输出信息日志
    AINFO << msg;
    // 写入日志文件
    log_file_ << msg << std::endl;

    // 计算经过的时间
    auto end_time = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end_time - start_time_;

    // 写入时间统计
    log_file_ << "Time elapsed(sec): " << elapsed_seconds.count() << std::endl
              << std::endl;

    // 关闭日志文件
    log_file_.close();
  }
}

/**
 * @brief 处理底盘消息
 *
 * @param chassis 底盘消息常量引用
 *
 * 功能说明：
 * 将底盘信息保存到chassis_feature_
 * 用于后续生成学习数据帧
 *
 * C++语法说明：
 * - chassis_feature_.set_xxx(value)：
 *   Protobuf消息的setter方法
 *   设置字段的值
 *
 * - chassis.header().timestamp_sec()：
 *   链式调用
 *   header()返回Header消息引用
 *   timestamp_sec()获取时间戳
 *
 * - chassis.speed_mps()：
 *   获取底盘速度（米/秒）
 *   mps = meters per second
 */
void MessageProcess::OnChassis(const apollo::canbus::Chassis& chassis) {
  // 保存时间戳（秒）
  chassis_feature_.set_message_timestamp_sec(chassis.header().timestamp_sec());
  // 保存速度（米/秒）
  chassis_feature_.set_speed_mps(chassis.speed_mps());
  // 保存油门百分比
  chassis_feature_.set_throttle_percentage(chassis.throttle_percentage());
  // 保存刹车百分比
  chassis_feature_.set_brake_percentage(chassis.brake_percentage());
  // 保存方向盘百分比
  chassis_feature_.set_steering_percentage(chassis.steering_percentage());
  // 保存档位位置
  chassis_feature_.set_gear_location(chassis.gear_location());
}

/**
 * @brief 处理HMI状态消息
 *
 * @param hmi_status HMI状态消息
 *
 * 功能说明：
 * 根据HMI选择的地图更新FLAGS_map_dir
 * 用于动态切换地图
 *
 * C++语法说明：
 * - map_m_.count(current_map)：
 *   map的count方法返回key出现的次数（0或1）
 *   用于检查key是否存在
 *
 * - FLAGS_map_dir = map_base_folder + map_name_：
 *   gflags变量可以直接赋值
 *   更新全局地图目录配置
 */
void MessageProcess::OnHMIStatus(apollo::dreamview::HMIStatus hmi_status) {
  // 获取当前地图名称
  const std::string& current_map = hmi_status.current_map();

  // 检查是否在映射表中
  if (map_m_.count(current_map) > 0) {
    // 更新地图名称
    map_name_ = map_m_[current_map];
    // 构造新的地图目录路径
    const std::string& map_base_folder = "/apollo/modules/map/data/";
    FLAGS_map_dir = map_base_folder + map_name_;
  }
}

/**
 * @brief 处理定位消息
 *
 * @param le 定位估计消息
 *
 * 功能说明：
 * 1. 检查定位消息时间间隔
 * 2. 积累定位历史
 * 3. 生成学习数据帧
 *
 * C++语法说明：
 * - static constexpr double kEpsilon = 1e-12：
 *   static：静态成员，类级别共享
 *   constexpr：编译时常量
 *   kEpsilon：极小值，用于浮点数比较
 *
 * - std::abs(...) < kEpsilon：
 *   std::abs用于绝对值
 *   检查时间戳是否为初始值（0或接近0）
 *
 * - if (time_diff < 1.0 / FLAGS_planning_loop_rate)：
 *   检查时间差是否小于规划循环周期
 *   1.0 / FLAGS_planning_loop_rate = 1/10 = 0.1秒（假设10Hz）
 *
 * - planning_config_.learning_mode() == PlanningConfig::RL_TEST：
 *   获取学习模式的枚举值比较
 *   用于判断是否为特殊测试模式
 *
 * - localizations_.push_back(le)：
 *   deque的push_back
 *   在末尾添加元素
 *
 * - localizations_.back().header().timestamp_sec()：
 *   back()返回最后一个元素的引用
 *   用于获取最新消息的时间戳
 *
 * - localizations_.pop_front()：
 *   deque的pop_front
 *   移除第一个元素
 *   用于维护固定长度的历史
 *
 * - while (!localizations_.empty())：
 *   循环直到历史长度符合要求
 *   empty()返回容器是否为空
 *
 * - FLAGS_trajectory_time_length：
 *   轨迹时间长度（秒）
 *   用于判断是否需要移除旧数据
 */
void MessageProcess::OnLocalization(const LocalizationEstimate& le) {
  // 静态常量：极小值，用于浮点数比较
  static constexpr double kEpsilon = 1e-12;

  // 检查是否首次接收定位消息
  if (std::abs(last_localization_message_timestamp_sec_) < kEpsilon) {
    // 记录首次接收的时间戳
    last_localization_message_timestamp_sec_ = le.header().timestamp_sec();
  }

  // 计算与上一次定位消息的时间差
  const double time_diff =
      le.header().timestamp_sec() - last_localization_message_timestamp_sec_;

  // 检查时间差是否小于规划周期
  if (time_diff < 1.0 / FLAGS_planning_loop_rate) {
    // 对于RL_TEST、E2E_TEST或HYBRID_TEST模式，跳过此检查
    // 以便第一帧可以继续处理
    if (!(planning_config_.learning_mode() == PlanningConfig::RL_TEST ||
          planning_config_.learning_mode() == PlanningConfig::E2E_TEST ||
          planning_config_.learning_mode() == PlanningConfig::HYBRID_TEST)) {
      // 时间间隔太短，忽略此消息
      return;
    }
  }

  // 检查是否丢失定位太久
  if (time_diff >= (1.0 * 2 / FLAGS_planning_loop_rate)) {
    // 构造错误消息
    const std::string msg = absl::StrCat(
        "missing localization too long: time_stamp[",
        le.header().timestamp_sec(), "] time_diff[", time_diff, "]");
    // 输出错误日志
    AERROR << msg;
    // 如果是离线学习模式，同时写入日志文件
    if (FLAGS_planning_offline_learning) {
      log_file_ << msg << std::endl;
    }
  }

  // 更新最后接收时间戳
  last_localization_message_timestamp_sec_ = le.header().timestamp_sec();

  // 将新定位消息添加到历史队列
  localizations_.push_back(le);

  // 维护历史队列长度，移除超出时间范围的旧数据
  while (!localizations_.empty()) {
    // 如果最旧和最新消息的时间差超过阈值，移除最旧的
    if (localizations_.back().header().timestamp_sec() -
            localizations_.front().header().timestamp_sec() <=
        FLAGS_trajectory_time_length) {
      // 时间差在范围内，停止移除
      break;
    }
    // 移除最旧的定位消息
    localizations_.pop_front();
  }

  // 输出调试信息
  ADEBUG << "OnLocalization: size[" << localizations_.size() << "] time_diff["
         << localizations_.back().header().timestamp_sec() -
                localizations_.front().header().timestamp_sec()
         << "]";

  // 生成学习数据帧
  LearningDataFrame learning_data_frame;
  if (GenerateLearningDataFrame(&learning_data_frame)) {
    // 输出
    if (FLAGS_planning_offline_learning) {
      // 离线模式：写入文件
      FeatureOutput::InsertLearningDataFrame(record_file_, learning_data_frame);
    } else {
      // 在线模式：插入到学习数据结构
      injector_->learning_based_data()->InsertLearningDataFrame(
          learning_data_frame);
    }
  }
}

/**
 * @brief 处理预测障碍物消息
 *
 * @param prediction_obstacles 预测障碍物列表消息
 *
 * 功能说明：
 * 1. 解析预测障碍物列表
 * 2. 更新障碍物历史轨迹
 * 3. 维护障碍物历史数据结构
 *
 * C++语法说明：
 * - prediction_obstacles.prediction_obstacle_size()：
 *   Protobuf的repeated字段大小
 *   返回预测障碍物的数量
 *
 * - prediction_obstacles.prediction_obstacle(i)：
 *   repeated字段的下标访问
 *   返回第i个PredictionObstacle
 *
 * - prediction_obstacles_map_[obstacle_id].CopyFrom(prediction_obstacle)：
 *   map的下标访问+Protobuf的CopyFrom
 *   如果key不存在会插入新的key-value
 *
 * - obstacle_history_map_[m.first]：
 *   m是map的键值对
 *   m.first是key（障碍物ID）
 *   m.second是value（障碍物历史列表）
 *
 * - mutable_xxx()：
 *   Protobuf的可变字段访问器
 *   返回指向字段的指针，允许修改
 *
 * - add_xxx()：
 *   Protobuf的repeated字段添加方法
 *   添加一个元素并返回其指针
 *
 * - std::unordered_map<int, std::list<PerceptionObstacleFeature>>：
 *   外层map：障碍物ID到历史列表的映射
 *   内层list：按时间排序的历史特征列表
 */
void MessageProcess::OnPrediction(
    const PredictionObstacles& prediction_obstacles) {
  // 清空当前帧的预测障碍物映射
  prediction_obstacles_map_.clear();

  // 遍历所有预测障碍物
  for (int i = 0; i < prediction_obstacles.prediction_obstacle_size(); ++i) {
    // 获取第i个预测障碍物
    const auto& prediction_obstacle =
        prediction_obstacles.prediction_obstacle(i);
    // 获取障碍物ID
    const int obstacle_id = prediction_obstacle.perception_obstacle().id();
    // 复制到映射表中
    prediction_obstacles_map_[obstacle_id].CopyFrom(prediction_obstacle);
  }

  // 注释掉的代码：移除不在当前预测中的障碍物历史
  // 目前放宽了此检查
  /*
  std::unordered_map<int, std::list<PerceptionObstacleFeature>>::iterator
      it = obstacle_history_map_.begin();
  while (it != obstacle_history_map_.end()) {
    const int obstacle_id = it->first;
    if (prediction_obstacles_map_.count(obstacle_id) == 0) {
      // not exist in current prediction msg
      it = obstacle_history_map_.erase(it);
    } else {
      ++it;
    }
  }
  */

  // 调试代码：输出障碍物历史（已注释）
  // add to obstacle history
  // for (const auto& m : obstacle_history_map_) {
  //  for (const auto& p :  m.second) {
  //    AERROR << "obstacle_history_map_: " << m.first << "; "
  //           << p.DebugString();
  //  }
  // }

  // 遍历当前预测障碍物，更新历史轨迹
  for (const auto& m : prediction_obstacles_map_) {
    // 获取感知障碍物
    const auto& perception_obstale = m.second.perception_obstacle();

    // 创建障碍物轨迹点特征
    PerceptionObstacleFeature obstacle_trajectory_point;

    // 设置时间戳
    obstacle_trajectory_point.set_timestamp_sec(perception_obstale.timestamp());

    // 复制位置
    obstacle_trajectory_point.mutable_position()->CopyFrom(
        perception_obstale.position());

    // 设置朝向角
    obstacle_trajectory_point.set_theta(perception_obstale.theta());

    // 复制速度
    obstacle_trajectory_point.mutable_velocity()->CopyFrom(
        perception_obstale.velocity());

    // 复制多边形顶点
    for (int j = 0; j < perception_obstale.polygon_point_size(); ++j) {
      auto polygon_point = obstacle_trajectory_point.add_polygon_point();
      polygon_point->CopyFrom(perception_obstale.polygon_point(j));
    }

    // 复制加速度
    obstacle_trajectory_point.mutable_acceleration()->CopyFrom(
        perception_obstale.acceleration());

    // 检查障碍物历史并添加新的轨迹点
    obstacle_history_map_[m.first].back().timestamp_sec();
    if (obstacle_history_map_[m.first].empty() ||
        obstacle_trajectory_point.timestamp_sec() -
                obstacle_history_map_[m.first].back().timestamp_sec() >
            0) {
      // 添加到历史列表末尾
      obstacle_history_map_[m.first].push_back(obstacle_trajectory_point);
    } else {
      // 异常感知数据：时间差<=0
      const double time_diff =
          obstacle_trajectory_point.timestamp_sec() -
          obstacle_history_map_[m.first].back().timestamp_sec();
      const std::string msg = absl::StrCat(
          "DISCARD: obstacle_id[", m.first, "] last_timestamp_sec[",
          obstacle_history_map_[m.first].back().timestamp_sec(),
          "] timestamp_sec[", obstacle_trajectory_point.timestamp_sec(),
          "] time_diff[", time_diff, "]");
      AERROR << msg;
      if (FLAGS_planning_offline_learning) {
        log_file_ << msg << std::endl;
      }
    }

    // 获取障碍物历史的引用
    auto& obstacle_history = obstacle_history_map_[m.first];

    // 维护历史长度，移除超出时间范围的旧数据
    while (!obstacle_history.empty()) {
      const double time_distance = obstacle_history.back().timestamp_sec() -
                                   obstacle_history.front().timestamp_sec();
      if (time_distance < FLAGS_learning_data_obstacle_history_time_sec) {
        break;
      }
      obstacle_history.pop_front();
    }
  }
}

/**
 * @brief 处理路由响应消息
 *
 * @param routing_response 路由响应消息
 *
 * 功能说明：
 * 保存路由响应到成员变量
 * 用于后续生成学习数据帧
 *
 * C++语法说明：
 * - routing_response_.CopyFrom(routing_response)：
 *   Protobuf消息深拷贝
 *   保存路由响应供后续使用
 */
void MessageProcess::OnRoutingResponse(
    const apollo::routing::RoutingResponse& routing_response) {
  // 输出调试信息
  ADEBUG << "routing_response received at frame["
         << total_learning_data_frame_num_ << "]";
  // 保存路由响应
  routing_response_.CopyFrom(routing_response);
}

/**
 * @brief 处理故事消息
 *
 * @param stories 故事列表消息
 *
 * 功能说明：
 * 从stories中提取各种交通元素的接近信息
 * 并保存到planning_tag_
 *
 * C++语法说明：
 * - stories.has_close_to_clear_area()：
 *   Protobuf的has_方法
 *   检查可选字段是否已设置
 *
 * - planning_tag_.mutable_clear_area()：
 *   mutable_xxx()返回可变指针
 *   用于设置子消息字段
 *
 * - set_id() / set_distance()：
 *   Protobuf的setter方法
 *   设置字段值
 *
 * - CloseToJunction::PNC_JUNCTION：
 *   枚举类型的作用域限定
 *   访问枚举值
 */
void MessageProcess::OnStoryTelling(
    const apollo::storytelling::Stories& stories) {
  // 清除区域
  if (stories.has_close_to_clear_area()) {
    // 获取可变的clear_area子消息
    auto clear_area_tag = planning_tag_.mutable_clear_area();
    // 设置ID和距离
    clear_area_tag->set_id(stories.close_to_clear_area().id());
    clear_area_tag->set_distance(stories.close_to_clear_area().distance());
  }

  // 人行横道
  if (stories.has_close_to_crosswalk()) {
    auto crosswalk_tag = planning_tag_.mutable_crosswalk();
    crosswalk_tag->set_id(stories.close_to_crosswalk().id());
    crosswalk_tag->set_distance(stories.close_to_crosswalk().distance());
  }

  // PNC交叉路口
  if (stories.has_close_to_junction() &&
      stories.close_to_junction().type() == CloseToJunction::PNC_JUNCTION) {
    auto pnc_junction_tag = planning_tag_.mutable_pnc_junction();
    pnc_junction_tag->set_id(stories.close_to_junction().id());
    pnc_junction_tag->set_distance(stories.close_to_junction().distance());
  }

  // 交通灯
  if (stories.has_close_to_signal()) {
    auto signal_tag = planning_tag_.mutable_signal();
    signal_tag->set_id(stories.close_to_signal().id());
    signal_tag->set_distance(stories.close_to_signal().distance());
  }

  // 停车标志
  if (stories.has_close_to_stop_sign()) {
    auto stop_sign_tag = planning_tag_.mutable_stop_sign();
    stop_sign_tag->set_id(stories.close_to_stop_sign().id());
    stop_sign_tag->set_distance(stories.close_to_stop_sign().distance());
  }

  // 让行标志
  if (stories.has_close_to_yield_sign()) {
    auto yield_sign_tag = planning_tag_.mutable_yield_sign();
    yield_sign_tag->set_id(stories.close_to_yield_sign().id());
    yield_sign_tag->set_distance(stories.close_to_yield_sign().distance());
  }

  // 输出调试信息
  ADEBUG << planning_tag_.DebugString();
}

/**
 * @brief 处理交通灯检测消息
 *
 * @param traffic_light_detection 交通灯检测消息
 *
 * 功能说明：
 * 解析交通灯检测结果
 * 提取每个交通灯的颜色、ID、置信度等信息
 *
 * C++语法说明：
 * - traffic_light_detection.traffic_light_size()：
 *   repeated字段的大小
 *   返回交通灯数量
 *
 * - traffic_light_detection.traffic_light(i)：
 *   获取第i个交通灯
 *
 * - set_xxx()：
 *   Protobuf setter方法
 *   设置特征消息的字段
 *
 * - traffic_lights_.push_back(traffic_light)：
 *   vector的push_back
 *   保存交通灯特征列表
 */
void MessageProcess::OnTrafficLightDetection(
    const TrafficLightDetection& traffic_light_detection) {
  // 记录消息时间戳
  traffic_light_detection_message_timestamp_ =
      traffic_light_detection.header().timestamp_sec();

  // 清空之前的交通灯列表
  traffic_lights_.clear();

  // 遍历所有检测到的交通灯
  for (int i = 0; i < traffic_light_detection.traffic_light_size(); ++i) {
    // 创建交通灯特征
    TrafficLightFeature traffic_light;
    // 设置颜色（红/黄/绿）
    traffic_light.set_color(traffic_light_detection.traffic_light(i).color());
    // 设置ID
    traffic_light.set_id(traffic_light_detection.traffic_light(i).id());
    // 设置置信度
    traffic_light.set_confidence(
        traffic_light_detection.traffic_light(i).confidence());
    // 设置跟踪时间
    traffic_light.set_tracking_time(
        traffic_light_detection.traffic_light(i).tracking_time());
    // 设置剩余时间
    traffic_light.set_remaining_time(
        traffic_light_detection.traffic_light(i).remaining_time());
    // 添加到列表
    traffic_lights_.push_back(traffic_light);
  }
}

/**
 * @brief 处理离线数据
 *
 * @param record_file 录制文件路径
 *
 * 功能说明：
 * 读取录制文件中的消息
 * 并分发给相应的处理函数
 *
 * C++语法说明：
 * - RecordReader(record_file)：
 *   构造函数，创建录制文件读取器
 *   直接初始化reader对象
 *
 * - reader.IsValid()：
 *   检查读取器是否有效
 *   可能失败：文件不存在、格式错误等
 *
 * - reader.ReadMessage(&message)：
 *   读取下一条消息
 *   返回bool表示是否成功
 *   message通过指针参数输出
 *
 * - message.channel_name：
 *   获取消息的通道名称
 *   用于判断消息类型
 *
 * - chassis.ParseFromString(message.content)：
 *   Protobuf的ParseFromString方法
 *   从字符串解析消息
 *   返回bool表示是否成功解析
 */
void MessageProcess::ProcessOfflineData(const std::string& record_file) {
  // 写入日志
  log_file_ << "Processing: " << record_file << std::endl;
  // 保存录制文件名
  record_file_ = record_file;

  // 创建录制文件读取器
  RecordReader reader(record_file);

  // 检查读取器是否有效
  if (!reader.IsValid()) {
    AERROR << "Fail to open " << record_file;
    return;
  }

  // 创建消息对象
  RecordMessage message;

  // 循环读取消息
  while (reader.ReadMessage(&message)) {
    // 根据通道名称分发消息
    if (message.channel_name ==
        planning_config_.topic_config().chassis_topic()) {
      // 底盘消息
      Chassis chassis;
      if (chassis.ParseFromString(message.content)) {
        OnChassis(chassis);
      }
    } else if (message.channel_name ==
               planning_config_.topic_config().localization_topic()) {
      // 定位消息
      LocalizationEstimate localization;
      if (localization.ParseFromString(message.content)) {
        OnLocalization(localization);
      }
    } else if (message.channel_name ==
               planning_config_.topic_config().hmi_status_topic()) {
      // HMI状态消息
      HMIStatus hmi_status;
      if (hmi_status.ParseFromString(message.content)) {
        OnHMIStatus(hmi_status);
      }
    } else if (message.channel_name ==
               planning_config_.topic_config().prediction_topic()) {
      // 预测障碍物消息
      PredictionObstacles prediction_obstacles;
      if (prediction_obstacles.ParseFromString(message.content)) {
        OnPrediction(prediction_obstacles);
      }
    } else if (message.channel_name ==
               planning_config_.topic_config().routing_response_topic()) {
      // 路由响应消息
      RoutingResponse routing_response;
      if (routing_response.ParseFromString(message.content)) {
        OnRoutingResponse(routing_response);
      }
    } else if (message.channel_name ==
               planning_config_.topic_config().story_telling_topic()) {
      // 故事消息
      Stories stories;
      if (stories.ParseFromString(message.content)) {
        OnStoryTelling(stories);
      }
    } else if (message.channel_name == planning_config_.topic_config()
                                           .traffic_light_detection_topic()) {
      // 交通灯检测消息
      TrafficLightDetection traffic_light_detection;
      if (traffic_light_detection.ParseFromString(message.content)) {
        OnTrafficLightDetection(traffic_light_detection);
      }
    }
  }
}

/**
 * @brief 获取自车当前的路由索引
 *
 * @param adc_road_index 输出：自车所在道路索引
 * @param adc_passage_index 输出：自车所在通道索引
 * @param adc_passage_s 输出：自车在通道中的位置
 * @return bool 是否成功找到
 *
 * 功能说明：
 * 根据自车当前位置
 * 找到在路由响应中的道路和通道索引
 *
 * C++语法说明：
 * - if (localizations_.empty()) return false：
 *   防御性检查
 *   空队列直接返回失败
 *
 * - localizations_.back().pose()：
 *   back()获取最后一个元素
 *   pose()获取定位姿态
 *
 * - HDMapUtil::BaseMapPtr()->GetLanes(...)：
 *   BaseMapPtr()返回地图单例指针
 *   ->调用指针方法
 *   GetLanes在给定位置周围搜索车道
 *
 * - routing_response_.road_size()：
 *   repeated字段大小
 *   返回道路数量
 *
 * - *adc_passage_s = passage_s：
 *   *解引用指针并赋值
 *   通过输出参数返回结果
 */
bool MessageProcess::GetADCCurrentRoutingIndex(int* adc_road_index,
                                               int* adc_passage_index,
                                               double* adc_passage_s) {
  // 防御性检查
  if (localizations_.empty()) return false;

  // 搜索半径（米）
  static constexpr double kRadius = 4.0;

  // 获取自车最新位置
  const auto& pose = localizations_.back().pose();

  // 搜索位置周围的车道
  std::vector<std::shared_ptr<const apollo::hdmap::LaneInfo>> lanes;
  apollo::hdmap::HDMapUtil::BaseMapPtr()->GetLanes(pose.position(), kRadius,
                                                   &lanes);

  // 遍历找到的车道
  for (auto& lane : lanes) {
    // 遍历路由响应中的所有道路
    for (int i = 0; i < routing_response_.road_size(); ++i) {
      // 初始化passage位置
      *adc_passage_s = 0;
      // 遍历道路中的所有通道
      for (int j = 0; j < routing_response_.road(i).passage_size(); ++j) {
        double passage_s = 0;
        // 遍历通道中的所有段
        for (int k = 0; k < routing_response_.road(i).passage(j).segment_size();
             ++k) {
          const auto& segment = routing_response_.road(i).passage(j).segment(k);
          // 累加段长度
          passage_s += (segment.end_s() - segment.start_s());
          // 检查是否是自车所在车道
          if (lane->id().id() == segment.id()) {
            *adc_road_index = i;
            *adc_passage_index = j;
            *adc_passage_s = passage_s;
            return true;
          }
        }
      }
    }
  }
  return false;
}

/**
 * @brief 获取当前位置所在的车道
 *
 * @param position ENU位置
 * @return LaneInfoConstPtr 车道信息指针，未找到返回nullptr
 *
 * 功能说明：
 * 在给定位置搜索所属车道
 * 从近及远扩大搜索半径
 *
 * C++语法说明：
 * - constexpr double kRadiusUnit = 0.1：
 *   constexpr：编译时常量
 *   搜索半径单位（0.1米）
 *
 * - for (int i = 1; i <= 10; ++i)：
 *   循环扩大搜索半径
 *   从0.1米到1.0米
 *
 * - HDMapUtil::BaseMapPtr()->GetLanes(position, i * kRadiusUnit, &lanes)：
 *   不断扩大搜索半径直到找到车道
 *
 * - if (lanes.size() > 0) break：
 *   找到至少一个车道，停止搜索
 *
 * - lane->id().id()：
 *   第一个id()获取LaneInfo的ID对象
 *   第二个id()获取ID对象的字符串值
 */
apollo::hdmap::LaneInfoConstPtr MessageProcess::GetCurrentLane(
    const apollo::common::PointENU& position) {
  // 搜索半径单位（0.1米）
  constexpr double kRadiusUnit = 0.1;

  // 存储找到的车道
  std::vector<std::shared_ptr<const apollo::hdmap::LaneInfo>> lanes;

  // 从近到远扩大搜索半径
  for (int i = 1; i <= 10; ++i) {
    // 调用HDMap工具搜索车道
    apollo::hdmap::HDMapUtil::BaseMapPtr()->GetLanes(position, i * kRadiusUnit,
                                                     &lanes);
    // 找到至少一个车道，停止搜索
    if (lanes.size() > 0) {
      break;
    }
  }

  // 在找到的车道中查找是否在路由路径上
  for (auto& lane : lanes) {
    // 遍历路由响应中的道路
    for (int i = 0; i < routing_response_.road_size(); ++i) {
      // 遍历通道
      for (int j = 0; j < routing_response_.road(i).passage_size(); ++j) {
        // 遍历通道中的段
        for (int k = 0; k < routing_response_.road(i).passage(j).segment_size();
             ++k) {
          // 检查车道是否在路由路径上
          if (lane->id().id() ==
              routing_response_.road(i).passage(j).segment(k).id()) {
            // 返回找到的车道
            return lane;
          }
        }
      }
    }
  }

  // 未找到在路由路径上的车道
  return nullptr;
}

/**
 * @brief 获取自车当前信息
 *
 * @param adc_curr_info 输出：自车当前信息
 * @return int 成功返回1，失败返回-1
 *
 * 功能说明：
 * 提取自车的位置、速度、加速度、朝向等信息
 * 保存到ADCCurrentInfo结构中
 *
 * C++语法说明：
 * - CHECK_NOTNULL(adc_curr_info)：
 *   Apollo断言宏
 *   检查指针是否为nullptr
 *
 * - std::make_pair(x, y)：
 *   创建pair对象
 *   用于存储x,y坐标
 *
 * - adc_cur_pose.position().x()：
 *   链式调用获取x坐标
 *   position()返回Point对象引用
 *   x()获取x坐标值
 *
 * - std::make_pair(adc_cur_pose.linear_velocity().x(), ...)：
 *   创建速度pair
 *   linear_velocity()返回Vector3d
 */
int MessageProcess::GetADCCurrentInfo(ADCCurrentInfo* adc_curr_info) {
  // 断言检查
  CHECK_NOTNULL(adc_curr_info);
  // 防御性检查
  if (localizations_.empty()) return -1;

  // 获取自车当前姿态
  const auto& adc_cur_pose = localizations_.back().pose();

  // 设置当前位置 (x, y)
  adc_curr_info->adc_cur_position_ =
      std::make_pair(adc_cur_pose.position().x(), adc_cur_pose.position().y());

  // 设置当前速度 (vx, vy)
  adc_curr_info->adc_cur_velocity_ = std::make_pair(
      adc_cur_pose.linear_velocity().x(), adc_cur_pose.linear_velocity().y());

  // 设置当前加速度 (ax, ay)
  adc_curr_info->adc_cur_acc_ =
      std::make_pair(adc_cur_pose.linear_acceleration().x(),
                     adc_cur_pose.linear_acceleration().y());

  // 设置当前朝向角
  adc_curr_info->adc_cur_heading_ = adc_cur_pose.heading();

  return 1;
}

/**
 * @brief 生成障碍物轨迹
 *
 * @param frame_num 帧号
 * @param obstacle_id 障碍物ID
 * @param adc_curr_info 自车当前信息（用于坐标转换）
 * @param obstacle_feature 输出：障碍物特征
 *
 * 功能说明：
 * 将障碍物的历史轨迹点
 * 从世界坐标转换为自车相对坐标
 *
 * C++语法说明：
 * - obstacle_feature->mutable_obstacle_trajectory()：
 *   mutable_xxx()获取可变指针
 *   obstacle_trajectory()返回障碍物轨迹子消息
 *
 * - obstacle_trajectory->add_perception_obstacle_history()：
 *   add_xxx()添加元素到repeated字段
 *   返回新元素的指针
 *
 * - util::WorldCoordToObjCoord(...)：
 *   自定义工具函数
 *   将世界坐标转换为相对坐标
 *
 * - std::make_pair(obj_traj_point.position().x(), ...)：
 *   构造pair作为坐标
 *
 * - perception_obstacle_history->mutable_position()->set_x(...)：
 *   链式调用设置位置
 *   mutable_position()获取可变指针
 *   set_x()设置x坐标
 */
void MessageProcess::GenerateObstacleTrajectory(
    const int frame_num, const int obstacle_id,
    const ADCCurrentInfo& adc_curr_info, ObstacleFeature* obstacle_feature) {
  // 获取可变障碍物轨迹指针
  auto obstacle_trajectory = obstacle_feature->mutable_obstacle_trajectory();

  // 获取障碍物历史
  const auto& obstacle_history = obstacle_history_map_[obstacle_id];

  // 遍历历史轨迹点
  for (const auto& obj_traj_point : obstacle_history) {
    // 添加一个历史感知障碍物点
    auto perception_obstacle_history =
        obstacle_trajectory->add_perception_obstacle_history();

    // 设置时间戳
    perception_obstacle_history->set_timestamp_sec(
        obj_traj_point.timestamp_sec());

    // 将位置转换为相对坐标
    const auto& relative_posistion = util::WorldCoordToObjCoord(
        std::make_pair(obj_traj_point.position().x(),
                       obj_traj_point.position().y()),
        adc_curr_info.adc_cur_position_, adc_curr_info.adc_cur_heading_);

    // 设置相对位置
    auto position = perception_obstacle_history->mutable_position();
    position->set_x(relative_posistion.first);
    position->set_y(relative_posistion.second);

    // 将朝向角转换为相对坐标
    const double relative_theta = util::WorldAngleToObjAngle(
        obj_traj_point.theta(), adc_curr_info.adc_cur_heading_);
    perception_obstacle_history->set_theta(relative_theta);

    // 将速度转换为相对坐标
    const auto& relative_velocity = util::WorldCoordToObjCoord(
        std::make_pair(obj_traj_point.velocity().x(),
                       obj_traj_point.velocity().y()),
        adc_curr_info.adc_cur_velocity_, adc_curr_info.adc_cur_heading_);
    auto velocity = perception_obstacle_history->mutable_velocity();
    velocity->set_x(relative_velocity.first);
    velocity->set_y(relative_velocity.second);

    // 将加速度转换为相对坐标
    const auto& relative_acc = util::WorldCoordToObjCoord(
        std::make_pair(obj_traj_point.acceleration().x(),
                       obj_traj_point.acceleration().y()),
        adc_curr_info.adc_cur_acc_, adc_curr_info.adc_cur_heading_);
    auto acceleration = perception_obstacle_history->mutable_acceleration();
    acceleration->set_x(relative_acc.first);
    acceleration->set_y(relative_acc.second);

    // 将多边形顶点转换为相对坐标
    for (int i = 0; i < obj_traj_point.polygon_point_size(); ++i) {
      const auto& relative_point = util::WorldCoordToObjCoord(
          std::make_pair(obj_traj_point.polygon_point(i).x(),
                         obj_traj_point.polygon_point(i).y()),
          adc_curr_info.adc_cur_position_, adc_curr_info.adc_cur_heading_);
      auto polygon_point = perception_obstacle_history->add_polygon_point();
      polygon_point->set_x(relative_point.first);
      polygon_point->set_y(relative_point.second);
    }
  }
}

/**
 * @brief 生成障碍物预测
 *
 * @param frame_num 帧号
 * @param prediction_obstacle 预测障碍物
 * @param adc_curr_info 自车当前信息
 * @param obstacle_feature 输出：障碍物特征
 *
 * 功能说明：
 * 处理障碍物的预测轨迹
 * 将轨迹点转换为自车相对坐标
 *
 * C++语法说明：
 * - obstacle_prediction->set_predicted_period(...)：
 *   设置预测时间段
 *
 * - obstacle_prediction->mutable_intent()->CopyFrom(...)：
 *   复制意图信息
 *   mutable_xxx()获取可变指针
 *
 * - trajectory->add_trajectory()：
 *   添加一条轨迹
 *
 * - trajectory->trajectory_point_size() > 0：
 *   检查轨迹点数量
 *   用于判断是否有上一个点
 *
 * - obstacle_trajectory_point.relative_time() < last_relative_time：
 *   检查相对时间是否递增
 *   异常则丢弃该点
 */
void MessageProcess::GenerateObstaclePrediction(
    const int frame_num, const PredictionObstacle& prediction_obstacle,
    const ADCCurrentInfo& adc_curr_info, ObstacleFeature* obstacle_feature) {
  // 获取障碍物ID
  const auto obstacle_id = obstacle_feature->id();

  // 获取可变预测子消息
  auto obstacle_prediction = obstacle_feature->mutable_obstacle_prediction();

  // 设置时间戳
  obstacle_prediction->set_timestamp_sec(prediction_obstacle.timestamp());

  // 设置预测时间段
  obstacle_prediction->set_predicted_period(
      prediction_obstacle.predicted_period());

  // 复制意图和优先级
  obstacle_prediction->mutable_intent()->CopyFrom(prediction_obstacle.intent());
  obstacle_prediction->mutable_priority()->CopyFrom(
      prediction_obstacle.priority());

  // 设置是否为静态障碍物
  obstacle_prediction->set_is_static(prediction_obstacle.is_static());

  // 遍历所有预测轨迹
  for (int i = 0; i < prediction_obstacle.trajectory_size(); ++i) {
    const auto& obstacle_trajectory = prediction_obstacle.trajectory(i);
    // 添加一条轨迹
    auto trajectory = obstacle_prediction->add_trajectory();

    // 设置概率
    trajectory->set_probability(obstacle_trajectory.probability());

    // 遍历轨迹点
    for (int j = 0; j < obstacle_trajectory.trajectory_point_size(); ++j) {
      const auto& obstacle_trajectory_point =
          obstacle_trajectory.trajectory_point(j);

      // 检查相对时间是否递增
      if (trajectory->trajectory_point_size() > 0) {
        const auto last_relative_time =
            trajectory
                ->trajectory_point(trajectory->trajectory_point_size() - 1)
                .trajectory_point()
                .relative_time();
        if (obstacle_trajectory_point.relative_time() < last_relative_time) {
          // 时间异常，记录并跳过
          const std::string msg = absl::StrCat(
              "DISCARD prediction trajectory point: frame_num[", frame_num,
              "] obstacle_id[", obstacle_id, "] last_relative_time[",
              last_relative_time, "] relative_time[",
              obstacle_trajectory_point.relative_time(), "]");
          AERROR << msg;
          if (FLAGS_planning_offline_learning) {
            log_file_ << msg << std::endl;
          }
          continue;
        }
      }

      // 添加轨迹点
      auto trajectory_point = trajectory->add_trajectory_point();

      // 获取路径点可变指针
      auto path_point =
          trajectory_point->mutable_trajectory_point()->mutable_path_point();

      // 将路径点位置转换为相对坐标
      const auto& relative_path_point = util::WorldCoordToObjCoord(
          std::make_pair(obstacle_trajectory_point.path_point().x(),
                         obstacle_trajectory_point.path_point().y()),
          adc_curr_info.adc_cur_position_, adc_curr_info.adc_cur_heading_);
      path_point->set_x(relative_path_point.first);
      path_point->set_y(relative_path_point.second);

      // 将路径点朝向转换为相对坐标
      const double relative_theta = util::WorldAngleToObjAngle(
          obstacle_trajectory_point.path_point().theta(),
          adc_curr_info.adc_cur_heading_);
      path_point->set_theta(relative_theta);

      // 设置s坐标和车道ID
      path_point->set_s(obstacle_trajectory_point.path_point().s());
      path_point->set_lane_id(obstacle_trajectory_point.path_point().lane_id());

      // 计算绝对时间戳
      const double timestamp_sec = prediction_obstacle.timestamp() +
                                   obstacle_trajectory_point.relative_time();
      trajectory_point->set_timestamp_sec(timestamp_sec);

      // 设置速度和加速度
      auto tp = trajectory_point->mutable_trajectory_point();
      tp->set_v(obstacle_trajectory_point.v());
      tp->set_a(obstacle_trajectory_point.a());
      tp->set_relative_time(obstacle_trajectory_point.relative_time());

      // 复制高斯信息
      tp->mutable_gaussian_info()->CopyFrom(
          obstacle_trajectory_point.gaussian_info());
    }
  }
}

/**
 * @brief 生成障碍物特征
 *
 * @param learning_data_frame 学习数据帧
 *
 * 功能说明：
 * 为所有障碍物生成特征
 * 包括历史轨迹和预测轨迹
 */
void MessageProcess::GenerateObstacleFeature(
    LearningDataFrame* learning_data_frame) {
  // 获取自车当前信息
  ADCCurrentInfo adc_curr_info;
  if (GetADCCurrentInfo(&adc_curr_info) == -1) {
    const std::string msg =
        absl::StrCat("fail to get ADC current info: frame_num[",
                     learning_data_frame->frame_num(), "]");
    AERROR << msg;
    if (FLAGS_planning_offline_learning) {
      log_file_ << msg << std::endl;
    }
    return;
  }

  // 获取帧号
  const int frame_num = learning_data_frame->frame_num();

  // 遍历所有预测障碍物
  for (const auto& m : prediction_obstacles_map_) {
    // 添加障碍物特征
    auto obstacle_feature = learning_data_frame->add_obstacle();

    // 获取感知障碍物
    const auto& perception_obstale = m.second.perception_obstacle();

    // 设置障碍物基本属性
    obstacle_feature->set_id(m.first);
    obstacle_feature->set_length(perception_obstale.length());
    obstacle_feature->set_width(perception_obstale.width());
    obstacle_feature->set_height(perception_obstale.height());
    obstacle_feature->set_type(perception_obstale.type());

    // 生成障碍物历史轨迹（相对坐标）
    GenerateObstacleTrajectory(frame_num, m.first, adc_curr_info,
                               obstacle_feature);

    // 生成障碍物预测（相对坐标）
    GenerateObstaclePrediction(frame_num, m.second, adc_curr_info,
                               obstacle_feature);
  }
}

/**
 * @brief 生成本地路由
 *
 * @param frame_num 帧号
 * @param local_routing 输出：本地路由特征
 * @param local_routing_lane_ids 输出：本地路由车道ID列表
 * @return bool 是否成功生成
 *
 * 功能说明：
 * 根据自车当前位置和完整路由
 * 生成一段本地路由（前向200米，后向100米）
 *
 * 算法流程：
 * 1. 计算每条道路的长度
 * 2. 找到自车所在道路和通道
 * 3. 确定本地路由的起止点
 * 4. 提取本地路由的车道段
 */
bool MessageProcess::GenerateLocalRouting(
    const int frame_num, RoutingResponseFeature* local_routing,
    std::vector<std::string>* local_routing_lane_ids) {
  // 清空输出
  local_routing->Clear();
  local_routing_lane_ids->clear();

  // 检查路由响应是否有效
  if (routing_response_.road_size() == 0 ||
      routing_response_.road(0).passage_size() == 0 ||
      routing_response_.road(0).passage(0).segment_size() == 0) {
    const std::string msg = absl::StrCat(
        "DISCARD: invalid routing_response. frame_num[", frame_num, "]");
    AERROR << msg;
    if (FLAGS_planning_offline_learning) {
      log_file_ << msg << std::endl;
    }
    return false;
  }

  // 计算每条道路的长度
  std::vector<std::pair<std::string, double>> road_lengths;
  for (int i = 0; i < routing_response_.road_size(); ++i) {
    ADEBUG << "road_id[" << routing_response_.road(i).id() << "] passage_size["
           << routing_response_.road(i).passage_size() << "]";
    double road_length = 0.0;
    for (int j = 0; j < routing_response_.road(i).passage_size(); ++j) {
      ADEBUG << "   passage: segment_size["
             << routing_response_.road(i).passage(j).segment_size() << "]";
      double passage_length = 0;
      for (int k = 0; k < routing_response_.road(i).passage(j).segment_size();
           ++k) {
        const auto& segment = routing_response_.road(i).passage(j).segment(k);
        passage_length += (segment.end_s() - segment.start_s());
      }
      ADEBUG << "      passage_length[" << passage_length << "]";
      road_length = std::max(road_length, passage_length);
    }

    road_lengths.push_back(
        std::make_pair(routing_response_.road(i).id(), road_length));
    ADEBUG << "   road_length[" << road_length << "]";
  }

  // 调试代码
  /*
  for (size_t i = 0; i < road_lengths.size(); ++i) {
    AERROR << i << ": " << road_lengths[i].first << "; "
           << road_lengths[i].second;
  }
  */

  // 获取自车在路由上的位置
  int adc_road_index = 0;
  int adc_passage_index = 0;
  double adc_passage_s = 0.0;
  if (!GetADCCurrentRoutingIndex(&adc_road_index, &adc_passage_index,
                                 &adc_passage_s) ||
      adc_road_index < 0 || adc_passage_index < 0 || adc_passage_s < 0) {
    // 无法定位自车，清空定位历史
    localizations_.clear();

    const std::string msg = absl::StrCat(
        "DISCARD: fail to locate ADC on routing. frame_num[", frame_num, "]");
    AERROR << msg;
    if (FLAGS_planning_offline_learning) {
      log_file_ << msg << std::endl;
    }
    return false;
  }
  ADEBUG << "adc_road_index[" << adc_road_index << "] adc_passage_index["
         << adc_passage_index << "] adc_passage_s[" << adc_passage_s << "]";

  // 本地路由长度参数
  constexpr double kLocalRoutingForwardLength = 200.0;   // 前向200米
  constexpr double kLocalRoutingBackwardLength = 100.0;  // 后向100米

  // 计算本地路由起点
  int local_routing_start_road_index = 0;
  double local_routing_start_road_s = 0;
  double backward_length = kLocalRoutingBackwardLength;
  for (int i = adc_road_index; i >= 0; --i) {
    const double road_length =
        (i == adc_road_index ? adc_passage_s : road_lengths[i].second);
    if (backward_length > road_length) {
      backward_length -= road_length;
    } else {
      local_routing_start_road_index = i;
      local_routing_start_road_s = road_length - backward_length;
      ADEBUG << "local_routing_start_road_index["
             << local_routing_start_road_index
             << "] local_routing_start_road_s[" << local_routing_start_road_s
             << "]";
      break;
    }
  }

  // 计算本地路由终点
  int local_routing_end_road_index = routing_response_.road_size() - 1;
  double local_routing_end_road_s =
      road_lengths[local_routing_end_road_index].second;
  double forwardward_length = kLocalRoutingForwardLength;
  for (int i = adc_road_index; i < routing_response_.road_size(); ++i) {
    const double road_length =
        (i == adc_road_index ? road_lengths[i].second - adc_passage_s
                             : road_lengths[i].second);
    if (forwardward_length > road_length) {
      forwardward_length -= road_length;
    } else {
      local_routing_end_road_index = i;
      local_routing_end_road_s =
          (i == adc_road_index ? adc_passage_s + forwardward_length
                               : forwardward_length);
      ADEBUG << "local_routing_end_road_index[" << local_routing_end_road_index
             << "] local_routing_end_road_s[" << local_routing_end_road_s
             << "]";
      break;
    }
  }

  ADEBUG << "local_routing: start_road_index[" << local_routing_start_road_index
         << "] start_road_s[" << local_routing_start_road_s
         << "] end_road_index[" << local_routing_end_road_index
         << "] end_road_s[" << local_routing_end_road_s << "]";

  // 构建本地路由
  bool local_routing_end = false;
  int last_passage_index = adc_passage_index;
  for (int i = local_routing_start_road_index;
       i <= local_routing_end_road_index; ++i) {
    if (local_routing_end) break;

    const auto& road = routing_response_.road(i);
    auto local_routing_road = local_routing->add_road();
    local_routing_road->set_id(road.id());

    for (int j = 0; j < road.passage_size(); ++j) {
      const auto& passage = road.passage(j);
      auto local_routing_passage = local_routing_road->add_passage();
      local_routing_passage->set_can_exit(passage.can_exit());
      local_routing_passage->set_change_lane_type(passage.change_lane_type());

      double road_s = 0;
      for (int k = 0; k < passage.segment_size(); ++k) {
        const auto& lane_segment = passage.segment(k);
        road_s += (lane_segment.end_s() - lane_segment.start_s());

        // 起始道路：跳过起点之前的段
        if (i == local_routing_start_road_index &&
            road_s < local_routing_start_road_s) {
          continue;
        }

        // 添加车道段
        local_routing_passage->add_segment()->CopyFrom(lane_segment);
        ADEBUG << "ADD road[" << i << "] id[" << road.id() << "] passage[" << j
               << "] id[" << lane_segment.id() << "] length["
               << lane_segment.end_s() - lane_segment.start_s() << "]";

        // 设置本地路由车道ID
        if (i == adc_road_index) {
          // 自车所在道路：选择自车所在通道
          if (j == adc_passage_index) {
            local_routing_lane_ids->push_back(lane_segment.id());
            ADEBUG << "ADD local_routing_lane_ids: road[" << i << "] passage["
                   << j << "]: " << lane_segment.id();
            last_passage_index = j;
          }
        } else {
          if (road.passage_size() == 1) {
            // 单通道
            ADEBUG << "ADD local_routing_lane_ids: road[" << i << "] passage["
                   << j << "]: " << lane_segment.id();
            local_routing_lane_ids->push_back(lane_segment.id());
            last_passage_index = j;
          } else {
            // 多通道
            if (i < adc_road_index) {
              // 自车后方的道路
              if (j == adc_passage_index ||
                  (j == road.passage_size() - 1 && j < adc_passage_index)) {
                ADEBUG << "ADD local_routing_lane_ids: road[" << i
                       << "] passage[" << j << "] adc_passage_index["
                       << adc_passage_index << "] passage_size["
                       << road.passage_size() << "]: " << lane_segment.id();
                local_routing_lane_ids->push_back(lane_segment.id());
              }
            } else {
              // 自车前方的道路：选择向左换道
              if (j == last_passage_index + 1 ||
                  (j == road.passage_size() - 1 &&
                   j < last_passage_index + 1)) {
                ADEBUG << "ADD local_routing_lane_ids: road[" << i
                       << "] passage[" << j << "] last_passage_index["
                       << last_passage_index << "] passage_size["
                       << road.passage_size() << "]: " << lane_segment.id();
                local_routing_lane_ids->push_back(lane_segment.id());
                last_passage_index = j;
              }
            }
          }
        }

        // 结束道路：到达终点后停止
        if (i == local_routing_end_road_index &&
            road_s >= local_routing_end_road_s) {
          local_routing_end = true;
          break;
        }
      }
    }
  }

  // 检查本地路由：过滤地图不匹配帧
  if (FLAGS_planning_offline_learning) {
    for (size_t i = 0; i < local_routing_lane_ids->size(); ++i) {
      const std::string lane_id = local_routing_lane_ids->at(i);
      const auto& lane =
          hdmap::HDMapUtil::BaseMap().GetLaneById(hdmap::MakeMapId(lane_id));
      if (lane == nullptr) {
        const std::string msg = absl::StrCat(
            "DISCARD: fail to find local_routing_lane on map. frame_num[",
            frame_num, "] lane[", lane_id, "]");
        AERROR << msg;
        if (FLAGS_planning_offline_learning) {
          log_file_ << msg << std::endl;
        }
        return false;
      }
    }
  }

  return true;
}

/**
 * @brief 生成路由特征
 *
 * @param local_routing 本地路由
 * @param local_routing_lane_ids 本地路由车道ID列表
 * @param learning_data_frame 学习数据帧
 */
void MessageProcess::GenerateRoutingFeature(
    const RoutingResponseFeature& local_routing,
    const std::vector<std::string>& local_routing_lane_ids,
    LearningDataFrame* learning_data_frame) {
  // 获取可变路由指针
  auto routing = learning_data_frame->mutable_routing();
  routing->Clear();

  // 复制路由响应测量距离
  routing->mutable_routing_response()->mutable_measurement()->set_distance(
      routing_response_.measurement().distance());

  // 复制所有道路
  for (int i = 0; i < routing_response_.road_size(); ++i) {
    routing->mutable_routing_response()->add_road()->CopyFrom(
        routing_response_.road(i));
  }

  // 复制本地路由车道ID
  for (const auto& lane_id : local_routing_lane_ids) {
    routing->add_local_routing_lane_id(lane_id);
  }

  // 复制本地路由
  routing->mutable_local_routing()->CopyFrom(local_routing);

  // 检查本地路由
  const int frame_num = learning_data_frame->frame_num();
  const int local_routing_lane_id_size = routing->local_routing_lane_id_size();
  if (local_routing_lane_id_size == 0) {
    const std::string msg =
        absl::StrCat("empty local_routing. frame_num[", frame_num, "]");
    AERROR << msg;
    if (FLAGS_planning_offline_learning) {
      log_file_ << msg << std::endl;
    }
  }
  if (local_routing_lane_id_size > 100) {
    const std::string msg = absl::StrCat(
        "LARGE local_routing. frame_num[", frame_num,
        "] local_routing_lane_id_size[", local_routing_lane_id_size, "]");
    AERROR << msg;
    if (FLAGS_planning_offline_learning) {
      log_file_ << msg << std::endl;
    }
  }
  ADEBUG << "local_routing: frame_num[" << frame_num << "] size["
         << routing->local_routing_lane_id_size() << "]";
}

/**
 * @brief 生成交通灯检测特征
 *
 * @param learning_data_frame 学习数据帧
 */
void MessageProcess::GenerateTrafficLightDetectionFeature(
    LearningDataFrame* learning_data_frame) {
  // 获取可变交通灯检测指针
  auto traffic_light_detection =
      learning_data_frame->mutable_traffic_light_detection();

  // 设置消息时间戳
  traffic_light_detection->set_message_timestamp_sec(
      traffic_light_detection_message_timestamp_);

  // 清空交通灯列表
  traffic_light_detection->clear_traffic_light();

  // 复制所有交通灯
  for (const auto& tl : traffic_lights_) {
    auto traffic_light = traffic_light_detection->add_traffic_light();
    traffic_light->CopyFrom(tl);
  }
}

/**
 * @brief 生成自车轨迹点
 *
 * @param localizations 定位历史
 * @param learning_data_frame 学习数据帧
 *
 * 功能说明：
 * 根据定位历史生成自车轨迹点
 * 并计算planning_tag（车道转弯、交叉路口等）
 */
void MessageProcess::GenerateADCTrajectoryPoints(
    const std::list<LocalizationEstimate>& localizations,
    LearningDataFrame* learning_data_frame) {
  // 将list转换为vector
  std::vector<LocalizationEstimate> localization_samples;
  for (const auto& le : localizations) {
    // insert在begin位置插入，实现逆序
    localization_samples.insert(localization_samples.begin(), le);
  }

  // 搜索半径
  constexpr double kSearchRadius = 1.0;

  // 初始化各元素的ID和距离
  std::string clear_area_id;
  double clear_area_distance = 0.0;
  std::string crosswalk_id;
  double crosswalk_distance = 0.0;
  std::string pnc_junction_id;
  double pnc_junction_distance = 0.0;
  std::string signal_id;
  double signal_distance = 0.0;
  std::string stop_sign_id;
  double stop_sign_distance = 0.0;
  std::string yield_sign_id;
  double yield_sign_distance = 0.0;

  // 轨迹点索引
  int trajectory_point_index = 0;
  std::vector<ADCTrajectoryPoint> adc_trajectory_points;

  // 遍历定位样本
  for (const auto& localization_sample : localization_samples) {
    // 创建ADC轨迹点
    ADCTrajectoryPoint adc_trajectory_point;
    adc_trajectory_point.set_timestamp_sec(
        localization_sample.measurement_time());

    // 获取可变轨迹点指针
    auto trajectory_point = adc_trajectory_point.mutable_trajectory_point();
    auto& pose = localization_sample.pose();

    // 设置路径点坐标
    trajectory_point->mutable_path_point()->set_x(pose.position().x());
    trajectory_point->mutable_path_point()->set_y(pose.position().y());
    trajectory_point->mutable_path_point()->set_z(pose.position().z());
    trajectory_point->mutable_path_point()->set_theta(pose.heading());

    // 计算速度大小
    const double v =
        std::sqrt(pose.linear_velocity().x() * pose.linear_velocity().x() +
                  pose.linear_velocity().y() * pose.linear_velocity().y());
    trajectory_point->set_v(v);

    // 计算加速度大小
    const double a = std::sqrt(
        pose.linear_acceleration().x() * pose.linear_acceleration().x() +
        pose.linear_acceleration().y() * pose.linear_acceleration().y());
    trajectory_point->set_a(a);

    // 获取planning_tag
    auto planning_tag = adc_trajectory_point.mutable_planning_tag();

    // 获取当前位置所在车道
    const auto& cur_point = common::util::PointFactory::ToPointENU(
        pose.position().x(), pose.position().y(), pose.position().z());
    LaneInfoConstPtr lane = GetCurrentLane(cur_point);

    // 车道转弯类型
    apollo::hdmap::Lane::LaneTurn lane_turn = apollo::hdmap::Lane::NO_TURN;
    if (lane != nullptr) {
      lane_turn = lane->lane().turn();
    }
    planning_tag->set_lane_turn(lane_turn);
    planning_tag_.set_lane_turn(lane_turn);

    // 计算planning_tag（仅离线学习模式）
    if (FLAGS_planning_offline_learning) {
      // 计算点间距离
      double point_distance = 0.0;
      if (trajectory_point_index > 0) {
        auto& next_point = adc_trajectory_points[trajectory_point_index - 1]
                               .trajectory_point()
                               .path_point();
        point_distance = common::util::DistanceXY(next_point, cur_point);
      }

      // 创建HDMap点
      common::PointENU hdmap_point;
      hdmap_point.set_x(cur_point.x());
      hdmap_point.set_y(cur_point.y());

      // 清除区域
      planning_tag->clear_clear_area();
      std::vector<ClearAreaInfoConstPtr> clear_areas;
      if (HDMapUtil::BaseMap().GetClearAreas(hdmap_point, kSearchRadius,
                                             &clear_areas) == 0 &&
          clear_areas.size() > 0) {
        clear_area_id = clear_areas.front()->id().id();
        clear_area_distance = 0.0;
      } else {
        if (!clear_area_id.empty()) {
          clear_area_distance += point_distance;
        }
      }
      if (!clear_area_id.empty()) {
        planning_tag->mutable_clear_area()->set_id(clear_area_id);
        planning_tag->mutable_clear_area()->set_distance(clear_area_distance);
      }

      // 人行横道
      planning_tag->clear_crosswalk();
      std::vector<CrosswalkInfoConstPtr> crosswalks;
      if (HDMapUtil::BaseMap().GetCrosswalks(hdmap_point, kSearchRadius,
                                             &crosswalks) == 0 &&
          crosswalks.size() > 0) {
        crosswalk_id = crosswalks.front()->id().id();
        crosswalk_distance = 0.0;
      } else {
        if (!crosswalk_id.empty()) {
          crosswalk_distance += point_distance;
        }
      }
      if (!crosswalk_id.empty()) {
        planning_tag->mutable_crosswalk()->set_id(crosswalk_id);
        planning_tag->mutable_crosswalk()->set_distance(crosswalk_distance);
      }

      // PNC交叉路口
      std::vector<PNCJunctionInfoConstPtr> pnc_junctions;
      if (HDMapUtil::BaseMap().GetPNCJunctions(hdmap_point, kSearchRadius,
                                               &pnc_junctions) == 0 &&
          pnc_junctions.size() > 0) {
        pnc_junction_id = pnc_junctions.front()->id().id();
        pnc_junction_distance = 0.0;
      } else {
        if (!pnc_junction_id.empty()) {
          pnc_junction_distance += point_distance;
        }
      }
      if (!pnc_junction_id.empty()) {
        planning_tag->mutable_pnc_junction()->set_id(pnc_junction_id);
        planning_tag->mutable_pnc_junction()->set_distance(
            pnc_junction_distance);
      }

      // 信号灯
      std::vector<SignalInfoConstPtr> signals;
      if (HDMapUtil::BaseMap().GetSignals(hdmap_point, kSearchRadius,
                                          &signals) == 0 &&
          signals.size() > 0) {
        signal_id = signals.front()->id().id();
        signal_distance = 0.0;
      } else {
        if (!signal_id.empty()) {
          signal_distance += point_distance;
        }
      }
      if (!signal_id.empty()) {
        planning_tag->mutable_signal()->set_id(signal_id);
        planning_tag->mutable_signal()->set_distance(signal_distance);
      }

      // 停车标志
      std::vector<StopSignInfoConstPtr> stop_signs;
      if (HDMapUtil::BaseMap().GetStopSigns(hdmap_point, kSearchRadius,
                                            &stop_signs) == 0 &&
          stop_signs.size() > 0) {
        stop_sign_id = stop_signs.front()->id().id();
        stop_sign_distance = 0.0;
      } else {
        if (!stop_sign_id.empty()) {
          stop_sign_distance += point_distance;
        }
      }
      if (!stop_sign_id.empty()) {
        planning_tag->mutable_stop_sign()->set_id(stop_sign_id);
        planning_tag->mutable_stop_sign()->set_distance(stop_sign_distance);
      }

      // 让行标志
      std::vector<YieldSignInfoConstPtr> yield_signs;
      if (HDMapUtil::BaseMap().GetYieldSigns(hdmap_point, kSearchRadius,
                                             &yield_signs) == 0 &&
          yield_signs.size() > 0) {
        yield_sign_id = yield_signs.front()->id().id();
        yield_sign_distance = 0.0;
      } else {
        if (!yield_sign_id.empty()) {
          yield_sign_distance += point_distance;
        }
      }
      if (!yield_sign_id.empty()) {
        planning_tag->mutable_yield_sign()->set_id(yield_sign_id);
        planning_tag->mutable_yield_sign()->set_distance(yield_sign_distance);
      }
    }

    // 添加轨迹点
    adc_trajectory_points.push_back(adc_trajectory_point);
    ++trajectory_point_index;
  }

  // 逆序（按时间正序）
  std::reverse(adc_trajectory_points.begin(), adc_trajectory_points.end());

  // 添加到学习数据帧
  for (const auto& trajectory_point : adc_trajectory_points) {
    auto adc_trajectory_point = learning_data_frame->add_adc_trajectory_point();
    adc_trajectory_point->CopyFrom(trajectory_point);
  }

  // 检查轨迹点数量
  if (adc_trajectory_points.size() <= 5) {
    const std::string msg =
        absl::StrCat("too few adc_trajectory_points: frame_num[",
                     learning_data_frame->frame_num(), "] size[",
                     adc_trajectory_points.size(), "]");
    AERROR << msg;
    if (FLAGS_planning_offline_learning) {
      log_file_ << msg << std::endl;
    }
  }
}

/**
 * @brief 生成规划标签
 *
 * @param learning_data_frame 学习数据帧
 */
void MessageProcess::GeneratePlanningTag(
    LearningDataFrame* learning_data_frame) {
  // 获取可变planning_tag指针
  auto planning_tag = learning_data_frame->mutable_planning_tag();

  if (FLAGS_planning_offline_learning) {
    // 离线学习模式：使用之前计算的planning_tag
    planning_tag->set_lane_turn(planning_tag_.lane_turn());
  } else {
    // 在线模式
    if (planning_config_.learning_mode() != PlanningConfig::NO_LEARNING) {
      // 在线学习：复制planning_tag
      planning_tag->CopyFrom(planning_tag_);
    }
  }
}

/**
 * @brief 生成学习数据帧
 *
 * @param learning_data_frame 输出：学习数据帧
 * @return bool 是否成功生成
 *
 * 功能说明：
 * 综合所有消息生成一个完整的学习数据帧
 * 包括：路由、底盘、定位、交通灯、障碍物、轨迹点等
 */
bool MessageProcess::GenerateLearningDataFrame(
    LearningDataFrame* learning_data_frame) {
  // 记录开始时间
  const double start_timestamp = Clock::NowInSeconds();

  // 生成本地路由
  RoutingResponseFeature local_routing;
  std::vector<std::string> local_routing_lane_ids;
  if (!GenerateLocalRouting(total_learning_data_frame_num_, &local_routing,
                            &local_routing_lane_ids)) {
    return false;
  }

  // 添加时间戳和帧号
  learning_data_frame->set_message_timestamp_sec(
      localizations_.back().header().timestamp_sec());
  learning_data_frame->set_frame_num(total_learning_data_frame_num_++);

  // 设置地图名称
  learning_data_frame->set_map_name(map_name_);

  // 生成planning_tag
  GeneratePlanningTag(learning_data_frame);

  // 添加底盘信息
  auto chassis = learning_data_frame->mutable_chassis();
  chassis->CopyFrom(chassis_feature_);

  // 添加定位信息
  auto localization = learning_data_frame->mutable_localization();
  localization->set_message_timestamp_sec(
      localizations_.back().header().timestamp_sec());
  const auto& pose = localizations_.back().pose();
  localization->mutable_position()->CopyFrom(pose.position());
  localization->set_heading(pose.heading());
  localization->mutable_linear_velocity()->CopyFrom(pose.linear_velocity());
  localization->mutable_linear_acceleration()->CopyFrom(
      pose.linear_acceleration());
  localization->mutable_angular_velocity()->CopyFrom(pose.angular_velocity());

  // 添加交通灯
  GenerateTrafficLightDetectionFeature(learning_data_frame);

  // 添加路由
  GenerateRoutingFeature(local_routing, local_routing_lane_ids,
                         learning_data_frame);

  // 添加障碍物
  GenerateObstacleFeature(learning_data_frame);

  // 添加轨迹点
  GenerateADCTrajectoryPoints(localizations_, learning_data_frame);

  // 计算处理时间
  const double end_timestamp = Clock::NowInSeconds();
  const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;
  ADEBUG << "MessageProcess: start_timestamp[" << start_timestamp
         << "] end_timestamp[" << end_timestamp << "] time_diff_ms["
         << time_diff_ms << "]";
  return true;
}

}  // namespace planning
}  // namespace apollo
