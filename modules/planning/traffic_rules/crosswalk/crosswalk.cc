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
 * @file
 * @brief 人行横道交通规则实现文件
 *
 * 功能说明：
 * 实现了人行横道(Crosswalk)交通规则，用于处理人行横道上的行人/自行车检测和停车决策
 * 当车辆接近人行横道时，需要检测并等待行人/自行车通过
 *
 * 核心概念：
 * - Crosswalk：人行横道，道路上供行人过街的标记区域
 * - PEDESTRIAN/BICYCLE：行人和自行车类型
 * - Stop Timer：停车计时器，用于处理静态行人/自行车的等待超时
 * - SL Boundary：沿路径(s)和垂直路径(l)的边界
 * - ST Boundary：时空边界，描述障碍物在时间和空间上的占用
 *
 * 停车决策逻辑：
 * 1. 横向距离 >= loose_l_distance：仅当路径交叉时停车
 * 2. 横向距离 <= strict_l_distance：
 *    - 在道路上：总是停车
 *    - 不在道路上：路径交叉时停车或行人向自车移动时停车
 * 3. 横向距离介于两者之间：使用历史决策平滑
 *
 * C++语法说明：
 * - std::unordered_map：哈希表容器，O(1)查找复杂度
 * - std::pair：模板类，表示一对值
 * - const_cast：常量类型转换
 * - protobuf消息：has_xxx()检查字段是否存在，mutable_xxx()获取可变引用
 **/
#include "modules/planning/traffic_rules/crosswalk/crosswalk.h"

#include <algorithm>
/**
 * @brief 标准算法库头文件
 *
 * 提供常用算法：
 * - std::find：查找算法
 * - std::min/max：极值算法
 * - std::sort：排序算法
 */

#include <limits>
/**
 * @brief 数值限制头文件
 *
 * 提供数值类型限制：
 * - std::numeric_limits<T>::max()：类型T的最大值
 * - std::numeric_limits<T>::min()：类型T的最小值
 */

#include <memory>
/**
 * @brief 智能指针头文件
 *
 * 提供智能指针模板类：
 * - std::shared_ptr：引用计数智能指针
 * - std::unique_ptr：独占所有权智能指针
 */

#include <unordered_map>
/**
 * @brief 哈希表容器头文件
 *
 * std::unordered_map：
 * - 基于哈希表实现
 * - 查找/插入/删除为O(1)平均复杂度
 * - 无序存储，不保证迭代顺序
 */

#include <utility>
/**
 * @brief 工具类头文件
 *
 * 提供：
 * - std::pair：模板类，表示一对值
 * - std::move：移动语义
 * - std::forward：完美转发
 */

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
/**
 * @brief PNC点消息头文件
 *
 * 包含：
 * - PathPoint：路径点，包含x,y,z,theta等信息
 * - SLPoint：SL坐标系点
 * - XYPoint：XY坐标系点
 */

#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
/**
 * @brief 感知障碍物消息头文件
 *
 * PerceptionObstacle：
 * - 感知系统检测到的障碍物信息
 * - 包含位置、速度、大小、类型等
 */

#include "modules/planning/planning_base/proto/planning_status.pb.h"
/**
 * @brief 规划状态消息头文件
 *
 * PlanningStatus：
 * - 规划模块的运行时状态
 * - 包含crosswalk等交通规则状态
 */

#include "cyber/time/clock.h"
/**
 * @brief Cyber RT时钟服务头文件
 *
 * apollo::cyber::Clock：
 * - Cyber RT的时间服务
 * - Clock::NowInSeconds()：获取当前时间（秒）
 */

#include "modules/common/util/util.h"
/**
 * @brief 通用工具函数头文件
 */

#include "modules/common/vehicle_state/vehicle_state_provider.h"
/**
 * @brief 车辆状态提供者头文件
 *
 * VehicleStateProvider：
 * - 提供车辆当前状态（位置、速度、角度等）
 */

#include "modules/map/hdmap/hdmap_util.h"
/**
 * @brief HD地图工具头文件
 *
 * HDMapUtil：
 * - 提供地图数据访问接口
 * - BaseMap()：获取基础地图实例
 */

#include "modules/planning/planning_base/common/ego_info.h"
/**
 * @brief 自车信息头文件
 *
 * EgoInfo：
 * - 存储自车的相关信息
 * - 包含起始点、SL边界等
 */

#include "modules/planning/planning_base/common/frame.h"
/**
 * @brief 规划帧头文件
 *
 * Frame：
 * - 表示一个规划周期内的所有数据
 * - 包含车辆状态、参考线、障碍物等
 */

#include "modules/planning/planning_base/common/planning_context.h"
/**
 * @brief 规划上下文头文件
 *
 * PlanningContext：
 * - 规划模块的全局上下文
 * - 跨帧存储规划状态信息
 */

#include "modules/planning/planning_base/common/util/common.h"
/**
 * @brief 规划通用工具头文件
 */

#include "modules/planning/planning_base/common/util/util.h"
/**
 * @brief 规划工具函数头文件
 */

namespace apollo {
/**
 * @brief Apollo项目主命名空间
 *
 * 命名空间说明：
 * apollo是百度自动驾驶项目的顶级命名空间
 * 包含common、hdmap、planning、control、cyber等子命名空间
 */
namespace planning {
/**
 * @brief 规划模块的命名空间
 *
 * 包含内容：
 * - 交通规则实现（traffic_rules）
 * - 场景管理（scenarios）
 * - 路径规划任务（tasks）
 * - 规划器实现（planners）
 */

using apollo::common::Status;
/**
 * @brief 使用apollo::common::Status类型
 *
 * 语法说明：
 * using声明将其他命名空间中的类型引入当前作用域
 * Status是Apollo中用于表示操作结果的状态类
 */

using apollo::common::math::Polygon2d;
/**
 * @brief 使用apollo::common::math::Polygon2d类型
 *
 * Polygon2d：二维多边形类
 * 主要功能：
 * - 表示二维平面上的多边形
 * - IsPointIn：判断点是否在多边形内
 * - ExpandByDistance：按距离扩展多边形
 */

using apollo::common::math::Vec2d;
/**
 * @brief 使用apollo::common::math::Vec2d类型
 *
 * Vec2d：二维向量类
 * 主要功能：
 * - 表示二维向量/点
 * - InnerProd：计算内积
 * - 计算向量加减法、缩放等
 */

using apollo::cyber::Clock;
/**
 * @brief 使用apollo::cyber::Clock类型
 *
 * Clock类说明：
 * Cyber RT的时间服务类
 * - Clock::NowInSeconds()：获取当前时间（秒）
 */

using apollo::hdmap::CrosswalkInfoConstPtr;
/**
 * @brief 使用apollo::hdmap::CrosswalkInfoConstPtr类型
 *
 * CrosswalkInfoConstPtr：人行横道信息常量指针
 * - 指向人行横道详细信息的智能指针
 * - Const表示内容不可修改
 */

using apollo::hdmap::HDMapUtil;
/**
 * @brief 使用apollo::hdmap::HDMapUtil类型
 *
 * HDMapUtil：高清地图工具类
 * - BaseMap()：获取全局地图实例
 */

using apollo::hdmap::PathOverlap;
/**
 * @brief 使用apollo::hdmap::PathOverlap类型
 *
 * PathOverlap：路径重叠区结构体
 * - object_id：重叠对象ID
 * - start_s/end_s：重叠区起止s坐标
 */

using apollo::perception::PerceptionObstacle;
/**
 * @brief 使用apollo::perception::PerceptionObstacle类型
 *
 * PerceptionObstacle：感知障碍物结构
 * - type：障碍物类型（PEDESTRIAN/BICYCLE等）
 * - position：位置
 * - velocity：速度
 */

using CrosswalkToStop =
    std::vector<std::pair<const hdmap::PathOverlap*, std::vector<std::string>>>;
/**
 * @brief 定义需要停车的人行横道类型
 *
 * 类型说明：
 * std::vector<std::pair<...>>：向量容器，每个元素是一个pair
 *
 * pair成分：
 * - const hdmap::PathOverlap*：人行横道重叠区指针
 * - std::vector<std::string>：需要等待的障碍物ID列表
 *
 * 用途：
 * 存储哪些人行横道需要停车，以及需要等待哪些行人/自行车
 */

using CrosswalkStopTimer =
    std::unordered_map<std::string, std::unordered_map<std::string, double>>;
/**
 * @brief 定义人行横道停车计时器类型
 *
 * 类型说明：
 * std::unordered_map<K, V>：哈希表容器
 *
 * 外层哈希表：
 * - key：std::string（人行横道ID）
 * - value：内层哈希表
 *
 * 内层哈希表：
 * - key：std::string（障碍物ID）
 * - value：double（停车时间戳，秒）
 *
 * 用途：
 * 记录每个行人/自行车在每个 人行横道的停车开始时间
 * 用于判断静态行人等待超时
 */

/**
 * @brief 初始化人行横道规则
 *
 * @param name 规则名称
 * @param injector 依赖注入器指针
 * @return bool 初始化是否成功
 *
 * 功能说明：
 * 初始化Crosswalk交通规则实例
 * 加载规则配置并准备处理人行横道
 *
 * 算法流程：
 * 1. 调用基类TrafficRule的Init方法进行基础初始化
 * 2. 如果基础初始化失败，返回false
 * 3. 从配置文件加载CrosswalkConfig配置
 * 4. 返回配置加载结果
 *
 * C++语法说明：
 * - const std::string& name：常量引用参数，避免拷贝
 * - std::shared_ptr<DependencyInjector>：智能指针
 * - &config_：成员变量地址，用于存储配置
 */
bool Crosswalk::Init(const std::string& name,
                     const std::shared_ptr<DependencyInjector>& injector) {
  if (!TrafficRule::Init(name, injector)) {
    return false;  /**< 基类初始化失败，返回false */
  }
  return TrafficRule::LoadConfig<CrosswalkConfig>(&config_);
  /**
   * @brief 加载人行横道配置
   *
   * TrafficRule::LoadConfig<T>模板函数：
   * 从protobuf配置文件加载CrosswalkConfig
   */
}

/**
 * @brief 应用人行横道规则
 *
 * @param frame 当前规划帧
 * @param reference_line_info 参考线信息
 * @return Status 应用结果状态
 *
 * 功能说明：
 * 人行横道规则的主入口函数
 * 查找人行横道并生成停车决策
 *
 * 算法流程：
 * 1. 空指针检查
 * 2. 查找参考线上的人行横道
 * 3. 如果没有找到人行横道，清除状态并返回
 * 4. 生成停车决策
 * 5. 返回成功状态
 */
Status Crosswalk::ApplyRule(Frame* const frame,
                            ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  /**
   * @brief 空指针检查
   *
   * CHECK_NOTNULL是Apollo定义的宏：
   * - 如果frame为nullptr，程序会终止
   * - 用于预防编程错误
   */
  CHECK_NOTNULL(reference_line_info);

  /**
   * @brief 查找人行横道
   *
   * FindCrosswalks：查找参考线上的人行横道
   * 如果没有找到，清除人行横道状态并返回成功
   */
  if (!FindCrosswalks(reference_line_info)) {
    injector_->planning_context()->mutable_planning_status()->clear_crosswalk();
    /**
     * @brief 清除人行横道状态
     *
     * mutable_planning_status()：获取可修改的规划状态
     * clear_crosswalk()：清除人行横道相关状态
     */
    return Status::OK();  /**< 没有人行横道，无需处理 */
  }

  /**
   * @brief 生成人行横道决策
   */
  MakeDecisions(frame, reference_line_info);
  return Status::OK();
}

/**
 * @brief 生成人行横道决策
 *
 * @param frame 当前规划帧
 * @param reference_line_info 参考线信息
 *
 * 功能说明：
 * 遍历所有人行横道，检测行人/自行车并决定是否停车
 *
 * 算法流程：
 * 1. 获取路径决策和自车位置
 * 2. 读取保存的停车计时器状态
 * 3. 遍历每个人行横道：
 *    - 跳过已通过的
 *    - 跳过已完成的
 *    - 检查每个障碍物是否需要停车
 *    - 处理静态行人的计时器
 * 4. 为需要停车的人行横道创建停车决策
 * 5. 更新人行横道状态
 *
 * C++语法说明：
 * - auto*：自动类型推导的原始指针
 * - std::unordered_map：哈希表容器
 * - mutable_xxx：protobuf可变成员访问器
 */
void Crosswalk::MakeDecisions(Frame* const frame,
                              ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  /**< @brief 空指针检查 */
  CHECK_NOTNULL(reference_line_info);

  /**
   * @brief 获取可修改的人行横道状态
   *
   * injector_->planning_context()：获取规划上下文
   * mutable_planning_status()：获取可修改的规划状态
   * mutable_crosswalk()：获取可修改的人行横道状态
   */
  auto* mutable_crosswalk_status = injector_->planning_context()
                                       ->mutable_planning_status()
                                       ->mutable_crosswalk();

  auto* path_decision = reference_line_info->path_decision();
  /**< @brief 获取路径决策指针 */
  double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
  /**< @brief 自车前边缘s坐标 */

  /**
   * @brief 定义需要停车的人行横道列表
   *
   * CrosswalkToStop类型：
   * std::vector<std::pair<const PathOverlap*, std::vector<std::string>>>
   * 每项包含：人行横道指针 + 需要等待的障碍物ID列表
   */
  CrosswalkToStop crosswalks_to_stop;

  /**
   * @brief 读取保存的停车计时器状态
   *
   * 从上帧保存的状态中恢复计时器信息
   * 用于处理跨帧的静态行人等待
   */
  CrosswalkStopTimer crosswalk_stop_timer;
  /**< @brief 人行横道停车计时器：crosswalk_id -> (obstacle_id -> timestamp) */
  std::unordered_map<std::string, double> stop_times;
  /**< @brief 临时存储单个障碍物的停车时间 */

  /**
   * @brief 遍历已保存的停车时间
   *
   * mutable_crosswalk_status->stop_time()：
   * 返回repeated字段，包含所有保存的停车时间
   */
  for (const auto& stop_time : mutable_crosswalk_status->stop_time()) {
    /**
     * @brief 构建停车时间映射
     *
     * stop_time.obstacle_id()：障碍物ID
     * stop_time.stop_timestamp_sec()：停车时间戳
     */
    stop_times.emplace(stop_time.obstacle_id(), stop_time.stop_timestamp_sec());
  }

  /**
   * @brief 添加到计时器哈希表
   *
   * crosswalk_id -> obstacle_id -> timestamp
   */
  crosswalk_stop_timer.emplace(mutable_crosswalk_status->crosswalk_id(),
                               stop_times);

  /**
   * @brief 获取已完成的人行横道列表
   *
   * finished_crosswalks：已处理完成的人行横道ID列表
   */
  const auto& finished_crosswalks =
      mutable_crosswalk_status->finished_crosswalk();

  const auto& reference_line = reference_line_info->reference_line();
  /**< @brief 获取参考线 */

  /**
   * @brief 遍历所有人行横道重叠区
   */
  for (auto crosswalk_overlap : crosswalk_overlaps_) {
    /**
     * @brief 获取人行横道详细信息
     *
     * HDMapUtil::BaseMap()：获取地图实例
     * GetCrosswalkById：根据ID获取人行横道信息
     * MakeMapId：构造地图ID
     */
    auto crosswalk_ptr = HDMapUtil::BaseMap().GetCrosswalkById(
        hdmap::MakeMapId(crosswalk_overlap->object_id));
    std::string crosswalk_id = crosswalk_ptr->id().id();
    /**< @brief 获取人行横道ID字符串 */

    /**
     * @brief 条件1：检查自车是否已通过人行横道
     *
     * 如果自车前边缘已经越过人行横道结束点一段距离
     * 说明已经通过，可以清除状态并跳过
     *
     * min_pass_s_distance：最小通过距离（缓冲距离）
     */
    if (adc_front_edge_s - crosswalk_overlap->end_s >
        config_.min_pass_s_distance()) {
      /**
       * @brief 清除当前人行横道状态
       */
      if (mutable_crosswalk_status->has_crosswalk_id() &&
          mutable_crosswalk_status->crosswalk_id() == crosswalk_id) {
        mutable_crosswalk_status->clear_crosswalk_id();
        mutable_crosswalk_status->clear_stop_time();
      }

      ADEBUG << "SKIP: crosswalk_id[" << crosswalk_id
             << "] crosswalk_overlap_end_s[" << crosswalk_overlap->end_s
             << "] adc_front_edge_s[" << adc_front_edge_s
             << "]. adc_front_edge passes crosswalk_end_s + buffer.";
      continue;  /**< 已通过此 人行横道，继续下一个 */
    }

    /**
     * @brief 条件2：检查人行横道是否已完成
     *
     * 使用std::find在已完成列表中查找
     * 如果找到，说明已经处理过，跳过
     */
    if (finished_crosswalks.end() != std::find(finished_crosswalks.begin(),
                                               finished_crosswalks.end(),
                                               crosswalk_id)) {
      ADEBUG << "SKIP: crosswalk_id[" << crosswalk_id << "] crosswalk_end_s["
             << crosswalk_overlap->end_s << "] finished already";
      continue;  /**< 已完成，跳过 */
    }

    /**
     * @brief 收集需要等待的行人/自行车
     *
     * pedestrians：需要等待的障碍物ID列表
     */
    std::vector<std::string> pedestrians;
    /**< @brief 需要等待的行人/自行车ID列表 */

    /**
     * @brief 遍历路径上的所有障碍物
     */
    for (const auto* obstacle : path_decision->obstacles().Items()) {
      /**
       * @brief 计算停车减速度
       *
       * GetADCStopDeceleration：
       * 计算从当前位置停车到目标位置所需的减速度
       *
       * 参数：
       * - injector_->vehicle_state()：车辆状态
       * - adc_front_edge_s：自车前边缘s坐标
       * - crosswalk_overlap->start_s：人行横道停车线s坐标
       */
      const double stop_deceleration = util::GetADCStopDeceleration(
          injector_->vehicle_state(), adc_front_edge_s,
          crosswalk_overlap->start_s);

      /**
       * @brief 检查是否为该障碍物停车
       *
       * CheckStopForObstacle：
       * 根据障碍物类型、位置、速度等判断是否需要停车
       */
      bool stop = CheckStopForObstacle(reference_line_info, crosswalk_ptr,
                                       *obstacle, stop_deceleration);

      const std::string& obstacle_id = obstacle->Id();
      /**< @brief 障碍物ID */
      const PerceptionObstacle& perception_obstacle = obstacle->Perception();
      /**< @brief 获取感知障碍物信息 */
      PerceptionObstacle::Type obstacle_type = perception_obstacle.type();
      /**< @brief 障碍物类型 */
      std::string obstacle_type_name =
          PerceptionObstacle_Type_Name(obstacle_type);
      /**< @brief 障碍物类型名称字符串 */

      /**
       * @brief 处理静态行人/自行车的计时器
       *
       * 功能：对于静态行人，等待一段时间后自动放行
       * 条件：
       * 1. stop == true（需要停车）
       * 2. !is_on_lane（不在车道上）
       * 3. 在启动计时器距离内
       *
       * 问题：静态行人可能长时间不动，需要超时机制
       */
      const bool is_on_lane =
          reference_line.IsOnLane(obstacle->PerceptionSLBoundary());
      if (stop && !is_on_lane &&
          crosswalk_overlap->start_s - adc_front_edge_s <=
              config_.start_watch_timer_distance()) {
        /**
         * @brief 检查是否为静态障碍物
         *
         * kMaxStopSpeed = 0.3 m/s：最大静止速度阈值
         * std::hypot：计算速度向量的模（欧几里得范数）
         */
        const double kMaxStopSpeed = 0.3;
        auto obstacle_speed = std::hypot(perception_obstacle.velocity().x(),
                                         perception_obstacle.velocity().y());
        if (obstacle_speed <= kMaxStopSpeed) {
          /**
           * @brief 静态障碍物：启动/检查计时器
           */

          /**
           * @brief 如果没有时间戳，记录当前时间
           */
          if (crosswalk_stop_timer[crosswalk_id].count(obstacle_id) < 1) {
            ADEBUG << "add timestamp: obstacle_id[" << obstacle_id
                   << "] timestamp[" << Clock::NowInSeconds() << "]";
            crosswalk_stop_timer[crosswalk_id].insert(
                {obstacle_id, Clock::NowInSeconds()});
            /**
             * @brief 插入新计时器记录
             */
          } else {
            /**
             * @brief 如果有时钟，检查是否超时
             */
            double stop_time = Clock::NowInSeconds() -
                               crosswalk_stop_timer[crosswalk_id][obstacle_id];
            /**< @brief 已停车的时间（秒） */
            ADEBUG << "stop_time: obstacle_id[" << obstacle_id << "] stop_time["
                   << stop_time << "]";

            /**
             * @brief 如果等待时间超过阈值，释放该障碍物
             *
             * stop_timeout：配置的等待超时时间
             */
            if (stop_time >= config_.stop_timeout()) {
              stop = false;  /**< 超时，不等待此障碍物 */
            }
          }
        }
      }

      /**
       * @brief 根据stop标志添加到等待列表或跳过
       */
      if (stop) {
        pedestrians.push_back(obstacle_id);
        /**< @brief 添加到需要等待的列表 */
        ADEBUG << "wait for: obstacle_id[" << obstacle_id << "] type["
               << obstacle_type_name << "] crosswalk_id[" << crosswalk_id
               << "]";
      } else {
        ADEBUG << "skip: obstacle_id[" << obstacle_id << "] type["
               << obstacle_type_name << "] crosswalk_id[" << crosswalk_id
               << "]";
      }
    }

    /**
     * @brief 如果有需要等待的行人/自行车
     */
    if (!pedestrians.empty()) {
      crosswalks_to_stop.emplace_back(crosswalk_overlap, pedestrians);
      /**< @brief 添加到需要停车的人行横道列表 */
      ADEBUG << "crosswalk_id[" << crosswalk_id << "] STOP";
    }
  }

  /**
   * @brief 为需要停车的人行横道创建停车决策
   *
   * 找到第一个（s坐标最小）需要停车的人行横道
   * 并创建停车决策
   */
  double min_s = std::numeric_limits<double>::max();
  /**< @brief 初始化为double最大值 */
  hdmap::PathOverlap* firsts_crosswalk_to_stop = nullptr;
  /**< @brief 第一个需要停车的人行横道 */

  /**
   * @brief 遍历需要停车的人行横道
   */
  for (auto crosswalk_to_stop : crosswalks_to_stop) {
    /**
     * @brief 构建停车决策
     */
    const auto* crosswalk_overlap = crosswalk_to_stop.first;
    /**< @brief 获取人行横道重叠区指针 */
    ADEBUG << "BuildStopDecision: crosswalk[" << crosswalk_overlap->object_id
           << "] start_s[" << crosswalk_overlap->start_s << "]";

    /**
     * @brief 创建虚拟障碍物ID
     *
     * CROSSWALK_VO_ID_PREFIX：人行横道虚拟障碍物ID前缀
     */
    std::string virtual_obstacle_id =
        CROSSWALK_VO_ID_PREFIX + crosswalk_overlap->object_id;

    /**
     * @brief 调用BuildStopDecision创建停车决策
     *
     * 参数：
     * - virtual_obstacle_id：虚拟障碍物ID
     * - crosswalk_overlap->start_s：停车线s坐标
     * - config_.stop_distance()：停车距离
     * - StopReasonCode::STOP_REASON_CROSSWALK：停车原因
     * - crosswalk_to_stop.second：需要等待的障碍物列表
     */
    util::BuildStopDecision(
        virtual_obstacle_id, crosswalk_overlap->start_s,
        config_.stop_distance(), StopReasonCode::STOP_REASON_CROSSWALK,
        crosswalk_to_stop.second, Getname(), frame, reference_line_info);

    /**
     * @brief 更新最小s坐标的人行横道
     */
    if (crosswalk_to_stop.first->start_s < min_s) {
      firsts_crosswalk_to_stop =
          const_cast<PathOverlap*>(crosswalk_to_stop.first);
      /**
       * @brief const_cast用法：
       *
       * 将const PathOverlap*转换为PathOverlap*
       * 因为firsts_crosswalk_to_stop需要是可修改的指针
       * 这是安全的，因为只用于后续的非const操作
       */
      min_s = crosswalk_to_stop.first->start_s;
    }
  }

  /**
   * @brief 更新人行横道状态
   *
   * 保存当前处理的 人行横道ID和计时器
   * 用于下一帧的恢复
   */
  if (firsts_crosswalk_to_stop) {
    /**
     * @brief 设置当前人行横道ID
     */
    std::string crosswalk = firsts_crosswalk_to_stop->object_id;
    mutable_crosswalk_status->set_crosswalk_id(crosswalk);
    mutable_crosswalk_status->clear_stop_time();
    /**< @brief 清除旧的停车时间 */

    /**
     * @brief 添加新的停车时间
     */
    for (const auto& timer : crosswalk_stop_timer[crosswalk]) {
      auto* stop_time = mutable_crosswalk_status->add_stop_time();
      /**< @brief 添加新元素到repeated字段 */
      stop_time->set_obstacle_id(timer.first);
      /**< @brief 设置障碍物ID */
      stop_time->set_stop_timestamp_sec(timer.second);
      /**< @brief 设置停车时间戳 */
      ADEBUG << "UPDATE stop_time: id[" << crosswalk << "] obstacle_id["
             << timer.first << "] stop_timestamp[" << timer.second << "]";
    }

    /**
     * @brief 更新已完成的人行横道列表
     *
     * 将s坐标小于当前人行横道的都标记为已完成
     */
    mutable_crosswalk_status->clear_finished_crosswalk();
    /**< @brief 清除旧的已完成列表 */
    for (auto crosswalk_overlap : crosswalk_overlaps_) {
      if (crosswalk_overlap->start_s < firsts_crosswalk_to_stop->start_s) {
        mutable_crosswalk_status->add_finished_crosswalk(
            crosswalk_overlap->object_id);
        ADEBUG << "UPDATE finished_crosswalk: " << crosswalk_overlap->object_id;
      }
    }
  }

  ADEBUG << "crosswalk_status: " << mutable_crosswalk_status->DebugString();
  /**
   * @brief 输出调试信息
   */
}

/**
 * @brief 查找人行横道
 *
 * @param reference_line_info 参考线信息
 * @return bool 是否找到人行横道
 *
 * 功能说明：
 * 在参考线上查找所有人行横道重叠区
 *
 * 算法流程：
 * 1. 清空原有的人行横道列表
 * 2. 获取参考线上的所有人行横道重叠区
 * 3. 将重叠区指针存储到列表中
 * 4. 返回是否找到
 */
bool Crosswalk::FindCrosswalks(ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(reference_line_info);
  /**< @brief 空指针检查 */

  crosswalk_overlaps_.clear();
  /**< @brief 清空原有列表 */

  /**
   * @brief 获取参考线上的人行横道重叠区
   *
   * reference_line().map_path()：获取地图路径
   * .crosswalk_overlaps()：获取人行横道重叠区列表
   */
  const std::vector<hdmap::PathOverlap>& crosswalk_overlaps =
      reference_line_info->reference_line().map_path().crosswalk_overlaps();

  /**
   * @brief 将重叠区指针添加到列表
   */
  for (const hdmap::PathOverlap& crosswalk_overlap : crosswalk_overlaps) {
    crosswalk_overlaps_.push_back(&crosswalk_overlap);
    /**
     * @brief push_back：将元素添加到向量末尾
     * &crosswalk_overlap：获取引用并取地址，得到指针
     */
  }

  return crosswalk_overlaps_.size() > 0;
  /**< @brief 返回是否找到人行横道 */
}

/**
 * @brief 检查是否为障碍物停车
 *
 * @param reference_line_info 参考线信息
 * @param crosswalk_ptr 人行横道信息指针
 * @param obstacle 障碍物
 * @param stop_deceleration 停车减速度
 * @return bool 是否需要停车
 *
 * 功能说明：
 * 根据障碍物类型、位置、速度等判断是否需要停车等待
 *
 * 算法流程：
 * 1. 检查障碍物类型（仅处理PEDESTRIAN和BICYCLE）
 * 2. 检查障碍物是否在扩展的人行横道区域内
 * 3. 根据横向距离分情况判断：
 *    - >= loose_l_distance：仅路径交叉时停车
 *    - <= strict_l_distance：
 *      - 在道路上：总是停车
 *      - 不在道路上：路径交叉或向自车移动时停车
 *    - 介于之间：使用历史决策
 * 4. 检查减速度是否合理
 *
 * C++语法说明：
 * - const CrosswalkInfoConstPtr：常量智能指针
 * - const Obstacle&：常量引用
 * - std::fabs：浮点数绝对值
 * - Vec2d::InnerProd：向量内积
 */
bool Crosswalk::CheckStopForObstacle(
    ReferenceLineInfo* const reference_line_info,
    const CrosswalkInfoConstPtr crosswalk_ptr, const Obstacle& obstacle,
    const double stop_deceleration) {
  CHECK_NOTNULL(reference_line_info);
  /**< @brief 空指针检查 */

  std::string crosswalk_id = crosswalk_ptr->id().id();
  /**< @brief 获取人行横道ID */

  const PerceptionObstacle& perception_obstacle = obstacle.Perception();
  /**< @brief 获取感知障碍物信息 */
  const std::string& obstacle_id = obstacle.Id();
  /**< @brief 障碍物ID */
  PerceptionObstacle::Type obstacle_type = perception_obstacle.type();
  /**< @brief 障碍物类型 */
  std::string obstacle_type_name = PerceptionObstacle_Type_Name(obstacle_type);
  /**< @brief 类型名称字符串 */
  double adc_end_edge_s = reference_line_info->AdcSlBoundary().start_s();
  /**< @brief 自车后边缘s坐标 */

  /**
   * @brief 条件1：检查障碍物类型
   *
   * 只处理行人和自行车
   * PEDESTRIAN：行人
   * BICYCLE：自行车
   */
  if (obstacle_type != PerceptionObstacle::PEDESTRIAN &&
      obstacle_type != PerceptionObstacle::BICYCLE) {
    ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
           << "]. skip";
    return false;  /**< 不是行人或自行车，不停车 */
  }

  /**
   * @brief 扩展人行横道多边形
   *
   * 人行横道扩展区域包含人行道等相邻区域
   * 用于更准确地判断行人是否会影响到车辆
   *
   * ExpandByDistance：按配置的距离扩展多边形
   * IsPointIn：判断点是否在多边形内
   */
  Vec2d point(perception_obstacle.position().x(),
              perception_obstacle.position().y());
  /**< @brief 障碍物位置二维向量 */
  const Polygon2d crosswalk_exp_poly =
      crosswalk_ptr->polygon().ExpandByDistance(config_.expand_s_distance());
  /**< @brief 扩展后的人行横道多边形 */
  bool in_expanded_crosswalk = crosswalk_exp_poly.IsPointIn(point);
  /**< @brief 障碍物是否在扩展区域内 */

  /**
   * @brief 如果障碍物不在扩展区域内，跳过
   */
  if (!in_expanded_crosswalk) {
    ADEBUG << "skip: obstacle_id[" << obstacle_id << "] type["
           << obstacle_type_name << "] crosswalk_id[" << crosswalk_id
           << "]: not in crosswalk expanded area";
    return false;
  }

  const auto& reference_line = reference_line_info->reference_line();
  /**< @brief 获取参考线 */

  /**
   * @brief 将障碍物位置转换为SL坐标
   *
   * XYToSL：将XY坐标（地理坐标）转换为SL坐标（沿路径/垂直路径）
   */
  common::SLPoint obstacle_sl_point;
  reference_line.XYToSL(perception_obstacle.position(), &obstacle_sl_point);

  auto& obstacle_sl_boundary = obstacle.PerceptionSLBoundary();
  /**< @brief 获取障碍物的SL边界 */

  /**
   * @brief 计算障碍物横向距离
   *
   * 取start_l和end_l的绝对值的最小值
   * 表示障碍物到路径中心线的最小横向距离
   */
  const double obstacle_l_distance =
      std::min(std::fabs(obstacle_sl_boundary.start_l()),
               std::fabs(obstacle_sl_boundary.end_l()));
  /**< @brief 障碍物横向距离 */

  /**
   * @brief 判断障碍物位置关系
   *
   * is_on_lane：是否在车道上
   * is_on_road：是否在道路上（比车道范围更广）
   * is_path_cross：预测轨迹是否与自车路径交叉
   */
  const bool is_on_lane =
      reference_line.IsOnLane(obstacle.PerceptionSLBoundary());
  const bool is_on_road =
      reference_line.IsOnRoad(obstacle.PerceptionSLBoundary());
  const bool is_path_cross = !obstacle.reference_line_st_boundary().IsEmpty();
  /**< @brief ST边界非空表示会与自车路径交叉 */

  ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
         << "] crosswalk_id[" << crosswalk_id << "] obstacle_l["
         << obstacle_sl_point.l() << "] within_expanded_crosswalk_area["
         << in_expanded_crosswalk << "] obstacle_l_distance["
         << obstacle_l_distance << "] on_lane[" << is_on_lane << "] is_on_road["
         << is_on_road << "] is_path_cross[" << is_path_cross << "]";

  /**
   * @brief 根据横向距离判断是否停车
   */
  bool stop = false;

  /**
   * @brief 情况1：横向距离足够大 >= loose_l_distance
   *
   * 只有当预测轨迹与自车路径交叉时才停车
   */
  if (obstacle_l_distance >= config_.stop_loose_l_distance()) {
    if (is_path_cross) {
      stop = true;
      ADEBUG << "need_stop(>=l2): obstacle_id[" << obstacle_id << "] type["
             << obstacle_type_name << "] crosswalk_id[" << crosswalk_id << "]";
    }
  }
  /**
   * @brief 情况2：横向距离较小 <= strict_l_distance
   */
  else if (obstacle_l_distance <= config_.stop_strict_l_distance()) {
    if (is_on_road) {
      /**
       * @brief 在道路上：总是停车
       *
       * 条件：障碍物在自车前方
       */
      if (obstacle_sl_point.s() > adc_end_edge_s) {
        stop = true;
        ADEBUG << "need_stop(<=l1): obstacle_id[" << obstacle_id << "] type["
               << obstacle_type_name << "] s[" << obstacle_sl_point.s()
               << "] adc_end_edge_s[ " << adc_end_edge_s << "] crosswalk_id["
               << crosswalk_id << "] ON_ROAD";
      }
    } else {
      /**
       * @brief 不在道路上（在人行横道/分隔带等）
       */
      if (is_path_cross) {
        /**
         * @brief 路径交叉时停车
         */
        stop = true;
        ADEBUG << "need_stop(<=l1): obstacle_id[" << obstacle_id << "] type["
               << obstacle_type_name << "] crosswalk_id[" << crosswalk_id
               << "] PATH_CRSOSS";
      } else {
        /**
         * @brief 检查行人是否向自车移动
         *
         * 使用速度向量和位置向量计算内积
         * 如果内积 > 0，表示障碍物在向自车方向移动
         */
        const auto obstacle_v = Vec2d(perception_obstacle.velocity().x(),
                                      perception_obstacle.velocity().y());
        /**< @brief 障碍物速度向量 */
        const auto adc_path_point =
            Vec2d(injector_->ego_info()->start_point().path_point().x(),
                  injector_->ego_info()->start_point().path_point().y());
        /**< @brief 自车位置向量 */
        const auto ovstacle_position =
            Vec2d(perception_obstacle.position().x(),
                  perception_obstacle.position().y());
        /**< @brief 障碍物位置向量 */
        auto obs_to_adc = adc_path_point - ovstacle_position;
        /**< @brief 障碍物指向自车的向量 */

        const double kEpsilon = 1e-6;
        /**< @brief 很小的时间，用于避免除零 */
        if (obstacle_v.InnerProd(obs_to_adc) > kEpsilon) {
          /**
           * @brief InnerProd > 0表示夹角小于90度，即向自车移动
           */
          stop = true;
          ADEBUG << "need_stop(<=l1): obstacle_id[" << obstacle_id << "] type["
                 << obstacle_type_name << "] crosswalk_id[" << crosswalk_id
                 << "] MOVING_TOWARD_ADC";
        }
      }
    }
  }
  /**
   * @brief 情况3：横向距离介于两者之间
   *
   * 使用历史决策平滑，避免决策不稳定
   * TODO注释说明这是临时实现
   */
  else {
    if (is_path_cross) {
      stop = true;
    }
    ADEBUG << "need_stop(between l1 & l2): obstacle_id[" << obstacle_id
           << "] type[" << obstacle_type_name << "] obstacle_l_distance["
           << obstacle_l_distance << "] crosswalk_id[" << crosswalk_id
           << "] USE_PREVIOUS_DECISION";
  }

  /**
   * @brief 检查停车减速度是否合理
   *
   * 如果减速度过大但障碍物距离足够远，可以忽略
   */
  if (stop) {
    if (stop_deceleration >= config_.max_stop_deceleration()) {
      if (obstacle_l_distance > config_.stop_strict_l_distance()) {
        /**
         * @brief 减速度过大但距离也较远，可以安全忽略
         */
        stop = false;
      }
      AWARN << "crosswalk_id[" << crosswalk_id << "] stop_deceleration["
            << stop_deceleration << "]";
    }
  }

  return stop;
}

}  // namespace planning
/**
 * @brief 命名空间结束标记
 */
}  // namespace apollo
/**
 * @brief Apollo命名空间结束标记
 */
