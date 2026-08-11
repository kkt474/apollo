/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
 * @file local_view.h
 * @brief 本地视图头文件
 *
 * 功能说明：
 * 定义了规划模块的输入数据结构LocalView
 * 包含规划所需的所有输入数据
 *
 * 核心概念：
 * - LocalView：本地视图，封装规划所需的所有输入
 * - std::shared_ptr：智能指针，管理数据生命周期
 * - Protobuf消息：跨模块数据结构序列化
 *
 * 设计目的：
 * - 提供规划模块的完整输入视图
 * - 避免在函数参数中传递大量单独参数
 * - 统一数据访问接口
 *
 * 数据流向：
 * 输入数据 -> LocalView -> 规划算法 -> 输出轨迹
 *
 * C++语法说明：
 * - #pragma once：头文件保护
 * - struct：结构体声明
 * - std::shared_ptr<T>：引用计数智能指针
 * - ::apollo::xxx：命名空间限定
 **/

/**
 * @brief 头文件保护
 *
 * #pragma once：
 * 防止头文件被重复包含
 * 现代编译器支持，简洁高效
 */
#pragma once

#include <memory>
/**
 * @brief 智能指针头文件
 *
 * 提供std::shared_ptr智能指针模板类
 * 用于管理对象的生命周期
 */

#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
/**
 * @brief 底盘消息头文件
 *
 * canbus::Chassis：
 * - 车辆底盘状态信息
 * - 包含速度、加速度、方向盘角度、档位等
 */

#include "modules/common_msgs/localization_msgs/localization.pb.h"
/**
 * @brief 定位消息头文件
 *
 * localization::LocalizationEstimate：
 * - 融合定位结果
 * - 包含位置(x,y,z)、姿态(roll,pitch,yaw)、速度等
 */

#include "modules/common_msgs/perception_msgs/traffic_light_detection.pb.h"
/**
 * @brief 交通灯检测消息头文件
 *
 * perception::TrafficLightDetection：
 * - 交通灯检测结果
 * - 包含灯的颜色、位置、置信度等
 */

#include "modules/common_msgs/planning_msgs/navigation.pb.h"
/**
 * @brief 导航消息头文件
 *
 * relative_map::MapMsg：
 * - 相对地图信息
 * - 用于相对位置导航
 */

#include "modules/common_msgs/planning_msgs/pad_msg.pb.h"
/**
 * @brief Pad消息头文件
 *
 * PadMessage：
 * - 人工干预指令
 * - 用于启动、停止、紧急刹车等
 */

#include "modules/common_msgs/planning_msgs/planning_command.pb.h"
/**
 * @brief 规划命令消息头文件
 *
 * PlanningCommand：
 * - 发送给规划模块的外部命令
 * - 用于重新路由、目的地变更等
 */

#include "modules/common_msgs/prediction_msgs/prediction_obstacle.pb.h"
/**
 * @brief 预测障碍物消息头文件
 *
 * prediction::PredictionObstacles：
 * - 障碍物轨迹预测结果
 * - 包含每个障碍物的预测轨迹和意图
 */

#include "modules/common_msgs/routing_msgs/routing.pb.h"
/**
 * @brief 路由消息头文件
 *
 * routing::LaneWaypoint：
 * - 车道路点
 * - 表示路由路径上的一个点
 */

#include "modules/common_msgs/storytelling_msgs/story.pb.h"
/**
 * @brief Storytelling消息头文件
 *
 * storytelling::Stories：
 * - 驾驶场景故事数据
 * - 用于记录驾驶过程中的重要事件
 */

#include "modules/common_msgs/control_msgs/control_interactive_msg.pb.h"
/**
 * @brief 控制交互消息头文件
 *
 * control::ControlInteractiveMsg：
 * - 与控制模块交互的信息
 */

namespace apollo {
/**
 * @brief Apollo项目主命名空间
 */

namespace planning {
/**
 * @brief 规划模块命名空间
 */

/**
 * @struct LocalView
 * @brief 本地视图结构体
 *
 * 功能说明：
 * 封装规划模块所需的所有输入数据
 * 以智能指针形式存储，支持空值
 *
 * 设计特点：
 * - 所有成员都是std::shared_ptr，可为空
 * - 使用命名空间限定避免类型名冲突
 * - 集中管理，便于维护和扩展
 *
 * C++语法说明：
 * - struct：结构体声明
 * - std::shared_ptr<T>：引用计数智能指针
 *   - shared_ptr：多个指针可共享所有权
 *   - 最后一个指针销毁时释放对象
 *   - 线程安全的引用计数
 */
struct LocalView {
  /**
   * @brief 预测障碍物
   *
   * std::shared_ptr<prediction::PredictionObstacles>：
   * - prediction::PredictionObstacles：预测模块输出的障碍物列表
   * - 包含所有感知到的障碍物的轨迹预测
   * - shared_ptr：智能指针，避免拷贝
   */
  std::shared_ptr<prediction::PredictionObstacles> prediction_obstacles;

  /**
   * @brief 底盘状态
   *
   * std::shared_ptr<canbus::Chassis>：
   * - canbus::Chassis：底盘CAN总线的车辆状态
   * - 包含：速度、加速度、方向盘角度、档位、刹车油门等
   */
  std::shared_ptr<canbus::Chassis> chassis;

  /**
   * @brief 定位估计
   *
   * std::shared_ptr<localization::LocalizationEstimate>：
   * - localization::LocalizationEstimate：融合定位结果
   * - 包含：位置(x,y,z)、姿态(roll,pitch,yaw)、速度、航向等
   */
  std::shared_ptr<localization::LocalizationEstimate> localization_estimate;

  /**
   * @brief 交通灯检测
   *
   * std::shared_ptr<perception::TrafficLightDetection>：
   * - perception::TrafficLightDetection：交通灯检测结果
   * - 包含：检测到的灯的颜色、位置、置信度等
   */
  std::shared_ptr<perception::TrafficLightDetection> traffic_light;

  /**
   * @brief 相对地图
   *
   * std::shared_ptr<relative_map::MapMsg>：
   * - relative_map::MapMsg：相对地图信息
   * - 用于没有绝对定位时的相对导航
   */
  std::shared_ptr<relative_map::MapMsg> relative_map;

  /**
   * @brief Pad消息（人工干预）
   *
   * std::shared_ptr<PadMessage>：
   * - PadMessage：驾驶员操作指令
   * - 包含：启动、停止、换道请求、巡航控制等
   */
  std::shared_ptr<PadMessage> pad_msg;

  /**
   * @brief 驾驶故事
   *
   * std::shared_ptr<storytelling::Stories>：
   * - storytelling::Stories：驾驶场景故事记录
   * - 用于记录驾驶过程中的重要事件和场景
   */
  std::shared_ptr<storytelling::Stories> stories;

  /**
   * @brief 规划命令
   *
   * std::shared_ptr<PlanningCommand>：
   * - PlanningCommand：外部发送给规划模块的命令
   * - 包含：重新路由、目的地变更等指令
   */
  std::shared_ptr<PlanningCommand> planning_command;

  /**
   * @brief 终点车道路点
   *
   * std::shared_ptr<routing::LaneWaypoint>：
   * - routing::LaneWaypoint：路由终点路点
   * - 描述路由的终点位置
   */
  std::shared_ptr<routing::LaneWaypoint> end_lane_way_point;

  /**
   * @brief 控制交互消息
   *
   * std::shared_ptr<control::ControlInteractiveMsg>：
   * - control::ControlInteractiveMsg：与控制模块的交互消息
   * - 用于双向通信和控制反馈
   */
  std::shared_ptr<control::ControlInteractiveMsg> control_interactive_msg;

  /**
   * @brief 感知精确停靠信息
   *
   * std::shared_ptr<perception::PerceptionAccurateDockInfo>：
   * - perception::PerceptionAccurateDockInfo：精确停靠检测信息
   * - 用于自动泊车等场景的精确停靠
   */
  std::shared_ptr<perception::PerceptionAccurateDockInfo> perception_dock;
};

}  // namespace planning
/**
 * @brief 规划命名空间结束标记
 */

}  // namespace apollo
/**
 * @brief Apollo命名空间结束标记
 */
