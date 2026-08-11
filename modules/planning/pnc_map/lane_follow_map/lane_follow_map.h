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
 ******************************************************************************/

/**
 * @file lane_follow_map.h
 *
 * @brief 车道跟随地图类头文件
 *
 * 功能说明：
 * 本头文件声明了LaneFollowMap类，是Apollo规划模块中处理车道跟随场景的核心地图管理类
 * 负责管理与路由相关的地图数据，包括：
 * - 路由航点索引管理
 * - 车辆在路由上的位置更新
 * - 路径段（RouteSegments）的生成与扩展
 * - 目的地距离计算
 *
 * 类的继承关系：
 * LaneFollowMap -> PncMapBase（父类）
 * PncMapBase是PNC地图的基类，定义了地图接口
 *
 * 核心概念：
 * - RouteSegments：路由段列表，描述车辆可行驶的路径
 * - LaneWaypoint：车道航点，包含车道指针和沿车道的距离s
 * - LaneSegment：车道段，包含车道的起止距离
 * - Passage：通道，道路中的一个通行区域
 *
 * C++语法说明：
 * - #pragma once：预处理器指令，防止头文件被重复包含
 * - class：类声明，用户定义类型
 * - public/private：访问限定符
 * - virtual/override：C++多态支持
 * - struct：结构体声明
 * - ::运算符：作用域解析运算符
 **/

#pragma once

/**
 * @brief 标准库头文件
 *
 * C++语法说明：
 * - <list>：双向链表容器，提供std::list
 * - <memory>：智能指针和内存管理工具
 * - <string>：字符串类
 * - <unordered_set>：基于哈希表的集合容器
 * - <vector>：动态数组容器
 */
#include <list>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

/**
 * @brief Apollo车辆状态消息
 *
 * C++语法说明：
 * - "modules/common/vehicle_state/proto/vehicle_state.pb.h"：
 *   protobuf生成的头文件
 *   定义了VehicleState消息类型
 *   包含车辆位置、速度、航向等信息
 */
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"

/**
 * @brief Apollo路由消息
 *
 * C++语法说明：
 * - "modules/common_msgs/routing_msgs/routing.pb.h"：
 *   路由模块的protobuf消息头文件
 *   定义了RoutingResponse、LaneWaypoint、RoadSegment等
 */
#include "modules/common_msgs/routing_msgs/routing.pb.h"

/**
 * @brief Cyber RT插件管理器
 *
 * C++语法说明：
 * - cyber/：Apollo Cyber RT框架的命名空间
 * - plugin_manager/：插件管理相关
 * - CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN：
 *   插件注册宏，将类注册为Cyber RT插件
 *   使类可以通过插件管理器动态加载
 */
#include "cyber/plugin_manager/plugin_manager.h"

/**
 * @brief 高精地图接口
 *
 * C++语法说明：
 * - hdmap.h：高精地图（High Definition Map）的接口定义
 *   提供GetLaneById、GetRoadById等地图查询方法
 *   定义了LaneInfo、RoadInfo等数据结构
 */
#include "modules/map/hdmap/hdmap.h"

/**
 * @brief PNC地图路径定义
 *
 * C++语法说明：
 * - path.h：定义Path、LaneWaypoint等路径相关结构
 *   LaneWaypoint包含lane指针、s值、l值
 *   用于描述车道上的位置
 */
#include "modules/map/pnc_map/path.h"

/**
 * @brief PNC地图基类
 *
 * C++语法说明：
 * - pnc_map_base.h：PncMapBase基类声明
 *   定义了PNC地图的接口（纯虚函数）
 *   LaneFollowMap继承自此基类
 */
#include "modules/map/pnc_map/pnc_map_base.h"

/**
 * @brief 路由段定义
 *
 * C++语法说明：
 * - route_segments.h：RouteSegments类定义
 *   继承自std::vector<LaneSegment>
 *   表示一系列连续的车道段
 */
#include "modules/map/pnc_map/route_segments.h"

namespace apollo {
/**
 * @brief Apollo主命名空间
 *
 * 功能说明：
 * 所有Apollo模块的代码都在apollo命名空间下
 * 避免与其他库的命名冲突
 */
namespace planning {
/**
 * @brief 规划模块命名空间
 *
 * 功能说明：
 * planning命名空间包含所有规划相关的类和函数
 */

/**
 * @class LaneFollowMap
 * @brief 车道跟随地图类
 *
 * 功能说明：
 * LaneFollowMap是处理车道跟随场景的核心地图管理类
 * 继承自PncMapBase基类
 * 负责管理与路由相关的地图数据
 *
 * 主要职责：
 * 1. 解析和处理路由请求
 * 2. 维护车辆在路由上的位置
 * 3. 生成可供规划使用的路径段
 * 4. 计算到目的地的距离
 * 5. 处理车道变更逻辑
 *
 * 使用场景：
 * - 车道跟随场景下的路径规划
 * - 路由跟踪和位置更新
 * - 相邻车道/通道查询
 *
 * C++语法说明：
 * - class LaneFollowMap : public PncMapBase：
 *   类继承声明
 *   LaneFollowMap公有继承自PncMapBase
 *   public继承意味着基类的public成员仍是public
 *
 * - virtual ~LaneFollowMap() = default：
 *   虚析构函数，允许通过基类指针删除派生类对象
 *   = default使用编译器生成的默认实现
 */
class LaneFollowMap : public PncMapBase {
 public:
  /**
   * @brief 默认构造函数
   *
   * 功能说明：
   * 初始化高精地图指针
   * 使用HDMapUtil获取全局地图实例
   *
   * C++语法说明：
   * - LaneFollowMap()：
   *   类名加括号表示构造函数
   *   没有返回类型声明
   */
  LaneFollowMap();

  /**
   * @brief 虚析构函数
   *
   * 功能说明：
   * 确保通过基类指针删除派生类对象时
   * 能正确调用派生类的析构函数
   *
   * C++语法说明：
   * - virtual：
   *   虚函数关键字，支持运行时多态
   *   与override配合实现多态
   *
   * - ~LaneFollowMap()：
   *   析构函数，类名前加~
   *   对象生命周期结束时自动调用
   *
   * - = default：
   *   默认实现关键字
   *   使用编译器自动生成的析构函数
   *   比起空实现更高效
   */
  virtual ~LaneFollowMap() = default;

  /**
   * @brief 检查是否可以处理该规划命令
   *
   * @param command 规划命令引用
   * @return bool 如果可以处理返回true
   *
   * 功能说明：
   * 判断传入的规划命令是否是车道跟随类型
   * 只有车道跟随命令才能被LaneFollowMap处理
   *
   * C++语法说明：
   * - const planning::PlanningCommand &command：
   *   const：承诺不修改参数
   *   &：引用，避免参数拷贝
   *   PlanningCommand：protobuf消息类型
   *
   * - const override：
   *   const：const成员函数，不会修改成员变量
   *   override：重写基类虚函数
   *   两个关键字都用于多态场景
   */
  bool CanProcess(const planning::PlanningCommand &command) const override;

  /**
   * @brief 更新规划命令
   *
   * @param command 新的规划命令
   * @return bool 更新成功返回true
   *
   * 功能说明：
   * 解析并存储新的规划命令
   * 构建路由索引和路径段信息
   *
   * C++语法说明：
   * - override：
   *   显式声明重写基类虚函数
   *   编译器会检查基类是否有同名虚函数
   */
  bool UpdatePlanningCommand(const planning::PlanningCommand &command) override;

  /**
   * @brief 获取路由段（使用默认前后向距离）
   *
   * @param vehicle_state 车辆状态
   * @param route_segments 输出参数，路由段列表指针
   * @return bool 获取成功返回true
   *
   * 功能说明：
   * 使用默认的前后向距离获取路由段
   * 前后向距离根据当前车速计算
   *
   * C++语法说明：
   * - std::list<apollo::hdmap::RouteSegments> *const route_segments：
   *   *const：指向RouteSegments列表的常量指针
   *   指针本身是常量（不能改变指向）
   *   但可以通过指针修改所指对象
   *
   * - override：
   *   表示重写基类的纯虚函数
   */
  bool GetRouteSegments(
      const common::VehicleState &vehicle_state,
      std::list<apollo::hdmap::RouteSegments> *const route_segments) override;

  /**
   * @brief 扩展路段
   *
   * @param segments 原始路段
   * @param start_s 起始距离
   * @param end_s 结束距离
   * @param truncated_segments 输出参数，扩展后的路段
   * @return bool 扩展成功返回true
   *
   * 功能说明：
   * 根据指定的起止距离扩展路段
   * 包括向后和向前扩展
   *
   * C++语法说明：
   * - const apollo::hdmap::RouteSegments &segments：
   *   常量引用参数
   *   确保不会修改原始路段
   *
   * - const override：
   *   const：const成员函数
   *   override：重写基类函数
   */
  bool ExtendSegments(
      const apollo::hdmap::RouteSegments &segments, double start_s,
      double end_s,
      apollo::hdmap::RouteSegments *const truncated_segments) const override;

  /**
   * @brief 获取剩余路由航点
   *
   * @return std::vector<routing::LaneWaypoint> 从下一个航点开始的剩余航点列表
   *
   * 功能说明：
   * 返回从下一个待处理航点开始的所有路由航点
   *
   * C++语法说明：
   * - std::vector<routing::LaneWaypoint>：
   *   返回值类型是航点向量
   *   包含routing::LaneWaypoint类型
   */
  std::vector<routing::LaneWaypoint> FutureRouteWaypoints() const override;

  /**
   * @brief 获取目的地航点
   *
   * @param end_point 输出参数，指向目的地航点的共享指针引用
   *
   * 功能说明：
   * 从路由请求中获取最后一个航点作为目的地
   *
   * C++语法说明：
   * - std::shared_ptr<routing::LaneWaypoint> &end_point：
   *   shared_ptr：智能指针，引用计数管理对象生命周期
   *   &：引用参数，用于输出结果
   */
  void GetEndLaneWayPoint(
      std::shared_ptr<routing::LaneWaypoint> &end_point) const override;

  /**
   * @brief 根据ID获取车道信息
   *
   * @param id 车道ID
   * @return hdmap::LaneInfoConstPtr 车道信息常量指针
   *
   * 功能说明：
   * 根据车道ID从HDMap中获取车道详细信息
   *
   * C++语法说明：
   * - hdmap::LaneInfoConstPtr：
   *   LaneInfo的常量指针类型
   *   通常是shared_ptr<const LaneInfo>
   */
  hdmap::LaneInfoConstPtr GetLaneById(const hdmap::Id &id) const override;

  /**
   * @brief 从路由中获取最近点
   *
   * @param state 车辆状态
   * @param waypoint 输出参数，最近的车道航点
   * @return bool 查找成功返回true
   *
   * 功能说明：
   * 根据车辆当前位置找到最近的路由航点
   */
  bool GetNearestPointFromRouting(
      const common::VehicleState &state,
      apollo::hdmap::LaneWaypoint *waypoint) const override;

  /**
   * @brief 获取到目的地距离
   *
   * @return double 到目的地的距离（米）
   *
   * 功能说明：
   * 计算自车到目的地的沿路由距离
   */
  double GetDistanceToDestination() const override;

  /**
   * @brief 获取自车航点
   *
   * @return apollo::hdmap::LaneWaypoint 自车当前位置对应的车道航点
   *
   * 功能说明：
   * 返回最近一次更新的自车航点
   */
  apollo::hdmap::LaneWaypoint GetAdcWaypoint() const override;

 private:
  /**
   * @brief 验证规划命令有效性（私有重写）
   *
   * @param command 待验证的规划命令
   * @return bool 命令有效返回true
   *
   * 功能说明：
   * 私有成员函数
   * 内部使用，不对外暴露
   *
   * C++语法说明：
   * - private：
   *   私有成员，只能在类内部访问
   *   子类也不能直接访问
   *
   * - override：
   *   重写基类的虚函数
   */
  bool IsValid(const planning::PlanningCommand &command) const override;

  /**
   * @brief 获取路由段（完整版本）
   *
   * @param vehicle_state 车辆状态
   * @param backward_length 后向距离
   * @param forward_length 前向距离
   * @param route_segments 输出参数，路由段列表
   * @return bool 获取成功返回true
   *
   * 功能说明：
   * 根据给定的距离范围获取车辆可行驶的路由段
   * 包括当前通道和相邻通道
   */
  bool GetRouteSegments(
      const common::VehicleState &vehicle_state, const double backward_length,
      const double forward_length,
      std::list<apollo::hdmap::RouteSegments> *const route_segments);

  /**
   * @brief 扩展路段（使用坐标点版本）
   *
   * @param segments 原始路段
   * @param point 坐标点
   * @param look_forward 前向扩展距离
   * @param look_backward 后向扩展距离
   * @param extended_segments 输出参数，扩展后的路段
   * @return bool 扩展成功返回true
   *
   * 功能说明：
   * 将点投影到路段上，然后调用距离版本进行扩展
   */
  bool ExtendSegments(const apollo::hdmap::RouteSegments &segments,
                      const common::PointENU &point, double look_forward,
                      double look_backward,
                      apollo::hdmap::RouteSegments *extended_segments);

  /**
   * @brief 更新车辆状态
   *
   * @param vehicle_state 车辆当前状态
   * @return bool 更新成功返回true
   *
   * 功能说明：
   * 根据车辆状态更新内部维护的车辆位置信息
   */
  bool UpdateVehicleState(const common::VehicleState &vehicle_state);

  /**
   * @brief 获取航点索引
   *
   * @param waypoint 目标车道航点
   * @return int 航点在路由中的索引
   *
   * 功能说明：
   * 综合使用前向和后向搜索找到航点的最佳匹配索引
   *
   * C++语法说明：
   * - int GetWaypointIndex(...) const：
   *   返回int类型的索引值
   *   const成员函数
   */
  int GetWaypointIndex(const apollo::hdmap::LaneWaypoint &waypoint) const;

  /**
   * @brief 将通道转换为路段列表
   *
   * @param passage 路由通道
   * @param segments 输出参数，转换后的路段列表
   * @return bool 转换成功返回true
   *
   * 功能说明：
   * 将routing::Passage转换为hdmap::RouteSegments
   */
  bool PassageToSegments(routing::Passage passage,
                         apollo::hdmap::RouteSegments *segments) const;

  /**
   * @brief 将点投影到路段
   *
   * @param point_enu ENU坐标点
   * @param segments 路段列表
   * @param waypoint 输出参数，投影后的航点
   * @return bool 投影成功返回true
   *
   * 功能说明：
   * 将ENU坐标点投影到路段上
   */
  bool ProjectToSegments(const common::PointENU &point_enu,
                         const apollo::hdmap::RouteSegments &segments,
                         apollo::hdmap::LaneWaypoint *waypoint) const;

  /**
   * @brief 将车道追加到点列表（静态方法）
   *
   * @param lane 车道指针
   * @param start_s 起始距离
   * @param end_s 结束距离
   * @param points 输出参数，地图路径点列表
   *
   * 功能说明：
   * 将指定车道的点追加到列表中
   *
   * C++语法说明：
   * - static：
   *   静态成员函数
   *   不依赖于类的实例即可调用
   *   不能访问类的非静态成员
   */
  static void AppendLaneToPoints(
      apollo::hdmap::LaneInfoConstPtr lane, const double start_s,
      const double end_s,
      std::vector<apollo::hdmap::MapPathPoint> *const points);

  /**
   * @brief 获取路由前驱车道
   *
   * @param lane 当前车道
   * @return hdmap::LaneInfoConstPtr 前驱车道指针
   *
   * 功能说明：
   * 获取指定车道的路由前驱车道
   */
  apollo::hdmap::LaneInfoConstPtr GetRoutePredecessor(
      apollo::hdmap::LaneInfoConstPtr lane) const;

  /**
   * @brief 获取路由后继车道
   *
   * @param lane 当前车道
   * @return hdmap::LaneInfoConstPtr 后继车道指针
   *
   * 功能说明：
   * 获取指定车道的路由后继车道
   */
  apollo::hdmap::LaneInfoConstPtr GetRouteSuccessor(
      apollo::hdmap::LaneInfoConstPtr lane) const;

  /**
   * @brief 获取相邻通道列表
   *
   * @param road 道路信息
   * @param start_passage 起始通道索引
   * @return std::vector<int> 相邻通道的索引列表
   *
   * 功能说明：
   * 根据起始通道找到所有可行驶的相邻通道
   * 用于车道变更决策
   */
  std::vector<int> GetNeighborPassages(const routing::RoadSegment &road,
                                       int start_passage) const;

  /**
   * @brief 将路由航点转换为HDMap航点
   *
   * @param waypoint 路由模块的车道航点
   * @return apollo::hdmap::LaneWaypoint HDMap的车道航点
   *
   * 功能说明：
   * 将routing::LaneWaypoint转换为hdmap::LaneWaypoint
   */
  apollo::hdmap::LaneWaypoint ToLaneWaypoint(
      const routing::LaneWaypoint &waypoint) const;

  /**
   * @brief 将路由段转换为HDMap段
   *
   * @param segment 路由模块的车道段
   * @return apollo::hdmap::LaneSegment HDMap的车道段
   *
   * 功能说明：
   * 将routing::LaneSegment转换为hdmap::LaneSegment
   */
  apollo::hdmap::LaneSegment ToLaneSegment(
      const routing::LaneSegment &segment) const;

  /**
   * @brief 更新下一个路由航点索引
   *
   * @param cur_index 当前航点索引
   *
   * 功能说明：
   * 根据车辆当前位置更新下一个需要处理的路由航点索引
   * 处理车辆前进和后退两种情况
   */
  void UpdateNextRoutingWaypointIndex(int cur_index);

  /**
   * @brief 向前搜索航点索引
   *
   * @param start 起始搜索位置
   * @param waypoint 待查找的车道航点
   * @return int 找到的索引
   *
   * 功能说明：
   * 从start位置向前遍历，找到第一个包含waypoint的路段
   */
  int SearchForwardWaypointIndex(
      int start, const apollo::hdmap::LaneWaypoint &waypoint) const;

  /**
   * @brief 向后搜索航点索引
   *
   * @param start 起始搜索位置
   * @param waypoint 待查找的车道航点
   * @return int 找到的索引
   *
   * 功能说明：
   * 从start位置向后遍历，找到第一个包含waypoint的路段
   */
  int SearchBackwardWaypointIndex(
      int start, const apollo::hdmap::LaneWaypoint &waypoint) const;

  /**
   * @brief 更新路由范围
   *
   * @param adc_index 自车当前位置索引
   *
   * 功能说明：
   * 根据自车位置计算需要关注的路由范围
   */
  void UpdateRoutingRange(int adc_index);

  /**
   * @brief 更新路由段车道ID集合
   *
   * @param route_segments 路由段列表
   *
   * 功能说明：
   * 遍历所有路由段，收集所有车道ID
   */
  void UpdateRouteSegmentsLaneIds(
      const std::list<hdmap::RouteSegments> *route_segments);

 private:
  /**
   * @struct RouteIndex
   * @brief 路由索引结构体
   *
   * 功能说明：
   * 存储路由中的车道段及其三维索引
   *
   * C++语法说明：
   * - struct：
   *   结构体声明
   *   默认访问限定符是public（与class不同）
   *   用于简单数据聚合
   */
  struct RouteIndex {
    apollo::hdmap::LaneSegment segment;  ///< 车道段信息
    std::array<int, 3> index;            ///< 三维索引：{道路索引, 通道索引, 车道索引}
  };

  /**
   * @brief 路由索引列表
   *
   * 功能说明：
   * 存储所有路由段及其索引信息
   *
   * C++语法说明：
   * - std::vector<RouteIndex>：
   *   动态数组，存储RouteIndex结构体
   *   可动态增长
   */
  std::vector<RouteIndex> route_indices_;

  /**
   * @brief 路由范围起始索引
   *
   * 功能说明：
   * 当前关注范围的起始位置
   */
  int range_start_ = 0;

  /**
   * @brief 路由范围结束索引
   *
   * 功能说明：
   * 当前关注范围的结束位置
   */
  int range_end_ = 0;

  /**
   * @brief 路由范围内的车道ID集合
   *
   * 功能说明：
   * 存储当前路由范围内的所有车道ID
   * 用于快速判断某车道是否在范围内
   *
   * C++语法说明：
   * - std::unordered_set<std::string>：
   *   无序集合（哈希表）
   *   查找时间复杂度为O(1)
   *   自动去重
   */
  std::unordered_set<std::string> range_lane_ids_;

  /**
   * @brief 所有车道ID集合
   *
   * 功能说明：
   * 存储路由请求涉及的所有车道ID
   */
  std::unordered_set<std::string> all_lane_ids_;

  /**
   * @brief 路由段内的车道ID集合
   *
   * 功能说明：
   * 存储当前路由段列表中的所有车道ID
   */
  std::unordered_set<std::string> route_segments_lane_ids_;

  /**
   * @brief 目的地车道段
   *
   * 功能说明：
   * 存储路由目的地所在的车道段
   *
   * C++语法说明：
   * - routing::LaneSegment：
   *   路由模块定义的车道段结构
   */
  routing::LaneSegment dest_lane_segment_;

  /**
   * @struct WaypointIndex
   * @brief 航点索引结构体
   *
   * 功能说明：
   * 将请求航点与路由索引关联
   *
   * C++语法说明：
   * - struct：
   *   结构体，包含构造函数
   *
   * - WaypointIndex(const apollo::hdmap::LaneWaypoint &waypoint, int index)
   *     : waypoint(waypoint), index(index) {}：
   *   构造函数初始化列表
   *   在函数体执行前初始化成员变量
   */
  struct WaypointIndex {
    apollo::hdmap::LaneWaypoint waypoint;  ///< 航点信息
    int index;                              ///< 航点在路由中的索引

    /**
     * @brief 构造函数
     *
     * @param waypoint 航点引用
     * @param index 索引值
     *
     * C++语法说明：
     * - : waypoint(waypoint), index(index)：
     *   初始化列表，直接初始化成员
     *   效率比在函数体内赋值更高
     */
    WaypointIndex(const apollo::hdmap::LaneWaypoint &waypoint, int index)
        : waypoint(waypoint), index(index) {}
  };

  /**
   * @brief 获取下一个航点索引
   *
   * @param index 当前索引
   * @return int 下一个航点索引
   *
   * 功能说明：
   * 边界处理的安全包装函数
   */
  int NextWaypointIndex(int index) const;

  /**
   * @brief 路由航点索引列表
   *
   * 功能说明：
   * 存储请求航点与路由索引的映射关系
   */
  std::vector<WaypointIndex> routing_waypoint_index_;

  /**
   * @brief 下一个待处理航点索引
   *
   * 功能说明：
   * 指向routing_waypoint_index_中下一个需要处理的航点
   *
   * C++语法说明：
   * - std::size_t：
   *   无符号整数类型
   *   用于表示大小和索引
   *   保证能存储任何对象的大小
   *
   * - = 0：
   *   默认初始化为0
   */
  std::size_t next_routing_waypoint_index_ = 0;

  /**
   * @brief 高精地图指针
   *
   * 功能说明：
   * 存储HDMap实例的指针
   *
   * C++语法说明：
   * - const hdmap::HDMap *：
   *   指向常量HDMap的指针
   *   指针本身可以修改
   *   但不能通过指针修改对象
   *
   * - = nullptr：
   *   空指针初始化
   *   表示当前没有有效的地图指针
   */
  const hdmap::HDMap *hdmap_ = nullptr;

  /**
   * @brief 路由是否相同标志
   *
   * 功能说明：
   * 标记当前路由是否与上一帧相同
   */
  bool is_same_routing_ = false;

  /**
   * @brief 自车状态
   *
   * 功能说明：
   * 存储最近一次更新的车辆状态
   *
   * C++语法说明：
   * - common::VehicleState：
   *   Apollo通用模块中的车辆状态类型
   *   包含位置、速度、航向等信息
   */
  common::VehicleState adc_state_;

  /**
   * @brief 自车路由索引
   *
   * 功能说明：
   * 自车在route_indices_中的索引
   * -1表示无效或未知位置
   */
  int adc_route_index_ = -1;

  /**
   * @brief 是否到达目的地标志
   *
   * 功能说明：
   * 标记自车是否应该开始考虑目的地
   * 在循环路由中，车辆可能多次经过目的地
   * 但只有在最后一次遇到目的地时才需要停车
   */
  bool stop_for_destination_ = false;
};

/**
 * @brief Cyber RT插件注册宏
 *
 * 功能说明：
 * 将LaneFollowMap类注册为Cyber RT的插件
 * 使得可以通过插件管理器动态加载此类
 *
 * C++语法说明：
 * - CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN：
 *   Apollo Cyber RT框架的插件注册宏
 *   第一个参数是类名
 *   第二个参数是基类名
 *   展开后会在全局注册表中添加条目
 *
 * 使用场景：
 * - 允许在运行时加载规划器
 * - 支持插件化的架构设计
 * - 方便扩展新的规划器类型
 */
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::LaneFollowMap,
                                     PncMapBase)

}  // namespace planning
}  // namespace apollo