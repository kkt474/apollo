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
 * @file path.h
 * @brief PNC地图路径类声明头文件
 *
 * 功能说明：
 * 该文件定义了PNC(Planning and Control)地图相关的路径类
 * 主要包括：
 * 1. LaneWaypoint：车道航点结构
 * 2. LaneSegment：车道段结构
 * 3. PathOverlap：路径重叠区域结构
 * 4. MapPathPoint：地图路径点类
 * 5. PathApproximation：路径近似类
 * 6. InterpolatedIndex：插值索引类
 * 7. Path：地图路径类
 *
 * 设计特点：
 * - 继承自Vec2d的路径点类
 * - 支持多种构造函数（值拷贝、移动语义）
 * - 支持路径近似以减少计算量
 * - 支持多种重叠区域类型
 *
 * C++语法说明：
 * - #pragma once：编译指示符，防止头文件重复包含
 * - struct vs class：struct默认public，class默认private
 * - = default：使用默认实现
 * - = delete：禁用特定函数
 * - std::move：移动语义
 * - std::function：函数包装器
 * - std::vector::emplace_back：就地构造元素
 */

/**
 * @brief 确保头文件只被包含一次
 *
 * C++语法说明：
 * #pragma once是编译指示符
 * 告诉编译器这个头文件只处理一次
 */
#pragma once

/**
 * @brief 标准库头文件
 *
 * C++语法说明：
 * - #include <string>：标准库字符串类型
 * - #include <utility>：工具类，包括std::move、std::pair
 * - #include <vector>：动态数组容器
 */
#include <string>
#include <utility>
#include <vector>

/**
 * @brief Apollo地图消息头文件
 *
 * C++语法说明：
 * map_lane.pb.h由proto文件编译生成
 * 包含地图车道相关的消息定义
 */
#include "modules/common_msgs/map_msgs/map_lane.pb.h"

/**
 * @brief Apollo 2D数学头文件
 *
 * C++语法说明：
 * - box2d.h：2D包围盒类
 * - line_segment2d.h：2D线段类
 * - vec2d.h：2D向量类
 */
#include "modules/common/math/box2d.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/vec2d.h"

/**
 * @brief Apollo HD地图头文件
 *
 * C++语法说明：
 * - hdmap.h：高精地图主类
 * - hdmap_common.h：地图通用类型定义
 * - hdmap_util.h：地图工具函数
 */
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/map/hdmap/hdmap_util.h"

namespace apollo {
/**
 * @brief Apollo外层命名空间
 */
namespace hdmap {
/**
 * @brief hdmap模块命名空间
 *
 * hdmap = High Definition Map（高精地图）
 */

/**
 * @struct LaneWaypoint
 * @brief 车道航点结构
 *
 * 功能说明：
 * 表示车道上的一个航点
 * 包含车道指针、沿车道的距离s、横向偏移l
 *
 * 成员变量说明：
 * - lane：指向车道信息的常量指针
 * - s：沿车道中心的累计距离（米）
 * - l：相对于车道中心的横向偏移（米），左正右负
 *
 * C++语法说明：
 * - struct：结构体，默认public访问
 * - = default：默认构造函数
 * - CHECK_NOTNULL：宏，检查指针非空
 * - : lane(x), s(y) {}：构造函数初始化列表
 * - = nullptr：默认初始化为nullptr
 */
struct LaneWaypoint {
  /**
   * @brief 默认构造函数
   *
   * C++语法说明：
   * = default：使用编译器生成的默认实现
   */
  LaneWaypoint() = default;

  /**
   * @brief 构造函数（带车道和s）
   *
   * @param lane 车道指针
   * @param s 沿车道的距离
   *
   * C++语法说明：
   * - : lane(CHECK_NOTNULL(lane)), s(s)：
   *   初始化列表，直接初始化成员
   * - CHECK_NOTNULL(lane)：
   *   Apollo宏，检查lane非空，为空则终止程序
   */
  LaneWaypoint(LaneInfoConstPtr lane, const double s)
      : lane(CHECK_NOTNULL(lane)), s(s) {}

  /**
   * @brief 构造函数（带车道、s和l）
   *
   * @param lane 车道指针
   * @param s 沿车道的距离
   * @param l 横向偏移
   */
  LaneWaypoint(LaneInfoConstPtr lane, const double s, const double l)
      : lane(CHECK_NOTNULL(lane)), s(s), l(l) {}

  /**
   * @brief 车道指针
   *
   * C++语法说明：
   * - LaneInfoConstPtr：
   *   Apollo定义的常量车道信息指针类型
   *   shared_ptr<const LaneInfo>
   * - = nullptr：默认初始化为空指针
   */
  LaneInfoConstPtr lane = nullptr;

  /**
   * @brief 沿车道的累计距离（米）
   */
  double s = 0.0;

  /**
   * @brief 横向偏移（米），左正右负
   */
  double l = 0.0;

  /**
   * @brief 获取调试字符串
   *
   * @return std::string 格式化的调试信息
   */
  std::string DebugString() const;
};

/**
 * @brief 获取航点处的左侧边界类型
 *
 * @param waypoint 车道航点
 * @return LaneBoundaryType::Type 边界类型
 *
 * C++语法说明：
 * LaneBoundaryType::Type：
 *   命名空间::类型的方式访问枚举类型
 */
LaneBoundaryType::Type LeftBoundaryType(const LaneWaypoint& waypoint);

/**
 * @brief 获取航点处的右侧边界类型
 *
 * @param waypoint 车道航点
 * @return LaneBoundaryType::Type 边界类型
 */
LaneBoundaryType::Type RightBoundaryType(const LaneWaypoint& waypoint);

/**
 * @brief 获取航点左侧相邻车道航点
 *
 * @param waypoint 当前航点
 * @return LaneWaypoint 左侧相邻车道航点
 *   如果不存在，Waypoint.lane为nullptr
 *
 * C++语法说明：
 * 返回值包含lane指针为空的情况表示不存在
 */
LaneWaypoint LeftNeighborWaypoint(const LaneWaypoint& waypoint);

/**
 * @brief 获取航点右侧相邻车道航点
 *
 * @param waypoint 当前航点
 * @return LaneWaypoint 右侧相邻车道航点
 *   如果不存在，Waypoint.lane为nullptr
 */
LaneWaypoint RightNeighborWaypoint(const LaneWaypoint& waypoint);

/**
 * @struct LaneSegment
 * @brief 车道段结构
 *
 * 功能说明：
 * 表示车道上的一个连续段
 * 用于描述路由路径中的车道段
 *
 * C++语法说明：
 * - struct默认public访问
 * - 构造函数初始化列表
 */
struct LaneSegment {
  /**
   * @brief 默认构造函数
   */
  LaneSegment() = default;

  /**
   * @brief 构造函数
   *
   * @param lane 车道指针
   * @param start_s 起始s距离
   * @param end_s 结束s距离
   *
   * C++语法说明：
   * : lane(CHECK_NOTNULL(lane)), start_s(start_s), end_s(end_s)：
   * 初始化列表初始化所有成员
   */
  LaneSegment(LaneInfoConstPtr lane, const double start_s, const double end_s)
      : lane(CHECK_NOTNULL(lane)), start_s(start_s), end_s(end_s) {}

  /**
   * @brief 车道指针
   */
  LaneInfoConstPtr lane = nullptr;

  /**
   * @brief 起始s距离（米）
   */
  double start_s = 0.0;

  /**
   * @brief 结束s距离（米）
   */
  double end_s = 0.0;

  /**
   * @brief 计算车道段长度
   *
   * @return double 长度（米）
   *
   * C++语法说明：
   * - const后缀：承诺不修改成员变量
   * - return end_s - start_s：返回计算结果
   */
  double Length() const { return end_s - start_s; }

  /**
   * @brief 合并相邻的相同ID车道段
   *
   * @param segments 车道段向量指针
   *
   * 功能说明：
   * 如果相邻的车道段具有相同的lane id
   * 将它们合并成一个更长的段
   *
   * C++语法说明：
   * static函数：属于类而非对象
   * std::vector<LaneSegment>*：指向向量的指针作为输出参数
   */
  static void Join(std::vector<LaneSegment>* segments);

  /**
   * @brief 获取调试字符串
   */
  std::string DebugString() const;
};

/**
 * @struct PathOverlap
 * @brief 路径重叠区域结构
 *
 * 功能说明：
 * 表示路径与某个元素的重叠区域
 * 用于描述交通元素（信号灯、停车标志等）与路径的关系
 *
 * 成员变量：
 * - object_id：重叠对象ID（如信号灯ID）
 * - start_s：重叠起始s距离
 * - end_s：重叠结束s距离
 */
struct PathOverlap {
  /**
   * @brief 默认构造函数
   */
  PathOverlap() = default;

  /**
   * @brief 构造函数
   *
   * @param object_id 对象ID
   * @param start_s 起始s距离
   * @param end_s 结束s距离
   *
   * C++语法说明：
   * - std::string object_id：
   *   参数名与成员名相同
   *   使用std::move避免拷贝
   */
  PathOverlap(std::string object_id, const double start_s, const double end_s)
      : object_id(std::move(object_id)), start_s(start_s), end_s(end_s) {}

  /**
   * @brief 重叠对象ID
   */
  std::string object_id;

  /**
   * @brief 起始s距离
   */
  double start_s = 0.0;

  /**
   * @brief 结束s距离
   */
  double end_s = 0.0;

  /**
   * @brief 获取调试字符串
   */
  std::string DebugString() const;
};

/**
 * @class MapPathPoint
 * @brief 地图路径点类
 *
 * 功能说明：
 * 继承自Vec2d的地图路径点
 * 包含位置、朝向、所属车道等信息
 *
 * 继承关系：
 * MapPathPoint -> Vec2d
 *
 * C++语法说明：
 * - : public common::math::Vec2d：
 *   公有继承自Vec2d类
 *   MapPathPoint is-a Vec2d
 * - 使用using引入父类成员
 */
class MapPathPoint : public common::math::Vec2d {
 public:
  /**
   * @brief 默认构造函数
   */
  MapPathPoint() = default;

  /**
   * @brief 构造函数（带点和朝向）
   *
   * @param point 2D点
   * @param heading 朝向角（弧度）
   *
   * C++语法说明：
   * - : Vec2d(point.x(), point.y())：
   *   初始化列表调用父类构造函数
   * - Vec2d(point.x(), point.y())：
   *   调用Vec2d的两个参数构造函数
   */
  MapPathPoint(const common::math::Vec2d& point, double heading)
      : Vec2d(point.x(), point.y()), heading_(heading) {}

  /**
   * @brief 构造函数（带点、朝向和单个航点）
   *
   * @param point 2D点
   * @param heading 朝向角
   * @param lane_waypoint 车道航点
   *
   * C++语法说明：
   * - std::move(lane_waypoint)：
   *   移动语义，将lane_waypoint的所有权转移
   *   避免拷贝，提高效率
   * - emplace_back(...)：
   *   在vector末尾就地构造元素
   *   避免先构造临时对象再拷贝
   */
  MapPathPoint(const common::math::Vec2d& point, double heading,
               LaneWaypoint lane_waypoint)
      : Vec2d(point.x(), point.y()), heading_(heading) {
    lane_waypoints_.emplace_back(std::move(lane_waypoint));
  }

  /**
   * @brief 构造函数（带点和多个航点）
   *
   * @param point 2D点
   * @param heading 朝向角
   * @param lane_waypoints 车道航点向量
   */
  MapPathPoint(const common::math::Vec2d& point, double heading,
               std::vector<LaneWaypoint> lane_waypoints)
      : Vec2d(point.x(), point.y()),
        heading_(heading),
        lane_waypoints_(std::move(lane_waypoints)) {}

  /**
   * @brief 获取朝向角
   *
   * @return double 朝向角（弧度）
   *
   * C++语法说明：
   * const后缀：承诺不修改成员变量
   */
  double heading() const { return heading_; }

  /**
   * @brief 设置朝向角
   *
   * @param heading 朝向角
   */
  void set_heading(const double heading) { heading_ = heading; }

  /**
   * @brief 获取车道航点列表
   *
   * @return const std::vector<LaneWaypoint>& 车道航点引用
   */
  const std::vector<LaneWaypoint>& lane_waypoints() const {
    return lane_waypoints_;
  }

  /**
   * @brief 添加单个车道航点
   *
   * @param lane_waypoint 车道航点
   *
   * C++语法说明：
   * emplace_back(std::move(...))：
   * 移动语义添加到vector
   */
  void add_lane_waypoint(LaneWaypoint lane_waypoint) {
    lane_waypoints_.emplace_back(std::move(lane_waypoint));
  }

  /**
   * @brief 添加多个车道航点
   *
   * @param lane_waypoints 车道航点向量
   */
  void add_lane_waypoints(const std::vector<LaneWaypoint>& lane_waypoints) {
    /**
     * @brief insert插入多个元素
     *
     * C++语法说明：
     * lane_waypoints_.end()：目标位置（末尾）
     * lane_waypoints.begin(), end()：源范围
     */
    lane_waypoints_.insert(lane_waypoints_.end(), lane_waypoints.begin(),
                           lane_waypoints.end());
  }

  /**
   * @brief 清空车道航点
   */
  void clear_lane_waypoints() { lane_waypoints_.clear(); }

  /**
   * @brief 移除重复路径点
   *
   * @param points 路径点向量指针
   *
   * C++语法说明：
   * static函数：属于类，可以通过ClassName::调用
   */
  static void RemoveDuplicates(std::vector<MapPathPoint>* points);

  /**
   * @brief 从车道段获取路径点
   *
   * @param segment 车道段
   * @return std::vector<MapPathPoint> 路径点列表
   */
  static std::vector<MapPathPoint> GetPointsFromSegment(
      const LaneSegment& segment);

  /**
   * @brief 从车道获取路径点
   *
   * @param lane 车道指针
   * @param start_s 起始s
   * @param end_s 结束s
   * @return std::vector<MapPathPoint> 路径点列表
   */
  static std::vector<MapPathPoint> GetPointsFromLane(LaneInfoConstPtr lane,
                                                     const double start_s,
                                                     const double end_s);

  /**
   * @brief 获取调试字符串
   */
  std::string DebugString() const;

 protected:
  /**
   * @brief 朝向角（弧度）
   *
   * C++语法说明：
   * protected成员：派生类可以访问
   */
  double heading_ = 0.0;

  /**
   * @brief 车道航点列表
   */
  std::vector<LaneWaypoint> lane_waypoints_;
};

/**
 * @class PathApproximation
 * @brief 路径近似类
 *
 * 功能说明：
 * 对路径进行近似表示以减少计算量
 * 使用Douglas-Peucker算法或类似方法
 * 在可接受的误差范围内简化路径
 *
 * 设计特点：
 * - 支持最大误差控制
 * - 保持路径段信息
 * - 提供快速投影查询
 */
class PathApproximation {
 public:
  /**
   * @brief 默认构造函数
   */
  PathApproximation() = default;

  /**
   * @brief 构造函数
   *
   * @param path 原始路径
   * @param max_error 最大允许误差
   *
   * C++语法说明：
   * - : max_error_(max_error), max_sqr_error_(...)：
   *   初始化列表初始化成员
   * - max_error * max_error：
   *   计算误差的平方，避免后续重复计算
   */
  PathApproximation(const Path& path, const double max_error)
      : max_error_(max_error), max_sqr_error_(max_error * max_error) {
    Init(path);
  }

  /**
   * @brief 获取最大误差
   */
  double max_error() const { return max_error_; }

  /**
   * @brief 获取原始点索引列表
   */
  const std::vector<int>& original_ids() const { return original_ids_; }

  /**
   * @brief 获取线段列表
   */
  const std::vector<common::math::LineSegment2d>& segments() const {
    return segments_;
  }

  /**
   * @brief 获取点在路径上的投影
   *
   * @param path 路径引用
   * @param point 查询点
   * @param accumulate_s 输出：累计s距离
   * @param lateral 输出：横向偏移
   * @param distance 输出：距离
   * @return bool 是否成功
   */
  bool GetProjection(const Path& path, const common::math::Vec2d& point,
                     double* accumulate_s, double* lateral,
                     double* distance) const;

  /**
   * @brief 检查是否与包围盒重叠
   *
   * @param path 路径引用
   * @param box 包围盒
   * @param width 宽度
   * @return bool 是否重叠
   */
  bool OverlapWith(const Path& path, const common::math::Box2d& box,
                   double width) const;

 protected:
  /**
   * @brief 初始化近似路径
   */
  void Init(const Path& path);

  /**
   * @brief 检查是否在最大误差内
   */
  bool is_within_max_error(const Path& path, const int s, const int t);

  /**
   * @brief 计算最大误差
   */
  double compute_max_error(const Path& path, const int s, const int t);

  /**
   * @brief 初始化稀释路径
   */
  void InitDilute(const Path& path);

  /**
   * @brief 初始化投影信息
   */
  void InitProjections(const Path& path);

 protected:
  /**
   * @brief 最大允许误差（米）
   */
  double max_error_ = 0;

  /**
   * @brief 最大误差的平方
   *
   * C++语法说明：
   * 预计算平方，避免运行时重复计算
   */
  double max_sqr_error_ = 0;

  /**
   * @brief 路径点数量
   */
  int num_points_ = 0;

  /**
   * @brief 原始点索引列表
   *
   * 近似后的点对应原始路径点的索引
   */
  std::vector<int> original_ids_;

  /**
   * @brief 线段列表
   */
  std::vector<common::math::LineSegment2d> segments_;

  /**
   * @brief 每段最大误差列表
   */
  std::vector<double> max_error_per_segment_;

  /**
   * @brief 投影列表
   */
  std::vector<double> projections_;

  /**
   * @brief 最大投影值
   */
  double max_projection_ = 0;

  /**
   * @brief 投影采样点数量
   */
  int num_projection_samples_ = 0;

  /**
   * @brief 原始点在线段上的投影列表
   */
  std::vector<double> original_projections_;

  /**
   * @brief max_original_projections_to_left_[i] = max(p[0], p[1], ... p[i])
   */
  std::vector<double> max_original_projections_to_left_;

  /**
   * @brief min_original_projections_to_right_[i] = min(p[i], p[i+1], ... p[size-1])
   */
  std::vector<double> min_original_projections_to_right_;

  /**
   * @brief 采样的最大原始投影索引
   */
  std::vector<int> sampled_max_original_projections_to_left_;
};

/**
 * @class InterpolatedIndex
 * @brief 插值索引类
 *
 * 功能说明：
 * 表示路径上的插值索引
 * 用于在路径上进行线性插值
 *
 * 成员变量说明：
 * - id：线段索引
 * - offset：在线段内的偏移量
 */
class InterpolatedIndex {
 public:
  /**
   * @brief 构造函数
   *
   * @param id 线段索引
   * @param offset 偏移量
   */
  InterpolatedIndex(int id, double offset) : id(id), offset(offset) {}

  /**
   * @brief 线段索引
   */
  int id = 0;

  /**
   * @brief 偏移量
   */
  double offset = 0.0;
};

/**
 * @class Path
 * @brief 地图路径类
 *
 * 功能说明：
 * 表示地图上的完整路径
 * 包含路径点、车道段、累计距离、线段等信息
 * 提供投影查询、宽度查询、重叠区域查询等功能
 *
 * 设计特点：
 * 1. 支持多种构造方式（值拷贝、移动语义）
 * 2. 支持路径近似优化
 * 3. 支持多种重叠区域类型
 * 4. 提供丰富的查询接口
 *
 * C++语法说明：
 * - explicit关键字：防止隐式类型转换
 * - std::vector<MapPathPoint>&&：右值引用，移动语义
 */
class Path {
 public:
  /**
   * @brief 默认构造函数
   */
  Path() = default;

  /**
   * @brief 用路径点构造（值拷贝）
   *
   * @param path_points 路径点向量
   *
   * C++语法说明：
   * explicit：防止隐式转换
   * const std::vector<MapPathPoint>&：
   *   常量引用，避免拷贝
   */
  explicit Path(const std::vector<MapPathPoint>& path_points);

  /**
   * @brief 用路径点构造（移动语义）
   *
   * @param path_points 路径点向量
   *
   * C++语法说明：
   * std::vector<MapPathPoint>&&：
   *   右值引用，用于移动语义
   *   避免拷贝，提高效率
   */
  explicit Path(std::vector<MapPathPoint>&& path_points);

  /**
   * @brief 用车道段构造（移动语义）
   */
  explicit Path(std::vector<LaneSegment>&& path_points);

  /**
   * @brief 用车道段构造（值拷贝）
   */
  explicit Path(const std::vector<LaneSegment>& path_points);

  /**
   * @brief 用路径点和车道段构造（值拷贝）
   */
  Path(const std::vector<MapPathPoint>& path_points,
       const std::vector<LaneSegment>& lane_segments);

  /**
   * @brief 用路径点和车道段构造（移动语义）
   */
  Path(std::vector<MapPathPoint>&& path_points,
       std::vector<LaneSegment>&& lane_segments);

  /**
   * @brief 用路径点、车道段和近似误差构造
   */
  Path(const std::vector<MapPathPoint>& path_points,
       const std::vector<LaneSegment>& lane_segments,
       const double max_approximation_error);

  /**
   * @brief 用路径点、车道段和近似误差构造（移动语义）
   */
  Path(std::vector<MapPathPoint>&& path_points,
       std::vector<LaneSegment>&& lane_segments,
       const double max_approximation_error);

  /**
   * @brief 根据插值索引获取平滑点
   *
   * @param index 插值索引
   * @return MapPathPoint 平滑路径点
   */
  MapPathPoint GetSmoothPoint(const InterpolatedIndex& index) const;

  /**
   * @brief 根据累计s距离获取平滑点
   *
   * @param s 累计距离
   * @return MapPathPoint 平滑路径点
   */
  MapPathPoint GetSmoothPoint(double s) const;

  /**
   * @brief 从索引获取累计s距离
   *
   * @param index 插值索引
   * @return double 累计s距离
   */
  double GetSFromIndex(const InterpolatedIndex& index) const;

  /**
   * @brief 从累计s距离获取插值索引
   *
   * @param s 累计距离
   * @return InterpolatedIndex 插值索引
   */
  InterpolatedIndex GetIndexFromS(double s) const;

  /**
   * @brief 从累计s距离获取车道索引
   *
   * @param s 累计距离
   * @return InterpolatedIndex 车道索引
   */
  InterpolatedIndex GetLaneIndexFromS(double s) const;

  /**
   * @brief 获取s范围内的车道段
   *
   * @param start_s 起始s
   * @param end_s 结束s
   * @return std::vector<hdmap::LaneSegment> 车道段列表
   */
  std::vector<hdmap::LaneSegment> GetLaneSegments(const double start_s,
                                                  const double end_s) const;

  /**
   * @brief 获取最近点（基础版本）
   */
  bool GetNearestPoint(const common::math::Vec2d& point, double* accumulate_s,
                       double* lateral) const;

  /**
   * @brief 获取最近点（带距离）
   */
  bool GetNearestPoint(const common::math::Vec2d& point, double* accumulate_s,
                       double* lateral, double* distance) const;

  /**
   * @brief 使用启发式参数获取投影
   */
  bool GetProjectionWithHueristicParams(const common::math::Vec2d& point,
                                        const double hueristic_start_s,
                                        const double hueristic_end_s,
                                        double* accumulate_s, double* lateral,
                                        double* min_distance) const;

  /**
   * @brief 获取投影（基础版本）
   */
  bool GetProjection(const common::math::Vec2d& point, double* accumulate_s,
                     double* lateral) const;

  /**
   * @brief 获取投影（带朝向）
   */
  bool GetProjection(const double heading, const common::math::Vec2d& point,
                     double* accumulate_s, double* lateral) const;

  /**
   * @brief 获取投影（带预热起点）
   */
  bool GetProjectionWithWarmStartS(const common::math::Vec2d& point,
                                   double* accumulate_s, double* lateral) const;

  /**
   * @brief 获取投影（带距离输出）
   */
  bool GetProjection(const common::math::Vec2d& point, double* accumulate_s,
                     double* lateral, double* distance) const;

  /**
   * @brief 获取投影（带朝向和距离）
   */
  bool GetProjection(const common::math::Vec2d& point, const double heading,
                     double* accumulate_s, double* lateral,
                     double* distance) const;

  /**
   * @brief 获取沿路径的朝向
   */
  bool GetHeadingAlongPath(const common::math::Vec2d& point,
                           double* heading) const;

  /**
   * @brief 获取路径点数量
   */
  int num_points() const { return num_points_; }

  /**
   * @brief 获取线段数量
   */
  int num_segments() const { return num_segments_; }

  /**
   * @brief 获取路径点列表
   */
  const std::vector<MapPathPoint>& path_points() const { return path_points_; }

  /**
   * @brief 获取车道段列表
   */
  const std::vector<LaneSegment>& lane_segments() const {
    return lane_segments_;
  }

  /**
   * @brief 获取到下一点的车道段列表
   */
  const std::vector<LaneSegment>& lane_segments_to_next_point() const {
    return lane_segments_to_next_point_;
  }

  /**
   * @brief 获取单位方向向量列表
   */
  const std::vector<common::math::Vec2d>& unit_directions() const {
    return unit_directions_;
  }

  /**
   * @brief 获取累计s距离列表
   */
  const std::vector<double>& accumulated_s() const { return accumulated_s_; }

  /**
   * @brief 获取线段列表
   */
  const std::vector<common::math::LineSegment2d>& segments() const {
    return segments_;
  }

  /**
   * @brief 获取路径近似对象
   */
  const PathApproximation* approximation() const { return &approximation_; }

  /**
   * @brief 获取路径总长度
   */
  double length() const { return length_; }

  /**
   * @brief 获取下一车道重叠区域
   */
  const PathOverlap* NextLaneOverlap(double s) const;

  /**
   * @brief 获取车道重叠区域列表
   */
  const std::vector<PathOverlap>& lane_overlaps() const {
    return lane_overlaps_;
  }

  /**
   * @brief 获取信号灯重叠区域列表
   */
  const std::vector<PathOverlap>& signal_overlaps() const {
    return signal_overlaps_;
  }

  /**
   * @brief 获取让行标志重叠区域列表
   */
  const std::vector<PathOverlap>& yield_sign_overlaps() const {
    return yield_sign_overlaps_;
  }

  /**
   * @brief 获取停车标志重叠区域列表
   */
  const std::vector<PathOverlap>& stop_sign_overlaps() const {
    return stop_sign_overlaps_;
  }

  /**
   * @brief 获取人行横道重叠区域列表
   */
  const std::vector<PathOverlap>& crosswalk_overlaps() const {
    return crosswalk_overlaps_;
  }

  /**
   * @brief 获取路口重叠区域列表
   */
  const std::vector<PathOverlap>& junction_overlaps() const {
    return junction_overlaps_;
  }

  /**
   * @brief 获取PNC路口重叠区域列表
   */
  const std::vector<PathOverlap>& pnc_junction_overlaps() const {
    return pnc_junction_overlaps_;
  }

  /**
   * @brief 获取清除区域重叠区域列表
   */
  const std::vector<PathOverlap>& clear_area_overlaps() const {
    return clear_area_overlaps_;
  }

  /**
   * @brief 获取减速带重叠区域列表
   */
  const std::vector<PathOverlap>& speed_bump_overlaps() const {
    return speed_bump_overlaps_;
  }

  /**
   * @brief 获取停车位重叠区域列表
   */
  const std::vector<PathOverlap>& parking_space_overlaps() const {
    return parking_space_overlaps_;
  }

  /**
   * @brief 获取死胡同重叠区域列表
   */
  const std::vector<PathOverlap>& dead_end_overlaps() const {
    return dead_end_overlaps_;
  }

  /**
   * @brief 获取区域重叠列表
   */
  const std::vector<PathOverlap>& area_overlaps() const {
    return area_overlaps_;
  }

  /**
   * @brief 获取车道左侧宽度
   */
  double GetLaneLeftWidth(const double s) const;

  /**
   * @brief 获取车道右侧宽度
   */
  double GetLaneRightWidth(const double s) const;

  /**
   * @brief 获取车道宽度
   *
   * @param s 累计距离
   * @param lane_left_width 输出：左侧宽度
   * @param lane_right_width 输出：右侧宽度
   * @return bool 是否成功
   */
  double GetLaneWidth(const double s, double* lane_left_width,
                      double* lane_right_width) const;

  /**
   * @brief 获取道路左侧宽度
   */
  double GetRoadLeftWidth(const double s) const;

  /**
   * @brief 获取道路右侧宽度
   */
  double GetRoadRightWidth(const double s) const;

  /**
   * @brief 获取道路宽度
   */
  bool GetRoadWidth(const double s, double* road_left_width,
                    double* road_ight_width) const;

  /**
   * @brief 检查点是否在路径上
   */
  bool IsOnPath(const common::math::Vec2d& point) const;

  /**
   * @brief 检查是否与包围盒重叠
   */
  bool OverlapWith(const common::math::Box2d& box, double width) const;

  /**
   * @brief 获取调试字符串
   */
  std::string DebugString() const;

 protected:
  /**
   * @brief 初始化路径
   */
  void Init();

  /**
   * @brief 初始化路径点
   */
  void InitPoints();

  /**
   * @brief 初始化车道段
   */
  void InitLaneSegments();

  /**
   * @brief 初始化宽度信息
   */
  void InitWidth();

  /**
   * @brief 初始化点索引
   */
  void InitPointIndex();

  /**
   * @brief 初始化重叠区域
   */
  void InitOverlaps();

  /**
   * @brief 从采样列表获取值
   *
   * @param samples 采样列表
   * @param s 累计距离
   * @return double 插值结果
   */
  double GetSample(const std::vector<double>& samples, const double s) const;

  /**
   * @brief 获取重叠区域函数类型
   *
   * C++语法说明：
   * - std::function<...>：
   *   函数包装器类型
   *   可以存储、拷贝和调用任何可调用对象
   * - const std::vector<OverlapInfoConstPtr>&(const LaneInfo&)：
   *   函数签名：接收LaneInfo，返回OverlapInfoConstPtr向量引用
   */
  using GetOverlapFromLaneFunc =
      std::function<const std::vector<OverlapInfoConstPtr>&(const LaneInfo&)>;

  /**
   * @brief 获取所有重叠区域
   */
  void GetAllOverlaps(GetOverlapFromLaneFunc GetOverlaps_from_lane,
                      std::vector<PathOverlap>* const overlaps) const;

 protected:
  /**
   * @brief 路径点数量
   */
  int num_points_ = 0;

  /**
   * @brief 线段数量
   */
  int num_segments_ = 0;

  /**
   * @brief 路径点列表
   */
  std::vector<MapPathPoint> path_points_;

  /**
   * @brief 车道段列表
   */
  std::vector<LaneSegment> lane_segments_;

  /**
   * @brief 车道累计s距离
   */
  std::vector<double> lane_accumulated_s_;

  /**
   * @brief 到下一点的车道段列表
   */
  std::vector<LaneSegment> lane_segments_to_next_point_;

  /**
   * @brief 单位方向向量列表
   */
  std::vector<common::math::Vec2d> unit_directions_;

  /**
   * @brief 路径总长度
   */
  double length_ = 0.0;

  /**
   * @brief 累计s距离列表
   */
  std::vector<double> accumulated_s_;

  /**
   * @brief 线段列表
   */
  std::vector<common::math::LineSegment2d> segments_;

  /**
   * @brief 是否使用路径近似
   */
  bool use_path_approximation_ = false;

  /**
   * @brief 路径近似对象
   */
  PathApproximation approximation_;

  /**
   * @brief 采样点数量
   */
  int num_sample_points_ = 0;

  /**
   * @brief 车道左侧宽度列表
   */
  std::vector<double> lane_left_width_;

  /**
   * @brief 车道右侧宽度列表
   */
  std::vector<double> lane_right_width_;

  /**
   * @brief 道路左侧宽度列表
   */
  std::vector<double> road_left_width_;

  /**
   * @brief 道路右侧宽度列表
   */
  std::vector<double> road_right_width_;

  /**
   * @brief 最后点索引列表
   */
  std::vector<int> last_point_index_;

  /**
   * @brief 车道重叠区域列表
   */
  std::vector<PathOverlap> lane_overlaps_;

  /**
   * @brief 信号灯重叠区域列表
   */
  std::vector<PathOverlap> signal_overlaps_;

  /**
   * @brief 让行标志重叠区域列表
   */
  std::vector<PathOverlap> yield_sign_overlaps_;

  /**
   * @brief 停车标志重叠区域列表
   */
  std::vector<PathOverlap> stop_sign_overlaps_;

  /**
   * @brief 人行横道重叠区域列表
   */
  std::vector<PathOverlap> crosswalk_overlaps_;

  /**
   * @brief 停车位重叠区域列表
   */
  std::vector<PathOverlap> parking_space_overlaps_;

  /**
   * @brief 死胡同重叠区域列表
   */
  std::vector<PathOverlap> dead_end_overlaps_;

  /**
   * @brief 路口重叠区域列表
   */
  std::vector<PathOverlap> junction_overlaps_;

  /**
   * @brief PNC路口重叠区域列表
   */
  std::vector<PathOverlap> pnc_junction_overlaps_;

  /**
   * @brief 清除区域重叠区域列表
   */
  std::vector<PathOverlap> clear_area_overlaps_;

  /**
   * @brief 减速带重叠区域列表
   */
  std::vector<PathOverlap> speed_bump_overlaps_;

  /**
   * @brief 区域重叠列表
   */
  std::vector<PathOverlap> area_overlaps_;

 private:
  /**
   * @brief 查找最近的线段索引
   *
   * @param left_index 搜索起始索引
   * @param right_index 搜索结束索引
   * @param target_s 目标s距离
   * @param mid_index 输出：中间索引
   *
   * 功能说明：
   * 二分查找找到target_s所在的线段索引
   */
  void FindIndex(int left_index, int right_index, double target_s,
                 int* mid_index) const;
};

/**
 * @brief 命名空间结束标记
 *
 * C++语法说明：
 * 两层命名空间的闭合注释
 */
}  // namespace hdmap
}  // namespace apollo
