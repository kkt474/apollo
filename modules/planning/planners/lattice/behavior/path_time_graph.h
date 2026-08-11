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
 * @file path_time_graph.h
 * @brief 路径-时间图类声明头文件
 *
 * 功能说明：
 * 路径-时间图(PathTimeGraph)用于管理ST图中的障碍物
 * ST图：S表示沿路径的距离，T表示时间
 * 通过ST边界(STBoundary)描述自车与障碍物的时空关系
 *
 * 主要功能：
 * 1. 构建所有障碍物的ST边界
 * 2. 查询特定障碍物的ST边界
 * 3. 获取路径阻塞区间
 * 4. 计算横向边界
 * 5. 获取障碍物周围点
 *
 * 设计特点：
 * - 使用map存储障碍物ID到ST边界的映射
 * - 使用vector存储所有ST边界
 * - 支持静态和动态障碍物
 * - 提供多种查询接口
 *
 * C++语法说明：
 * - #pragma once：编译指示符，防止头文件重复包含
 * - std::array<double, 3>：固定大小数组
 * - std::unordered_map：基于哈希表的关联容器
 * - std::pair：键值对容器
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
 * - #include <unordered_map>：基于哈希表的关联容器，O(1)查找
 * - #include <utility>：工具类，包括std::pair
 * - #include <vector>：动态数组容器
 */
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

/**
 * @brief Apollo几何消息头文件
 *
 * C++语法说明：
 * geometry.pb.h由proto文件编译生成
 * 包含Point、Vector等几何类型定义
 */
#include "modules/common_msgs/basic_msgs/geometry.pb.h"

/**
 * @brief Apollo 2D数学头文件
 *
 * C++语法说明：
 * - polygon2d.h：2D多边形类
 * - Vec2d：2D向量类
 */
#include "modules/common/math/polygon2d.h"

/**
 * @brief Apollo规划基础头文件
 *
 * C++语法说明：
 * - frame.h：规划帧数据类Frame
 * - obstacle.h：障碍物类Obstacle
 * - reference_line_info.h：参考线信息类ReferenceLineInfo
 */
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/obstacle.h"
#include "modules/planning/planning_base/common/reference_line_info.h"

/**
 * @brief Apollo速度规划头文件
 *
 * C++语法说明：
 * - st_boundary.h：ST边界类STBoundary
 * - st_point.h：ST点类STPoint
 */
#include "modules/planning/planning_base/common/speed/st_boundary.h"
#include "modules/planning/planning_base/common/speed/st_point.h"

/**
 * @brief Apollo参考线头文件
 *
 * C++语法说明：
 * reference_line.h：参考线类ReferenceLine
 */
#include "modules/planning/planning_base/reference_line/reference_line.h"

namespace apollo {
/**
 * @brief Apollo外层命名空间
 */
namespace planning {

/**
 * @class PathTimeGraph
 * @brief 路径-时间图类
 *
 * 功能说明：
 * PathTimeGraph管理ST图中的所有障碍物
 * ST图是Lattice规划器的核心数据结构
 * 用于描述自车与障碍物在时空平面上的相对关系
 *
 * 使用场景：
 * - Lattice规划器使用此类管理动态障碍物
 * - 查询特定障碍物的ST边界
 * - 获取路径阻塞区间用于路径规划
 *
 * 核心概念：
 * - ST边界：描述障碍物在时空平面上的占用区域
 * - 上边界(s_upper)：自车需要保持在此距离之外
 * - 下边界(s_lower)：自车不能超过此距离
 *
 * C++语法说明：
 * - class PathTimeGraph：
 *   类声明，默认访问限定符为private
 */
class PathTimeGraph {
 public:
  /**
   * @brief 构造函数
   *
   * @param obstacles 障碍物指针向量
   * @param discretized_ref_points 离散化的参考线点
   * @param ptr_reference_line_info 参考线信息指针
   * @param s_start S方向起始位置
   * @param s_end S方向结束位置
   * @param t_start T方向起始时间
   * @param t_end T方向结束时间
   * @param init_d 初始横向状态[d, dd, ddd]
   *
   * 功能说明：
   * 1. 初始化时间范围和路径范围
   * 2. 设置参考线信息
   * 3. 调用SetupObstacles构建所有障碍物的ST边界
   *
   * C++语法说明：
   * - const std::vector<const Obstacle*>&：
   *   常量引用，避免拷贝
   *   vector存储指向const Obstacle的指针
   *
   * - const std::vector<common::PathPoint>&：
   *   常量引用，离散化的参考线点
   *
   * - const ReferenceLineInfo*：
   *   指向参考线信息的指针
   *
   * - const double s_start, const double s_end：
   *   const值参数，承诺不修改
   *
   * - const std::array<double, 3>& init_d：
   *   固定大小数组的常量引用
   *   init_d[0]=d位置, init_d[1]=dd速度, init_d[2]=ddd加速度
   */
  PathTimeGraph(const std::vector<const Obstacle*>& obstacles,
                const std::vector<common::PathPoint>& discretized_ref_points,
                const ReferenceLineInfo* ptr_reference_line_info,
                const double s_start, const double s_end, const double t_start,
                const double t_end, const std::array<double, 3>& init_d);

  /**
   * @brief 获取所有ST边界
   *
   * @return const std::vector<STBoundary>& 所有ST边界的引用
   *
   * 功能说明：
   * 返回图中所有障碍物的ST边界
   *
   * C++语法说明：
   * - const std::vector<STBoundary>&：
   *   常量引用返回，避免拷贝
   *   const保证返回内容不可变
   */
  const std::vector<STBoundary>& GetPathTimeObstacles() const;

  /**
   * @brief 根据ID获取特定障碍物的ST边界
   *
   * @param obstacle_id 障碍物ID
   * @param path_time_obstacle 输出参数：ST边界
   * @return bool 是否成功获取
   *
   * C++语法说明：
   * - STBoundary* path_time_obstacle：
   *   指针作为输出参数
   *   函数内部修改其指向的内容
   */
  bool GetPathTimeObstacle(const std::string& obstacle_id,
                           STBoundary* path_time_obstacle);

  /**
   * @brief 获取指定时刻的路径阻塞区间
   *
   * @param t 时刻
   * @return std::vector<std::pair<double, double>> 阻塞区间列表
   *   每对(s_min, s_max)表示该时刻被阻塞的路径区间
   *
   * 功能说明：
   * 查询在给定时刻t，哪些路径区间被障碍物占用
   *
   * C++语法说明：
   * - std::vector<std::pair<double, double>>：
   *   向量存储键值对
   *   pair<double, double>表示(s_min, s_max)区间
   */
  std::vector<std::pair<double, double>> GetPathBlockingIntervals(
      const double t) const;

  /**
   * @brief 获取时间范围内的路径阻塞区间
   *
   * @param t_start 起始时刻
   * @param t_end 结束时刻
   * @param t_resolution 时间分辨率
   * @return std::vector<std::vector<std::pair<double, double>>> 阻塞区间
   *
   * 功能说明：
   * 查询时间范围内每个采样时刻的路径阻塞区间
   *
   * C++语法说明：
   * - std::vector<std::vector<...>>：
   *   外层vector：不同采样时刻
   *   内层vector：该时刻的多个阻塞区间
   */
  std::vector<std::vector<std::pair<double, double>>> GetPathBlockingIntervals(
      const double t_start, const double t_end, const double t_resolution);

  /**
   * @brief 获取路径范围
   *
   * @return std::pair<double, double> (s_min, s_max)
   *
   * C++语法说明：
   * std::pair<double, double>：存储两个double值
   */
  std::pair<double, double> get_path_range() const;

  /**
   * @brief 获取时间范围
   *
   * @return std::pair<double, double> (t_min, t_max)
   */
  std::pair<double, double> get_time_range() const;

  /**
   * @brief 获取障碍物周围点
   *
   * @param obstacle_id 障碍物ID
   * @param s_dist S方向采样距离
   * @param t_density T方向采样密度
   * @return std::vector<STPoint> 周围ST点列表
   *
   * 功能说明：
   * 获取障碍物ST边界周围的采样点
   * 用于ST图的可视化或分析
   *
   * C++语法说明：
   * - const std::string& obstacle_id：
   *   常量引用，障碍物ID
   */
  std::vector<STPoint> GetObstacleSurroundingPoints(
      const std::string& obstacle_id, const double s_dist,
      const double t_density) const;

  /**
   * @brief 检查障碍物是否在图中
   *
   * @param obstacle_id 障碍物ID
   * @return bool 是否在图中
   */
  bool IsObstacleInGraph(const std::string& obstacle_id);

  /**
   * @brief 获取横向边界
   *
   * @param s_start 起始S位置
   * @param s_end 结束S位置
   * @param s_resolution S方向分辨率
   * @return std::vector<std::pair<double, double>> 横向边界列表
   *
   * 功能说明：
   * 计算路径区间内的横向边界
   * 考虑所有障碍物的影响
   */
  std::vector<std::pair<double, double>> GetLateralBounds(
      const double s_start, const double s_end, const double s_resolution);

 private:
  /**
   * @brief 构建障碍物ST边界
   *
   * @param obstacles 障碍物列表
   * @param discretized_ref_points 离散化参考线点
   *
   * 功能说明：
   * 遍历所有障碍物
   * 根据障碍物类型调用SetStaticObstacle或SetDynamicObstacle
   */
  void SetupObstacles(
      const std::vector<const Obstacle*>& obstacles,
      const std::vector<common::PathPoint>& discretized_ref_points);

  /**
   * @brief 计算障碍物SL边界
   *
   * @param vertices 障碍物顶点多边形
   * @param discretized_ref_points 离散化参考线点
   * @return SLBoundary SL边界
   *
   * 功能说明：
   * 将障碍物的笛卡尔坐标顶点转换为SL坐标
   *
   * C++语法说明：
   * - const std::vector<common::math::Vec2d>&：
   *   Vec2d的常量引用向量
   *   Vec2d是Apollo的2D向量类
   */
  SLBoundary ComputeObstacleBoundary(
      const std::vector<common::math::Vec2d>& vertices,
      const std::vector<common::PathPoint>& discretized_ref_points) const;

  /**
   * @brief 设置ST点
   *
   * @param obstacle_id 障碍物ID
   * @param s S坐标
   * @param t T坐标
   * @return STPoint ST点
   */
  STPoint SetPathTimePoint(const std::string& obstacle_id, const double s,
                           const double t) const;

  /**
   * @brief 设置静态障碍物的ST边界
   *
   * @param obstacle 障碍物指针
   * @param discretized_ref_points 离散化参考线点
   *
   * 功能说明：
   * 静态障碍物在整个时间范围内位置不变
   * 直接创建上下边界相同的ST边界
   */
  void SetStaticObstacle(
      const Obstacle* obstacle,
      const std::vector<common::PathPoint>& discretized_ref_points);

  /**
   * @brief 设置动态障碍物的ST边界
   *
   * @param obstacle 障碍物指针
   * @param discretized_ref_points 离散化参考线点
   *
   * 功能说明：
   * 动态障碍物有预测轨迹
   * 根据轨迹点的位置随时间变化构建ST边界
   */
  void SetDynamicObstacle(
      const Obstacle* obstacle,
      const std::vector<common::PathPoint>& discretized_ref_points);

  /**
   * @brief 根据障碍物更新横向边界
   *
   * @param sl_boundary 障碍物SL边界
   * @param discretized_path 离散化路径
   * @param s_start 起始S
   * @param s_end 结束S
   * @param bounds 输出参数：横向边界
   *
   * C++语法说明：
   * - std::vector<std::pair<double, double>>* const bounds：
   *   指向vector的常量指针
   *   指针本身不可变，但可以修改指针指向的内容
   */
  void UpdateLateralBoundsByObstacle(
      const SLBoundary& sl_boundary,
      const std::vector<double>& discretized_path, const double s_start,
      const double s_end, std::vector<std::pair<double, double>>* const bounds);

 private:
  /**
   * @brief 时间范围
   *
   * C++语法说明：
   * std::pair<double, double>：(t_min, t_max)
   * 下划线后缀是Apollo命名约定，表示成员变量
   */
  std::pair<double, double> time_range_;

  /**
   * @brief 路径范围
   *
   * C++语法说明：
   * std::pair<double, double>：(s_min, s_max)
   */
  std::pair<double, double> path_range_;

  /**
   * @brief 参考线信息指针
   *
   * C++语法说明：
   * - const ReferenceLineInfo*：
   *   指向常量ReferenceLineInfo的指针
   *   不拥有对象所有权
   *   ptr_前缀表示指针类型成员变量
   */
  const ReferenceLineInfo* ptr_reference_line_info_;

  /**
   * @brief 初始横向状态
   *
   * C++语法说明：
   * - std::array<double, 3>：
   *   固定大小数组，3个double元素
   *   init_d_[0]=d位置, init_d_[1]=dd速度, init_d_[2]=ddd加速度
   */
  std::array<double, 3> init_d_;

  /**
   * @brief 障碍物ID到ST边界的映射
   *
   * C++语法说明：
   * - std::unordered_map<std::string, STBoundary>：
   *   键：障碍物ID (string)
   *   值：STBoundary对象
   *   unordered_map使用哈希表，O(1)查找
   */
  std::unordered_map<std::string, STBoundary> path_time_obstacle_map_;

  /**
   * @brief 所有ST边界列表
   *
   * C++语法说明：
   * - std::vector<STBoundary>：
   *   存储所有障碍物的ST边界
   *   用于需要遍历所有障碍物的场景
   */
  std::vector<STBoundary> path_time_obstacles_;

  /**
   * @brief 静态障碍物SL边界列表
   *
   * C++语法说明：
   * std::vector<SLBoundary>：
   * 存储静态障碍物的SL边界
   */
  std::vector<SLBoundary> static_obs_sl_boundaries_;
};

/**
 * @brief 命名空间结束标记
 *
 * C++语法说明：
 * // 注释用于说明命名空间结束
 * 两层命名空间的闭合：
 * }  // namespace planning
 * }  // namespace apollo
 */
}  // namespace planning
}  // namespace apollo
