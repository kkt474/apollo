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
 * @brief PathTimeGraph类实现文件
 *
 * 本文件实现了路径-时间图(Path-Time Graph)的构建和管理功能
 * PathTimeGraph是晶格规划器(Lattice Planner)中用于表示障碍物时空关系的数据结构
 *
 * 功能说明：
 * 1. 将障碍物映射到ST图(路径-时间图)
 * 2. 区分静态障碍物和动态障碍物
 * 3. 计算任意时刻的路径阻塞区间
 * 4. 计算横向约束边界
 *
 * 坐标系说明：
 * - S坐标：沿参考线/路径的距离
 * - T坐标：时间
 * - ST图：以S为横坐标、T为纵坐标的二维时空图
 *
 * 相关C++语法说明：
 * - std::vector<const Obstacle*>: 存储障碍物指针的向量
 * - std::map/std::unordered_map: 用于高效查找的关联容器
 * - std::array<T, N>: 固定大小的数组
 * - std::pair<T1, T2>: 存储两个不同类型值的结构
 **/
/**
 * @brief 路径-时间图头文件
 *
 * 包含PathTimeGraph类的完整定义
 */
#include "modules/planning/planners/lattice/behavior/path_time_graph.h"

/**
 * @brief 标准库头文件
 * <algorithm>: 提供std::sort, std::lower_bound, std::upper_bound等算法
 * <limits>: 提供std::numeric_limits获取类型极值
 */
#include <algorithm>
#include <limits>

/**
 * @brief SL边界protobuf消息头文件
 * 包含SLBoundary消息的定义，用于存储Frenet坐标系下的边界
 */
#include "modules/common_msgs/planning_msgs/sl_boundary.pb.h"

/**
 * @brief Apollo配置和数学工具头文件
 * VehicleConfigHelper: 车辆配置助手
 * linear_interpolation: 线性插值函数
 * path_matcher: 路径匹配工具
 */
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/path_matcher.h"

/**
 * @brief 规划模块GFlags头文件
 * GFlags用于从命令行或配置文件获取配置参数
 */
#include "modules/planning/planning_base/gflags/planning_gflags.h"

/**
 * @brief Apollo命名空间开始
 */
namespace apollo {
/**
 * @brief 规划模块命名空间
 */
namespace planning {

/**
 * @brief 使用别名声明，简化类型引用
 *
 * C++语法说明：
 * using声明：引入其他命名空间的类型到当前作用域
 * apollo::common::PathPoint: 路径点类型，包含x,y,z,theta,kappa等信息
 * apollo::common::TrajectoryPoint: 轨迹点类型，包含路径点和时间信息
 */
using apollo::common::PathPoint;
using apollo::common::TrajectoryPoint;

/**
 * @brief 数学库类型别名
 *
 * Box2d: 二维边界框，用于表示障碍物的紧凑形状
 * lerp: 线性插值函数
 * PathMatcher: 路径匹配工具，用于XY到SL坐标的转换
 * Polygon2d: 二维多边形，用于表示障碍物的精确形状
 */
using apollo::common::math::Box2d;
using apollo::common::math::lerp;
using apollo::common::math::PathMatcher;
using apollo::common::math::Polygon2d;

/**
 * @brief PathTimeGraph构造函数
 *
 * @param obstacles 障碍物指针列表
 * @param discretized_ref_points 离散化的参考线点
 * @param ptr_reference_line_info 参考线信息指针
 * @param s_start S坐标范围起始值
 * @param s_end S坐标范围结束值
 * @param t_start T坐标(时间)范围起始值
 * @param t_end T坐标(时间)范围结束值
 * @param init_d 初始横向状态 [d, d', d'']，Frenet坐标系下的横向位移、速度、加速度
 *
 * 功能说明：
 * 初始化PathTimeGraph，设置时空范围，保存参考线信息和初始状态
 * 然后调用SetupObstacles处理所有障碍物
 *
 * C++语法说明：
 * - std::vector<const Obstacle*>: const指针的向量，避免修改障碍物
 * - std::array<double, 3>: 固定大小数组，存储三维状态量
 * - std::pair<double, double>: 存储起止范围
 */
PathTimeGraph::PathTimeGraph(
    const std::vector<const Obstacle*>& obstacles,
    const std::vector<PathPoint>& discretized_ref_points,
    const ReferenceLineInfo* ptr_reference_line_info, const double s_start,
    const double s_end, const double t_start, const double t_end,
    const std::array<double, 3>& init_d) {
  /**
   * @brief 断言检查：确保s和t的起止值有效
   * CHECK_LT: Apollo断言宏，Less Than，检查第一个参数小于第二个
   */
  CHECK_LT(s_start, s_end);   /**< S起始值必须小于结束值 */
  CHECK_LT(t_start, t_end);   /**< T起始值必须小于结束值 */

  /**
   * @brief 设置S坐标范围
   * path_range_: std::pair<double, double>类型
   * first: 起始S坐标
   * second: 结束S坐标
   */
  path_range_.first = s_start;
  path_range_.second = s_end;

  /**
   * @brief 设置T坐标范围（时间范围）
   * time_range_: 存储时间范围的起止
   */
  time_range_.first = t_start;
  time_range_.second = t_end;

  /**
   * @brief 保存参考线信息指针
   * ptr_reference_line_info_: 成员变量，指向常量ReferenceLineInfo对象
   */
  ptr_reference_line_info_ = ptr_reference_line_info;

  /**
   * @brief 保存初始横向状态
   * init_d_: [d, d', d'']，Frenet坐标系下的横向位移、速度、加速度
   */
  init_d_ = init_d;

  /**
   * @brief 设置障碍物
   * 遍历所有障碍物，根据类型分别处理静态和动态障碍物
   */
  SetupObstacles(obstacles, discretized_ref_points);
}

/**
 * @brief 计算障碍物的SL边界
 *
 * @param vertices 障碍物的顶点列表（XY坐标系）
 * @param discretized_ref_points 离散化的参考线点
 * @return SLBoundary 障碍物在Frenet坐标系下的边界
 *
 * 功能说明：
 * 将障碍物的多边形顶点从XY坐标系转换到SL坐标系
 * 然后计算SL边界框（s方向的min/max和l方向的min/max）
 *
 * 算法流程：
 * 1. 遍历所有顶点
 * 2. 将每个顶点从XY坐标转换到SL坐标
 * 3. 记录s和l方向的最小最大值
 *
 * C++语法说明：
 * - const std::vector<common::math::Vec2d>&: 常量引用，避免拷贝
 * - Vec2d: 二维向量类型，包含x和y坐标
 * - std::numeric_limits<double>::max(): double类型的最大正数
 * - std::numeric_limits<double>::lowest(): double类型的最小正数
 */
SLBoundary PathTimeGraph::ComputeObstacleBoundary(
    const std::vector<common::math::Vec2d>& vertices,
    const std::vector<PathPoint>& discretized_ref_points) const {
  /**
   * @brief 初始化边界值为极端值
   *
   * start_s: S方向起始位置，初始化为最大值，后面取min
   * end_s: S方向结束位置，初始化为最小值，后面取max
   * start_l: L方向起始位置，初始化为最大值，后面取min
   * end_l: L方向结束位置，初始化为最小值，后面取max
   */
  double start_s(std::numeric_limits<double>::max());
  double end_s(std::numeric_limits<double>::lowest());
  double start_l(std::numeric_limits<double>::max());
  double end_l(std::numeric_limits<double>::lowest());

  /**
   * @brief 遍历障碍物的所有顶点
   * for (const auto& point : vertices):
   *   范围for循环，point是每个顶点的常量引用
   */
  for (const auto& point : vertices) {
    /**
     * @brief 将XY坐标转换为SL坐标
     *
     * PathMatcher::GetPathFrenetCoordinate():
     *   将XY坐标点匹配到路径上，返回SL坐标
     *   返回值是std::pair<double, double>，(s, l)
     *
     * point.x(), point.y():
     *   获取Vec2d类型的x和y坐标
     */
    auto sl_point = PathMatcher::GetPathFrenetCoordinate(discretized_ref_points,
                                                         point.x(), point.y());

    /**
     * @brief 更新S和L方向的边界值
     *
     * std::fmin: 取两个浮点数的较小值
     * std::fmax: 取两个浮点数的较大值
     * 用于跟踪遍历过程中的最小最大值
     */
    start_s = std::fmin(start_s, sl_point.first);   /**< 更新S方向起始（最小值） */
    end_s = std::fmax(end_s, sl_point.first);       /**< 更新S方向结束（最大值） */
    start_l = std::fmin(start_l, sl_point.second);  /**< 更新L方向起始（最小值） */
    end_l = std::fmax(end_l, sl_point.second);       /**< 更新L方向结束（最大值） */
  }

  /**
   * @brief 创建并返回SL边界对象
   * 使用set方法设置protobuf消息的字段值
   */
  SLBoundary sl_boundary;
  sl_boundary.set_start_s(start_s);   /**< 设置S方向起始 */
  sl_boundary.set_end_s(end_s);       /**< 设置S方向结束 */
  sl_boundary.set_start_l(start_l);   /**< 设置L方向起始 */
  sl_boundary.set_end_l(end_l);       /**< 设置L方向结束 */

  return sl_boundary;  /**< 返回构建好的SL边界 */
}

/**
 * @brief 设置障碍物（主函数）
 *
 * @param obstacles 障碍物指针列表
 * @param discretized_ref_points 离散化的参考线点
 *
 * 功能说明：
 * 遍历所有障碍物，区分静态和动态障碍物分别处理
 * 静态障碍物直接映射到ST图
 * 动态障碍物需要根据预测轨迹在不同时间点映射
 *
 * 算法流程：
 * 1. 遍历所有障碍物
 * 2. 如果是虚拟障碍物，跳过
 * 3. 如果没有预测轨迹，作为静态障碍物处理
 * 4. 如果有预测轨迹，作为动态障碍物处理
 * 5. 对静态障碍物按start_s排序
 * 6. 将path_time_obstacle_map_中的障碍物转移到path_time_obstacles_向量
 */
void PathTimeGraph::SetupObstacles(
    const std::vector<const Obstacle*>& obstacles,
    const std::vector<PathPoint>& discretized_ref_points) {
  /**
   * @brief 遍历所有障碍物
   * for (const Obstacle* obstacle : obstacles):
   *   obstacles是const vector，但元素指针不是const，可以修改指向的对象
   */
  for (const Obstacle* obstacle : obstacles) {
    /**
     * @brief 跳过虚拟障碍物
     * IsVirtual(): 判断是否为虚拟障碍物（如人为创建的停车墙）
     */
    if (obstacle->IsVirtual()) {
      continue;  /**< 跳过，继续下一个障碍物 */
    }

    /**
     * @brief 检查障碍物是否有预测轨迹
     * HasTrajectory(): 判断是否有移动轨迹
     * 没有轨迹视为静态障碍物，有轨迹视为动态障碍物
     */
    if (!obstacle->HasTrajectory()) {
      /**
       * @brief 处理静态障碍物
       * SetStaticObstacle(): 将静态障碍物添加到ST图
       */
      SetStaticObstacle(obstacle, discretized_ref_points);
    } else {
      /**
       * @brief 处理动态障碍物
       * SetDynamicObstacle(): 根据预测轨迹在多个时间点添加障碍物到ST图
       */
      SetDynamicObstacle(obstacle, discretized_ref_points);
    }
  }

  /**
   * @brief 对静态障碍物的SL边界按start_s排序
   *
   * std::sort: 标准库排序算法
   * lambda表达式作为比较函数
   * 按SLBoundary的start_s字段升序排列
   *
   * C++语法说明：
   * [](const SLBoundary& sl0, const SLBoundary& sl1) { return sl0.start_s() < sl1.start_s(); }
   *   - []: lambda捕获列表，空表示不捕获外部变量
   *   - (const SLBoundary& sl0, const SLBoundary& sl1): 参数列表
   *   - return sl0.start_s() < sl1.start_s(): 返回比较结果
   */
  std::sort(static_obs_sl_boundaries_.begin(), static_obs_sl_boundaries_.end(),
            [](const SLBoundary& sl0, const SLBoundary& sl1) {
              return sl0.start_s() < sl1.start_s();
            });

  /**
   * @brief 将障碍物从map转移到vector
   *
   * path_time_obstacle_map_: std::map<std::string, PathTimeObstacle>
   * path_time_obstacles_: std::vector<STBoundary>
   *
   * 遍历map，将每个障碍物添加到向量中
   */
  for (auto& path_time_obstacle : path_time_obstacle_map_) {
    path_time_obstacles_.push_back(path_time_obstacle.second);
  }
}

/**
 * @brief 设置静态障碍物
 *
 * @param obstacle 障碍物指针
 * @param discretized_ref_points 离散化的参考线点
 *
 * 功能说明：
 * 将静态障碍物映射到ST图
 * 静态障碍物在ST图中是一个矩形区域（不随时间变化）
 *
 * ST图表示：
 * - 矩形的左边界和右边界对应障碍物的start_s和end_s
 * - 矩形的上边界和下边界对应时间范围的起点和终点
 *   （FLAGS_trajectory_time_length是规划时间总长度）
 *
 * 位置关系示意图：
 * T
 * ^
 * |    upper_left    upper_right
 * |       |--------------|
 * |       |   静态障碍物   |
 * |       |--------------|
 * |    bottom_left   bottom_right
 * +------------------------------> S
 */
void PathTimeGraph::SetStaticObstacle(
    const Obstacle* obstacle,
    const std::vector<PathPoint>& discretized_ref_points) {
  /**
   * @brief 获取障碍物的多边形表示
   * PerceptionPolygon(): 返回障碍物的感知多边形
   */
  const Polygon2d& polygon = obstacle->PerceptionPolygon();

  /**
   * @brief 获取障碍物ID
   * Id(): 返回障碍物的唯一标识符
   */
  std::string obstacle_id = obstacle->Id();

  /**
   * @brief 计算障碍物的SL边界
   * GetAllVertices(): 获取多边形的所有顶点
   * ComputeObstacleBoundary(): 将顶点从XY转换到SL坐标
   */
  SLBoundary sl_boundary =
      ComputeObstacleBoundary(polygon.GetAllVertices(), discretized_ref_points);

  /**
   * @brief 获取车道宽度
   *
   * 首先使用默认参考线宽度的一半作为左右宽度
   * 然后调用GetLaneWidth获取实际的车道宽度
   *
   * FLAGS_default_reference_line_width: 默认参考线宽度
   * left_width: 左侧（中心线到左边界）的距离
   * right_width: 右侧（中心线到右边界）的距离
   */
  double left_width = FLAGS_default_reference_line_width * 0.5;
  double right_width = FLAGS_default_reference_line_width * 0.5;
  ptr_reference_line_info_->reference_line().GetLaneWidth(
      sl_boundary.start_s(), &left_width, &right_width);

  /**
   * @brief 检查障碍物是否在考虑范围内
   *
   * 跳过的情况：
   * 1. 障碍物完全在路径范围之外（end_s < path_range_.first 或 start_s > path_range_.second）
   * 2. 障碍物完全在车道范围之外（start_l > left_width 或 end_l < -right_width）
   *
   * ADEBUG: Apollo调试级别日志，仅在调试模式下输出
   */
  if (sl_boundary.start_s() > path_range_.second ||
      sl_boundary.end_s() < path_range_.first ||
      sl_boundary.start_l() > left_width ||
      sl_boundary.end_l() < -right_width) {
    ADEBUG << "Obstacle [" << obstacle_id << "] is out of range.";
    return;  /**< 障碍物不在考虑范围内，返回 */
  }

  /**
   * @brief 设置PathTimeObstacle的四个角点
   *
   * 在ST图中，静态障碍物表示为一个矩形
   * - bottom_left: (start_s, 0) - 时间起点时的左下角
   * - bottom_right: (start_s, T_max) - 时间终点时的左下角
   * - upper_left: (end_s, 0) - 时间起点时的右上角
   * - upper_right: (end_s, T_max) - 时间终点时的右上角
   *
   * T_max = FLAGS_trajectory_time_length（规划时间总长度）
   *
   * SetPathTimePoint(): 创建STPoint(s, t)
   */
  path_time_obstacle_map_[obstacle_id].set_id(obstacle_id);  /**< 设置障碍物ID */
  path_time_obstacle_map_[obstacle_id].set_bottom_left_point(
      SetPathTimePoint(obstacle_id, sl_boundary.start_s(), 0.0));  /**< 设置左下点 */
  path_time_obstacle_map_[obstacle_id].set_bottom_right_point(SetPathTimePoint(
      obstacle_id, sl_boundary.start_s(), FLAGS_trajectory_time_length));  /**< 设置右下点 */
  path_time_obstacle_map_[obstacle_id].set_upper_left_point(
      SetPathTimePoint(obstacle_id, sl_boundary.end_s(), 0.0));  /**< 设置左上点 */
  path_time_obstacle_map_[obstacle_id].set_upper_right_point(SetPathTimePoint(
      obstacle_id, sl_boundary.end_s(), FLAGS_trajectory_time_length));  /**< 设置右上点 */

  /**
   * @brief 保存静态障碍物的SL边界
   * 用于后续横向约束边界的计算
   */
  static_obs_sl_boundaries_.push_back(std::move(sl_boundary));

  /**
   * @brief 打印调试信息
   * 输出障碍物的ST映射结果
   */
  ADEBUG << "ST-Graph mapping static obstacle: " << obstacle_id
         << ", start_s : " << sl_boundary.start_s()
         << ", end_s : " << sl_boundary.end_s()
         << ", start_l : " << sl_boundary.start_l()
         << ", end_l : " << sl_boundary.end_l();
}

/**
 * @brief 设置动态障碍物
 *
 * @param obstacle 障碍物指针
 * @param discretized_ref_points 离散化的参考线点
 *
 * 功能说明：
 * 将动态障碍物（沿预测轨迹移动）映射到ST图
 * 动态障碍物在ST图中是一个梯形区域（随时间变化）
 *
 * 算法流程：
 * 1. 从时间范围起点开始，逐步增加时间
 * 2. 在每个时间点，获取障碍物的位置和边界框
 * 3. 将边界框顶点转换为SL坐标
 * 4. 更新PathTimeObstacle的角点
 * 5. 当障碍物离开考虑范围时停止
 *
 * ST图表示：
 * - 动态障碍物在ST图中是一个梯形
 * - 上边和下边对应不同时刻的s边界
 * - 左右边界是时间方向的延伸
 */
void PathTimeGraph::SetDynamicObstacle(
    const Obstacle* obstacle,
    const std::vector<PathPoint>& discretized_ref_points) {
  /**
   * @brief 从时间范围起点开始
   * relative_time: 当前处理的时间点
   */
  double relative_time = time_range_.first;

  /**
   * @brief 时间循环，直到超过时间范围终点
   * while循环：条件为真时继续循环
   */
  while (relative_time < time_range_.second) {
    /**
     * @brief 获取障碍物在当前时间点的位置
     * GetPointAtTime(): 根据相对时间获取轨迹点
     * 返回TrajectoryPoint，包含位置和朝向信息
     */
    TrajectoryPoint point = obstacle->GetPointAtTime(relative_time);

    /**
     * @brief 获取障碍物在当前时间点的边界框
     * GetBoundingBox(): 根据轨迹点计算旋转后的边界框
     * 返回Box2d类型
     */
    Box2d box = obstacle->GetBoundingBox(point);

    /**
     * @brief 计算障碍物边界框的SL边界
     * GetAllCorners(): 获取边界框的四个角点
     */
    SLBoundary sl_boundary =
        ComputeObstacleBoundary(box.GetAllCorners(), discretized_ref_points);

    /**
     * @brief 获取当前s位置的车道宽度
     */
    double left_width = FLAGS_default_reference_line_width * 0.5;
    double right_width = FLAGS_default_reference_line_width * 0.5;
    ptr_reference_line_info_->reference_line().GetLaneWidth(
        sl_boundary.start_s(), &left_width, &right_width);

    /**
     * @brief 检查障碍物是否在考虑范围内
     *
     * 如果障碍物不在范围内：
     * - 如果之前已经在map中，说明障碍物刚离开，break
     * - 如果之前不在map中，继续下一个时间点
     */
    if (sl_boundary.start_s() > path_range_.second ||
        sl_boundary.end_s() < path_range_.first ||
        sl_boundary.start_l() > left_width ||
        sl_boundary.end_l() < -right_width) {
      /**
       * @brief 检查障碍物是否已经在map中
       * find()返回一个迭代器，如果未找到则等于end()
       */
      if (path_time_obstacle_map_.find(obstacle->Id()) !=
          path_time_obstacle_map_.end()) {
        break;  /**< 障碍物已经离开，停止处理 */
      }
      /**
       * @brief 增加时间步长，继续检查
       * FLAGS_trajectory_time_resolution: 时间分辨率/步长
       */
      relative_time += FLAGS_trajectory_time_resolution;
      continue;  /**< 跳过本次循环，继续下一个时间点 */
    }

    /**
     * @brief 如果障碍物不在map中，初始化其记录
     * 只在第一次遇到有效位置时设置ID和左边界点
     */
    if (path_time_obstacle_map_.find(obstacle->Id()) ==
        path_time_obstacle_map_.end()) {
      path_time_obstacle_map_[obstacle->Id()].set_id(obstacle->Id());

      /**
       * @brief 设置左边界点（上左和下左）
       * 这些点对应障碍物在relative_time时刻的start_s
       * 初始左边界在时间维度上保持不变
       */
      path_time_obstacle_map_[obstacle->Id()].set_bottom_left_point(
          SetPathTimePoint(obstacle->Id(), sl_boundary.start_s(),
                           relative_time));
      path_time_obstacle_map_[obstacle->Id()].set_upper_left_point(
          SetPathTimePoint(obstacle->Id(), sl_boundary.end_s(), relative_time));
    }

    /**
     * @brief 更新右边界点
     * 右边界点会随着时间更新，代表障碍物在每个时刻的边界
     * 这样最后形成的是一系列右边界点的包络
     */
    path_time_obstacle_map_[obstacle->Id()].set_bottom_right_point(
        SetPathTimePoint(obstacle->Id(), sl_boundary.start_s(), relative_time));
    path_time_obstacle_map_[obstacle->Id()].set_upper_right_point(
        SetPathTimePoint(obstacle->Id(), sl_boundary.end_s(), relative_time));

    /**
     * @brief 增加时间步长
     */
    relative_time += FLAGS_trajectory_time_resolution;
  }
}

/**
 * @brief 创建ST点
 *
 * @param obstacle_id 障碍物ID
 * @param s S坐标
 * @param t T坐标（时间）
 * @return STPoint ST图中的一点
 *
 * 功能说明：
 * 简单工厂函数，创建STPoint对象
 */
STPoint PathTimeGraph::SetPathTimePoint(const std::string& obstacle_id,
                                        const double s, const double t) const {
  /**
   * @brief 创建STPoint并返回
   * STPoint构造函数接受(s, t)
   */
  STPoint path_time_point(s, t);
  return path_time_point;
}

/**
 * @brief 获取所有路径时间障碍物
 *
 * @return std::vector<STBoundary> 障碍物边界列表
 *
 * C++语法说明：
 * const成员函数：承诺不修改成员变量
 */
const std::vector<STBoundary>& PathTimeGraph::GetPathTimeObstacles() const {
  return path_time_obstacles_;  /**< 返回存储的障碍物向量引用 */
}

/**
 * @brief 根据ID获取特定障碍物
 *
 * @param obstacle_id 障碍物ID
 * @param path_time_obstacle 输出：障碍物的ST边界
 * @return bool 是否找到
 *
 * 功能说明：
 * 在path_time_obstacle_map_中查找指定ID的障碍物
 *
 * C++语法说明：
 * - 非const成员函数：可以修改成员变量
 * - Map的find()返回迭代器，未找到时等于end()
 */
bool PathTimeGraph::GetPathTimeObstacle(const std::string& obstacle_id,
                                        STBoundary* path_time_obstacle) {
  /**
   * @brief 查找障碍物
   * find()返回迭代器，如果未找到则等于end()
   */
  if (path_time_obstacle_map_.find(obstacle_id) ==
      path_time_obstacle_map_.end()) {
    return false;  /**< 未找到，返回false */
  }

  /**
   * @brief 找到障碍物，复制到输出参数
   * *解引用赋值
   */
  *path_time_obstacle = path_time_obstacle_map_[obstacle_id];
  return true;  /**< 成功找到，返回true */
}

/**
 * @brief 获取指定时刻的路径阻塞区间
 *
 * @param t 指定时刻
 * @return std::vector<std::pair<double, double>> S方向的[下限, 上限]区间列表
 *
 * 功能说明：
 * 给定时刻t，返回所有障碍物在该时刻阻塞的S区间
 * 用于判断自车在该时刻能否通行
 *
 * 算法说明：
 * 对每个时刻t，通过线性插值计算障碍物左右边界在t处的S值
 * 因为ST图中边界点是用多个(s,t)对定义的
 */
std::vector<std::pair<double, double>> PathTimeGraph::GetPathBlockingIntervals(
    const double t) const {
  /**
   * @brief 断言检查：t必须在时间范围内
   * ACHECK: Apollo断言，仅在调试模式下生效
   */
  ACHECK(time_range_.first <= t && t <= time_range_.second);

  /**
   * @brief 存储结果的区间向量
   * 每个元素是一个pair<double, double>表示[S下限, S上限]
   */
  std::vector<std::pair<double, double>> intervals;

  /**
   * @brief 遍历所有障碍物
   * 只考虑在t时刻存在的障碍物
   */
  for (const auto& pt_obstacle : path_time_obstacles_) {
    /**
     * @brief 检查t是否在障碍物的时间范围内
     * min_t(): 障碍物的最早时间
     * max_t(): 障碍物的最晚时间
     * 如果t在范围外，跳过
     */
    if (t > pt_obstacle.max_t() || t < pt_obstacle.min_t()) {
      continue;
    }

    /**
     * @brief 线性插值计算上边界的S值
     *
     * lerp: 线性插值函数
     * 参数：(x0, y0, x1, y1, y)
     * 返回在y处x的线性插值
     *
     * upper_left_point: 左上角点 (s_upper_left, t_upper_left)
     * upper_right_point: 右上角点 (s_upper_right, t_upper_right)
     * t: 当前时间
     *
     * 公式：s_upper = s0 + (s1 - s0) * (t - t0) / (t1 - t0)
     */
    double s_upper = lerp(pt_obstacle.upper_left_point().s(),
                          pt_obstacle.upper_left_point().t(),
                          pt_obstacle.upper_right_point().s(),
                          pt_obstacle.upper_right_point().t(), t);

    /**
     * @brief 线性插值计算下边界的S值
     * 同样的方法处理下边界
     */
    double s_lower = lerp(pt_obstacle.bottom_left_point().s(),
                          pt_obstacle.bottom_left_point().t(),
                          pt_obstacle.bottom_right_point().s(),
                          pt_obstacle.bottom_right_point().t(), t);

    /**
     * @brief 添加区间到结果
     * emplace_back: 直接构造，避免拷贝
     */
    intervals.emplace_back(s_lower, s_upper);
  }

  return intervals;  /**< 返回所有阻塞区间 */
}

/**
 * @brief 获取多个时刻的路径阻塞区间
 *
 * @param t_start 起始时刻
 * @param t_end 结束时刻
 * @param t_resolution 时间分辨率
 * @return std::vector<std::vector<std::pair<double, double>>> 各时刻的区间列表
 *
 * 功能说明：
 * 对[t_start, t_end]范围内按t_resolution间隔采样
 * 返回每个采样时刻的阻塞区间
 */
std::vector<std::vector<std::pair<double, double>>>
PathTimeGraph::GetPathBlockingIntervals(const double t_start,
                                        const double t_end,
                                        const double t_resolution) {
  /**
   * @brief 存储所有时刻的区间
   * 外层vector是时间，内层vector是各障碍物的区间
   */
  std::vector<std::vector<std::pair<double, double>>> intervals;

  /**
   * @brief 时间循环
   * for循环：初始化; 条件; 增量
   */
  for (double t = t_start; t <= t_end; t += t_resolution) {
    /**
     * @brief 获取当前时刻的阻塞区间
     * push_back: 添加到结果向量
     */
    intervals.push_back(GetPathBlockingIntervals(t));
  }

  return intervals;
}

/**
 * @brief 获取S坐标范围
 * @return std::pair<double, double> [起始, 结束]
 */
std::pair<double, double> PathTimeGraph::get_path_range() const {
  return path_range_;
}

/**
 * @brief 获取T坐标范围（时间范围）
 * @return std::pair<double, double> [起始, 结束]
 */
std::pair<double, double> PathTimeGraph::get_time_range() const {
  return time_range_;
}

/**
 * @brief 获取障碍物周围的ST采样点
 *
 * @param obstacle_id 障碍物ID
 * @param s_dist S方向的距离偏移（正值为前方，负值为后方）
 * @param t_min_density 时间方向最小采样密度
 * @return std::vector<STPoint> 采样点列表
 *
 * 功能说明：
 * 在障碍物边界附近生成一系列ST采样点
 * 用于速度规划的约束采样
 *
 * 算法说明：
 * 1. 根据s_dist的正负确定使用上边界还是下边界
 * 2. 计算时间范围
 * 3. 按t_min_density采样
 * 4. 对每个采样点，在S方向偏移s_dist
 */
std::vector<STPoint> PathTimeGraph::GetObstacleSurroundingPoints(
    const std::string& obstacle_id, const double s_dist,
    const double t_min_density) const {
  /**
   * @brief 断言检查：采样密度必须为正
   */
  ACHECK(t_min_density > 0.0);

  /**
   * @brief 初始化结果向量
   */
  std::vector<STPoint> pt_pairs;

  /**
   * @brief 检查障碍物是否存在
   */
  if (path_time_obstacle_map_.find(obstacle_id) ==
      path_time_obstacle_map_.end()) {
    return pt_pairs;  /**< 未找到，返回空向量 */
  }

  /**
   * @brief 获取障碍物数据
   * at(): Map的访问函数，带边界检查
   */
  const auto& pt_obstacle = path_time_obstacle_map_.at(obstacle_id);

  /**
   * @brief 根据s_dist确定使用的边界点
   *
   * s_dist > 0: 使用上边界（障碍物前方）
   * s_dist < 0: 使用下边界（障碍物后方）
   */
  double s0 = 0.0;
  double s1 = 0.0;
  double t0 = 0.0;
  double t1 = 0.0;

  if (s_dist > 0.0) {
    /**< 上边界的S和T */
    s0 = pt_obstacle.upper_left_point().s();
    s1 = pt_obstacle.upper_right_point().s();
    t0 = pt_obstacle.upper_left_point().t();
    t1 = pt_obstacle.upper_right_point().t();
  } else {
    /**< 下边界的S和T */
    s0 = pt_obstacle.bottom_left_point().s();
    s1 = pt_obstacle.bottom_right_point().s();
    t0 = pt_obstacle.bottom_left_point().t();
    t1 = pt_obstacle.bottom_right_point().t();
  }

  /**
   * @brief 计算时间间隔
   * std::fabs: 取绝对值
   */
  double time_gap = t1 - t0;
  ACHECK(time_gap > -FLAGS_numerical_epsilon);
  time_gap = std::fabs(time_gap);

  /**
   * @brief 计算采样数量和间隔
   * static_cast<size_t>: 显式类型转换，转换为无符号大小
   */
  size_t num_sections = static_cast<size_t>(time_gap / t_min_density + 1);
  double t_interval = time_gap / static_cast<double>(num_sections);

  /**
   * @brief 生成采样点
   */
  for (size_t i = 0; i <= num_sections; ++i) {
    /**
     * @brief 计算当前采样点的T值
     */
    double t = t_interval * static_cast<double>(i) + t0;

    /**
     * @brief 插值计算S值，并偏移s_dist
     */
    double s = lerp(s0, t0, s1, t1, t) + s_dist;

    /**
     * @brief 创建并添加STPoint
     */
    STPoint ptt;
    ptt.set_t(t);
    ptt.set_s(s);
    pt_pairs.push_back(std::move(ptt));  /**< 移动语义，避免拷贝 */
  }

  return pt_pairs;
}

/**
 * @brief 检查障碍物是否在图中
 * @param obstacle_id 障碍物ID
 * @return bool 是否存在
 */
bool PathTimeGraph::IsObstacleInGraph(const std::string& obstacle_id) {
  return path_time_obstacle_map_.find(obstacle_id) !=
         path_time_obstacle_map_.end();  /**< 找到则返回true */
}

/**
 * @brief 获取横向约束边界
 *
 * @param s_start S范围起始
 * @param s_end S范围结束
 * @param s_resolution S方向分辨率
 * @return std::vector<std::pair<double, double>> [左边界, 右边界]列表
 *
 * 功能说明：
 * 计算沿路径各点的横向约束边界
 * 考虑车道边界和静态障碍物的约束
 *
 * 算法流程：
 * 1. 初始化：按车道宽度设置边界
 * 2. 减去自车宽度的一半（因为边界是相对于自车中心）
 * 3. 遍历所有静态障碍物，收缩边界
 * 4. 最终边界是车辆中心可以安全行驶的范围
 */
std::vector<std::pair<double, double>> PathTimeGraph::GetLateralBounds(
    const double s_start, const double s_end, const double s_resolution) {
  /**
   * @brief 断言检查
   * CHECK_LT: 起始小于结束
   * CHECK_GT: 分辨率大于零
   */
  CHECK_LT(s_start, s_end);
  CHECK_GT(s_resolution, FLAGS_numerical_epsilon);

  /**
   * @brief 初始化结果和中间变量
   * bounds: 存储每点的[左边界, 右边界]
   * discretized_path: 存储对应的S坐标
   */
  std::vector<std::pair<double, double>> bounds;
  std::vector<double> discretized_path;

  /**
   * @brief 计算采样点数量
   */
  double s_range = s_end - s_start;
  double s_curr = s_start;
  size_t num_bound = static_cast<size_t>(s_range / s_resolution);

  /**
   * @brief 获取车辆宽度
   */
  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  double ego_width = vehicle_config.vehicle_param().width();

  /**
   * @brief 第一步：按车道宽度初始化边界
   *
   * 对于每个采样点：
   * 1. 获取该S位置的车道宽度
   * 2. 计算自车中心的初始横向范围
   * 3. 应用边界缓冲区
   */
  for (size_t i = 0; i < num_bound; ++i) {
    /**
     * @brief 获取车道宽度
     */
    double left_width = FLAGS_default_reference_line_width / 2.0;
    double right_width = FLAGS_default_reference_line_width / 2.0;
    ptr_reference_line_info_->reference_line().GetLaneWidth(s_curr, &left_width,
                                                            &right_width);

    /**
     * @brief 计算自车中心的初始边界
     * init_d_[0]: 初始横向位移d
     * ego_width/2: 车辆宽度的一半
     */
    double ego_d_lower = init_d_[0] - ego_width / 2.0;
    double ego_d_upper = init_d_[0] + ego_width / 2.0;

    /**
     * @brief 应用边界缓冲区并存储
     * std::min/std::max: 取较小/较大值
     * FLAGS_bound_buffer: 边界缓冲区
     */
    bounds.emplace_back(
        std::min(-right_width, ego_d_lower - FLAGS_bound_buffer),
        std::max(left_width, ego_d_upper + FLAGS_bound_buffer));

    discretized_path.push_back(s_curr);  /**< 保存S坐标 */
    s_curr += s_resolution;  /**< 步进 */
  }

  /**
   * @brief 第二步：根据静态障碍物更新边界
   * 遍历所有静态障碍物的SL边界
   */
  for (const SLBoundary& static_sl_boundary : static_obs_sl_boundaries_) {
    UpdateLateralBoundsByObstacle(static_sl_boundary, discretized_path, s_start,
                                  s_end, &bounds);
  }

  /**
   * @brief 第三步：收缩边界以适应自车宽度
   * 边界的实际含义是自车中心可以行驶的范围
   * 需要减去自车宽度的一半
   */
  for (size_t i = 0; i < bounds.size(); ++i) {
    bounds[i].first += ego_width / 2.0;   /**< 左边界右移 */
    bounds[i].second -= ego_width / 2.0;  /**< 右边界左移 */

    /**
     * @brief 如果边界重叠，设为零
     */
    if (bounds[i].first >= bounds[i].second) {
      bounds[i].first = 0.0;
      bounds[i].second = 0.0;
    }
  }

  return bounds;
}

/**
 * @brief 根据障碍物更新横向边界
 *
 * @param sl_boundary 障碍物的SL边界
 * @param discretized_path 离散化的S坐标列表
 * @param s_start S范围起始
 * @param s_end S范围结束
 * @param bounds 输入/输出：边界向量
 *
 * 功能说明：
 * 根据障碍物的L方向位置，更新受影响区间的边界
 *
 * 情况分析：
 * 1. 障碍物跨越中心线（end_l > 0 且 start_l < 0）：完全阻塞
 * 2. 障碍物在左侧（end_l < 0）：收缩右边界
 * 3. 障碍物在右侧（start_l > 0）：收缩左边界
 *
 * 算法说明：
 * 使用二分查找确定障碍物在S方向上影响的区间
 */
void PathTimeGraph::UpdateLateralBoundsByObstacle(
    const SLBoundary& sl_boundary, const std::vector<double>& discretized_path,
    const double s_start, const double s_end,
    std::vector<std::pair<double, double>>* const bounds) {
  /**
   * @brief 检查障碍物是否在S范围内
   */
  if (sl_boundary.start_s() > s_end || sl_boundary.end_s() < s_start) {
    return;  /**< 不在范围内，返回 */
  }

  /**
   * @brief 使用二分查找确定障碍物影响的索引范围
   * lower_bound: 找到第一个 >= value 的位置
   * upper_bound: 找到第一个 > value 的位置
   */
  auto start_iter = std::lower_bound(
      discretized_path.begin(), discretized_path.end(), sl_boundary.start_s());
  auto end_iter = std::upper_bound(
      discretized_path.begin(), discretized_path.end(), sl_boundary.start_s());

  /**
   * @brief 计算索引
   */
  size_t start_index = start_iter - discretized_path.begin();
  size_t end_index = end_iter - discretized_path.begin();

  /**
   * @brief 情况1：障碍物跨越中心线
   * end_l > 0 且 start_l < 0（有一定容差FLAGS_numerical_epsilon）
   * 此时需要将边界收缩到中心线附近
   */
  if (sl_boundary.end_l() > -FLAGS_numerical_epsilon &&
      sl_boundary.start_l() < FLAGS_numerical_epsilon) {
    /**
     * @brief 对所有受影响点，设置狭窄的边界
     */
    for (size_t i = start_index; i < end_index; ++i) {
      bounds->operator[](i).first = -FLAGS_numerical_epsilon;
      bounds->operator[](i).second = FLAGS_numerical_epsilon;
    }
    return;
  }

  /**
   * @brief 情况2：障碍物完全在左侧（end_l < 0）
   * 收缩右边界（左边界不变）
   * 车辆应该靠右行驶
   */
  if (sl_boundary.end_l() < FLAGS_numerical_epsilon) {
    for (size_t i = start_index; i < std::min(end_index + 1, bounds->size());
         ++i) {
      /**
       * @brief 右边界取当前值和障碍物边界的较大值
       * + FLAGS_nudge_buffer: 添加避让缓冲区
       */
      bounds->operator[](i).first =
          std::max(bounds->operator[](i).first,
                   sl_boundary.end_l() + FLAGS_nudge_buffer);
    }
    return;
  }

  /**
   * @brief 情况3：障碍物完全在右侧（start_l > 0）
   * 收缩左边界（右边界不变）
   * 车辆应该靠左行驶
   */
  if (sl_boundary.start_l() > -FLAGS_numerical_epsilon) {
    for (size_t i = start_index; i < std::min(end_index + 1, bounds->size());
         ++i) {
      /**
       * @brief 左边界取当前值和障碍物边界的较小值
       * - FLAGS_nudge_buffer: 添加避让缓冲区
       */
      bounds->operator[](i).second =
          std::min(bounds->operator[](i).second,
                   sl_boundary.start_l() - FLAGS_nudge_buffer);
    }
    return;
  }
}

}  // namespace planning
}  // namespace apollo
