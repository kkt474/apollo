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
 * @brief CollisionChecker碰撞检查器实现文件
 *
 * 本文件实现了晶格规划器(Lattice Planner)中的碰撞检查功能
 * CollisionChecker用于检查自车轨迹与障碍物之间是否发生碰撞
 *
 * 功能说明：
 * 1. 构建预测环境：预先计算所有障碍物在未来时刻的边界框
 * 2. 碰撞检测：检查自车轨迹的每个点是否与障碍物重叠
 * 3. 车辆位置判断：判断自车是否在车道内、障碍物是否在自车后方
 *
 * 算法原理：
 * - 使用轴对齐边界框(AABB)或定向边界框(OBB)表示车辆和障碍物
 * - HasOverlap()函数检测两个边界框是否重叠
 * - 在轨迹的每个时间点检查碰撞
 *
 * 相关C++语法说明：
 * - std::shared_ptr<T>: 智能指针，多个对象可以共享所有权
 * - std::vector<Box2d>: 存储边界框的向量
 * - const TrajectoryPoint&: 常量引用，避免拷贝
 **/
/**
 * @brief 碰撞检查器头文件
 *
 * 包含CollisionChecker类的完整定义
 */
#include "modules/planning/planners/lattice/behavior/collision_checker.h"

/**
 * @brief 标准库头文件
 * <utility>: 提供std::pair等工具
 */
#include <utility>

/**
 * @brief 预测障碍物protobuf消息头文件
 * 包含PredictionObstacle等消息的定义
 */
#include "modules/common_msgs/prediction_msgs/prediction_obstacle.pb.h"

/**
 * @brief Cyber RT日志系统头文件
 * 提供AWARN、AERROR、ACHECK等日志宏
 */
#include "cyber/common/log.h"

/**
 * @brief Apollo配置和数学工具头文件
 * VehicleConfigHelper: 车辆配置助手
 * path_matcher: 路径匹配工具
 * vec2d: 二维向量运算
 */
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/path_matcher.h"
#include "modules/common/math/vec2d.h"

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
 * PathPoint: 路径点类型，包含位置、朝向、曲率等信息
 * TrajectoryPoint: 轨迹点类型，包含路径点和相对时间
 * Box2d: 二维边界框，用于碰撞检测
 * PathMatcher: 路径匹配工具，用于XY到SL坐标转换
 * Vec2d: 二维向量类型
 */
using apollo::common::PathPoint;
using apollo::common::TrajectoryPoint;
using apollo::common::math::Box2d;
using apollo::common::math::PathMatcher;
using apollo::common::math::Vec2d;

/**
 * @brief CollisionChecker构造函数
 *
 * @param obstacles 障碍物指针列表
 * @param ego_vehicle_s 自车在Frenet坐标系下的s坐标
 * @param ego_vehicle_d 自车在Frenet坐标系下的d坐标（横向偏移）
 * @param discretized_reference_line 离散化的参考线点
 * @param ptr_reference_line_info 参考线信息指针
 * @param ptr_path_time_graph PathTimeGraph智能指针，用于查询障碍物
 *
 * 功能说明：
 * 初始化碰撞检查器，保存参考线信息和PathTimeGraph指针
 * 然后调用BuildPredictedEnvironment构建预测环境
 *
 * C++语法说明：
 * - std::shared_ptr<PathTimeGraph>: 共享所有权的智能指针
 *   多个对象可以持有同一个PathTimeGraph，引用计数为0时自动释放
 */
CollisionChecker::CollisionChecker(
    const std::vector<const Obstacle*>& obstacles, const double ego_vehicle_s,
    const double ego_vehicle_d,
    const std::vector<PathPoint>& discretized_reference_line,
    const ReferenceLineInfo* ptr_reference_line_info,
    const std::shared_ptr<PathTimeGraph>& ptr_path_time_graph) {
  /**
   * @brief 保存参考线信息指针
   * ptr_reference_line_info_: 指向常量ReferenceLineInfo对象
   */
  ptr_reference_line_info_ = ptr_reference_line_info;

  /**
   * @brief 保存PathTimeGraph智能指针
   * std::shared_ptr: 引用计数智能指针
   * 多个地方共享同一个PathTimeGraph时，只要还有一个持有者，对象就不会被释放
   */
  ptr_path_time_graph_ = ptr_path_time_graph;

  /**
   * @brief 构建预测环境
   * 在规划时间范围内，预先计算所有障碍物在每个时间点的边界框
   * 这样在碰撞检测时不需要每次都计算障碍物的边界框
   */
  BuildPredictedEnvironment(obstacles, ego_vehicle_s, ego_vehicle_d,
                            discretized_reference_line);
}

/**
 * @brief 碰撞检测函数（重载版本1）
 *
 * @param obstacles 障碍物指针列表
 * @param ego_trajectory 自车轨迹
 * @param ego_length 自车长度
 * @param ego_width 自车宽度
 * @param ego_back_edge_to_center 自车后边缘到几何中心的距离
 * @return bool 是否发生碰撞
 *
 * 功能说明：
 * 对自车轨迹的每个时间点，检查自车边界框与所有障碍物边界框是否重叠
 * 这是完整的碰撞检测实现，直接使用障碍物列表进行检测
 *
 * 算法流程：
 * 1. 遍历轨迹的每个点
 * 2. 获取该时刻自车的边界框
 * 3. 校正边界框中心点与几何中心点的不一致
 * 4. 获取该时刻所有障碍物的边界框
 * 5. 检查自车边界框与障碍物边界框是否有重叠
 * 6. 如果有任何重叠，返回true（发生碰撞）
 *
 * C++语法说明：
 * - DiscretizedTrajectory: 离散轨迹类型，本质是TrajectoryPoint的向量
 * - NumOfPoints(): 返回轨迹点的数量
 * - TrajectoryPointAt(i): 获取指定索引的轨迹点
 * - HasOverlap(): Box2d类方法，检查两个边界框是否重叠
 */
bool CollisionChecker::InCollision(
    const std::vector<const Obstacle*>& obstacles,
    const DiscretizedTrajectory& ego_trajectory, const double ego_length,
    const double ego_width, const double ego_back_edge_to_center) {
  /**
   * @brief 遍历自车轨迹的每个点
   * for循环：初始化i=0，条件i < 点数，增量i++
   * NumOfPoints(): 返回轨迹中轨迹点的数量
   */
  for (size_t i = 0; i < ego_trajectory.NumOfPoints(); ++i) {
    /**
     * @brief 获取当前轨迹点
     * TrajectoryPointAt(): 获取指定索引的轨迹点
     * static_cast<std::uint32_t>: 显式类型转换，将size_t转为uint32_t
     */
    const auto& ego_point =
        ego_trajectory.TrajectoryPointAt(static_cast<std::uint32_t>(i));

    /**
     * @brief 获取相对时间和朝向角
     * relative_time(): 该点相对于轨迹起点的相对时间
     * path_point().theta(): 该点的航向角（弧度）
     */
    const auto relative_time = ego_point.relative_time();
    const auto ego_theta = ego_point.path_point().theta();

    /**
     * @brief 创建自车边界框
     *
     * Box2d构造函数参数：
     * 1. center: 边界框中心点坐标 {x, y}
     * 2. heading: 边界框朝向角（弧度）
     * 3. length: 边界框长度
     * 4. width: 边界框宽度
     *
     * ego_point.path_point().x/y():
     *   获取路径点的x和y坐标
     */
    Box2d ego_box({ego_point.path_point().x(), ego_point.path_point().y()},
                  ego_theta, ego_length, ego_width);

    /**
     * @brief 校正边界框中心点与几何中心点的不一致
     *
     * TODO注释：这段逻辑应该在构造ego_box之前完成
     *
     * 问题说明：
     * 轨迹点是车辆后边缘的位置，而Box2d使用几何中心作为参考点
     * 需要将边界框从后边缘平移到几何中心
     *
     * 计算方法：
     * shift_distance = 车辆长度/2 - 后边缘到中心的距离
     * shift_vec: 平移向量，根据朝向角计算x和y分量
     */
    double shift_distance = ego_length / 2.0 - ego_back_edge_to_center;
    Vec2d shift_vec(shift_distance * std::cos(ego_theta),
                    shift_distance * std::sin(ego_theta));
    ego_box.Shift(shift_vec);  /**< Shift方法将边界框沿指定向量平移 */

    /**
     * @brief 存储当前时刻的障碍物边界框
     */
    std::vector<Box2d> obstacle_boxes;

    /**
     * @brief 遍历所有障碍物
     * for (const auto obstacle : obstacles):
     *   范围for循环，obstacle是const Obstacle*类型
     */
    for (const auto obstacle : obstacles) {
      /**
       * @brief 获取障碍物在当前时刻的位置
       * GetPointAtTime(): 根据相对时间获取障碍物的轨迹点
       * 如果障碍物没有轨迹（静态障碍物），则返回固定位置
       */
      auto obtacle_point = obstacle->GetPointAtTime(relative_time);

      /**
       * @brief 获取障碍物在当前位置的边界框
       * GetBoundingBox(): 根据轨迹点计算旋转后的边界框
       */
      Box2d obstacle_box = obstacle->GetBoundingBox(obtacle_point);

      /**
       * @brief 检查自车与障碍物是否重叠
       * HasOverlap(): 检测两个Box2d是否相交/重叠
       */
      if (ego_box.HasOverlap(obstacle_box)) {
        return true;  /**< 发生碰撞，返回true */
      }
    }
  }

  return false;  /**< 所有点检查完毕，没有碰撞，返回false */
}

/**
 * @brief 碰撞检测函数（重载版本2）
 *
 * @param discretized_trajectory 离散化的轨迹
 * @return bool 是否发生碰撞
 *
 * 功能说明：
 * 使用预先构建的预测环境（predicted_bounding_rectangles_）进行碰撞检测
 * 这个版本更高效，因为障碍物边界框已经预计算好了
 *
 * 算法流程：
 * 1. 验证轨迹点数量不超过预测环境大小
 * 2. 获取车辆参数
 * 3. 遍历轨迹的每个点
 * 4. 创建自车边界框并校正中心点
 * 5. 获取该时刻的预测障碍物边界框列表
 * 6. 检查是否有任何重叠
 *
 * C++语法说明：
 * - predicted_bounding_rectangles_: 3D向量
 *   外层是时间（轨迹点索引）
 *   内层是该时刻所有障碍物的边界框列表
 * - CHECK_LE: Apollo断言宏，检查左参数 <= 右参数
 */
bool CollisionChecker::InCollision(
    const DiscretizedTrajectory& discretized_trajectory) {
  /**
   * @brief 断言检查
   * CHECK_LE: Less Than or Equal，检查轨迹点数不超过预测环境大小
   */
  CHECK_LE(discretized_trajectory.NumOfPoints(),
           predicted_bounding_rectangles_.size());

  /**
   * @brief 获取车辆配置
   * VehicleConfigHelper::Instance()->GetConfig():
   *   获取车辆配置单例
   * vehicle_param().length/width():
   *   获取车辆长度和宽度
   */
  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  double ego_length = vehicle_config.vehicle_param().length();
  double ego_width = vehicle_config.vehicle_param().width();

  /**
   * @brief 遍历轨迹的每个点
   */
  for (size_t i = 0; i < discretized_trajectory.NumOfPoints(); ++i) {
    /**
     * @brief 获取当前轨迹点
     */
    const auto& trajectory_point =
        discretized_trajectory.TrajectoryPointAt(static_cast<std::uint32_t>(i));

    /**
     * @brief 获取朝向角并创建自车边界框
     */
    double ego_theta = trajectory_point.path_point().theta();
    Box2d ego_box(
        {trajectory_point.path_point().x(), trajectory_point.path_point().y()},
        ego_theta, ego_length, ego_width);

    /**
     * @brief 校正边界框中心点
     * 计算从后边缘到几何中心的平移距离
     */
    double shift_distance =
        ego_length / 2.0 - vehicle_config.vehicle_param().back_edge_to_center();
    Vec2d shift_vec{shift_distance * std::cos(ego_theta),
                    shift_distance * std::sin(ego_theta)};
    ego_box.Shift(shift_vec);

    /**
     * @brief 遍历该时刻的预测障碍物边界框
     * predicted_bounding_rectangles_[i]:
     *   获取第i个时间点所有障碍物的边界框
     *   返回类型是std::vector<Box2d>
     */
    for (const auto& obstacle_box : predicted_bounding_rectangles_[i]) {
      /**
       * @brief 检查是否与自车重叠
       */
      if (ego_box.HasOverlap(obstacle_box)) {
        return true;  /**< 发生碰撞 */
      }
    }
  }

  return false;  /**< 没有碰撞 */
}

/**
 * @brief 构建预测环境
 *
 * @param obstacles 障碍物指针列表
 * @param ego_vehicle_s 自车在Frenet坐标系下的s坐标
 * @param ego_vehicle_d 自车在Frenet坐标系下的d坐标
 * @param discretized_reference_line 离散化的参考线点
 *
 * 功能说明：
 * 在规划时间范围内，预先计算所有障碍物在每个时间点的边界框
 * 存储在predicted_bounding_rectangles_成员变量中
 *
 * 算法流程：
 * 1. 判断自车是否在车道内
 * 2. 如果在车道内，忽略同车道的后方障碍物
 * 3. 在每个时间步，获取所有障碍物的边界框
 * 4. 对边界框进行扩展（前后左右各扩展一定距离）
 * 5. 存储到predicted_bounding_rectangles_
 *
 * C++语法说明：
 * - predicted_bounding_rectangles_: std::vector<std::vector<Box2d>>
 *   外层vector的每个元素代表一个时间步
 *   内层vector存储该时间步所有障碍物的边界框
 * - ACHECK: Apollo断言，仅在调试模式下生效
 * - std::move: 移动语义，避免大数据拷贝
 */
void CollisionChecker::BuildPredictedEnvironment(
    const std::vector<const Obstacle*>& obstacles, const double ego_vehicle_s,
    const double ego_vehicle_d,
    const std::vector<PathPoint>& discretized_reference_line) {
  /**
   * @brief 断言检查：确保预测环境为空
   * ACHECK: Apollo断言宏，仅在调试模式下生效
   * 在构建前清空之前的预测数据
   */
  ACHECK(predicted_bounding_rectangles_.empty());

  /**
   * @brief 判断自车是否在车道内
   * IsEgoVehicleInLane():
   *   检查自车的d坐标是否在车道宽度范围内
   */
  bool ego_vehicle_in_lane = IsEgoVehicleInLane(ego_vehicle_s, ego_vehicle_d);

  /**
   * @brief 需要考虑的障碍物列表
   * 经过筛选后，只包含需要检测碰撞的障碍物
   */
  std::vector<const Obstacle*> obstacles_considered;

  /**
   * @brief 遍历所有障碍物
   */
  for (const Obstacle* obstacle : obstacles) {
    /**
     * @brief 跳过虚拟障碍物
     * IsVirtual(): 判断是否为虚拟障碍物（如停车墙）
     */
    if (obstacle->IsVirtual()) {
      continue;
    }

    /**
     * @brief 如果自车在车道内，进一步筛选障碍物
     *
     * 忽略条件（满足任一则忽略）：
     * 1. 障碍物在自车后方（同车道）
     * 2. 障碍物不在ST图中（不在考虑范围内）
     */
    if (ego_vehicle_in_lane &&
        (IsObstacleBehindEgoVehicle(obstacle, ego_vehicle_s,
                                    discretized_reference_line) ||
         !ptr_path_time_graph_->IsObstacleInGraph(obstacle->Id()))) {
      continue;  /**< 忽略该障碍物 */
    }

    /**
     * @brief 添加到需要考虑的障碍物列表
     */
    obstacles_considered.push_back(obstacle);
  }

  /**
   * @brief 时间循环：遍历整个规划时间范围
   *
   * FLAGS_trajectory_time_length: 轨迹时间长度（如8秒）
   * FLAGS_trajectory_time_resolution: 时间分辨率（如0.1秒）
   *
   * while循环：条件为真时继续，直到relative_time >= 时间长度
   */
  double relative_time = 0.0;
  while (relative_time < FLAGS_trajectory_time_length) {
    /**
     * @brief 当前时刻的预测环境
     * 存储该时刻所有障碍物的边界框
     */
    std::vector<Box2d> predicted_env;

    /**
     * @brief 遍历所有需要考虑的障碍物
     */
    for (const Obstacle* obstacle : obstacles_considered) {
      /**
       * @brief 获取障碍物在当前时刻的位置
       *
       * 注释说明：
       * 如果障碍物没有轨迹（静态障碍物），GetPointAtTime会返回固定位置
       * Obstacle类内部已经处理了这种情况
       */
      TrajectoryPoint point = obstacle->GetPointAtTime(relative_time);

      /**
       * @brief 获取障碍物在当前位置的边界框
       */
      Box2d box = obstacle->GetBoundingBox(point);

      /**
       * @brief 扩展边界框
       *
       * 纵向扩展：前后各扩展2倍的安全距离
       * 横向扩展：左右各扩展2倍的安全距离
       *
       * FLAGS_lon_collision_buffer: 纵向碰撞缓冲区（如0.1米）
       * FLAGS_lat_collision_buffer: 横向碰撞缓冲区（如0.05米）
       *
       * 扩展的目的是留出安全余量，避免极限情况下的碰撞
       */
      box.LongitudinalExtend(2.0 * FLAGS_lon_collision_buffer);
      box.LateralExtend(2.0 * FLAGS_lat_collision_buffer);

      /**
       * @brief 添加到当前时刻的预测环境
       * std::move: 移动语义，避免边界框数据拷贝
       */
      predicted_env.push_back(std::move(box));
    }

    /**
     * @brief 将当前时刻的预测环境添加到总列表
     * 这样就形成了一个时间序列的预测环境
     */
    predicted_bounding_rectangles_.push_back(std::move(predicted_env));

    /**
     * @brief 时间步进
     * 增加一个时间分辨率
     */
    relative_time += FLAGS_trajectory_time_resolution;
  }
}

/**
 * @brief 判断自车是否在车道内
 *
 * @param ego_vehicle_s 自车s坐标
 * @param ego_vehicle_d 自车d坐标（横向偏移）
 * @return bool 是否在车道内
 *
 * 功能说明：
 * 检查自车的横向偏移d是否在车道宽度范围内
 *
 * 车道宽度判断：
 * - left_width: 中心线到左边界
 * - right_width: 中心线到右边界
 * - 如果 d < left_width 且 d > -right_width，则在车道内
 */
bool CollisionChecker::IsEgoVehicleInLane(const double ego_vehicle_s,
                                          const double ego_vehicle_d) {
  /**
   * @brief 获取默认车道宽度的一半作为初始值
   * FLAGS_default_reference_line_width: 默认参考线宽度（如3.75米 * 3 = 11.25米）
   */
  double left_width = FLAGS_default_reference_line_width * 0.5;
  double right_width = FLAGS_default_reference_line_width * 0.5;

  /**
   * @brief 获取实际的车道宽度
   * 在参考线的不同位置，车道宽度可能不同
   */
  ptr_reference_line_info_->reference_line().GetLaneWidth(
      ego_vehicle_s, &left_width, &right_width);

  /**
   * @brief 判断d坐标是否在车道范围内
   * d > -right_width: 不超过右边界
   * d < left_width: 不超过左边界
   */
  return ego_vehicle_d < left_width && ego_vehicle_d > -right_width;
}

/**
 * @brief 判断障碍物是否在自车后方（同车道）
 *
 * @param obstacle 障碍物指针
 * @param ego_vehicle_s 自车s坐标
 * @param discretized_reference_line 离散化的参考线
 * @return bool 是否在后方
 *
 * 功能说明：
 * 检查障碍物是否满足以下条件：
 * 1. 在自车后方（障碍物s < 自车s）
 * 2. 在同一车道内（|障碍物d| < 车道宽度一半）
 *
 * 如果是，则该障碍物不需要考虑（已被自车超越或不在行驶路径上）
 */
bool CollisionChecker::IsObstacleBehindEgoVehicle(
    const Obstacle* obstacle, const double ego_vehicle_s,
    const std::vector<PathPoint>& discretized_reference_line) {
  /**
   * @brief 获取车道宽度的一半
   */
  double half_lane_width = FLAGS_default_reference_line_width * 0.5;

  /**
   * @brief 获取障碍物在t=0时刻的位置
   * 预测轨迹中t=0对应的是当前时刻
   */
  TrajectoryPoint point = obstacle->GetPointAtTime(0.0);

  /**
   * @brief 将障碍物位置转换到Frenet坐标系
   * GetPathFrenetCoordinate():
   *   将XY坐标转换到SL坐标系
   *   返回pair<double, double>：(s, l)
   */
  auto obstacle_reference_line_position = PathMatcher::GetPathFrenetCoordinate(
      discretized_reference_line, point.path_point().x(),
      point.path_point().y());

  /**
   * @brief 判断障碍物是否在自车后方且同车道
   *
   * 条件：
   * 1. obstacle_reference_line_position.first < ego_vehicle_s
   *    障碍物的s坐标 < 自车的s坐标（障碍物在后方）
   * 2. std::fabs(obstacle_reference_line_position.second) < half_lane_width
   *    障碍物的|d| < 车道宽度一半（在同一车道）
   */
  if (obstacle_reference_line_position.first < ego_vehicle_s &&
      std::fabs(obstacle_reference_line_position.second) < half_lane_width) {
    ADEBUG << "Ignore obstacle [" << obstacle->Id() << "]";
    return true;  /**< 在后方，返回true */
  }

  return false;  /**< 不在后方，返回false */
}

}  // namespace planning
}  // namespace apollo
