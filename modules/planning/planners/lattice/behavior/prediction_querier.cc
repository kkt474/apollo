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
 * @file prediction_querier.cc
 * @brief 预测查询器实现文件
 *
 * 功能说明：
 * 预测查询器用于访问障碍物的预测轨迹信息
 * 主要功能：
 * 1. 存储和查询障碍物列表
 * 2. 根据ID查找障碍物
 * 3. 计算障碍物沿参考线的投影速度
 *
 * 设计特点：
 * - 障碍物数据存储在map中，支持O(1)查找
 * - 支持预测轨迹的时间查询
 * - 提供速度投影计算，用于相对速度分析
 *
 * C++语法说明：
 * - namespace嵌套：apollo::planning两层命名空间
 * - initializer list：构造函数初始化列表
 * - std::lower_bound：二分查找下界
 * - lambda表达式：用于比较函数
 * - std::map::at：按键访问值
 */

#include "modules/planning/planners/lattice/behavior/prediction_querier.h"

#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/path_matcher.h"

namespace apollo {
/**
 * @brief Apollo外层命名空间
 */
namespace planning {

/**
 * @brief 预测查询器构造函数
 *
 * @param obstacles 障碍物指针向量
 * @param ptr_reference_line 参考线路径点共享指针
 *
 * 功能说明：
 * 1. 初始化参考线指针
 * 2. 遍历障碍物列表
 * 3. 将障碍物插入map（去重）
 * 4. 同时维护vector保持顺序
 *
 * C++语法说明：
 * - : ptr_reference_line_(ptr_reference_line)：
 *   构造函数的初始化列表
 *   用于初始化引用成员变量ptr_reference_line_
 *   引用必须在构造时绑定，不能后来赋值
 *
 * - const std::vector<const Obstacle*>&：
 *   常量引用，避免拷贝
 *   vector存储指向const Obstacle的指针
 *
 * - const std::shared_ptr<std::vector<common::PathPoint>>&：
 *   shared_ptr智能指针的常量引用
 *   共享所有权，引用计数管理生命周期
 *
 * - for (const auto ptr_obstacle : obstacles)：
 *   范围for循环遍历障碍物
 *   const auto：常量推导，避免拷贝
 *
 * - common::util::InsertIfNotPresent(&map, key, value)：
 *   Apollo工具函数
 *   如果key不存在，插入value，返回true
 *   如果key已存在，不插入，返回false
 *
 * - obstacles_.push_back(ptr_obstacle)：
 *   向vector添加障碍物指针
 *   维护插入顺序
 *
 * - AWARN << ...：
 *   Apollo WARNING级别日志
 *   输出重复障碍物警告
 */
PredictionQuerier::PredictionQuerier(
    const std::vector<const Obstacle*>& obstacles,
    const std::shared_ptr<std::vector<common::PathPoint>>& ptr_reference_line)
    : ptr_reference_line_(ptr_reference_line) {
  /**
   * @brief 遍历障碍物列表
   */
  for (const auto ptr_obstacle : obstacles) {
    /**
     * @brief 插入障碍物到map（去重）
     *
     * 如果障碍物ID不存在，插入并添加到vector
     * 如果已存在，输出警告
     */
    if (common::util::InsertIfNotPresent(&id_obstacle_map_, ptr_obstacle->Id(),
                                         ptr_obstacle)) {
      obstacles_.push_back(ptr_obstacle);
    } else {
      AWARN << "Duplicated obstacle found [" << ptr_obstacle->Id() << "]";
    }
  }
}

/**
 * @brief 获取所有障碍物
 *
 * @return std::vector<const Obstacle*> 障碍物指针向量
 *
 * 功能说明：
 * 返回存储的所有障碍物列表
 *
 * C++语法说明：
 * - std::vector<const Obstacle*>：
 *   vector存储常量指针
 *   返回值拷贝，调用者获得副本
 *
 * - const后缀：
 *   成员函数承诺不修改成员变量
 */
std::vector<const Obstacle*> PredictionQuerier::GetObstacles() const {
  return obstacles_;
}

/**
 * @brief 计算障碍物沿参考线的投影速度
 *
 * @param obstacle_id 障碍物ID
 * @param s 沿参考线的距离坐标
 * @param t 预测时间
 * @return double 投影速度（米/秒）
 *
 * 功能说明：
 * 计算障碍物在指定时刻t、沿参考线s位置处的速度
 * 在参考线方向上的投影分量
 *
 * 算法流程：
 * 1. 根据ID查找障碍物轨迹
 * 2. 检查轨迹是否有效
 * 3. 使用二分查找找到t时刻对应的轨迹点
 * 4. 提取该点的速度大小和朝向
 * 5. 将速度分解为x和y分量
 * 6. 获取参考线在s处的朝向
 * 7. 计算沿参考线方向的投影速度
 *
 * C++语法说明：
 * - const std::string& obstacle_id：
 *   常量引用，障碍物ID
 *
 * - ACHECK(condition)：
 *   Apollo断言宏
 *   如果条件不满足，程序终止
 *   用于检查前置条件
 *
 * - id_obstacle_map_.find(obstacle_id)：
 *   map的find方法
 *   返回迭代器，找到返回迭代器，未找到返回end()
 *
 * - id_obstacle_map_.at(obstacle_id)：
 *   map的at方法
 *   返回键对应的值
 *   如果键不存在，抛出std::out_of_range异常
 *
 * - trajectory.trajectory_point_size()：
 *   protobuf的repeated字段大小
 *   返回轨迹点数量
 *
 * - trajectory.trajectory_point(i)：
 *   protobufrepeated字段的随机访问
 *   返回第i个轨迹点
 *
 * - std::lower_bound(begin, end, value, comp)：
 *   二分查找第一个不小于value的元素
 *   comp定义比较规则
 *
 * - lambda表达式：[](const TrajectoryPoint& p, const double t) { return p.relative_time() < t; }：
 *   匿名函数对象
 *   比较轨迹点的relative_time与目标时间t
 *
 * - matched_it->v()：
 *   迭代器->成员访问
 *   获取匹配点的速度大小
 *
 * - matched_it->path_point().theta()：
 *   链式成员访问
 *   获取匹配点的朝向角
 *
 * - std::cos(theta) * v_x + std::sin(theta) * v_y：
 *   速度向量与参考线朝向的点积
 *   计算沿参考线方向的投影
 *
 * - common::math::PathMatcher::MatchToPath(...)：
 *   静态函数调用
 *   将(s, t)坐标转换为路径上的点
 */
double PredictionQuerier::ProjectVelocityAlongReferenceLine(
    const std::string& obstacle_id, const double s, const double t) const {
  /**
   * @brief 检查障碍物ID是否存在
   *
   * C++语法说明：
   * ACHECK是Apollo断言宏
   * 如果find返回end()，说明障碍物不存在
   */
  ACHECK(id_obstacle_map_.find(obstacle_id) != id_obstacle_map_.end());

  /**
   * @brief 获取障碍物轨迹
   */
  const auto& trajectory = id_obstacle_map_.at(obstacle_id)->Trajectory();

  /**
   * @brief 检查轨迹点数量
   *
   * 如果轨迹点少于2个，无法计算速度
   */
  int num_traj_point = trajectory.trajectory_point_size();
  if (num_traj_point < 2) {
    return 0.0;
  }

  /**
   * @brief 检查时间是否在轨迹范围内
   *
   * 如果时间小于起点或大于终点，返回0
   */
  if (t < trajectory.trajectory_point(0).relative_time() ||
      t > trajectory.trajectory_point(num_traj_point - 1).relative_time()) {
    return 0.0;
  }

  /**
   * @brief 二分查找匹配点
   *
   * 找到第一个relative_time >= t的轨迹点
   *
   * C++语法说明：
   * - std::lower_bound：
   *   二分查找下界
   *   返回第一个不小于t的迭代器
   *
   * - lambda比较函数：
   *   比较轨迹点的relative_time与t
   */
  auto matched_it =
      std::lower_bound(trajectory.trajectory_point().begin(),
                       trajectory.trajectory_point().end(), t,
                       [](const common::TrajectoryPoint& p, const double t) {
                         return p.relative_time() < t;
                       });

  /**
   * @brief 提取匹配点的速度和朝向
   */
  double v = matched_it->v();  // 速度大小
  double theta = matched_it->path_point().theta();  // 速度朝向角

  /**
   * @brief 分解速度为x和y分量
   *
   * v_x = v * cos(theta)
   * v_y = v * sin(theta)
   */
  double v_x = v * std::cos(theta);
  double v_y = v * std::sin(theta);

  /**
   * @brief 获取参考线上s位置处的点
   *
   * 使用PathMatcher::MatchToPath将s坐标匹配到参考线
   */
  common::PathPoint obstacle_point_on_ref_line =
      common::math::PathMatcher::MatchToPath(*ptr_reference_line_, s);
  auto ref_theta = obstacle_point_on_ref_line.theta();  // 参考线朝向

  /**
   * @brief 计算沿参考线方向的投影速度
   *
   * 速度向量与参考线朝向的点积
   * 这就是障碍物相对于自车的纵向速度分量
   */
  return std::cos(ref_theta) * v_x + std::sin(ref_theta) * v_y;
}

/**
 * @brief 命名空间结束标记
 *
 * C++语法说明：
 * 两层命名空间的闭合注释
 */
}  // namespace planning
}  // namespace apollo
