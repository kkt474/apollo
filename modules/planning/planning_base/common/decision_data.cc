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
 * @file decision_data.cc
 * @brief 决策数据类实现文件
 *
 * 功能说明：
 * 决策数据类负责管理和处理预测障碍物数据
 * 包括障碍物分类、查询、虚拟障碍物创建等功能
 *
 * 主要实现：
 * 1. IsValidTrajectoryPoint：验证轨迹点有效性
 * 2. IsValidTrajectory：验证轨迹有效性
 * 3. DecisionData构造函数：初始化并分类障碍物
 * 4. GetObstacleById/Type：查询障碍物
 * 5. CreateVirtualObstacle：创建虚拟障碍物
 *
 * C++语法说明：
 * - ::作用域解析运算符：类名::函数名表示静态成员函数
 * - std::lock_guard：RAII方式的互斥锁管理
 * - emplace_back：直接构造元素，避免拷贝
 * - protobuf消息的set_/mutable_/add_方法
 */

#include "modules/planning/planning_base/common/decision_data.h"

#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
/**
 * @brief Apollo外层命名空间
 */
namespace planning {

/**
 * @brief 验证轨迹点是否有效
 *
 * @param point 轨迹点
 * @return bool 是否有效
 *
 * 功能说明：
 * 检查轨迹点的所有数值字段是否为有效值（非NaN）
 * 这是数据预处理的重要步骤，避免无效数据导致规划失败
 *
 * 检查字段：
 * - has_path_point()：是否有路径点
 * - x(), y(), z()：位置坐标
 * - kappa()：曲率
 * - s()：累计距离
 * - dkappa()：曲率变化率
 * - ddkappa()：曲率二阶导数
 * - v()：速度
 * - a()：加速度
 * - relative_time()：相对时间
 *
 * C++语法说明：
 * - !((!point.has_path_point()) || ... )：
 *   双重否定表达"有效"条件
 *   !point.has_path_point()检查"没有路径点"
 *   外层!将其取反，表示"必须有路径点"
 *
 * - std::isnan()：
 *   标准库函数，检查浮点数是否为NaN
 *   NaN表示"不是一个数字"，通常由0/0、inf-inf等运算产生
 *
 * - || 逻辑或运算符：
 *   短路求值，如果第一个条件为true则不再计算后续条件
 *   这里使用||连接所有无效条件
 *
 * - point.path_point().x()：
 *   链式成员访问
 *   path_point()返回const引用，再调用x()
 */
bool DecisionData::IsValidTrajectoryPoint(
    const common::TrajectoryPoint& point) {
  return !((!point.has_path_point()) || std::isnan(point.path_point().x()) ||
           std::isnan(point.path_point().y()) ||
           std::isnan(point.path_point().z()) ||
           std::isnan(point.path_point().kappa()) ||
           std::isnan(point.path_point().s()) ||
           std::isnan(point.path_point().dkappa()) ||
           std::isnan(point.path_point().ddkappa()) || std::isnan(point.v()) ||
           std::isnan(point.a()) || std::isnan(point.relative_time()));
}

/**
 * @brief 验证轨迹是否有效
 *
 * @param trajectory 轨迹
 * @return bool 是否有效
 *
 * 功能说明：
 * 遍历轨迹的所有点，验证每个点是否有效
 * 只要有一个点无效，轨迹就无效
 *
 * C++语法说明：
 * - for (const auto& point : trajectory.trajectory_point())：
 *   范围for循环遍历trajectory_point字段
 *   trajectory.trajectory_point()返回const引用
 *   const auto&避免拷贝，提高效率
 *
 * - trajectory.ShortDebugString()：
 *   protobuf消息的调试方法
 *   返回人类可读的消息摘要字符串
 *   用于日志输出
 */
bool DecisionData::IsValidTrajectory(const prediction::Trajectory& trajectory) {
  for (const auto& point : trajectory.trajectory_point()) {
    if (!IsValidTrajectoryPoint(point)) {
      AERROR << " TrajectoryPoint: " << trajectory.ShortDebugString()
             << " is NOT valid.";
      return false;
    }
  }
  return true;
}

/**
 * @brief 决策数据构造函数
 *
 * @param prediction_obstacles 预测障碍物数据
 * @param reference_line 参考线
 *
 * 功能说明：
 * 1. 遍历所有预测障碍物
 * 2. 对每个障碍物：
 *    - 如果没有轨迹，视为静态障碍物
 *    - 如果有轨迹，验证轨迹有效性，创建动态障碍物
 * 3. 将障碍物添加到相应的列表和map中
 *
 * C++语法说明：
 * - : reference_line_(reference_line)：
 *   初始化列表，用于初始化引用成员变量
 *   引用必须在构造时绑定，不能后来赋值
 *
 * - prediction_obstacles.prediction_obstacle()：
 *   protobuf repeated字段的访问方法
 *   返回const引用，包含所有预测障碍物
 *
 * - std::to_string()：
 *   标准库函数，将数值转换为字符串
 *   perception_obstacle.id()是int64类型
 *
 * - prediction_obstacle.trajectory().empty()：
 *   检查轨迹容器是否为空
 *   empty()比size() == 0更高效
 *
 * - obstacles_.emplace_back(new Obstacle(...))：
 *   emplace_back直接在容器末尾构造元素
 *   参数传递给Obstacle构造函数
 *   相比push_back，避免了先构造临时对象再拷贝
 *
 * - obstacles_.back().get()：
 *   back()返回最后一个元素的引用（unique_ptr）
 *   get()返回unique_ptr管理的原始指针
 *
 * - all_obstacle_.emplace_back(obstacles_.back().get())：
 *   向列表添加原始指针，不拥有所有权
 *
 * - obstacle_map_[perception_id] = obstacles_.back().get()：
 *   向map添加键值对
 *   []运算符会在键不存在时插入默认构造的值
 *
 * - absl::StrCat(perception_id, "_", trajectory_index)：
 *   Abseil库的字符串拼接函数
 *   比std::to_string + +更高效
 *
 * - ++trajectory_index：
 *   前置自增，返回递增后的值
 *   用于为每个轨迹生成唯一ID
 */
DecisionData::DecisionData(
    const prediction::PredictionObstacles& prediction_obstacles,
    const ReferenceLine& reference_line)
    : reference_line_(reference_line) {
  /**
   * @brief 遍历所有预测障碍物
   */
  for (const auto& prediction_obstacle :
       prediction_obstacles.prediction_obstacle()) {
    /**
     * @brief 将障碍物ID转换为字符串
     *
     * 预测障碍物的ID是数值类型
     * 需要转换为字符串用于后续存储和查找
     */
    const std::string perception_id =
        std::to_string(prediction_obstacle.perception_obstacle().id());

    /**
     * @brief 处理没有轨迹的障碍物（静态障碍物）
     *
     * 没有轨迹的障碍物被认为是静态障碍物
     * 直接创建Obstacle对象并添加到各列表和map
     */
    if (prediction_obstacle.trajectory().empty()) {
      obstacles_.emplace_back(new Obstacle(
          perception_id, prediction_obstacle.perception_obstacle()));
      all_obstacle_.emplace_back(obstacles_.back().get());
      practical_obstacle_.emplace_back(obstacles_.back().get());
      static_obstacle_.emplace_back(obstacles_.back().get());
      obstacle_map_[perception_id] = obstacles_.back().get();
      continue;
    }

    /**
     * @brief 处理有轨迹的障碍物（动态障碍物）
     *
     * 一个感知障碍物可能有多个预测轨迹
     * 每个有效轨迹创建一个Obstacle对象
     */
    int trajectory_index = 0;
    for (const auto& trajectory : prediction_obstacle.trajectory()) {
      /**
       * @brief 验证轨迹有效性
       *
       * 无效轨迹会被跳过，不会创建障碍物
       */
      if (!IsValidTrajectory(trajectory)) {
        AERROR << "obj:" << perception_id;
        continue;
      }

      /**
       * @brief 为每个轨迹创建唯一的障碍物ID
       *
       * 格式：perception_id_trajectory_index
       * 例如：1234_0, 1234_1, 1234_2
       */
      const std::string obstacle_id =
          absl::StrCat(perception_id, "_", trajectory_index);
      obstacles_.emplace_back(new Obstacle(
          obstacle_id, prediction_obstacle.perception_obstacle(), trajectory));
      all_obstacle_.emplace_back(obstacles_.back().get());
      practical_obstacle_.emplace_back(obstacles_.back().get());
      dynamic_obstacle_.emplace_back(obstacles_.back().get());
      obstacle_map_[obstacle_id] = obstacles_.back().get();
      ++trajectory_index;
    }
  }
}

/**
 * @brief 根据ID获取障碍物
 *
 * @param id 障碍物ID
 * @return Obstacle* 障碍物指针，未找到返回nullptr
 *
 * 功能说明：
 * 在obstacle_map_中查找指定ID的障碍物
 * 这是O(1)时间复杂度的查找操作
 *
 * C++语法说明：
 * - std::lock_guard<std::mutex> lock(mutex_)：
 *   RAII方式的互斥锁管理
 *   构造时自动加锁，析构时自动解锁
 *   即使发生异常也能确保锁被释放
 *
 * - common::util::FindPtrOrNull()：
 *   Apollo常用工具函数
 *   在map中查找键，找到返回对应值，未找到返回nullptr
 */
Obstacle* DecisionData::GetObstacleById(const std::string& id) {
  std::lock_guard<std::mutex> lock(mutex_);
  return common::util::FindPtrOrNull(obstacle_map_, id);
}

/**
 * @brief 根据类型获取障碍物列表
 *
 * @param type 虚拟对象类型
 * @return std::vector<Obstacle*> 障碍物指针列表
 *
 * 功能说明：
 * 1. 获取指定类型的所有障碍物ID
 * 2. 根据ID获取对应的障碍物指针
 * 3. 过滤掉nullptr（已删除的障碍物）
 *
 * C++语法说明：
 * - std::lock_guard<std::mutex> lock(transaction_mutex_)：
 *   使用transaction_mutex_保护虚拟障碍物相关操作
 *
 * - std::unordered_set<std::string>：
 *   无序集合，存储ID列表
 *
 * - GetObstacleIdByType()：
 *   返回指定类型的所有障碍物ID
 *
 * - ret.back() == nullptr：
 *   获取返回向量的最后一个元素
 *   检查是否为空指针
 *
 * - ret.pop_back()：
 *   删除最后一个元素
 *   用于移除无效（nullptr）的障碍物
 */
std::vector<Obstacle*> DecisionData::GetObstacleByType(
    const VirtualObjectType& type) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);

  std::unordered_set<std::string> ids = GetObstacleIdByType(type);
  if (ids.empty()) {
    return std::vector<Obstacle*>();
  }
  std::vector<Obstacle*> ret;
  for (const std::string& id : ids) {
    ret.emplace_back(GetObstacleById(id));
    if (ret.back() == nullptr) {
      AERROR << "Ignore. can't find obstacle by id: " << id;
      ret.pop_back();
    }
  }
  return ret;
}

/**
 * @brief 根据类型获取障碍物ID集合
 *
 * @param type 虚拟对象类型
 * @return std::unordered_set<std::string> 障碍物ID集合
 *
 * 功能说明：
 * 在virtual_obstacle_id_map_中查找指定类型的所有障碍物ID
 *
 * C++语法说明：
 * - common::util::FindWithDefault()：
 *   Apollo常用工具函数
 *   在map中查找键，找到返回对应值，未找到返回默认值
 *   默认值是空集合{}
 */
std::unordered_set<std::string> DecisionData::GetObstacleIdByType(
    const VirtualObjectType& type) {
  std::lock_guard<std::mutex> lock(mutex_);
  return common::util::FindWithDefault(virtual_obstacle_id_map_, type, {});
}

/**
 * @brief 获取所有静态障碍物
 *
 * @return const std::vector<Obstacle*>& 静态障碍物引用
 */
const std::vector<Obstacle*>& DecisionData::GetStaticObstacle() const {
  return static_obstacle_;
}

/**
 * @brief 获取所有动态障碍物
 *
 * @return const std::vector<Obstacle*>& 动态障碍物引用
 */
const std::vector<Obstacle*>& DecisionData::GetDynamicObstacle() const {
  return dynamic_obstacle_;
}

/**
 * @brief 获取所有虚拟障碍物
 *
 * @return const std::vector<Obstacle*>& 虚拟障碍物引用
 */
const std::vector<Obstacle*>& DecisionData::GetVirtualObstacle() const {
  return virtual_obstacle_;
}

/**
 * @brief 获取所有实际障碍物
 *
 * @return const std::vector<Obstacle*>& 实际障碍物引用
 */
const std::vector<Obstacle*>& DecisionData::GetPracticalObstacle() const {
  return practical_obstacle_;
}

/**
 * @brief 获取所有障碍物
 *
 * @return const std::vector<Obstacle*>& 所有障碍物引用
 */
const std::vector<Obstacle*>& DecisionData::GetAllObstacle() const {
  return all_obstacle_;
}

/**
 * @brief 根据参考点创建虚拟障碍物
 *
 * @param point 参考点
 * @param type 虚拟对象类型
 * @param id 输出参数，创建的障碍物ID
 * @return bool 是否创建成功
 *
 * 功能说明：
 * 1. 将参考点转换为SL坐标
 * 2. 创建虚拟障碍物的包围盒
 * 3. 调用CreateVirtualObstacle创建虚拟障碍物
 *
 * C++语法说明：
 * - reference_line_.XYToSL(point, &sl_point)：
 *   XYToSL将笛卡尔坐标转换为SL坐标
 *   第二个参数是输出参数，使用指针
 *   返回bool表示转换是否成功
 *
 * - FLAGS_virtual_stop_wall_length：
 *   GFlags定义的全局标志变量
 *   用于控制虚拟停止墙的长度
 *
 * - reference_line_.GetReferencePoint(box_center_s)：
 *   根据s坐标获取参考线上的点
 *
 * - reference_line_.GetLaneWidth()：
 *   获取指定s处的车道宽度
 *   返回左右宽度
 */
bool DecisionData::CreateVirtualObstacle(const ReferencePoint& point,
                                         const VirtualObjectType& type,
                                         std::string* const id) {
  common::SLPoint sl_point;
  if (!reference_line_.XYToSL(point, &sl_point)) {
    return false;
  }
  double obstacle_s = sl_point.s();
  const double box_center_s = obstacle_s + FLAGS_virtual_stop_wall_length / 2.0;
  auto box_center = reference_line_.GetReferencePoint(box_center_s);
  double heading = reference_line_.GetReferencePoint(obstacle_s).heading();
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  reference_line_.GetLaneWidth(obstacle_s, &lane_left_width, &lane_right_width);
  common::math::Box2d box(box_center, heading, FLAGS_virtual_stop_wall_length,
                          lane_left_width + lane_right_width);
  return CreateVirtualObstacle(box, type, id);
}

/**
 * @brief 根据S坐标创建虚拟障碍物
 *
 * @param point_s 沿参考线的S坐标
 * @param type 虚拟对象类型
 * @param id 输出参数，创建的障碍物ID
 * @return bool 是否创建成功
 *
 * 功能说明：
 * 1. 计算虚拟障碍物包围盒的中心s坐标
 * 2. 获取参考点和朝向
 * 3. 获取车道宽度
 * 4. 创建Box2d包围盒
 * 5. 调用CreateVirtualObstacle创建虚拟障碍物
 */
bool DecisionData::CreateVirtualObstacle(const double point_s,
                                         const VirtualObjectType& type,
                                         std::string* const id) {
  const double box_center_s = point_s + FLAGS_virtual_stop_wall_length / 2.0;
  auto box_center = reference_line_.GetReferencePoint(box_center_s);
  double heading = reference_line_.GetReferencePoint(point_s).heading();
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  reference_line_.GetLaneWidth(point_s, &lane_left_width, &lane_right_width);
  common::math::Box2d box(box_center, heading, FLAGS_virtual_stop_wall_length,
                          lane_left_width + lane_right_width);
  return CreateVirtualObstacle(box, type, id);
}

/**
 * @brief 根据包围盒创建虚拟障碍物（内部实现）
 *
 * @param obstacle_box 障碍物包围盒
 * @param type 虚拟对象类型
 * @param id 输出参数，创建的障碍物ID
 * @return bool 是否创建成功
 *
 * 功能说明：
 * 1. 创建PerceptionObstacle消息
 * 2. 设置障碍物属性（位置、朝向、尺寸等）
 * 3. 设置为UNKNOWN_UNMOVABLE类型（虚拟障碍物）
 * 4. 添加多边形顶点
 * 5. 创建Obstacle对象并添加到各列表
 * 6. 更新virtual_obstacle_id_map_
 *
 * C++语法说明：
 * - std::lock_guard<std::mutex> transaction_lock(transaction_mutex_)：
 *   std::lock_guard<std::mutex> lock(mutex_)：
 *   双重锁保护，确保虚拟障碍物创建的原子性
 *
 * - perception_obstacle.set_id()：
 *   protobuf的setter方法，设置字段值
 *
 * - perception_obstacle.mutable_position()：
 *   protobuf的mutable方法，返回可变指针
 *   用于设置嵌套消息字段
 *
 * - perception_obstacle.mutable_position()->set_x()：
 *   链式调用
 *   mutable_position()返回Position*指针
 *   ->set_x()设置x坐标
 *
 * - perception_obstacle.set_theta()：
 *   设置朝向角（弧度制）
 *
 * - perception_obstacle.set_type()：
 *   设置感知障碍物类型
 *   UNKNOWN_UNMOVABLE表示未知类型的静止障碍物
 *
 * - obstacle_box.GetAllCorners(&corner_points)：
 *   获取包围盒的所有角点
 *   corner_points是输出参数
 *
 * - perception_obstacle.add_polygon_point()：
 *   protobuf的add方法
 *   向repeated字段添加新元素，返回新元素的指针
 *
 * - *id = std::to_string(perception_obstacle.id())：
 *   解引用id指针，赋值
 *   将新创建的障碍物ID返回给调用者
 *
 * - obstacles_.emplace_back(new Obstacle(*id, perception_obstacle))：
 *   使用字符串ID构造Obstacle
 *
 * - virtual_obstacle_id_map_[type].insert(*id)：
 *   向指定类型的ID集合插入新ID
 *   []运算符会创建空集合如果不存在
 */
bool DecisionData::CreateVirtualObstacle(
    const common::math::Box2d& obstacle_box, const VirtualObjectType& type,
    std::string* const id) {
  std::lock_guard<std::mutex> transaction_lock(transaction_mutex_);
  std::lock_guard<std::mutex> lock(mutex_);

  perception::PerceptionObstacle perception_obstacle;
  perception_obstacle.set_id(virtual_obstacle_.size() + 1);
  perception_obstacle.mutable_position()->set_x(obstacle_box.center().x());
  perception_obstacle.mutable_position()->set_y(obstacle_box.center().y());
  perception_obstacle.set_theta(obstacle_box.heading());
  perception_obstacle.mutable_velocity()->set_x(0);
  perception_obstacle.mutable_velocity()->set_y(0);
  perception_obstacle.set_length(obstacle_box.length());
  perception_obstacle.set_width(obstacle_box.width());
  perception_obstacle.set_height(FLAGS_virtual_stop_wall_height);
  perception_obstacle.set_type(
      perception::PerceptionObstacle::UNKNOWN_UNMOVABLE);
  perception_obstacle.set_tracking_time(1.0);

  std::vector<common::math::Vec2d> corner_points;
  obstacle_box.GetAllCorners(&corner_points);
  for (const auto& corner_point : corner_points) {
    auto* point = perception_obstacle.add_polygon_point();
    point->set_x(corner_point.x());
    point->set_y(corner_point.y());
  }
  *id = std::to_string(perception_obstacle.id());
  obstacles_.emplace_back(new Obstacle(*id, perception_obstacle));
  all_obstacle_.emplace_back(obstacles_.back().get());
  virtual_obstacle_.emplace_back(obstacles_.back().get());
  static_obstacle_.emplace_back(obstacles_.back().get());

  virtual_obstacle_id_map_[type].insert(*id);
  obstacle_map_[*id] = obstacles_.back().get();
  return true;
}

}  // namespace planning
}  // namespace apollo
