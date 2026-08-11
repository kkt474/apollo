/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
 * @file open_space_fallback_decider_park.cc
 *
 * @brief 开放空间后备决策器（停车场场景）实现文件
 *
 * 功能说明：
 * 本文件实现了 OpenSpaceFallbackDeciderPark 类
 * 负责开放空间场景下的轨迹安全回退决策
 * 当检测到轨迹与障碍物碰撞时，生成安全的回退轨迹
 *
 * 核心概念：
 * - Fallback Trajectory（回退轨迹）：当检测到碰撞时的安全轨迹
 * - 静态碰撞（Static Collision）：与静止障碍物的碰撞
 * - 动态碰撞（Dynamic Collision）：与移动障碍物的碰撞
 * - 二次方程求解：计算停车距离
 *
 * C++语法说明：
 * - namespace：命名空间，避免命名冲突
 * - std::shared_ptr：智能指针，引用计数管理
 * - protobuf消息操作：mutable_xxx(), set_xxx()
 * - std::vector：动态数组容器
 **/

#include "modules/planning/tasks/open_space_fallback_decider_park/open_space_fallback_decider_park.h"

/**
 * @brief Apollo命名空间开始
 */
namespace apollo {
/**
 * @brief 规划模块命名空间
 */
namespace planning {

/**
 * @brief 类型别名声明
 *
 * C++语法说明：
 * - using：类型别名声明，等价于typedef
 * - apollo::common::Status：Apollo通用状态类型
 * - apollo::common::TrajectoryPoint：轨迹点结构
 * - apollo::common::math::Box2d/Polygon2d/Vec2d：2D数学工具
 */
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::math::Box2d;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;

/**
 * @brief 初始化函数
 *
 * @param config_dir 配置目录路径
 * @param name 任务名称
 * @param injector 依赖注入器指针
 * @return bool 初始化成功返回true
 *
 * 功能说明：
 * 1. 调用基类Decider的Init方法进行基础初始化
 * 2. 加载开放空间后备决策器配置
 * 3. 获取碰撞距离参数
 *
 * C++语法说明：
 * - const std::string& config_dir：
 *   常量引用参数，避免拷贝
 *
 * - const std::shared_ptr<DependencyInjector>& injector：
 *   shared_ptr智能指针的常量引用
 *
 * - Decider::Init(config_dir, name, injector)：
 *   调用基类Decider的Init方法
 *
 * - Decider::LoadConfig<T>(&config_)：
 *   模板方法加载protobuf配置
 *
 * - open_space_fallback_collision_distance_：
 *   成员变量，存储碰撞距离阈值
 */
bool OpenSpaceFallbackDeciderPark::Init(
    const std::string& config_dir, const std::string& name,
    const std::shared_ptr<DependencyInjector>& injector) {
  /**
   * @brief 调用基类初始化
   */
  if (!Decider::Init(config_dir, name, injector)) {
    return false;
  }
  /**
   * @brief 加载后备决策器配置
   */
  if (!Decider::LoadConfig<OpenSpaceFallBackDeciderParkConfig>(&config_)) {
    AERROR << "Failed to load config file";
    return false;
  } else {
    /**
     * @brief 获取碰撞距离配置
     *
     * config_.open_space_fallback_collision_distance()：
     *   protobuf配置的字段访问方法
     *   返回用于判断碰撞的距离阈值
     */
    open_space_fallback_collision_distance_ =
        config_.open_space_fallback_collision_distance();
    return true;
  }
}

/**
 * @brief 二次方程求较小根
 *
 * @param a 二次项系数 ax²
 * @param b 一次项系数 bx
 * @param c 常数项
 * @param sol 输出参数，求得的较小根
 * @return bool 求解成功返回true
 *
 * 功能说明：
 * 求解二次方程 ax² + bx + c = 0
 * 返回两个根中绝对值较小的那个
 *
 * 数学公式：
 * x = (-b ± √(b²-4ac)) / 2a
 *
 * C++语法说明：
 * - const double a/b/c：
 *   常量引用参数，承诺不修改
 *
 * - double* sol：
 *   指针参数，用于输出结果
 *
 * - std::abs(std::sqrt(tmp))：
 *   std::abs：绝对值函数
 *   std::sqrt：平方根函数
 *
 * - kEpsilon = 1e-6：
 *   epsilon容差值，用于数值稳定性
 */
bool OpenSpaceFallbackDeciderPark::QuardraticFormulaLowerSolution(const double a,
                                                              const double b,
                                                              const double c,
                                                              double* sol) {
  /**
   * @brief 二次方程 ax² + bx + c = 0，求较小根
   *
   * TODO(QiL): use const from common::math
   */
  const double kEpsilon = 1e-6;  /**< 数值稳定性容差 */
  *sol = 0.0;  /**< 初始化输出参数 */
  /**
   * @brief 检查a是否为零（退化为一次方程）
   */
  if (std::abs(a) < kEpsilon) {
    return false;
  }

  /**
   * @brief 计算判别式 Δ = b² - 4ac
   */
  double tmp = b * b - 4 * a * c;
  /**
   * @brief 检查判别式是否非负
   */
  if (tmp < kEpsilon) {
    return false;
  }
  /**
   * @brief 计算两个根
   *
   * x1 = (-b + √Δ) / 2a
   * x2 = (-b - √Δ) / 2a
   */
  double sol1 = (-b + std::sqrt(tmp)) / (2.0 * a);
  double sol2 = (-b - std::sqrt(tmp)) / (2.0 * a);

  /**
   * @brief 返回绝对值较小的根
   */
  *sol = std::abs(std::min(sol1, sol2));
  ADEBUG << "QuardraticFormulaLowerSolution finished with sol: " << *sol
         << "sol1: " << sol1 << ", sol2: " << sol2 << "a: " << a << "b: " << b
         << "c: " << c;
  return true;
}

/**
 * @brief 主处理函数
 *
 * @param frame 当前规划帧
 * @return Status 处理状态
 *
 * 功能说明：
 * 1. 检测当前轨迹是否与障碍物碰撞
 * 2. 如果碰撞，生成安全的回退轨迹
 * 3. 处理静态和动态障碍物碰撞
 *
 * C++语法说明：
 * - Frame* frame：
 *   裸指针，指向当前规划帧
 *
 * - std::vector<std::vector<Box2d>>：
 *   二维向量，存储预测的边界框
 *
 * - std::vector<Obstacle*>：
 *   动态数组，存储障碍物指针
 */
Status OpenSpaceFallbackDeciderPark::Process(Frame* frame) {
  /**
   * @brief 打印调试信息
   */
  AINFO << "frame->open_space_info().chosen_partitioned_trajectory(): " <<
        frame->open_space_info().chosen_partitioned_trajectory().first.size();
  AINFO << "frame->open_space_info().optimizer_trajectory_data(): " <<
        frame->open_space_info().optimizer_trajectory_data().size();

  /**
   * @brief 声明变量
   */
  std::vector<std::vector<common::math::Box2d>> predicted_bounding_rectangles;  /**< 动态障碍物预测边界框 */
  std::vector<Obstacle*> static_obstacles;  /**< 静态障碍物列表 */
  int first_collision_index = 0;  /**< 首次碰撞的轨迹点索引 */
  int fallback_start_index = 0;  /**< 回退起始索引 */
  /**
   * @brief mayaochang add
   */
  bool is_collision_with_static_obstacle = false;  /**< 是否与静态障碍物碰撞 */
  bool is_collision_with_dynamic_obstacle = false;  /**< 是否与动态障碍物碰撞 */

  /**
   * @brief 构建动态障碍物预测环境
   *
   * OpenSpaceFallbackUtil::BuildPredictedEnvironment(...)：
   *   工具函数，根据车辆位置和障碍物生成预测边界框
   *
   * Vec2d(x, y)：
   *   构造2D向量，表示车辆位置
   *
   * frame->obstacles()：
   *   获取当前帧的障碍物列表
   *
   * config_.open_space_prediction_time_period()：
   *   预测时间范围
   *
   * config_.collision_check_range()：
   *   碰撞检测范围
   */
  OpenSpaceFallbackUtil::BuildPredictedEnvironment(
      Vec2d(frame->vehicle_state().x(), frame->vehicle_state().y()),
      frame->obstacles(),
      predicted_bounding_rectangles,
      config_.open_space_prediction_time_period(),   // 5s
      config_.collision_check_range());   //  10.0

  /**
   * @brief 构建静态障碍物环境
   */
  OpenSpaceFallbackUtil::BuildStaticObstacleEnvironment(
      Vec2d(frame->vehicle_state().x(), frame->vehicle_state().y()),
      frame->obstacles(),
      static_obstacles,
      config_.collision_check_range());

  ADEBUG << "Numbers of obstsacles are: " << frame->obstacles().size();
  ADEBUG << "Numbers of predicted bounding rectangles of dynamic obstacle are: "
         << predicted_bounding_rectangles[0].size()
         << " and : " << predicted_bounding_rectangles.size()
         << "Numbers of static obstsacles are:" << static_obstacles.size();

  /**
   * @brief 检查是否有生成轨迹
   */
  if (frame->open_space_info().chosen_partitioned_trajectory().first.empty()) {
    AERROR << "No trajectory is generated for fallback";

    /**
     * @brief 无轨迹时的处理
     *
     * 设置相对停车时间
     */
    static constexpr double relative_stop_time = 0.1;  /**< 停车点时间间隔 */

    /**
     * @brief 获取可变的分段轨迹
     */
    auto fallback_tra_pair =
        frame_->mutable_open_space_info()->
            mutable_chosen_partitioned_trajectory();
    fallback_tra_pair->first.clear();  /**< 清空轨迹 */

    /**
     * @brief 获取当前速度
     */
    auto current_speed = injector_->vehicle_state()->
        vehicle_state().linear_velocity();
    AINFO << "open_space_fallback_collision_distance_: " << open_space_fallback_collision_distance_;

    /**
     * @brief 计算停车减速度
     *
     * 使用物理公式：v² = 2as
     * 其中 v = current_speed, s = collision_distance
     * 得 a = -v² / 2s
     *
     * std::max(std::min(...))：
     *   限制减速度在合理范围内
     *   上限 -0.2 m/s²（不能太急）
     *   下限 -4.0 m/s²（最大减速度）
     */
    double stop_deceleration_for_static_collision =
        std::max(std::min(-1 * current_speed * current_speed /
                      (2 * (open_space_fallback_collision_distance_ - 1e-12)),
                      -0.2),
                  -4.0);
    AINFO << "stop_deceleration_for_static_collision: " << stop_deceleration_for_static_collision;

    /**
     * @brief 创建回退轨迹候选
     */
    TrajGearPair fallback_trajectory_pair_candidate;
    fallback_trajectory_pair_candidate.second =
        frame_->local_view().chassis->gear_location();

    /**
     * @brief 计算回退轨迹
     */
    CalculateFallbackTrajectory(fallback_trajectory_pair_candidate,
                                stop_deceleration_for_static_collision,
                                relative_stop_time, fallback_tra_pair);

    /**
     * @brief 更新碰撞距离
     */
    open_space_fallback_collision_distance_ = std::max (1e-6,
        open_space_fallback_collision_distance_ - fabs(fallback_tra_pair->first[1].path_point().s()));

    AINFO << "open_space_fallback_collision_ditance_: " << open_space_fallback_collision_distance_;

    /**
     * @brief 检查回退轨迹是否无碰撞
     */
    TrajGearPair fallback_trajectory =
        frame_->open_space_info().chosen_partitioned_trajectory();
    if (!OpenSpaceFallbackUtil::IsCollisionFreeTrajectory(
            fallback_trajectory, predicted_bounding_rectangles,
            static_obstacles, &fallback_start_index, &first_collision_index,
            is_collision_with_static_obstacle,
            is_collision_with_dynamic_obstacle,
            config_.open_space_fallback_collision_time_buffer())) {
      AINFO << " fallback trajectory still has collision with obstacles";
      /**
       * @brief 如果仍有碰撞，使用最大减速度停车
       */
      CalculateFallbackTrajectory(fallback_trajectory, -4.0,
                                  relative_stop_time, fallback_tra_pair);
    }
    return Status::OK();
  }

  /**
   * @brief 重置碰撞距离
   */
  open_space_fallback_collision_distance_ =
        config_.open_space_fallback_collision_distance();

  /**
   * @brief 检查当前轨迹是否无碰撞
   *
   * IsCollisionFreeTrajectory：
   *   检查分段轨迹是否与障碍物碰撞
   *   输出首次碰撞索引和碰撞类型
   */
  if (!OpenSpaceFallbackUtil::IsCollisionFreeTrajectory(
          frame->open_space_info().chosen_partitioned_trajectory(),
          predicted_bounding_rectangles, static_obstacles,
          &fallback_start_index, &first_collision_index,
          is_collision_with_static_obstacle,
          is_collision_with_dynamic_obstacle,
          config_.open_space_fallback_collision_time_buffer())) {
    AINFO << "trajectory still has collision with obstacles";
    /**
     * @brief 碰撞时的处理
     *
     * 基于当前分段轨迹生成回退轨迹
     * 在安全距离内将车速降至零
     */

    /**
     * @brief 获取碰撞点信息
     */
    TrajGearPair fallback_trajectory_pair_candidate =
        frame->open_space_info().chosen_partitioned_trajectory();
    const auto future_collision_point =
        fallback_trajectory_pair_candidate.first[first_collision_index];

    /**
     * @brief 获取回退起始点
     *
     * Fallback从当前位置开始，保持当前车速
     */
    auto fallback_start_point =
        fallback_trajectory_pair_candidate.first[fallback_start_index];
    const auto& vehicle_state = injector_->vehicle_state()->vehicle_state();

    /**
     * @brief 计算停车距离
     *
     * TODO(QiL): move 1.0 to configs
     *
     * 根据档位调整停车距离方向：
     * - GEAR_DRIVE：前进方向
     * - GEAR_REVERSE：后退方向
     *
     * 停车距离 = 碰撞点s - 起始点s - 与障碍物距离
     */
    double stop_distance =
        fallback_trajectory_pair_candidate.second == canbus::Chassis::GEAR_DRIVE
            ? std::max(future_collision_point.path_point().s() -
                           fallback_start_point.path_point().s() - config_.distance_to_obs(),
                       0.0)
            : std::min(future_collision_point.path_point().s() -
                           fallback_start_point.path_point().s() + config_.distance_to_obs(),
                       0.0);

    ADEBUG << "stop distance : " << stop_distance;

    /**
     * @brief 获取车辆参数
     */
    const auto& vehicle_param =
        common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
    const double max_adc_stop_speed =
        vehicle_param.max_abs_speed_when_stopped();  /**< 停车速度阈值 */

    /**
     * @brief 静态障碍物碰撞处理
     */
    if (is_collision_with_static_obstacle) {
      AINFO << "collision with static obstacle";

      /**
       * @brief 清空所有轨迹数据
       */
      frame_->mutable_open_space_info()->
          mutable_optimizer_trajectory_data()->clear();
      frame_->mutable_open_space_info()->
          mutable_path_planning_trajectory_result()->clear();
      frame_->mutable_open_space_info()->
          mutable_interpolated_trajectory_result()->clear();
      frame_->mutable_open_space_info()->
          mutable_partitioned_trajectories()->clear();
      frame_->mutable_open_space_info()->
          mutable_chosen_partitioned_trajectory()->first.clear();

      /**
       * @brief 设置相对停车时间
       */
      const auto& open_space_fallback_collision_distance =
          config_.open_space_fallback_collision_distance();
      static constexpr double relative_stop_time = 0.1;

      double relative_time = 0.0;
      double current_s_distance = 0.0;
      auto fallback_tra_pair =
          frame_->mutable_open_space_info()->
          mutable_chosen_partitioned_trajectory();
      fallback_tra_pair->first.clear();
      double stop_deceleration_for_static_collision = 0;

      auto current_speed = vehicle_state.linear_velocity();
      AINFO << "open_space_fallback_collision_distance: "
            << open_space_fallback_collision_distance
            << "stop_distance: " << stop_distance;

      /**
       * @brief 判断碰撞距离是否足够
       */
      if (open_space_fallback_collision_distance > std::abs(stop_distance)) {
        AINFO << "The collision distance with static obstacle is too small, "
                  "distance is :"
               << stop_distance << "!";
        /**
         * @brief 距离太小，直接使用最大减速度
         */
        stop_deceleration_for_static_collision = -4.0;
      } else {
        AINFO << "The collision distance with static obstacle is large, stop "
                  "with set distance is :"
               << open_space_fallback_collision_distance << "!";
        /**
         * @brief 计算停车减速度
         *
         * a = -v² / 2s
         */
        stop_deceleration_for_static_collision =
            std::max(-1 * current_speed * current_speed /
                         (2 * (open_space_fallback_collision_distance - 1e-6)),
                     -4.0);
        AINFO << "current_speed: " << current_speed
              << "open_space_fallback_collision_distance: "
              << open_space_fallback_collision_distance;
      }
      AINFO << "stop deceleration for static collision is : "
            << stop_deceleration_for_static_collision
            << "relative_stop_time: " << relative_stop_time;

      /**
       * @brief 根据配置选择回退轨迹计算方法
       */
      if (config_.is_use_trajectory_pose()) {
        CalculateFallbackTrajectoryWithTrajectoryPose(
                                  fallback_trajectory_pair_candidate,
                                  fallback_start_point, future_collision_point, stop_distance,
                                  relative_stop_time, fallback_tra_pair);
      } else {
        CalculateFallbackTrajectory(fallback_trajectory_pair_candidate,
                                  open_space_fallback_collision_distance,
                                  relative_stop_time, fallback_tra_pair);
      }
      AINFO << "fallback trajectory has collision with obstacles";

      /**
       * @brief 再次检查碰撞
       */
      TrajGearPair fallback_trajectory =
          frame_->open_space_info().chosen_partitioned_trajectory();
      AINFO << "fallback trajectory has collision with obstacles";
      if (!OpenSpaceFallbackUtil::IsCollisionFreeTrajectory(
              fallback_trajectory, predicted_bounding_rectangles,
              static_obstacles, &fallback_start_index, &first_collision_index,
              is_collision_with_static_obstacle,
              is_collision_with_dynamic_obstacle,
              config_.open_space_fallback_collision_time_buffer())) {
        AINFO << " fallback trajectory still has collision with obstacles";
        /**
         * @brief 使用最大减速度紧急停车
         */
        CalculateFallbackTrajectory(fallback_trajectory, -4.0,
                                    relative_stop_time, fallback_tra_pair);
      }
    }

    /**
     * @brief 动态障碍物碰撞处理
     */
    if (is_collision_with_dynamic_obstacle) {
      auto fallback_tra_pair =
          frame_->mutable_open_space_info()->
          mutable_chosen_partitioned_trajectory();
      fallback_tra_pair->first.clear();

      /**
       * @brief 设置起始点速度为当前车速
       */
      fallback_start_point.set_v(vehicle_state.linear_velocity());

      /**
       * @brief 存储碰撞点信息
       */
      *(frame_->mutable_open_space_info()->mutable_future_collision_point()) =
          future_collision_point;

      /**
       * @brief 计算最小停车距离
       *
       * s = v² / 2a
       * 其中 v = 起始速度, a = 最大加速度
       */
      double min_stop_distance =
          0.5 * fallback_start_point.v() * fallback_start_point.v() / 4.0;

      /**
       * @brief 获取车辆最大加速度/减速度
       */
      const double vehicle_max_acc = 4.0;   /**< 最大加速度 */
      const double vehicle_max_dec =
          -4.0;  /**< 最大减速度 */

      double stop_deceleration = 0.0;

      /**
       * @brief 根据档位计算停车减速度
       *
       * 使用公式：a = -v² / 2s
       */
      if (fallback_trajectory_pair_candidate.second ==
          canbus::Chassis::GEAR_REVERSE) {
        stop_deceleration =
            std::min(fallback_start_point.v() * fallback_start_point.v() /
                         (2.0 * (stop_distance + 1e-6)),
                     vehicle_max_acc);
        stop_distance = std::min(-1 * min_stop_distance, stop_distance);
      } else {
        stop_deceleration =
            std::max(-fallback_start_point.v() * fallback_start_point.v() /
                         (2.0 * (stop_distance + 1e-6)),
                     vehicle_max_dec);
        stop_distance = std::max(min_stop_distance, stop_distance);
      }

      AINFO << "stop_deceleration: " << stop_deceleration;

      /**
       * @brief 在轨迹上搜索停车点索引
       */
      int stop_index = fallback_start_index;

      for (int i = fallback_start_index;
           i < fallback_trajectory_pair_candidate.first.NumOfPoints(); ++i) {
        if (std::abs(
                fallback_trajectory_pair_candidate.first[i].path_point().s()) >=
            std::abs(fallback_start_point.path_point().s() + stop_distance)) {
          stop_index = i;
          break;
        }
      }

      AINFO << "stop index before is: " << stop_index
             << "; fallback_start index before is: " << fallback_start_index;

      /**
       * @brief 修改回退起始点的速度和加速度
       */
      for (int i = 0; i < fallback_start_index; ++i) {
        fallback_trajectory_pair_candidate.first[i].set_v(
            fallback_start_point.v());
        fallback_trajectory_pair_candidate.first[i].set_a(stop_deceleration);
      }

      /**
       * @brief TODO(QiL): refine the logic and remove redundant code, change 0.5 to
       * from loading optimizer configs
       */

      /**
       * @brief 停车点在回退起始点之前
       */
      if (fallback_start_index >= stop_index) {
        /**
         * @brief 1. 设置回退起始速度为0，加速度为最大加速度
         */
        AINFO << "Stop distance within safety buffer, stop now!";
        fallback_start_point.set_v(0.0);
        fallback_start_point.set_a(0.0);
        fallback_trajectory_pair_candidate.first[stop_index].set_v(0.0);
        fallback_trajectory_pair_candidate.first[stop_index].set_a(0.0);

        /**
         * @brief 2. 裁剪停车点之后的所有轨迹点
         *
         * .erase(begin + index)：
         *   删除指定位置之后的所有元素
         */
        fallback_trajectory_pair_candidate.first.erase(
            fallback_trajectory_pair_candidate.first.begin() + stop_index + 1,
            fallback_trajectory_pair_candidate.first.end());

        /**
         * @brief 3. 追加相同位置但零速度的轨迹点
         */
        for (int i = 0; i < 20; ++i) {
          common::TrajectoryPoint trajectory_point(
              fallback_trajectory_pair_candidate.first[stop_index]);
          trajectory_point.set_relative_time(
              i * 0.5 + 0.5 +
              fallback_trajectory_pair_candidate.first[stop_index]
                  .relative_time());
          fallback_trajectory_pair_candidate.first.AppendTrajectoryPoint(
              trajectory_point);
        }

        *(frame_->mutable_open_space_info()->mutable_fallback_trajectory()) =
            fallback_trajectory_pair_candidate;

        return Status::OK();
      }

      AINFO << "before change, size : "
             << fallback_trajectory_pair_candidate.first.size()
             << ", first index information : "
             << fallback_trajectory_pair_candidate.first[0].DebugString()
             << ", second index information : "
             << fallback_trajectory_pair_candidate.first[1].DebugString();

      /**
       * @brief 停车点在回退起始点之后
       *
       * 逐点计算新的时间、速度和加速度
       */
      for (int i = fallback_start_index; i <= stop_index; ++i) {
        double new_relative_time = 0.0;
        double temp_v = 0.0;
        /**
         * @brief 二次方程常数项
         *
         * 由运动学公式推导：
         * s = v₀t + 0.5 * a * t²
         * 2s = 2v₀t + at²
         * at² + 2v₀t - 2s = 0
         * 其中 a = stop_deceleration, v₀ = fallback_start_point.v()
         */
        double c =
            -2.0 * fallback_trajectory_pair_candidate.first[i].path_point().s();

        /**
         * @brief 求解二次方程得到时间
         */
        if (QuardraticFormulaLowerSolution(stop_deceleration,
                                           2.0 * fallback_start_point.v(), c,
                                           &new_relative_time) &&
            std::abs(
                fallback_trajectory_pair_candidate.first[i].path_point().s()) <=
                std::abs(stop_distance)) {
          ADEBUG << "new_relative_time" << new_relative_time;
          /**
           * @brief 计算新速度
           *
           * v = v₀ + at
           */
          temp_v =
              fallback_start_point.v() + stop_deceleration * new_relative_time;

          /**
           * @brief 速度限幅
           */
          if (std::abs(temp_v) < 1.0) {
            fallback_trajectory_pair_candidate.first[i].set_v(temp_v);
          } else {
            fallback_trajectory_pair_candidate.first[i].set_v(
                temp_v / std::abs(temp_v) * 1.0);
          }
          fallback_trajectory_pair_candidate.first[i].set_a(stop_deceleration);
          fallback_trajectory_pair_candidate.first[i].set_relative_time(
              new_relative_time);
        } else {
          if (i != 0) {
            /**
             * @brief 复制前一轨迹点
             */
            fallback_trajectory_pair_candidate.first[i]
                .mutable_path_point()
                ->CopyFrom(fallback_trajectory_pair_candidate.first[i - 1]
                               .path_point());
            fallback_trajectory_pair_candidate.first[i].set_v(0.0);
            fallback_trajectory_pair_candidate.first[i].set_a(0.0);
            fallback_trajectory_pair_candidate.first[i].set_relative_time(
                fallback_trajectory_pair_candidate.first[i - 1]
                    .relative_time() +
                0.5);
          } else {
            fallback_trajectory_pair_candidate.first[i].set_v(0.0);
            fallback_trajectory_pair_candidate.first[i].set_a(0.0);
          }
        }
      }

      ADEBUG << "fallback start point after changes: "
             << fallback_start_point.DebugString();
      ADEBUG << "stop index: " << stop_index;
      ADEBUG << "fallback start index: " << fallback_start_index;

      /**
       * @brief 裁剪轨迹
       */
      fallback_trajectory_pair_candidate.first.erase(
          fallback_trajectory_pair_candidate.first.begin() + stop_index + 1,
          fallback_trajectory_pair_candidate.first.end());

      /**
       * @brief 追加停车点
       */
      for (int i = 0; i < 20; ++i) {
        common::TrajectoryPoint trajectory_point(
            fallback_trajectory_pair_candidate.first[stop_index]);
        trajectory_point.set_relative_time(
            i * 0.5 + 0.5 +
            fallback_trajectory_pair_candidate.first[stop_index]
                .relative_time());
        fallback_trajectory_pair_candidate.first.AppendTrajectoryPoint(
            trajectory_point);
      }
      *(frame_->mutable_open_space_info()->
          mutable_chosen_partitioned_trajectory()) =
          fallback_trajectory_pair_candidate;
    }
  }
  return Status::OK();
}

/**
 * @brief 检查车辆边界框是否无碰撞
 *
 * @return bool 无碰撞返回true
 *
 * 功能说明：
 * 检查当前帧中车辆是否与任何静态障碍物重叠
 *
 * C++语法说明：
 * - Polygon2d::HasOverlap()：
 *   检查两个多边形是否有重叠
 */
bool OpenSpaceFallbackDeciderPark::IsCollisionFreeEgoBox() {
  /**
   * @brief 获取车辆状态
   */
  const auto& vehicle_state = frame_->vehicle_state();
  double x = vehicle_state.x();
  double y = vehicle_state.y();
  double heading = vehicle_state.heading();

  /**
   * @brief 获取车辆配置参数
   */
  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  double ego_length = vehicle_config.vehicle_param().length();
  double ego_width = vehicle_config.vehicle_param().width();

  /**
   * @brief 创建车辆边界框和多边形
   */
  Box2d ego_box({x, y}, heading, ego_length, ego_width);
  Polygon2d ego_polygon = Polygon2d(ego_box);

  /**
   * @brief 遍历所有障碍物检查碰撞
   */
  for (const Obstacle* obstacle : frame_->obstacles()) {
    /**
     * @brief 过滤条件
     *
     * - 非虚拟障碍物
     * - 非静态障碍物
     * - 尺寸过小的障碍物（< 0.2m）
     */
    if (obstacle->IsVirtual() || !obstacle->IsStatic() ||
        obstacle->Perception().width() < 0.2 ||
        obstacle->Perception().length() < 0.2) {
      continue;
    }
    Polygon2d obstacle_polygon = obstacle->PerceptionPolygon();
    /**
     * @brief 检查重叠
     */
    if (ego_polygon.HasOverlap(obstacle_polygon)) {
      return false;  /**< 有碰撞 */
    }
  }
  return true;  /**< 无碰撞 */
}

/**
 * @brief 计算回退轨迹
 *
 * @param ChosenPartitionedTrajectory 选择的分段轨迹
 * @param deceleration 减速度
 * @param relative_time_interval 时间间隔
 * @param traj_gear_pair 输出：生成的轨迹
 *
 * 功能说明：
 * 从当前位置开始，按给定减速度生成停车轨迹
 *
 * C++语法说明：
 * - while循环：迭代生成轨迹点直到速度降至阈值
 * - std::cos/sin：三角函数计算位置
 * - emplace_back：原位构造轨迹点
 */
void OpenSpaceFallbackDeciderPark::CalculateFallbackTrajectory(
    const TrajGearPair& ChosenPartitionedTrajectory, const double& deceleration,
    const double& relative_time_interval, TrajGearPair* traj_gear_pair) {
  /**
   * @brief 清空输出轨迹
   */
  traj_gear_pair->first.clear();

  /**
   * @brief 获取当前速度
   */
  double current_speed = fabs(frame_->vehicle_state().linear_velocity());
  double pre_speed = current_speed;
  double decelerate = fabs(deceleration);

  double current_s_distance = 0;
  double relative_time = 0;
  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();

  /**
   * @brief 根据档位确定运动方向
   *
   * GEAR_DRIVE: alpha = 1 (前进)
   * GEAR_REVERSE: alpha = -1 (后退)
   */
  double moving_heading = frame_->vehicle_state().heading();
  int alpha =
      frame_->local_view().chassis->gear_location()
          == canbus::Chassis::GEAR_DRIVE ? 1 : -1;

  /**
   * @brief 迭代生成轨迹点
   */
  while (pre_speed >
         vehicle_config.vehicle_param().max_abs_speed_when_stopped()) {
    TrajectoryPoint point;
    static constexpr double kSpeedEpsilon = 1e-9;

    /**
     * @brief 设置轨迹点位置
     *
     * x = x₀ + s * cos(θ)
     * y = y₀ + s * sin(θ)
     */
    point.mutable_path_point()->set_x(frame_->vehicle_state().x() +
                                      current_s_distance * alpha *
                                          std::cos(moving_heading));
    point.mutable_path_point()->set_y(frame_->vehicle_state().y() +
                                      current_s_distance * alpha *
                                          std::sin(moving_heading));
    point.mutable_path_point()->set_theta(frame_->vehicle_state().heading());
    point.mutable_path_point()->set_s(current_s_distance * alpha);
    point.mutable_path_point()->set_kappa(0.0);
    point.set_relative_time(relative_time);
    point.set_v(current_speed* alpha);
    point.set_a(decelerate* -1 * alpha);
    traj_gear_pair->first.emplace_back(point);

    /**
     * @brief 更新状态
     */
    relative_time += relative_time_interval;
    pre_speed = current_speed;
    current_speed -= relative_time_interval * decelerate;
    /**
     * @brief 计算行驶距离（梯形积分）
     *
     * s = (v₀ + v₁) / 2 * Δt
     */
    current_s_distance +=
        (pre_speed + current_speed) / 2 * relative_time_interval;
    if (pre_speed < kSpeedEpsilon) break;
  }

  traj_gear_pair->second = frame_->local_view().chassis->gear_location();

  /**
   * @brief 如果轨迹为空，生成静止轨迹
   */
  if (traj_gear_pair->first.empty()) {
    for (int i = 0; i < 20; i++) {
      TrajectoryPoint point;
      point.mutable_path_point()->set_x(frame_->vehicle_state().x());
      point.mutable_path_point()->set_y(frame_->vehicle_state().y());
      point.mutable_path_point()->set_theta(frame_->vehicle_state().heading());
      point.mutable_path_point()->set_s(0);
      point.mutable_path_point()->set_kappa(0.0);
      point.set_relative_time(relative_time);
      point.set_v(0);
      point.set_a(-1.0 * alpha);
      traj_gear_pair->first.emplace_back(point);
      relative_time += relative_time_interval;
    }
    traj_gear_pair->second = frame_->local_view().chassis->gear_location();
  }
}

/**
 * @brief 使用轨迹姿态计算回退轨迹
 *
 * @param ChosenPartitionedTrajectory 选择的分段轨迹
 * @param start_point 起始点
 * @param collision_point 碰撞点
 * @param stop_distance 停车距离
 * @param relative_time_interval 时间间隔
 * @param traj_gear_pair 输出：生成的轨迹
 *
 * 功能说明：
 * 基于起始点和碰撞点生成更精确的回退轨迹
 *
 * C++语法说明：
 * - Vec2d::Normalize()：向量归一化
 * - 匀减速运动公式
 */
void OpenSpaceFallbackDeciderPark::CalculateFallbackTrajectoryWithTrajectoryPose(
    const TrajGearPair& ChosenPartitionedTrajectory, const TrajectoryPoint start_point, const TrajectoryPoint collision_point, const double& stop_distance,
    const double& relative_time_interval, TrajGearPair* traj_gear_pair) {
  static int count = 0;  /**< 计数器，用于调整时间偏移 */
  AINFO << "stop_distance:" << stop_distance;

  if (std::fabs(stop_distance) >= 1e-3) {
    count ++;
    /**
     * @brief 确定运动方向
     */
    int alpha = stop_distance < 0 ? -1 : 1;

    /**
     * @brief 计算运动参数
     *
     * 匀减速运动：
     * v = v_max - a * t
     * s = v_max * t - 0.5 * a * t²
     *
     * 设 t = total_time, v_max = 0.5 m/s
     * 则 s = v_max * t - 0.5 * a * t²
     * 其中 s = stop_distance
     */
    double v_max = 0.5;  /**< 最大速度 */
    double t  = std::fabs(stop_distance) / v_max * 2;  /**< 总时间 */
    double a = v_max / t;  /**< 加速度 */

    /**
     * @brief 计算方向向量
     */
    Vec2d unit_vec(collision_point.path_point().x() - start_point.path_point().x(),
                   collision_point.path_point().y() - start_point.path_point().y());
    Vec2d start_vec(start_point.path_point().x(), start_point.path_point().y());
    unit_vec.Normalize();  /**< 归一化为单位向量 */

    /**
     * @brief 生成轨迹点
     */
    for (double relative_t = 0; relative_t < t; relative_t += relative_time_interval){
      TrajectoryPoint point;
      double point_v, point_a, point_s;
      /**
       * @brief 匀减速运动公式
       */
      point_v = a * (t - relative_t);  /**< 速度 */
      point_a = -1 * a;  /**< 加速度 */
      point_s = std::fabs(stop_distance) - 0.5 * point_v * (t - relative_t);  /**< 距离 */

      /**
       * @brief 计算位置
       */
      Vec2d point_vec = start_vec + unit_vec * point_s;
      point.mutable_path_point()->set_x(point_vec.x());
      point.mutable_path_point()->set_y(point_vec.y());
      point.mutable_path_point()->set_theta(start_point.path_point().theta());
      point.mutable_path_point()->set_s(point_s * alpha);
      point.mutable_path_point()->set_kappa(0.0);
      point.set_relative_time(relative_t - 0.1 * count);
      point.set_v(point_v * alpha);
      point.set_a(point_a * alpha);
      AINFO << "point: " << point.DebugString();
      traj_gear_pair->first.emplace_back(point);
    }

    /**
     * @brief 添加终点
     */
    TrajectoryPoint point;
    Vec2d point_vec = start_vec + unit_vec * std::fabs(stop_distance);
    point.mutable_path_point()->set_x(point_vec.x());
    point.mutable_path_point()->set_y(point_vec.y());
    point.mutable_path_point()->set_theta(start_point.path_point().theta());
    point.mutable_path_point()->set_s(stop_distance);
    point.mutable_path_point()->set_kappa(0.0);
    point.set_relative_time(t - 0.1 * count);
    point.set_v(0);
    point.set_a(0);
    AINFO << "point: " << point.DebugString();
    traj_gear_pair->first.emplace_back(point);
    traj_gear_pair->second = ChosenPartitionedTrajectory.second;
  }

  /**
   * @brief 如果轨迹为空，生成静止轨迹
   */
  if (traj_gear_pair->first.empty()) {
    double relative_time = 0;
    int alpha =
      frame_->local_view().chassis->gear_location()
          == canbus::Chassis::GEAR_DRIVE ? 1 : -1;
    for (int i = 0; i < 20; i++) {
      TrajectoryPoint point;
      point.mutable_path_point()->set_x(frame_->vehicle_state().x());
      point.mutable_path_point()->set_y(frame_->vehicle_state().y());
      point.mutable_path_point()->set_theta(frame_->vehicle_state().heading());
      point.mutable_path_point()->set_s(0);
      point.mutable_path_point()->set_kappa(0.0);
      point.set_relative_time(relative_time);
      point.set_v(0);
      point.set_a(-1.0 * alpha);
      traj_gear_pair->first.emplace_back(point);
      relative_time += relative_time_interval;
    }
    traj_gear_pair->second = frame_->local_view().chassis->gear_location();
  }
}

/**
 * @brief 命名空间结束标记
 */
}  // namespace planning
}  // namespace apollo
