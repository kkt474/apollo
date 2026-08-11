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
 * @file open_space_fallback_decider.cc
 * @brief 开放空间后备决策器实现文件
 *
 * 功能说明：
 * 开放空间后备决策器用于在开放空间规划场景（如停车场）中
 * 当主轨迹发生碰撞时生成后备（fallback）轨迹
 *
 * 应用场景：
 * - 停车场泊车
 * - 狭窄空间掉头
 * - 自主泊车（APA/AVP）
 *
 * 工作原理：
 * 1. 检查主轨迹是否无碰撞
 * 2. 如果发生碰撞，计算安全停车距离
 * 3. 生成减速至停车的后备轨迹
 * 4. 如果自车当前位置已有碰撞，直接生成停车轨迹
 */

#include "modules/planning/tasks/open_space_fallback_decider/open_space_fallback_decider.h"

/**
 * @brief Apollo命名空间开始
 *
 * C++语法说明：
 * - namespace apollo：最外层命名空间
 * - namespace planning：规划模块子命名空间
 */
namespace apollo {
namespace planning {

/**
 * @brief 类型别名声明
 *
 * 功能说明：
 * 为常用类型创建别名，简化代码书写
 *
 * C++语法说明：
 * - using XXX = YYY：
 *   类型别名声明，与typedef等价但更直观
 * - apollo::common::Status：
 *   Apollo通用状态类，表示操作的成功/失败状态
 * - apollo::common::TrajectoryPoint：
 *   轨迹点类，包含位置、时间、速度、加速度等信息
 * - apollo::common::math::Box2d：
 *   2D矩形框类，用于表示障碍物的边界
 * - apollo::common::math::Polygon2d：
 *   2D多边形类，用于更精确的碰撞检测
 * - apollo::common::math::Vec2d：
 *   2D向量类，用于几何计算
 */
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::math::Box2d;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;

/**
 * @brief 初始化后备决策器
 *
 * @param config_dir 配置文件目录
 * @param name 任务名称
 * @param injector 依赖注入器指针
 * @return bool 初始化是否成功
 *
 * 功能说明：
 * 初始化开放空间后备决策器
 * 加载配置文件和依赖项
 *
 * 算法流程：
 * 1. 调用基类Decider的Init方法进行基础初始化
 * 2. 加载OpenSpaceFallBackDeciderConfig配置
 *
 * C++语法说明：
 * - const std::string&：
 *   常量引用，避免拷贝的同时防止修改
 * - std::shared_ptr<DependencyInjector>：
 *   共享指针，多个所有者可以共享同一个对象
 *   最后一个指针销毁时自动删除对象
 * - !Decider::Init(...)：
 *   逻辑非运算符，取反判断
 * - Decider::LoadConfig<T>(...)：
 *   模板方法，从配置文件加载指定类型的配置
 *   T = OpenSpaceFallBackDeciderConfig
 */
bool OpenSpaceFallbackDecider::Init(
    const std::string& config_dir, const std::string& name,
    const std::shared_ptr<DependencyInjector>& injector) {
  // 调用基类Decider的Init方法进行基础初始化
  // 包括加载配置目录、任务名称、依赖注入器
  if (!Decider::Init(config_dir, name, injector)) {
    return false;  // 如果基类初始化失败，返回false
  }
  // 加载开放空间后备决策器的配置
  // LoadConfig是模板方法，T指定配置类型
  // &config_获取config_的地址传给LoadConfig
  return Decider::LoadConfig<OpenSpaceFallBackDeciderConfig>(&config_);
}

/**
 * @brief 求一元二次方程的较小解
 *
 * @param a 二次项系数 ax^2 + bx + c = 0
 * @param b 一次项系数
 * @param c 常数项
 * @param sol 输出：较小的解
 * @return bool 是否存在实数解
 *
 * 功能说明：
 * 求解一元二次方程，返回较小的实数解（绝对值）
 * 用于计算停车所需的时间
 *
 * 数学公式：
 * x = (-b ± √(b²-4ac)) / 2a
 * 需要判别式 b²-4ac >= 0 才有实数解
 *
 * C++语法说明：
 * - const double：
 *   常量double类型，函数内不修改
 * - double* sol：
 *   指针参数，用于输出结果
 * - std::abs()：
 *   标准库绝对值函数
 *   用于浮点数（注意与std::fabs的区别）
 * - std::sqrt()：
 *   标准库平方根函数
 */
bool OpenSpaceFallbackDecider::QuardraticFormulaLowerSolution(const double a,
                                                              const double b,
                                                              const double c,
                                                              double* sol) {
  // 一元二次方程：ax² + bx + c = 0，返回较小的解
  // TODO(QiL): use const from common::math
  const double kEpsilon = 1e-6;  // 浮点数容差，用于判断是否为零
  *sol = 0.0;  // 初始化解为0

  // 如果a接近0，则不是有效的二次方程（退化为一次方程）
  if (std::abs(a) < kEpsilon) {
    return false;  // 无效，返回false
  }

  // 计算判别式 delta = b² - 4ac
  double tmp = b * b - 4 * a * c;

  // 如果判别式小于容差，则无实数解
  if (tmp < kEpsilon) {
    return false;
  }

  // 计算两个解
  // x1 = (-b + √delta) / 2a
  // x2 = (-b - √delta) / 2a
  double sol1 = (-b + std::sqrt(tmp)) / (2.0 * a);
  double sol2 = (-b - std::sqrt(tmp)) / (2.0 * a);

  // 返回绝对值较小的解
  *sol = std::abs(std::min(sol1, sol2));
  
  // 输出调试信息
  ADEBUG << "QuardraticFormulaLowerSolution finished with sol: " << *sol
         << "sol1: " << sol1 << ", sol2: " << sol2 << "a: " << a << "b: " << b
         << "c: " << c;
  return true;
}

/**
 * @brief 主处理函数
 *
 * @param frame 规划帧数据
 * @return Status 处理状态
 *
 * 功能说明：
 * 后备决策器的主处理流程：
 * 1. 构建预测环境（障碍物在未来时间的边界框）
 * 2. 检查主轨迹是否有碰撞
 * 3. 如果有碰撞，生成减速停车的后备轨迹
 * 4. 如果自车当前位置有碰撞，生成紧急停车轨迹
 *
 * C++语法说明：
 * - Status：
 *   Apollo通用状态类，包含错误码和错误信息
 * - std::vector<std::vector<Box2d>>：
 *   二维向量，外层表示时间步，内层表示该时刻的障碍物边界框
 * - frame_->open_space_info()：
 *   frame_是指向Frame对象的指针
 *   open_space_info()返回开放空间信息
 * - mutable_xxx()：
 *   可变访问器，返回非const指针，允许修改成员
 */
Status OpenSpaceFallbackDecider::Process(Frame* frame) {
  // 预测的障碍物边界框容器
  // 外层vector: 不同时间步的预测
  // 内层vector: 同一时间步的多个障碍物边界框
  std::vector<std::vector<common::math::Box2d>> predicted_bounding_rectangles;
  
  size_t first_collision_index = 0;  // 首次碰撞的轨迹点索引
  size_t fallback_start_index = 0;   // 后备轨迹起始索引

  // 检查是否已经做过fallback决策
  // 如果fallback_flag为true，说明已经处理过，跳过
  if (frame_->open_space_info().fallback_flag()) {
    AINFO << "has got fallback decider, skip";
    return Status::OK();  // 返回成功状态
  }

  // 构建预测环境
  // 遍历障碍物，生成每个时间步的预测边界框
  BuildPredictedEnvironment(frame->obstacles(), predicted_bounding_rectangles);
  
  // 输出调试信息：障碍物数量和预测边界框数量
  ADEBUG << "Numbers of obstsacles are: " << frame->obstacles().size();
  ADEBUG << "Numbers of predicted bounding rectangles are: "
         << predicted_bounding_rectangles[0].size()
         << " and : " << predicted_bounding_rectangles.size();

  // 检查选定的分区轨迹是否无碰撞
  if (!IsCollisionFreeTrajectory(
          frame->open_space_info().chosen_partitioned_trajectory(),
          predicted_bounding_rectangles, &fallback_start_index,
          &first_collision_index)) {
    // -------------------- 轨迹有碰撞，需要生成后备轨迹 --------------------
    
    // 设置停止标志为true
    frame_->mutable_open_space_info()->set_stop_flag(true);

    // 基于当前分区轨迹生成后备轨迹
    // 车辆速度在安全距离内递减至零
    TrajGearPair fallback_trajectory_pair_candidate =
        frame->open_space_info().chosen_partitioned_trajectory();

    // 获取首次碰撞点
    const auto future_collision_point =
        fallback_trajectory_pair_candidate.first[first_collision_index];

    // 从当前位置开始生成后备轨迹，但使用当前车辆速度
    auto fallback_start_point =
        fallback_trajectory_pair_candidate.first[fallback_start_index];
    
    // 获取车辆状态
    const auto& vehicle_state = injector_->vehicle_state()->vehicle_state();
    
    // 设置后备起始点的速度为当前车辆速度
    fallback_start_point.set_v(vehicle_state.linear_velocity());

    // 保存首次碰撞点到frame中
    *(frame_->mutable_open_space_info()->mutable_future_collision_point()) =
        future_collision_point;

    // 计算最小停车距离
    // 公式: d = v² / (2*a)，这里a=4.0（最大加速度）
    // 0.5 * v² / a = v² / (2a)
    double min_stop_distance =
        0.5 * fallback_start_point.v() * fallback_start_point.v() / 4.0;

    // 计算实际停车距离
    // 根据档位判断方向：前进档正向，倒车档负向
    // TODO(QiL): move 1.0 to configs
    double stop_distance =
        fallback_trajectory_pair_candidate.second == canbus::Chassis::GEAR_DRIVE
            ? std::max(future_collision_point.path_point().s() -
                           fallback_start_point.path_point().s() - 1.0,
                       0.0)  // 前进档：取max(0, 碰撞点s - 起始点s - 1)
            : std::min(future_collision_point.path_point().s() -
                           fallback_start_point.path_point().s() + 1.0,
                       0.0);  // 倒车档：取min(0, 碰撞点s - 起始点s + 1)

    ADEBUG << "stop distance : " << stop_distance;

    // 车辆最大加速度和减速度
    // const double vehicle_max_acc = 4.0;   // 最大加速度
    // const double vehicle_max_dec = -4.0;  // 最大减速度

    double stop_deceleration = 0.0;  // 停车减速度

    // 根据档位计算停车减速度和距离
    if (fallback_trajectory_pair_candidate.second ==
        canbus::Chassis::GEAR_REVERSE) {
      // 倒车档
      // 计算减速度：a = v² / (2*d)
      stop_deceleration =
          std::min(fallback_start_point.v() * fallback_start_point.v() /
                       (2.0 * (stop_distance + 1e-6)),
                   vehicle_max_acc);  // 取计算值和最大加速度中的较小值
      
      // 停车距离取负值最小值
      stop_distance = std::min(-1 * min_stop_distance, stop_distance);
    } else {
      // 前进档
      // 计算减速度（负值）
      stop_deceleration =
          std::max(-fallback_start_point.v() * fallback_start_point.v() /
                       (2.0 * (stop_distance + 1e-6)),
                   vehicle_max_dec);  // 取计算值和最大减速度中的较大值（负值）
      
      // 停车距离取正值最大值
      stop_distance = std::max(min_stop_distance, stop_distance);
    }

    ADEBUG << "stop_deceleration: " << stop_deceleration;

    // -------------------- 搜索停车索引 --------------------
    // 在选定轨迹上搜索满足停车距离的索引
    size_t stop_index = fallback_start_index;

    // 遍历从fallback_start开始的轨迹点
    for (size_t i = fallback_start_index;
         i < fallback_trajectory_pair_candidate.first.NumOfPoints(); ++i) {
      // 检查当前点的s坐标是否超过起始点s + 停车距离
      if (std::abs(
              fallback_trajectory_pair_candidate.first[i].path_point().s()) >=
          std::abs(fallback_start_point.path_point().s() + stop_distance)) {
        stop_index = i;  // 记录停车索引
        break;            // 找到后跳出循环
      }
    }

    ADEBUG << "stop index before is: " << stop_index
           << "; fallback_start index before is: " << fallback_start_index;

    // -------------------- 更新fallback_start之前的点 --------------------
    // 将fallback_start之前的点设置为恒定速度和减速度
    for (size_t i = 0; i < fallback_start_index; ++i) {
      fallback_trajectory_pair_candidate.first[i].set_v(
          fallback_start_point.v());  // 设置为起始速度
      fallback_trajectory_pair_candidate.first[i].set_a(stop_deceleration);  // 设置减速度
    }

    // TODO(QiL): refine the logic and remove redundant code, change 0.5 to from
    // loading optimizer configs

    // -------------------- 处理停车索引 --------------------
    // 情况1：停车距离在安全缓冲区内，需要立即停车
    if (fallback_start_index >= stop_index) {
      // 1. 设置fallback起始速度为0，加速度为最大加速度
      AINFO << "Stop distance within safety buffer, stop now!";
      fallback_start_point.set_v(0.0);  // 速度设为0
      fallback_start_point.set_a(0.0);  // 加速度设为0
      
      // 设置停车点的速度和加速度为0
      fallback_trajectory_pair_candidate.first[stop_index].set_v(0.0);
      fallback_trajectory_pair_candidate.first[stop_index].set_a(0.0);

      // 2. 移除停车点之后的所有轨迹点
      fallback_trajectory_pair_candidate.first.erase(
          fallback_trajectory_pair_candidate.first.begin() + stop_index + 1,
          fallback_trajectory_pair_candidate.first.end());

      // 3. 追加相同的停车点（位置相同但速度为零）
      // 追加20个点，每个点间隔0.5秒
      for (int i = 0; i < 20; ++i) {
        // 从停车点复制创建新轨迹点
        common::TrajectoryPoint trajectory_point(
            fallback_trajectory_pair_candidate.first[stop_index]);
        // 设置相对时间：i * 0.5 + 0.5 + 原有点的相对时间
        trajectory_point.set_relative_time(
            i * 0.5 + 0.5 +
            fallback_trajectory_pair_candidate.first[stop_index]
                .relative_time());
        // 追加到轨迹末尾
        fallback_trajectory_pair_candidate.first.AppendTrajectoryPoint(
            trajectory_point);
      }

      // 保存后备轨迹到frame
      *(frame_->mutable_open_space_info()->mutable_fallback_trajectory()) =
          fallback_trajectory_pair_candidate;

      return Status::OK();  // 处理完成
    }

    // 输出调试信息
    ADEBUG << "before change, size : "
           << fallback_trajectory_pair_candidate.first.size()
           << ", first index information : "
           << fallback_trajectory_pair_candidate.first[0].DebugString()
           << ", second index information : "
           << fallback_trajectory_pair_candidate.first[1].DebugString();

    // -------------------- 情况2：停车距离足够，需要减速停车 --------------------
    // 从fallback_start到stop_index之间重新计算速度曲线
    
    // 遍历fallback_start到stop_index之间的点
    for (size_t i = fallback_start_index; i <= stop_index; ++i) {
      double new_relative_time = 0.0;  // 新的相对时间
      double temp_v = 0.0;             // 临时速度变量
      // 计算常数项c: -2.0 * s
      // 来自公式: s = v0 * t + 0.5 * a * t²
      // 整理得: 0.5 * a * t² + v0 * t - s = 0
      double c =
          -2.0 * fallback_trajectory_pair_candidate.first[i].path_point().s();

      // 求解二次方程得到时间
      if (QuardraticFormulaLowerSolution(stop_deceleration,
                                         2.0 * fallback_start_point.v(), c,
                                         &new_relative_time) &&
          // 检查计算出的s是否在停车距离内
          std::abs(
              fallback_trajectory_pair_candidate.first[i].path_point().s()) <=
              std::abs(stop_distance)) {
        ADEBUG << "new_relative_time" << new_relative_time;
        
        // 计算新速度: v = v0 + a * t
        temp_v =
            fallback_start_point.v() + stop_deceleration * new_relative_time;
        
        // 速度限制：最大1.0 m/s
        if (std::abs(temp_v) < 1.0) {
          fallback_trajectory_pair_candidate.first[i].set_v(temp_v);
        } else {
          // 限制速度方向并设为1.0
          fallback_trajectory_pair_candidate.first[i].set_v(
              temp_v / std::abs(temp_v) * 1.0);
        }
        // 设置加速度
        fallback_trajectory_pair_candidate.first[i].set_a(stop_deceleration);

        // 设置相对时间
        fallback_trajectory_pair_candidate.first[i].set_relative_time(
            new_relative_time);
      } else {
        // 无法求解二次方程，使用备选方案
        if (i != 0) {
          // 将当前点的位置设为前一个点的位置
          fallback_trajectory_pair_candidate.first[i]
              .mutable_path_point()
              ->CopyFrom(
                  fallback_trajectory_pair_candidate.first[i - 1].path_point());
          fallback_trajectory_pair_candidate.first[i].set_v(0.0);  // 速度设为0
          fallback_trajectory_pair_candidate.first[i].set_a(0.0);  // 加速度设为0
          // 相对时间设为前一个点的时间加0.5秒
          fallback_trajectory_pair_candidate.first[i].set_relative_time(
              fallback_trajectory_pair_candidate.first[i - 1].relative_time() +
              0.5);
        } else {
          // 第一个点，设为静止
          fallback_trajectory_pair_candidate.first[i].set_v(0.0);
          fallback_trajectory_pair_candidate.first[i].set_a(0.0);
        }
      }
    }

    ADEBUG << "fallback start point after changes: "
           << fallback_start_point.DebugString();

    ADEBUG << "stop index: " << stop_index;
    ADEBUG << "fallback start index: " << fallback_start_index;

    // 2. 移除停车点之后的所有轨迹点
    fallback_trajectory_pair_candidate.first.erase(
        fallback_trajectory_pair_candidate.first.begin() + stop_index + 1,
        fallback_trajectory_pair_candidate.first.end());

    // 3. 追加相同的停车点（位置相同但速度为零）
    for (int i = 0; i < 20; ++i) {
      common::TrajectoryPoint trajectory_point(
          fallback_trajectory_pair_candidate.first[stop_index]);
      trajectory_point.set_relative_time(
          i * 0.5 + 0.5 +
          fallback_trajectory_pair_candidate.first[stop_index].relative_time());
      fallback_trajectory_pair_candidate.first.AppendTrajectoryPoint(
          trajectory_point);
    }

    // 保存后备轨迹到frame
    *(frame_->mutable_open_space_info()->mutable_fallback_trajectory()) =
        fallback_trajectory_pair_candidate;
  } else {
    // 轨迹无碰撞，设置停止标志为false
    frame_->mutable_open_space_info()->set_stop_flag(false);
  }

  // -------------------- 检查自车当前位置是否有碰撞 --------------------
  if (!IsCollisionFreeEgoBox()) {
    // 自车当前位置有碰撞，需要生成紧急停车轨迹
    double relative_time = 0.0;
    
    // TODO(Jinyun) Move to conf
    static constexpr int stop_trajectory_length = 10;         // 停车轨迹长度
    static constexpr double relative_stop_time = 0.1;          // 停车点时间间隔
    
    // 获取可修改的后备轨迹指针
    auto fallback_tra_pair =
        frame_->mutable_open_space_info()->mutable_fallback_trajectory();

    // 设置档位为前进档
    fallback_tra_pair->second = canbus::Chassis::GEAR_DRIVE;

    // 清空现有轨迹
    fallback_tra_pair->first.clear();
    
    // 生成10个停车点
    for (size_t i = 0; i < stop_trajectory_length; i++) {
      TrajectoryPoint point;
      // 设置路径点位置为当前车辆位置
      point.mutable_path_point()->set_x(frame_->vehicle_state().x());
      point.mutable_path_point()->set_y(frame_->vehicle_state().y());
      point.mutable_path_point()->set_theta(frame_->vehicle_state().heading());
      point.mutable_path_point()->set_s(0.0);       // s设为0
      point.mutable_path_point()->set_kappa(0.0);   // 曲率设为0
      
      // 设置时间和速度
      point.set_relative_time(relative_time);
      point.set_v(0.0);       // 速度为0
      point.set_a(-4.0);      // 加速度为-4.0（减速）
      
      // 添加到轨迹
      fallback_tra_pair->first.emplace_back(point);
      
      // 更新时间间隔
      relative_time += relative_stop_time;
    }

    // 设置fallback标志为true
    frame_->mutable_open_space_info()->set_fallback_flag(true);

  } else {
    // 自车位置无碰撞，设置fallback标志为false
    frame_->mutable_open_space_info()->set_fallback_flag(false);
  }

  return Status::OK();  // 处理完成
}

/**
 * @brief 构建预测环境
 *
 * @param obstacles 障碍物列表
 * @param predicted_bounding_rectangles 输出：预测的边界框
 *
 * 功能说明：
 * 根据障碍物的预测轨迹，生成每个时间步的边界框
 * 用于碰撞检测
 *
 * 算法流程：
 * 1. 从时间0开始，按固定时间分辨率递增
 * 2. 对每个障碍物，获取其在特定时间的预测位置
 * 3. 计算该位置的边界框
 * 4. 收集所有障碍物的边界框
 *
 * C++语法说明：
 * - std::vector<const Obstacle*>：
 *   存储障碍物指针的向量
 *   使用指针避免拷贝，提高效率
 * - std::move()：
 *   移动语义，避免拷贝
 * - FLAGS_trajectory_time_resolution：
 *   全局flag，轨迹时间分辨率
 */
void OpenSpaceFallbackDecider::BuildPredictedEnvironment(
    const std::vector<const Obstacle*>& obstacles,
    std::vector<std::vector<common::math::Box2d>>&
        predicted_bounding_rectangles) {
  // 清空预测边界框
  predicted_bounding_rectangles.clear();
  
  double relative_time = 0.0;  // 相对时间，从0开始

  // 按时间步遍历，直到超过配置的预测时间
  while (relative_time < config_.open_space_prediction_time_period()) {
    std::vector<Box2d> predicted_env;  // 当前时间步的障碍物边界框
    
    // 遍历所有障碍物
    for (const Obstacle* obstacle : obstacles) {
      // 跳过虚拟障碍物
      if (!obstacle->IsVirtual()) {
        // 获取障碍物在relative_time时刻的预测点
        TrajectoryPoint point = obstacle->GetPointAtTime(relative_time);
        // 获取该点的边界框
        Box2d box = obstacle->GetBoundingBox(point);
        // 移动到预测环境容器中
        predicted_env.push_back(std::move(box));
      }
    }
    
    // 将当前时间步的预测添加到总预测中
    predicted_bounding_rectangles.emplace_back(std::move(predicted_env));
    
    // 时间递增
    relative_time += FLAGS_trajectory_time_resolution;
  }
}

/**
 * @brief 检查自车包围盒是否无碰撞
 *
 * @return bool 如果自车位置无碰撞返回true
 *
 * 功能说明：
 * 检查自车当前位置是否与任何静态障碍物碰撞
 * 用于判断是否需要紧急停车
 *
 * C++语法说明：
 * - common::VehicleConfigHelper::Instance()：
 *   单例模式，获取车辆配置Helper实例
 * - Box2d({x, y}, heading, length, width)：
 *   使用初始化列表构造2D边界框
 * - Polygon2d(ego_box)：
 *   从Box2d构造多边形
 * - HasOverlap()：
 *   多边形重叠检测方法
 */
bool OpenSpaceFallbackDecider::IsCollisionFreeEgoBox() {
  // 预测时间分辨率: FLAGS_trajectory_time_resolution
  
  // 获取车辆状态
  const auto& vehicle_state = frame_->vehicle_state();
  double x = vehicle_state.x();           // 车辆x坐标
  double y = vehicle_state.y();           // 车辆y坐标
  double heading = vehicle_state.heading();  // 车辆朝向角

  // 获取车辆配置
  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  double ego_length = vehicle_config.vehicle_param().length();  // 车辆长度
  double ego_width = vehicle_config.vehicle_param().width();    // 车辆宽度

  // 创建自车包围盒
  Box2d ego_box({x, y}, heading, ego_length, ego_width);
  
  // 转换为多边形（用于更精确的碰撞检测）
  Polygon2d ego_polygon = Polygon2d(ego_box);

  // 遍历所有障碍物
  for (const Obstacle* obstacle : frame_->obstacles()) {
    // 跳过虚拟障碍物、非静态障碍物、以及太小的障碍物
    if (obstacle->IsVirtual() || !obstacle->IsStatic() ||
        obstacle->Perception().width() < 0.2 ||
        obstacle->Perception().length() < 0.2) {
      continue;  // 跳过这些障碍物
    }
    
    // 获取障碍物的多边形
    Polygon2d obstacle_polygon = obstacle->PerceptionPolygon();
    
    // 检查是否有重叠
    if (ego_polygon.HasOverlap(obstacle_polygon)) {
      return false;  // 有碰撞，返回false
    }
  }
  
  return true;  // 无碰撞，返回true
}

/**
 * @brief 路径点归一化（坐标变换）
 *
 * @param rotate_angle 旋转角度
 * @param translate_origin 平移原点
 * @param x 输入/输出：x坐标
 * @param y 输入/输出：y坐标
 * @param phi 输入/输出：朝向角
 *
 * 功能说明：
 * 将路径点从世界坐标系转换到局部坐标系
 * 1. 平移：将原点移到translate_origin
 * 2. 旋转：将坐标系统rotate_angle旋转
 *
 * 旋转公式（逆时针旋转θ）：
 * x' = x*cos(θ) - y*sin(θ)
 * y' = x*sin(θ) + y*cos(θ)
 *
 * C++语法说明：
 * - double*：
 *   指针参数，用于输入输出
 *   函数内直接修改指针指向的值
 * - std::cos/std::sin：
 *   标准库三角函数，接受弧度值
 * - common::math::NormalizeAngle：
 *   角度归一化，将角度归一到[-π, π]范围
 */
void OpenSpaceFallbackDecider::PathPointNormalizing(
    double rotate_angle, const Vec2d& translate_origin, double* x, double* y,
    double* phi) {
  // 第一步：平移 - 减去平移原点
  *x -= translate_origin.x();
  *y -= translate_origin.y();
  
  // 保存原始x值用于计算新y
  double tmp_x = *x;
  
  // 第二步：旋转（绕原点逆时针旋转-rotate_angle，相当于顺时针旋转rotate_angle）
  // x' = x*cos(-θ) - y*sin(-θ) = x*cos(θ) + y*sin(θ)
  *x = (*x) * std::cos(-rotate_angle) - (*y) * std::sin(-rotate_angle);
  // y' = x*sin(-θ) + y*cos(-θ) = -x*sin(θ) + y*cos(θ)
  *y = tmp_x * std::sin(-rotate_angle) + (*y) * std::cos(-rotate_angle);
  
  // 第三步：角度归一化
  // 将角度减去旋转角，并归一化到[-π, π]
  *phi = common::math::NormalizeAngle(*phi - rotate_angle);
}

/**
 * @brief 检查轨迹是否无碰撞
 *
 * @param trajectory_gear_pair 轨迹和档位对
 * @param predicted_bounding_rectangles 预测的障碍物边界框
 * @param current_index 输出：当前索引
 * @param first_collision_index 输出：首次碰撞索引
 * @return bool 如果轨迹无碰撞返回true
 *
 * 功能说明：
 * 检查给定轨迹是否与任何预测的障碍物边界框碰撞
 * 返回首次碰撞的位置索引
 *
 * 算法流程：
 * 1. 遍历轨迹的每个点
 * 2. 计算自车在每个点的边界框
 * 3. 与该时刻的障碍物边界框进行碰撞检测
 * 4. 如果有碰撞，返回首次碰撞的索引
 *
 * C++语法说明：
 * - TrajGearPair：
 *   轨迹和档位的配对
 *   first: 轨迹 (DiscretizedTrajectory)
 *   second: 档位 (Chassis::GearPosition)
 * - trajectory_pb.NumOfPoints()：
 *   获取轨迹点的数量
 * - trajectory_pb.QueryLowerBoundPoint(0.0)：
 *   查询时间大于等于0.0的第一个点的索引
 * - Box2d(...).Shift(...)：
 *   构造后立即Shift，移动边界框
 */
bool OpenSpaceFallbackDecider::IsCollisionFreeTrajectory(
    const TrajGearPair& trajectory_gear_pair,
    const std::vector<std::vector<common::math::Box2d>>&
        predicted_bounding_rectangles,
    size_t* current_index, size_t* first_collision_index) {
  // 预测时间分辨率: FLAGS_trajectory_time_resolution
  
  // 获取车辆配置
  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  double ego_length = vehicle_config.vehicle_param().length();  // 车辆长度
  double ego_width = vehicle_config.vehicle_param().width();      // 车辆宽度
  
  // 获取轨迹
  auto trajectory_pb = trajectory_gear_pair.first;
  const size_t point_size = trajectory_pb.NumOfPoints();  // 轨迹点数量

  // 初始化当前索引为时间0对应的点
  *current_index = trajectory_pb.QueryLowerBoundPoint(0.0);

  // 遍历轨迹的每个点
  for (size_t i = *current_index; i < point_size; ++i) {
    // 获取轨迹点
    const auto& trajectory_point = trajectory_pb.TrajectoryPointAt(i);
    double ego_theta = trajectory_point.path_point().theta();  // 自车朝向角
    
    // 创建自车边界框
    Box2d ego_box(
        {trajectory_point.path_point().x(), trajectory_point.path_point().y()},
        ego_theta, ego_length, ego_width);
    
    // 计算后边缘中心点的偏移量
    // 后边缘中心到车辆中心的距离 = 车辆长度/2 - 后边缘到中心的距离
    double shift_distance =
        ego_length / 2.0 - vehicle_config.vehicle_param().back_edge_to_center();
    
    // 计算偏移向量
    Vec2d shift_vec{shift_distance * std::cos(ego_theta),
                    shift_distance * std::sin(ego_theta)};
    
    // 移动边界框到后边缘中心
    ego_box.Shift(shift_vec);
    
    // 获取预测时间范围
    size_t predicted_time_horizon = predicted_bounding_rectangles.size();
    
    // 遍历每个预测时间步
    for (size_t j = 0; j < predicted_time_horizon; j++) {
      // 遍历该时间步的所有障碍物边界框
      for (const auto& obstacle_box : predicted_bounding_rectangles[j]) {
        // 检查自车边界框与障碍物边界框是否重叠
        if (ego_box.HasOverlap(obstacle_box)) {
          ADEBUG << "HasOverlap(obstacle_box) [" << i << "]";
          
          // 获取车辆状态
          const auto& vehicle_state = frame_->vehicle_state();
          Vec2d vehicle_vec({vehicle_state.x(), vehicle_state.y()});
          
          // 判断碰撞是否在可接受的时间缓冲区内
          // 如果碰撞点的时间接近轨迹点的时间，且轨迹点时间大于0
          if (std::abs(trajectory_point.relative_time() -
                       static_cast<double>(j) *
                           FLAGS_trajectory_time_resolution) <
                  config_.open_space_fallback_collision_time_buffer() &&
              trajectory_point.relative_time() > 0.0) {
            ADEBUG << "first_collision_index: [" << i << "]";
            *first_collision_index = i;  // 记录首次碰撞索引
            return false;  // 有碰撞，返回false
          }
        }
      }
    }
  }

  return true;  // 无碰撞，返回true
}

}  // namespace planning
}  // namespace apollo