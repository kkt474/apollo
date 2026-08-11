/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
 * @file open_space_path_planning.cc
 *
 * @brief 开源空间路径规划实现文件
 *
 * 功能说明：
 * 本文件实现了 OpenSpacePathPlanning 类
 * 用于处理开放空间（如停车场）的路径规划场景
 * 主要使用 Hybrid A* 算法进行路径搜索
 *
 * 核心概念：
 * - 开源空间规划（Open Space Planning）：适用于停车场、狭窄道路等非结构化场景
 * - Hybrid A*：混合A*算法，结合连续空间和离散搜索
 * - 停车类型：垂直停车(PARALLEL_PARKING)、垂直停车(VERTICAL_PARKING)
 *
 * C++语法说明：
 * - #include：预处理指令，包含头文件
 * - namespace：命名空间，避免命名冲突
 * - class：类声明
 * - ::运算符：作用域解析，类名::函数名表示成员函数
 * - std::shared_ptr：智能指针，引用计数管理生命周期
 * - std::atomic：原子类型，用于多线程安全标志
 * - std::thread：线程支持
 * - std::mutex：互斥锁，用于线程同步
 * - std::future：异步操作的结果
 **/

#include "modules/planning/tasks/open_space_path_planning/open_space_path_planning.h"

namespace apollo {
/**
 * @brief Apollo主命名空间
 */
namespace planning {

/**
 * @brief 类型别名声明
 *
 * C++语法说明：
 * - using：类型别名声明，等价于typedef
 * - apollo::common::ErrorCode：Apollo通用错误码
 * - apollo::common::Status：Apollo通用状态
 * - common::math::Vec2d：2D向量数学工具类
 */
using apollo::common::ErrorCode;
using apollo::common::Status;
using common::math::Vec2d;

/**
 * @brief 初始化函数
 *
 * @param config_dir 配置目录路径
 * @param name 任务名称
 * @param injector 依赖注入器指针
 * @return bool 初始化成功返回true
 *
 * 功能说明：
 * 1. 调用基类Task的Init方法进行基础初始化
 * 2. 加载开源空间路径规划配置
 * 3. 创建HybridAStar路径搜索器实例
 * 4. 打印配置信息用于调试
 *
 * C++语法说明：
 * - const std::string &config_dir：
 *   常量引用参数，承诺不修改，避免拷贝
 *
 * - const std::shared_ptr<DependencyInjector> &injector：
 *   shared_ptr智能指针的常量引用
 *   共享所有权，引用计数管理生命周期
 *
 * - &config_：成员变量地址，用于输出
 *
 * - Task::LoadConfig<OpenSpacePathPlanningConfig>(&config_)：
 *   模板函数调用，LoadConfig<T>加载特定类型的配置
 *
 * - std::shared_ptr<HybridAStar>：智能指针，管理HybridAStar实例
 *   - new HybridAStar(config_.warm_start_config())：
 *     在堆上创建对象
 *   - .reset()：重置智能指针，释放旧资源
 *
 * - AINFO：Apollo信息级别日志宏
 *
 * - config_.DebugString()：
 *   protobuf消息的调试字符串方法
 */
bool OpenSpacePathPlanning::Init(
    const std::string &config_dir,
    const std::string &name,
    const std::shared_ptr<DependencyInjector> &injector) {
  /**
   * @brief 调用基类初始化
   *
   * Task::Init(config_dir, name, injector)：
   *   调用父类Task的Init方法
   *   返回false表示初始化失败
   */
  if (!Task::Init(config_dir, name, injector)) {
    return false;
  }
  /**
   * @brief 加载开源空间路径规划配置
   *
   * LoadConfig<OpenSpacePathPlanningConfig>：
   *   模板方法，加载指定类型的protobuf配置
   *   &config_：输出参数，存储加载的配置
   */
  if (!Task::LoadConfig<OpenSpacePathPlanningConfig>(&config_)) {
    return false;
  }
  /**
   * @brief 创建HybridAStar路径搜索器
   *
   * path_finder_：类成员变量，shared_ptr类型
   * .reset(new HybridAStar(...))：
   *   重置智能指针，释放旧资源（如果有）
   *   在堆上创建新的HybridAStar实例
   *
   * config_.warm_start_config()：
   *   获取热启动配置，用于加速搜索
   */
  path_finder_.reset(new HybridAStar(config_.warm_start_config()));
  AINFO << config_.DebugString();
  return true;
}

/**
 * @brief 析构函数
 *
 * 功能说明：
 * 如果启用了路径规划线程，则在析构时停止线程
 * 确保线程安全退出
 *
 * C++语法说明：
 * - ~OpenSpacePathPlanning()：析构函数，类名加~前缀
 * - 虚析构函数确保通过基类指针删除派生类对象时正确调用析构函数
 */
OpenSpacePathPlanning::~OpenSpacePathPlanning() {
  /**
   * @brief 检查是否启用了路径规划线程
   *
   * config_.enable_path_planning_thread()：
   *   protobuf配置的布尔字段访问方法
   *   返回是否启用异步线程规划
   */
  if (config_.enable_path_planning_thread()) {
    Stop();  /**< 调用Stop方法停止线程 */
  }
}

/**
 * @brief 停止函数
 *
 * 功能说明：
 * 停止异步路径规划线程
 * 确保线程安全退出并清理资源
 *
 * C++语法说明：
 * - std::atomic<bool>：原子布尔类型
 *   - .store()：原子写入
 *   - .load()：原子读取
 *   保证多线程下的可见性
 *
 * - std::atomic::load()：
 *   读取原子变量的当前值
 *
 * - thread_init_flag_：线程初始化标志
 * - data_ready_：数据就绪标志
 * - path_update_：路径更新标志
 *
 * - task_future_.get()：
 *   获取异步任务的结果
 *   .get()会阻塞直到任务完成
 */
void OpenSpacePathPlanning::Stop() {
  if (config_.enable_path_planning_thread()) {
    if (thread_init_flag_) {
      task_future_.get();  /**< 等待线程完成 */
    }
    data_ready_.store(false);   /**< 重置数据就绪标志 */
    path_update_.store(false);   /**< 重置路径更新标志 */
  }
}

/**
 * @brief 主处理函数
 *
 * @return Status 处理状态
 *
 * 功能说明：
 * 开源空间路径规划的主入口函数
 * 1. 检查是否需要重规划
 * 2. 如果有上一帧结果则复用
 * 3. 否则执行路径规划
 *
 * C++语法说明：
 * - Status：Apollo通用状态类型
 *   - Status::OK()：成功状态
 *   - 可包含错误码和消息
 *
 * - injector_->frame_history()->Latest()：
 *   链式调用获取历史帧
 *   ->frame_history()：调用 injector 的方法
 *   ->Latest()：调用 frame_history 的方法
 *
 * - frame_->open_space_info()：
 *   获取开源空间信息
 *   frame_ 是当前帧指针
 *
 * - mutable_open_space_info()：
 *   protobuf可变成员访问方法
 *   用于获取可修改的引用
 *
 * - *(ptr) = value：
 *   解引用指针并赋值
 *   拷贝赋值
 *
 * - ternary operator (? :)：
 *   条件运算符
 *   condition ? value_if_true : value_if_false
 */
Status OpenSpacePathPlanning::Process() {
  AINFO << "data_ready_: " << (data_ready_ ? "true" : "false");
  /**
   * @brief 获取上一帧
   *
   * injector_->frame_history()：
   *   获取帧历史记录管理器
   *   用于访问历史规划帧数据
   *
   * ->Latest()：
   *   获取最新的历史帧
   */
  auto* previous_frame = injector_->frame_history()->Latest();
  /**
   * @brief 检查是否需要重规划
   *
   * frame_->open_space_info().replan_flag()：
   *   获取重规划标志
   *   true表示需要重新规划
   *   false表示可以复用上一帧结果
   */
  if (!frame_->open_space_info().replan_flag()) {
    AINFO << "Not replan";
    /**
     * @brief 复用上一帧的路径规划结果
     *
     * *(frame_->mutable_open_space_info()->
     *     mutable_path_planning_trajectory_result())：
     *   获取当前帧的可变轨迹结果引用
     *
     * = previous_frame->open_space_info().path_planning_trajectory_result()：
     *   拷贝上一帧的结果
     */
    *(frame_->mutable_open_space_info()->
        mutable_path_planning_trajectory_result()) =
        previous_frame->open_space_info().path_planning_trajectory_result();
    return Status::OK();
  }
  /**
   * @brief 检查上一帧是否有路径规划结果
   */
  if (!previous_frame->open_space_info().
      path_planning_trajectory_result().empty()) {
    AINFO << "Previous frame has path planning result";
    /**
     * @brief 复用上一帧结果
     */
    *(frame_->mutable_open_space_info()->
        mutable_path_planning_trajectory_result()) =
        previous_frame->open_space_info().path_planning_trajectory_result();
    return Status::OK();
  }
  AINFO << "Path planning";

  /**
   * @brief 检查是否有更新的路径
   *
   * path_update_.load()：
   *   原子读取路径更新标志
   */
  if (path_update_) {
    AINFO << "Path planning updated";
    LoadResult(frame_->mutable_open_space_info()->
        mutable_path_planning_trajectory_result());
    path_update_.store(false);  /**< 重置更新标志 */
    return Status::OK();
  }
  /**
   * @brief 执行路径规划
   *
   * PathPlanning()：
   *   调用路径规划函数
   *   可能同步或异步执行
   */
  PathPlanning();
  return Status::OK();
}

/**
 * @brief 路径规划函数
 *
 * 功能说明：
 * 1. 如果启用异步线程，检查并启动路径规划线程
 * 2. 获取车辆当前状态
 * 3. 进行坐标归一化
 * 4. 调用HybridAStar进行路径搜索
 *
 * C++语法说明：
 * - cyber::Async(&ClassName::Method, this)：
 *   Cyber RT异步调用函数
 *   第一个参数：成员函数指针
 *   第二个参数：this指针，传递对象实例
 *   返回std::future用于获取结果
 *
 * - const auto&：常量引用，自动类型推导
 *   用于避免拷贝，提高性能
 *
 * - injector_->vehicle_state()：
 *   获取车辆状态
 *   ->x(), ->y(), ->heading()：
 *     获取车辆位置和航向角
 *
 * - std::lock_guard<std::mutex>：
 *   RAII风格的互斥锁
 *   构造时加锁，析构时自动解锁
 *   避免手动锁管理错误
 *
 * - std::initializer_list初始化：
 *   {start_x, start_y, start_theta}
 *   用于初始化std::array或其他容器
 */
void OpenSpacePathPlanning::PathPlanning() {
  /**
   * @brief 检查是否启用异步线程并初始化
   *
   * config_.enable_path_planning_thread()：
   *   配置项：是否启用路径规划线程
   *
   * cyber::Async(...)：
   *   Cyber RT的异步执行接口
   *   创建新线程执行指定函数
   */
  if (config_.enable_path_planning_thread() && !thread_init_flag_) {
    task_future_ = cyber::Async(
        &OpenSpacePathPlanning::GeneratePathThread, this);
    thread_init_flag_.store(true);  /**< 标记线程已初始化 */
  }

  /**
   * @brief 获取车辆当前状态
   *
   * injector_->vehicle_state()：
   *   获取车辆状态提供者
   *
   * ->x(), ->y(), ->heading()：
   *   获取车辆的位置(x,y)和航向角(theta)
   */
  const auto& open_space_info = frame_->open_space_info();
  double start_x = injector_->vehicle_state()->x();
  double start_y = injector_->vehicle_state()->y();
  double start_theta = injector_->vehicle_state()->heading();
  /**
   * @brief 坐标归一化
   *
   * 将全局坐标转换到局部坐标系
   * 便于后续的路径规划和优化
   *
   * OpenSpaceTrajectoryOptimizerUtil::PathPointNormalizing(...)：
   *   静态工具函数，进行坐标变换
   *   输入：航向角、原点、原始坐标
   *   输出：归一化后的坐标
   */
  OpenSpaceTrajectoryOptimizerUtil::PathPointNormalizing(
      open_space_info.origin_heading(),
      open_space_info.origin_point(),
      &start_x, &start_y, &start_theta);
  /**
   * @brief 根据配置选择同步或异步执行
   */
  if (config_.enable_path_planning_thread()) {
    if (!data_ready_) {
      /**
       * @brief 准备线程数据
       *
       * std::lock_guard<std::mutex> lock(data_mutex_)：
       *   RAII互斥锁
       *   构造时自动加锁
       *   析构时自动解锁
       */
      std::lock_guard<std::mutex> lock(data_mutex_);
      /**
       * @brief 设置起始位姿
       *
       * {start_x, start_y, start_theta}：
       *   std::initializer_list初始化
       *   用于初始化std::array或类似容器
       */
      thread_data_.start_pose = {start_x, start_y, start_theta};
      thread_data_.end_pose = open_space_info.open_space_end_pose();
      thread_data_.rotate_angle = open_space_info.origin_heading();
      thread_data_.translate_origin = open_space_info.origin_point();
      /**
       * @brief 设置障碍物信息
       *
       * 障碍物用凸多边形表示：
       * - edges_num：顶点数
       * - A, b：线性不等式 Ax <= b 表示多边形
       * - vertices_vec：顶点坐标列表
       */
      thread_data_.obstacles_edges_num = open_space_info.obstacles_edges_num();
      thread_data_.obstacles_A = open_space_info.obstacles_A();
      thread_data_.obstacles_b = open_space_info.obstacles_b();
      thread_data_.obstacles_vertices_vec =
          open_space_info.obstacles_vertices_vec();
      /**
       * @brief 设置软边界
       *
       * 软边界是允许侵入但会有惩罚的边界
       * 用于处理动态环境
       */
      thread_data_.soft_boundary_vertices_vec =
          open_space_info.soft_boundary_vertices_vec();
      thread_data_.XYbounds = open_space_info.ROI_xy_boundary();
      /**
       * @brief 设置最后一段是否直行
       *
       * ternary operator (? :)：
       *   条件表达式
       *   用于根据停车类型和配置决定是否直行
       */
      thread_data_.reeds_sheep_last_straight =
          ((config_.enable_vertical_parking_last_trajectory_straight() &&
          open_space_info.parking_type() == ParkingType::VERTICAL_PARKING) || 
          (config_.enable_parallel_parking_last_trajectory_straight() &&
          open_space_info.parking_type() == ParkingType::PARALLEL_PARKING)) ?
              true : false;
      data_ready_.store(true);  /**< 标记数据已准备好 */
    }
  } else {
    /**
     * @brief 同步执行路径规划
     */
    const auto& end_pose = open_space_info.open_space_end_pose();
    const auto& rotate_angle = open_space_info.origin_heading();
    const auto& translate_origin = open_space_info.origin_point();
    const auto& obstacles_edges_num = open_space_info.obstacles_edges_num();
    const auto& obstacles_A = open_space_info.obstacles_A();
    const auto& obstacles_b = open_space_info.obstacles_b();
    const auto& obstacles_vertices_vec =
        open_space_info.obstacles_vertices_vec();
    const auto& soft_boundary_vertices_vec =
        open_space_info.soft_boundary_vertices_vec();
    const auto& XYbounds = open_space_info.ROI_xy_boundary();
    const bool reeds_sheep_last_straight =
        ((config_.enable_vertical_parking_last_trajectory_straight() &&
          open_space_info.parking_type() == ParkingType::VERTICAL_PARKING) || 
          (config_.enable_parallel_parking_last_trajectory_straight() &&
          open_space_info.parking_type() == ParkingType::PARALLEL_PARKING)) ?
              true : false;
    /**
     * @brief 调用HybridAStar进行路径规划
     *
     * path_finder_->Plan(...)：
     *   HybridAStar的Plan方法
     *   混合A*路径搜索算法
     *
     * 参数说明：
     * - start_x, start_y, start_theta：起始位姿
     * - end_pose[0], [1], [2]：终点位姿(x, y, theta)
     * - XYbounds：搜索区域边界
     * - obstacles_vertices_vec：障碍物顶点列表
     * - soft_boundary_vertices_vec：软边界
     * - reeds_sheep_last_straight：最后是否直行
     * - &result_：输出参数，存储规划结果
     */
    if (path_finder_->Plan(start_x, start_y, start_theta,
                           end_pose[0], end_pose[1],
                           end_pose[2], XYbounds,
                           obstacles_vertices_vec,
                           &result_,
                           soft_boundary_vertices_vec,
                           reeds_sheep_last_straight)) {
      path_update_.store(true);
      AINFO << "State warm start problem solved successfully!";
    } else {
      AERROR << "State warm start problem failed to solve";
    }
  }
}

/**
 * @brief 路径规划线程函数
 *
 * 功能说明：
 * 独立的路径规划线程函数
 * 从线程数据中获取参数
 * 执行HybridAStar路径搜索
 *
 * C++语法说明：
 * - void GeneratePathThread()：
 *   线程入口函数
 *   无参数，通过成员变量获取数据
 *
 * - usleep(10)：
 *   线程休眠10微秒
 *   等待数据准备完成
 *
 * - std::atomic::load()：
 *   原子读取布尔值
 *   检查data_ready_标志
 *
 * - std::lock_guard<std::mutex> lock(data_mutex_)：
 *   RAII互斥锁
 *   确保数据拷贝的线程安全
 *
 * - path_finder_.reset(new HybridAStar(...))：
 *   重置智能指针
 *   重新创建HybridAStar实例
 *   用于重置搜索器状态
 */
void OpenSpacePathPlanning::GeneratePathThread() {
  /**
   * @brief 等待数据准备完成
   *
   * while (!data_ready_.load())：
   *   原子读取data_ready_标志
   *   如果未准备好则继续等待
   *
   * usleep(10)：
   *   休眠10微秒
   *   避免CPU忙等待
   */
  while (!data_ready_) {
    usleep(10);
  }
  /**
   * @brief 拷贝线程数据
   *
   * std::lock_guard<std::mutex>：
   *   RAII锁，自动管理加锁/解锁
   *   确保线程间数据安全
   */
  OpenSpacePathPlanningThreadData thread_data;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    thread_data = thread_data_;  /**< 拷贝数据 */
  }

  AINFO << "start pose: " << thread_data.start_pose[0]
        << " " << thread_data.start_pose[1]
        << " " << thread_data.start_pose[2];
  /**
   * @brief 调用HybridAStar进行路径搜索
   */
  bool ret = path_finder_->Plan(
      thread_data.start_pose[0], thread_data.start_pose[1],
      thread_data.start_pose[2], thread_data.end_pose[0],
      thread_data.end_pose[1], thread_data.end_pose[2],
      thread_data.XYbounds, thread_data.obstacles_vertices_vec,
      &result_,thread_data_.soft_boundary_vertices_vec,
      thread_data.reeds_sheep_last_straight);

  /**
   * @brief 处理搜索结果
   */
  if (ret) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    path_update_.store(true);  /**< 标记路径已更新 */
    AINFO << "path find success!";
  } else {
    AERROR << "path find failed";
  }
  /**
   * @brief 重置状态和搜索器
   */
  thread_init_flag_.store(false);
  data_ready_.store(false);
  /**
   * @brief 重置HybridAStar搜索器
   *
   * path_finder_.reset(new HybridAStar(...))：
   *   重新创建搜索器实例
   *   清除内部状态
   *   准备下一次规划
   */
  path_finder_.reset(new HybridAStar(config_.warm_start_config()));
}

/**
 * @brief 加载结果函数
 *
 * @param trajectory_data 输出参数，轨迹数据指针
 *
 * 功能说明：
 * 1. 将规划结果转换为轨迹格式
 * 2. 进行坐标反归一化
 * 3. 构建完整的轨迹点信息
 *
 * C++语法说明：
 * - DiscretizedTrajectory* const trajectory_data：
 *   指向常量的指针
 *   指针本身是常量，不能改变指向
 *   但可以修改所指对象的内容
 *
 * - trajectory_data->clear()：
 *   清空轨迹容器
 *   准备填充新的轨迹点
 *
 * - result_.x.back() = value：
 *   获取容器最后一个元素的引用并赋值
 *   确保终点位姿正确
 *
 * - for (size_t i = 0; i < result_.x.size(); ++i)：
 *   size_t：无符号整数，适合表示容器大小
 *   .size()：返回容器元素个数
 *
 * - OpenSpaceTrajectoryOptimizerUtil::PathPointDeNormalizing(...)：
 *   静态工具函数
 *   进行坐标反归一化（归一化的逆操作）
 *
 * - common::TrajectoryPoint：
 *   轨迹点结构
 *   包含位置、航向、速度、加速度等信息
 *
 * - point.mutable_path_point()：
 *   protobuf可变字段访问
 *   获取可修改的path_point引用
 *
 * - .set_x(), .set_y(), .set_theta()：
 *   protobuf的setter方法
 *   设置各字段的值
 *
 * - .set_v(), .set_steer(), .set_a()：
 *   设置速度、转向角、加速度
 *
 * - .set_relative_time(i * 0.5)：
 *   设置相对时间
 *   每个点间隔0.5秒
 *
 * - trajectory_data->emplace_back(point)：
 *   在容器末尾直接构造元素
 *   比push_back更高效
 *   使用移动语义避免拷贝
 */
void OpenSpacePathPlanning::LoadResult(
    DiscretizedTrajectory* const trajectory_data) {
  trajectory_data->clear();
  /**
   * @brief 确保终点位姿正确
   *
   * result_.x.back() = ...：
   *   获取结果向量最后一个元素的引用
   *   将终点x坐标设置为目标位置
   */
  result_.x.back() = frame_->open_space_info().open_space_end_pose()[0];
  result_.y.back() = frame_->open_space_info().open_space_end_pose()[1];
  result_.phi.back() = frame_->open_space_info().open_space_end_pose()[2];
  /**
   * @brief 遍历所有路径点
   */
  for (size_t i = 0; i < result_.x.size(); i++) {
    /**
     * @brief 坐标反归一化
     *
     * 将局部坐标转换回全局坐标
     * OpenSpaceTrajectoryOptimizerUtil::PathPointDeNormalizing：
     *   静态成员函数
     *   第一个参数：航向角
     *   第二个参数：原点
     *   第三、四个参数：要反归一化的坐标（指针输入输出）
     */
    OpenSpaceTrajectoryOptimizerUtil::PathPointDeNormalizing(
        frame_->open_space_info().origin_heading(),
        frame_->open_space_info().origin_point(),
        &result_.x[i],
        &result_.y[i],
        &result_.phi[i]);
    /**
     * @brief 创建轨迹点
     *
     * common::TrajectoryPoint：
     *   Apollo的轨迹点数据结构
     *   包含路径点和时间/速度信息
     */
    common::TrajectoryPoint point;
    /**
     * @brief 设置路径点信息
     *
     * mutable_path_point()：
     *   protobuf可变方法
     *   返回可修改的PathPoint引用
     *
     * set_x(), set_y(), set_theta()：
     *   设置位置和航向
     */
    point.mutable_path_point()->set_x(result_.x[i]);
    point.mutable_path_point()->set_y(result_.y[i]);
    point.mutable_path_point()->set_theta(result_.phi[i]);
    /**
     * @brief 设置动力学参数
     */
    point.set_v(result_.v[i]);       /**< 设置速度 */
    point.set_steer(result_.steer[i]);  /**< 设置转向角 */
    point.set_a(result_.a[i]);       /**< 设置加速度 */
    /**
     * @brief 设置时间戳
     *
     * set_relative_time(i * 0.5)：
     *   每个轨迹点间隔0.5秒
     *   i=0时为0秒，i=1时为0.5秒，依次类推
     */
    point.set_relative_time(i * 0.5);
    /**
     * @brief 添加轨迹点
     *
     * emplace_back(point)：
     *   在容器末尾构造元素
     *   使用移动语义避免拷贝
     */
    trajectory_data->emplace_back(point);
  }
}

/**
 * @brief 命名空间结束标记
 *
 * C++语法说明：
 * }  // namespace planning：
 *   单行注释说明命名空间结束
 *   便于代码阅读
 */
}  // namespace planning
}  // namespace apollo
