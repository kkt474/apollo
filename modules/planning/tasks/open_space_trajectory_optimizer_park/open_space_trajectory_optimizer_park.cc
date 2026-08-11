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
 * @file open_space_trajectory_optimizer_park.cc
 *
 * @brief 开放空间轨迹优化器（停车场场景）实现文件
 *
 * 功能说明：
 * 本文件实现了 OpenSpaceTrajectoryOptimizerPark 类
 * 负责停车场等开放空间场景的轨迹优化
 * 使用混合A*初始解 +迭代优化算法
 *
 * 核心概念：
 * - 轨迹优化：在给定边界约束下优化轨迹
 * - 开放空间规划：停车场等非结构化场景
 * - 迭代优化：通过迭代求解非线性优化问题
 *
 * C++语法说明：
 * - namespace：命名空间，避免命名冲突
 * - class：类声明
 * - std::shared_ptr：智能指针，引用计数管理
 * - std::atomic<bool>：原子布尔类型
 * - std::mutex：互斥锁
 * - std::future：异步操作结果
 **/

#include "modules/planning/tasks/open_space_trajectory_optimizer_park/open_space_trajectory_optimizer_park.h"

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
 * - apollo::common::ErrorCode：Apollo错误码枚举
 * - apollo::common::Status：Apollo通用状态类型
 */
using apollo::common::ErrorCode;
using apollo::common::Status;

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
 * 2. 加载开放空间轨迹优化器配置
 * 3. 创建轨迹优化器实例
 *
 * C++语法说明：
 * - const std::string &config_dir：
 *   常量引用参数，承诺不修改，避免拷贝
 *
 * - const std::shared_ptr<DependencyInjector> &injector：
 *   shared_ptr智能指针的常量引用
 *   共享所有权，引用计数管理生命周期
 *
 * - Task::Init(config_dir, name, injector)：
 *   调用基类Task的Init方法
 *   ::作用域限定符
 *
 * - Task::LoadConfig<OpenSpaceTrajectoryOptimizerParkConfig>(&config_)：
 *   模板方法调用，加载特定类型的protobuf配置
 *   LoadConfig<T>是模板函数，T是配置类型
 *   &config_：输出参数，存储加载的配置
 *
 * - trajectory_optimizer_.reset(new Optimizer(config_))：
 *   reset()重置智能指针
 *   在堆上创建新的Optimizer实例
 *   使用配置初始化优化器
 */
bool OpenSpaceTrajectoryOptimizerPark::Init(
    const std::string &config_dir, const std::string &name,
    const std::shared_ptr<DependencyInjector> &injector) {
  /**
   * @brief 调用基类初始化
   *
   * Task::Init(config_dir, name, injector)：
   *   调用父类Task的Init方法
   *   进行通用的初始化工作
   *   返回false表示初始化失败
   */
  if (!Task::Init(config_dir, name, injector)) {
    return false;
  }
  /**
   * @brief 加载轨迹优化器配置
   *
   * Task::LoadConfig<OpenSpaceTrajectoryOptimizerParkConfig>：
   *   模板方法，加载指定类型的protobuf配置
   *   &config_：输出参数，存储加载的配置
   *   返回false表示加载失败
   */
  if (!Task::LoadConfig<OpenSpaceTrajectoryOptimizerParkConfig>(&config_)) {
    return false;
  }
  /**
   * @brief 创建轨迹优化器实例
   *
   * trajectory_optimizer_.reset(new Optimizer(config_))：
   *   reset()：重置智能指针
   *   new Optimizer(config_)：在堆上创建优化器实例
   *   config_：使用配置初始化优化器
   *
   * trajectory_optimizer_：
   *   类成员变量，shared_ptr类型
   *   管理Optimizer对象的生命周期
   */
  trajectory_optimizer_.reset(new Optimizer(config_));
  return true;
}

/**
 * @brief 析构函数
 *
 * 功能说明：
 * 如果启用了轨迹优化线程，则在析构时停止线程
 * 确保线程安全退出并清理资源
 *
 * C++语法说明：
 * - ~OpenSpaceTrajectoryOptimizerPark()：
 *   析构函数，类名加~前缀
 *   对象生命周期结束时自动调用
 *
 * - config_.enable_trajectory_optimize_thread()：
 *   protobuf配置的布尔字段访问方法
 *   返回是否启用异步线程优化
 *
 * - Stop()：
 *   停止线程的方法
 */
OpenSpaceTrajectoryOptimizerPark::~OpenSpaceTrajectoryOptimizerPark() {
  /**
   * @brief 检查是否启用了轨迹优化线程
   */
  if (config_.enable_trajectory_optimize_thread()) {
    Stop();  /**< 调用Stop方法停止线程 */
  }
}

/**
 * @brief 停止函数
 *
 * 功能说明：
 * 停止异步轨迹优化线程
 * 确保线程安全退出并清理资源
 *
 * C++语法说明：
 * - std::atomic<bool>：
 *   原子布尔类型
 *   - .store()：原子写入
 *   - .load()：原子读取
 *   保证多线程下的可见性
 *
 * - thread_init_flag_：
 *   线程初始化标志
 *   原子类型，支持多线程安全读写
 *
 * - task_future_.get()：
 *   获取异步任务的结果
 *   .get()会阻塞直到任务完成
 *   用于确保线程安全退出
 */
void OpenSpaceTrajectoryOptimizerPark::Stop() {
  if (config_.enable_trajectory_optimize_thread()) {
    if (thread_init_flag_) {
      task_future_.get();  /**< 等待线程完成，阻塞调用 */
    }
  }
}

/**
 * @brief 主处理函数
 *
 * @return Status 处理状态
 *
 * 功能说明：
 * 开放空间轨迹优化的主入口函数
 * 1. 检查是否需要重规划
 * 2. 如果有上一帧结果则复用
 * 3. 否则执行轨迹优化
 *
 * C++语法说明：
 * - Status：Apollo通用状态类型
 *   - Status::OK()：成功状态
 *   - 可包含错误码和消息
 *
 * - injector_->frame_history()->Latest()：
 *   链式调用获取历史帧
 *   ->frame_history()：调用injector的方法
 *   ->Latest()：获取最新的历史帧
 *
 * - frame_->open_space_info()：
 *   获取开源空间信息
 *   frame_是当前帧指针
 *
 * - mutable_open_space_info()：
 *   protobuf可变成员访问方法
 *   用于获取可修改的引用
 *
 * - FLAGS_enable_record_debug：
 *   gflags全局配置变量
 *   用于控制是否记录调试信息
 *
 * - *(ptr) = value：
 *   解引用指针并赋值
 *   拷贝赋值
 */
Status OpenSpaceTrajectoryOptimizerPark::Process() {
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
     * @brief 复用上一帧的优化轨迹结果
     *
     * *(frame_->mutable_open_space_info()->
     *     mutable_optimizer_trajectory_data())：
     *   获取当前帧的可变轨迹数据引用
     *
     * = previous_frame->open_space_info().optimizer_trajectory_data()：
     *   拷贝上一帧的优化轨迹数据
     */
    *(frame_->mutable_open_space_info()->mutable_optimizer_trajectory_data()) =
        previous_frame->open_space_info().optimizer_trajectory_data();
    /**
     * @brief 复制开放空间提供者成功标志
     */
    frame_->mutable_open_space_info()->set_open_space_provider_success(
      previous_frame->open_space_info().open_space_provider_success());
    /**
     * @brief 复制调试信息（如果启用）
     *
     * FLAGS_enable_record_debug：
     *   gflags全局布尔变量
     *   控制是否启用调试信息记录
     */
    if (FLAGS_enable_record_debug) {
      // 复制上一帧的调试信息
      ReuseLastFrameDebug(previous_frame);
    }
    return Status::OK();
  }

  /**
   * @brief 重置规划标志
   *
   * replan_flag_：
   *   成员变量，表示需要重规划
   *   设置为false，等待重新规划
   *
   * ->mutable_path_planning_trajectory_result()->clear()：
   *   清空路径规划轨迹结果
   *   为新的优化做准备
   */
  if (replan_flag_) {
    replan_flag_ = false;
    frame_->mutable_open_space_info()->
        mutable_path_planning_trajectory_result()->clear();
  }

  /**
   * @brief 检查是否有初始轨迹
   *
   * path_planning_trajectory_result()：
   *   路径规划阶段的轨迹结果
   *   轨迹优化需要初始解
   *
   * .empty()：
   *   检查容器是否为空
   */
  if (frame_->open_space_info().path_planning_trajectory_result().empty()) {
    AINFO << "No initial trajectory solution";
    return Status::OK();
  }

  /**
   * @brief 执行轨迹优化
   *
   * Optimize()：
   *   调用优化函数
   *   可能同步或异步执行
   */
  Optimize();

  /**
   * @brief 处理优化结果
   */
  if (trajectory_update_) {
    /**
     * @brief 复制调试信息到当前帧
     *
     * frame_->mutable_open_space_info()->mutable_debug()：
     *   获取可变debug信息指针
     *
     * trajectory_optimizer_->UpdateDebugInfo(...)：
     *   更新调试信息
     *   优化器将内部调试数据导出
     */
    auto* ptr_debug = frame_->mutable_open_space_info()->mutable_debug();
    trajectory_optimizer_->UpdateDebugInfo(
        ptr_debug->mutable_planning_data()->mutable_open_space());
    /**
     * @brief 同步调试实例
     */
    frame_->mutable_open_space_info()->sync_debug_instance();

    /**
     * @brief 加载优化结果到轨迹数据
     */
    LoadResult(frame_->mutable_open_space_info()->
        mutable_optimizer_trajectory_data());
    /**
     * @brief 重置状态标志
     */
    thread_init_flag_ = false;
    trajectory_update_ = false;
  }

  return Status::OK();
}

/**
 * @brief 优化函数
 *
 * 功能说明：
 * 1. 如果启用异步线程，启动独立的优化线程
 * 2. 否则同步执行优化
 *
 * C++语法说明：
 * - cyber::Async(&ClassName::Method, this)：
 *   Cyber RT异步调用函数
 *   第一个参数：成员函数指针
 *   第二个参数：this指针，传递对象实例
 *   返回std::future用于获取结果
 *
 * - thread_init_flag_：
 *   原子布尔类型
 *   标记线程是否已初始化
 *
 * - GenerateTrajectoryThread()：
 *   实际的优化线程函数
 */
void OpenSpaceTrajectoryOptimizerPark::Optimize() {
  /**
   * @brief 检查是否启用异步优化
   */
  if (config_.enable_trajectory_optimize_thread()) {
    if (thread_init_flag_) {
      AINFO << "trajectory is optimizing, please wait!";
      return;
    }

    /**
     * @brief 启动异步优化线程
     *
     * cyber::Async(&OpenSpaceTrajectoryOptimizerPark::GenerateTrajectoryThread, this)：
     *   Cyber RT的异步执行接口
     *   创建新线程执行GenerateTrajectoryThread
     *   返回std::future用于获取结果
     *
     * task_future_：
     *   成员变量，存储future
     *   用于后续获取结果或等待完成
     */
    task_future_ = cyber::Async(
        &OpenSpaceTrajectoryOptimizerPark::GenerateTrajectoryThread, this);
    thread_init_flag_.store(true);  /**< 标记线程已初始化 */
  } else {
    /**
     * @brief 同步执行优化
     */
    GenerateTrajectoryThread();
    LoadResult(frame_->mutable_open_space_info()->
        mutable_optimizer_trajectory_data());
  }
}

/**
 * @brief 轨迹优化线程函数
 *
 * 功能说明：
 * 独立的轨迹优化线程函数
 * 调用优化器进行轨迹优化
 *
 * C++语法说明：
 * - void GenerateTrajectoryThread()：
 *   线程入口函数
 *   无参数，通过成员变量获取数据
 *
 * - trajectory_optimizer_->Plan(...)：
 *   Optimizer的Plan方法
 *   执行实际的轨迹优化算法
 *
 * - std::lock_guard<std::mutex>：
 *   RAII风格的互斥锁
 *   构造时加锁，析构时自动解锁
 *   避免手动锁管理错误
 *
 * - trajectory_update_.store(true)：
 *   原子写入，标记轨迹已更新
 */
void OpenSpaceTrajectoryOptimizerPark::GenerateTrajectoryThread() {
  /**
   * @brief 调用优化器执行轨迹优化
   *
   * trajectory_optimizer_->Plan(frame_->open_space_info())：
   *   Plan方法：
   *   - 输入：开放空间信息（边界、障碍物、起点终点等）
   *   - 输出：Status（成功/失败）
   *   - 算法：混合A* + 迭代优化
   *
   * frame_->open_space_info()：
   *   获取当前帧的开放空间信息
   *   包含所有优化所需的输入数据
   */
  Status ret = trajectory_optimizer_->Plan(frame_->open_space_info());

  /**
   * @brief 处理优化结果
   */
  if (ret == Status::OK()) {
    /**
     * @brief 优化成功
     *
     * std::lock_guard<std::mutex> lock(data_mutex_)：
     *   RAII互斥锁
     *   构造时自动加锁
     *   析构时自动解锁
     *   确保线程间数据安全
     */
    std::lock_guard<std::mutex> lock(data_mutex_);
    trajectory_update_.store(true);  /**< 标记轨迹已更新 */
    AINFO << "tarjectory optimize successfully!";
  } else {
    /**
     * @brief 优化失败
     *
     * thread_init_flag_：
     *   重置线程初始化标志
     *   允许下次重新启动优化
     *
     * replan_flag_：
     *   设置为true
     *   通知上层需要重新规划
     */
    thread_init_flag_.store(false);
    replan_flag_ = true;
    AERROR << "tarjectory optimize failed";
  }
}

/**
 * @brief 加载结果函数
 *
 * @param trajectory_data 输出参数，轨迹数据指针
 *
 * 功能说明：
 * 1. 清空轨迹容器
 * 2. 从优化器获取优化后的轨迹
 * 3. 设置开放空间提供者成功标志
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
 * - trajectory_optimizer_->GetOptimizedTrajectory(*trajectory_data)：
 *   从优化器获取优化后的轨迹
 *   *trajectory_data解引用传递可变引用
 *
 * - set_open_space_provider_success(true)：
 *   设置开放空间提供者成功标志
 *   表示轨迹优化阶段成功
 */
void OpenSpaceTrajectoryOptimizerPark::LoadResult(
    DiscretizedTrajectory* const trajectory_data) {
  /**
   * @brief 清空轨迹数据
   *
   * clear()：
   *   清空容器中的所有元素
   *   释放内存（可选）
   */
  trajectory_data->clear();

  /**
   * @brief 获取优化后的轨迹
   *
   * trajectory_optimizer_->GetOptimizedTrajectory(*trajectory_data)：
   *   获取优化器内部存储的优化轨迹
   *   *trajectory_data：解引用获取引用
   *   轨迹数据通过参数输出
   */
  trajectory_optimizer_->GetOptimizedTrajectory(
      *trajectory_data);

  /**
   * @brief 设置成功标志
   */
  frame_->mutable_open_space_info()->set_open_space_provider_success(true);
}

/**
 * @brief 复用上一帧调试信息
 *
 * @param last_frame 上一帧指针
 *
 * 功能说明：
 * 1. 复制上一帧的调试实例
 * 2. 加载当前帧的障碍物信息
 *
 * C++语法说明：
 * - const Frame* last_frame：
 *   指向常量的指针
 *   函数承诺不修改last_frame
 *
 * - ptr_debug->mutable_planning_data()->mutable_open_space()->MergeFrom(...)：
 *   protobuf的MergeFrom方法
 *   合并两个消息，保留所有字段
 *
 * - last_frame->open_space_info().debug_instance()：
 *   获取上一帧的调试实例
 *
 * - add_obstacles()：
 *   protobuf的repeated字段添加方法
 *   返回新添加元素的指针
 *
 * - add_vertices_x_coords(vertex.x())：
 *   向repeated字段添加元素
 *   x坐标
 *
 * - add_vertices_y_coords(vertex.y())：
 *   向repeated字段添加元素
 *   y坐标
 */
void OpenSpaceTrajectoryOptimizerPark::ReuseLastFrameDebug(
    const Frame* last_frame) {
  /**
   * @brief 复制上一帧的调试实例
   *
   * frame_->mutable_open_space_info()->mutable_debug_instance()：
   *   获取当前帧的可变调试实例
   *
   * ->mutable_planning_data()->mutable_open_space()：
   *   获取planning_data的open_space子消息
   *
   * ->MergeFrom(last_frame->...):
   *   protobuf合并方法
   *   从last_frame复制所有调试数据
   */
  auto* ptr_debug = frame_->mutable_open_space_info()->mutable_debug_instance();
  ptr_debug->mutable_planning_data()->mutable_open_space()->MergeFrom(
      last_frame->open_space_info()
          .debug_instance()
          .planning_data()
          .open_space());
  /**
   * @brief 清空障碍物列表
   *
   * clear_obstacles()：
   *   protobuf的repeated字段清空方法
   *   为加载新的障碍物数据做准备
   */
  ptr_debug->mutable_planning_data()->mutable_open_space()->clear_obstacles();
  /**
   * @brief 加载当前帧的障碍物
   *
   * for (const auto& obstacle_vertices : ...):
   *   范围for循环
   *   遍历所有障碍物的顶点列表
   *
   * frame_->open_space_info().obstacles_vertices_vec()：
   *   获取障碍物顶点向量
   *   obstacles_vertices_vec是repeated类型
   *
   * add_obstacles()：
   *   添加一个新的障碍物消息
   *   返回可变指针用于设置字段
   */
  for (const auto& obstacle_vertices :
      frame_->open_space_info().obstacles_vertices_vec()) {
    /**
     * @brief 添加单个障碍物
     */
    auto* obstacle_ptr = ptr_debug->mutable_planning_data()->
        mutable_open_space()->add_obstacles();
    /**
     * @brief 遍历障碍物的顶点
     *
     * for (const auto& vertex : obstacle_vertices)：
     *   遍历单个障碍物的所有顶点
     *
     * add_vertices_x_coords(vertex.x())：
     *   添加顶点的x坐标
     *   protobuf repeated字段添加方法
     *
     * add_vertices_y_coords(vertex.y())：
     *   添加顶点的y坐标
     */
    for (const auto& vertex : obstacle_vertices) {
      obstacle_ptr->add_vertices_x_coords(vertex.x());
      obstacle_ptr->add_vertices_y_coords(vertex.y());
    }
  }
}

/**
 * @brief 命名空间结束标记
 *
 * C++语法说明：
 * }  // namespace planning：
 *   单行注释说明命名空间结束
 *   便于代码阅读和导航
 */
}  // namespace planning
}  // namespace apollo
