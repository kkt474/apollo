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
 * @file stage.cc
 * @brief Stage(阶段)基类实现文件
 *
 * 功能说明：
 * Stage是Apollo规划模块中所有阶段的基类
 * 阶段是场景的子单元，负责具体的规划和决策任务
 *
 * 设计理念：
 * 1. 阶段由多个任务(Task)组成
 * 2. 每个任务执行特定的规划子功能
 * 3. 支持在参考线上执行和开放空间执行两种模式
 * 4. 提供后备轨迹生成机制确保安全
 *
 * 任务执行流程：
 * 1. Init() - 初始化阶段，加载任务插件
 * 2. ExecuteTaskOnReferenceLine() - 在参考线上执行任务
 * 3. 如果出错，执行后备任务(FastStopTrajectoryFallback)
 * 4. CombinePathAndSpeedProfile() - 合并路径和速度生成轨迹
 * 5. SetTrajectory() - 设置最终轨迹
 *
 * 应用场景：
 * - 车道跟随
 * - 车道变更
 * - 自主泊车
 * - 开放空间规划
 */

/**
 * @brief 头文件包含
 *
 * C++标准库头文件：
 * - <unordered_map>：基于哈希表的键值对容器，提供O(1)查找
 * - <utility>：工具库，提供std::pair等
 *
 * Apollo/Cyber RT头文件：
 * - plugin_manager.h：插件管理器，支持动态加载任务
 * - clock.h：时钟服务，获取当前时间
 *
 * 规划模块头文件：
 * - frame.h：规划帧数据结构
 * - planning_context.h：规划上下文
 * - speed_profile_generator.h：速度轮廓生成器
 * - publishable_trajectory.h：可发布轨迹
 * - config_util.h：配置工具
 * - task.h：任务基类
 */

/**
 * @brief Stage基类实现文件
 *
 * 功能说明：
 * 包含Stage类的所有方法实现
 */
#include "modules/planning/planning_interface_base/scenario_base/stage.h"

/**
 * @brief C++标准库无序映射头文件
 *
 * C++语法说明：
 * - #include <unordered_map>：
 *   无序映射容器头文件
 *   基于哈希表实现，提供平均O(1)时间复杂度的查找
 *   键值对存储，适合需要频繁查找的配置场景
 */
#include <unordered_map>

/**
 * @brief C++标准库工具头文件
 *
 * C++语法说明：
 * - #include <utility>：
 *   提供了std::pair等工具
 *   std::pair表示两个值的元组
 */
#include <utility>

/**
 * @brief Cyber RT插件管理器头文件
 *
 * 功能说明：
 * - cyber/plugin_manager/plugin_manager.h：
 *   Apollo Cyber RT的插件系统
 *   允许动态加载和创建任务实例
 *
 * C++语法说明：
 * - PluginManager::Instance()：
 *   单例模式获取插件管理器实例
 */
#include "cyber/plugin_manager/plugin_manager.h"

/**
 * @brief Cyber RT时钟头文件
 *
 * 功能说明：
 * - cyber/time/clock.h：
 *   提供时间获取功能
 *   Clock::NowInSeconds()获取当前时间戳
 *
 * C++语法说明：
 * - apollo::cyber::Clock：
 *   Clock类用于获取系统时间
 *   支持多种时间格式
 */
#include "cyber/time/clock.h"

/**
 * @brief 规划帧头文件
 *
 * 功能说明：
 * - modules/planning/planning_base/common/frame.h：
 *   Frame类定义
 *   包含当前帧的所有规划相关信息
 *
 * C++语法说明：
 * - Frame*：
 *   规划帧指针
 *   用于在任务间传递规划数据
 */
#include "modules/planning/planning_base/common/frame.h"

/**
 * @brief 规划上下文头文件
 *
 * 功能说明：
 * - planning_context.h：
 *   包含规划上下文定义
 *   用于存储跨帧的规划状态
 */
#include "modules/planning/planning_base/common/planning_context.h"

/**
 * @brief 速度轮廓生成器头文件
 *
 * 功能说明：
 * - speed_profile_generator.h：
 *   用于生成速度轮廓
 *   将路径和速度信息结合生成轨迹
 */
#include "modules/planning/planning_base/common/speed_profile_generator.h"

/**
 * @brief 可发布轨迹头文件
 *
 * 功能说明：
 * - publishable_trajectory.h：
 *   PublishableTrajectory类
 *   将轨迹包装成可发布格式
 */
#include "modules/planning/planning_base/common/trajectory/publishable_trajectory.h"

/**
 * @brief 配置工具头文件
 *
 * 功能说明：
 * - config_util.h：
 *   ConfigUtil类
 *   提供配置文件加载和路径转换功能
 */
#include "modules/planning/planning_base/common/util/config_util.h"

/**
 * @brief 任务基类头文件
 *
 * 功能说明：
 * - task.h：
 *   Task类定义
 *   所有规划任务继承自此类
 */
#include "modules/planning/planning_interface_base/task_base/task.h"

/**
 * @namespace apollo::cyber
 * @brief Apollo Cyber RT命名空间
 *
 * C++语法说明：
 * - namespace apollo：
 *   Apollo最外层命名空间
 * - namespace cyber：
 *   Cyber RT框架子命名空间
 */
namespace apollo {
namespace cyber {

/**
 * @brief Clock类型别名
 *
 * 功能说明：
 * 为apollo::cyber::Clock创建别名
 * 简化后续代码书写
 *
 * C++语法说明：
 * - using Clock = apollo::cyber::Clock：
 *   类型别名声明，与typedef等价但更直观
 *   允许直接使用Clock代替完整命名空间路径
 */
using apollo::cyber::Clock;

}  // namespace cyber
}  // namespace apollo

/**
 * @namespace apollo::planning
 * @brief Apollo规划模块命名空间
 */
namespace apollo {
namespace planning {

/**
 * @brief Stage默认构造函数
 *
 * 功能说明：
 * 初始化Stage的所有成员变量为默认值
 *
 * 初始化列表：
 * - next_stage_ = ""：空字符串，表示无下一阶段
 * - context_ = nullptr：空指针，表示无上下文
 * - injector_ = nullptr：空指针，表示无依赖注入器
 * - name_ = ""：空字符串，表示无名称
 *
 * C++语法说明：
 * - Stage() :
 *   构造函数声明
 *
 * - : next_stage_(""), context_(nullptr), injector_(nullptr), name_("")：
 *   构造函数的初始化列表
 *   在构造函数体执行前初始化成员变量
 *   效率比在函数体内赋值更高
 *   对于const成员和引用成员必须使用初始化列表
 *
 * - next_stage_("")：
 *   使用空字符串初始化std::string
 *
 * - context_(nullptr)：
 *   使用nullptr初始化void*指针
 *
 * 示例：
 * @code
 *   Stage stage;  // 调用默认构造函数
 *   // 所有成员被初始化为默认值
 * @endcode
 */
Stage::Stage()
    : next_stage_(""), context_(nullptr), injector_(nullptr), name_("") {}

/**
 * @brief 初始化阶段
 *
 * @param config 阶段管道配置
 * @param injector 依赖注入器
 * @param config_dir 配置文件目录
 * @param context 场景上下文
 * @return bool 初始化是否成功
 *
 * 功能说明：
 * 初始化阶段的内部状态，包括：
 * 1. 保存配置和依赖注入器
 * 2. 更新规划状态中的阶段类型
 * 3. 加载任务插件列表
 * 4. 加载后备任务
 *
 * 算法流程：
 * 1. 保存配置到成员变量pipeline_config_
 * 2. 从配置中获取阶段名称
 * 3. 更新规划上下文中的阶段类型
 * 4. 遍历配置中的任务列表，加载每个任务插件
 * 5. 加载后备任务用于错误处理
 *
 * C++语法说明：
 * - const StagePipeline& config：
 *   常量引用，传入阶段配置
 *   const保证配置不会被修改
 *
 * - const std::shared_ptr<DependencyInjector>& injector：
 *   共享指针的常量引用
 *   shared_ptr允许多个所有者共享对象
 *
 * - void* context：
 *   通用指针类型，可以指向任何类型
 *   用于传递场景上下文
 *
 * - pipeline_config_ = config：
 *   赋值操作
 *   Protobuf消息支持赋值运算符
 *
 * - injector_->planning_context()：
 *   -> 调用依赖注入器的方法
 *   injector_是shared_ptr，需要用->访问
 *
 * - mutable_planning_status()：
 *   Protobuf的mutable_前缀方法
 *   返回可修改的状态指针
 *
 * - ConfigUtil::TransformToPathName(name_)：
 *   静态方法调用
 *   ::作用域解析符访问命名空间中的类方法
 *
 * - pipeline_config_.task_size()：
 *   Protobuf的重复字段方法
 *   返回任务列表的大小
 *
 * - pipeline_config_.task(i)：
 *   获取第i个任务配置
 *
 * - apollo::cyber::plugin_manager::PluginManager::Instance()：
 *   单例模式获取插件管理器实例
 *
 * - PluginManager::CreateInstance<Task>(...)：
 *   动态创建Task类型的实例
 *   模板参数指定要创建的类类型
 *
 * - task_list_.push_back(task_ptr)：
 *   将任务智能指针添加到列表
 *   push_back在vector末尾添加元素
 *
 * - if (!fallback_task_->Init(...))：
 *   逻辑非运算符!判断初始化是否失败
 */
bool Stage::Init(const StagePipeline& config,
                 const std::shared_ptr<DependencyInjector>& injector,
                 const std::string& config_dir, void* context) {
  // 保存阶段管道配置
  pipeline_config_ = config;

  // 从配置中获取阶段名称并设置为下一阶段
  next_stage_ = config.name();

  // 保存依赖注入器
  injector_ = injector;

  // 保存阶段名称
  name_ = config.name();

  // 保存上下文指针
  context_ = context;

  /**
   * 更新规划上下文中的阶段类型
   *
   * injector_->planning_context()：
   *   获取规划上下文
   *
   * ->mutable_planning_status()：
   *   获取可修改的规划状态指针
   *
   * ->mutable_scenario()：
   *   获取可修改的场景状态
   *
   * ->set_stage_type(name_)：
   *   设置当前阶段类型为阶段名称
   */
  injector_->planning_context()
      ->mutable_planning_status()
      ->mutable_scenario()
      ->set_stage_type(name_);

  /**
   * 构建任务配置目录路径
   *
   * ConfigUtil::TransformToPathName(name_)：
   *   将阶段名称转换为路径格式
   *   通常是将名称转换为小写
   *
   * config_dir + "/" + path_name：
   *   字符串拼接
   *   构建完整的任务配置目录路径
   *
   * 示例：
   *   config_dir = "/apollo/config"
   *   name_ = "LaneFollowScenario"
   *   path_name = "lanefollowscenario"
   *   task_config_dir = "/apollo/config/lanefollowscenario"
   */
  std::string path_name = ConfigUtil::TransformToPathName(name_);
  std::string task_config_dir = config_dir + "/" + path_name;

  /**
   * 加载任务插件
   *
   * for循环遍历配置中的每个任务
   * pipeline_config_.task_size()返回任务数量
   */
  for (int i = 0; i < pipeline_config_.task_size(); ++i) {
    // 获取第i个任务配置
    auto task = pipeline_config_.task(i);

    // 获取任务类型
    auto task_type = task.type();

    /**
     * 创建任务实例
     *
     * PluginManager::Instance()：
     *   获取插件管理器的单例实例
     *
     * ->CreateInstance<Task>(...)：
     *   动态创建Task类型的实例
     *   模板参数指定要创建的类类型
     *   参数是类的完全限定名称
     *
     * ConfigUtil::GetFullPlanningClassName(task_type)：
     *   获取任务类的完全限定名称
     *   格式："apollo::planning::<TaskType>"
     */
    auto task_ptr = apollo::cyber::plugin_manager::PluginManager::Instance()
                        ->CreateInstance<Task>(
                            ConfigUtil::GetFullPlanningClassName(task_type));

    // 检查任务创建是否成功
    if (nullptr == task_ptr) {
      AERROR << "Create task " << task.name() << " of " << name_ << " failed!";
      return false;  // 创建失败，返回false
    }

    // 初始化任务
    if (task_ptr->Init(task_config_dir, task.name(), injector)) {
      // 初始化成功，将任务添加到任务列表
      task_list_.push_back(task_ptr);
    } else {
      // 初始化失败，输出错误日志
      AERROR << task.name() << " init failed!";
      return false;
    }
  }

  /**
   * 加载后备轨迹任务
   *
   * 后备任务用于在主要任务失败时生成安全停车轨迹
   * 如果没有配置后备任务，使用默认的"FastStopTrajectoryFallback"
   */
  std::string fallback_task_type = "FastStopTrajectoryFallback";
  std::string fallback_task_name = "FAST_STOP_TRAJECTORY_FALLBACK";

  // 检查配置中是否指定了后备任务
  if (pipeline_config_.has_fallback_task()) {
    // 使用配置中指定的后备任务
    fallback_task_type = pipeline_config_.fallback_task().type();
    fallback_task_name = pipeline_config_.fallback_task().name();
  }

  /**
   * 创建并初始化后备任务
   *
   * 与主要任务相同的创建和初始化流程
   */
  fallback_task_ =
      apollo::cyber::plugin_manager::PluginManager::Instance()
          ->CreateInstance<Task>(
              ConfigUtil::GetFullPlanningClassName(fallback_task_type));

  // 检查后备任务创建是否成功
  if (nullptr == fallback_task_) {
    AERROR << "Create fallback task " << fallback_task_name << " of " << name_
           << " failed!";
    return false;
  }

  // 初始化后备任务
  if (!fallback_task_->Init(task_config_dir, fallback_task_name, injector)) {
    AERROR << fallback_task_name << " init failed!";
    return false;
  }

  return true;  // 初始化成功
}

/**
 * @brief 获取阶段名称
 *
 * @return const std::string& 阶段名称的常量引用
 *
 * 功能说明：
 * 返回阶段的名称标识
 *
 * C++语法说明：
 * - const std::string&：
 *   返回常量引用，避免拷贝
 *   const保证返回的字符串不可修改
 *
 * - Name() const：
 *   const成员函数
 *   表示this指针是常量
 *   在函数内不能修改成员变量
 */
const std::string& Stage::Name() const { return name_; }

/**
 * @brief 在参考线上执行任务
 *
 * @param planning_start_point 规划起始点
 * @param frame 规划帧数据
 * @return StageResult 阶段执行结果
 *
 * 功能说明：
 * 在所有参考线上执行阶段配置的任务列表
 * 这是主要的轨迹生成逻辑
 *
 * 算法流程：
 * 1. 检查参考线是否为空
 * 2. 遍历每条参考线
 * 3. 检查参考线是否可行驶
 * 4. 在参考线上执行任务列表
 * 5. 如果出错，执行后备任务
 * 6. 合并路径和速度生成轨迹
 * 7. 设置轨迹到参考线信息
 *
 * C++语法说明：
 * - const common::TrajectoryPoint& planning_start_point：
 *   常量引用，传入轨迹点
 *   包含位置、速度、加速度等信息
 *
 * - Frame* frame：
 *   原始指针，传入规划帧
 *   使用指针因为frame需要被修改
 *
 * - for (auto& reference_line_info : *frame->mutable_reference_line_info())：
 *   范围for循环遍历参考线列表
 *   auto&自动推导类型
 *   mutable_reference_line_info()返回可修改的引用
 *
 * - task->Execute(frame, &reference_line_info)：
 *   在参考线上执行单个任务
 *   &reference_line_info取地址传递给任务
 */
StageResult Stage::ExecuteTaskOnReferenceLine(
    const common::TrajectoryPoint& planning_start_point, Frame* frame) {
  // 创建阶段结果对象
  StageResult stage_result;

  // 检查参考线信息是否为空
  if (frame->reference_line_info().empty()) {
    AERROR << "referenceline is empty in stage" << name_;
    return stage_result.SetStageStatus(StageStatusType::ERROR);
  }

  /**
   * 遍历所有参考线
   *
   * mutable_reference_line_info()：
   *   返回可修改的参考线信息列表引用
   *   允许在遍历过程中修改参考线
   */
  for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
    // 检查参考线是否可行驶
    if (!reference_line_info.IsDrivable()) {
      AERROR << "The generated path is not drivable skip";
      reference_line_info.SetDrivable(false);
      continue;  // 跳过不可行驶的参考线
    }

    // 检查是否是换道路径
    if (reference_line_info.IsChangeLanePath()) {
      AERROR << "The generated refline is change lane path, skip";
      reference_line_info.SetDrivable(false);
      continue;  // 跳过换道路径
    }

    // 初始化返回状态
    common::Status ret = common::Status::OK();

    /**
     * 遍历任务列表执行每个任务
     *
     * for (auto task : task_list_)：
     *   遍历任务列表
     *   task是std::shared_ptr<Task>类型
     */
    for (auto task : task_list_) {
      // 记录任务开始时间
      const double start_timestamp = Clock::NowInSeconds();

      // 在当前参考线上执行任务
      ret = task->Execute(frame, &reference_line_info);

      // 记录任务结束时间
      const double end_timestamp = Clock::NowInSeconds();

      // 计算任务执行时间（毫秒）
      const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;

      // 输出调试信息
      ADEBUG << "after task[" << task->Name()
             << "]: " << reference_line_info.PathSpeedDebugString();
      ADEBUG << task->Name() << " time spend: " << time_diff_ms << " ms.";

      // 输出性能日志
      AINFO << "Planning Perf: task name [" << task->Name() << "], "
            << time_diff_ms << " ms.";

      // 记录调试信息
      RecordDebugInfo(&reference_line_info, task->Name(), time_diff_ms);

      // 检查任务执行是否有错误
      if (!ret.ok()) {
        stage_result.SetTaskStatus(ret);  // 设置任务状态
        AERROR << "Failed to run tasks[" << task->Name()
               << "], Error message: " << ret.error_message();
        break;  // 任务失败，跳出循环
      }
    }

    /**
     * 如果任务执行出错，执行后备任务
     *
     * 后备任务是安全停车轨迹
     * 确保即使主要规划失败，车辆也能安全停止
     */
    if (!ret.ok()) {
      fallback_task_->Execute(frame, &reference_line_info);
    }

    /**
     * 合并路径和速度生成轨迹
     *
     * CombinePathAndSpeedProfile：
     *   将路径和速度轮廓结合生成完整轨迹
     *   参数1：相对时间起始点
     *   参数2：路径起始点的s坐标
     *   参数3：输出轨迹指针
     */
    DiscretizedTrajectory trajectory;
    if (!reference_line_info.CombinePathAndSpeedProfile(
            planning_start_point.relative_time(),
            planning_start_point.path_point().s(), &trajectory)) {
      AERROR << "Fail to aggregate planning trajectory."
             << reference_line_info.IsChangeLanePath();
      reference_line_info.SetDrivable(false);
      continue;  // 失败，跳到下一条参考线
    }

    // 设置轨迹到参考线信息
    reference_line_info.SetTrajectory(trajectory);
    reference_line_info.SetDrivable(true);

    // 返回成功结果
    return stage_result;
  }

  // 所有参考线都失败，返回结果
  return stage_result;
}

/**
 * @brief 在线学习模式下在参考线上执行任务
 *
 * @param planning_start_point 规划起始点
 * @param frame 规划帧数据
 * @return StageResult 阶段执行结果
 *
 * 功能说明：
 * 在线学习模式的执行逻辑
 * 与普通ExecuteTaskOnReferenceLine的区别：
 * 1. 只使用第一条参考线
 * 2. 使用学习模型生成的轨迹调整
 * 3. 不生成后备轨迹
 *
 * C++语法说明：
 * - FIXME(all)：
 *   TODO注释的一种，表示此处有已知问题需要修复
 *   这里的注释说明当前只使用第一条参考线
 */
StageResult Stage::ExecuteTaskOnReferenceLineForOnlineLearning(
    const common::TrajectoryPoint& planning_start_point, Frame* frame) {
  // 在线学习模式：将所有参考线设为不可行驶
  for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
    reference_line_info.SetDrivable(false);
  }

  // 创建阶段结果对象
  StageResult stage_result;

  // FIXME(all): 当前只使用第一条参考线来使用学习模型轨迹
  auto& picked_reference_line_info =
      frame->mutable_reference_line_info()->front();

  // 遍历任务列表执行
  for (auto task : task_list_) {
    const double start_timestamp = Clock::NowInSeconds();

    const auto ret = task->Execute(frame, &picked_reference_line_info);

    const double end_timestamp = Clock::NowInSeconds();
    const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;

    ADEBUG << "task[" << task->Name() << "] time spent: " << time_diff_ms
           << " ms.";

    RecordDebugInfo(&picked_reference_line_info, task->Name(), time_diff_ms);

    if (!ret.ok()) {
      stage_result.SetTaskStatus(ret);
      AERROR << "Failed to run tasks[" << task->Name()
             << "], Error message: " << ret.error_message();
      break;
    }
  }

  /**
   * 获取自车未来轨迹点
   *
   * 从已选择的参考线中获取轨迹
   */
  const std::vector<common::TrajectoryPoint>& adc_future_trajectory_points =
      picked_reference_line_info.trajectory();

  // 创建轨迹对象
  DiscretizedTrajectory trajectory;

  /**
   * 调整从当前位置开始的轨迹
   *
   * AdjustTrajectoryWhichStartsFromCurrentPos：
   *   使用学习模型的结果调整轨迹
   */
  if (picked_reference_line_info.AdjustTrajectoryWhichStartsFromCurrentPos(
          planning_start_point, adc_future_trajectory_points, &trajectory)) {
    picked_reference_line_info.SetTrajectory(trajectory);
    picked_reference_line_info.SetDrivable(true);
    picked_reference_line_info.SetCost(0);
  }

  return stage_result;
}

/**
 * @brief 在开放空间执行任务
 *
 * @param frame 规划帧数据
 * @return StageResult 阶段执行结果
 *
 * 功能说明：
 * 开放空间规划的执行逻辑
 * 用于停车场等非结构化场景
 *
 * 与ExecuteTaskOnReferenceLine的区别：
 * - 不在参考线上执行
 * - 直接在开放空间信息上操作
 * - 根据fallback_flag决定使用哪个轨迹
 */
StageResult Stage::ExecuteTaskOnOpenSpace(Frame* frame) {
  // 初始化返回状态为OK
  auto ret = common::Status::OK();

  // 创建阶段结果对象
  StageResult stage_result;

  // 遍历任务列表执行
  for (auto task : task_list_) {
    const double start_timestamp = Clock::NowInSeconds();

    // 执行任务（无参考线参数）
    ret = task->Execute(frame);

    // 检查任务执行是否有错误
    if (!ret.ok()) {
      stage_result.SetTaskStatus(ret);
      AERROR << "Failed to run tasks[" << task->Name()
             << "], Error message: " << ret.error_message();

      const double end_timestamp = Clock::NowInSeconds();
      const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;
      AINFO << "Planning Perf: task name [" << task->Name() << "], "
            << time_diff_ms << " ms.";

      return stage_result;  // 失败直接返回
    }

    const double end_timestamp = Clock::NowInSeconds();
    const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;
    AINFO << "Planning Perf: task name [" << task->Name() << "], "
          << time_diff_ms << " ms.";
  }

  /**
   * 根据标志选择要发布的轨迹
   *
   * fallback_flag：是否使用后备轨迹
   * stop_flag：是否停止
   *
   * 如果需要使用后备轨迹或停止，使用后备轨迹
   * 否则使用选定的分区轨迹
   */
  if (frame->open_space_info().fallback_flag() ||
      frame->open_space_info().stop_flag()) {
    // 使用后备轨迹
    auto& trajectory = frame->open_space_info().fallback_trajectory().first;
    auto& gear = frame->open_space_info().fallback_trajectory().second;

    /**
     * 创建可发布轨迹
     *
     * PublishableTrajectory：
     *   将轨迹包装成可发布格式
     *   参数1：当前时间戳
     *   参数2：轨迹数据
     */
    PublishableTrajectory publishable_trajectory(Clock::NowInSeconds(),
                                                 trajectory);

    // 创建轨迹和档位的配对
    auto publishable_traj_and_gear =
        std::make_pair(std::move(publishable_trajectory), gear);

    // 移动到开放空间信息中
    *(frame->mutable_open_space_info()->mutable_publishable_trajectory_data()) =
        std::move(publishable_traj_and_gear);
  } else {
    // 使用选定的分区轨迹
    auto& trajectory =
        frame->open_space_info().chosen_partitioned_trajectory().first;
    auto& gear =
        frame->open_space_info().chosen_partitioned_trajectory().second;

    PublishableTrajectory publishable_trajectory(Clock::NowInSeconds(),
                                                 trajectory);
    auto publishable_traj_and_gear =
        std::make_pair(std::move(publishable_trajectory), gear);

    *(frame->mutable_open_space_info()->mutable_publishable_trajectory_data()) =
        std::move(publishable_traj_and_gear);
  }

  return stage_result;
}

/**
 * @brief 完成场景
 *
 * @return StageResult 阶段结果
 *
 * 功能说明：
 * 标记当前场景为已完成
 * 设置下一阶段为空字符串，表示场景结束
 *
 * C++语法说明：
 * - next_stage_ = ""：
 *   设置下一阶段为空
 *   空字符串表示没有下一阶段
 */
StageResult Stage::FinishScenario() {
  next_stage_ = "";
  return StageResult(StageStatusType::FINISHED);
}

/**
 * @brief 记录调试信息
 *
 * @param reference_line_info 参考线信息指针
 * @param name 任务名称
 * @param time_diff_ms 执行时间（毫秒）
 *
 * 功能说明：
 * 将任务的执行信息记录到参考线信息中
 * 用于调试和性能分析
 *
 * C++语法说明：
 * - ReferenceLineInfo*：
 *   参考线信息指针
 *   使用指针因为可能需要修改
 *
 * - FLAGS_enable_record_debug：
 *   全局flag，控制是否记录调试信息
 *   gflags库定义的全局变量
 *
 * - ptr_latency_stats->add_task_stats()：
 *   Protobuf的add方法
 *   在重复字段中添加一个新元素
 *   返回新元素的指针
 *
 * - ptr_stats->set_name(name)：
 *   Protobuf的set方法
 *   设置字段的值
 */
void Stage::RecordDebugInfo(ReferenceLineInfo* reference_line_info,
                            const std::string& name,
                            const double time_diff_ms) {
  // 检查是否启用调试信息记录
  if (!FLAGS_enable_record_debug) {
    ADEBUG << "Skip record debug info";
    return;
  }

  // 检查指针是否为空
  if (reference_line_info == nullptr) {
    AERROR << "Reference line info is null.";
    return;
  }

  // 获取可修改的延迟统计指针
  auto ptr_latency_stats = reference_line_info->mutable_latency_stats();

  // 添加一个新的任务统计
  auto ptr_stats = ptr_latency_stats->add_task_stats();

  // 设置任务名称
  ptr_stats->set_name(name);

  // 设置执行时间
  ptr_stats->set_time_ms(time_diff_ms);
}

/**
 * @namespace命名空间结束
 *
 * C++语法说明：
 * - }  // namespace planning：
 *   结束planning命名空间
 * - }  // namespace apollo：
 *   结束apollo命名空间
 */
}  // namespace planning
}  // namespace apollo