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
 * @file lane_follow_path.cc
 * @brief 车道跟随路径任务实现文件
 *
 * 本文件实现LaneFollowPath类，是Apollo规划模块中生成车道跟随路径的核心任务。
 * 该任务负责在已知参考线的基础上，生成安全、平滑的车道跟随轨迹。
 *
 * 主要功能：
 * 1. 路径边界决定 - 根据车道信息和障碍物确定可行区域
 * 2. 路径优化 - 在边界内优化生成平滑路径
 * 3. 路径评估 - 验证生成路径的有效性
 *
 * 设计特点：
 * - 基于Frenet坐标系进行路径规划
 * - 支持动态扩展车道边界
 * - 多阶段路径生成流程
 *
 * C++语法说明：
 * - std::vector<T>: 动态数组容器
 * - std::shared_ptr<T>: 共享所有权智能指针
 * - std::array<T, N>: 固定大小数组
 * - std::move(): 移动语义，避免拷贝
 * - auto&: 自动类型推导的引用
 * - lambda表达式: 匿名函数
 * - static_cast<T>: 编译时类型转换
 * - std::pair<T1, T2>: 键值对容器
 */
#include "modules/planning/tasks/lane_follow_path/lane_follow_path.h"

#include <algorithm>   /**< C++标准算法库：std::sort, std::find等 */
#include <memory>      /**< 智能指针：std::shared_ptr, std::unique_ptr */
#include <string>      /**< 字符串类型 */
#include <utility>     /**< 工具函数：std::move, std::pair */
#include <vector>      /**< 动态数组容器 */

#include "modules/common/configs/vehicle_config_helper.h" /**< 车辆配置助手 */
#include "modules/planning/planning_base/common/util/print_debug_info.h" /**< 调试信息打印 */
#include "modules/planning/planning_interface_base/task_base/common/path_generation.h" /**< 路径生成基类 */
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_assessment_decider_util.h" /**< 路径评估工具 */
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_bounds_decider_util.h" /**< 路径边界决定工具 */
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_optimizer_util.h" /**< 路径优化工具 */

namespace apollo {
/**
 * apollo:: - Apollo最外层命名空间
 */
namespace planning {

/**
 * using声明 - 将其他命名空间的类型引入当前作用域，避免重复书写命名空间前缀
 */
using apollo::common::Status;                     /**< Apollo通用状态类型 */
using apollo::common::VehicleConfigHelper;         /**< 车辆配置助手 */

/**
 * @brief 车道跟随路径任务初始化
 *
 * 初始化车道跟随路径任务，包括：
 * 1. 调用基类Task的初始化方法
 * 2. 加载任务专用配置LaneFollowPathConfig
 *
 * @param config_dir 配置文件目录路径
 * @param name 任务名称
 * @param injector 依赖注入器指针（共享指针）
 * @return bool 初始化是否成功
 *
 * 语法说明：
 * - const std::string&: 常量引用参数，避免拷贝
 * - const std::shared_ptr<DependencyInjector>&: 共享指针的常量引用
 * - Task::LoadConfig<T>: 模板函数，加载指定类型的配置
 * - &config_: 取地址操作符，获取成员变量的指针
 */
bool LaneFollowPath::Init(const std::string& config_dir,
                          const std::string& name,
                          const std::shared_ptr<DependencyInjector>& injector) {
  /**
   * 调用基类Task的Init方法进行基础初始化
   * if (!Task::Init(...)) - 检查初始化是否失败
   */
  if (!Task::Init(config_dir, name, injector)) {
    return false;  /**< 初始化失败返回false */
  }
  /**
   * Load the config this task.
   * Task::LoadConfig<LaneFollowPathConfig>: 模板方法，加载LaneFollowPathConfig类型配置
   * &config_: 成员变量指针，用于存储加载的配置
   * 返回true表示加载成功
   */
  return Task::LoadConfig<LaneFollowPathConfig>(&config_);
}

/**
 * @brief 车道跟随路径处理主函数
 *
 * 这是LaneFollowPath任务的核心处理函数，执行以下步骤：
 * 1. 获取起始点SL状态
 * 2. 决定路径边界（根据车道和障碍物）
 * 3. 优化生成路径
 * 4. 评估路径有效性
 *
 * @param frame 规划帧指针，包含当前帧的所有信息
 * @param reference_line_info 参考线信息指针
 * @return Status 处理状态（OK表示成功）
 *
 * 语法说明：
 * - Frame*: 普通指针，Frame类包含规划帧的所有数据
 * - ReferenceLineInfo*: 参考线信息指针
 * - reference_line_info->path_data(): 箭头操作符，通过指针访问成员
 * - Status::OK(): 静态方法，返回成功状态
 */
apollo::common::Status LaneFollowPath::Process(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  /**
   * 检查路径是否已存在或可复用
   * if (!reference_line_info->path_data().Empty() || reference_line_info->path_reusable())
   *   - path_data().Empty(): 检查路径数据是否为空
   *   - path_reusable(): 检查路径是否可复用
   *   - !表示取反，两个条件任一为真则跳过处理
   */
  if (!reference_line_info->path_data().Empty() ||
      reference_line_info->path_reusable()) {
    ADEBUG << "Skip this time path empty:"
           << reference_line_info->path_data().Empty()
           << "path reusable: " << reference_line_info->path_reusable();
    return Status::OK();  
  } 

  /**
   * 声明候选路径边界和路径数据容器
   * std::vector<PathBoundary>: 路径边界向量
   * std::vector<PathData>: 路径数据向量
   * 使用std::vector而非数组，因为路径数量在编译时不确定
   */
  std::vector<PathBoundary> candidate_path_boundaries;
  std::vector<PathData> candidate_path_data;

  /**
   * 获取起始点的SL状态
   * init_sl_state_是类成员变量，存储自车在SL坐标系下的状态
   */
  GetStartPointSLState();

  /**
   * 步骤1：决定路径边界
   * DecidePathBounds(): 根据车道信息和障碍物确定可行路径边界
   * &candidate_path_boundaries: 取地址，传入指针用于输出
   */
  if (!DecidePathBounds(&candidate_path_boundaries)) {
    AERROR << "Decide path bound failed";
    return Status::OK();  /**< 边界决定失败，返回OK继续下一帧 */
  }

  /**
   * 步骤2：优化生成路径
   * OptimizePath(): 在给定边界内优化生成平滑路径
   */
  if (!OptimizePath(candidate_path_boundaries, &candidate_path_data)) {
    AERROR << "Optmize path failed";
    return Status::OK();
  }

  /**
   * 步骤3：评估路径有效性
   * AssessPath(): 验证生成路径是否满足安全和平滑要求
   * reference_line_info->mutable_path_data(): 获取可写的路径数据指针
   */
  if (!AssessPath(&candidate_path_data,
                  reference_line_info->mutable_path_data())) {
    AERROR << "Path assessment failed";
  }

  return Status::OK();  /**< 处理完成，返回成功状态 */
}

/**
 * @brief 决定路径边界
 *
 * 根据自车道信息和静态障碍物确定路径的可行区域边界。
 * 路径边界定义了自车可以安全行驶的横向范围。
 *
 * 边界决定流程：
 * 1. 初始化为无限大的区域
 * 2. 根据自车道收束边界
 * 3. 根据静态障碍物进一步限制边界
 * 4. 添加额外边界点避免零长度路径
 *
 * @param boundary 输出：路径边界向量指针
 * @return bool 是否成功决定边界
 *
 * 语法说明：
 * - std::vector<PathBoundary>*: 路径边界向量的指针
 * - boundary->emplace_back(): 通过指针调用emplace_back，在向量末尾构造新元素
 * - auto& path_bound = boundary->back(): auto自动推导类型，&引用避免拷贝
 * - std::string: 字符串类型
 * - config_.is_extend_lane_bounds_to_include_adc(): 获取配置值
 */
bool LaneFollowPath::DecidePathBounds(std::vector<PathBoundary>* boundary) {
  /**
   * 在向量末尾添加一个新的空PathBoundary
   * emplace_back() vs push_back():
   * - push_back() 接收已构造的对象，可能产生拷贝
   * - emplace_back() 在容器内部直接构造，效率更高
   */
  boundary->emplace_back();
  /**
   * 获取刚添加的边界引用
   * auto&: 自动类型推导为PathBoundary&引用
   * boundary->back(): 返回向量最后一个元素的引用
   */
  auto& path_bound = boundary->back();

  /**
   * 初始化路径边界相关变量
   * blocking_obstacle_id: 阻塞障碍物ID，标识导致路径终止的障碍物
   * lane_type: 车道类型
   * path_narrowest_width: 路径最窄宽度
   */
  std::string blocking_obstacle_id = "";   /**< 阻塞障碍物ID初始化为空 */
  std::string lane_type = "";              /**< 车道类型初始化为空 */
  double path_narrowest_width = 0;        /**< 最窄宽度初始化为0 */

  /**
   * 1. Initialize the path boundaries to be an indefinitely large area.
   * 步骤1：初始化路径边界为无限大区域
   *
   * PathBoundsDeciderUtil::InitPathBoundary():
   *   - 静态工具函数，初始化路径边界
   *   - *reference_line_info_: 解引用获取参考线信息引用
   *   - &path_bound: 传入边界引用用于填充
   *   - init_sl_state_: 类成员，存储起始SL状态
   */
  if (!PathBoundsDeciderUtil::InitPathBoundary(*reference_line_info_,
                                               &path_bound, init_sl_state_)) {
    const std::string msg = "Failed to initialize path boundaries.";
    AERROR << msg;
    return false;  /**< 初始化失败返回false */
  }

  /**
   * 配置检查：是否需要扩展车道边界以包含自车
   * config_.is_extend_lane_bounds_to_include_adc():
   *   - 从配置中获取是否扩展车道边界包含自车
   * injector_->planning_context()->planning_status().path_decider().is_in_path_lane_borrow_scenario():
   *   - 获取是否处于车道借用场景
   * !xxx: 取反操作
   * &&: 逻辑与运算符
   */
  std::string borrow_lane_type;
  bool is_include_adc = config_.is_extend_lane_bounds_to_include_adc() &&
                        !injector_->planning_context()
                             ->planning_status()
                             .path_decider()
                             .is_in_path_lane_borrow_scenario();

  /**
   * 2. Decide a rough boundary based on lane info and ADC's position
   * 步骤2：根据车道信息和自车位置决定粗略边界
   *
   * PathBoundsDeciderUtil::GetBoundaryFromSelfLane():
   *   - 根据自车道信息获取路径边界
   *   - 输入：参考线信息、初始SL状态
   *   - 输出：填充后的path_bound
   */
  if (!PathBoundsDeciderUtil::GetBoundaryFromSelfLane(
          *reference_line_info_, init_sl_state_, &path_bound)) {
    AERROR << "Failed to decide a rough boundary based on self lane.";
    return false;
  }

  /**
   * 如果配置要求包含自车，则扩展边界
   * PathBoundsDeciderUtil::ExtendBoundaryByADC():
   *   - 扩展边界以确保自车位置在边界内
   *   - config_.extend_buffer(): 获取扩展缓冲区大小
   */
  if (is_include_adc) {
    PathBoundsDeciderUtil::ExtendBoundaryByADC(
        *reference_line_info_, init_sl_state_, config_.extend_buffer(),
        &path_bound);
  }

  /**
   * 调试信息打印准备
   * PrintCurves: 调试曲线打印工具类
   * 用于将障碍物SL边界打印到日志
   */
  PrintCurves print_curve;
  /**
   * 获取路径决策中的所有障碍物
   * reference_line_info_->path_decision()->obstacles():
   *   - path_decision(): 获取路径决策引用
   *   - obstacles(): 获取障碍物集合
   *   - Items(): 返回障碍物指针向量
   */
  auto indexed_obstacles = reference_line_info_->path_decision()->obstacles();

  /**
   * 遍历所有障碍物，提取SL边界点
   * for (const auto* obs : indexed_obstacles.Items())
   *   - range-based for循环（C++11）
   *   - const auto*: 自动推导障碍物指针类型
   *   - obs: 当前遍历的障碍物指针
   */
  for (const auto* obs : indexed_obstacles.Items()) {
    /**
     * 获取障碍物的SL边界
     * obs->PerceptionSLBoundary():
     *   - 获取障碍物在SL坐标系下的感知边界
     *   - 返回SLBoundary类型
     */
    const auto& sl_bound = obs->PerceptionSLBoundary();

    /**
     * 遍历SL边界的边界点
     * for (int i = 0; i < sl_bound.boundary_point_size(); i++)
     *   - boundary_point_size(): 获取边界点数量
     *   - 注意：这里用int而不是size_t，可能是因为protobuf生成的接口
     */
    for (int i = 0; i < sl_bound.boundary_point_size(); i++) {
      /**
       * 构建障碍物边界点名称
       * obs->Id() + "_obs_sl_boundary":
       *   - Id(): 获取障碍物唯一ID
       *   - +: 字符串拼接操作符
       */
      std::string name = obs->Id() + "_obs_sl_boundary";

      /**
       * 添加边界点到调试曲线
       * sl_bound.boundary_point(i).s(): 获取第i个边界点的s坐标
       * sl_bound.boundary_point(i).l(): 获取第i个边界点的l坐标
       */
      print_curve.AddPoint(name, sl_bound.boundary_point(i).s(),
                           sl_bound.boundary_point(i).l());
    }
  }

  /**
   * 打印所有障碍物边界曲线到日志
   */
  print_curve.PrintToLog();

  /**
   * 设置路径边界标签
   * path_bound.set_label():
   *   - 设置路径边界的标识标签
   * absl::StrCat("regular/", "self"):
   *   - Abseil字符串拼接函数，比+操作符更高效
   *   - 拼接结果："regular/self"
   */
  path_bound.set_label(absl::StrCat("regular/", "self"));

  /**
   * 3. Fine-tune the boundary based on static obstacles
   * 步骤3：根据静态障碍物精细调整边界
   *
   * 先保存当前边界作为临时备份
   * PathBound temp_path_bound = path_bound:
   *   - 拷贝构造临时边界，用于后续添加额外点
   */
  PathBound temp_path_bound = path_bound;

  /**
   * 清空障碍物SL多边形容器
   * obs_sl_polygons_.clear():
   *   - clear()清空容器但保留容量
   *   - 这是类成员变量，用于存储障碍物SL多边形
   */
  obs_sl_polygons_.clear();

  /**
   * 获取SL多边形
   * PathBoundsDeciderUtil::GetSLPolygons():
   *   - 根据参考线和障碍物生成SL多边形
   *   - &obs_sl_polygons_: 传入容器引用用于填充
   */
  PathBoundsDeciderUtil::GetSLPolygons(*reference_line_info_, &obs_sl_polygons_,
                                       init_sl_state_);

  /**
   * 根据静态障碍物调整路径边界
   * PathBoundsDeciderUtil::GetBoundaryFromStaticObstacles():
   *   - 分析静态障碍物对路径边界的影响
   *   - &blocking_obstacle_id: 输出阻塞障碍物ID
   *   - &path_narrowest_width: 输出最窄路径宽度
   */
  if (!PathBoundsDeciderUtil::GetBoundaryFromStaticObstacles(
          *reference_line_info_, &obs_sl_polygons_, init_sl_state_, &path_bound,
          &blocking_obstacle_id, &path_narrowest_width)) {
    const std::string msg =
        "Failed to decide fine tune the boundaries after "
        "taking into consideration all static obstacles.";
    AERROR << msg;
    return false;
  }

  /**
   * 4. Append some extra path bound points to avoid zero-length path data.
   * 步骤4：添加额外边界点避免零长度路径
   *
   * 当存在阻塞障碍物时，在路径末尾添加临时边界的点
   * 这确保路径不会因为遇到障碍物而变成零长度
   *
   * counter: 计数器，限制添加点的数量
   * FLAGS_num_extra_tail_bound_point: 配置参数，最大额外点数
   */
  int counter = 0;  /**< 初始化计数器 */
  while (!blocking_obstacle_id.empty() &&         /**< 存在阻塞障碍物 */
         path_bound.size() < temp_path_bound.size() && /**< 当前边界小于临时边界 */
         counter < FLAGS_num_extra_tail_bound_point) { /**< 未超过最大点数 */
    /**
     * 将临时边界中的下一个点添加到当前边界
     * path_bound.push_back(temp_path_bound[path_bound.size()]):
     *   - push_back(): 在向量末尾添加元素
     *   - temp_path_bound[path_bound.size()]: 使用当前边界大小作为索引获取临时边界点
     */
    path_bound.push_back(temp_path_bound[path_bound.size()]);
    counter++;  /**< 计数器加1 */
  }

  /**
   * 更新车道跟随状态
   * injector_->planning_context()->mutable_planning_status()->mutable_lane_follow():
   *   - 获取车道跟随状态的可写指针
   *   - mutable_前缀表示可写
   */
  auto* lane_follow_status = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_lane_follow();

  /**
   * 如果存在阻塞障碍物，更新阻塞状态
   */
  if (!blocking_obstacle_id.empty()) {
    /**
     * 获取当前时间
     * ::apollo::cyber::Clock::NowInSeconds():
     *   - Cyber RT时钟，获取当前时间（秒）
     *   - ::前缀表示全局命名空间
     */
    double current_time = ::apollo::cyber::Clock::NowInSeconds();

    /**
     * 设置阻塞障碍物ID
     */
    lane_follow_status->set_block_obstacle_id(blocking_obstacle_id);

    /**
     * 更新阻塞持续时间
     * if (lane_follow_status->lane_follow_block()):
     *   - 检查之前是否处于阻塞状态
     */
    if (lane_follow_status->lane_follow_block()) {
      /**
       * 之前处于阻塞状态，累加持续时间
       * lane_follow_status->block_duration() + current_time - last_block_timestamp:
       *   - 之前持续时间 + 当前时间 - 上次记录时间
       */
      lane_follow_status->set_block_duration(
          lane_follow_status->block_duration() + current_time -
          lane_follow_status->last_block_timestamp());
    } else {
      /**
       * 之前未处于阻塞状态，初始化持续时间
       */
      lane_follow_status->set_block_duration(0);
      lane_follow_status->set_lane_follow_block(true);
    }

    /**
     * 更新最后阻塞时间戳
     */
    lane_follow_status->set_last_block_timestamp(current_time);
  } else {
    /**
     * 不存在阻塞障碍物，清除阻塞状态
     */
    if (lane_follow_status->lane_follow_block()) {
      lane_follow_status->set_block_duration(0);
      lane_follow_status->set_lane_follow_block(false);
      lane_follow_status->set_last_block_timestamp(0);
    }
  }

  /**
   * 调试日志输出
   * ADEBUG: 调试级别日志宏
   */
  ADEBUG << "Completed generating path boundaries.";

  /**
   * 检查自车初始位置是否在路径边界内
   * init_sl_state_.second[0]: 获取初始横向位置l
   * path_bound[0].l_upper.l: 第一个边界点的上界
   * path_bound[0].l_lower.l: 第一个边界点的下界
   *
   * 如果初始位置不在边界内，可能是车道借用或超出道路
   */
  if (init_sl_state_.second[0] > path_bound[0].l_upper.l ||
      init_sl_state_.second[0] < path_bound[0].l_lower.l) {
    AINFO << "not in self lane maybe lane borrow or out of road. init l : "
          << init_sl_state_.second[0] << ", path_bound l: [ "
          << path_bound[0].l_lower.l << "," << path_bound[0].l_upper.l << " ]";
    return false;  /**< 初始位置不在边界内，返回失败 */
  }

  /**
   * 以下是旧代码注释掉的调试代码，用于生成规则路径边界对
   * // std::vector<std::pair<double, double>> regular_path_bound_pair;
   * // for (size_t i = 0; i < path_bound.size(); ++i) {
   * //   regular_path_bound_pair.emplace_back(std::get<1>(path_bound[i]),
   * //                                        std::get<2>(path_bound[i]));
   * // }
   */

  /**
   * 设置路径边界的阻塞障碍物ID
   * path_bound.set_blocking_obstacle_id():
   *   - 将之前识别的阻塞障碍物ID设置到边界中
   */
  path_bound.set_blocking_obstacle_id(blocking_obstacle_id);

  /**
   * 记录调试信息
   * RecordDebugInfo():
   *   - 将路径边界信息记录到调试输出
   */
  RecordDebugInfo(path_bound, path_bound.label(), reference_line_info_);
  return true;  /**< 边界决定成功 */
}

/**
 * @brief 优化生成路径
 *
 * 根据给定的路径边界，使用优化算法生成平滑的车道跟随路径。
 * 使用Piecewise Jerk方法进行路径优化。
 *
 * @param path_boundaries 输入：路径边界向量
 * @param candidate_path_data 输出：候选路径数据向量
 * @return bool 是否成功优化出路径
 *
 * 语法说明：
 * - const std::vector<PathBoundary>&: 常量引用输入参数
 * - std::vector<PathData>*: 指针输出参数
 * - std::array<double, 3>: 固定大小数组，存储end_state[位置, 速度, 加速度]
 * - size_t: 无符号整数类型，用于表示大小和索引
 * - static_cast<double>: 显式类型转换，将int转为double
 */
bool LaneFollowPath::OptimizePath(
    const std::vector<PathBoundary>& path_boundaries,
    std::vector<PathData>* candidate_path_data) {
  /**
   * 获取路径优化器配置
   * config_.path_optimizer_config():
   *   - 从类成员config获取路径优化配置
   *   - 返回PathOptimizerConfig类型
   */
  const auto& config = config_.path_optimizer_config();

  /**
   * 获取参考线引用
   * reference_line_info_->reference_line():
   *   - 获取参考线信息中的参考线
   *   - 返回ReferenceLine引用
   */
  const ReferenceLine& reference_line = reference_line_info_->reference_line();

  /**
   * 定义末端状态数组
   * std::array<double, 3>: 固定大小数组，包含3个double元素
   * end_state = {0.0, 0.0, 0.0}:
   *   - [0]: 末端横向位置l = 0
   *   - [1]: 末端横向速度dl = 0
   *   - [2]: 末端横向加速度ddl = 0
   * 初始化为在车道中心、静止状态
   */
  std::array<double, 3> end_state = {0.0, 0.0, 0.0};

  /**
   * 遍历所有路径边界进行处理
   * for (const auto& path_boundary : path_boundaries)
   *   - range-based for循环
   *   - const auto&: 常量引用，避免拷贝
   */
  for (const auto& path_boundary : path_boundaries) {
    /**
     * 获取当前路径边界的大小
     * path_boundary.boundary().size():
     *   - boundary(): 获取边界点向量
     *   - size(): 返回向量大小
     */
    size_t path_boundary_size = path_boundary.boundary().size();

    /**
     * 检查边界大小是否有效
     * path_boundary_size <= 1U:
     *   - 如果边界点数量<=1，说明无法生成有效路径
     *   - 1U表示无符号整数1
     */
    if (path_boundary_size <= 1U) {
      AERROR << "Get invalid path boundary with size: " << path_boundary_size;
      return false;  /**< 边界无效，返回失败 */
    }

    /**
     * 声明优化变量
     * opt_l: 优化后的横向位置
     * opt_dl: 优化后的横向速度
     * opt_ddl: 优化后的横向加速度
     */
    std::vector<double> opt_l, opt_dl, opt_ddl;

    /**
     * 加速度边界
     * ddl_bounds: 每一步的加速度上/下界
     */
    std::vector<std::pair<double, double>> ddl_bounds;

    /**
     * 计算加速度边界
     * PathOptimizerUtil::CalculateAccBound():
     *   - 根据路径边界和曲率计算加速度约束
     *   - &ddl_bounds: 传入指针用于输出
     */
    PathOptimizerUtil::CalculateAccBound(path_boundary, reference_line,
                                         &ddl_bounds);

    /**
     * 调试曲线准备
     */
    PrintCurves print_debug;

    /**
     * 遍历边界点，收集曲率信息
     * for (size_t i = 0; i < path_boundary_size; ++i)
     *   - ++i vs i++: 前置递增 vs 后置递增
     *   - 在循环中效果相同，但++i效率略高
     */
    for (size_t i = 0; i < path_boundary_size; ++i) {
      /**
       * 计算当前点的s坐标
       * static_cast<double>(i): 将size_t转为double
       * path_boundary.delta_s(): 相邻边界点的s间隔
       * path_boundary.start_s(): 起始s坐标
       */
      double s = static_cast<double>(i) * path_boundary.delta_s() +
                 path_boundary.start_s();

      /**
       * 获取参考线上最近点的曲率
       * reference_line.GetNearestReferencePoint(s):
       *   - 获取参考线上s处最近的参考点
       *   - .kappa(): 获取该点的曲率
       */
      double kappa = reference_line.GetNearestReferencePoint(s).kappa();

      /**
       * 添加曲率点到调试曲线
       */
      print_debug.AddPoint("ref_kappa", s, kappa);
    }

    /**
     * 打印曲率调试信息
     */
    print_debug.PrintToLog();

    /**
     * 估算加加速度（Jerk）边界
     * PathOptimizerUtil::EstimateJerkBoundary():
     *   - 根据自车横向速度估算允许的最大加加速度
     * std::fmax(init_sl_state_.first[1], 1e-12):
     *   - 取自车横向速度和最小值的较大值
     *   - 1e-12避免除零或过小值
     */
    const double jerk_bound = PathOptimizerUtil::EstimateJerkBoundary(
        std::fmax(init_sl_state_.first[1], 1e-12));

    /**
     * 初始化参考线横向位置和权重
     * ref_l: 参考线l值向量
     * weight_ref_l: 参考线权重向量
     * std::vector<double>(size, value): 构造指定大小的向量
     */
    std::vector<double> ref_l(path_boundary_size, 0);      /**< 初始为0，即沿参考线 */
    std::vector<double> weight_ref_l(path_boundary_size, 0); /**< 初始权重为0 */

    /**
     * 根据边界更新参考线
     * PathOptimizerUtil::UpdatePathRefWithBound():
     *   - 根据路径边界调整参考线和权重
     *   - config.path_reference_l_weight(): 获取参考线权重参数
     */
    PathOptimizerUtil::UpdatePathRefWithBound(
        path_boundary, config.path_reference_l_weight(), &ref_l, &weight_ref_l);

    /**
     * 执行路径优化
     * PathOptimizerUtil::OptimizePath():
     *   - 使用Piecewise Jerk方法优化路径
     *   - 输入：初始状态、末端状态、参考线、边界、约束、配置
     *   - 输出：优化后的opt_l, opt_dl, opt_ddl
     */
    bool res_opt = PathOptimizerUtil::OptimizePath(
        init_sl_state_, end_state, ref_l, weight_ref_l, path_boundary,
        ddl_bounds, jerk_bound, config, &opt_l, &opt_dl, &opt_ddl);

    /**
     * 如果优化成功
     */
    if (res_opt) {
      /**
       * 转换为分段加加速度路径
       * PathOptimizerUtil::ToPiecewiseJerkPath():
       *   - 将优化结果转换为Frenet坐标系下的分段加加速度路径
       *   - 输入：优化后的l, dl, ddl，边界间隔和起始s
       */
      auto frenet_frame_path = PathOptimizerUtil::ToPiecewiseJerkPath(
          opt_l, opt_dl, opt_ddl, path_boundary.delta_s(),
          path_boundary.start_s());

      /**
       * 构造路径数据
       */
      PathData path_data;

      /**
       * 设置路径数据的参考线指针
       * path_data.SetReferenceLine():
       *   - 关联路径数据与参考线
       *   - 使路径数据可以查询参考线信息
       */
      path_data.SetReferenceLine(&reference_line);

      /**
       * 设置Frenet路径
       * path_data.SetFrenetPath(std::move(frenet_frame_path)):
       *   - 使用移动语义，避免拷贝
       *   - std::move将左值转为右值引用
       *   - FrenetPath通常较大，拷贝开销高
       */
      path_data.SetFrenetPath(std::move(frenet_frame_path));

      /**
       * 检查是否使用前轴中心进行路径规划
       * FLAGS_use_front_axe_center_in_path_planning:
       *   - 配置标志，决定是否使用前轴中心
       */
      if (FLAGS_use_front_axe_center_in_path_planning) {
        /**
         * 将路径点从参考点（前轴中心）转换为后轴中心
         * PathOptimizerUtil::ConvertPathPointRefFromFrontAxeToRearAxe():
         *   - 由于规划和控制使用后轴中心，需要转换
         * DiscretizedPath():
         *   - 构造离散路径
         */
        auto discretized_path = DiscretizedPath(
            PathOptimizerUtil::ConvertPathPointRefFromFrontAxeToRearAxe(
                path_data));
        path_data.SetDiscretizedPath(discretized_path);
      }

      /**
       * 设置路径标签
       * path_data.set_path_label():
       *   - 设置路径标识，用于调试和日志
       */
      path_data.set_path_label(path_boundary.label());

      /**
       * 设置阻塞障碍物ID
       * path_data.set_blocking_obstacle_id():
       *   - 记录导致路径终止的障碍物
       */
      path_data.set_blocking_obstacle_id(path_boundary.blocking_obstacle_id());

      /**
       * 将路径数据添加到候选列表
       * candidate_path_data->push_back(std::move(path_data)):
       *   - 使用移动语义，避免path_data拷贝
       */
      candidate_path_data->push_back(std::move(path_data));

      /**
       * 调试：打印路径曲率
       */
      PrintCurves print_path_kappa;
      /**
       * 遍历生成路径的所有路径点
       * candidate_path_data->back().discretized_path():
       *   - back(): 获取向量最后一个元素（刚添加的路径）
       *   - discretized_path(): 获取离散路径点
       */
      for (const auto& p : candidate_path_data->back().discretized_path()) {
        /**
         * 添加曲率点到调试曲线
         * path_boundary.label() + "_path_kappa":
         *   - 路径标签作为曲线名称前缀
         * p.s() + init_sl_state_.first[0]:
         *   - 加上起始s坐标得到全局s
         */
        print_path_kappa.AddPoint(path_boundary.label() + "_path_kappa",
                                  p.s() + init_sl_state_.first[0], p.kappa());
      }
      print_path_kappa.PrintToLog();
    }
  }

  /**
   * 检查是否有有效路径生成
   * candidate_path_data->empty():
   *   - empty()比size()==0更清晰，效率相同
   */
  if (candidate_path_data->empty()) {
    return false;  /**< 没有生成有效路径 */
  }
  return true;  /**< 优化成功 */
}

/**
 * @brief 评估路径有效性
 *
 * 评估生成的候选路径是否满足安全和平滑要求，
 * 并将最终路径设置到参考线信息中。
 *
 * @param candidate_path_data 输入/输出：候选路径数据向量
 * @param final_path 输出：最终路径数据指针
 * @return bool 路径是否有效
 *
 * 语法说明：
 * - PathData&: 左值引用，直接修改传入的路径数据
 * - PathData*: 指针参数，用于输出
 * - std::move(): 移动语义
 */
bool LaneFollowPath::AssessPath(std::vector<PathData>* candidate_path_data,
                                PathData* final_path) {
  /**
   * 获取最后一个候选路径的引用
   * candidate_path_data->back():
   *   - back(): 返回向量最后一个元素的引用
   * PathData&: 左值引用
   */
  PathData& curr_path_data = candidate_path_data->back();

  /**
   * 记录路径调试信息
   * RecordDebugInfo():
   *   - 将路径信息记录到调试输出
   */
  RecordDebugInfo(curr_path_data, curr_path_data.path_label(),
                  reference_line_info_);

  /**
   * 检查路径是否有效
   * PathAssessmentDeciderUtil::IsValidRegularPath():
   *   - 检查路径是否满足规则要求
   *   - 包括路径点数量、连续性、边界满足等
   */
  if (!PathAssessmentDeciderUtil::IsValidRegularPath(*reference_line_info_,
                                                     curr_path_data)) {
    AINFO << "Lane follow path is invalid";
    return false;  /**< 路径无效 */
  }

  /**
   * 初始化路径点决策容器
   * std::vector<PathPointDecision>:
   *   - 存储路径上每个点的决策信息
   *   - PathPointDecision: 路径点决策类型
   */
  std::vector<PathPointDecision> path_decision;

  /**
   * 初始化路径点决策
   * PathAssessmentDeciderUtil::InitPathPointDecision():
   *   - 为路径上的每个点设置默认决策
   * PathData::PathPointType::IN_LANE:
   *   - 路径点类型：在车道内
   */
  PathAssessmentDeciderUtil::InitPathPointDecision(
      curr_path_data, PathData::PathPointType::IN_LANE, &path_decision);

  /**
   * 设置路径点的决策指南
   * curr_path_data.SetPathPointDecisionGuide(std::move(path_decision)):
   *   - 使用移动语义，避免拷贝
   */
  curr_path_data.SetPathPointDecisionGuide(std::move(path_decision));

  /**
   * 再次检查路径是否为空
   * curr_path_data.Empty():
   *   - 可能在某些裁剪操作后变为空
   */
  if (curr_path_data.Empty()) {
    AINFO << "Lane follow path is empty after trimed";
    return false;
  }

  /**
   * 将评估通过的路径设置为最终路径
   * *final_path = curr_path_data:
   *   - 解引用指针，然后拷贝赋值
   */
  *final_path = curr_path_data;

  /**
   * 记录日志信息
   */
  AINFO << final_path->path_label() << final_path->blocking_obstacle_id();

  /**
   * 将最终路径添加到参考线信息的候选路径列表
   * reference_line_info_->MutableCandidatePathData()->push_back(*final_path):
   *   - MutableCandidatePathData(): 获取可写的候选路径数据指针
   *   - push_back(): 添加到列表
   */
  reference_line_info_->MutableCandidatePathData()->push_back(*final_path);

  /**
   * 设置阻塞障碍物
   * reference_line_info_->SetBlockingObstacle():
   *   - 将路径的阻塞障碍物设置到参考线信息中
   */
  reference_line_info_->SetBlockingObstacle(
      curr_path_data.blocking_obstacle_id());

  /**
   * 移动障碍物SL多边形到参考线信息
   * *(reference_line_info_->mutable_obs_sl_polygons()) = std::move(obs_sl_polygons_):
   *   - mutable_obs_sl_polygons(): 获取可写的障碍物SL多边形指针
   *   - std::move(): 移动语义，将当前对象的内存转移给目标
   *   - 移动后obs_sl_polygons_变为空，不能再使用
   */
  *(reference_line_info_->mutable_obs_sl_polygons()) = std::move(obs_sl_polygons_);
  return true;  /**< 路径评估通过 */
}

}  // namespace planning
}  // namespace apollo
