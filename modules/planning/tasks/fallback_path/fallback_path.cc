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
 * @file fallback_path.cc
 * @brief 回退路径规划任务实现文件
 *
 * 本文件实现了FallbackPath类，作为路径规划的**备选/回退方案**。
 *
 * 回退路径的作用：
 * - 当正常路径规划失败时，使用回退路径保证车辆能继续行驶
 * - 回退路径生成一条沿参考线的保守路径
 * - 重点是**安全性和可达性**，而非最优性
 *
 * 与正常路径规划的区别：
 * - 只在当前没有有效路径时执行
 * - 不考虑借道、换道等复杂场景
 * - 简单沿当前车道中心线行驶
 *
 * C++语法说明：
 * - std::shared_ptr<T>: 智能指针，引用计数
 * - absl::StrCat: Abseil库的字符串拼接函数
 */

#include "modules/planning/tasks/fallback_path/fallback_path.h"

/**
 * @brief 标准库头文件
 * <memory>: 提供std::shared_ptr智能指针
 * <string>: 提供std::string字符串
 * <utility>: 提供std::pair, std::move等工具
 * <vector>: 提供std::vector动态数组
 */
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
/**
 * @brief 车辆配置辅助类
 * VehicleConfigHelper::GetConfig(): 获取车辆配置参数
 */
#include "modules/planning/planning_interface_base/task_base/common/path_generation.h"
/**
 * @brief 路径生成基类
 * Task: 所有规划任务的基类
 */
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_assessment_decider_util.h"
/**
 * @brief 路径评估工具
 * 评估路径是否有效
 */
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_bounds_decider_util.h"
/**
 * @brief 路径边界决策工具
 * 决定路径的左右边界
 */
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_optimizer_util.h"
/**
 * @brief 路径优化工具
 * 使用QP算法优化路径
 */

namespace apollo {
/**
 * @brief Apollo顶层命名空间
 */
namespace planning {

/**
 * @brief 类型别名定义
 * using: 类型别名，简化代码书写
 */
using apollo::common::Status;
using apollo::common::VehicleConfigHelper;

/**
 * @brief 初始化回退路径任务
 *
 * @param config_dir 配置文件目录
 * @param name 任务名称
 * @param injector 依赖注入器
 * @return bool 初始化是否成功
 *
 * 初始化流程：
 * 1. 调用基类Task::Init进行基础初始化
 * 2. 加载FallbackPathConfig配置
 */
bool FallbackPath::Init(const std::string& config_dir, const std::string& name,
                        const std::shared_ptr<DependencyInjector>& injector) {
  /**
   * @brief 调用基类初始化
   * Task::Init: 基类Task的初始化函数
   * 如果失败则返回false
   */
  if (!Task::Init(config_dir, name, injector)) {
    return false;
  }

  /**
   * @brief 加载任务特定配置
   * Task::LoadConfig<T>: 模板函数，从配置文件加载配置
   * &config_: 将配置存储到成员变量config_
   */
  return Task::LoadConfig<FallbackPathConfig>(&config_);
}

/**
 * @brief 执行回退路径规划
 *
 * @param frame 当前规划帧
 * @param reference_line_info 参考线信息
 * @return apollo::common::Status 执行状态
 *
 * 执行条件：
 * - 当前没有有效路径（path_data为空）
 * - 不是换道路径
 *
 * 主要流程：
 * 1. 检查是否需要执行回退路径
 * 2. 决定路径边界
 * 3. 优化路径
 * 4. 评估路径
 *
 * C++语法说明：
 * - Frame*: 原始指针，需要保证frame生命周期有效
 * - ReferenceLineInfo*: 参考线信息
 * - Status::OK(): 返回成功状态，回退路径即使失败也返回OK
 */
apollo::common::Status FallbackPath::Process(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  /**
   * @brief 检查是否需要执行回退路径
   *
   * 条件1：path_data().Empty() - 当前没有有效路径
   * 条件2：!IsChangeLanePath() - 不是换道路径
   *
   * 如果已有有效路径或正在换道，直接返回OK跳过
   */
  if (!reference_line_info->path_data().Empty() ||
      reference_line_info->IsChangeLanePath()) {
    return Status::OK();  /**< 返回成功，不执行回退 */
  }

  /**
   * @brief 创建候选路径容器
   */
  std::vector<PathBoundary> candidate_path_boundaries;
  std::vector<PathData> candidate_path_data;

  /**
   * @brief 获取起始点SL状态
   */
  GetStartPointSLState();

  /**
   * @brief 决定路径边界
   */
  if (!DecidePathBounds(&candidate_path_boundaries)) {
    return Status::OK();  /**< 注意：失败也返回OK */
  }

  /**
   * @brief 优化路径
   */
  if (!OptimizePath(candidate_path_boundaries, &candidate_path_data)) {
    return Status::OK();  /**< 注意：失败也返回OK */
  }

  /**
   * @brief 评估路径并更新参考线信息
   */
  if (!AssessPath(&candidate_path_data,
                  reference_line_info->mutable_path_data())) {
    AERROR << "Path assessment failed";  /**< 评估失败但仍返回OK */
  }

  return Status::OK();
}

/**
 * @brief 决定路径边界
 *
 * @param boundary 输出：路径边界数组
 * @return bool 是否成功
 *
 * 回退路径的边界决定流程（简化版）：
 * 1. 初始化边界为无限大区域
 * 2. 根据自车道信息收窄边界
 * 3. 根据自车位置扩展边界
 *
 * 与正常路径的区别：
 * - 不考虑借道、换道
 * - 不处理静态障碍物
 * - 简单沿车道中心行驶
 *
 * C++语法说明：
 * - boundary->emplace_back(): 在vector末尾构造空对象
 * - boundary->back(): 获取最后一个元素的引用
 */
bool FallbackPath::DecidePathBounds(std::vector<PathBoundary>* boundary) {
  /**
   * @brief 添加一个空的路径边界
   */
  boundary->emplace_back();

  /**
   * @brief 获取边界引用
   */
  auto& path_bound = boundary->back();

  /**
   * @brief 第1步：初始化路径边界为无限大区域
   * PathBoundsDeciderUtil::InitPathBoundary():
   *   将边界初始化为足够大的区域
   */
  if (!PathBoundsDeciderUtil::InitPathBoundary(*reference_line_info_,
                                               &path_bound, init_sl_state_)) {
    const std::string msg = "Failed to initialize path boundaries.";
    AERROR << msg;
    return false;
  }

  /**
   * @brief 第2步：根据自车道信息决定粗糙边界
   * GetBoundaryFromSelfLane():
   *   根据当前车道宽度、位置收窄边界
   */
  if (!PathBoundsDeciderUtil::GetBoundaryFromSelfLane(
          *reference_line_info_, init_sl_state_, &path_bound)) {
    AERROR << "Failed to decide a rough boundary based on self lane.";
    return false;
  }

  /**
   * @brief 第3步：根据自车位置扩展边界
   * ExtendBoundaryByADC():
   *   考虑自车几何尺寸，确保自车在边界内
   * config_.extend_buffer(): 扩展缓冲区大小
   */
  if (!PathBoundsDeciderUtil::ExtendBoundaryByADC(
          *reference_line_info_, init_sl_state_, config_.extend_buffer(),
          &path_bound)) {
    AERROR << "Failed to decide a rough boundary based on adc.";
    return false;
  }

  /**
   * @brief 设置路径标签
   * "fallback/self": 表示这是回退路径，沿自车道行驶
   * absl::StrCat(): Abseil库的字符串拼接函数
   */
  path_bound.set_label(absl::StrCat("fallback/", "self"));

  /**
   * @brief 记录调试信息
   */
  RecordDebugInfo(path_bound, path_bound.label(), reference_line_info_);
  return true;
}

/**
 * @brief 优化路径
 *
 * @param path_boundaries 输入：路径边界数组
 * @param candidate_path_data 输出：候选路径数据数组
 * @return bool 是否成功
 *
 * 优化流程（与正常路径相同）：
 * 1. 计算加速度边界
 * 2. 估计jerk边界
 * 3. 调用QP优化器
 * 4. 转换结果为PathData格式
 *
 * C++语法说明：
 * - const std::vector<PathBoundary>&: 常量引用，输入参数
 * - std::vector<PathData>*: 指针参数，输出结果
 * - std::array<double, 3>: 固定大小数组，存储状态[x, dx, ddx]
 */
bool FallbackPath::OptimizePath(
    const std::vector<PathBoundary>& path_boundaries,
    std::vector<PathData>* candidate_path_data) {
  /**
   * @brief 获取路径优化配置
   */
  const auto& config = config_.path_optimizer_config();

  /**
   * @brief 获取参考线引用
   */
  const ReferenceLine& reference_line = reference_line_info_->reference_line();

  /**
   * @brief 设置终点状态
   * end_state = {0.0, 0.0, 0.0}:
   * - 终点横向位置为0（沿参考线）
   * - 终点横向速度为0
   * - 终点横向加速度为0
   */
  std::array<double, 3> end_state = {0.0, 0.0, 0.0};

  /**
   * @brief 遍历每个路径边界
   */
  for (const auto& path_boundary : path_boundaries) {
    /**
     * @brief 获取边界大小
     */
    size_t path_boundary_size = path_boundary.boundary().size();

    /**
     * @brief 检查边界有效性
     * 必须至少有两个点才能进行优化
     */
    if (path_boundary_size <= 1U) {
      AERROR << "Get invalid path boundary with size: " << path_boundary_size;
      return false;
    }

    /**
     * @brief 创建优化结果容器
     * opt_l: 优化后的横向位置
     * opt_dl: 优化后的横向速度
     * opt_ddl: 优化后的横向加速度
     * ddl_bounds: 加速度边界
     */
    std::vector<double> opt_l, opt_dl, opt_ddl;
    std::vector<std::pair<double, double>> ddl_bounds;

    /**
     * @brief 计算加速度边界
     * CalculateAccBound():
     *   根据车辆物理限制和道路曲率计算
     */
    PathOptimizerUtil::CalculateAccBound(path_boundary, reference_line,
                                         &ddl_bounds);

    /**
     * @brief 估计jerk边界
     * EstimateJerkBoundary():
     *   根据车速计算最大允许jerk
     * std::fmax(): 取较大值，确保分母不为零
     */
    const double jerk_bound = PathOptimizerUtil::EstimateJerkBoundary(
        std::fmax(init_sl_state_.first[1], 1e-12));

    /**
     * @brief 初始化参考线和权重
     * ref_l: 参考横向位置（初始化为0）
     * weight_ref_l: 参考线权重（使用配置的权重值）
     */
    std::vector<double> ref_l(path_boundary_size, 0);
    std::vector<double> weight_ref_l(path_boundary_size,
                                     config.path_reference_l_weight());

    /**
     * @brief 执行路径优化
     * OptimizePath():
     *   使用分段jerk优化算法
     */
    bool res_opt = PathOptimizerUtil::OptimizePath(
        init_sl_state_, end_state, ref_l, weight_ref_l, path_boundary,
        ddl_bounds, jerk_bound, config, &opt_l, &opt_dl, &opt_ddl);

    /**
     * @brief 如果优化成功，转换结果
     */
    if (res_opt) {
      /**
       * @brief 转换为Frenet帧路径
       * ToPiecewiseJerkPath():
       *   将优化结果(x, dx, ddx)转换为FrenetFramePath
       */
      auto frenet_frame_path = PathOptimizerUtil::ToPiecewiseJerkPath(
          opt_l, opt_dl, opt_ddl, path_boundary.delta_s(),
          path_boundary.start_s());

      /**
       * @brief 创建PathData并设置属性
       */
      PathData path_data;
      path_data.SetReferenceLine(&reference_line);
      path_data.SetFrenetPath(std::move(frenet_frame_path));

      /**
       * @brief 如果使用前轴中心作为规划参考
       * 需要转换为后轴中心
       */
      if (FLAGS_use_front_axe_center_in_path_planning) {
        auto discretized_path = DiscretizedPath(
            PathOptimizerUtil::ConvertPathPointRefFromFrontAxeToRearAxe(
                path_data));
        path_data.SetDiscretizedPath(discretized_path);
      }

      /**
       * @brief 设置路径标签和阻塞障碍物ID
       */
      path_data.set_path_label(path_boundary.label());
      path_data.set_blocking_obstacle_id(path_boundary.blocking_obstacle_id());

      /**
       * @brief 添加到候选路径数组
       * std::move(path_data): 移动语义，避免拷贝
       */
      candidate_path_data->push_back(std::move(path_data));
    }
  }

  /**
   * @brief 检查是否有有效路径
   */
  if (candidate_path_data->empty()) {
    return false;
  }
  return true;
}

/**
 * @brief 评估路径
 *
 * @param candidate_path_data 输入/输出：候选路径数组
 * @param final_path 输出：最终路径
 * @return bool 是否通过评估
 *
 * 评估检查（比正常路径更严格）：
 * 1. 路径非空
 * 2. 路径不能偏离参考线太远
 * 3. 路径不能偏离道路太远
 *
 * 回退路径的评估重点：
 * - 安全性优先
 * - 确保车辆能安全到达
 *
 * C++语法说明：
 * - PathData curr_path_data = candidate_path_data->back():
 *   使用副本而非引用，因为后续可能修改
 */
bool FallbackPath::AssessPath(std::vector<PathData>* candidate_path_data,
                              PathData* final_path) {
  /**
   * @brief 获取最后一个候选路径
   * 回退路径只有一个候选，直接使用
   */
  PathData curr_path_data = candidate_path_data->back();

  /**
   * @brief 记录调试信息
   */
  RecordDebugInfo(curr_path_data, curr_path_data.path_label(),
                  reference_line_info_);

  /**
   * @brief 第1项检查：路径非空
   */
  if (curr_path_data.Empty()) {
    ADEBUG << "Fallback Path: path data is empty.";
    return false;
  }

  /**
   * @brief 第2项检查：路径不能严重偏离参考线
   * IsGreatlyOffReferenceLine():
   *   检查路径是否远离参考线
   *   回退路径要求更严格，因为要保证沿参考线行驶
   */
  if (PathAssessmentDeciderUtil::IsGreatlyOffReferenceLine(curr_path_data)) {
    ADEBUG << "Fallback Path: ADC is greatly off reference line.";
    return false;
  }

  /**
   * @brief 第3项检查：路径不能严重偏离道路
   * IsGreatlyOffRoad():
   *   检查路径是否在道路范围内
   *   确保车辆不会驶出道路边界
   */
  if (PathAssessmentDeciderUtil::IsGreatlyOffRoad(*reference_line_info_,
                                                  curr_path_data)) {
    ADEBUG << "Fallback Path: ADC is greatly off road.";
    return false;
  }

  /**
   * @brief 再次检查路径是否为空
   */
  if (curr_path_data.Empty()) {
    AINFO << "Lane follow path is empty after trimed";
    return false;
  }

  /**
   * @brief 设置最终路径
   */
  *final_path = curr_path_data;
  return true;
}

/**
 * @brief 命名空间结束标记
 */
}  // namespace planning
}  // namespace apollo
