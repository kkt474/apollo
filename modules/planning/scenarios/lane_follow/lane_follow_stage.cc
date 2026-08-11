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
 * @file lane_follow_stage.cc
 * @brief 车道跟随阶段实现文件
 *
 * 本文件实现LaneFollowStage类，是车道跟随场景的执行阶段。
 * 负责在参考线上进行路径规划和速度规划，生成最终的行驶轨迹。
 *
 * 主要功能：
 * 1. 执行任务列表（路径优化、速度优化等）
 * 2. 管理参考线规划
 * 3. 处理障碍物决策
 * 4. 轨迹合成与验证
 *
 * 设计特点：
 * - 继承自Stage基类
 * - 任务列表驱动：按顺序执行一系列Task
 * - 支持多参考线：遍历所有参考线进行规划
 * - 回退机制：任务失败时执行fallback任务
 *
 * C++语法说明：
 * - for (auto& x : container): 范围for循环，引用避免拷贝
 * - std::chrono: C++时间库，用于精确计时
 * - Clock::NowInSeconds(): Cyber RT时钟获取当前时间
 * - mutable_xxx(): protobuf可写访问器
 * - ->: 指针调用成员方法
 */
#include "modules/planning/scenarios/lane_follow/lane_follow_stage.h"

#include <utility>  /**< C++标准实用工具库 */

#include "cyber/common/log.h"  /**< Cyber RT日志系统 */
#include "cyber/time/clock.h"  /**< Cyber RT时钟系统 */
#include "modules/common/math/math_utils.h"  /**< Apollo数学工具 */
#include "modules/common/util/point_factory.h" /**< 点工厂工具 */
#include "modules/common/util/string_util.h"  /**< 字符串工具 */
#include "modules/common/vehicle_state/vehicle_state_provider.h" /**< 车辆状态提供者 */
#include "modules/map/hdmap/hdmap.h"  /**< 高精地图 */
#include "modules/map/hdmap/hdmap_common.h" /**< 地图通用定义 */
#include "modules/planning/planning_base/common/ego_info.h" /**< 自车信息 */
#include "modules/planning/planning_base/common/frame.h" /**< 规划帧 */
#include "modules/planning/planning_base/common/speed_profile_generator.h" /**< 速度剖面生成器 */
#include "modules/planning/planning_base/gflags/planning_gflags.h" /**< 规划配置标志 */
#include "modules/planning/planning_base/math/constraint_checker/constraint_checker.h" /**< 约束检查器 */
#include "modules/planning/planning_interface_base/task_base/task.h" /**< 任务基类 */

namespace apollo {
/**
 * apollo:: - Apollo最外层命名空间
 */
namespace planning {

/**
 * using声明 - 将其他命名空间的类型引入当前作用域
 */
using apollo::common::ErrorCode;      /**< 错误代码 */
using apollo::common::SLPoint;        /**< SL坐标点 */
using apollo::common::Status;         /**< 状态类型 */
using apollo::common::TrajectoryPoint; /**< 轨迹点 */
using apollo::common::util::PointFactory; /**< 点工厂 */
using apollo::cyber::Clock;           /**< Cyber RT时钟 */

/**
 * 匿名命名空间内定义常量
 * static constexpr: 静态编译时常量
 * kStraightForwardLineCost: 直行路径的固定代价
 */
namespace {
constexpr double kStraightForwardLineCost = 10.0;  /**< 直行路径代价：10 */
}  // namespace

/**
 * @brief 记录障碍物调试信息
 *
 * 将障碍物的决策信息记录到调试数据结构中。
 * 用于可视化障碍物处理结果和决策过程。
 *
 * @param reference_line_info 参考线信息指针
 *
 * 语法说明：
 * - ReferenceLineInfo*: 原始指针指向参考线信息
 * - if (!FLAGS_enable_record_debug): 检查是否启用调试记录
 * - mutable_xxx(): protobuf可写访问器
 * - for (const auto& x : container): 范围for循环遍历常量容器
 * - .Items(): 获取容器中所有元素的vector
 * - add_xxx(): 在repeated字段中添加新元素
 * - CopyFrom(): 深拷贝protobuf消息
 */
void LaneFollowStage::RecordObstacleDebugInfo(
    ReferenceLineInfo* reference_line_info) {
  /**
   * 检查是否启用调试记录
   * FLAGS_enable_record_debug: gflags配置标志
   */
  if (!FLAGS_enable_record_debug) {
    ADEBUG << "Skip record debug info";
    return;  /**< 未启用则直接返回 */
  }

  /**
   * 获取调试信息可写指针
   * reference_line_info->mutable_debug()
   *   - ->: 指针调用成员方法
   *   - mutable_debug(): 返回Debug类型的可写指针
   */
  auto ptr_debug = reference_line_info->mutable_debug();

  /**
   * 获取路径决策
   * reference_line_info->path_decision()
   *   - 返回PathDecision对象，包含所有障碍物决策
   */
  const auto path_decision = reference_line_info->path_decision();

  /**
   * 遍历所有障碍物
   * for (const auto obstacle : path_decision->obstacles().Items())
   *   - obstacle是const Obstacle*类型
   *   - .Items()返回障碍物列表的vector
   */
  for (const auto obstacle : path_decision->obstacles().Items()) {
    /**
     * 添加障碍物调试信息
     * ptr_debug->mutable_planning_data()->add_obstacle()
     *   - mutable_planning_data(): 获取PlanningData可写指针
     *   - add_obstacle(): 在repeated字段中添加新障碍物
     */
    auto obstacle_debug = ptr_debug->mutable_planning_data()->add_obstacle();

    /**
     * 设置障碍物ID
     * obstacle->Id(): 获取障碍物唯一标识符
     * obstacle_debug->set_id(): 设置ID字段
     */
    obstacle_debug->set_id(obstacle->Id());

    /**
     * 拷贝感知SL边界
     * mutable_sl_boundary(): 获取SL边界可写指针
     * CopyFrom(): 深拷贝消息
     */
    obstacle_debug->mutable_sl_boundary()->CopyFrom(
        obstacle->PerceptionSLBoundary());

    /**
     * 获取决策标签和决策列表
     * obstacle->decider_tags(): 返回决策标签列表
     * obstacle->decisions(): 返回决策列表
     */
    const auto& decider_tags = obstacle->decider_tags();
    const auto& decisions = obstacle->decisions();

    /**
     * 检查标签和决策数量是否匹配
     * if (decider_tags.size() != decisions.size())
     */
    if (decider_tags.size() != decisions.size()) {
      AERROR << "decider_tags size: " << decider_tags.size()
             << " different from decisions size:" << decisions.size();
    }

    /**
     * 遍历所有决策标签
     * for (size_t i = 0; i < decider_tags.size(); ++i)
     *   - size_t: 无符号整数类型，用于索引
     *   - ++i: 前置递增
     */
    for (size_t i = 0; i < decider_tags.size(); ++i) {
      /**
       * 添加决策标签
       * obstacle_debug->add_decision_tag()
       *   - 在repeated字段中添加新元素
       */
      auto decision_tag = obstacle_debug->add_decision_tag();

      /**
       * 设置决策标签和决策
       * decision_tag->set_decider_tag(): 设置标签
       * decision_tag->mutable_decision()->CopyFrom(): 拷贝决策
       */
      decision_tag->set_decider_tag(decider_tags[i]);
      decision_tag->mutable_decision()->CopyFrom(decisions[i]);
    }
  }
}

/**
 * @brief 车道跟随阶段处理函数
 *
 * LaneFollowStage的核心函数，在每个规划周期被调用。
 * 遍历所有参考线进行规划，找到可行驶的参考线。
 *
 * 处理流程：
 * 1. 检查参考线列表是否为空
 * 2. 遍历每条参考线执行规划
 * 3. 检查是否存在可行驶的参考线
 *
 * @param planning_start_point 规划起始点
 * @param frame 规划帧数据
 * @return StageResult 阶段处理结果
 *
 * 语法说明：
 * - const TrajectoryPoint&: 常量引用，避免拷贝
 * - Frame*: 原始指针，指向规划帧
 * - StageResult: 阶段结果类，包含状态和错误信息
 * - .empty(): 容器方法，检查是否为空
 * - unsigned int: 无符号整数类型
 * - for (auto& x : container): 范围for循环，可写引用
 */
StageResult LaneFollowStage::Process(
    const TrajectoryPoint& planning_start_point, Frame* frame) {
  /**
   * 检查参考线列表是否为空
   * frame->reference_line_info().empty()
   *   - frame->: 指针解引用访问成员
   *   - reference_line_info(): 获取参考线信息列表
   *   - .empty(): 检查是否为空
   */
  if (frame->reference_line_info().empty()) {
    return StageResult(StageStatusType::FINISHED);  /**< 参考线为空，阶段完成 */
  }

  /**
   * has_drivable_reference_line: 是否有可行驶参考线的标志
   * 初始化为false
   */
  bool has_drivable_reference_line = false;

  /**
   * 输出调试信息
   * ADEBUG: Apollo调试级别日志
   */
  ADEBUG << "Number of reference lines:\t"
         << frame->mutable_reference_line_info()->size();

  /**
   * count: 参考线计数器
   * unsigned int: 无符号整数，从0开始计数
   */
  unsigned int count = 0;

  /**
   * result: 阶段结果对象
   * StageResult: 包含阶段状态和任务状态
   */
  StageResult result;

  /**
   * 遍历所有参考线
   * for (auto& reference_line_info : *frame->mutable_reference_line_info())
   *   - frame->mutable_reference_line_info(): 获取可写的参考线列表
   *   - *: 解引用获取list对象
   *   - auto&: 引用避免拷贝，可修改元素
   */
  for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
    /**
     * TODO(SHU): 需要重构
     * 计数器自增并检查是否越界
     */
    if (count++ == frame->mutable_reference_line_info()->size()) {
      break;  /**< 越界则跳出循环 */
    }

    /**
     * 输出当前参考线编号
     */
    ADEBUG << "No: [" << count << "] Reference Line.";
    ADEBUG << "IsChangeLanePath: " << reference_line_info.IsChangeLanePath();

    /**
     * 如果已经找到可行驶参考线
     * if (has_drivable_reference_line)
     */
    if (has_drivable_reference_line) {
      reference_line_info.SetDrivable(false);  /**< 设置为不可行驶 */
      break;  /**< 跳出循环 */
    }

    /**
     * 在当前参考线上规划
     * PlanOnReferenceLine(planning_start_point, frame, &reference_line_info)
     *   - 将参考线信息指针传递给规划函数
     */
    result =
        PlanOnReferenceLine(planning_start_point, frame, &reference_line_info);

    /**
     * 检查规划是否有错误
     * if (!result.HasError())
     */
    if (!result.HasError()) {
      /**
       * 规划成功，检查是否为换道路径
       */
      if (!reference_line_info.IsChangeLanePath()) {
        ADEBUG << "reference line is NOT lane change ref.";
        has_drivable_reference_line = true;  /**< 非换道参考线，可行驶 */
        continue;  /**< 继续下一条参考线 */
      }

      /**
       * 换道路径，检查代价是否足够小
       * if (reference_line_info.Cost() < kStraightForwardLineCost)
       */
      if (reference_line_info.Cost() < kStraightForwardLineCost) {
        /**
         * 换道成功
         * 注释：路径和速度优化在目标车道成功
         */
        has_drivable_reference_line = true;
        reference_line_info.SetDrivable(true);
      } else {
        reference_line_info.SetDrivable(false);  /**< 代价太大，不可行驶 */
        ADEBUG << "\tlane change failed";
      }
    } else {
      reference_line_info.SetDrivable(false);  /**< 规划失败，不可行驶 */
    }
  }

  /**
   * 返回阶段结果
   * 三元运算符：根据has_drivable_reference_line设置状态
   * ? : operator: 条件 ? 值1 : 值2
   */
  return has_drivable_reference_line
             ? result.SetStageStatus(StageStatusType::RUNNING)  /**< 继续运行 */
             : result.SetStageStatus(StageStatusType::ERROR);     /**< 发生错误 */
}

/**
 * @brief 在参考线上规划
 *
 * 对单条参考线进行完整的路径和速度规划。
 * 按顺序执行任务列表中的所有任务。
 *
 * 任务列表通常包括：
 * 1. 路径边界决策
 * 2. 路径优化
 * 3. 速度边界决策
 * 4. 速度优化
 * 5. 轨迹合成
 *
 * @param planning_start_point 规划起始点
 * @param frame 规划帧数据
 * @param reference_line_info 参考线信息
 * @return StageResult 阶段结果
 *
 * 语法说明：
 * - std::chrono: C++时间库
 * - std::chrono::duration<double>: 以秒为单位的duration
 * - std::chrono::system_clock::now(): 系统当前时间点
 * - .time_since_epoch(): 获取从epoch到现在的时长
 * - for (auto task : task_list_): 遍历任务列表
 */
StageResult LaneFollowStage::PlanOnReferenceLine(
    const TrajectoryPoint& planning_start_point, Frame* frame,
    ReferenceLineInfo* reference_line_info) {
  /**
   * 如果不是换道路径，添加直行代价
   * if (!reference_line_info->IsChangeLanePath())
   * reference_line_info->AddCost(kStraightForwardLineCost)
   *   - 增加规划代价，使换道路径优先被选择
   */
  if (!reference_line_info->IsChangeLanePath()) {
    reference_line_info->AddCost(kStraightForwardLineCost);
  }

  /**
   * 输出调试信息
   */
  ADEBUG << "planning start point:" << planning_start_point.DebugString();
  ADEBUG << "Current reference_line_info is IsChangeLanePath: "
         << reference_line_info->IsChangeLanePath();

  /**
   * ret: 初始化阶段结果
   * StageResult: 默认状态为UNKNOWN
   */
  StageResult ret;

  /**
   * 遍历任务列表执行任务
   * for (auto task : task_list_)
   *   - task_list_: LaneFollowStage的成员变量，存储任务列表
   *   - task是shared_ptr<Task>类型
   */
  for (auto task : task_list_) {
    /**
     * 记录任务开始时间
     * Clock::NowInSeconds(): Cyber RT时钟，返回当前时间（秒）
     */
    const double start_timestamp = Clock::NowInSeconds();

    /**
     * 另一种计时方式，使用std::chrono
     * std::chrono::duration<double>: 表示秒级精度的时间
     * std::chrono::system_clock::now(): 系统当前时间点
     * .time_since_epoch(): 转换为从epoch开始的duration
     * .count(): 获取duration的数值
     */
    const auto start_planning_perf_timestamp =
        std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();

    /**
     * 执行任务
     * task->Execute(frame, reference_line_info)
     *   - Execute是Task的虚函数，由具体任务实现
     *   - 返回TaskStatus表示执行结果
     * ret.SetTaskStatus(): 保存任务状态
     */
    ret.SetTaskStatus(task->Execute(frame, reference_line_info));

    /**
     * 记录任务结束时间
     */
    const double end_timestamp = Clock::NowInSeconds();
    const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;  /**< 转换为毫秒 */

    /**
     * 输出调试信息
     * ADEBUG: Apollo调试日志
     */
    ADEBUG << "after task[" << task->Name()
           << "]:" << reference_line_info->PathSpeedDebugString();
    ADEBUG << task->Name() << " time spend: " << time_diff_ms << " ms.";

    /**
     * 记录任务调试信息
     * RecordDebugInfo(): 基类方法，记录任务执行信息
     */
    RecordDebugInfo(reference_line_info, task->Name(), time_diff_ms);

    /**
     * 记录性能时间戳
     */
    const auto end_planning_perf_timestamp =
        std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();
    const auto plnning_perf_ms =
        (end_planning_perf_timestamp - start_planning_perf_timestamp) * 1000;
    AINFO << "Planning Perf: task name [" << task->Name() << "], "
          << plnning_perf_ms << " ms.";

    /**
     * 检查任务执行是否有错误
     * if (ret.IsTaskError())
     */
    if (ret.IsTaskError()) {
      AERROR << "Failed to run tasks[" << task->Name()
             << "], Error message: " << ret.GetTaskStatus().error_message();
      break;  /**< 出错则跳出任务循环 */
    }

    /**
     * TODO(SHU): 暂时禁用参考线顺序更改
     * 注释掉的代码用于在lane_change_decider之后更新参考线
     */
  }

  /**
   * 记录障碍物调试信息
   */
  RecordObstacleDebugInfo(reference_line_info);

  /**
   * 设置轨迹类型为普通
   * reference_line_info->set_trajectory_type(ADCTrajectory::NORMAL)
   */
  reference_line_info->set_trajectory_type(ADCTrajectory::NORMAL);

  /**
   * 如果任务执行出错，执行fallback任务
   * fallback_task_: 后备任务，用于规划失败时的处理
   */
  if (ret.IsTaskError()) {
    fallback_task_->Execute(frame, reference_line_info);
  }

  /**
   * 轨迹合成
   * DiscretizedTrajectory: 离散轨迹类
   * CombinePathAndSpeedProfile(): 合并路径和速度剖面生成完整轨迹
   */
  DiscretizedTrajectory trajectory;
  if (!reference_line_info->CombinePathAndSpeedProfile(
          planning_start_point.relative_time(),
          planning_start_point.path_point().s(), &trajectory)) {
    const std::string msg = "Fail to aggregate planning trajectory.";
    AERROR << msg;
    return ret.SetStageStatus(StageStatusType::ERROR, msg);  /**< 轨迹合成失败 */
  }

  /**
   * 检查参考线上是否有目的地
   * dest_stop_s: 目的地停车点的s坐标，-1表示无目的地
   */
  double dest_stop_s = -1.0;
  for (const auto* obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    /**
     * 检查障碍物是否有STOP决策且原因为DESTINATION
     * obstacle->LongitudinalDecision().has_stop()
     *   - 获取纵向决策，判断是否有停车决策
     * STOP_REASON_DESTINATION: 停车原因枚举，目的地
     */
    if (obstacle->LongitudinalDecision().has_stop() &&
        obstacle->LongitudinalDecision().stop().reason_code() ==
            STOP_REASON_DESTINATION) {
      /**
       * 获取停车点SL坐标
       * GetStopSL(): 根据停车决策计算SL坐标
       */
      SLPoint dest_sl = GetStopSL(obstacle->LongitudinalDecision().stop(),
                                  reference_line_info->reference_line());
      dest_stop_s = dest_sl.s();  /**< 保存目的地s坐标 */
    }
  }

  /**
   * 遍历所有静态障碍物，添加停车代价
   * 用于在目的地之前有障碍物时减速停车
   */
  for (const auto* obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    if (obstacle->IsVirtual()) {
      continue;  /**< 跳过虚拟障碍物 */
    }
    if (!obstacle->IsStatic()) {
      continue;  /**< 跳过动态障碍物 */
    }
    if (obstacle->LongitudinalDecision().has_stop()) {
      bool add_stop_obstacle_cost = false;
      if (dest_stop_s < 0.0) {
        add_stop_obstacle_cost = true;  /**< 无目的地，添加停车代价 */
      } else {
        SLPoint stop_sl = GetStopSL(obstacle->LongitudinalDecision().stop(),
                                    reference_line_info->reference_line());
        /**
         * 检查停车点是否在目的地之前
         * 且距离自车足够近（20米以内）
         */
        if (stop_sl.s() < dest_stop_s &&
            (dest_stop_s - reference_line_info->AdcSlBoundary().end_s()) <
                20.0) {
          add_stop_obstacle_cost = true;
        }
      }
      if (add_stop_obstacle_cost) {
        static constexpr double kReferenceLineStaticObsCost = 1e3;  /**< 静态障碍物代价 */
        reference_line_info->AddCost(kReferenceLineStaticObsCost);
      }
    }
  }

  /**
   * 检查轨迹是否有效
   * FLAGS_enable_trajectory_check: 配置标志
   * ConstraintChecker::ValidTrajectory(): 验证轨迹是否满足约束
   */
  if (FLAGS_enable_trajectory_check) {
    if (ConstraintChecker::ValidTrajectory(trajectory) !=
        ConstraintChecker::Result::VALID) {
      const std::string msg = "Current planning trajectory is not valid.";
      AERROR << msg;
      return ret.SetStageStatus(StageStatusType::ERROR, msg);  /**< 轨迹无效 */
    }
  }

  /**
   * 设置轨迹和可行驶标志
   * reference_line_info->SetTrajectory(trajectory): 设置规划轨迹
   * reference_line_info->SetDrivable(true): 设置为可行驶
   */
  reference_line_info->SetTrajectory(trajectory);
  reference_line_info->SetDrivable(true);
  ret.SetStageStatus(StageStatusType::RUNNING);  /**< 设置阶段状态为运行中 */
  return ret;  /**< 返回结果 */
}

/**
 * @brief 获取停车点的SL坐标
 *
 * 将停车点的XY坐标转换为参考线的SL坐标。
 *
 * @param stop_decision 停车决策
 * @param reference_line 参考线
 * @return SLPoint SL坐标点
 *
 * 语法说明：
 * - const ObjectStop&: 常量引用停车决策
 * - const ReferenceLine&: 常量引用参考线
 * - reference_line.XYToSL(): 将XY坐标转换为SL坐标
 */
SLPoint LaneFollowStage::GetStopSL(const ObjectStop& stop_decision,
                                   const ReferenceLine& reference_line) const {
  SLPoint sl_point;  /**< SL坐标点 */
  reference_line.XYToSL(stop_decision.stop_point(), &sl_point);  /**< 坐标转换 */
  return sl_point;  /**< 返回SL坐标 */
}

}  // namespace planning
}  // namespace apollo
