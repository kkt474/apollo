/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
 * @file
 * @brief 保持清空区域交通规则实现文件
 *
 * 功能说明：
 * 实现了保持清空(Keep Clear)区域交通规则，用于处理：
 * 1. 禁止停车区域（Clear Area）：如加油站、充电站等禁止停车的区域
 * 2. 交叉路口（Junction）：需要保持清空让其他车辆通行的区域
 *
 * 核心概念：
 * - Keep Clear Zone：保持清空区域，车辆不能在此区域停留
 * - PNC Junction：PNC（规划与控制）路口，多条路径交汇处
 * - Creeping：缓慢行驶阶段，在某些场景下需要缓慢通过路口
 * - STBoundary：时空边界，描述障碍物在时空中的占用区域
 *
 * C++语法说明：
 * - std::vector：动态数组容器
 * - protobuf：Google Protocol Buffers，用于结构化数据序列化
 * - const_cast：常量类型转换，用于在某些情况下移除常量性
 **/
#include "modules/planning/traffic_rules/keepclear/keep_clear.h"

#include <memory>
#include <vector>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/planning/planning_base/proto/planning_config.pb.h"
#include "modules/planning/planning_base/proto/planning_status.pb.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/planning/planning_base/common/planning_context.h"

namespace apollo {
/**
 * @brief Apollo项目主命名空间
 *
 * 命名空间说明：
 * apollo是百度自动驾驶项目的顶级命名空间
 * 包含common、hdmap、planning、control等多个子命名空间
 */
namespace planning {
/**
 * @brief 规划模块的命名空间
 *
 * 包含内容：
 * - 交通规则实现（traffic_rules）
 * - 场景管理（scenarios）
 * - 路径规划任务（tasks）
 * - 规划器实现（planners）
 */

using apollo::common::Status;
/**
 * @brief 使用apollo::common::Status类型
 *
 * 语法说明：
 * using声明将其他命名空间中的类型引入当前作用域
 * Status是Apollo中用于表示操作结果的状态类
 * 包含OK（成功）、ERROR（错误）等状态
 */

using apollo::hdmap::PathOverlap;
/**
 * @brief 使用apollo::hdmap::PathOverlap类型
 *
 * PathOverlap结构体说明：
 * 表示地图中路标与路径的交叉区域
 * 主要成员：
 * - object_id：对象ID
 * - start_s：重叠区域起始点沿路径的s坐标
 * - end_s：重叠区域结束点沿路径的s坐标
 */

/**
 * @brief 初始化保持清空规则
 *
 * @param name 规则名称
 * @param injector 依赖注入器指针
 * @return bool 初始化是否成功
 *
 * 功能说明：
 * 初始化KeepClear交通规则实例
 * 加载规则配置并准备处理保持清空区域
 *
 * 算法流程：
 * 1. 调用基类TrafficRule的Init方法进行基础初始化
 * 2. 如果基础初始化失败，返回false
 * 3. 从配置文件加载KeepClearConfig配置
 * 4. 返回配置加载结果
 */
bool KeepClear::Init(const std::string& name,
                     const std::shared_ptr<DependencyInjector>& injector) {
  if (!TrafficRule::Init(name, injector)) {
    return false;  /**< 基类初始化失败，返回false */
  }
  return TrafficRule::LoadConfig<KeepClearConfig>(&config_);
  /**
   * @brief 加载保持清空区域配置
   *
   * TrafficRule::LoadConfig<T>模板函数：
   * 从protobuf配置文件加载KeepClearConfig
   * &config_将配置存储到成员变量中
   */
}

/**
 * @brief 应用保持清空规则
 *
 * @param frame 当前规划帧
 * @param reference_line_info 参考线信息
 * @return Status 应用结果状态
 *
 * 功能说明：
 * 保持清空规则的主入口函数，处理两种场景：
 * 1. 保持清空区域（enable_keep_clear_zone）
 * 2. 交叉路口（enable_junction）
 *
 * 算法流程：
 * 1. 检查规则是否启用
 * 2. 处理保持清空区域
 * 3. 处理交叉路口（如果启用）
 * 4. 返回成功状态
 */
Status KeepClear::ApplyRule(Frame* const frame,
                            ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  /**
   * @brief 空指针检查
   *
   * CHECK_NOTNULL是Apollo定义的宏：
   * 如果frame为nullptr，程序会终止并输出错误信息
   */
  CHECK_NOTNULL(reference_line_info);

  /**
   * @brief 处理保持清空区域
   *
   * 配置项：config_.enable_keep_clear_zone()
   * 功能：是否启用保持清空区域处理
   */
  if (config_.enable_keep_clear_zone()) {
    /**
     * @brief 获取所有清空区域重叠区
     *
     * reference_line_info->reference_line().map_path()：获取地图路径
     * .clear_area_overlaps()：获取所有清空区域（如加油站、充电站等）
     *
     * 返回类型：std::vector<PathOverlap>
     */
    const std::vector<PathOverlap>& keep_clear_overlaps =
        reference_line_info->reference_line().map_path().clear_area_overlaps();

    /**
     * @brief 遍历所有清空区域
     */
    for (const auto& keep_clear_overlap : keep_clear_overlaps) {
      /**
       * @brief 创建虚拟障碍物ID
       *
       * KEEP_CLEAR_VO_ID_PREFIX：清空区域虚拟障碍物ID前缀
       * 格式：KEEP_CLEAR_VO_ID_PREFIX + 清空区域object_id
       */
      const auto obstacle_id =
          KEEP_CLEAR_VO_ID_PREFIX + keep_clear_overlap.object_id;

      /**
       * @brief 构建清空区域障碍物
       *
       * 参数说明：
       * - frame：当前规划帧
       * - reference_line_info：参考线信息
       * - obstacle_id：虚拟障碍物ID
       * - keep_clear_overlap.start_s：清空区域起始s坐标
       * - keep_clear_overlap.end_s：清空区域结束s坐标
       *
       * 返回值：true表示成功创建障碍物
       */
      if (BuildKeepClearObstacle(frame, reference_line_info, obstacle_id,
                                 keep_clear_overlap.start_s,
                                 keep_clear_overlap.end_s)) {
        /**
         * @brief 创建成功，输出调试信息
         *
         * ADEBUG：调试日志宏，仅在调试模式输出
         */
        ADEBUG << "KEEP_CLAER for keep_clear_zone["
               << keep_clear_overlap.object_id << "] s["
               << keep_clear_overlap.start_s << ", " << keep_clear_overlap.end_s
               << "] BUILD";
        /**
         * @note KEEP_CLAER可能是KEEP_CLEAR的拼写错误
         */
      }
    }
  }

  /**
   * @brief 处理交叉路口
   *
   * 配置项：config_.enable_junction()
   * 功能：是否启用交叉路口处理
   *
   * 交叉路口处理流程：
   * 1. 获取首次遇到的各类重叠区（人行横道、停车标志、交通灯、PNC路口）
   * 2. 如果有PNC路口，调整其start_s与其他"停车类型"重叠区对齐
   * 3. 创建清空障碍物
   */
  if (config_.enable_junction()) {
    /**
     * @brief 定义各类重叠区指针
     *
     * 初始化为nullptr，表示尚未遇到该类型重叠区
     * 使用原始指针（而非智能指针）因为指向的是容器内的元素
     */
    hdmap::PathOverlap* crosswalk_overlap = nullptr;
    /**< @brief 人行横道重叠区指针 */
    hdmap::PathOverlap* stop_sign_overlap = nullptr;
    /**< @brief 停车标志重叠区指针 */
    hdmap::PathOverlap* traffic_light_overlap = nullptr;
    /**< @brief 交通灯重叠区指针 */
    hdmap::PathOverlap* pnc_junction_overlap = nullptr;
    /**< @brief PNC路口重叠区指针 */

    /**
     * @brief 获取首次遇到的各类重叠区
     *
     * FirstEncounteredOverlaps()返回类型：
     * std::vector<std::pair<OverlapType, PathOverlap>>
     * 表示沿路径首次遇到的各类重叠区列表
     */
    const auto& first_encountered_overlaps =
        reference_line_info->FirstEncounteredOverlaps();

    /**
     * @brief 遍历首次遇到的重叠区
     */
    for (const auto& overlap : first_encountered_overlaps) {
      ADEBUG << overlap.first << ", " << overlap.second.DebugString();
      /**
       * @brief 根据重叠区类型分类保存
       *
       * switch语句：C++多分支选择语句
       * overlap.first：重叠区类型（枚举值）
       * overlap.second：重叠区详细信息（PathOverlap）
       */
      switch (overlap.first) {
        /**
         * @brief 人行横道重叠区
         */
        case ReferenceLineInfo::CROSSWALK:
          ADEBUG << "CROSSWALK[" << overlap.second.object_id << "] s["
                 << overlap.second.start_s << ", " << overlap.second.end_s
                 << "]";
          /**
           * @brief const_cast用法说明：
           *
           * const_cast用于移除或添加const限定符
           * 这里overlap.second是const引用，但需要赋值给非const指针
           * 使用const_cast移除const限定
           *
           * 注意：这是安全的，因为指针只用于读取
           */
          crosswalk_overlap = const_cast<PathOverlap*>(&overlap.second);
          break;  /**< 跳出switch语句 */
        /**
         * @brief 停车标志重叠区
         */
        case ReferenceLineInfo::STOP_SIGN:
          ADEBUG << "STOP_SIGN[" << overlap.second.object_id << "] s["
                 << overlap.second.start_s << ", " << overlap.second.end_s
                 << "]";
          stop_sign_overlap = const_cast<PathOverlap*>(&overlap.second);
          break;
        /**
         * @brief 交通信号灯重叠区
         */
        case ReferenceLineInfo::SIGNAL:
          ADEBUG << "SIGNAL[" << overlap.second.object_id << "] s["
                 << overlap.second.start_s << ", " << overlap.second.end_s
                 << "]";
          traffic_light_overlap = const_cast<PathOverlap*>(&overlap.second);
          break;
        /**
         * @brief PNC路口重叠区
         */
        case ReferenceLineInfo::PNC_JUNCTION:
          ADEBUG << "PNC_JUNCTION[" << overlap.second.object_id << "] s["
                 << overlap.second.start_s << ", " << overlap.second.end_s
                 << "]";
          pnc_junction_overlap = const_cast<PathOverlap*>(&overlap.second);
          break;
        /**
         * @brief 其他类型重叠区，不处理
         */
        default:
          break;  /**< 默认情况，什么都不做 */
      }
    }

    /**
     * @brief 处理PNC路口
     *
     * 条件：存在PNC路口重叠区
     */
    if (pnc_junction_overlap != nullptr) {
      /**
       * @brief 获取自车前边缘s坐标
       */
      const double adc_front_edge_s =
          reference_line_info->AdcSlBoundary().end_s();

      /**
       * @brief 检查是否处于缓慢行驶阶段
       *
       * 如果处于缓慢行驶阶段，则不需要创建保持清空障碍物
       * 这是为了避免在缓慢行驶时重复创建障碍物
       */
      if (!IsCreeping(pnc_junction_overlap->start_s, adc_front_edge_s)) {
        /**
         * @brief 调整PNC路口start_s对齐
         *
         * 问题：PNC路口的start_s可能与其他"停车类型"重叠区不对齐
         * 解决：将PNC路口的start_s调整到与其他重叠区对齐
         *
         * 对齐优先级：
         * 1. 交通灯
         * 2. 停车标志
         * 3. 人行横道
         */

        double pnc_junction_start_s = pnc_junction_overlap->start_s;
        /**
         * @brief 待调整的PNC路口起始s坐标
         */

        /**
         * @brief 尝试与交通灯对齐
         *
         * std::fabs：浮点数绝对值函数
         * config_.align_with_traffic_sign_tolerance()：对齐容差
         */
        if (traffic_light_overlap != nullptr &&
            std::fabs(pnc_junction_start_s - traffic_light_overlap->start_s) <=
                config_.align_with_traffic_sign_tolerance()) {
          ADEBUG << "adjust pnc_junction_start_s[" << pnc_junction_start_s
                 << "] to traffic_light_start_s"
                 << traffic_light_overlap->start_s << "]";
          pnc_junction_start_s = traffic_light_overlap->start_s;
        }
        /**
         * @brief 尝试与停车标志对齐
         */
        else if (stop_sign_overlap != nullptr &&
                   std::fabs(pnc_junction_start_s -
                             stop_sign_overlap->start_s) <=
                       config_.align_with_traffic_sign_tolerance()) {
          ADEBUG << "adjust pnc_junction_start_s[" << pnc_junction_start_s
                 << "] to stop_sign_start_s" << stop_sign_overlap->start_s
                 << "]";
          pnc_junction_start_s = stop_sign_overlap->start_s;
        }
        /**
         * @brief 尝试与人行横道对齐
         */
        else if (crosswalk_overlap != nullptr &&
                   std::fabs(pnc_junction_start_s -
                             crosswalk_overlap->start_s) <=
                       config_.align_with_traffic_sign_tolerance()) {
          ADEBUG << "adjust pnc_junction_start_s[" << pnc_junction_start_s
                 << "] to cross_walk_start_s" << crosswalk_overlap->start_s
                 << "]";
          pnc_junction_start_s = crosswalk_overlap->start_s;
        }

        /**
         * @brief 创建PNC路口虚拟障碍物ID
         */
        const auto obstacle_id =
            KEEP_CLEAR_JUNCTION_VO_ID_PREFIX + pnc_junction_overlap->object_id;

        /**
         * @brief 构建PNC路口清空障碍物
         */
        if (BuildKeepClearObstacle(frame, reference_line_info, obstacle_id,
                                   pnc_junction_start_s,
                                   pnc_junction_overlap->end_s)) {
          ADEBUG << "KEEP_CLAER for junction["
                 << pnc_junction_overlap->object_id << "] s["
                 << pnc_junction_start_s << ", " << pnc_junction_overlap->end_s
                 << "] BUILD";
        }
      }
    }
  }

  return Status::OK();
  /**
   * @brief 返回成功状态
   */
}

/**
 * @brief 检查是否处于缓慢行驶阶段
 *
 * @param pnc_junction_start_s PNC路口起始s坐标
 * @param adc_front_edge_s 自车前边缘s坐标
 * @return bool 是否处于缓慢行驶阶段
 *
 * 功能说明：
 * 在某些场景下（如STOP_SIGN_UNPROTECTED_CREEP），车辆需要缓慢行驶通过路口
 * 如果处于这些场景的缓慢阶段，则不需要创建保持清空障碍物
 *
 * 算法流程：
 * 1. 获取当前场景阶段类型
 * 2. 检查是否是需要缓慢行驶的特定阶段
 * 3. 检查自车是否接近PNC路口（距离小于5米）
 */
bool KeepClear::IsCreeping(const double pnc_junction_start_s,
                           const double adc_front_edge_s) const {
  /**
   * @brief 获取当前场景阶段类型
   *
   * injector_->planning_context()：规划上下文
   * ->planning_status()：获取规划状态
   * ->scenario().stage_type()：获取当前场景阶段类型
   *
   * 返回值：字符串，表示当前阶段的类型
   */
  const auto& stage_type =
      injector_->planning_context()->planning_status().scenario().stage_type();

  /**
   * @brief 检查是否是需要缓慢行驶的阶段
   *
   * 需要缓慢行驶的阶段：
   * 1. STOP_SIGN_UNPROTECTED_CREEP：停车标志不保护左转缓慢行驶
   * 2. TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_CREEP：交通灯不保护右转缓慢行驶
   * 3. TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_CREEP：交通灯不保护左转缓慢行驶
   *
   * 如果不是这些阶段，返回false（不是缓慢行驶）
   */
  if (stage_type != "STOP_SIGN_UNPROTECTED_CREEP" &&
      stage_type != "TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_CREEP" &&
      stage_type != "TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_CREEP") {
    return false;  /**< 不是缓慢行驶阶段 */
  }

  /**
   * @brief 检查距离条件
   *
   * static constexpr：编译时常量
   * kDistance = 5.0：判断距离阈值（米）
   * fabs：浮点数绝对值
   *
   * 条件：自车前边缘与PNC路口起点的距离小于等于5米
   */
  static constexpr double kDistance = 5.0;
  /**
   * @brief 缓慢行驶距离阈值
   *
   * 使用static constexpr定义编译时常量
   * 优点：编译时确定值，无需运行时计算
   */
  return (fabs(adc_front_edge_s - pnc_junction_start_s) <= kDistance);
  /**
   * @brief 返回是否为缓慢行驶阶段
   */
}

/**
 * @brief 构建保持清空障碍物
 *
 * @param frame 当前规划帧
 * @param reference_line_info 参考线信息
 * @param virtual_obstacle_id 虚拟障碍物ID
 * @param keep_clear_start_s 清空区域起始s坐标
 * @param keep_clear_end_s 清空区域结束s坐标
 * @return bool 是否成功创建障碍物
 *
 * 功能说明：
 * 创建虚拟静态障碍物来表示保持清空区域
 *
 * 算法流程：
 * 1. 获取自车前边缘s坐标
 * 2. 检查自车是否已经在清空区域内
 * 3. 创建虚拟静态障碍物
 * 4. 将障碍物添加到参考线信息
 * 5. 设置ST边界类型为KEEP_CLEAR
 *
 * C++语法说明：
 * - auto*：自动类型推导的原始指针
 * - frame->CreateStaticObstacle：成员函数调用
 */
bool KeepClear::BuildKeepClearObstacle(
    Frame* const frame, ReferenceLineInfo* const reference_line_info,
    const std::string& virtual_obstacle_id, const double keep_clear_start_s,
    const double keep_clear_end_s) {
  CHECK_NOTNULL(frame);
  /**< @brief 规划帧空指针检查 */
  CHECK_NOTNULL(reference_line_info);
  /**< @brief 参考线信息空指针检查 */

  /**
   * @brief 获取自车前边缘s坐标
   */
  const double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();

  /**
   * @brief 检查自车是否已经在清空区域内
   *
   * 条件：adc_front_edge_s - keep_clear_start_s > min_pass_s_distance
   * 含义：自车前边缘已经进入清空区域一段距离
   *
   * 如果自车已经在区域内，跳过创建障碍物
   */
  if (adc_front_edge_s - keep_clear_start_s > config_.min_pass_s_distance()) {
    ADEBUG << "adc inside keep_clear zone[" << virtual_obstacle_id << "] s["
           << keep_clear_start_s << ", " << keep_clear_end_s
           << "] adc_front_edge_s[" << adc_front_edge_s
           << "]. skip this keep clear zone";
    return false;  /**< 自车已在清空区域内，跳过 */
  }

  ADEBUG << "keep clear obstacle: [" << keep_clear_start_s << ", "
         << keep_clear_end_s << "]";

  /**
   * @brief 创建虚拟静态障碍物
   *
   * auto*：自动类型推导，推断为Obstacle*类型
   * frame->CreateStaticObstacle：创建静态障碍物
   * 参数：
   * - reference_line_info：参考线信息
   * - virtual_obstacle_id：虚拟障碍物ID
   * - keep_clear_start_s：清空区域起始s坐标
   * - keep_clear_end_s：清空区域结束s坐标
   *
   * 返回值：Obstacle*指针，如果失败返回nullptr
   */
  auto* obstacle =
      frame->CreateStaticObstacle(reference_line_info, virtual_obstacle_id,
                                  keep_clear_start_s, keep_clear_end_s);
  if (!obstacle) {
    /**
     * @brief 创建失败
     *
     * AERROR：错误日志宏
     */
    AERROR << "Failed to create obstacle [" << virtual_obstacle_id << "]";
    return false;  /**< 创建失败 */
  }

  /**
   * @brief 将障碍物添加到参考线信息
   *
   * reference_line_info->AddObstacle：添加障碍物到参考线
   * 返回值：PathObstacle*指针，如果失败返回nullptr
   */
  auto* path_obstacle = reference_line_info->AddObstacle(obstacle);
  if (!path_obstacle) {
    AERROR << "Failed to create path_obstacle: " << virtual_obstacle_id;
    return false;  /**< 添加失败 */
  }

  /**
   * @brief 设置ST边界类型
   *
   * STBoundary::BoundaryType::KEEP_CLEAR：
   * 表示该障碍物是保持清空类型
   * 用于后续路径规划时识别该区域需要保持清空
   *
   * SetReferenceLineStBoundaryType：
   * 设置障碍物在参考线上的ST边界类型
   */
  path_obstacle->SetReferenceLineStBoundaryType(
      STBoundary::BoundaryType::KEEP_CLEAR);

  return true;  /**< 创建成功 */
}

}  // namespace planning
/**
 * @brief 命名空间结束标记
 */
}  // namespace apollo
/**
 * @brief Apollo命名空间结束标记
 */
