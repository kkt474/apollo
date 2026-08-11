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
 * @brief 速度限制决策器实现文件
 *
 * 本文件实现了速度限制决策器，用于根据以下因素计算沿路径的速度限制：
 * 1. 地图给出的速度限制（参考线上的限速标志）
 * 2. 路径曲率导致的速度限制（向心力加速度约束）
 * 3. 附近障碍物（侧向决策为nudge的障碍物）导致的速度限制
 *
 * 相关C++语法说明：
 * - const ReferenceLine&: 常量引用，避免拷贝同时保证数据不被修改
 * - initializer_list: 初始化列表，用于std::min等函数
 * - std::numeric_limits<double>::max(): 获取double类型的最大值作为初始值
 **/

#include "modules/planning/tasks/speed_bounds_decider/speed_limit_decider.h"

/**
 * @brief 标准库头文件
 * <algorithm>: 提供std::min, std::fmax, std::fmin等算法函数
 * <limits>: 提供std::numeric_limits获取类型极值
 */
#include <algorithm>
#include <limits>

/**
 * @brief Apollo消息类型头文件
 * pnc_point.pb.h: 包含PathPoint、TrajectoryPoint等点云数据结构
 * decision.pb.h: 包含障碍物决策相关的protobuf消息定义
 */
#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/common_msgs/planning_msgs/decision.pb.h"

/**
 * @brief Cyber RT日志系统头文件
 * cyber/common/log.h: 提供AWARN、AERROR等日志宏
 */
#include "cyber/common/log.h"

/**
 * @brief 车辆配置助手头文件
 * VehicleConfigHelper::GetConfig(): 获取车辆参数配置
 */
#include "modules/common/configs/vehicle_config_helper.h"

/**
 * @brief 调试信息打印工具头文件
 * PrintCurves: 用于将曲线数据打印到日志
 */
#include "modules/planning/planning_base/common/util/print_debug_info.h"

/**
 * @brief Apollo命名空间开始
 *
 * C++语法说明：
 * namespace关键字用于定义命名空间，避免命名冲突
 * apollo: Apollo项目的根命名空间
 * planning: 规划模块的命名空间
 */
namespace apollo {
/**
 * @brief 规划模块命名空间开始
 */
namespace planning {

/**
 * @brief 使用apollo::common::Status类型
 *
 * C++语法说明：
 * using声明：引入其他命名空间的类型，简化后续代码中的类型引用
 * Status: Apollo通用的状态返回类型，包含错误码和错误信息
 */
using apollo::common::Status;

/**
 * @brief 速度限制决策器构造函数
 *
 * @param config 速度边界决策器配置参数
 * @param reference_line 参考线引用
 * @param path_data 路径数据引用
 *
 * 功能说明：
 * 初始化速度限制决策器，保存配置、参考线和路径数据的引用
 * 同时获取车辆参数配置
 *
 * C++语法说明：
 * - SpeedBoundsDeciderConfig: 从protobuf配置文件中加载的配置结构
 * - const ReferenceLine&: 常量引用，避免拷贝，提高效率
 * - vehicle_param_: 车辆参数结构体，包含前后悬、左右边缘到中心的距离等
 */
SpeedLimitDecider::SpeedLimitDecider(const SpeedBoundsDeciderConfig& config,
                                     const ReferenceLine& reference_line,
                                     const PathData& path_data)
    : speed_bounds_config_(config),                        /**< 初始化列表：直接初始化成员变量 */
      reference_line_(reference_line),
      path_data_(path_data),
      /**
       * @brief 获取车辆配置
       * VehicleConfigHelper::GetConfig(): 单例模式获取车辆配置
       * .vehicle_param(): 获取车辆参数子结构
       */
      vehicle_param_(common::VehicleConfigHelper::GetConfig().vehicle_param()) {
}

/**
 * @brief 获取速度限制主函数
 *
 * @param obstacles 障碍物列表（索引列表）
 * @param speed_limit_data 输出：速度限制数据
 * @return Status 执行状态
 *
 * 功能流程：
 * 1. 遍历路径上的所有点
 * 2. 对每个点计算三部分速度限制：
 *    - 地图给出的速度限制（参考线限速）
 *    - 曲率限制的速度限制（向心力加速度）
 *    - 附近障碍物限制的速度限制
 * 3. 取三者的最小值作为当前点的速度限制
 *
 * C++语法说明：
 * - const IndexedList<std::string, Obstacle>&: 索引列表的常量引用
 *   IndexedList是Apollo自定义的容器，按string键索引Obstacle对象
 * - SpeedLimit* const speed_limit_data: 指向常量的指针（指针本身是常量）
 * - CHECK_NOTNULL: Apollo断言宏，检查指针是否为空
 */
Status SpeedLimitDecider::GetSpeedLimits(
    const IndexedList<std::string, Obstacle>& obstacles,
    SpeedLimit* const speed_limit_data) const {
  /**
   * @brief 断言检查
   * CHECK_NOTNULL(pointer): 确保指针非空，否则程序终止
   * 这里检查输出参数speed_limit_data是否为空
   */
  CHECK_NOTNULL(speed_limit_data);

  /**
   * @brief 获取离散化的路径数据
   * path_data_.discretized_path(): 返回路径点的向量
   * const auto&: 常量引用，避免拷贝
   *
   * C++语法说明：
   * discretized_path()是PathData类的成员函数，返回DiscretizedPath类型
   * DiscretizedPath本质上是std::vector<PathPoint>
   */
  const auto& discretized_path = path_data_.discretized_path();

  /**
   * @brief 获取Frenet坐标系下的路径数据
   * frenet_frame_path(): 返回Frenet坐标系下的路径
   * 用于获取各点的横向偏移l
   */
  const auto& frenet_path = path_data_.frenet_frame_path();

  /**
   * @brief 创建调试曲线打印对象
   * PrintCurves: 用于记录各种曲线数据，方便调试
   */
  PrintCurves print_curve;

  /**
   * @brief 遍历路径上的所有点
   *
   * C++语法说明：
   * - uint32_t: 无符号32位整数类型，用于表示大小和索引
   * - i++: 后置递增运算符，先使用当前值再递增
   * - discretized_path.size(): 返回路径点数量
   * - discretized_path.at(i): 访问指定索引的路径点（带边界检查）
   */
  for (uint32_t i = 0; i < discretized_path.size(); ++i) {
    /**
     * @brief 获取当前路径点的s坐标
     * discretized_path.at(i).s(): 获取PathPoint的累积距离s
     * s坐标表示从路径起点沿参考线到该点的距离
     */
    const double path_s = discretized_path.at(i).s();

    /**
     * @brief 获取Frenet坐标系下对应点的s坐标
     * frenet_path.at(i).s(): Frenet路径点的s坐标
     * 理论上应该与path_s相同，但在数值计算中可能存在微小差异
     */
    const double reference_line_s = frenet_path.at(i).s();

    /**
     * @brief 检查s坐标是否超出参考线范围
     * reference_line_.Length(): 获取参考线的总长度
     * 如果超出，输出警告并停止处理
     */
    if (reference_line_s > reference_line_.Length()) {
      /**
       * @brief 输出警告日志
       * AWARN: Apollo警告级别日志宏
       * 使用<<运算符连接字符串和变量
       */
      AWARN << "path w.r.t. reference line at [" << reference_line_s
            << "] is LARGER than reference_line_ length ["
            << reference_line_.Length() << "]. Please debug before proceeding.";
      break;  /**< 跳出循环，停止处理 */
    }

    /**
     * @brief (1) 获取地图速度限制
     *
     * GetSpeedLimitFromS(reference_line_s):
     *   根据s坐标从参考线获取该点的限速值
     *   参考线上不同位置可能有不同的限速（如弯道、路口等）
     *
     * 调试曲线记录：将限速值按(path_s, speed_limit)记录
     */
    double speed_limit_from_reference_line =
        reference_line_.GetSpeedLimitFromS(reference_line_s);
    print_curve.AddPoint("speed_limit_from_ref", path_s,
                         speed_limit_from_reference_line);

    /**
     * @brief (2) 获取曲率速度限制
     *
     * 原理：根据向心力公式 a = v^2 / r = v^2 * kappa
     * 其中kappa是曲率，a是向心加速度
     * 因此 v = sqrt(a / kappa)
     *
     * max_centric_acceleration_limit: 配置的最大向心加速度限制
     * minimal_kappa: 最小曲率阈值，避免除零
     */
    // -- 2.1: limit by centripetal force (acceleration)
    /**
     * @brief 计算向心加速度限制的速度
     *
     * std::fmax/std::fabs: 浮点数比较和绝对值函数
     * std::sqrt: 平方根函数
     *
     * discretized_path.at(i).kappa(): 获取当前路径点的曲率kappa
     * 使用fmax确保曲率不低于最小阈值
     */
    const double speed_limit_from_centripetal_acc =
        std::sqrt(speed_bounds_config_.max_centric_acceleration_limit() /
                  std::fmax(std::fabs(discretized_path.at(i).kappa()),
                            speed_bounds_config_.minimal_kappa()));
    print_curve.AddPoint("speed_limit_from_centripetal_acc", path_s,
                         speed_limit_from_centripetal_acc);

    /**
     * @brief (3) 获取附近障碍物速度限制
     *
     * 初始值设为double最大值，后面会根据实际情况减小
     *
     * C++语法说明：
     * std::numeric_limits<double>::max(): 获取double类型的最大正值
     * 这是一个很大的数，用作初始值表示"无限"
     */
    double speed_limit_from_nearby_obstacles =
        std::numeric_limits<double>::max();

    /**
     * @brief 安全碰撞距离范围
     * 从配置中获取，用于确定障碍物与自车之间的安全距离
     */
    const double collision_safety_range =
        speed_bounds_config_.collision_safety_range();

    /**
     * @brief 遍历所有障碍物
     *
     * obstacles.Items():
     *   IndexedList的Items()方法返回所有障碍物的指针列表
     *   返回类型是std::vector<const Obstacle*>
     *
     * C++语法说明：
     * for (const auto* ptr_obstacle : obstacles.Items()):
     *   范围for循环遍历向量，ptr_obstacle是指向Obstacle的常量指针
     */
    for (const auto* ptr_obstacle : obstacles.Items()) {
      /**
       * @brief 跳过虚拟障碍物
       *
       * IsVirtual():
       *   判断障碍物是否为虚拟障碍物（如停车墙等人为添加的）
       * 虚拟障碍物不需要考虑侧向nudge决策
       */
      if (ptr_obstacle->IsVirtual()) {
        continue;  /**< 跳过当前障碍物，继续下一个 */
      }

      /**
       * @brief 检查障碍物是否有侧向nudge决策
       *
       * LateralDecision().has_nudge():
       *   检查是否存在nudge（避让）决策
       * 只有明确需要避让的障碍物才考虑其对速度的限制
       *
       * nudge决策表示自车需要侧向移动来避让该障碍物
       */
      if (!ptr_obstacle->LateralDecision().has_nudge()) {
        continue;
      }

      /**
       * @brief 示意图解释位置关系
       *
       * -------------------------------
       *    start_s   end_s
       * ------|  adc   |---------------
       * ------------|  obstacle |------
       *
       * adc: 自车（autonomous driving car）
       * start_s/end_s: 障碍物在s方向上的边界
       * 障碍物可能在自车前方或后方
       */

      /**
       * @brief 计算自车前后边缘的s坐标
       *
       * vehicle_param_.front_edge_to_center():
       *   车辆前端到几何中心的前向距离
       * vehicle_param_.back_edge_to_center():
       *   车辆后端到几何中心的后向距离
       *
       * vehicle_front_s: 自车前边缘的s坐标
       * vehicle_back_s: 自车后边缘的s坐标
       */
      const double vehicle_front_s =
          reference_line_s + vehicle_param_.front_edge_to_center();
      const double vehicle_back_s =
          reference_line_s - vehicle_param_.back_edge_to_center();

      /**
       * @brief 计算障碍物前后边缘的s坐标
       *
       * PerceptionSLBoundary():
       *   获取障碍物在SL坐标系下的边界框
       * start_s: 障碍物在s方向上的起始位置
       * end_s: 障碍物在s方向上的结束位置
       */
      const double obstacle_front_s =
          ptr_obstacle->PerceptionSLBoundary().end_s();
      const double obstacle_back_s =
          ptr_obstacle->PerceptionSLBoundary().start_s();

      /**
       * @brief 检查自车与障碍物在s方向上是否有重叠
       *
       * 如果满足以下条件之一，则没有重叠：
       * - 自车前端在障碍物后端之后（自车完全在障碍物前方）
       * - 自车后端在障碍物前端之前（自车完全在障碍物后方）
       *
       * 只有存在重叠才需要考虑避让
       */
      if (vehicle_front_s < obstacle_back_s ||
          vehicle_back_s > obstacle_front_s) {
        continue;  /**< 没有重叠，跳过该障碍物 */
      }

      /**
       * @brief 获取nudge决策的详细信息
       *
       * LateralDecision().nudge():
       *   获取nudge子消息的引用
       * nudge决策包含避让类型和避让距离
       */
      const auto& nudge_decision = ptr_obstacle->LateralDecision().nudge();

      /**
       * @brief 获取当前路径点在Frenet坐标系下的横向偏移l
       *
       * 请注意adc_l和frenet_point_l之间的区别：
       * - frenet_point_l: 当前路径点在Frenet坐标系下的l值
       * - 自车实际位置可能与当前路径点有偏差
       *
       * l值表示到参考线的横向距离，正值表示左侧，负值表示右侧
       */
      const double frenet_point_l = frenet_path.at(i).l();

      /**
       * @brief 判断障碍物是否在自车左侧且距离较近
       *
       * 条件：
       * 1. nudge决策类型是LEFT_NUDGE（需要向左避让）
       * 2. 当前路径点的l值 - 车辆右边缘到中心的距离 - 安全距离
       *    < 障碍物左边界
       *
       * 这意味着障碍物在车辆的右后方，需要向左变道避让
       *
       * C++语法说明：
       * ObjectNudge::LEFT_NUDGE: 枚举值，表示向左避让
       * right_edge_to_center: 车辆右边缘到几何中心的距离
       */
      bool is_close_on_left =
          (nudge_decision.type() == ObjectNudge::LEFT_NUDGE) &&
          (frenet_point_l - vehicle_param_.right_edge_to_center() -
               collision_safety_range <
           ptr_obstacle->PerceptionSLBoundary().end_l());

      /**
       * @brief 判断障碍物是否在自车右侧且距离较近
       *
       * 条件：
       * 1. nudge决策类型是RIGHT_NUDGE（需要向右避让）
       * 2. 障碍物右边界 + 安全距离 < 当前路径点的l值 + 车辆左边缘到中心的距离
       *
       * 这意味着障碍物在车辆的左后方，需要向右变道避让
       */
      bool is_close_on_right =
          (nudge_decision.type() == ObjectNudge::RIGHT_NUDGE) &&
          (ptr_obstacle->PerceptionSLBoundary().start_l() -
               collision_safety_range <
           frenet_point_l + vehicle_param_.left_edge_to_center());

      /**
       * @brief 如果障碍物在附近且需要避让，则计算减速比例
       *
       * TODO注释说明：动态障碍物目前没有nudge决策
       *
       * 静态障碍物和动态障碍物使用不同的减速比例
       */
      if (is_close_on_left || is_close_on_right) {
        double nudge_speed_ratio = 1.0;  /**< 初始减速比例为1.0（不减速） */

        /**
         * @brief 根据障碍物类型选择减速比例
         *
         * IsStatic(): 判断障碍物是否为静态障碍物
         * 静态障碍物使用静态障碍物减速比例
         * 动态障碍物使用动态障碍物减速比例
         */
        if (ptr_obstacle->IsStatic()) {
          /**
           * @brief 静态障碍物的减速比例
           * 从配置中获取，如0.5表示速度限制为原来的一半
           */
          nudge_speed_ratio =
              speed_bounds_config_.static_obs_nudge_speed_ratio();
        } else {
          /**
           * @brief 动态障碍物的减速比例
           * 动态障碍物需要更保守的减速比例
           */
          nudge_speed_ratio =
              speed_bounds_config_.dynamic_obs_nudge_speed_ratio();
        }

        /**
         * @brief 计算障碍物限制的速度
         * 速度限制 = 参考线限速 × 减速比例
         */
        speed_limit_from_nearby_obstacles =
            nudge_speed_ratio * speed_limit_from_reference_line;
        break;  /**< 找到最近的障碍物后跳出循环 */
      }
    }

    /**
     * @brief 计算当前点的最终速度限制
     *
     * 取以下各项的最小值：
     * 1. 参考线速度限制
     * 2. 向心加速度限制
     * 3. 附近障碍物限制（如有）
     *
     * 然后与最低速度进行比较，取较大值确保车辆不会停止
     */
    double curr_speed_limit = 0.0;

    /**
     * @brief 检查是否启用nudge减速功能
     * FLAGS_enable_nudge_slowdown: 配置开关
     */
    if (speed_bounds_config_.enable_nudge_slowdown()) {
      /**
       * @brief 启用nudge减速时的速度限制
       *
       * std::fmax: 取两个浮点数的较大值
       * std::min: 取多个值中的最小值（C++11初始化列表形式）
       *
       * 确保速度不低于最低速度，同时不超过各项限制
       */
      curr_speed_limit =
          std::fmax(speed_bounds_config_.lowest_speed(),
                    std::min({speed_limit_from_reference_line,      /**< 地图限速 */
                              speed_limit_from_centripetal_acc,     /**< 曲率限速 */
                              speed_limit_from_nearby_obstacles})); /**< 障碍物限速 */
    } else {
      /**
       * @brief 禁用nudge减速时的速度限制
       * 只考虑地图限速和曲率限速
       */
      curr_speed_limit =
          std::fmax(speed_bounds_config_.lowest_speed(),
                    std::min({speed_limit_from_reference_line,
                              speed_limit_from_centripetal_acc}));
    }

    /**
     * @brief 如果存在障碍物限制，记录调试曲线
     *
     * 判断条件：障碍物限制速度小于double最大值
     * 这意味着确实找到了需要避让的障碍物
     */
    if (speed_limit_from_nearby_obstacles <
        std::numeric_limits<double>::max()) {
      print_curve.AddPoint("speed_limit_from_nearby_obstacles", path_s,
                           speed_limit_from_nearby_obstacles);
    }

    /**
     * @brief 将速度限制添加到结果中
     *
     * AppendSpeedLimit(path_s, curr_speed_limit):
     *   在指定的s位置添加速度限制
     *   SpeedLimit内部会按s坐标排序存储
     */
    speed_limit_data->AppendSpeedLimit(path_s, curr_speed_limit);

    /**
     * @brief 记录当前速度限制到调试曲线
     */
    print_curve.AddPoint("curr_speed_limit", path_s, curr_speed_limit);
  }

  /**
   * @brief 打印所有调试曲线到日志
   * PrintToLog(): 将记录的曲线数据以日志形式输出
   * 用于规划结果的可视化和调试
   */
  print_curve.PrintToLog();

  return Status::OK();  /**< 返回成功状态 */
}

}  // namespace planning
}  // namespace apollo
