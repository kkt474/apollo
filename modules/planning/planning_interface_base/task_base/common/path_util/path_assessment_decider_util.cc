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
 * @file path_assessment_decider_util.cc
 * @brief 路径评估决策工具实现文件
 *
 * 功能说明：
 * 实现了路径评估(Path Assessment)工具类，用于评估生成路径的有效性和安全性
 * 主要进行路径合理性检查，包括偏离参考线、偏离道路、碰撞检测等
 *
 * 核心概念：
 * - PathData：路径数据，包含Frenet坐标路径和离散路径
 * - ReferenceLineInfo：参考线信息，提供地图和道路信息
 * - Frenet坐标系：沿路径方向(s)和垂直方向(l)的坐标系
 * - 路径有效性：路径是否满足基本要求
 * - 碰撞检测：路径是否与静态障碍物相交
 *
 * 评估项目：
 * 1. 路径是否为空
 * 2. 路径是否严重偏离参考线
 * 3. 路径是否严重偏离道路
 * 4. 路径是否与静态障碍物碰撞
 * 5. 停车点是否在反向相邻车道
 *
 * C++语法说明：
 * - std::string::npos：字符串查找失败的返回值
 * - std::get<N>：获取tuple的第N个元素
 * - std::move：移动语义，避免拷贝
 * - std::numeric_limits：数值类型限制
 * - std::fabs：浮点数绝对值
 **/
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_assessment_decider_util.h"

#include <algorithm>
/**
 * @brief 标准算法库头文件
 *
 * 提供常用算法：
 * - std::max/min：极值函数
 * - std::find：查找算法
 */

#include <utility>
/**
 * @brief 工具类头文件
 *
 * 提供：
 * - std::pair：模板类
 * - std::move：移动语义
 */

#include "modules/common/configs/vehicle_config_helper.h"
/**
 * @brief 车辆配置辅助类头文件
 *
 * VehicleConfigHelper：
 * - 获取车辆参数（尺寸、轮距等）
 * - Instance()：获取单例实例
 * - GetBoundingBox()：获取车辆边界框
 */

#include "modules/planning/planning_interface_base/task_base/common/path_util/path_bounds_decider_util.h"
/**
 * @brief 路径边界决策工具头文件
 *
 * PathBoundsDeciderUtil：
 * - IsWithinPathDeciderScopeObstacle：判断障碍物是否在处理范围内
 */

namespace apollo {
/**
 * @brief Apollo项目主命名空间
 */

namespace planning {
/**
 * @brief 规划模块命名空间
 */

using apollo::common::math::Box2d;
/**
 * @brief 使用Box2d类型（二维边界框）
 */

using apollo::common::math::Polygon2d;
/**
 * @brief 使用Polygon2d类型（二维多边形）
 */

using apollo::common::math::Vec2d;
/**
 * @brief 使用Vec2d类型（二维向量/点）
 */

/**
 * @brief 判断路径是否为有效的常规路径
 *
 * @param reference_line_info 参考线信息
 * @param path_data 路径数据
 * @return bool 路径是否有效
 *
 * 功能说明：
 * 对生成路径进行一系列基本有效性检查
 * 如果任何检查失败，路径将被判定为无效
 *
 * 检查项目：
 * 1. 路径是否为空
 * 2. 路径是否严重偏离参考线
 * 3. 路径是否严重偏离道路
 * 4. [已注释]路径是否与静态障碍物碰撞
 * 5. 停车点是否在反向相邻车道
 *
 * C++语法说明：
 * - const ReferenceLineInfo&：常量引用，避免拷贝
 * - path_data.path_label()：获取路径标签，用于日志标识
 */
bool PathAssessmentDeciderUtil::IsValidRegularPath(
    const ReferenceLineInfo& reference_line_info, const PathData& path_data) {
  /**
   * @brief 基本合理性检查
   */

  if (path_data.Empty()) {
    /**
     * @brief 检查1：路径是否为空
     *
     * path_data.Empty()：判断路径数据是否为空
     * 如果为空，输出错误日志并返回false
     */
    AERROR << path_data.path_label() << ": path data is empty.";
    return false;
    /**
     * @brief 路径为空，无效
     */
  }

  /**
   * @brief 检查2：路径是否严重偏离参考线
   */
  if (IsGreatlyOffReferenceLine(path_data)) {
    /**
     * @brief 调用专用函数检查偏离参考线
     *
     * IsGreatlyOffReferenceLine：
     * 遍历路径点，检查l值是否超过阈值
     */
    AERROR << path_data.path_label() << ": ADC is greatly off reference line.";
    return false;
    /**
     * @brief 严重偏离参考线，无效
     */
  }

  /**
   * @brief 检查3：路径是否严重偏离道路
   */
  if (IsGreatlyOffRoad(reference_line_info, path_data)) {
    /**
     * @brief 调用专用函数检查偏离道路
     *
     * IsGreatlyOffRoad：
     * 获取道路宽度，比较l值是否超出范围
     */
    AERROR << path_data.path_label() << ": ADC is greatly off road.";
    return false;
    /**
     * @brief 严重偏离道路，无效
     */
  }

  /**
   * @brief 检查4：路径是否与静态障碍物碰撞（已注释）
   *
   * 原代码检查路径是否与静态障碍物碰撞
   * 目前被注释掉，可能因为碰撞检测计算量大
   */
  // if (IsCollidingWithStaticObstacles(reference_line_info, path_data)) {
  //   AINFO << path_data.path_label() << ": ADC has collision.";
  //   return false;
  // }

  /**
   * @brief 检查5：停车点是否在反向相邻车道
   */
  if (IsStopOnReverseNeighborLane(reference_line_info, path_data)) {
    /**
     * @brief 检查停车决策是否在反向车道
     *
     * IsStopOnReverseNeighborLane：
     * 对于变道路径，检查停车点是否在反向车道
     * 如果是，路径无效（不应该在反向车道停车）
     */
    AERROR << path_data.path_label() << ": stop at reverse neighbor lane";
    return false;
    /**
     * @brief 停车点在反向车道，无效
     */
  }

  return true;
  /**
   * @brief 所有检查通过，路径有效
   */
}

/**
 * @brief 检查路径是否严重偏离参考线
 *
 * @param path_data 路径数据
 * @return bool 是否严重偏离
 *
 * 功能说明：
 * 遍历路径的所有Frenet点，检查l值（到参考线的垂直距离）是否超过阈值
 * 如果任何一点的l值超过阈值，说明路径严重偏离参考线
 *
 * 算法流程：
 * 1. 定义偏离阈值（20米）
 * 2. 遍历Frenet路径的所有点
 * 3. 检查每点的l值绝对值是否超过阈值
 * 4. 如果超过，返回true
 *
 * C++语法说明：
 * - static constexpr double：编译时常量
 * - std::fabs：浮点数绝对值函数
 * - frenet_path：Frenet坐标路径（std::vector<FrenetFramePoint>）
 */
bool PathAssessmentDeciderUtil::IsGreatlyOffReferenceLine(
    const PathData& path_data) {
  static constexpr double kOffReferenceLineThreshold = 20.0;
  /**
   * @brief 偏离参考线阈值（米）
   *
   * static constexpr：
   * - static：文件内唯一实例
   * - constexpr：编译时可确定值
   * - const：常量，不可修改
   */

  const auto& frenet_path = path_data.frenet_frame_path();
  /**
   * @brief 获取Frenet路径引用
   *
   * frenet_frame_path()：
   * 返回路径的Frenet坐标表示
   * 类型：std::vector<FrenetFramePoint>
   */

  for (const auto& frenet_path_point : frenet_path) {
    /**
     * @brief 遍历所有Frenet路径点
     *
     * 范围for循环（C++11特性）：
     * - const auto&：常量引用，避免拷贝
     * - frenet_path_point：当前遍历的路径点
     */
    if (std::fabs(frenet_path_point.l()) > kOffReferenceLineThreshold) {
      /**
       * @brief 检查l值是否超过阈值
       *
       * std::fabs(frenet_path_point.l())：
       * - std::fabs：浮点数绝对值
       * - frenet_path_point.l()：获取当前点的l值
       *
       * 如果l值（到参考线的垂直距离）超过20米
       */
      AINFO << "Greatly off reference line at s = " << frenet_path_point.s()
            << ", with l = " << frenet_path_point.l();
      /**
       * @brief 输出调试信息
       *
       * AINFO：信息日志宏
       * 输出偏离点的s坐标和l值
       */
      return true;
      /**
       * @brief 返回true：严重偏离参考线
       */
    }
  }
  return false;
  /**
   * @brief 所有点都在阈值内，返回false
   */
}

/**
 * @brief 检查路径是否严重偏离道路
 *
 * @param reference_line_info 参考线信息
 * @param path_data 路径数据
 * @return bool 是否严重偏离
 *
 * 功能说明：
 * 遍历路径的所有Frenet点，检查是否超出道路边界
 * 道路边界由左右宽度决定，l值应在内侧范围内
 *
 * 算法流程：
 * 1. 定义偏离阈值（10米）
 * 2. 获取道路左右宽度
 * 3. 检查l值是否超出范围：l > 左宽度+阈值 或 l < -(右宽度+阈值)
 *
 * C++语法说明：
 * - reference_line.GetRoadWidth(s, &left, &right)：获取指定s处的道路宽度
 * - &：引用参数，用于获取输出值
 */
bool PathAssessmentDeciderUtil::IsGreatlyOffRoad(
    const ReferenceLineInfo& reference_line_info, const PathData& path_data) {
  static constexpr double kOffRoadThreshold = 10.0;
  /**
   * @brief 偏离道路阈值（米）
   */

  const auto& frenet_path = path_data.frenet_frame_path();
  /**
   * @brief 获取Frenet路径引用
   */

  for (const auto& frenet_path_point : frenet_path) {
    /**
     * @brief 遍历所有Frenet路径点
     */
    double road_left_width = 0.0;
    /**< @brief 道路左侧宽度 */
    double road_right_width = 0.0;
    /**< @brief 道路右侧宽度 */

    if (reference_line_info.reference_line().GetRoadWidth(
            frenet_path_point.s(), &road_left_width, &road_right_width)) {
      /**
       * @brief 获取当前s处的道路宽度
       *
       * GetRoadWidth(s, &left, &right)：
       * - 参数s：沿路径的位置
       * - &road_left_width：左侧宽度输出参数
       * - &road_right_width：右侧宽度输出参数
       * - 返回值：是否成功获取宽度
       *
       * 注意：道路宽度是相对于参考线的
       */

      /**
       * @brief 检查是否超出道路范围
       *
       * 条件：
       * - l > 左宽度 + 阈值：在道路左侧越界
       * - l < -(右宽度 + 阈值)：在道路右侧越界
       */
      if (frenet_path_point.l() > road_left_width + kOffRoadThreshold ||
          frenet_path_point.l() < -road_right_width - kOffRoadThreshold) {
        AINFO << "Greatly off-road at s = " << frenet_path_point.s()
              << ", with l = " << frenet_path_point.l();
        /**
         * @brief 输出调试信息
         */
        return true;
        /**
         * @brief 返回true：严重偏离道路
         */
      }
    }
  }
  return false;
  /**
   * @brief 所有点都在道路内，返回false
   */
}

/**
 * @brief 检查路径是否与静态障碍物碰撞
 *
 * @param reference_line_info 参考线信息
 * @param path_data 路径数据
 * @return bool 是否碰撞
 *
 * 功能说明：
 * 对路径上的每个点，检查车辆边界框是否与任何障碍物多边形相交
 * 使用车辆四角的SL边界与障碍物SL边界进行快速粗筛
 *
 * 算法流程：
 * 1. 获取所有相关障碍物
 * 2. 遍历路径的每个离散点
 * 3. 跳过末尾点（处理完毕区域）
 * 4. 计算路径点的SL边界
 * 5. 粗筛：检查与障碍物SL边界是否可能相交
 * 6. 精筛：检查车辆四角是否在障碍物多边形内
 *
 * C++语法说明：
 * - std::vector<const Obstacle*>：障碍物指针向量
 * - indexed_obstacles.Items()：获取所有障碍物
 * - Polygon2d::IsPointIn：判断点是否在多边形内
 * - VehicleConfigHelper::GetBoundingBox：获取车辆边界框
 */
bool PathAssessmentDeciderUtil::IsCollidingWithStaticObstacles(
    const ReferenceLineInfo& reference_line_info, const PathData& path_data) {
  /**
   * @brief 获取所有障碍物并转换为Frenet坐标系多边形
   */

  std::vector<const Obstacle*> obstacles;
  /**< @brief 障碍物指针列表 */

  const auto& indexed_obstacles =
      reference_line_info.path_decision().obstacles();
  /**
   * @brief 获取路径决策中的所有障碍物
   *
   * indexed_obstacles：
   * 索引化障碍物容器，支持按ID快速查找
   */

  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  /**
   * @brief 获取车辆参数
   */
  double front_edge_to_center = vehicle_param.front_edge_to_center();
  /**< @brief 前边缘到车辆中心的距离 */
  double back_edge_to_center = vehicle_param.back_edge_to_center();
  /**< @brief 后边缘到车辆中心的距离 */
  double path_point_lateral_buffer =
      std::max(vehicle_param.width() / 2.0, vehicle_param.length() / 2.0);
  /**
   * @brief 路径点横向缓冲
   *
   * 取车宽和车长一半的最大值
   * 作为车辆横向占用的缓冲
   */

  for (const auto* obstacle : indexed_obstacles.Items()) {
    /**
     * @brief 遍历所有障碍物
     */
    if (!PathBoundsDeciderUtil::IsWithinPathDeciderScopeObstacle(*obstacle)) {
      /**
       * @brief 过滤不相关的障碍物
       *
       * IsWithinPathDeciderScopeObstacle：
       * 判断障碍物是否在路径决策处理范围内
       * 例如：距离太远的障碍物可以忽略
       */
      continue;
      /**
       * @brief 跳过当前障碍物，继续下一个
       */
    }

    /**
     * @brief 忽略太小的障碍物
     */
    const auto& obstacle_sl = obstacle->PerceptionSLBoundary();
    /**
     * @brief 获取障碍物的SL边界
     *
     * PerceptionSLBoundary()：
     * 返回障碍物在SL坐标系下的边界
     */

    if ((obstacle_sl.end_s() - obstacle_sl.start_s()) *
            (obstacle_sl.end_l() - obstacle_sl.start_l()) <
        kMinObstacleArea) {
      /**
       * @brief 检查障碍物面积是否太小
       *
       * 计算：(s范围) * (l范围)
       * 如果面积小于最小阈值，忽略
       */
      continue;
      /**
       * @brief 跳过太小的障碍物
       */
    }
    obstacles.push_back(obstacle);
    /**
     * @brief 添加到有效障碍物列表
     */
  }

  /**
   * @brief 遍历路径的每个点，检查碰撞
   */
  const auto& frenet_path = path_data.frenet_frame_path();
  /**
   * @brief 获取Frenet路径 */

  for (size_t i = 0; i < path_data.discretized_path().size(); ++i) {
    /**
     * @brief 遍历离散路径的每个点
     *
     * size_t：无符号整数类型，用于数组索引
     * ++i：前自增，效率略高于i++
     */

    /**
     * @brief 跳过末尾点
     *
     * 如果剩余路径长度小于：
     * - 额外尾点数量 * 分辨率 + 车辆长度
     * 说明接近规划终点，跳过碰撞检测
     */
    if (path_data.frenet_frame_path().back().s() -
            path_data.frenet_frame_path()[i].s() <
        (FLAGS_num_extra_tail_bound_point + 1) *
                FLAGS_path_bounds_decider_resolution +
            vehicle_param.length()) {
      break;
      /**
       * @brief 跳出循环
       */
    }

    /**
     * @brief 计算路径点的SL边界
     *
     * 需要考虑车辆前后边缘和横向缓冲
     */
    double path_point_start_s = frenet_path[i].s() - back_edge_to_center;
    /**< @brief 路径点后边缘s坐标 */
    double path_point_end_s = frenet_path[i].s() + front_edge_to_center;
    /**< @brief 路径点前边缘s坐标 */
    double path_point_start_l = frenet_path[i].l() - path_point_lateral_buffer;
    /**< @brief 路径点左侧l坐标 */
    double path_point_end_l = frenet_path[i].l() + path_point_lateral_buffer;
    /**< @brief 路径点右侧l坐标 */

    /**
     * @brief 检查附近的障碍物
     */
    for (const auto* obstacle : obstacles) {
      /**
       * @brief 遍历所有有效障碍物
       */
      const auto& obstacle_sl = obstacle->PerceptionSLBoundary();
      /**
       * @brief 获取障碍物SL边界 */

      /**
       * @brief 按s范围过滤
       *
       * 如果障碍物完全在路径点s范围之外，跳过
       * 条件：obstacle_s_start > path_point_end_s 或 obstacle_s_end < path_point_start_s
       */
      if (obstacle_sl.start_s() > path_point_end_s ||
          obstacle_sl.end_s() < path_point_start_s) {
        continue;
      }

      /**
       * @brief 按l范围过滤
       *
       * 如果障碍物完全在路径点l范围之外，跳过
       */
      if (obstacle_sl.start_l() > path_point_end_l ||
          obstacle_sl.end_l() < path_point_start_l) {
        continue;
      }

      /**
       * @brief 精确碰撞检测
       *
       * 将障碍物和车辆都转换为多边形
       * 检查车辆四角是否在障碍物多边形内
       */
      const auto& path_point = path_data.discretized_path()[i];
      /**
       * @brief 获取离散路径点（包含XY坐标） */
      const auto& vehicle_box =
          common::VehicleConfigHelper::Instance()->GetBoundingBox(path_point);
      /**
       * @brief 获取车辆边界框
       *
       * GetBoundingBox(PathPoint)：
       * 根据路径点生成车辆边界框
       */
      const std::vector<Vec2d>& ABCDpoints = vehicle_box.GetAllCorners();
      /**
       * @brief 获取车辆四角的XY坐标
       *
       * GetAllCorners()：
       * 返回车辆边界框的四个角点
       * A、B、C、D通常表示左前、右前、右后、左后
       */
      const common::math::Polygon2d& obstacle_polygon =
          obstacle->PerceptionPolygon();
      /**
       * @brief 获取障碍物多边形
       *
       * PerceptionPolygon()：
       * 返回障碍物的感知多边形
       */

      for (const auto& corner_point : ABCDpoints) {
        /**
         * @brief 检查每个角点
         */
        if (obstacle_polygon.IsPointIn(corner_point)) {
          /**
           * @brief 角点在障碍物多边形内 = 碰撞
           *
           * IsPointIn：
           * 判断点是否在多边形内部
           */
          AERROR << "ADC is colliding with obstacle at path s = "
                 << path_point.s() << ", with obstacle " << obstacle->Id();
          return true;
          /**
           * @brief 发生碰撞，返回true
           */
        }
      }
    }
  }
  return false;
  /**
   * @brief 未发生碰撞，返回false
   */
}

/**
 * @brief 检查停车点是否在反向相邻车道
 *
 * @param reference_line_info 参考线信息
 * @param path_data 路径数据
 * @return bool 是否在反向车道
 *
 * 功能说明：
 * 对于变道路径（包含"left"或"right"标签），检查停车点是否在反向相邻车道
 * 如果是反向车道停车，路径无效（危险）
 *
 * 算法流程：
 * 1. 检查路径标签是否为变道路径
 * 2. 获取所有停车点的SL坐标
 * 3. 找到距离最远的停车点
 * 4. 检查停车点是否在反向车道
 *
 * C++语法说明：
 * - std::string::npos：字符串查找失败返回值
 * - std::string::find：查找子字符串
 */
bool PathAssessmentDeciderUtil::IsStopOnReverseNeighborLane(
    const ReferenceLineInfo& reference_line_info, const PathData& path_data) {
  /**
   * @brief 检查路径标签
   *
   * 变道路径标签包含"left"或"right"
   */
  if (path_data.path_label().find("left") == std::string::npos &&
      path_data.path_label().find("right") == std::string::npos) {
    /**
     * @brief 如果路径标签不包含"left"和"right"
     *
     * std::string::find：
     * - 查找子字符串
     * - 成功：返回位置索引
     * - 失败：返回std::string::npos
     *
     * 如果既不包含"left"也不包含"right"，不是变道路径
     */
    return false;
    /**
     * @brief 非变道路径，无需检查反向车道
     */
  }

  std::vector<common::SLPoint> all_stop_point_sl =
      reference_line_info.GetAllStopDecisionSLPoint();
  /**
   * @brief 获取所有停车决策的SL坐标
   *
   * GetAllStopDecisionSLPoint：
   * 返回参考线上所有停车点的SL坐标列表
   */

  if (all_stop_point_sl.empty()) {
    /**
     * @brief 如果没有停车点
     */
    return false;
    /**
     * @brief 无停车点，无需检查
     */
  }

  double check_s = 0.0;
  /**< @brief 待检查的s坐标 */

  static constexpr double kLookForwardBuffer = 5.0;
  /**
   * @brief 前向查看缓冲（米）
   */
  const double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
  /**
   * @brief 获取自车后边缘s坐标
   */

  for (const auto& stop_point_sl : all_stop_point_sl) {
    /**
     * @brief 遍历所有停车点
     */
    if (stop_point_sl.s() - adc_end_s < kLookForwardBuffer) {
      /**
       * @brief 跳过太近的停车点
       *
       * 如果停车点距离自车小于5米，跳过
       * 重点关注前方较远的停车点
       */
      continue;
    }
    check_s = stop_point_sl.s();
    /**
     * @brief 记录待检查的s坐标
     */
    break;
    /**
     * @brief 找到第一个有效停车点，退出循环
     */
  }

  if (check_s <= 0.0) {
    /**
     * @brief 如果没有有效的停车点s坐标
     */
    return false;
  }

  double lane_left_width = 0.0;
  /**< @brief 车道左侧宽度 */
  double lane_right_width = 0.0;
  /**< @brief 车道右侧宽度 */

  if (!reference_line_info.reference_line().GetLaneWidth(
          check_s, &lane_left_width, &lane_right_width)) {
    /**
     * @brief 获取车道宽度失败
     */
    return false;
  }

  static constexpr double kSDelta = 0.3;
  /**< @brief s坐标匹配容差（米） */
  common::SLPoint path_point_sl;
  /**< @brief 路径点SL坐标 */

  for (const auto& frenet_path_point : path_data.frenet_frame_path()) {
    /**
     * @brief 遍历路径点
     */
    if (std::fabs(frenet_path_point.s() - check_s) < kSDelta) {
      /**
       * @brief 如果路径点的s接近check_s
       */
      path_point_sl.set_s(frenet_path_point.s());
      path_point_sl.set_l(frenet_path_point.l());
      /**
       * @brief 记录该路径点的SL坐标
       */
    }
  }

  ADEBUG << "path_point_sl[" << path_point_sl.s() << ", " << path_point_sl.l()
         << "] lane_left_width[" << lane_left_width << "] lane_right_width["
         << lane_right_width << "]";

  hdmap::Id neighbor_lane_id;
  /**< @brief 相邻车道ID */
  double neighbor_lane_width = 0.0;
  /**< @brief 相邻车道宽度 */

  /**
   * @brief 向左变道的情况
   */
  if (path_data.path_label().find("left") != std::string::npos &&
      path_point_sl.l() > lane_left_width) {
    /**
     * @brief 路径向左且停车点在自车道左侧
     *
     * 判断逻辑：
     * - 路径标签包含"left"
     * - 停车点l值 > 车道左边界（说明在左侧车道）
     */

    if (reference_line_info.GetNeighborLaneInfo(
            ReferenceLineInfo::LaneType::LeftForward, path_point_sl.s(),
            &neighbor_lane_id, &neighbor_lane_width)) {
      /**
       * @brief 获取左侧相邻车道信息
       *
       * GetNeighborLaneInfo(LaneType::LeftForward, s, &id, &width)：
       * - LaneType::LeftForward：左侧前方车道
       * - 返回：车道是否存在及其宽度
       *
       * 如果存在左侧前方车道
       */
      AINFO << "stop path point at LeftForward neighbor lane["
            << neighbor_lane_id.id() << "]";
      return false;
      /**
       * @brief 在正确的前方相邻车道停车，有效
       */
    } else {
      /**
       * @brief 左侧相邻车道不存在
       */
      AINFO << "stop path point at LeftReverse neighbor lane";
      return true;
      /**
       * @brief 在反向车道停车，无效
       */
    }
  }
  /**
   * @brief 向右变道的情况
   */
  else if (path_data.path_label().find("right") != std::string::npos &&
           path_point_sl.l() < -lane_right_width) {
    /**
     * @brief 路径向右且停车点在自车道右侧
     */

    if (reference_line_info.GetNeighborLaneInfo(
            ReferenceLineInfo::LaneType::RightForward, path_point_sl.s(),
            &neighbor_lane_id, &neighbor_lane_width)) {
      /**
       * @brief 获取右侧相邻车道信息
       */
      AINFO << "stop path point at RightForward neighbor lane["
            << neighbor_lane_id.id() << "]";
      return false;
      /**
       * @brief 在正确的前方相邻车道停车，有效
       */
    } else {
      /**
       * @brief 右侧相邻车道不存在
       */
      AINFO << "stop path point at RightReverse neighbor lane";
      return true;
      /**
       * @brief 在反向车道停车，无效
       */
    }
  }
  return false;
  /**
   * @brief 其他情况，有效
   */
}

/**
 * @brief 初始化路径点决策
 *
 * @param path_data 路径数据
 * @param type 路径点类型
 * @param path_point_decision 输出：路径点决策列表
 *
 * 功能说明：
 * 为路径的每个点初始化默认决策
 * 通常用于初始化路径点决策列表
 *
 * 算法流程：
 * 1. 清空输出列表
 * 2. 遍历路径的所有Frenet点
 * 3. 为每点创建默认决策
 *
 * C++语法说明：
 * - CHECK_NOTNULL：空指针检查宏
 * - std::numeric_limits<double>::max()：double最大值作为默认代价
 * - emplace_back：原地构造元素
 */
void PathAssessmentDeciderUtil::InitPathPointDecision(
    const PathData& path_data, const PathData::PathPointType type,
    std::vector<PathPointDecision>* const path_point_decision) {
  /**
   * @brief 合理性检查
   */
  CHECK_NOTNULL(path_point_decision);
  /**
   * @brief 空指针检查
   *
   * CHECK_NOTNULL(ptr)：
   * - 如果ptr为nullptr，程序终止
   * - 用于预防编程错误
   */

  path_point_decision->clear();
  /**
   * @brief 清空决策列表
   */

  /**
   * @brief 遍历路径的每个点，初始化决策
   */
  for (const auto& frenet_path_point : path_data.frenet_frame_path()) {
    /**
     * @brief 为每个Frenet路径点创建默认决策
     *
     * emplace_back参数：
     * - frenet_path_point.s()：路径点s坐标
     * - type：路径点类型
     * - std::numeric_limits<double>::max()：最大代价（表示未确定）
     */
    path_point_decision->emplace_back(frenet_path_point.s(), type,
                                      std::numeric_limits<double>::max());
    /**
     * @brief emplace_back：
     * - 直接在向量末尾构造元素
     * - 避免拷贝或移动
     * - 比push_back更高效
     *
     * std::numeric_limits<double>::max()：
     * - double类型的最大值
     * - 作为初始代价，表示尚未计算最优代价
     */
  }
}

/**
 * @brief 裁剪尾部出车道点
 *
 * @param path_data 输入/输出路径数据
 *
 * 功能说明：
 * 移除路径尾部不在车道内的点
 * 用于清理变道路径的尾部，确保路径结束在有效车道内
 *
 * 算法流程：
 * 1. 检查路径标签，self车道和fallback路径不裁剪
 * 2. 从尾部向前遍历
 * 3. 移除所有不在车道内的点
 * 4. 更新路径数据和决策引导
 *
 * C++语法说明：
 * - std::get<N>：获取tuple的第N个元素
 * - std::move：移动语义，避免数据拷贝
 * - std::string::npos：字符串查找失败值
 * - std::string::find：查找子字符串
 */
void PathAssessmentDeciderUtil::TrimTailingOutLanePoints(
    PathData* const path_data) {
  /**
   * @brief 不裁剪self车道路径或fallback路径
   */
  if (path_data->path_label().find("fallback") != std::string::npos ||
      path_data->path_label().find("self") != std::string::npos) {
    /**
     * @brief 如果路径标签包含"fallback"或"self"
     *
     * std::string::find != std::string::npos：
     * - 如果find返回不是npos，说明找到了
     * - 即路径标签包含该关键字
     */
    return;
    /**
     * @brief 这些特殊路径不裁剪
     */
  }

  /**
   * @brief 执行裁剪
   */
  AINFO << "Trimming " << path_data->path_label();
  /**
   * @brief 输出裁剪日志
   */

  auto frenet_path = path_data->frenet_frame_path();
  /**
   * @brief 获取路径副本
   *
   * 注意：这是拷贝，不是引用
   * 后续会通过pop_back修改
   */
  auto path_point_decision = path_data->path_point_decision_guide();
  /**
   * @brief 获取决策引导副本
   */

  while (!path_point_decision.empty() &&
         std::get<1>(path_point_decision.back()) !=
             PathData::PathPointType::IN_LANE) {
    /**
     * @brief 从尾部向前移除非车道内点
     *
     * 循环条件：
     * - path_point_decision非空
     * - 且尾部点不是IN_LANE类型
     */

    if (std::get<1>(path_point_decision.back()) ==
        PathData::PathPointType::OUT_ON_FORWARD_LANE) {
      /**
       * @brief 尾部点是前方出车道点
       */
      AINFO << "Trimming out forward lane point";
    } else if (std::get<1>(path_point_decision.back()) ==
               PathData::PathPointType::OUT_ON_REVERSE_LANE) {
      /**
       * @brief 尾部点是反向出车道点
       */
      AINFO << "Trimming out reverse lane point";
    } else {
      /**
       * @brief 尾部点是未知类型
       */
      AINFO << "Trimming unknown lane point";
    }

    frenet_path.pop_back();
    /**
     * @brief 移除路径尾部点
     *
     * pop_back：
     * - 删除向量最后一个元素
     * - O(1)时间复杂度
     */
    path_point_decision.pop_back();
    /**
     * @brief 移除决策引导尾部点
     */
  }

  path_data->SetFrenetPath(std::move(frenet_path));
  /**
   * @brief 更新Frenet路径
   *
   * std::move：
   * - 将左值转换为右值引用
   * - 触发移动语义，避免拷贝
   * - 移动后frenet_path变为空
   */

  path_data->SetPathPointDecisionGuide(std::move(path_point_decision));
  /**
   * @brief 更新决策引导
   */

  AINFO << "After TrimTailingOutLanePoints: FrenetPath size: "
        << path_data->frenet_frame_path().size();
  /**
   * @brief 输出裁剪后的路径大小
   */
}

}  // namespace planning
/**
 * @brief 命名空间结束标记
 */
}  // namespace apollo
/**
 * @brief Apollo命名空间结束标记
 */
