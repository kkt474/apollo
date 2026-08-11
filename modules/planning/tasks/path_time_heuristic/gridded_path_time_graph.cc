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
 * @file gridded_path_time_graph.cc
 * @brief 网格化路径时间图实现文件
 *
 * 本文件实现了网格化路径时间图(GriddedPathTimeGraph)的搜索算法
 * 用于在ST图上进行基于动态规划的速度优化
 *
 * 功能说明：
 * 1. 构建ST图的网格结构
 * 2. 初始化代价表
 * 3. 使用动态规划计算最优速度曲线
 * 4. 检索最优速度剖面
 *
 * 算法原理（动态规划）：
 * - ST图网格化：将(路径S, 时间T)空间划分为网格
 * - 节点代价：每个网格点有obstacle_cost和spatial_potential_cost
 * - 边代价：相邻节点之间有speed_cost, accel_cost, jerk_cost
 * - 最优解：通过DP从起点到终点找到代价最小的路径
 *
 * 网格设计：
 * - 时间维度T：均匀网格（unit_t_）
 * - 空间维度S：混合网格（dense_unit_s_近距离 + sparse_unit_s_远距离）
 *
 * 相关C++语法说明：
 * - std::vector<std::vector<T>>: 二维向量，用于存储代价表
 * - std::future/std::async: 异步编程，用于并行计算
 * - lambda表达式: 用于回调函数
 **/
/**
 * @brief 网格化路径时间图头文件
 *
 * 包含GriddedPathTimeGraph类的完整定义
 */
#include "modules/planning/tasks/path_time_heuristic/gridded_path_time_graph.h"

/**
 * @brief 标准库头文件
 * <algorithm>: 提供std::min, std::max, std::ceil, std::lower_bound等算法
 * <limits>: 提供std::numeric_limits获取类型极值
 * <string>: 提供std::string字符串类型
 */
#include <algorithm>
#include <limits>
#include <string>

/**
 * @brief PNC点protobuf消息头文件
 * 包含PathPoint、SpeedPoint、TrajectoryPoint等数据结构
 */
#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"

/**
 * @brief Cyber RT头文件
 * cyber/common/log.h: 日志系统
 * cyber/task/task.h: 异步任务支持
 */
#include "cyber/common/log.h"
#include "cyber/task/task.h"

/**
 * @brief Apollo数学工具头文件
 * vec2d: 二维向量运算
 * point_factory: 点工厂，用于创建各种点
 */
#include "modules/common/math/vec2d.h"
#include "modules/common/util/point_factory.h"

/**
 * @brief 调试信息打印工具
 * PrintCurves/PrintPoints: 用于记录曲线数据
 */
#include "modules/planning/planning_base/common/util/print_debug_info.h"

/**
 * @brief 规划模块GFlags
 * FLAGS_*: 从配置文件获取的参数
 */
#include "modules/planning/planning_base/gflags/planning_gflags.h"

/**
 * @brief Apollo命名空间开始
 */
namespace apollo {

/**
 * @brief 规划模块命名空间
 */
namespace planning {

/**
 * @brief 使用别名声明
 *
 * C++语法说明：
 * using声明：简化类型引用
 */
using apollo::common::ErrorCode;
using apollo::common::SpeedPoint;
using apollo::common::Status;
using apollo::common::util::PointFactory;

/**
 * @brief 匿名命名空间
 *
 * C++语法说明：
 * namespace {}: 匿名命名空间，内部定义的符号仅当前文件可见
 * 类似于static关键字，但更符合C++风格
 */
namespace {

/**
 * @brief 双重精度极小值
 *
 * 用于浮点数比较，避免精度问题
 * 1.0e-6是一个足够小的正数
 */
static constexpr double kDoubleEpsilon = 1.0e-6;

/**
 * @brief 检查ST图上两点连线是否与障碍物重叠
 *
 * @param boundaries ST边界列表
 * @param p1 起点ST点
 * @param p2 终点ST点
 * @return bool 是否重叠（碰撞）
 *
 * 功能说明：
 * 使用线性插值作为闭环动力学模型
 * 检查线段(p1, p2)是否与任何ST边界多边形重叠
 *
 * 算法流程：
 * 1. 如果启用可行驶边界，直接返回false（不检查）
 * 2. 遍历所有ST边界
 * 3. 跳过KEEP_CLEAR类型边界（斑马线等）
 * 4. 检查线段与多边形是否有重叠
 * 5. 有重叠返回true，否则返回false
 *
 * C++语法说明：
 * - const std::vector<const STBoundary*>&: 常量引用，指向常量的指针向量
 * - const StGraphPoint&: 常量引用
 * - boundary->boundary_type(): ->运算符，通过指针访问成员
 */
bool CheckOverlapOnDpStGraph(const std::vector<const STBoundary*>& boundaries,
                             const StGraphPoint& p1, const StGraphPoint& p2) {
  /**
   * @brief 如果启用可行驶边界，跳过碰撞检查
   * FLAGS_use_st_drivable_boundary: 配置开关
   */
  if (FLAGS_use_st_drivable_boundary) {
    return false;  /**< 不检查碰撞 */
  }

  /**
   * @brief 遍历所有ST边界
   */
  for (const auto* boundary : boundaries) {
    /**
     * @brief 跳过KEEP_CLEAR类型边界
     *
     * KEEP_CLEAR: 禁行区，如斑马线、路口等
     * 这些区域不需要碰撞检查
     */
    if (boundary->boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
      continue;  /**< 跳过，继续下一个边界 */
    }

    /**
     * @brief 检查线段与多边形是否重叠
     *
     * HasOverlap({p1.point(), p2.point()}):
     *   创建一个线段对象，检查是否与ST边界多边形重叠
     *   point()返回STPoint，即(s, t)坐标点
     */
    if (boundary->HasOverlap({p1.point(), p2.point()})) {
      return true;  /**< 发生重叠/碰撞 */
    }
  }

  return false;  /**< 没有碰撞 */
}

}  // namespace

/**
 * @brief GriddedPathTimeGraph构造函数
 *
 * @param st_graph_data ST图数据
 * @param dp_config DP速度优化器配置
 * @param obstacles 障碍物列表
 * @param init_point 规划起点
 *
 * 功能说明：
 * 初始化网格化ST图，构建搜索空间
 *
 * 初始化列表说明：
 * : st_graph_data_(st_graph_data), 表示将参数st_graph_data赋值给成员st_graph_data_
 * 这种方式比在函数体内赋值更高效
 *
 * C++语法说明：
 * - StGraphData: ST图数据结构，包含边界、速度限制等
 * - DpStSpeedOptimizerConfig: DP ST优化器配置protobuf消息
 * - std::vector<const Obstacle*>: 障碍物指针向量
 */
GriddedPathTimeGraph::GriddedPathTimeGraph(
    const StGraphData& st_graph_data, const DpStSpeedOptimizerConfig& dp_config,
    const std::vector<const Obstacle*>& obstacles,
    const common::TrajectoryPoint& init_point)
    : st_graph_data_(st_graph_data),                              /**< 初始化ST图数据 */
      gridded_path_time_graph_config_(dp_config),                 /**< 初始化配置 */
      obstacles_(obstacles),                                      /**< 初始化障碍物列表 */
      init_point_(init_point),                                   /**< 初始化规划起点 */
      dp_st_cost_(dp_config, st_graph_data_.total_time_by_conf(),/**< 初始化DP ST代价计算器 */
                  st_graph_data_.path_length(), obstacles,
                  st_graph_data_.st_drivable_boundary(), init_point_) {
  /**
   * @brief 保存时间范围
   * total_time_by_conf(): 从配置获取的总规划时间
   */
  total_length_t_ = st_graph_data_.total_time_by_conf();

  /**
   * @brief 时间网格单元
   * unit_t_: 时间方向的网格分辨率
   */
  unit_t_ = gridded_path_time_graph_config_.unit_t();

  /**
   * @brief 保存路径长度
   */
  total_length_s_ = st_graph_data_.path_length();

  /**
   * @brief 密集空间网格单元（近距离）
   * dense_unit_s_: 近距离使用的高分辨率
   */
  dense_unit_s_ = gridded_path_time_graph_config_.dense_unit_s();

  /**
   * @brief 稀疏空间网格单元（远距离）
   * sparse_unit_s_: 远距离使用的低分辨率
   */
  sparse_unit_s_ = gridded_path_time_graph_config_.sparse_unit_s();

  /**
   * @brief 密集网格数量
   * dense_dimension_s_: 在多远距离内使用密集网格
   */
  dense_dimension_s_ = gridded_path_time_graph_config_.dense_dimension_s();

  /**
   * @brief 计算最大加速度（安全方法）
   *
   * 取车辆参数和配置参数中的较小值
   * 防止不可达的加减速导致规划失败
   *
   * C++语法说明：
   * - std::min/std::abs: 标准库算法
   * - vehicle_param_.max_acceleration(): 车辆最大加速度
   * - gridded_path_time_graph_config_.max_acceleration(): 配置的最大加速度
   */
  max_acceleration_ =
      std::min(std::abs(vehicle_param_.max_acceleration()),
               std::abs(gridded_path_time_graph_config_.max_acceleration()));

  /**
   * @brief 计算最大减速度（负值）
   *
   * 减速度取绝对值后加负号
   * 确保max_deceleration_是负值（减速为负方向）
   */
  max_deceleration_ =
      -1.0 *
      std::min(std::abs(vehicle_param_.max_deceleration()),
               std::abs(gridded_path_time_graph_config_.max_deceleration()));
}

/**
 * @brief 搜索主函数
 *
 * @param speed_data 输出：速度数据
 * @return Status 搜索状态
 *
 * 功能说明：
 * 网格化ST图上的动态规划搜索入口
 * 协调各子步骤完成最优速度剖面的搜索
 *
 * 算法流程：
 * 1. 检查初始点是否与障碍物碰撞
 * 2. 初始化代价表
 * 3. 初始化速度限制查询表
 * 4. 计算总代价
 * 5. 检索最优速度剖面
 *
 * C++语法说明：
 * - SpeedData* const: 指向常量的指针，指针本身是常量（不能改变指向）
 */
Status GriddedPathTimeGraph::Search(SpeedData* const speed_data) {
  /**
   * @brief 边界容差值
   * 用于浮点数比较，避免精度问题
   */
  static constexpr double kBounadryEpsilon = 1e-2;

  /**
   * @brief 遍历所有ST边界
   *
   * st_graph_data_.st_boundaries():
   *   获取所有障碍物的ST边界
   */
  for (const auto& boundary : st_graph_data_.st_boundaries()) {
    /**
     * @brief 跳过KEEP_CLEAR类型边界
     */
    if (boundary->boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
      continue;  /**< 不考虑KEEP_CLEAR障碍物的决策 */
    }

    /**
     * @brief 检查初始点是否与障碍物碰撞
     *
     * 两种情况判定为碰撞：
     * 1. 初始点(0,0)在障碍物边界内
     * 2. 初始点非常接近边界（考虑容差）
     */
    if (boundary->IsPointInBoundary({0.0, 0.0}) ||
        (std::fabs(boundary->min_t()) < kBounadryEpsilon &&
         std::fabs(boundary->min_s()) < kBounadryEpsilon)) {
      /**
       * @brief 发生碰撞，启动速度回退
       *
       * 计算时间维度大小
       * std::ceil: 向上取整
       * static_cast<uint32_t>: 显式类型转换
       */
      dimension_t_ = static_cast<uint32_t>(std::ceil(
                         total_length_t_ / static_cast<double>(unit_t_))) +
                     1;

      /**
       * @brief 生成匀速通过的速度剖面
       * 速度设为0，表示缓慢通过
       */
      std::vector<SpeedPoint> speed_profile;
      double t = 0.0;
      for (uint32_t i = 0; i < dimension_t_; ++i, t += unit_t_) {
        speed_profile.push_back(PointFactory::ToSpeedPoint(0, t));
      }
      *speed_data = SpeedData(speed_profile);  /**< 返回匀速剖面 */
      return Status::OK();  /**< 视为成功 */
    }
  }

  /**
   * @brief 初始化代价表
   *
   * 代价表是动态规划的核心数据结构
   * cost_table_[t][s] = ST图上(t,s)位置的最优代价
   */
  if (!InitCostTable().ok()) {
    const std::string msg = "Initialize cost table failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  /**
   * @brief 初始化速度限制查询表
   *
   * speed_limit_by_index_: 根据s坐标索引查询速度限制
   * 预计算可以加速后续的查询
   */
  if (!InitSpeedLimitLookUp().ok()) {
    const std::string msg = "Initialize speed limit lookup table failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  /**
   * @brief 计算总代价
   *
   * 动态规划的核心：从起点向终点递推
   * 每列（时间点）计算到各行的最小代价
   */
  if (!CalculateTotalCost().ok()) {
    const std::string msg = "Calculate total cost failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  /**
   * @brief 检索最优速度剖面
   *
   * 从代价表回溯，找到最优路径
   * 将路径转换为速度剖面
   */
  if (!RetrieveSpeedProfile(speed_data).ok()) {
    const std::string msg = "Retrieve best speed profile failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  return Status::OK();  /**< 全部成功 */
}

/**
 * @brief 初始化代价表
 *
 * @return Status 初始化状态
 *
 * 功能说明：
 * 构建ST图的网格结构
 * 为每个网格点创建StGraphPoint对象
 *
 * 网格设计：
 * - 时间维度T：均匀网格，大小为unit_t_
 * - 空间维度S：混合分辨率
 *   - 密集区域[0, dense_dimension_s_*dense_unit_s_)：使用dense_unit_s_
 *   - 稀疏区域：使用sparse_unit_s_
 *
 * C++语法说明：
 * - std::vector<std::vector<StGraphPoint>>: 二维向量
 *   外层是时间维度，内层是空间维度
 * - std::ceil: 向上取整函数
 */
Status GriddedPathTimeGraph::InitCostTable() {
  /**
   * @brief 时间维度均匀，空间维度双重分辨率
   *
   * 注释说明：
   * 时间维度使用均匀采样
   * 空间维度使用双重分辨率：近距离密集，远距离稀疏
   * 这样可以在保证精度的同时减少计算量
   */

  /**
   * @brief 数值稳定性检查
   * 检查unit_t_是否过小
   */
  if (unit_t_ < kDoubleEpsilon) {
    const std::string msg = "unit_t is smaller than the kDoubleEpsilon.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  /**
   * @brief 检查空间维度设置
   * dense_dimension_s_至少为1
   */
  if (dense_dimension_s_ < 1) {
    const std::string msg = "dense_dimension_s is at least 1.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  /**
   * @brief 计算时间维度大小
   *
   * std::ceil(total_length_t_ / unit_t_):
   *   向上取整，确保覆盖整个时间范围
   * +1: 加上起点
   */
  dimension_t_ = static_cast<uint32_t>(std::ceil(
                     total_length_t_ / static_cast<double>(unit_t_))) +
                 1;

  /**
   * @brief 计算稀疏区域长度
   *
   * sparse_length_s = 总长度 - 密集区域长度
   * 密集区域长度 = (dense_dimension_s_ - 1) * dense_unit_s_
   */
  double sparse_length_s =
      total_length_s_ -
      static_cast<double>(dense_dimension_s_ - 1) * dense_unit_s_;

  /**
   * @brief 计算稀疏维度
   * 如果稀疏区域长度大于0，则计算网格数
   */
  sparse_dimension_s_ =
      sparse_length_s > std::numeric_limits<double>::epsilon()
          ? static_cast<uint32_t>(std::ceil(sparse_length_s / sparse_unit_s_))
          : 0;

  /**
   * @brief 修正密集维度
   * 如果稀疏区域长度为0或负，则密集区域覆盖全部
   */
  dense_dimension_s_ =
      sparse_length_s > std::numeric_limits<double>::epsilon()
          ? dense_dimension_s_
          : static_cast<uint32_t>(std::ceil(total_length_s_ / dense_unit_s_)) +
                1;

  /**
   * @brief 计算总空间维度
   */
  dimension_s_ = dense_dimension_s_ + sparse_dimension_s_;

  /**
   * @brief 创建调试曲线记录器
   */
  PrintCurves debug;

  /**
   * @brief 最终健全性检查
   * 确保维度有效
   */
  if (dimension_t_ < 1 || dimension_s_ < 1) {
    const std::string msg = "Dp st cost table size incorrect.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  /**
   * @brief 创建代价表
   *
   * 二维向量：cost_table_[t][s]
   * 外层大小 = dimension_t_（时间维度）
   * 内层大小 = dimension_s_（空间维度）
   * 每个元素是StGraphPoint对象
   *
   * C++语法说明：
   * std::vector<std::vector<StGraphPoint>>(
   *     dimension_t_, std::vector<StGraphPoint>(dimension_s_, StGraphPoint()))
   *   - 外层构造dimension_t_个元素
   *   - 每个元素是dimension_s_个StGraphPoint的向量
   */
  cost_table_ = std::vector<std::vector<StGraphPoint>>(
      dimension_t_, std::vector<StGraphPoint>(dimension_s_, StGraphPoint()));

  /**
   * @brief 初始化每个网格点
   *
   * 时间循环
   */
  double curr_t = 0.0;
  for (uint32_t i = 0; i < cost_table_.size(); ++i, curr_t += unit_t_) {
    auto& cost_table_i = cost_table_[i];  /**< 引用，避免重复索引计算 */
    double curr_s = 0.0;

    /**
     * @brief 密集空间维度初始化
     * 使用密集分辨率
     */
    for (uint32_t j = 0; j < dense_dimension_s_; ++j, curr_s += dense_unit_s_) {
      cost_table_i[j].Init(i, j, STPoint(curr_s, curr_t));
      debug.AddPoint("dp_node_points", curr_t, curr_s);  /**< 记录调试点 */
    }

    /**
     * @brief 稀疏空间维度初始化
     * 从密集区域末端开始，使用稀疏分辨率
     */
    curr_s = static_cast<double>(dense_dimension_s_ - 1) * dense_unit_s_ +
             sparse_unit_s_;
    for (uint32_t j = dense_dimension_s_; j < cost_table_i.size();
         ++j, curr_s += sparse_unit_s_) {
      cost_table_i[j].Init(i, j, STPoint(curr_s, curr_t));
      debug.AddPoint("dp_node_points", curr_t, curr_s);
    }
  }

  /**
   * @brief 创建空间距离查询表
   *
   * spatial_distance_by_index_: 根据索引快速查询s坐标
   * 用于二分查找优化
   */
  const auto& cost_table_0 = cost_table_[0];
  spatial_distance_by_index_ = std::vector<double>(cost_table_0.size(), 0.0);
  for (uint32_t i = 0; i < cost_table_0.size(); ++i) {
    spatial_distance_by_index_[i] = cost_table_0[i].point().s();
  }

  return Status::OK();  /**< 初始化成功 */
}

/**
 * @brief 初始化速度限制查询表
 *
 * @return Status 初始化状态
 *
 * 功能说明：
 * 预计算每个空间网格点的速度限制
 * 加速后续DP搜索中的速度限制查询
 *
 * 算法流程：
 * 1. 清空现有查询表
 * 2. 调整大小为dimension_s_
 * 3. 对每个空间点，从速度限制曲线查询对应的速度限制值
 */
Status GriddedPathTimeGraph::InitSpeedLimitLookUp() {
  /**
   * @brief 清空查询表
   */
  speed_limit_by_index_.clear();

  /**
   * @brief 调整大小
   */
  speed_limit_by_index_.resize(dimension_s_);

  /**
   * @brief 获取速度限制曲线
   * st_graph_data_.speed_limit(): 获取沿路径的速度限制曲线
   */
  const auto& speed_limit = st_graph_data_.speed_limit();

  /**
   * @brief 对每个空间点查询速度限制
   *
   * GetSpeedLimitByS(s):
   *   根据s坐标从速度限制曲线获取该点的限速值
   * cost_table_[0][i].point().s():
   *   获取第i个空间网格点的s坐标
   */
  for (uint32_t i = 0; i < dimension_s_; ++i) {
    speed_limit_by_index_[i] =
        speed_limit.GetSpeedLimitByS(cost_table_[0][i].point().s());
  }

  return Status::OK();
}

/**
 * @brief 计算总代价（动态规划主循环）
 *
 * @return Status 计算状态
 *
 * 功能说明：
 * 动态规划的核心算法
 * 从时间列0向最终列递推
 * 计算每个网格点的最小累积代价
 *
 * 算法流程：
 * 1. 初始化：第0列只有起点代价为0
 * 2. 对每个时间列c：
 *    a. 确定当前可达的行范围[next_lowest_row, next_highest_row]
 *    b. 并行计算该范围内所有点的代价
 *    c. 根据计算结果更新下一列的可达范围
 *
 * C++语法说明：
 * - std::future<void>: 异步任务返回对象
 * - cyber::Async: Cyber RT的异步调用函数
 * - std::make_shared: 创建shared_ptr
 */
Status GriddedPathTimeGraph::CalculateTotalCost() {
  /**
   * @brief col和row对应ST图
   * t对应col（时间）
   * s对应row（空间）
   */

  /**
   * @brief 下一列的可达行范围
   * 初始化为0
   */
  size_t next_highest_row = 0;
  size_t next_lowest_row = 0;

  /**
   * @brief 时间列循环
   * 动态规划从第0列向最后一列递推
   */
  for (size_t c = 0; c < cost_table_.size(); ++c) {
    size_t highest_row = 0;  /**< 当前列可达的最高行 */
    size_t lowest_row = cost_table_.back().size() - 1;  /**< 当前列可达的最低行 */

    /**
     * @brief 计算当前可达范围内的所有点
     *
     * count = 可达行数
     */
    int count = static_cast<int>(next_highest_row) -
                static_cast<int>(next_lowest_row) + 1;

    if (count > 0) {
      std::vector<std::future<void>> results;  /**< 异步任务结果列表 */

      /**
       * @brief 并行或串行计算可达点的代价
       */
      for (size_t r = next_lowest_row; r <= next_highest_row; ++r) {
        auto msg = std::make_shared<StGraphMessage>(c, r);

        /**
         * @brief 根据配置决定并行或串行
         */
        if (gridded_path_time_graph_config_
                .enable_multi_thread_in_dp_st_graph()) {
          /**
           * @brief 并行计算
           * cyber::Async: 异步执行函数
           * &GriddedPathTimeGraph::CalculateCostAt: 成员函数指针
           * this: 对象指针
           * msg: 参数
           */
          results.push_back(
              cyber::Async(&GriddedPathTimeGraph::CalculateCostAt, this, msg));
        } else {
          /**
           * @brief 串行计算
           */
          CalculateCostAt(msg);
        }
      }

      /**
       * @brief 等待所有并行任务完成
       */
      if (gridded_path_time_graph_config_
              .enable_multi_thread_in_dp_st_graph()) {
        for (auto& result : results) {
          result.get();  /**< 获取任务结果，阻塞直到完成 */
        }
      }
    }

    /**
     * @brief 更新可达行范围
     * 基于当前列的计算结果确定下一列的可达范围
     */
    for (size_t r = next_lowest_row; r <= next_highest_row; ++r) {
      const auto& cost_cr = cost_table_[c][r];
      if (cost_cr.total_cost() < std::numeric_limits<double>::infinity()) {
        size_t h_r = 0;
        size_t l_r = 0;
        GetRowRange(cost_cr, &h_r, &l_r);
        highest_row = std::max(highest_row, h_r);
        lowest_row = std::min(lowest_row, l_r);
      }
    }
    next_highest_row = highest_row;
    next_lowest_row = lowest_row;

    /**
     * @brief 调试输出第1列的代价
     */
    if (c == 1) {
      for (size_t r = 0; r < dimension_s_; ++r) {
        BINFO << "cost_table_[1][" << r << "] total_cost = "
              << cost_table_[1][r].total_cost();
      }
    }
  }

  return Status::OK();
}

/**
 * @brief 获取某点的可达行范围
 *
 * @param point ST图上的点
 * @param next_highest_row 输出：可达的最高行
 * @param next_lowest_row 输出：可达的最低行
 *
 * 功能说明：
 * 基于当前点的速度和加速度限制
 * 确定在unit_t_时间内可达的s坐标范围
 *
 * 物理模型：
 * s = v0 * t + 0.5 * a * t^2
 * 根据最大加速度/减速度限制计算上下界
 *
 * C++语法说明：
 * - size_t*: 输出参数指针
 * - std::lower_bound: 二分查找第一个>=value的位置
 * - std::distance: 计算两个迭代器之间的距离
 */
void GriddedPathTimeGraph::GetRowRange(const StGraphPoint& point,
                                       size_t* next_highest_row,
                                       size_t* next_lowest_row) {
  /**
   * @brief 初始速度
   */
  double v0 = 0.0;

  /**
   * @brief 加速度缩放系数
   *
   * TODO注释：由于缺乏准确的速度信息，使用缩放系数
   * 默认值1.0，使用过去1秒的平均速度作为近似
   */
  double acc_coeff = 0.5;

  /**
   * @brief 获取当前速度
   *
   * 如果没有前驱点（起点），使用初始点的速度
   * 否则使用最优速度
   */
  if (!point.pre_point()) {
    v0 = init_point_.v();
  } else {
    v0 = point.GetOptimalSpeed();
  }

  /**
   * @brief 空间维度大小（最大索引）
   */
  const auto max_s_size = dimension_s_ - 1;

  /**
   * @brief 时间单元的平方
   */
  const double t_squared = unit_t_ * unit_t_;

  /**
   * @brief 计算S上界
   *
   * s_upper = v0 * t + 0.5 * max_acceleration * t^2 + current_s
   * 使用最大加速度约束
   */
  const double s_upper_bound = v0 * unit_t_ +
                               acc_coeff * max_acceleration_ * t_squared +
                               point.point().s();

  /**
   * @brief 二分查找确定上界索引
   *
   * std::lower_bound:
   *   在sorted范围内查找第一个>=value的元素
   *   返回迭代器
   */
  const auto next_highest_itr =
      std::lower_bound(spatial_distance_by_index_.begin(),
                       spatial_distance_by_index_.end(), s_upper_bound);

  /**
   * @brief 转换索引
   */
  if (next_highest_itr == spatial_distance_by_index_.end()) {
    *next_highest_row = max_s_size;  /**< 超出范围，取最大值 */
  } else {
    *next_highest_row =
        std::distance(spatial_distance_by_index_.begin(), next_highest_itr);
  }

  /**
   * @brief 计算S下界
   *
   * s_lower = max(0, v0 * t + 0.5 * max_deceleration * t^2) + current_s
   * max_deceleration是负值，所以加法实际是减法
   * 确保下界不小于0
   */
  const double s_lower_bound =
      std::fmax(0.0, v0 * unit_t_ + acc_coeff * max_deceleration_ * t_squared) +
      point.point().s();

  /**
   * @brief 二分查找确定下界索引
   */
  const auto next_lowest_itr =
      std::lower_bound(spatial_distance_by_index_.begin(),
                       spatial_distance_by_index_.end(), s_lower_bound);

  if (next_lowest_itr == spatial_distance_by_index_.end()) {
    *next_lowest_row = max_s_size;
  } else {
    *next_lowest_row =
        std::distance(spatial_distance_by_index_.begin(), next_lowest_itr);
  }
}

/**
 * @brief 计算指定点的代价
 *
 * @param msg 包含列c和行r的消息对象
 *
 * 功能说明：
 * 计算ST图上点(c,r)的总代价
 * 包括：obstacle_cost + spatial_potential_cost + edge_cost
 *
 * 算法流程：
 * 1. 计算障碍物代价
 * 2. 如果代价无穷大，直接返回
 * 3. 计算空间势能代价
 * 4. 计算边代价（从上一列到当前点）
 * 5. 更新最优前驱和最优速度
 *
 * C++语法说明：
 * - std::shared_ptr<StGraphMessage>: 消息的智能指针
 * - const auto&: 常量引用
 */
void GriddedPathTimeGraph::CalculateCostAt(
    const std::shared_ptr<StGraphMessage>& msg) {
  /**
   * @brief 从消息中获取坐标
   */
  const uint32_t c = msg->c;
  const uint32_t r = msg->r;

  /**
   * @brief 获取当前点的代价对象引用
   */
  auto& cost_cr = cost_table_[c][r];

  /**
   * @brief 计算障碍物代价
   * dp_st_cost_.GetObstacleCost(): 计算到障碍物的代价
   */
  cost_cr.SetObstacleCost(dp_st_cost_.GetObstacleCost(cost_cr));

  /**
   * @brief 如果障碍物代价无穷大，该点不可达
   */
  if (cost_cr.obstacle_cost() > std::numeric_limits<double>::max()) {
    return;  /**< 不可达，返回 */
  }

  /**
   * @brief 计算空间势能代价
   * 与道路曲率、车道边界等相关的代价
   */
  cost_cr.SetSpatialPotentialCost(dp_st_cost_.GetSpatialPotentialCost(cost_cr));

  /**
   * @brief 获取起点代价
   */
  const auto& cost_init = cost_table_[0][0];

  /**
   * @brief 起点处理
   */
  if (c == 0) {
    /**
     * @brief 断言：起点行索引必须为0
     */
    DCHECK_EQ(r, 0U) << "Incorrect. Row should be 0 with col = 0. row: " << r;

    /**
     * @brief 起点总代价为0
     */
    cost_cr.SetTotalCost(0.0);
    cost_cr.SetOptimalSpeed(init_point_.v());  /**< 设置初始速度 */
    return;  /**< 起点处理完成 */
  }

  /**
   * @brief 获取速度限制和巡航速度
   */
  const double speed_limit = speed_limit_by_index_[r];
  const double cruise_speed = st_graph_data_.cruise_speed();

  /**
   * @brief 最小S距离阈值
   *
   * 用于判断是否考虑速度因素
   * default: dense_unit_s_ * dimension_t_ = 0.25 * 7 = 1.75m
   */
  const double min_s_consider_speed = dense_unit_s_ * dimension_t_;

  /**
   * @brief 第1列的特殊处理
   *
   * 第1列只能从起点(0,0)到达
   * 需要验证加速度约束
   */
  if (c == 1) {
    /**
     * @brief 计算加速度
     *
     * a = 2 * (s / t - v0) / t
     * 匀加速公式推导：s = v0*t + 0.5*a*t^2
     */
    const double acc =
        2 * (cost_cr.point().s() / unit_t_ - init_point_.v()) / unit_t_;

    /**
     * @brief 检查加速度约束
     */
    if (acc < max_deceleration_ || acc > max_acceleration_) {
      return;  /**< 加速度超出限制 */
    }

    /**
     * @brief 检查速度是否为负
     */
    if (init_point_.v() + acc * unit_t_ < -kDoubleEpsilon &&
        cost_cr.point().s() > min_s_consider_speed) {
      return;  /**< 速度不能为负 */
    }

    /**
     * @brief 检查碰撞
     */
    if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                cost_init)) {
      return;  /**< 发生碰撞 */
    }

    /**
     * @brief 计算总代价
     *
     * total = obstacle_cost + spatial_potential_cost + 前驱总代价 + 边代价
     */
    cost_cr.SetTotalCost(
        cost_cr.obstacle_cost() + cost_cr.spatial_potential_cost() +
        cost_init.total_cost() +
        CalculateEdgeCostForSecondCol(r, speed_limit, cruise_speed));

    /**
     * @brief 设置最优前驱和速度
     */
    cost_cr.SetPrePoint(cost_init);
    cost_cr.SetOptimalSpeed(init_point_.v() + acc * unit_t_);
    return;  /**< 第1列处理完成 */
  }

  /**
   * @brief 速度范围缓冲区
   */
  static constexpr double kSpeedRangeBuffer = 0.20;

  /**
   * @brief 计算前驱点的最小s值
   *
   * 基于最大速度限制估算
   * FLAGS_planning_upper_speed_limit: 全局最大速度限制
   */
  const double pre_lowest_s =
      cost_cr.point().s() -
      FLAGS_planning_upper_speed_limit * (1 + kSpeedRangeBuffer) * unit_t_;

  /**
   * @brief 二分查找确定前驱点搜索下界
   */
  const auto pre_lowest_itr =
      std::lower_bound(spatial_distance_by_index_.begin(),
                       spatial_distance_by_index_.end(), pre_lowest_s);

  /**
   * @brief 计算前驱搜索范围
   */
  uint32_t r_low = 0;
  if (pre_lowest_itr == spatial_distance_by_index_.end()) {
    r_low = dimension_s_ - 1;
  } else {
    r_low = static_cast<uint32_t>(
        std::distance(spatial_distance_by_index_.begin(), pre_lowest_itr));
  }

  /**
   * @brief 前驱列需要搜索的点数
   */
  const uint32_t r_pre_size = r - r_low + 1;

  /**
   * @brief 获取前驱列引用
   */
  const auto& pre_col = cost_table_[c - 1];

  /**
   * @brief 当前速度限制（初值）
   */
  double curr_speed_limit = speed_limit;

  /**
   * @brief 第2列的处理
   *
   * 第2列需要特殊处理，因为前驱点没有速度信息
   */
  if (c == 2) {
    /**
     * @brief 遍历前驱候选点
     */
    for (uint32_t i = 0; i < r_pre_size; ++i) {
      uint32_t r_pre = r - i;

      /**
       * @brief 检查前驱点是否有效
       */
      if (std::isinf(pre_col[r_pre].total_cost()) ||
          pre_col[r_pre].pre_point() == nullptr) {
        continue;  /**< 前驱无效，跳过 */
      }

      /**
       * @brief 计算当前加速度
       *
       * TODO注释：需要更精确的加速度计算方法
       *
       * curr_v = (point.s - pre_point.s) / unit_t
       * pre_v = (pre_point.s - prepre_point.s) / unit_t
       * curr_a = (curr_v - pre_v) / unit_t
       *        = (point.s + prepre_point.s - 2*pre_point.s) / (unit_t^2)
       */
      const double curr_a =
          2 *
          ((cost_cr.point().s() - pre_col[r_pre].point().s()) / unit_t_ -
           pre_col[r_pre].GetOptimalSpeed()) /
          unit_t_;

      /**
       * @brief 检查加速度约束
       */
      if (curr_a < max_deceleration_ || curr_a > max_acceleration_) {
        continue;  /**< 加速度超出限制 */
      }

      /**
       * @brief 检查速度是否为负
       */
      if (pre_col[r_pre].GetOptimalSpeed() + curr_a * unit_t_ <
              -kDoubleEpsilon &&
          cost_cr.point().s() > min_s_consider_speed) {
        continue;  /**< 速度为负 */
      }

      /**
       * @brief 检查碰撞
       */
      if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                  pre_col[r_pre])) {
        continue;  /**< 发生碰撞 */
      }

      /**
       * @brief 更新速度限制
       */
      curr_speed_limit =
          std::fmin(curr_speed_limit, speed_limit_by_index_[r_pre]);

      /**
       * @brief 计算总代价
       */
      const double cost = cost_cr.obstacle_cost() +
                          cost_cr.spatial_potential_cost() +
                          pre_col[r_pre].total_cost() +
                          CalculateEdgeCostForThirdCol(
                              r, r_pre, curr_speed_limit, cruise_speed);

      /**
       * @brief 更新最优解
       */
      if (cost < cost_cr.total_cost()) {
        cost_cr.SetTotalCost(cost);
        cost_cr.SetPrePoint(pre_col[r_pre]);
        cost_cr.SetOptimalSpeed(pre_col[r_pre].GetOptimalSpeed() +
                                curr_a * unit_t_);
      }
    }
    return;  /**< 第2列处理完成 */
  }

  /**
   * @brief 第3列及之后的处理
   *
   * 完整的DP处理，可以使用jerk代价
   */
  for (uint32_t i = 0; i < r_pre_size; ++i) {
    uint32_t r_pre = r - i;

    /**
     * @brief 检查前驱有效性
     */
    if (std::isinf(pre_col[r_pre].total_cost()) ||
        pre_col[r_pre].pre_point() == nullptr) {
      continue;
    }

    /**
     * @brief 计算加速度
     */
    const double curr_a =
        2 *
        ((cost_cr.point().s() - pre_col[r_pre].point().s()) / unit_t_ -
         pre_col[r_pre].GetOptimalSpeed()) /
        unit_t_;

    /**
     * @brief 加速度约束检查
     */
    if (curr_a > max_acceleration_ || curr_a < max_deceleration_) {
      continue;
    }

    /**
     * @brief 速度非负检查
     */
    if (pre_col[r_pre].GetOptimalSpeed() + curr_a * unit_t_ < -kDoubleEpsilon &&
        cost_cr.point().s() > min_s_consider_speed) {
      continue;
    }

    /**
     * @brief 碰撞检查
     */
    if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                pre_col[r_pre])) {
      continue;
    }

    /**
     * @brief 获取前前驱点
     */
    uint32_t r_prepre = pre_col[r_pre].pre_point()->index_s();
    const StGraphPoint& prepre_graph_point = cost_table_[c - 2][r_prepre];

    /**
     * @brief 检查前前驱有效性
     */
    if (std::isinf(prepre_graph_point.total_cost())) {
      continue;
    }

    if (!prepre_graph_point.pre_point()) {
      continue;
    }

    /**
     * @brief 获取路径点
     */
    const STPoint& triple_pre_point = prepre_graph_point.pre_point()->point();
    const STPoint& prepre_point = prepre_graph_point.point();
    const STPoint& pre_point = pre_col[r_pre].point();
    const STPoint& curr_point = cost_cr.point();

    /**
     * @brief 更新速度限制
     */
    curr_speed_limit =
        std::fmin(curr_speed_limit, speed_limit_by_index_[r_pre]);

    /**
     * @brief 计算总代价
     *
     * 包括：
     * - obstacle_cost: 障碍物代价
     * - spatial_potential_cost: 空间势能代价
     * - pre_col[r_pre].total_cost(): 前驱累积代价
     * - edge_cost: 边代价（speed + accel + jerk）
     */
    double cost = cost_cr.obstacle_cost() + cost_cr.spatial_potential_cost() +
                  pre_col[r_pre].total_cost() +
                  CalculateEdgeCost(triple_pre_point, prepre_point, pre_point,
                                    curr_point, curr_speed_limit, cruise_speed);

    /**
     * @brief 更新最优解
     */
    if (cost < cost_cr.total_cost()) {
      cost_cr.SetTotalCost(cost);
      cost_cr.SetPrePoint(pre_col[r_pre]);
      cost_cr.SetOptimalSpeed(pre_col[r_pre].GetOptimalSpeed() +
                              curr_a * unit_t_);
    }
  }
}

/**
 * @brief 检索最优速度剖面
 *
 * @param speed_data 输出：速度剖面
 * @return Status 检索状态
 *
 * 功能说明：
 * 从代价表回溯找到最优路径
 * 将路径转换为速度剖面
 *
 * 算法流程：
 * 1. 在最后一列找到代价最小的点
 * 2. 如果最后一列没有有效点，检查最后一行
 * 3. 从最优终点回溯到起点
 * 4. 反转顺序得到从起点到终点的路径
 * 5. 计算各段速度填入SpeedPoint
 *
 * C++语法说明：
 * - std::isinf: 检查是否是无穷大
 * - std::numeric_limits<double>::infinity(): 无穷大常量
 * - std::reverse: 反转容器顺序
 */
Status GriddedPathTimeGraph::RetrieveSpeedProfile(SpeedData* const speed_data) {
  /**
   * @brief 初始化最小代价和最优终点
   */
  double min_cost = std::numeric_limits<double>::infinity();
  const StGraphPoint* best_end_point = nullptr;

  /**
   * @brief 调试：记录所有代价表中的点
   */
  PrintPoints debug("dp_node_edge");
  for (const auto& points_vec : cost_table_) {
    for (const auto& pt : points_vec) {
      debug.AddPoint(pt.point().t(), pt.point().s());
    }
  }

  /**
   * @brief 在最后一列查找最优终点
   */
  for (const StGraphPoint& cur_point : cost_table_.back()) {
    if (!std::isinf(cur_point.total_cost()) &&
        cur_point.total_cost() < min_cost) {
      best_end_point = &cur_point;
      min_cost = cur_point.total_cost();
    }
  }

  /**
   * @brief 在最后一行查找最优终点
   * 作为备选方案
   */
  for (const auto& row : cost_table_) {
    const StGraphPoint& cur_point = row.back();
    if (!std::isinf(cur_point.total_cost()) &&
        cur_point.total_cost() < min_cost) {
      best_end_point = &cur_point;
      min_cost = cur_point.total_cost();
    }
  }

  /**
   * @brief 如果没有找到有效终点，返回错误
   */
  if (best_end_point == nullptr) {
    const std::string msg = "Fail to find the best feasible trajectory.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  /**
   * @brief 回溯构建速度剖面
   */
  std::vector<SpeedPoint> speed_profile;
  const StGraphPoint* cur_point = best_end_point;

  /**
   * @brief 调试记录
   */
  PrintPoints debug_res("dp_result");

  /**
   * @brief 通过前驱指针回溯
   * pre_point()返回指向前驱点的指针
   */
  while (cur_point != nullptr) {
    ADEBUG << "Time: " << cur_point->point().t();
    ADEBUG << "S: " << cur_point->point().s();
    ADEBUG << "V: " << cur_point->GetOptimalSpeed();

    /**
     * @brief 创建速度点并添加到剖面
     */
    SpeedPoint speed_point;
    debug_res.AddPoint(cur_point->point().t(), cur_point->point().s());
    speed_point.set_s(cur_point->point().s());
    speed_point.set_t(cur_point->point().t());
    speed_profile.push_back(speed_point);

    cur_point = cur_point->pre_point();  /**< 移动到前驱点 */
  }

  /**
   * @brief 反转顺序
   *
   * 回溯得到的是从终点到起点的顺序
   * 需要反转得到从起点到终点的顺序
   */
  std::reverse(speed_profile.begin(), speed_profile.end());

  /**
   * @brief 检查起点有效性
   */
  static constexpr double kEpsilon = std::numeric_limits<double>::epsilon();
  if (speed_profile.front().t() > kEpsilon ||
      speed_profile.front().s() > kEpsilon) {
    const std::string msg = "Fail to retrieve speed profile.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  /**
   * @brief 计算各段速度
   *
   * v = ds / dt
   * 对相邻点计算速度
   */
  for (size_t i = 0; i + 1 < speed_profile.size(); ++i) {
    const double v = (speed_profile[i + 1].s() - speed_profile[i].s()) /
                     (speed_profile[i + 1].t() - speed_profile[i].t() + 1e-3);
    speed_profile[i].set_v(v);  /**< 设置速度 */
  }

  /**
   * @brief 最后一锥面包容没有速度的点
   * 设为0速度
   */
  speed_profile[speed_profile.size() - 1].set_v(0.0);

  /**
   * @brief 转换为SpeedData并返回
   */
  *speed_data = SpeedData(speed_profile);
  return Status::OK();
}

/**
 * @brief 计算边代价（通用版本）
 *
 * @param first 前前驱点
 * @param second 前驱点
 * @param third 当前点的前驱
 * @param forth 当前点
 * @param speed_limit 速度限制
 * @param cruise_speed 巡航速度
 * @return double 边代价
 *
 * 功能说明：
 * 计算从third到forth的边代价
 * 包括：速度代价、加速度代价、jerk代价
 */
double GriddedPathTimeGraph::CalculateEdgeCost(
    const STPoint& first, const STPoint& second, const STPoint& third,
    const STPoint& forth, const double speed_limit, const double cruise_speed) {
  /**
   * @brief 速度代价：与限速和巡航速度的偏差
   */
  return dp_st_cost_.GetSpeedCost(third, forth, speed_limit, cruise_speed) +
         /**
          * @brief 加速度代价：基于三个点计算
          */
         dp_st_cost_.GetAccelCostByThreePoints(second, third, forth) +
         /**
          * @brief jerk代价：基于四个点计算
          * jerk是加速度的变化率
          */
         dp_st_cost_.GetJerkCostByFourPoints(first, second, third, forth);
}

/**
 * @brief 计算第2列的边代价
 *
 * @param row 当前行索引
 * @param speed_limit 速度限制
 * @param cruise_speed 巡航速度
 * @return double 边代价
 *
 * 功能说明：
 * 第2列特殊处理，因为前驱只有起点
 */
double GriddedPathTimeGraph::CalculateEdgeCostForSecondCol(
    const uint32_t row, const double speed_limit, const double cruise_speed) {
  /**
   * @brief 获取初始速度和加速度
   */
  double init_speed = init_point_.v();
  double init_acc = init_point_.a();

  /**
   * @brief 获取路径点
   */
  const STPoint& pre_point = cost_table_[0][0].point();
  const STPoint& curr_point = cost_table_[1][row].point();

  /**
   * @brief 计算边代价
   *
   * 包括：
   * - 速度代价
   * - 加速度代价（两点版本）
   * - jerk代价（两点版本）
   */
  return dp_st_cost_.GetSpeedCost(pre_point, curr_point, speed_limit,
                                  cruise_speed) +
         dp_st_cost_.GetAccelCostByTwoPoints(init_speed, pre_point,
                                             curr_point) +
         dp_st_cost_.GetJerkCostByTwoPoints(init_speed, init_acc, pre_point,
                                            curr_point);
}

/**
 * @brief 计算第3列的边代价
 *
 * @param curr_row 当前行索引
 * @param pre_row 前流行索引
 * @param speed_limit 速度限制
 * @param cruise_speed 巡航速度
 * @return double 边代价
 */
double GriddedPathTimeGraph::CalculateEdgeCostForThirdCol(
    const uint32_t curr_row, const uint32_t pre_row, const double speed_limit,
    const double cruise_speed) {
  /**
   * @brief 获取初始速度
   */
  double init_speed = init_point_.v();

  /**
   * @brief 获取路径点
   */
  const STPoint& first = cost_table_[0][0].point();
  const STPoint& second = cost_table_[1][pre_row].point();
  const STPoint& third = cost_table_[2][curr_row].point();

  /**
   * @brief 计算边代价
   */
  return dp_st_cost_.GetSpeedCost(second, third, speed_limit, cruise_speed) +
         dp_st_cost_.GetAccelCostByThreePoints(first, second, third) +
         dp_st_cost_.GetJerkCostByThreePoints(init_speed, first, second, third);
}

}  // namespace planning
}  // namespace apollo
