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
 * @file path_time_heuristic_optimizer.cc
 * @brief 路径时间启发式优化器实现文件
 *
 * 本文件实现了基于启发式搜索的速度优化器(PathTimeHeuristicOptimizer)
 * 用于在路径-时间(ST)图上进行速度规划
 *
 * 功能说明：
 * 1. 使用网格化的路径时间图(GriddedPathTimeGraph)进行搜索
 * 2. 基于动态规划(Dynamic Programming)算法搜索最优速度曲线
 * 3. 考虑障碍物的时空约束
 * 4. 支持换道场景和普通场景的不同配置
 *
 * 算法原理：
 * - 在ST图(纵轴为时间T，横轴为路径S)上进行网格化搜索
 * - 每个网格点代表一个(路径点, 时间点)的状态
 * - 通过动态规划找到从起点到终点的最优速度曲线
 * - 考虑速度限制、加速度限制、jerk限制等约束
 *
 * 相关C++语法说明：
 * - std::shared_ptr<T>: 智能指针，用于共享所有权的对象
 * - std::string: 字符串类型
 * - const引用: 避免拷贝，提高效率
 * - 初始化列表: 用于构造函数初始化成员变量
 **/

/**
 * @brief 路径时间启发式优化器头文件
 *
 * 包含PathTimeHeuristicOptimizer类的完整定义
 */
#include "modules/planning/tasks/path_time_heuristic/path_time_heuristic_optimizer.h"

/**
 * @brief Apollo配置和状态工具头文件
 * VehicleConfigHelper: 车辆配置助手，用于获取车辆参数
 * VehicleStateProvider: 车辆状态提供者，用于获取当前车辆状态
 */
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"

/**
 * @brief ST图数据头文件
 * StGraphData: 存储ST图相关数据的数据结构
 */
#include "modules/planning/planning_base/common/st_graph_data.h"

/**
 * @brief 规划模块GFlags头文件
 * GFlags用于从命令行或配置文件获取配置参数
 */
#include "modules/planning/planning_base/gflags/planning_gflags.h"

/**
 * @brief 网格化路径时间图头文件
 * GriddedPathTimeGraph: 网格化的ST图，用于启发式搜索
 */
#include "modules/planning/tasks/path_time_heuristic/gridded_path_time_graph.h"

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
 * @brief 使用别名声明，简化类型引用
 *
 * C++语法说明：
 * using声明：引入其他命名空间的类型，简化后续代码中的类型引用
 * ErrorCode: Apollo错误码类型，用于表示操作的成功/失败状态
 * Status: Apollo通用状态类型，包含错误码和错误信息
 */
using apollo::common::ErrorCode;
using apollo::common::Status;

/**
 * @brief 初始化函数
 *
 * @param config_dir 配置文件目录路径
 * @param name 优化器名称
 * @param injector 依赖注入器指针，用于获取各种服务和数据
 * @return bool 初始化是否成功
 *
 * 功能说明：
 * 初始化路径时间启发式优化器
 * 1. 调用父类SpeedOptimizer的初始化函数
 * 2. 加载SpeedHeuristicOptimizerConfig配置
 *
 * 算法流程：
 * 1. 首先调用父类SpeedOptimizer的Init进行基础初始化
 * 2. 如果基础初始化成功，则加载本任务特有的配置
 * 3. 返回初始化结果
 *
 * C++语法说明：
 * - const std::string&: 常量引用，string是标准库字符串类型
 * - std::shared_ptr<DependencyInjector>: 共享所有权的智能指针
 *   用于多个对象共享同一个DependencyInjector，引用计数为0时自动释放
 * - std::shared_ptr<T>::operator*: 解引用获取原始指针
 * - SpeedOptimizer::LoadConfig<T>: 模板函数，从配置文件加载指定类型的配置
 */
bool PathTimeHeuristicOptimizer::Init(
    const std::string& config_dir, const std::string& name,
    const std::shared_ptr<DependencyInjector>& injector) {
  /**
   * @brief 调用父类SpeedOptimizer的Init进行基础初始化
   *
   * SpeedOptimizer::Init():
   *   父类的初始化函数，执行基础初始化操作
   *   如：保存配置目录、名称、依赖注入器等
   */
  if (!SpeedOptimizer::Init(config_dir, name, injector)) {
    return false;  /**< 父类初始化失败，返回false */
  }

  /**
   * @brief 加载本任务的配置
   *
   * SpeedOptimizer::LoadConfig<T>():
   *   模板成员函数，用于从配置目录加载配置
   *   T指定了配置的类型，这里是SpeedHeuristicOptimizerConfig
   *   配置内容可能包括：搜索参数、约束条件、代价函数权重等
   *
   * C++语法说明：
   * - template<typename T>: 函数模板，T是模板参数
   * - &config_: 取成员变量的地址，LoadConfig将配置写入config_
   */
  return SpeedOptimizer::LoadConfig<SpeedHeuristicOptimizerConfig>(&config_);
}

/**
 * @brief 搜索路径时间图
 *
 * @param speed_data 输出：搜索得到的速度数据
 * @return bool 搜索是否成功
 *
 * 功能说明：
 * 在网格化的路径时间图上进行动态规划搜索
 * 找到从初始状态到最终状态的最优速度曲线
 *
 * 算法流程：
 * 1. 根据是否换道选择相应的配置
 * 2. 创建GriddedPathTimeGraph对象
 * 3. 调用Search方法进行搜索
 * 4. 将搜索结果写入speed_data
 *
 * C++语法说明：
 * - SpeedData*: 指向速度数据的指针，用于输出结果
 * - const ... & : 常量引用，避免拷贝
 * - ternary operator (?:): 条件运算符，根据IsChangeLanePath()选择不同配置
 */
bool PathTimeHeuristicOptimizer::SearchPathTimeGraph(
    SpeedData* speed_data) const {
  /**
   * @brief 根据场景选择配置
   *
   * reference_line_info_->IsChangeLanePath():
   *   判断当前是否在换道过程中
   *
   * ternary operator (? :):
   *   条件 ? 值1 : 值2
   *   条件为真时返回"值1"，否则返回"值2"
   *
   * - 如果是换道场景：使用lane_change_speed_config（换道速度配置）
   * - 如果是普通场景：使用default_speed_config（默认速度配置）
   *
   * 配置内容可能包括：
   * - 搜索网格大小
   * - 速度/加速度/jerk限制
   * - 代价函数权重
   */
  const auto& dp_st_speed_optimizer_config =
      reference_line_info_->IsChangeLanePath()
          ? config_.lane_change_speed_config()
          : config_.default_speed_config();

  /**
   * @brief 创建网格化路径时间图对象
   *
   * GriddedPathTimeGraph构造函数参数：
   * 1. reference_line_info_->st_graph_data():
   *    ST图数据，包含障碍物的ST边界信息
   * 2. dp_st_speed_optimizer_config:
   *    速度优化器配置
   * 3. reference_line_info_->path_decision()->obstacles().Items():
   *    路径决策中的所有障碍物列表
   * 4. init_point_:
   *    规划起点（上一帧轨迹的终点）
   *
   * C++语法说明：
   * - reference_line_info_->: ->运算符，通过原始指针访问成员
   * - path_decision()->obstacles().Items():
   *   ()调用函数，->访问指针，.访问成员
   * - const auto&: 常量引用，避免拷贝
   */
  GriddedPathTimeGraph st_graph(
      reference_line_info_->st_graph_data(), dp_st_speed_optimizer_config,
      reference_line_info_->path_decision()->obstacles().Items(), init_point_);

  /**
   * @brief 在ST图上搜索最优速度曲线
   *
   * st_graph.Search(speed_data):
   *   在网格化的ST图上进行动态规划搜索
   *   返回Status对象表示搜索结果
   *
   * C++语法说明：
   * - .ok(): Status类的方法，检查状态是否成功
   * - speed_data: 输出参数，搜索结果写入此对象
   */
  if (!st_graph.Search(speed_data).ok()) {
    AERROR << "failed to search graph with dynamic programming.";
    return false;  /**< 搜索失败，返回false */
  }

  return true;  /**< 搜索成功，返回true */
}

/**
 * @brief 处理函数（主入口）
 *
 * @param path_data 路径数据
 * @param init_point 规划起点
 * @param speed_data 输出：速度数据
 * @return Status 处理状态
 *
 * 功能说明：
 * 速度规划的入口函数
 * 1. 保存规划起点
 * 2. 检查输入数据有效性
 * 3. 调用SearchPathTimeGraph进行搜索
 * 4. 记录调试信息
 *
 * 算法流程：
 * 1. 将init_point保存为成员变量init_point_
 * 2. 检查path_data是否为空
 * 3. 调用SearchPathTimeGraph进行ST图搜索
 * 4. 如果搜索失败，记录调试信息并返回错误
 * 5. 如果搜索成功，记录调试信息并返回成功
 *
 * C++语法说明：
 * - const PathData&: 常量引用，PathData是路径数据结构
 * - const common::TrajectoryPoint&: 常量引用，TrajectoryPoint包含位置、速度、加速度等
 * - SpeedData* const speed_data: 指向常量的指针（指针本身是常量）
 *   指针指向的对象可以被修改，但指针本身不能改变指向
 */
Status PathTimeHeuristicOptimizer::Process(
    const PathData& path_data, const common::TrajectoryPoint& init_point,
    SpeedData* const speed_data) {
  /**
   * @brief 保存规划起点
   *
   * init_point_: 成员变量，保存当前帧的规划起点
   * 这个起点是上一帧轨迹的终点，用于保证轨迹的连续性
   *
   * C++语法说明：
   * init_point_: 成员变量赋值
   */
  init_point_ = init_point;

  /**
   * @brief 检查路径数据是否为空
   *
   * path_data.discretized_path().empty():
   *   检查离散化路径是否为空
   *   discretized_path()返回路径点向量，empty()检查向量是否为空
   *
   * 如果路径为空，说明规划失败，返回错误状态
   */
  if (path_data.discretized_path().empty()) {
    const std::string msg = "Empty path data";  /**< 错误信息 */
    AERROR << msg;  /**< 记录错误日志 */
    return Status(ErrorCode::PLANNING_ERROR, msg);  /**< 返回规划错误状态 */
  }

  /**
   * @brief 搜索ST图获取速度曲线
   *
   * SearchPathTimeGraph(speed_data):
   *   在ST图上进行动态规划搜索
   *   speed_data作为输出参数，存储搜索得到的速度曲线
   */
  if (!SearchPathTimeGraph(speed_data)) {
    /**
     * @brief 搜索失败，记录错误信息
     *
     * absl::StrCat():
     *   Abseil库的字符串拼接函数
     *   用于将多个字符串拼接成一个
     *   这里用于生成完整的错误信息，包含优化器名称
     *
     * Name():
     *   返回优化器的名称，通过宏定义或继承获得
     */
    const std::string msg = absl::StrCat(
        Name(), ": Failed to search graph with dynamic programming.");
    AERROR << msg;  /**< 记录错误日志 */

    /**
     * @brief 记录调试信息
     *
     * RecordDebugInfo():
     *   将速度数据和ST图调试信息记录到debug结构
     *   用于可视化分析和问题排查
     *
     * C++语法说明：
     * - reference_line_info_->mutable_st_graph_data():
     *   mutable_前缀的方法表示可以修改const对象中的可变成员
     * - ->mutable_st_graph_debug():
     *   获取ST图调试信息的可修改指针
     */
    RecordDebugInfo(*speed_data, reference_line_info_->mutable_st_graph_data()
                                     ->mutable_st_graph_debug());

    return Status(ErrorCode::PLANNING_ERROR, msg);  /**< 返回规划错误状态 */
  }

  /**
   * @brief 搜索成功，记录调试信息
   *
   * 即使搜索成功，也记录调试信息以便后续分析
   */
  RecordDebugInfo(
      *speed_data,
      reference_line_info_->mutable_st_graph_data()->mutable_st_graph_debug());

  return Status::OK();  /**< 返回成功状态 */
}

}  // namespace planning
}  // namespace apollo
