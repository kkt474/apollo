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
 * @file lane_borrow_path.h
 * @brief 车道借道路径生成器头文件
 *
 * 功能说明：
 * 定义了车道借道路径生成器类的接口
 * 当主车道被阻塞时，借用相邻车道超车或绕行
 *
 * 应用场景：
 * - 车道前方有静止障碍物
 * - 车道拥堵需要借道超车
 * - 紧急情况下需要借道避让
 *
 * 借道方向：
 * - LEFT_BORROW：向左借道
 * - RIGHT_BORROW：向右借道
 *
 * 设计理念：
 * 1. 继承自PathGeneration基类
 * 2. 标准化三阶段处理：边界决定、路径优化、路径评估
 * 3. 支持左右两侧借道决策
 * 4. 使用插件机制动态加载
 *
 * 工作流程：
 * 1. Init() - 初始化，加载配置
 * 2. Process() - 执行路径生成主流程
 * 3. DecidePathBounds() - 决定路径边界
 * 4. OptimizePath() - 优化路径
 * 5. AssessPath() - 评估并选择最佳路径
 *
 * C++语法说明：
 * - class：类声明关键字
 * - public：公有继承
 * - virtual：虚函数，支持运行时多态
 * - override：重写基类虚函数
 * - std::shared_ptr：共享所有权智能指针
 * - std::string：标准库字符串
 * - std::vector：动态数组容器
 * - std::tuple：元组容器
 * - #pragma once：头文件保护
 */

/**
 * @brief 头文件保护指令
 *
 * C++语法说明：
 * - #pragma once：
 *   预处理指令，防止头文件被多次包含
 *   现代编译器支持，比#ifndef...#define...#endif更简洁
 *   优点：无需手动写保护宏，编译器自动处理
 */
#pragma once

/**
 * @brief C++标准库头文件
 *
 * C++语法说明：
 * - #include <memory>：
 *   智能指针头文件
 *   提供std::shared_ptr、std::unique_ptr等智能指针
 *   智能指针可以自动管理内存，避免内存泄漏
 *
 * - #include <string>：
 *   标准库字符串类
 *   std::string是处理字符串的常用类型
 *
 * - #include <tuple>：
 *   元组容器头文件
 *   std::tuple可以存储不同类型的多个值
 *   类似Python的tuple
 *
 * - #include <vector>：
 *   动态数组容器
 *   std::vector是C++中最常用的容器之一
 */
#include <memory>
#include <string>
#include <tuple>
#include <vector>

/**
 * @brief Protobuf配置头文件
 *
 * 功能说明：
 * - lane_borrow_path.pb.h：
 *   Protobuf编译生成的代码
 *   定义了LaneBorrowPathConfig配置结构
 *   包含借道路径生成器的所有配置参数
 *
 * C++语法说明：
 * - "modules/planning/tasks/lane_borrow_path/proto/..."：
 *   使用引号表示自定义头文件路径
 *   protobuf生成的代码通常放在proto子目录
 */
#include "modules/planning/tasks/lane_borrow_path/proto/lane_borrow_path.pb.h"

/**
 * @brief Cyber RT插件管理器头文件
 *
 * 功能说明：
 * - cyber/plugin_manager/plugin_manager.h：
 *   Apollo Cyber RT的插件系统
 *   允许动态加载和创建Task实例
 *
 * C++语法说明：
 * - CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN：
 *   宏，用于注册插件到插件管理器
 */
#include "cyber/plugin_manager/plugin_manager.h"

/**
 * @brief 路径生成基类头文件
 *
 * 功能说明：
 * - path_generation.h：
 *   定义了PathGeneration类
 *   所有路径生成任务继承自此类
 *   提供路径生成的通用接口和流程
 *
 * C++语法说明：
 * - PathGeneration：
 *   路径生成基类
 *   LaneBorrowPath继承自PathGeneration
 *   继承关系：LaneBorrowPath -> PathGeneration -> Task -> Decider
 */
#include "modules/planning/planning_interface_base/task_base/common/path_generation.h"

/**
 * @namespace apollo::planning
 * @brief Apollo规划模块命名空间
 *
 * C++语法说明：
 * - namespace apollo：
 *   Apollo最外层命名空间
 *   所有Apollo代码都在此命名空间下
 * - namespace planning：
 *   规划子命名空间
 */
namespace apollo {
namespace planning {

/**
 * @enum SidePassDirection
 * @brief 借道方向枚举
 *
 * 功能说明：
 * 定义了借道行驶的方向
 *
 * 枚举值：
 * - LEFT_BORROW = 1：向左借道
 * - RIGHT_BORROW = 2：向右借道
 *
 * C++语法说明：
 * - enum SidePassDirection：
 *   枚举类型定义
 *   定义一组命名的整型常量
 *
 * - LEFT_BORROW = 1：
 *   显式指定枚举值为1
 *   默认第一个枚举值为0
 *
 * - RIGHT_BORROW = 2：
 *   显式指定枚举值为2
 */
enum SidePassDirection { LEFT_BORROW = 1, RIGHT_BORROW = 2 };

/**
 * @class LaneBorrowPath
 * @brief 车道借道路径生成器类
 *
 * 功能说明：
 * 当主车道被阻塞时，生成借用相邻车道的路径
 * 继承自PathGeneration类，是Task的派生类
 *
 * 继承关系：
 * LaneBorrowPath -> PathGeneration -> Task -> Decider
 *
 * 设计模式：
 * - 模板方法模式：基类定义算法骨架
 * - 策略模式：不同任务有不同行为
 * - 插件模式：动态加载
 *
 * C++语法说明：
 * - class LaneBorrowPath :
 *   类声明，定义一个名为LaneBorrowPath的类
 *
 * - public PathGeneration：
 *   公有继承自PathGeneration类
 *   public继承表示：
 *   - 基类的public成员仍是public
 *   - 基类的protected成员仍是protected
 *   - 基类的private成员不可直接访问
 */
class LaneBorrowPath : public PathGeneration {
 public:
  /**
   * @brief 初始化车道借道路径生成器
   *
   * @param config_dir 配置文件目录
   * @param name 任务名称
   * @param injector 依赖注入器指针
   * @return bool 初始化是否成功
   *
   * 功能说明：
   * 初始化生成器的内部状态和配置
   *
   * 重写说明：
   * 该方法重写了基类PathGeneration的Init方法
   *
   * C++语法说明：
   * - bool Init(...)：
   *   返回布尔值表示成功/失败
   *
   * - const std::string& config_dir：
   *   常量引用，传入配置文件目录路径
   *   const保证配置不会被修改
   *   引用避免拷贝大型字符串
   *
   * - const std::string& name：
   *   常量引用，传入任务名称
   *
   * - const std::shared_ptr<DependencyInjector>& injector：
   *   共享指针的常量引用
   *   - shared_ptr：多个所有者共享对象
   *   - const引用：既不拷贝也不获得所有权
   *
   * - override：
   *   C++11关键字，显式表示重写基类虚函数
   *   编译器会检查基类是否有可重写的对应虚函数
   */
  bool Init(const std::string& config_dir, const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;

 private:
  /**
   * @brief 处理路径生成
   *
   * @param frame 规划帧数据
   * @param reference_line_info 参考线信息
   * @return apollo::common::Status 处理状态
   *
   * 功能说明：
   * 借道路径生成的核心处理函数
   * 执行完整的路径生成流程
   *
   * 重写说明：
   * 该方法重写了基类PathGeneration的Process方法
   *
   * C++语法说明：
   * - apollo::common::Status：
   *   Apollo通用状态类，包含错误码和消息
   *
   * - Frame*：
   *   原始指针，指向规划帧
   *
   * - ReferenceLineInfo*：
   *   参考线信息指针
   *
   * - override：
   *   重写基类虚函数
   */
  apollo::common::Status Process(
      Frame* frame, ReferenceLineInfo* reference_line_info) override;

  /**
   * @brief 计算所有路径边界
   *
   * @param boundary 输出：计算的路径边界
   * @return bool 计算是否成功
   *
   * 功能说明：
   * 决定所有可能的路径边界
   * 包括自车道边界和借道边界
   *
   * C++语法说明：
   * - std::vector<PathBoundary>* boundary：
   *   指向路径边界向量的指针
   *   用于输出多个路径边界
   */
  bool DecidePathBounds(std::vector<PathBoundary>* boundary);

  /**
   * @brief 为每个路径边界优化路径
   *
   * @param path_boundaries 输入的路径边界
   * @param candidate_path_data 输出：候选路径数据
   * @return bool 优化是否成功
   *
   * 功能说明：
   * 根据给定的路径边界生成优化路径
   *
   * C++语法说明：
   * - const std::vector<PathBoundary>& path_boundaries：
   *   常量引用，输入的路径边界
   *
   * - std::vector<PathData>* candidate_path_data：
   *   指针输出参数
   */
  bool OptimizePath(const std::vector<PathBoundary>& path_boundaries,
                    std::vector<PathData>* candidate_path_data);

  /**
   * @brief 评估每条路径的可行性并选择最佳路径
   *
   * @param candidate_path_data 输入的候选路径
   * @param final_path 输出：最佳路径
   * @return bool 评估是否成功
   *
   * 功能说明：
   * 评估所有候选路径的可行性
   *
   * C++语法说明：
   * - std::vector<PathData>* candidate_path_data：
   *   指针输入/输出参数
   *
   * - PathData* final_path：
   *   指针输出参数
   */
  bool AssessPath(std::vector<PathData>* candidate_path_data,
                  PathData* final_path);

  /**
   * @brief 从相邻车道和自车道生成路径边界
   *
   * @param pass_direction 借道方向（左侧或右侧）
   * @param path_bound 输出：路径边界
   * @param borrow_lane_type 输出：借道类型
   * @return bool 生成是否成功
   *
   * 功能说明：
   * 根据借道方向从相邻车道或自车道生成路径边界
   *
   * C++语法说明：
   * - const SidePassDirection pass_direction：
   *   借道方向枚举
   *
   * - PathBoundary* const path_bound：
   *   指向常量的指针
   *   - PathBoundary*：指针类型
   *   - const：指针指向的内容不可修改
   *
   * - std::string* borrow_lane_type：
   *   指针输出参数
   */
  bool GetBoundaryFromNeighborLane(const SidePassDirection pass_direction,
                                   PathBoundary* const path_bound,
                                   std::string* borrow_lane_type);

  /**
   * @brief 判断是否需要借道
   *
   * @return bool 如果需要借道返回true
   *
   * 功能说明：
   * 判断当前情况下是否需要借道行驶
   * 考虑障碍物、车道条件等因素
   */
  bool IsNecessaryToBorrowLane();

  /**
   * @brief 检查是否有单条参考线
   *
   * @param frame 规划帧数据
   * @return bool 如果只有单条参考线返回true
   *
   * C++语法说明：
   * - const Frame& frame：
   *   常量引用
   */
  bool HasSingleReferenceLine(const Frame& frame);

  /**
   * @brief 检查自车速度是否在借道速度范围内
   *
   * @param frame 规划帧数据
   * @return bool 如果速度适合借道返回true
   *
   * 功能说明：
   * 借道需要一定速度条件
   * 速度太快或太慢都不适合借道
   */
  bool IsWithinSidePassingSpeedADC(const Frame& frame);

  /**
   * @brief 判断是否为长期阻塞障碍物
   *
   * @return bool 如果是长期阻塞障碍物返回true
   *
   * 功能说明：
   * 只有长期阻塞的障碍物才需要借道
   * 短暂阻塞可以等待
   */
  bool IsLongTermBlockingObstacle();

  /**
   * @brief 判断阻塞障碍物是否在目的地前方
   *
   * @param reference_line_info 参考线信息
   * @return bool 如果在目的地前方返回true
   *
   * 功能说明：
   * 如果障碍物在目的地之后，不值得借道
   */
  bool IsBlockingObstacleWithinDestination(
      const ReferenceLineInfo& reference_line_info);

  /**
   * @brief 判断阻塞障碍物是否远离十字路口
   *
   * @param reference_line_info 参考线信息
   * @return bool 如果远离十字路口返回true
   *
   * 功能说明：
   * 在十字路口附近一般不允许借道
   */
  bool IsBlockingObstacleFarFromIntersection(
      const ReferenceLineInfo& reference_line_info);

  /**
   * @brief 判断是否可以借道通过障碍物
   *
   * @param reference_line_info 参考线信息
   * @return bool 如果可以借道通过返回true
   *
   * 功能说明：
   * 综合考虑障碍物类型、位置等因素
   * 判断是否可以安全借道通过
   */
  bool IsSidePassableObstacle(const ReferenceLineInfo& reference_line_info);

  /**
   * @brief 更新自车道路径信息
   *
   * 功能说明：
   * 更新自车道路径的决策信息
   */
  void UpdateSelfPathInfo();

  /**
   * @brief 检查相邻车道是否可以借道
   *
   * @param reference_line_info 参考线信息
   * @param left_neighbor_lane_borrowable 输出：左侧相邻车道是否可借
   * @param right_neighbor_lane_borrowable 输出：右侧相邻车道是否可借
   *
   * 功能说明：
   * 检查左右两侧相邻车道是否允许借道
   * 考虑车道线类型、路况等因素
   *
   * C++语法说明：
   * - bool* left_neighbor_lane_borrowable：
   *   指针输出参数
   *   函数内赋值表示左侧是否可借
   *
   * - bool* right_neighbor_lane_borrowable：
   *   指针输出参数
   */
  void CheckLaneBorrow(const ReferenceLineInfo& reference_line_info,
                       bool* left_neighbor_lane_borrowable,
                       bool* right_neighbor_lane_borrowable);

  /**
   * @brief 根据边界类型检查车道是否可借
   *
   * @param reference_line_info 参考线信息
   * @param check_s 检查的s位置
   * @param lane_borrow_info 借道方向
   * @return bool 如果可借返回true
   *
   * 功能说明：
   * 检查指定位置的车道边界类型
   * 如果是实线则不能借道
   *
   * C++语法说明：
   * - const double check_s：
   *   const double类型，检查的s坐标
   *
   * - const SidePassDirection& lane_borrow_info：
   *   常量引用，借道方向枚举
   */
  bool CheckLaneBoundaryType(const ReferenceLineInfo& reference_line_info,
                             const double check_s,
                             const SidePassDirection& lane_borrow_info);

  /**
   * @brief 设置路径点决策指南信息
   *
   * @param path_data 路径数据
   *
   * 功能说明：
   * 设置路径点的决策信息
   * 描述路径的类型和属性
   *
   * C++语法说明：
   * - PathData* const path_data：
   *   指向常量的指针
   */
  void SetPathInfo(PathData* const path_data);

 private:
  /**
   * @brief 借道路径生成器配置
   *
   * 功能说明：
   * 存储借道路径生成器的配置参数
   *
   * C++语法说明：
   * - LaneBorrowPathConfig：
   *   Protobuf消息类型
   *
   * - config_：
   *   成员变量命名约定：xxx_后缀
   */
  LaneBorrowPathConfig config_;

  /**
   * @brief 决定的借道方向列表
   *
   * 功能说明：
   * 存储已决定的借道方向
   *
   * C++语法说明：
   * - std::vector<SidePassDirection>：
   *   存储借道方向枚举的向量
   */
  std::vector<SidePassDirection> decided_side_pass_direction_;

  /**
   * @brief 是否使用自车道
   *
   * 功能说明：
   * 标记是否决定使用自车道
   * -1表示未决定，0表示不使用，1表示使用
   *
   * C++语法说明：
   * - int：
   *   整数类型
   */
  int use_self_lane_;

  /**
   * @brief 阻塞障碍物ID
   *
   * 功能说明：
   * 存储导致需要借道的障碍物ID
   *
   * C++语法说明：
   * - std::string：
   *   标准库字符串类型
   */
  std::string blocking_obstacle_id_;
};

///////////////////////////////////////////////////////////////////////////////
// Below are helper functions.
// 以下是辅助函数

/**
 * @brief 检查路径决策中是否存在出口
 *
 * @param path_point_decision 路径点决策元组列表
 *        每个元组包含：(s坐标, 路径点类型, 距离)
 * @return int 返回1表示有出口，0表示没有
 *
 * 功能说明：
 * 检查路径决策中是否存在驶出自车道的点
 *
 * C++语法说明：
 * - const std::vector<std::tuple<double, PathData::PathPointType, double>>&：
 *   常量引用，传入路径点决策元组列表
 *   - std::vector：动态数组
 *   - std::tuple：元组模板类
 *   - double：s坐标类型
 *   - PathData::PathPointType：路径点类型枚举
 *   - double：距离类型
 */
int ContainsOutOnReverseLane(
    const std::vector<std::tuple<double, PathData::PathPointType, double>>&
        path_point_decision);

/**
 * @brief 获取返回车道内的索引
 *
 * @param path_point_decision 路径点决策元组列表
 * @return int 返回开始返回车道的位置索引
 *
 * 功能说明：
 * 找到路径中开始返回自车道的位置索引
 *
 * C++语法说明：
 * - 返回值：
 *   -1表示未找到
 *   其他值表示索引位置
 */
int GetBackToInLaneIndex(
    const std::vector<std::tuple<double, PathData::PathPointType, double>>&
        path_point_decision);

/**
 * @brief 比较两条路径的优先级
 *
 * @param lhs 左侧路径数据
 * @param rhs 右侧路径数据
 * @param blocking_obstacle 阻塞障碍物
 * @return bool 如果lhs比rhs更优返回true
 *
 * 功能说明：
 * 比较两条候选路径的优劣
 * 选择更好的作为最终路径
 *
 * C++语法说明：
 * - const PathData& lhs/rhs：
 *   常量引用，输入路径数据
 *
 * - const Obstacle* blocking_obstacle：
 *   指向常量障碍物的指针
 */
bool ComparePathData(const PathData& lhs, const PathData& rhs,
                     const Obstacle* blocking_obstacle);

/**
 * @brief Cyber插件注册宏
 *
 * 功能说明：
 * 将LaneBorrowPath类注册为Task类型的插件
 * 允许系统在运行时发现和加载这个路径生成器
 *
 * C++语法说明：
 * - CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN：
 *   Apollo Cyber RT的插件注册宏
 *   这是一个类级别的注册机制
 *
 * - apollo::planning::LaneBorrowPath：
 *   要注册的类名（完整命名空间）
 *
 * - Task：
 *   注册的插件基类型
 */
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::LaneBorrowPath, Task)

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