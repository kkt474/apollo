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
 * @file decision_data.h
 * @brief 决策数据类头文件
 *
 * 功能说明：
 * 该文件定义了决策数据类，用于管理和处理预测障碍物数据
 * 支持静态障碍物、动态障碍物、虚拟障碍物的分类管理
 * 提供障碍物查询、虚拟障碍物创建等功能
 *
 * C++语法说明：
 * - #pragma once：确保头文件只被包含一次
 * - 嵌套命名空间：apollo::planning
 * - enum class：强类型枚举，比传统enum更安全
 * - std::mutex：线程互斥锁，用于保护共享数据
 * - std::list<std::unique_ptr<Obstacle>>：拥有所有权的障碍物列表
 */

#pragma once

#include "modules/common_msgs/prediction_msgs/prediction_obstacle.pb.h"
#include "modules/common/math/box2d.h"
#include "modules/planning/planning_base/common/obstacle.h"
#include "modules/planning/planning_base/reference_line/reference_line.h"

namespace apollo {
/**
 * @brief Apollo外层命名空间
 */
namespace planning {

/**
 * @enum VirtualObjectType
 * @brief 虚拟对象类型枚举
 *
 * 功能说明：
 * 定义了虚拟障碍物的类型，用于交通规则处理
 *
 * 枚举值说明：
 * - DESTINATION = 0：目的地虚拟障碍物
 * - CROSSWALK = 1：人行横道虚拟障碍物（减速让行）
 * - TRAFFIC_LIGHT = 2：交通信号灯虚拟障碍物（红灯停）
 * - CLEAR_ZONE = 3：清除区域虚拟障碍物（禁止停车区域）
 * - REROUTE = 4：重路由虚拟障碍物
 * - DECISION_JUMP = 5：决策跳转虚拟障碍物
 * - PRIORITY = 6：优先级虚拟障碍物
 *
 * C++语法说明：
 * - enum class：强类型枚举（作用域枚举）
 *   - 优点：不会污染命名空间，不会隐式转换为int
 *   - 访问方式：VirtualObjectType::DESTINATION
 *   - 传统enum访问方式：DESTINATION（可能冲突）
 */
enum class VirtualObjectType {
  DESTINATION = 0,     /**< 目的地 */
  CROSSWALK = 1,       /**< 人行横道 */
  TRAFFIC_LIGHT = 2,   /**< 交通信号灯 */
  CLEAR_ZONE = 3,      /**< 清除区域 */
  REROUTE = 4,         /**< 重路由 */
  DECISION_JUMP = 5,   /**< 决策跳转 */
  PRIORITY = 6         /**< 优先级 */
};

/**
 * @struct EnumClassHash
 * @brief 枚举类哈希函数对象
 *
 * 功能说明：
 * 为enum class类型的枚举值提供哈希函数
 * 用于std::unordered_map等容器的键类型
 *
 * C++语法说明：
 * - struct：结构体（默认public访问）
 * - 模板类：template <typename T>使结构体可泛化
 * - operator()：函数调用运算符，重载后可像函数一样使用
 *   DecisionData d; EnumClassHash h; h(some_enum); 等价于 h.operator()(some_enum)
 * - static_cast<size_t>(t)：将枚举值转换为size_t类型
 *   static_cast是编译时类型转换，比C风格转换更安全
 *
 * 使用示例：
 * @code
 * std::unordered_map<VirtualObjectType, std::unordered_set<std::string>, EnumClassHash> map;
 * @endcode
 */
struct EnumClassHash {
  /**
   * @brief 重载函数调用运算符
   * @tparam T 枚举类型
   * @param t 枚举值
   * @return size_t 哈希值
   */
  template <typename T>
  size_t operator()(T t) const {
    return static_cast<size_t>(t);
  }
};

/**
 * @class DecisionData
 * @brief 决策数据管理类
 *
 * 功能说明：
 * 决策数据类负责管理和处理预测障碍物数据
 * 主要功能：
 * 1. 障碍物分类：静态/动态/虚拟/实际障碍物
 * 2. 障碍物查询：按ID查询、按类型查询
 * 3. 虚拟障碍物创建：创建用于交通规则的虚拟障碍物
 *
 * 线程安全：
 * - 使用mutex保护障碍物map的访问
 * - 使用transaction_mutex保护虚拟障碍物的创建
 *
 * C++语法说明：
 * - class默认访问限定符为private
 * - public/private/protected访问限定符控制成员可见性
 * - std::vector<Obstacle*>：存储指向Obstacle的指针
 *   使用指针而非对象，避免拷贝开销
 * - std::list<std::unique_ptr<Obstacle>>：拥有所有权的智能指针列表
 *   unique_ptr独占所有权，销毁时自动释放资源
 * - std::unordered_map<K, V>：哈希表容器，查找效率O(1)
 * - std::mutex：互斥锁，用于线程同步
 */
class DecisionData {
 public:
  /**
   * @brief 构造函数
   * @param prediction_obstacles 预测障碍物数据
   * @param reference_line 参考线
   *
   * 功能说明：
   * 根据预测障碍物数据初始化决策数据
   * 将预测障碍物分类为静态/动态障碍物
   * 创建障碍物到各个分类列表的映射
   *
   * C++语法说明：
   * - initializer list：初始化列表: reference_line_(reference_line)
   *   用于初始化引用成员变量reference_line_
   * - prediction_obstacles.prediction_obstacle()：
   *   protobuf repeated字段的访问方法，返回容器引用
   */
  DecisionData(const prediction::PredictionObstacles& prediction_obstacles,
               const ReferenceLine& reference_line);

  /**
   * @brief 默认析构函数
   *
   * C++语法说明：
   * = default：显式声明使用编译器生成的析构函数
   * 编译器生成的析构函数会自动调用成员变量的析构函数
   * unique_ptr成员会自动释放资源
   */
  ~DecisionData() = default;

 public:
  /**
   * @brief 根据ID获取障碍物
   * @param id 障碍物ID
   * @return Obstacle* 障碍物指针，未找到返回nullptr
   *
   * C++语法说明：
   * - std::lock_guard<std::mutex>：RAII方式的互斥锁管理
   *   构造时加锁，析构时解锁，自动处理异常情况
   * - FindPtrOrNull：Apollo常用工具函数
   *   在map中查找键，返回对应值或nullptr
   */
  Obstacle* GetObstacleById(const std::string& id);

  /**
   * @brief 根据类型获取障碍物列表
   * @param type 虚拟对象类型
   * @return std::vector<Obstacle*> 障碍物指针列表
   *
   * 功能说明：
   * 从virtual_obstacle_id_map_中查找指定类型的所有障碍物ID
   * 再根据ID获取对应的障碍物指针
   */
  std::vector<Obstacle*> GetObstacleByType(const VirtualObjectType& type);

  /**
   * @brief 根据类型获取障碍物ID集合
   * @param type 虚拟对象类型
   * @return std::unordered_set<std::string> 障碍物ID集合
   */
  std::unordered_set<std::string> GetObstacleIdByType(
      const VirtualObjectType& type);

  /**
   * @brief 获取所有静态障碍物
   * @return const std::vector<Obstacle*>& 静态障碍物引用
   *
   * C++语法说明：
   * - const std::vector<Obstacle*>&：常量引用
   *   返回引用避免拷贝开销
   *   const保证调用者不能修改容器内容
   * - const后缀：成员函数承诺不修改成员变量
   */
  const std::vector<Obstacle*>& GetStaticObstacle() const;

  /**
   * @brief 获取所有动态障碍物
   * @return const std::vector<Obstacle*>& 动态障碍物引用
   */
  const std::vector<Obstacle*>& GetDynamicObstacle() const;

  /**
   * @brief 获取所有虚拟障碍物
   * @return const std::vector<Obstacle*>& 虚拟障碍物引用
   */
  const std::vector<Obstacle*>& GetVirtualObstacle() const;

  /**
   * @brief 获取所有实际障碍物（虚拟障碍物之外的）
   * @return const std::vector<Obstacle*>& 实际障碍物引用
   */
  const std::vector<Obstacle*>& GetPracticalObstacle() const;

  /**
   * @brief 获取所有障碍物
   * @return const std::vector<Obstacle*>& 所有障碍物引用
   */
  const std::vector<Obstacle*>& GetAllObstacle() const;

 public:
  /**
   * @brief 根据参考点创建虚拟障碍物
   * @param point 参考点
   * @param type 虚拟对象类型
   * @param id 输出参数，创建的障碍物ID
   * @return bool 是否创建成功
   *
   * 功能说明：
   * 1. 将参考点转换为SL坐标
   * 2. 创建虚拟障碍物的包围盒（Box2d）
   * 3. 调用CreateVirtualObstacle创建虚拟障碍物
   *
   * C++语法说明：
   * - std::string* const id：指向string的常量指针
   *   指针本身不可变，但可以修改指针指向的内容
   */
  bool CreateVirtualObstacle(const ReferencePoint& point,
                             const VirtualObjectType& type,
                             std::string* const id);

  /**
   * @brief 根据S坐标创建虚拟障碍物
   * @param point_s 沿参考线的S坐标
   * @param type 虚拟对象类型
   * @param id 输出参数，创建的障碍物ID
   * @return bool 是否创建成功
   */
  bool CreateVirtualObstacle(const double point_s,
                             const VirtualObjectType& type,
                             std::string* const id);

 private:
  /**
   * @brief 验证轨迹点是否有效
   * @param point 轨迹点
   * @return bool 是否有效
   *
   * 功能说明：
   * 检查轨迹点的所有数值字段是否为NaN
   * 包括位置(x,y,z)、曲率(kappa)、曲率变化率(dkappa/ddkappa)、
   * 速度(v)、加速度(a)、相对时间(relative_time)
   *
   * C++语法说明：
   * - std::isnan()：检查浮点数是否为NaN
   *   如果是NaN返回true
   * - || 逻辑或：短路求值，任一条件为true则整体为true
   * - !point.has_path_point()：检查是否有路径点
   *   protobuf的has_前缀方法检查字段是否已设置
   */
  bool IsValidTrajectoryPoint(const common::TrajectoryPoint& point);

  /**
   * @brief 验证轨迹是否有效
   * @param trajectory 轨迹
   * @return bool 是否有效
   *
   * 功能说明：
   * 遍历轨迹的所有点，验证每个点是否有效
   * 只要有一个点无效，轨迹就无效
   */
  bool IsValidTrajectory(const prediction::Trajectory& trajectory);

  /**
   * @brief 根据包围盒创建虚拟障碍物（内部实现）
   * @param obstacle_box 障碍物包围盒
   * @param type 虚拟对象类型
   * @param id 输出参数，创建的障碍物ID
   * @return bool 是否创建成功
   *
   * C++语法说明：
   * - std::lock_guard<std::mutex>：双重锁保护
   *   transaction_lock先锁，lock后锁，保证创建操作的原子性
   * - mutable_position()：protobuf的mutable方法，返回可变指针
   * - add_polygon_point()：protobuf的add方法，向repeated字段添加元素
   * - absl::StrCat：Abseil库的字符串拼接函数
   */
  bool CreateVirtualObstacle(const common::math::Box2d& obstacle_box,
                             const VirtualObjectType& type,
                             std::string* const id);

 private:
  /**
   * @brief 静态障碍物列表
   *
   * 功能说明：
   * 不移动的障碍物，如停车车辆、基础设施等
   * 没有预测轨迹的障碍物也被视为静态障碍物
   *
   * C++语法说明：
   * std::vector<Obstacle*>：存储指向Obstacle对象的指针
   * 指针列表，不拥有对象所有权
   */
  std::vector<Obstacle*> static_obstacle_;

  /**
   * @brief 动态障碍物列表
   *
   * 功能说明：
   * 会移动的障碍物，如行驶中的车辆、行人等
   * 有有效预测轨迹的障碍物被视为动态障碍物
   */
  std::vector<Obstacle*> dynamic_obstacle_;

  /**
   * @brief 虚拟障碍物列表
   *
   * 功能说明：
   * 由交通规则生成的虚拟障碍物
   * 如：停车线、让行线、红灯等
   */
  std::vector<Obstacle*> virtual_obstacle_;

  /**
   * @brief 实际障碍物列表
   *
   * 功能说明：
   * 虚拟障碍物之外的障碍物
   * 即真实感知到的障碍物
   */
  std::vector<Obstacle*> practical_obstacle_;

  /**
   * @brief 所有障碍物列表
   *
   * 功能说明：
   * 包含所有障碍物：静态+动态+虚拟
   */
  std::vector<Obstacle*> all_obstacle_;

 private:
  /**
   * @brief 参考线引用
   *
   * C++语法说明：
   * const ReferenceLine&：常量引用
   * 引用必须初始化，且不可重新绑定
   * const保证参考线不会被修改
   */
  const ReferenceLine& reference_line_;

  /**
   * @brief 障碍物所有权列表
   *
   * 功能说明：
   * 存储所有障碍物的所有权
   * 使用unique_ptr确保障碍物被正确销毁
   *
   * C++语法说明：
   * - std::list：双向链表容器
   *   插入/删除效率高，迭代器不会因插入而失效
   * - std::unique_ptr<T>：独占所有权的智能指针
   *   不能复制，只能移动
   *   销毁时自动调用delete释放资源
   * - obstacles_.back().get()：
   *   back()返回最后一个元素的引用
   *   get()返回unique_ptr管理的原始指针
   */
  std::list<std::unique_ptr<Obstacle>> obstacles_;

  /**
   * @brief 障碍物ID到障碍物的映射
   *
   * 功能说明：
   * 用于O(1)时间复杂度的障碍物查找
   *
   * C++语法说明：
   * std::unordered_map<std::string, Obstacle*>：
   *   键为string类型的ID，值为Obstacle指针
   *   哈希表实现，查找效率高
   */
  std::unordered_map<std::string, Obstacle*> obstacle_map_;

  /**
   * @brief 虚拟障碍物类型到ID集合的映射
   *
   * 功能说明：
   * 按虚拟对象类型组织障碍物ID
   * 用于快速查找特定类型的所有虚拟障碍物
   *
   * C++语法说明：
   * std::unordered_map<VirtualObjectType, std::unordered_set<std::string>, EnumClassHash>：
   *   键为VirtualObjectType枚举类型
   *   值为一组string类型的ID集合
   *   EnumClassHash提供哈希函数
   */
  std::unordered_map<VirtualObjectType, std::unordered_set<std::string>,
                     EnumClassHash>
      virtual_obstacle_id_map_;

  /**
   * @brief 障碍物查询的互斥锁
   *
   * 功能说明：
   * 保护obstacle_map_的并发访问
   *
   * C++语法说明：
   * std::mutex：独占互斥锁
   *   lock()加锁，unlock()解锁
   *   通常配合std::lock_guard使用
   */
  std::mutex mutex_;

  /**
   * @brief 虚拟障碍物创建的互斥锁
   *
   * 功能说明：
   * 保护虚拟障碍物创建操作的原子性
   * 包括virtual_obstacle_列表和virtual_obstacle_id_map_的更新
   */
  std::mutex transaction_mutex_;
};

}  // namespace planning
}  // namespace apollo
