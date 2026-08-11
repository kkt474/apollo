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
 * @file prediction_querier.h
 * @brief 预测查询器类声明头文件
 *
 * 功能说明：
 * 预测查询器类用于访问障碍物的预测轨迹信息
 * 主要功能：
 * 1. 存储和查询障碍物列表
 * 2. 根据ID查找障碍物
 * 3. 计算障碍物沿参考线的投影速度
 *
 * 设计特点：
 * - 使用map存储障碍物，支持O(1)查找
 * - 同时维护vector保持障碍物插入顺序
 * - 使用shared_ptr管理参考线生命周期
 * - 虚析构函数支持派生类
 *
 * C++语法说明：
 * - #pragma once：编译指示符，防止头文件重复包含
 * - class PredictionQuerier：预测查询器类
 * - std::unordered_map：基于哈希表的关联容器
 * - std::shared_ptr：引用计数智能指针
 * - virtual ~PredictionQuerier() = default：虚析构函数
 */

/**
 * @brief 确保头文件只被包含一次
 *
 * C++语法说明：
 * #pragma once是编译指示符
 * 告诉编译器这个头文件只处理一次
 * 作用类似于传统的#ifndef宏保护
 * 优点：更简洁，编译器直接处理
 */
#pragma once

/**
 * @brief 标准库头文件
 *
 * C++语法说明：
 * - #include <memory>：
 *   智能指针相关
 *   - std::shared_ptr：引用计数智能指针
 *   - std::make_shared：创建shared_ptr的工厂函数
 *
 * - #include <string>：
 *   标准库字符串类型
 *
 * - #include <unordered_map>：
 *   基于哈希表的关联容器
 *   - unordered_map：键值对容器
 *   - 查找时间复杂度O(1)
 *
 * - #include <vector>：
 *   动态数组容器
 */
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

/**
 * @brief Apollo障碍物头文件
 *
 * C++语法说明：
 * obstacle.h定义了Obstacle类
 * 包含障碍物的位置、速度、预测轨迹等信息
 */
#include "modules/planning/planning_base/common/obstacle.h"

namespace apollo {
/**
 * @brief Apollo外层命名空间
 */
namespace planning {

/**
 * @class PredictionQuerier
 * @brief 预测查询器类
 *
 * 功能说明：
 * 预测查询器用于访问障碍物的预测轨迹信息
 * Lattice规划器使用此类查询障碍物数据
 *
 * 使用场景：
 * - 获取所有障碍物列表
 * - 根据ID查找特定障碍物
 * - 计算障碍物在参考线方向上的投影速度
 *
 * 成员变量说明：
 * - id_obstacle_map_：障碍物ID到障碍物指针的映射
 * - obstacles_：保持插入顺序的障碍物列表
 * - ptr_reference_line_：参考线路径点智能指针
 *
 * C++语法说明：
 * - class PredictionQuerier：
 *   类声明，默认访问限定符为private
 *
 * - public: / private:：
 *   访问限定符
 *   public：成员可以从类外部访问
 *   private：成员只能在类内部访问
 */
class PredictionQuerier {
 public:
  /**
   * @brief 构造函数
   *
   * @param obstacles 障碍物指针向量
   * @param ptr_reference_line 参考线路径点共享指针
   *
   * 功能说明：
   * 1. 初始化参考线指针
   * 2. 遍历障碍物列表并插入map（去重）
   * 3. 同时维护vector保持顺序
   *
   * C++语法说明：
   * - const std::vector<const Obstacle*>&：
   *   常量引用输入参数
   *   vector存储指向const Obstacle的指针
   *   使用指针而非对象引用，避免拷贝
   *
   * - const std::shared_ptr<std::vector<common::PathPoint>>&：
   *   shared_ptr智能指针的常量引用
   *   共享参考线所有权
   */
  PredictionQuerier(const std::vector<const Obstacle*>& obstacles,
                    const std::shared_ptr<std::vector<common::PathPoint>>&
                        ptr_reference_line);

  /**
   * @brief 虚析构函数
   *
   * C++语法说明：
   * - virtual ~PredictionQuerier() = default：
   *   virtual：虚函数，支持运行时多态
   *   = default：使用编译器生成的默认实现
   *
   * 设计意图：
   * 虚析构函数确保通过基类指针删除派生类对象时
   * 能够正确调用派生类的析构函数
   * 虽然当前没有派生类，但为扩展性预留
   */
  virtual ~PredictionQuerier() = default;

  /**
   * @brief 获取所有障碍物
   *
   * @return std::vector<const Obstacle*> 障碍物指针向量
   *
   * 功能说明：
   * 返回存储的所有障碍物列表
   *
   * C++语法说明：
   * - std::vector<const Obstacle*>：
   *   vector存储常量指针
   *   返回值拷贝，调用者获得副本
   *
   * - const后缀：
   *   成员函数承诺不修改成员变量
   */
  std::vector<const Obstacle*> GetObstacles() const;

  /**
   * @brief 计算障碍物沿参考线的投影速度
   *
   * @param obstacle_id 障碍物ID
   * @param s 沿参考线的距离坐标
   * @param t 预测时间
   * @return double 投影速度（米/秒）
   *
   * 功能说明：
   * 计算障碍物在指定时刻t、沿参考线s位置处的速度
   * 在参考线方向上的投影分量
   *
   * 算法流程（详见实现文件）：
   * 1. 根据ID查找障碍物轨迹
   * 2. 使用二分查找找到t时刻对应的轨迹点
   * 3. 提取速度大小和朝向
   * 4. 分解为x和y分量
   * 5. 计算沿参考线方向的投影速度
   *
   * C++语法说明：
   * - const std::string&：
   *   常量引用，障碍物ID
   *
   * - const double s, const double t：
   *   const值参数，承诺不修改
   *
   * - const后缀：
   *   成员函数承诺不修改成员变量
   */
  double ProjectVelocityAlongReferenceLine(const std::string& obstacle_id,
                                           const double s,
                                           const double t) const;

 private:
  /**
   * @brief 障碍物ID到障碍物的映射
   *
   * 功能说明：
   * 使用unordered_map存储障碍物
   * 支持O(1)时间复杂度的ID查找
   *
   * C++语法说明：
   * - std::unordered_map<std::string, const Obstacle*>：
   *   键为std::string类型（障碍物ID）
   *   值为指向const Obstacle的指针
   *   unordered_map使用哈希表实现，查找效率高
   *
   * - id_obstacle_map_：
   *   下划线后缀是Apollo的命名约定
   *   表示这是类的成员变量
   */
  std::unordered_map<std::string, const Obstacle*> id_obstacle_map_;

  /**
   * @brief 障碍物列表（保持插入顺序）
   *
   * 功能说明：
   * 使用vector存储障碍物指针
   * 保持障碍物的插入顺序
   * 用于需要按顺序遍历障碍物的场景
   *
   * C++语法说明：
   * - std::vector<const Obstacle*>：
   *   vector存储常量指针
   *   支持随机访问和顺序遍历
   */
  std::vector<const Obstacle*> obstacles_;

  /**
   * @brief 参考线路径点智能指针
   *
   * 功能说明：
   * 使用shared_ptr管理参考线的生命周期
   * 多个对象可以共享参考线所有权
   *
   * C++语法说明：
   * - std::shared_ptr<std::vector<common::PathPoint>>：
   *   shared_ptr管理vector<PathPoint>的生命周期
   *   引用计数：最后一个shared_ptr销毁时释放资源
   *
   * - ptr_前缀：
   *   Apollo的命名约定
   *   表示这是指针类型的成员变量
   */
  std::shared_ptr<std::vector<common::PathPoint>> ptr_reference_line_;
};

/**
 * @brief 命名空间结束标记
 *
 * C++语法说明：
 * // 注释用于说明命名空间结束
 * 两层命名空间的闭合：
 * }  // namespace planning
 * }  // namespace apollo
 */
}  // namespace planning
}  // namespace apollo
