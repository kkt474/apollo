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
 * @file lattice_planner.h
 * @brief Lattice规划器类声明头文件
 *
 * 功能说明：
 * 该文件定义了Lattice规划器类
 * Lattice规划器是基于Frenet坐标系的解耦规划算法
 * 将三维规划问题分解为纵向和横向两个一维问题
 *
 * 设计特点：
 * - 继承自PlannerWithReferenceLine类
 * - 使用Frenet坐标系进行规划
 * - 支持多参考线规划
 * - 支持插件化部署（Cyber RT插件机制）
 *
 * C++语法说明：
 * - #pragma once：编译指示符，防止头文件重复包含
 * - class LatticePlanner : public PlannerWithReferenceLine：
 *   公有继承，规划器类层次结构
 * - virtual ~LatticePlanner() = default：
 *   虚析构函数，默认实现
 * - override关键字：显式声明覆盖基类虚函数
 * - CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN：Cyber RT插件注册宏
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
 * - #include <memory>：智能指针相关
 *   - std::shared_ptr：引用计数智能指针
 *   - std::make_shared：创建shared_ptr的工厂函数
 *
 * - #include <string>：字符串类
 *   - std::string：标准库字符串类型
 */
#include <memory>
#include <string>

/**
 * @brief Protobuf消息头文件
 *
 * C++语法说明：
 * planning_config.pb.h由planning_config.proto编译生成
 * 包含规划器的配置参数定义
 */
#include "modules/planning/planning_base/proto/planning_config.pb.h"

/**
 * @brief Cyber RT插件管理头文件
 *
 * C++语法说明：
 * cyber/是Apollo Cyber RT框架的核心模块
 * - plugin_manager：插件管理系统
 * - PluginManager：管理插件的加载和卸载
 *
 * Cyber RT是Apollo自主研发的实时计算框架
 * 支持插件化部署，动态加载规划器等模块
 */
#include "cyber/plugin_manager/plugin_manager.h"

/**
 * @brief Apollo状态管理头文件
 *
 * C++语法说明：
 * modules/common/status/status.h
 * - apollo::common::Status：操作状态类
 *   - OK()：返回成功状态
 *   - Status(ErrorCode, msg)：构造错误状态
 * - ErrorCode：错误码枚举
 */
#include "modules/common/status/status.h"

/**
 * @brief 规划基础数据头文件
 *
 * C++语法说明：
 * Frame类包含当前规划帧的所有数据
 * - 车辆状态
 * - 参考线信息
 * - 障碍物信息
 * - 轨迹输出
 */
#include "modules/planning/planning_base/common/frame.h"

/**
 * @brief 参考线信息头文件
 *
 * C++语法说明：
 * ReferenceLineInfo类包含单条参考线的规划结果
 * - 参考线本身
 * - 路径数据
 * - 速度数据
 * - 决策信息
 */
#include "modules/planning/planning_base/common/reference_line_info.h"

/**
 * @brief 规划器基类头文件
 *
 * C++语法说明：
 * Planner是所有规划器的基类
 * PlannerWithReferenceLine是支持参考线的规划器基类
 * 定义了规划器的接口：
 * - Init()：初始化
 * - Plan()：主规划函数
 * - Stop()：停止
 */
#include "modules/planning/planning_interface_base/planner_base/planner.h"

namespace apollo {
/**
 * @brief Apollo外层命名空间
 *
 * C++语法说明：
 * namespace关键字用于声明命名空间
 * 所有Apollo相关代码都位于apollo命名空间下
 */
namespace planning {

/**
 * @class LatticePlanner
 * @brief Lattice规划器类
 *
 * 功能说明：
 * Lattice规划器是基于Frenet坐标系的解耦规划算法
 * 采用纵向-横向分离的策略进行轨迹规划
 *
 * 继承关系：
 * LatticePlanner -> PlannerWithReferenceLine -> Planner
 *
 * 设计特点：
 * 1. 基于Frenet坐标系，适合道路场景
 * 2. 纵向规划：速度曲线优化
 * 3. 横向规划：路径曲线优化
 * 4. 通过轨迹评估器选择最优轨迹对
 * 5. 支持约束检查和碰撞检测
 *
 * C++语法说明：
 * - : public PlannerWithReferenceLine：
 *   公有继承，LatticePlanner is-a PlannerWithReferenceLine
 *   可以调用父类的所有公有成员函数
 *
 * - virtual ~LatticePlanner() = default：
 *   虚析构函数，允许基类指针删除派生类对象
 *   = default使用编译器生成的默认实现
 *   虚析构确保通过基类指针删除对象时正确调用派生类析构函数
 *
 * - override关键字（C++11）：
 *   显式声明覆盖基类虚函数
 *   如果基类中没有对应的虚函数，编译器报错
 *   提高代码可读性和安全性
 */
class LatticePlanner : public PlannerWithReferenceLine {
 public:
  /**
   * @brief 虚析构函数
   *
   * C++语法说明：
   * - virtual：
   *   虚函数，允许运行时多态
   *   通过基类指针删除派生类对象时调用正确的析构函数
   *
   * - ~LatticePlanner()：
   *   析构函数，对象销毁时自动调用
   *   释放LatticePlanner申请的资源
   *
   * - = default：
   *   默认实现，编译器生成
   *   LatticePlanner没有额外的资源需要释放
   */
  virtual ~LatticePlanner() = default;

  /**
   * @brief 获取规划器名称
   *
   * @return std::string 规划器名称字符串
   *
   * 功能说明：
   * 返回规划器的标识名称
   * 用于日志输出和调试
   *
   * C++语法说明：
   * - std::string Name() override：
   *   override显式声明覆盖基类虚函数
   *   返回std::string类型的名称
   *
   * - return "LATTICE"：
   *   返回常量字符串"LATTICE"
   *   用于标识这是Lattice规划器
   */
  std::string Name() override { return "LATTICE"; }

  /**
   * @brief 初始化规划器
   *
   * @param injector 依赖注入器指针
   * @param config_path 配置文件路径（可选）
   * @return common::Status 初始化状态
   *
   * 功能说明：
   * 初始化Lattice规划器
   * 加载配置参数，建立依赖关系
   *
   * C++语法说明：
   * - const std::shared_ptr<DependencyInjector>&：
   *   DependencyInjector是依赖注入器类
   *   shared_ptr智能指针管理对象生命周期
   *   const引用避免拷贝
   *
   * - const std::string& config_path = ""：
   *   字符串引用参数
   *   = ""默认参数值为空字符串
   *   可选参数，简化调用
   *
   * - override：
   *   显式声明覆盖基类虚函数
   *
   * - return Planner::Init(injector, config_path)：
   *   调用基类的Init方法
   *   ::作用域解析运算符访问基类成员
   */
  common::Status Init(const std::shared_ptr<DependencyInjector>& injector,
                      const std::string& config_path = "") override {
    return Planner::Init(injector, config_path);
  }

  /**
   * @brief 停止规划器
   *
   * 功能说明：
   * 停止规划器的运行
   * 清理资源，保存状态
   *
   * C++语法说明：
   * - void Stop() override：
   *   无返回值的成员函数
   *   override显式声明覆盖
   *
   * - {}：
   *   空函数体，LatticePlanner当前不需要执行停止操作
   *   预留接口，子类可重写
   */
  void Stop() override {}

  /**
   * @brief 主规划函数 - 多参考线版本
   *
   * @param planning_init_point 规划起始点
   * @param frame 当前规划帧
   * @param ptr_computed_trajectory 输出参数：计算出的轨迹
   * @return common::Status 规划状态
   *
   * 功能说明：
   * 对所有参考线进行规划
   * 返回成功的规划数量
   *
   * 重写说明：
   * 此函数重写了基类Planner的Plan虚函数
   * 是Lattice规划器的主入口
   *
   * C++语法说明：
   * - const common::TrajectoryPoint&：
   *   TrajectoryPoint包含位置、速度、加速度等信息
   *   const引用避免拷贝
   *
   * - Frame*：
   *   裸指针指向Frame对象
   *   Frame包含当前帧的所有规划数据
   *
   * - ADCTrajectory* ptr_computed_trajectory：
   *   ptr_前缀表示这是输出参数
   *   指向轨迹对象的指针
   *
   * - override：
   *   显式声明覆盖基类虚函数
   */
  /**
   * @brief 主规划函数声明
   *
   * @details
   * 该函数遍历所有参考线进行规划
   * 如果任何参考线规划成功，返回OK状态
   *
   * 参数说明：
   * - planning_init_point：车辆当前位置和状态
   * - frame：包含所有参考线和障碍物信息
   * - ptr_computed_trajectory：输出的轨迹数据
   */
  common::Status Plan(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame,
                      ADCTrajectory* ptr_computed_trajectory) override;

  /**
   * @brief 单参考线规划函数
   *
   * @param planning_init_point 规划起始点
   * @param frame 当前规划帧
   * @param reference_line_info 参考线信息
   * @return common::Status 规划状态
   *
   * 功能说明：
   * 在单条参考线上进行Lattice规划
   * 这是Lattice规划的核心算法实现
   *
   * 重写说明：
   * 此函数重写了基类PlannerWithReferenceLine的PlanOnReferenceLine虚函数
   *
   * 算法流程（详见实现文件）：
   * 1. 获取参考线并转换为PathPoint格式
   * 2. 匹配初始点到参考线
   * 3. 转换为Frenet坐标系初始状态
   * 4. 解析决策获取规划目标
   * 5. 生成纵向和横向1维轨迹束
   * 6. 评估轨迹对的可行性和成本
   * 7. 选择最优无碰撞轨迹
   *
   * C++语法说明：
   * - ReferenceLineInfo*：
   *   裸指针指向参考线信息
   *   函数内部可能修改其内容
   */
  /**
   * @brief 单参考线规划函数声明
   *
   * @details
   * 该函数实现Lattice规划的核心算法
   * 采用Frenet坐标系下的纵向-横向解耦规划
   *
   * 参数说明：
   * - planning_init_point：规划起始点的位姿和速度信息
   * - frame：当前帧的完整数据
   * - reference_line_info：单条参考线的规划结果输出
   */
  common::Status PlanOnReferenceLine(
      const common::TrajectoryPoint& planning_init_point, Frame* frame,
      ReferenceLineInfo* reference_line_info) override;
};

/**
 * @brief Cyber RT插件注册宏
 *
 * 功能说明：
 * 将LatticePlanner注册为Cyber RT插件
 * 允许运行时动态加载此规划器
 *
 * C++语法说明：
 * - CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN：
 *   Apollo定义的宏，用于插件注册
 *   这是Cyber RT框架的插件机制
 *
 * - apollo::planning::LatticePlanner：
 *   完全限定名指定要注册的类
 *
 * - PlannerWithReferenceLine：
 *   指定插件的基类接口
 *   框架通过此接口与插件交互
 *
 * 使用说明：
 * 当Cyber RT启动时，会自动加载此插件
 * 允许在配置文件中指定使用Lattice规划器
 */
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::LatticePlanner,
                                     PlannerWithReferenceLine)

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
