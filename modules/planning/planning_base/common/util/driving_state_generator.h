/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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
 * @file driving_state_generator.h
 * @brief 驾驶状态生成器头文件
 *
 * 功能说明：
 * 该文件定义了驾驶状态生成器类，用于生成自车的驾驶状态信息
 * 状态信息包括：车道ID、车道内位置、目的地距离、是否在路口、
 * 路径信息、车道信息、绕行信息等
 *
 * C++语法说明：
 * - #pragma once：编译指示符，确保头文件只被包含一次
 *   这是现代C++中常用的头文件保护方式，比传统的#ifndef宏保护更简洁
 *
 * - #include <list>：STL容器头文件
 *   std::list是双向链表容器，但在此文件中实际未直接使用
 *   可能为未来扩展预留
 */

#pragma once

#include <list>

#include "modules/common_msgs/planning_msgs/planning_internal.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/planning_base/common/ego_info.h"
#include "modules/planning/planning_base/common/obstacle_blocking_analyzer.h"
#include "modules/planning/planning_base/common/reference_line_info.h"
#include "modules/planning/planning_base/reference_line/reference_line.h"

namespace apollo {
/**
 * @brief Apollo命名空间
 *
 * 功能说明：
 * Apollo项目所有代码都位于apollo命名空间下
 * 这是一个嵌套命名空间，外层为apollo，内层为planning
 *
 * C++语法说明：
 * - 命名空间(namespace)：用于组织代码，避免命名冲突
 * - 嵌套命名空间：这是C++17之前的写法，C++17支持嵌套命名空间语法
 */
namespace planning {

/**
 * @class DrivingStateGenerator
 * @brief 驾驶状态生成器类
 *
 * 功能说明：
 * 驾驶状态生成器负责收集和生成自车的当前驾驶状态
 * 这些状态信息用于日志记录、数据采集、仿真等场景
 *
 * 主要功能：
 * 1. generate()：生成完整的驾驶状态
 * 2. add_path_info()：添加路径信息（车道变化、车道借用）
 * 3. add_lane_info()：添加车道信息（当前转弯、下一转弯）
 * 4. add_nudge_info()：添加绕行信息（障碍物绕行决策）
 *
 * 使用示例：
 * @code
 * DrivingStateGenerator generator;
 * planning_internal::ADCDrivingState state;
 * generator.generate(reference_line_info, ego_info, &state);
 * @endcode
 *
 * C++语法说明：
 * - class类声明：默认访问限定符为private
 * - public成员函数：提供对外接口
 * - const关键字：表示函数不会修改成员变量
 * - 指向常量的指针参数：const ReferenceLineInfo* 表示指针指向的内容不可变
 * - 指向常量的指针的指针：planning_internal::ADCDrivingState* const state
 *   表示指针本身不可变（即不能修改state的指向）
 */
class DrivingStateGenerator {
 public:
  /**
   * @brief 生成完整的驾驶状态信息
   *
   * @param reference_line_info 参考线信息指针（可为nullptr）
   * @param ego_info 自车信息指针（不可为nullptr）
   * @param state 输出参数，驾驶状态结果
   *
   * 功能说明：
   * 这是主入口函数，调用其他辅助函数生成完整的驾驶状态
   * 包括车道ID、车道内位置、目的地距离、是否在路口、
   * 路径信息、车道信息、绕行信息等
   *
   * C++语法说明：
   * - const ReferenceLineInfo*：指向常量的指针
   *   可以传入nullptr，也可以在函数内检查是否为nullptr
   *
   * - const EgoInfo*：指向常量的指针
   *   函数内部假设ego_info不为nullptr，不会检查其有效性
   *
   * - planning_internal::ADCDrivingState* const state：
   *   指向状态对象的常量指针（指针本身不可变）
   *   使用const表示这是输出参数，函数会修改state指向的内容
   *
   * - nullptr是C++11引入的关键字，表示空指针
   *   相比NULL（通常是#define NULL 0），nullptr类型安全
   */
  void generate(const ReferenceLineInfo* reference_line_info,
                const EgoInfo* ego_info,
                planning_internal::ADCDrivingState* const state);

  /**
   * @brief 添加路径信息到驾驶状态
   *
   * @param reference_line_info 参考线信息指针
   * @param state 输出参数，驾驶状态结果
   *
   * 功能说明：
   * 分析并添加当前路径的相关信息：
   * 1. 车道变化类型：无变化/向左变道/向右变道
   * 2. 车道借用类型：无借用/借左道/借右道
   * 3. 是否为fallback路径
   *
   * C++语法说明：
   * - routing::ChangeLaneType::LEFT/RIGHT：
   *   枚举类型成员访问，ChangeLaneType是routing命名空间中的枚举
   *   使用::作用域解析运算符访问枚举值
   *
   * - std::string::npos：
   *   string类的静态成员常量，表示"未找到"
   *   find()函数返回npos表示搜索失败
   */
  void add_path_info(const ReferenceLineInfo* reference_line_info,
                     planning_internal::ADCDrivingState* const state);

  /**
   * @brief 添加车道信息到驾驶状态
   *
   * @param reference_line_info 参考线信息指针
   * @param state 输出参数，驾驶状态结果
   *
   * 功能说明：
   * 分析并添加车道相关信息：
   * 1. 当前所在路段的转弯类型（直行/左转/右转/U型转弯）
   * 2. 下一路段的转弯类型及距离
   *
   * 算法流程：
   * 1. 计算自车在路由路径上的位置（s坐标）
   * 2. 遍历所有车道段，找到包含自车的段
   * 3. 设置当前转弯信息
   * 4. 检查下一个转弯（如果是左转/右转/U型转弯）
   *
   * C++语法说明：
   * - for (const auto& seg : reference_line_info->Lanes())：
   *   范围for循环，遍历Lanes返回的所有车道段
   *   const auto&表示常量引用，避免拷贝
   *
   * - hdmap::Lane::LEFT_TURN等：
   *   枚举类型，Lane命名空间下的转弯类型枚举
   */
  void add_lane_info(const ReferenceLineInfo* reference_line_info,
                     planning_internal::ADCDrivingState* const state);

  /**
   * @brief 添加绕行信息到驾驶状态
   *
   * @param reference_line_info 参考线信息指针
   * @param state 输出参数，驾驶状态结果
   *
   * 功能说明：
   * 分析并添加障碍物绕行决策信息：
   * 1. 遍历所有SL多边形障碍物
   * 2. 判断每个障碍物的绕行类型（左绕/右绕/阻塞）
   * 3. 计算最近绕行障碍物的距离
   * 4. 判断是否处于绕行状态
   *
   * C++语法说明：
   * - SLPolygon::IGNORE/UNDEFINED/LEFT_NUDGE等：
   *   枚举类型，SLPolygon类的绕行信息枚举
   *
   * - mutable关键字：
   *   在const成员函数中，mutable成员可以被修改
   *   mutable_next_turn_info()返回的对象可以修改
   *
   * - std::abs()：
   *   标准库绝对值函数，处理浮点数时建议使用fabs()
   *   此处可能应该使用fabs()
   */
  void add_nudge_info(const ReferenceLineInfo* reference_line_info,
                     planning_internal::ADCDrivingState* const state);
};

}  // namespace planning
}  // namespace apollo
