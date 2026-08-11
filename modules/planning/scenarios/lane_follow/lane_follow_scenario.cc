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
 * @file
 * @brief 车道跟随场景实现文件
 *
 * 本文件实现LaneFollowScenario类，是车道跟随驾驶场景的核心实现。
 * 车道跟随是自动驾驶最基本的场景之一，车辆沿车道中心线行驶。
 *
 * 主要功能：
 * 1. 判断是否可以切换到车道跟随场景
 * 2. 管理车道跟随场景的生命周期
 *
 * 设计特点：
 * - 继承自Scenario基类
 * - 作为默认场景，通常是自动驾驶的主要行驶模式
 * - 可与其他场景（换道、停车等）进行切换
 *
 * C++语法说明：
 * - const Scenario*: 常量指针，不能修改指向的对象
 * - const Frame&: 常量引用，避免拷贝
 * - if (!ptr): 指针隐式转换为bool检查空值
 * - has_xxx(): protobuf的has方法，检查字段是否存在
 * - .empty(): 容器方法，检查是否为空
 */
#include "modules/planning/scenarios/lane_follow/lane_follow_scenario.h"

#include "cyber/common/log.h"  /**< Cyber RT日志系统 */
#include "modules/planning/scenarios/lane_follow/lane_follow_stage.h" /**< 车道跟随阶段 */

namespace apollo {
/**
 * apollo:: - Apollo最外层命名空间
 */
namespace planning {

/**
 * @brief 判断是否可以切换到车道跟随场景
 *
 * 判断当前帧数据是否满足切换到车道跟随场景的条件。
 * 这是场景转换的入口点。
 *
 * 切换条件：
 * 1. 规划命令包含车道跟随命令
 * 2. 参考线信息非空
 * 3. 其他场景为空（首次进入）或者其他场景可以转换
 *
 * @param other_scenario 当前正在运行的其他场景指针（可能为空）
 * @param frame 当前规划帧数据
 * @return bool 是否可以切换到车道跟随场景
 *
 * 语法说明：
 * - const Scenario*: 常量原始指针
 *   - const修饰指针指向的数据不能修改
 *   - 但指针本身可以改变指向
 * - const Frame&: 常量引用
 *   - 函数内只读，不会拷贝Frame对象
 *   - Frame是规划模块的核心数据结构，包含所有输入输出
 * - return false/true: 布尔返回值
 * - if (!condition): if语句，条件为true时执行
 * - ptr->method(): 通过指针调用方法
 * - .empty(): 容器方法，返回容器是否为空
 */
bool LaneFollowScenario::IsTransferable(const Scenario* other_scenario,
                                        const Frame& frame) {
  /**
   * 检查条件1：是否有车道跟随命令
   *
   * frame.local_view().planning_command->has_lane_follow_command()
   *   - frame: 当前规划帧
   *   - .local_view(): 获取本地视图（输入数据）
   *   - .planning_command: 获取规划命令（路由命令）
   *   - ->: 指针访问成员
   *   - has_lane_follow_command(): protobuf方法
   *     检查lane_follow_command字段是否存在
   *
   * if (!has_lane_follow_command()) 如果没有车道跟随命令
   *   return false: 不能切换到车道跟随场景
   */
  if (!frame.local_view().planning_command->has_lane_follow_command()) {
    return false;  /**< 没有车道跟随命令，返回false */
  }

  /**
   * 检查条件2：参考线信息是否非空
   *
   * frame.reference_line_info().empty()
   *   - frame.reference_line_info(): 获取参考线信息列表
   *   - .empty(): 检查列表是否为空
   *
   * 参考线信息用于描述可行驶路径，没有参考线则无法规划
   */
  if (frame.reference_line_info().empty()) {
    return false;  /**< 参考线为空，返回false */
  }

  /**
   * 检查条件3：其他场景是否为空
   *
   * if (other_scenario == nullptr)
   *   - other_scenario: 指向其他场景的指针
   *   - == nullptr: 与空指针比较
   *   - 如果为空，说明当前没有场景在运行
   *
   * 这种情况可以直接切换到车道跟随场景
   */
  if (other_scenario == nullptr) {
    return true;  /**< 无其他场景，可以切换 */
  }

  /**
   * 默认情况：允许切换到车道跟随场景
   *
   * return true
   *   - 如果其他场景存在且以上检查都通过
   *   - 允许从其他场景切换到车道跟随场景
   */
  return true;  /**< 允许切换 */
}

}  // namespace planning
}  // namespace apollo
