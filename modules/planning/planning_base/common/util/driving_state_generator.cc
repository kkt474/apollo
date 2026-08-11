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
 * @file driving_state_generator.cc
 * @brief 驾驶状态生成器实现文件
 *
 * 功能说明：
 * 驾驶状态生成器负责收集和生成自车的当前驾驶状态
 * 这些状态信息用于日志记录、数据采集、仿真等场景
 *
 * 主要功能模块：
 * 1. generate()：主入口函数，协调各子函数生成完整状态
 * 2. add_path_info()：添加路径信息（车道变化、车道借用）
 * 3. add_lane_info()：添加车道信息（当前转弯、下一转弯）
 * 4. add_nudge_info()：添加绕行信息（障碍物绕行决策）
 *
 * C++语法说明：
 * - #include：文件包含指令，用于引入头文件
 * - namespace：命名空间，用于组织代码和避免命名冲突
 * - ::作用域解析运算符：用于访问命名空间或类的成员
 */

#include "modules/planning/planning_base/common/util/driving_state_generator.h"

#include "modules/planning/planning_base/common/path/path_data.h"
#include "modules/planning/planning_base/common/util/util.h"

namespace apollo {
/**
 * @brief Apollo外层命名空间
 *
 * C++语法说明：
 * namespace关键字用于声明一个命名空间
 * 所有Apollo相关代码都包裹在apollo命名空间内
 */
namespace planning {
/**
 * @brief planning内层命名空间
 *
 * 嵌套命名空间结构：apollo::planning
 * planning命名空间包含所有规划模块相关的类和函数
 */

void DrivingStateGenerator::generate(
    const ReferenceLineInfo* reference_line_info, const EgoInfo* ego_info,
    planning_internal::ADCDrivingState* const state) {
  /**
   * @brief 生成完整的驾驶状态信息
   *
   * @param reference_line_info 参考线信息指针（可为nullptr）
   * @param ego_info 自车信息指针
   * @param state 输出参数，存储生成的驾驶状态
   *
   * 功能说明：
   * 这是主入口函数，整合多个信息源生成完整的驾驶状态
   *
   * 算法流程：
   * 1. 设置车道ID和车道内位置（如果ego_waypoint有有效的lane信息）
   * 2. 设置到目的地的距离
   * 3. 检查参考线信息是否为nullptr（如果是则报错返回）
   * 4. 判断是否在路口内
   * 5. 添加路径信息、车道信息、绕行信息
   *
   * C++语法说明：
   * - nullptr != ego_info->adc_waypoint().lane：
   *   nullptr比较可以写在比较表达式的任意一侧
   *   这种风格使条件更直观（"如果ego_info的adc_waypoint有有效lane"）
   *
   * - state->set_lane_id()：
   *   protobuf消息的setter方法，用于设置字段值
   *   返回void，设置失败时会通过其他机制（如CHECK宏）报告
   *
   * - ego_info->adc_waypoint().lane->id().id()：
   *   链式调用访问成员
   *   adc_waypoint()返回Waypoint对象，lane成员是shared_ptr
   *   ->id()返回LaneIdentifier对象，再调用id()获取字符串ID
   *
   * - ego_info->distance_to_destination()：
   *   EgoInfo类的方法，计算到目的地的距离
   *
   * - if (nullptr == reference_line_info)：
   *   空指针检查，如果reference_line_info为空
   *   AERROR宏记录错误日志，函数提前返回
   *
   * - AERROR << ...：
   *   Apollo的日志宏，输出ERROR级别日志
   *   <<运算符用于拼接日志内容（类似cout）
   *
   * - planning::util::CheckInsideJunction()：
   *   完全限定名调用
   *   planning::util命名空间中的工具函数
   *   判断参考线是否在路口内
   *
   * - return; vs return;（隐式返回）：
   *   void函数可以隐式返回（执行到函数末尾）
   *   也可以显式使用return;提前返回
   */
  if (nullptr != ego_info->adc_waypoint().lane) {
    /**
     * @brief 检查自车是否在有效车道上
     *
     * adc_waypoint包含自车当前位置对应的车道信息
     * 如果lane指针有效，说明自车在某个车道上
     *
     * C++语法说明：
     * - ego_info->adc_waypoint()：
     *   EgoInfo的adc_waypoint()方法返回Waypoint结构
     *   包含车道信息(lane)和位置信息(s)
     */
    state->set_lane_id(ego_info->adc_waypoint().lane->id().id());
    /**
     * @brief 设置车道内的位置
     *
     * s表示沿车道中心的累计距离
     */
    state->set_s_in_lane(ego_info->adc_waypoint().s);
  }

  /**
   * @brief 设置到目的地的距离
   *
   * EgoInfo提供的distance_to_destination()方法
   * 计算自车当前位置到最终目的地的距离
   */
  state->set_distance_to_destination(ego_info->distance_to_destination());

  /**
   * @brief 检查参考线信息有效性
   *
   * 如果reference_line_info为空，记录错误并返回
   * 后面的处理都依赖reference_line_info，所以必须有效
   */
  if (nullptr == reference_line_info) {
    AERROR << "DriveReferenceLineInfo is nullptr";
    return;
  }

  /**
   * @brief 判断自车是否在路口内
   *
   * 使用planning::util命名空间中的CheckInsideJunction函数
   * 该函数分析参考线信息判断是否处于路口区域
   */
  state->set_is_in_junction(
      planning::util::CheckInsideJunction(*reference_line_info));

  /**
   * @brief 添加详细的状态信息
   *
   * 调用三个辅助函数分别添加：
   * 1. 路径信息（车道变化、车道借用）
   * 2. 车道信息（转弯类型）
   * 3. 绕行信息（障碍物决策）
   */
  add_path_info(reference_line_info, state);
  add_lane_info(reference_line_info, state);
  add_nudge_info(reference_line_info, state);
}

void DrivingStateGenerator::add_path_info(
    const ReferenceLineInfo* reference_line_info,
    planning_internal::ADCDrivingState* const state) {
  /**
   * @brief 添加路径信息到驾驶状态
   *
   * @param reference_line_info 参考线信息指针
   * @param state 输出参数，存储路径信息
   *
   * 功能说明：
   * 分析当前路径的类型和特征，设置相应的状态标志
   *
   * 算法流程：
   * 1. 初始化车道变化类型和车道借用类型为"无"
   * 2. 获取当前路径数据
   * 3. 判断是否为变道路径：
   *    - 如果是变道路径，根据PreviousAction设置左/右变道
   *    - 否则，检查路径标签是否包含"left"/"right"设置借道类型
   * 4. 检查路径标签是否包含"fallback"设置fallback标志
   *
   * C++语法说明：
   * - state->set_lane_change_type()：
   *   protobuf的setter方法，设置枚举字段
   *   planning_internal::NO_CHANGE是枚举值
   *
   * - const PathData current_path_data = reference_line_info->path_data()：
   *   值拷贝获取当前路径数据
   *   PathData可能较大，但此函数需要读取其path_label
   *
   * - reference_line_info->IsChangeLanePath()：
   *   判断当前路径是否为变道路径的方法
   *   返回bool类型
   *
   * - std::string::npos：
   *   string类的静态常量，表示"未找到"
   *   find()返回npos表示子串不存在
   *
   * - std::string::find()：
   *   在字符串中查找子串，返回首次出现的位置
   *   参数是要查找的子串
   *
   * - routing::ChangeLaneType::LEFT/RIGHT：
   *   路由模块定义的枚举类型
   *   PreviousAction()返回上一次的变道动作
   *
   * - path_label()：
   *   PathData类的方法，返回路径的标签字符串
   *   标签格式如"lane_change_left"、"lane_borrow_right"等
   *
   * - state->set_is_fallback(true/false)：
   *   设置是否为fallback路径
   *   fallback路径是主路径规划失败时的后备路径
   */
  state->set_lane_change_type(planning_internal::NO_CHANGE);
  state->set_lane_borrow_type(planning_internal::NO_BORROW);

  /**
   * @brief 获取当前路径数据
   *
   * PathData包含路径的完整信息：路径点、长度、标签等
   */
  const PathData current_path_data = reference_line_info->path_data();

  /**
   * @brief 判断是否为变道路径
   *
   * 两种判断方式：
   * 1. IsChangeLanePath()方法直接判断
   * 2. path_label中包含"lane_change"字符串
   */
  if (reference_line_info->IsChangeLanePath() ||
      current_path_data.path_label().find("lane_change") != std::string::npos) {
    /**
     * @brief 处理变道路径
     *
     * 根据路由模块的PreviousAction判断变道方向
     * routing::ChangeLaneType::LEFT表示向左变道
     * routing::ChangeLaneType::RIGHT表示向右变道
     */
    if (reference_line_info->Lanes().PreviousAction() ==
        routing::ChangeLaneType::LEFT) {
      state->set_lane_change_type(planning_internal::LEFT_CHANGE);
    } else if (reference_line_info->Lanes().PreviousAction() ==
               routing::ChangeLaneType::RIGHT) {
      state->set_lane_change_type(planning_internal::RIGHT_CHANGE);
    }
  } else {
    /**
     * @brief 处理非变道路径（可能是借道路径）
     *
     * 检查path_label中是否包含"left"或"right"
     * 如果包含"left"表示借用左侧车道
     * 如果包含"right"表示借用右侧车道
     */
    if (current_path_data.path_label().find("left") != std::string::npos) {
      state->set_lane_borrow_type(planning_internal::LEFT_BORROW);
    } else if (current_path_data.path_label().find("right") !=
               std::string::npos) {
      state->set_lane_borrow_type(planning_internal::RIGHT_BORROW);
    }
  }

  /**
   * @brief 判断是否为fallback路径
   *
   * fallback路径是当主路径规划失败时使用的安全路径
   * 通常是将车停下的路径
   */
  if (current_path_data.path_label().find("fallback") != std::string::npos) {
    state->set_is_fallback(true);
  }
}

void DrivingStateGenerator::add_lane_info(
    const ReferenceLineInfo* reference_line_info,
    planning_internal::ADCDrivingState* const state) {
  /**
   * @brief 添加车道信息到驾驶状态
   *
   * @param reference_line_info 参考线信息指针
   * @param state 输出参数，存储车道信息
   *
   * 功能说明：
   * 分析当前自车所在车道及前方车道的转弯信息
   *
   * 算法流程：
   * 1. 计算自车在路由路径上的中心位置s
   * 2. 遍历所有车道段，累加计算每段的起止s值
   * 3. 找到包含自车的车道段，设置当前转弯信息
   * 4. 检查下一车道的转弯类型（如果是转弯）
   *
   * C++语法说明：
   * - (reference_line_info->AdcSlBoundary().start_s() +
   *   reference_line_info->AdcSlBoundary().end_s()) * 0.5：
   *   计算自车SL边界的中点作为自车在s方向的位置
   *   AdcSlBoundary返回自车在参考线坐标系下的边界
   *
   * - route_lane_start_s = route_lane_end_s：
   *   将上一个路段的结束s作为当前路段的起始s
   *   累计计算路由路径上的绝对位置
   *
   * - route_lane_end_s += seg.end_s - seg.start_s：
   *   累加当前路段的长度到route_lane_end_s
   *   seg是遍历中的当前车道段
   *
   * - if (route_lane_end_s < adc_s) { continue; }：
   *   如果当前路段还未到达自车位置，跳过
   *   continue跳过后续代码，开始下一次循环
   *
   * - if (route_lane_start_s < adc_s + 0.01)：
   *   检查自车是否在当前路段内（添加小量容差）
   *   0.01米的容差用于处理边界情况
   *
   * - state->set_cur_turn_info(seg.lane->lane().turn())：
   *   设置当前路段的转弯类型
   *   seg.lane->lane()获取Lane对象
   *   turn()返回Lane::TurnType枚举
   *
   * - hdmap::Lane::LEFT_TURN/RIGHT_TURN/U_TURN：
   *   hdmap命名空间中的转弯类型枚举
   *
   * - state->mutable_next_turn_info()->set_turn_type(turn_type)：
   *   mutable_next_turn_info()返回可变的消息指针
   *   用于设置嵌套消息字段
   *   这是protobuf C++ API的常见用法
   *
   * - state->mutable_next_turn_info()->set_dis_to_turn(route_lane_start_s - adc_s)：
   *   设置到下一转弯的距离
   *   即当前路段起点到自车位置的距离
   */
  double adc_s = (reference_line_info->AdcSlBoundary().start_s() +
                  reference_line_info->AdcSlBoundary().end_s()) *
                 0.5;
  double route_lane_start_s = 0.0;
  double route_lane_end_s = 0.0;

  /**
   * @brief 遍历所有车道段
   *
   * reference_line_info->Lanes()返回包含所有车道段的容器
   * 每个seg代表路由路径上的一个车道段
   */
  for (const auto& seg : reference_line_info->Lanes()) {
    /**
     * @brief 获取当前车道段的转弯类型
     *
     * seg.lane->lane().turn()返回hdmap::Lane::TurnType枚举
     * 可能值包括：NO_TURN, LEFT_TURN, RIGHT_TURN, U_TURN
     */
    const auto& turn_type = seg.lane->lane().turn();

    /**
     * @brief 累计计算路由路径上的s坐标
     */
    route_lane_start_s = route_lane_end_s;
    route_lane_end_s += seg.end_s - seg.start_s;

    AINFO << "s: " << route_lane_start_s << ", " << route_lane_end_s
          << ", ego s: " << adc_s;
    AINFO << "turn_type: " << turn_type;

    /**
     * @brief 跳过尚未到达的路段
     *
     * 如果当前路段的结束s小于自车位置s，说明自车还未到达该路段
     */
    if (route_lane_end_s < adc_s) {
      continue;
    }

    /**
     * @brief 设置当前转弯信息
     *
     * 如果自车在当前路段内（route_lane_start_s < adc_s + 0.01）
     * 则设置当前转弯类型
     */
    if (route_lane_start_s < adc_s + 0.01) {
      state->set_cur_turn_info(seg.lane->lane().turn());
      continue;
    }

    /**
     * @brief 检查并设置下一转弯信息
     *
     * 只有转弯类型（LEFT_TURN/RIGHT_TURN/U_TURN）才需要设置下一转弯
     * 直行(NO_TURN)不需要设置
     */
    if (turn_type == hdmap::Lane::LEFT_TURN ||
        turn_type == hdmap::Lane::RIGHT_TURN ||
        turn_type == hdmap::Lane::U_TURN) {
      state->mutable_next_turn_info()->set_turn_type(turn_type);
      state->mutable_next_turn_info()->set_dis_to_turn(route_lane_start_s -
                                                       adc_s);
      break;
    }
  }
}

void DrivingStateGenerator::add_nudge_info(
    const ReferenceLineInfo* reference_line_info,
    planning_internal::ADCDrivingState* const state) {
  /**
   * @brief 添加绕行信息到驾驶状态
   *
   * @param reference_line_info 参考线信息指针
   * @param state 输出参数，存储绕行信息
   *
   * 功能说明：
   * 分析自车周围障碍物的绕行决策信息
   *
   * 算法流程：
   * 1. 获取自车的SL边界
   * 2. 获取路径决策（包含所有障碍物）
   * 3. 初始化最近绕行障碍物距离为较大值
   * 4. 遍历所有SL多边形障碍物：
   *    - 跳过IGNORE/UNDEFINED类型或不与参考线重叠的障碍物
   *    - 查找对应的感知障碍物
   *    - 设置绕行类型（LEFT_NUDGE/RIGHT_NUDGE/BLOCKED）
   *    - 更新最近障碍物距离
   * 5. 根据最近障碍物距离和自车横向位置判断是否处于绕行状态
   *
   * C++语法说明：
   * - SLBoundary adc_sl_boundary = reference_line_info->AdcSlBoundary()：
   *   值拷贝获取自车SL边界
   *   SLBoundary是包含start_s/end_s/start_l/end_l的结构
   *
   * - const auto path_decision = reference_line_info->path_decision()：
   *   const引用获取路径决策
   *   避免不必要的拷贝
   *
   * - double nearest_nudge_obs_dis = 1000.0：
   *   初始化为较大值（1000米），表示"无穷远"
   *
   * - for (auto sl_polygon : reference_line_info->obs_sl_polygons())：
   *   遍历所有SL多边形障碍物
   *   obs_sl_polygons()返回障碍物在SL坐标系的表示
   *
   * - SLPolygon::IGNORE/UNDEFINED/LEFT_NUDGE/RIGHT_NUDGE/BLOCKED：
   *   SLPolygon类的枚举类型，表示障碍物的绕行决策
   *
   * - SLPolygon::OverlapeWithReferCenter()：
   *   判断SL多边形是否与参考线中心有重叠
   *   只有有重叠的障碍物才需要考虑绕行
   *
   * - path_decision.Find(sl_polygon.id())：
   *   根据障碍物ID查找感知障碍物
   *   返回const Obstacle*指针
   *
   * - state->add_nudge_obstacle()：
   *   protobuf的add方法，向repeated字段添加新元素
   *   返回新添加元素的指针
   *
   * - mutable_obstacle()：
   *   protobuf的mutable方法，返回可变指针用于修改嵌套消息
   *
   * - perception_obs->PerceptionSLBoundary()：
   *   获取障碍物的感知SL边界
   *   PerceptionSLBoundary返回的是SLBoundary对象
   *
   * - CopyFrom()：
   *   protobuf消息的拷贝方法
   *   将源消息的内容拷贝到目标消息
   *
   * - std::abs()：
   *   标准库绝对值函数
   *   注意：对于double类型，建议使用std::fabs()
   *
   * - FLAGS_driving_state_nudge_check_l：
   *   GFlags定义的全局标志变量
   *   用于控制绕行检查的横向阈值
   *
   * - state->set_is_nudging(true/false)：
   *   设置是否处于绕行状态
   *   条件：最近绕行障碍物距离<6.0米 且 横向偏移大于阈值
   */
  SLBoundary adc_sl_boundary = reference_line_info->AdcSlBoundary();
  const auto path_decision = reference_line_info->path_decision();
  double nearest_nudge_obs_dis = 1000.0;

  /**
   * @brief 遍历所有SL多边形障碍物
   */
  for (auto sl_polygon : reference_line_info->obs_sl_polygons()) {
    /**
     * @brief 跳过不需要考虑的障碍物
     *
     * IGNORE：被忽略的障碍物
     * UNDEFINED：未定义绕行类型的障碍物
     * 不与参考线重叠的障碍物
     */
    if (sl_polygon.NudgeInfo() == SLPolygon::IGNORE ||
        sl_polygon.NudgeInfo() == SLPolygon::UNDEFINED ||
        !sl_polygon.OverlapeWithReferCenter()) {
      continue;
    }

    /**
     * @brief 查找对应的感知障碍物
     *
     * SL多边形和感知障碍物通过ID关联
     */
    const Obstacle* perception_obs = path_decision.Find(sl_polygon.id());
    if (!perception_obs) {
      AERROR << "Failed to find obstacle : " << sl_polygon.id();
      continue;
    }

    /**
     * @brief 添加绕行障碍物调试信息
     */
    auto nudge_obstacle_debug = state->add_nudge_obstacle();
    nudge_obstacle_debug->set_is_blocked(false);

    /**
     * @brief 设置绕行类型
     *
     * 根据SL多边形的NudgeInfo枚举值设置对应的绕行类型
     */
    if (sl_polygon.NudgeInfo() == SLPolygon::LEFT_NUDGE) {
      nudge_obstacle_debug->set_type(
          planning_internal::NudgeObstacleDebug::LEFT_NUDGE);
    } else if (sl_polygon.NudgeInfo() == SLPolygon::RIGHT_NUDGE) {
      nudge_obstacle_debug->set_type(
          planning_internal::NudgeObstacleDebug::RIGHT_NUDGE);
    } else if (sl_polygon.NudgeInfo() == SLPolygon::BLOCKED) {
      nudge_obstacle_debug->set_type(
          planning_internal::NudgeObstacleDebug::NO_NUDGE);
      nudge_obstacle_debug->set_is_blocked(true);
    }

    /**
     * @brief 更新最近绕行障碍物距离
     *
     * 计算当前障碍物起始s与自车末端s的差值
     * 如果比之前的最小值更小，则更新
     */
    if (sl_polygon.MinS() - adc_sl_boundary.end_s() < nearest_nudge_obs_dis) {
      nearest_nudge_obs_dis = sl_polygon.MinS() - adc_sl_boundary.end_s();
    }

    /**
     * @brief 设置障碍物详细信息
     *
     * 将感知障碍物的ID和SL边界拷贝到调试信息中
     */
    nudge_obstacle_debug->mutable_obstacle()->set_id(perception_obs->Id());
    nudge_obstacle_debug->mutable_obstacle()->mutable_sl_boundary()->CopyFrom(
        perception_obs->PerceptionSLBoundary());
  }

  /**
   * @brief 获取参考线牵引距离
   *
   * reference_line_towing_l可能包含多个值
   * 取第一个值作为牵引距离
   */
  const auto& reference_line_towing_l =
      reference_line_info->reference_line_towing_l();
  double towing_l = 0.0;
  if (reference_line_towing_l.size() > 0) {
    towing_l = reference_line_towing_l.at(0);
  }

  /**
   * @brief 判断是否处于绕行状态
   *
   * 条件1：最近绕行障碍物距离 < 6.0米
   * 条件2：自车横向位置与牵引距离之差的绝对值 > 阈值
   *
   * C++语法说明：
   * - (adc_sl_boundary.start_l() + adc_sl_boundary.end_l()) * 0.5：
   *   计算自车横向位置的中心点
   *   start_l是左边界，end_l是右边界
   *
   * - std::abs(...) > FLAGS_driving_state_nudge_check_l：
   *   如果横向偏移大于阈值，说明自车不在正常车道中心
   *   可能正在绕行
   */
  if (nearest_nudge_obs_dis < 6.0 &&
      std::abs((adc_sl_boundary.start_l() + adc_sl_boundary.end_l()) * 0.5 -
               towing_l) > FLAGS_driving_state_nudge_check_l) {
    state->set_is_nudging(true);
  } else {
    state->set_is_nudging(false);
  }
}

}  // namespace planning
}  // namespace apollo
