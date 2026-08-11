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
 * @file planning_base.cc
 * @brief 规划基础类实现文件
 *
 * 本文件实现了规划模块的基础类PlanningBase
 * 作为所有规划器的基类，提供规划流程的公共基础设施
 *
 * 功能说明：
 * 1. 规划器初始化和生命周期管理
 * 2. 轨迹类型判断（普通规划/开放空间规划）
 * 3. 规划结果填充
 * 4. 车道宽度计算
 * 5. 插件式规划器加载
 *
 * 架构说明：
 * PlanningBase是规划组件的基类
 * - 持有一个Planner实例（插件式）
 * - 持有DependencyInjector进行依赖注入
 * - 提供公共的规划接口
 *
 * 相关C++语法说明：
 * - std::shared_ptr<T>: 智能指针，共享所有权
 * - const成员函数: 承诺不修改成员变量
 * - 初始化列表: 构造函数中使用，用于初始化成员变量
 **/

/**
 * @brief 本类的头文件
 *
 * 包含PlanningBase类的完整定义
 */
#include "modules/planning/planning_component/planning_base.h"

/**
 * @brief 规划内部消息protobuf头文件
 *
 * planning_internal.pb.h:
 *   包含规划模块内部使用的数据结构
 *   如debug信息、统计数据等
 */
#include "modules/common_msgs/planning_msgs/planning_internal.pb.h"

/**
 * @brief Cyber RT插件管理器头文件
 *
 * cyber/plugin_manager/plugin_manager.h:
 *   插件管理器，用于动态加载规划器插件
 *   支持运行时选择不同的规划算法
 */
#include "cyber/plugin_manager/plugin_manager.h"

/**
 * @brief Cyber RT时钟头文件
 *
 * cyber/time/clock.h:
 *   提供系统时间获取功能
 *   用于时间戳同步
 */
#include "cyber/time/clock.h"

/**
 * @brief 高精度地图头文件
 *
 * hdmap.h:
 *   包含HD Map数据结构
 *   提供道路、车道、路口等信息
 */
#include "modules/map/hdmap/hdmap.h"

/**
 * @brief 地图工具头文件
 *
 * hdmap_util.h:
 *   提供地图查询工具函数
 */
#include "modules/map/hdmap/hdmap_util.h"

/**
 * @brief 依赖注入器头文件
 *
 * dependency_injector.h:
 *   依赖注入器，提供各模块的依赖服务
 *   实现控制反转(IoC)模式
 */
#include "modules/planning/planning_base/common/dependency_injector.h"

/**
 * @brief 规划帧头文件
 *
 * frame.h:
 *   包含Frame类，存储一帧规划的所有数据
 *   包括车辆状态、障碍物、参考线等
 */
#include "modules/planning/planning_base/common/frame.h"

/**
 * @brief 本地视图头文件
 *
 * local_view.h:
 *   包含LocalView结构，存储规划所需的输入数据
 *   包括感知、预测、车辆状态等
 */
#include "modules/planning/planning_base/common/local_view.h"

/**
 * @brief 规划上下文头文件
 *
 * planning_context.h:
 *   规划上下文，存储跨帧状态信息
 *   如目的地、换道状态等
 */
#include "modules/planning/planning_base/common/planning_context.h"

/**
 * @brief 可发布轨迹头文件
 *
 * publishable_trajectory.h:
 *   包含PublishableTrajectory类
 *   用于将规划轨迹发布给控制模块
 */
#include "modules/planning/planning_base/common/trajectory/publishable_trajectory.h"

/**
 * @brief 配置工具头文件
 *
 * config_util.h:
 *   配置工具函数
 *   用于获取完整的规划器类名
 */
#include "modules/planning/planning_base/common/util/config_util.h"

/**
 * @brief 规划模块GFlags头文件
 *
 * FLAGS_*: 从配置文件获取的参数
 * 如规划频率、最大速度等
 */
#include "modules/planning/planning_base/gflags/planning_gflags.h"

/**
 * @brief 规划器基类头文件
 *
 * planner.h:
 *   定义规划器的接口
 *   所有具体规划器都继承自Planner基类
 */
#include "modules/planning/planning_interface_base/planner_base/planner.h"

/**
 * @brief Apollo命名空间开始
 */
namespace apollo {

/**
 * @brief 规划模块命名空间
 */
namespace planning {

/**
 * @brief 使用别名声明，简化类型引用
 *
 * C++语法说明：
 * using声明：引入其他命名空间的类型到当前作用域
 */
using apollo::common::Status;  /**< Apollo通用状态类型 */

/**
 * @brief PlanningBase构造函数
 *
 * @param injector 依赖注入器智能指针
 *
 * 功能说明：
 * 使用初始化列表初始化成员变量injector_
 *
 * C++语法说明：
 * - const std::shared_ptr<DependencyInjector>&:
 *   常量引用，避免拷贝
 *   shared_ptr是引用计数智能指针
 * - : injector_(injector):
 *   初始化列表语法，在构造函数体执行之前初始化成员变量
 *   这是初始化const成员变量或引用类型成员的唯一方式
 */
PlanningBase::PlanningBase(const std::shared_ptr<DependencyInjector>& injector)
    : injector_(injector) {}  /**< 使用初始化列表初始化依赖注入器 */

/**
 * @brief PlanningBase析构函数
 *
 * 功能说明：
 * 默认析构函数
 * 由于使用智能指针管理资源，不需要手动释放
 *
 * C++语法说明：
 * ~PlanningBase():
 *   析构函数，对象生命周期结束时自动调用
 *   使用= default表示使用默认实现
 */
PlanningBase::~PlanningBase() {}

/**
 * @brief 规划基础类初始化函数
 *
 * @param config 规划配置
 * @return Status 初始化状态
 *
 * 功能说明：
 * 1. 初始化规划上下文
 * 2. 保存规划配置
 *
 * 算法流程：
 * 1. 调用injector_->planning_context()->Init()初始化规划上下文
 *    - 规划上下文存储跨帧的状态信息
 * 2. 将配置保存到成员变量config_
 *
 * C++语法说明：
 * - const PlanningConfig& config:
 *   常量引用，PlanningConfig是protobuf消息类型
 * - injector_->planning_context():
 *   ->运算符，通过智能指针调用成员函数
 *   planning_context()返回PlanningContext指针
 * - config_ = config:
 *   赋值运算符，重载赋值
 */
Status PlanningBase::Init(const PlanningConfig& config) {
  /**
   * @brief 初始化规划上下文
   *
   * injector_->planning_context():
   *   获取规划上下文指针
   * PlanningContext::Init():
   *   初始化上下文中的状态变量
   *   如目的地、换道状态等
   */
  injector_->planning_context()->Init();

  /**
   * @brief 保存规划配置
   *
   * config_是类的成员变量
   * 保存配置以便后续使用
   */
  config_ = config;

  return Status::OK();  /**< 返回成功状态 */
}

/**
 * @brief 判断规划是否完成
 *
 * @param current_trajectory_type 当前轨迹类型
 * @return bool 是否完成
 *
 * 功能说明：
 * 判断当前帧的规划是否已经完成
 * 根据轨迹类型采用不同的判断逻辑
 *
 * 轨迹类型说明：
 * - OPEN_SPACE: 开放空间规划（如泊车）
 * - 普通规划: 基于参考线的车道内/换道规划
 *
 * 算法说明：
 * 1. OPEN_SPACE类型：检查openspace_planning_finish标志
 * 2. 普通规划类型：检查是否到达目的地
 *
 * C++语法说明：
 * - const ADCTrajectory::TrajectoryType&:
 *   常量引用，轨迹类型枚举
 * - const成员函数: 承诺不修改成员变量
 * - injector_->frame_history()->Latest():
 *   获取当前帧历史中的最新帧
 */
bool PlanningBase::IsPlanningFinished(
    const ADCTrajectory::TrajectoryType& current_trajectory_type) const {
  /**
   * @brief 获取当前帧
   *
   * injector_->frame_history():
   *   获取帧历史记录器
   * Latest():
   *   获取最新的帧
   */
  const auto frame = injector_->frame_history()->Latest();

  /**
   * @brief 判断轨迹类型
   *
   * if-else分支：
   * 根据轨迹类型选择不同的检查逻辑
   */
  if (current_trajectory_type == apollo::planning::ADCTrajectory::OPEN_SPACE) {
    /**
     * @brief 开放空间规划处理
     *
     * OPEN_SPACE用于：
     * - 泊车场景
     * - 狭窄空间 maneuvering
     * - 非结构化道路环境
     */
    AINFO << "Current trajectory type is: OPEN SPACE";

    /**
     * @brief 检查开放空间规划是否完成
     *
     * frame->open_space_info().openspace_planning_finish():
     *   获取开放空间规划的状态标志
     */
    if (frame->open_space_info().openspace_planning_finish()) {
      AINFO << "OPEN SPACE: planning finished";
      return true;  /**< 开放空间规划已完成 */
    } else {
      AINFO << "OPEN SPACE: planning not finished";
      return false;  /**< 开放空间规划未完成 */
    }
  } else {
    /**
     * @brief 普通规划处理（基于参考线）
     *
     * 检查以下条件：
     * 1. 帧有效
     * 2. 参考线信息非空
     * 3. 规划命令有效
     */

    /**
     * @brief 检查帧有效性
     *
     * nullptr == frame:
     *   检查帧指针是否为空
     * frame->reference_line_info().empty():
     *   检查参考线信息是否为空
     * local_view_.planning_command:
     *   本地视图中的规划命令
     */
    if (nullptr == frame || frame->reference_line_info().empty() ||
        nullptr == local_view_.planning_command) {
      AINFO << "Current reference point is empty;";
      return true;  /**< 无有效参考线，规划结束 */
    }

    /**
     * @brief 获取主参考线信息
     *
     * frame->reference_line_info().front():
     *   获取参考线列表中的第一条
     *   通常是自车所在车道
     */
    const auto& reference_line_info = frame->reference_line_info().front();

    /**
     * @brief 获取参考线的参考点列表
     *
     * reference_line_info.reference_line().reference_points():
     *   获取参考线上的一系列离散点
     *   用于判断是否到达道路终点
     */
    const auto& reference_points =
        reference_line_info.reference_line().reference_points();

    /**
     * @brief 检查参考点是否为空
     */
    if (reference_points.empty()) {
      AINFO << "Current reference points is empty;";
      return true;  /**< 无参考点，规划结束 */
    }

    /**
     * @brief 获取最后一个参考点
     *
     * reference_points.back():
     *   获取vector的最后一个元素
     *   这个点代表道路的终点
     */
    const auto& last_reference_point = reference_points.back();

    /**
     * @brief 获取终点所在的车道信息
     *
     * last_reference_point.lane_waypoints():
     *   获取该参考点所在的车道waypoint列表
     */
    const std::vector<hdmap::LaneWaypoint>& lane_way_points =
        last_reference_point.lane_waypoints();

    /**
     * @brief 检查车道waypoint是否为空
     */
    if (lane_way_points.empty()) {
      AINFO << "Last reference point is empty;";
      return true;  /**< 无车道信息，规划结束 */
    }

    /**
     * @brief 检查终点车道waypoint是否有效
     *
     * frame->local_view().end_lane_way_point:
     *   规划路径的终点车道waypoint
     */
    if (nullptr == frame->local_view().end_lane_way_point) {
      AINFO << "Current end lane way is empty;";
      return true;  /**< 无终点车道，规划结束 */
    }

    /**
     * @brief 检查是否已通过目的地
     *
     * injector_->planning_context()->planning_status():
     *   获取规划状态
     * .destination().has_passed_destination():
     *   检查是否已经经过目的地
     *   这是一个标志，表示自车已经驶过目的地点
     *
     * 这个判断用于确定规划是否完成
     * 如果已经通过目的地，则不需要继续规划
     */
    bool is_has_passed_destination = injector_->planning_context()
                                         ->planning_status()
                                         .destination()
                                         .has_passed_destination();

    /**
     * @brief 打印调试信息
     */
    AINFO << "Current passed destination:" << is_has_passed_destination;

    /**
     * @brief 返回是否已通过目的地
     */
    return is_has_passed_destination;
  }
}

/**
 * @brief 填充规划结果消息
 *
 * @param timestamp 当前时间戳
 * @param trajectory_pb 输出：轨迹消息
 *
 * 功能说明：
 * 将规划结果填充到ADCTrajectory protobuf消息中
 * 用于发布给下游模块（控制、定位等）
 *
 * 填充内容：
 * 1. 消息头时间戳
 * 2. 传感器时间戳（激光雷达、相机、雷达）
 * 3. 路由头信息
 *
 * C++语法说明：
 * - const double timestamp:
 *   时间戳，单位秒
 * - ADCTrajectory* const trajectory_pb:
 *   指向常量的指针（指针本身是常量）
 *   可以修改指针指向的内容
 */
void PlanningBase::FillPlanningPb(const double timestamp,
                                  ADCTrajectory* const trajectory_pb) {
  /**
   * @brief 设置消息头时间戳
   *
   * trajectory_pb->mutable_header():
   *   mutable_header()返回可修改的Header指针
   * set_timestamp_sec():
   *   设置时间戳（秒）
   *
   * C++语法说明：
   * mutable_前缀是protobuf的特殊方法
   * 用于在const对象上获取可修改的成员
   */
  trajectory_pb->mutable_header()->set_timestamp_sec(timestamp);

  /**
   * @brief 填充传感器时间戳
   *
   * 如果prediction_obstacles包含有效的header
   * 则将其传感器时间戳复制到轨迹消息
   *
   * 传感器时间戳用于：
   * - 时间同步
   * - 延迟补偿
   */
  if (local_view_.prediction_obstacles->has_header()) {
    /**
     * @brief 设置激光雷达时间戳
     *
     * local_view_.prediction_obstacles->header().lidar_timestamp():
     *   获取预测障碍物消息头中的激光雷达时间戳
     */
    trajectory_pb->mutable_header()->set_lidar_timestamp(
        local_view_.prediction_obstacles->header().lidar_timestamp());

    /**
     * @brief 设置相机时间戳
     */
    trajectory_pb->mutable_header()->set_camera_timestamp(
        local_view_.prediction_obstacles->header().camera_timestamp());

    /**
     * @brief 设置雷达时间戳
     */
    trajectory_pb->mutable_header()->set_radar_timestamp(
        local_view_.prediction_obstacles->header().radar_timestamp());
  }

  /**
   * @brief 复制路由头信息
   *
   * trajectory_pb->mutable_routing_header():
   *   获取可修改的路由头指针
   * CopyFrom():
   *   从另一个消息复制所有字段
   *
   * 路由头信息包含：
   * - 路由请求ID
   * - 路由时间戳
   * - 路由类型等
   */
  trajectory_pb->mutable_routing_header()->CopyFrom(
      local_view_.planning_command->header());
}

/**
 * @brief 加载规划器
 *
 * 功能说明：
 * 使用插件管理器加载指定的规划器
 * 支持运行时选择不同的规划算法
 *
 * 算法流程：
 * 1. 确定规划器名称
 * 2. 获取完整的类名
 * 3. 创建规划器实例
 *
 * 默认规划器：
 * 如果配置中未指定，使用PublicRoadPlanner
 *
 * C++语法说明：
 * - cyber::plugin_manager::PluginManager::Instance():
 *   单例模式获取插件管理器实例
 * - CreateInstance<Planner>():
 *   模板方法，根据类名创建插件实例
 * - ConfigUtil::GetFullPlanningClassName():
 *   工具函数，将短类名转换为完整类名
 */
void PlanningBase::LoadPlanner() {
  /**
   * @brief 设置默认规划器名称
   *
   * PublicRoadPlanner:
   *   公共道路规划器，是Apollo的默认规划器
   *   支持车道保持、换道、跟车等场景
   */
  std::string planner_name = "apollo::planning::PublicRoadPlanner";

  /**
   * @brief 检查配置中是否指定了规划器
   *
   * config_.planner():
   *   从配置中获取规划器名称
   *   如果为空字符串，使用默认规划器
   */
  if ("" != config_.planner()) {
    planner_name = config_.planner();  /**< 使用配置的规划器 */

    /**
     * @brief 获取完整的类名
     *
     * 配置文件中的名称可能是短名称
     * 需要转换为完整的命名空间类名
     * 如 "PublicRoadPlanner" -> "apollo::planning::PublicRoadPlanner"
     */
    planner_name = ConfigUtil::GetFullPlanningClassName(planner_name);
  }

  /**
   * @brief 创建规划器实例
   *
   * cyber::plugin_manager::PluginManager::Instance()->CreateInstance<Planner>():
   *   使用插件管理器根据类名创建规划器实例
   *   返回一个shared_ptr<Planner>
   *
   * 插件机制：
   * - 规划器作为插件加载
   * - 支持运行时替换不同的规划算法
   * - 通过配置文件指定使用哪个规划器
   */
  planner_ =
      cyber::plugin_manager::PluginManager::Instance()->CreateInstance<Planner>(
          planner_name);
}

/**
 * @brief 生成车道宽度点
 *
 * @param current_location 当前车辆位置（XY坐标系）
 * @param left_point 输出：左侧边界点
 * @param right_point 输出：右侧边界点
 * @return bool 是否成功获取
 *
 * 功能说明：
 * 根据车辆当前位置，计算车道左右边界点
 * 用于可视化或车道保持功能
 *
 * 算法流程：
 * 1. 获取当前帧
 * 2. 获取车辆SL坐标
 * 3. 查询该位置的车道宽度
 * 4. 计算左右边界点的XY坐标
 *
 * 坐标系说明：
 * - 输入：XY坐标系下的车辆位置
 * - SL坐标系：沿参考线的距离和横向偏移
 * - 输出：XY坐标系下的车道边界点
 *
 * C++语法说明：
 * - const Vec2d& current_location:
 *   当前车辆位置（常量引用）
 * - Vec2d& left_point, Vec2d& right_point:
 *   输出参数，引用类型
 */
bool PlanningBase::GenerateWidthOfLane(const Vec2d& current_location,
                                       Vec2d& left_point, Vec2d& right_point) {
  /**
   * @brief 初始化左右宽度
   */
  double left_width = 0, right_width = 0;

  /**
   * @brief 获取当前帧
   */
  const auto frame = injector_->frame_history()->Latest();  // ??????????????????

  /**
   * @brief 检查帧有效性
   *
   * 如果帧为空或没有参考线，返回失败
   */
  if (nullptr == frame || frame->reference_line_info().empty()) {
    AINFO << "Reference lane is empty!";
    return false;  /**< 获取帧失败 */
  }

  /**
   * @brief 获取主参考线信息
   */
  const auto& reference_line_info = frame->reference_line_info().front();

  /**
   * @brief 获取当前位置的SL坐标
   *
   * reference_line.XYToSL():
   *   将XY坐标转换为SL坐标
   *   XYToSL(x, y, &sl_point)
   *
   * SL坐标：
   * - s: 沿参考线的累积距离
   * - l: 横向偏移（正为左侧，负为右侧）
   */
  common::SLPoint current_sl;
  reference_line_info.reference_line().XYToSL(current_location, &current_sl);

  /**
   * @brief 获取车道宽度
   *
   * reference_line.GetLaneWidth(s, &left_width, &right_width):
   *   根据s坐标获取该点的车道宽度
   *   left_width: 中心线到左边界
   *   right_width: 中心线到右边界
   *
   * 返回值表示是否成功获取
   */
  bool get_width_of_lane = reference_line_info.reference_line().GetLaneWidth(
      current_sl.s(), &left_width, &right_width);

  /**
   * @brief 打印调试信息
   */
  AINFO << "get_width_of_lane: " << get_width_of_lane
        << ", left_width: " << left_width << ", right_width: " << right_width;

  /**
   * @brief 检查是否成功获取且宽度有效
   */
  if (get_width_of_lane && left_width != 0 && right_width != 0) {
    AINFO << "Get the width of lane successfully!";

    /**
     * @brief 创建SL坐标的左右边界点
     *
     * SLPoint:
     *   - s: 沿参考线的距离（与current_sl.s()相同）
     *   - l: 横向偏移
     *     - left_width: 左侧边界
     *     - -right_width: 右侧边界（负值表示在参考线右侧）
     */
    SLPoint sl_left_point, sl_right_point;
    sl_left_point.set_s(current_sl.s());   /**< 设置s坐标 */
    sl_left_point.set_l(left_width);       /**< 设置l坐标（正值=左侧） */
    sl_right_point.set_s(current_sl.s()); /**< 设置s坐标 */
    sl_right_point.set_l(-right_width);   /**< 设置l坐标（负值=右侧） */

    /**
     * @brief 将SL坐标转换回XY坐标
     *
     * reference_line.SLToXY(sl_point, &xy_point):
     *   将SL坐标转换为XY坐标
     */
    reference_line_info.reference_line().SLToXY(sl_left_point, &left_point);
    reference_line_info.reference_line().SLToXY(sl_right_point, &right_point);

    return true;  /**< 成功获取车道宽度 */
  } else {
    /**
     * @brief 获取失败
     */
    AINFO << "Failed to get the width of lane!";
    return false;
  }
}

}  // namespace planning
}  // namespace apollo
