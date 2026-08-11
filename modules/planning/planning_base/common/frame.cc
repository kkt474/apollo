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
 * @file frame.cc
 * @brief 规划帧实现文件
 *
 * 本文件实现Frame类，它是Apollo规划模块的核心数据结构之一。
 * Frame（规划帧）包含单个规划周期内的所有输入输出数据。
 *
 * 语法说明：
 * - std::list<T>: C++双向链表容器，支持高效的插入删除操作
 * - std::map<K, V>: C++红黑树实现的关联数组，键值对有序排列
 * - std::vector<T>: C++动态数组，支持随机访问
 * - std::unique_ptr<T>: 独占所有权的智能指针，不可复制
 * - std::const_reference: 常量引用类型
 * - auto关键字: 编译器自动推断变量类型
 * - nullptr: C++11空指针字面量
 */
#include "modules/planning/planning_base/common/frame.h"

#include <algorithm>              /**< C++标准算法库，包含std::find, std::sort等 */
#include <limits>                  /**< C++数值极限库，包含std::numeric_limits等 */

#include "absl/strings/str_cat.h" /**< Abseil字符串拼接库，StrCat用于连接字符串 */

#include "modules/common_msgs/routing_msgs/routing.pb.h" /**< 路由消息protobuf定义 */

#include "cyber/common/log.h"     /**< Cyber RT日志系统 */
#include "cyber/time/clock.h"     /**< Cyber RT时钟系统，获取当前时间 */
#include "modules/common/configs/vehicle_config_helper.h" /**< 车辆配置助手 */
#include "modules/common/math/vec2d.h"   /**< 二维向量数学库 */
#include "modules/common/util/point_factory.h"  /**< 点工厂工具 */
#include "modules/common/vehicle_state/vehicle_state_provider.h" /**< 车辆状态提供者 */
#include "modules/map/hdmap/hdmap_util.h"    /**< 高精地图工具 */
#include "modules/map/pnc_map/path.h"        /**< PNC地图路径 */
#include "modules/planning/planning_base/common/feature_output.h" /**< 特征输出 */
#include "modules/planning/planning_base/common/planning_context.h" /**< 规划上下文 */
#include "modules/planning/planning_base/common/util/util.h"        /**< 规划工具函数 */
#include "modules/planning/planning_base/gflags/planning_gflags.h" /**< 规划配置标志 */
#include "modules/planning/planning_base/reference_line/reference_line_provider.h" /**< 参考线提供者 */

namespace apollo {
/**
 * apollo:: - Apollo最外层命名空间
 * 所有Apollo相关代码都位于此命名空间下
 */
namespace planning {

/**
 * using声明 - 将其他命名空间的类型引入当前作用域
 * 语法：using 命名空间::类型名;
 * 之后可以直接使用类型名而不需要完整命名空间前缀
 */
using apollo::common::ErrorCode;       /**< 错误代码枚举类型 */
using apollo::common::Status;         /**< Apollo通用状态类型，包含error_code和message */
using apollo::common::math::Box2d;     /**< 二维包围盒类，用于碰撞检测 */
using apollo::common::math::Polygon2d; /**< 二维多边形类 */
using apollo::cyber::Clock;           /**< Cyber RT时钟，用于获取当前时间 */
using apollo::prediction::PredictionObstacles; /**< 预测障碍物消息类型 */

/**
 * 静态成员变量定义
 * PadMessage::DrivingAction - 驾驶动作枚举（NONE, STOP, CHANGE_LANE等）
 * 这是类级别的静态变量，所有Frame实例共享同一个值
 */
PadMessage::DrivingAction Frame::pad_msg_driving_action_ = PadMessage::NONE;

/**
 * FrameHistory构造函数
 * 使用初始化列表调用基类IndexedQueue的构造函数
 * FLAGS_max_frame_history_num - 配置标志，指定最大历史帧数
 *
 * 语法说明：
 * - : IndexedQueue<uint32_t, Frame>(FLAGS_max_frame_history_num)
 *   初始化列表语法，用于在构造函体之前初始化基类和成员变量
 * - IndexedQueue<K, V> - 模板类，以K为键V为值的索引队列
 */
FrameHistory::FrameHistory()
    : IndexedQueue<uint32_t, Frame>(FLAGS_max_frame_history_num) {}

/**
 * Frame类构造函数（序列号构造）
 * @param sequence_num 规划帧的序列号，用于标识帧顺序
 *
 * 语法说明：
 * - : sequence_num_(sequence_num) 初始化列表，将参数赋值给成员变量
 * - monitor_logger_buffer_ - 监控日志缓冲区，用于记录规划过程的关键事件
 */
Frame::Frame(uint32_t sequence_num)
    : sequence_num_(sequence_num),    /**< 初始化序列号 */
      monitor_logger_buffer_(common::monitor::MonitorMessageItem::PLANNING) {} /**< 初始化监控日志 */

/**
 * Frame类完整构造函数
 * @param sequence_num 规划帧序列号
 * @param local_view 本地视图，包含所有输入数据
 * @param planning_start_point 规划起始点（自车当前位置）
 * @param vehicle_state 车辆状态
 * @param reference_line_provider 参考线提供者指针（可为nullptr）
 *
 * 语法说明：
 * - const std::shared_ptr<T>&: 常量引用参数，避免不必要的拷贝
 * - const T&: 常量引用，只读访问
 * - T*: 原始指针，可以为空（nullptr）
 * - initializer_list初始化: {param1, param2} 用于初始化成员
 */
Frame::Frame(uint32_t sequence_num, const LocalView &local_view,
             const common::TrajectoryPoint &planning_start_point,
             const common::VehicleState &vehicle_state,
             ReferenceLineProvider *reference_line_provider)
    : sequence_num_(sequence_num),           /**< 帧序列号 */
      local_view_(local_view),               /**< 本地输入视图（拷贝构造） */
      planning_start_point_(planning_start_point), /**< 规划起始点 */
      vehicle_state_(vehicle_state),         /**< 车辆状态 */
      reference_line_provider_(reference_line_provider), /**< 参考线提供者（可能为空） */
      monitor_logger_buffer_(common::monitor::MonitorMessageItem::PLANNING) {} /**< 监控日志 */

/**
 * Frame委托构造函数
 * 将构造工作委托给上面的完整构造函数，reference_line_provider默认为nullptr
 *
 * 语法说明：
 * - : Frame(...) 委托构造，将参数传递给另一个构造函数
 * - nullptr: C++11空指针常量
 */
Frame::Frame(uint32_t sequence_num, const LocalView &local_view,
             const common::TrajectoryPoint &planning_start_point,
             const common::VehicleState &vehicle_state)
    : Frame(sequence_num, local_view, planning_start_point, vehicle_state,
            nullptr) {} /**< 委托给四参数构造函数，reference_line_provider为nullptr */

/**
 * 获取规划起始点的常量引用
 * @return const common::TrajectoryPoint& 起始点只读引用
 *
 * 语法说明：
 * - const T&: 常量引用返回，表示返回后不能修改该对象
 * - 函数后加const表示这是const成员函数，this指针是const的
 */
const common::TrajectoryPoint &Frame::PlanningStartPoint() const {
  return planning_start_point_;  /**< 返回成员变量的常量引用 */
}

/**
 * 获取车辆状态的常量引用
 * @return const common::VehicleState& 车辆状态只读引用
 */
const common::VehicleState &Frame::vehicle_state() const {
  return vehicle_state_;
}

/**
 * @brief 发起重路由请求
 *
 * 当车辆偏离原定路线或遇到无法通行的情况时调用此函数。
 * 函数将重路由标志设置为true，并填充重路由命令的路径点信息。
 *
 * @param planning_context 规划上下文，用于存储重路由状态
 * @return bool 重路由是否成功发起
 *
 * 语法说明：
 * - std::lock_guard<std::mutex>: RAII锁，构造时加锁析构时自动解锁
 * - auto* : 原始指针，不拥有对象所有权
 * - mutable_xxx(): 返回可写指针
 * - std::make_shared<T>(args): 创建共享指针
 */
bool Frame::Rerouting(PlanningContext *planning_context) {
  /**
   * 导航模式不支持重路由，直接返回false
   * FLAGS_use_navigation_mode - 判断是否使用导航模式
   */
  if (FLAGS_use_navigation_mode) {
    AERROR << "Rerouting not supported in navigation mode";
    return false;
  }

  /**
   * 检查是否有之前的路由可用
   * local_view_.planning_command - 包含路由命令的结构
   * nullptr - 空指针检查
   */
  if (local_view_.planning_command == nullptr) {
    AERROR << "No previous routing available";
    return false;
  }

  /**
   * 检查高精地图是否有效
   * hdmap_ - Frame类的成员变量，指向高精地图的指针
   */
  if (!hdmap_) {
    AERROR << "Invalid HD Map.";
    return false;
  }

  /**
   * 获取可写的重路由状态指针
   * planning_context->mutable_planning_status() - 获取规划状态的可写指针
   * ->mutable_rerouting() - 获取重路由状态的可写指针
   */
  auto *rerouting =
      planning_context->mutable_planning_status()->mutable_rerouting();
  rerouting->set_need_rerouting(true);  /**< 设置需要重路由标志 */

  /**
   * 获取可写的车道跟随命令指针
   * mutable_lane_follow_command() - 返回车道跟随命令的可写指针
   */
  auto *lane_follow_command = rerouting->mutable_lane_follow_command();

  /**
   * 检查未来路径点是否足够
   * future_route_waypoints_ - 存储未来导航路径点
   * size() - 返回容器大小
   * < 1 表示为空
   */
  if (future_route_waypoints_.size() < 1) {
    AERROR << "Failed to find future waypoints";
    return false;
  }

  /**
   * 循环填充路径点（除最后一个外）
   * for循环：初始化; 条件判断; 增量表达式
   * size()返回size_t无符号类型
   * add_way_point() - 在repeated字段中添加一个新元素
   * set_x()/set_y()/set_heading() - 设置路径点的坐标和航向角
   */
  for (size_t i = 0; i < future_route_waypoints_.size() - 1; i++) {
    auto waypoint = lane_follow_command->add_way_point(); /**< 添加路径点 */
    waypoint->set_x(future_route_waypoints_[i].pose().x());     /**< 设置X坐标 */
    waypoint->set_y(future_route_waypoints_[i].pose().y());     /**< 设置Y坐标 */
    waypoint->set_heading(future_route_waypoints_[i].heading()); /**< 设置航向角 */
  }

  /**
   * 设置终点位置
   * future_route_waypoints_.back() - 获取最后一个元素
   * mutable_end_pose() - 获取终点pose的可写指针
   */
  auto *end_pose = lane_follow_command->mutable_end_pose();
  end_pose->set_x(future_route_waypoints_.back().pose().x());
  end_pose->set_y(future_route_waypoints_.back().pose().y());
  end_pose->set_heading(future_route_waypoints_.back().heading());

  /**
   * 记录监控日志
   * monitor_logger_buffer_.INFO() - 输出INFO级别日志
   */
  monitor_logger_buffer_.INFO("Planning send Rerouting request");
  return true;  /**< 重路由请求成功发起 */
}

/**
 * 获取参考线信息列表的常量引用
 * @return const std::list<ReferenceLineInfo>& 参考线信息列表只读引用
 *
 * 语法说明：
 * - std::list<T>: C++双向链表容器
 *   - 支持常数时间插入删除
 *   - 不支持随机访问
 *   - 迭代器是双向的
 */
const std::list<ReferenceLineInfo> &Frame::reference_line_info() const {
  return reference_line_info_;  /**< 返回成员变量常量引用 */
}

/**
 * 获取参考线信息列表的可写指针
 * @return std::list<ReferenceLineInfo>* 可写指针
 *
 * 语法说明：
 * - 返回指针而非引用，允许调用者修改列表内容
 * - 调用者负责确保线程安全访问
 */
std::list<ReferenceLineInfo> *Frame::mutable_reference_line_info() {
  return &reference_line_info_;  /**< 返回成员变量地址 */
}

/**
 * @brief 更新参考线的优先级
 *
 * 根据提供的优先级映射表更新各参考线的优先级。
 * 用于多参考线决策时确定哪条参考线优先选择。
 *
 * @param id_to_priority 映射表：参考线ID到优先级的映射
 *
 * 语法说明：
 * - const std::map<std::string, uint32_t>&: 常量引用参数
 * - std::map<K, V>: C++红黑树实现的关联数组
 *   - 键值对按键排序
 *   - 查找/插入/删除为O(log n)
 * - std::pair<const K, V>: 键值对类型
 * - const auto&: const自动类型推断的引用
 */
void Frame::UpdateReferenceLinePriority(
    const std::map<std::string, uint32_t> &id_to_priority) {
  /**
   * 遍历优先级映射表
   * for (const auto& pair : id_to_priority)
   *   - range-based for循环，C++11引入
   *   - pair.first为键(string ID)
   *   - pair.second为值(uint32_t优先级)
   */
  for (const auto &pair : id_to_priority) {
    const auto id = pair.first;       /**< 提取参考线ID */
    const auto priority = pair.second; /**< 提取优先级 */

    /**
     * std::find_if - 在范围内查找满足条件的第一个元素
     * - reference_line_info_.begin()/end() - 获取迭代器
     * - lambda表达式: [&id](const ReferenceLineInfo& ref_line_info) {...}
     *   - [&id]: 捕获外部变量id的引用
     *   - 返回bool表示是否匹配
     */
    auto ref_line_info_itr =
        std::find_if(reference_line_info_.begin(), reference_line_info_.end(),
                     [&id](const ReferenceLineInfo &ref_line_info) {
                       return ref_line_info.Lanes().Id() == id; /**< 比较ID */
                     });

    /**
     * 检查是否找到匹配的参考线
     * != end() 表示找到了
     */
    if (ref_line_info_itr != reference_line_info_.end()) {
      ref_line_info_itr->SetPriority(priority); /**< 设置优先级 */
    }
  }
}

/**
 * @brief 创建参考线信息
 *
 * 根据提供的参考线和路径段列表创建参考线信息对象。
 * 这是规划初始化过程中的关键步骤。
 *
 * @param reference_lines 参考线列表
 * @param segments 路径段列表
 * @return bool 创建是否成功
 *
 * 语法说明：
 * - std::list<T>: 双向链表
 * - emplace_back(args): 就地构造元素，避免拷贝
 * - std::back(): 返回最后一个元素的引用
 * - ++iter: 前置递增，迭代器移动到下一个位置
 */
bool Frame::CreateReferenceLineInfo(
    const std::list<ReferenceLine> &reference_lines,
    const std::list<hdmap::RouteSegments> &segments) {
  reference_line_info_.clear();  /**< 清空现有参考线信息 */

  /**
   * 如果参考线列表为空，直接返回true（不是错误）
   */
  if (reference_lines.empty()) {
    return true;
  }

  /**
   * 获取起始迭代器
   * auto: 编译器自动推断迭代器类型
   */
  auto ref_line_iter = reference_lines.begin();  /**< 参考线迭代器 */
  auto segments_iter = segments.begin();         /**< 路径段迭代器 */
  std::size_t ref_line_index = 0;                 /**< 参考线索引，从0开始 */

  /**
   * while循环遍历参考线和路径段
   * 条件：迭代器不等于end()
   */
  while (ref_line_iter != reference_lines.end()) {
    /**
     * 检查当前路径段是否标记为目的地停车
     * StopForDestination() - 路径段方法，判断是否需要在此停车
     */
    if (segments_iter->StopForDestination()) {
      is_near_destination_ = true;  /**< 标记接近目的地 */
    }

    /**
     * emplace_back - 就地构造新元素
     * 构造函数参数直接传递给ReferenceLineInfo构造函数
     * 避免创建临时对象再拷贝
     */
    reference_line_info_.emplace_back(vehicle_state_, planning_start_point_,
                                      *ref_line_iter, *segments_iter);

    /**
     * set_index - 设置参考线索引
     * back()返回最后一个元素的引用
     */
    reference_line_info_.back().set_index(ref_line_index);
    ++ref_line_index;    /**< 索引递增 */
    ++ref_line_iter;     /**< 参考线迭代器前移 */
    ++segments_iter;     /**< 路径段迭代器前移 */
  }

  /**
   * 如果有两条参考线，计算它们之间的偏移量
   * 这用于处理换道场景
   */
  if (reference_line_info_.size() == 2) {
    /**
     * Vec2d - Apollo二维向量类
     * {x, y}列表初始化
     */
    common::math::Vec2d xy_point(vehicle_state_.x(), vehicle_state_.y());

    /**
     * common::SLPoint - SL坐标系点
     * s: 沿参考线的距离
     * l: 横向偏移（正为左，负为右）
     */
    common::SLPoint first_sl;

    /**
     * XYToSL - 将XY坐标转换为SL坐标
     * &first_sl - 输出参数指针
     * 返回bool表示转换是否成功
     */
    if (!reference_line_info_.front().reference_line().XYToSL(xy_point,
                                                              &first_sl)) {
      return false;  /**< 转换失败返回false */
    }

    common::SLPoint second_sl;  /**< 第二条参考线的SL点 */
    if (!reference_line_info_.back().reference_line().XYToSL(xy_point,
                                                             &second_sl)) {
      return false;
    }

    /**
     * 计算两条参考线之间的横向偏移差
     * first_sl.l() - first参考线的l值
     * second_sl.l() - second参考线的l值
     */
    const double offset = first_sl.l() - second_sl.l();

    /**
     * 设置相对偏移
     * front()返回首元素引用
     * back()返回尾元素引用
     */
    reference_line_info_.front().SetOffsetToOtherReferenceLine(offset);
    reference_line_info_.back().SetOffsetToOtherReferenceLine(-offset);
  }

  /**
   * 获取目标速度
   * FLAGS_default_cruise_speed - 默认巡航速度配置
   * 如果规划命令中有目标速度则使用它
   */
  double target_speed = FLAGS_default_cruise_speed;
  if (local_view_.planning_command->has_target_speed()) {
    target_speed = local_view_.planning_command->target_speed();
  }

  bool has_valid_reference_line = false;  /**< 是否有有效参考线标志 */
  ref_line_index = 0;                      /**< 重置索引 */

  /**
   * 遍历参考线信息列表进行初始化
   * for循环：初始化; 条件; 增量
   * 注意：循环体内有erase操作，需要特别处理迭代器
   */
  for (auto iter = reference_line_info_.begin();
       iter != reference_line_info_.end();) {
    /**
     * Init - 初始化参考线信息
     * obstacles() - 获取障碍物列表
     * target_speed - 目标速度
     * 返回bool表示初始化是否成功
     */
    if (!iter->Init(obstacles(), target_speed)) {
      /**
       * 初始化失败，删除此参考线
       * erase(iter++) - 删除当前元素并让迭代器指向下一个
       * post-increment返回旧值，但iter已经指向下一个
       */
      reference_line_info_.erase(iter++);
    } else {
      /**
       * 初始化成功
       * has_valid_reference_line - 设置有效标志
       */
      has_valid_reference_line = true;
      iter->set_index(ref_line_index); /**< 设置索引 */
      AINFO << "get referenceline: index: " << iter->index()
            << ", id: " << iter->id() << ", key: " << iter->key(); /**< 打印日志 */
      ref_line_index++;
      iter++;  /**< 迭代器前移 */
    }
  }

  /**
   * 如果没有有效参考线，记录日志
   */
  if (!has_valid_reference_line) {
    AINFO << "No valid reference line";
  }
  return true;  /**< 创建参考线信息成功 */
}

/**
 * @brief 创建停车障碍物
 *
 * 在指定位置创建一个虚拟停车墙障碍物。
 * 主要用于创建停车点、让行点等虚拟障碍物。
 *
 * @param reference_line_info 参考线信息指针
 * @param obstacle_id 障碍物ID
 * @param obstacle_s 障碍物在参考线上的s坐标
 * @param stop_wall_width 停车墙宽度
 * @return const Obstacle* 创建的障碍物指针（可能为空）
 *
 * 语法说明：
 * - T* const: 指向T的常量指针，指针本身是const
 * - nullptr: 空指针检查
 * - const T&: 常量引用，参数不可修改
 */
const Obstacle *Frame::CreateStopObstacle(
    ReferenceLineInfo *const reference_line_info,
    const std::string &obstacle_id, const double obstacle_s,
    double stop_wall_width) {
  /**
   * 空指针检查
   * == nullptr 可以简写为 !ptr
   */
  if (reference_line_info == nullptr) {
    AERROR << "reference_line_info nullptr";
    return nullptr;
  }

  /**
   * 获取参考线引用
   * const auto&: const引用，避免拷贝
   */
  const auto &reference_line = reference_line_info->reference_line();

  /**
   * 计算停车墙中心点的s坐标
   * FLAGS_virtual_stop_wall_length - 虚拟停车墙长度配置
   * box_center_s = obstacle_s + stop_wall_length / 2
   */
  const double box_center_s = obstacle_s + FLAGS_virtual_stop_wall_length / 2.0;

  /**
   * GetReferencePoint - 获取参考线上指定s坐标的点
   * 返回参考点（包含x, y, heading等信息）
   */
  auto box_center = reference_line.GetReferencePoint(box_center_s);

  /**
   * 获取障碍物处的航向角
   * GetReferencePoint(s).heading() - 获取该点的航向角
   */
  double heading = reference_line.GetReferencePoint(obstacle_s).heading();

  /**
   * Box2d - 二维包围盒类
   * 构造参数：(中心点位置, 航向角, 长度, 宽度)
   * 使用列表初始化
   */
  Box2d stop_wall_box{box_center, heading, FLAGS_virtual_stop_wall_length,
                      stop_wall_width};

  /**
   * CreateStaticVirtualObstacle - 创建静态虚拟障碍物
   * 返回创建的障碍物指针
   */
  return CreateStaticVirtualObstacle(obstacle_id, stop_wall_box);
}

/**
 * @brief 基于车道ID创建停车障碍物
 *
 * @param obstacle_id 障碍物ID
 * @param lane_id 车道ID
 * @param lane_s 障碍物在车道上的s坐标
 * @return const Obstacle* 创建的障碍物指针
 *
 * 语法说明：
 * - hdmap::LaneInfoConstPtr: 车道信息常量指针类型
 *   - Ptr后缀表示指针类型
 *   - Const后缀表示指针指向的内容不可修改
 */
const Obstacle *Frame::CreateStopObstacle(const std::string &obstacle_id,
                                          const std::string &lane_id,
                                          const double lane_s) {
  /**
   * 检查hdmap_是否有效
   * !hdmap_ 相当于 hdmap_ == nullptr
   */
  if (!hdmap_) {
    AERROR << "Invalid HD Map.";
    return nullptr;
  }

  /**
   * 获取车道信息
   * reference_line_provider_可能为空，需要处理
   */
  hdmap::LaneInfoConstPtr lane = nullptr;
  if (nullptr == reference_line_provider_) {
    /**
     * 直接从hdmap_获取车道
     * hdmap_->GetLaneById() - 根据ID获取车道信息
     * hdmap::MakeMapId(lane_id) - 将字符串转换为地图ID类型
     */
    lane = hdmap_->GetLaneById(hdmap::MakeMapId(lane_id));
  } else {
    /**
     * 通过reference_line_provider获取车道
     */
    lane = reference_line_provider_->GetLaneById(hdmap::MakeMapId(lane_id));
  }

  /**
   * 检查车道是否有效
   * !lane 相当于 lane == nullptr
   */
  if (!lane) {
    AERROR << "Failed to find lane[" << lane_id << "]";
    return nullptr;
  }

  /**
   * std::max - 返回两个值的较大者
   * 确保lane_s不为负数
   */
  double dest_lane_s = std::max(0.0, lane_s);

  /**
   * GetSmoothPoint - 获取车道上指定s坐标的光滑点
   * 返回PointENU类型（包含x, y, z坐标）
   */
  auto dest_point = lane->GetSmoothPoint(dest_lane_s);

  /**
   * 获取车道宽度
   * GetWidth(s, &left_width, &right_width) - 输出参数
   * left_width: 左侧宽度
   * right_width: 右侧宽度
   */
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  lane->GetWidth(dest_lane_s, &lane_left_width, &lane_right_width);

  /**
   * 创建停车墙包围盒
   * {{dest_point.x(), dest_point.y()}, ...} - 嵌套列表初始化
   * Heading(s) - 获取车道上指定s坐标点的航向角
   * lane_left_width + lane_right_width - 车道总宽度
   */
  Box2d stop_wall_box{{dest_point.x(), dest_point.y()},
                      lane->Heading(dest_lane_s),
                      FLAGS_virtual_stop_wall_length,
                      lane_left_width + lane_right_width};

  return CreateStaticVirtualObstacle(obstacle_id, stop_wall_box);
}

/**
 * @brief 创建静态障碍物
 *
 * 根据参考线和起止s坐标创建静态障碍物。
 * 用于创建道路边界、减速带等静态障碍物。
 *
 * @param reference_line_info 参考线信息指针
 * @param obstacle_id 障碍物ID
 * @param obstacle_start_s 障碍物起始s坐标
 * @param obstacle_end_s 障碍物结束s坐标
 * @return const Obstacle* 创建的障碍物指针
 *
 * 语法说明：
 * - std::numeric_limits<double>::max() - double类型的最大值
 */
const Obstacle *Frame::CreateStaticObstacle(
    ReferenceLineInfo *const reference_line_info,
    const std::string &obstacle_id, const double obstacle_start_s,
    const double obstacle_end_s) {
  /**
   * 空指针检查
   */
  if (reference_line_info == nullptr) {
    AERROR << "reference_line_info nullptr";
    return nullptr;
  }

  /**
   * 获取参考线引用
   */
  const auto &reference_line = reference_line_info->reference_line();

  /**
   * ========== 计算起始点的XY坐标 ==========
   */

  /**
   * common::SLPoint - SL坐标点
   * set_s()/set_l() - 设置s和l坐标值
   */
  common::SLPoint sl_point;
  sl_point.set_s(obstacle_start_s); /**< 设置起始s坐标 */
  sl_point.set_l(0.0);              /**< l坐标设为0（参考线上） */

  /**
   * Vec2d - 二维向量类，用于存储XY坐标
   */
  common::math::Vec2d obstacle_start_xy;

  /**
   * SLToXY - 将SL坐标转换为XY坐标
   * &obstacle_start_xy - 输出参数指针
   * 返回bool表示转换是否成功
   */
  if (!reference_line.SLToXY(sl_point, &obstacle_start_xy)) {
    AERROR << "Failed to get start_xy from sl: " << sl_point.DebugString();
    return nullptr;
  }

  /**
   * ========== 计算结束点的XY坐标 ==========
   */
  sl_point.set_s(obstacle_end_s);   /**< 设置结束s坐标 */
  sl_point.set_l(0.0);              /**< l坐标设为0 */
  common::math::Vec2d obstacle_end_xy;
  if (!reference_line.SLToXY(sl_point, &obstacle_end_xy)) {
    AERROR << "Failed to get end_xy from sl: " << sl_point.DebugString();
    return nullptr;
  }

  /**
   * ========== 获取车道宽度 ==========
   */
  double left_lane_width = 0.0;
  double right_lane_width = 0.0;

  /**
   * GetLaneWidth - 获取指定s坐标处的车道宽度
   * 输出参数：left_lane_width, right_lane_width
   */
  if (!reference_line.GetLaneWidth(obstacle_start_s, &left_lane_width,
                                   &right_lane_width)) {
    AERROR << "Failed to get lane width at s[" << obstacle_start_s << "]";
    return nullptr;
  }

  /**
   * ========== 创建障碍物包围盒 ==========
   *
   * LineSegment2d - 二维线段类
   * 构造参数：起点、终点
   * Box2d包围盒：(线段, 宽度)
   */
  common::math::Box2d obstacle_box{
      common::math::LineSegment2d(obstacle_start_xy, obstacle_end_xy),
      left_lane_width + right_lane_width}; /**< 总宽度 */

  return CreateStaticVirtualObstacle(obstacle_id, obstacle_box);
}

/**
 * @brief 创建静态虚拟障碍物
 *
 * 将给定的包围盒包装为虚拟障碍物并添加到障碍物列表中。
 *
 * @param id 障碍物ID
 * @param box 障碍物包围盒
 * @return const Obstacle* 创建的障碍物指针
 *
 * 语法说明：
 * - const Obstacle* : 返回常量指针，指向的障碍物不可修改
 * - obstacles_.Find(id) - 在容器中查找ID对应的障碍物
 * - obstacles_.Add(id, obstacle) - 添加障碍物到容器
 */
const Obstacle *Frame::CreateStaticVirtualObstacle(const std::string &id,
                                                   const Box2d &box) {
  /**
   * 先查找是否已存在同名障碍物
   * obstacles_ - IndexedQueue类型，存储所有障碍物
   * Find(id) - 按ID查找障碍物
   */
  const auto *object = obstacles_.Find(id);
  if (object) {
    AWARN << "obstacle " << id << " already exist.";
    return object;  /**< 已存在则返回现有对象 */
  }

  /**
   * 创建新的虚拟障碍物
   * Obstacle::CreateStaticVirtualObstacles(id, box) - 静态工厂方法
   * obstacles_.Add(id, *ptr) - 添加到障碍物列表
   * *ptr 解引用获取对象
   */
  auto *ptr =
      obstacles_.Add(id, *Obstacle::CreateStaticVirtualObstacles(id, box));
  if (!ptr) {
    AERROR << "Failed to create virtual obstacle " << id;
  }
  return ptr;
}

/**
 * @brief Frame初始化函数
 *
 * 完整的Frame初始化，包括帧数据初始化和参考线信息创建。
 *
 * @param vehicle_state_provider 车辆状态提供者
 * @param reference_lines 参考线列表
 * @param segments 路径段列表
 * @param future_route_waypoints 未来路线导航点
 * @param ego_info 自车信息
 * @return Status 初始化状态，成功返回OK
 *
 * 语法说明：
 * - Status - Apollo通用状态类型
 *   - ok() - 判断是否成功
 *   - ToString() - 获取状态描述
 * - std::list<T>: 双向链表
 * - std::vector<T>: 动态数组
 */
Status Frame::Init(
    const common::VehicleStateProvider *vehicle_state_provider,
    const std::list<ReferenceLine> &reference_lines,
    const std::list<hdmap::RouteSegments> &segments,
    const std::vector<routing::LaneWaypoint> &future_route_waypoints,
    const EgoInfo *ego_info) {
  // TODO(QiL): refactor this to avoid redundant nullptr checks in scenarios.

  /**
   * InitFrameData - 初始化帧数据（车辆状态、障碍物等）
   */
  auto status = InitFrameData(vehicle_state_provider, ego_info);
  if (!status.ok()) {
    AERROR << "failed to init frame:" << status.ToString();
    return status;  /**< 初始化失败返回错误状态 */
  }

  /**
   * CreateReferenceLineInfo - 创建参考线信息
   */
  if (!CreateReferenceLineInfo(reference_lines, segments)) {
    const std::string msg = "Failed to init reference line info.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  /**
   * 保存未来路线导航点
   * = 操作符赋值，拷贝所有元素
   */
  future_route_waypoints_ = future_route_waypoints;

  /**
   * 遍历打印参考线调试信息
   * for (auto& ref_line_info : reference_line_info_)
   *   - 范围for循环，auto&为引用避免拷贝
   *   - mutable对象才能调用非const方法
   */
  for (auto &reference_line_info : reference_line_info_) {
    reference_line_info.PrintReferenceSegmentDebugString();
  }
  return Status::OK();  /**< 初始化成功 */
}

/**
 * @brief 开放空间规划初始化
 *
 * 专门用于开放空间（如泊车）场景的Frame初始化。
 *
 * @param vehicle_state_provider 车辆状态提供者
 * @param ego_info 自车信息
 * @return Status 初始化状态
 *
 * 语法说明：
 * - return InitFrameData(...): 委托给另一个重载的InitFrameData
 */
Status Frame::InitForOpenSpace(
    const common::VehicleStateProvider *vehicle_state_provider,
    const EgoInfo *ego_info) {
  return InitFrameData(vehicle_state_provider, ego_info);
}

/**
 * @brief 帧数据初始化（核心初始化函数）
 *
 * 初始化帧的核心数据，包括：
 * 1. 高精地图
 * 2. 车辆状态
 * 3. 障碍物列表
 * 4. 交通信号灯
 * 5. Pad消息
 *
 * @param vehicle_state_provider 车辆状态提供者
 * @param ego_info 自车信息
 * @return Status 初始化状态
 *
 * 语法说明：
 * - CHECK_NOTNULL(ptr): 断言检查指针非空，失败则终止程序
 * - std::boolalpha: 将bool值输出为true/false字符串
 */
Status Frame::InitFrameData(
    const common::VehicleStateProvider *vehicle_state_provider,
    const EgoInfo *ego_info) {
  /**
   * hdmap_ - 获取高精地图单例指针
   * HDMapUtil::BaseMapPtr() - 静态方法，返回地图指针
   */
  hdmap_ = hdmap::HDMapUtil::BaseMapPtr();

  /**
   * CHECK_NOTNULL - 断言检查，失败会输出错误并终止程序
   * 不同于返回false，这里是致命错误
   */
  CHECK_NOTNULL(hdmap_);

  /**
   * vehicle_state_ - 从车辆状态提供者获取车辆状态
   * vehicle_state_provider->vehicle_state() - 返回车辆状态拷贝
   */
  vehicle_state_ = vehicle_state_provider->vehicle_state();

  /**
   * IsVehicleStateValid - 检查车辆状态是否有效
   * 例如检查位置、速度等是否在合理范围
   */
  if (!util::IsVehicleStateValid(vehicle_state_)) {
    AERROR << "Adc init point is not set";
    return Status(ErrorCode::PLANNING_ERROR, "Adc init point is not set");
  }

  /**
   * ADEBUG - Apollo调试级别日志
   * std::boolalpha - 输出bool为true/false而非1/0
   */
  ADEBUG << "Enabled align prediction time ? : " << std::boolalpha
         << FLAGS_align_prediction_time;

  /**
   * 预测时间对齐
   * 如果启用此功能，将预测障碍物的时间戳与规划起始时间对齐
   */
  if (FLAGS_align_prediction_time) {
    auto prediction = *(local_view_.prediction_obstacles); /**< 拷贝预测数据 */
    AlignPredictionTime(vehicle_state_.timestamp(), &prediction); /**< 对齐时间 */
    local_view_.prediction_obstacles->CopyFrom(prediction); /**< 更新预测数据 */
  }

  /**
   * 创建障碍物列表
   * Obstacle::CreateObstacles(*) - 静态工厂方法，从预测障碍物创建规划障碍物
   * 返回std::vector<std::unique_ptr<Obstacle>>
   * for循环遍历并调用AddObstacle添加到帧
   */
  for (auto &ptr :
       Obstacle::CreateObstacles(*local_view_.prediction_obstacles)) {
    AddObstacle(*ptr);  /**< 解引用获取Obstacle对象并添加 */
  }

  /**
   * 检查静止障碍物碰撞
   * planning_start_point_.v() - 获取起始点速度
   * < 1e-3 表示速度非常小（近似为零）
   * FindCollisionObstacle - 查找是否有碰撞
   */
  if (planning_start_point_.v() < 1e-3) {
    const auto *collision_obstacle = FindCollisionObstacle(ego_info);
    if (collision_obstacle != nullptr) {
      /**
       * 发生碰撞，记录错误并返回
       * absl::StrCat - Abseil字符串拼接
       */
      const std::string msg = absl::StrCat("Found collision with obstacle: ",
                                           collision_obstacle->Id());
      AERROR << msg;
      monitor_logger_buffer_.ERROR(msg);  /**< 发送错误到监控日志 */
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }

  /**
   * 读取交通信号灯信息
   */
  ReadTrafficLights();

  /**
   * 读取Pad消息驾驶动作
   */
  ReadPadMsgDrivingAction();

  return Status::OK();  /**< 初始化成功 */
}

/**
 * @brief 查找碰撞障碍物
 *
 * 检查自车多边形是否与任何障碍物有重叠。
 *
 * @param ego_info 自车信息
 * @return const Obstacle* 发生碰撞的障碍物指针，无碰撞返回nullptr
 *
 * 语法说明：
 * - Polygon2d - 二维多边形类
 * - ego_info->ego_box() - 获取自车包围盒
 * - HasOverlap() - 判断两个多边形是否有重叠
 */
const Obstacle *Frame::FindCollisionObstacle(const EgoInfo *ego_info) const {
  /**
   * 检查障碍物列表是否为空
   * obstacles_.Items() - 返回所有障碍物的vector
   */
  if (obstacles_.Items().empty()) {
    return nullptr;
  }

  /**
   * 创建自车多边形
   * Polygon2d构造函数接收Box2d包围盒
   */
  const auto &adc_polygon = Polygon2d(ego_info->ego_box());

  /**
   * 遍历所有障碍物检查碰撞
   */
  for (const auto &obstacle : obstacles_.Items()) {
    /**
     * IsVirtual - 判断是否为虚拟障碍物
     * 虚拟障碍物（如停车墙）不参与碰撞检查
     */
    if (obstacle->IsVirtual()) {
      continue;  /**< 跳过虚拟障碍物 */
    }

    /**
     * PerceptionPolygon - 获取障碍物感知多边形
     * HasOverlap - 检查是否有重叠
     */
    const auto &obstacle_polygon = obstacle->PerceptionPolygon();
    if (obstacle_polygon.HasOverlap(adc_polygon)) {
      return obstacle;  /**< 发生碰撞，返回障碍物 */
    }
  }
  return nullptr;  /**< 无碰撞 */
}

/**
 * 获取帧序列号
 * @return uint32_t 序列号
 *
 * 语法说明：
 * - 函数后加const表示const成员函数
 * - this指针是const的，不能修改成员变量
 */
uint32_t Frame::SequenceNum() const { return sequence_num_; }

/**
 * 获取调试字符串
 * @return std::string 格式化的调试信息
 *
 * 语法说明：
 * - absl::StrCat - 字符串拼接函数
 */
std::string Frame::DebugString() const {
  return absl::StrCat("Frame: ", sequence_num_);
}

/**
 * @brief 记录输入调试信息
 *
 * 将本帧的输入数据记录到debug消息中，用于调试和可视化。
 *
 * @param debug 调试消息指针
 *
 * 语法说明：
 * - mutable_xxx(): 获取可写指针
 * - CopyFrom(): 深拷贝消息
 * - *ptr: 解引用获取对象
 */
void Frame::RecordInputDebug(planning_internal::Debug *debug) {
  /**
   * 空指针检查
   */
  if (!debug) {
    ADEBUG << "Skip record input into debug";
    return;
  }

  /**
   * 获取可写的planning_data指针
   * mutable_planning_data() - 返回可写指针
   */
  auto *planning_debug_data = debug->mutable_planning_data();

  /**
   * 记录ADC位置
   */
  auto *adc_position = planning_debug_data->mutable_adc_position();
  adc_position->CopyFrom(*local_view_.localization_estimate);

  /**
   * 记录底盘信息
   */
  auto debug_chassis = planning_debug_data->mutable_chassis();
  debug_chassis->CopyFrom(*local_view_.chassis);

  /**
   * 记录路由信息（非导航模式）
   */
  if (!FLAGS_use_navigation_mode) {
    auto debug_routing = planning_debug_data->mutable_routing();
    debug_routing->CopyFrom(
        local_view_.planning_command->lane_follow_command());
  }

  /**
   * 记录预测头信息
   */
  planning_debug_data->mutable_prediction_header()->CopyFrom(
      local_view_.prediction_obstacles->header());
}

/**
 * @brief 对齐预测时间
 *
 * 将预测障碍物的时间戳调整为相对于规划起始时间。
 * 确保预测轨迹的时间与规划周期时间基准一致。
 *
 * @param planning_start_time 规划起始时间（秒）
 * @param prediction_obstacles 预测障碍物指针
 *
 * 语法说明：
 * - mutable_prediction_obstacle(): 获取可写的预测障碍物列表指针
 * - mutable_trajectory(): 获取可写的轨迹指针
 * - mutable_trajectory_point(): 获取可写的轨迹点列表指针
 * - relative_time: 相对时间，相对于规划起始点的时间
 */
void Frame::AlignPredictionTime(const double planning_start_time,
                                PredictionObstacles *prediction_obstacles) {
  /**
   * 检查预测数据是否有效
   */
  if (!prediction_obstacles || !prediction_obstacles->has_header() ||
      !prediction_obstacles->header().has_timestamp_sec()) {
    return;
  }

  /**
   * 获取预测消息头的时间戳
   */
  double prediction_header_time =
      prediction_obstacles->header().timestamp_sec();

  /**
   * 遍历所有预测障碍物
   * mutable_prediction_obstacle(): 返回repeated字段的可写迭代器
   */
  for (auto &obstacle : *prediction_obstacles->mutable_prediction_obstacle()) {
    /**
     * 遍历障碍物的所有预测轨迹
     * mutable_trajectory(): 获取轨迹列表的可写引用
     */
    for (auto &trajectory : *obstacle.mutable_trajectory()) {
      /**
       * 遍历轨迹的所有点
       * mutable_trajectory_point(): 获取轨迹点列表的可写引用
       */
      for (auto &point : *trajectory.mutable_trajectory_point()) {
        /**
         * 调整相对时间
         * original_relative_time = prediction_header_time + original_time
         * new_relative_time = original - planning_start_time
         */
        point.set_relative_time(prediction_header_time + point.relative_time() -
                                planning_start_time);
      }

      /**
       * 删除时间为负的轨迹点
       * 这些点表示过去的点，已经过时
       *
       * 条件：
       * - 轨迹点列表非空
       * - 第一个点的时间为负
       */
      if (!trajectory.trajectory_point().empty() &&
          trajectory.trajectory_point().begin()->relative_time() < 0) {
        /**
         * 找到第一个非负时间的点
         * while循环：条件为真时持续执行
         */
        auto it = trajectory.trajectory_point().begin();
        while (it != trajectory.trajectory_point().end() &&
               it->relative_time() < 0) {
          ++it;  /**< 前置递增 */
        }

        /**
         * erase(begin, it) - 删除从开始到it的所有元素
         * 删除所有负时间的点
         */
        trajectory.mutable_trajectory_point()->erase(
            trajectory.trajectory_point().begin(), it);
      }
    }
  }
}

/**
 * @brief 根据ID查找障碍物
 *
 * @param id 障碍物ID
 * @return Obstacle* 找到的障碍物指针，未找到返回nullptr
 *
 * 语法说明：
 * - obstacles_.Find(id) - IndexedQueue的Find方法
 */
Obstacle *Frame::Find(const std::string &id) { return obstacles_.Find(id); }

/**
 * @brief 添加障碍物到帧
 *
 * @param obstacle 要添加的障碍物（按值拷贝）
 *
 * 语法说明：
 * - obstacles_.Add(id, obstacle) - 添加障碍物
 * - obstacle.Id() - 获取障碍物ID
 */
void Frame::AddObstacle(const Obstacle &obstacle) {
  obstacles_.Add(obstacle.Id(), obstacle);
}

/**
 * @brief 读取交通信号灯信息
 *
 * 从本地视图读取交通信号灯检测结果，
 * 并检查数据时效性，过期数据将被忽略。
 *
 * 语法说明：
 * - Clock::NowInSeconds() - 获取当前时间（秒）
 * - FLAGS_signal_expire_time_sec - 信号过期时间阈值
 * - std::map<K, V>: 关联数组，按键存储
 */
void Frame::ReadTrafficLights() {
  traffic_lights_.clear();  /**< 清空之前的交通灯信息 */

  /**
   * 获取交通灯检测消息
   * local_view_.traffic_light - shared_ptr可能为空
   */
  const auto traffic_light_detection = local_view_.traffic_light;
  if (traffic_light_detection == nullptr) {
    return;  /**< 无数据直接返回 */
  }

  /**
   * 计算消息延迟
   * timestamp_sec() - 获取消息时间戳
   * Clock::NowInSeconds() - 获取当前时间
   * delay > 0 表示消息是过去的（延迟）
   */
  const double delay =
      traffic_light_detection->header().timestamp_sec() - Clock::NowInSeconds();

  /**
   * 检查是否过期
   */
  if (delay > FLAGS_signal_expire_time_sec) {
    ADEBUG << "traffic signals msg is expired, delay = " << delay
           << " seconds.";
    return;  /**< 过期数据不处理 */
  }

  /**
   * 遍历所有检测到的交通灯
   * traffic_light_detection->traffic_light() - 返回repeated字段
   * traffic_light.id() - 获取交通灯ID
   * traffic_lights_[id] = &traffic_light - 存入map
   */
  for (const auto &traffic_light : traffic_light_detection->traffic_light()) {
    traffic_lights_[traffic_light.id()] = &traffic_light;
  }
}

/**
 * @brief 获取指定ID的交通信号灯
 *
 * @param traffic_light_id 交通灯ID
 * @return perception::TrafficLight 交通灯信息
 *
 * 语法说明：
 * - apollo::common::util::FindPtrOrNull - 在map中查找键值
 * - 如果未找到，返回UNKNOWN颜色的默认交通灯
 */
perception::TrafficLight Frame::GetSignal(
    const std::string &traffic_light_id) const {
  /**
   * FindPtrOrNull - 在map中查找键
   * 返回找到的指针，未找到返回nullptr
   */
  const auto *result =
      apollo::common::util::FindPtrOrNull(traffic_lights_, traffic_light_id);

  /**
   * 未找到返回默认交通灯
   */
  if (result == nullptr) {
    perception::TrafficLight traffic_light;
    traffic_light.set_id(traffic_light_id);
    traffic_light.set_color(perception::TrafficLight::UNKNOWN); /**< 颜色未知 */
    traffic_light.set_confidence(0.0);   /**< 置信度为零 */
    traffic_light.set_tracking_time(0.0); /**< 跟踪时间为零 */
    return traffic_light;
  }
  return *result;  /**< 返回找到的交通灯 */
}

/**
 * @brief 读取Pad消息驾驶动作
 *
 * 从本地视图读取Pad消息的动作类型。
 * Pad消息通常来自车载交互设备。
 *
 * 语法说明：
 * - local_view_.pad_msg - shared_ptr可能为空
 * - pad_msg->has_action() - 检查是否有action字段
 * - pad_msg->action() - 获取action值
 */
void Frame::ReadPadMsgDrivingAction() {
  if (local_view_.pad_msg) {  /**< 检查指针非空 */
    if (local_view_.pad_msg->has_action()) {  /**< 检查has_字段 */
      pad_msg_driving_action_ = local_view_.pad_msg->action(); /**< 获取动作 */
    }
  }
}

/**
 * @brief 重置Pad消息驾驶动作
 *
 * 将驾驶动作重置为NONE（无动作）。
 */
void Frame::ResetPadMsgDrivingAction() {
  pad_msg_driving_action_ = PadMessage::NONE;
}

/**
 * @brief 查找可驾驶的参考线信息
 *
 * 遍历所有参考线，找到代价最小且可驾驶的参考线。
 * 这通常是多参考线规划中的主参考线选择逻辑。
 *
 * @return const ReferenceLineInfo* 找到的参考线指针，未找到返回nullptr
 *
 * 语法说明：
 * - std::numeric_limits<double>::infinity() - double类型的正无穷
 * - IsDrivable() - 判断参考线是否可驾驶
 * - Cost() - 获取参考线代价
 */
const ReferenceLineInfo *Frame::FindDriveReferenceLineInfo() {
  double min_cost = std::numeric_limits<double>::infinity(); /**< 初始化为无穷大 */
  drive_reference_line_info_ = nullptr;  /**< 初始化为空 */

  /**
   * 遍历所有参考线
   * reference_line_info_ - std::list<ReferenceLineInfo>
   */
  for (const auto &reference_line_info : reference_line_info_) {
    /**
     * 检查是否可驾驶且代价更小
     */
    if (reference_line_info.IsDrivable() &&
        reference_line_info.Cost() < min_cost) {
      drive_reference_line_info_ = &reference_line_info; /**< 更新最优参考线 */
      min_cost = reference_line_info.Cost();            /**< 更新最小代价 */
    }
  }
  return drive_reference_line_info_;  /**< 返回最优参考线 */
}

/**
 * @brief 查找目标参考线信息
 *
 * 优先返回换道路径的参考线，如果没有则返回首条参考线。
 *
 * @return const ReferenceLineInfo* 目标参考线指针
 *
 * 语法说明：
 * - IsChangeLanePath() - 判断是否为换道路径
 */
const ReferenceLineInfo *Frame::FindTargetReferenceLineInfo() {
  const ReferenceLineInfo *target_reference_line_info = nullptr;

  for (const auto &reference_line_info : reference_line_info_) {
    /**
     * 优先返回换道路径
     */
    if (reference_line_info.IsChangeLanePath()) {
      return &reference_line_info;
    }
    /**
     * 否则记录第一个作为后备
     */
    target_reference_line_info = &reference_line_info;
  }
  return target_reference_line_info;
}

/**
 * @brief 查找失败的参考线信息
 *
 * 查找不成功的换道路径参考线。
 *
 * @return const ReferenceLineInfo* 失败的参考线指针，未找到返回nullptr
 *
 * 语法说明：
 * - !IsDrivable() - 表示不可驾驶
 * - IsChangeLanePath() - 表示是换道路径
 */
const ReferenceLineInfo *Frame::FindFailedReferenceLineInfo() {
  for (const auto &reference_line_info : reference_line_info_) {
    /**
     * 找到不成功的换道路径
     */
    if (!reference_line_info.IsDrivable() &&
        reference_line_info.IsChangeLanePath()) {
      return &reference_line_info;
    }
  }
  return nullptr;
}

/**
 * 获取驾驶参考线信息
 * @return const ReferenceLineInfo* 之前FindDriveReferenceLineInfo找到的参考线
 */
const ReferenceLineInfo *Frame::DriveReferenceLineInfo() const {
  return drive_reference_line_info_;
}

/**
 * @brief 获取所有障碍物列表
 *
 * @return std::vector<const Obstacle*> 障碍物指针向量
 *
 * 语法说明：
 * - obstacles_.Items() - 返回所有障碍物的vector
 * - const T*: 常量指针，指向的元素不可修改
 */
const std::vector<const Obstacle *> Frame::obstacles() const {
  return obstacles_.Items();
}

}  // namespace planning
}  // namespace apollo
