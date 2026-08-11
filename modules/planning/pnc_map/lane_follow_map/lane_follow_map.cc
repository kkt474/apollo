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
 ******************************************************************************/
/**
 * @file lane_follow_map.cc
 *
 * @brief 车道跟随地图实现文件
 *
 * 功能说明：
 * 本文件实现了LaneFollowMap类，是Apollo规划模块中处理车道跟随场景的核心地图管理类
 * 负责管理与路由相关的地图数据，包括：
 * - 路由航点索引管理
 * - 车辆在路由上的位置更新
 * - 路径段（RouteSegments）的生成与扩展
 * - 目的地距离计算
 *
 * 核心概念：
 * - RouteSegments：路由段列表，描述车辆可行驶的路径
 * - LaneWaypoint：车道航点，包含车道指针和沿车道的距离s
 * - LaneSegment：车道段，包含车道的起止距离
 *
 * C++语法说明：
 * - #include：预处理指令，将其他文件内容包含到当前文件
 * - namespace：命名空间，用于组织代码，避免命名冲突
 * - class：类声明，用户定义类型，包含数据成员和成员函数
 * - using：类型别名声明，将长类型名简化为短名
 **/

#include "modules/planning/pnc_map/lane_follow_map/lane_follow_map.h"

/**
 * @brief 标准库头文件
 *
 * C++语法说明：
 * - <algorithm>：提供std::max、std::min、std::find等算法函数
 * - <limits>：提供std::numeric_limits获取数值类型极限值
 */
#include <algorithm>
#include <limits>

/**
 * @brief Abseil字符串处理库
 *
 * C++语法说明：
 * - absl/strings/str_cat.h：提供字符串拼接功能
 * - ::运算符：作用域解析运算符，访问命名空间内的符号
 */
#include "absl/strings/str_cat.h"
#include "google/protobuf/text_format.h"

/**
 * @brief Apollo地图ID消息定义
 *
 * C++语法说明：
 * - "modules/common_msgs/map_msgs/map_id.pb.h"：
 *   protobuf生成的头文件
 *   protobuf是一种序列化数据结构的方法
 *   .pb.h是protobuf编译器生成的C++头文件
 */
#include "modules/common_msgs/map_msgs/map_id.pb.h"

/**
 * @brief Cyber RT日志系统
 *
 * C++语法说明：
 * - cyber/：ApolloCyber RT框架的命名空间
 * - common/log.h：提供日志宏如AERROR、ADEBUG、AINFO
 * - AERROR：错误级别日志
 * - ADEBUG：调试级别日志
 * - ACHECK：断言宏，类似CHECK但用于调试
 */
#include "cyber/common/log.h"

/**
 * @brief Apollo工具类
 *
 * C++语法说明：
 * - PointFactory：点工厂类，用于创建各种格式的坐标点
 * - StringUtil：字符串工具类
 * - Util：通用工具类
 */
#include "modules/common/util/point_factory.h"
#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"

/**
 * @brief 高精地图工具
 *
 * C++语法说明：
 * - hdmap/hdmap_util.h：HDMap相关工具函数
 * - HDMapUtil::BaseMapPtr()：静态方法获取地图单例指针
 */
#include "modules/map/hdmap/hdmap_util.h"

/**
 * @brief 调试信息打印
 *
 * 功能说明：
 * 提供规划模块的调试信息打印功能
 */
#include "modules/planning/planning_base/common/util/print_debug_info.h"

/**
 * @brief 规划模块配置参数
 *
 * C++语法说明：
 * - gflags/planning_gflags.h：
 *   gflags是Google的命令行标志库
 *   FLAGS_开头的是全局配置变量
 *   通过DECLARE_xxx宏声明，定义在.cc文件中
 */
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
/**
 * @brief Apollo主命名空间
 *
 * 功能说明：
 * 所有Apollo模块的代码都在apollo命名空间下
 * 避免与其他库的命名冲突
 */
namespace planning {
/**
 * @brief 规划模块的命名空间
 *
 * 功能说明：
 * planning命名空间包含所有规划相关的类和函数
 */

/**
 * @brief 类型别名声明
 *
 * C++语法说明：
 * using别名声明，将长类型名简化为短名
 * 类似于typedef，但语法更直观
 *
 * - apollo::common::PointENU：
 *   Apollo通用模块中的ENU坐标系点
 *   ENU是East-North-Up局部坐标系
 *
 * - apollo::common::VehicleState：
 *   车辆状态，包含位置、速度、航向等信息
 *
 * - apollo::common::util::PointFactory：
 *   点工厂类，用于创建坐标点
 *
 * - apollo::routing::RoutingResponse：
 *   路由响应消息类型
 */
using apollo::common::PointENU;
using apollo::common::VehicleState;
using apollo::common::util::PointFactory;
using apollo::routing::RoutingResponse;

namespace {
/**
 * @brief 匿名命名空间
 *
 * C++语法说明：
 * 匿名命名空间内的符号仅在当前文件可见
 * 相当于static关键字的作用，但更符合现代C++风格
 * 用于定义文件内部使用的常量
 */

/**
 * @brief 轨迹近似最大误差（米）
 *
 * 功能说明：
 * 在轨迹近似计算中使用的最大允许误差
 * 用于控制轨迹拟合的精度
 *
 * C++语法说明：
 * - const double：
 *   常量double类型，编译期确定
 * - kTrajectoryApproximationMaxError：
 *   k开头是Apollo的常量命名惯例
 *   表示这是一个配置常量
 */
const double kTrajectoryApproximationMaxError = 2.0;

}  // namespace

/**
 * @brief 构造函数
 *
 * 功能说明：
 * 默认构造函数，初始化高精地图指针
 * 使用HDMapUtil获取全局地图实例的指针
 *
 * C++语法说明：
 * - LaneFollowMap()：
 *   类名加括号表示构造函数
 *   没有返回类型声明
 *
 * - : hdmap_(hdmap::HDMapUtil::BaseMapPtr())：
 *   初始化列表语法，在构造函数的函数体执行前初始化成员变量
 *   效率比在函数体内赋值更高
 *   BaseMapPtr()是静态方法，通过类名::直接调用
 *   返回全局唯一的高精地图实例的共享指针
 *
 * - hdmap_：
 *   类成员变量，存储高精地图的指针
 *   _后缀是Apollo的成员变量命名惯例
 */
LaneFollowMap::LaneFollowMap() : hdmap_(hdmap::HDMapUtil::BaseMapPtr()) {}

/**
 * @brief 检查是否可以处理该规划命令
 *
 * @param command 规划命令引用
 * @return bool 如果可以处理返回true，否则返回false
 *
 * 功能说明：
 * 判断传入的规划命令是否是车道跟随类型
 * 只有车道跟随命令才能被LaneFollowMap处理
 *
 * C++语法说明：
 * - const planning::PlanningCommand &command：
 *   const：承诺不修改参数
 *   &：引用，避免参数拷贝
 *   PlanningCommand：protobuf消息类型
 *
 * - command.has_lane_follow_command()：
 *   protobuf的has_方法
 *   用于检查可选字段是否已设置
 *   如果字段存在返回true
 *
 * - const后缀：
 *   表示这是一个const成员函数
 *   不会修改类的成员变量
 */
bool LaneFollowMap::CanProcess(const planning::PlanningCommand &command) const {
  return command.has_lane_follow_command();
}

/**
 * @brief 将路由航点转换为HDMap航点
 *
 * @param waypoint 路由模块的车道航点
 * @return hdmap::LaneWaypoint HDMap的车道航点
 *
 * 功能说明：
 * 将routing::LaneWaypoint转换为hdmap::LaneWaypoint
 * routing模块和hdmap模块使用不同的车道表示方式
 * 需要通过车道ID从HDMap中获取完整的车道信息
 *
 * C++语法说明：
 * - const routing::LaneWaypoint &waypoint：
 *   routing模块定义的车道航点结构
 *   const引用避免拷贝
 *
 * - auto lane = ...：
 *   auto：自动类型推导
 *   编译器根据右侧表达式推断变量类型
 *   这里推断为hdmap::LaneInfoConstPtr类型
 *
 * - hdmap_->GetLaneById(hdmap::MakeMapId(waypoint.id()))：
 *   hdmap_->：hdmap_是类成员指针，用->访问成员
 *   GetLaneById()：根据车道ID获取车道信息指针
 *   MakeMapId()：将字符串ID转换为hdmap::Id类型
 *   waypoint.id()：获取路由航点的车道ID字符串
 *
 * - ACHECK(lane) << "Invalid lane id: " << waypoint.id()：
 *   Apollo的断言宏
 *   如果条件为false，输出错误信息并终止程序
 *   <<运算符用于连接字符串
 *   类似std::cout的流输出语法
 *
 * - return hdmap::LaneWaypoint(lane, waypoint.s())：
 *   构造并返回hdmap::LaneWaypoint对象
 *   使用lane指针和路由航点的s值（沿车道的距离）
 */
hdmap::LaneWaypoint LaneFollowMap::ToLaneWaypoint(
    const routing::LaneWaypoint &waypoint) const {
  auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(waypoint.id()));
  ACHECK(lane) << "Invalid lane id: " << waypoint.id();
  return hdmap::LaneWaypoint(lane, waypoint.s());
}

/**
 * @brief 将路由段转换为HDMap段
 *
 * @param segment 路由模块的车道段
 * @return hdmap::LaneSegment HDMap的车道段
 *
 * 功能说明：
 * 将routing::LaneSegment转换为hdmap::LaneSegment
 * 包含车道的起止距离信息
 *
 * C++语法说明：
 * - routing::LaneSegment：
 *   路由模块定义的车道段结构
 *   包含id、start_s、end_s等字段
 *
 * - ToLaneSegment内部逻辑：
 *   1. 通过车道ID获取车道指针
 *   2. 检查车道是否有效
 *   3. 构造hdmap::LaneSegment
 */
hdmap::LaneSegment LaneFollowMap::ToLaneSegment(
    const routing::LaneSegment &segment) const {
  auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(segment.id()));
  ACHECK(lane) << "Invalid lane id: " << segment.id();
  return hdmap::LaneSegment(lane, segment.start_s(), segment.end_s());
}

/**
 * @brief 更新下一个路由航点索引
 *
 * @param cur_index 当前航点索引
 *
 * 功能说明：
 * 根据车辆当前位置更新下一个需要处理的路由航点索引
 * 处理车辆前进和后退两种情况
 *
 * 工作原理：
 * 1. 首先检查索引边界
 * 2. 如果车辆后退（cur_index小于当前索引），向前搜索
 * 3. 如果车辆前进，向后搜索
 * 4. 更新next_routing_waypoint_index_
 *
 * C++语法说明：
 * - if (cur_index < 0)：
 *   条件判断，负索引无效
 *
 * - static_cast<int>(route_indices_.size())：
 *   静态类型转换
 *   将size_t转换为int
 *   size()返回的是size_t类型（无符号）
 *
 * - while (条件) { ... }：
 *   while循环
 *   条件为true时重复执行循环体
 *
 * - --next_routing_waypoint_index_：
 *   前置递减运算符
 *   等同于 next_routing_waypoint_index_ = next_routing_waypoint_index_ - 1
 *
 * - &&：逻辑与运算符
 *   所有条件都为true时整个表达式才为true
 *
 * - routing_waypoint_index_[i].index：
 *   []数组下标运算符访问向量元素
 *   .index访问结构体成员
 *
 * - adc_waypoint_.s：
 *   结构体成员访问
 *   adc_waypoint_是hdmap::LaneWaypoint类型
 *   s是沿车道的累计距离
 */
void LaneFollowMap::UpdateNextRoutingWaypointIndex(int cur_index) {
  if (cur_index < 0) {
    next_routing_waypoint_index_ = 0;
    return;
  }
  if (cur_index >= static_cast<int>(route_indices_.size())) {
    next_routing_waypoint_index_ = routing_waypoint_index_.size() - 1;
    return;
  }
  /**
   * @brief 向后搜索逻辑
   *
   * 场景说明：
   * 当车辆后退行驶时，需要沿路由向后搜索
   * 例如车辆倒车或重新定位
   *
   * 条件说明：
   * - next_routing_waypoint_index_ != 0：
   *   防止索引越界
   * - next_routing_waypoint_index_ < routing_waypoint_index_.size()：
   *   确保在有效范围内
   * - routing_waypoint_index_[next_routing_waypoint_index_].index > cur_index：
   *   当前索引对应的路由位置在当前位置之后
   *   说明车辆已经后退到上一个航点之前
   */
  while (next_routing_waypoint_index_ != 0 &&
         next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         routing_waypoint_index_[next_routing_waypoint_index_].index >
             cur_index) {
    --next_routing_waypoint_index_;
  }
  /**
   * @brief 精确向后搜索
   *
   * 场景说明：
   * 在索引相同的情况下，检查s值
   * 如果车辆在当前航点的s值之前，说明还需要后退
   */
  while (next_routing_waypoint_index_ != 0 &&
         next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         routing_waypoint_index_[next_routing_waypoint_index_].index ==
             cur_index &&
         adc_waypoint_.s <
             routing_waypoint_index_[next_routing_waypoint_index_].waypoint.s) {
    --next_routing_waypoint_index_;
  }
  /**
   * @brief 向前搜索逻辑
   *
   * 场景说明：
   * 车辆正常前进时，沿路由向前搜索
   * 更新next_routing_waypoint_index_到第一个大于等于cur_index的位置
   *
   * C++语法说明：
   * - ++next_routing_waypoint_index_：
   *   前置递增运算符
   *   先递增，再使用新值
   */
  while (next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         routing_waypoint_index_[next_routing_waypoint_index_].index <
             cur_index) {
    ++next_routing_waypoint_index_;
  }
  /**
   * @brief 精确向前搜索
   *
   * 场景说明：
   * 在索引相同的情况下，检查s值
   * 如果车辆已经超过当前航点的s值，需要继续前进
   */
  while (next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         cur_index ==
             routing_waypoint_index_[next_routing_waypoint_index_].index &&
         adc_waypoint_.s >=
             routing_waypoint_index_[next_routing_waypoint_index_].waypoint.s) {
    ++next_routing_waypoint_index_;
  }
  if (next_routing_waypoint_index_ >= routing_waypoint_index_.size()) {
    next_routing_waypoint_index_ = routing_waypoint_index_.size() - 1;
  }
}

/**
 * @brief 获取剩余路由航点
 *
 * @return std::vector<routing::LaneWaypoint> 从下一个航点开始的剩余航点列表
 *
 * 功能说明：
 * 返回从next_routing_waypoint_index_开始的所有路由航点
 * 用于规划模块获取前方还有多少航点需要处理
 *
 * C++语法说明：
 * - const auto &waypoints =
 *     last_command_.lane_follow_command().routing_request().waypoint()：
 *   const引用避免拷贝
 *   auto自动类型推导
 *   链式调用获取嵌套protobuf消息
 *   waypoint()返回repeated字段（类似vector）
 *
 * - std::vector<routing::LaneWaypoint>(...)：
 *   构造一个vector，使用迭代器范围初始化
 *
 * - waypoints.begin() + next_routing_waypoint_index_：
 *   begin()返回指向首元素的迭代器
 *   + 整数表示向前移动迭代器位置
 *   end()返回指向末尾的迭代器
 *   这种构造方式创建子向量
 */
std::vector<routing::LaneWaypoint> LaneFollowMap::FutureRouteWaypoints() const {
  const auto &waypoints =
      last_command_.lane_follow_command().routing_request().waypoint();
  return std::vector<routing::LaneWaypoint>(
      waypoints.begin() + next_routing_waypoint_index_, waypoints.end());
}

/**
 * @brief 获取目的地航点
 *
 * @param end_point 输出参数，指向目的地航点的共享指针的引用
 *
 * 功能说明：
 * 从路由请求中获取最后一个航点作为目的地
 * 将结果通过引用参数返回
 *
 * C++语法说明：
 * - std::shared_ptr<routing::LaneWaypoint> &end_point：
 *   shared_ptr：智能指针，引用计数管理对象生命周期
 *   &：引用参数，用于输出结果
 *
 * - !last_command_.has_lane_follow_command()：
 *   逻辑非运算符
 *   检查车道跟随命令是否存在
 *
 * - std::make_shared<routing::LaneWaypoint>()：
 *   创建shared_ptr并构造新对象
 *   相当于 new routing::LaneWaypoint() 的智能指针版本
 *
 * - end_point->CopyFrom(*(...))：
 *   ->：智能指针解引用访问成员
 *   CopyFrom：protobuf消息的拷贝方法
 *   *(...).rbegin()：解引用逆向迭代器首元素
 *   rbegin()返回指向最后一个元素的逆向迭代器
 */
void LaneFollowMap::GetEndLaneWayPoint(
    std::shared_ptr<routing::LaneWaypoint> &end_point) const {
  if (!last_command_.has_lane_follow_command() ||
      !last_command_.lane_follow_command().has_routing_request()) {
    end_point = nullptr;
    return;
  }
  const auto &routing_request =
      last_command_.lane_follow_command().routing_request();
  if (routing_request.waypoint().size() < 1) {
    end_point = nullptr;
    return;
  }
  end_point = std::make_shared<routing::LaneWaypoint>();
  end_point->CopyFrom(*(routing_request.waypoint().rbegin()));
}

/**
 * @brief 根据ID获取车道信息
 *
 * @param id 车道ID
 * @return hdmap::LaneInfoConstPtr 车道信息常量指针
 *
 * 功能说明：
 * 根据车道ID从HDMap中获取车道详细信息
 * 用于查询特定车道的几何和属性信息
 *
 * C++语法说明：
 * - nullptr == hdmap_：
 *   将nullptr放在左边是一种编程习惯
 *   防止意外的赋值操作（如写成hdmap_ = nullptr）
 *   这是防御性编程的一种技巧
 *
 * - return hdmap_->GetLaneById(id)：
 *   hdmap_->：指针解引用访问成员
 *   GetLaneById：HDMap查询方法
 */
hdmap::LaneInfoConstPtr LaneFollowMap::GetLaneById(const hdmap::Id &id) const {
  if (nullptr == hdmap_) {
    return nullptr;
  }
  return hdmap_->GetLaneById(id);
}

/**
 * @brief 验证规划命令有效性
 *
 * @param command 待验证的规划命令
 * @return bool 命令有效返回true
 *
 * 功能说明：
 * 全面检查规划命令的合法性
 * 包括命令类型、路由请求、航点完整性等
 *
 * 验证项目：
 * 1. 必须是车道跟随命令
 * 2. 道路数量必须大于0
 * 3. 路由请求必须存在且航点数量至少为2
 * 4. 每个航点必须包含id和s值
 *
 * C++语法说明：
 * - !CanProcess(command)：
 *   逻辑非，检查命令类型
 *
 * - routing.road_size()：
 *   protobuf的size()方法
 *   返回repeated字段的元素数量
 *
 * - routing.has_routing_request()：
 *   protobuf的has_方法检查字段存在
 *
 * - routing.routing_request().waypoint_size() < 2：
 *   链式调用
 *   waypoint_size()返回航点数量
 *   至少需要起点和终点
 *
 * - AERROR << "Routing does not have request."：
 *   AERROR：Apollo错误日志宏
 *   类似std::cerr的流输出
 *
 * - for (const auto &waypoint : routing.routing_request().waypoint())：
 *   范围for循环
 *   遍历protobuf的repeated字段
 *   const引用避免拷贝，提高效率
 *
 * - !waypoint.has_id() || !waypoint.has_s()：
 *   逻辑或，只要有一个条件满足就继续
 *   检查航点的必要字段是否存在
 */
bool LaneFollowMap::IsValid(const planning::PlanningCommand &command) const {
  if (!CanProcess(command)) {
    return false;
  }
  const auto &routing = command.lane_follow_command();
  const int num_road = routing.road_size();
  if (num_road == 0) {
    return false;
  }
  if (!routing.has_routing_request() ||
      routing.routing_request().waypoint_size() < 2) {
    AERROR << "Routing does not have request.";
    return false;
  }
  for (const auto &waypoint : routing.routing_request().waypoint()) {
    if (!waypoint.has_id() || !waypoint.has_s()) {
      AERROR << "Routing waypoint has no lane_id or s.";
      return false;
    }
  }
  return true;
}

/**
 * @brief 更新路由范围
 *
 * @param adc_index 自车当前位置索引
 *
 * 功能说明：
 * 根据自车位置计算需要关注的路由范围
 * 更新range_lane_ids_、range_start_、range_end_
 *
 * 算法说明：
 * 从adc_index向前遍历，将经过的车道ID加入集合
 * 直到遇到已见过的车道ID（表示环路）或遍历完所有段
 *
 * C++语法说明：
 * - range_lane_ids_.clear()：
 *   clear()清空unordered_set
 *
 * - std::max(0, adc_index - 1)：
 *   std::max是<algorithm>中的模板函数
 *   返回两个值中的较大者
 *   避免range_start_变为负数
 *
 * - range_lane_ids_.count(lane_id) != 0：
 *   count()返回集合中指定元素的数量
 *   非0表示元素已存在
 *
 * - range_lane_ids_.insert(lane_id)：
 *   insert()将元素加入集合
 *   返回pair<iterator, bool>，但这里忽略
 *
 * - ++range_end_：
 *   后置递增运算符（虽然这里用的是前置）
 *   等同于 range_end_ = range_end_ + 1
 */
void LaneFollowMap::UpdateRoutingRange(int adc_index) {
  range_lane_ids_.clear();
  range_start_ = std::max(0, adc_index - 1);
  range_end_ = range_start_;
  while (range_end_ < static_cast<int>(route_indices_.size())) {
    const auto &lane_id = route_indices_[range_end_].segment.lane->id().id();
    if (range_lane_ids_.count(lane_id) != 0) {
      break;
    }
    range_lane_ids_.insert(lane_id);
    ++range_end_;
  }
}

/**
 * @brief 更新车辆状态
 *
 * @param vehicle_state 车辆当前状态
 * @return bool 更新成功返回true
 *
 * 功能说明：
 * 根据车辆状态更新内部维护的车辆位置信息
 * 包括：
 * 1. 验证路由命令有效性
 * 2. 检测位置是否重置
 * 3. 获取最近的路由航点
 * 4. 更新路由索引和范围
 *
 * C++语法说明：
 * - const VehicleState &vehicle_state：
 *   常量引用参数，避免拷贝
 *
 * - common::util::DistanceXY(adc_state_, vehicle_state)：
 *   ::作用域解析运算符
 *   调用apollo common模块的工具函数
 *   DistanceXY计算二维欧氏距离
 *
 * - FLAGS_replan_lateral_distance_threshold：
 *   gflags定义的配置变量
 *   横向重规划距离阈值
 *
 * - FLAGS_replan_longitudinal_distance_threshold：
 *   纵向重规划距离阈值
 *
 * - GetNearestPointFromRouting(...)：
 *   根据车辆位置找到最近的路由航点
 *   通过输出参数返回结果
 *
 * - adc_route_index_ = route_index：
 *   更新当前路由索引
 *   用于后续的路径段查询
 */
bool LaneFollowMap::UpdateVehicleState(const VehicleState &vehicle_state) {
  if (!IsValid(last_command_)) {
    AERROR << "The routing is invalid when updating vehicle state.";
    route_segments_lane_ids_.clear();
    return false;
  }
  /**
   * @brief 位置重置检测
   *
   * 功能说明：
   * 如果当前位置与上次记录的位置距离超过阈值
   * 说明车辆位置被重置（如GPS跳变）
   * 需要重置路由索引，不触发重规划
   *
   * C++语法说明：
   * - (条件1) || (条件2)：
   *   逻辑或运算符
   *   满足任一条件即触发重置检测
   * - common::util::DistanceXY：
   *   计算两个车辆状态之间的二维距离
   * - FLAGS_xxx：
   *   gflags配置的全局参数
   */
  if (!adc_state_.has_x() ||
      (common::util::DistanceXY(adc_state_, vehicle_state) >
       FLAGS_replan_lateral_distance_threshold +
           FLAGS_replan_longitudinal_distance_threshold)) {
    next_routing_waypoint_index_ = 0;
    adc_route_index_ = -1;
    stop_for_destination_ = false;
  }

  adc_state_ = vehicle_state;
  // waypoint: 自车当前位置在全局路线上所匹配的点的数据信息
  if (!GetNearestPointFromRouting(vehicle_state, &adc_waypoint_)) {
    AERROR << "Failed to get waypoint from routing with point: " << "("
           << vehicle_state.x() << ", " << vehicle_state.y() << ", "
           << vehicle_state.z() << ").";
    route_segments_lane_ids_.clear();
    return false;
  }
  int route_index = GetWaypointIndex(adc_waypoint_);
  if (route_index < 0 ||
      route_index >= static_cast<int>(route_indices_.size())) {
    AERROR << "Cannot find waypoint: " << adc_waypoint_.DebugString();
    return false;
  }
  ADEBUG << "adc_waypoint_" << adc_waypoint_.DebugString() << "route_index"
         << route_index;
  UpdateNextRoutingWaypointIndex(route_index);
  adc_route_index_ = route_index;
  UpdateRoutingRange(adc_route_index_);

  if (routing_waypoint_index_.empty()) {
    AERROR << "No routing waypoint index.";
    return false;
  }

  if (next_routing_waypoint_index_ == routing_waypoint_index_.size() - 1) {
    stop_for_destination_ = true;
  }
  return true;
}

/**
 * @brief 更新规划命令
 *
 * @param command 新的规划命令
 * @return bool 更新成功返回true
 *
 * 功能说明：
 * 解析并存储新的规划命令
 * 构建路由索引和路径段信息
 *
 * 处理流程：
 * 1. 检查命令类型
 * 2. 调用基类方法更新命令
 * 3. 清空并重建路由索引
 * 4. 遍历道路-通道-车道三级结构
 * 5. 提取目的地车道段
 * 6. 建立航点-索引映射
 *
 * C++语法说明：
 * - for (int road_index = 0; road_index < routing.road_size(); ++road_index)：
 *   传统for循环
 *   遍历routing中的所有道路
 *   road_size()返回道路数量
 *
 * - routing.road(road_index)：
 *   protobuf的[]运算符
 *   返回指定索引的道路
 *
 * - ++passage_index：
 *   后置递增，先使用当前值再递增
 *
 * - all_lane_ids_.insert(...)：
 *   unordered_set的insert方法
 *   自动去重
 *
 * - route_indices_.emplace_back()：
 *   emplace_back比push_back更高效
 *   直接在容器末尾构造对象
 *   避免拷贝或移动
 *
 * - route_indices_.back().segment = ...：
 *   back()返回最后一个元素的引用
 *   然后访问其segment成员
 *
 * - std::array<int, 3> index = {road_index, passage_index, lane_index}：
 *   固定大小数组，存储三维索引
 *   用于唯一标识路由中的位置
 */
bool LaneFollowMap::UpdatePlanningCommand(
    const planning::PlanningCommand &command) {
  if (!CanProcess(command)) {
    AERROR << "Command cannot be processed by LaneFollowMap!";
    return false;
  }
  if (!PncMapBase::UpdatePlanningCommand(command)) {
    return false;
  }
  const auto &routing = command.lane_follow_command();
  range_lane_ids_.clear();
  route_indices_.clear();
  all_lane_ids_.clear();
  route_segments_lane_ids_.clear();
  /**
   * @brief 三层嵌套遍历：道路 -> 通道 -> 车道段
   *
   * 数据结构说明：
   * - Road由多个Passage组成
   * - Passage由多个LaneSegment组成
   * - LaneSegment是路由的最小单位
   */
  for (int road_index = 0; road_index < routing.road_size(); ++road_index) {
    const auto &road_segment = routing.road(road_index);
    for (int passage_index = 0; passage_index < road_segment.passage_size();
         ++passage_index) {
      const auto &passage = road_segment.passage(passage_index);
      for (int lane_index = 0; lane_index < passage.segment_size();
           ++lane_index) {
        all_lane_ids_.insert(passage.segment(lane_index).id());
        route_indices_.emplace_back();
        route_indices_.back().segment =
            ToLaneSegment(passage.segment(lane_index));
        if (route_indices_.back().segment.lane == nullptr) {
          AERROR << "Failed to get lane segment from passage.";
          return false;
        }
        route_indices_.back().index = {road_index, passage_index, lane_index};

        /**
         * @brief 识别目的地车道段
         *
         * 条件说明：
         * 1. road_index == routing.road_size() - 1：最后一条道路
         * 2. lane_index == passage.segment_size() - 1：最后一个车道
         * 3. passage.can_exit()：该通道可以驶出
         *
         * C++语法说明：
         * - routing.road_size() - 1：
         *   size()返回size_t，需要减1转为int
         * - route_indices_.back()：
         *   back()返回最后元素引用
         */
        if (road_index == routing.road_size() - 1 &&
            lane_index == passage.segment_size() - 1 && passage.can_exit()) {
          dest_lane_segment_ = passage.segment(lane_index);
        }
      }
    }
  }

  range_start_ = 0;
  range_end_ = 0;
  adc_route_index_ = -1;
  next_routing_waypoint_index_ = 0;
  UpdateRoutingRange(adc_route_index_);

  /**
   * @brief 建立航点索引映射
   *
   * 功能说明：
   * 将请求中的航点映射到route_indices_中的对应位置
   * 用于快速查找特定航点
   *
   * 算法说明：
   * 双重循环：
   * - 外层遍历route_indices_
   * - 内层遍历request_waypoints
   * - WithinLaneSegment检查航点是否在路段内
   */
  routing_waypoint_index_.clear();
  const auto &request_waypoints = routing.routing_request().waypoint();
  if (request_waypoints.empty()) {
    AERROR << "Invalid routing: no request waypoints.";
    return false;
  }
  int i = 0;
  for (size_t j = 0; j < route_indices_.size(); ++j) {
    while (i < request_waypoints.size() &&
           hdmap::RouteSegments::WithinLaneSegment(route_indices_[j].segment,
                                                   request_waypoints.Get(i))) {
      routing_waypoint_index_.emplace_back(
          hdmap::LaneWaypoint(route_indices_[j].segment.lane,
                              request_waypoints.Get(i).s()),
          j);
      ++i;
    }
  }
  adc_waypoint_ = hdmap::LaneWaypoint();
  stop_for_destination_ = false;
  return true;
}

/**
 * @brief 向前搜索航点索引
 *
 * @param start 起始搜索位置
 * @param waypoint 待查找的车道航点
 * @return int 找到的索引，未找到返回route_indices_.size()
 *
 * 功能说明：
 * 从start位置向前遍历，找到第一个包含waypoint的路段
 *
 * C++语法说明：
 * - std::max(start, 0)：
 *   确保起始索引非负
 *
 * - static_cast<int>(...)：
 *   显式类型转换，将size_t转为int
 *
 * - !hdmap::RouteSegments::WithinLaneSegment(...)：
 *   静态方法调用
 *   WithinLaneSegment检查航点是否在给定路段内
 */
int LaneFollowMap::SearchForwardWaypointIndex(
    int start, const hdmap::LaneWaypoint &waypoint) const {
  int i = std::max(start, 0);
  while (i < static_cast<int>(route_indices_.size()) &&
         !hdmap::RouteSegments::WithinLaneSegment(route_indices_[i].segment,
                                                  waypoint)) {
    ++i;
  }
  return i;
}

/**
 * @brief 向后搜索航点索引
 *
 * @param start 起始搜索位置
 * @param waypoint 待查找的车道航点
 * @return int 找到的索引，未找到返回-1
 *
 * 功能说明：
 * 从start位置向后遍历，找到第一个包含waypoint的路段
 *
 * C++语法说明：
 * - std::min(static_cast<int>(route_indices_.size() - 1), start)：
 *   确保起始索引不超过最大索引
 *   size() - 1得到最后一个有效索引
 */
int LaneFollowMap::SearchBackwardWaypointIndex(
    int start, const hdmap::LaneWaypoint &waypoint) const {
  int i = std::min(static_cast<int>(route_indices_.size() - 1), start);
  while (i >= 0 && !hdmap::RouteSegments::WithinLaneSegment(
                       route_indices_[i].segment, waypoint)) {
    --i;
  }
  return i;
}

/**
 * @brief 获取下一个航点索引
 *
 * @param index 当前索引
 * @return int 下一个航点索引
 *
 * 功能说明：
 * 边界处理的安全包装函数
 * 确保索引始终在有效范围内
 *
 * C++语法说明：
 * - if (... ? ... : ...)：
 *   三元运算符
 *   条件 ? 值1 : 值2
 *   条件为true时返回值1，否则返回值2
 *
 * - route_indices_.size() - 1：
 *   size()返回size_t，减1后仍是size_t
 *   但这里在比较和返回时会被转换为int
 */
int LaneFollowMap::NextWaypointIndex(int index) const {
  if (index >= static_cast<int>(route_indices_.size() - 1)) {
    return static_cast<int>(route_indices_.size()) - 1;
  } else if (index < 0) {
    return 0;
  } else {
    return index + 1;
  }
}

/**
 * @brief 获取航点索引
 *
 * @param waypoint 目标车道航点
 * @return int 航点在路由中的索引
 *
 * 功能说明：
 * 综合使用前向和后向搜索找到航点的最佳匹配索引
 * 优先返回与当前adc_route_index_相近的索引
 *
 * 算法说明：
 * 1. 前向搜索得到forward_index
 * 2. 如果超出范围，后向搜索
 * 3. 如果forward_index相邻，返回它
 * 4. 否则比较forward和backward，选择更近的
 *
 * C++语法说明：
 * - (backward_index + 1 == adc_route_index_) ? backward_index : forward_index：
 *   三元运算符
 *   如果backward_index的下一个就是当前索引
 *   说明backward更准确
 */
int LaneFollowMap::GetWaypointIndex(const hdmap::LaneWaypoint &waypoint) const {
  int forward_index = SearchForwardWaypointIndex(adc_route_index_, waypoint);
  if (forward_index >= static_cast<int>(route_indices_.size())) {
    return SearchBackwardWaypointIndex(adc_route_index_, waypoint);
  }
  if (forward_index == adc_route_index_ ||
      forward_index == adc_route_index_ + 1) {
    return forward_index;
  }
  auto backward_index = SearchBackwardWaypointIndex(adc_route_index_, waypoint);
  if (backward_index < 0) {
    return forward_index;
  }

  return (backward_index + 1 == adc_route_index_) ? backward_index
                                                  : forward_index;
}

/**
 * @brief 将通道转换为路段列表
 *
 * @param passage 路由通道
 * @param segments 输出参数，转换后的路段列表
 * @return bool 转换成功返回true
 *
 * 功能说明：
 * 将routing::Passage转换为hdmap::RouteSegments
 * 提取通道内所有车道的几何信息
 *
 * C++语法说明：
 * - CHECK_NOTNULL(segments)：
 *   Apollo的检查宏
 *   确保指针不为空
 *   为空时打印错误信息并终止程序
 *
 * - std::max(0.0, lane.start_s())：
 *   确保起始距离非负
 *
 * - std::min(lane_ptr->total_length(), lane.end_s())：
 *   确保结束距离不超过车道总长
 *
 * - segments->emplace_back(...)：
 *   通过智能指针调用emplace_back
 *   直接构造LaneSegment对象
 */
bool LaneFollowMap::PassageToSegments(routing::Passage passage,
                                      hdmap::RouteSegments *segments) const {
  CHECK_NOTNULL(segments);
  segments->clear();
  for (const auto &lane : passage.segment()) {
    auto lane_ptr = hdmap_->GetLaneById(hdmap::MakeMapId(lane.id()));
    if (!lane_ptr) {
      AERROR << "Failed to find lane: " << lane.id();
      return false;
    }
    segments->emplace_back(lane_ptr, std::max(0.0, lane.start_s()),
                           std::min(lane_ptr->total_length(), lane.end_s()));
  }
  return !segments->empty();
}

/**
 * @brief 获取相邻通道列表
 *
 * @param road 道路信息
 * @param start_passage 起始通道索引
 * @return std::vector<int> 相邻通道的索引列表
 *
 * 功能说明：
 * 根据起始通道找到所有可行驶的相邻通道
 * 用于车道变更决策
 *
 * 处理逻辑：
 * 1. 如果是直行通道，直接返回
 * 2. 如果可以驶出（到达目的地），直接返回
 * 3. 否则根据变道方向（LEFT/RIGHT）查找相邻车道
 * 4. 在道路的所有通道中找出包含相邻车道的
 *
 * C++语法说明：
 * - CHECK_GE(start_passage, 0)：
 *   CHECK_GREATER_OR_EQUAL
 *   检查start_passage >= 0
 *
 * - CHECK_LE(start_passage, road.passage_size())：
 *   CHECK_LESS_OR_EQUAL
 *   检查start_passage <= road.passage_size()
 *
 * - std::unordered_set<std::string> neighbor_lanes：
 *   无序集合，O(1)平均查找复杂度
 *   存储相邻车道的ID
 *
 * - source_passage.change_lane_type() == routing::LEFT：
 *   routing::LEFT是枚举值
 *   表示向左变道
 *
 * - segment.lane->lane().left_neighbor_forward_lane_id()：
 *   两次成员访问
 *   第一层获取LaneInfo
 *   第二层lane()获取proto中的Lane信息
 *   left_neighbor_forward_lane_id()返回左邻车道ID列表
 */
std::vector<int> LaneFollowMap::GetNeighborPassages(
    const routing::RoadSegment &road, int start_passage) const {
  CHECK_GE(start_passage, 0);
  CHECK_LE(start_passage, road.passage_size());
  std::vector<int> result;
  const auto &source_passage = road.passage(start_passage);
  result.emplace_back(start_passage);
  if (source_passage.change_lane_type() == routing::FORWARD) {
    return result;
  }
  if (source_passage.can_exit()) {
    return result;
  }
  hdmap::RouteSegments source_segments;
  if (!PassageToSegments(source_passage, &source_segments)) {
    AERROR << "Failed to convert passage to segments";
    return result;
  }
  if (next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
      source_segments.IsWaypointOnSegment(
          routing_waypoint_index_[next_routing_waypoint_index_].waypoint)) {
    ADEBUG << "Need to pass next waypoint[" << next_routing_waypoint_index_
           << "] before change lane";
    return result;
  }
  std::unordered_set<std::string> neighbor_lanes;
  if (source_passage.change_lane_type() == routing::LEFT) {
    for (const auto &segment : source_segments) {
      for (const auto &left_id :
           segment.lane->lane().left_neighbor_forward_lane_id()) {
        neighbor_lanes.insert(left_id.id());
      }
    }
  } else if (source_passage.change_lane_type() == routing::RIGHT) {
    for (const auto &segment : source_segments) {
      for (const auto &right_id :
           segment.lane->lane().right_neighbor_forward_lane_id()) {
        neighbor_lanes.insert(right_id.id());
      }
    }
  }

  for (int i = 0; i < road.passage_size(); ++i) {
    if (i == start_passage) {
      continue;
    }
    const auto &target_passage = road.passage(i);
    for (const auto &segment : target_passage.segment()) {
      if (neighbor_lanes.count(segment.id())) {
        result.emplace_back(i);
        break;
      }
    }
  }
  return result;
}

/**
 * @brief 获取路由段（简化版本）
 *
 * @param vehicle_state 车辆状态
 * @param route_segments 输出参数，路由段列表
 * @return bool 获取成功返回true
 */
bool LaneFollowMap::GetRouteSegments(
    const VehicleState &vehicle_state,
    std::list<hdmap::RouteSegments> *const route_segments) {
  double look_forward_distance =
      LookForwardDistance(vehicle_state.linear_velocity());
  double look_backward_distance = FLAGS_look_backward_distance;
  return GetRouteSegments(vehicle_state, look_backward_distance,
                          look_forward_distance, route_segments);
}

/**
 * @brief 获取路由段（完整版本）
 *
 * @param vehicle_state 车辆状态
 * @param backward_length 后向距离:自车后方需要生成的参考线长度
 * @param forward_length 前向距离:自车前方需要生成的参考线长度
 * @param route_segments 输出参数，路由段列表
 * @return bool 获取成功返回true
 *
 * 功能说明：
 * 根据给定的距离范围获取车辆可行驶的路由段
 * 包括当前通道和相邻通道（如车道变更）
 *
 * 处理流程：
 * 1. 更新车辆状态
 * 2. 获取相邻通道列表
 * 3. 对每个通道：
 *    a. 转换为RouteSegments
 *    b. 获取最近点投影
 *    c. 检查是否可从当前车道驶入
 *    d. 扩展路段到指定距离
 *    e. 设置路段属性（是否在主路段、变道方向等）
 *
 * C++语法说明：
 * - std::list<hdmap::RouteSegments>：
 *   list是双向链表容器
 *   适合频繁的插入删除操作
 *
 * - *const route_segments：
 *   指向常量的指针
 *   指针本身是const（不能改变指向）
 *   但可以通过它修改所指对象
 *
 * - route_segments->emplace_back()：
 *   通过指针调用emplace_back
 *   在list末尾构造新元素
 *
 * - const auto last_waypoint = segments.LastWaypoint()：
 *   const auto自动推导类型
 *   LastWaypoint()返回路段的最后一个航点
 *
 * - ExtendSegments(... &route_segments->back())：
 *   back()返回最后一个元素的引用
 *   将其作为输出参数传入
 */
bool LaneFollowMap::GetRouteSegments(
    const VehicleState &vehicle_state, const double backward_length,
    const double forward_length,
    std::list<hdmap::RouteSegments> *const route_segments) {
  if (!UpdateVehicleState(vehicle_state)) {
    AERROR << "Failed to update vehicle state in pnc_map.";
    return false;
  }
  if (!adc_waypoint_.lane || adc_route_index_ < 0 ||
      adc_route_index_ >= static_cast<int>(route_indices_.size())) {
    AERROR << "Invalid vehicle state in pnc_map, update vehicle state first.";
    return false;
  }
  const auto &route_index = route_indices_[adc_route_index_].index;
  const int road_index = route_index[0];
  const int passage_index = route_index[1];
  const auto &road = last_command_.lane_follow_command().road(road_index);
  auto drive_passages = GetNeighborPassages(road, passage_index);
  for (const int index : drive_passages) {
    const auto &passage = road.passage(index);
    hdmap::RouteSegments segments;
    if (!PassageToSegments(passage, &segments)) {
      ADEBUG << "Failed to convert passage to lane segments.";
      continue;
    }
    /**
     * @brief 计算最近点
     *
     * 场景说明：
     * 如果是当前通道，使用车辆在当前车道的投影点
     * 如果是相邻通道，使用车辆当前位置
     *
     * C++语法说明：
     * - index == passage_index ? A : B：
     *   三元运算符选择
     * - adc_waypoint_.lane->GetSmoothPoint(adc_waypoint_.s)：
     *   获取车道上指定s值的平滑点
     * - PointFactory::ToPointENU(adc_state_)：
     *   静态方法调用
     *   将车辆状态转换为ENU坐标点
     */
    const PointENU nearest_point =
        index == passage_index
            ? adc_waypoint_.lane->GetSmoothPoint(adc_waypoint_.s)
            : PointFactory::ToPointENU(adc_state_);
    common::SLPoint sl;
    hdmap::LaneWaypoint segment_waypoint;
    if (!segments.GetProjection(nearest_point, &sl, &segment_waypoint)) {
      ADEBUG << "Failed to get projection from point: "
             << nearest_point.ShortDebugString();
      continue;
    }
    if (index != passage_index) {
      if (!segments.CanDriveFrom(adc_waypoint_)) {
        ADEBUG << "You cannot drive from current waypoint to passage: "
               << index;
        continue;
      }
    }
    route_segments->emplace_back();
    const auto last_waypoint = segments.LastWaypoint();
    if (!ExtendSegments(segments, sl.s() - backward_length,
                        sl.s() + forward_length, &route_segments->back())) {
      AERROR << "Failed to extend segments with s=" << sl.s()
             << ", backward: " << backward_length
             << ", forward: " << forward_length;
      return false;
    }
    if (route_segments->back().IsWaypointOnSegment(last_waypoint)) {
      route_segments->back().SetRouteEndWaypoint(last_waypoint);
    }
    route_segments->back().SetCanExit(passage.can_exit());
    route_segments->back().SetNextAction(passage.change_lane_type());
    const std::string route_segment_id = absl::StrCat(road_index, "_", index);
    route_segments->back().SetId(route_segment_id);
    route_segments->back().SetStopForDestination(stop_for_destination_);
    if (index == passage_index) {
      route_segments->back().SetIsOnSegment(true);
      route_segments->back().SetPreviousAction(routing::FORWARD);
    } else if (sl.l() > 0) {
      route_segments->back().SetPreviousAction(routing::RIGHT);
    } else {
      route_segments->back().SetPreviousAction(routing::LEFT);
    }
  }
  UpdateRouteSegmentsLaneIds(route_segments);
  return !route_segments->empty();
}

/**
 * @brief 从路由中获取最近点
 *
 * @param state 车辆状态
 * @param waypoint 输出参数，最近的车道航点
 * @return bool 查找成功返回true
 *
 * 功能说明：
 * 根据车辆当前位置找到最近的路由航点
 * 考虑车道范围、投影有效性、航向一致性
 *
 * 算法说明：
 * 1. 在all_lane_ids_中筛选出在range_lane_ids_内的车道
 * 2. 对每个车道计算投影
 * 3. 检查投影是否在车道范围内
 * 4. 检查车道航向与车辆航向的一致性
 * 5. 选择横向距离最近且航向一致的车道
 *
 * C++语法说明：
 * - std::vector<hdmap::LaneInfoConstPtr> valid_lanes：
 *   存储所有有效的车道指针
 *
 * - lane->GetProjection({point.x(), point.y()}, &s, &l)：
 *   将XY坐标投影到车道上
 *   返回沿车道距离s和横向偏移l
 *
 * - std::fabs(...)：
 *   计算浮点数的绝对值
 *
 * - M_PI_2：
 *   π/2的数学常数（约1.5708）
 *   定义在<cmath>中
 *
 * - std::numeric_limits<double>::max()：
 *   获取double类型的最大值
 *   用于初始化距离为极大值
 */
bool LaneFollowMap::GetNearestPointFromRouting(
    const common::VehicleState &state, hdmap::LaneWaypoint *waypoint) const {
  waypoint->lane = nullptr;
  std::vector<hdmap::LaneInfoConstPtr> lanes;
  const auto point = PointFactory::ToPointENU(state);
  std::vector<hdmap::LaneInfoConstPtr> valid_lanes;
  for (auto lane_id : all_lane_ids_) {
    hdmap::Id id = hdmap::MakeMapId(lane_id);
    auto lane = hdmap_->GetLaneById(id);
    if (nullptr != lane) {
      valid_lanes.emplace_back(lane);
    }
  }

  /**
   * @brief 查找有效航点
   *
   * 筛选条件：
   * 1. 车道在range_lane_ids_范围内
   * 2. 如果route_segments_lane_ids_非空，还需在上一帧的route_segments中
   * 3. 投影点在车道长度范围内（加一定容差）
   * 4. 车道航向与车辆航向一致（夹角小于135°）
   */
  std::vector<hdmap::LaneWaypoint> valid_way_points;
  for (const auto &lane : valid_lanes) {
    if (range_lane_ids_.count(lane->id().id()) == 0) {
      ADEBUG << "not in range" << lane->id().id();
      continue;
    }
    if (route_segments_lane_ids_.size() > 0 &&
        route_segments_lane_ids_.count(lane->id().id()) == 0) {
      ADEBUG << "not in last frame route_segments: " << lane->id().id();
      continue;
    }
    double s = 0.0;
    double l = 0.0;
    {
      if (!lane->GetProjection({point.x(), point.y()}, &s, &l)) {
        continue;
      }
      ADEBUG << lane->id().id() << "," << s << "," << l;
      static constexpr double kEpsilon = 0.5;
      if (s > (lane->total_length() + kEpsilon) || (s + kEpsilon) < 0.0) {
        continue;
      }
      double lane_heading = lane->Heading(s);
      if (std::fabs(common::math::AngleDiff(lane_heading, state.heading())) >
          M_PI_2 * 1.5) {
        continue;
      }
    }

    valid_way_points.emplace_back();
    auto &last = valid_way_points.back();
    last.lane = lane;
    last.s = s;
    last.l = l;
    ADEBUG << "distance:" << std::fabs(l);
  }
  if (valid_way_points.empty()) {
    AERROR << "Failed to find nearest point: " << point.ShortDebugString();
    return false;
  }

  /**
   * @brief 选择最近的航点
   *
   * 筛选条件：
   * 1. 航向一致（夹角小于135°）
   * 2. 横向距离最近
   */
  int closest_index = -1;
  double distance = std::numeric_limits<double>::max();
  double lane_heading = 0.0;
  double vehicle_heading = state.heading();
  for (size_t i = 0; i < valid_way_points.size(); i++) {
    lane_heading = valid_way_points[i].lane->Heading(valid_way_points[i].s);
    if (std::abs(common::math::AngleDiff(lane_heading, vehicle_heading)) >
        M_PI_2 * 1.5) {
      continue;
    }
    if (std::fabs(valid_way_points[i].l) < distance) {
      distance = std::fabs(valid_way_points[i].l);
      closest_index = i;
    }
  }
  if (closest_index == -1) {
    AERROR << "Can not find nearest waypoint. vehicle heading:"
           << vehicle_heading << "lane heading:" << lane_heading;
    return false;
  }
  waypoint->lane = valid_way_points[closest_index].lane;
  waypoint->s = valid_way_points[closest_index].s;
  waypoint->l = valid_way_points[closest_index].l;
  return true;
}

/**
 * @brief 获取路由后继车道
 *
 * @param lane 当前车道
 * @return hdmap::LaneInfoConstPtr 后继车道指针
 *
 * 功能说明：
 * 获取指定车道的路由后继车道
 * 在range_lane_ids_范围内优先选择
 *
 * C++语法说明：
 * - lane->lane().successor_id()：
 *   第一个lane()获取LaneInfo
 *   第二个lane()获取proto消息
 *   successor_id()返回后继车道ID列表
 *
 * - range_lane_ids_.count(lane_id.id()) != 0：
 *   检查后继车道是否在当前路由范围内
 */
hdmap::LaneInfoConstPtr LaneFollowMap::GetRouteSuccessor(
    hdmap::LaneInfoConstPtr lane) const {
  if (lane->lane().successor_id().empty()) {
    return nullptr;
  }
  hdmap::Id preferred_id = lane->lane().successor_id(0);
  for (const auto &lane_id : lane->lane().successor_id()) {
    if (range_lane_ids_.count(lane_id.id()) != 0) {
      preferred_id = lane_id;
      break;
    }
  }
  return hdmap_->GetLaneById(preferred_id);
}

/**
 * @brief 获取路由前驱车道
 *
 * @param lane 当前车道
 * @return hdmap::LaneInfoConstPtr 前驱车道指针
 *
 * 功能说明：
 * 获取指定车道的路由前驱车道
 * 在route_indices_中查找属于路由一部分的前驱
 *
 * C++语法说明：
 * - std::unordered_set<std::string> predecessor_ids：
 *   使用无序集合存储所有前驱ID
 *   便于快速查找
 */
hdmap::LaneInfoConstPtr LaneFollowMap::GetRoutePredecessor(
    hdmap::LaneInfoConstPtr lane) const {
  if (lane->lane().predecessor_id().empty()) {
    return nullptr;
  }

  std::unordered_set<std::string> predecessor_ids;
  for (const auto &lane_id : lane->lane().predecessor_id()) {
    predecessor_ids.insert(lane_id.id());
  }

  hdmap::Id preferred_id = lane->lane().predecessor_id(0);
  for (const auto &route_index : route_indices_) {
    auto &lane = route_index.segment.lane->id();
    if (predecessor_ids.count(lane.id()) != 0) {
      preferred_id = lane;
      break;
    }
  }
  return hdmap_->GetLaneById(preferred_id);
}

/**
 * @brief 扩展路段（使用坐标点版本）
 *
 * @param segments 原始路段
 * @param point 坐标点
 * @param look_backward 后向扩展距离
 * @param look_forward 前向扩展距离
 * @param extended_segments 输出参数，扩展后的路段
 * @return bool 扩展成功返回true
 *
 * 功能说明：
 * 将点投影到路段上，然后调用重载版本进行扩展
 *
 * C++语法说明：
 * - const common::PointENU &point：
 *   坐标点常量引用
 */
bool LaneFollowMap::ExtendSegments(const hdmap::RouteSegments &segments,
                                   const common::PointENU &point,
                                   double look_backward, double look_forward,
                                   hdmap::RouteSegments *extended_segments) {
  common::SLPoint sl;
  hdmap::LaneWaypoint waypoint;
  if (!segments.GetProjection(point, &sl, &waypoint)) {
    AERROR << "point: " << point.ShortDebugString() << " is not on segment";
    return false;
  }
  return ExtendSegments(segments, sl.s() - look_backward, sl.s() + look_forward,
                        extended_segments);
}

/**
 * @brief 扩展路段（使用距离范围版本）
 *
 * @param segments 原始路段
 * @param start_s 起始距离
 * @param end_s 结束距离
 * @param truncated_segments 输出参数，扩展后的路段
 * @return bool 扩展成功返回true
 *
 * 功能说明：
 * 根据指定的起止距离扩展路段
 * 包括：
 * 1. 向后扩展到start_s
 * 2. 截取中间部分
 * 3. 向前扩展到end_s
 * 4. 处理环路检测
 *
 * C++语法说明：
 * - truncated_segments->SetProperties(segments)：
 *   将原始路段的属性复制到结果路段
 *
 * - static constexpr double kRouteEpsilon = 1e-3：
 *   静态常量表达式
 *   非常小的值用于浮点数比较
 *
 * - truncated_segments->insert(...rbegin(), ...rend())：
 *   insert插入位置 + 逆向迭代器范围
 *   rbegin()指向末尾，rend()指向开头
 *   将元素反向插入开头
 *
 * - found_loop = true：
 *   检测到环路时跳出循环
 */
bool LaneFollowMap::ExtendSegments(
    const hdmap::RouteSegments &segments, double start_s, double end_s,
    hdmap::RouteSegments *const truncated_segments) const {
  if (segments.empty()) {
    AERROR << "The input segments is empty";
    return false;
  }
  CHECK_NOTNULL(truncated_segments);
  truncated_segments->SetProperties(segments);

  if (start_s >= end_s) {
    AERROR << "start_s(" << start_s << " >= end_s(" << end_s << ")";
    return false;
  }
  std::unordered_set<std::string> unique_lanes;
  static constexpr double kRouteEpsilon = 1e-3;

  /**
   * @brief 向后扩展
   *
   * 算法说明：
   * 1. 如果start_s < 0，需要向后扩展
   * 2. 从第一个路段开始，使用前驱车道扩展
   * 3. 直到extend_s耗尽或无法继续扩展
   */
  if (start_s < 0) {
    const auto &first_segment = *segments.begin();
    auto lane = first_segment.lane;
    double s = first_segment.start_s;
    double extend_s = -start_s;
    std::vector<hdmap::LaneSegment> extended_lane_segments;
    while (extend_s > kRouteEpsilon) {
      if (s <= kRouteEpsilon) {
        lane = GetRoutePredecessor(lane);
        if (lane == nullptr ||
            unique_lanes.find(lane->id().id()) != unique_lanes.end()) {
          break;
        }
        s = lane->total_length();
      } else {
        const double length = std::min(s, extend_s);
        extended_lane_segments.emplace_back(lane, s - length, s);
        extend_s -= length;
        s -= length;
        unique_lanes.insert(lane->id().id());
      }
    }
    truncated_segments->insert(truncated_segments->begin(),
                               extended_lane_segments.rbegin(),
                               extended_lane_segments.rend());
  }

  /**
   * @brief 中间截取
   *
   * 算法说明：
   * 遍历每个路段，计算在[start_s, end_s]范围内的部分
   * 处理连续相同车道的情况（合并）
   * 检测环路（车道重复出现）
   */
  bool found_loop = false;
  double router_s = 0;
  for (const auto &lane_segment : segments) {
    const double adjusted_start_s = std::max(
        start_s - router_s + lane_segment.start_s, lane_segment.start_s);
    const double adjusted_end_s =
        std::min(end_s - router_s + lane_segment.start_s, lane_segment.end_s);
    if (adjusted_start_s < adjusted_end_s) {
      if (!truncated_segments->empty() &&
          truncated_segments->back().lane->id().id() ==
              lane_segment.lane->id().id()) {
        truncated_segments->back().end_s = adjusted_end_s;
      } else if (unique_lanes.find(lane_segment.lane->id().id()) ==
                 unique_lanes.end()) {
        truncated_segments->emplace_back(lane_segment.lane, adjusted_start_s,
                                         adjusted_end_s);
        unique_lanes.insert(lane_segment.lane->id().id());
      } else {
        found_loop = true;
        break;
      }
    }
    router_s += (lane_segment.end_s - lane_segment.start_s);
    if (router_s > end_s) {
      break;
    }
  }
  if (found_loop) {
    return true;
  }

  /**
   * @brief 目的地处理
   *
   * 条件说明：
   * 1. 如果到达目的地车道
   * 2. 且当前不在目的地车道或虽在但未到达终点
   * 需要额外扩展确保能到达目的地
   */
  double last_lane_segment_length =
      segments.back().end_s - segments.back().start_s;
  auto last_lane = segments.back().lane;
  bool last_lane_can_exit =
      last_lane->id().id() == dest_lane_segment_.id() &&
      (adc_waypoint_.lane->id().id() != dest_lane_segment_.id() ||
       (adc_waypoint_.lane->id().id() == dest_lane_segment_.id() &&
        adc_waypoint_.s < dest_lane_segment_.end_s() + 1.0));
  if (last_lane_can_exit) {
    end_s =
        std::max(router_s - kRouteEpsilon,
                 router_s - last_lane_segment_length +
                     dest_lane_segment_.end_s() - dest_lane_segment_.start_s() +
                     FLAGS_reference_line_endpoint_extend_length);
  }

  /**
   * @brief 向前扩展
   *
   * 算法说明：
   * 1. 如果router_s < end_s，继续向前扩展
   * 2. 首先扩展当前最后一个车道
   * 3. 然后通过后继车道继续扩展
   * 4. 直到router_s >= end_s或无法继续
   */
  if (router_s < end_s && !truncated_segments->empty()) {
    auto &back = truncated_segments->back();
    if (back.lane->total_length() > back.end_s) {
      double origin_end_s = back.end_s;
      back.end_s =
          std::min(back.end_s + end_s - router_s, back.lane->total_length());
      router_s += back.end_s - origin_end_s;
    }
  }

  while (router_s < end_s - kRouteEpsilon) {
    last_lane = GetRouteSuccessor(last_lane);
    if (last_lane == nullptr ||
        unique_lanes.find(last_lane->id().id()) != unique_lanes.end()) {
      break;
    }

    bool last_lane_can_exit =
        last_lane->id().id() == dest_lane_segment_.id() &&
        (adc_waypoint_.lane->id().id() != dest_lane_segment_.id() ||
         (adc_waypoint_.lane->id().id() == dest_lane_segment_.id() &&
          adc_waypoint_.s < dest_lane_segment_.end_s() + 1.0));

    if (last_lane_can_exit) {
      end_s = std::max(router_s + 1.0,
                       router_s + dest_lane_segment_.end_s() -
                           dest_lane_segment_.start_s() +
                           FLAGS_reference_line_endpoint_extend_length);
    }

    const double length = std::min(end_s - router_s, last_lane->total_length());
    truncated_segments->emplace_back(last_lane, 0, length);
    unique_lanes.insert(last_lane->id().id());
    router_s += length;
    AINFO << last_lane->id().id() << ", add length: " << length << ", router_s: " << router_s << ", end_s: " << end_s;
  }
  return true;
}

/**
 * @brief 将车道追加到点列表
 *
 * @param lane 车道指针
 * @param start_s 起始距离
 * @param end_s 结束距离
 * @param points 输出参数，地图路径点列表
 *
 * 功能说明：
 * 将指定车道的[start_s, end_s]区间内的点追加到points列表
 * 用于生成路径的几何信息
 *
 * C++语法说明：
 * - lane->points().size()：
 *   points()返回车道的中心线点列表
 *   size()获取点的数量
 *
 * - lane->headings()[i]：
 *   headings()返回每个点的航向角列表
 *
 * - segment.start() + segment.unit_direction() * (...):
 *   向量加法
 *   start()获取线段起点
 *   unit_direction()获取单位方向向量
 *   乘以标量得到沿方向的位移
 */
void LaneFollowMap::AppendLaneToPoints(
    hdmap::LaneInfoConstPtr lane, const double start_s, const double end_s,
    std::vector<hdmap::MapPathPoint> *const points) {
  if (points == nullptr || start_s >= end_s) {
    return;
  }
  double accumulate_s = 0.0;
  for (size_t i = 0; i < lane->points().size(); ++i) {
    if (accumulate_s >= start_s && accumulate_s <= end_s) {
      points->emplace_back(lane->points()[i], lane->headings()[i],
                           hdmap::LaneWaypoint(lane, accumulate_s));
    }
    if (i < lane->segments().size()) {
      const auto &segment = lane->segments()[i];
      const double next_accumulate_s = accumulate_s + segment.length();
      if (start_s > accumulate_s && start_s < next_accumulate_s) {
        points->emplace_back(segment.start() + segment.unit_direction() *
                                                   (start_s - accumulate_s),
                             lane->headings()[i],
                             hdmap::LaneWaypoint(lane, start_s));
      }
      if (end_s > accumulate_s && end_s < next_accumulate_s) {
        points->emplace_back(
            segment.start() + segment.unit_direction() * (end_s - accumulate_s),
            lane->headings()[i], hdmap::LaneWaypoint(lane, end_s));
      }
      accumulate_s = next_accumulate_s;
    }
    if (accumulate_s > end_s) {
      break;
    }
  }
}

/**
 * @brief 更新路由段车道ID集合
 *
 * @param route_segments 路由段列表
 *
 * 功能说明：
 * 遍历所有路由段，收集所有车道ID
 * 存储到route_segments_lane_ids_集合中
 * 用于快速判断某车道是否在当前路由中
 *
 * C++语法说明：
 * - for (auto &route_seg : *route_segments)：
 *   范围for循环遍历list
 *   *route_segments解引用获取list本身
 *
 * - route_segments_lane_ids_.insert(lane_seg.lane->id().id())：
 *   将车道ID字符串插入集合
 */
void LaneFollowMap::UpdateRouteSegmentsLaneIds(
    const std::list<hdmap::RouteSegments> *route_segments) {
  route_segments_lane_ids_.clear();
  for (auto &route_seg : *route_segments) {
    for (auto &lane_seg : route_seg) {
      if (nullptr == lane_seg.lane) {
        continue;
      }
      route_segments_lane_ids_.insert(lane_seg.lane->id().id());
    }
  }
}

/**
 * @brief 获取自车航点
 *
 * @return apollo::hdmap::LaneWaypoint 自车当前位置对应的车道航点
 *
 * 功能说明：
 * 返回最近一次UpdateVehicleState更新的自车航点
 *
 * C++语法说明：
 * - return adc_waypoint_：
 *   直接返回成员变量
 *   返回类型是hdmap::LaneWaypoint
 */
apollo::hdmap::LaneWaypoint LaneFollowMap::GetAdcWaypoint() const {
  return adc_waypoint_;
}

/**
 * @brief 获取到目的地距离
 *
 * @return double 到目的地的距离（米）
 *
 * 功能说明：
 * 计算自车到目的地的沿路由距离
 * 考虑车道变更和通道切换
 *
 * 算法说明：
 * 1. 从自车所在位置开始
 * 2. 遍历后续道路和通道
 * 3. 累加每个车道的距离
 * 4. 只累加can_exit的通道（可到达目的地的通道）
 *
 * C++语法说明：
 * - for (...; road_index < routing.road_size(); ++road_index)：
 *   嵌套for循环遍历道路和通道
 *
 * - if (!passage.can_exit()) continue：
 *   continue跳过后续代码，进入下一次循环
 *
 * - dis_to_destination += ...：
 *   累加距离
 */
double LaneFollowMap::GetDistanceToDestination() const {
  if (adc_route_index_ < 0 || adc_route_index_ >= route_indices_.size()) {
    AERROR << "adc_route_index error, can not get distance to destination, "
              "return 0.";
    return 0.0;
  }
  const auto &routing = last_command_.lane_follow_command();
  int adc_road_index = route_indices_[adc_route_index_].index[0];
  int adc_passage_index = route_indices_[adc_route_index_].index[1];
  int adc_lane_index = route_indices_[adc_route_index_].index[2];

  bool get_adc_exit_waypoint =
      routing.road(adc_road_index).passage(adc_passage_index).can_exit();
  int start_passage_index = adc_passage_index;
  int start_lane_index = adc_lane_index;
  double start_lane_s = adc_waypoint_.s;

  double dis_to_destination = 0.0;

  for (int road_index = adc_road_index; road_index < routing.road_size();
       ++road_index) {
    const auto &road_segment = routing.road(road_index);
    for (int passage_index = 0; passage_index < road_segment.passage_size();
         ++passage_index) {
      const auto &passage = road_segment.passage(passage_index);
      if (!passage.can_exit()) {
        continue;
      }
      /**
       * @brief 查找自车所在的可驶出通道
       *
       * 条件说明：
       * 1. 首次找到can_exit的通道
       * 2. 在该通道内查找自车实际位置
       */
      if (!get_adc_exit_waypoint) {
        get_adc_exit_waypoint = true;
        for (int index = 0; index < passage.segment_size(); ++index) {
          auto lane = hdmap_->GetLaneById(
              hdmap::MakeMapId(passage.segment(index).id()));
          double s = 0.0;
          double l = 0.0;
          if (!lane->GetProjection({adc_state_.x(), adc_state_.y()}, &s, &l)) {
            continue;
          }
          static constexpr double kEpsilon = 0.5;
          if (s > (lane->total_length() + kEpsilon) || (s + kEpsilon) < 0.0) {
            continue;
          }
          start_passage_index = passage_index;
          start_lane_index = index;
          start_lane_s = s;
        }
      }

      /**
       * @brief 累加到目的地距离
       *
       * 分为三种情况：
       * 1. 当前道路且当前通道：从当前位置开始累加
       * 2. 当前道路但不同通道：累加整个通道
       * 3. 其他道路：累加整个通道
       */
      if (get_adc_exit_waypoint) {
        if (road_index == adc_road_index &&
            passage_index == start_passage_index) {
          for (int lane_index = start_lane_index;
               lane_index < passage.segment_size(); ++lane_index) {
            if (lane_index == start_lane_index) {
              dis_to_destination +=
                  passage.segment(lane_index).end_s() - start_lane_s;
            } else {
              dis_to_destination += passage.segment(lane_index).end_s() -
                                    passage.segment(lane_index).start_s();
            }
          }
        } else {
          for (int lane_index = 0; lane_index < passage.segment_size();
               ++lane_index) {
            dis_to_destination += passage.segment(lane_index).end_s() -
                                  passage.segment(lane_index).start_s();
          }
        }
        break;
      }
    }
  }
  return dis_to_destination;
}

}  // namespace planning
}  // namespace apollo