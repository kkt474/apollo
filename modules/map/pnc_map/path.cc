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
 * @file path.cc
 * @brief 路径(Path)数据结构和算法实现文件
 *
 * 功能说明：
 * 实现Apollo地图模块中与路径(Path)相关的数据结构和算法
 * Path是PNC(Planning and Control)系统中重要的地图数据结构
 *
 * 主要功能：
 * 1. 路径点管理（MapPathPoint）- 存储路径上的点及属性
 * 2. 车道航路点（LaneWaypoint）- 关联到车道的路径点
 * 3. 车道段（LaneSegment）- 车道上的一段
 * 4. 路径初始化和插值 - 构建路径并支持点间插值
 * 5. 路径投影（XY↔SL坐标转换）- 地图坐标与路径坐标互转
 * 6. 路径近似算法 - 用较少点近似原始路径加速计算
 * 7. 重叠区域检测 - 检测与其他地图对象的重叠
 *
 * 设计特点：
 * - 累积s坐标系统：预计算所有点的s坐标，加速查询
 * - 采样点系统：按固定距离采样用于快速查找
 * - 路径近似：用较少点近似原始路径加速投影计算
 * - 多车道支持：LaneSegment支持车道拼接
 *
 * C++语法说明：
 * - std::vector<T>: 动态数组容器，支持随机访问
 * - std::bind: 绑定函数和参数，创建函数对象
 * - std::placeholders::_1: 占位符，用于bind指定参数位置
 * - std::numeric_limits<T>::infinity(): 类型极限值，表示无穷大
 * - lambda表达式: 匿名函数对象，[]捕获列表，(参数)->返回类型{函数体}
 * - std::move(): 移动语义，将左值转为右值引用
 * - initializer_list: 初始化列表，{value1, value2, ...}
 * - auto: 自动类型推导，根据初始化值推断变量类型
 * - const & : 常量引用，避免拷贝且保证不被修改
 * - static_cast<T>: 显式类型转换，编译时类型检查
 * - std::unordered_map: 基于哈希表的键值对容器
 * - 模板类: generic programming，类型作为参数
 * - 命名空间: namespace，避免命名冲突
 * - 引用折叠规则: T& & => T&, T&& & => T&, 等等
 */
#include "modules/map/pnc_map/path.h"

/**
 * @brief C++标准库头文件
 *
 * #include <algorithm>:
 *   - std::sort: 排序算法
 *   - std::lower_bound: 二分查找第一个不小于给定值的元素
 *   - std::upper_bound: 二分查找第一个大于给定值的元素
 *   - std::min/std::max: 最小值/最大值
 *
 * #include <limits>:
 *   - std::numeric_limits<T>::infinity(): 浮点类型的正无穷
 *   - std::numeric_limits<T>::epsilon(): 机器epsilon
 *
 * #include <unordered_map>:
 *   - std::unordered_map<Key, Value>: 基于哈希表的关联容器
 *   - 平均查找时间复杂度O(1)
 */
#include <algorithm>        /**< C++标准算法库：std::sort, std::lower_bound等 */
#include <limits>          /**< 类型极限值：std::numeric_limits */
#include <unordered_map>    /**< 无序映射：基于哈希表 */

/**
 * @brief Abseil库头文件
 *
 * absl::StrCat(): 高效字符串拼接，比+运算符更快
 * absl::StrJoin(): 将容器中的元素用分隔符连接成字符串
 */
#include "absl/strings/str_cat.h"     /**< Abseil字符串拼接 */
#include "absl/strings/str_join.h"    /**< Abseil字符串连接 */

/**
 * @brief Cyber RT框架头文件
 *
 * cyber::common::log.h: Cyber RT的日志系统
 * AINFO/AWARN/AERROR: 不同级别的日志宏
 * CHECK_NOTNULL: 空指针检查断言
 */
#include "cyber/common/log.h"    /**< Cyber日志宏 */

/**
 * @brief Apollo数学库头文件
 *
 * line_segment2d.h: 2D线段类，计算点到线段的距离、投影等
 * math_utils.h: 数学工具函数，如NormalizeAngle, Sqr等
 * polygon2d.h: 2D多边形类，Polygon2d
 */
#include "modules/common/math/line_segment2d.h" /**< 2D线段 */
#include "modules/common/math/math_utils.h"    /**< 数学工具 */
#include "modules/common/math/polygon2d.h"     /**< 2D多边形 */

/**
 * @brief Apollo通用工具头文件
 *
 * string_util.h: 字符串工具，如DebugStringFormatter
 */
#include "modules/common/util/string_util.h"   /**< 字符串工具 */

namespace apollo {
/**
 * @namespace apollo::
 * @brief Apollo最外层命名空间
 *
 * namespace是C++组织代码的机制：
 * - 避免全局命名冲突
 * - 逻辑分组相关功能
 * - 可嵌套，如apollo::hdmap
 */
namespace hdmap {

/**
 * @brief using类型别名声明
 *
 * using是C++11引入的类型别名声明方式，比typedef更直观
 * 语法: using 别名 = 原始类型;
 *
 * using vs typedef:
 * - typedef int (*FP)(int, int);  // 函数指针，语法不直观
 * - using FP = int (*)(int, int);  // 语法更清晰
 * - using IntVector = std::vector<int>;  // 模板类型更直观
 */
using apollo::common::math::Box2d;           /**< 2D包围盒类型 */
using apollo::common::math::kMathEpsilon;     /**< 数学极小值常量，用于浮点比较 */
using apollo::common::math::LineSegment2d;     /**< 2D线段类型 */
using apollo::common::math::Sqr;               /**< 平方函数 Sqr(x) => x*x */
using apollo::common::math::Vec2d;             /**< 2D向量类型 */
using apollo::common::util::DebugStringFormatter;  /**< 调试字符串格式化 */

/**
 * @brief std::bind占位符
 *
 * std::placeholders::_1是std::bind的占位符
 * _1表示绑定后函数的第一个参数
 * _2表示第二个参数，以此类推
 *
 * 示例:
 * auto f = std::bind(func, _1, 5);
 * f(x) 等价于 func(x, 5);
 *
 * 在文件中的用途:
 * std::bind(&LaneInfo::cross_lanes, _1)
 * 创建了一个函数对象，当调用时传递一个LaneInfo引用
 * 并调用其cross_lanes()方法
 */
using std::placeholders::_1;  /**< std::bind的第一个占位符 */

/**
 * @namespace 匿名命名空间
 *
 * 匿名命名空间内的符号仅当前文件可见
 * 相当于static关键字的作用，但更现代
 * 用于文件内部使用的辅助函数和常量
 */
namespace {

/**
 * @brief 采样距离常量
 *
 * const double: 常量类型，值不可修改
 * kSampleDistance: 命名约定，常量以k开头
 * 0.25米: 用于路径插值和快速查找的固定采样间隔
 *
 * 为什么用0.25米?
 * - 足够细粒度以保证精度
 * - 不会太细导致数据量过大
 * - 经验值，适用于大多数场景
 */
const double kSampleDistance = 0.25;  /**< 0.25米采样间隔 */

/**
 * @brief 查找两个路径点之间的车道段
 *
 * 遍历两个路径点的车道航路点列表，找到属于同一车道的段
 * 这是路径与车道关联的核心函数
 *
 * @param p1 起始路径点(MapPathPoint类型)
 * @param p2 终止路径点
 * @param lane_segment 输出参数：找到的车道段指针
 * @return bool 是否找到有效车道段
 *
 * C++语法说明:
 * - for (const auto& wp1 : p1.lane_waypoints()):
 *   range-based for循环(C++11)
 *   const auto&: 常量引用，避免拷贝
 *   wp1是lane_waypoints()返回容器中的元素引用
 *
 * - nullptr == wp1.lane:
 *   nullptr是C++11的空指针字面量
 *   优先使用nullptr而非NULL或0
 *
 * - wp1.lane->id().id():
 *   ->是成员指针解引用
 *   lane->id()返回LaneInfo的ID
 *   .id()返回ID的字符串值
 *
 * - LaneSegment(wp1.lane, wp1.s, wp2.s):
 *   构造函数的参数传递方式
 *   按值创建LaneSegment对象
 */
bool FindLaneSegment(const MapPathPoint& p1, const MapPathPoint& p2,
                    LaneSegment* const lane_segment) {
  /**
   * 外层循环：遍历第一个路径点的所有车道航路点
   * p1.lane_waypoints()返回std::vector<LaneWaypoint>
   */
  for (const auto& wp1 : p1.lane_waypoints()) {
    /**
     * 空指针检查
     * nullptr表示空指针，比NULL更类型安全
     */
    if (nullptr == wp1.lane) {  /**< 空指针检查，跳过无效航路点 */
      continue;  /**< 跳过当前迭代，继续下一次 */
    }
    /**
     * 内层循环：遍历第二个路径点的所有车道航路点
     * 寻找与wp1同一车道且s值递增的航路点
     */
    for (const auto& wp2 : p2.lane_waypoints()) {
      if (nullptr == wp2.lane) {  /**< 空指针检查 */
        continue;
      }
      /**
       * 检查是否同一车道且s值递增
       * wp1.lane->id().id(): 获取第一个航路点所在车道的ID字符串
       * wp2.lane->id().id(): 获取第二个航路点所在车道的ID字符串
       * wp1.s < wp2.s: 确保s值递增(路径方向)
       */
      if (wp1.lane->id().id() == wp2.lane->id().id() && wp1.s < wp2.s) {
        /**
         * 找到匹配，创建车道段
         * LaneSegment构造函数: (lane_ptr, start_s, end_s)
         * *lane_segment = ...: 解引用指针并赋值
         */
        *lane_segment = LaneSegment(wp1.lane, wp1.s, wp2.s);  /**< 构造车道段 */
        return true;  /**< 找到有效车道段，返回true */
      }
    }
  }
  return false;  /**< 未找到匹配的车道段 */
}

}  // namespace

/**
 * @brief LaneWaypoint调试字符串
 *
 * @return std::string 格式化字符串
 *
 * C++语法说明:
 * - std::string: 标准库字符串类型
 * - absl::StrCat(): Abseil库的高效字符串拼接
 *   内部使用stringstream或类似机制，比+更高效
 * - lane->id().id(): 获取车道ID
 *   lane->id()返回const Id&
 *   .id()返回const std::string&
 */
std::string LaneWaypoint::DebugString() const {
  /**
   * nullptr检查
   * 如果lane为空指针，返回"(lane is null)"
   */
  if (lane == nullptr) {  /**< 空指针检查 */
    return "(lane is null)";
  }
  /**
   * StrCat自动将参数转换为字符串并拼接
   * 参数类型: const char*, int, double等
   */
  return absl::StrCat("id = ", lane->id().id(), "  s = ", s);
}

/**
 * @brief 获取左侧车道边界类型
 *
 * 遍历车道的左侧边界类型列表，找到给定s处对应的边界类型
 *
 * @param waypoint 车道航路点
 * @return LaneBoundaryType::Type 边界类型枚举值
 *
 * C++语法说明:
 * - waypoint.lane->lane().left_boundary():
 *   第一个lane()是LaneWaypoint的成员(LaneInfoConstPtr)
 *   第二个lane()是LaneInfo的方法，返回const Lane&引用
 *   left_boundary()返回LaneBoundary类型
 *
 * - boundary_type(): 返回repeated字段的迭代器
 *   repeated是Protobuf的动态数组类型
 *   支持类似vector的操作
 *
 * - types_size(): 返回repeated字段的元素个数
 * - types(i): 返回第i个元素
 */
LaneBoundaryType::Type LeftBoundaryType(const LaneWaypoint& waypoint) {
  /**
   * 空指针检查
   * !waypoint.lane 等价于 waypoint.lane == nullptr
   */
  if (!waypoint.lane) {  /**< 空指针检查 */
    return LaneBoundaryType::UNKNOWN;
  }
  /**
   * 遍历左侧边界类型列表
   * auto推导类型: const LaneBoundaryType&
   */
  for (const auto& type :
       waypoint.lane->lane().left_boundary().boundary_type()) {
    /**
     * 找到第一个s值大于等于waypoint.s的边界段
     * 这是因为boundary_type()按s值排序
     */
    if (type.s() <= waypoint.s) {
      /**
       * 检查是否有有效类型
       * types_size() > 0表示有类型定义
       * types(0)返回第一个类型
       */
      if (type.types_size() > 0) {  /**< 有有效类型 */
        return type.types(0);      /**< 返回第一个类型 */
      } else {
        return LaneBoundaryType::UNKNOWN;
      }
    }
  }
  return LaneBoundaryType::UNKNOWN;
}

/**
 * @brief 获取右侧车道边界类型
 *
 * 与LeftBoundaryType类似，遍历右侧边界
 * 代码结构完全相同，只是访问right_boundary()
 */
LaneBoundaryType::Type RightBoundaryType(const LaneWaypoint& waypoint) {
  if (!waypoint.lane) {
    return LaneBoundaryType::UNKNOWN;
  }
  for (const auto& type :
       waypoint.lane->lane().right_boundary().boundary_type()) {
    if (type.s() <= waypoint.s) {
      if (type.types_size() > 0) {
        return type.types(0);
      } else {
        return LaneBoundaryType::UNKNOWN;
      }
    }
  }
  return LaneBoundaryType::UNKNOWN;
}

/**
 * @brief 获取左侧相邻车道航路点
 *
 * 在当前车道的左侧相邻前向车道中查找最近点
 *
 * @param waypoint 当前车道航路点
 * @return LaneWaypoint 左侧相邻车道的航路点(如果存在)
 *
 * C++语法说明:
 * - auto point = waypoint.lane->GetSmoothPoint(waypoint.s):
 *   auto自动类型推导
 *   GetSmoothPoint返回MapPathPoint类型
 *
 * - HDMapUtil::BaseMapPtr():
 *   静态成员函数调用
 *   BaseMapPtr()返回HDMap单例的智能指针
 *
 * - CHECK_NOTNULL(map_ptr):
 *   Apollo的断言宏
 *   如果指针为nullptr，程序会报错并终止
 *   仅在debug模式生效
 *
 * - map_ptr->GetLaneById(lane_id):
 *   ->调用智能指针的get()返回的原始指针的方法
 *   或者智能指针直接支持->操作符
 *
 * - lane->GetProjection({point.x(), point.y()}, &s, &l):
 *   {point.x(), point.y()}是braced-init-list
 *   构造Vec2d临时对象
 *   &s, &l是输出参数，指针传递
 */
LaneWaypoint LeftNeighborWaypoint(const LaneWaypoint& waypoint) {
  LaneWaypoint neighbor;  /**< 默认构造空的航路点 */
  if (!waypoint.lane) {
    return neighbor;  /**< 返回默认构造的空LaneWaypoint */
  }
  /**
   * 获取当前车道上对应s坐标的平滑点
   * GetSmoothPoint返回路径上的点，包括x,y,heading
   */
  auto point = waypoint.lane->GetSmoothPoint(waypoint.s);
  auto map_ptr = HDMapUtil::BaseMapPtr();  /**< 获取地图单例 */
  CHECK_NOTNULL(map_ptr);  /**< 断言检查，非空才继续 */

  /**
   * 遍历左侧相邻前向车道列表
   * left_neighbor_forward_lane_id()返回repeated Id字段
   */
  for (const auto& lane_id :
       waypoint.lane->lane().left_neighbor_forward_lane_id()) {
    /**
     * 根据ID获取车道指针
     * GetLaneById返回LaneInfoConstPtr
     */
    auto lane = map_ptr->GetLaneById(lane_id);  /**< 根据ID获取车道 */
    if (!lane) {
      return neighbor;  /**< 无效车道，返回空 */
    }
    double s = 0.0;  /**< 投影后的累积距离 */
    double l = 0.0;  /**< 投影后的横向偏移 */
    /**
     * 计算点在线上的投影
     * {point.x(), point.y()}构造Vec2d
     * &s, &l是输出参数
     */
    if (!lane->GetProjection({point.x(), point.y()}, &s, &l)) {
      continue;  /**< 投影失败，尝试下一车道 */
    }

    /**
     * 检查s坐标是否在车道长度范围内(带容差)
     * kSampleDistance作为容差
     * -kSampleDistance允许少量超出起点
     * lane->total_length() + kSampleDistance允许少量超出终点
     */
    if (s < -kSampleDistance || s > lane->total_length() + kSampleDistance) {
      continue;  /**< 超出范围，尝试下一车道 */
    } else {
      /**
       * 找到有效航路点
       * LaneWaypoint(lane, s)构造新的航路点
       * 横向偏移l从投影结果获取
       */
      return LaneWaypoint(lane, s);  /**< 找到有效航路点 */
    }
  }
  return neighbor;  /**< 未找到，返回空 */
}

/**
 * @brief 拼接相邻的同类车道段
 *
 * 将属于同一车道的连续段合并，减少车道段数量
 * 优化车道段的表示，减少数据量
 *
 * @param segments 输入/输出：车道段向量指针
 *
 * 核心逻辑：
 * 1. 遍历找到属于同一车道的连续段
 * 2. 合并起止s坐标
 * 3. 处理边界(接近0或total_length时修正)
 *
 * C++语法说明:
 * - static constexpr double:
 *   static: 类级别常量，所有对象共享
 *   constexpr: 编译时常量，编译时求值
 *   比const更严格的常量声明
 *
 * - std::size_t:
 *   无符号整数类型，用于表示大小和索引
 *   保证足够大以表示任何容器大小
 *
 * - segments->at(i):
 *   向量的at方法，带边界检查
 *   越界时抛出std::out_of_range异常
 *   比operator[]更安全
 *
 * - ++j vs j++:
 *   ++j是前置递增，返回递增后的值
 *   j++是后置递增，返回递增前的值副本
 *   在循环中++j比j++更高效(不需要保存临时副本)
 *
 * - segments->resize(k):
 *   调整向量大小
 *   如果k小于当前大小，保留前k个元素
 *   如果k大于当前大小，添加默认构造元素
 *
 * - shrink_to_fit():
 *   请求容器释放额外内存
 *   不保证一定释放，但给了容器优化内存的机会
 */
void LaneSegment::Join(std::vector<LaneSegment>* segments) {
  static constexpr double kSegmentDelta = 0.5;  /**< 段拼接容差，0.5米 */
  std::size_t k = 0;   /**< 输出索引，指向下一个有效段的位置 */
  std::size_t i = 0;   /**< 输入索引，遍历原始段 */

  /**
   * 外层循环：遍历所有段
   * while vs for:
   * while适合不确定迭代次数的场景
   */
  while (i < segments->size()) {
    std::size_t j = i;  /**< 初始化内层循环索引 */

    /**
     * 内层循环：找连续相同车道的段
     * segments->at(i).lane == segments->at(j + 1).lane:
     *   比较两个段的lane指针是否相同
     *   相同说明属于同一车道
     * j + 1 < segments->size():
     *   确保j+1索引有效
     */
    while (j + 1 < segments->size() &&
           segments->at(i).lane == segments->at(j + 1).lane) {
      ++j;  /**< 前置递增，移动到下一个段 */
    }

    /**
     * 合并段[i, j]
     * auto& segment_k: 引用，避免拷贝
     */
    auto& segment_k = segments->at(k);  /**< 获取输出位置引用 */
    segment_k.lane = segments->at(i).lane;  /**< 复制车道指针 */
    segment_k.start_s = segments->at(i).start_s;  /**< 起始s取第一个段的start_s */
    segment_k.end_s = segments->at(j).end_s;  /**< 结束s取最后一个段的end_s */

    /**
     * 边界修正：如果起始s接近0，强制设为0
     * kSegmentDelta = 0.5作为容差阈值
     */
    if (segment_k.start_s < kSegmentDelta) {
      segment_k.start_s = 0.0;
    }

    /**
     * 边界修正：如果结束s接近车道总长，设为总长
     * segment_k.lane->total_length()获取车道总长度
     */
    if (segment_k.end_s + kSegmentDelta >= segment_k.lane->total_length()) {
      segment_k.end_s = segment_k.lane->total_length();
    }

    i = j + 1;  /**< 移动到下一段(内层循环找到的最后一个的下一个) */
    ++k;        /**< 输出索引递增，指向下一个输出位置 */
  }

  segments->resize(k);  /**< 调整向量大小，丢弃多余的空段 */
  segments->shrink_to_fit();  /**< 释放多余内存 */
}

/**
 * @brief 获取右侧相邻车道航路点
 *
 * 与LeftNeighborWaypoint类似，查找右侧相邻车道
 * 代码结构完全相同，只是访问right_neighbor_forward_lane_id()
 */
LaneWaypoint RightNeighborWaypoint(const LaneWaypoint& waypoint) {
  LaneWaypoint neighbor;
  if (!waypoint.lane) {
    return neighbor;
  }
  auto point = waypoint.lane->GetSmoothPoint(waypoint.s);
  auto map_ptr = HDMapUtil::BaseMapPtr();
  CHECK_NOTNULL(map_ptr);

  /**
   * 遍历右侧相邻前向车道
   */
  for (const auto& lane_id :
       waypoint.lane->lane().right_neighbor_forward_lane_id()) {
    auto lane = map_ptr->GetLaneById(lane_id);
    if (!lane) {
      return neighbor;
    }
    double s = 0.0;
    double l = 0.0;
    if (!lane->GetProjection({point.x(), point.y()}, &s, &l)) {
      continue;
    }

    if (s < -kSampleDistance || s > lane->total_length() + kSampleDistance) {
      continue;
    } else {
      return LaneWaypoint(lane, s);
    }
  }
  return neighbor;
}

/**
 * @brief LaneSegment调试字符串
 *
 * std::string DebugString() const:
 * - const成员函数，this指针不可修改成员
 * - 返回std::string
 */
std::string LaneSegment::DebugString() const {
  if (lane == nullptr) {
    return "(lane is null)";
  }
  /**
   * absl::StrCat支持多个参数自动字符串转换
   */
  return absl::StrCat("id = ", lane->id().id(), "  start_s = ", start_s,
                      "  end_s = ", end_s);
}

/**
 * @brief 从车道段获取路径点
 *
 * @param segment 车道段
 * @return std::vector<MapPathPoint> 路径点向量
 *
 * 函数调用另一个静态函数，代码复用
 */
std::vector<MapPathPoint> MapPathPoint::GetPointsFromSegment(
    const LaneSegment& segment) {
  return GetPointsFromLane(segment.lane, segment.start_s, segment.end_s);
}

/**
 * @brief 从车道获取路径点
 *
 * 根据起止s坐标，从车道上提取路径点
 * 包括端点处的插值点
 *
 * @param lane 车道指针
 * @param start_s 起始s坐标
 * @param end_s 终止s坐标
 * @return std::vector<MapPathPoint> 路径点向量
 *
 * 核心逻辑：
 * 1. 遍历车道的所有点
 * 2. 收集在[start_s, end_s]范围内的点
 * 3. 在边界处进行线性插值
 *
 * C++语法说明:
 * - std::vector<MapPathPoint> points:
 *   创建空向量，存储MapPathPoint对象
 *
 * - if (start_s >= end_s):
 *   参数有效性检查
 *   提前返回避免无效计算
 *
 * - accumulate_s += segment.length():
 *   累积s坐标，每段长度累加
 *   等价于: accumulate_s = accumulate_s + segment.length()
 *
 * - points.emplace_back(args...):
 *   在容器末尾就地构造元素
 *   vs push_back:
 *   - push_back需要先创建对象，再拷贝/移动
 *   - emplace_back直接传递参数给构造函数
 *   更高效，减少拷贝
 *
 * - segment.start() + segment.unit_direction() * (start_s - accumulate_s):
 *   向量加法: 起点 + 单位方向 * 距离
 *   用于计算插值点的位置
 */
std::vector<MapPathPoint> MapPathPoint::GetPointsFromLane(LaneInfoConstPtr lane,
                                                          const double start_s,
                                                          const double end_s) {
  std::vector<MapPathPoint> points;  /**< 结果向量 */

  /**
   * 非法范围检查
   * start_s >= end_s表示无效的范围
   */
  if (start_s >= end_s) {  /**< 非法范围检查 */
    return points;  /**< 返回空向量 */
  }

  double accumulate_s = 0.0;  /**< 累积s坐标，从0开始 */

  /**
   * 遍历车道上的所有点
   * lane->points()返回const std::vector<Vec2d>&
   */
  for (size_t i = 0; i < lane->points().size(); ++i) {
    /**
     * 检查当前点是否在[start_s, end_s]范围内
     * accumulate_s是累积到当前点的s坐标
     */
    if (accumulate_s >= start_s && accumulate_s <= end_s) {
      /**
       * 添加在范围内的点
       * emplace_back直接在vector末尾构造
       * MapPathPoint(pos, heading, lane_waypoint)
       */
      points.emplace_back(lane->points()[i], lane->headings()[i],
                          LaneWaypoint(lane, accumulate_s));
    }

    /**
     * 处理相邻点之间的插值
     * i < lane->segments().size()确保i+1不会越界
     */
    if (i < lane->segments().size()) {
      const auto& segment = lane->segments()[i];
      /**
       * 计算下一个累积s坐标
       * 下一个点 = 当前点 + 线段长度
       */
      const double next_accumulate_s = accumulate_s + segment.length();

      /**
       * 检查起始边界
       * start_s > accumulate_s: start_s在当前点和下一点之间
       * start_s < next_accumulate_s: start_s确实在段内
       */
      if (start_s > accumulate_s && start_s < next_accumulate_s) {
        /**
         * 计算插值点
         * segment.start(): 线段起点
         * segment.unit_direction(): 线段单位方向向量
         * (start_s - accumulate_s): 从当前点到start_s的距离
         * 插值公式: start + unit_direction * distance
         */
        points.emplace_back(segment.start() + segment.unit_direction() *
                                                  (start_s - accumulate_s),
                            lane->headings()[i], LaneWaypoint(lane, start_s));
      }

      /**
       * 检查结束边界，类似起始边界处理
       */
      if (end_s > accumulate_s && end_s < next_accumulate_s) {
        points.emplace_back(
            segment.start() + segment.unit_direction() * (end_s - accumulate_s),
            lane->headings()[i], LaneWaypoint(lane, end_s));
      }

      accumulate_s = next_accumulate_s;  /**< 更新累积s坐标 */
    }

    /**
     * 超出范围，停止遍历
     * 优化：不需要继续遍历已超过end_s的点
     */
    if (accumulate_s > end_s) {
      break;
    }
  }
  return points;
}

/**
 * @brief 移除重复路径点
 *
 * 合并距离过近的点，并合并它们的车道航路点列表
 * 用于路径去重和简化
 *
 * @param points 输入/输出：路径点向量指针
 *
 * C++语法说明:
 * - static constexpr double:
 *   编译时常量，类级别共享
 *   用于判定重复的距离阈值
 *
 * - (*points)[i].DistanceSquareTo(...):
 *   解引用智能指针/指针
 *   调用对象的成员函数
 *
 * - (*points)[count - 1].add_lane_waypoints(...):
 *   add_lane_waypoints是repeated字段的添加方法
 *   合并两个航路点列表
 */
void MapPathPoint::RemoveDuplicates(std::vector<MapPathPoint>* points) {
  /**
   * 重复点判定阈值
   * 1e-7米，约0.1微米
   */
  static constexpr double kDuplicatedPointsEpsilon = 1e-7;  /**< 重复点判定阈值 */
  /**
   * 距离平方阈值
   * 使用平方比较避免开方，提高效率
   */
  static constexpr double limit =
      kDuplicatedPointsEpsilon * kDuplicatedPointsEpsilon;  /**< 距离平方阈值 */

  CHECK_NOTNULL(points);  /**< 断言检查 */
  int count = 0;          /**< 去重后的计数，指向下一个有效位置 */

  /**
   * 遍历所有点
   */
  for (size_t i = 0; i < points->size(); ++i) {
    /**
     * 判断是否重复
     * 条件1：count == 0，第一个点一定保留
     * 条件2：距离平方 > 阈值，距离足够远才保留
     *
     * DistanceSquareTo返回double类型，表示距离的平方
     */
    if (count == 0 ||
        (*points)[i].DistanceSquareTo((*points)[count - 1]) > limit) {
      /**
       * 保留该点
       * (*points)[count++] = (*points)[i]:
       * 1. 赋值到count位置
       * 2. count++返回原值后再递增
       */
      (*points)[count++] = (*points)[i];  /**< 保留该点 */
    } else {
      /**
       * 重复点：合并车道航路点
       * 将当前点的航路点列表合并到前一个有效点的列表中
       */
      (*points)[count - 1].add_lane_waypoints((*points)[i].lane_waypoints());
    }
  }

  points->resize(count);  /**< 调整向量大小，丢弃重复点 */
}

/**
 * @brief MapPathPoint调试字符串
 *
 * absl::StrJoin用于连接容器中的元素
 * 语法: StrJoin(容器, 分隔符, 格式化器)
 */
std::string MapPathPoint::DebugString() const {
  return absl::StrCat(
      "x = ", x_, "  y = ", y_, "  heading = ", heading_,
      "  lwp = "
      "{(",
      /**
       * StrJoin遍历lane_waypoints_，用"), ("连接
       * DebugStringFormatter()是自定义格式化器
       */
      absl::StrJoin(lane_waypoints_, "), (", DebugStringFormatter()), ")}");
}

/**
 * @brief Path调试字符串
 */
std::string Path::DebugString() const {
  return absl::StrCat(
      "num_points = ", num_points_,
      "  points = "
      "{(",
      absl::StrJoin(path_points_, "), (", DebugStringFormatter()),
      ")}  "
      "numlane_segments_ = ",
      lane_segments_.size(),
      "  lane_segments = "
      "{(",
      absl::StrJoin(lane_segments_, "), (", DebugStringFormatter()), ")}");
}

/**
 * @brief PathOverlap调试字符串
 */
std::string PathOverlap::DebugString() const {
  return absl::StrCat(object_id, " ", start_s, " ", end_s);
}

/**
 * @brief Path构造函数(vector拷贝)
 *
 * 使用初始化列表直接初始化成员
 * 拷贝构造，效率较低
 *
 * @param path_points 路径点向量(按值传递，会拷贝)
 *
 * C++语法说明:
 * - Path(const std::vector<MapPathPoint>& path_points):
 *   const引用参数，避免拷贝但不可修改
 *   但初始化列表中仍需拷贝
 *
 * - : path_points_(path_points):
 *   构造函数初始化列表
 *   语法: 成员名(参数名)
 *   在构造函数体执行前初始化成员
 *
 * - Init():
 *   调用初始化函数，设置其他成员
 */
Path::Path(const std::vector<MapPathPoint>& path_points)
    : path_points_(path_points) {  /**< 拷贝构造 */
  Init();  /**< 调用初始化函数 */
}

/**
 * @brief Path构造函数(vector移动)
 *
 * 使用移动语义，避免拷贝
 * 效率更高，尤其对于大型vector
 *
 * @param path_points 路径点向量(右值引用，可以移动)
 *
 * C++语法说明:
 * - std::vector<MapPathPoint>&&:
 *   右值引用，绑定到临时对象或std::move()的结果
 *   右值引用可以修改绑定的对象
 *
 * - std::move(path_points):
 *   将左值转换为右值引用
 *   告诉编译器这是一个可以"移动"的对象
 *   移动而非拷贝，大幅提高效率
 *
 * - path_points_(std::move(path_points)):
 *   移动构造函数
 *   将path_points的内容转移给path_points_
 *   转移后path_points变为空
 */
Path::Path(std::vector<MapPathPoint>&& path_points)
    : path_points_(std::move(path_points)) {  /**< 移动语义 */
  Init();
}

/**
 * @brief Path构造函数(点和段拷贝)
 */
Path::Path(const std::vector<MapPathPoint>& path_points,
           const std::vector<LaneSegment>& lane_segments)
    : path_points_(path_points), lane_segments_(lane_segments) {
  Init();
}

/**
 * @brief Path构造函数(点和段移动)
 */
Path::Path(std::vector<MapPathPoint>&& path_points,
           std::vector<LaneSegment>&& lane_segments)
    : path_points_(std::move(path_points)),
      lane_segments_(std::move(lane_segments)) {
  Init();
}

/**
 * @brief Path构造函数(带近似误差)
 *
 * @param path_points 路径点
 * @param lane_segments 车道段
 * @param max_approximation_error 最大近似误差
 *
 * 如果max_approximation_error > 0，则启用路径近似
 * 路径近似用于加速投影计算
 */
Path::Path(const std::vector<MapPathPoint>& path_points,
           const std::vector<LaneSegment>& lane_segments,
           const double max_approximation_error)
    : path_points_(path_points), lane_segments_(lane_segments) {
  Init();
  /**
   * 条件判断启用近似
   */
  if (max_approximation_error > 0.0) {
    use_path_approximation_ = true;
    approximation_ = PathApproximation(*this, max_approximation_error);
  }
}

/**
 * @brief Path构造函数(从LaneSegment向量)
 *
 * 从车道段生成路径点
 * 遍历每个车道段，提取路径点
 */
Path::Path(const std::vector<LaneSegment>& segments)
    : lane_segments_(segments) {
  /**
   * 从每个车道段提取路径点
   */
  for (const auto& segment : lane_segments_) {
    /**
     * GetPointsFromLane是静态方法
     * segment.lane是LaneInfoConstPtr类型
     */
    const auto points = MapPathPoint::GetPointsFromLane(
        segment.lane, segment.start_s, segment.end_s);
    /**
     * insert(end, begin, end):
     * 在vector末尾插入另一个范围的元素
     * 效率比循环push_back高
     */
    path_points_.insert(path_points_.end(), points.begin(), points.end());
  }
  MapPathPoint::RemoveDuplicates(&path_points_);  /**< 去重 */
  CHECK_GE(path_points_.size(), 2U);             /**< 断言至少2个点 */
  Init();
}

/**
 * @brief Path构造函数(LaneSegment移动版本)
 */
Path::Path(std::vector<LaneSegment>&& segments)
    : lane_segments_(std::move(segments)) {
  for (const auto& segment : lane_segments_) {
    const auto points = MapPathPoint::GetPointsFromLane(
        segment.lane, segment.start_s, segment.end_s);
    path_points_.insert(path_points_.end(), points.begin(), points.end());
  }
  MapPathPoint::RemoveDuplicates(&path_points_);
  CHECK_GE(path_points_.size(), 2U);
  Init();
}

/**
 * @brief Path构造函数(移动+近似)
 */
Path::Path(std::vector<MapPathPoint>&& path_points,
           std::vector<LaneSegment>&& lane_segments,
           const double max_approximation_error)
    : path_points_(std::move(path_points)),
      lane_segments_(std::move(lane_segments)) {
  Init();
  if (max_approximation_error > 0.0) {
    use_path_approximation_ = true;
    approximation_ = PathApproximation(*this, max_approximation_error);
  }
}

/**
 * @brief 路径初始化总入口
 *
 * 调用各个子初始化函数
 * 按照依赖顺序初始化
 */
void Path::Init() {
  InitPoints();          /**< 初始化路径点 */
  InitLaneSegments();     /**< 初始化车道段 */
  InitPointIndex();       /**< 初始化点索引 */
  InitWidth();            /**< 初始化宽度 */
  InitOverlaps();         /**< 初始化重叠区域 */
}

/**
 * @brief 初始化路径点相关数据
 *
 * 计算：
 * 1. 累积s坐标 (accumulated_s_)
 * 2. 线段列表 (segments_)
 * 3. 单位方向向量 (unit_directions_)
 * 4. 路径总长度 (length_)
 * 5. 采样点数量 (num_sample_points_)
 *
 * C++语法说明:
 * - accumulated_s_.reserve(num_points):
 *   预分配容量，避免多次重新分配内存
 *   只分配内存，不改变size()
 *
 * - heading.Normalize():
 *   归一化向量，使其长度为1
 *   修改调用对象本身
 *
 * - static_cast<int>(...):
 *   显式类型转换，编译时检查
 *   比C风格(int)更安全
 */
void Path::InitPoints() {
  num_points_ = static_cast<int>(path_points_.size());  /**< 路径点数量 */
  CHECK_GE(num_points_, 2);  /**< 断言至少2个点 */

  accumulated_s_.clear();
  accumulated_s_.reserve(num_points_);  /**< 预分配容量 */
  segments_.clear();
  segments_.reserve(num_points_);
  unit_directions_.clear();
  unit_directions_.reserve(num_points_);

  double s = 0.0;  /**< 累积s坐标 */

  /**
   * 遍历所有路径点
   */
  for (int i = 0; i < num_points_; ++i) {
    accumulated_s_.push_back(s);  /**< 保存当前s坐标 */

    Vec2d heading;  /**< 方向向量 */
    if (i + 1 >= num_points_) {
      /**
       * 最后一个点：使用前一个点的方向
       * path_points_[i] - path_points_[i - 1]:
       *   向量减法，从前一个点到当前点
       */
      heading = path_points_[i] - path_points_[i - 1];
      heading.Normalize();  /**< 归一化 */
    } else {
      /**
       * 创建线段：相邻两点之间的线段
       */
      segments_.emplace_back(path_points_[i], path_points_[i + 1]);
      /**
       * 计算方向向量
       * path_points_[i + 1] - path_points_[i]:
       *   从当前指向前一个点
       */
      heading = path_points_[i + 1] - path_points_[i];
      float heading_length = heading.Length();
      s += heading_length;  /**< 累积距离 */
      if (heading_length > 0.0) {
        heading /= heading_length;  /**< 归一化，除以长度 */
      }
    }
    unit_directions_.push_back(heading);
  }

  length_ = s;  /**< 总长度 */
  /**
   * 计算采样点数量
   * length_ / kSampleDistance: 多少个采样间隔
   * +1: 包含起点
   */
  num_sample_points_ = static_cast<int>(length_ / kSampleDistance) + 1;
  num_segments_ = num_points_ - 1;  /**< 线段数量 = 点数 - 1 */

  /**
   * 验证数据一致性
   * CHECK_EQ比较两个值是否相等，不等则报错
   */
  CHECK_EQ(accumulated_s_.size(), static_cast<size_t>(num_points_));
  CHECK_EQ(unit_directions_.size(), static_cast<size_t>(num_points_));
  CHECK_EQ(segments_.size(), static_cast<size_t>(num_segments_));
}

/**
 * @brief 初始化车道段
 *
 * 如果车道段为空，从路径点自动查找
 * 然后调用Join合并相邻同类段
 */
void Path::InitLaneSegments() {
  /**
   * 如果lane_segments_为空，从路径点查找
   */
  if (lane_segments_.empty()) {
    /**
     * 遍历相邻路径点对
     * i + 1 < num_points_: 确保i+1有效
     */
    for (int i = 0; i + 1 < num_points_; ++i) {
      LaneSegment lane_segment;
      if (FindLaneSegment(path_points_[i], path_points_[i + 1],
                          &lane_segment)) {
        lane_segments_.push_back(lane_segment);
      }
    }
  }

  /**
   * Join合并相邻的同类车道段
   * &lane_segments_: 取地址，传递指针
   */
  LaneSegment::Join(&lane_segments_);  /**< 合并相邻段 */

  if (lane_segments_.empty()) {
    return;  /**< 无车道段，直接返回 */
  }

  /**
   * 初始化车道段累积s坐标
   * 用于车道级别的查询
   */
  lane_accumulated_s_.resize(lane_segments_.size());
  lane_accumulated_s_[0] = lane_segments_[0].Length();
  for (std::size_t i = 1; i < lane_segments_.size(); ++i) {
    lane_accumulated_s_[i] =
        lane_accumulated_s_[i - 1] + lane_segments_[i].Length();
  }

  /**
   * 初始化点到车道的映射
   */
  lane_segments_to_next_point_.clear();
  lane_segments_to_next_point_.reserve(num_points_);
  for (int i = 0; i + 1 < num_points_; ++i) {
    LaneSegment lane_segment;
    if (FindLaneSegment(path_points_[i], path_points_[i + 1], &lane_segment)) {
      lane_segments_to_next_point_.push_back(lane_segment);
    } else {
      lane_segments_to_next_point_.push_back(LaneSegment());  /**< 添加空段 */
    }
  }

  CHECK_EQ(lane_segments_to_next_point_.size(),
           static_cast<size_t>(num_segments_));
}

/**
 * @brief 初始化路径宽度
 *
 * 按采样距离计算每个采样点的车道宽度和道路宽度
 * 考虑横向偏移l的影响
 *
 * 核心逻辑：
 * 1. 按采样距离遍历
 * 2. 找到对应路径点
 * 3. 获取该点的车道宽度和道路宽度
 * 4. 考虑横向偏移l的影响
 *
 * C++语法说明:
 * - sample_s += kSampleDistance:
 *   累加采样间隔
 *   等价于: sample_s = sample_s + kSampleDistance
 *
 * - cur_waypoint->lane->GetWidth(waypoint_s, &left_width, &right_width):
 *   ->->连续解引用
 *   cur_waypoint是指针，->获取lane成员
 *   lane也是指针，再->调用GetWidth
 */
void Path::InitWidth() {
  lane_left_width_.clear();
  lane_left_width_.reserve(num_sample_points_);
  lane_right_width_.clear();
  lane_right_width_.reserve(num_sample_points_);

  road_left_width_.clear();
  road_left_width_.reserve(num_sample_points_);
  road_right_width_.clear();
  road_right_width_.reserve(num_sample_points_);

  double sample_s = 0;        /**< 当前采样s坐标 */
  double segment_end_s = -1.0;  /**< 当前段结束s */
  double segment_start_s = -1.0;
  double waypoint_s = 0.0;    /**< 航路点s坐标 */
  double left_width = 0.0;    /**< 左边宽度 */
  double right_width = 0.0;   /**< 右边宽度 */
  const LaneWaypoint* cur_waypoint = nullptr;  /**< 当前航路点指针 */
  bool is_reach_to_end = false;
  int path_point_index = 0;

  /**
   * 采样点遍历
   * num_sample_points_ = length_ / kSampleDistance + 1
   */
  for (int i = 0; i < num_sample_points_; ++i) {
    /**
     * 找到sample_s对应的路径段
     * 使用while循环是因为sample_s会跳过多个段
     */
    while (segment_end_s < sample_s && !is_reach_to_end) {
      const auto& cur_point = path_points_[path_point_index];
      cur_waypoint = &(cur_point.lane_waypoints()[0]);
      CHECK_NOTNULL(cur_waypoint->lane);
      segment_start_s = accumulated_s_[path_point_index];
      segment_end_s = segment_start_s + segments_[path_point_index].length();
      if (++path_point_index >= num_points_) {  /**< 前置递增并检查 */
        is_reach_to_end = true;
      }
    }

    /**
     * 计算航路点s坐标
     * waypoint_s = lane_waypoint.s + (sample_s - segment_start_s)
     * 加上段内偏移
     */
    waypoint_s = cur_waypoint->s + sample_s - segment_start_s;

    /**
     * 获取车道宽度(相对于车道中心线)
     * 需要减去横向偏移l
     * l > 0表示在车道左侧
     * l < 0表示在车道右侧
     */
    // lane->GetWidth() 返回的 left_width 和 right_width 是 HD Map 中存储的原始车道宽度，相对于车道中心线
    cur_waypoint->lane->GetWidth(waypoint_s, &left_width, &right_width);
    // cur_waypoint->l 是 MapPath 路径点相对于车道中心线的横向偏移（左正右负）
    // 转换后存储的值变为相对于 MapPath 路径线的宽度
    /*
    若 MapPath 在车道中心左侧（l > 0）：左边界距离减小，右边界距离增大
    若 MapPath 在车道中心右侧（l < 0）：左边界距离增大，右边界距离减小
    */
    lane_left_width_.push_back(left_width - cur_waypoint->l);
    lane_right_width_.push_back(right_width + cur_waypoint->l);

    /**
     * 获取道路宽度
     */
    cur_waypoint->lane->GetRoadWidth(waypoint_s, &left_width, &right_width);
    road_left_width_.push_back(left_width - cur_waypoint->l);
    road_right_width_.push_back(right_width + cur_waypoint->l);

    sample_s += kSampleDistance;  /**< 移动到下一采样点 */
  }

  /**
   * 验证数组大小
   */
  auto num_sample_points = static_cast<size_t>(num_sample_points_);
  CHECK_EQ(lane_left_width_.size(), num_sample_points);
  CHECK_EQ(lane_right_width_.size(), num_sample_points);
  CHECK_EQ(road_left_width_.size(), num_sample_points);
  CHECK_EQ(road_right_width_.size(), num_sample_points);
}

/**
 * @brief 初始化采样点索引
 *
 * 建立采样s坐标到路径点索引的映射
 * 用于加速投影计算
 *
 * 使用简单算法建立映射：
 * 从起点开始，每kSampleDistance距离记录一次索引
 */
void Path::InitPointIndex() {
  last_point_index_.clear();
  last_point_index_.reserve(num_sample_points_);

  double s = 0.0;
  int last_index = 0;

  /**
   * 遍历所有采样点
   */
  for (int i = 0; i < num_sample_points_; ++i) {
    /**
     * 二分查找找到最后一个accumulated_s <= s的点
     * 但这里用线性扫描，因为数据是顺序的
     *
     * while条件：
     * - last_index + 1 < num_points_: 确保下一个索引有效
     * - accumulated_s_[last_index + 1] <= s: 下一个点仍在s之前
     */
    while (last_index + 1 < num_points_ &&
           accumulated_s_[last_index + 1] <= s) {
      ++last_index;
    }
    last_point_index_.push_back(last_index);
    s += kSampleDistance;
  }

  CHECK_EQ(last_point_index_.size(), static_cast<size_t>(num_sample_points_));
}

/**
 * @brief 获取所有重叠区域
 *
 * @param GetOverlaps_from_lane 获取重叠的函数
 * @param overlaps 输出：重叠区域向量指针
 *
 * C++语法说明:
 * - std::bind(&LaneInfo::cross_lanes, _1):
 *   绑定成员函数
 *   &LaneInfo::cross_lanes: 成员函数指针
 *   _1: 占位符，表示第一个参数
 *   创建一个函数对象，调用时传递LaneInfo引用
 *
 * - std::unordered_map<std::string, std::vector<std::pair<double, double>>>:
 *   嵌套容器
 *   外层map: 对象ID -> 重叠区间列表
 *   内层vector: 多个重叠区间
 *   pair: (start_s, end_s)
 *
 * - std::sort(segments.begin(), segments.end()):
 *   对容器排序
 *   默认升序排列
 */
void Path::GetAllOverlaps(GetOverlapFromLaneFunc GetOverlaps_from_lane,
                         std::vector<PathOverlap>* const overlaps) const {
  if (overlaps == nullptr) {
    return;
  }
  overlaps->clear();

  /**
   * 按对象ID分组存储重叠区间
   * 外层map的key是对象ID，value是区间列表
   */
  std::unordered_map<std::string, std::vector<std::pair<double, double>>>
      overlaps_by_id;

  double s = 0.0;

  /**
   * 遍历所有车道段
   */
  for (const auto& lane_segment : lane_segments_) {
    if (lane_segment.lane == nullptr) {
      continue;
    }

    /**
     * 获取该车道的所有重叠
     * GetOverlaps_from_lane是一个函数对象
     */
    for (const auto& overlap : GetOverlaps_from_lane(*(lane_segment.lane))) {
      const auto& overlap_info =
          overlap->GetObjectOverlapInfo(lane_segment.lane->id());
      if (overlap_info == nullptr) {
        continue;
      }

      const auto& lane_overlap_info = overlap_info->lane_overlap_info();

      /**
       * 检查重叠区间与车道段是否有交集
       * 区间重叠条件：start1 <= end2 && end1 >= start2
       */
      if (lane_overlap_info.start_s() <= lane_segment.end_s &&
          lane_overlap_info.end_s() >= lane_segment.start_s) {
        /**
         * 计算相对于路径的s坐标
         * ref_s: 车道段起点在路径上的s坐标
         * adjusted_start_s/end_s: 调整后的重叠区间
         */
        const double ref_s = s - lane_segment.start_s;
        const double adjusted_start_s =
            std::max(lane_overlap_info.start_s(), lane_segment.start_s) + ref_s;
        const double adjusted_end_s =
            std::min(lane_overlap_info.end_s(), lane_segment.end_s) + ref_s;

        /**
         * 遍历重叠对象
         * 排除与当前车道自身的重叠
         */
        for (const auto& object : overlap->overlap().object()) {
          if (object.id().id() != lane_segment.lane->id().id()) {
            overlaps_by_id[object.id().id()].emplace_back(adjusted_start_s,
                                                          adjusted_end_s);
          }
        }
      }
    }
    s += lane_segment.end_s - lane_segment.start_s;
  }

  /**
   * 合并相邻重叠区间
   */
  for (auto& overlaps_one_object : overlaps_by_id) {
    const std::string& object_id = overlaps_one_object.first;
    auto& segments = overlaps_one_object.second;
    std::sort(segments.begin(), segments.end());  /**< 按起始s排序 */

    const double kMinOverlapDistanceGap = 1.5;  /**< 最小合并间距 */

    /**
     * 合并距离过近的区间
     * 如果新区间的起始s接近前一区间的结束s，则合并
     */
    for (const auto& segment : segments) {
      if (!overlaps->empty() && overlaps->back().object_id == object_id &&
          segment.first - overlaps->back().end_s <= kMinOverlapDistanceGap) {
        overlaps->back().end_s =
            std::max(overlaps->back().end_s, segment.second);
      } else {
        overlaps->emplace_back(object_id, segment.first, segment.second);
      }
    }
  }

  /**
   * 按起始s排序所有重叠
   * 使用lambda表达式作为比较函数
   * lambda: [捕获列表](参数)->返回类型{函数体}
   */
  std::sort(overlaps->begin(), overlaps->end(),
            [](const PathOverlap& overlap1, const PathOverlap& overlap2) {
              return overlap1.start_s < overlap2.start_s;
            });
}

/**
 * @brief 获取下一个车道重叠
 *
 * @param s 当前s坐标
 * @return const PathOverlap* 下一个重叠区域指针
 *
 * C++语法说明:
 * - std::upper_bound:
 *   二分查找第一个大于给定值的元素
 *   返回迭代器
 *   要求容器已排序
 *
 * - lambda表达式:
 *   [](double s, const PathOverlap& o) { return s < o.start_s; }
 *   自定义比较函数，按start_s比较
 *
 * - &(*next):
 *   *解引用迭代器得到引用
 *   &取地址得到指针
 */
const PathOverlap* Path::NextLaneOverlap(double s) const {
  auto next = std::upper_bound(
      lane_overlaps_.begin(), lane_overlaps_.end(), s,
      [](double s, const PathOverlap& o) { return s < o.start_s; });
  if (next == lane_overlaps_.end()) {
    return nullptr;
  } else {
    return &(*next);
  }
}

/**
 * @brief 初始化所有重叠区域
 *
 * 使用std::bind绑定不同类型的重叠获取函数
 *
 * C++语法说明:
 * - std::bind(&LaneInfo::cross_lanes, _1):
 *   绑定成员函数cross_lanes
 *   _1表示bind后的函数第一个参数传给cross_lanes的第一个参数
 *   创建可调用对象
 */
void Path::InitOverlaps() {
  GetAllOverlaps(std::bind(&LaneInfo::cross_lanes, _1), &lane_overlaps_);
  GetAllOverlaps(std::bind(&LaneInfo::signals, _1), &signal_overlaps_);
  GetAllOverlaps(std::bind(&LaneInfo::yield_signs, _1), &yield_sign_overlaps_);
  GetAllOverlaps(std::bind(&LaneInfo::stop_signs, _1), &stop_sign_overlaps_);
  GetAllOverlaps(std::bind(&LaneInfo::crosswalks, _1), &crosswalk_overlaps_);
  GetAllOverlaps(std::bind(&LaneInfo::junctions, _1), &junction_overlaps_);
  GetAllOverlaps(std::bind(&LaneInfo::pnc_junctions, _1),
                 &pnc_junction_overlaps_);
  GetAllOverlaps(std::bind(&LaneInfo::clear_areas, _1), &clear_area_overlaps_);
  GetAllOverlaps(std::bind(&LaneInfo::speed_bumps, _1), &speed_bump_overlaps_);
  GetAllOverlaps(std::bind(&LaneInfo::parking_spaces, _1),
                 &parking_space_overlaps_);
  GetAllOverlaps(std::bind(&LaneInfo::areas, _1), &area_overlaps_);
}

/**
 * @brief 根据插值索引获取平滑路径点
 *
 * @param index 插值索引(包含基准点ID和偏移量)
 * @return MapPathPoint 平滑路径点
 *
 * 核心逻辑：
 * 1. 如果偏移量足够大，进行插值计算
 * 2. 否则直接返回基准点
 *
 * C++语法说明:
 * - const MapPathPoint& ref_point = path_points_[index.id]:
 *   const引用，避免拷贝
 *   从vector下标获取元素
 *
 * - const Vec2d delta = unit_directions_[index.id] * index.offset:
 *   Vec2d支持operator*重载
 *   表示方向向量乘以标量
 */
MapPathPoint Path::GetSmoothPoint(const InterpolatedIndex& index) const {
  CHECK_GE(index.id, 0);    /**< 断言索引有效 */
  CHECK_LT(index.id, num_points_);

  const MapPathPoint& ref_point = path_points_[index.id];  /**< 基准点 */

  /**
   * 检查偏移量
   * std::abs: 绝对值函数
   * kMathEpsilon: 极小值阈值
   */
  if (std::abs(index.offset) > kMathEpsilon) {
    /**
     * 计算偏移后的点
     * delta = unit_direction * offset
     * 沿路径方向偏移index.offset距离
     */
    const Vec2d delta = unit_directions_[index.id] * index.offset;
    /**
     * 构造新的路径点
     * {x, y}是braced-init-list构造Vec2d
     */
    MapPathPoint point({ref_point.x() + delta.x(), ref_point.y() + delta.y()},
                       ref_point.heading());

    /**
     * 更新车道航路点信息
     */
    if (index.id < num_segments_ && !ref_point.lane_waypoints().empty()) {
      const LaneSegment& lane_segment = lane_segments_to_next_point_[index.id];
      auto ref_lane_waypoint = ref_point.lane_waypoints()[0];
      if (lane_segment.lane != nullptr) {
        /**
         * 找到与当前段匹配的车道航路点
         */
        for (const auto& lane_waypoint : ref_point.lane_waypoints()) {
          if (lane_waypoint.lane->id().id() == lane_segment.lane->id().id()) {
            ref_lane_waypoint = lane_waypoint;
            break;
          }
        }
        /**
         * 添加新的航路点
         */
        point.add_lane_waypoint(
            LaneWaypoint(lane_segment.lane, lane_segment.start_s + index.offset,
                         ref_lane_waypoint.l));
      }
    }

    if (point.lane_waypoints().empty() && !ref_point.lane_waypoints().empty()) {
      point.add_lane_waypoint(ref_point.lane_waypoints()[0]);
    }

    return point;
  } else {
    return ref_point;  /**< 偏移量为0，返回基准点 */
  }
}

/**
 * @brief 根据s坐标获取平滑路径点
 */
MapPathPoint Path::GetSmoothPoint(double s) const {
  return GetSmoothPoint(GetIndexFromS(s));
}

/**
 * @brief 根据插值索引获取s坐标
 */
double Path::GetSFromIndex(const InterpolatedIndex& index) const {
  if (index.id < 0) {
    return 0.0;
  }
  if (index.id >= num_points_) {
    return length_;
  }
  return accumulated_s_[index.id] + index.offset;
}

/**
 * @brief 根据s坐标获取插值索引
 *
 * 使用二分查找加速定位
 *
 * @param s 累积距离坐标
 * @return InterpolatedIndex 插值索引
 *
 * 算法：
 * 1. 快速定位到采样区间
 * 2. 在区间内二分查找精确定位
 *
 * C++语法说明:
 * - {0, 0.0}:
 *   braced-init-list构造InterpolatedIndex
 *   假设InterpolatedIndex有相应构造函数
 *
 * - s / kSampleDistance:
 *   计算采样索引
 *
 * - (low + high) >> 1:
 *   右移1位相当于除以2
 *   等价于 (low + high) / 2
 *   但可能溢出风险更低
 */
InterpolatedIndex Path::GetIndexFromS(double s) const {
  if (s <= 0.0) {
    return {0, 0.0};  /**< 初始化列表构造 */
  }
  CHECK_GT(num_points_, 0);
  if (s >= length_) {
    return {num_points_ - 1, 0.0};
  }

  /**
   * 快速定位采样ID
   */
  const int sample_id = static_cast<int>(s / kSampleDistance);
  if (sample_id >= num_sample_points_) {
    return {num_points_ - 1, 0.0};
  }

  const int next_sample_id = sample_id + 1;
  int low = last_point_index_[sample_id];
  /**
   * ternary operator: 条件 ? 值1 : 值2
   * std::min: 取较小值
   */
  int high = (next_sample_id < num_sample_points_
                  ? std::min(num_points_, last_point_index_[next_sample_id] + 1)
                  : num_points_);

  /**
   * 二分查找精确定位
   * 经典二分查找算法
   */
  while (low + 1 < high) {
    const int mid = (low + high) >> 1;  /**< 右移1位相当于除以2 */
    if (accumulated_s_[mid] <= s) {
      low = mid;
    } else {
      high = mid;
    }
  }

  /**
   * 构建并返回插值索引
   * {low, s - accumulated_s_[low]}:
   * low是路径点索引
   * s - accumulated_s_[low]是段内偏移
   */
  return {low, s - accumulated_s_[low]};
}

/**
 * @brief 根据s坐标获取车道索引
 *
 * 使用std::lower_bound二分查找
 */
InterpolatedIndex Path::GetLaneIndexFromS(double s) const {
  if (s <= 0.0) {
    return {0, 0.0};
  }
  CHECK_GT(lane_segments_.size(), 0U);
  if (s >= length_) {
    return {static_cast<int>(lane_segments_.size() - 1),
            lane_segments_.back().Length()};
  }

  /**
   * 使用lower_bound二分查找
   * lower_bound返回第一个不小于s的元素迭代器
   */
  auto iter = std::lower_bound(lane_accumulated_s_.begin(),
                               lane_accumulated_s_.end(), s);
  if (iter == lane_accumulated_s_.end()) {
    return {static_cast<int>(lane_segments_.size() - 1),
            lane_segments_.back().Length()};
  }
  int index =
      static_cast<int>(std::distance(lane_accumulated_s_.begin(), iter));
  if (index == 0) {
    return {index, s};
  } else {
    return {index, s - lane_accumulated_s_[index - 1]};
  }
}

/**
 * @brief 获取指定s范围的车道段
 *
 * @param start_s 起始s坐标
 * @param end_s 终止s坐标
 * @return std::vector<hdmap::LaneSegment> 车道段向量
 */
std::vector<hdmap::LaneSegment> Path::GetLaneSegments(
    const double start_s, const double end_s) const {
  std::vector<hdmap::LaneSegment> lanes;
  if (start_s + kMathEpsilon > end_s) {
    return lanes;
  }

  auto start_index = GetLaneIndexFromS(start_s);

  /**
   * 检查起始偏移是否超出第一段
   */
  if (start_index.offset + kMathEpsilon >=
      lane_segments_[start_index.id].Length()) {
    start_index.id += 1;
    start_index.offset = 0;
  }

  const int num_lanes = static_cast<int>(lane_segments_.size());
  if (start_index.id >= num_lanes) {
    return lanes;
  }

  /**
   * 添加起始段
   */
  lanes.emplace_back(lane_segments_[start_index.id].lane, start_index.offset,
                     lane_segments_[start_index.id].Length());

  auto end_index = GetLaneIndexFromS(end_s);

  /**
   * 添加中间完整段
   */
  for (int i = start_index.id; i < end_index.id && i < num_lanes; ++i) {
    lanes.emplace_back(lane_segments_[i]);
  }

  /**
   * 添加结束段
   */
  if (end_index.offset >= kMathEpsilon) {
    lanes.emplace_back(lane_segments_[end_index.id].lane, 0, end_index.offset);
  }

  return lanes;
}

/**
 * @brief 获取最近点(简化版)
 */
bool Path::GetNearestPoint(const Vec2d& point, double* accumulate_s,
                           double* lateral) const {
  double distance = 0.0;
  return GetNearestPoint(point, accumulate_s, lateral, &distance);
}

/**
 * @brief 获取最近点
 *
 * @param point 查询点
 * @param accumulate_s 输出：累积s坐标
 * @param lateral 输出：横向偏移
 * @param min_distance 输出：最小距离
 * @return bool 是否成功
 */
bool Path::GetNearestPoint(const Vec2d& point, double* accumulate_s,
                           double* lateral, double* min_distance) const {
  if (!GetProjection(point, accumulate_s, lateral, min_distance)) {
    return false;
  }

  /**
   * 边界处理
   */
  if (*accumulate_s < 0.0) {
    *accumulate_s = 0.0;
    *min_distance = point.DistanceTo(path_points_[0]);
  } else if (*accumulate_s > length_) {
    *accumulate_s = length_;
    *min_distance = point.DistanceTo(path_points_.back());
  }
  return true;
}

/**
 * @brief 获取投影(简化版)
 */
bool Path::GetProjection(const common::math::Vec2d& point, double* accumulate_s,
                        double* lateral) const {
  double distance = 0.0;
  return GetProjection(point, accumulate_s, lateral, &distance);
}

/**
 * @brief 获取投影(带头信息)
 */
bool Path::GetProjection(const double heading, const common::math::Vec2d& point,
                        double* accumulate_s, double* lateral) const {
  double distance = 0.0;
  return GetProjection(point, heading, accumulate_s, lateral, &distance);
}

/**
 * @brief 使用起始猜测s获取投影
 *
 * 加速投影计算，利用起始猜测值进行区间搜索
 *
 * @param point 查询点
 * @param accumulate_s 输入/输出：累积s坐标
 * @param lateral 输出：横向偏移
 * @return bool 是否成功
 *
 * 算法：
 * 1. 使用warm_start_s定位搜索区间
 * 2. 二分搜索找到最近段
 * 3. 计算投影和横向偏移
 */
bool Path::GetProjectionWithWarmStartS(const common::math::Vec2d& point,
                                      double* accumulate_s,
                                      double* lateral) const {
  if (segments_.empty()) {
    return false;
  }
  if (accumulate_s == nullptr || lateral == nullptr) {
    return false;
  }

  /**
   * 边界处理
   */
  if (*accumulate_s < 0.0) {
    *accumulate_s = 0.0;
  } else if (*accumulate_s > length()) {
    *accumulate_s = length();
  }

  CHECK_GE(num_points_, 2);
  double warm_start_s = *accumulate_s;

  /**
   * 初始化搜索区间
   */
  int left_index = 0;
  int right_index = num_segments_;
  int mid_index = 0;

  /**
   * 二分查找最近段
   */
  while (right_index > left_index + 1) {
    FindIndex(left_index, right_index, warm_start_s, &mid_index);

    const auto& segment = segments_[mid_index];
    const auto& start_point = segment.start();

    /**
     * 计算投影
     * 投影公式: delta · unit_direction
     */
    double delta_x = point.x() - start_point.x();
    double delta_y = point.y() - start_point.y();
    const auto& unit_direction = segment.unit_direction();
    double proj = delta_x * unit_direction.x() + delta_y * unit_direction.y();

    *accumulate_s = accumulated_s_[mid_index] + proj;
    /**
     * 计算横向偏移
     * 横向偏移 = 垂直于单位方向的分量
     * 公式: unit_direction.x() * delta_y - unit_direction.y() * delta_x
     */
    *lateral = unit_direction.x() * delta_y - unit_direction.y() * delta_x;

    /**
     * 检查是否在段内
     */
    if (proj > 0.0) {
      if (proj < segment.length()) {
        return true;
      }
      if (mid_index == right_index) {
        *accumulate_s = accumulated_s_[mid_index];
        return true;
      }
      left_index = mid_index + 1;
    } else {
      if (mid_index == left_index) {
        *accumulate_s = accumulated_s_[mid_index];
        return true;
      }
      if (std::abs(proj) < segments_[mid_index - 1].length()) {
        return true;
      }
      right_index = mid_index - 1;
    }
    warm_start_s = segment.length() + proj;
  }
  return true;
}

/**
 * @brief 使用启发式参数获取投影
 *
 * 在给定s区间内搜索最近点
 */
bool Path::GetProjectionWithHueristicParams(const Vec2d& point,
                                          const double hueristic_start_s,
                                          const double hueristic_end_s,
                                          double* accumulate_s,
                                          double* lateral,
                                          double* min_distance) const {
  if (segments_.empty()) {
    return false;
  }
  if (accumulate_s == nullptr || lateral == nullptr ||
      min_distance == nullptr) {
    return false;
  }
  CHECK_GE(num_points_, 2);
  *min_distance = std::numeric_limits<double>::infinity();  /**< 初始化为无穷大 */

  /**
   * 获取搜索区间索引
   */
  int start_interpolation_index = GetIndexFromS(hueristic_start_s).id;
  int end_interpolation_index = static_cast<int>(
      std::fmin(num_segments_, GetIndexFromS(hueristic_end_s).id + 1));

  int min_index = start_interpolation_index;

  /**
   * 遍历搜索区间找最近段
   */
  for (int i = start_interpolation_index; i <= end_interpolation_index; ++i) {
    const double distance = segments_[i].DistanceSquareTo(point);
    if (distance < *min_distance) {
      min_index = i;
      *min_distance = distance;
    }
  }

  *min_distance = std::sqrt(*min_distance);
  const auto& nearest_seg = segments_[min_index];
  const auto prod = nearest_seg.ProductOntoUnit(point);
  const auto proj = nearest_seg.ProjectOntoUnit(point);

  /**
   * 根据最近段位置计算结果
   */
  if (min_index == 0) {
    *accumulate_s = std::min(proj, nearest_seg.length());
    if (proj < 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
    }
  } else if (min_index == num_segments_ - 1) {
    *accumulate_s = accumulated_s_[min_index] + std::max(0.0, proj);
    if (proj > 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
    }
  } else {
    *accumulate_s = accumulated_s_[min_index] +
                    std::max(0.0, std::min(proj, nearest_seg.length()));
    *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
  }
  return true;
}

/**
 * @brief 获取投影(完整版)
 *
 * 遍历所有段找最近点
 */
bool Path::GetProjection(const Vec2d& point, double* accumulate_s,
                        double* lateral, double* min_distance) const {
  if (segments_.empty()) {
    return false;
  }
  if (accumulate_s == nullptr || lateral == nullptr ||
      min_distance == nullptr) {
    return false;
  }

  /**
   * 使用路径近似加速
   */
  if (use_path_approximation_) {
    return approximation_.GetProjection(*this, point, accumulate_s, lateral,
                                        min_distance);
  }

  CHECK_GE(num_points_, 2);
  *min_distance = std::numeric_limits<double>::infinity();
  int min_index = 0;

  /**
   * 遍历所有段找最近
   */
  for (int i = 0; i < num_segments_; ++i) {
    const double distance = segments_[i].DistanceSquareTo(point);
    if (distance < *min_distance) {
      min_index = i;
      *min_distance = distance;
    }
  }

  *min_distance = std::sqrt(*min_distance);
  const auto& nearest_seg = segments_[min_index];
  const auto prod = nearest_seg.ProductOntoUnit(point);
  const auto proj = nearest_seg.ProjectOntoUnit(point);

  /**
   * 根据位置计算accumulate_s和lateral
   */
  if (min_index == 0) {
    *accumulate_s = std::min(proj, nearest_seg.length());
    if (proj < 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
    }
  } else if (min_index == num_segments_ - 1) {
    *accumulate_s = accumulated_s_[min_index] + std::max(0.0, proj);
    if (proj > 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
    }
  } else {
    *accumulate_s = accumulated_s_[min_index] +
                    std::max(0.0, std::min(proj, nearest_seg.length()));
    *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
  }
  return true;
}

/**
 * @brief 获取投影(带头信息版)
 *
 * 只考虑与给定航向夹角小于90度的段
 */
bool Path::GetProjection(const Vec2d& point, const double heading,
                        double* accumulate_s, double* lateral,
                        double* min_distance) const {
  if (segments_.empty()) {
    return false;
  }
  if (accumulate_s == nullptr || lateral == nullptr ||
      min_distance == nullptr) {
    return false;
  }

  if (use_path_approximation_) {
    return approximation_.GetProjection(*this, point, accumulate_s, lateral,
                                        min_distance);
  }

  CHECK_GE(num_points_, 2);
  *min_distance = std::numeric_limits<double>::infinity();
  int min_index = 0;

  /**
   * 遍历所有段，跳过航向夹角>=90度的段
   * AngleDiff计算两个角度的差值
   * M_PI_2是π/2，90度
   */
  for (int i = 0; i < num_segments_; ++i) {
    if (abs(common::math::AngleDiff(segments_[i].heading(), heading)) >= M_PI_2)
      continue;
    const double distance = segments_[i].DistanceSquareTo(point);
    if (distance < *min_distance) {
      min_index = i;
      *min_distance = distance;
    }
  }

  *min_distance = std::sqrt(*min_distance);
  const auto& nearest_seg = segments_[min_index];
  const auto prod = nearest_seg.ProductOntoUnit(point);
  const auto proj = nearest_seg.ProjectOntoUnit(point);

  if (min_index == 0) {
    *accumulate_s = std::min(proj, nearest_seg.length());
    if (proj < 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
    }
  } else if (min_index == num_segments_ - 1) {
    *accumulate_s = accumulated_s_[min_index] + std::max(0.0, proj);
    if (proj > 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
    }
  } else {
    *accumulate_s = accumulated_s_[min_index] +
                    std::max(0.0, std::min(proj, nearest_seg.length()));
    *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
  }
  return true;
}

/**
 * @brief 获取沿路径的航向
 */
bool Path::GetHeadingAlongPath(const Vec2d& point, double* heading) const {
  if (heading == nullptr) {
    return false;
  }
  double s = 0;
  double l = 0;
  if (GetProjection(point, &s, &l)) {
    *heading = GetSmoothPoint(s).heading();  /**< 获取平滑点的航向 */
    return true;
  }
  return false;
}

/**
 * @brief 获取车道左侧宽度
 */
double Path::GetLaneLeftWidth(const double s) const {
  return GetSample(lane_left_width_, s);
}

/**
 * @brief 获取车道右侧宽度
 */
double Path::GetLaneRightWidth(const double s) const {
  return GetSample(lane_right_width_, s);
}

/**
 * @brief 获取车道宽度
 */
 // 直接返回预处理数组中的值，即相对于 MapPath 路径线的宽度
bool Path::GetLaneWidth(const double s, double* lane_left_width,
                        double* lane_right_width) const {
  CHECK_NOTNULL(lane_left_width);
  CHECK_NOTNULL(lane_right_width);

  if (s < 0.0 || s > length_) {
    return false;
  }
  *lane_left_width = GetSample(lane_left_width_, s);
  *lane_right_width = GetSample(lane_right_width_, s);
  return true;
}

/**
 * @brief 获取道路左侧宽度
 */
double Path::GetRoadLeftWidth(const double s) const {
  return GetSample(road_left_width_, s);
}

/**
 * @brief 获取道路右侧宽度
 */
double Path::GetRoadRightWidth(const double s) const {
  return GetSample(road_right_width_, s);
}

/**
 * @brief 获取道路宽度
 */
bool Path::GetRoadWidth(const double s, double* road_left_width,
                        double* road_right_width) const {
  CHECK_NOTNULL(road_left_width);
  CHECK_NOTNULL(road_right_width);

  if (s < 0.0 || s > length_) {
    return false;
  }

  *road_left_width = GetSample(road_left_width_, s);
  *road_right_width = GetSample(road_right_width_, s);
  return true;
}

/**
 * @brief 获取采样值
 *
 * 在预计算的采样数组中进行线性插值
 *
 * @param samples 采样值数组
 * @param s 查询s坐标
 * @return double 插值结果
 *
 * 算法：
 * 1. 如果s<=0，返回第一个值
 * 2. 如果s超过最后采样点，返回最后一个值
 * 3. 否则进行线性插值
 *
 * C++语法说明:
 * - samples.back():
 *   返回vector最后一个元素的引用
 *   O(1)时间复杂度
 *
 * - (1.0 - ratio) * samples[idx] + ratio * samples[idx + 1]:
 *   线性插值公式
 *   ratio是[0,1]之间的比例
 */
double Path::GetSample(const std::vector<double>& samples,
                       const double s) const {
  if (samples.empty()) {
    return 0.0;
  }
  if (s <= 0.0) {
    return samples[0];
  }

  /**
   * 计算采样索引
   */
  const int idx = static_cast<int>(s / kSampleDistance);   // 0.25
  if (idx >= num_sample_points_ - 1) {
    return samples.back();
  }

  /**
   * 线性插值
   * ratio: [0, 1]之间的比例
   */
  const double ratio = (s - idx * kSampleDistance) / kSampleDistance;
  return samples[idx] * (1.0 - ratio) + samples[idx + 1] * ratio;
}

/**
 * @brief 检查点是否在路径上
 *
 * 点在路径上条件：
 * 1. 能投影到路径
 * 2. 横向偏移在车道宽度范围内
 */
bool Path::IsOnPath(const Vec2d& point) const {
  double accumulate_s = 0.0;
  double lateral = 0.0;
  if (!GetProjection(point, &accumulate_s, &lateral)) {
    return false;
  }
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  if (!GetLaneWidth(accumulate_s, &lane_left_width, &lane_right_width)) {
    return false;
  }
  if (lateral < lane_left_width && lateral > -lane_right_width) {
    return true;
  }
  return false;
}

/**
 * @brief 检查是否与包围盒重叠
 *
 * @param box 包围盒
 * @param width 扩展宽度
 * @return bool 是否重叠
 */
bool Path::OverlapWith(const common::math::Box2d& box, double width) const {
  if (use_path_approximation_) {
    return approximation_.OverlapWith(*this, box, width);
  }
  const Vec2d center = box.center();
  const double radius_sqr = Sqr(box.diagonal() / 2.0 + width) + kMathEpsilon;

  /**
   * 遍历所有段检查距离
   */
  for (const auto& segment : segments_) {
    if (segment.DistanceSquareTo(center) > radius_sqr) {
      continue;  /**< 跳过不相交的段 */
    }
    if (box.DistanceTo(segment) <= width + kMathEpsilon) {
      return true;
    }
  }
  return false;
}

/**
 * @brief 计算路径段的最大误差
 *
 * 用于路径近似算法
 *
 * @param path 路径
 * @param s 起始索引
 * @param t 终止索引
 * @return double 最大距离误差
 */
double PathApproximation::compute_max_error(const Path& path, const int s,
                                           const int t) {
  if (s + 1 >= t) {
    return 0.0;
  }
  const auto& points = path.path_points();
  const LineSegment2d segment(points[s], points[t]);
  double max_distance_sqr = 0.0;
  for (int i = s + 1; i < t; ++i) {
    max_distance_sqr =
        std::max(max_distance_sqr, segment.DistanceSquareTo(points[i]));
  }
  return sqrt(max_distance_sqr);
}

/**
 * @brief 检查路径段是否在最大误差内
 */
bool PathApproximation::is_within_max_error(const Path& path, const int s,
                                            const int t) {
  if (s + 1 >= t) {
    return true;
  }
  const auto& points = path.path_points();
  const LineSegment2d segment(points[s], points[t]);
  for (int i = s + 1; i < t; ++i) {
    if (segment.DistanceSquareTo(points[i]) > max_sqr_error_) {
      return false;
    }
  }
  return true;
}

/**
 * @brief 路径近似初始化
 */
void PathApproximation::Init(const Path& path) {
  InitDilute(path);      /**< 稀释路径点 */
  InitProjections(path);  /**< 初始化投影 */
}

/**
 * @brief 稀释路径点
 *
 * 使用二分查找找到满足误差要求的最大间隔
 */
void PathApproximation::InitDilute(const Path& path) {
  const int num_original_points = path.num_points();
  original_ids_.clear();
  int last_idx = 0;

  while (last_idx < num_original_points - 1) {
    original_ids_.push_back(last_idx);

    int next_idx = last_idx + 1;
    int delta = 2;

    /**
     * 倍增查找最大间隔
     */
    for (; last_idx + delta < num_original_points; delta *= 2) {
      if (!is_within_max_error(path, last_idx, last_idx + delta)) {
        break;
      }
      next_idx = last_idx + delta;
    }

    /**
     * 二分细化
     */
    for (; delta > 0; delta /= 2) {
      if (next_idx + delta < num_original_points &&
          is_within_max_error(path, last_idx, next_idx + delta)) {
        next_idx += delta;
      }
    }

    last_idx = next_idx;
  }

  original_ids_.push_back(last_idx);
  num_points_ = static_cast<int>(original_ids_.size());
  if (num_points_ == 0) {
    return;
  }

  /**
   * 构建近似线段
   */
  segments_.clear();
  segments_.reserve(num_points_ - 1);
  for (int i = 0; i < num_points_ - 1; ++i) {
    segments_.emplace_back(path.path_points()[original_ids_[i]],
                           path.path_points()[original_ids_[i + 1]]);
  }

  /**
   * 计算每段的误差
   */
  max_error_per_segment_.clear();
  max_error_per_segment_.reserve(num_points_ - 1);
  for (int i = 0; i < num_points_ - 1; ++i) {
    max_error_per_segment_.push_back(
        compute_max_error(path, original_ids_[i], original_ids_[i + 1]));
  }
}

/**
 * @brief 初始化投影数据
 */
void PathApproximation::InitProjections(const Path& path) {
  if (num_points_ == 0) {
    return;
  }

  projections_.clear();
  projections_.reserve(segments_.size() + 1);
  double s = 0.0;
  projections_.push_back(0);
  for (const auto& segment : segments_) {
    s += segment.length();
    projections_.push_back(s);
  }

  const auto& original_points = path.path_points();
  const int num_original_points = static_cast<int>(original_points.size());

  /**
   * 计算原始点到近似路径的投影
   */
  original_projections_.clear();
  original_projections_.reserve(num_original_points);
  for (size_t i = 0; i < projections_.size(); ++i) {
    original_projections_.push_back(projections_[i]);
    if (i + 1 < projections_.size()) {
      const auto& segment = segments_[i];
      for (int idx = original_ids_[i] + 1; idx < original_ids_[i + 1]; ++idx) {
        const double proj = segment.ProjectOntoUnit(original_points[idx]);
        original_projections_.push_back(
            projections_[i] + std::max(0.0, std::min(proj, segment.length())));
      }
    }
  }

  /**
   * 计算向左最大投影
   */
  max_original_projections_to_left_.resize(num_original_points);
  double last_projection = -std::numeric_limits<double>::infinity();
  for (int i = 0; i < num_original_points; ++i) {
    last_projection = std::max(last_projection, original_projections_[i]);
    max_original_projections_to_left_[i] = last_projection;
  }
  for (int i = 0; i + 1 < num_original_points; ++i) {
    CHECK_LE(max_original_projections_to_left_[i],
             max_original_projections_to_left_[i + 1] + kMathEpsilon);
  }

  /**
   * 计算向右最小投影
   */
  min_original_projections_to_right_.resize(original_projections_.size());
  last_projection = std::numeric_limits<double>::infinity();
  for (int i = num_original_points - 1; i >= 0; --i) {
    last_projection = std::min(last_projection, original_projections_[i]);
    min_original_projections_to_right_[i] = last_projection;
  }
  for (int i = 0; i + 1 < num_original_points; ++i) {
    CHECK_LE(min_original_projections_to_right_[i],
             min_original_projections_to_right_[i + 1] + kMathEpsilon);
  }

  /**
   * 采样最大投影
   */
  max_projection_ = projections_.back();
  num_projection_samples_ =
      static_cast<int>(max_projection_ / kSampleDistance) + 1;
  sampled_max_original_projections_to_left_.clear();
  sampled_max_original_projections_to_left_.reserve(num_projection_samples_);

  double proj = 0.0;
  int last_index = 0;
  for (int i = 0; i < num_projection_samples_; ++i) {
    while (last_index + 1 < num_original_points &&
           max_original_projections_to_left_[last_index + 1] < proj) {
      ++last_index;
    }
    sampled_max_original_projections_to_left_.push_back(last_index);
    proj += kSampleDistance;
  }

  CHECK_EQ(sampled_max_original_projections_to_left_.size(),
           static_cast<size_t>(num_projection_samples_));
}

/**
 * @brief 路径近似的投影计算
 */
bool PathApproximation::GetProjection(const Path& path,
                                    const common::math::Vec2d& point,
                                    double* accumulate_s, double* lateral,
                                    double* min_distance) const {
  if (num_points_ == 0) {
    return false;
  }
  if (accumulate_s == nullptr || lateral == nullptr ||
      min_distance == nullptr) {
    return false;
  }

  double min_distance_sqr = std::numeric_limits<double>::infinity();
  int estimate_nearest_segment_idx = -1;
  std::vector<double> distance_sqr_to_segments;
  distance_sqr_to_segments.reserve(segments_.size());

  /**
   * 找最近的近似段
   */
  for (size_t i = 0; i < segments_.size(); ++i) {
    const double distance_sqr = segments_[i].DistanceSquareTo(point);
    distance_sqr_to_segments.push_back(distance_sqr);
    if (distance_sqr < min_distance_sqr) {
      min_distance_sqr = distance_sqr;
      estimate_nearest_segment_idx = static_cast<int>(i);
    }
  }

  if (estimate_nearest_segment_idx < 0) {
    return false;
  }

  const auto& original_segments = path.segments();
  const int num_original_segments = static_cast<int>(original_segments.size());
  const auto& original_accumulated_s = path.accumulated_s();

  double min_distance_sqr_with_error =
      Sqr(sqrt(min_distance_sqr) +
          max_error_per_segment_[estimate_nearest_segment_idx] + max_error_);
  *min_distance = std::numeric_limits<double>::infinity();
  int nearest_segment_idx = -1;

  /**
   * 精确搜索原始段
   */
  for (size_t i = 0; i < segments_.size(); ++i) {
    if (distance_sqr_to_segments[i] >= min_distance_sqr_with_error) {
      continue;
    }
    int first_segment_idx = original_ids_[i];
    int last_segment_idx = original_ids_[i + 1] - 1;
    double max_original_projection = std::numeric_limits<double>::infinity();

    if (first_segment_idx < last_segment_idx) {
      const auto& segment = segments_[i];
      const double projection = segment.ProjectOntoUnit(point);
      const double prod_sqr = Sqr(segment.ProductOntoUnit(point));
      if (prod_sqr >= min_distance_sqr_with_error) {
        continue;
      }
      const double scan_distance = sqrt(min_distance_sqr_with_error - prod_sqr);
      const double min_projection = projection - scan_distance;
      max_original_projection = projections_[i] + projection + scan_distance;
      if (min_projection > 0.0) {
        const double limit = projections_[i] + min_projection;
        const int sample_index =
            std::max(0, static_cast<int>(limit / kSampleDistance));
        if (sample_index >= num_projection_samples_) {
          first_segment_idx = last_segment_idx;
        } else {
          first_segment_idx =
              std::max(first_segment_idx,
                       sampled_max_original_projections_to_left_[sample_index]);
          if (first_segment_idx >= last_segment_idx) {
            first_segment_idx = last_segment_idx;
          } else {
            while (first_segment_idx < last_segment_idx &&
                   max_original_projections_to_left_[first_segment_idx + 1] <
                       limit) {
              ++first_segment_idx;
            }
          }
        }
      }
    }

    bool min_distance_updated = false;
    bool is_within_end_point = false;
    for (int idx = first_segment_idx; idx <= last_segment_idx; ++idx) {
      if (min_original_projections_to_right_[idx] > max_original_projection) {
        break;
      }
      const auto& original_segment = original_segments[idx];
      const double x0 = point.x() - original_segment.start().x();
      const double y0 = point.y() - original_segment.start().y();
      const double ux = original_segment.unit_direction().x();
      const double uy = original_segment.unit_direction().y();
      double proj = x0 * ux + y0 * uy;
      double distance = 0.0;
      if (proj < 0.0) {
        if (is_within_end_point) {
          continue;
        }
        is_within_end_point = true;
        distance = hypot(x0, y0);
      } else if (proj <= original_segment.length()) {
        is_within_end_point = true;
        distance = std::abs(x0 * uy - y0 * ux);
      } else {
        is_within_end_point = false;
        if (idx != last_segment_idx) {
          continue;
        }
        distance = original_segment.end().DistanceTo(point);
      }
      if (distance < *min_distance) {
        min_distance_updated = true;
        *min_distance = distance;
        nearest_segment_idx = idx;
      }
    }
    if (min_distance_updated) {
      min_distance_sqr_with_error = Sqr(*min_distance + max_error_);
    }
  }

  if (nearest_segment_idx >= 0) {
    const auto& segment = original_segments[nearest_segment_idx];
    double proj = segment.ProjectOntoUnit(point);
    const double prod = segment.ProductOntoUnit(point);
    if (nearest_segment_idx > 0) {
      proj = std::max(0.0, proj);
    }
    if (nearest_segment_idx + 1 < num_original_segments) {
      proj = std::min(segment.length(), proj);
    }
    *accumulate_s = original_accumulated_s[nearest_segment_idx] + proj;
    if ((nearest_segment_idx == 0 && proj < 0.0) ||
        (nearest_segment_idx + 1 == num_original_segments &&
         proj > segment.length())) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0 ? (*min_distance) : -(*min_distance));
    }
    return true;
  }
  return false;
}

/**
 * @brief 路径近似的重叠检测
 */
bool PathApproximation::OverlapWith(const Path& path, const Box2d& box,
                                   double width) const {
  if (num_points_ == 0) {
    return false;
  }
  const Vec2d center = box.center();
  const double radius = box.diagonal() / 2.0 + width;
  const double radius_sqr = Sqr(radius);
  const auto& original_segments = path.segments();

  for (size_t i = 0; i < segments_.size(); ++i) {
    const LineSegment2d& segment = segments_[i];
    const double max_error = max_error_per_segment_[i];
    const double radius_sqr_with_error = Sqr(radius + max_error);
    if (segment.DistanceSquareTo(center) > radius_sqr_with_error) {
      continue;
    }
    int first_segment_idx = original_ids_[i];
    int last_segment_idx = original_ids_[i + 1] - 1;
    double max_original_projection = std::numeric_limits<double>::infinity();
    if (first_segment_idx < last_segment_idx) {
      const auto& segment = segments_[i];
      const double projection = segment.ProjectOntoUnit(center);
      const double prod_sqr = Sqr(segment.ProductOntoUnit(center));
      if (prod_sqr >= radius_sqr_with_error) {
        continue;
      }
      const double scan_distance = sqrt(radius_sqr_with_error - prod_sqr);
      const double min_projection = projection - scan_distance;
      max_original_projection = projections_[i] + projection + scan_distance;
      if (min_projection > 0.0) {
        const double limit = projections_[i] + min_projection;
        const int sample_index =
            std::max(0, static_cast<int>(limit / kSampleDistance));
        if (sample_index >= num_projection_samples_) {
          first_segment_idx = last_segment_idx;
        } else {
          first_segment_idx =
              std::max(first_segment_idx,
                       sampled_max_original_projections_to_left_[sample_index]);
          if (first_segment_idx >= last_segment_idx) {
            first_segment_idx = last_segment_idx;
          } else {
            while (first_segment_idx < last_segment_idx &&
                   max_original_projections_to_left_[first_segment_idx + 1] <
                       limit) {
              ++first_segment_idx;
            }
          }
        }
      }
    }
    for (int idx = first_segment_idx; idx <= last_segment_idx; ++idx) {
      if (min_original_projections_to_right_[idx] > max_original_projection) {
        break;
      }
      const auto& original_segment = original_segments[idx];
      if (original_segment.DistanceSquareTo(center) > radius_sqr) {
        continue;
      }
      if (box.DistanceTo(original_segment) <= width) {
        return true;
      }
    }
  }
  return false;
}

/**
 * @brief 二分查找定位段索引
 *
 * @param left_index 搜索区间左索引
 * @param right_index 搜索区间右索引
 * @param target_s 目标s坐标
 * @param mid_index 输出：找到的段索引
 *
 * 算法：经典二分查找
 *
 * C++语法说明:
 * - ((left_index + right_index) >> 1):
 *   右移1位 = 除以2
 *   等价于 (left_index + right_index) / 2
 */
void Path::FindIndex(int left_index, int right_index, double target_s,
                    int* mid_index) const {
  /**
   * 二分查找找到target_s所在的段
   */
  while (right_index > left_index + 1) {
    *mid_index = ((left_index + right_index) >> 1);  /**< 右移1位=除以2 */
    if (accumulated_s_[*mid_index] < target_s) {
      left_index = *mid_index;
      continue;
    }
    if (accumulated_s_[*mid_index - 1] > target_s) {
      right_index = *mid_index - 1;
      continue;
    }
    return;
  }
}

}  // namespace hdmap
}  // namespace apollo
