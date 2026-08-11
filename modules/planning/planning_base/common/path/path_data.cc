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
 * @file path_data.cc
 **/

#include "modules/planning/planning_base/common/path/path_data.h"

/**
 * @brief 标准库算法头文件
 *
 * C++语法说明：
 * - #include <algorithm>：
 *   标准库algorithm头文件
 *   提供了大量通用算法，如std::sort, std::find, std::transform等
 *   在本文件中用于查找、比较等操作
 */

#include <algorithm>

/**
 * @brief Abseil字符串处理库头文件
 *
 * 功能说明：
 * - absl/strings/str_cat.h：字符串连接功能
 * - absl/strings/str_join.h：字符串Join功能
 *
 * C++语法说明：
 * - absl::StrCat：类似Python的str1 + str2 + ...，但更高效
 * - absl::StrJoin：将字符串数组用分隔符连接
 * - 这些是Google Abseil库的组件，Apollo中广泛使用
 */

#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"

/**
 * @brief Cyber RT日志系统头文件
 *
 * 功能说明：
 * - cyber/common/log.h：Cyber RT框架的日志宏定义
 *
 * C++语法说明：
 * - AERROR：错误级别日志
 * - ACHECK：断言宏，失败时输出错误并终止程序
 * - ADEBUG：调试级别日志
 * - ADOVECALL：详细调试日志
 */

#include "cyber/common/log.h"

/**
 * @brief 笛卡尔坐标系与Frenet坐标系转换头文件
 *
 * 功能说明：
 * - common/math/cartesian_frenet_conversion.h：
 *   提供了CartesianFrenetConverter类
 *   用于在笛卡尔坐标(x,y)和Frenet坐标(s,l)之间转换
 *
 * Frenet坐标系：
 * - 以参考线为s轴，垂直于参考线的方向为l轴
 * - s：沿参考线的累积距离
 * - l：到参考线的横向距离（有正负）
 */

#include "modules/common/math/cartesian_frenet_conversion.h"

/**
 * @brief 点工厂工具头文件
 *
 * 功能说明：
 * - common/util/point_factory.h：
 *   提供便捷创建各种类型点的工厂函数
 *   如PathPoint, SLPoint, Vec2d等
 *
 * C++语法说明：
 * - PointFactory::ToSLPoint：创建SL点
 * - PointFactory::ToVec2d：创建2D向量
 * - PointFactory::ToPathPoint：创建路径点
 */

#include "modules/common/util/point_factory.h"

/**
 * @brief 字符串工具头文件
 *
 * 功能说明：
 * - common/util/string_util.h：
 *   提供字符串处理的工具函数
 */

#include "modules/common/util/string_util.h"

/**
 * @brief 规划模块全局flags头文件
 *
 * 功能说明：
 * - planning_gflags.h：
 *   定义了规划模块使用的gflags全局变量
 *   如FLAGS_trajectory_point_num_for_debug用于调试时限制输出点数
 *
 * C++语法说明：
 * - gflags是Google开发的命令行参数处理库
 * - FLAGS_xxx：宏命名约定，表示全局flag变量
 */

#include "modules/planning/planning_base/gflags/planning_gflags.h"

/**
 * @brief Apollo命名空间开始
 *
 * C++语法说明：
 * - namespace apollo：
 *   最外层命名空间，Apollo项目所有代码都在此命名空间下
 * - namespace planning：
 *   规划模块的子命名空间
 *   使用嵌套命名空间来组织代码，避免命名冲突
 */
namespace apollo {
namespace planning {

/**
 * @brief 类型别名声明
 *
 * 功能说明：
 * 为常用类型创建别名，简化代码书写
 *
 * C++语法说明：
 * - using XXX = YYY：
 *   类型别名声明，与typedef等价但更直观
 *   using apollo::common::PathPoint = 类型简化书写
 *
 * - apollo::common::PathPoint：
 *   路径点类型，包含x,y,z,theta,kappa等信息
 *
 * - apollo::common::PointENU：
 *   ENU坐标系的点类型（East-North-Up）
 *
 * - apollo::common::SLPoint：
 *   SL坐标系的点类型（沿参考线距离-横向距离）
 *
 * - apollo::common::math::CartesianFrenetConverter：
 *   坐标系转换工具类
 *
 * - apollo::common::util::PointFactory：
 *   点工厂类，用于创建各种点对象
 */
using apollo::common::PathPoint;
using apollo::common::PointENU;
using apollo::common::SLPoint;
using apollo::common::math::CartesianFrenetConverter;
using apollo::common::util::PointFactory;

/**
 * @brief 设置离散化路径
 *
 * @param path 离散化路径（笛卡尔坐标系）
 * @return bool 设置是否成功
 *
 * 功能说明：
 * 将离散化路径（笛卡尔坐标系）设置为PathData的路径
 * 同时自动计算对应的Frenet坐标系路径
 *
 * 算法流程：
 * 1. 检查参考线是否有效
 * 2. 将传入的路径移动到成员变量discretized_path_
 * 3. 调用XYToSL将笛卡尔路径转换为Frenet路径
 * 4. 验证两个路径的点数一致
 *
 * C++语法说明：
 * - std::move(path)：
 *   移动语义，将path的资源转移给discretized_path_
 *   避免不必要的数据拷贝，提高效率
 *   移动后path变为空状态
 *
 * - &frenet_path_：
 *   传入指针用于输出Frenet路径结果
 *
 * - DCHECK_EQ(a, b)：
 *   Debug模式下断言，验证a==b
 *   如果不相等则输出错误信息
 *   Release模式下不执行
 */
bool PathData::SetDiscretizedPath(DiscretizedPath path) {
  // 检查参考线指针是否为空
  // 参考线是坐标转换的必要参考，必须先设置
  if (reference_line_ == nullptr) {
    // 输出错误日志，说明需要先设置参考线
    AERROR << "Should NOT set discretized path when reference line is nullptr. "
              "Please set reference line first.";
    return false;
  }
  // 使用移动语义设置离散化路径，避免拷贝
  discretized_path_ = std::move(path);
  // 将笛卡尔坐标系的路径转换为Frenet坐标系的路径
  if (!XYToSL(discretized_path_, &frenet_path_)) {
    AERROR << "Fail to transfer discretized path to frenet path.";
    return false;
  }
  // 调试断言：确保两个路径的点数相同
  // 这是因为每个笛卡尔点都应该有对应的Frenet点
  DCHECK_EQ(discretized_path_.size(), frenet_path_.size());
  return true;
}

/**
 * @brief 设置Frenet路径
 *
 * @param frenet_path Frenet坐标系的路径
 * @return bool 设置是否成功
 *
 * 功能说明：
 * 将Frenet路径设置为PathData的路径
 * 同时自动计算对应的笛卡尔坐标系路径
 *
 * 与SetDiscretizedPath的区别：
 * - SetDiscretizedPath：输入是笛卡尔路径，输出是Frenet路径
 * - SetFrenetPath：输入是Frenet路径，输出是笛卡尔路径
 *
 * C++语法说明：
 * - std::move(frenet_path)：
 *   移动语义，将frenet_path的资源转移给成员变量
 */
bool PathData::SetFrenetPath(FrenetFramePath frenet_path) {
  // 检查参考线是否有效
  if (reference_line_ == nullptr) {
    AERROR << "Should NOT set frenet path when reference line is nullptr. "
              "Please set reference line first.";
    return false;
  }
  // 使用移动语义设置Frenet路径
  frenet_path_ = std::move(frenet_path);
  // 将Frenet坐标系的路径转换为笛卡尔坐标系的路径
  if (!SLToXY(frenet_path_, &discretized_path_)) {
    AERROR << "Fail to transfer frenet path to discretized path.";
    return false;
  }
  // 调试断言：确保两个路径的点数相同
  DCHECK_EQ(discretized_path_.size(), frenet_path_.size());
  return true;
}

/**
 * @brief 设置路径点决策指南
 *
 * @param path_point_decision_guide 路径点决策指南
 *        每个元素是tuple(沿参考线距离, 路径点类型, 到最近障碍物的距离)
 * @return bool 设置是否成功
 *
 * 功能说明：
 * 设置路径点的决策指南，用于指导速度限制的生成
 * 这些信息由路径分析器生成，帮助速度边界决策器确定速度限制
 *
 * tuple结构说明：
 * - 第一个元素double：路径点沿参考线的s坐标
 * - 第二个元素PathPointType：路径点的类型（车道内、车道外等）
 * - 第三个元素double：到最近障碍物的距离
 *
 * C++语法说明：
 * - std::tuple<double, PathPointType, double>：
 *   元组模板类，可以存储不同类型的多个值
 *   类似Python的tuple，但需要预先定义元素类型
 *
 * - std::move(path_point_decision_guide)：
 *   移动语义，避免大数据结构拷贝
 */
bool PathData::SetPathPointDecisionGuide(
    std::vector<std::tuple<double, PathData::PathPointType, double>>
        path_point_decision_guide) {
  // 检查参考线是否有效
  if (reference_line_ == nullptr) {
    AERROR << "Should NOT set path_point_decision_guide when reference line is "
              "nullptr. ";
    return false;
  }
  // 检查Frenet路径和笛卡尔路径是否非空
  if (frenet_path_.empty() || discretized_path_.empty()) {
    AERROR << "Should NOT set path_point_decision_guide when frenet_path or "
              "world frame trajectory is empty. ";
    return false;
  }
  // 使用移动语义设置决策指南
  path_point_decision_guide_ = std::move(path_point_decision_guide);
  return true;
}

/**
 * @brief 获取离散化路径的常量引用
 *
 * @return const DiscretizedPath& 离散化路径的常量引用
 *
 * 功能说明：
 * 返回笛卡尔坐标系下的离散化路径
 * 返回常量引用避免拷贝，同时防止外部修改
 *
 * C++语法说明：
 * - const DiscretizedPath&：
 *   返回常量引用
 *   - const：返回的引用不可修改
 *   - &：返回引用而非副本，避免拷贝开销
 */
const DiscretizedPath &PathData::discretized_path() const {
  return discretized_path_;
}

/**
 * @brief 获取Frenet路径的常量引用
 *
 * @return const FrenetFramePath& Frenet路径的常量引用
 *
 * 功能说明：
 * 返回Frenet坐标系下的路径
 * 用于需要使用Frenet坐标进行计算的场景
 *
 * C++语法说明：
 * - const成员函数：
 *   函数声明后的const表示this指针是常量
 *   在const成员函数中不能修改任何成员变量
 */
const FrenetFramePath &PathData::frenet_frame_path() const {
  return frenet_path_;
}

/**
 * @brief 获取路径点决策指南的常量引用
 *
 * @return const vector<tuple>& 决策指南的常量引用
 *
 * C++语法说明：
 * - std::vector<std::tuple<double, PathData::PathPointType, double>>：
 *   存储元组的向量
 *   PathData::PathPointType指定使用PathData类内定义的枚举类型
 */
const std::vector<std::tuple<double, PathData::PathPointType, double>>
    &PathData::path_point_decision_guide() const {
  return path_point_decision_guide_;
}

/**
 * @brief 检查路径是否为空
 *
 * @return bool 如果两条路径都为空返回true
 *
 * 功能说明：
 * 判断PathData是否为空（既没有笛卡尔路径也没有Frenet路径）
 */
bool PathData::Empty() const {
  // 使用逻辑与运算符
  // 当且仅当两个路径都为空时才返回true
  return discretized_path_.empty() && frenet_path_.empty();
}

/**
 * @brief 设置参考线
 *
 * @param reference_line 指向参考线的指针
 *
 * 功能说明：
 * 设置PathData使用的参考线
 * 参考线用于坐标系转换（笛卡尔<->Frenet）
 *
 * 算法流程：
 * 1. 调用Clear()清空现有数据
 * 2. 设置新的参考线指针
 *
 * C++语法说明：
 * - const ReferenceLine *：
 *   指向常量ReferenceLine对象的指针
 *   使用指针而非引用，因为引用不能为空
 */
void PathData::SetReferenceLine(const ReferenceLine *reference_line) {
  Clear();
  reference_line_ = reference_line;
}

/**
 * @brief 根据路径s坐标获取路径点
 *
 * @param s 沿路径的累积距离
 * @return common::PathPoint 在该s位置的道路点
 *
 * 功能说明：
 * 根据沿路径的累积距离s获取对应的路径点
 * 直接在离散化路径上通过插值获取
 *
 * C++语法说明：
 * - discretized_path_.Evaluate(s)：
 *   DiscretizedPath类的Evaluate方法
 *   根据s值在路径点上插值计算对应的PathPoint
 */
common::PathPoint PathData::GetPathPointWithPathS(const double s) const {
  return discretized_path_.Evaluate(s);
}

/**
 * @brief 根据参考线s坐标获取路径点
 *
 * @param ref_s 沿参考线的累积距离
 * @param path_point 输出：对应的路径点
 * @return bool 是否成功获取
 *
 * 功能说明：
 * 根据沿参考线的累积距离ref_s找到对应的路径点
 * 这个方法会在Frenet路径上进行插值
 *
 * 算法流程：
 * 1. 检查输入参数的有效性（ref_s > 0且不超过路径长度）
 * 2. 在Frenet路径上找到ref_s所在的区间
 * 3. 在该区间内进行线性插值
 * 4. 使用插值得到的s在笛卡尔路径上计算实际路径点
 *
 * C++语法说明：
 * - common::PathPoint *const path_point：
 *   指向PathPoint的常量指针
 *   指针本身是常量（不能改变指向），但可以修改指向的对象
 *
 * - ACHECK(reference_line_)：
 *   断言reference_line_非空
 *   如果为空，程序会报错并终止
 *
 * - uint32_t：
 *   无符号32位整数类型
 *   用于数组索引，因为索引总是非负的
 *
 * - fabs(ref_s - frenet_path_.at(i).s()) < kDistanceEpsilon：
 *   浮点数比较，考虑了数值误差
 *   kDistanceEpsilon = 1e-3 是容差值
 */
bool PathData::GetPathPointWithRefS(const double ref_s,
                                    common::PathPoint *const path_point) const {
  ACHECK(reference_line_);  // 断言参考线已设置
  DCHECK_EQ(discretized_path_.size(), frenet_path_.size());  // 调试断言

  // 检查ref_s是否为负数
  if (ref_s < 0) {
    AERROR << "ref_s[" << ref_s << "] should be > 0";
    return false;
  }

  // 检查ref_s是否超过Frenet路径长度
  if (ref_s > frenet_path_.back().s()) {
    AERROR << "ref_s is larger than the length of frenet_path_ length ["
           << frenet_path_.back().s() << "].";
    return false;
  }

  // 初始化索引
  uint32_t index = 0;
  // 容差常量，用于浮点数比较
  const double kDistanceEpsilon = 1e-3;

  // 遍历Frenet路径，找到ref_s所在的区间
  for (uint32_t i = 0; i + 1 < frenet_path_.size(); ++i) {
    // 检查是否非常接近某个点
    if (fabs(ref_s - frenet_path_.at(i).s()) < kDistanceEpsilon) {
      // 直接使用该点的笛卡尔坐标
      path_point->CopyFrom(discretized_path_.at(i));
      return true;
    }
    // 检查ref_s是否在当前点和下一点之间
    if (frenet_path_.at(i).s() < ref_s && ref_s <= frenet_path_.at(i + 1).s()) {
      index = i;  // 记录区间左端点索引
      break;       // 找到区间，跳出循环
    }
  }

  // 计算在线段中的相对位置 [0, 1]
  // 线性插值公式：r = (ref_s - s_i) / (s_{i+1} - s_i)
  double r = (ref_s - frenet_path_.at(index).s()) /
             (frenet_path_.at(index + 1).s() - frenet_path_.at(index).s());

  // 根据相对位置计算在离散化路径上的s值
  const double discretized_path_s = discretized_path_.at(index).s() +
                                    r * (discretized_path_.at(index + 1).s() -
                                         discretized_path_.at(index).s());

  // 使用计算得到的s值在离散化路径上插值获取路径点
  path_point->CopyFrom(discretized_path_.Evaluate(discretized_path_s));

  return true;
}

/**
 * @brief 清空所有路径数据
 *
 * 功能说明：
 * 重置PathData的所有成员变量
 * 包括清空路径、决策指南和路径参考
 *
 * C++语法说明：
 * - .clear()：
 *   容器的清空方法
 *   移除所有元素，但不影响容器容量
 *
 * - reference_line_ = nullptr：
 *   将指针设为空
 *   表示当前没有有效的参考线
 */
void PathData::Clear() {
  discretized_path_.clear();              // 清空笛卡尔路径
  frenet_path_.clear();                   // 清空Frenet路径
  path_point_decision_guide_.clear();    // 清空决策指南
  path_reference_.clear();               // 清空路径参考
  reference_line_ = nullptr;             // 重置参考线指针
}

/**
 * @brief 生成调试字符串
 *
 * @return std::string 格式化的调试字符串
 *
 * 功能说明：
 * 生成PathData的调试表示字符串
 * 包含笛卡尔路径的点信息
 *
 * 算法流程：
 * 1. 限制输出点的数量（不超过FLAGS_trajectory_point_num_for_debug）
 * 2. 使用absl::StrJoin将点连接成字符串
 *
 * C++语法说明：
 * - absl::StrCat：
 *   Google Abseil库的字符串连接函数
 *   高效地连接多个字符串
 *
 * - absl::StrJoin：
 *   将容器中的元素用分隔符连接成字符串
 *   参数：容器起始迭代器，结束迭代器，分隔符，格式化器
 *
 * - std::min(a, static_cast<size_t>(b))：
 *   计算较小值
 *   使用static_cast进行显式类型转换
 *
 * - apollo::common::util::DebugStringFormatter()：
 *   Apollo提供的调试字符串格式化器
 */
std::string PathData::DebugString() const {
  // 限制输出点的数量，避免日志过长
  const auto limit =
      std::min(discretized_path_.size(),
               static_cast<size_t>(FLAGS_trajectory_point_num_for_debug));

  // 使用Abseil库生成格式化的调试字符串
  return absl::StrCat(
      "[\n",  // 字符串开头
      // 将路径点连接成字符串，每个点后换行，缩进
      absl::StrJoin(discretized_path_.begin(),
                    discretized_path_.begin() + limit, ",\n",
                    apollo::common::util::DebugStringFormatter()),
      "]\n");  // 字符串结尾
}

/**
 * @brief 将Frenet路径转换为笛卡尔路径
 *
 * @param frenet_path 输入的Frenet路径
 * @param discretized_path 输出：转换后的笛卡尔路径
 * @return bool 转换是否成功
 *
 * 功能说明：
 * 将Frenet坐标系的路径转换为笛卡尔坐标系的路径
 * 这是规划模块中的核心坐标转换函数之一
 *
 * 转换原理：
 * 对于Frenet坐标系中的每个点(s, l)：
 * 1. 在参考线上找到对应的s点
 * 2. 计算该点的切线方向（heading）
 * 3. 沿切线方向移动s距离，垂直方向移动l距离
 * 4. 得到笛卡尔坐标(x, y)
 *
 * 算法流程：
 * 1. 遍历Frenet路径的每个点
 * 2. 将SL点转换为XY点
 * 3. 计算每个点的朝向(theta)和曲率(kappa)
 * 4. 累加计算路径长度s和曲率变化率dkappa
 * 5. 构建DiscretizedPath
 *
 * C++语法说明：
 * - for (const common::FrenetFramePoint &frenet_point : frenet_path)：
 *   范围for循环遍历Frenet路径
 *   const &避免拷贝，提高效率
 *
 * - PointFactory::ToSLPoint：
 *   创建SL点对象的工厂方法
 *
 * - reference_line_->SLToXY：
 *   参考线的SL转XY方法
 *   使用指针调用，因为reference_line_是指针类型
 *
 * - CartesianFrenetConverter::CalculateTheta：
 *   静态方法，计算Frenet点的朝向角
 *   根据参考点朝向、曲率和横向位置计算
 *
 * - CartesianFrenetConverter::CalculateKappa：
 *   静态方法，计算Frenet点的曲率
 *   根据参考点曲率、曲率导数、横向位置及其导数计算
 *
 * - Vec2d::Length()：
 *   计算2D向量的长度（欧几里得距离）
 *
 * - PointFactory::ToPathPoint：
 *   创建完整PathPoint的工厂方法
 *   参数：x, y, z, s, theta, kappa, dkappa
 *
 * - std::move(path_points)：
 *   移动语义，将临时vector的内容转移给DiscretizedPath
 *   避免拷贝整个向量
 */
bool PathData::SLToXY(const FrenetFramePath &frenet_path,
                      DiscretizedPath *const discretized_path) {
  // 创建路径点向量
  std::vector<common::PathPoint> path_points;

  // 遍历Frenet路径的每个点
  for (const common::FrenetFramePoint &frenet_point : frenet_path) {
    // 第一步：创建SL点并转换为XY坐标
    const common::SLPoint sl_point =
        PointFactory::ToSLPoint(frenet_point.s(), frenet_point.l());
    common::math::Vec2d cartesian_point;
    if (!reference_line_->SLToXY(sl_point, &cartesian_point)) {
      AERROR << "Fail to convert sl point to xy point";
      return false;
    }

    // 第二步：获取对应参考点，用于计算朝向和曲率
    const ReferencePoint ref_point =
        reference_line_->GetReferencePoint(frenet_point.s());

    // 第三步：计算该Frenet点的朝向角
    // 公式：theta = ref_heading + atan2(dl, ds)
    // 其中dl是横向位移的一阶导数
    const double theta = CartesianFrenetConverter::CalculateTheta(
        ref_point.heading(), ref_point.kappa(), frenet_point.l(),
        frenet_point.dl());

    ADEBUG << "frenet_point: " << frenet_point.ShortDebugString();

    // 第四步：计算该Frenet点的曲率
    // 考虑了横向位置及其导数对曲率的影响
    const double kappa = CartesianFrenetConverter::CalculateKappa(
        ref_point.kappa(), ref_point.dkappa(), frenet_point.l(),
        frenet_point.dl(), frenet_point.ddl());

    // 第五步：计算路径长度s和曲率变化率dkappa
    double s = 0.0;      // 初始化路径长度
    double dkappa = 0.0;  // 初始化曲率变化率
    if (!path_points.empty()) {
      // 如果不是第一个点，计算与前一个点的距离
      common::math::Vec2d last = PointFactory::ToVec2d(path_points.back());
      const double distance = (last - cartesian_point).Length();
      // 累加得到当前点的路径长度
      s = path_points.back().s() + distance;
      // 计算曲率变化率
      dkappa = (kappa - path_points.back().kappa()) / distance;
    }

    // 第六步：创建路径点并添加到容器
    path_points.push_back(PointFactory::ToPathPoint(cartesian_point.x(),
                                                    cartesian_point.y(), 0.0, s,
                                                    theta, kappa, dkappa));
  }

  // 使用移动语义将路径点转移给输出的DiscretizedPath
  *discretized_path = DiscretizedPath(std::move(path_points));

  return true;
}

/**
 * @brief 将笛卡尔路径转换为Frenet路径
 *
 * @param discretized_path 输入的笛卡尔路径
 * @param frenet_path 输出：转换后的Frenet路径
 * @return bool 转换是否成功
 *
 * 功能说明：
 * 将笛卡尔坐标系的路径转换为Frenet坐标系的路径
 * 这是SLToXY的逆过程
 *
 * 转换原理：
 * 对于笛卡尔坐标系中的每个点(x, y)：
 * 1. 将(x, y)投影到参考线上
 * 2. 计算投影点沿参考线的距离s
 * 3. 计算到参考线的垂直距离l（有正负）
 * 4. 得到Frenet坐标(s, l)
 *
 * 算法流程：
 * 1. 遍历笛卡尔路径的每个点
 * 2. 将XY点转换为SL点
 * 3. 限制s值在[0, max_len]范围内
 * 4. 构建FrenetFramePath
 *
 * C++语法说明：
 * - ACHECK(reference_line_)：
 *   断言参考线已设置
 *
 * - reference_line_->GetFrenetPoint：
 *   获取笛卡尔点对应的Frenet点
 *   如果点在参考线上，则直接获取
 *
 * - if (!frenet_point.has_s())：
 *   检查Frenet点是否有有效的s值
 *   has_s()是Protobuf消息的方法，检查字段是否被设置
 *
 * - reference_line_->XYToSL：
 *   参考线的XY转SL方法
 *   将笛卡尔点转换为SL点
 *
 * - std::max(0.0, std::min(sl_point.s(), max_len))：
 *   将s值限制在[0, max_len]范围内
 *   防止越界
 *
 * - std::move(frenet_point)：
 *   移动语义，将临时对象转移到向量中
 */
bool PathData::XYToSL(const DiscretizedPath &discretized_path,
                      FrenetFramePath *const frenet_path) {
  ACHECK(reference_line_);  // 断言参考线已设置
  std::vector<common::FrenetFramePoint> frenet_frame_points;  // 创建Frenet点向量
  const double max_len = reference_line_->Length();  // 获取参考线长度

  // 遍历笛卡尔路径的每个点
  for (const auto &path_point : discretized_path) {
    // 尝试获取该点对应的Frenet点
    common::FrenetFramePoint frenet_point =
        reference_line_->GetFrenetPoint(path_point);

    // 如果没有有效的s值，需要进行XY到SL的转换
    if (!frenet_point.has_s()) {
      SLPoint sl_point;
      if (!reference_line_->XYToSL(path_point, &sl_point)) {
        AERROR << "Fail to transfer cartesian point to frenet point.";
        return false;
      }
      common::FrenetFramePoint frenet_point;
      // 注意：这里没有设置dl和ddl，如有需要可以添加
      frenet_point.set_s(std::max(0.0, std::min(sl_point.s(), max_len)));
      frenet_point.set_l(sl_point.l());
      frenet_frame_points.push_back(std::move(frenet_point));
      continue;
    }

    // 限制s值在有效范围内
    frenet_point.set_s(std::max(0.0, std::min(frenet_point.s(), max_len)));
    frenet_frame_points.push_back(std::move(frenet_point));
  }

  // 使用移动语义将Frenet点转移到输出的FrenetFramePath
  *frenet_path = FrenetFramePath(std::move(frenet_frame_points));
  return true;
}

/**
 * @brief 从指定Frenet点开始裁剪路径的左侧
 *
 * @param frenet_point 裁剪起始点的Frenet坐标
 * @return bool 裁剪是否成功
 *
 * 功能说明：
 * 保留从指定Frenet点开始的路径部分
 * 用于在轨迹拼接时去除已行驶的部分
 *
 * 算法流程：
 * 1. 从指定点开始创建新的Frenet路径
 * 2. 跳过s值与指定点非常接近的点（1e-6容差）
 * 3. 添加所有s值大于指定点的点
 * 4. 调用SetFrenetPath设置新的Frenet路径
 *
 * C++语法说明：
 * - std::fabs(fp.s() - frenet_point.s()) < 1e-6：
 *   fabs：浮点数绝对值函数
 *   用于判断两个浮点数是否"相等"（在容差范围内）
 *
 * - frenet_frame_points.emplace_back(frenet_point)：
 *   emplace_back：在向量末尾直接构造元素
 *   相比push_back，避免了拷贝或移动操作
 */
bool PathData::LeftTrimWithRefS(const common::FrenetFramePoint &frenet_point) {
  ACHECK(reference_line_);  // 断言参考线已设置
  std::vector<common::FrenetFramePoint> frenet_frame_points;
  frenet_frame_points.emplace_back(frenet_point);  // 添加起始点

  // 遍历现有Frenet路径
  for (const common::FrenetFramePoint fp : frenet_path_) {
    // 跳过与起始点非常接近的点（避免重复）
    if (std::fabs(fp.s() - frenet_point.s()) < 1e-6) {
      continue;
    }
    // 添加所有s值大于起始点的点
    if (fp.s() > frenet_point.s()) {
      frenet_frame_points.push_back(fp);
    }
  }

  // 使用新的Frenet路径设置PathData
  SetFrenetPath(FrenetFramePath(std::move(frenet_frame_points)));
  return true;
}

/**
 * @brief 更新Frenet路径的参考线
 *
 * @param reference_line 新的参考线指针
 * @return bool 更新是否成功
 *
 * 功能说明：
 * 当参考线发生变化时，更新PathData的Frenet路径
 * 会重新计算笛卡尔路径
 *
 * C++语法说明：
 * - reference_line_ = reference_line：
 *   更新成员变量指向新的参考线
 *
 * - SetDiscretizedPath(discretized_path_)：
 *   使用现有的笛卡尔路径重新设置
 *   这会触发XYToSL转换，更新Frenet路径
 */
bool PathData::UpdateFrenetFramePath(const ReferenceLine *reference_line) {
  reference_line_ = reference_line;
  return SetDiscretizedPath(discretized_path_);
}

/**
 * @brief 设置路径标签
 *
 * @param label 路径标签字符串
 *
 * C++语法说明：
 * - set_xxx命名约定：
 *   Apollo中setter方法的命名规范
 *   与getter配对使用
 */
void PathData::set_path_label(const std::string &label) { path_label_ = label; }

/**
 * @brief 获取路径标签
 *
 * @return const std::string& 路径标签的常量引用
 *
 * C++语法说明：
 * - const std::string&：
 *   返回常量引用，避免拷贝
 *   返回const防止外部修改
 */
const std::string &PathData::path_label() const { return path_label_; }

/**
 * @brief 获取路径参考
 *
 * @return const vector<PathPoint>& 路径参考点的引用
 *
 * 功能说明：
 * 返回学习模型输出的路径参考
 * 用于基于学习的规划方法
 */
const std::vector<PathPoint> &PathData::path_reference() const {
  return path_reference_;
}

/**
 * @brief 设置路径参考
 *
 * @param path_reference 路径参考点向量
 *
 * C++语法说明：
 * - std::move(path_reference)：
 *   移动语义，将输入参数转移到成员变量
 *   移动后path_reference变为空
 */
void PathData::set_path_reference(
    const std::vector<PathPoint> &path_reference) {
  path_reference_ = std::move(path_reference);
}

/**
 * @brief Apollo命名空间结束
 *
 * C++语法说明：
 * - }  // namespace planning：
 *   结束planning命名空间
 *
 * - }  // namespace apollo：
 *   结束apollo命名空间
 */
}  // namespace planning
}  // namespace apollo