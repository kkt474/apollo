/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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
 * @file print_debug_info.cc
 * @brief 调试信息打印工具类实现文件
 *
 * 功能说明：
 * 实现将规划数据打印到日志系统的功能
 * 提供点集、曲线、边界框的日志输出支持
 */

#include "modules/planning/planning_base/common/util/print_debug_info.h"

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {

/**
 * @brief Apollo外层命名空间
 */
namespace planning {

/**
 * @brief 使用别名简化Vec2d类型引用
 *
 * C++语法说明：
 * - using：类型别名声明
 * - apollo::common::math::Vec2d：
 *   Apollo数学库中的二维向量类
 *   这里定义别名后可直接使用Vec2d
 */
using apollo::common::math::Vec2d;

/**
 * @brief 设置点集ID的实现
 *
 * @param id 要设置的标识符字符串
 *
 * C++语法说明：
 * - void PrintPoints::set_id：
 *   类外成员函数定义
 *   :: 是作用域运算符，表示这是PrintPoints类的成员函数
 * - id_ = id：
 *   直接赋值给成员变量
 */
void PrintPoints::set_id(std::string id) { id_ = id; }

/**
 * @brief 向点集添加点的实现
 *
 * @param x 点的X坐标
 * @param y 点的Y坐标
 *
 * C++语法说明：
 * - points.emplace_back(x, y)：
 *   vector的emplace_back方法
 *   - 与push_back的区别：
 *     push_back需要先创建pair，再拷贝或移动
 *     emplace_back直接在容器中构造对象，避免拷贝
 *   - 参数直接传递给pair的构造函数
 *   - 这里使用了C++11的容器插入优化
 */
void PrintPoints::AddPoint(double x, double y) { points.emplace_back(x, y); }

/**
 * @brief 将点集打印到日志的实现
 *
 * 功能说明：
 * 检查全局开关FLAGS_enable_print_curve
 * 如果启用，则将所有点格式化输出到日志
 *
 * C++语法说明：
 * - if (!FLAGS_enable_print_curve)：
 *   FLAGS是gflags库的变量前缀
 *   enable_print_curve是控制是否打印曲线的全局开关
 * - std::stringstream：
 *   字符串流，用于格式化字符串
 *   类似ostream，但写入到字符串而不是文件或cout
 * - std::fixed：
 *   浮点数输出格式标志
 *   使用固定小数点表示法，而非科学计数法
 * - AINFO：
 *   Apollo信息级别日志宏
 */
void PrintPoints::PrintToLog() {
  // 检查全局开关，如果禁用则直接返回
  if (!FLAGS_enable_print_curve) {
    return;
  }

  // 创建字符串流用于格式化输出
  std::stringstream ssm;

  // 格式：print_id: (x1, y1); (x2, y2); ...
  ssm << "print_" << id_ << ":";

  // 遍历所有点，格式化输出
  for (size_t i = 0; i < points.size(); i++) {
    ssm << std::fixed               // 固定小数格式
        << "(" << points[i].first   // x坐标
        << ", " << points[i].second  // y坐标
        << ");";
  }

  // 输出到日志
  AINFO << ssm.str();
}

/**
 * @brief 向指定曲线添加点（x,y坐标版本）的实现
 *
 * @param key 曲线名称
 * @param x 点的X坐标
 * @param y 点的Y坐标
 *
 * 算法流程：
 * 1. 检查全局开关
 * 2. 如果曲线不存在，创建新的PrintPoints
 * 3. 添加点到曲线
 *
 * C++语法说明：
 * - curve_map_.count(key) == 0：
 *   map的count方法返回key存在的个数（0或1）
 *   用于检查key是否存在
 * - curve_map_[key] = PrintPoints(key)：
 *   如果key不存在，会默认构造一个PrintPoints
 *   然后用key构造的临时对象赋值给它
 *   这会调用默认构造函数和赋值运算符
 */
void PrintCurves::AddPoint(std::string key, double x, double y) {
  // 检查全局开关
  if (!FLAGS_enable_print_curve) {
    return;
  }

  // 检查曲线是否已存在
  if (curve_map_.count(key) == 0) {
    // 曲线不存在，创建新的PrintPoints并用key作为ID
    curve_map_[key] = PrintPoints(key);
  }

  // 添加点到指定曲线
  curve_map_[key].AddPoint(x, y);
}

/**
 * @brief 向指定曲线添加点（Vec2d版本）的实现
 *
 * @param key 曲线名称
 * @param point Vec2d类型的点对象
 *
 * C++语法说明：
 * - const apollo::common::math::Vec2d& point：
 *   Vec2d对象的常量引用
 *   Vec2d是Apollo定义的二维向量类
 *   包含x()和y()方法获取坐标
 * - point.x()：
 *   Vec2d的x坐标访问方法
 *   注意：x()是方法，不是成员变量
 */
void PrintCurves::AddPoint(std::string key,
                          const apollo::common::math::Vec2d& point) {
  // 检查全局开关
  if (!FLAGS_enable_print_curve) {
    return;
  }

  // 检查曲线是否已存在
  if (curve_map_.count(key) == 0) {
    curve_map_[key] = PrintPoints(key);
  }

  // 添加点，使用Vec2d的x()和y()方法获取坐标
  curve_map_[key].AddPoint(point.x(), point.y());
}

/**
 * @brief 向指定曲线添加多个点（vector版本）的实现
 *
 * @param key 曲线名称
 * @param points Vec2d点的向量
 *
 * C++语法说明：
 * - const std::vector<common::math::Vec2d>& points：
 *   Vec2d对象的常量引用向量
 *   const保证输入不会被修改
 * - for (const auto& point : points)：
 *   范围for循环
 *   - const：循环变量不可修改
 *   - auto：自动推导类型
 *   - &：引用，避免拷贝
 */
void PrintCurves::AddPoint(std::string key,
                          const std::vector<common::math::Vec2d>& points) {
  // 检查全局开关
  if (!FLAGS_enable_print_curve) {
    return;
  }

  // 遍历每个点，递归调用单个点的AddPoint
  for (const auto& point : points) {
    AddPoint(key, point);
  }
}

/**
 * @brief 将所有曲线打印到日志的实现
 *
 * C++语法说明：
 * - for (auto iter = curve_map_.begin(); iter != curve_map_.end(); iter++)：
 *   map的迭代器遍历
 *   - begin()：返回指向第一个元素的迭代器
 *   - end()：返回指向末尾的迭代器（实际是最后一个元素之后）
 *   - iter != curve_map_.end()：迭代器比较
 * - iter->second：
 *   迭代器的箭头运算符
 *   iter是指向pair的指针
 *   ->second访问pair的第二个元素（PrintPoints对象）
 */
void PrintCurves::PrintToLog() {
  // 检查全局开关
  if (!FLAGS_enable_print_curve) {
    return;
  }

  // 遍历所有曲线
  for (auto iter = curve_map_.begin(); iter != curve_map_.end(); iter++) {
    // 调用每个PrintPoints的PrintToLog方法
    // iter->first是曲线名称（string）
    // iter->second是曲线点集（PrintPoints对象）
    iter->second.PrintToLog();
  }
}

/**
 * @brief 添加自车边界框的实现
 *
 * @param x 车辆位置X坐标
 * @param y 车辆位置Y坐标
 * @param heading 车辆朝向角（弧度）
 * @param is_rear_axle_point 是否以后轴中心为参考点
 *
 * 功能说明：
 * 将车辆边界框信息添加到打印列表
 * 如果输入是后轴中心点，会转换为后轴中心到车辆中心的偏移
 *
 * C++语法说明：
 * - apollo::planning::PrintBox::AddAdcBox：
 *   类外定义，使用完整命名空间限定
 * - const auto& vehicle_param：
 *   const引用，避免拷贝
 *   auto自动推导类型
 * - apollo::common::VehicleConfigHelper::GetConfig()：
 *   车辆配置辅助类的单例获取方法
 *   .vehicle_param()获取车辆参数
 * - vehicle_param.front_edge_to_center()：
 *   车辆前边缘到中心的长度
 * - vehicle_param.length()：
 *   车辆总长度
 * - rear_axle_to_center：
 *   计算后轴中心到车辆中心的偏移量
 *   = 前边缘到中心的距离 - 车辆长度的一半
 */
void apollo::planning::PrintBox::AddAdcBox(double x, double y, double heading,
                                          bool is_rear_axle_point) {
  // 获取车辆配置参数
  const auto& vehicle_param =
      apollo::common::VehicleConfigHelper::GetConfig().vehicle_param();

  // 如果输入是后轴中心点，需要转换为车辆中心点
  if (is_rear_axle_point) {
    // rear center情况
    // 计算从后轴中心到车辆中心的偏移量
    // 这个偏移量用于将后轴中心坐标转换为车辆几何中心坐标
    double rear_axle_to_center =
        vehicle_param.front_edge_to_center() - vehicle_param.length() / 2.0;

    // 根据朝向角计算偏移量的x和y分量
    // cos(heading) * offset = x方向偏移
    // sin(heading) * offset = y方向偏移
    x += rear_axle_to_center * cos(heading);
    y += rear_axle_to_center * sin(heading);
  }

  // 将边界框信息存储到容器
  // 格式：{x, y, heading, length, width}
  // x, y: 车辆中心位置
  // heading: 车辆朝向角
  // length: 车辆长度
  // width: 车辆宽度
  box_points.push_back(
      {x, y, heading, vehicle_param.length(), vehicle_param.width()});
}

/**
 * @brief 将边界框打印到日志的实现
 *
 * C++语法说明：
 * - std::stringstream：
 *   字符串流，用于格式化复杂的字符串输出
 * - std::fixed：
 *   固定小数格式输出
 * - box_points[i][j]：
 *   二维vector的访问方式
 *   外层[i]访问第i个边界框
 *   内层[j]访问边界框的第j个元素
 * - AINFO：
 *   Apollo信息日志宏
 */
void apollo::planning::PrintBox::PrintToLog() {
  // 创建字符串流
  std::stringstream ssm;
  ssm << "print_" << id_ << ":";

  // 遍历所有边界框
  for (size_t i = 0; i < box_points.size(); i++) {
    ssm << "(";

    // 遍历边界框的每个元素
    for (size_t j = 0; j < box_points[i].size(); j++) {
      // 输出当前元素，使用固定小数格式
      ssm << std::fixed << box_points[i][j];

      // 如果不是最后一个元素，添加逗号分隔
      if (j != box_points[i].size() - 1) {
        ssm << ", ";
      }
    }

    ssm << ")";

    // 如果不是最后一个边界框，添加逗号分隔
    if (i != box_points.size() - 1) {
      ssm << ", ";
    }
  }

  // 输出到日志
  AINFO << ssm.str();
}

}  // namespace planning
}  // namespace apollo
