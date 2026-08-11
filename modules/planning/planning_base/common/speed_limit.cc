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
 * @file speed_limit.cc
 * @brief 速度限制实现文件
 *
 * 功能说明：
 * 实现了速度限制(SpeedLimit)类，用于管理沿路径的速度限制约束
 * 速度限制以(s, v)点对的形式存储，表示在路径位置s处的最大速度v
 *
 * 核心概念：
 * - Speed Limit：速度限制，规定车辆在特定位置的最大允许速度
 * - s coordinate：沿路径的距离坐标
 * - v value：该位置的速度上限
 * - 插值：对于任意s位置，通过二分查找获取最近的速度限制
 *
 * 数据结构：
 * - speed_limit_points_：存储(s, v)点对的向量，按s升序排列
 * - 二分查找：O(log n)时间复杂度查找任意s处的速度限制
 *
 * C++语法说明：
 * - std::vector<std::pair<double, double>>：向量容器，存储点对
 * - std::lower_bound：二分查找下界算法
 * - lambda表达式：匿名函数用于自定义比较
 * - emplace_back：原地构造元素，避免拷贝
 **/

#include "modules/planning/planning_base/common/speed_limit.h"
/**
 * @brief 速度限制头文件
 *
 * 包含SpeedLimit类的定义
 */

#include <algorithm>
/**
 * @brief 标准算法库头文件
 *
 * 提供常用算法：
 * - std::lower_bound：二分查找下界
 * - std::max/min：极值函数
 */

#include "cyber/common/log.h"
/**
 * @brief Cyber RT日志头文件
 *
 * 提供日志宏：
 * - ADEBUG：调试日志
 * - AERROR：错误日志
 * - DCHECK_GE：调试断言（大于等于）
 */

namespace apollo {
/**
 * @brief Apollo项目主命名空间
 */

namespace planning {
/**
 * @brief 规划模块命名空间
 */

/**
 * @brief 添加速度限制点
 *
 * @param s 沿路径的位置坐标（米）
 * @param v 该位置的最大速度限制（米/秒）
 *
 * 功能说明：
 * 向速度限制列表添加一个新的(s, v)点对
 * 这些点按s坐标升序存储，用于后续二分查找
 *
 * 算法流程：
 * 1. 检查是否非空
 * 2. 如果非空，验证s是否大于等于最后一个点的s（保证升序）
 * 3. 使用emplace_back原地构造点对
 *
 * C++语法说明：
 * - const double s：常量引用参数，避免拷贝
 * - speed_limit_points_.empty()：检查向量是否为空
 * - speed_limit_points_.back()：获取向量最后一个元素的引用
 * - .first：获取pair的第一个元素（s坐标）
 * - speed_limit_points_.emplace_back(s, v)：
 *   在向量末尾原地构造新元素
 *   相比push_back，避免拷贝构造
 * - DCHECK_GE：调试断言宏，要求s >= 上一个点的s
 */
void SpeedLimit::AppendSpeedLimit(const double s, const double v) {
  if (!speed_limit_points_.empty()) {
    /**
     * @brief 检查列表非空
     *
     * 如果非空，验证s坐标是否递增
     */
    DCHECK_GE(s, speed_limit_points_.back().first);
    /**
     * @brief 调试断言：s必须 >= 最后一个点的s
     *
     * DCHECK_GE(a, b)：
     * - 调试模式下，如果a < b，程序终止
     * - 发布模式下不进行检查
     * - 用于捕获编程错误
     */
  }
  speed_limit_points_.emplace_back(s, v);
  /**
   * @brief 添加速度限制点
   *
   * emplace_back优点：
   * - 直接在向量内存中构造对象
   * - 避免复制或移动临时对象
   * - 更高效的插入操作
   *
   * pair<double, double>构造：
   * - 第一个值：s（位置）
   * - 第二个值：v（速度限制）
   */
}

/**
 * @brief 获取所有速度限制点
 *
 * @return const std::vector<std::pair<double, double>>& 速度限制点列表的常量引用
 *
 * 功能说明：
 * 提供对速度限制点列表的只读访问
 * 返回常量引用，避免拷贝开销
 *
 * C++语法说明：
 * - const std::vector<...>&：
 *   返回常量引用，只读访问
 *   不触发拷贝，提高效率
 */
const std::vector<std::pair<double, double>>& SpeedLimit::speed_limit_points()
    const {
  return speed_limit_points_;
  /**
   * @brief 返回成员变量的引用
   *
   * 注意：返回的是引用而非拷贝
   * 调用者只能读取，不能修改
   */
}

/**
 * @brief 根据s坐标获取速度限制
 *
 * @param s 沿路径的位置坐标（米）
 * @return double 该位置的速度限制（米/秒）
 *
 * 功能说明：
 * 使用二分查找找到给定s位置的速度限制
 * 如果s恰好等于某个点的s，返回该点的速度
 * 如果s在两点之间，返回前一个点的速度（阶梯函数）
 *
 * 算法流程：
 * 1. 验证数据有效性（至少2个点，s在有效范围内）
 * 2. 使用std::lower_bound二分查找
 * 3. 处理边界情况（s超出范围）
 * 4. 返回找到的速度限制值
 *
 * C++语法说明：
 * - std::lower_bound：二分查找，返回第一个不小于目标值的迭代器
 * - lambda表达式：[](const pair&, double) { return point.first < s; }
 * - auto it_lower：自动类型推导迭代器
 * - it_lower->second：访问pair的第二个元素（速度值）
 */
double SpeedLimit::GetSpeedLimitByS(const double s) const {
  CHECK_GE(speed_limit_points_.size(), 2U);
  /**
   * @brief 运行时断言：至少需要2个点
   *
   * CHECK_GE(a, b)：
   * - 如果a < b，程序终止并输出错误
   * - 用于运行时错误检测
   * - size()返回size_t，是无符号类型
   */
  DCHECK_GE(s, speed_limit_points_.front().first);
  /**
   * @brief 调试断言：s必须 >= 第一个点的s
   *
   * speed_limit_points_.front()：获取向量第一个元素
   * .first：获取pair的第一个元素
   */

  /**
   * @brief 定义比较函数（lambda表达式）
   *
   * lambda语法：
   * [capture](parameters) { body }
   * - capture：捕获列表（此处为空）
   * - parameters：函数参数
   * - body：函数体
   *
   * 功能：比较pair的first与s的大小
   * 用于std::lower_bound的自定义比较
   */
  auto compare_s = [](const std::pair<double, double>& point, const double s) {
    return point.first < s;
    /**
     * @brief 如果point.first < s，返回true
     * 这是std::lower_bound需要的比较函数语义
     */
  };

  /**
   * @brief 二分查找下界
   *
   * std::lower_bound算法：
   * - 在有序范围内查找第一个"不小于"s的元素
   * - 返回指向该元素的迭代器
   *
   * 参数说明：
   * - speed_limit_points_.begin()：范围起始
   * - speed_limit_points_.end()：范围结束
   * - s：要查找的值
   * - compare_s：自定义比较函数
   *
   * 返回值：
   * - 如果找到：指向第一个s' >= s的迭代器
   * - 如果没找到：end()迭代器
   */
  auto it_lower = std::lower_bound(speed_limit_points_.begin(),
                                   speed_limit_points_.end(), s, compare_s);

  /**
   * @brief 处理s超出最大范围的情况
   *
   * 如果lower_bound返回end()，
   * 说明s大于所有点的s，返回最后一个点的速度
   */
  if (it_lower == speed_limit_points_.end()) {
    return (it_lower - 1)->second;
    /**
     * @brief 返回最后一个点的速度
     *
     * it_lower - 1：获取最后一个元素
     * ->second：获取pair的第二个元素（速度值）
     */
  }
  return it_lower->second;
  /**
   * @brief 返回找到的速度限制
   *
   * it_lower指向第一个s' >= s的点
   * 返回该点的速度限制
   */
}

/**
 * @brief 清除所有速度限制点
 *
 * 功能说明：
 * 清空速度限制列表
 * 通常在重新规划时调用，重置状态
 *
 * C++语法说明：
 * - speed_limit_points_.clear()：
 *   清空向量，释放内存
 *   所有元素被析构
 */
void SpeedLimit::Clear() {
  speed_limit_points_.clear();
  /**
   * @brief 清除所有点
   *
   * clear()成员函数：
   * - 删除所有元素
   * - size()变为0
   * - capacity()保持不变（可能）
   */
}

}  // namespace planning
/**
 * @brief 命名空间结束标记
 */
}  // namespace apollo
/**
 * @brief Apollo命名空间结束标记
 */
