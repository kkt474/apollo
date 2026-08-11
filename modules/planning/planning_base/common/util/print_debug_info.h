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
 * @file print_debug_info.h
 * @brief 调试信息打印工具类头文件
 *
 * 功能说明：
 * 提供将规划数据（点集、曲线、边界框）打印到日志系统的工具类
 * 用于调试和可视化规划结果
 */

#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "cyber/common/log.h"
#include "modules/common/math/vec2d.h"

namespace apollo {
/**
 * @brief Apollo最外层命名空间
 */
namespace planning {

/**
 * @class PrintPoints
 * @brief 点集打印类
 *
 * 功能说明：
 * 存储和管理一组二维点，并提供日志打印功能
 * 用于调试时记录路径点、障碍物位置等数据
 *
 * 设计模式：
 * - 简单数据容器类
 * - 支持链式调用（通过返回引用）
 *
 * C++语法说明：
 * - class：类声明
 * - public：公有成员访问限定符
 * - std::vector<std::pair<double, double>>：
 *   存储点集的容器，每个点是(x,y)坐标对
 */
class PrintPoints {
 public:
  /**
   * @brief 默认构造函数
   *
   * 功能说明：
   * 创建一个空的点集打印对象
   * 不设置ID
   */
  PrintPoints() {}

  /**
   * @brief 带ID的构造函数
   *
   * @param id 点集的标识符字符串
   *
   * 功能说明：
   * 创建一个带有指定ID的点集打印对象
   * 使用explicit关键字防止隐式类型转换
   *
   * C++语法说明：
   * - explicit：
   *   防止单参数构造函数的隐式调用
   *   PrintPoints p = "curve1" 这样的代码会编译错误
   *   必须显式调用 PrintPoints p("curve1")
   * - : id_(id)：
   *   构造函数初始化列表
   *   在构造函数体执行前初始化成员变量
   *   这是初始化const成员或引用成员的推荐方式
   *
   * 示例：
   * @code
   *   PrintPoints path_points("ego_path");  // 创建一个ID为"ego_path"的点集
   * @endcode
   */
  explicit PrintPoints(std::string id) : id_(id) {}

  /**
   * @brief 设置点集ID
   *
   * @param id 要设置的标识符
   *
   * 功能说明：
   * 为点集设置或更新标识符
   *
   * C++语法说明：
   * - void：函数没有返回值
   * - std::string：标准库字符串类型
   */
  void set_id(std::string id);

  /**
   * @brief 向点集添加一个点
   *
   * @param x 点的X坐标
   * @param y 点的Y坐标
   *
   * 功能说明：
   * 将一个二维点添加到点集末尾
   *
   * 算法流程：
   * 1. 创建std::pair<double, double>表示点
   * 2. 调用vector::push_back添加到容器末尾
   *
   * C++语法说明：
   * - std::pair<double, double>：
   *   标准库模板类，表示两个元素的元组
   *   first是x坐标，second是y坐标
   * - std::make_pair：
   *   创建pair的辅助函数
   *
   * 示例：
   * @code
   *   PrintPoints curve;
   *   curve.AddPoint(0.0, 0.0);  // 添加点(0, 0)
   *   curve.AddPoint(1.0, 2.0);  // 添加点(1, 2)
   * @endcode
   */
  void AddPoint(double x, double y);

  /**
   * @brief 将点集打印到日志
   *
   * 功能说明：
   * 将存储的所有点打印到日志系统
   * 便于调试和可视化分析
   *
   * C++语法说明：
   * - void：函数没有返回值
   * - AWARN：
   *   Apollo警告日志宏
   *   用于输出调试信息
   */
  void PrintToLog();

 private:
  /**
   * @brief 点集标识符
   *
   * std::string：标准库字符串类型
   * 用于标识这个点集，方便日志中区分不同数据
   */
  std::string id_;

  /**
   * @brief 存储点的容器
   *
   * std::vector<std::pair<double, double>>：
   * - std::vector：动态数组容器
   * - std::pair<double, double>：每个点是一个坐标对
   *
   * C++语法说明：
   * - std::vector：
   *   顺序容器，支持随机访问
   *   动态大小，可在运行时添加元素
   * - private：
   *   私有成员访问限定符
   *   只有类内部可以访问，外部代码不能直接访问
   */
  std::vector<std::pair<double, double>> points;
};

/**
 * @class PrintCurves
 * @brief 多曲线打印类
 *
 * 功能说明：
 * 管理多条命名的曲线（每条曲线是一个点集）
 * 支持按key添加点到不同曲线，并统一打印
 *
 * 设计模式：
 * - 组管理器模式：管理多个相关对象
 * - Key-Value存储：使用map管理命名曲线
 *
 * C++语法说明：
 * - std::map<std::string, PrintPoints>：
 *   关联容器，按key（字符串）索引PrintPoints对象
 *   - map自动按key排序
 *   - 支持高效的查找和插入
 */
class PrintCurves {
 public:
  /**
   * @brief 向指定曲线添加点（使用x,y坐标）
   *
   * @param key 曲线名称
   * @param x 点的X坐标
   * @param y 点的Y坐标
   *
   * 功能说明：
   * 如果指定名称的曲线不存在，会自动创建
   * 如果已存在，会在现有点集末尾添加新点
   *
   * 算法流程：
   * 1. 在curve_map_中查找key对应的曲线
   * 2. 如果不存在，使用默认构造函数创建新的PrintPoints
   * 3. 调用曲线的AddPoint方法添加点
   *
   * C++语法说明：
   * - std::map::operator[]：
   *   如果key存在，返回对应的value引用
   *   如果key不存在，默认构造一个value并插入
   *   这里利用这个特性自动创建不存在的曲线
   */
  void AddPoint(std::string key, double x, double y);

  /**
   * @brief 向指定曲线添加点（使用Vec2d对象）
   *
   * @param key 曲线名称
   * @param point apollo的Vec2d二维向量对象
   *
   * 功能说明：
   * 重载版本，接受Vec2d类型的点对象
   * Vec2d是Apollo定义的二维向量类，提供更丰富的向量运算
   *
   * C++语法说明：
   * - const common::math::Vec2d&：
   *   Vec2d对象的常量引用
   *   - common::math：Apollo数学库的命名空间
   *   - Vec2d：Apollo定义的二维向量类
   *   - &：引用传递，避免拷贝
   */
  void AddPoint(std::string key, const common::math::Vec2d& point);

  /**
   * @brief 向指定曲线添加多个点
   *
   * @param key 曲线名称
   * @param points Vec2d点的向量
   *
   * 功能说明：
   * 批量添加多个点到指定曲线
   * 内部循环调用单个AddPoint
   *
   * C++语法说明：
   * - const std::vector<common::math::Vec2d>&：
   *   Vec2d对象的常量引用向量
   *   - std::vector：动态数组容器
   *   - const：输入参数不可被修改
   */
  void AddPoint(std::string key,
                const std::vector<common::math::Vec2d>& points);

  /**
   * @brief 将所有曲线打印到日志
   *
   * 功能说明：
   * 遍历所有曲线，依次打印每条曲线的数据
   * 格式为：curve_name: [(x1,y1), (x2,y2), ...]
   */
  void PrintToLog();

 private:
  /**
   * @brief 存储所有曲线的数据结构
   *
   * std::map<std::string, PrintPoints>：
   * - string：曲线的名称（key）
   * - PrintPoints：曲线包含的点集（value）
   *
   * C++语法说明：
   * - std::map：
   *   关联容器，键值对存储
   *   - 自动按键排序
   *   - 键唯一，不重复
   *   - 提供对数时间复杂度的查找
   */
  std::map<std::string, PrintPoints> curve_map_;
};

/**
 * @class PrintBox
 * @brief 边界框打印类
 *
 * 功能说明：
 * 存储和打印车辆的矩形边界框信息
 * 用于调试时可视化车辆在地图中的位置和朝向
 *
 * 数据格式：
 * 每个边界框由四个角点组成
 * 存储格式：[x, y, theta, length, width]
 * - x, y: 车辆位置
 * - theta: 车辆朝向角（弧度）
 * - length: 车辆长度
 * - width: 车辆宽度
 */
class PrintBox {
 public:
  /**
   * @brief 带ID的构造函数
   *
   * @param id 边界框的标识符
   *
   * 功能说明：
   * 创建一个带有指定ID的边界框打印对象
   *
   * C++语法说明：
   * - explicit：防止隐式类型转换
   * - : id_(id)：初始化列表，初始化成员变量
   */
  explicit PrintBox(std::string id) : id_(id) {}

  /**
   * @brief 添加自车边界框
   *
   * @param x 车辆位置X坐标
   * @param y 车辆位置Y坐标
   * @param heading 车辆朝向角（弧度）
   * @param is_rear_axle_point 是否以后轴中心为参考点
   *
   * 功能说明：
   * 将车辆边界框信息添加到打印列表
   * 用于可视化车辆当前位置和方向
   *
   * 算法流程：
   * 1. 存储x, y, heading作为边界框参考信息
   * 2. 后续PrintToLog会计算四个角点坐标并输出
   *
   * C++语法说明：
   * - bool：布尔类型
   * - is_rear_axle_point默认为true：
   *   表示(x,y)是后轴中心点
   *   如果为false，则(x,y)是后轴中心点
   */
  void AddAdcBox(double x, double y, double heading,
                 bool is_rear_axle_point = true);

  /**
   * @brief 将边界框打印到日志
   *
   * 功能说明：
   * 输出存储的所有边界框信息
   * 便于在日志中可视化车辆位置
   */
  void PrintToLog();

 private:
  /**
   * @brief 边界框标识符
   */
  std::string id_;

  /**
   * @brief 存储边界框数据的容器
   *
   * std::vector<std::vector<double>>：
   * - 外层vector：存储多个边界框
   * - 内层vector：每个边界框的数据 [x, y, theta, length, width]
   *
   * C++语法说明：
   * - std::vector<std::vector<double>>：
   *   二维动态数组
   *   可以看作"数组的数组"
   */
  std::vector<std::vector<double>> box_points;
};

}  // namespace planning
}  // namespace apollo
