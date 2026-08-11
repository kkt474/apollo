/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
 * @file util.h
 * @brief 规划模块工具函数声明头文件
 *
 * 功能说明：
 * 该文件声明了Apollo规划模块的各种工具函数
 * 这些函数被组织在apollo::planning::util命名空间中
 * 主要包括：
 * 1. 车辆状态验证
 * 2. 路由变更检测
 * 3. 停车减速计算
 * 4. 停止线和交通信号灯检查
 * 5. 路口判断
 * 6. 文件操作
 * 7. 等效自车宽度计算
 * 8. 弧线边界计算
 *
 * C++语法说明：
 * - #pragma once：编译指示符，防止头文件重复包含
 * - namespace嵌套：apollo::planning::util三层命名空间
 * - 函数声明：仅声明函数签名，不包含实现
 * - 前向声明：避免循环依赖
 * - using声明：简化类型名称
 */

/**
 * @brief 确保头文件只被包含一次
 *
 * C++语法说明：
 * #pragma once是编译指示符
 * 告诉编译器这个头文件只处理一次
 * 作用类似于传统的#ifndef宏保护
 * 优点：更简洁，编译器直接处理
 * 缺点：不是所有编译器都支持（现代编译器都支持）
 */
#pragma once

/**
 * @brief 标准库头文件
 *
 * C++语法说明：
 * #include <string>：字符串类模板
 *   - std::string是std::basic_string<char>的特化
 *   - 提供字符串存储和操作功能
 *
 * #include <vector>：动态数组容器
 *   - std::vector是STL容器
 *   - 支持随机访问，动态大小
 */
#include <string>
#include <vector>

/**
 * @brief Boost库头文件
 *
 * C++语法说明：
 * #include <boost/filesystem.hpp>：
 *   - Boost.Filesystem库提供跨平台文件系统操作
 *   - boost::filesystem::path：路径类型
 *   - boost::filesystem::directory_iterator：目录迭代器
 *   - 提供exists()、is_regular_file()、is_directory()等函数
 *
 * #include <boost/range/iterator_range.hpp>：
 *   - Boost.Range库提供范围封装
 *   - boost::make_iterator_range()：创建迭代器范围
 *   - 简化范围for循环的写法
 */
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

/**
 * @brief Protobuf消息头文件
 *
 * C++语法说明：
 * protobuf是Google的跨平台数据序列化协议
 * .pb.h文件由.proto文件编译生成
 * 提供了消息类的定义
 *
 * - vehicle_state.pb.h：车辆状态消息
 * - planning_command.pb.h：规划命令消息
 * - routing.pb.h：路由消息
 */
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/common_msgs/planning_msgs/planning_command.pb.h"
#include "modules/common_msgs/routing_msgs/routing.pb.h"

/**
 * @brief Apollo其他模块的头文件
 *
 * C++语法说明：
 * 这些是Apollo内部模块之间的接口
 * 用于获取车辆状态、地图信息、参考线信息等
 */
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/planning/planning_base/common/reference_line_info.h"
#include "modules/planning/planning_base/common/util/print_debug_info.h"

/**
 * @brief Apollo外层命名空间
 *
 * C++语法说明：
 * namespace关键字用于声明命名空间
 * 命名空间用于组织代码，避免命名冲突
 * Apollo项目所有代码都位于apollo命名空间下
 */
namespace apollo {

/**
 * @brief planning模块命名空间
 *
 * 嵌套命名空间结构：apollo::planning
 * planning命名空间包含所有规划模块相关的类和函数
 */
namespace planning {

/**
 * @brief util工具函数命名空间
 *
 * 嵌套命名空间结构：apollo::planning::util
 * 所有规划相关的工具函数都封装在这个命名空间中
 * 这样可以将相关函数组织在一起，同时避免与其他模块冲突
 */
namespace util {

/**
 * @brief 检查车辆状态是否有效
 *
 * @param vehicle_state 待检查的车辆状态常量引用
 * @return bool 如果所有关键参数都有效（非NaN）返回true
 *
 * 功能说明：
 * 验证车辆状态的所有关键参数是否为有效值
 * 检查参数包括：x, y, z坐标、heading朝向角、kappa曲率、
 * linear_velocity线性速度、linear_acceleration线性加速度
 *
 * 使用场景：
 * - 规划模块计算前必须验证输入的车辆状态
 * - 避免NaN值导致的计算错误
 *
 * C++语法说明：
 * - const apollo::common::VehicleState&：
 *   - const：承诺不修改参数
 *   - apollo::common::VehicleState：完全限定类型名
 *   - &：引用，避免拷贝
 *
 * - bool：函数返回类型
 *   - true表示状态有效
 *   - false表示状态无效（包含NaN）
 */
bool IsVehicleStateValid(const apollo::common::VehicleState& vehicle_state);

/**
 * @brief 检查两次路由命令是否不同
 *
 * @param first 第一次路由命令的常量引用
 * @param second 第二次路由命令的常量引用
 * @return bool 如果路由不同返回true，相同返回false
 *
 * 功能说明：
 * 通过比较序列号、模块名、时间戳判断两次路由是否相同
 * 用于检测路由是否发生变化，从而决定是否需要重新规划
 *
 * C++语法说明：
 * - const PlanningCommand&：
 *   PlanningCommand是protobuf生成的消息类
 *   const引用避免拷贝
 *
 * - return bool：
 *   使用括号包围返回表达式是良好风格
 *   明确表示返回值的计算
 */
bool IsDifferentRouting(const PlanningCommand& first,
                        const PlanningCommand& second);

/**
 * @brief 计算自车停车减速度
 *
 * @param vehicle_state 车辆状态提供者指针
 * @param adc_front_edge_s 自车前端沿参考线的s坐标
 * @param stop_line_s 停止线沿参考线的s坐标
 * @return double 所需减速度（米/秒^2）
 *
 * 功能说明：
 * 根据当前速度和停止距离计算匀减速停车所需的减速度
 * 使用物理公式：a = v² / (2d)
 * 这是确定安全停车速度限制的重要计算
 *
 * 使用场景：
 * - 交通规则处理（红灯停车、让行停车）
 * - 障碍物前安全停车
 *
 * C++语法说明：
 * - apollo::common::VehicleStateProvider*：
 *   - 裸指针，不拥有对象所有权
 *   - 外部保证对象生命周期和有效性
 *   - 使用指针而非引用，因为可以为nullptr
 *
 * - const double adc_front_edge_s：
 *   - const：输入参数承诺不修改
 *   - double：浮点数类型
 *   - 参数按值传递
 *
 * - const double stop_line_s：
 *   停止线的s坐标
 *
 * - return double：
 *   返回计算得到的减速度值
 */
double GetADCStopDeceleration(
    apollo::common::VehicleStateProvider* vehicle_state,
    const double adc_front_edge_s, const double stop_line_s);

/**
 * @brief 检查停止标志重叠是否仍在参考线上
 *
 * @param reference_line_info 参考线信息的常量引用
 * @param stop_sign_overlap_id 停止标志重叠ID的常量引用
 * @return bool 如果仍在参考线上返回true
 *
 * 功能说明：
 * 验证指定的停止标志重叠是否仍然有效
 * 用于判断当车辆驶离停止标志后是否需要清除相关决策
 *
 * 算法：
 * 在参考线的stop_sign_overlaps列表中查找指定的ID
 *
 * C++语法说明：
 * - const ReferenceLineInfo&：
 *   ReferenceLineInfo是参考线信息类
 *   const引用避免拷贝
 *
 * - const std::string&：
 *   const引用避免字符串拷贝
 *   std::string是标准库字符串类
 *
 * - return bool：
 *   返回查找结果
 */
bool CheckStopSignOnReferenceLine(const ReferenceLineInfo& reference_line_info,
                                  const std::string& stop_sign_overlap_id);

/**
 * @brief 检查交通信号灯重叠是否仍在参考线上
 *
 * @param reference_line_info 参考线信息的常量引用
 * @param traffic_light_overlap_id 交通信号灯重叠ID的常量引用
 * @return bool 如果仍在参考线上返回true
 *
 * 功能说明：
 * 与CheckStopSignOnReferenceLine类似
 * 用于验证交通信号灯重叠是否仍然有效
 *
 * C++语法说明：
 * 与CheckStopSignOnReferenceLine相同
 * 区别在于调用的是signal_overlaps()而非stop_sign_overlaps()
 */
bool CheckTrafficLightOnReferenceLine(
    const ReferenceLineInfo& reference_line_info,
    const std::string& traffic_light_overlap_id);

/**
 * @brief 检查自车是否仍在PNC junction内
 *
 * @param reference_line_info 参考线信息的常量引用
 * @return bool 如果仍在junction内返回true
 *
 * 功能说明：
 * 判断自车是否已经通过路口
 * 用于控制进入/退出路口场景的处理逻辑
 *
 * 算法：
 * 1. 获取自车前后边缘的s坐标
 * 2. 查询自车前方是否有junction
 * 3. 如果没有junction，返回false
 * 4. 计算自车后边缘与junction末端的距离
 * 5. 如果距离小于阈值（2米），认为仍在junction内
 *
 * C++语法说明：
 * - const ReferenceLineInfo&：
 *   输入参考线信息
 *
 * - return bool：
 *   返回是否在junction内
 */
bool CheckInsideJunction(const ReferenceLineInfo& reference_line_info);

/**
 * @brief 递归获取指定路径下的所有文件
 *
 * @param path boost文件系统路径的常量引用
 * @param files 指向字符串向量的指针，作为输出参数
 *
 * 功能说明：
 * 递归遍历目录，获取所有文件的绝对路径
 * 常用于日志文件搜索、数据文件遍历等场景
 *
 * C++语法说明：
 * - const boost::filesystem::path&：
 *   - boost::filesystem::path：Boost.Filesystem库的路径类型
 *   - const引用避免拷贝
 *
 * - std::vector<std::string>* files：
 *   - 指向向量的指针作为输出参数
 *   - 调用者负责初始化向量
 *   - 函数通过指针修改向量的内容
 *
 * - void GetFilesByPath(...)：
 *   - 无返回值，通过指针参数返回结果
 */
void GetFilesByPath(const boost::filesystem::path& path,
                    std::vector<std::string>* files);

/**
 * @brief 计算等效自车宽度（考虑弯道曲率）
 *
 * @param reference_line_info 参考线信息的常量引用
 * @param s 沿参考线的位置
 * @param is_left 指向bool的指针，作为输出参数
 * @return double 等效自车宽度的一半
 *
 * 功能说明：
 * 考虑车辆在弯道行驶时的横向偏移
 * 弯道内侧的轮迹比外侧短，需要更宽的等效宽度来补偿
 * 用于更精确的路径规划和碰撞检测
 *
 * 算法流程：
 * 1. 获取车辆参数（轴距、前后悬等）
 * 2. 计算前后轴位置的参考线朝向
 * 3. 计算前后轴位置的曲率kappa_f和kappa_b
 * 4. 根据曲率计算等效宽度
 *
 * C++语法说明：
 * - double s：
 *   沿参考线的距离坐标
 *
 * - bool* is_left：
 *   指向bool的指针作为输出参数
 *   函数通过解引用赋值：*is_left = value
 *
 * - return double：
 *   返回等效宽度的一半
 */
double CalculateEquivalentEgoWidth(const ReferenceLineInfo& reference_line_info,
                                   double s, bool* is_left);

/**
 * @brief 计算等效自车宽度（基于LaneInfo）
 *
 * @param lane_info 车道信息常指针
 * @param s 沿车道的距离
 * @param is_left 指向bool的指针，作为输出参数
 * @return double 等效自车宽度的一半
 *
 * 功能说明：
 * 与上一个函数功能相同
 * 区别在于使用LaneInfo而非ReferenceLineInfo
 * 提供更底层的车道信息访问
 *
 * C++语法说明：
 * - apollo::hdmap::LaneInfoConstPtr：
 *   - LaneInfo的const共享指针类型
 *   - shared_ptr<const LaneInfo>
 *   - ConstPtr是Apollo定义的类型别名
 *
 * - const apollo::hdmap::LaneInfoConstPtr：
 *   - const修饰指针指向的内容不变
 *   - 但这个声明可能有问题，应该是LaneInfoConstPtr而非const
 */
double CalculateEquivalentEgoWidth(
    const apollo::hdmap::LaneInfoConstPtr lane_info, double s, bool* is_left);

/**
 * @brief 计算左侧弧线边界
 *
 * @param delta_x 相对位移
 * @param r 弧线半径
 * @param heading 朝向角
 * @param result 指向double的指针，作为输出参数
 * @return bool 计算是否成功
 *
 * 功能说明：
 * 根据给定半径和初始朝向，计算左侧弧线的边界位移
 * 用于路径边界约束计算
 *
 * 数学原理：
 * 圆的标准方程：(x - R*sin(θ))² + (y + R*cos(θ))² = R²
 * 上半圆：y = sqrt(R² - (x - R*sin(θ))²) - R*cos(θ)
 *
 * C++语法说明：
 * - double delta_x, r, heading：
 *   输入参数，按值传递
 *
 * - double* result：
 *   输出参数指针
 *   通过解引用赋值
 *
 * - return bool：
 *   表示计算是否成功
 *   false表示参数超出有效范围
 */
bool left_arc_bound_with_heading(double delta_x, double r, double heading,
                                 double* result);

/**
 * @brief 计算右侧弧线边界
 *
 * @param delta_x 相对位移
 * @param r 弧线半径
 * @param heading 朝向角
 * @param result 指向double的指针，作为输出参数
 * @return bool 计算是否成功
 *
 * 功能说明：
 * 根据给定半径和初始朝向，计算右侧弧线的边界位移
 *
 * 数学原理：
 * 圆的标准方程：(x + R*sin(θ))² + (y - R*cos(θ))² = R²
 * 下半圆：y = R*cos(θ) - sqrt(R² - (x + R*sin(θ))²)
 */
bool right_arc_bound_with_heading(double delta_x, double r, double heading,
                                  double* result);

/**
 * @brief 计算带反向曲率的左侧弧线边界
 *
 * @param delta_x 相对位移
 * @param r 弧线半径
 * @param heading 朝向角
 * @param kappa 曲率
 * @param result 指向double的指针，作为输出参数
 * @return bool 计算是否成功
 *
 * 功能说明：
 * 考虑曲率变化的左侧弧线边界计算
 * 用于更精确的弯道边界建模
 *
 * C++语法说明：
 * - double kappa：
 *   额外的曲率参数
 *   用于更复杂的弯道建模
 */
bool left_arc_bound_with_heading_with_reverse_kappa(double delta_x, double r,
                                                    double heading,
                                                    double kappa,
                                                    double* result);

/**
 * @brief 计算带反向曲率的右侧弧线边界
 *
 * @param delta_x 相对位移
 * @param r 弧线半径
 * @param heading 朝向角
 * @param kappa 曲率
 * @param result 指向double的指针，作为输出参数
 * @return bool 计算是否成功
 *
 * 功能说明：
 * 考虑曲率变化的右侧弧线边界计算
 */
bool right_arc_bound_with_heading_with_reverse_kappa(double delta_x, double r,
                                                     double heading,
                                                     double kappa,
                                                     double* result);

/**
 * @brief 命名空间结束标记
 *
 * C++语法说明：
 * // 注释用于说明命名空间结束
 * 三层命名空间的闭合需要按顺序：
 *
 * }  // namespace util
 * - 关闭util命名空间
 *
 * }  // namespace planning
 * - 关闭planning命名空间
 *
 * }  // namespace apollo
 * - 关闭apollo命名空间
 *
 * 这种注释风格有助于阅读大型代码库的嵌套命名空间
 */
}  // namespace util
}  // namespace planning
}  // namespace apollo
