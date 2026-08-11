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
 * @file util.cc
 * @brief 规划模块工具函数实现文件
 *
 * 功能说明：
 * 该文件实现了Apollo规划模块的各种工具函数
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
 * - namespace嵌套：apollo::planning::util三層命名空间
 * - using声明：引入其他命名空间的符号，避免冗长前缀
 * - std::isnan()：检查浮点数是否为NaN
 * - std::find_if：算法库中的查找函数
 * - lambda表达式：匿名函数对象用于std::find_if
 * - 智能引用和链式调用
 */

#include "modules/planning/planning_base/common/util/util.h"

#include <algorithm>
#include <limits>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
/**
 * @brief Apollo外层命名空间
 *
 * C++语法说明：
 * namespace关键字用于声明命名空间
 * 所有Apollo相关代码都位于apollo命名空间下
 */
namespace planning {
/**
 * @brief planning模块命名空间
 *
 * 嵌套命名空间结构：apollo::planning
 */
namespace util {
/**
 * @brief util工具函数命名空间
 *
 * 嵌套命名空间结构：apollo::planning::util
 * 所有工具函数都封装在这个命名空间中
 */

/**
 * @brief 使用using声明引入外部命名空间的类型
 *
 * C++语法说明：
 * - using声明：using apollo::common::VehicleState;
 *   使VehicleState可以直接使用，不必写完整限定名apollo::common::VehicleState
 * - 这只是声明，不是typedef，不能用于声明变量
 * - 通常放在.cpp文件顶部，简化代码书写
 */
using apollo::common::VehicleState;
using apollo::hdmap::PathOverlap;

/**
 * @brief 检查车辆状态是否有效
 *
 * @param vehicle_state 待检查的车辆状态引用
 * @return bool 如果状态有效返回true，无效返回false
 *
 * 功能说明：
 * 验证车辆状态的所有关键参数是否为有效值（非NaN）
 * 这是规划模块进行轨迹计算前的必要检查
 *
 * 检查的参数包括：
 * - x, y, z：车辆位置坐标
 * - heading：车辆朝向角
 * - kappa：路径曲率
 * - linear_velocity：线性速度
 * - linear_acceleration：线性加速度
 *
 * C++语法说明：
 * - const VehicleState& vehicle_state：
 *   常量引用参数，避免拷贝开销
 *   函数承诺不修改vehicle_state
 *
 * - std::isnan()：
 *   标准库函数，检查浮点数是否为NaN（Not a Number）
 *   如果参数是NaN返回true，否则返回false
 *   NaN通常由0/0、inf-inf等无效运算产生
 *
 * - || 逻辑或运算符：
 *   短路求值，任一条件为true则整体为true
 *   这里连接所有NaN检查，任何一个为NaN则返回false
 *
 * - return false/true：
 *   返回bool类型的值
 *   整个函数是一个bool表达式
 */
bool IsVehicleStateValid(const VehicleState& vehicle_state) {
  /**
   * @brief NaN检查表达式
   *
   * 使用||连接所有需要检查的字段
   * 如果任何一个字段是NaN，整个表达式为true
   * 外层!取反，表示"如果有任何NaN，返回false"
   */
  if (std::isnan(vehicle_state.x()) || std::isnan(vehicle_state.y()) ||
      std::isnan(vehicle_state.z()) || std::isnan(vehicle_state.heading()) ||
      std::isnan(vehicle_state.kappa()) ||
      std::isnan(vehicle_state.linear_velocity()) ||
      std::isnan(vehicle_state.linear_acceleration())) {
    return false;
  }
  return true;
}

/**
 * @brief 检查两次路由命令是否不同
 *
 * @param first 第一次路由命令
 * @param second 第二次路由命令
 * @return bool 如果路由不同返回true，相同返回false
 *
 * 功能说明：
 * 比较两次PlanningCommand的序列号、模块名、时间戳
 * 用于判断是否需要重新规划路径
 *
 * C++语法说明：
 * - const PlanningCommand&：
 *   常量引用，避免拷贝
 *
 * - first.has_header()：
 *   protobuf消息的has_方法
 *   检查可选字段是否已设置
 *
 * - const auto& first_header = first.header()：
 *   auto自动推导类型为const Header&
 *   const auto&避免拷贝
 *
 * - first_header.sequence_num()：
 *   protobuf消息的getter方法
 *   返回字段值（这里是int64）
 *
 * - first_header.module_name()：
 *   返回string类型的模块名
 *
 * - first_header.timestamp_sec()：
 *   返回double类型的时间戳（秒）
 *
 * - != 运算符：
 *   比较操作符，返回bool
 *
 * - || 逻辑或：
 *   多个条件任一不同则返回true
 */
bool IsDifferentRouting(const PlanningCommand& first,
                        const PlanningCommand& second) {
  /**
   * @brief 首先检查两个命令是否有header
   *
   * 如果任一命令没有header，无法进行比较
   * 视为不同路由
   */
  if (first.has_header() && second.has_header()) {
    const auto& first_header = first.header();
    const auto& second_header = second.header();
    /**
     * @brief 比较header的三个字段
     *
     * sequence_num：序列号，不同规划周期递增
     * module_name：模块名称
     * timestamp_sec：时间戳（秒级）
     * 任一不同则路由不同
     */
    return (first_header.sequence_num() != second_header.sequence_num() ||
            first_header.module_name() != second_header.module_name() ||
            first_header.timestamp_sec() != second_header.timestamp_sec());
  }
  return true;
}

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
 * 这是确定安全停车速度限制的重要计算
 *
 * 算法说明：
 * 使用匀减速运动公式：v^2 = 2*a*d
 * 推导得：a = v^2 / (2*d)
 * 其中v是当前速度，d是停止距离
 *
 * C++语法说明：
 * - apollo::common::VehicleStateProvider*：
 *   裸指针，不拥有对象所有权
 *   外部保证对象生命周期
 *
 * - common::VehicleConfigHelper::Instance()：
 *   单例模式获取车辆配置助手实例
 *   ->GetConfig()获取配置对象
 *   .vehicle_param()获取车辆参数结构
 *
 * - max_abs_speed_when_stopped()：
 *   车辆停止时的最大绝对速度阈值
 *   低于此速度视为已停止
 *
 * - std::numeric_limits<double>::max()：
 *   返回double类型的最大值（约1.8e308）
 *   用于表示"无限大"的减速度
 *
 * - 1e-5：科学计数法，表示0.00001
 *   用于浮点数相等性判断的容差
 */
double GetADCStopDeceleration(
    apollo::common::VehicleStateProvider* vehicle_state,
    const double adc_front_edge_s, const double stop_line_s) {
  /**
   * @brief 获取当前车速
   *
   * linear_velocity()返回double类型的速度（米/秒）
   */
  double adc_speed = vehicle_state->linear_velocity();

  /**
   * @brief 获取车辆停止速度阈值
   *
   * 从车辆配置中读取max_abs_speed_when_stopped参数
   * 这是一个很小的值（如0.1 m/s），低于此值视为已停止
   */
  const double max_adc_stop_speed = common::VehicleConfigHelper::Instance()
                                        ->GetConfig()
                                        .vehicle_param()
                                        .max_abs_speed_when_stopped();

  /**
   * @brief 如果速度已低于阈值，无需减速
   */
  if (adc_speed < max_adc_stop_speed) {
    return 0.0;
  }

  double stop_distance = 0;

  /**
   * @brief 计算停止距离
   *
   * stop_line_s > adc_front_edge_s：
   * 停止线在自车前方时的距离计算
   * 否则停止距离为0（已在或越过停止线）
   */
  if (stop_line_s > adc_front_edge_s) {
    stop_distance = stop_line_s - adc_front_edge_s;
  }

  /**
   * @brief 距离过小时返回最大减速度
   *
   * stop_distance < 1e-5：
   * 距离非常小（如已在停止线上）
   * 此时需要极大的减速度，实际无法安全停车
   * 返回最大值表示这种情况
   */
  if (stop_distance < 1e-5) {
    return std::numeric_limits<double>::max();
  }

  /**
   * @brief 匀减速运动公式计算减速度
   *
   * a = v^2 / (2 * d)
   * 推导自：v^2 - v0^2 = 2*a*d（其中v=0为终止速度）
   * 这里假设匀减速到停止
   */
  return (adc_speed * adc_speed) / (2 * stop_distance);
}

/*
 * @brief: 检查停止标志是否仍在参考线上
 *
 * C++语法说明：
 * /* */ 是传统的C风格多行注释
 * 也可以使用//单行注释
 */
/**
 * @brief 检查停止标志重叠是否仍在参考线上
 *
 * @param reference_line_info 参考线信息
 * @param stop_sign_overlap_id 停止标志重叠ID
 * @return bool 如果仍在参考线上返回true
 *
 * 功能说明：
 * 验证指定的停止标志重叠是否仍然有效
 * 用于判断当车辆驶离停止标志后是否需要清除相关决策
 *
 * C++语法说明：
 * - const ReferenceLineInfo&：
 *   常量引用，避免拷贝
 *
 * - const std::string&：
 *   常量字符串引用，避免拷贝
 *
 * - std::find_if：
 *   标准库算法，在范围内查找满足条件的元素
 *   返回满足条件的第一个元素的迭代器
 *   未找到则返回end()
 *
 * - stop_sign_overlaps.begin() / end()：
 *   获取向量的起始和结束迭代器
 *   形成要查找的范围
 *
 * - lambda表达式：[&stop_sign_overlap_id](const PathOverlap& overlap)
 *   [&stop_sign_overlap_id]：捕获列表
 *   &表示按引用捕获外部变量
 *   stop_sign_overlap_id是要查找的ID
 *
 * - return overlap.object_id == stop_sign_overlap_id：
 *   lambda函数体，返回bool
 *   谓词：判断当前overlap是否是目标
 *
 * - stop_sign_overlap_it != stop_sign_overlaps.end()：
 *   比较迭代器，检查是否找到
 *   end()返回超出容器末尾的位置
 */
bool CheckStopSignOnReferenceLine(const ReferenceLineInfo& reference_line_info,
                                  const std::string& stop_sign_overlap_id) {
  /**
   * @brief 获取参考线上所有停止标志重叠
   *
   * reference_line_info.reference_line().map_path()：
   * 链式调用获取地图路径
   *
   * .stop_sign_overlaps()：
   * 返回std::vector<PathOverlap>
   * 包含所有停止标志重叠信息
   */
  const std::vector<PathOverlap>& stop_sign_overlaps =
      reference_line_info.reference_line().map_path().stop_sign_overlaps();

  /**
   * @brief 使用std::find_if查找目标重叠
   *
   * auto自动推导迭代器类型
   * std::find_if在[begin, end)范围内查找
   * 找到第一个满足lambda条件的元素
   */
  auto stop_sign_overlap_it =
      std::find_if(stop_sign_overlaps.begin(), stop_sign_overlaps.end(),
                   [&stop_sign_overlap_id](const PathOverlap& overlap) {
                     return overlap.object_id == stop_sign_overlap_id;
                   });

  /**
   * @brief 返回查找结果
   *
   * 如果找到，返回true
   * 如果未找到（迭代器等于end()），返回false
   */
  return (stop_sign_overlap_it != stop_sign_overlaps.end());
}

/*
 * @brief: check if a traffic_light_overlap is still along reference_line
 */
/**
 * @brief 检查交通信号灯重叠是否仍在参考线上
 *
 * @param reference_line_info 参考线信息
 * @param traffic_light_overlap_id 交通信号灯重叠ID
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
    const std::string& traffic_light_overlap_id) {
  const std::vector<PathOverlap>& traffic_light_overlaps =
      reference_line_info.reference_line().map_path().signal_overlaps();
  auto traffic_light_overlap_it =
      std::find_if(traffic_light_overlaps.begin(), traffic_light_overlaps.end(),
                   [&traffic_light_overlap_id](const PathOverlap& overlap) {
                     return overlap.object_id == traffic_light_overlap_id;
                   });
  return (traffic_light_overlap_it != traffic_light_overlaps.end());
}

/*
 * @brief: check if ADC is till inside a pnc-junction
 */
/**
 * @brief 检查自车是否仍在PNC junction（路径交汇区）内
 *
 * @param reference_line_info 参考线信息
 * @return bool 如果仍在junction内返回true
 *
 * 功能说明：
 * 判断自车是否已经通过路口
 * 用于控制进入/退出路口场景的处理逻辑
 *
 * 算法流程：
 * 1. 获取自车前后边缘的s坐标
 * 2. 查询自车前方是否有junction
 * 3. 如果没有junction，返回false
 * 4. 计算自车后边缘与junction末端的距离
 * 5. 如果距离小于阈值（2米），认为仍在junction内
 *
 * C++语法说明：
 * - reference_line_info.AdcSlBoundary()：
 *   获取自车的SL边界
 *   返回包含start_s和end_s的结构
 *
 * - .end_s() / .start_s()：
 *   分别获取边界在s方向的结束和起始坐标
 *   end_s是前端，start_s是后端
 *
 * - reference_line_info.GetJunction()：
 *   根据s坐标查询junction信息
 *   如果该位置有junction，填充junction_overlap
 *
 * - junction_overlap.object_id.empty()：
 *   检查string是否为空
 *   empty()比size() == 0更高效
 *
 * - static constexpr double：
 *   static：文件作用域，链接时不导出
 *   constexpr：编译时常量
 *   用于定义常量kIntersectionPassDist = 2.0
 *
 * - ADEBUG << ...：
 *   Apollo调试日志宏
 *   仅在DEBUG模式下输出
 *   <<运算符拼接日志内容
 */
bool CheckInsideJunction(const ReferenceLineInfo& reference_line_info) {
  /**
   * @brief 获取自车在s方向的前后边缘
   */
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double adc_back_edge_s = reference_line_info.AdcSlBoundary().start_s();

  /**
   * @brief 定义junction重叠变量
   *
   * 用于存储查询到的junction信息
   */
  hdmap::PathOverlap junction_overlap;

  /**
   * @brief 查询自车前方的junction
   */
  reference_line_info.GetJunction(adc_front_edge_s, &junction_overlap);

  /**
   * @brief 如果没有junction，返回false
   */
  if (junction_overlap.object_id.empty()) {
    return false;
  }

  /**
   * @brief 定义通过路口的距离阈值
   *
   * 单位：米
   * 2米是一个经验值，表示即将离开路口的距离
   */
  static constexpr double kIntersectionPassDist = 2.0;

  /**
   * @brief 计算自车后边缘与junction末端的距离
   *
   * 如果junction已过，distance_adc_pass_intersection > 0
   * 如果还未到达junction末端，distance_adc_pass_intersection < 0
   */
  const double distance_adc_pass_intersection =
      adc_back_edge_s - junction_overlap.end_s;

  /**
   * @brief 调试日志输出
   */
  ADEBUG << "distance_adc_pass_intersection[" << distance_adc_pass_intersection
         << "] junction_overlap[" << junction_overlap.object_id << "] start_s["
         << junction_overlap.start_s << "]";

  /**
   * @brief 判断是否仍在junction内
   *
   * 如果distance_adc_pass_intersection < 2.0
   * 说明自车后边缘还未离开junction末端
   */
  return distance_adc_pass_intersection < kIntersectionPassDist;
}

/*
 * @brief: get files at a path
 */
/**
 * @brief 递归获取指定路径下的所有文件
 *
 * @param path boost文件系统路径
 * @param files 输出参数，存储文件列表
 *
 * 功能说明：
 * 递归遍历目录，获取所有文件的绝对路径
 * 常用于日志文件搜索、数据文件遍历等场景
 *
 * C++语法说明：
 * - boost::filesystem::path：
 *   Boost.Filesystem库的路径类型
 *   提供跨平台的文件路径操作
 *
 * - std::vector<std::string>*：
 *   指向字符串向量的指针作为输出参数
 *   调用者负责初始化向量
 *
 * - ACHECK(files)：
 *   Apollo的断言宏
 *   如果条件为false，程序终止并输出错误
 *   用于检查必要的前置条件
 *
 * - boost::filesystem::exists(path)：
 *   检查路径是否存在（文件或目录）
 *
 * - boost::filesystem::is_regular_file(path)：
 *   检查是否是普通文件
 *
 * - boost::filesystem::is_directory(path)：
 *   检查是否是目录
 *
 * - boost::filesystem::directory_iterator(path)：
 *   目录迭代器，用于遍历目录内容
 *
 * - boost::make_iterator_range：
 *   创建迭代器范围
 *   与directory_iterator配合使用
 *
 * - boost::filesystem::directory_iterator(path), {}：
 *   {}是end迭代器，表示目录迭代结束
 *   make_iterator_range创建[begin, end)范围
 *
 * - entry.path()：
 *   获取目录条目的路径
 *   返回boost::filesystem::path对象
 *
 * - GetFilesByPath递归调用：
 *   递归处理子目录
 */
void GetFilesByPath(const boost::filesystem::path& path,
                    std::vector<std::string>* files) {
  /**
   * @brief 检查输出参数有效性
   */
  ACHECK(files);

  /**
   * @brief 如果路径不存在，直接返回
   */
  if (!boost::filesystem::exists(path)) {
    return;
  }

  /**
   * @brief 如果是普通文件，添加到列表
   */
  if (boost::filesystem::is_regular_file(path)) {
    AINFO << "Found record file: " << path.c_str();
    files->push_back(path.c_str());
    return;
  }

  /**
   * @brief 如果是目录，递归遍历子目录
   */
  if (boost::filesystem::is_directory(path)) {
    /**
     * @brief 遍历目录中的每个条目
     */
    for (auto& entry : boost::make_iterator_range(
             boost::filesystem::directory_iterator(path), {})) {
      /**
       * @brief 递归处理子条目
       */
      GetFilesByPath(entry.path(), files);
    }
  }
}

/*
 * @brief: get path equivalent ego width
 */
/**
 * @brief 计算等效自车宽度（考虑弯道曲率）
 *
 * @param reference_line_info 参考线信息
 * @param s 沿参考线的位置
 * @param is_left 输出参数，标识车道弯曲方向
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
 * - bool* is_left：
 *   指向bool的指针作为输出参数
 *   函数通过解引用修改其值
 *
 * - common::VehicleConfigHelper::Instance()：
 *   单例模式获取配置实例
 *
 * - vehicle_param.min_turn_radius()：
 *   车辆最小转弯半径
 *   1.0 / min_turn_radius = 最大曲率
 *
 * - 0.5 * vehicle_param.wheel_base()：
 *   wheel_base是前后轴中心距
 *   half_wb = 前后轴距离的一半
 *
 * - reference_line_info.reference_line()：
 *   获取参考线引用
 *
 * - .GetReferencePoint(s)：
 *   根据s坐标获取参考线上的点
 *
 * - .heading()：
 *   获取该点的朝向角（弧度）
 *
 * - apollo::common::math::NormalizeAngle()：
 *   角度归一化函数
 *   将角度归一化到[-π, π]范围
 *
 * - kappa_f * kappa_b < 0.0：
 *   检查两个曲率是否异号
 *   异号表示车道向不同方向弯曲
 *
 * - std::min(fabs(kappa_f), max_kappa)：
 *   fabs：浮点数绝对值
 *   限制曲率不超过最大允许值
 *
 * - apollo::common::math::Vec2d：
 *   2D向量类，提供几何运算
 *   Angle()：计算向量角度
 *   Length()：计算向量长度
 */
double CalculateEquivalentEgoWidth(const ReferenceLineInfo& reference_line_info,
                                   double s, bool* is_left) { 
  /**
   * @brief 获取车辆参数
   */
  const auto& vehicle_param =
      common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param();

  /**
   * @brief 计算最大曲率（最小转弯半径的倒数）
   */
  double max_kappa = 1.0 / vehicle_param.min_turn_radius();

  /**
   * @brief 计算车辆几何参数
   *
   * half_wb：前后轴距离的一半
   * front_l：几何中心到车头的距离
   * half_w：车宽的一半
   */
  double half_wb = 0.5 * vehicle_param.wheel_base();
  double front_l = vehicle_param.front_edge_to_center() - half_wb;
  double half_w = 0.5 * vehicle_param.width();

  /**
   * @brief 获取参考线上三个位置的朝向
   *
   * current_heading：当前位置
   * heading_f：前轴位置
   * heading_b：后轴位置
   */
  double current_heading =
      reference_line_info.reference_line().GetReferencePoint(s).heading();
  double heading_f = reference_line_info.reference_line()
                         .GetReferencePoint(s + front_l)
                         .heading();
  double heading_b = reference_line_info.reference_line()
                         .GetReferencePoint(s - half_wb)
                         .heading();

  /**
   * @brief 计算前轴到中心的平均曲率
   *
   * NormalizeAngle归一化角度差
   * 除以前轴到中心的距离
   */
  double kappa_f =
      apollo::common::math::NormalizeAngle(heading_f - current_heading) /
      front_l;

  /**
   * @brief 计算中心到后轴的平均曲率
   */
  double kappa_b =
      apollo::common::math::NormalizeAngle(current_heading - heading_b) /
      half_wb;

  /**
   * @brief 判断车道弯曲方向
   *
   * kappa_b < 0.0 表示向左弯
   * kappa_b >= 0.0 表示向右弯或直行
   */
  *is_left = (kappa_b < 0.0);

  /**
   * @brief 特殊情况处理
   *
   * - kappa_f * kappa_b < 0.0：前后曲率异号，车道弯曲方向变化
   * - fabs(kappa_f) < 1e-6：曲率接近0，直行或无法计算
   * 两种情况都返回实际宽度的一半
   */
  if (kappa_f * kappa_b < 0.0 || fabs(kappa_f) < 1e-6) {
    return half_w;
  }

  /**
   * @brief 限制曲率不超过最大值
   */
  kappa_f = std::min(fabs(kappa_f), max_kappa);
  kappa_b = std::min(fabs(kappa_b), max_kappa);

  /**
   * @brief 计算等效宽度
   *
   * 使用向量运算计算弯道等效宽度
   * 参考Apollo论文中的几何计算方法
   */
  // Vec2d(x, y).Angle()-----> atan2(y,x)
  double theta = apollo::common::math::Vec2d(1.0, half_wb * kappa_b).Angle();
  double sint = std::sin(theta);
  double cost = std::cos(theta);
  double r_f = 1.0 / kappa_f;
  double eq_half_w =
      apollo::common::math::Vec2d(front_l * cost - half_w * sint,
                                  r_f + front_l * sint + half_w * cost)
          .Length() -
      r_f;

  /**
   * @brief 返回等效宽度和实际宽度的较大值
   */
  return std::max(eq_half_w, half_w);
}

/**
 * @brief 计算等效自车宽度（基于LaneInfo）
 *
 * @param lane_info 车道信息指针
 * @param s 沿车道的距离
 * @param is_left 输出参数，标识车道弯曲方向
 * @return double 等效自车宽度的一半
 *
 * 功能说明：
 * 与上一个函数功能相同
 * 区别在于使用LaneInfo而非ReferenceLineInfo
 * 提供更底层的车道信息访问
 *
 * C++语法说明：
 * - apollo::hdmap::LaneInfoConstPtr：
 *   LaneInfo的const共享指针类型
 *   shared_ptr<const LaneInfo>
 */
double CalculateEquivalentEgoWidth(
    const apollo::hdmap::LaneInfoConstPtr lane_info, double s, bool* is_left) {
  /**
   * @brief 获取车辆参数
   */
  const auto& vehicle_param =
      common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
  double max_kappa = 1.0 / vehicle_param.min_turn_radius();
  double half_wb = 0.5 * vehicle_param.wheel_base();
  double front_l = vehicle_param.front_edge_to_center() - half_wb;
  double half_w = 0.5 * vehicle_param.width();

  /**
   * @brief 使用LaneInfo的Heading方法获取朝向
   *
   * lane_info->Heading(s)：
   * 直接从LaneInfo获取指定s处的朝向
   * 更直接，不需要先生成参考点
   */
  double current_heading = lane_info->Heading(s);
  double heading_f = lane_info->Heading(s + front_l);
  double heading_b = lane_info->Heading(s - half_wb);

  /**
   * @brief 计算曲率和等效宽度
   *
   * 与上一个函数相同的算法
   */
  double kappa_f =
      apollo::common::math::NormalizeAngle(heading_f - current_heading) /
      front_l;
  double kappa_b =
      apollo::common::math::NormalizeAngle(current_heading - heading_b) /
      half_wb;
  *is_left = (kappa_b < 0.0);
  if (kappa_f * kappa_b < 0.0 || fabs(kappa_f) < 1e-6) {
    return half_w;
  }

  kappa_f = std::min(fabs(kappa_f), max_kappa);
  kappa_b = std::min(fabs(kappa_b), max_kappa);
  double theta = apollo::common::math::Vec2d(1.0, half_wb * kappa_b).Angle();
  double sint = std::sin(theta);
  double cost = std::cos(theta);
  double r_f = 1.0 / kappa_f;
  double eq_half_w =
      apollo::common::math::Vec2d(front_l * cost - half_w * sint,
                                  r_f + front_l * sint + half_w * cost)
          .Length() -
      r_f;
  return std::max(eq_half_w, half_w);
}

/**
 * @brief 计算左侧弧线边界
 *
 * @param delta_x 相对位移
 * @param r 弧线半径
 * @param heading 朝向角
 * @param result 输出参数，计算结果
 * @return bool 计算是否成功
 *
 * 功能说明：
 * 根据给定半径和初始朝向，计算左侧弧线的边界位移
 * 用于路径边界约束计算
 *
 * 数学原理：
 * 圆的标准方程：(x - R*sin(heading))^2 + (y + R*cos(heading))^2 = R^2
 * 上半圆：y = sqrt(R^2 - (x - R*sin(heading))^2) - R*cos(heading)
 *
 * C++语法说明：
 * - double* result：
 *   指针作为输出参数
 *
 * - std::numeric_limits<double>::lowest()：
 *   返回double类型的最小值
 *   用于表示计算失败时的结果
 *
 * - std::sqrt(r * r - std::pow(delta_x - r * std::sin(heading), 2))：
 *   std::sqrt：平方根函数
 *   std::pow(x, 2)：x的平方
 *
 * - 1e-6：科学计数法，0.000001
 *   用作浮点数比较的容差
 */
bool left_arc_bound_with_heading(double delta_x, double r, double heading,
                                 double* result) {
  /**
   * @brief 边界条件检查
   *
   * delta_x > r * (1.0 + std::sin(heading)) - 1e-6：
   * 如果位移超过圆弧可达到的范围
   * 此时无解，返回false
   */
  if (delta_x > r * (1.0 + std::sin(heading)) - 1e-6) {
    *result = std::numeric_limits<double>::lowest();
    return false;
  }

  /**
   * @brief 计算左侧弧线边界
   *
   * 使用圆的参数方程计算y值（弧线位移）
   */
  *result = std::sqrt(r * r - std::pow(delta_x - r * std::sin(heading), 2)) -
            r * std::cos(heading);
  return true;
}

/**
 * @brief 计算右侧弧线边界
 *
 * @param delta_x 相对位移
 * @param r 弧线半径
 * @param heading 朝向角
 * @param result 输出参数，计算结果
 * @return bool 计算是否成功
 *
 * 功能说明：
 * 根据给定半径和初始朝向，计算右侧弧线的边界位移
 *
 * 数学原理：
 * 圆的标准方程：(x + R*sin(heading))^2 + (y - R*cos(heading))^2 = R^2
 * 下半圆：y = R*cos(heading) - sqrt(R^2 - (x + R*sin(heading))^2)
 */
bool right_arc_bound_with_heading(double delta_x, double r, double heading,
                                  double* result) {
  /**
   * @brief 边界条件检查
   */
  if (delta_x > r * (1.0 - std::sin(heading)) - 1e-6) {
    *result = std::numeric_limits<double>::max();
    return false;
  }

  /**
   * @brief 计算右侧弧线边界
   */
  *result = r * std::cos(heading) -
            std::sqrt(r * r - std::pow(delta_x + r * std::sin(heading), 2));
  return true;
}

/**
 * @brief 计算带反向曲率的左侧弧线边界
 *
 * @param delta_x 相对位移
 * @param r 弧线半径
 * @param heading 朝向角
 * @param kappa 曲率
 * @param result 输出参数，计算结果
 * @return bool 计算是否成功
 *
 * 功能说明：
 * 考虑曲率变化的左侧弧线边界计算
 * 用于更精确的弯道边界建模
 *
 * C++语法说明：
 * - heading > 0 || kappa < 0：
 *   组合条件检查
 *   heading > 0：向左转
 *   kappa < 0：曲率为负（表示左转）
 *   两者不一致时无解
 */
bool left_arc_bound_with_heading_with_reverse_kappa(double delta_x, double r,
                                                    double heading,
                                                    double kappa,
                                                    double* result) {
  /**
   * @brief 参数有效性检查
   *
   * heading > 0：要求heading <= 0
   * kappa < 0：要求kappa >= 0
   * delta_x超出范围：无解
   */
  if (heading > 0 || kappa < 0 ||
      delta_x > r * (1.0 - std::sin(heading)) - 1e-6) {
    *result = std::numeric_limits<double>::lowest();
    return false;
  }

  /**
   * @brief 分段计算
   *
   * 根据delta_x与-r*sin(heading)的关系
   * 选择不同的计算公式
   */
  if (delta_x < -r * std::sin(heading)) {
    *result = r * std::cos(heading) -
              std::sqrt(r * r - std::pow(delta_x - r * std::sin(heading), 2));
  } else {
    *result = std::sqrt(r * r - std::pow(delta_x + r * std::sin(heading), 2)) -
              r * (2 - std::cos(heading));
  }
  return true;
}

/**
 * @brief 计算带反向曲率的右侧弧线边界
 *
 * @param delta_x 相对位移
 * @param r 弧线半径
 * @param heading 朝向角
 * @param kappa 曲率
 * @param result 输出参数，计算结果
 * @return bool 计算是否成功
 *
 * 功能说明：
 * 考虑曲率变化的右侧弧线边界计算
 *
 * C++语法说明：
 * - heading < 0 || kappa > 0：
 *   组合条件检查
 *   heading < 0：要求heading >= 0
 *   kappa > 0：要求kappa <= 0
 *   两者不一致时无解
 */
bool right_arc_bound_with_heading_with_reverse_kappa(double delta_x, double r,
                                                     double heading,
                                                     double kappa,
                                                     double* result) {
  /**
   * @brief 参数有效性检查
   */
  if (heading < 0 || kappa > 0 ||
      delta_x > r * (1.0 - std::sin(heading)) - 1e-6) {
    *result = std::numeric_limits<double>::max();
    return false;
  }

  /**
   * @brief 分段计算
   */
  if (delta_x < r * std::sin(heading)) {
    *result = std::sqrt(r * r - std::pow(delta_x - r * std::sin(heading), 2)) -
              r * std::cos(heading);
  } else {
    *result = r * (2 - std::cos(heading)) -
              std::sqrt(r * r - std::pow(delta_x - r * std::sin(heading), 2));
  }
  return true;
}

/**
 * @brief 命名空间结束标记
 *
 * C++语法说明：
 * // 注释用于说明命名空间结束
 * 三层命名空间的闭合：
 * }  // namespace util
 * }  // namespace planning
 * }  // namespace apollo
 */
}  // namespace util
}  // namespace planning
}  // namespace apollo
