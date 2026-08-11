/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
 * @file path_generation.cc
 * @brief 路径生成基类实现文件
 *
 * 本文件实现PathGeneration类，是Apollo规划模块中路径生成任务的基类。
 * 提供路径生成的通用功能，包括调试信息记录和起始点SL状态获取。
 *
 * 主要功能：
 * 1. 路径生成任务执行接口
 * 2. 调试信息记录（边界、路径数据）
 * 3. 起始点SL状态获取
 * 4. SL边界计算
 *
 * 设计特点：
 * - 作为路径生成任务的基类
 * - 提供通用的调试可视化功能
 * - 支持Frenet坐标系转换
 *
 * C++语法说明：
 * - override: 覆盖基类虚函数
 * - final: 禁止进一步覆盖
 * - mutable: 允许在const成员函数中修改
 * - std::move(): 移动语义
 * - 初始化列表: 构造对象时直接初始化成员
 */
#include "modules/planning/planning_interface_base/task_base/common/path_generation.h"

#include <string>      /**< 字符串类型 */
#include <utility>     /**< 工具函数：std::move, std::pair */
#include <vector>      /**< 动态数组容器 */

#include "modules/common/configs/vehicle_config_helper.h" /**< 车辆配置助手 */

namespace apollo {
/**
 * apollo:: - Apollo最外层命名空间
 */
namespace planning {

/**
 * @brief 执行路径生成任务
 *
 * 模板方法：先调用基类Execute执行通用流程，再调用子类Process处理具体逻辑。
 *
 * @param frame 规划帧指针
 * @param reference_line_info 参考线信息指针
 * @return Status 处理状态
 *
 * 语法说明：
 * - Frame*: 普通指针，Frame包含当前帧所有数据
 * - ReferenceLineInfo*: 参考线信息指针
 * - Task::Execute(): 调用基类方法执行通用初始化
 * - Process(): 纯虚函数，子类实现具体逻辑
 */
apollo::common::Status PathGeneration::Execute(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  /**
   * 调用基类Task的Execute方法
   * 执行通用的任务初始化逻辑
   */
  Task::Execute(frame, reference_line_info);
  /**
   * 调用子类的Process方法处理具体路径生成逻辑
   * 返回处理状态
   */
  return Process(frame, reference_line_info);
}

/**
 * @brief 重载版本：单参考线执行
 *
 * @param frame 规划帧指针（无参考线版本）
 * @return Status 处理状态
 */
apollo::common::Status PathGeneration::Execute(Frame* frame) {
  Task::Execute(frame);  /**< 调用基类单参考线版本 */
  return Process(frame);  /**< 调用子类处理 */
}

/**
 * @brief 记录路径边界调试信息
 *
 * 将路径边界转换为可视化格式，用于模拟器显示。
 * 分别记录左边界和右边界。
 *
 * @param path_boundaries 路径边界数据
 * @param debug_name 调试信息名称
 * @param reference_line_info 参考线信息指针
 *
 * 语法说明：
 * - const PathBound&: 常量引用输入，避免拷贝
 * - ReferenceLineInfo* const: 指针本身是const，不能改变指向
 * - std::vector<common::FrenetFramePoint>: Frenet帧点向量
 * - for (const Type& var : container): range-based for循环
 */
void PathGeneration::RecordDebugInfo(
    const PathBound& path_boundaries, const std::string& debug_name,
    ReferenceLineInfo* const reference_line_info) {
  /**
   * Sanity checks.
   * 健全性检查：确保输入有效
   */
  if (path_boundaries.empty()) {  /**< 检查边界是否为空 */
    AINFO << "path boundary is empty!";
    return;  /**< 为空直接返回 */
  }

  /**
   * CHECK_NOTNULL: 断言检查指针非空
   * 如果为空程序会终止并输出错误
   */
  CHECK_NOTNULL(reference_line_info);

  /**
   * Take the left and right path boundaries, and transform them into two
   * PathData so that they can be displayed in simulator.
   * 将左右路径边界转换为PathData格式用于模拟器显示
   */

  /**
   * 声明左右边界的Frenet帧点向量
   * std::vector<common::FrenetFramePoint>:
   *   - FrenetFramePoint: Frenet坐标系下的点
   *   - 包含s, l, dl, ddl等信息
   */
  std::vector<common::FrenetFramePoint> frenet_frame_left_boundaries;
  std::vector<common::FrenetFramePoint> frenet_frame_right_boundaries;

  /**
   * 遍历路径边界点
   * for (const PathBoundPoint& path_bound_point : path_boundaries)
   *   - range-based for循环（C++11）
   *   - const PathBoundPoint&: 常量引用，避免拷贝
   */
  for (const PathBoundPoint& path_bound_point : path_boundaries) {
    /**
     * 创建Frenet帧点
     */
    common::FrenetFramePoint frenet_frame_point;

    /**
     * 设置Frenet帧点的基本属性
     * frenet_frame_point.set_s(): 设置s坐标（沿参考线距离）
     * path_bound_point.s: 边界点的s坐标
     */
    frenet_frame_point.set_s(path_bound_point.s);

    /**
     * 设置横向速度dl和横向加速度ddl为0
     * 边界点是静态的，不需要速度信息
     */
    frenet_frame_point.set_dl(0.0);   /**< 横向速度 = 0 */
    frenet_frame_point.set_ddl(0.0);  /**< 横向加速度 = 0 */

    /**
     * 设置横向位置l为下界（左边界）
     * path_bound_point.l_lower.l:
     *   - l_lower: 下边界
     *   - .l: 获取边界的l值
     * push_back(): 在向量末尾添加元素
     */
    frenet_frame_point.set_l(path_bound_point.l_lower.l);
    frenet_frame_right_boundaries.push_back(frenet_frame_point);  /**< 右边界 */

    /**
     * 设置横向位置l为上界（右边界）
     * 然后添加到左边界向量
     */
    frenet_frame_point.set_l(path_bound_point.l_upper.l);
    frenet_frame_left_boundaries.push_back(frenet_frame_point);  /**< 左边界 */
  }

  /**
   * 构造Frenet帧路径
   * FrenetFramePath(std::move(...)):
   *   - 使用移动语义，避免拷贝大对象
   *   - std::move将左值转为右值引用
   *   - FrenetFramePath接收右值引用
   */
  auto frenet_frame_left_path =
      FrenetFramePath(std::move(frenet_frame_left_boundaries));
  auto frenet_frame_right_path =
      FrenetFramePath(std::move(frenet_frame_right_boundaries));

  /**
   * 构造PathData对象
   * PathData: 包含路径的所有信息
   */
  PathData left_path_data;
  /**
   * 设置参考线
   * left_path_data.SetReferenceLine():
   *   - 关联PathData与参考线
   *   - 传入指针：reference_line_info->reference_line()返回引用，&获取地址
   */
  left_path_data.SetReferenceLine(&(reference_line_info->reference_line()));
  /**
   * 设置Frenet路径
   * 使用移动语义，避免拷贝
   */
  left_path_data.SetFrenetPath(std::move(frenet_frame_left_path));

  /**
   * 同样的操作处理右边界
   */
  PathData right_path_data;
  right_path_data.SetReferenceLine(&(reference_line_info->reference_line()));
  right_path_data.SetFrenetPath(std::move(frenet_frame_right_path));

  /**
   * Insert the transformed PathData into the simulator display.
   * 将转换后的PathData插入到模拟器显示
   */

  /**
   * 获取调试数据指针
   * reference_line_info->mutable_debug():
   *   - mutable_debug(): 获取可写的调试信息
   *   - 返回Debug类指针
   * mutable_planning_data()->add_path():
   *   - 添加新的路径数据到planning_data
   *   - add_path()返回新增路径的指针
   */
  auto* ptr_display_path_1 =
      reference_line_info->mutable_debug()->mutable_planning_data()->add_path();

  /**
   * 设置路径名称
   * std::string("planning_path_boundary_1_") + debug_name:
   *   - 字符串拼接
   *   - 命名格式：planning_path_boundary_1_[debug_name]
   */
  ptr_display_path_1->set_name(std::string("planning_path_boundary_1_") +
                               debug_name);

  /**
   * 复制路径点到调试数据
   * ptr_display_path_1->mutable_path_point():
   *   - 获取可写的路径点
   * CopyFrom({...}):
   *   - 使用初始化列表构造临时对象并复制
   *   - left_path_data.discretized_path().begin()/end():
   *     获取离散路径的迭代器范围
   */
  ptr_display_path_1->mutable_path_point()->CopyFrom(
      {left_path_data.discretized_path().begin(),
       left_path_data.discretized_path().end()});

  /**
   * 同样的操作处理第二条边界路径
   */
  auto* ptr_display_path_2 =
      reference_line_info->mutable_debug()->mutable_planning_data()->add_path();
  ptr_display_path_2->set_name(std::string("planning_path_boundary_2_") +
                               debug_name);
  ptr_display_path_2->mutable_path_point()->CopyFrom(
      {right_path_data.discretized_path().begin(),
       right_path_data.discretized_path().end()});
}

/**
 * @brief 重载版本：记录路径数据调试信息
 *
 * 将PathData直接复制到调试数据用于显示。
 *
 * @param path_data 路径数据
 * @param debug_name 调试信息名称
 * @param reference_line_info 参考线信息指针
 */
void PathGeneration::RecordDebugInfo(
    const PathData& path_data, const std::string& debug_name,
    ReferenceLineInfo* const reference_line_info) {
  /**
   * 获取路径点
   * const auto& path_points = path_data.discretized_path():
   *   - discretized_path(): 获取离散路径点
   *   - const auto&: 常量引用，避免拷贝
   */
  const auto& path_points = path_data.discretized_path();

  /**
   * 添加新路径到调试数据
   */
  auto* ptr_optimized_path =
      reference_line_info->mutable_debug()->mutable_planning_data()->add_path();

  /**
   * 设置路径名称：candidate_path_[debug_name]
   */
  ptr_optimized_path->set_name(std::string("candidate_path_") + debug_name);

  /**
   * 使用初始化列表复制路径点
   * {path_points.begin(), path_points.end()}:
   *   - 构造临时vector并复制范围内的元素
   */
  ptr_optimized_path->mutable_path_point()->CopyFrom(
      {path_points.begin(), path_points.end()});
}

/**
 * @brief 获取起始点SL状态
 *
 * 将规划起始点转换到Frenet坐标系（SL坐标系）。
 * 如果使用前轴中心规划，会将起始点从前轴中心转换到后轴中心。
 *
 * 转换公式：
 * x_new = x + wheel_base * cos(theta)
 * y_new = y + wheel_base * sin(theta)
 *
 * 语法说明：
 * - void: 没有返回值，结果存储在成员变量init_sl_state_中
 * - FLAGS_use_front_axe_center_in_path_planning: 配置标志
 */
void PathGeneration::GetStartPointSLState() {
  /**
   * 获取参考线引用
   * reference_line_info_->reference_line():
   *   - reference_line_info_: 类成员（Task基类）
   *   - reference_line(): 获取参考线
   *   - const ReferenceLine&: 常量引用
   */
  const ReferenceLine& reference_line = reference_line_info_->reference_line();

  /**
   * 获取规划起始点
   * frame_->PlanningStartPoint():
   *   - frame_: 类成员，规划帧指针
   *   - PlanningStartPoint(): 返回TrajectoryPoint类型
   * common::TrajectoryPoint: 轨迹点类型，包含路径点+速度+时间
   */
  common::TrajectoryPoint planning_start_point = frame_->PlanningStartPoint();

  /**
   * 检查是否使用前轴中心进行路径规划
   * FLAGS_use_front_axe_center_in_path_planning:
   *   - GFLAGS变量，配置是否使用前轴中心
   *   - 如果为true，需要将起始点从后轴中心转移到前轴中心
   */
  if (FLAGS_use_front_axe_center_in_path_planning) {
    /**
     * 获取前后轴距离
     * apollo::common::VehicleConfigHelper::GetConfig():
     *   - 获取车辆配置单例
     * .vehicle_param().wheel_base():
     *   - vehicle_param(): 获取车辆参数
     *   - wheel_base(): 前后轴距离（wheelbase）
     */
    double front_to_rear_axe_distance =
        apollo::common::VehicleConfigHelper::GetConfig()
            .vehicle_param()
            .wheel_base();

    /**
     * 计算新的x坐标
     * planning_start_point.mutable_path_point()->set_x(...):
     *   - mutable_path_point(): 获取可写的PathPoint
     *   - set_x(): 设置x坐标
     * planning_start_point.path_point().x():
     *   - path_point(): 获取常量PathPoint引用
     *   - x(): 获取x坐标
     * std::cos(...): 余弦函数
     */
     // x_new = x + wheel_base * cos(theta)
    planning_start_point.mutable_path_point()->set_x(
        planning_start_point.path_point().x() +
        front_to_rear_axe_distance *
            std::cos(planning_start_point.path_point().theta()));

    /**
     * 计算新的y坐标
     * std::sin(...): 正弦函数
     */
     // y_new = y + wheel_base * sin(theta)
    planning_start_point.mutable_path_point()->set_y(
        planning_start_point.path_point().y() +
        front_to_rear_axe_distance *
            std::sin(planning_start_point.path_point().theta()));
  }

  /**
   * 打印调试信息
   * std::fixed: 固定小数点格式
   * planning_start_point.path_point().x()/y()/theta():
   *   - 获取路径点的坐标和航向角
   */
  AINFO << std::fixed << "Plan at the starting point: x = "
        << planning_start_point.path_point().x()
        << ", y = " << planning_start_point.path_point().y()
        << ", and angle = " << planning_start_point.path_point().theta();

  /**
   * Initialize some private variables.
   * ADC s/l info.
   * 将规划起始点转换到Frenet坐标系
   *
   * reference_line.ToFrenetFrame(planning_start_point):
   *   - ToFrenetFrame(): 参考线方法，将XY坐标系点转换到SL坐标系
   *   - 返回pair<s, l>表示沿参考线的距离和横向偏移
   * init_sl_state_: 类成员变量，存储转换后的SL状态
   */
  init_sl_state_ = reference_line.ToFrenetFrame(planning_start_point);
}

/**
 * @brief 获取指定点的SL边界
 *
 * 根据路径数据和指定点索引，计算自车在该位置的SL边界。
 * 用于判断自车与障碍物的相对位置关系。
 *
 * @param path_data 路径数据
 * @param point_index 路径点索引
 * @param reference_line_info 参考线信息指针
 * @param sl_boundary 输出：SL边界
 * @return bool 是否成功计算
 *
 * 语法说明：
 * - const PathData&: 常量引用输入
 * - int point_index: 点索引
 * - SLBoundary* const: 指针本身是const
 */
bool PathGeneration::GetSLBoundary(const PathData& path_data, int point_index,
                                   const ReferenceLineInfo* reference_line_info,
                                   SLBoundary* const sl_boundary) {
  CHECK_NOTNULL(sl_boundary);  /**< 断言检查输出参数非空 */

  /**
   * 获取离散路径
   * const auto& discrete_path = path_data.discretized_path():
   *   - discretized_path(): 返回离散路径点向量
   */
  const auto& discrete_path = path_data.discretized_path();

  /**
   * 检查索引有效性
   * point_index < 0: 负数索引无效
   * static_cast<size_t>(point_index) > discrete_path.size():
   *   - 将int转为size_t进行比较
   *   - 索引不能等于或超过向量大小
   */
  if (point_index < 0 ||
      static_cast<size_t>(point_index) > discrete_path.size()) {
    return false;  /**< 索引无效返回false */
  }

  /**
   * 清空SL边界点
   * sl_boundary->mutable_boundary_point()->Clear():
   *   - mutable_boundary_point(): 获取可写的边界点
   *   - Clear(): 清空所有点
   */
  sl_boundary->mutable_boundary_point()->Clear();

  /**
   * Get vehicle config parameters.
   * 获取车辆配置参数
   *
   * common::VehicleConfigHelper::Instance()->GetConfig():
   *   - Instance(): 获取单例实例
   *   - GetConfig(): 获取配置
   */
  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();

  /**
   * 获取车辆尺寸参数
   * vehicle_config.vehicle_param().length():
   *   - length: 车辆长度
   * width: 车辆宽度
   * back_edge_to_center: 后边缘到车辆中心的距离
   */
  const double ego_length = vehicle_config.vehicle_param().length();
  const double ego_width = vehicle_config.vehicle_param().width();
  const double ego_back_to_center =
      vehicle_config.vehicle_param().back_edge_to_center();

  /**
   * 计算车辆中心相对于后轴中心的偏移距离
   * ego_center_shift_distance = 车辆长度/2 - 后边缘到中心距离
   * 例如：车辆长5m，后边缘到中心2m，则中心偏移(5/2 - 2) = 0.5m
   */
  const double ego_center_shift_distance =
      ego_length / 2.0 - ego_back_to_center;

  /**
   * Generate vehicle bounding box.
   * 生成车辆包围盒
   */

  /**
   * 获取指定索引处的路径点（后轴中心）
   * discrete_path[point_index]:
   *   - 下标操作符访问向量元素
   *   - 返回PathPoint引用
   */
  const auto& rear_center_path_point = discrete_path[point_index];

  /**
   * 获取车辆航向角
   * rear_center_path_point.theta():
   *   - theta: 航向角（弧度）
   */
  const double ego_theta = rear_center_path_point.theta();

  /**
   * 创建车辆二维包围盒
   * common::math::Box2d:
   *   - 二维包围盒类
   *   - 参数：中心点(x, y), 航向角, 长度, 宽度
   *   - 包围盒用于碰撞检测和SL边界计算
   */
  common::math::Box2d ego_box(
      {rear_center_path_point.x(), rear_center_path_point.y()}, ego_theta,
      ego_length, ego_width);

  /**
   * 计算包围盒中心偏移向量
   * ego_center_shift_distance * ego_box.cos_heading():
   *   - cos_heading(): 航向角的余弦值
   *   - ego_box.sin_heading(): 航向角的正弦值
   * common::math::Vec2d:
   *   - 二维向量类
   */
  common::math::Vec2d shift_vec{
      ego_center_shift_distance * ego_box.cos_heading(),
      ego_center_shift_distance * ego_box.sin_heading()};

  /**
   * 移动包围盒到几何中心位置
   * ego_box.Shift(shift_vec):
   *   - 将包围盒从后轴中心移动到几何中心
   *   - 几何中心 = 后轴中心 + 偏移向量
   */
  ego_box.Shift(shift_vec);

  /**
   * Set warm_start_s near the geometry center of the box.
   * 设置SL边界计算的初始s值
   *
   * path_data.frenet_frame_path()[point_index].s():
   *   - frenet_frame_path(): 获取Frenet路径
   *   - [point_index]: 获取指定点的Frenet点
   *   - .s(): 获取该点的s坐标
   * + ego_center_shift_distance:
   *   - 加上中心偏移，得到几何中心对应的s
   */
  double warm_start_s = path_data.frenet_frame_path()[point_index].s() +
                        ego_center_shift_distance;

  /**
   * Get the SL boundary of vehicle.
   * 计算车辆的SL边界
   *
   * reference_line_info->reference_line().GetSLBoundary(ego_box, sl_boundary, warm_start_s):
   *   - GetSLBoundary(): 参考线方法，将XY包围盒转换为SL边界
   *   - ego_box: 车辆的XY坐标系包围盒
   *   - sl_boundary: 输出参数，SL边界
   *   - warm_start_s: 初始s值，帮助算法更快收敛
   */
  reference_line_info->reference_line().GetSLBoundary(ego_box, sl_boundary,
                                                      warm_start_s);
  return true;  /**< 计算成功 */
}

}  // namespace planning
}  // namespace apollo
