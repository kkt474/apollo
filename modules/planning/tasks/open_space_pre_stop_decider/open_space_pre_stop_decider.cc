/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
 * @file open_space_pre_stop_decider.cc
 * @brief 开放空间预停车决策器实现文件
 *
 * 功能说明：
 * 在开放空间规划（如自主泊车）场景中，计算预停车位置
 * 预停车是指车辆在到达最终目标（如停车位）之前需要先停下的位置
 *
 * 应用场景：
 * - 自主泊车(Valet Parking)
 * - 靠边停车(Pull Over)
 * - 停车场导航
 *
 * 停车类型：
 * - PARKING：停车位停车
 * - PULL_OVER：靠边停车
 *
 * 工作流程：
 * 1. Init() - 初始化，加载配置
 * 2. Process() - 根据停车类型选择处理函数
 * 3. CheckParkingSpotPreStop() / CheckPullOverPreStop() - 检查预停车条件
 * 4. SetParkingSpotStopFence() / SetPullOverStopFence() - 设置停车围栏
 */

/**
 * @brief 当前文件的头文件
 *
 * C++语法说明：
 * - "modules/planning/tasks/open_space_pre_stop_decider/open_space_pre_stop_decider.h"：
 *   使用引号表示自定义头文件
 *   编译器从当前文件目录开始搜索
 *   定义了OpenSpacePreStopDecider类的接口
 */
#include "modules/planning/tasks/open_space_pre_stop_decider/open_space_pre_stop_decider.h"

/**
 * @brief C++标准库头文件
 *
 * C++语法说明：
 * - #include <memory>：
 *   智能指针头文件，提供std::shared_ptr等
 *
 * - #include <string>：
 *   标准库字符串类
 *
 * - #include <vector>：
 *   动态数组容器
 */
#include <memory>
#include <string>
#include <vector>

/**
 * @brief Apollo车辆状态提供者头文件
 *
 * 功能说明：
 * - VehicleStateProvider类：
 *   提供车辆实时状态（位置、速度、加速度等）
 *
 * C++语法说明：
 * - injector_->vehicle_state()：
 *   通过依赖注入器获取车辆状态服务
 */
#include "modules/common/vehicle_state/vehicle_state_provider.h"

/**
 * @brief Apollo PNC地图路径头文件
 *
 * 功能说明：
 * - apollo::hdmap::Path：
 *   地图路径类
 *   提供路径查询、投影等功能
 */
#include "modules/map/pnc_map/path.h"

/**
 * @brief 规划上下文头文件
 *
 * 功能说明：
 * - planning_context.h：
 *   包含规划上下文定义
 *   用于存储跨帧的规划状态
 *   如pull_over状态等
 */
#include "modules/planning/planning_base/common/planning_context.h"

/**
 * @brief 通用工具头文件
 *
 * 功能说明：
 * - common.h：
 *   提供通用工具函数
 *   如BuildStopDecision()用于构建停车决策
 */
#include "modules/planning/planning_base/common/util/common.h"

/**
 * @namespace apollo::planning
 * @brief Apollo规划模块命名空间
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
 *   在大型项目中常用此方式简化长类型名
 *
 * - apollo::common::ErrorCode：
 *   Apollo错误码枚举
 *
 * - apollo::common::Status：
 *   Apollo状态类，包含错误码和消息
 *
 * - apollo::common::VehicleState：
 *   车辆状态类
 *
 * - apollo::common::math::Vec2d：
 *   2D向量类，用于几何计算
 *
 * - apollo::hdmap::ParkingSpaceInfoConstPtr：
 *   停车场位信息常量指针
 */
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleState;
using apollo::common::math::Vec2d;
using apollo::hdmap::ParkingSpaceInfoConstPtr;

/**
 * @brief 初始化开放空间预停车决策器
 *
 * @param config_dir 配置文件目录
 * @param name 任务名称
 * @param injector 依赖注入器指针
 * @return bool 初始化是否成功
 *
 * 功能说明：
 * 初始化决策器的内部状态和配置
 *
 * 算法流程：
 * 1. 调用基类Decider的Init方法进行基础初始化
 * 2. 加载OpenSpacePreStopDeciderConfig配置
 * 3. 输出配置调试信息
 *
 * C++语法说明：
 * - const std::string& config_dir：
 *   常量引用，传入配置文件目录路径
 *   const保证不会被修改，引用避免拷贝
 *
 * - const std::string& name：
 *   常量引用，传入任务名称
 *
 * - const std::shared_ptr<DependencyInjector>& injector：
 *   共享指针的常量引用
 *   - shared_ptr：多个所有者共享对象
 *   - const引用：既不拷贝也不获得所有权
 *
 * - if (!Decider::Init(...))：
 *   作用域解析运算符::调用基类的Init方法
 *   逻辑非运算符!判断返回值
 *
 * - Decider::LoadConfig<T>(&config_)：
 *   模板方法，从配置文件加载指定类型的配置
 *   T = OpenSpacePreStopDeciderConfig
 *   &config_传入配置存储地址
 *
 * - config_.DebugString()：
 *   Protobuf消息的调试字符串方法
 *   返回消息的可读文本表示
 */
bool OpenSpacePreStopDecider::Init(
    const std::string& config_dir, const std::string& name,
    const std::shared_ptr<DependencyInjector>& injector) {
  // 调用基类Decider的Init方法进行基础初始化
  if (!Decider::Init(config_dir, name, injector)) {
    return false;  // 基类初始化失败，返回false
  }

  // 加载开放空间预停车决策器的配置
  bool res = Decider::LoadConfig<OpenSpacePreStopDeciderConfig>(&config_);

  // 输出配置调试信息
  // AINFO是Apollo信息日志宏
  // config_.DebugString()返回Protobuf消息的调试字符串
  AINFO << "Load config:" << config_.DebugString();

  return res;  // 返回配置加载结果
}

/**
 * @brief 处理预停车决策
 *
 * @param frame 规划帧数据
 * @param reference_line_info 参考线信息
 * @return Status 处理状态
 *
 * 功能说明：
 * 根据配置中的停车类型，选择相应的处理函数
 *
 * 停车类型：
 * - PARKING：停车位停车
 * - PULL_OVER：靠边停车
 *
 * C++语法说明：
 * - Frame*：
 *   原始指针，指向规划帧
 *   使用指针因为frame需要被修改
 *
 * - ReferenceLineInfo*：
 *   参考线信息指针
 *   用于访问和修改参考线相关信息
 *
 * - CHECK_NOTNULL(pointer)：
 *   断言宏，确保指针非空
 *   如果为空，程序会报错并终止
 *
 * - switch(stop_type)：
 *   switch语句根据停车类型选择分支
 *   stop_type是OpenSpacePreStopDeciderConfig::StopType枚举
 *
 * - case OpenSpacePreStopDeciderConfig::PARKING：
 *   停车位停车分支
 *
 * - case OpenSpacePreStopDeciderConfig::PULL_OVER：
 *   靠边停车分支
 *
 * - default:
 *   默认分支，处理未知停车类型
 */
Status OpenSpacePreStopDecider::Process(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  // 断言检查指针非空
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // 初始化目标s坐标为0
  double target_s = 0.0;

  // 从配置中获取停车类型
  const auto& stop_type = config_.stop_type();

  // 根据停车类型选择处理分支
  switch (stop_type) {
    // 停车位停车
    case OpenSpacePreStopDeciderConfig::PARKING:
      // 检查停车位预停车条件
      // taget_s: 后轴中心所处的位置
      if (!CheckParkingSpotPreStop(frame, reference_line_info, &target_s)) {
        // 检查失败，构造错误消息
        const std::string msg = "Checking parking spot pre stop fails";
        AERROR << msg;  // 输出错误日志
        // 返回错误状态，携带错误码和消息
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }
      // 检查成功，设置停车位停车围栏
      SetParkingSpotStopFence(target_s, frame, reference_line_info);
      break;  // 跳出switch

    // 靠边停车
    case OpenSpacePreStopDeciderConfig::PULL_OVER:
      // 检查靠边停车预停车条件
      if (!CheckPullOverPreStop(frame, reference_line_info, &target_s)) {
        const std::string msg = "Checking pull over pre stop fails";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }
      // 设置靠边停车围栏
      SetPullOverStopFence(target_s, frame, reference_line_info);
      break;

    // 默认分支：未知停车类型
    default:
      const std::string msg = "This stop type not implemented";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 处理成功，返回OK状态
  return Status::OK();
}

/**
 * @brief 检查靠边停车预停车条件
 *
 * @param frame 规划帧数据
 * @param reference_line_info 参考线信息
 * @param target_s 输出：目标s坐标
 * @return bool 检查是否成功
 *
 * 功能说明：
 * 检查靠边停车的预停车条件
 * 从规划上下文中获取靠边停车目标位置
 *
 * 算法流程：
 * 1. 获取靠边停车状态
 * 2. 检查是否有有效的目标位置
 * 3. 将XY坐标转换为SL坐标
 * 4. 输出目标s坐标
 *
 * C++语法说明：
 * - Frame* const frame：
 *   指向常量的指针
 *   - Frame*：指针类型
 *   - const：指针指向的内容不可修改
 *   这表示函数不会通过此指针修改frame指向的内容
 *
 * - ReferenceLineInfo* const reference_line_info：
 *   同样是指向常量的指针
 *
 * - double* target_s：
 *   指针输出参数
 *   函数内通过解引用赋值输出结果
 *
 * - *target_s = 0.0：
 *   解引用指针并赋值
 *
 * - injector_->planning_context()->planning_status().pull_over()：
 *   链式调用：
 *   - injector_->：依赖注入器指针
 *   - planning_context()：获取规划上下文
 *   - planning_status()：获取规划状态
 *   - pull_over()：获取靠边停车状态
 *
 * - pull_over_status.has_position()：
 *   Protobuf的has_方法检查字段是否存在
 *
 * - pull_over_status.position().has_x()：
 *   链式has检查嵌套字段
 *
 * - reference_line.XYToSL(...)：
 *   将笛卡尔坐标(XY)转换为SL坐标
 *   - 输入：pull_over_status.position()
 *   - 输出：pull_over_sl
 */
bool OpenSpacePreStopDecider::CheckPullOverPreStop(
    Frame* const frame, ReferenceLineInfo* const reference_line_info,
    double* target_s) {
  // 初始化目标s为0
  *target_s = 0.0;

  // 获取靠边停车状态
  const auto& pull_over_status =
      injector_->planning_context()->planning_status().pull_over();

  // 检查是否有有效的目标位置
  if (pull_over_status.has_position() && pull_over_status.position().has_x() &&
      pull_over_status.position().has_y()) {
    // 创建SL点对象
    common::SLPoint pull_over_sl;

    // 获取参考线
    const auto& reference_line = reference_line_info->reference_line();

    // 将XY坐标转换为SL坐标
    reference_line.XYToSL(pull_over_status.position(), &pull_over_sl);

    // 输出目标s坐标
    *target_s = pull_over_sl.s();
  }

  return true;  // 检查成功
}

/**
 * @brief 检查停车位预停车条件
 *
 * @param frame 规划帧数据
 * @param reference_line_info 参考线信息
 * @param target_s 输出：目标s坐标
 * @return bool 检查是否成功
 *
 * 功能说明：
 * 检查停车位的预停车条件
 * 从地图路径中找到目标停车位并计算其中心点
 *
 * 算法流程：
 * 1. 获取目标停车位ID
 * 2. 在路径上搜索目标停车位
 * 3. 获取停车位多边形角点
 * 4. 计算停车位中心点
 * 5. 将中心点投影到路径获取s坐标
 *
 * C++语法说明：
 * - frame->open_space_info()：
 *   ->调用规划帧的方法
 *   open_space_info返回开放空间信息
 *
 * - .target_parking_spot_id()：
 *   获取目标停车位ID
 *
 * - reference_line_info->reference_line().map_path()：
 *   ->调用获取参考线，再调用获取地图路径
 *
 * - nearby_path.parking_space_overlaps()：
 *   获取路径上的停车位重叠区域列表
 *
 * - for (const auto& parking_overlap : parking_space_overlaps)：
 *   范围for循环遍历所有停车位重叠区域
 *   const auto&避免拷贝，提高效率
 *
 * - parking_overlap.object_id == target_parking_spot_id：
 *   比较停车位ID
 *
 * - hdmap::Id id; id.set_id(...)：
 *   创建地图ID并设置值
 *
 * - hdmap->GetParkingSpaceById(id)：
 *   通过ID从地图获取停车位信息
 *
 * - target_parking_spot_ptr->polygon().points().at(0)：
 *   ->polygon()获取多边形
 *   ->points()获取顶点列表
 *   ->at(0)获取第一个顶点（带边界检查）
 *
 * - Vec2d center_point = (left_bottom_point + ... + left_up_point) / 4.0：
 *   向量加法和标量除法
 *   计算四个角点的平均值得到中心点
 *
 * - nearby_path.GetNearestPoint(center_point, &target_area_center_s, &center_l)：
 *   将点投影到路径上
 *   获取该点在路径上的s（沿路径距离）和l（横向距离）坐标
 */
bool OpenSpacePreStopDecider::CheckParkingSpotPreStop(
    Frame* const frame, ReferenceLineInfo* const reference_line_info,
    double* target_s) {
  // 获取目标停车位ID
  const auto& target_parking_spot_id =
      frame->open_space_info().target_parking_spot_id();

  // 获取参考线对应的地图路径
  const auto& nearby_path = reference_line_info->reference_line().map_path();

  // 检查目标停车位ID是否有效
  if (target_parking_spot_id.empty()) {
    AERROR << "no target parking spot id found when setting pre stop fence";
    return false;  // ID无效，返回失败
  }

  // 初始化目标区域中心s坐标
  double target_area_center_s = 0.0;
  bool target_area_found = false;  // 标记是否找到目标

  // 获取路径上的所有停车位重叠区域
  const auto& parking_space_overlaps = nearby_path.parking_space_overlaps();

  // 停车场位信息指针
  ParkingSpaceInfoConstPtr target_parking_spot_ptr;

  // 获取HD地图指针
  const hdmap::HDMap* hdmap = hdmap::HDMapUtil::BaseMapPtr();

  // 遍历所有停车位重叠区域
  for (const auto& parking_overlap : parking_space_overlaps) {
    // 检查是否是目标停车位
    if (parking_overlap.object_id == target_parking_spot_id) {
      // TODO(Jinyun) parking overlap s are wrong on map, not usable
      // 注意：地图上的parking overlap s值有误，暂不使用
      // target_area_center_s =
      //     (parking_overlap.start_s + parking_overlap.end_s) / 2.0;

      // 创建停车位ID
      hdmap::Id id;
      id.set_id(parking_overlap.object_id);

      // 通过ID获取停车位详细信息
      target_parking_spot_ptr = hdmap->GetParkingSpaceById(id);

      // 获取停车位多边形的四个角点
      // 假设多边形按顺时针或逆时针顺序存储四个顶点
      Vec2d left_bottom_point =
          target_parking_spot_ptr->polygon().points().at(0);  // 左下角
      Vec2d right_bottom_point =
          target_parking_spot_ptr->polygon().points().at(1);  // 右下角
      Vec2d right_up_point =
          target_parking_spot_ptr->polygon().points().at(2);  // 右上角
      Vec2d left_up_point =
          target_parking_spot_ptr->polygon().points().at(3);  // 左上角

      // 计算停车位的几何中心点
      // 通过四个角点的坐标平均值得到中心点
      Vec2d center_point = (left_bottom_point + right_bottom_point +
                            right_up_point + left_up_point) /
                           4.0;

      double center_l;  // 横向距离（输出）
      // 将中心点投影到路径上，获取s和l坐标
      nearby_path.GetNearestPoint(center_point, &target_area_center_s,
                                  &center_l);

      // 标记找到目标
      target_area_found = true;
    }
  }

  // 检查是否找到目标停车位
  if (!target_area_found) {
    AERROR << "no target parking spot found on reference line";
    return false;  // 未找到，返回失败
  }

  // 输出目标s坐标
  *target_s = target_area_center_s;
  return true;  // 检查成功
}

/**
 * @brief 设置停车位停车围栏
 *
 * @param target_s 目标s坐标（停车位中心）
 * @param frame 规划帧数据
 * @param reference_line_info 参考线信息
 *
 * 功能说明：
 * 计算并设置停车位的预停车围栏位置
 *
 * 算法流程：
 * 1. 获取车辆前边缘位置
 * 2. 计算停车线位置 = 目标s + 前边缘到中心距离 + 停车缓冲距离
 * 3. 保存停车线s坐标到规划帧
 * 4. 调用BuildStopDecision构建停车决策
 *
 * C++语法说明：
 * - const double target_s：
 *   const double类型，传入的目标s坐标
 *
 * - reference_line_info->AdcSlBoundary()：
 *   获取自车(SL)边界框
 *   AdcSlBoundary返回自车在SL坐标系下的边界
 *
 * - .end_s()：
 *   边界框在s方向的最大值（前端）
 *
 * - common::VehicleConfigHelper::Instance()->GetConfig()：
 *   单例模式获取车辆配置
 *   ->GetConfig()获取配置消息
 *
 * - .vehicle_param()：
 *   获取车辆参数子消息
 *
 * - .front_edge_to_center()：
 *   获取车辆前边缘到中心的距离
 *
 * - adc_front_edge_s - front_edge_to_center：
 *   计算车辆中心在参考线上的s坐标
 *
 * - config_.stop_distance_to_target()：
 *   从配置获取到目标的停车距离
 *
 * - static constexpr double kStopBuffer = 0.2：
 *   static：静态常量，类内共享
 *   constexpr：编译时常量
 *   kStopBuffer：停车缓冲距离常量
 *
 * - CHECK_GE(a, b)：
 *   断言宏，检查 a >= b
 *   如果不满足，程序报错并终止
 *
 * - stop_line_s = target_s + front_edge_to_center + config_.stop_buffer_to_target()：
 *   计算停车线s坐标
 *   = 目标中心 + 前边缘到中心 + 缓冲距离
 *
 * - OPEN_SPACE_STOP_ID：
 *   开放空间停止墙的ID常量
 *
 * - util::BuildStopDecision(...)：
 *   工具函数，构建停车决策
 *   参数：停止墙ID、停止线s坐标、停止距离、停止原因等
 */
void OpenSpacePreStopDecider::SetParkingSpotStopFence(
    const double target_s, Frame* const frame,
    ReferenceLineInfo* const reference_line_info) {
  // 获取参考线对应的地图路径
  const auto& nearby_path = reference_line_info->reference_line().map_path();

  // 获取自车前边缘在参考线上的s坐标
  const double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();

  // 获取车辆前边缘到中心的距离
  const double front_edge_to_center = common::VehicleConfigHelper::Instance()
                                          ->GetConfig()
                                          .vehicle_param()
                                          .front_edge_to_center();

  // 计算自车中心在参考线上的s坐标
  double ego_s = adc_front_edge_s - front_edge_to_center;

  // 获取车辆状态（当前未使用，但保留接口）
  const VehicleState& vehicle_state = frame->vehicle_state();

  // 初始化停车线s坐标
  double stop_line_s = 0.0;

  // 从配置获取到目标的停车距离
  double stop_distance_to_target = config_.stop_distance_to_target();

  // 静态速度epsilon，用于判断是否静止
  double static_linear_velocity_epsilon = 1.0e-2;

  // 静态停车缓冲常量
  static constexpr double kStopBuffer = 0.2;

  // 检查停车距离是否有效
  CHECK_GE(stop_distance_to_target, 1.0e-8);

  // 计算停车线s坐标
  // stop_line_s = target_s + front_edge_to_center + stop_buffer
  // 这样停车线会比目标中心靠前一个前边缘到中心的距离
  stop_line_s =
      target_s + front_edge_to_center + config_.stop_buffer_to_target();

  // 停止墙ID
  const std::string stop_wall_id = OPEN_SPACE_STOP_ID;

  // 等待让行的障碍物列表（初始为空）
  std::vector<std::string> wait_for_obstacles;

  // 保存停车线s坐标到规划帧
  frame->mutable_open_space_info()->set_open_space_pre_stop_fence_s(
      stop_line_s);

  // 构建停车决策
  // util::BuildStopDecision参数：
  // - stop_wall_id：停止墙ID
  // - stop_line_s：停车线s坐标
  // - 0.0：停车距离（这里设为0）
  // - STOP_REASON_PRE_OPEN_SPACE_STOP：停车原因
  // - wait_for_obstacles：等待让行的障碍物列表
  // - "OpenSpacePreStopDecider"：决策器名称
  // - frame：规划帧
  // - reference_line_info：参考线信息
  util::BuildStopDecision(stop_wall_id, stop_line_s, 0.0,
                          StopReasonCode::STOP_REASON_PRE_OPEN_SPACE_STOP,
                          wait_for_obstacles, "OpenSpacePreStopDecider", frame,
                          reference_line_info);
}

/**
 * @brief 设置靠边停车围栏
 *
 * @param target_s 目标s坐标
 * @param frame 规划帧数据
 * @param reference_line_info 参考线信息
 *
 * 功能说明：
 * 计算并设置靠边停车的预停车围栏位置
 * 包含两种情况：
 * 1. 目标距离足够，正常靠边停车
 * 2. 目标距离不足，需要紧急停车
 *
 * 算法流程：
 * 1. 计算目标与车辆的距离差
 * 2. 如果距离充足，设置停车线在目标前stop_distance
 * 3. 如果距离不足，根据速度决定停车位置
 *
 * C++语法说明：
 * - target_vehicle_offset = target_s - adc_front_edge_s：
 *   计算目标与车辆前边缘的距离差
 *   正值表示目标在车辆前方
 *
 * - if (target_vehicle_offset > stop_distance_to_target)：
 *   判断目标距离是否充足
 *
 * - frame->open_space_info().pre_stop_rightaway_flag()：
 *   获取预停车紧急标志
 *   true表示需要紧急停车
 *
 * - frame->mutable_open_space_info()->mutable_pre_stop_rightaway_point()：
 *   mutable_前缀返回可修改指针
 *   用于设置预停车点
 *
 * - nearby_path.GetSmoothPoint(stop_line_s)：
 *   获取路径上指定s坐标的光滑点
 *
 * - std::abs(vehicle_state.linear_velocity()) < static_linear_velocity_epsilon：
 *   判断车辆是否静止
 *   使用绝对值比较速度是否接近0
 *
 * - nearby_path.GetNearestPoint(point, &stop_point_s, &stop_point_l)：
 *   将点投影到路径获取SL坐标
 */
void OpenSpacePreStopDecider::SetPullOverStopFence(
    const double target_s, Frame* const frame,
    ReferenceLineInfo* const reference_line_info) {
  // 获取参考线对应的地图路径
  const auto& nearby_path = reference_line_info->reference_line().map_path();

  // 获取自车前边缘在参考线上的s坐标
  const double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();

  // 获取车辆状态
  const VehicleState& vehicle_state = frame->vehicle_state();

  // 初始化停车线s坐标
  double stop_line_s = 0.0;

  // 从配置获取到目标的停车距离
  double stop_distance_to_target = config_.stop_distance_to_target();   // 5.0

  // 静态速度epsilon，用于判断是否静止
  double static_linear_velocity_epsilon = 1.0e-2;

  // 检查停车距离是否有效
  CHECK_GE(stop_distance_to_target, 1.0e-8);

  // 计算目标与车辆前边缘的距离差
  double target_vehicle_offset = target_s - adc_front_edge_s;

  // 判断目标距离是否充足
  if (target_vehicle_offset > stop_distance_to_target) {
    // 距离充足，正常靠边停车
    // 停车线设置在目标前方stop_distance处
    stop_line_s = target_s - stop_distance_to_target;
  } else {
    // 距离不足，需要判断是否已经发出紧急停车标志
    if (!frame->open_space_info().pre_stop_rightaway_flag()) {
      // TODO(Jinyun) Use constant comfortable deacceleration rather than
      // distance by config to set stop fence
      // 注释：应该使用舒适的恒定减速度来设置停车围栏，而不是配置的距离

      // 计算紧急停车位置
      stop_line_s = adc_front_edge_s + config_.rightaway_stop_distance();

      // 如果车辆已经静止，停车线直接设为当前位置
      if (std::abs(vehicle_state.linear_velocity()) <
          static_linear_velocity_epsilon) {
        stop_line_s = adc_front_edge_s;
      }

      // 保存预停车点到规划帧
      *(frame->mutable_open_space_info()->mutable_pre_stop_rightaway_point()) =
          nearby_path.GetSmoothPoint(stop_line_s);

      // 设置紧急停车标志为true
      frame->mutable_open_space_info()->set_pre_stop_rightaway_flag(true);
    } else {
      // 已经发出过紧急停车，使用之前的预停车点
      double stop_point_s = 0.0;
      double stop_point_l = 0.0;

      // 将预停车点投影到路径获取SL坐标
      nearby_path.GetNearestPoint(
          frame->open_space_info().pre_stop_rightaway_point(), &stop_point_s,
          &stop_point_l);

      // 使用投影后的s坐标
      stop_line_s = stop_point_s;
    }
  }

  // 停止墙ID
  const std::string stop_wall_id = OPEN_SPACE_STOP_ID;

  // 等待让行的障碍物列表（初始为空）
  std::vector<std::string> wait_for_obstacles;

  // 保存停车线s坐标到规划帧
  frame->mutable_open_space_info()->set_open_space_pre_stop_fence_s(
      stop_line_s);

  // 构建停车决策
  util::BuildStopDecision(stop_wall_id, stop_line_s, 0.0,
                          StopReasonCode::STOP_REASON_PRE_OPEN_SPACE_STOP,
                          wait_for_obstacles, "OpenSpacePreStopDecider", frame,
                          reference_line_info);
}

/**
 * @namespace命名空间结束
 *
 * C++语法说明：
 * - }  // namespace planning：
 *   结束planning命名空间
 * - }  // namespace apollo：
 *   结束apollo命名空间
 */
}  // namespace planning
}  // namespace apollo