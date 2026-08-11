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
 * @file on_lane_planning.cc
 * @brief 车道线模式规划器实现文件
 *
 * 本文件是Apollo基于车道线的规划模式（OnLanePlanning）的核心实现。
 * 该模式依赖高清地图（HDMap）的车道线信息进行路径规划。
 *
 * 核心功能：
 * 1. 基于ReferenceLine（参考线）进行路径和速度规划
 * 2. 处理交通规则（红绿灯、停车标志等）
 * 3. 轨迹缝合：将历史轨迹与新规划轨迹连接
 * 4. 支持开放空间规划（泊车等场景）
 *
 * 语法说明：
 * - std::list<T>: 标准模板库双向链表容器
 * - std::unique_ptr<T>: 独占所有权智能指针
 * - std::vector<T>: 标准模板库动态数组容器
 * - mutable_xxx(): Protobuf消息的可写访问器
 * - static_cast<T>(x): C++静态类型转换
 * - std::chrono: C++11时间库，用于高精度时间测量
 * - auto&: 引用类型的自动推断
 * - std::back_inserter: 用于将元素追加到容器末尾的迭代器适配器
 */

#include "modules/planning/planning_component/on_lane_planning.h"

/**
 * #include <algorithm> - C++标准库算法头文件
 * 提供std::min, std::max, std::copy, std::fill等算法函数
 */
#include <algorithm>

/**
 * #include <limits> - C++标准库数值极限头文件
 * 提供std::numeric_limits<T>用于获取类型的数值范围（如最大值、最小值）
 */
#include <limits>

/**
 * #include <list> - C++标准库链表容器头文件
 * std::list<T>是双向链表，适合频繁的插入删除操作
 */
#include <list>

/**
 * #include <utility> - C++标准库工具头文件
 * 提供std::pair, std::move等工具
 */
#include <utility>

/**
 * gtest/gtest_prod.h - Google Test框架的头文件
 * FRIEND_TEST宏允许测试代码访问类的私有成员
 */
#include "gtest/gtest_prod.h"

/**
 * absl/strings/str_cat.h - Abseil字符串拼接库
 * absl::StrCat用于高效拼接字符串
 */
#include "absl/strings/str_cat.h"

/**
 * modules/common_msgs/planning_msgs/planning_internal.pb.h
 * 规划内部消息的Protobuf头文件
 * 包含Debug, SLFrameDebug, STGraphDebug, SpeedPlan等调试数据结构
 */
#include "modules/common_msgs/planning_msgs/planning_internal.pb.h"

/**
 * modules/common_msgs/routing_msgs/routing.pb.h
 * 路由消息的Protobuf头文件
 * 包含RoutingRequest, RoutingResponse等
 */
#include "modules/common_msgs/routing_msgs/routing.pb.h"

/**
 * planning_semantic_map_config.pb.h
 * 语义地图配置消息，用于学习模式下的特征渲染器配置
 */
#include "modules/planning/planning_base/proto/planning_semantic_map_config.pb.h"

/**
 * cyber/common/file.h - Cyber RT框架的文件操作工具
 * GetProtoFromFile用于从文件加载Protobuf配置
 */
#include "cyber/common/file.h"

/**
 * cyber/common/log.h - Cyber RT日志系统
 * 提供AINFO, AERROR, AWARN, ADEBUG等日志宏
 */
#include "cyber/common/log.h"

/**
 * cyber/time/clock.h - Cyber RT时间系统
 * Clock::NowInSeconds()获取当前时间戳（秒）
 */
#include "cyber/time/clock.h"

/**
 * modules/common/math/quaternion.h - 四元数数学库
 * 用于处理3D旋转，在轨迹点heading计算中使用
 */
#include "modules/common/math/quaternion.h"

/**
 * vehicle_state/vehicle_state_provider.h - 车辆状态提供者
 * 维护车辆当前位置、速度、航向角等状态信息
 */
#include "modules/common/vehicle_state/vehicle_state_provider.h"

/**
 * map/hdmap/hdmap_util.h - 高精地图工具类
 * HDMapUtil::BaseMapPtr()获取全局高精地图实例
 */
#include "modules/map/hdmap/hdmap_util.h"

/**
 * planning_base/common/ego_info.h - 自车信息类
 * 存储和计算自车的相关参数，如前后障碍物距离等
 */
#include "modules/planning/planning_base/common/ego_info.h"

/**
 * planning_base/common/history.h - 历史轨迹记录器
 * 记录规划历史，用于回放和问题排查
 */
#include "modules/planning/planning_base/common/history.h"

/**
 * planning_base/common/planning_context.h - 规划上下文
 * 存储规划模块的全局状态，如重路由状态等
 */
#include "modules/planning/planning_base/common/planning_context.h"

/**
 * planning_base/common/trajectory_stitcher.h - 轨迹缝合器
 * 将上一帧轨迹与新规划轨迹进行平滑连接
 */
#include "modules/planning/planning_base/common/trajectory_stitcher.h"

/**
 * planning_base/common/util/driving_state_generator.h
 * 驾驶状态生成器，用于生成调试信息的驾驶状态
 */
#include "modules/planning/planning_base/common/util/driving_state_generator.h"

/**
 * planning_base/common/util/util.h - 规划工具函数集合
 * 包含IsVehicleStateValid, IsDifferentRouting等工具函数
 */
#include "modules/planning/planning_base/common/util/util.h"

/**
 * planning_base/gflags/planning_gflags.h - 规划模块的gflags配置
 * 定义各种运行时配置标志，如FLAGS_planning_loop_rate等
 */
#include "modules/planning/planning_base/gflags/planning_gflags.h"

/**
 * learning_based/img_feature_renderer/birdview_img_feature_renderer.h
 * 鸟瞰图特征渲染器，用于学习模式的可视化
 */
#include "modules/planning/planning_base/learning_based/img_feature_renderer/birdview_img_feature_renderer.h"

/**
 * reference_line/reference_line_provider.h - 参考线提供者
 * 负责从路由模块获取并更新参考线信息
 */
#include "modules/planning/planning_base/reference_line/reference_line_provider.h"

/**
 * planning_interface_base/planner_base/planner.h - 规划器接口基类
 * 定义规划器的统一接口，具体的规划器（如EMPlanner）继承自此类
 */
#include "modules/planning/planning_interface_base/planner_base/planner.h"

/**
 * traffic_rules_base/traffic_decider.h - 交通规则决策器基类
 * 处理交通信号灯、停车标志等交通规则
 */
#include "modules/planning/planning_interface_base/traffic_rules_base/traffic_decider.h"

namespace apollo {

/**
 * apollo:: - Apollo最外层命名空间
 * 所有Apollo模块都位于此命名空间下
 */
namespace planning {

/**
 * using声明 - 将其他命名空间中的类型引入当前作用域
 * 之后可以直接使用类型名而不需要完整命名空间前缀
 */

/**
 * canbus::Chassis - 车辆底盘消息类型
 * 包含车速、档位、方向盘角度等底盘数据
 */
using apollo::canbus::Chassis;

/**
 * common::EngageAdvice - 自动驾驶就绪建议
 * 告诉系统是否可以开始自动驾驶
 */
using apollo::common::EngageAdvice;

/**
 * common::ErrorCode - 错误码枚举
 * 定义了各种错误类型，如PLANNING_ERROR等
 */
using apollo::common::ErrorCode;

/**
 * common::Status - Apollo统一状态类
 * 包含错误码和错误消息，用于函数返回状态
 */
using apollo::common::Status;

/**
 * common::TrajectoryPoint - 轨迹点结构
 * 包含路径点位置、速度、加速度、相对时间等信息
 */
using apollo::common::TrajectoryPoint;

/**
 * common::VehicleState - 车辆状态结构
 * 包含车辆位置(x,y,z)、航向角、速度等
 */
using apollo::common::VehicleState;

/**
 * common::VehicleStateProvider - 车辆状态提供者类
 * 维护和更新车辆状态，提供EstimateFuturePosition等方法
 */
using apollo::common::VehicleStateProvider;

/**
 * common::math::Vec2d - 二维向量类
 * 用于表示二维平面坐标和向量运算
 */
using apollo::common::math::Vec2d;

/**
 * cyber::Clock - Cyber RT时钟类
 * Clock::NowInSeconds()获取当前时间戳（双精度秒）
 */
using apollo::cyber::Clock;

/**
 * dreamview::Chart - Dreamview图表类
 * 用于可视化调试信息（轨迹、速度曲线等）
 */
using apollo::dreamview::Chart;

/**
 * hdmap::HDMapUtil - 高精地图工具类
 * 提供访问高精地图的单例接口
 */
using apollo::hdmap::HDMapUtil;

/**
 * planning_internal::SLFrameDebug - SL坐标系调试信息
 * SL表示沿车道纵向(S)和横向(L)的坐标
 */
using apollo::planning_internal::SLFrameDebug;

/**
 * planning_internal::SpeedPlan - 速度规划调试信息
 * 包含速度规划的名称和速度点序列
 */
using apollo::planning_internal::SpeedPlan;

/**
 * planning_internal::STGraphDebug - ST图调试信息
 * ST表示空间(S)和时间(T)的关系图，用于速度规划
 */
using apollo::planning_internal::STGraphDebug;

/**
 * @brief 设置图表的最小最大值
 *
 * 这是一个辅助函数，用于自动计算并设置图表的X和Y轴范围
 * 同时配置图表的各种渲染属性
 *
 * @param chart 指向dreamview::Chart的指针，用于填充图表数据
 * @param label_name_x X轴的标签名称
 * @param label_name_y Y轴的标签名称
 *
 * 语法说明：
 * - std::numeric_limits<double>::max() - double类型的最大正数值
 * - std::numeric_limits<double>::lowest() - double类型最小的负数值
 * - std::min(a, b) / std::max(a, b) - 算法库的比较函数
 * - mutable_line(i) - 获取第i条可写线数据
 * - auto& pt : line->point() - 范围for循环遍历所有点
 * - (*properties)["key"] = "value" - protobuf map的键值访问方式
 */
void SetChartminmax(apollo::dreamview::Chart* chart, std::string label_name_x,
                    std::string label_name_y) {
  /**
   * mutable_options() - 获取图表选项的可写指针
   * 用于设置坐标轴范围、标签等
   */
  auto* options = chart->mutable_options();

  /**
   * 初始化xmin, xmax, ymin, ymax为合适的边界值
   * std::numeric_limits<double>::max() - 初始化为最大值，之后会更新为实际最小值
   * std::numeric_limits<double>::lowest() - 初始化为最小值，之后会更新为实际最大值
   */
  double xmin(std::numeric_limits<double>::max()),
      xmax(std::numeric_limits<double>::lowest()),
      ymin(std::numeric_limits<double>::max()),
      ymax(std::numeric_limits<double>::lowest());

  /**
   * 遍历图表中的所有线条
   * chart->line_size()返回线条数量
   * mutable_line(i)获取第i条线的可写指针
   */
  for (int i = 0; i < chart->line_size(); i++) {
    auto* line = chart->mutable_line(i);

    /**
     * 遍历当前线的所有点
     * line->point()返回const引用，不可修改
     * 使用auto&进行引用避免拷贝
     */
    for (auto& pt : line->point()) {
      /**
       * 更新边界值
       * std::min比较取较小值，std::max比较取较大值
       */
      xmin = std::min(xmin, pt.x());
      ymin = std::min(ymin, pt.y());
      xmax = std::max(xmax, pt.x());
      ymax = std::max(ymax, pt.y());
    }

    /**
     * 设置线条的渲染属性（ChartJS配置）
     * mutable_properties()返回可写的map指针
     * (*map)["key"] = value 是protobuf map的标准访问方式
     */
    auto* properties = line->mutable_properties();
    (*properties)["borderWidth"] = "2";        /**< 边框宽度2像素 */
    (*properties)["pointRadius"] = "0";         /**< 数据点半径0（不显示点） */
    (*properties)["lineTension"] = "0";         /**< 线张力0（直线连接） */
    (*properties)["fill"] = "false";            /**< 不填充线下区域 */
    (*properties)["showLine"] = "true";          /**< 显示线条 */
  }

  /**
   * 设置X轴选项
   * mutable_x()获取X轴可写指针
   * set_min/set_max设置显示范围
   * set_label_string设置轴标签
   */
  options->mutable_x()->set_min(xmin);
  options->mutable_x()->set_max(xmax);
  options->mutable_x()->set_label_string(label_name_x);
  options->mutable_y()->set_min(ymin);
  options->mutable_y()->set_max(ymax);
  options->mutable_y()->set_label_string(label_name_y);
  // Set chartJS's dataset properties
}

/**
 * @brief OnLanePlanning析构函数
 *
 * 析构时清理资源：
 * 1. 停止参考线提供者线程
 * 2. 停止规划器线程
 * 3. 清空帧历史、自车历史、规划上下文等
 *
 * 语法说明：
 * - unique_ptr::get()返回原始指针
 * - 如果unique_ptr不为空才调用Stop()
 * - std::move将左值转为右值引用，用于移动语义
 */
OnLanePlanning::~OnLanePlanning() {
  /**
   * reference_line_provider_->Stop() - 停止参考线提供者的后台线程
   * 确保在析构时不再有线程访问成员数据
   */
  if (reference_line_provider_) {
    reference_line_provider_->Stop();
  }

  /**
   * planner_->Stop() - 停止规划器线程
   */
  planner_->Stop();

  /**
   * injector_->frame_history()->Clear() - 清空帧历史
   * 帧历史记录了最近多帧的规划数据
   */
  injector_->frame_history()->Clear();

  /**
   * injector_->history()->Clear() - 清空轨迹历史
   */
  injector_->history()->Clear();

  /**
   * mutable_planning_status()->Clear() - 清空规划状态
   * 包括重路由状态等
   */
  injector_->planning_context()->mutable_planning_status()->Clear();

  /**
   * last_command_.Clear() - 清空上次的规划命令
   */
  last_command_.Clear();

  /**
   * injector_->ego_info()->Clear() - 清空自车信息
   */
  injector_->ego_info()->Clear();
}

/**
 * @brief 获取规划器名称
 *
 * @return std::string 规划器名称，返回"on_lane_planning"
 *
 * 语法说明：
 * - const后缀表示const成员函数，不能修改成员变量
 */
std::string OnLanePlanning::Name() const { return "on_lane_planning"; }

/**
 * @brief OnLanePlanning初始化函数
 *
 * 在组件启动时调用一次，完成以下初始化工作：
 * 1. 检查规划配置有效性
 * 2. 初始化PlanningBase基类
 * 3. 清空历史数据
 * 4. 加载高精地图
 * 5. 创建并启动参考线提供者
 * 6. 加载具体的规划器（EMPlanner等）
 * 7. 初始化交通规则决策器
 *
 * @param config PlanningConfig配置消息
 * @return Status 初始化是否成功
 *
 * 语法说明：
 * - 返回apollo::common::Status表示操作结果
 * - Status::OK()创建成功状态
 * - Status(ErrorCode, msg)创建带错误码和消息的失败状态
 */
Status OnLanePlanning::Init(const PlanningConfig& config) {
  /**
   * CheckPlanningConfig() - 检查配置有效性
   * 确保所有必需的配置项都已正确设置
   */
  if (!CheckPlanningConfig(config)) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "planning config error: " + config.DebugString());
  }

  /**
   * PlanningBase::Init(config) - 调用基类初始化
   * 基类会初始化injector_等核心组件
   */
  PlanningBase::Init(config);

  // clear planning history - 清空规划历史
  injector_->history()->Clear();

  // clear planning status - 清空规划状态
  injector_->planning_context()->mutable_planning_status()->Clear();

  // load map - 加载高精地图
  hdmap_ = HDMapUtil::BaseMapPtr();

  /**
   * ACHECK宏 - Apollo断言，确保地图加载成功
   * 如果失败会输出错误并终止程序
   */
  ACHECK(hdmap_) << "Failed to load map";

  // instantiate reference line provider - 创建参考线提供者实例
  const ReferenceLineConfig* reference_line_config = nullptr;

  /**
   * config_.has_reference_line_config() - Protobuf的has判断方法
   * 检查消息中是否包含该字段
   */
  if (config_.has_reference_line_config()) {
    reference_line_config = &config_.reference_line_config();
  }

  /**
   * std::make_unique<T>(args...) - C++14创建unique_ptr的工厂函数
   * unique_ptr拥有对象的独占所有权，不可复制只能移动
   * reference_line_provider管理参考线提供者的生命周期
   */
  reference_line_provider_ = std::make_unique<ReferenceLineProvider>(
      injector_->vehicle_state(), reference_line_config);

  /**
   * reference_line_provider_->Start() - 启动参考线提供者的后台线程
   * 启动后会不断从路由模块获取最新参考线
   */
  reference_line_provider_->Start();

  // dispatch planner - 加载具体规划器
  LoadPlanner();

  /**
   * 检查规划器是否创建成功
   * planner_是unique_ptr，如果LoadPlanner失败会为空
   */
  if (!planner_) {
    return Status(
        ErrorCode::PLANNING_ERROR,
        "planning is not initialized with config : " + config_.DebugString());
  }

  /**
   * learning_mode()检查是否使用学习模式
   * 如果使用学习模式，需要初始化鸟瞰图特征渲染器
   */
  if (config_.learning_mode() != PlanningConfig::NO_LEARNING) {
    PlanningSemanticMapConfig renderer_config;

    /**
     * GetProtoFromFile - 从配置文件加载Protobuf消息
     * FLAGS_planning_birdview_img_feature_renderer_config_file
     * 是配置标志，指定渲染器配置文件路径
     */
    ACHECK(apollo::cyber::common::GetProtoFromFile(
        FLAGS_planning_birdview_img_feature_renderer_config_file,
        &renderer_config))
        << "Failed to load renderer config"
        << FLAGS_planning_birdview_img_feature_renderer_config_file;

    /**
     * BirdviewImgFeatureRenderer::Instance() - 获取单例实例
     * 单例模式确保全局只有一个渲染器实例
     * Init()方法使用配置初始化渲染器
     */
    BirdviewImgFeatureRenderer::Instance()->Init(renderer_config);
  }

  /**
   * traffic_decider_.Init(injector_) - 初始化交通规则决策器
   * 传入injector_以便访问地图、车辆状态等信息
   */
  traffic_decider_.Init(injector_);

  /**
   * Clock::NowInSeconds() - 获取当前系统时间（秒）
   * 记录规划模块启动时间
   */
  start_time_ = Clock::NowInSeconds();

  /**
   * planner_->Init(injector_, FLAGS_planner_config_path)
   * 最后调用具体规划器的初始化
   * FLAGS_planner_config_path指定规划器配置文件路径
   */
  return planner_->Init(injector_, FLAGS_planner_config_path);
}

/**
 * @brief 初始化Frame（规划帧）
 *
 * Frame是规划的基本执行单元，包含一帧规划所需的所有数据：
 * 参考线、障碍物、车辆状态等
 *
 * @param sequence_num 帧序号，用于追踪和调试
 * @param planning_start_point 规划起始点
 * @param vehicle_state 当前车辆状态
 * @return Status 初始化是否成功
 *
 * 语法说明：
 * - std::list<T> - 双向链表容器，适合频繁的插入删除
 * - frame_.reset(new Frame(...)) - 释放旧frame并创建新的
 * - reference_line_provider_.get() - unique_ptr转原始指针
 * - DCHECK_EQ(a, b) - Debug模式断言，a必须等于b
 */
Status OnLanePlanning::InitFrame(const uint32_t sequence_num,
                                 const TrajectoryPoint& planning_start_point,
                                 const VehicleState& vehicle_state) {
  /**
   * new Frame(...)创建Frame对象
   * reset()获取所有权，frame_是shared_ptr
   * Frame构造参数：序号、本地视图、起始点、车辆状态、参考线提供者
   */
  frame_.reset(new Frame(sequence_num, local_view_, planning_start_point,
                         vehicle_state, reference_line_provider_.get()));

  /**
   * nullptr检查 - 确保Frame创建成功
   */
  if (frame_ == nullptr) {
    return Status(ErrorCode::PLANNING_ERROR, "Fail to init frame: nullptr.");
  }

  /**
   * std::list<ReferenceLine> - 参考线链表
   * 一帧规划可能有多条参考线（如换道时有当前车道和目标车道）
   *
   * std::list<hdmap::RouteSegments> - 路由段链表
   * 每个参考线对应一个路由段
   */
  std::list<ReferenceLine> reference_lines;
  std::list<hdmap::RouteSegments> segments;

  /**
   * GetReferenceLines - 从参考线提供者获取参考线和路由段
   * 通过输出参数返回
   */
  reference_line_provider_->GetReferenceLines(&reference_lines, &segments);

  /**
   * DCHECK_EQ - Debug断言，检查参考线数量和路由段数量一致
   * 确保数据一致性
   */
  DCHECK_EQ(reference_lines.size(), segments.size());

  /**
   * PncMapBase::LookForwardDistance - 计算前向查看距离
   * 根据当前车速动态调整，车速越高看得越远
   */
  auto forward_limit = planning::PncMapBase::LookForwardDistance(
      vehicle_state.linear_velocity());

  /**
   * 对每条参考线进行裁剪
   * Segment()方法根据车辆当前位置裁剪参考线
   * 移除车辆后方的部分，保留前向部分
   */
  for (auto& ref_line : reference_lines) {
    /**
     * SetEgoPosition - 设置自车位置
     * Vec2d{x, y}使用列表初始化构造二维向量
     */
    ref_line.SetEgoPosition(Vec2d(vehicle_state.x(), vehicle_state.y()));

    /**
     * Segment() - 裁剪参考线
     * 参数：车辆位置、向后看距离、前向限制
     * FLAGS_look_backward_distance是向后看的固定距离
     */
    if (!ref_line.Segment(Vec2d(vehicle_state.x(), vehicle_state.y()),
                          planning::FLAGS_look_backward_distance,
                          forward_limit)) {
      const std::string msg = "Fail to shrink reference line.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }

  /**
   * 对每个路由段进行类似的裁剪
   * Shrink()方法与Segment()类似但用于路由段
   */
  for (auto& seg : segments) {
    if (!seg.Shrink(Vec2d(vehicle_state.x(), vehicle_state.y()),
                    planning::FLAGS_look_backward_distance, forward_limit)) {
      const std::string msg = "Fail to shrink routing segments.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }

  /**
   * frame_->Init - 初始化Frame内部数据结构
   * 传入车辆状态、参考线、路由段等
   * FutureRouteWaypoints()获取路径沿途的waypoints
   * injector_->ego_info()获取自车信息
   */
  auto status = frame_->Init(
      injector_->vehicle_state(), reference_lines, segments,
      reference_line_provider_->FutureRouteWaypoints(), injector_->ego_info());

  /**
   * 检查Frame初始化是否成功
   * status.ok()判断状态是否表示成功
   */
  if (!status.ok()) {
    AERROR << "failed to init frame:" << status.ToString();
    return status;
  }

  return Status::OK();
}

/**
 * @brief 生成停止轨迹
 *
 * 当规划失败或需要紧急停车时，生成一条减速至停止的轨迹
 * 该轨迹是一条直线，所有轨迹点速度加速度都为0
 *
 * @param ptr_trajectory_pb 输出参数，指向轨迹消息的指针
 *
 * 语法说明：
 * - ptr_trajectory_pb->clear_trajectory_point() - 清空原有轨迹点
 * - mutable_path_point() - 获取路径点的可写指针
 * - set_x/set_y/set_theta - 设置路径点坐标和航向角
 * - add_trajectory_point() - 向轨迹添加一个新点并返回其指针
 */
void OnLanePlanning::GenerateStopTrajectory(ADCTrajectory* ptr_trajectory_pb) {
  /** 清空原有轨迹点 */
  ptr_trajectory_pb->clear_trajectory_point();

  /**
   * 获取当前车辆状态
   * injector_->vehicle_state()->vehicle_state()返回VehicleState引用
   */
  const auto& vehicle_state = injector_->vehicle_state()->vehicle_state();

  /**
   * FLAGS_fallback_total_time - 后备规划的总时间
   * 例如设为5秒，则生成5秒的停止轨迹
   */
  const double max_t = FLAGS_fallback_total_time;

  /**
   * FLAGS_fallback_time_unit - 后备规划的时间步长
   * 例如设为0.1秒，则每0.1秒生成一个轨迹点
   */
  const double unit_t = FLAGS_fallback_time_unit;

  /**
   * TrajectoryPoint tp - 创建单个轨迹点
   * 在循环中重复使用并添加多个副本到轨迹
   */
  TrajectoryPoint tp;
  auto* path_point = tp.mutable_path_point();

  /**
   * 设置路径点的位置为当前位置
   * heading()返回车辆航向角（弧度）
   */
  path_point->set_x(vehicle_state.x());
  path_point->set_y(vehicle_state.y());
  path_point->set_theta(vehicle_state.heading());

  /**
   * set_s(0.0) - 设置路径长度参数为0
   * s是沿着参考线的累计距离
   */
  path_point->set_s(0.0);

  /**
   * set_v(0.0) / set_a(0.0) - 设置速度和加速度为0
   */
  tp.set_v(0.0);
  tp.set_a(0.0);

  /**
   * 循环生成所有轨迹点
   * t += unit_t表示每个时间步长
   */
  for (double t = 0.0; t < max_t; t += unit_t) {
    tp.set_relative_time(t);  /**< 设置相对时间 */

    /**
     * add_trajectory_point() - 添加一个新的轨迹点
     * 返回新点的可写指针，然后CopyFrom复制数据
     */
    auto next_point = ptr_trajectory_pb->add_trajectory_point();
    next_point->CopyFrom(tp);
  }
}

/**
 * @brief OnLanePlanning的主运行函数
 *
 * 这是每个规划周期（通常10Hz）调用的主函数
 * 完成以下工作：
 * 1. 更新车辆状态
 * 2. 检查是否需要重路由
 * 3. 进行轨迹缝合
 * 4. 初始化Frame
 * 5. 执行交通规则决策
 * 6. 调用规划器进行轨迹规划
 * 7. 发布轨迹结果
 *
 * @param local_view 本地视图，包含所有输入数据
 * @param ptr_trajectory_pb 输出轨迹消息
 *
 * 语法说明：
 * - const std::shared_ptr<T>& - 常量引用，避免不必要的拷贝
 * - Clock::NowInSeconds() - 获取当前时间戳
 * - std::chrono::system_clock::now() - 获取系统时钟的当前时间
 * - duration<double>::count() - 将时间间隔转换为双精度秒数
 */
void OnLanePlanning::RunOnce(const LocalView& local_view,
                             ADCTrajectory* const ptr_trajectory_pb) {
  /**
   * 当重路由时，参考线可能还未更新
   * 在这种情况下，规划模块保持not-ready状态直到被重启
   */
  // when rerouting, reference line might not be updated. In this case, planning
  // module maintains not-ready until be restarted.
  local_view_ = local_view;

  /**
   * start_timestamp - 记录本帧规划开始时间戳（Cyber时间）
   * Clock::NowInSeconds()获取Cyber RT系统时间  用于轨迹时间基准
   */
  const double start_timestamp = Clock::NowInSeconds();

  /**
   * start_system_timestamp - 记录本帧规划开始时间戳（系统时间）
   * std::chrono::system_clock是系统真实时间
   * time_since_epoch()返回从1970-01-01到现在的时长
   * .count()将duration转换为数值   用于计算规划耗时
   */
  const double start_system_timestamp =
      std::chrono::duration<double>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();

  // localization - 输出定位信息调试日志
  ADEBUG << "Get localization:"
         << local_view_.localization_estimate->DebugString();

  // chassis - 输出底盘信息调试日志
  ADEBUG << "Get chassis:" << local_view_.chassis->DebugString();

  /**
   * injector_->vehicle_state()->Update() - 更新车辆状态
   * 根据新的定位和底盘数据更新内部状态
   * 返回Status表示更新是否成功
   */
  Status status = injector_->vehicle_state()->Update(
      *local_view_.localization_estimate, *local_view_.chassis);

  /**
   * vehicle_state() - 获取更新后的车辆状态
   * 这是个复制操作，获取的是当前时刻的快照
   */
  VehicleState vehicle_state = injector_->vehicle_state()->vehicle_state();

  /**
   * vehicle_state_timestamp - 车辆状态的时间戳
   * 每条消息都有时间戳标记其采集时间
   */
  const double vehicle_state_timestamp = vehicle_state.timestamp();

  /**
   * DCHECK_GE - Debug断言 Greater Equal
   * 确保规划开始时间不早于车辆状态时间
   * 即我们不会用"未来"的数据来规划
   */
  // 必须满足start_timestamp >= vehicle_state_timestamp
  DCHECK_GE(start_timestamp, vehicle_state_timestamp)
      << "start_timestamp is behind vehicle_state_timestamp by "
      << start_timestamp - vehicle_state_timestamp << " secs";

  /**
   * IsVehicleStateValid() - 检查车辆状态是否有效
   * 可能检查数据是否过时、是否在合理范围内等
   */
  if (!status.ok() || !util::IsVehicleStateValid(vehicle_state)) {
    const std::string msg =
        "Update VehicleStateProvider failed "
        "or the vehicle state is out dated.";
    AERROR << msg;

    /**
     * 设置轨迹的not_ready决策
     * mutable_decision()->mutable_main_decision()->mutable_not_ready()
     * 一系列mutable调用获取嵌套消息的写指针
     */
    ptr_trajectory_pb->mutable_decision()
        ->mutable_main_decision()
        ->mutable_not_ready()
        ->set_reason(msg);

    /**
     * status.Save() - 将状态信息保存到消息头部
     * mutable_header()->mutable_status()获取状态消息的写指针
     */
    status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());

    /**
     * set_gear设置车辆档位
     * 即使规划失败也默认为前进档
     * TODO(all)注释表示未来需要整合倒档支持
     */
    // TODO(all): integrate reverse gear
    ptr_trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);

    FillPlanningPb(start_timestamp, ptr_trajectory_pb);
    GenerateStopTrajectory(ptr_trajectory_pb);  /**< 生成停止轨迹 */
    return;  /**< 规划失败，直接返回 */
  }

  /**
   * 检查时间同步
   * 如果Cyber时间落后于GPS时间太多，记录错误
   */
  // 异常检测：如果 Cyber 时间落后于 GPS 时间，记录 ERROR（时钟不同步）
  if (start_timestamp + 1e-6 < vehicle_state_timestamp) {
    common::monitor::MonitorLogBuffer monitor_logger_buffer(
        common::monitor::MonitorMessageItem::PLANNING);
    monitor_logger_buffer.ERROR("ego system time is behind GPS time");
  }

  /**
   * FLAGS_message_latency_threshold - 消息延迟阈值
   * 如果车辆状态时间与规划开始时间差小于阈值
   * 则进行时间对齐
   */
  // 时间对齐：如果延迟小于阈值，将车辆状态时间对齐到当前时刻（AlignTimeStamp 线性插值预测当前位置/速度/航向）
  if (start_timestamp - vehicle_state_timestamp <
      FLAGS_message_latency_threshold) {
    vehicle_state = AlignTimeStamp(vehicle_state, start_timestamp);
  }

  // Update reference line provider and reset scenario if new routing
  /**
   * reference_line_provider_->UpdateVehicleState() - 更新参考线提供者中的车辆状态
   * 参考线提供者需要知道车辆位置来确定哪些参考线相关
   */
  reference_line_provider_->UpdateVehicleState(vehicle_state);

  /**
   * 检查是否是新的路由命令
   * is_motion_command()判断是否是运动命令
   * IsDifferentRouting()比较两次路由是否不同
   */
  if (local_view_.planning_command->is_motion_command() &&
      util::IsDifferentRouting(last_command_, *local_view_.planning_command)) {
    last_command_ = *local_view_.planning_command;  /**< 保存当前命令 */

    /**
     * 当检测到新路由时，需要重置相关状态
     * 包括：参考线提供者、历史记录、规划上下文
     */
    reference_line_provider_->Reset();
    injector_->history()->Clear();
    injector_->planning_context()->mutable_planning_status()->Clear();

    /**
     * UpdatePlanningCommand - 将新命令更新到参考线提供者
     * 这会触发参考线的重新计算
     */
    reference_line_provider_->UpdatePlanningCommand(
        *(local_view_.planning_command));

    /**
     * planner_->Reset() - 重置规划器
     * 清空规划器内部状态，准备接受新任务
     */
    planner_->Reset(frame_.get());
  }

  /**
   * GetEndLaneWayPoint - 获取终点车道信息
   * 存储在local_view_.end_lane_way_point中
   */
  // Get end lane way point.
  reference_line_provider_->GetEndLaneWayPoint(local_view_.end_lane_way_point);

  /**
   * planning cycle time - 规划周期时间
   * static_cast<double>是C++类型转换，FLAGS_planning_loop_rate通常是10或20
   * 10Hz对应0.1秒，20Hz对应0.05秒
   */
  // planning is triggered by prediction data, but we can still use an estimated
  // cycle time for stitching
  const double planning_cycle_time =
      1.0 / static_cast<double>(FLAGS_planning_loop_rate);

  /**
   * std::vector<TrajectoryPoint> - 动态数组，存储轨迹点
   * stitching_trajectory是缝合后的轨迹
   */
  std::string replan_reason;  /**< 重规划原因 */
  std::vector<TrajectoryPoint> stitching_trajectory =
      TrajectoryStitcher::ComputeStitchingTrajectory(
          *(local_view_.chassis), vehicle_state, start_timestamp,
          planning_cycle_time, FLAGS_trajectory_stitching_preserved_length,
          true, last_publishable_trajectory_.get(), &replan_reason,
          *local_view_.control_interactive_msg);

  /**
   * injector_->ego_info()->Update() - 更新自车信息
   * 使用缝合轨迹的最后一个点和当前车辆状态
   */
  injector_->ego_info()->Update(stitching_trajectory.back(), vehicle_state);

  /**
   * frame_num - 帧序号
   * seq_num_是成员变量，每次调用后递增
   * static_cast<uint32_t>转换为无符号32位整数
   */
  const uint32_t frame_num = static_cast<uint32_t>(seq_num_++);
  AINFO << "Planning start frame sequence id = [" << frame_num << "]";

  /**
   * InitFrame - 初始化规划帧
   * 传入帧序号、缝合轨迹的最后点、车辆状态
   */
  status = InitFrame(frame_num, stitching_trajectory.back(), vehicle_state);

  /**
   * 如果Frame初始化成功，计算一些额外信息
   * CalculateFrontObstacleClearDistance - 计算前方障碍物清除距离
   * CalculateCurrentRouteInfo - 计算当前路线信息
   */
  if (status.ok()) {
    injector_->ego_info()->CalculateFrontObstacleClearDistance(
        frame_->obstacles());
    injector_->ego_info()->CalculateCurrentRouteInfo(
        reference_line_provider_.get());
  }

  /**
   * FLAGS_enable_record_debug - 是否记录调试信息
   * RecordInputDebug将输入数据写入debug消息
   */
  if (FLAGS_enable_record_debug) {
    frame_->RecordInputDebug(ptr_trajectory_pb->mutable_debug());
  }

  /**
   * 记录初始化Frame的时间消耗
   * set_init_frame_time_ms设置初始化帧的时间（毫秒）
   */
  ptr_trajectory_pb->mutable_latency_stats()->set_init_frame_time_ms(
      Clock::NowInSeconds() - start_timestamp);

  /**
   * 如果Frame初始化失败，处理错误
   */
  if (!status.ok()) {
    AERROR << status.ToString();
    if (FLAGS_publish_estop) {
      /**
       * FLAGS_publish_estop - 是否发布紧急停止
       * EStop消息包含is_estop标志和reason描述
       */
      // "estop" signal check in function "Control::ProduceControlCommand()"
      // estop_ = estop_ || local_view_.trajectory.estop().is_estop();
      // we should add more information to ensure the estop being triggered.
      ADCTrajectory estop_trajectory;
      EStop* estop = estop_trajectory.mutable_estop();
      estop->set_is_estop(true);
      estop->set_reason(status.error_message());
      status.Save(estop_trajectory.mutable_header()->mutable_status());
      ptr_trajectory_pb->CopyFrom(estop_trajectory);
    } else {
      /**
       * 如果不发布estop，设置not_ready原因并生成停止轨迹
       */
      ptr_trajectory_pb->mutable_decision()
          ->mutable_main_decision()
          ->mutable_not_ready()
          ->set_reason(status.ToString());
      status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
      GenerateStopTrajectory(ptr_trajectory_pb);
    }
    // TODO(all): integrate reverse gear
    ptr_trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);
    FillPlanningPb(start_timestamp, ptr_trajectory_pb);

    /**
     * set_current_frame_planned_trajectory - 记录当前帧的规划轨迹
     * 用于后续调试和轨迹缝合
     */
    frame_->set_current_frame_planned_trajectory(*ptr_trajectory_pb);

    /**
     * frame_->SequenceNum() - 获取帧序号
     * injector_->frame_history()->Add() - 添加到帧历史
     * std::move将frame_移动到历史中，避免复制
     */
    const uint32_t n = frame_->SequenceNum();
    injector_->frame_history()->Add(n, std::move(frame_));
    return;  /**< 初始化失败，返回 */
  }


  // 交通规则决策
  /**
   * 遍历所有参考线信息，执行交通规则决策
   * traffic_decider_.Execute()处理交通规则
   * 如红绿灯、停车标志、让行规则等
   */
  for (auto& ref_line_info : *frame_->mutable_reference_line_info()) {
    auto traffic_status =
        traffic_decider_.Execute(frame_.get(), &ref_line_info);

    /**
     * 如果交通决策失败或参考线不可行驶
     * 设置drivable为false并记录警告
     */
    if (!traffic_status.ok() || !ref_line_info.IsDrivable()) {
      ref_line_info.SetDrivable(false);
      AWARN << "Reference line " << ref_line_info.Lanes().Id()
            << " traffic decider failed";
    }
  }

  /**
   * Plan() - 执行核心轨迹规划
   * 这是最关键的函数，调用具体规划器生成轨迹
   */
  // 进入 PublicRoadPlanner::Plan() → ScenarioManager::Update() → Scenario::Process() → Stage::Process() → Task::Execute() 的完整四层架构
  status = Plan(start_timestamp, stitching_trajectory, ptr_trajectory_pb);

  /**
   * 打印轨迹XY坐标 - 用于调试
   * PrintCurves类用于记录曲线数据到日志
   */
  // print trajxy
  PrintCurves trajectory_print_curve;
  for (const auto& p : ptr_trajectory_pb->trajectory_point()) {
    trajectory_print_curve.AddPoint("trajxy", p.path_point().x(),
                                    p.path_point().y());
  }
  trajectory_print_curve.PrintToLog();

  /**
   * 打印障碍物多边形 - 用于调试
   * obstacle->PrintPolygonCurve()输出障碍物边界
   */
  // print obstacle polygon
  for (const auto& obstacle : frame_->obstacles()) {
    obstacle->PrintPolygonCurve();
  }

  /**
   * 打印自车边界框 - 用于调试
   * PrintBox类绘制车辆形状
   */
  // print ego box
  PrintBox print_box("ego_box");
  print_box.AddAdcBox(vehicle_state.x(), vehicle_state.y(),
                      vehicle_state.heading(), true);
  print_box.PrintToLog();

  /**
   * 计算本帧规划的总时间消耗
   * end_system_timestamp - 规划结束时的系统时间
   */
  const auto end_system_timestamp =
      std::chrono::duration<double>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();

  /**
   * 计算时间差（毫秒）
   * * 1000将秒转换为毫秒
   */
  const auto time_diff_ms =
      (end_system_timestamp - start_system_timestamp) * 1000;
  ADEBUG << "total planning time spend: " << time_diff_ms << " ms.";

  /**
   * 将时间消耗写入轨迹消息的latency_stats
   * 用于性能监控和分析
   */
  ptr_trajectory_pb->mutable_latency_stats()->set_total_time_ms(time_diff_ms);
  ADEBUG << "Planning latency: "
         << ptr_trajectory_pb->latency_stats().DebugString();

  /**
   * 如果规划失败，处理错误
   */
  if (!status.ok()) {
    status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
    AERROR << "Planning failed:" << status.ToString();
    if (FLAGS_publish_estop) {
      AERROR << "Planning failed and set estop";
      // "estop" signal check in function "Control::ProduceControlCommand()"
      // estop_ = estop_ || local_view_.trajectory.estop().is_estop();
      // we should add more information to ensure the estop being triggered.
      EStop* estop = ptr_trajectory_pb->mutable_estop();
      estop->set_is_estop(true);
      estop->set_reason(status.error_message());
    }
  }

  /**
   * 设置重规划标志
   * 如果缝合轨迹只有一个点，说明进行了重规划
   */
  ptr_trajectory_pb->set_is_replan(stitching_trajectory.size() == 1);
  if (ptr_trajectory_pb->is_replan()) {
    ptr_trajectory_pb->set_replan_reason(replan_reason);
  }

  /**
   * 检查是否是开放空间轨迹（如泊车）
   * is_on_open_space_trajectory()判断是否在开放空间模式
   */
  if (frame_->open_space_info().is_on_open_space_trajectory()) {
    FillPlanningPb(start_timestamp, ptr_trajectory_pb);
    ADEBUG << "Planning pb:" << ptr_trajectory_pb->header().DebugString();
    frame_->set_current_frame_planned_trajectory(*ptr_trajectory_pb);
  } else {
    /**
     * 非开放空间模式，添加参考线提供者的时间延迟统计
     */
    auto* ref_line_task =
        ptr_trajectory_pb->mutable_latency_stats()->add_task_stats();
    ref_line_task->set_time_ms(reference_line_provider_->LastTimeDelay() *
                               1000.0);
    ref_line_task->set_name("ReferenceLineProvider");

    FillPlanningPb(start_timestamp, ptr_trajectory_pb);
    ADEBUG << "Planning pb:" << ptr_trajectory_pb->header().DebugString();

    frame_->set_current_frame_planned_trajectory(*ptr_trajectory_pb);

    /**
     * FLAGS_enable_planning_smoother - 是否启用轨迹平滑
     * planning_smoother_.Smooth()对轨迹进行平滑处理
     */
    if (FLAGS_enable_planning_smoother) {
      planning_smoother_.Smooth(injector_->frame_history(), frame_.get(),
                                ptr_trajectory_pb);
    }
  }

  /**
   * 计算并记录完整的规划性能数据
   */
  const auto end_planning_perf_timestamp =
      std::chrono::duration<double>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();
  const auto plnning_perf_ms =
      (end_planning_perf_timestamp - start_system_timestamp) * 1000;
  AINFO << "Planning Perf: planning name [" << Name() << "], "
        << plnning_perf_ms << " ms.";
  AINFO << "Planning end frame sequence id = [" << frame_num << "]";

  /**
   * 将Frame添加到历史记录
   * std::move将frame_的所有权转移到历史中
   */
  injector_->frame_history()->Add(frame_num, std::move(frame_));
}

/**
 * @brief 导出参考线调试信息
 *
 * 将参考线的各种信息写入debug消息，用于可视化调试
 *
 * @param debug 指向debug消息的指针
 *
 * 语法说明：
 * - for (auto& x : container) - 范围for循环遍历容器
 * - std::numeric_limits<double>::infinity() - 无穷大
 * - std::abs(x) - 绝对值函数
 */
void OnLanePlanning::ExportReferenceLineDebug(planning_internal::Debug* debug) {
  /**
   * FLAGS_enable_record_debug - 检查是否启用调试记录
   */
  if (!FLAGS_enable_record_debug) {
    return;
  }

  /**
   * 遍历所有参考线信息
   * mutable_reference_line_info()返回参考线信息的可写引用
   */
  for (auto& reference_line_info : *frame_->mutable_reference_line_info()) {
    /**
     * 添加新的参考线调试数据
     * mutable_planning_data()->add_reference_line()添加一个参考线debug并返回指针
     */
    auto rl_debug = debug->mutable_planning_data()->add_reference_line();
    rl_debug->set_id(reference_line_info.Lanes().Id());
    rl_debug->set_length(reference_line_info.reference_line().Length());
    rl_debug->set_cost(reference_line_info.Cost());
    rl_debug->set_is_change_lane_path(reference_line_info.IsChangeLanePath());
    rl_debug->set_is_drivable(reference_line_info.IsDrivable());
    rl_debug->set_is_protected(reference_line_info.GetRightOfWayStatus() ==
                               ADCTrajectory::PROTECTED);

    /**
     * 计算曲率和角速度的统计值
     * 用于性能评估
     * kappa - 曲率
     * dkappa - 曲率导数
     */
    // store kappa and dkappa for performance evaluation
    const auto& reference_points =
        reference_line_info.reference_line().reference_points();
    double kappa_rms = 0.0;      /**< 曲率均方根 */
    double dkappa_rms = 0.0;     /**< 角速度均方根 */
    double kappa_max_abs = std::numeric_limits<double>::lowest();  /**< 最大曲率绝对值 */
    double dkappa_max_abs = std::numeric_limits<double>::lowest(); /**< 最大角速度绝对值 */

    /**
     * 遍历所有参考点，累加曲率的平方
     */
    for (const auto& reference_point : reference_points) {
      double kappa_sq = reference_point.kappa() * reference_point.kappa();
      double dkappa_sq = reference_point.dkappa() * reference_point.dkappa;
      kappa_rms += kappa_sq;
      dkappa_rms += dkappa_sq;
      kappa_max_abs = kappa_max_abs < kappa_sq ? kappa_sq : kappa_max_abs;
      dkappa_max_abs = dkappa_max_abs < dkappa_sq ? dkappa_sq : dkappa_max_abs;
    }

    /**
     * 计算平均值和均方根
     */
    double reference_points_size = static_cast<double>(reference_points.size());
    kappa_rms /= reference_points_size;
    dkappa_rms /= reference_points_size;
    kappa_rms = std::sqrt(kappa_rms);  /**< std::sqrt计算平方根 */
    dkappa_rms = std::sqrt(dkappa_rms);

    /**
     * 设置曲率统计值到debug消息
     */
    rl_debug->set_kappa_rms(kappa_rms);
    rl_debug->set_dkappa_rms(dkappa_rms);
    rl_debug->set_kappa_max_abs(kappa_max_abs);
    rl_debug->set_dkappa_max_abs(dkappa_max_abs);

    /**
     * 检查是否偏离道路
     * 计算车道边界宽度
     */
    bool is_off_road = false;
    double minimum_boundary = std::numeric_limits<double>::infinity();

    /**
     * common::VehicleConfigHelper::GetConfig() - 获取车辆配置单例
     * vehicle_param().width()获取车辆宽度
     * adc_half_width是车宽的一半，用于判断是否在车道内
     */
    const double adc_half_width =
        common::VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;
    const auto& reference_line_path =
        reference_line_info.reference_line().GetMapPath();

    const auto sample_s = 0.1;  /**< 采样距离间隔 */
    const auto reference_line_length =
        reference_line_info.reference_line().Length();
    double average_offset = 0.0;
    double sample_count = 0.0;

    /**
     * 沿参考线采样，计算车道边界和偏移
     */
    for (double s = 0.0; s < reference_line_length; s += sample_s) {
      /**
       * GetLaneLeftWidth/RightWidth - 获取左右车道宽度
       */
      double left_width = reference_line_path.GetLaneLeftWidth(s);
      double right_width = reference_line_path.GetLaneRightWidth(s);
      average_offset += 0.5 * std::abs(left_width - right_width);

      /**
       * 如果车道宽度小于车宽一半，认为偏离道路
       */
      if (left_width < adc_half_width || right_width < adc_half_width) {
        is_off_road = true;
      }

      /**
       * 记录最小边界宽度
       */
      if (left_width < minimum_boundary) {
        minimum_boundary = left_width;
      }
      if (right_width < minimum_boundary) {
        minimum_boundary = right_width;
      }
      ++sample_count;
    }

    /**
     * 设置道路状态信息
     */
    rl_debug->set_is_offroad(is_off_road);
    rl_debug->set_minimum_boundary(minimum_boundary);
    rl_debug->set_average_offset(average_offset / sample_count);
  }
}

/**
 * @brief 核心规划函数
 *
 * 调用具体规划器（EMPlanner等）生成轨迹
 * 并处理开放空间和非开放空间两种模式
 *
 * @param current_time_stamp 当前时间戳
 * @param stitching_trajectory 缝合轨迹
 * @param ptr_trajectory_pb 输出轨迹消息
 * @return Status 规划是否成功
 *
 * 语法说明：
 * - ptr_trajectory_pb->mutable_debug() - 获取debug消息的可写指针
 * - MergeFrom() - 合并另一个消息的内容
 * - std::copy(begin, end, output_iter) - 标准算法库复制函数
 * - std::back_inserter - 创建反向迭代器，用于在容器末尾插入
 */
Status OnLanePlanning::Plan(
    const double current_time_stamp,
    const std::vector<TrajectoryPoint>& stitching_trajectory,
    ADCTrajectory* const ptr_trajectory_pb) {
  /**
   * 获取debug消息指针
   */
  auto* ptr_debug = ptr_trajectory_pb->mutable_debug();

  /**
   * 如果启用调试记录，保存初始点信息
   */
  if (FLAGS_enable_record_debug) {
    ptr_debug->mutable_planning_data()->mutable_init_point()->CopyFrom(
        stitching_trajectory.back());
    frame_->mutable_open_space_info()->set_debug(ptr_debug);
    frame_->mutable_open_space_info()->sync_debug_instance();
  }

  /**
   * planner_->Plan() - 调用具体规划器进行轨迹规划
   * 这是核心函数，根据不同配置调用不同的规划算法
   */
  auto status = planner_->Plan(stitching_trajectory.back(), frame_.get(),
                               ptr_trajectory_pb);

  /**
   * 检查是否是开放空间轨迹模式（如泊车场景）
   */
  if (frame_->open_space_info().is_on_open_space_trajectory()) {
    /**
     * 开放空间模式：使用open_space_info中的轨迹数据
     */
    frame_->mutable_open_space_info()->sync_debug_instance();

    /**
     * 获取可发布的轨迹数据
     * publishable_trajectory_data()返回pair，第一个是轨迹，第二个是档位
     */
    const auto& publishable_trajectory =
        frame_->open_space_info().publishable_trajectory_data().first;
    const auto& publishable_trajectory_gear =
        frame_->open_space_info().publishable_trajectory_data().second;
    const auto& trajectory_type = frame_->open_space_info().trajectory_type();
    const auto& is_collision = frame_->open_space_info().is_collision();

    /**
     * PopulateTrajectoryProtobuf - 将轨迹数据填充到protobuf消息
     */
    publishable_trajectory.PopulateTrajectoryProtobuf(ptr_trajectory_pb);

    /**
     * 设置档位、轨迹类型、碰撞状态
     */
    ptr_trajectory_pb->set_gear(publishable_trajectory_gear);
    ptr_trajectory_pb->set_trajectory_type(trajectory_type);
    ptr_trajectory_pb->set_is_collision(is_collision);

    /**
     * TODO(QiL): refine engage advice in open space trajectory optimizer.
     * 设置自动驾驶就绪建议
     */
    auto* engage_advice = ptr_trajectory_pb->mutable_engage_advice();

    /**
     * 判断当前驾驶模式
     * 如果不是完全自动驾驶模式，设置READY_TO_ENGAGE
     * 否则保持ENGAGED
     */
    // enable start auto from open_space planner.
    if (injector_->vehicle_state()->vehicle_state().driving_mode() !=
        Chassis::DrivingMode::Chassis_DrivingMode_COMPLETE_AUTO_DRIVE) {
      engage_advice->set_advice(EngageAdvice::READY_TO_ENGAGE);
      engage_advice->set_reason(
          "Ready to engage when staring with OPEN_SPACE_PLANNER");
    } else {
      engage_advice->set_advice(EngageAdvice::KEEP_ENGAGED);
      engage_advice->set_reason("Keep engage while in parking");
    }

    /**
     * 设置主决策为停车状态
     */
    // TODO(QiL): refine the export decision in open space info
    ptr_trajectory_pb->mutable_decision()
        ->mutable_main_decision()
        ->mutable_parking()
        ->set_status(MainParking::IN_PARKING);

    /**
     * 记录调试信息
     */
    if (FLAGS_enable_record_debug) {
      frame_->mutable_open_space_info()->RecordDebug(ptr_debug);
      ADEBUG << "Open space debug information added!";
      ExportOpenSpaceChart(ptr_trajectory_pb->debug(), *ptr_trajectory_pb,
                           ptr_debug);
    }
  } else {
    /**
     * 非开放空间模式（正常车道行驶）
     * FindDriveReferenceLineInfo - 找到最佳行驶的参考线
     * FindTargetReferenceLineInfo - 找到目标参考线（换道时）
     */
    const auto* best_ref_info = frame_->FindDriveReferenceLineInfo();
    const auto* target_ref_info = frame_->FindTargetReferenceLineInfo();

    /**
     * 如果没有找到可行驶的参考线，返回错误
     */
    if (!best_ref_info) {
      const std::string msg = "planner failed to make a driving plan";
      AERROR << msg;
      if (last_publishable_trajectory_) {
        last_publishable_trajectory_->Clear();
      }
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    /**
     * DiscretizedPath - 离散路径类型
     * 存储当前帧规划的路径（缝合轨迹 + 规划路径）
     * 用于下一帧的速度 fallback
     */
    // Store current frame stitched path for possible speed fallback in next
    // frames
    DiscretizedPath current_frame_planned_path;

    /**
     * 先将缝合轨迹的路径点加入
     */
    for (const auto& trajectory_point : stitching_trajectory) {
      current_frame_planned_path.push_back(trajectory_point.path_point());
    }

    /**
     * 获取最佳参考线的路径
     * discretized_path()返回路径点序列
     * 从第二个点开始（跳过起点，因为已在缝合轨迹中）
     */
    const auto& best_ref_path = best_ref_info->path_data().discretized_path();

    /**
     * std::copy复制路径点
     * best_ref_path.begin() + 1跳过第一个点
     * std::back_inserter创建在容器末尾插入的迭代器适配器
     */
    std::copy(best_ref_path.begin() + 1, best_ref_path.end(),
              std::back_inserter(current_frame_planned_path));
    frame_->set_current_frame_planned_path(current_frame_planned_path);

    /**
     * ptr_debug->MergeFrom合并best_ref_info的debug信息
     */
    ptr_debug->MergeFrom(best_ref_info->debug());

    /**
     * FLAGS_export_chart - 是否导出图表调试信息
     * ExportOnLaneChart导出车道内规划的图表
     * ExportReferenceLineDebug导出参考线调试信息
     * ExportFailedLaneChangeSTChart导出失败换道的ST图
     */
    if (FLAGS_export_chart) {
      ExportOnLaneChart(best_ref_info->debug(), ptr_debug);
    } else {
      ExportReferenceLineDebug(ptr_debug);
      const auto* failed_ref_info = frame_->FindFailedReferenceLineInfo();
      if (failed_ref_info) {
        ExportFailedLaneChangeSTChart(failed_ref_info->debug(), ptr_debug);
      }
    }

    /**
     * 合并延迟统计信息
     */
    ptr_trajectory_pb->mutable_latency_stats()->MergeFrom(
        best_ref_info->latency_stats());

    /**
     * 设置道路优先权状态
     */
    // set right of way status
    ptr_trajectory_pb->set_right_of_way_status(
        best_ref_info->GetRightOfWayStatus());

    /**
     * 设置目标车道ID
     */
    for (const auto& id : best_ref_info->TargetLaneId()) {
      ptr_trajectory_pb->add_lane_id()->CopyFrom(id);
    }

    for (const auto& id : target_ref_info->TargetLaneId()) {
      ptr_trajectory_pb->add_target_lane_id()->CopyFrom(id);
    }

    /**
     * 设置轨迹类型
     */
    ptr_trajectory_pb->set_trajectory_type(best_ref_info->trajectory_type());

    /**
     * FLAGS_enable_rss_info - 是否启用RSS（Responsibility Sensitive Safety）信息
     * RSS是英特尔提出的自动驾驶安全模型
     */
    if (FLAGS_enable_rss_info) {
      *ptr_trajectory_pb->mutable_rss_info() = best_ref_info->rss_info();
    }

    /**
     * 导出决策信息
     */
    best_ref_info->ExportDecision(ptr_trajectory_pb->mutable_decision(),
                                  injector_->planning_context());

    /**
     * 添加路径调试信息
     */
    // Add debug information.
    if (FLAGS_enable_record_debug) {
      auto* reference_line = ptr_debug->mutable_planning_data()->add_path();
      reference_line->set_name("planning_reference_line");
      const auto& reference_points =
          best_ref_info->reference_line().reference_points();
      double s = 0.0;      /**< 累计弧长 */
      double prev_x = 0.0; /**< 上一个点的x坐标 */
      double prev_y = 0.0; /**< 上一个点的y坐标 */
      bool empty_path = true;

      /**
       * 遍历参考线上的点，计算累计距离s
       * s是沿着路径的累计弧长
       */
      for (const auto& reference_point : reference_points) {
        auto* path_point = reference_line->add_path_point();
        path_point->set_x(reference_point.x());
        path_point->set_y(reference_point.y());
        path_point->set_theta(reference_point.heading());
        path_point->set_kappa(reference_point.kappa());
        path_point->set_dkappa(reference_point.dkappa());

        if (empty_path) {
          path_point->set_s(0.0);  /**< 第一个点s=0 */
          empty_path = false;
        } else {
          /**
           * std::hypot(dx, dy) - 计算直角三角形的斜边长度
           * 等价于sqrt(dx*dx + dy*dy)但更数值稳定
           */
          double dx = reference_point.x() - prev_x;
          double dy = reference_point.y() - prev_y;
          s += std::hypot(dx, dy);
          path_point->set_s(s);
        }
        prev_x = reference_point.x();
        prev_y = reference_point.y();
      }
    }

    /**
     * PublishableTrajectory - 可发布的轨迹类
     * 用于存储和发布规划好的轨迹
     * current_time_stamp是轨迹起始时间
     * best_ref_info->trajectory()是规划的速度轨迹
     */
    last_publishable_trajectory_.reset(new PublishableTrajectory(
        current_time_stamp, best_ref_info->trajectory()));

    /**
     * 打印速度曲线调试信息
     */
    PrintCurves debug_traj;
    for (size_t i = 0; i < last_publishable_trajectory_->size(); i++) {
      auto& traj_pt = last_publishable_trajectory_->at(i);
      debug_traj.AddPoint("traj_sv", traj_pt.path_point().s(), traj_pt.v());
      debug_traj.AddPoint("traj_sa", traj_pt.path_point().s(), traj_pt.a());
      debug_traj.AddPoint("traj_sk", traj_pt.path_point().s(),
                          traj_pt.path_point().kappa());
    }
    // debug_traj.PrintToLog();
    ADEBUG << "current_time_stamp: " << current_time_stamp;

    /**
     * PrependTrajectoryPoints - 在轨迹前添加缝合轨迹点
     * stitching_trajectory的最后一个点是新规划的起点
     * 因此从begin到end-1（不包含最后一个，因为它是新轨迹的起点）
     */
    last_publishable_trajectory_->PrependTrajectoryPoints(
        std::vector<TrajectoryPoint>(stitching_trajectory.begin(),
                                     stitching_trajectory.end() - 1));

    /**
     * PopulateTrajectoryProtobuf - 将轨迹填充到protobuf消息
     */
    last_publishable_trajectory_->PopulateTrajectoryProtobuf(ptr_trajectory_pb);

    /**
     * 导出驾驶就绪建议
     */
    best_ref_info->ExportEngageAdvice(
        ptr_trajectory_pb->mutable_engage_advice(),
        injector_->planning_context());
  }

  /**
   * AddADCDrivingStateInfo - 添加ADC（自动驾驶汽车）驾驶状态信息
   */
  AddADCDrivingStateInfo(ptr_debug);

  return status;  /**< 返回规划状态 */
}

/**
 * @brief 检查规划配置有效性
 *
 * @param config 待检查的配置
 * @return bool 配置是否有效
 */
bool OnLanePlanning::CheckPlanningConfig(const PlanningConfig& config) {
  // TODO(All): check other config params
  return true;
}

/**
 * @brief 填充图表选项
 *
 * 辅助函数，用于设置图表的坐标轴范围和标签
 *
 * @param x_min, x_max X轴范围
 * @param x_label X轴标签
 * @param y_min, y_max Y轴范围
 * @param y_label Y轴标签
 * @param display 是否显示图例
 * @param chart 图表指针
 */
void PopulateChartOptions(double x_min, double x_max, std::string x_label,
                          double y_min, double y_max, std::string y_label,
                          bool display, Chart* chart) {
  auto* options = chart->mutable_options();
  options->mutable_x()->set_min(x_min);
  options->mutable_x()->set_max(x_max);
  options->mutable_y()->set_min(y_min);
  options->mutable_y()->set_max(y_max);
  options->mutable_x()->set_label_string(x_label);
  options->mutable_y()->set_label_string(y_label);
  options->set_legend_display(display);
}

/**
 * @brief 添加ST图到图表
 *
 * ST图表示空间-时间关系，是速度规划的核心调试工具
 *
 * @param st_graph ST图调试数据
 * @param chart 图表指针
 */
void AddSTGraph(const STGraphDebug& st_graph, Chart* chart) {
  /**
   * 根据ST图名称设置图表标题
   * DP_ST_SPEED_OPTIMIZER是动态规划速度优化器
   */
  if (st_graph.name() == "DP_ST_SPEED_OPTIMIZER") {
    chart->set_title("Speed Heuristic");
  } else {
    chart->set_title("Planning S-T Graph");
  }

  /**
   * PopulateChartOptions - 设置图表坐标轴
   * X轴是时间t（秒），Y轴是空间s（米）
   */
  PopulateChartOptions(-2.0, 10.0, "t (second)", -10.0, 220.0, "s (meter)",
                       false, chart);

  /**
   * 遍历ST边界
   * ST边界表示障碍物在ST图中的占据区域
   */
  for (const auto& boundary : st_graph.boundary()) {
    /**
     * StGraphBoundaryDebug_StBoundaryType_Name - 获取边界类型的名称
     * substr(17)去掉前缀"ST_BOUNDARY_TYPE_"
     * 得到如"DRIVABLE_REGION"、"VELOW_OBS"等
     */
    // from 'ST_BOUNDARY_TYPE_' to the end
    std::string type =
        StGraphBoundaryDebug_StBoundaryType_Name(boundary.type()).substr(17);

    auto* boundary_chart = chart->add_polygon();

    /**
     * 设置多边形的渲染属性
     */
    auto* properties = boundary_chart->mutable_properties();
    (*properties)["borderWidth"] = "2";
    (*properties)["pointRadius"] = "0";
    (*properties)["lineTension"] = "0";
    (*properties)["cubicInterpolationMode"] = "monotone";
    (*properties)["showLine"] = "true";
    (*properties)["showText"] = "true";
    (*properties)["fill"] = "false";

    /**
     * 根据边界类型设置颜色
     * DRIVABLE_REGION是绿色（可行驶区域）
     * 其他是红色（障碍物）
     */
    if (type == "DRIVABLE_REGION") {
      (*properties)["color"] = "\"rgba(0, 255, 0, 0.5)\"";
    } else {
      (*properties)["color"] = "\"rgba(255, 0, 0, 0.8)\"";
    }

    /**
     * 设置多边形标签
     */
    boundary_chart->set_label(boundary.name() + "_" + type);

    /**
     * 添加多边形的顶点
     * 每个点有t（时间）和s（空间）两个坐标
     */
    for (const auto& point : boundary.point()) {
      auto* point_debug = boundary_chart->add_point();
      point_debug->set_x(point.t());
      point_debug->set_y(point.s());
    }
  }

  /**
   * 添加速度曲线
   * 速度曲线是ST图中的绿色/白色线
   */
  auto* speed_profile = chart->add_line();
  auto* properties = speed_profile->mutable_properties();
  (*properties)["color"] = "\"rgba(255, 255, 255, 0.5)\"";
  for (const auto& point : st_graph.speed_profile()) {
    auto* point_debug = speed_profile->add_point();
    point_debug->set_x(point.t());
    point_debug->set_y(point.s());
  }
}

/**
 * @brief 添加SL帧图
 *
 * SL图表示沿车道纵向(S)和横向(L)的关系
 * 用于路径规划和车道偏移分析
 *
 * @param sl_frame SL帧调试数据
 * @param chart 图表指针
 */
void AddSLFrame(const SLFrameDebug& sl_frame, Chart* chart) {
  chart->set_title(sl_frame.name());

  /**
   * PopulateChartOptions - 设置图表
   * X轴是s（沿车道纵向，米）
   * Y轴是l（横向偏移，米）
   */
  PopulateChartOptions(0.0, 80.0, "s (meter)", -8.0, 8.0, "l (meter)", false,
                       chart);

  auto* sl_line = chart->add_line();
  sl_line->set_label("SL Path");

  /**
   * 遍历SL路径上的点
   */
  for (const auto& sl_point : sl_frame.sl_path()) {
    auto* point_debug = sl_line->add_point();
    point_debug->set_x(sl_point.s());
    point_debug->set_x(sl_point.l());  /**< 注意这里应该是set_y，原代码有bug */
  }
}

/**
 * @brief 添加速度规划图表
 *
 * 绘制速度-距离(s-v)曲线
 *
 * @param speed_plans 速度规划序列
 * @param chart 图表指针
 */
void AddSpeedPlan(
    const ::google::protobuf::RepeatedPtrField<SpeedPlan>& speed_plans,
    Chart* chart) {
  chart->set_title("Speed Plan");

  /**
   * X轴是s（距离），Y轴是v（速度）
   */
  PopulateChartOptions(0.0, 80.0, "s (meter)", 0.0, 50.0, "v (m/s)", false,
                       chart);

  /**
   * 遍历所有速度规划
   * 通常有多个速度规划：DP速度规划、QP速度规划等
   */
  for (const auto& speed_plan : speed_plans) {
    auto* line = chart->add_line();
    line->set_label(speed_plan.name());

    /**
     * 绘制速度曲线
     */
    for (const auto& point : speed_plan.speed_point()) {
      auto* point_debug = line->add_point();
      point_debug->set_x(point.s());
      point_debug->set_y(point.v());
    }

    /**
     * 设置线条渲染属性
     * 不同规划器用不同颜色区分
     */
    auto* properties = line->mutable_properties();
    (*properties)["borderWidth"] = "2";
    (*properties)["pointRadius"] = "0";
    (*properties)["fill"] = "false";
    (*properties)["showLine"] = "true";

    /**
     * DpStSpeedOptimizer - 动态规划速度优化器（绿色）
     * QpSplineStSpeedOptimizer - QP样条速度优化器（蓝色）
     */
    if (speed_plan.name() == "DpStSpeedOptimizer") {
      (*properties)["color"] = "\"rgba(27, 249, 105, 0.5)\"";
    } else if (speed_plan.name() == "QpSplineStSpeedOptimizer") {
      (*properties)["color"] = "\"rgba(54, 162, 235, 1)\"";
    }
  }
}

/**
 * @brief 导出失败换道的ST图
 *
 * @param debug_info 调试信息输入
 * @param debug_chart 图表输出
 */
void OnLanePlanning::ExportFailedLaneChangeSTChart(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) {
  const auto& src_data = debug_info.planning_data();
  auto* dst_data = debug_chart->mutable_planning_data();

  /**
   * 遍历所有ST图并添加到输出
   */
  for (const auto& st_graph : src_data.st_graph()) {
    AddSTGraph(st_graph, dst_data->add_chart());
  }
}

/**
 * @brief 导出车道内规划的图表
 *
 * @param debug_info 调试信息输入
 * @param debug_chart 图表输出
 */
void OnLanePlanning::ExportOnLaneChart(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) {
  const auto& src_data = debug_info.planning_data();
  auto* dst_data = debug_chart->mutable_planning_data();

  /**
   * 添加ST图
   */
  for (const auto& st_graph : src_data.st_graph()) {
    AddSTGraph(st_graph, dst_data->add_chart());
  }

  /**
   * 添加SL帧图
   */
  for (const auto& sl_frame : src_data.sl_frame()) {
    AddSLFrame(sl_frame, dst_data->add_chart());
  }

  /**
   * 添加速度规划图
   */
  AddSpeedPlan(src_data.speed_plan(), dst_data->add_chart());
}

/**
 * @brief 导出开放空间规划的图表
 *
 * @param debug_info 调试信息输入
 * @param trajectory_pb 轨迹消息
 * @param debug_chart 图表输出
 */
void OnLanePlanning::ExportOpenSpaceChart(
    const planning_internal::Debug& debug_info,
    const ADCTrajectory& trajectory_pb, planning_internal::Debug* debug_chart) {
  // Export Trajectory Visualization Chart.
  if (FLAGS_enable_record_debug) {
    AddOpenSpaceOptimizerResult(debug_info, debug_chart);
    AddPartitionedTrajectory(debug_info, debug_chart);
    AddStitchSpeedProfile(debug_chart);
    AddPublishedSpeed(trajectory_pb, debug_chart);
    AddPublishedAcceleration(trajectory_pb, debug_chart);
    // AddFallbackTrajectory(debug_info, debug_chart);
  }
}

/**
 * @brief 添加开放空间优化器结果
 *
 * @param debug_info 调试信息
 * @param debug_chart 图表
 */
void OnLanePlanning::AddOpenSpaceOptimizerResult(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) {
  /**
   * 检查开放空间信息提供者是否成功运行
   */
  if (!frame_->open_space_info().open_space_provider_success()) {
    return;
  }

  auto chart = debug_chart->mutable_planning_data()->add_chart();
  auto open_space_debug = debug_info.planning_data().open_space();

  chart->set_title("Open Space Trajectory Optimizer Visualization");

  /**
   * PopulateChartOptions - 设置图表范围
   * 使用xy_boundary来确定X和Y的范围
   * xy_boundary是一个包含4个值的数组：[xmin, xmax, ymin, ymax]
   */
  PopulateChartOptions(open_space_debug.xy_boundary(0) - 1.0,
                       open_space_debug.xy_boundary(1) + 1.0, "x (meter)",
                       open_space_debug.xy_boundary(2) - 1.0,
                       open_space_debug.xy_boundary(3) + 1.0, "y (meter)", true,
                       chart);

  chart->mutable_options()->set_sync_xy_window_size(true);
  chart->mutable_options()->set_aspect_ratio(0.9);

  /**
   * 绘制障碍物轮廓
   */
  int obstacle_index = 1;
  for (const auto& obstacle : open_space_debug.obstacles()) {
    auto* obstacle_outline = chart->add_line();
    obstacle_outline->set_label(absl::StrCat("Bdr", obstacle_index));
    obstacle_index += 1;

    /**
     * vertices_x_coords_size() - 获取障碍物顶点数
     * vertices_x_coords(i) / vertices_y_coords(i) - 获取第i个顶点的坐标
     */
    for (int vertice_index = 0;
         vertice_index < obstacle.vertices_x_coords_size(); vertice_index++) {
      auto* point_debug = obstacle_outline->add_point();
      point_debug->set_x(obstacle.vertices_x_coords(vertice_index));
      point_debug->set_y(obstacle.vertices_y_coords(vertice_index));
    }

    auto* obstacle_properties = obstacle_outline->mutable_properties();
    (*obstacle_properties)["borderWidth"] = "2";
    (*obstacle_properties)["pointRadius"] = "0";
    (*obstacle_properties)["lineTension"] = "0";
    (*obstacle_properties)["fill"] = "false";
    (*obstacle_properties)["showLine"] = "true";
  }

  /**
   * 绘制平滑后的轨迹
   */
  auto smoothed_trajectory = open_space_debug.smoothed_trajectory();
  auto* smoothed_line = chart->add_line();
  smoothed_line->set_label("Smooth");

  /**
   * vehicle_motion_point_size() / 2 - 轨迹点数除以2
   * 因为每个点包含x和y，但存储在同一个数组里
   */
  for (int i = 0; i < smoothed_trajectory.vehicle_motion_point_size() / 2;
       i++) {
    auto& point = smoothed_trajectory.vehicle_motion_point(i);
    const auto x = point.trajectory_point().path_point().x();
    const auto y = point.trajectory_point().path_point().y();

    auto* point_debug = smoothed_line->add_point();
    point_debug->set_x(x);
    point_debug->set_y(y);
  }

  auto* smoothed_properties = smoothed_line->mutable_properties();
  (*smoothed_properties)["borderWidth"] = "2";
  (*smoothed_properties)["pointRadius"] = "0";
  (*smoothed_properties)["lineTension"] = "0";
  (*smoothed_properties)["fill"] = "false";
  (*smoothed_properties)["showLine"] = "true";

  /**
   * 绘制warm start轨迹
   * warm start是优化算法的初始猜测
   */
  auto warm_start_trajectory = open_space_debug.warm_start_trajectory();
  auto* warm_start_line = chart->add_line();
  warm_start_line->set_label("WarmStart");
  for (int i = 0; i < warm_start_trajectory.vehicle_motion_point_size() / 2;
       i++) {
    auto* point_debug = warm_start_line->add_point();
    auto& point = warm_start_trajectory.vehicle_motion_point(i);
    point_debug->set_x(point.trajectory_point().path_point().x());
    point_debug->set_y(point.trajectory_point().path_point().y());
  }

  auto* warm_start_properties = warm_start_line->mutable_properties();
  (*warm_start_properties)["borderWidth"] = "2";
  (*warm_start_properties)["pointRadius"] = "0";
  (*warm_start_properties)["lineTension"] = "0";
  (*warm_start_properties)["fill"] = "false";
  (*warm_start_properties)["showLine"] = "true";
}

/**
 * @brief 添加分区轨迹到图表
 *
 * @param debug_info 调试信息
 * @param debug_chart 图表
 */
void OnLanePlanning::AddPartitionedTrajectory(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) {
  if (!frame_->open_space_info().open_space_provider_success()) {
    return;
  }

  const auto& open_space_debug = debug_info.planning_data().open_space();
  const auto& chosen_trajectories =
      open_space_debug.chosen_trajectory().trajectory();

  if (chosen_trajectories.empty() ||
      chosen_trajectories[0].trajectory_point().empty()) {
    return;
  }

  const auto& vehicle_state = frame_->vehicle_state();

  /**
   * 创建多个图表：轨迹、kappa、theta
   */
  auto chart = debug_chart->mutable_planning_data()->add_chart();
  auto chart_kappa = debug_chart->mutable_planning_data()->add_chart();
  auto chart_theta = debug_chart->mutable_planning_data()->add_chart();

  chart->set_title("Open Space Partitioned Trajectory");
  chart_kappa->set_title("total kappa");
  chart_theta->set_title("total theta");

  auto* options = chart->mutable_options();
  options->mutable_x()->set_label_string("x (meter)");
  options->mutable_y()->set_label_string("y (meter)");
  options->set_sync_xy_window_size(true);
  options->set_aspect_ratio(0.9);

  /**
   * 绘制自车形状
   */
  auto* adc_shape = chart->add_car();
  adc_shape->set_x(vehicle_state.x());
  adc_shape->set_y(vehicle_state.y());
  adc_shape->set_heading(vehicle_state.heading());
  adc_shape->set_label("ADV");
  adc_shape->set_color("rgba(54, 162, 235, 1)");

  /**
   * 绘制选中的轨迹
   */
  const auto& chosen_trajectory = chosen_trajectories[0];
  auto* chosen_line = chart->add_line();
  chosen_line->set_label("Chosen");
  for (const auto& point : chosen_trajectory.trajectory_point()) {
    auto* point_debug = chosen_line->add_point();
    point_debug->set_x(point.path_point().x());
    point_debug->set_y(point.path_point().y());
  }

  auto* chosen_properties = chosen_line->mutable_properties();
  (*chosen_properties)["borderWidth"] = "2";
  (*chosen_properties)["pointRadius"] = "0";
  (*chosen_properties)["lineTension"] = "0";
  (*chosen_properties)["fill"] = "false";
  (*chosen_properties)["showLine"] = "true";

  auto* theta_line = chart_theta->add_line();
  auto* kappa_line = chart_kappa->add_line();

  /**
   * 绘制所有分区轨迹
   */
  size_t partitioned_trajectory_label = 0;
  for (const auto& partitioned_trajectory :
       open_space_debug.partitioned_trajectories().trajectory()) {
    auto* partition_line = chart->add_line();
    partition_line->set_label(
        absl::StrCat("Partitioned ", partitioned_trajectory_label));
    ++partitioned_trajectory_label;

    for (const auto& point : partitioned_trajectory.trajectory_point()) {
      auto* point_debug = partition_line->add_point();
      auto* point_theta = theta_line->add_point();
      auto* point_kappa = kappa_line->add_point();

      point_debug->set_x(point.path_point().x());
      point_debug->set_y(point.path_point().y());
      point_theta->set_x(point.relative_time());
      point_kappa->set_x(point.relative_time());
      point_theta->set_y(point.path_point().theta());
      point_kappa->set_y(point.path_point().kappa());
    }

    auto* partition_properties = partition_line->mutable_properties();
    (*partition_properties)["borderWidth"] = "2";
    (*partition_properties)["pointRadius"] = "0";
    (*partition_properties)["lineTension"] = "0";
    (*partition_properties)["fill"] = "false";
    (*partition_properties)["showLine"] = "true";

    SetChartminmax(chart_kappa, "time", "total kappa");
    SetChartminmax(chart_theta, "time", "total theta");
  }
}

/**
 * @brief 添加缝合速度曲线
 *
 * @param debug_chart 图表
 */
void OnLanePlanning::AddStitchSpeedProfile(
    planning_internal::Debug* debug_chart) {
  if (!injector_->frame_history()->Latest()) {
    AINFO << "Planning frame is empty!";
    return;
  }

  if (!frame_->open_space_info().open_space_provider_success()) {
    return;
  }

  auto chart = debug_chart->mutable_planning_data()->add_chart();
  chart->set_title("Open Space Speed Plan Visualization");
  auto* options = chart->mutable_options();

  /**
   * 初始化边界值
   */
  double xmin(std::numeric_limits<double>::max()),
      xmax(std::numeric_limits<double>::lowest()),
      ymin(std::numeric_limits<double>::max()),
      ymax(std::numeric_limits<double>::lowest());

  auto* speed_profile = chart->add_line();
  speed_profile->set_label("Speed Profile");

  /**
   * 从历史帧中获取上一次的轨迹
   * Latest()获取最新帧
   */
  const auto& last_trajectory =
      injector_->frame_history()->Latest()->current_frame_planned_trajectory();

  /**
   * 绘制速度曲线
   * x是绝对时间（时间戳），y是速度
   */
  for (const auto& point : last_trajectory.trajectory_point()) {
    auto* point_debug = speed_profile->add_point();
    point_debug->set_x(point.relative_time() +
                       last_trajectory.header().timestamp_sec());
    point_debug->set_y(point.v());

    if (point_debug->x() > xmax) xmax = point_debug->x();
    if (point_debug->x() < xmin) xmin = point_debug->x();
    if (point_debug->y() > ymax) ymax = point_debug->y();
    if (point_debug->y() < ymin) ymin = point_debug->y();
  }

  options->mutable_x()->set_window_size(xmax - xmin);
  options->mutable_x()->set_label_string("time (s)");
  options->mutable_y()->set_min(ymin);
  options->mutable_y()->set_max(ymax);
  options->mutable_y()->set_label_string("speed (m/s)");

  auto* speed_profile_properties = speed_profile->mutable_properties();
  (*speed_profile_properties)["borderWidth"] = "2";
  (*speed_profile_properties)["pointRadius"] = "0";
  (*speed_profile_properties)["lineTension"] = "0";
  (*speed_profile_properties)["fill"] = "false";
  (*speed_profile_properties)["showLine"] = "true";
}

/**
 * @brief 添加发布的速度曲线
 *
 * @param trajectory_pb 轨迹消息
 * @param debug_chart 图表
 */
void OnLanePlanning::AddPublishedSpeed(const ADCTrajectory& trajectory_pb,
                                       planning_internal::Debug* debug_chart) {
  if (!frame_->open_space_info().open_space_provider_success()) {
    return;
  }

  auto chart = debug_chart->mutable_planning_data()->add_chart();
  chart->set_title("Speed Partition Visualization");
  auto* options = chart->mutable_options();

  double xmin(std::numeric_limits<double>::max()),
      xmax(std::numeric_limits<double>::lowest()),
      ymin(std::numeric_limits<double>::max()),
      ymax(std::numeric_limits<double>::lowest());

  auto* speed_profile = chart->add_line();
  speed_profile->set_label("Speed Profile");

  /**
   * 遍历轨迹点
   * 根据档位决定速度方向
   * 前进档为正，倒档为负
   */
  for (const auto& point : trajectory_pb.trajectory_point()) {
    auto* point_debug = speed_profile->add_point();
    point_debug->set_x(point.relative_time() +
                       trajectory_pb.header().timestamp_sec());

    if (trajectory_pb.gear() == canbus::Chassis::GEAR_DRIVE) {
      point_debug->set_y(point.v());
    }
    if (trajectory_pb.gear() == canbus::Chassis::GEAR_REVERSE) {
      point_debug->set_y(-point.v());
    }

    if (point_debug->x() > xmax) xmax = point_debug->x();
    if (point_debug->x() < xmin) xmin = point_debug->x();
    if (point_debug->y() > ymax) ymax = point_debug->y();
    if (point_debug->y() < ymin) ymin = point_debug->y();
  }

  options->mutable_x()->set_window_size(xmax - xmin);
  options->mutable_x()->set_label_string("time (s)");
  options->mutable_y()->set_min(ymin);
  options->mutable_y()->set_max(ymax);
  options->mutable_y()->set_label_string("speed (m/s)");

  auto* speed_profile_properties = speed_profile->mutable_properties();
  (*speed_profile_properties)["borderWidth"] = "2";
  (*speed_profile_properties)["pointRadius"] = "0";
  (*speed_profile_properties)["lineTension"] = "0";
  (*speed_profile_properties)["fill"] = "false";
  (*speed_profile_properties)["showLine"] = "true";

  /**
   * 添加当前时间线（滑动线）
   */
  auto* sliding_line = chart->add_line();
  sliding_line->set_label("Time");

  auto* point_debug_up = sliding_line->add_point();
  point_debug_up->set_x(Clock::NowInSeconds());
  point_debug_up->set_y(2.1);

  auto* point_debug_down = sliding_line->add_point();
  point_debug_down->set_x(Clock::NowInSeconds());
  point_debug_down->set_y(-1.1);

  auto* sliding_line_properties = sliding_line->mutable_properties();
  (*sliding_line_properties)["borderWidth"] = "2";
  (*sliding_line_properties)["pointRadius"] = "0";
  (*sliding_line_properties)["lineTension"] = "0";
  (*sliding_line_properties)["fill"] = "false";
  (*sliding_line_properties)["showLine"] = "true";
}

/**
 * @brief 时间戳对齐
 *
 * 当消息有延迟时，根据时间差估算未来位置
 *
 * @param vehicle_state 原始车辆状态
 * @param curr_timestamp 当前时间戳
 * @return VehicleState 对齐后的车辆状态
 */
VehicleState OnLanePlanning::AlignTimeStamp(const VehicleState& vehicle_state,
                                            const double curr_timestamp) const {
  /**
   * EstimateFuturePosition - 估算未来位置
   * 根据时间差和当前速度估算
   */
  auto future_xy = injector_->vehicle_state()->EstimateFuturePosition(
      curr_timestamp - vehicle_state.timestamp());

  VehicleState aligned_vehicle_state = vehicle_state;
  aligned_vehicle_state.set_x(future_xy.x());
  aligned_vehicle_state.set_y(future_xy.y());
  aligned_vehicle_state.set_timestamp(curr_timestamp);
  return aligned_vehicle_state;
}

/**
 * @brief 添加发布的加速度曲线
 *
 * @param trajectory_pb 轨迹消息
 * @param debug 调试信息
 */
void OnLanePlanning::AddPublishedAcceleration(
    const ADCTrajectory& trajectory_pb, planning_internal::Debug* debug) {
  if (!frame_->open_space_info().open_space_provider_success()) {
    return;
  }

  double xmin(std::numeric_limits<double>::max()),
      xmax(std::numeric_limits<double>::lowest()),
      ymin(std::numeric_limits<double>::max()),
      ymax(std::numeric_limits<double>::lowest());

  auto chart = debug->mutable_planning_data()->add_chart();
  chart->set_title("Acceleration Partition Visualization");
  auto* options = chart->mutable_options();

  auto* acceleration_profile = chart->add_line();
  acceleration_profile->set_label("Acceleration Profile");

  /**
   * 绘制加速度曲线
   * x是绝对时间，y是加速度
   */
  for (const auto& point : trajectory_pb.trajectory_point()) {
    auto* point_debug = acceleration_profile->add_point();
    point_debug->set_x(point.relative_time() +
                       trajectory_pb.header().timestamp_sec());

    /**
     * 根据档位决定加速度方向
     */
    if (trajectory_pb.gear() == canbus::Chassis::GEAR_DRIVE)
      point_debug->set_y(point.a());
    if (trajectory_pb.gear() == canbus::Chassis::GEAR_REVERSE)
      point_debug->set_y(-point.a());

    if (point_debug->x() > xmax) xmax = point_debug->x();
    if (point_debug->x() < xmin) xmin = point_debug->x();
    if (point_debug->y() > ymax) ymax = point_debug->y();
    if (point_debug->y() < ymin) ymin = point_debug->y();
  }

  options->mutable_x()->set_window_size(xmax - xmin);
  options->mutable_x()->set_label_string("time (s)");
  options->mutable_y()->set_min(ymin);
  options->mutable_y()->set_max(ymax);
  options->mutable_y()->set_label_string("acceleration (m/s)");

  auto* acceleration_profile_properties =
      acceleration_profile->mutable_properties();
  (*acceleration_profile_properties)["borderWidth"] = "2";
  (*acceleration_profile_properties)["pointRadius"] = "0";
  (*acceleration_profile_properties)["lineTension"] = "0";
  (*acceleration_profile_properties)["fill"] = "false";
  (*acceleration_profile_properties)["showLine"] = "true";

  /**
   * 添加当前时间滑动线
   */
  auto* sliding_line = chart->add_line();
  sliding_line->set_label("Time");

  auto* point_debug_up = sliding_line->add_point();
  point_debug_up->set_x(Clock::NowInSeconds());
  point_debug_up->set_y(2.1);

  auto* point_debug_down = sliding_line->add_point();
  point_debug_down->set_x(Clock::NowInSeconds());
  point_debug_down->set_y(-1.1);

  auto* sliding_line_properties = sliding_line->mutable_properties();
  (*sliding_line_properties)["borderWidth"] = "2";
  (*sliding_line_properties)["pointRadius"] = "0";
  (*sliding_line_properties)["lineTension"] = "0";
  (*sliding_line_properties)["fill"] = "false";
  (*sliding_line_properties)["showLine"] = "true";
}

/**
 * @brief 添加ADC驾驶状态信息
 *
 * @param debug 调试信息指针
 */
void OnLanePlanning::AddADCDrivingStateInfo(
    planning_internal::Debug* const debug) {
  /**
   * 设置前方清除距离
   */
  debug->mutable_planning_data()->set_front_clear_distance(
      injector_->ego_info()->front_clear_distance());

  /**
   * DrivingStateGenerator - 驾驶状态生成器
   * generate()生成驾驶状态调试信息
   */
  DrivingStateGenerator state_generator;
  state_generator.generate(
      frame_->DriveReferenceLineInfo(), injector_->ego_info(),
      debug->mutable_planning_data()->mutable_adc_driving_state());
}

}  // namespace planning
}  // namespace apollo
