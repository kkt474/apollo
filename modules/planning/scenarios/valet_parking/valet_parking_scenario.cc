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
 * @file valet_parking_scenario.cc
 * @brief 自主泊车（Valet Parking）场景实现文件
 *
 * 功能说明：
 * 自主泊车场景是Apollo规划模块中用于实现自动泊车功能的核心组件
 * 车辆可以自动寻找停车位、规划路径并完成泊入操作
 *
 * 应用场景：
 * - 停车场自动泊车（AVP - Autonomous Valet Parking）
 * - 召唤车辆到指定停车位
 * - 从停车场自动驶出
 *
 * 工作流程：
 * 1. 接收泊车命令（parking_command）
 * 2. 检查是否有有效的目标停车位
 * 3. 验证车辆与停车位的距离是否在可启动范围内
 * 4. 进入泊车阶段执行
 */

#include "modules/planning/scenarios/valet_parking/valet_parking_scenario.h"

/**
 * @brief 头文件包含
 *
 * C++语法说明：
 * - #include "modules/planning/planning_base/common/frame.h"：
 *   Frame类定义，规划帧数据结构
 *   包含当前帧的所有规划相关信息
 *
 * - #include "modules/planning/scenarios/valet_parking/stage_approaching_parking_spot.h"：
 *   接近停车位阶段的定义
 *
 * - #include "modules/planning/scenarios/valet_parking/stage_parking.h"：
 *   泊车阶段的定义
 */

/**
 * @brief Apollo命名空间开始
 *
 * C++语法说明：
 * - namespace apollo：最外层命名空间，Apollo项目所有代码都在此
 * - namespace planning：规划模块子命名空间
 */
namespace apollo {
namespace planning {

/**
 * @brief 类型别名声明
 *
 * 功能说明：
 * 为常用类型创建别名，简化代码书写，提高可读性
 *
 * C++语法说明：
 * - using XXX = YYY：
 *   类型别名声明，与typedef等价但更直观
 *   在大型项目中常用此方式简化长类型名
 *
 * - apollo::common::VehicleState：
 *   车辆状态类，包含位置、速度、朝向等信息
 *
 * - apollo::common::math::Vec2d：
 *   2D向量类，用于几何计算
 *
 * - apollo::hdmap::ParkingSpaceInfoConstPtr：
 *   停车场位信息的常量指针类型
 *   命名规则：XXXConstPtr表示常量指针
 *
 * - apollo::hdmap::Path：
 *   地图路径类，包含路径点和路径信息
 *
 * - apollo::hdmap::PathOverlap：
 *   路径重叠区域类，用于表示停车位等重叠区域
 */
using apollo::common::VehicleState;
using apollo::common::math::Vec2d;
using apollo::hdmap::ParkingSpaceInfoConstPtr;
using apollo::hdmap::Path;
using apollo::hdmap::PathOverlap;

/**
 * @brief 初始化泊车场景
 *
 * @param injector 依赖注入器指针，用于获取各种服务
 * @param name 场景名称
 * @return bool 初始化是否成功
 *
 * 功能说明：
 * 初始化自主泊车场景，包括：
 * 1. 调用基类Scenario的Init方法进行基础初始化
 * 2. 加载泊车场景配置
 * 3. 获取HD地图指针
 *
 * 算法流程：
 * 1. 检查是否已初始化，如果已初始化则直接返回true
 * 2. 调用基类Init进行通用初始化
 * 3. 加载ScenarioValetParkingConfig配置
 * 4. 获取HD地图实例
 *
 * C++语法说明：
 * - std::shared_ptr<DependencyInjector>：
 *   共享指针，多个所有者共享同一个DependencyInjector对象
 *   当最后一个指针销毁时自动删除对象
 *   相比unique_ptr允许复制共享所有权
 *
 * - const std::string& name：
 *   常量引用，传入场景名称
 *   使用引用避免字符串拷贝，const防止修改
 *
 * - if (!Scenario::Init(...))：
 *   作用域解析运算符::调用基类的Init方法
 *   逻辑非运算符!判断返回的bool值
 *
 * - Scenario::LoadConfig<T>(...)：
 *   模板方法，从配置文件加载指定类型T的配置
 *   T = ScenarioValetParkingConfig
 *
 * - hdmap::HDMapUtil::BaseMapPtr()：
 *   静态方法，获取HD地图的共享指针
 *   HDMapUtil是地图工具类单例
 *
 * - CHECK_NOTNULL(hdmap_)：
 *   断言宏，检查指针是否为空
 *   如果为空程序会报错并终止
 *   不同于if(ptr == nullptr)，CHECK_NOTNULL在release模式也会执行
 *
 * - init_ = true：
 *   设置初始化标志，防止重复初始化
 */
bool ValetParkingScenario::Init(std::shared_ptr<DependencyInjector> injector,
                                const std::string& name) {
  // 检查是否已经初始化
  // 如果已初始化，直接返回true，避免重复初始化
  if (init_) {
    return true;
  }

  // 调用基类Scenario的Init方法进行通用初始化
  // 包括初始化依赖注入器、场景名称等
  if (!Scenario::Init(injector, name)) {
    AERROR << "failed to init scenario" << Name();  // 输出错误日志
    return false;  // 基类初始化失败，返回false
  }

  // 加载泊车场景的特定配置
  // LoadConfig是模板方法，从配置系统加载指定类型的配置
  // &context_.scenario_config传入配置存储地址
  if (!Scenario::LoadConfig<ScenarioValetParkingConfig>(
          &context_.scenario_config)) {
    AERROR << "fail to get config of scenario" << Name();  // 输出错误日志
    return false;  // 配置加载失败，返回false
  }

  // 获取HD地图的共享指针
  // HDMapUtil::BaseMapPtr()是静态方法，直接通过类名调用
  hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
  
  // 断言检查hdmap_指针非空
  // 确保地图数据可用，否则后续操作无意义
  CHECK_NOTNULL(hdmap_);
  
  // 设置初始化标志
  init_ = true;
  return true;  // 初始化成功
}

/**
 * @brief 判断是否可以转换到泊车场景
 *
 * @param other_scenario 当前场景指针
 * @param frame 规划帧数据
 * @return bool 如果可以转换返回true
 *
 * 功能说明：
 * 判断当前是否可以退出上一个场景并进入自主泊车场景
 * 这是场景管理器进行场景转换时的判断依据
 *
 * 判断条件：
 * 1. 规划命令中包含泊车命令（parking_command）
 * 2. 存在有效的参考线信息
 * 3. 目标停车位ID有效
 * 4. 停车位存在于地图路径上
 * 5. 车辆与停车位的距离在可启动范围内
 *
 * C++语法说明：
 * - const Scenario* const other_scenario：
 *   指向常量的常量指针
 *   第一个const：指针指向的内容不可修改
 *   第二个const：指针本身不可修改指向
 *
 * - const Frame& frame：
 *   常量引用，传入规划帧数据
 *   使用引用避免拷贝大型结构
 *
 * - frame.local_view().planning_command：
 *   链式调用访问local_view中的planning_command
 *   local_view包含规划所需的所有输入数据
 *
 * - has_parking_command()：
 *   Protobuf消息的方法，检查字段是否存在
 *   planning_command是嵌套的protobuf消息
 *
 * - frame.reference_line_info().empty()：
 *   检查参考线信息容器是否为空
 *   如果为空，说明缺少必要的路径信息
 *
 * - ->parking_command().parking_spot_id()：
 *   箭头运算符->访问嵌套protobuf消息
 *   parking_command包含泊车相关命令
 *
 * - context_.target_parking_spot_id：
 *   context_是场景的上下文数据成员
 *   存储当前场景的目标停车位ID
 */
bool ValetParkingScenario::IsTransferable(const Scenario* const other_scenario,
                                          const Frame& frame) {
  // TODO(all) Implement available parking spot detection by preception results
  // TODO: 应该通过感知结果实现可用停车位检测，而不是仅依赖路由命令

  // 检查规划命令是否包含泊车命令
  // 如果没有泊车命令，说明用户没有要求泊车
  if (!frame.local_view().planning_command->has_parking_command()) {
    return false;  // 没有泊车命令，不能转换到泊车场景
  }

  // 检查其他场景指针和参考线信息有效性
  if (other_scenario == nullptr || frame.reference_line_info().empty()) {
    return false;  // 缺少必要的场景或路径信息
  }

  // 提取目标停车位ID
  std::string target_parking_spot_id;
  
  // 再次检查泊车命令和停车位ID
  if (frame.local_view().planning_command->has_parking_command() &&
      frame.local_view()
          .planning_command->parking_command()
          .has_parking_spot_id()) {
    // 从泊车命令中获取目标停车位ID
    target_parking_spot_id = frame.local_view()
                                 .planning_command->parking_command()
                                 .parking_spot_id();
  } else {
    // 如果没有停车位ID，输出调试信息并返回
    ADEBUG << "No parking space id from routing";
    return false;
  }

  // 检查目标停车位ID是否有效
  if (target_parking_spot_id.empty()) {
    return false;  // 空ID无效
  }

  // 获取参考线对应的地图路径
  const auto& nearby_path =
      frame.reference_line_info().front().reference_line().map_path();
  
  // 停车位重叠区域变量
  PathOverlap parking_space_overlap;
  
  // 获取车辆状态
  const auto& vehicle_state = frame.vehicle_state();

  // 在路径上搜索目标停车位
  if (!SearchTargetParkingSpotOnPath(nearby_path, target_parking_spot_id,
                                     &parking_space_overlap)) {
    ADEBUG << "No such parking spot found after searching all path forward "
              "possible"
           << target_parking_spot_id;
    return false;  // 找不到目标停车位
  }

  // 获取配置中的启动距离阈值
  double parking_spot_range_to_start =
      context_.scenario_config.parking_spot_range_to_start();
  
  // 检查车辆与停车位的距离是否满足启动条件
  if (!CheckDistanceToParkingSpot(frame, vehicle_state, nearby_path,
                                  parking_spot_range_to_start,
                                  parking_space_overlap)) {
    ADEBUG << "target parking spot found, but too far, distance larger than "
              "pre-defined distance"
           << target_parking_spot_id;
    return false;  // 距离太远，不适合启动泊车
  }

  // 所有检查通过，设置目标停车位ID到上下文
  context_.target_parking_spot_id = target_parking_spot_id;
  return true;  // 可以转换到泊车场景
}

/**
 * @brief 在路径上搜索目标停车位
 *
 * @param nearby_path 地图路径
 * @param target_parking_id 目标停车位ID
 * @param parking_space_overlap 输出：停车位重叠区域信息
 * @return bool 是否找到目标停车位
 *
 * 功能说明：
 * 在给定的地图路径上查找指定ID的停车位
 *
 * 算法流程：
 * 1. 获取路径上的所有停车位重叠区域
 * 2. 遍历比较每个停车位的ID
 * 3. 找到匹配的目标停车位并返回
 *
 * C++语法说明：
 * - const Path& nearby_path：
 *   常量引用，传入地图路径
 *   使用引用避免拷贝整个路径结构
 *
 * - const std::string& target_parking_id：
 *   常量引用，传入目标停车位ID字符串
 *
 * - PathOverlap* parking_space_overlap：
 *   指针参数，用于输出停车位重叠区域
 *   使用指针允许修改外部变量
 *
 * - nearby_path.parking_space_overlaps()：
 *   获取路径上所有停车位重叠区域的列表
 *   返回类型是vector或类似容器
 *
 * - for (const auto& parking_overlap : parking_space_overlaps)：
 *   范围for循环，遍历所有停车位重叠区域
 *   const auto&避免拷贝，提高效率
 *
 * - parking_overlap.object_id == target_parking_id：
 *   比较停车位ID与目标ID
 *   object_id是PathOverlap结构体的成员
 *
 * - *parking_space_overlap = parking_overlap：
 *   解引用指针并赋值
 *   将找到的停车位信息复制到输出参数
 */
bool ValetParkingScenario::SearchTargetParkingSpotOnPath(
    const Path& nearby_path, const std::string& target_parking_id,
    PathOverlap* parking_space_overlap) {
  // 获取路径上的所有停车位重叠区域
  const auto& parking_space_overlaps = nearby_path.parking_space_overlaps();

  // 遍历所有停车位重叠区域
  for (const auto& parking_overlap : parking_space_overlaps) {
    // 比较停车位ID是否匹配目标ID
    if (parking_overlap.object_id == target_parking_id) {
      // 找到匹配，将结果复制到输出参数
      *parking_space_overlap = parking_overlap;
      return true;  // 找到目标停车位
    }
  }
  
  return false;  // 未找到目标停车位
}

/**
 * @brief 检查到停车位的距离是否满足条件
 *
 * @param frame 规划帧数据
 * @param vehicle_state 车辆状态
 * @param nearby_path 地图路径
 * @param parking_start_range 启动距离阈值
 * @param parking_space_overlap 停车位重叠区域
 * @return bool 如果距离满足条件返回true
 *
 * 功能说明：
 * 计算车辆与目标停车位中心的距离
 * 判断是否小于启动泊车的距离阈值
 *
 * 算法流程：
 * 1. 通过停车位ID获取停车位的详细信息
 * 2. 计算停车位的几何中心点
 * 3. 将中心点和车辆位置投影到参考线上
 * 4. 比较两者的s坐标差值与阈值
 *
 * C++语法说明：
 * - const hdmap::HDMap* hdmap：
 *   指向HD地图的原始指针
 *   用于查询地图元素信息
 *
 * - hdmap::HDMapUtil::BaseMapPtr()：
 *   静态方法获取地图指针
 *   BaseMapPtr返回共享指针，这里赋值给原始指针
 *
 * - hdmap::Id id：
 *   地图元素ID结构
 *   用于唯一标识地图元素
 *
 * - id.set_id(parking_space_overlap.object_id)：
 *   调用set_id方法设置ID值
 *   parking_space_overlap.object_id是停车位的唯一标识符
 *
 * - hdmap->GetParkingSpaceById(id)：
 *   通过ID查询停车位信息
 *   返回ParkingSpaceInfoConstPtr类型的指针
 *
 * - target_parking_spot_ptr->polygon().points().at(0)：
 *   箭头运算符访问成员
 *   ->polygon()获取停车位多边形
 *   ->points()获取多边形的顶点列表
 *   ->at(0)获取第一个顶点（带边界检查）
 *
 * - Vec2d center_point = (left_bottom_point + ... + left_top_point) / 4.0：
 *   向量加法和标量除法
 *   计算四个角点的平均值得到中心点
 *
 * - nearby_path.GetNearestPoint(center_point, &center_point_s, &center_point_l)：
 *   将笛卡尔坐标点投影到路径上
 *   获取该点在路径上的s（沿路径距离）和l（横向距离）坐标
 *
 * - std::abs(center_point_s - vehicle_point_s) < parking_start_range：
 *   std::abs计算绝对值
 *   比较距离差值与阈值
 */
bool ValetParkingScenario::CheckDistanceToParkingSpot(
    const Frame& frame, const VehicleState& vehicle_state,
    const Path& nearby_path, const double parking_start_range,
    const PathOverlap& parking_space_overlap) {
  // TODO(Jinyun) parking overlap s are wrong on map, not usable
  // TODO: 地图上停车位的s坐标可能有误，暂时不使用

  // 获取HD地图指针
  const hdmap::HDMap* hdmap = hdmap::HDMapUtil::BaseMapPtr();
  
  // 创建停车位ID对象
  hdmap::Id id;
  id.set_id(parking_space_overlap.object_id);  // 设置ID为停车位的object_id
  
  // 通过ID获取停车位的详细信息
  ParkingSpaceInfoConstPtr target_parking_spot_ptr =
      hdmap->GetParkingSpaceById(id);

  // 获取停车位多边形的四个角点
  // 假设多边形按顺时针或逆时针顺序存储四个顶点
  Vec2d left_bottom_point = target_parking_spot_ptr->polygon().points().at(0);  // 左下角
  Vec2d right_bottom_point = target_parking_spot_ptr->polygon().points().at(1);  // 右下角
  Vec2d right_top_point = target_parking_spot_ptr->polygon().points().at(2);  // 右上角
  Vec2d left_top_point = target_parking_spot_ptr->polygon().points().at(3);  // 左上角

  // 计算停车位的几何中心点
  // 通过四个角点的坐标平均值得到中心点
  Vec2d center_point = (left_bottom_point + right_bottom_point +
                        right_top_point + left_top_point) /
                       4.0;

  // 将停车位中心点投影到路径上，获取s和l坐标
  double center_point_s = 0.0;  // 沿路径距离
  double center_point_l = 0.0;   // 横向距离
  nearby_path.GetNearestPoint(center_point, &center_point_s, &center_point_l);

  // 计算车辆当前位置的投影坐标
  double vehicle_point_s = 0.0;
  double vehicle_point_l = 0.0;
  Vec2d vehicle_vec(vehicle_state.x(), vehicle_state.y());  // 创建车辆位置向量
  nearby_path.GetNearestPoint(vehicle_vec, &vehicle_point_s, &vehicle_point_l);

  // 比较车辆与停车位中心在s方向的距离差
  if (std::abs(center_point_s - vehicle_point_s) < parking_start_range) {
    return true;  // 距离小于阈值，可以启动泊车
  }
  
  return false;  // 距离大于阈值
}

}  // namespace planning
}  // namespace apollo