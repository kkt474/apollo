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
 * @file config_gflags.cc
 * @brief Apollo通用配置参数定义文件
 *
 * 功能说明：
 * 定义了Apollo各模块使用的通用配置参数
 * 这些参数通过GFlags配置系统管理，支持命令行覆盖和配置文件修改
 *
 * GFlags说明：
 * GFlags（Google Flags）是Google开发的命令行参数解析库
 * - DEFINE_bool：定义布尔类型参数
 * - DEFINE_int32：定义32位整数类型参数
 * - DEFINE_double：定义双精度浮点数类型参数
 * - DEFINE_string：定义字符串类型参数
 * - 参数值可通过命令行 --flag=value 覆盖
 * - 也可通过FLAGS_xxx在代码中访问
 *
 * 配置分类：
 * 1. 地图配置 - 地图文件路径和目录
 * 2. 车辆配置 - 车辆参数文件路径
 * 3. 时间配置 - 时间源选择
 * 4. TF配置 - 坐标变换帧ID
 * 5. 导航模式配置 - 相对位置导航
 * 6. 车辆参数 - 尺寸和前瞻时间
 * 7. 仿真配置 - 仿真相关开关
 * 8. 定位配置 - 定位相关参数
 *
 * C++语法说明：
 * - DEFINE_xxx：GFlags宏，用于定义配置参数
 * - FLAGS_xxx：在代码中访问参数的变量名
 **/

#include "modules/common/configs/config_gflags.h"
/**
 * @brief 配置参数头文件
 *
 * 包含GFlags的声明，通常由build系统自动生成
 * 通过#include引入，使定义的FLAGS_xxx变量在编译时可见
 */

namespace apollo {
/**
 * @brief Apollo项目主命名空间
 */

namespace config {
/**
 * @brief 配置模块命名空间
 */

/**
 * @brief 地图配置定义
 *
 * Apollo使用高清地图（HD Map）进行定位和路径规划
 * 地图包含道路结构、车道线、交通标志等信息
 */

/**
 * @brief 地图目录路径
 *
 * DEFINE_string语法：
 * DEFINE_string(变量名, 默认值, 描述)
 *
 * - map_dir：地图数据所在目录
 * - 默认值：/apollo/modules/map/data/sunnyvale_loop
 * - 内容：该目录下包含一组相关的地图文件
 *
 * 地图目录通常包含：
 * - base_map.bin/xml/txt：基础地图
 * - sim_map.bin/txt：仿真地图
 * - routing_map.bin/txt：路由地图
 */
DEFINE_STRING(map_dir, "/apollo/modules/map/data/sunnyvale_loop",
              "Directory which contains a group of related maps.");

/**
 * @brief UTM区域ID
 *
 * DEFINE_int32语法：
 * DEFINE_int32(变量名, 默认值, 描述)
 *
 * - local_utm_zone_id：UTM（通用横轴墨卡托投影）区域编号
 * - 默认值：10
 * - UTM将地球划分为60个区域，每个区域有唯一的编号
 * - 用于GNSS定位的坐标转换
 */
DEFINE_INT32(local_utm_zone_id, 10, "UTM zone id.");

/**
 * @brief 测试基础地图文件名
 *
 * - test_base_map_filename：测试用基础地图文件
 * - 默认值：空字符串
 * - 如果非空，优先使用指定的测试地图文件
 * - 用于调试和特殊测试场景
 */
DEFINE_STRING(test_base_map_filename, "",
              "If not empty, use this test base map files.");

/**
 * @brief 基础地图文件名
 *
 * - base_map_filename：基础地图文件名
 * - 默认值：base_map.bin|base_map.xml|base_map.txt
 * - 搜索顺序：按|分隔的顺序依次查找
 * - 支持多种格式：二进制(.bin)、XML(.xml)、文本(.txt)
 *
 * 地图文件格式说明：
 * - .bin：Protobuf二进制格式，效率高
 * - .xml：可扩展标记语言，通用性好
 * - .txt：文本格式，便于调试
 */
DEFINE_STRING(base_map_filename, "base_map.bin|base_map.xml|base_map.txt",
              "Base map files in the map_dir, search in order.");

/**
 * @brief 仿真地图文件名
 *
 * - sim_map_filename：仿真使用的地图
 * - 默认值：sim_map.bin|sim_map.txt
 * - 与基础地图的区别：可能包含简化或特殊处理的数据
 * - 用于Dreamview仿真环境
 */
DEFINE_STRING(sim_map_filename, "sim_map.bin|sim_map.txt",
              "Simulation map files in the map_dir, search in order.");

/**
 * @brief 路由地图文件名
 *
 * - routing_map_filename：路由模块使用的地图
 * - 默认值：routing_map.bin|routing_map.txt
 * - 专门为路由计算优化的地图数据
 * - 可能只包含路由相关的信息（如车道连接关系）
 */
DEFINE_STRING(routing_map_filename, "routing_map.bin|routing_map.txt",
              "Routing map files in the map_dir, search in order.");

/**
 * @brief 终点路点文件名
 *
 * - end_way_point_filename：地图中定义的默认终点
 * - 默认值：default_end_way_point.txt
 * - 内容：用于RoutingRequest的默认终点坐标
 * - 当没有指定目的地时使用
 */
DEFINE_STRING(end_way_point_filename, "default_end_way_point.txt",
              "End way point of the map, will be sent in RoutingRequest.");

/**
 * @brief 默认循环路由文件名
 *
 * - default_routing_filename：默认循环路由
 * - 默认值：default_cycle_routing.txt
 * - 用途：用于循环行驶场景
 * - 内容：将发送给任务管理器模块
 */
DEFINE_STRING(default_routing_filename, "default_cycle_routing.txt",
              "Default cycle routing of the map, will be sent in Task to Task "
              "Manager Module.");

/**
 * @brief 当前起始点文件名
 *
 * - current_start_point_filename：当前车辆起始点
 * - 默认值：current_start_point.txt
 * - 用途：在Dreamview中设置起点后，会保存到该文件
 * - 路由编辑模式下会重置此文件
 */
DEFINE_STRING(current_start_point_filename, "current_start_point.txt",
              "The current starting point of the vehicle. Setting the starting "
              "point in route editing will be reset.");

/**
 * @brief 泊车出发路由文件名
 *
 * - park_go_routing_filename：泊车后出发路由
 * - 默认值：park_go_routing.txt
 * - 用途：支持Dreamview竞赛模式
 * - 包含从停车场出发的路由
 */
DEFINE_STRING(park_go_routing_filename, "park_go_routing.txt",
              "Park go routing of the map, support for dreamview contest.");

/**
 * @brief 速度控制区域文件名
 *
 * - speed_control_filename：速度控制区域定义
 * - 默认值：speed_control.pb.txt
 * - 内容：地图中定义的特殊速度限制区域
 * - 格式：Protobuf文本格式
 */
DEFINE_STRING(speed_control_filename, "speed_control.pb.txt",
              "The speed control region in a map.");

/**
 * @brief 车辆配置定义
 *
 * 车辆配置包含车辆物理参数，用于规划和控制算法
 */

/**
 * @brief 车辆配置文件路径
 *
 * - vehicle_config_path：车辆参数配置文件的路径
 * - 默认值：/apollo/modules/common/data/vehicle_param.pb.txt
 * - 内容：车辆尺寸、动力学参数等
 * - 文件格式：Protobuf文本格式
 */
DEFINE_STRING(vehicle_config_path,
              "/apollo/modules/common/data/vehicle_param.pb.txt",
              "the file path of vehicle config file");

/**
 * @brief 车辆模型配置文件路径
 *
 * - vehicle_model_config_filename：车辆动力学模型配置
 * - 默认值：/apollo/modules/common/vehicle_model/conf/vehicle_model_config.pb.txt
 * - 内容：车辆模型的详细参数
 * - 用于仿真和模型预测控制
 */
DEFINE_STRING(
    vehicle_model_config_filename,
    "/apollo/modules/common/vehicle_model/conf/vehicle_model_config.pb.txt",
    "the file path of vehicle model config file");

/**
 * @brief 时间配置定义
 *
 * 控制时间源的选择
 */

/**
 * @brief 是否使用Cyber时间
 *
 * DEFINE_bool语法：
 * DEFINE_bool(变量名, 默认值, 描述)
 *
 * - use_cyber_time：时间源选择
 * - 默认值：false
 * - false：使用系统时钟 system_clock::now()
 * - true：使用Cyber RT时钟 Clock::Now()
 *
 * 使用Cyber时间的场景：
 * - 与其他Cyber模块同步
 * - 需要精确的时间戳对齐
 */
DEFINE_BOOL(use_cyber_time, false,
            "Whether Clock::Now() gets time from system_clock::now() or from "
            "Cyber.");

/**
 * @brief TF坐标变换配置定义
 *
 * TF（Transform）系统管理不同坐标系之间的变换关系
 */

/**
 * @brief TF父帧ID
 *
 * - localization_tf2_frame_id：TF变换的父坐标系ID
 * - 默认值：world
 * - 通常是全局世界坐标系
 * - 所有定位结果都相对于此坐标系
 */
DEFINE_STRING(localization_tf2_frame_id, "world", "the tf2 transform frame id");

/**
 * @brief TF子帧ID
 *
 * - localization_tf2_child_frame_id：TF变换的子坐标系ID
 * - 默认值：localization
 * - 表示车辆定位所在的坐标系
 * - 相对于world坐标系
 */
DEFINE_STRING(localization_tf2_child_frame_id, "localization",
              "the tf2 transform child frame id");

/**
 * @brief 导航模式配置定义
 *
 * 导航模式使用相对位置进行规划
 */

/**
 * @brief 是否使用导航模式
 *
 * - use_navigation_mode：导航模式开关
 * - 默认值：false
 * - true：使用相对位置进行导航
 * - 用于没有绝对定位的环境
 */
DEFINE_BOOL(use_navigation_mode, false,
            "Use relative position in navigation mode");

/**
 * @brief 导航模式终点文件
 *
 * - navigation_mode_end_way_point_file：导航模式终点文件
 * - 默认值：modules/dreamview/conf/navigation_mode_default_end_way_point.txt
 * - 当导航模式启用时使用此文件
 */
DEFINE_STRING(
    navigation_mode_end_way_point_file,
    "modules/dreamview/conf/navigation_mode_default_end_way_point.txt",
    "end_way_point file used if navigation mode is set.");

/**
 * @brief 车辆参数定义
 *
 * 车辆物理参数
 */

/**
 * @brief 车辆半宽
 *
 * DEFINE_double语法：
 * DEFINE_double(变量名, 默认值, 描述)
 *
 * - half_vehicle_width：车辆半宽（米）
 * - 默认值：1.05
 * - 即车辆宽度的一半
 * - 用于碰撞检测和路径规划
 */
DEFINE_DOUBLE(half_vehicle_width, 1.05, "half vehicle width");

/**
 * @brief 前向看时间
 *
 * - look_forward_time_sec：前瞻时间（秒）
 * - 默认值：8.0
 * - 计算方式：look_forward_distance = look_forward_time_sec × adc_speed
 * - 用途：在从路由创建参考线时使用
 * - 影响：前瞻距离越远，路径规划越提前
 */
DEFINE_DOUBLE(look_forward_time_sec, 8.0,
              "look forward time times adc speed to calculate this distance "
              "when creating reference line from routing");

/**
 * @brief 仿真配置定义
 *
 * 仿真相关参数
 */

/**
 * @brief 是否使用仿真时间
 *
 * - use_sim_time：仿真时间开关
 * - 默认值：false
 * - true：使用bag包中的时间（消息时间戳）
 * - false：使用真实系统时间
 * - 用于回放数据时的模拟时间模式
 */
DEFINE_BOOL(use_sim_time, false, "Use bag time in mock time mode.");

/**
 * @brief 倒车状态标志
 *
 * - reverse_heading_vehicle_state：倒车测试标志
 * - 默认值：false
 * - 用于测试倒车场景
 */
DEFINE_BOOL(reverse_heading_vehicle_state, false,
            "test flag for reverse driving.");

/**
 * @brief 倒车时坐标变换开关
 *
 * - state_transform_to_com_reverse：倒车时坐标变换
 * - 默认值：false
 * - true：启用坐标变换（从后轴中心到质心）
 * - 倒车时的车辆状态坐标系转换
 */
DEFINE_BOOL(state_transform_to_com_reverse, false,
            "Enable vehicle states coordinate transformation from center of "
            "rear-axis to center of mass, during reverse driving");

/**
 * @brief 前进时坐标变换开关
 *
 * - state_transform_to_com_drive：前进时坐标变换
 * - 默认值：true
 * - true：启用坐标变换（从后轴中心到质心）
 * - 前进时的车辆状态坐标系转换
 *
 * 说明：Apollo通常使用后轴中心作为参考点
 * 但某些算法需要质心位置
 */
DEFINE_BOOL(state_transform_to_com_drive, true,
            "Enable vehicle states coordinate transformation from center of "
            "rear-axis to center of mass, during forward driving");

/**
 * @brief 多线程运行标志
 *
 * - multithread_run：多线程模式
 * - 默认值：false
 * - 主要用于仿真环境
 * - true：启用多线程规划
 */
DEFINE_BOOL(multithread_run, false,
            "multi-thread run flag mainly used by simulation");

/**
 * @brief 定位配置定义
 *
 * 定位相关参数
 */

/**
 * @brief 是否启用地图参考统一
 *
 * - enable_map_reference_unify：地图参考统一开关
 * - 默认值：true
 * - true：启用IMU数据到地图参考系的转换
 * - 用于多传感器融合定位
 */
DEFINE_BOOL(enable_map_reference_unify, true,
            "enable IMU data convert to map reference");

}  // namespace config
/**
 * @brief 配置命名空间结束标记
 */

}  // namespace apollo
/**
 * @brief Apollo命名空间结束标记
 */
