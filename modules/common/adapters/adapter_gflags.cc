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
 * @file adapter_gflags.cc
 * @brief Apollo适配器配置参数定义文件
 *
 * 功能说明：
 * 定义了Apollo各模块之间通信使用的Cyber RT话题名称
 * 这些参数通过GFlags配置系统管理，支持命令行覆盖和配置文件修改
 *
 * GFlags说明：
 * GFlags（Google Flags）是Google开发的命令行参数解析库
 * - DEFINE_bool：定义布尔类型参数
 * - DEFINE_string：定义字符串类型参数
 * - 参数值可通过命令行 --flag=value 覆盖
 * - 也可通过FLAGS_xxx在代码中访问
 *
 * 话题通信架构：
 * Apollo使用Cyber RT的发布-订阅模式进行模块间通信
 * 每个模块通过特定话题名称发送和接收数据
 * 话题名称采用层次化命名：/apollo/模块/子模块/数据类型
 *
 * C++语法说明：
 * - DEFINE_bool/DEFINE_string：GFlags宏，用于定义配置参数
 * - gflags命名空间：Google Flags库
 * - 字符串字面量：R"()"原始字符串（未使用）
 *
 * 文件结构：
 * 1. GPS/IMU相关话题
 * 2. 车辆底盘话题
 * 3. 定位话题
 * 4. 规划话题
 * 5. 控制话题
 * 6. 激光雷达话题
 * 7. 摄像头话题
 * 8. 毫米波雷达话题
 * 9.  Perception感知话题
 * 10. V2X车联网话题
 * 11. 其他话题
 **/

#include "modules/common/adapters/adapter_gflags.h"
/**
 * @brief 适配器配置参数头文件
 *
 * 包含GFlags的声明，通常由build系统自动生成
 */

#include <string>
/**
 * @brief 字符串头文件
 *
 * 提供std::string类型定义
 * 虽然DEFINE_string不需要，但其他代码可能用到
 */

namespace apollo {
/**
 * @brief Apollo项目主命名空间
 */

namespace adapter {
/**
 * @brief 适配器模块命名空间
 *
 * 包含消息适配器相关定义
 */

using apollo::FLAGS_enable_adapter_dump;
/**
 * @brief 使用enable_adapter_dump标志
 *
 * 通过using声明引入FLAGS_xxx变量到当前作用域
 */

/**
 * @brief 适配器调试开关
 *
 * DEFINE_bool语法：
 * DEFINE_bool(变量名, 默认值, 描述字符串)
 *
 * - enable_adapter_dump：是否启用适配器消息转储
 * - false：默认关闭
 * - 描述：是否将消息转储到 /tmp/adapters/<topic_name>/<seq_num>.txt 用于调试
 *
 * 用途：
 * 调试时开启可以将发布的消息保存到文件
 * 便于分析消息内容和时序问题
 */
DEFINE_bool(enable_adapter_dump, false,
            "Whether enable dumping the messages to "
            "/tmp/adapters/<topic_name>/<seq_num>.txt for debugging purposes.");

/**
 * @brief GPS/IMU传感器话题定义
 *
 * DEFINE_string语法：
 * DEFINE_string(变量名, 默认值, 描述字符串)
 *
 * GPS相关话题：
 * - gnss/odometry：GNSS里程计，提供车辆位置和速度
 * - gnss/corrected_imu：修正后的IMU数据
 * - gnss/imu：原始IMU数据
 */

/**
 * @brief GPS里程计话题
 *
 * 话题：/apollo/sensor/gnss/odometry
 * 数据类型：Odometry
 * 内容：GNSS计算的车辆位置、速度和航向信息
 */
DEFINE_STRING(gps_topic, "/apollo/sensor/gnss/odometry", "GPS topic name");

/**
 * @brief 修正后的IMU话题
 *
 * 话题：/apollo/sensor/gnss/corrected_imu
 * 数据类型：CorrectedImu
 * 内容：经过误差修正的惯性测量单元数据
 * 包括：加速度、角速度等
 */
DEFINE_STRING(imu_topic, "/apollo/sensor/gnss/corrected_imu", "IMU topic name");

/**
 * @brief 原始IMU话题
 *
 * 话题：/apollo/sensor/gnss/imu
 * 数据类型：Imu
 * 内容：未经处理的原始IMU数据
 */
DEFINE_STRING(raw_imu_topic, "/apollo/sensor/gnss/imu", "Raw IMU topic name");

/**
 * @brief 音频检测话题
 *
 * 话题：/apollo/audio_detection
 * 数据类型：AudioDetection
 * 内容：音频检测结果，如语音命令等
 */
DEFINE_STRING(audio_detection_topic, "/apollo/audio_detection",
              "audio detection topic name");

/**
 * @brief 车辆底盘话题定义
 *
 * 底盘话题用于获取车辆状态信息和发送控制命令
 * 底盘是车辆控制的核心数据通道
 */

/**
 * @brief 底盘状态话题
 *
 * 话题：/apollo/canbus/chassis
 * 数据类型：Chassis
 * 内容：车辆底盘状态信息
 * 包括：速度、加速度、方向盘角度、档位、刹车油门等
 */
DEFINE_STRING(chassis_topic, "/apollo/canbus/chassis", "chassis topic name");

/**
 * @brief 底盘详细信息话题
 *
 * 话题：/apollo/canbus/chassis_detail
 * 数据类型：ChassisDetail
 * 内容：更详细的底盘信息
 */
DEFINE_STRING(chassis_detail_topic, "/apollo/canbus/chassis_detail",
              "chassis detail topic name");

/**
 * @brief 底盘详情发送者话题
 *
 * 话题：/apollo/canbus/chassis_detail_sender
 * 数据类型：ChassisDetail
 * 内容：底盘详情发送器专用话题
 */
DEFINE_STRING(chassis_detail_sender_topic,
              "/apollo/canbus/chassis_detail_sender",
              "chassis detail sender_topic name");

/**
 * @brief 底盘控制命令话题
 *
 * 话题：/apollo/chassis_control
 * 数据类型：ChassisCommand
 * 内容：向底盘发送的控制命令
 * 包括：目标速度、加速度、方向盘角度等
 */
DEFINE_STRING(chassis_command_topic, "/apollo/chassis_control",
              "chassis command topic name");

/**
 * @brief 定位模块话题定义
 *
 * 定位模块融合多种传感器数据提供精确车辆位置
 */

/**
 * @brief 定位结果话题
 *
 * 话题：/apollo/localization/pose
 * 数据类型：LocalizationEstimate
 * 内容：融合定位结果
 * 包括：位置(x,y,z)、姿态(roll,pitch,yaw)、速度等
 */
DEFINE_STRING(localization_topic, "/apollo/localization/pose",
              "localization topic name");

/**
 * @brief 规划学习数据话题
 *
 * 话题：/apollo/planning/learning_data
 * 数据类型：PlanningLearningData
 * 内容：用于模仿学习或强化学习的规划数据
 */
DEFINE_STRING(planning_learning_data_topic, "/apollo/planning/learning_data",
              "planning learning data");

/**
 * @brief 规划模块话题定义
 *
 * 规划模块负责生成车辆行驶轨迹
 */

/**
 * @brief 规划轨迹话题
 *
 * 话题：/apollo/planning
 * 数据类型：ADCTrajectory
 * 内容：规划生成的轨迹
 * 包括：路径点序列、时间戳、速度曲线等
 */
DEFINE_STRING(planning_trajectory_topic, "/apollo/planning",
              "planning trajectory topic name");

/**
 * @brief 规划命令话题
 *
 * 话题：/apollo/planning/command
 * 数据类型：PlanningCommand
 * 内容：发送给规划模块的外部命令
 * 用于：重新路由、目的地变更等
 */
DEFINE_STRING(planning_command, "/apollo/planning/command",
              "Topic name of input command to planning module.");

/**
 * @brief 规划命令状态话题
 *
 * 话题：/apollo/planning/command_status
 * 数据类型：PlanningCommandStatus
 * 内容：规划模块对命令的执行状态反馈
 */
DEFINE_STRING(planning_command_status, "/apollo/planning/command_status",
              "Topic name of planning command status.");

/**
 * @brief 监控模块话题定义
 */

/**
 * @brief 系统监控话题
 *
 * 话题：/apollo/monitor
 * 数据类型：MonitorMessage
 * 内容：系统运行状态监控信息
 */
DEFINE_STRING(monitor_topic, "/apollo/monitor", "Monitor");

/**
 * @brief 控制模块话题定义
 *
 * 控制模块根据规划轨迹生成车辆控制命令
 */

/**
 * @brief 控制pad话题
 *
 * 话题：/apollo/control/pad
 * 数据类型：PadMessage
 * 内容：人工干预指令
 * 用于：启动、停止、紧急刹车等
 */
DEFINE_STRING(pad_topic, "/apollo/control/pad",
              "control pad message topic name");

/**
 * @brief 控制命令话题
 *
 * 话题：/apollo/control
 * 数据类型：ControlCommand
 * 内容：发送给底盘的控制命令
 * 包括：油门、刹车、方向盘等
 */
DEFINE_STRING(control_command_topic, "/apollo/control",
              "control command topic name");

/**
 * @brief 控制调试信息话题
 *
 * 话题：/apollo/control/debug
 * 数据类型：ControlDebug
 * 内容：控制模块内部调试数据
 * 用于：问题诊断和算法优化
 */
DEFINE_STRING(control_debug_info_topic, "/apollo/control/debug",
              "control debug info topic name");

/**
 * @brief 控制交互信息话题
 *
 * 话题：/apollo/control/interactive
 * 数据类型：InteractiveInfo
 * 内容：与其他控制模块交互的信息
 */
DEFINE_STRING(control_interative_topic, "/apollo/control/interactive",
              "control interactive info to others topic name");

/**
 * @brief 控制预处理话题
 *
 * 话题：/apollo/control/preprocessor
 * 数据类型：ControlPreprocessor
 * 内容：控制命令预处理结果
 */
DEFINE_STRING(control_preprocessor_topic, "/apollo/control/preprocessor",
              "control preprocessor topic name");

/**
 * @brief 控制本地视图话题
 *
 * 话题：/apollo/control/localview
 * 数据类型：LocalView
 * 内容：控制模块需要的完整感知视野
 * 包含：定位、轨迹、障碍物等所有输入
 */
DEFINE_STRING(control_local_view_topic, "/apollo/control/localview",
              "control local view topic name");

/**
 * @brief 控制核心算法话题
 *
 * 话题：/apollo/control/controlcore
 * 数据类型：ControlCommand
 * 内容：控制核心算法的输入/输出
 */
DEFINE_STRING(control_core_command_topic, "/apollo/control/controlcore",
              "control command core algorithm topic name");

/**
 * @brief 激光雷达传感器话题定义
 *
 * 激光雷达提供高精度的环境三维点云数据
 * Apollo支持多种型号的激光雷达
 */

/**
 * @brief 128线激光雷达补偿后点云话题
 *
 * 话题：/apollo/sensor/lidar128/compensator/PointCloud2
 * 数据类型：PointCloud2
 * 内容：128线激光雷达经过运动补偿的点云
 * 补偿：消除车辆运动造成的点云畸变
 */
DEFINE_STRING(pointcloud_topic,
              "/apollo/sensor/lidar128/compensator/PointCloud2",
              "pointcloud topic name");

/**
 * @brief 16线激光雷达补偿后点云话题
 *
 * 话题：/apollo/sensor/lidar16/compensator/PointCloud2
 * 数据类型：PointCloud2
 * 内容：16线激光雷达经过运动补偿的点云
 */
DEFINE_STRING(pointcloud_16_topic,
              "/apollo/sensor/lidar16/compensator/PointCloud2",
              "16 beam Lidar pointcloud topic name");

/**
 * @brief 16线激光雷达原始点云话题
 *
 * 话题：/apollo/sensor/lidar16/PointCloud2
 * 数据类型：PointCloud2
 * 内容：16线激光雷达未处理的原始点云
 */
DEFINE_STRING(pointcloud_16_raw_topic, "/apollo/sensor/lidar16/PointCloud2",
              "16 beam Lidar raw pointcloud topic name");

/**
 * @brief 16线左激光雷达原始点云话题
 *
 * 话题：/apollo/sensor/lidar16/left/PointCloud2
 * 数据类型：PointCloud2
 * 内容：左侧16线激光雷达原始点云
 */
DEFINE_STRING(pointcloud_16_front_left_raw_topic,
              "/apollo/sensor/lidar16/left/PointCloud2",
              "16 left beam Lidar raw pointcloud topic name");

/**
 * @brief 16线右激光雷达原始点云话题
 *
 * 话题：/apollo/sensor/lidar16/right/PointCloud2
 * 数据类型：PointCloud2
 * 内容：右侧16线激光雷达原始点云
 */
DEFINE_STRING(pointcloud_16_front_right_raw_topic,
              "/apollo/sensor/lidar16/right/PointCloud2",
              "16 left beam Lidar raw pointcloud topic name");

/**
 * @brief 16线上方激光雷达补偿后点云话题
 *
 * 话题：/apollo/sensor/lidar16/front/up/compensator/PointCloud2
 * 数据类型：PointCloud2
 * 内容：安装在车辆上方前部的16线激光雷达点云
 */
DEFINE_STRING(pointcloud_16_front_up_topic,
              "/apollo/sensor/lidar16/front/up/compensator/PointCloud2",
              "Front up 16 beam Lidar pointcloud topic name");

/**
 * @brief 64线激光雷达补偿后点云话题
 *
 * 话题：/apollo/sensor/velodyne64/compensator/PointCloud2
 * 数据类型：PointCloud2
 * 内容：64线激光雷达经过运动补偿的点云
 */
DEFINE_STRING(pointcloud_64_topic,
              "/apollo/sensor/velodyne64/compensator/PointCloud2",
              "pointcloud topic name");

/**
 * @brief 128线激光雷达点云话题（与pointcloud_topic相同）
 *
 * 话题：/apollo/sensor/lidar128/compensator/PointCloud2
 * 数据类型：PointCloud2
 * 内容：128线激光雷达点云
 */
DEFINE_STRING(pointcloud_128_topic,
              "/apollo/sensor/lidar128/compensator/PointCloud2",
              "pointcloud topic name for 128 beam lidar");

/**
 * @brief 禾赛40线激光雷达点云话题
 *
 * 话题：/apollo/sensor/hesai40/compensator/PointCloud2
 * 数据类型：PointCloud2
 * 内容：禾赛40线激光雷达补偿后点云
 */
DEFINE_STRING(pointcloud_hesai_40p_topic,
              "/apollo/sensor/hesai40/compensator/PointCloud2",
              "pointcloud topic name for hesai40p lidar");

/**
 * @brief 64线激光雷达原始点云话题
 *
 * 话题：/apollo/sensor/velodyne64/PointCloud2
 * 数据类型：PointCloud2
 * 内容：64线激光雷达未处理的原始点云
 */
DEFINE_STRING(pointcloud_raw_topic, "/apollo/sensor/velodyne64/PointCloud2",
              "pointcloud raw topic name");

/**
 * @brief Velodyne原始扫描数据话题
 *
 * 话题：/apollo/sensor/velodyne64/VelodyneScanUnified
 * 数据类型：VelodyneScanUnified
 * 内容：Velodyne雷达的原始扫描数据包
 */
DEFINE_STRING(velodyne_raw_topic,
              "/apollo/sensor/velodyne64/VelodyneScanUnified",
              "velodyne64 raw data topic name");

/**
 * @brief 64线激光雷达融合点云话题
 *
 * 话题：/apollo/sensor/velodyne64/fusion/PointCloud2
 * 数据类型：PointCloud2
 * 内容：多雷达融合后的点云
 */
DEFINE_STRING(pointcloud_fusion_topic,
              "/apollo/sensor/velodyne64/fusion/PointCloud2",
              "pointcloud fusion topic name");

/**
 * @brief VLP16激光雷达补偿后点云话题
 *
 * 话题：/apollo/sensor/velodyne16/compensator/PointCloud2
 * 数据类型：PointCloud2
 * 内容：Velodyne VLP-16雷达补偿后点云
 */
DEFINE_STRING(vlp16_pointcloud_topic,
              "/apollo/sensor/velodyne16/compensator/PointCloud2",
              "16 beam Lidar pointcloud topic name");

/**
 * @brief 16线激光雷达各安装位置话题
 *
 * 定义16线激光雷达在不同安装位置的话题
 */

/**
 * @brief 前部中央16线激光雷达话题
 *
 * 话题：/apollo/sensor/lidar16/front/center/PointCloud2
 */
DEFINE_STRING(lidar_16_front_center_topic,
              "/apollo/sensor/lidar16/front/center/PointCloud2",
              "front center 16 beam lidar topic name");

/**
 * @brief 前部上方16线激光雷达话题
 *
 * 话题：/apollo/sensor/lidar16/front/up/PointCloud2
 */
DEFINE_STRING(lidar_16_front_up_topic,
              "/apollo/sensor/lidar16/front/up/PointCloud2",
              "front up 16 beam lidar topic name");

/**
 * @brief 后部左侧16线激光雷达话题
 *
 * 话题：/apollo/sensor/lidar16/rear/left/PointCloud2
 */
DEFINE_STRING(lidar_16_rear_left_topic,
              "/apollo/sensor/lidar16/rear/left/PointCloud2",
              "rear left 16 beam lidar topic name");

/**
 * @brief 后部右侧16线激光雷达话题
 *
 * 话题：/apollo/sensor/lidar16/rear/right/PointCloud2
 */
DEFINE_STRING(lidar_16_rear_right_topic,
              "/apollo/sensor/lidar16/rear/right/PointCloud2",
              "rear right 16 beam lidar topic name");

/**
 * @brief 16线激光雷达融合话题
 *
 * 话题：/apollo/sensor/lidar16/fusion/PointCloud2
 */
DEFINE_STRING(lidar_16_fusion_topic,
              "/apollo/sensor/lidar16/fusion/PointCloud2",
              "16 beam lidar fusion topic name");

/**
 * @brief 16线激光雷达融合补偿后话题
 *
 * 话题：/apollo/sensor/lidar16/fusion/compensator/PointCloud2
 */
DEFINE_STRING(lidar_16_fusion_compensator_topic,
              "/apollo/sensor/lidar16/fusion/compensator/PointCloud2",
              "16 beam lidar fusion compensator topic name");

/**
 * @brief 128线激光雷达原始点云话题
 *
 * 话题：/apollo/sensor/lidar128/PointCloud2
 */
DEFINE_STRING(lidar_128_topic, "/apollo/sensor/lidar128/PointCloud2",
              "128 beam lidar topic name");

/**
 * @brief 预测模块话题定义
 *
 * 预测模块对感知到的障碍物进行轨迹预测
 */

/**
 * @brief 障碍物预测话题
 *
 * 话题：/apollo/prediction
 * 数据类型：PredictionObstacles
 * 内容：障碍物轨迹预测结果
 * 包括：每个障碍物的预测轨迹和意图
 */
DEFINE_STRING(prediction_topic, "/apollo/prediction", "prediction topic name");

/**
 * @brief 预测容器话题
 *
 * 话题：/apollo/prediction_container
 * 数据类型：PredictionContainer
 * 内容：预测模块的容器数据
 */
DEFINE_STRING(prediction_container_topic, "/apollo/prediction_container",
              "prediction container submodule topic name");

/**
 * @brief 感知模块话题定义
 *
 * 感知模块融合多种传感器数据检测环境障碍物
 */

/**
 * @brief 感知障碍物话题
 *
 * 话题：/apollo/perception/obstacles
 * 数据类型：PerceptionObstacles
 * 内容：感知融合后的障碍物列表
 * 包括：车辆、行人、自行车等
 */
DEFINE_STRING(perception_obstacle_topic, "/apollo/perception/obstacles",
              "perception obstacle topic name");

/**
 * @brief 驾驶事件话题
 *
 * 话题：/apollo/drive_event
 * 数据类型：DriveEvent
 * 内容：驾驶过程中的重要事件
 * 如：紧急刹车、碰撞预警等
 */
DEFINE_STRING(drive_event_topic, "/apollo/drive_event",
              "drive event topic name");

/**
 * @brief 交通灯检测话题
 *
 * 话题：/apollo/perception/traffic_light
 * 数据类型：TrafficLightDetection
 * 内容：交通灯检测结果
 * 包括：灯的颜色、形状、位置等
 */
DEFINE_STRING(traffic_light_detection_topic, "/apollo/perception/traffic_light",
              "traffic light detection topic name");

/**
 * @brief 车道线分割话题
 *
 * 话题：/apollo/perception/lane_mask
 * 数据类型：LaneMaskSegmentation
 * 内容：车道线分割结果
 * 以掩码形式表示道路车道线
 */
DEFINE_STRING(perception_lane_mask_segmentation_topic,
              "/apollo/perception/lane_mask",
              "lane mask segmentation topic name");

/**
 * @brief 路由模块话题定义
 *
 * 路由模块计算从起点到终点的全局路径
 */

/**
 * @brief 路由响应话题
 *
 * 话题：/apollo/routing_response
 * 数据类型：RoutingResponse
 * 内容：路由计算结果
 * 包括：路径点序列、车道信息等
 */
DEFINE_STRING(routing_response_topic, "/apollo/routing_response",
              "routing response topic name");

/**
 * @brief 路由历史响应话题
 *
 * 话题：/apollo/routing_response_history
 * 数据类型：RoutingResponse
 * 内容：历史路由响应
 * 用于：查询之前的路由结果
 */
DEFINE_STRING(routing_response_history_topic,
              "/apollo/routing_response_history",
              "routing response history topic name");

/**
 * @brief 相对里程计话题
 *
 * 话题：/apollo/calibration/relative_odometry
 * 数据类型：RelativeOdometry
 * 内容：相对定位结果
 * 用于：标定和相对定位
 */
DEFINE_STRING(relative_odometry_topic, "/apollo/calibration/relative_odometry",
              "relative odometry topic name");

/**
 * @brief GNSS/INS状态话题定义
 */

/**
 * @brief INS状态话题
 *
 * 话题：/apollo/sensor/gnss/ins_stat
 * 数据类型：InsStat
 * 内容：惯性导航系统状态
 */
DEFINE_STRING(ins_stat_topic, "/apollo/sensor/gnss/ins_stat",
              "ins stat topic name");

/**
 * @brief INS状态详情话题
 *
 * 话题：/apollo/sensor/gnss/ins_status
 * 数据类型：InsStatus
 * 内容：INS状态详细信息
 */
DEFINE_STRING(ins_status_topic, "/apollo/sensor/gnss/ins_status",
              "ins status topic name");

/**
 * @brief GNSS状态话题
 *
 * 话题：/apollo/sensor/gnss/gnss_status
 * 数据类型：GnssStatus
 * 内容：全球导航卫星系统状态
 */
DEFINE_STRING(gnss_status_topic, "/apollo/sensor/gnss/gnss_status",
              "gnss status topic name");

/**
 * @brief 系统状态话题
 *
 * 话题：/apollo/monitor/system_status
 * 数据类型：SystemStatus
 * 内容：整个系统的运行状态
 */
DEFINE_STRING(system_status_topic, "/apollo/monitor/system_status",
              "System status topic name");

/**
 * @brief 静态信息话题
 *
 * 话题：/apollo/monitor/static_info
 * 数据类型：StaticInfo
 * 内容：静态配置信息
 */
DEFINE_STRING(static_info_topic, "/apollo/monitor/static_info",
              "Static info topic name");

/**
 * @brief Mobileye视觉传感器话题
 *
 * Mobileye是一款主流的视觉感知设备
 */

/**
 * @brief Mobileye障碍物话题
 *
 * 话题：/apollo/sensor/mobileye
 * 数据类型：Mobileye
 * 内容：Mobileye视觉感知结果
 */
DEFINE_STRING(mobileye_topic, "/apollo/sensor/mobileye", "mobileye topic name");

/**
 * @brief SmarterEye障碍物话题
 *
 * 话题：/apollo/sensor/smartereye/obstacles
 * 数据类型：SmarterEyeObstacles
 * 内容：SmarterEye视觉感知障碍物结果
 */
DEFINE_STRING(smartereye_obstacles_topic, "/apollo/sensor/smartereye/obstacles",
              "smartereye obstacles topic name");

/**
 * @brief SmarterEye车道线话题
 *
 * 话题：/apollo/sensor/smartereye/lanemark
 * 数据类型：SmarterEyeLanemark
 * 内容：SmarterEye车道线检测结果
 */
DEFINE_STRING(smartereye_lanemark_topic, "/apollo/sensor/smartereye/lanemark",
              "smartereye lanemark topic name");

/**
 * @brief SmarterEye图像话题
 *
 * 话题：/apollo/sensor/smartereye/image
 * 数据类型：Image
 * 内容：SmarterEye摄像头原始图像
 */
DEFINE_STRING(smartereye_image_topic, "/apollo/sensor/smartereye/image",
              "smartereye image topic name");

/**
 * @brief 毫米波雷达话题定义
 *
 * 毫米波雷达提供远距离障碍物检测
 */

/**
 * @brief Delphi ESR雷达话题
 *
 * 话题：/apollo/sensor/delphi_esr
 * 数据类型：DelphiESR
 * 内容：Delphi ESR毫米波雷达目标
 */
DEFINE_STRING(delphi_esr_topic, "/apollo/sensor/delphi_esr",
              "delphi esr radar topic name");

/**
 * @brief Continental雷达话题
 *
 * 话题：/apollo/sensor/conti_radar
 * 数据类型：ContiRadar
 * 内容：Continental毫米波雷达目标
 */
DEFINE_STRING(conti_radar_topic, "/apollo/sensor/conti_radar",
              "continental radar topic name");

/**
 * @brief Racobit雷达话题
 *
 * 话题：/apollo/sensor/racobit_radar
 * 数据类型：RacobitRadar
 * 内容：Racobit雷达目标
 */
DEFINE_STRING(racobit_radar_topic, "/apollo/sensor/racobit_radar",
              "racobit radar topic name");

/**
 * @brief 超声波雷达话题
 *
 * 话题：/apollo/sensor/ultrasonic_radar
 * 数据类型：UltrasonicRadar
 * 内容：超声波雷达检测结果
 * 用于：泊车辅助和近距离检测
 */
DEFINE_STRING(ultrasonic_radar_topic, "/apollo/sensor/ultrasonic_radar",
              "ultrasonic esr radar topic name");

/**
 * @brief 前向雷达话题
 *
 * 话题：/apollo/sensor/radar/front
 * 数据类型：RadarObstacles
 * 内容：前向雷达检测目标
 */
DEFINE_STRING(front_radar_topic, "/apollo/sensor/radar/front",
              "front radar topic name");

/**
 * @brief 后向雷达话题
 *
 * 话题：/apollo/sensor/radar/rear
 * 数据类型：RadarObstacles
 * 内容：后向雷达检测目标
 */
DEFINE_STRING(rear_radar_topic, "/apollo/sensor/radar/rear",
              "rear radar topic name");

/**
 * @brief 摄像头话题定义
 *
 * TODO注释：建议修改话题名称
 * 摄像头是自动驾驶感知的重要组成部分
 */

/**
 * @brief 压缩图像话题
 *
 * 话题：camera/image_raw（非标准Apollo话题）
 * 数据类型：CompressedImage
 * 内容：压缩后的图像数据
 */
DEFINE_STRING(compressed_image_topic, "camera/image_raw",
              "CompressedImage topic name");

/**
 * @brief 前向摄像头图像话题（用于障碍物检测）
 *
 * 话题：/apollo/sensor/camera/front_6mm/image
 * 数据类型：Image
 * 内容：前向6mm焦距摄像头图像
 * 用途：障碍物检测、车道线检测等
 */
DEFINE_STRING(image_front_topic, "/apollo/sensor/camera/front_6mm/image",
              "front camera image topic name for obstacles from camera");

/**
 * @brief 短焦摄像头压缩图像话题
 *
 * 话题：/apollo/sensor/camera/front_6mm/image/compressed
 * 数据类型：CompressedImage
 * 内容：短焦摄像头压缩图像
 */
DEFINE_STRING(image_short_topic,
              "/apollo/sensor/camera/front_6mm/image/compressed",
              "short camera image topic name");

/**
 * @brief 长焦摄像头图像话题
 *
 * 话题：/apollo/sensor/camera/traffic/image_long
 * 数据类型：Image
 * 内容：长焦摄像头图像
 * 用途：交通灯远距离检测
 */
DEFINE_STRING(image_long_topic, "/apollo/sensor/camera/traffic/image_long",
              "long camera image topic name");

/**
 * @brief USB摄像头图像话题
 *
 * 话题：/apollo/sensor/camera/image_usb_cam
 * 数据类型：Image
 * 内容：USB摄像头图像
 */
DEFINE_STRING(image_usb_cam_topic, "/apollo/sensor/camera/image_usb_cam",
              "USB camera image topic name");

/**
 * @brief 长焦摄像头图像话题（图像模块）
 *
 * 话题：/apollo/sensor/camera/image_long
 */
DEFINE_STRING(camera_image_long_topic, "/apollo/sensor/camera/image_long",
              "long camera image topic name");

/**
 * @brief 短焦摄像头图像话题（图像模块）
 *
 * 话题：/apollo/sensor/camera/image_short
 */
DEFINE_STRING(camera_image_short_topic, "/apollo/sensor/camera/image_short",
              "short camera image topic name");

/**
 * @brief 前向6mm摄像头话题
 *
 * 话题：/apollo/sensor/camera/front_6mm/image
 */
DEFINE_STRING(camera_front_6mm_topic, "/apollo/sensor/camera/front_6mm/image",
              "front 6mm camera topic name");

/**
 * @brief 前向6mm摄像头话题2
 *
 * 话题：/apollo/sensor/camera/front_6mm_2/image
 * 用于：双目前视摄像头
 */
DEFINE_STRING(camera_front_6mm_2_topic,
              "/apollo/sensor/camera/front_6mm_2/image",
              "front 6mm camera topic name 2");

/**
 * @brief 前向12mm摄像头话题
 *
 * 话题：/apollo/sensor/camera/front_12mm/image
 * 12mm焦距，提供更窄视角和更远检测距离
 */
DEFINE_STRING(camera_front_12mm_topic, "/apollo/sensor/camera/front_12mm/image",
              "front 12mm camera topic name");

/**
 * @brief 前向6mm摄像头压缩话题
 *
 * 话题：/apollo/sensor/camera/front_6mm/image/compressed
 */
DEFINE_STRING(camera_front_6mm_compressed_topic,
              "/apollo/sensor/camera/front_6mm/image/compressed",
              "front 6mm camera compressed topic name");

/**
 * @brief 前向12mm摄像头压缩话题
 *
 * 话题：/apollo/sensor/camera/front_12mm/image/compressed
 */
DEFINE_STRING(camera_front_12mm_compressed_topic,
              "/apollo/sensor/camera/front_12mm/image/compressed",
              "front 12mm camera compressed topic name");

/**
 * @brief 左侧鱼眼摄像头压缩话题
 *
 * 话题：/apollo/sensor/camera/left_fisheye/image/compressed
 * 鱼眼摄像头提供大视角，用于环视和泊车
 */
DEFINE_STRING(camera_left_fisheye_compressed_topic,
              "/apollo/sensor/camera/left_fisheye/image/compressed",
              "left fisheye camera compressed topic name");

/**
 * @brief 右侧鱼眼摄像头压缩话题
 *
 * 话题：/apollo/sensor/camera/right_fisheye/image/compressed
 */
DEFINE_STRING(camera_right_fisheye_compressed_topic,
              "/apollo/sensor/camera/right_fisheye/image/compressed",
              "right fisheye camera compressed topic name");

/**
 * @brief 后方摄像头压缩话题
 *
 * 话题：/apollo/sensor/camera/rear_6mm/image/compressed
 */
DEFINE_STRING(camera_rear_6mm_compressed_topic,
              "/apollo/sensor/camera/rear_6mm/image/compressed",
              "front 6mm camera compressed topic name");

/**
 * @brief 前向6mm摄像头视频压缩话题
 *
 * 话题：/apollo/sensor/camera/front_6mm/video/compressed
 */
DEFINE_STRING(camera_front_6mm_video_compressed_topic,
              "/apollo/sensor/camera/front_6mm/video/compressed",
              "front 6mm camera video compressed topic name");

/**
 * @brief 前向12mm摄像头视频压缩话题
 *
 * 话题：/apollo/sensor/camera/front_12mm/video/compressed
 */
DEFINE_STRING(camera_front_12mm_video_compressed_topic,
              "/apollo/sensor/camera/front_12mm/video/compressed",
              "front 12mm camera video compressed topic name");

/**
 * @brief 左侧鱼眼摄像头视频压缩话题
 *
 * 话题：/apollo/sensor/camera/left_fisheye/video/compressed
 */
DEFINE_STRING(camera_left_fisheye_video_compressed_topic,
              "/apollo/sensor/camera/left_fisheye/video/compressed",
              "left fisheye camera video compressed topic name");

/**
 * @brief 右侧鱼眼摄像头视频压缩话题
 *
 * 话题：/apollo/sensor/camera/right_fisheye/video/compressed
 */
DEFINE_STRING(camera_right_fisheye_video_compressed_topic,
              "/apollo/sensor/camera/right_fisheye/video/compressed",
              "right fisheye camera video compressed topic name");

/**
 * @brief 后方摄像头视频压缩话题
 *
 * 话题：/apollo/sensor/camera/rear_6mm/video/compressed
 */
DEFINE_STRING(camera_rear_6mm_video_compressed_topic,
              "/apollo/sensor/camera/rear_6mm/video/compressed",
              "front 6mm camera video compressed topic name");

/**
 * @brief GNSS RTK相关话题定义
 *
 * RTK（实时动态定位）提供厘米级定位精度
 */

/**
 * @brief GNSS RTK观测数据话题
 *
 * 话题：/apollo/sensor/gnss/rtk_obs
 * 数据类型：GnssRtkObs
 * 内容：RTK观测数据
 */
DEFINE_STRING(gnss_rtk_obs_topic, "/apollo/sensor/gnss/rtk_obs",
              "Gnss rtk observation topic name");

/**
 * @brief GNSS RTK星历数据话题
 *
 * 话题：/apollo/sensor/gnss/rtk_eph
 * 数据类型：GnssRtkEph
 * 内容：RTK星历数据
 */
DEFINE_STRING(gnss_rtk_eph_topic, "/apollo/sensor/gnss/rtk_eph",
              "Gnss rtk ephemeris topic name");

/**
 * @brief GNSS最佳定位结果话题
 *
 * 话题：/apollo/sensor/gnss/best_pose
 * 数据类型：BestPose
 * 内容：GNSS最佳定位结果
 */
DEFINE_STRING(gnss_best_pose_topic, "/apollo/sensor/gnss/best_pose",
              "Gnss rtk best gnss pose");

/**
 * @brief 多传感器融合定位话题定义
 *
 * MSF（Multi-Sensor Fusion）融合多种传感器数据
 */

/**
 * @brief GNSS融合定位话题
 *
 * 话题：/apollo/localization/msf_gnss
 * 数据类型：LocalizationEstimate
 * 内容：基于GNSS的融合定位结果
 */
DEFINE_STRING(localization_gnss_topic, "/apollo/localization/msf_gnss",
              "Gnss localization measurement topic name");

/**
 * @brief 激光雷达融合定位话题
 *
 * 话题：/apollo/localization/msf_lidar
 * 数据类型：LocalizationEstimate
 * 内容：基于激光雷达的融合定位结果
 */
DEFINE_STRING(localization_lidar_topic, "/apollo/localization/msf_lidar",
              "Lidar localization measurement topic name");

/**
 * @brief NDT融合定位话题
 *
 * 话题：/apollo/localization/ndt_lidar
 * 数据类型：LocalizationEstimate
 * 内容：基于NDT算法的激光雷达定位结果
 */
DEFINE_STRING(localization_ndt_topic, "/apollo/localization/ndt_lidar",
              "NDT localization lidar measurement topic name");

/**
 * @brief SINS PVA定位话题
 *
 * 话题：/apollo/localization/msf_sins_pva
 * 数据类型：LocalizationEstimate
 * 内容：SINS（ strapdown inertial navigation system）的PVA结果
 */
DEFINE_STRING(localization_sins_pva_topic, "/apollo/localization/msf_sins_pva",
              "Localization sins pva topic name");

/**
 * @brief MSF定位状态话题
 *
 * 话题：/apollo/localization/msf_status
 * 数据类型：MsfStatus
 * 内容：多传感器融合定位系统状态
 */
DEFINE_STRING(localization_msf_status, "/apollo/localization/msf_status",
              "msf localization status");

/**
 * @brief 相对地图和导航话题
 */

/**
 * @brief 相对地图话题
 *
 * 话题：/apollo/relative_map
 * 数据类型：RelativeMap
 * 内容：相对地图信息
 */
DEFINE_STRING(relative_map_topic, "/apollo/relative_map", "relative map");

/**
 * @brief 导航话题
 *
 * 话题：/apollo/navigation
 * 数据类型：NavigationInfo
 * 内容：导航相关信息
 */
DEFINE_STRING(navigation_topic, "/apollo/navigation", "navigation");

/**
 * @brief HMI（人机界面）话题定义
 */

/**
 * @brief HMI状态话题
 *
 * 话题：/apollo/hmi/status
 * 数据类型：HMIStatus
 * 内容：人机界面状态
 */
DEFINE_STRING(hmi_status_topic, "/apollo/hmi/status", "HMI status topic name.");

/**
 * @brief HMI音频采集话题
 *
 * 话题：/apollo/hmi/audio_capture
 * 数据类型：AudioCapture
 * 内容：音频采集数据
 */
DEFINE_STRING(audio_capture_topic, "/apollo/hmi/audio_capture",
              "HMI audio capture topic name.");

/**
 * @brief V2X（车联网）话题定义
 *
 * V2X实现车辆与基础设施、其他车辆的通信
 */

/**
 * @brief V2X OBU交通灯话题
 *
 * 话题：/apollo/v2x/obu/internal/traffic_light
 * 数据类型：V2XTrafficLight
 * 内容：来自OBU（车载单元）的交通灯信息
 */
DEFINE_STRING(v2x_obu_traffic_light_topic,
              "/apollo/v2x/obu/internal/traffic_light",
              "v2x obu traffic_light topic name");

/**
 * @brief V2X内部障碍物话题
 *
 * 话题：/apollo/v2x/obu/internal/obstacles
 * 数据类型：V2XObstacles
 * 内容：OBU内部障碍物信息
 */
DEFINE_STRING(v2x_internal_obstacle_topic, "/apollo/v2x/obu/internal/obstacles",
              "v2x internal obstacles topic name");

/**
 * @brief V2X障碍物话题
 *
 * 话题：/apollo/v2x/obstacles
 * 数据类型：V2XObstacles
 * 内容：V2X障碍物信息
 */
DEFINE_STRING(v2x_obstacle_topic, "/apollo/v2x/obstacles",
              "v2x obstacles topic name");

/**
 * @brief V2X交通灯话题
 *
 * 话题：/apollo/v2x/traffic_light
 * 数据类型：V2XTrafficLight
 * 内容：V2X交通灯信息
 */
DEFINE_STRING(v2x_traffic_light_topic, "/apollo/v2x/traffic_light",
              "v2x traffic light topic name");

/**
 * @brief V2X交通灯HMI话题
 *
 * 话题：/apollo/v2x/traffic_light/for_hmi
 * 数据类型：V2XTrafficLight
 * 内容：用于HMI显示的V2X交通灯信息
 */
DEFINE_STRING(v2x_traffic_light_for_hmi_topic,
              "/apollo/v2x/traffic_light/for_hmi",
              "v2x traffic light topic name for hmi");

/**
 * @brief V2X RSI话题
 *
 * 话题：/apollo/v2x/rsi
 * 数据类型：V2XRSI
 * 内容：路侧信息（Road Side Information）
 */
DEFINE_STRING(v2x_rsi_topic, "/apollo/v2x/rsi", "v2x rsi topic name");

/**
 * @brief Storytelling话题
 *
 * Storytelling用于记录驾驶场景故事
 */

/**
 * @brief Storytelling话题
 *
 * 话题：/apollo/storytelling
 * 数据类型：Storytelling
 * 内容：驾驶场景故事数据
 */
DEFINE_STRING(storytelling_topic, "/apollo/storytelling",
              "Storytelling topic.");

/**
 * @brief 音频事件话题
 *
 * 话题：/apollo/audio_event
 * 数据类型：AudioEvent
 * 内容：音频事件检测结果
 */
DEFINE_STRING(audio_event_topic, "/apollo/audio_event", "Audio event topic.");

/**
 * @brief Guardian话题
 *
 * Guardian是Apollo的安全监控模块
 */

/**
 * @brief Guardian话题
 *
 * 话题：/apollo/guardian
 * 数据类型：GuardianCommand
 * 内容：Guardian模块命令
 */
DEFINE_STRING(guardian_topic, "/apollo/guardian", "Guardian topic.");

/**
 * @brief GNSS原始数据话题
 *
 * 话题：/apollo/sensor/gnss/raw_data
 * 数据类型：GnssRawData
 * 内容：GNSS原始观测数据
 */
DEFINE_STRING(gnss_raw_data_topic, "/apollo/sensor/gnss/raw_data",
              "gnss raw data topic name");

/**
 * @brief GNSS流状态话题
 *
 * 话题：/apollo/sensor/gnss/stream_status
 * 数据类型：StreamStatus
 * 内容：GNSS数据流状态
 */
DEFINE_STRING(stream_status_topic, "/apollo/sensor/gnss/stream_status",
              "gnss stream status topic name");

/**
 * @brief 航向角话题
 *
 * 话题：/apollo/sensor/gnss/heading
 * 数据类型：Heading
 * 内容：GNSS航向角信息
 */
DEFINE_STRING(heading_topic, "/apollo/sensor/gnss/heading",
              "gnss heading topic name");

/**
 * @brief RTCM数据话题
 *
 * 话题：/apollo/sensor/gnss/rtcm_data
 * 数据类型：RtcmData
 * 内容：RTCM（差分修正数据）
 */
DEFINE_STRING(rtcm_data_topic, "/apollo/sensor/gnss/rtcm_data",
              "gnss rtcm data topic name");

/**
 * @brief 坐标变换话题
 *
 * TF（Transform）是Apollo的坐标变换系统
 */

/**
 * @brief TF动态变换话题
 *
 * 话题：/tf
 * 数据类型：TFMessage
 * 内容：坐标系之间的动态变换关系
 */
DEFINE_STRING(tf_topic, "/tf", "Transform topic.");

/**
 * @brief TF静态变换话题
 *
 * 话题：/tf_static
 * 数据类型：TFMessage
 * 内容：坐标系之间的静态变换关系
 * 静态变换在系统启动后不变化
 */
DEFINE_STRING(tf_static_topic, "/tf_static", "Transform static topic.");

/**
 * @brief 数据记录器状态话题
 *
 * 话题：/apollo/data/recorder/status
 * 数据类型：RecorderStatus
 * 内容：数据记录器运行状态
 */
DEFINE_STRING(recorder_status_topic, "/apollo/data/recorder/status",
              "Recorder status topic.");

/**
 * @brief 延迟记录话题
 *
 * 话题：/apollo/common/latency_records
 * 数据类型：LatencyRecord
 * 内容：模块处理延迟记录
 */
DEFINE_STRING(latency_recording_topic, "/apollo/common/latency_records",
              "Latency recording topic.");

/**
 * @brief 延迟报告话题
 *
 * 话题：/apollo/common/latency_reports
 * 数据类型：LatencyReport
 * 内容：延迟统计报告
 */
DEFINE_STRING(latency_reporting_topic, "/apollo/common/latency_reports",
              "Latency reporting topic.");

/**
 * @brief 任务管理器话题
 *
 * 话题：/apollo/task_manager
 * 数据类型：TaskCommand
 * 内容：任务管理器命令
 */
DEFINE_STRING(task_topic, "/apollo/task_manager", "task manager topic name");

/**
 * @brief 激光雷达型号配置
 *
 * 注释说明：可选值 velodyne128, velodyne64, velodyne16
 * 如果不设置，将根据传感器名称自动加载
 */

/**
 * @brief 激光雷达型号版本
 *
 * 有效值：
 * - velodyne128：128线激光雷达
 * - velodyne64：64线激光雷达
 * - velodyne16：16线激光雷达
 * - ""（空）：根据传感器名称自动检测
 *
 * 用途：
 * 指定加载哪个型号的激光雷达驱动
 */
DEFINE_STRING(lidar_model_version, "",
              "It determins which lidar model(16 ,64 or 128) to load, "
              "if not to set, the model will be loaded by the sensor name.");

/**
 * @brief 记录信息话题
 *
 * 话题：/apollo/cyber/record_info
 * 数据类型：RecordInfo
 * 内容：Cyber RT数据记录信息
 */
DEFINE_STRING(record_info_topic, "/apollo/cyber/record_info",
              "record info topic");

}  // namespace adapter
/**
 * @brief 适配器命名空间结束标记
 */

}  // namespace apollo
/**
 * @brief Apollo命名空间结束标记
 */
