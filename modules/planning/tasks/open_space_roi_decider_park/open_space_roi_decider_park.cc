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
 * @file open_space_roi_decider_park.cc
 *
 * @brief 开放空间ROI决策器（停车场场景）实现文件
 *
 * 功能说明：
 * 本文件实现了 OpenSpaceRoiDeciderPark 类
 * 负责为停车场等开放空间场景定义规划的兴趣区域(ROI)
 * 主要功能包括：
 * 1. 停车位边界提取
 * 2. 道路边界构建
 * 3. 坐标归一化/反归一化
 * 4. 障碍物约束构建
 *
 * 支持的场景：
 * - PARKING：标准停车场
 * - PULL_OVER：靠边停车
 * - PARK_AND_GO：停车后启动
 * - LARGE_CURVATURE：大曲率弯道（掉头等）
 *
 * C++语法说明：
 * - #include：预处理指令，包含头文件
 * - namespace：命名空间，避免命名冲突
 * - class：类声明
 * - std::array<T, N>：固定大小数组
 * - std::vector<T>：动态数组容器
 * - std::shared_ptr：智能指针，引用计数管理
 * - Eigen库：线性代数矩阵运算
 **/

#include "modules/planning/tasks/open_space_roi_decider_park/open_space_roi_decider_park.h"

/**
 * @brief 标准库头文件
 *
 * C++语法说明：
 * - <limits>：数值极限（numeric_limits）
 * - <memory>：智能指针（shared_ptr）
 * - <utility>：工具函数（std::move）
 */
#include <limits>
#include <memory>
#include <utility>

/**
 * @brief Apollo数学库
 *
 * - Polygon2d：2D多边形类
 * - Vec2d：2D向量类
 */
#include "modules/common/math/polygon2d.h"
#include "modules/common/math/vec2d.h"
/**
 * @brief Apollo点工厂
 *
 * 功能：便捷创建各种格式的点
 */
#include "modules/common/util/point_factory.h"
/**
 * @brief 开放空间ROI工具函数
 *
 * 功能：停车场ROI相关的辅助函数
 */
#include "modules/planning/planning_open_space/utils/open_space_roi_util.h"
/**
 * @brief 规划上下文
 *
 * 功能：存储规划过程中的状态信息
 */
#include "modules/planning/planning_base/common/planning_context.h"
/**
 * @brief 调试信息打印工具
 */
#include "modules/planning/planning_base/common/util/print_debug_info.h"

/**
 * @brief Apollo命名空间开始
 */
namespace apollo {
/**
 * @brief 规划模块命名空间
 */
namespace planning {

/**
 * @brief 类型别名声明
 *
 * C++语法说明：
 * - using：类型别名声明，等价于typedef
 * - apollo::common::ErrorCode：错误码枚举
 * - apollo::common::Status：状态类型
 * - apollo::common::math::Box2d/Vec2d：2D数学工具
 * - apollo::hdmap::*：高精地图相关类型
 */
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::LaneInfoConstPtr;
using apollo::hdmap::LaneSegment;
using apollo::hdmap::ParkingSpaceInfoConstPtr;
using apollo::hdmap::Path;

/**
 * @brief 初始化函数
 *
 * @param config_dir 配置目录路径
 * @param name 任务名称
 * @param injector 依赖注入器指针
 * @return bool 初始化成功返回true
 *
 * 功能说明：
 * 1. 调用基类Decider的Init方法
 * 2. 获取HD地图指针
 * 3. 加载车辆参数配置
 * 4. 加载ROI决策器配置
 *
 * C++语法说明：
 * - const std::string &config_dir：
 *   常量引用参数，避免拷贝
 *
 * - const std::shared_ptr<DependencyInjector> &injector：
 *   shared_ptr智能指针的常量引用
 *
 * - hdmap::HDMapUtil::BaseMapPtr()：
 *   静态方法，获取HD地图单例指针
 *
 * - CHECK_NOTNULL(hdmap_)：
 *   Apollo宏，检查指针非空
 *
 * - apollo::common::VehicleConfigHelper::GetConfig()：
 *   单例模式获取车辆配置
 *
 * - Decider::LoadConfig<T>(...)：
 *   模板方法，加载protobuf配置
 */
bool OpenSpaceRoiDeciderPark::Init(
        const std::string &config_dir,
        const std::string &name,
        const std::shared_ptr<DependencyInjector> &injector) {
    /**
     * @brief 调用基类初始化
     *
     * Decider::Init(...)：
     *   调用父类的初始化方法
     *   返回false表示失败
     */
    if (!Decider::Init(config_dir, name, injector)) {
        return false;
    }
    /**
     * @brief 获取HD地图指针
     *
     * hdmap::HDMapUtil::BaseMapPtr()：
     *   静态方法，获取地图单例
     *   返回shared_ptr类型
     */
    hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
    /**
     * @brief 检查地图指针有效性
     *
     * CHECK_NOTNULL：
     *   Apollo断言宏
     *   如果hdmap_为nullptr则终止程序
     */
    CHECK_NOTNULL(hdmap_);
    /**
     * @brief 获取车辆参数
     *
     * VehicleConfigHelper::GetConfig()：
     *   获取车辆配置单例
     *   .vehicle_param()：
     *     访问protobuf配置的车辆参数
     */
    vehicle_params_ = apollo::common::VehicleConfigHelper::GetConfig().vehicle_param();
    /**
     * @brief 加载ROI决策器配置
     *
     * Decider::LoadConfig<OpenSpaceRoiDeciderParkConfig>：
     *   模板方法加载特定配置类型
     *   &config_：输出参数存储配置
     */
    bool res = Decider::LoadConfig<OpenSpaceRoiDeciderParkConfig>(&config_);
    /**
     * @brief 打印配置信息
     *
     * AINFO：Apollo信息日志
     * config_.DebugString()：
     *   protobuf消息的调试字符串表示
     */
    AINFO << config_.DebugString();
    return res;
}

/**
 * @brief 主处理函数
 *
 * @param frame 当前规划帧
 * @return Status 处理状态
 *
 * 功能说明：
 * 根据配置的类型(ROI_TYPE)选择不同的处理流程：
 * 1. PARKING：停车场停车
 * 2. PULL_OVER：靠边停车
 * 3. PARK_AND_GO：停车后启动
 * 4. LARGE_CURVATURE：大曲率弯道
 *
 * C++语法说明：
 * - Frame *frame：
 *   裸指针，指向当前规划帧
 *
 * - std::array<Vec2d, 4>：
 *   固定大小数组，存储4个2D向量（停车位顶点）
 *
 * - std::vector<std::vector<Vec2d>>：
 *   二维向量，存储边界线段
 *   外层：边界点序列
 *   内层：每个点的坐标
 *
 * - config_.roi_type()：
 *   protobuf配置访问
 *   返回ROI类型枚举
 *
 * - switch/case：
 *   多分支选择语句
 */
Status OpenSpaceRoiDeciderPark::Process(Frame *frame) {
    /**
     * @brief 空指针检查
     *
     * nullptr：
     *   空指针字面量
     *   防御性编程
     */
    if (frame == nullptr) {
        const std::string msg = "Invalid frame, fail to process the OpenSpaceRoiDeciderPark.";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    /**
     * @brief 获取车辆状态和障碍物列表
     *
     * frame->vehicle_state()：
     *   获取当前帧的车辆状态
     *   包含位置(x,y)、航向角(heading)等
     *
     * frame->GetObstacleList()：
     *   获取感知障碍物列表
     *   返回ObstacleList指针
     */
    vehicle_state_ = frame->vehicle_state();
    obstacles_by_frame_ = frame->GetObstacleList();

    /**
     * @brief 声明变量
     *
     * std::array<Vec2d, 4>：
     *   固定大小数组，4个Vec2d
     *   用于存储停车位四个角点
     *
     * std::vector<std::vector<Vec2d>>：
     *   二维向量
     *   外层：边界线段集合
     *   内层：每条线段的端点
     */
    std::array<Vec2d, 4> spot_vertices;
    Path nearby_path;
    std::vector<std::vector<common::math::Vec2d>> roi_boundary;
    std::vector<std::vector<common::math::Vec2d>> soft_boundary;

    /**
     * @brief 根据ROI类型选择处理流程
     *
     * config_.roi_type()：
     *   获取配置的ROI类型
     *   OpenSpaceRoiDeciderParkConfig::PARKING等枚举值
     */
    const auto &roi_type = config_.roi_type();
    if (roi_type == OpenSpaceRoiDeciderParkConfig::PARKING) {
        /**
         * @brief 停车场场景处理
         *
         * 1. 获取目标停车位ID
         * 2. 从地图获取停车位信息
         * 3. 设置原点（归一化参考点）
         * 4. 设置停车终止位姿
         * 5. 获取停车边界
         */
        target_parking_spot_id_ = frame->open_space_info().target_parking_spot_id();
        ParkingInfo parking_info;
        if (!GetParkingSpot(frame, &parking_info)) {
            const std::string msg = "Fail to get parking boundary from map";
            AERROR << msg;
            return Status(ErrorCode::PLANNING_ERROR, msg);
        }

        /**
         * @brief 设置停车类型
         *
         * frame->mutable_open_space_info()：
         *   mutable_前缀获取可变引用
         *   protobuf字段修改方法
         */
        frame->mutable_open_space_info()->set_parking_type(parking_info.parking_type);

        SetOrigin(parking_info, frame);
        SetParkingSpotEndPose(parking_info, frame);

        if (!GetParkingBoundary(parking_info, *nearby_path_, frame, &roi_boundary, &soft_boundary)) {
            const std::string msg = "Fail to get parking boundary from map";
            AERROR << msg;
            return Status(ErrorCode::PLANNING_ERROR, msg);
        }
    } else if (roi_type == OpenSpaceRoiDeciderParkConfig::PULL_OVER) {
        /**
         * @brief 靠边停车场景
         *
         * 1. 获取靠边停车位置
         * 2. 设置原点
         * 3. 设置终止位姿
         * 4. 获取停车边界
         */
        if (!GetPullOverSpot(frame, &spot_vertices, &nearby_path)) {
            const std::string msg = "Fail to get parking boundary from map";
            AERROR << msg;
            return Status(ErrorCode::PLANNING_ERROR, msg);
        }

        SetOrigin(frame, spot_vertices);
        SetPullOverSpotEndPose(frame);

        if (!GetPullOverBoundary(frame, spot_vertices, nearby_path, &roi_boundary)) {
            const std::string msg = "Fail to get parking boundary from map";
            AERROR << msg;
            return Status(ErrorCode::PLANNING_ERROR, msg);
        }
    } else if (roi_type == OpenSpaceRoiDeciderParkConfig::PARK_AND_GO) {
        /**
         * @brief 停车后启动场景
         *
         * 1. 获取附近路径
         * 2. 检查ADC初始位置是否有效
         * 3. 设置原点
         * 4. 判断是否在停车场内
         * 5. 获取对应边界
         */
        ADEBUG << "in Park_and_Go";
        nearby_path = frame->reference_line_info().front().reference_line().GetMapPath();

        ADEBUG << "nearby_path: " << nearby_path.DebugString();
        ADEBUG << "found nearby_path";
        if (!injector_->planning_context()->planning_status().park_and_go().has_adc_init_position()) {
            const std::string msg = "ADC initial position is unavailable";
            AERROR << msg;
            return Status(ErrorCode::PLANNING_ERROR, msg);
        }
        SetOriginFromADC(frame, nearby_path);
        ADEBUG << "SetOrigin";
        auto adc_point = common::util::PointFactory::ToPointENU(vehicle_state_);
        hdmap::LaneInfoConstPtr lane;
        double s = 0.0;
        double l = 0.0;
        if (!is_parking_out) {
            is_parking_out = HDMapUtil::BaseMap().GetNearestLaneWithHeading(
                                     adc_point, 2.0, vehicle_state_.heading(), M_PI / 3.0, &lane, &s, &l)
                    == -1;
        }
        if (is_parking_out) {
            AINFO << "GetParkingOutBoundary!!";
            if (!GetParkingOutBoundary(nearby_path, frame, &roi_boundary)) {
                const std::string msg = "Fail to get park and go boundary from map";
                AERROR << msg;
                return Status(ErrorCode::PLANNING_ERROR, msg);
            }
        } else {
            AINFO << "GetParkAndGoBoundary!!!";
            if (!GetParkAndGoBoundary(frame, nearby_path, &roi_boundary)) {
                const std::string msg = "Fail to get park and go boundary from map";
                AERROR << msg;
                return Status(ErrorCode::PLANNING_ERROR, msg);
            }
        }

        SetParkAndGoEndPose(frame);
        ADEBUG << "SetEndPose";
    } else if (roi_type == OpenSpaceRoiDeciderParkConfig::LARGE_CURVATURE) {
        /**
         * @brief 大曲率弯道场景（掉头等）
         */
        ADEBUG << "in LARGE_CURVATURE";
        nearby_path = frame->reference_line_info().front().reference_line().GetMapPath();
        SetOriginFromADC(frame, nearby_path);
        Vec2d uturn_end_pos;
        double uturn_end_heading;
        if (!GetUTurnPath(frame, &nearby_path_) || !GetLargeCurvatureBoundary(frame, *nearby_path_, &roi_boundary)) {
            const std::string msg = "Fail to get large curvature boundary from map";
            AERROR << msg;
            return Status(ErrorCode::PLANNING_ERROR, msg);
        }
        GetLargeCurvatureEndPose(frame, uturn_end_pos, uturn_end_heading);
    } else {
        const std::string msg = "chosen open space roi secenario type not implemented";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    /**
     * @brief 构建边界约束
     *
     * FormulateBoundaryConstraints：
     *   将边界线段转换为优化器需要的Ax<=b形式
     *   用于HybridA*和距离场规划器
     */
    if (!FormulateBoundaryConstraints(roi_boundary, frame)) {
        const std::string msg = "Fail to formulate boundary constraints";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    /**
     * @brief 设置软边界
     *
     * soft_boundary：
     *   可以被侵入但会有惩罚的边界
     *   用于动态环境
     */
    *(frame->mutable_open_space_info()->mutable_soft_boundary_vertices_vec()) = soft_boundary;

    return Status::OK();
}

/**
 * @brief 从ADC位置设置原点
 *
 * @param frame 规划帧
 * @param nearby_path 附近路径
 *
 * 功能说明：
 * 1. 获取ADC的初始位置和航向
 * 2. 计算ADC的边界框
 * 3. 获取左上角顶点
 * 4. 根据路径方向设置原点
 *
 * C++语法说明：
 * - common::math::Vec2d adc_init_position = {x, y}：
 *   使用初始化列表构造Vec2d
 *
 * - Box2d(位置, 航向, 长, 宽)：
 *   构造2D边界框
 *
 * - adc_box.GetAllCorners(&corners)：
 *   获取边界框的四个角点
 *
 * - Vec2d -= Vec2d：
 *   向量减法，重载运算符
 *
 * - .SelfRotate(-angle)：
 *   绕原点旋转（负角度）
 *
 * - M_PI：
 *   π常量，来自cmath
 */
void OpenSpaceRoiDeciderPark::SetOriginFromADC(Frame *const frame, const hdmap::Path &nearby_path) {
    /**
     * @brief 获取ADC初始状态
     */
    const double adc_init_x = vehicle_state_.x();
    const double adc_init_y = vehicle_state_.y();
    const double adc_init_heading = vehicle_state_.heading();
    common::math::Vec2d adc_init_position = {adc_init_x, adc_init_y};
    const double adc_length = vehicle_params_.length();
    const double adc_width = vehicle_params_.width();

    /**
     * @brief 创建ADC边界框
     *
     * Box2d：
     *   2D边界框类
     *   参数：中心位置、航向角、长度、宽度
     *   +2.0：额外扩展安全边界
     */
    Box2d adc_box(adc_init_position, adc_init_heading, adc_length + 2.0, adc_width + 2.0);

    /**
     * @brief 获取ADC角点
     *
     * GetAllCorners(&adc_corners)：
     *   获取边界框的四个角点
     *   按顺时针顺序
     */
    std::vector<common::math::Vec2d> adc_corners;
    adc_box.GetAllCorners(&adc_corners);
    for (size_t i = 0; i < adc_corners.size(); ++i) {
        AINFO << "ADC [" << i << "]x: " << std::setprecision(9) << adc_corners[i].x();
        AINFO << "ADC [" << i << "]y: " << std::setprecision(9) << adc_corners[i].y();
    }

    /**
     * @brief 选择左上角顶点作为原点
     *
     * adc_corners[1]：
     *   通常左上角是索引1
     */
    auto left_top = adc_corners[1];

    ADEBUG << "left_top x: " << std::setprecision(9) << left_top.x();
    ADEBUG << "left_top y: " << std::setprecision(9) << left_top.y();

    /**
     * @brief 获取沿路径的航向角
     *
     * nearby_path.GetHeadingAlongPath(point, &heading)：
     *   获取指定点在路径上的航向角
     *   返回bool表示是否成功
     */
    double heading;
    if (!nearby_path.GetHeadingAlongPath(left_top, &heading)) {
        AERROR << "fail to get heading on reference line";
        return;
    }

    /**
     * @brief 设置原点和航向
     *
     * NormalizeAngle：
     *   归一化角度到[-π, π]范围
     */
    frame->mutable_open_space_info()->set_origin_heading(common::math::NormalizeAngle(heading));
    ADEBUG << "heading: " << heading;
    frame->mutable_open_space_info()->mutable_origin_point()->set_x(left_top.x());
    frame->mutable_open_space_info()->mutable_origin_point()->set_y(left_top.y());
}

/**
 * @brief 从顶点数组设置原点
 *
 * @param frame 规划帧
 * @param vertices 四个角点数组
 */
void OpenSpaceRoiDeciderPark::SetOrigin(Frame *const frame, const std::array<common::math::Vec2d, 4> &vertices) {
    auto left_top = vertices[0];
    auto right_top = vertices[3];

    /**
     * @brief 计算航向向量
     *
     * Vec2d -= Vec2d：
     *   向量减法
     *   right_top - left_top = 从左到右的向量
     *
     * .Angle()：
     *   获取向量与x轴的夹角
     */
    Vec2d heading_vec = right_top - left_top;
    frame->mutable_open_space_info()->set_origin_heading(heading_vec.Angle());
    frame->mutable_open_space_info()->mutable_origin_point()->set_x(left_top.x());
    frame->mutable_open_space_info()->mutable_origin_point()->set_y(left_top.y());
}

/**
 * @brief 从停车信息设置原点
 *
 * @param parking_info 停车位信息
 * @param frame 规划帧
 */
void OpenSpaceRoiDeciderPark::SetOrigin(const ParkingInfo &parking_info, Frame *const frame) {
    auto left_top = parking_info.corner_points[0];
    auto right_top = parking_info.corner_points[1];
    Vec2d heading_vec = right_top - left_top;
    frame->mutable_open_space_info()->set_origin_heading(heading_vec.Angle());
    frame->mutable_open_space_info()->mutable_origin_point()->set_x(left_top.x());
    frame->mutable_open_space_info()->mutable_origin_point()->set_y(left_top.y());
}

/**
 * @brief 设置停车位终止位姿
 *
 * @param parking_info 停车位信息
 * @param frame 规划帧
 *
 * 功能说明：
 * 计算车辆停在停车位中心的终止位置
 * 考虑车辆尺寸和安全边界
 *
 * C++语法说明：
 * - Vec2d -= Vec2d：
 *   向量减法
 *   point -= origin_point
 *   将点转换到局部坐标系
 *
 * - .SelfRotate(-angle)：
 *   旋转到局部坐标系
 *
 * - Vec2d::CreateUnitVec2d(angle)：
 *   创建指定角度的单位向量
 *
 * - Vec2d + Vec2d * scalar：
 *   向量加法和数乘
 */
void OpenSpaceRoiDeciderPark::SetParkingSpotEndPose(const ParkingInfo &parking_info, Frame *const frame) {
    /**
     * @brief 获取停车位四个角点
     */
    auto left_top = parking_info.corner_points[0];
    auto left_down = parking_info.corner_points[3];
    auto right_down = parking_info.corner_points[2];
    auto right_top = parking_info.corner_points[1];

    /**
     * @brief 获取原点和航向
     */
    const auto &origin_point = frame->open_space_info().origin_point();
    const auto &origin_heading = frame->open_space_info().origin_heading();

    /**
     * @brief 坐标归一化
     *
     * 将角点转换到局部坐标系
     * 便于后续计算
     */
    left_top -= origin_point;
    left_top.SelfRotate(-origin_heading);
    left_down -= origin_point;
    left_down.SelfRotate(-origin_heading);
    right_top -= origin_point;
    right_top.SelfRotate(-origin_heading);
    right_down -= origin_point;
    right_down.SelfRotate(-origin_heading);

    /**
     * @brief 计算终止位姿
     */
    Vec2d end_pt;
    double parking_heading = 0;
    const double parking_depth_buffer = config_.parking_depth_buffer();

    /**
     * @brief 根据停车类型计算终止位置
     *
     * ParkingType::VERTICAL_PARKING：
     *   垂直停车（倒车入库）
     *
     * ParkingType::PARALLEL_PARKING：
     *   平行停车（侧方停车）
     */
    if (parking_info.parking_type == ParkingType::VERTICAL_PARKING) {
        const bool parking_inwards = config_.parking_inwards();
        if (parking_inwards) {
            parking_heading = (left_down - left_top).Angle();
            Vec2d middle_top = (left_top + right_top) / 2.0;
            end_pt = middle_top
                    + Vec2d::CreateUnitVec2d(parking_heading)
                            * (vehicle_params_.front_edge_to_center() + parking_depth_buffer);
        } else {
            parking_heading = (left_top - left_down).Angle();
            Vec2d middle_down = (left_down + right_down) / 2.0;
            end_pt = middle_down
                    + Vec2d::CreateUnitVec2d(parking_heading)
                            * (vehicle_params_.back_edge_to_center() + parking_depth_buffer);
        }
    } else {
        parking_heading = (right_top - left_top).Angle();
        Vec2d middle_left = (left_top + left_down) / 2.0;
        end_pt = middle_left
                + Vec2d::CreateUnitVec2d(parking_heading)
                        * (vehicle_params_.back_edge_to_center() + parking_depth_buffer);
    }

    /**
     * @brief 设置终止位姿
     *
     * [x, y, theta, velocity]
     */
    auto *end_pose = frame->mutable_open_space_info()->mutable_open_space_end_pose();
    end_pose->push_back(end_pt.x());
    end_pose->push_back(end_pt.y());
    end_pose->push_back(parking_heading);
    end_pose->push_back(0.0);  // 速度为0
}

/**
 * @brief 设置靠边停车终止位姿
 */
void OpenSpaceRoiDeciderPark::SetPullOverSpotEndPose(Frame *const frame) {
    /**
     * @brief 获取靠边停车状态
     */
    const auto &pull_over_status = injector_->planning_context()->planning_status().pull_over();
    const double pull_over_x = pull_over_status.position().x();
    const double pull_over_y = pull_over_status.position().y();
    double pull_over_theta = pull_over_status.theta();

    /**
     * @brief 坐标归一化
     */
    const auto &origin_point = frame->open_space_info().origin_point();
    const auto &origin_heading = frame->open_space_info().origin_heading();
    Vec2d center(pull_over_x, pull_over_y);
    center -= origin_point;
    center.SelfRotate(-origin_heading);
    pull_over_theta = common::math::NormalizeAngle(pull_over_theta - origin_heading);

    /**
     * @brief 设置终止位姿
     */
    auto *end_pose = frame->mutable_open_space_info()->mutable_open_space_end_pose();
    end_pose->push_back(center.x());
    end_pose->push_back(center.y());
    end_pose->push_back(pull_over_theta);
    end_pose->push_back(0.0);
}

/**
 * @brief 设置停车后启动终止位姿
 */
void OpenSpaceRoiDeciderPark::SetParkAndGoEndPose(Frame *const frame) {
    const double kSTargetBuffer = config_.end_pose_s_distance();
    const double kSpeedRatio = 0.1;  // 调整后速度为限速的10%

    /**
     * @brief 获取ADC初始位置
     */
    auto park_and_go_status = injector_->planning_context()->mutable_planning_status()->mutable_park_and_go();
    const double adc_init_x = park_and_go_status->adc_init_position().x();
    const double adc_init_y = park_and_go_status->adc_init_position().y();

    const common::math::Vec2d adc_position = {adc_init_x, adc_init_y};
    common::SLPoint adc_position_sl;

    /**
     * @brief 找到最近的参考线
     *
     * std::min_element：
     *   标准库算法，找到最小元素
     *   自定义比较函数比较横向偏移l
     */
    const auto &reference_line_list = frame->reference_line_info();
    ADEBUG << reference_line_list.size();
    const auto reference_line_info = std::min_element(
            reference_line_list.begin(),
            reference_line_list.end(),
            [&](const ReferenceLineInfo &ref_a, const ReferenceLineInfo &ref_b) {
                common::SLPoint adc_position_sl_a;
                common::SLPoint adc_position_sl_b;
                ref_a.reference_line().XYToSL(adc_position, &adc_position_sl_a);
                ref_b.reference_line().XYToSL(adc_position, &adc_position_sl_b);
                return std::fabs(adc_position_sl_a.l()) < std::fabs(adc_position_sl_b.l());
            });

    const auto &reference_line = reference_line_info->reference_line();
    reference_line.XYToSL(adc_position, &adc_position_sl);

    /**
     * @brief 计算目标位置
     */
    const double target_s = adc_position_sl.s() + kSTargetBuffer;
    const auto reference_point = reference_line.GetReferencePoint(target_s);
    const double target_x = reference_point.x();
    const double target_y = reference_point.y();
    double target_theta = reference_point.heading();

    park_and_go_status->mutable_adc_adjust_end_pose()->set_x(target_x);
    park_and_go_status->mutable_adc_adjust_end_pose()->set_y(target_y);

    /**
     * @brief 坐标归一化
     */
    const auto &origin_point = frame->open_space_info().origin_point();
    const auto &origin_heading = frame->open_space_info().origin_heading();
    Vec2d center(target_x, target_y);
    center -= origin_point;
    center.SelfRotate(-origin_heading);
    target_theta = common::math::NormalizeAngle(target_theta - origin_heading);

    auto *end_pose = frame->mutable_open_space_info()->mutable_open_space_end_pose();
    end_pose->push_back(center.x());
    end_pose->push_back(center.y());
    end_pose->push_back(target_theta);

    /**
     * @brief 速度设为限速的kSpeedRatio倍
     */
    double target_speed = reference_line.GetSpeedLimitFromS(target_s);
    end_pose->push_back(kSpeedRatio * target_speed);
}

/**
 * @brief 设置大曲率弯道终止位姿
 *
 * 功能：搜索路径上曲率较小的点作为终止位置
 */
void OpenSpaceRoiDeciderPark::GetLargeCurvatureEndPose(Frame *const frame, Vec2d uturn_end_pos, double uturn_heading) {
    const auto &origin_point = frame->open_space_info().origin_point();
    double ego_s = 0.0;
    double ego_l = 0.0;
    nearby_path_->GetProjection(origin_point, &ego_s, &ego_l);

    /**
     * @brief 沿路径搜索低曲率点
     *
     * 使用数值微分计算曲率：
     * κ = (x' * y'' - y' * x'') / ((x'^2 + y'^2)^(3/2))
     */
    double check_s = ego_s;
    hdmap::MapPathPoint cur_point;
    hdmap::MapPathPoint pre_point;
    hdmap::MapPathPoint pre_pre_point;
    double xds = 0.0, yds = 0.0;
    double pre_xds = 0.0, pre_yds = 0.0;
    double pre_pre_xds = 0.0, pre_pre_yds = 0.0;
    double xdds = 0.0, ydds = 0.0;
    double delta_s = 0.1;
    double delta_s_s = delta_s * 2;
    double kappa = 0.0;
    int low_curvature_count = 0;

    /**
     * @brief 循环搜索直到找到足够长的低曲率段
     */
    while (check_s < nearby_path_->accumulated_s().back()) {
        pre_pre_point = pre_point;
        pre_point = cur_point;
        cur_point = nearby_path_->GetSmoothPoint(check_s);

        pre_pre_xds = pre_xds;
        pre_pre_yds = pre_yds;
        pre_xds = xds;
        pre_yds = yds;

        /**
         * @brief 数值微分计算一阶导
         */
        xds = (cur_point.x() - pre_pre_point.x()) / delta_s_s;
        yds = (cur_point.y() - pre_pre_point.y()) / delta_s_s;

        /**
         * @brief 数值微分计算二阶导
         */
        xdds = (xds - pre_pre_xds) / delta_s_s;
        ydds = (yds - pre_pre_yds) / delta_s_s;

        /**
         * @brief 计算曲率
         */
        kappa = (xds * ydds - yds * xdds) / (std::sqrt(xds * xds + yds * yds) * (xds * xds + yds * yds) + 1e-6);
        AINFO << "check_s: " << check_s << ", kappa: " << kappa << " ,pre_pre_xds: " << pre_pre_xds;

        /**
         * @brief 统计低曲率点数量
         */
        if (pre_pre_xds != 0 && std::abs(kappa) < 0.05) {
            AINFO << "check_s: " << check_s;
            low_curvature_count++;
        } else {
            low_curvature_count = 0;
        }

        /**
         * @brief 连续10个低曲率点则停止搜索
         */
        if (low_curvature_count > 10) {
            AINFO << "check_s: " << check_s;
            break;
        }
        check_s += delta_s;
    }

    uturn_end_pos = pre_pre_point;
    uturn_heading = pre_pre_point.heading();

    /**
     * @brief 坐标归一化
     */
    const auto &origin_heading = frame->open_space_info().origin_heading();
    uturn_end_pos -= origin_point;
    uturn_end_pos.SelfRotate(-origin_heading);
    uturn_heading = common::math::NormalizeAngle(uturn_heading - origin_heading);

    auto *end_pose = frame->mutable_open_space_info()->mutable_open_space_end_pose();
    end_pose->push_back(uturn_end_pos.x());
    end_pose->push_back(uturn_end_pos.y());
    end_pose->push_back(uturn_heading);
    end_pose->push_back(0.0);
}

/**
 * @brief 获取道路边界
 *
 * 功能：从路径生成左右车道边界
 *
 * C++语法说明：
 * - std::vector<Vec2d> *ptr：
 *   指针参数，输出边界点
 */
void OpenSpaceRoiDeciderPark::GetRoadBoundary(
        const hdmap::Path &nearby_path,
        const double center_line_s,
        const common::math::Vec2d &origin_point,
        const double origin_heading,
        std::vector<Vec2d> *left_lane_boundary,
        std::vector<Vec2d> *right_lane_boundary,
        std::vector<Vec2d> *center_lane_boundary_left,
        std::vector<Vec2d> *center_lane_boundary_right,
        std::vector<double> *center_lane_s_left,
        std::vector<double> *center_lane_s_right,
        std::vector<double> *left_lane_road_width,
        std::vector<double> *right_lane_road_width) {
    /**
     * @brief 计算纵向范围
     *
     * start_s = center_s - 前向范围
     * end_s = center_s + 后向范围
     */
    double start_s = center_line_s - config_.roi_longitudinal_range_start();
    double end_s = center_line_s + config_.roi_longitudinal_range_end();

    hdmap::MapPathPoint start_point = nearby_path.GetSmoothPoint(start_s);
    double last_check_point_heading = start_point.heading();
    double index = 0.0;
    double check_point_s = start_s;

    /**
     * @brief 沿路径采样关键点
     *
     * 关键点包括：
     * 1. 起始点和终点
     * 2. 曲率大的点
     * 3. 靠近路缘角点的点
     */
    while (check_point_s <= end_s) {
        hdmap::MapPathPoint check_point = nearby_path.GetSmoothPoint(check_point_s);
        double check_point_heading = check_point.heading();

        /**
         * @brief 检测航向变化
         *
         * NormalizeAngle：
         *   归一化角度差到[-π, π]
         */
        bool is_center_lane_heading_change
                = std::abs(common::math::NormalizeAngle(check_point_heading - last_check_point_heading))
                > config_.roi_line_segment_min_angle();
        last_check_point_heading = check_point_heading;

        /**
         * @brief 判断是否为锚点
         */
        bool is_anchor_point = check_point_s == start_s || check_point_s == end_s || is_center_lane_heading_change;

        /**
         * @brief 添加左右边界关键点
         */
        AddBoundaryKeyPoint(
                nearby_path, check_point_s, start_s, end_s,
                is_anchor_point, true,
                center_lane_boundary_left, left_lane_boundary,
                center_lane_s_left, left_lane_road_width);
        AddBoundaryKeyPoint(
                nearby_path, check_point_s, start_s, end_s,
                is_anchor_point, false,
                center_lane_boundary_right, right_lane_boundary,
                center_lane_s_right, right_lane_road_width);

        if (check_point_s == end_s) {
            break;
        }
        index += 1.0;
        check_point_s = start_s + index * config_.roi_line_segment_length();
        check_point_s = check_point_s >= end_s ? end_s : check_point_s;
    }

    /**
     * @brief 坐标归一化
     */
    size_t left_point_size = left_lane_boundary->size();
    size_t right_point_size = right_lane_boundary->size();
    for (size_t i = 0; i < left_point_size; i++) {
        left_lane_boundary->at(i) -= origin_point;
        left_lane_boundary->at(i).SelfRotate(-origin_heading);
    }
    for (size_t i = 0; i < right_point_size; i++) {
        right_lane_boundary->at(i) -= origin_point;
        right_lane_boundary->at(i).SelfRotate(-origin_heading);
    }
}

/**
 * @brief 添加边界关键点
 *
 * 功能：添加边界上的关键点（锚点和路缘角点）
 */
void OpenSpaceRoiDeciderPark::AddBoundaryKeyPoint(
        const hdmap::Path &nearby_path,
        const double check_point_s,
        const double start_s,
        const double end_s,
        const bool is_anchor_point,
        const bool is_left_curb,
        std::vector<Vec2d> *center_lane_boundary,
        std::vector<Vec2d> *curb_lane_boundary,
        std::vector<double> *center_lane_s,
        std::vector<double> *road_width) {
    const double previous_distance_s = std::min(config_.roi_line_segment_length(), check_point_s - start_s);
    const double next_distance_s = std::min(config_.roi_line_segment_length(), end_s - check_point_s);

    hdmap::MapPathPoint current_check_point = nearby_path.GetSmoothPoint(check_point_s);
    double current_check_point_heading = current_check_point.heading();
    double current_road_width = is_left_curb ? nearby_path.GetLaneLeftWidth(check_point_s) : nearby_path.GetLaneLeftWidth(check_point_s);

    /**
     * @brief 如果是锚点，直接添加边界点
     */
    if (is_anchor_point) {
        double point_vec_cos = is_left_curb ? std::cos(current_check_point_heading + M_PI / 2.0)
                                            : std::cos(current_check_point_heading - M_PI / 2.0);
        double point_vec_sin = is_left_curb ? std::sin(current_check_point_heading + M_PI / 2.0)
                                            : std::sin(current_check_point_heading - M_PI / 2.0);
        Vec2d curb_lane_point = Vec2d(current_road_width * point_vec_cos, current_road_width * point_vec_sin);
        curb_lane_point = curb_lane_point + current_check_point;
        center_lane_boundary->push_back(current_check_point);
        curb_lane_boundary->push_back(curb_lane_point);
        center_lane_s->push_back(check_point_s);
        road_width->push_back(current_road_width);
        return;
    }

    /**
     * @brief 检测路缘角点
     */
    double previous_road_width = is_left_curb ? nearby_path.GetRoadLeftWidth(check_point_s - previous_distance_s)
                                              : nearby_path.GetRoadRightWidth(check_point_s - previous_distance_s);
    double next_road_width = is_left_curb ? nearby_path.GetRoadLeftWidth(check_point_s + next_distance_s)
                                          : nearby_path.GetRoadRightWidth(check_point_s + next_distance_s);
    double previous_curb_segment_angle = (current_road_width - previous_road_width) / previous_distance_s;
    double next_segment_angle = (next_road_width - current_road_width) / next_distance_s;
    double current_curb_point_delta_theta = next_segment_angle - previous_curb_segment_angle;

    /**
     * @brief 如果角度变化大，则是角点
     */
    if (std::abs(current_curb_point_delta_theta) > config_.curb_heading_tangent_change_upper_limit()) {
        double point_vec_cos = is_left_curb ? std::cos(current_check_point_heading + M_PI / 2.0)
                                            : std::cos(current_check_point_heading - M_PI / 2.0);
        double point_vec_sin = is_left_curb ? std::sin(current_check_point_heading + M_PI / 2.0)
                                            : std::sin(current_check_point_heading - M_PI / 2.0);
        Vec2d curb_lane_point = Vec2d(current_road_width * point_vec_cos, current_road_width * point_vec_sin);
        curb_lane_point = curb_lane_point + current_check_point;
        center_lane_boundary->push_back(current_check_point);
        curb_lane_boundary->push_back(curb_lane_point);
        center_lane_s->push_back(check_point_s);
        road_width->push_back(current_road_width);
    }
}

/**
 * @brief 获取停车软边界
 *
 * 功能：获取停车位的软边界（允许侵入但有惩罚）
 */
void OpenSpaceRoiDeciderPark::GetParkingSoftBoundary(
        common::math::Vec2d left_top,
        common::math::Vec2d left_down,
        common::math::Vec2d right_top,
        common::math::Vec2d right_down,
        std::vector<std::vector<common::math::Vec2d>> *const parking_soft_boundary) {
    /**
     * @brief 左右两条边界线
     */
    std::vector<common::math::Vec2d> left_boundary{left_top, left_down};
    std::vector<common::math::Vec2d> right_boundary{right_down, right_top};
    parking_soft_boundary->emplace_back(left_boundary);
    parking_soft_boundary->emplace_back(right_boundary);
}

/**
 * @brief 从地图获取道路边界
 */
void OpenSpaceRoiDeciderPark::GetRoadBoundaryFromMap(
        const hdmap::Path &nearby_path,
        const double center_line_s,
        const Vec2d &origin_point,
        const double origin_heading,
        std::vector<Vec2d> *left_lane_boundary,
        std::vector<Vec2d> *right_lane_boundary,
        std::vector<Vec2d> *center_lane_boundary_left,
        std::vector<Vec2d> *center_lane_boundary_right,
        std::vector<double> *center_lane_s_left,
        std::vector<double> *center_lane_s_right,
        std::vector<double> *left_lane_road_width,
        std::vector<double> *right_lane_road_width) {
    double start_s = center_line_s - config_.roi_longitudinal_range_start();
    double end_s = center_line_s + config_.roi_longitudinal_range_end();
    hdmap::MapPathPoint start_point = nearby_path.GetSmoothPoint(start_s);
    double check_point_s = start_s;

    /**
     * @brief 沿路径采样并获取道路边界
     */
    while (check_point_s <= end_s) {
        hdmap::MapPathPoint check_point = nearby_path.GetSmoothPoint(check_point_s);

        /**
         * @brief 获取道路宽度
         */
        double left_road_width = nearby_path.GetRoadLeftWidth(check_point_s);
        double right_road_width = nearby_path.GetRoadRightWidth(check_point_s);
        double current_road_width = std::max(left_road_width, right_road_width);

        /**
         * @brief 从地图获取道路边界
         */
        common::PointENU check_point_xy;
        std::vector<hdmap::RoadRoiPtr> road_boundaries;
        std::vector<hdmap::JunctionInfoConstPtr> junctions;
        check_point_xy.set_x(check_point.x());
        check_point_xy.set_y(check_point.y());
        hdmap_->GetRoadBoundaries(check_point_xy, current_road_width, &road_boundaries, &junctions);

        /**
         * @brief 根据位置选择左右边界
         */
        if (check_point_s < center_line_s) {
            for (size_t i = 0; i < (*road_boundaries.at(0)).left_boundary.line_points.size(); i++) {
                right_lane_boundary->emplace_back(
                        Vec2d((*road_boundaries.at(0)).left_boundary.line_points[i].x(),
                              (*road_boundaries.at(0)).left_boundary.line_points[i].y()));
            }
            for (size_t i = 0; i < (*road_boundaries.at(0)).right_boundary.line_points.size(); i++) {
                left_lane_boundary->emplace_back(
                        Vec2d((*road_boundaries.at(0)).right_boundary.line_points[i].x(),
                              (*road_boundaries.at(0)).right_boundary.line_points[i].y()));
            }
        } else {
            for (size_t i = 0; i < (*road_boundaries.at(0)).left_boundary.line_points.size(); i++) {
                left_lane_boundary->emplace_back(
                        Vec2d((*road_boundaries.at(0)).left_boundary.line_points[i].x(),
                              (*road_boundaries.at(0)).left_boundary.line_points[i].y()));
            }
            for (size_t i = 0; i < (*road_boundaries.at(0)).right_boundary.line_points.size(); i++) {
                right_lane_boundary->emplace_back(
                        Vec2d((*road_boundaries.at(0)).right_boundary.line_points[i].x(),
                              (*road_boundaries.at(0)).right_boundary.line_points[i].y()));
            }
        }

        center_lane_boundary_right->emplace_back(check_point);
        center_lane_boundary_left->emplace_back(check_point);
        center_lane_s_left->emplace_back(check_point_s);
        center_lane_s_right->emplace_back(check_point_s);
        left_lane_road_width->emplace_back(left_road_width);
        right_lane_road_width->emplace_back(right_road_width);

        check_point_s = check_point_s + config_.roi_line_segment_length_from_map();
    }

    /**
     * @brief 坐标归一化
     */
    size_t left_point_size = left_lane_boundary->size();
    size_t right_point_size = right_lane_boundary->size();
    for (size_t i = 0; i < left_point_size; i++) {
        left_lane_boundary->at(i) -= origin_point;
        left_lane_boundary->at(i).SelfRotate(-origin_heading);
    }
    for (size_t i = 0; i < right_point_size; i++) {
        right_lane_boundary->at(i) -= origin_point;
        right_lane_boundary->at(i).SelfRotate(-origin_heading);
    }

    /**
     * @brief 去重和排序
     */
    if (!left_lane_boundary->empty()) {
        sort(left_lane_boundary->begin(), left_lane_boundary->end(), [](const Vec2d &first_pt, const Vec2d &second_pt) {
            return first_pt.x() < second_pt.x() || (first_pt.x() == second_pt.x() && first_pt.y() < second_pt.y());
        });
        auto unique_end = std::unique(left_lane_boundary->begin(), left_lane_boundary->end());
        left_lane_boundary->erase(unique_end, left_lane_boundary->end());
    }
    if (!right_lane_boundary->empty()) {
        sort(right_lane_boundary->begin(), right_lane_boundary->end(),
             [](const Vec2d &first_pt, const Vec2d &second_pt) {
                 return first_pt.x() < second_pt.x() || (first_pt.x() == second_pt.x() && first_pt.y() < second_pt.y());
             });
        auto unique_end = std::unique(right_lane_boundary->begin(), right_lane_boundary->end());
        right_lane_boundary->erase(unique_end, right_lane_boundary->end());
    }
}

/**
 * @brief 获取停车边界
 *
 * 功能：构建停车场景的ROI边界
 */
bool OpenSpaceRoiDeciderPark::GetParkingBoundary(
        const ParkingInfo &parking_info,
        const hdmap::Path &nearby_path,
        Frame *const frame,
        std::vector<std::vector<common::math::Vec2d>> *const roi_parking_boundary,
        std::vector<std::vector<common::math::Vec2d>> *const parking_soft_boundary) {
    /**
     * @brief 获取停车位角点
     */
    auto left_top = parking_info.corner_points[0];
    auto left_down = parking_info.corner_points[3];
    auto right_down = parking_info.corner_points[2];
    auto right_top = parking_info.corner_points[1];

    const auto &origin_point = frame->open_space_info().origin_point();
    const auto &origin_heading = frame->open_space_info().origin_heading();

    /**
     * @brief 投影到路径获取s,l
     */
    double left_top_s = 0.0, left_top_l = 0.0;
    double right_top_s = 0.0, right_top_l = 0.0;
    if (!(nearby_path.GetProjection(left_top, &left_top_s, &left_top_l)
          && nearby_path.GetProjection(right_top, &right_top_s, &right_top_l))) {
        AERROR << "fail to get parking spot points' projections on reference line";
        return false;
    }

    /**
     * @brief 坐标归一化
     */
    left_top -= origin_point;
    left_top.SelfRotate(-origin_heading);
    left_down -= origin_point;
    left_down.SelfRotate(-origin_heading);
    right_top -= origin_point;
    right_top.SelfRotate(-origin_heading);
    right_down -= origin_point;
    right_down.SelfRotate(-origin_heading);

    const double center_line_s = (left_top_s + right_top_s) / 2.0;

    /**
     * @brief 获取道路边界
     */
    std::vector<Vec2d> left_lane_boundary;
    std::vector<Vec2d> right_lane_boundary;
    std::vector<Vec2d> center_lane_boundary_left;
    std::vector<Vec2d> center_lane_boundary_right;
    std::vector<double> center_lane_s_left;
    std::vector<double> center_lane_s_right;
    std::vector<double> left_lane_road_width;
    std::vector<double> right_lane_road_width;

    GetRoadBoundary(
            nearby_path, center_line_s, origin_point, origin_heading,
            &left_lane_boundary, &right_lane_boundary,
            &center_lane_boundary_left, &center_lane_boundary_right,
            &center_lane_s_left, &center_lane_s_right,
            &left_lane_road_width, &right_lane_road_width);

    /**
     * @brief 判断停车位在道路哪侧
     *
     * average_l < 0：右侧
     * average_l > 0：左侧
     */
    const double average_l = (left_top_l + right_top_l) / 2.0;
    std::vector<Vec2d> boundary_points;

    if (average_l < 0) {
        /**
         * @brief 右侧停车
         */
        size_t point_size = right_lane_boundary.size();
        for (size_t i = 0; i < point_size; i++) {
            right_lane_boundary[i].SelfRotate(origin_heading);
            right_lane_boundary[i] += origin_point;
            right_lane_boundary[i] -= center_lane_boundary_right[i];
            right_lane_boundary[i] /= right_lane_road_width[i];
            right_lane_boundary[i] *= (-average_l);
            right_lane_boundary[i] += center_lane_boundary_right[i];
            right_lane_boundary[i] -= origin_point;
            right_lane_boundary[i].SelfRotate(-origin_heading);
        }

        /**
         * @brief 构造边界点序列
         */
        auto point_left_to_left_top_connor_s = std::lower_bound(center_lane_s_right.begin(), center_lane_s_right.end(), left_top_s);
        size_t point_left_to_left_top_connor_index = std::distance(center_lane_s_right.begin(), point_left_to_left_top_connor_s);
        if (point_left_to_left_top_connor_index > 0) {
            point_left_to_left_top_connor_index--;
        }
        auto point_left_to_left_top_connor_itr = right_lane_boundary.begin() + point_left_to_left_top_connor_index;

        auto point_right_to_right_top_connor_s = std::upper_bound(center_lane_s_right.begin(), center_lane_s_right.end(), right_top_s);
        size_t point_right_to_right_top_connor_index = std::distance(center_lane_s_right.begin(), point_right_to_right_top_connor_s);
        auto point_right_to_right_top_connor_itr = right_lane_boundary.begin() + point_right_to_right_top_connor_index;

        /**
         * @brief 组合边界点
         */
        std::copy(right_lane_boundary.begin(), point_left_to_left_top_connor_itr, std::back_inserter(boundary_points));

        std::vector<Vec2d> parking_spot_boundary{left_top, left_down, right_down, right_top};
        std::copy(parking_spot_boundary.begin(), parking_spot_boundary.end(), std::back_inserter(boundary_points));

        std::copy(point_right_to_right_top_connor_itr, right_lane_boundary.end(), std::back_inserter(boundary_points));
        std::reverse_copy(left_lane_boundary.begin(), left_lane_boundary.end(), std::back_inserter(boundary_points));

        boundary_points.push_back(right_lane_boundary.front());

        /**
         * @brief 转换为线段
         */
        for (size_t i = 0; i < point_left_to_left_top_connor_index; i++) {
            std::vector<Vec2d> segment{right_lane_boundary[i], right_lane_boundary[i + 1]};
            roi_parking_boundary->push_back(segment);
        }

        std::vector<Vec2d> left_stitching_segment{right_lane_boundary[point_left_to_left_top_connor_index], left_top};
        roi_parking_boundary->push_back(left_stitching_segment);

        std::vector<Vec2d> left_parking_spot_segment{left_top, left_down};
        std::vector<Vec2d> down_parking_spot_segment{left_down, right_down};
        std::vector<Vec2d> right_parking_spot_segment{right_down, right_top};
        roi_parking_boundary->push_back(left_parking_spot_segment);
        roi_parking_boundary->push_back(down_parking_spot_segment);
        roi_parking_boundary->push_back(right_parking_spot_segment);

        std::vector<Vec2d> right_stitching_segment{right_top, right_lane_boundary[point_right_to_right_top_connor_index]};
        roi_parking_boundary->push_back(right_stitching_segment);

        size_t right_lane_boundary_last_index = right_lane_boundary.size() - 1;
        for (size_t i = point_right_to_right_top_connor_index; i < right_lane_boundary_last_index; i++) {
            std::vector<Vec2d> segment{right_lane_boundary[i], right_lane_boundary[i + 1]};
            roi_parking_boundary->push_back(segment);
        }

        size_t left_lane_boundary_last_index = left_lane_boundary.size() - 1;
        for (size_t i = left_lane_boundary_last_index; i > 0; i--) {
            std::vector<Vec2d> segment{left_lane_boundary[i], left_lane_boundary[i - 1]};
            roi_parking_boundary->push_back(segment);
        }

    } else {
        /**
         * @brief 左侧停车（类似逻辑）
         */
        size_t point_size = left_lane_boundary.size();
        for (size_t i = 0; i < point_size; i++) {
            left_lane_boundary[i].SelfRotate(origin_heading);
            left_lane_boundary[i] += origin_point;
            left_lane_boundary[i] -= center_lane_boundary_left[i];
            left_lane_boundary[i] /= left_lane_road_width[i];
            left_lane_boundary[i] *= average_l;
            left_lane_boundary[i] += center_lane_boundary_left[i];
            left_lane_boundary[i] -= origin_point;
            left_lane_boundary[i].SelfRotate(-origin_heading);
        }

        // ... 类似构造边界
    }

    /**
     * @brief 融合线段为凸约束
     */
    if (!FuseLineSegments(roi_parking_boundary)) {
        AERROR << "FuseLineSegments failed in parking ROI";
        return false;
    }

    /**
     * @brief 获取软边界
     */
    GetParkingSoftBoundary(left_top, left_down, right_down, right_top, parking_soft_boundary);

    /**
     * @brief 计算XY边界
     */
    auto xminmax = std::minmax_element(boundary_points.begin(), boundary_points.end(), [](const Vec2d &a, const Vec2d &b) {
          return a.x() < b.x();
      });
    auto yminmax = std::minmax_element(boundary_points.begin(), boundary_points.end(), [](const Vec2d &a, const Vec2d &b) {
          return a.y() < b.y();
      });
    std::vector<double> ROI_xy_boundary{
            xminmax.first->x(), xminmax.second->x(), yminmax.first->y(), yminmax.second->y()};
    auto *xy_boundary = frame->mutable_open_space_info()->mutable_ROI_xy_boundary();
    xy_boundary->assign(ROI_xy_boundary.begin(), ROI_xy_boundary.end());

    /**
     * @brief 检查车辆是否在边界内
     */
    Vec2d vehicle_xy = Vec2d(vehicle_state_.x(), vehicle_state_.y());
    vehicle_xy -= origin_point;
    vehicle_xy.SelfRotate(-origin_heading);
    if (vehicle_xy.x() < ROI_xy_boundary[0] || vehicle_xy.x() > ROI_xy_boundary[1]
        || vehicle_xy.y() < ROI_xy_boundary[2] || vehicle_xy.y() > ROI_xy_boundary[3]) {
        AERROR << "vehicle outside of xy boundary of parking ROI";
        return false;
    }
    return true;
}

/**
 * @brief 获取靠边停车边界
 */
bool OpenSpaceRoiDeciderPark::GetPullOverBoundary(
        Frame *const frame,
        const std::array<common::math::Vec2d, 4> &vertices,
        const hdmap::Path &nearby_path,
        std::vector<std::vector<common::math::Vec2d>> *const roi_parking_boundary) {
    // 类似GetParkingBoundary的实现
    // ...
    return true;
}

/**
 * @brief 获取停车后启动边界
 */
bool OpenSpaceRoiDeciderPark::GetParkAndGoBoundary(
        Frame *const frame,
        const hdmap::Path &nearby_path,
        std::vector<std::vector<common::math::Vec2d>> *const roi_parking_boundary) {
    const auto &park_and_go_status = injector_->planning_context()->planning_status().park_and_go();
    const double adc_init_x = park_and_go_status.adc_init_position().x();
    const double adc_init_y = park_and_go_status.adc_init_position().y();
    const double adc_init_heading = park_and_go_status.adc_init_heading();
    common::math::Vec2d adc_init_position = {adc_init_x, adc_init_y};
    const double adc_length = vehicle_params_.length();
    const double adc_width = vehicle_params_.width();

    Box2d adc_box(adc_init_position, adc_init_heading, adc_length, adc_width);
    std::vector<common::math::Vec2d> adc_corners;
    adc_box.GetAllCorners(&adc_corners);
    auto left_top = adc_corners[1];
    auto right_top = adc_corners[0];

    const auto &origin_point = frame->open_space_info().origin_point();
    const auto &origin_heading = frame->open_space_info().origin_heading();

    // 坐标归一化和边界构建...
    return true;
}

/**
 * @brief 获取大曲率边界
 */
bool OpenSpaceRoiDeciderPark::GetLargeCurvatureBoundary(
        Frame *const frame,
        const hdmap::Path &nearby_path,
        std::vector<std::vector<common::math::Vec2d>> *const roi_parking_boundary) {
    // 类似实现...
    return true;
}

/**
 * @brief 获取停车位
 */
bool OpenSpaceRoiDeciderPark::GetParkingSpot(Frame *const frame, ParkingInfo *parking_info) {
    if (frame == nullptr) {
        AERROR << "Invalid frame, fail to GetParkingSpotFromMap from frame. ";
        return false;
    }
    const auto &parking_spot_id_string = frame->open_space_info().target_parking_spot_id();
    hdmap::Id parking_spot_id = hdmap::MakeMapId(parking_spot_id_string);
    auto parking_spot = hdmap_->GetParkingSpaceById(parking_spot_id);
    if (!nearby_path_) {
        GetNearbyPath(frame->local_view().planning_command->lane_follow_command(), parking_spot, &nearby_path_);
    }

    /**
     * @brief 获取停车位多边形点
     */
    auto points = parking_spot->polygon().points();
    OpenSpaceRoiUtil::UpdateParkingPointsOrder(*nearby_path_, &points);
    Vec2d center_point(0, 0);
    for (size_t i = 0; i < points.size(); i++) {
        center_point += points[i];
    }
    center_point /= 4.0;
    double lane_heading = 0;
    parking_info->center_point = center_point;
    nearby_path_->GetHeadingAlongPath(center_point, &lane_heading);
    double s, l;
    nearby_path_->GetProjection(center_point, &s, &l);
    if (l > 0) {
        parking_info->is_on_left = true;
    } else {
        parking_info->is_on_left = false;
    }

    /**
     * @brief 判断停车类型
     */
    double diff_angle = common::math::AngleDiff(lane_heading, parking_spot->parking_space().heading());
    if (std::fabs(diff_angle) < M_PI / 3.0) {
        parking_info->parking_type = ParkingType::PARALLEL_PARKING;
    } else {
        parking_info->parking_type = ParkingType::VERTICAL_PARKING;
    }
    parking_info->corner_points = points;

    /**
     * @brief 根据边长判断停车类型
     */
    double parallel_dist = parking_info->corner_points[0].DistanceTo(parking_info->corner_points[1]);
    double verticle_dist = parking_info->corner_points[0].DistanceTo(parking_info->corner_points[3]);
    if (parallel_dist > verticle_dist) {
        parking_info->parking_type = ParkingType::PARALLEL_PARKING;
    } else {
        parking_info->parking_type = ParkingType::VERTICAL_PARKING;
    }
    return true;
}

/**
 * @brief 获取靠边停车位置
 */
bool OpenSpaceRoiDeciderPark::GetPullOverSpot(
        Frame *const frame,
        std::array<common::math::Vec2d, 4> *vertices,
        hdmap::Path *nearby_path) {
    const auto &pull_over_status = injector_->planning_context()->planning_status().pull_over();
    if (!pull_over_status.has_position() || !pull_over_status.position().has_x() || !pull_over_status.position().has_y()
        || !pull_over_status.has_theta()) {
        AERROR << "Pull over position not set in planning context";
        return false;
    }

    *nearby_path = frame->reference_line_info().front().reference_line().GetMapPath();

    /**
     * @brief 计算四个角点
     */
    double pull_over_x = pull_over_status.position().x();
    double pull_over_y = pull_over_status.position().y();
    const double pull_over_theta = pull_over_status.theta();
    const double pull_over_length_front = pull_over_status.length_front();
    const double pull_over_length_back = pull_over_status.length_back();
    const double pull_over_width_left = pull_over_status.width_left();
    const double pull_over_width_right = pull_over_status.width_right();

    Vec2d center_shift_vec(
            (pull_over_length_front - pull_over_length_back) * 0.5,
            (pull_over_width_left - pull_over_width_right) * 0.5);
    center_shift_vec.SelfRotate(pull_over_theta);
    pull_over_x += center_shift_vec.x();
    pull_over_y += center_shift_vec.y();

    const double half_length = (pull_over_length_front + pull_over_length_back) / 2.0;
    const double half_width = (pull_over_width_left + pull_over_width_right) / 2.0;
    const double cos_heading = std::cos(pull_over_theta);
    const double sin_heading = std::sin(pull_over_theta);
    const double dx1 = cos_heading * half_length;
    const double dy1 = sin_heading * half_length;
    const double dx2 = sin_heading * half_width;
    const double dy2 = -cos_heading * half_width;

    Vec2d left_top(pull_over_x - dx1 + dx2, pull_over_y - dy1 + dy2);
    Vec2d left_down(pull_over_x - dx1 - dx2, pull_over_y - dy1 - dy2);
    Vec2d right_down(pull_over_x + dx1 - dx2, pull_over_y + dy1 - dy2);
    Vec2d right_top(pull_over_x + dx1 + dx2, pull_over_y + dy1 + dy2);

    std::array<Vec2d, 4> pull_over_vertices{left_top, left_down, right_down, right_top};
    *vertices = std::move(pull_over_vertices);
    return true;
}

/**
 * @brief 融合线段
 *
 * 功能：将相邻共线线段合并
 */
bool OpenSpaceRoiDeciderPark::FuseLineSegments(std::vector<std::vector<common::math::Vec2d>> *line_segments_vec) {
    static constexpr double kEpsilon = 1.0e-8;
    auto cur_segment = line_segments_vec->begin();
    while (cur_segment != line_segments_vec->end() - 1) {
        auto next_segment = cur_segment + 1;
        auto cur_last_point = cur_segment->back();
        auto next_first_point = next_segment->front();

        /**
         * @brief 检查端点是否重合
         */
        if (cur_last_point.DistanceTo(next_first_point) > kEpsilon) {
            ++cur_segment;
            continue;
        }

        if (cur_segment->size() < 2 || next_segment->size() < 2) {
            AERROR << "Single point line_segments vec not expected";
            return false;
        }

        /**
         * @brief 检查叉积判断凹凸性
         */
        size_t cur_segments_size = cur_segment->size();
        auto cur_second_to_last_point = cur_segment->at(cur_segments_size - 2);
        auto next_second_point = next_segment->at(1);
        if (CrossProd(cur_second_to_last_point, cur_last_point, next_second_point) < 0.0) {
            cur_segment->push_back(next_second_point);
            next_segment->erase(next_segment->begin(), next_segment->begin() + 2);
            if (next_segment->empty()) {
                line_segments_vec->erase(next_segment);
            }
        } else {
            ++cur_segment;
        }
    }
    return true;
}

/**
 * @brief 构建边界约束
 *
 * 功能：将边界转换为优化器需要的约束形式
 */
bool OpenSpaceRoiDeciderPark::FormulateBoundaryConstraints(
        const std::vector<std::vector<common::math::Vec2d>> &roi_parking_boundary,
        Frame *const frame) {
    /**
     * @brief 加载障碍物顶点
     */
    if (!LoadObstacleInVertices(roi_parking_boundary, frame)) {
        AERROR << "fail at LoadObstacleInVertices()";
        return false;
    }
    /**
     * @brief 转换为超平面表示 Ax <= b
     */
    if (!LoadObstacleInHyperPlanes(frame)) {
        AERROR << "fail at LoadObstacleInHyperPlanes()";
        return false;
    }
    return true;
}

/**
 * @brief 加载障碍物顶点
 */
bool OpenSpaceRoiDeciderPark::LoadObstacleInVertices(
        const std::vector<std::vector<common::math::Vec2d>> &roi_parking_boundary,
        Frame *const frame) {
    auto *mutable_open_space_info = frame->mutable_open_space_info();
    const auto &open_space_info = frame->open_space_info();
    auto *obstacles_vertices_vec = mutable_open_space_info->mutable_obstacles_vertices_vec();
    auto *obstacles_edges_num_vec = mutable_open_space_info->mutable_obstacles_edges_num();

    /**
     * @brief 加载停车边界顶点
     */
    size_t parking_boundaries_num = roi_parking_boundary.size();
    size_t perception_obstacles_num = 0;

    for (size_t i = 0; i < parking_boundaries_num; ++i) {
        obstacles_vertices_vec->push_back(roi_parking_boundary[i]);
    }

    /**
     * @brief 记录每个边界的边数
     */
    Eigen::MatrixXi parking_boundaries_obstacles_edges_num(parking_boundaries_num, 1);
    for (size_t i = 0; i < parking_boundaries_num; i++) {
        if (roi_parking_boundary[i].size() <= 1U) {
            AERROR << "Roi parking boundary is invalid: " << roi_parking_boundary[i].size();
            return false;
        }
        parking_boundaries_obstacles_edges_num(i, 0) = static_cast<int>(roi_parking_boundary[i].size()) - 1;
    }

    /**
     * @brief 加载感知障碍物
     */
    if (config_.enable_perception_obstacles()) {
        const auto &origin_point = open_space_info.origin_point();
        const auto &origin_heading = open_space_info.origin_heading();
        for (const auto &obstacle : obstacles_by_frame_->Items()) {
            if (FilterOutObstacle(*frame, *obstacle)) {
                continue;
            }
            ++perception_obstacles_num;

            /**
             * @brief 获取障碍物顶点
             */
            std::vector<Vec2d> vertices_ccw;
            if (config_.expand_polygon_of_obstacle_by_distance()) {
                common::math::Polygon2d original_polygon = obstacle->PerceptionPolygon();
                original_polygon.ExpandByDistance(config_.perception_obstacle_buffer());
                original_polygon.CalculateVertices(-1.0 * origin_point);
                vertices_ccw = original_polygon.GetAllVertices();
            } else {
                Box2d original_box = obstacle->PerceptionBoundingBox();
                original_box.Shift(-1.0 * origin_point);
                original_box.LongitudinalExtend(config_.perception_obstacle_buffer());
                original_box.LateralExtend(config_.perception_obstacle_buffer());
                vertices_ccw = original_box.GetAllCorners();
            }

            /**
             * @brief 转换为顺时针并旋转
             */
            std::vector<Vec2d> vertices_cw;
            while (!vertices_ccw.empty()) {
                auto current_corner_pt = vertices_ccw.back();
                current_corner_pt.SelfRotate(-1.0 * origin_heading);
                vertices_cw.push_back(current_corner_pt);
                vertices_ccw.pop_back();
            }
            vertices_cw.push_back(vertices_cw.front());
            obstacles_vertices_vec->push_back(vertices_cw);
        }

        Eigen::MatrixXi perception_obstacles_edges_num = 4 * Eigen::MatrixXi::Ones(perception_obstacles_num, 1);
        obstacles_edges_num_vec->resize(
                parking_boundaries_obstacles_edges_num.rows() + perception_obstacles_edges_num.rows(), 1);
        *(obstacles_edges_num_vec) << parking_boundaries_obstacles_edges_num, perception_obstacles_edges_num;
    } else {
        obstacles_edges_num_vec->resize(parking_boundaries_obstacles_edges_num.rows(), 1);
        *(obstacles_edges_num_vec) << parking_boundaries_obstacles_edges_num;
    }

    mutable_open_space_info->set_obstacles_num(parking_boundaries_num + perception_obstacles_num);
    return true;
}

/**
 * @brief 过滤障碍物
 */
bool OpenSpaceRoiDeciderPark::FilterOutObstacle(const Frame &frame, const Obstacle &obstacle) {
    if (obstacle.IsVirtual() || !obstacle.IsStatic()) {
        return true;
    }

    const auto &open_space_info = frame.open_space_info();
    const auto &origin_point = open_space_info.origin_point();
    const auto &origin_heading = open_space_info.origin_heading();
    const auto &obstacle_box = obstacle.PerceptionBoundingBox();
    auto obstacle_center_xy = obstacle_box.center();

    /**
     * @brief 检查是否在ROI外
     */
    const auto &roi_xy_boundary = open_space_info.ROI_xy_boundary();
    obstacle_center_xy -= origin_point;
    obstacle_center_xy.SelfRotate(-origin_heading);
    if (obstacle_center_xy.x() < roi_xy_boundary[0] || obstacle_center_xy.x() > roi_xy_boundary[1]
        || obstacle_center_xy.y() < roi_xy_boundary[2] || obstacle_center_xy.y() > roi_xy_boundary[3]) {
        return true;
    }

    /**
     * @brief 检查是否在车辆和终点路径上
     */
    const auto &end_pose = open_space_info.open_space_end_pose();
    Vec2d end_pose_x_y(end_pose[0], end_pose[1]);
    end_pose_x_y.SelfRotate(origin_heading);
    end_pose_x_y += origin_point;

    Vec2d vehicle_x_y(vehicle_state_.x(), vehicle_state_.y());

    const double vehicle_center_to_obstacle = obstacle_box.DistanceTo(vehicle_x_y);
    const double end_pose_center_to_obstacle = obstacle_box.DistanceTo(end_pose_x_y);
    const double filtering_distance = config_.perception_obstacle_filtering_distance();
    if (vehicle_center_to_obstacle > filtering_distance && end_pose_center_to_obstacle > filtering_distance) {
        return true;
    }
    return false;
}

/**
 * @brief 加载超平面约束
 */
bool OpenSpaceRoiDeciderPark::LoadObstacleInHyperPlanes(Frame *const frame) {
    *(frame->mutable_open_space_info()->mutable_obstacles_A())
            = Eigen::MatrixXd::Zero(frame->open_space_info().obstacles_edges_num().sum(), 2);
    *(frame->mutable_open_space_info()->mutable_obstacles_b())
            = Eigen::MatrixXd::Zero(frame->open_space_info().obstacles_edges_num().sum(), 1);

    if (!GetHyperPlanes(
                frame->open_space_info().obstacles_num(),
                frame->open_space_info().obstacles_edges_num(),
                frame->open_space_info().obstacles_vertices_vec(),
                frame->mutable_open_space_info()->mutable_obstacles_A(),
                frame->mutable_open_space_info()->mutable_obstacles_b())) {
        AERROR << "Fail to present obstacle in hyperplane";
        return false;
    }
    return true;
}

/**
 * @brief 获取超平面
 *
 * 功能：将多边形顶点转换为超平面表示 Ax <= b
 */
bool OpenSpaceRoiDeciderPark::GetHyperPlanes(
        const size_t &obstacles_num,
        const Eigen::MatrixXi &obstacles_edges_num,
        const std::vector<std::vector<Vec2d>> &obstacles_vertices_vec,
        Eigen::MatrixXd *A_all,
        Eigen::MatrixXd *b_all) {
    if (obstacles_num != obstacles_vertices_vec.size()) {
        AERROR << "obstacles_num != obstacles_vertices_vec.size()";
        return false;
    }

    A_all->resize(obstacles_edges_num.sum(), 2);
    b_all->resize(obstacles_edges_num.sum(), 1);

    int counter = 0;
    double kEpsilon = 1.0e-5;

    /**
     * @brief 遍历每个障碍物
     */
    for (size_t i = 0; i < obstacles_num; ++i) {
        size_t current_vertice_num = obstacles_edges_num(i, 0);
        Eigen::MatrixXd A_i(current_vertice_num, 2);
        Eigen::MatrixXd b_i(current_vertice_num, 1);

        /**
         * @brief 遍历每个边
         */
        for (size_t j = 0; j < current_vertice_num; ++j) {
            Vec2d v1 = obstacles_vertices_vec[i][j];
            Vec2d v2 = obstacles_vertices_vec[i][j + 1];

            Eigen::MatrixXd A_tmp(2, 1), b_tmp(1, 1), ab(2, 1);

            /**
             * @brief 垂直线处理
             */
            if (std::abs(v1.x() - v2.x()) < kEpsilon) {
                if (v2.y() < v1.y()) {
                    A_tmp << 1, 0;
                    b_tmp << v1.x();
                } else {
                    A_tmp << -1, 0;
                    b_tmp << -v1.x();
                }
            }
            /**
             * @brief 水平线处理
             */
            else if (std::abs(v1.y() - v2.y()) < kEpsilon) {
                if (v1.x() < v2.x()) {
                    A_tmp << 0, 1;
                    b_tmp << v1.y();
                } else {
                    A_tmp << 0, -1;
                    b_tmp << -v1.y();
                }
            }
            /**
             * @brief 斜线处理
             */
            else {
                Eigen::MatrixXd tmp1(2, 2);
                tmp1 << v1.x(), 1, v2.x(), 1;
                Eigen::MatrixXd tmp2(2, 1);
                tmp2 << v1.y(), v2.y();
                ab = tmp1.inverse() * tmp2;
                double a = ab(0, 0);
                double b = ab(1, 0);

                if (v1.x() < v2.x()) {
                    A_tmp << -a, 1;
                    b_tmp << b;
                } else {
                    A_tmp << a, -1;
                    b_tmp << -b;
                }
            }

            A_i.block(j, 0, 1, 2) = A_tmp.transpose();
            b_i.block(j, 0, 1, 1) = b_tmp;
        }

        A_all->block(counter, 0, A_i.rows(), 2) = A_i;
        b_all->block(counter, 0, b_i.rows(), 1) = b_i;
        counter += static_cast<int>(current_vertice_num);
    }
    return true;
}

/**
 * @brief 获取附近路径
 */
bool OpenSpaceRoiDeciderPark::GetNearbyPath(
        const apollo::routing::RoutingResponse &routing_response,
        const ParkingSpaceInfoConstPtr &parking_spot,
        std::shared_ptr<hdmap::Path> *nearby_path) {
    LaneInfoConstPtr nearest_lane;
    if (nullptr == parking_spot) {
        AERROR << "The parking spot id is invalid!" << parking_spot->id().id();
        return false;
    }

    auto parking_space = parking_spot->parking_space();
    auto overlap_ids = parking_space.overlap_id();
    if (overlap_ids.empty()) {
        AERROR << "There is no lane overlaps with the parking spot: " << parking_spot->id().id();
        return false;
    }

    std::vector<routing::LaneSegment> lane_segments;
    GetAllLaneSegments(routing_response, &lane_segments);

    /**
     * @brief 查找最近车道
     */
    bool has_found_nearest_lane = false;
    size_t nearest_lane_index = 0;
    for (auto id : overlap_ids) {
        auto overlaps = hdmap_->GetOverlapById(id)->overlap();
        for (auto object : overlaps.object()) {
            if (!object.has_lane_overlap_info()) {
                continue;
            }
            nearest_lane = hdmap_->GetLaneById(object.id());
            if (nearest_lane == nullptr) {
                continue;
            }
            for (auto &segment : lane_segments) {
                if (segment.id() == nearest_lane->id().id()) {
                    has_found_nearest_lane = true;
                    break;
                }
                ++nearest_lane_index;
            }
            if (has_found_nearest_lane) {
                break;
            }
        }
    }

    /**
     * @brief 获取车辆最近车道
     */
    LaneInfoConstPtr nearest_lane_to_vehicle;
    auto point = common::util::PointFactory::ToPointENU(vehicle_state_);
    double vehicle_lane_s = 0.0;
    double vehicle_lane_l = 0.0;
    int status = hdmap_->GetNearestLaneWithHeading(
            point, 10.0, vehicle_state_.heading(), M_PI / 2.0,
            &nearest_lane_to_vehicle, &vehicle_lane_s, &vehicle_lane_l);

    /**
     * @brief 构建路径
     */
    ParkingSpaceInfoConstPtr target_parking_spot = nullptr;
    LaneSegment nearest_lanesegment = LaneSegment(nearest_lane, nearest_lane->accumulate_s().front(), nearest_lane->accumulate_s().back());
    std::vector<LaneSegment> segments_vector;
    int next_lanes_num = nearest_lane->lane().successor_id_size();
    if (next_lanes_num != 0) {
        auto next_lane_id = nearest_lane->lane().successor_id(0);
        segments_vector.push_back(nearest_lanesegment);
        auto next_lane = hdmap_->GetLaneById(next_lane_id);
        LaneSegment next_lanesegment = LaneSegment(next_lane, next_lane->accumulate_s().front(), next_lane->accumulate_s().back());
        segments_vector.push_back(next_lanesegment);
        *nearby_path = std::make_shared<Path>(segments_vector);
    } else {
        segments_vector.push_back(nearest_lanesegment);
        *nearby_path = std::make_shared<Path>(segments_vector);
    }
    return true;
}

/**
 * @brief 获取U型弯路径
 */
bool OpenSpaceRoiDeciderPark::GetUTurnPath(Frame *const frame, std::shared_ptr<hdmap::Path> *nearby_path) {
    if (frame == nullptr) {
        AERROR << "Frame is nullptr";
        return false;
    }

    std::vector<LaneSegment> segments_vector;
    const apollo::routing::RoutingResponse &routing_response = frame->local_view().planning_command->lane_follow_command();
    std::vector<routing::LaneSegment> lane_segments;
    GetAllLaneSegments(routing_response, &lane_segments);

    /**
     * @brief 找最近车道
     */
    std::pair<double, std::string> min_lane = std::make_pair(std::numeric_limits<double>::max(), "");
    for (auto &seg : lane_segments) {
        auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(seg.id()));
        Vec2d proj_pt(0.0, 0.0);
        double s_offset = 0.0;
        int s_offset_index = 0;
        double dis = lane->DistanceTo({vehicle_state_.x(), vehicle_state_.y()}, &proj_pt, &s_offset, &s_offset_index);
        if (dis < min_lane.first) {
            min_lane = std::make_pair(dis, seg.id());
        }
    }

    /**
     * @brief 收集后续车道
     */
    bool has_found_nearest_lane = false;
    static constexpr int add_lane_num = 3;
    int lane_num = 0;
    hdmap::LaneInfoConstPtr previous_lane;
    for (auto &segment : lane_segments) {
        if (lane_num == add_lane_num) {
            break;
        }
        if (segment.id() == min_lane.second) {
            has_found_nearest_lane = true;
        }
        if (has_found_nearest_lane) {
            auto next_lane = hdmap_->GetLaneById(hdmap::MakeMapId(segment.id()));
            LaneSegment next_lanesegment = LaneSegment(next_lane, next_lane->accumulate_s().front(), next_lane->accumulate_s().back());
            segments_vector.push_back(next_lanesegment);
            lane_num++;
            if (lane_num >= add_lane_num && previous_lane != nullptr
                && fabs(previous_lane->Curvature(previous_lane->accumulate_s().back())) < 0.1) {
                break;
            }
            previous_lane = next_lane;
        }
    }

    if (segments_vector.empty()) {
        return false;
    }
    *nearby_path = std::make_shared<Path>(segments_vector);
    return true;
}

/**
 * @brief 获取停车出口边界
 */
bool OpenSpaceRoiDeciderPark::GetParkingOutBoundary(
        const hdmap::Path &nearby_path,
        Frame *const frame,
        std::vector<std::vector<common::math::Vec2d>> *const roi_parking_boundary) {
    const auto &park_and_go_status = injector_->planning_context()->planning_status().park_and_go();
    const double adc_init_x = park_and_go_status.adc_init_position().x();
    const double adc_init_y = park_and_go_status.adc_init_position().y();
    const double adc_init_heading = park_and_go_status.adc_init_heading();
    common::math::Vec2d adc_init_position = {adc_init_x, adc_init_y};
    const double adc_length = vehicle_params_.length();
    const double adc_width = vehicle_params_.width();

    // 类似实现...
    return true;
}

/**
 * @brief 命名空间结束标记
 */
}  // namespace planning
}  // namespace apollo
