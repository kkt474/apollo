# Apollo Planning 模块详细分析

## 一、模块总体架构

Planning 模块采用 **Planner -> Scenario -> Stage -> Task** 四层架构，通过插件机制实现高度解耦。

```
PlanningComponent (CyberRT组件入口)
  └── PlanningBase (规划基类)
        ├── OnLanePlanning (车道线模式 - 主要)
        └── NaviPlanning (导航模式)
              └── Planner (规划器)
                    ├── PublicRoadPlanner (公开道路规划器 - 主力)
                    ├── LatticePlanner (格子规划器)
                    ├── NaviPlanner (导航规划器)
                    └── RTKPlanner (回放规划器)
                          └── ScenarioManager (场景管理器)
                                └── Scenario (场景) x 19种
                                      └── Stage (阶段)
                                            └── Task (任务) x 35种
```

---

## 二、核心数据流

```
输入:
  PredictionObstacles (预测障碍物) ─┐
  Chassis (底盘状态)               ─┤
  LocalizationEstimate (定位)      ─┤─> PlanningComponent::Proc()
  TrafficLightDetection (红绿灯)   ─┤      │
  PadMessage (人工干预)            ─┤      ├── CheckRerouting()
  PlanningCommand (规划命令)       ─┤      ├── CheckInput()
  Stories (场景故事)               ─┤      ├── LocalView 组装
  ControlInteractiveMsg (控制交互) ─┘      │
                                          └── PlanningBase::RunOnce()
                                                │
                                                ▼
                                          OnLanePlanning::RunOnce()
                                                │
                                  ┌─────────────┼─────────────┐
                                  │             │             │
                          TrajectoryStitcher  TrafficDecider  Frame::Init()
                          (轨迹缝合)         (交通规则决策)  (帧初始化)
                                  │             │             │
                                  └─────────────┼─────────────┘
                                                │
                                          Planner::Plan()
                                                │
                                          Scenario::Process()
                                                │
                                          Stage::Process()
                                                │
                                          Task::Execute()
                                                │
                                                ▼
输出:
  ADCTrajectory (自动驾驶轨迹) ──> Control模块
  CommandStatus (命令状态)     ──> 外部系统
```

---

## 三、10个子模块详解

### 3.1 planning_component/ - CyberRT组件层

**核心文件**:
- `planning_component.h` - CyberRT组件，订阅3个主话题 + 6个辅助话题
- `planning_component.cc` - Init()创建Reader/Writer，Proc()每周期调用
- `planning_base.h` - 规划基类，定义RunOnce/Plan接口
- `on_lane_planning.h/.cc` - 基于车道线的规划，主要模式
- `navi_planning.h/.cc` - 导航模式规划

**关键逻辑**:
1. `Init()`: 根据FLAGS_use_navigation_mode选择OnLanePlanning或NaviPlanning
2. `Proc()`: 接收3个主输入(prediction/chassis/localization)，组装LocalView，调用RunOnce
3. 输入检查: localization/chassis/map/planning_command必须就绪
4. 重路由: 检查planning_context中的need_rerouting标志

**输入消息** (PlanningComponent订阅):

| 消息类型 | 来源 | 角色 |
|----------|------|------|
| PredictionObstacles | Prediction模块 | 主输入-障碍物预测 |
| Chassis | CAN总线 | 主输入-底盘状态 |
| LocalizationEstimate | Localization模块 | 主输入-定位 |
| TrafficLightDetection | Perception模块 | 红绿灯检测 |
| PadMessage | 人机交互 | 驾驶员指令 |
| PlanningCommand | 外部系统 | 规划命令 |
| Stories | Storytelling模块 | 场景故事 |
| ControlInteractiveMsg | Control模块 | 控制交互 |
| MapMsg | 相对地图 | 仅导航模式 |

**输出消息** (PlanningComponent发布):

| 消息类型 | 目标 | 角色 |
|----------|------|------|
| ADCTrajectory | Control模块 | 主输出-规划轨迹 |
| CommandStatus | 外部系统 | 命令状态反馈 |
| PlanningLearningData | 学习系统 | 学习数据 |
| RoutingRequest | Routing模块 | 重路由请求 |

### 3.2 planning_base/ - 规划基础设施层

**核心子目录**:

#### common/ (61个文件) - 核心数据结构

| 类名 | 文件 | 说明 |
|------|------|------|
| Frame | common/frame.h | 规划帧，一帧规划的所有数据(参考线/障碍物/车辆状态) |
| ReferenceLineInfo | common/reference_line_info.h | 参考线信息，一条参考线的所有规划数据(路径/速度/决策/代价) |
| Obstacle | common/obstacle.h | 障碍物，包含感知和预测信息 |
| PathDecision | common/path_decision.h | 路径决策，对障碍物的nudge/overtake/stop决策 |
| EgoInfo | common/ego_info.h | 自车信息(位置/速度/边界框) |
| LocalView | common/local_view.h | 输入数据结构(9个shared_ptr) |
| DependencyInjector | common/dependency_injector.h | 依赖注入器(PlanningContext/FrameHistory/History/EgoInfo/VehicleState/LearningData) |
| PlanningContext | common/planning_context.h | 跨帧规划状态(重路由/借道/换道等) |
| TrajectoryStitcher | common/trajectory_stitcher.h | 轨迹缝合器，连接历史轨迹与新规划 |
| SpeedLimit | common/speed_limit.h | 速度限制曲线 |
| StGraphData | common/st_graph_data.h | ST图数据 |
| DecisionData | common/decision_data.h | 决策数据 |
| SLPolygon | common/sl_polygon.h | SL多边形，障碍物在SL坐标系的表示 |

#### common/path/ - 路径数据

- DiscretizedPath - 离散化路径
- FrenetFramePath - Frenet坐标系路径
- PathData - 路径数据容器
- PathBoundary - 路径边界

#### common/speed/ - 速度数据

- SpeedData - 速度数据序列
- STBoundary - ST图边界
- STPoint - ST图点(s, t)

#### common/trajectory/ - 轨迹数据

- DiscretizedTrajectory - 离散化轨迹
- PublishableTrajectory - 可发布轨迹

#### common/trajectory1d/ - 一维轨迹

- ConstantDecelerationTrajectory1d - 恒减速一维轨迹
- ConstantJerkTrajectory1d - 恒Jerk一维轨迹
- PiecewiseAccelerationTrajectory1d - 分段加速度一维轨迹
- PiecewiseJerkTrajectory1d - 分段Jerk一维轨迹
- PiecewiseTrajectory1d - 分段一维轨迹
- StandingStillTrajectory1d - 静止一维轨迹

#### reference_line/ (20个文件) - 参考线生成与平滑

| 类名 | 说明 |
|------|------|
| ReferenceLineProvider | 参考线提供者(从路由获取参考线) |
| ReferenceLine | 参考线类(车道中心线+平滑) |
| DiscretePointsReferenceLineSmoother | 离散点参考线平滑器 |
| QpSplineReferenceLineSmoother | QP样条参考线平滑器 |
| SpiralReferenceLineSmoother | 螺旋线参考线平滑器 |

#### 其他子目录

| 目录 | 说明 |
|------|------|
| proto/ | 14个Protobuf定义(PlanningConfig, PlanningStatus, LearningData等) |
| math/ | 数学工具(constraint_checker, curve1d, piecewise_jerk, smoothing_spline) |
| gflags/ | 规划配置标志(planning_gflags.h 10.2KB, planning_gflags.cc 25.1KB) |
| learning_based/ | 学习模式数据(鸟瞰图特征渲染) |
| tools/ | 规划工具(13个文件) |

### 3.3 planners/ - 规划器层

**4种规划器**:

| 规划器 | 路径 | 基类 | 说明 |
|--------|------|------|------|
| PublicRoadPlanner | planners/public_road/ | PlannerWithReferenceLine | 主力规划器，基于Scenario架构 |
| LatticePlanner | planners/lattice/ | PlannerWithReferenceLine | 格子规划器，采样+评估 |
| NaviPlanner | planners/navi/ | PlannerWithReferenceLine | 导航规划器 |
| RTKPlanner | planners/rtk/ | Planner | 录制回放规划器 |

**PublicRoadPlanner核心**:
- `public_road_planner.h/.cc` - 委托给ScenarioManager和Scenario
- `scenario_manager.h/.cc` - 管理场景切换(遍历scenario_list_，检查IsTransferable)

**场景切换逻辑**:
```
ScenarioManager::Update():
  1. 如果当前场景STATUS_PROCESSING -> 不切换
  2. 遍历所有场景，检查IsTransferable()
  3. 如果可切换: Exit(当前) -> Reset(新) -> Enter(新)
  4. 默认场景: LANE_FOLLOW
```

### 3.4 scenarios/ - 场景层（19种场景）

| 场景 | 说明 | 阶段数 |
|------|------|--------|
| **lane_follow** | 车道跟随（默认场景） | 1 |
| bare_intersection_unprotected | 无保护交叉口 | 2 |
| emergency_pull_over | 紧急靠边 | 3 |
| emergency_stop | 紧急停车 | 2 |
| free_space | 自由空间(泊车) | 1 |
| lane_escape | 车道逃离 | 1 |
| lane_follow_park | 车道跟随泊车 | 1 |
| large_curvature | 大曲率 | 1 |
| park_and_go | 靠边停车后起步 | 4 |
| precise_parking | 精确泊车 | 3 |
| **pull_over** | 靠边停车 | 3 |
| square | 方形区域泊车 | 2 |
| **stop_sign_unprotected** | 无保护停车标志 | 4 |
| **traffic_light_protected** | 有保护红绿灯 | 2 |
| traffic_light_unprotected_left_turn | 无保护左转 | 3 |
| traffic_light_unprotected_right_turn | 无保护右转 | 3 |
| **valet_parking** | 代客泊车 | 2 |
| valet_parking_park | 代客泊车-泊入 | 3 |
| yield_sign | 让行标志 | 2 |

**每个场景结构**:
```
scenario_name/
  ├── scenario_name_scenario.h/.cc   (场景类)
  ├── scenario_name_stage_*.h/.cc    (阶段类，多个)
  ├── conf/                          (配置文件: pipeline.pb.txt, scenario_conf.pb.txt)
  ├── plugins.xml                    (插件注册)
  └── BUILD                          (编译规则)
```

### 3.5 tasks/ - 任务层（35种任务）

#### 路径规划类

| 任务 | 路径 | 说明 |
|------|------|------|
| LaneFollowPath | tasks/lane_follow_path | 车道跟随路径规划 |
| LaneBorrowPath | tasks/lane_borrow_path | 借道路径规划 |
| LaneChangePath | tasks/lane_change_path | 换道路径规划 |
| LaneBorrowPathGeneric | tasks/lane_borrow_path_generic | 通用借道路径 |
| LaneChangePathGeneric | tasks/lane_change_path_generic | 通用换道路径 |
| PullOverPath | tasks/pull_over_path | 靠边停车路径 |
| FallbackPath | tasks/fallback_path | 后备路径 |
| ReusePath | tasks/reuse_path | 路径复用 |
| ReversePath | tasks/reverse_path | 倒车路径 |
| SquarePath | tasks/square_path | 方形路径 |

#### 速度规划类

| 任务 | 路径 | 说明 |
|------|------|------|
| SpeedBoundsDecider | tasks/speed_bounds_decider | 速度边界决策(先验+最终两轮) |
| STBoundsDecider | tasks/st_bounds_decider | ST边界决策 |
| PathTimeHeuristicOptimizer | tasks/path_time_heuristic | 路径时间启发式优化(DP) |
| PiecewiseJerkSpeedOptimizer | tasks/piecewise_jerk_speed | 分段Jerk速度优化(QP) |
| PiecewiseJerkSpeedNonlinearOptimizer | tasks/piecewise_jerk_speed_nonlinear | 非线性分段Jerk速度优化 |
| SpeedDecider | tasks/speed_decider | 速度决策(跟车/超车/停车) |
| ReverseSpeed | tasks/reverse_speed | 倒车速度规划 |

#### 决策类

| 任务 | 路径 | 说明 |
|------|------|------|
| ObstacleNudgeDecider | tasks/obstacle_nudge_decider | 障碍物微调决策 |
| PathDecider | tasks/path_decider | 路径决策(忽略/绕行/停止) |
| PathReferenceDecider | tasks/path_reference_decider | 路径参考决策 |
| RuleBasedStopDecider | tasks/rule_based_stop_decider | 基于规则停车决策 |
| RssDecider | tasks/rss_decider | RSS安全决策 |
| OpenSpaceFallbackDecider | tasks/open_space_fallback_decider | 开放空间后备决策 |
| OpenSpaceReplanDecider | tasks/open_space_replan_decider | 开放空间重规划决策 |

#### 开放空间类

| 任务 | 路径 | 说明 |
|------|------|------|
| OpenSpaceRoiDecider | tasks/open_space_roi_decider | ROI区域决策 |
| OpenSpaceRoiDeciderPark | tasks/open_space_roi_decider_park | 泊车ROI区域决策 |
| OpenSpacePreStopDecider | tasks/open_space_pre_stop_decider | 开放空间预停车决策 |
| OpenSpacePathPlanning | tasks/open_space_path_planning | 开放空间路径规划(HybridA*) |
| OpenSpaceTrajectoryProvider | tasks/open_space_trajectory_provider | 开放空间轨迹提供 |
| OpenSpaceTrajectoryOptimizerPark | tasks/open_space_trajectory_optimizer_park | 泊车开放空间轨迹优化 |
| OpenSpaceTrajectoryPartition | tasks/open_space_trajectory_partition | 开放空间轨迹分割(前/后/换挡) |
| OpenSpaceTrajectoryPostProcess | tasks/open_space_trajectory_post_process | 开放空间轨迹后处理 |
| OpenSpaceFallbackDeciderPark | tasks/open_space_fallback_decider_park | 泊车开放空间后备决策 |

#### 后备类

| 任务 | 路径 | 说明 |
|------|------|------|
| FastStopTrajectoryFallback | tasks/fast_stop_trajectory_fallback | 快速停车轨迹后备 |
| SmoothStopTrajectoryFallback | tasks/smooth_stop_trajectory_fallback | 平滑停车轨迹后备 |

### 3.6 traffic_rules/ - 交通规则层（10种规则）

| 规则 | 说明 |
|------|------|
| backside_vehicle | 后方车辆处理 |
| crosswalk | 人行横道让行 |
| destination | 目的地停车 |
| keepclear | 禁停区处理 |
| reference_line_end | 参考线终点处理 |
| rerouting | 重路由触发 |
| speed_setting | 速度设置 |
| stop_sign | 停车标志处理 |
| traffic_light | 红绿灯处理 |
| yield_sign | 让行标志处理 |

**执行流程**: `TrafficDecider::Execute()` 按配置顺序执行所有TrafficRule

### 3.7 planning_interface_base/ - 接口基类层

| 基类 | 文件 | 关键方法 |
|------|------|----------|
| Planner | planner_base/planner.h | Init(), Plan(), Stop() |
| PlannerWithReferenceLine | planner_base/planner.h | PlanOnReferenceLine() |
| Scenario | scenario_base/scenario.h | Init(), Process(), IsTransferable(), Enter(), Exit(), Reset() |
| Stage | scenario_base/stage.h | Init(), Process(), ExecuteTaskOnReferenceLine(), ExecuteTaskOnOpenSpace() |
| Task | task_base/task.h | Init(), Execute() |
| TrafficDecider | traffic_rules_base/traffic_decider.h | Init(), Execute() |
| TrafficRule | traffic_rules_base/traffic_rule.h | Init(), ApplyRule() |

### 3.8 planning_open_space/ - 开放空间规划

| 子目录 | 说明 |
|--------|------|
| coarse_trajectory_generator/ | 粗轨迹生成(HybridA*) |
| trajectory_smoother/ | 轨迹平滑(35个文件) |
| utils/ | 工具函数 |
| proto/ | 配置定义 |

### 3.9 pnc_map/ - 路径导航地图

- `lane_follow_map/` - 车道跟随地图(8个文件)

### 3.10 park_data_center/ - 泊车数据中心

- `ParkDataCenter` - 泊车数据管理
- `NudgeInfo` - 微调信息

---

## 四、核心类关系图

```
PlanningComponent
  ├── LocalView (输入数据聚合, 9个shared_ptr)
  ├── DependencyInjector (依赖注入)
  │     ├── PlanningContext (跨帧规划状态)
  │     ├── FrameHistory (历史帧记录)
  │     ├── History (历史轨迹记录)
  │     ├── EgoInfo (自车信息)
  │     ├── VehicleStateProvider (车辆状态提供者)
  │     └── LearningBasedData (学习数据)
  ├── PlanningBase
  │     ├── TrafficDecider
  │     │     └── TrafficRule[] (10种规则)
  │     ├── Frame (规划帧)
  │     │     ├── ReferenceLineInfo[] (参考线信息)
  │     │     │     ├── ReferenceLine (参考线)
  │     │     │     ├── PathData (路径数据)
  │     │     │     ├── SpeedData (速度数据)
  │     │     │     ├── PathDecision (路径决策)
  │     │     │     ├── StGraphData (ST图数据)
  │     │     │     └── DiscretizedTrajectory (离散轨迹)
  │     │     └── Obstacle[] (障碍物)
  │     └── Planner
  │           └── ScenarioManager
  │                 └── Scenario (当前场景)
  │                       └── Stage (当前阶段)
  │                             └── Task[] (任务列表)
  └── ADCTrajectory (输出轨迹)
```

---

## 五、类继承关系

```
cyber::Component<PredictionObstacles, Chassis, LocalizationEstimate>
  └── PlanningComponent

PlanningBase (基类)
  ├── OnLanePlanning
  └── NaviPlanning

Planner (基类)
  └── PlannerWithReferenceLine
        ├── PublicRoadPlanner
        ├── LatticePlanner
        └── NaviPlanner

Scenario (基类)
  ├── LaneFollowScenario
  ├── ValetParkingScenario
  ├── PullOverScenario
  ├── TrafficLightProtectedScenario
  ├── StopSignUnprotectedScenario
  └── ... (19种)

Stage (基类)
  ├── LaneFollowStage
  ├── BaseStageCreep (蠕行阶段基类)
  ├── BaseStageCruise (巡航阶段基类)
  └── 各场景特定Stage

Task (基类)
  ├── 路径规划类Task (LaneFollowPath, LaneBorrowPath, ...)
  ├── 速度规划类Task (SpeedBoundsDecider, SpeedDecider, ...)
  ├── 决策类Task (PathDecider, RssDecider, ...)
  └── 开放空间类Task (OpenSpaceRoiDecider, ...)

TrafficRule (基类)
  ├── Crosswalk
  ├── TrafficLight
  ├── StopSign
  └── ... (10种)
```

---

## 六、场景-阶段-任务完整映射

### lane_follow (车道跟随, 默认场景)

| 阶段 | 类型 | 任务列表 |
|------|------|----------|
| LANE_FOLLOW_STAGE | LaneFollowStage | LaneChangePath, LaneFollowPath, LaneBorrowPath, FallbackPath, PathDecider, RuleBasedStopDecider, SpeedBoundsDecider(priori), PathTimeHeuristicOptimizer, SpeedDecider, SpeedBoundsDecider(final), PiecewiseJerkSpeedOptimizer |

### bare_intersection_unprotected (无保护交叉口)

| 阶段 | 类型 | 任务列表 |
|------|------|----------|
| BARE_INTERSECTION_UNPROTECTED_APPROACH | BareIntersectionUnprotectedStageApproach | LaneFollowPath, LaneBorrowPath, FallbackPath, PathDecider, RuleBasedStopDecider, SpeedBoundsDecider(priori), PathTimeHeuristicOptimizer, SpeedDecider, SpeedBoundsDecider(final), PiecewiseJerkSpeedOptimizer |
| BARE_INTERSECTION_UNPROTECTED_INTERSECTION_CRUISE | BareIntersectionUnprotectedStageIntersectionCruise | LaneFollowPath, LaneBorrowPath, FallbackPath, PathDecider, RuleBasedStopDecider, SpeedBoundsDecider(priori), PathTimeHeuristicOptimizer, SpeedDecider, SpeedBoundsDecider(final), PiecewiseJerkSpeedOptimizer |

### emergency_pull_over (紧急靠边)

| 阶段 | 类型 | 任务列表 |
|------|------|----------|
| EMERGENCY_PULL_OVER_SLOW_DOWN | EmergencyPullOverStageSlowDown | LaneFollowPath, LaneBorrowPath, FallbackPath, PathDecider, RuleBasedStopDecider, SpeedBoundsDecider(priori), PathTimeHeuristicOptimizer, SpeedDecider, SpeedBoundsDecider(final), PiecewiseJerkSpeedOptimizer |
| EMERGENCY_PULL_OVER_APPROACH | EmergencyPullOverStageApproach | **PullOverPath**, LaneFollowPath, FallbackPath, PathDecider, RuleBasedStopDecider, SpeedBoundsDecider(priori), PathTimeHeuristicOptimizer, SpeedDecider, SpeedBoundsDecider(final), PiecewiseJerkSpeedOptimizer |
| EMERGENCY_PULL_OVER_STANDBY | EmergencyPullOverStageStandby | LaneFollowPath, LaneBorrowPath, FallbackPath, PathDecider, RuleBasedStopDecider, SpeedBoundsDecider(priori), PathTimeHeuristicOptimizer, SpeedDecider, SpeedBoundsDecider(final), PiecewiseJerkSpeedOptimizer |

### emergency_stop (紧急停车)

| 阶段 | 类型 | 任务列表 |
|------|------|----------|
| EMERGENCY_STOP_APPROACH | EmergencyStopStageApproach | LaneFollowPath, FallbackPath, PathDecider, RuleBasedStopDecider, SpeedBoundsDecider(priori), PathTimeHeuristicOptimizer, SpeedDecider, SpeedBoundsDecider(final), PiecewiseJerkSpeedOptimizer |
| EMERGENCY_STOP_STANDBY | EmergencyStopStageStandby | LaneFollowPath, LaneBorrowPath, FallbackPath, PathDecider, RuleBasedStopDecider, SpeedBoundsDecider(priori), PathTimeHeuristicOptimizer, SpeedDecider, SpeedBoundsDecider(final), PiecewiseJerkSpeedOptimizer |

### free_space (自由空间/泊车)

| 阶段 | 类型 | 任务列表 |
|------|------|----------|
| STAGE_FREE_SPACE | StageFreeSpace | OpenSpaceTrajectoryProvider, OpenSpaceTrajectoryPartition, OpenSpaceFallbackDecider |

### large_curvature (大曲率)

| 阶段 | 类型 | 任务列表 |
|------|------|----------|
| LARGE_CURVATURE | StageLargeCurvature | OpenSpaceReplanDecider, OpenSpaceRoiDeciderPark, OpenSpacePathPlanning, OpenSpaceTrajectoryOptimizerPark, OpenSpaceTrajectoryPostProcess, OpenSpaceFallbackDeciderPark |

### park_and_go (靠边停车后起步)

| 阶段 | 类型 | 任务列表 |
|------|------|----------|
| PARK_AND_GO_CHECK | ParkAndGoStageCheck | OpenSpaceRoiDecider, OpenSpaceTrajectoryProvider, OpenSpaceTrajectoryPartition, OpenSpaceFallbackDecider |
| PARK_AND_GO_ADJUST | ParkAndGoStageAdjust | OpenSpaceRoiDecider, OpenSpaceTrajectoryProvider, OpenSpaceTrajectoryPartition, OpenSpaceFallbackDecider |
| PARK_AND_GO_PRE_CRUISE | ParkAndGoStagePreCruise | OpenSpaceRoiDecider, OpenSpaceTrajectoryProvider, OpenSpaceTrajectoryPartition, OpenSpaceFallbackDecider |
| PARK_AND_GO_CRUISE | ParkAndGoStageCruise | LaneFollowPath, LaneBorrowPath, FallbackPath, PathDecider, RuleBasedStopDecider, **STBoundsDecider**, SpeedBoundsDecider(priori), PathTimeHeuristicOptimizer, SpeedDecider, SpeedBoundsDecider(final), PiecewiseJerkSpeedOptimizer, **RssDecider** |

### pull_over (靠边停车)

| 阶段 | 类型 | 任务列表 |
|------|------|----------|
| PULL_OVER_APPROACH | PullOverStageApproach | **PullOverPath**, LaneFollowPath, FallbackPath, PathDecider, RuleBasedStopDecider, SpeedBoundsDecider(priori), PathTimeHeuristicOptimizer, SpeedDecider, SpeedBoundsDecider(final), PiecewiseJerkSpeedOptimizer, **RssDecider** |
| PULL_OVER_RETRY_APPROACH_PARKING (disabled) | PullOverStageRetryApproachParking | OpenSpacePreStopDecider, PullOverPath, LaneFollowPath, PathDecider, RuleBasedStopDecider, **STBoundsDecider**, SpeedBoundsDecider(priori), PathTimeHeuristicOptimizer, SpeedDecider, SpeedBoundsDecider(final), PiecewiseJerkSpeedOptimizer |
| PULL_OVER_RETRY_PARKING | PullOverStageRetryParking | OpenSpaceRoiDecider, OpenSpaceTrajectoryProvider, OpenSpaceTrajectoryPartition, OpenSpaceFallbackDecider |

### square (方形区域泊车)

| 阶段 | 类型 | 任务列表 |
|------|------|----------|
| SQUARE_LANE_FOLLOW_STAGE | SquareLaneFollowStage | **ObstacleNudgeDecider**, **SquarePath**, **LaneBorrowPathGeneric**, FallbackPath, RuleBasedStopDecider, SpeedBoundsDecider(priori), PathTimeHeuristicOptimizer, SpeedDecider, SpeedBoundsDecider(final), PiecewiseJerkSpeedOptimizer |
| EXTRICATE_STAGE | ExtricateStage | **ObstacleNudgeDecider**, LaneFollowPath, **LaneBorrowPathGeneric**, **ReversePath**, **ReverseSpeed**, PiecewiseJerkSpeedOptimizer |

### stop_sign_unprotected (无保护停车标志)

| 阶段 | 类型 | 任务列表 |
|------|------|----------|
| STOP_SIGN_UNPROTECTED_PRE_STOP | StopSignUnprotectedStagePreStop | LaneFollowPath, LaneBorrowPath, FallbackPath, PathDecider, RuleBasedStopDecider, SpeedBoundsDecider(priori), PathTimeHeuristicOptimizer, SpeedDecider, SpeedBoundsDecider(final), PiecewiseJerkSpeedOptimizer |
| STOP_SIGN_UNPROTECTED_STOP | StopSignUnprotectedStageStop | LaneFollowPath, LaneBorrowPath, FallbackPath, PathDecider, RuleBasedStopDecider, SpeedBoundsDecider(priori), PathTimeHeuristicOptimizer, SpeedDecider, SpeedBoundsDecider(final), PiecewiseJerkSpeedOptimizer |
| STOP_SIGN_UNPROTECTED_CREEP | StopSignUnprotectedStageCreep | LaneFollowPath, LaneBorrowPath, FallbackPath, PathDecider, RuleBasedStopDecider, SpeedBoundsDecider(priori), PathTimeHeuristicOptimizer, SpeedDecider, SpeedBoundsDecider(final), PiecewiseJerkSpeedOptimizer |
| STOP_SIGN_UNPROTECTED_INTERSECTION_CRUISE | StopSignUnprotectedStageIntersectionCruise | LaneFollowPath, LaneBorrowPath, FallbackPath, PathDecider, RuleBasedStopDecider, SpeedBoundsDecider(priori), PathTimeHeuristicOptimizer, SpeedDecider, SpeedBoundsDecider(final), PiecewiseJerkSpeedOptimizer |

### traffic_light_protected (有保护红绿灯)

| 阶段 | 类型 | 任务列表 |
|------|------|----------|
| TRAFFIC_LIGHT_PROTECTED_APPROACH | TrafficLightProtectedStageApproach | LaneFollowPath, LaneBorrowPath, FallbackPath, PathDecider, RuleBasedStopDecider, **STBoundsDecider**, SpeedBoundsDecider(priori), PathTimeHeuristicOptimizer, SpeedDecider, SpeedBoundsDecider(final), PiecewiseJerkSpeedOptimizer |
| TRAFFIC_LIGHT_PROTECTED_INTERSECTION_CRUISE | TrafficLightProtectedStageIntersectionCruise | LaneFollowPath, LaneBorrowPath, FallbackPath, PathDecider, RuleBasedStopDecider, **STBoundsDecider**, SpeedBoundsDecider(priori), PathTimeHeuristicOptimizer, SpeedDecider, SpeedBoundsDecider(final), PiecewiseJerkSpeedOptimizer |

### traffic_light_unprotected_left_turn (无保护红绿灯左转)

| 阶段 | 类型 | 任务列表 |
|------|------|----------|
| TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_APPROACH | TrafficLightUnprotectedLeftTurnStageApproach | LaneFollowPath, LaneBorrowPath, FallbackPath, PathDecider, RuleBasedStopDecider, **STBoundsDecider**, SpeedBoundsDecider(priori), PathTimeHeuristicOptimizer, SpeedDecider, SpeedBoundsDecider(final), PiecewiseJerkSpeedOptimizer |
| TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_CREEP | TrafficLightUnprotectedLeftTurnStageCreep | LaneFollowPath, LaneBorrowPath, FallbackPath, PathDecider, RuleBasedStopDecider, **STBoundsDecider**, SpeedBoundsDecider(priori), PathTimeHeuristicOptimizer, SpeedDecider, SpeedBoundsDecider(final), PiecewiseJerkSpeedOptimizer |
| TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_INTERSECTION_CRUISE | TrafficLightUnprotectedLeftTurnStageIntersectionCruise | LaneFollowPath, LaneBorrowPath, FallbackPath, PathDecider, RuleBasedStopDecider, **STBoundsDecider**, SpeedBoundsDecider(priori), PathTimeHeuristicOptimizer, SpeedDecider, SpeedBoundsDecider(final), PiecewiseJerkSpeedOptimizer |

### traffic_light_unprotected_right_turn (无保护红绿灯右转)

| 阶段 | 类型 | 任务列表 |
|------|------|----------|
| TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_STOP | TrafficLightUnprotectedRightTurnStageStop | LaneFollowPath, LaneBorrowPath, FallbackPath, PathDecider, RuleBasedStopDecider, **STBoundsDecider**, SpeedBoundsDecider(priori), PathTimeHeuristicOptimizer, SpeedDecider, SpeedBoundsDecider(final), PiecewiseJerkSpeedOptimizer |
| TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_CREEP | TrafficLightUnprotectedRightTurnStageCreep | LaneFollowPath, LaneBorrowPath, FallbackPath, PathDecider, RuleBasedStopDecider, **STBoundsDecider**, SpeedBoundsDecider(priori), PathTimeHeuristicOptimizer, SpeedDecider, SpeedBoundsDecider(final), PiecewiseJerkSpeedOptimizer |
| TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_INTERSECTION_CRUISE | TrafficLightUnprotectedRightTurnStageIntersectionCruise | LaneFollowPath, LaneBorrowPath, FallbackPath, PathDecider, RuleBasedStopDecider, **STBoundsDecider**, SpeedBoundsDecider(priori), PathTimeHeuristicOptimizer, SpeedDecider, SpeedBoundsDecider(final), PiecewiseJerkSpeedOptimizer |

### valet_parking (代客泊车)

| 阶段 | 类型 | 任务列表 |
|------|------|----------|
| VALET_PARKING_APPROACHING_PARKING_SPOT | StageApproachingParkingSpot | **OpenSpacePreStopDecider**, LaneFollowPath, LaneBorrowPath, FallbackPath, PathDecider, RuleBasedStopDecider, SpeedBoundsDecider(priori), PathTimeHeuristicOptimizer, SpeedDecider, SpeedBoundsDecider(final), PiecewiseJerkSpeedOptimizer |
| VALET_PARKING_PARKING | StageParking | **OpenSpaceRoiDecider**, **OpenSpaceTrajectoryProvider**, **OpenSpaceTrajectoryPartition**, **OpenSpaceFallbackDecider** |

### valet_parking_park (代客泊车-泊入)

| 阶段 | 类型 | 任务列表 |
|------|------|----------|
| VALET_PARKING_APPROACHING_PARKING_SPOT_PARK | StageApproachingParkingSpotPark | OpenSpacePreStopDecider, **ObstacleNudgeDecider**, **LaneChangePathGeneric**, LaneFollowPath, **LaneBorrowPathGeneric**, FallbackPath, PathDecider, RuleBasedStopDecider, SpeedBoundsDecider(priori), PathTimeHeuristicOptimizer, SpeedDecider, SpeedBoundsDecider(final), PiecewiseJerkSpeedOptimizer (fallback: **SmoothStopTrajectoryFallback**) |
| VALET_PARKING_PARKING_PARK | StageParkingPark | **OpenSpaceReplanDecider**, **OpenSpaceRoiDecider**, **OpenSpacePathPlanning**, **OpenSpaceTrajectoryOptimizerPark**, **OpenSpaceTrajectoryPostProcess**, **OpenSpaceFallbackDeciderPark** |
| VALET_PARKING_RETRY_PARK | StageParkingRetryPark | **OpenSpaceReplanDecider**, **OpenSpaceRoiDecider**, **OpenSpacePathPlanning**, **OpenSpaceTrajectoryOptimizerPark**, **OpenSpaceTrajectoryPostProcess**, **OpenSpaceFallbackDeciderPark** |

### yield_sign (让行标志)

| 阶段 | 类型 | 任务列表 |
|------|------|----------|
| YIELD_SIGN_APPROACH | YieldSignStageApproach | LaneFollowPath, LaneBorrowPath, FallbackPath, PathDecider, RuleBasedStopDecider, SpeedBoundsDecider(priori), PathTimeHeuristicOptimizer, SpeedDecider, SpeedBoundsDecider(final), PiecewiseJerkSpeedOptimizer |
| YIELD_SIGN_CREEP | YieldSignStageCreep | LaneFollowPath, LaneBorrowPath, FallbackPath, PathDecider, RuleBasedStopDecider, SpeedBoundsDecider(priori), PathTimeHeuristicOptimizer, SpeedDecider, SpeedBoundsDecider(final), PiecewiseJerkSpeedOptimizer |

---

## 七、一次完整规划周期流程

```
Step 1: PlanningComponent::Proc()
  ├── 组装LocalView (9个输入)
  ├── CheckRerouting() (检查重路由标志)
  ├── CheckInput() (检查数据就绪: localization/chassis/map/planning_command)
  ├── 学习模式数据处理 (如果启用)
  └── 调用 PlanningBase::RunOnce()

Step 2: OnLanePlanning::RunOnce()
  ├── 更新ReferenceLineProvider (从路由获取参考线)
  ├── TrajectoryStitcher::Stitch() (缝合历史轨迹)
  ├── InitFrame() (初始化规划帧)
  │     ├── 获取参考线列表
  │     ├── 裁剪参考线 (FLAGS_look_backward_distance + LookForwardDistance)
  │     ├── Frame::Init() (初始化障碍物/红绿灯/PadMsg等)
  │     └── TrafficDecider::Execute() (按顺序执行10种交通规则)
  └── Planner::Plan() (执行规划)

Step 3: PublicRoadPlanner::Plan()
  ├── ScenarioManager::Update() (场景切换判断)
  │     ├── 当前场景STATUS_PROCESSING -> 不切换
  │     ├── 遍历scenario_list_检查IsTransferable()
  │     └── 可切换: Exit(当前) -> Reset(新) -> Enter(新)
  └── Scenario::Process() (场景处理)

Step 4: Stage::Process() -> ExecuteTaskOnReferenceLine()
  │
  ├── 路径规划阶段:
  │     1. LaneChangePath / LaneFollowPath / LaneBorrowPath (候选路径生成)
  │     2. FallbackPath (后备路径)
  │     3. PathDecider (路径决策: 忽略/绕行/停止)
  │     4. RuleBasedStopDecider (基于规则停车决策)
  │
  ├── 速度规划阶段:
  │     5. SpeedBoundsDecider(priori) (先验速度边界)
  │     6. PathTimeHeuristicOptimizer (DP速度优化)
  │     7. SpeedDecider (速度决策: 跟车/超车/停车)
  │     8. SpeedBoundsDecider(final) (最终速度边界)
  │     9. PiecewiseJerkSpeedOptimizer (QP速度优化)
  │
  └── 合并阶段:
        10. CombinePathAndSpeedProfile (合并路径和速度为轨迹)

Step 5: 输出处理
  ├── CombinePathAndSpeedProfile() (合并路径和速度曲线)
  ├── FillPlanningPb() (填充轨迹消息头部/时间戳)
  ├── SetLocation() (设置位置和车道边界)
  ├── 修正轨迹点relative_time
  ├── planning_writer_->Write() (发布ADCTrajectory)
  ├── 发布CommandStatus (RUNNING/FINISHED/ERROR)
  └── History::Add() (记录到历史)
```

---

## 八、路径-速度分离规划算法

### 8.1 路径规划阶段 (Frenet坐标系横向优化)

```
1. 生成候选PathBoundary
   ├── LaneFollowPath: 车道内路径边界
   ├── LaneBorrowPath: 借道路径边界
   ├── LaneChangePath: 换道路径边界
   └── PullOverPath: 靠边停车路径边界

2. 在PathBoundary内优化路径 (Frenet l坐标)
   ├── 使用二次规划(QP)或非线性优化
   └── 代价函数: 边界偏离 + 平滑性 + 障碍物距离

3. 选择最优路径 (基于代价函数排序)

4. PathDecider对障碍物做决策
   ├── IGNORE: 忽略障碍物
   ├── NUDGE: 微调绕行
   └── STOP: 停车
```

### 8.2 速度规划阶段 (ST图纵向优化)

```
1. SpeedBoundsDecider(priori) - 计算先验ST图边界
   ├── 根据障碍物ST边界确定可行驶区域
   └── 考虑曲率限速等约束

2. PathTimeHeuristicOptimizer - DP求粗略速度曲线
   ├── 在ST图上使用动态规划搜索
   └── 代价函数: 加速度 + Jerk + 偏离参考速度

3. SpeedDecider - 速度决策
   ├── FOLLOW: 跟车
   ├── OVERTAKE: 超车
   └── STOP: 停车

4. SpeedBoundsDecider(final) - 用决策结果重新计算ST边界
   └── 根据超车/跟车决策调整可行驶区域

5. PiecewiseJerkSpeedOptimizer - QP精细优化速度曲线
   ├── 在最终ST边界内用二次规划优化
   └── 代价函数: 偏离DP结果 + 平滑性 + 边界约束
```

### 8.3 合并阶段

```
CombinePathAndSpeedProfile:
  ├── 每个路径点(s, l) + 速度点(s, t)
  └── 插值计算轨迹点(x, y, v, a, relative_time, theta, kappa)
```

---

## 九、任务使用频率统计

### 9.1 高频任务 (大部分OnLane场景使用)

| 任务 | 使用次数 | 说明 |
|------|----------|------|
| LaneFollowPath | 16 | 几乎所有OnLane场景 |
| FallbackPath | 15 | 几乎所有OnLane场景 |
| PathDecider | 14 | 大部分OnLane场景 |
| RuleBasedStopDecider | 14 | 大部分OnLane场景 |
| SpeedBoundsDecider | 16 | 所有OnLane场景(priori+final两轮) |
| PathTimeHeuristicOptimizer | 16 | 所有OnLane场景 |
| SpeedDecider | 16 | 所有OnLane场景 |
| PiecewiseJerkSpeedOptimizer | 16 | 所有OnLane场景 |
| LaneBorrowPath | 13 | 大部分OnLane场景 |
| LaneBorrowPath | 13 | 大部分OnLane场景 |

### 9.2 专用任务 (仅特定场景使用)

| 任务 | 使用次数 | 使用场景 |
|------|----------|----------|
| PullOverPath | 3 | pull_over, emergency_pull_over |
| RssDecider | 2 | pull_over, park_and_go |
| STBoundsDecider | 5 | traffic_light_*, park_and_go, pull_over(retry) |
| SquarePath | 1 | square |
| ReversePath | 1 | square |
| ReverseSpeed | 1 | square |
| ObstacleNudgeDecider | 3 | square, valet_parking_park |
| LaneBorrowPathGeneric | 3 | square, valet_parking_park |
| LaneChangePathGeneric | 1 | valet_parking_park |
| OpenSpacePathPlanning | 3 | large_curvature, valet_parking_park |
| OpenSpaceTrajectoryOptimizerPark | 3 | large_curvature, valet_parking_park |
| OpenSpaceReplanDecider | 3 | large_curvature, valet_parking_park |
| OpenSpaceFallbackDeciderPark | 3 | large_curvature, valet_parking_park |
| SmoothStopTrajectoryFallback | 1 | valet_parking_park (fallback_task) |

---

## 十、文件统计

| 子模块 | 文件数 | 核心类 |
|--------|--------|--------|
| planning_component | 16 | PlanningComponent, OnLanePlanning, NaviPlanning |
| planning_base/common | 61 | Frame, ReferenceLineInfo, Obstacle, PathDecision |
| planning_base/reference_line | 20 | ReferenceLine, ReferenceLineProvider |
| planning_base/proto | 14 | PlanningConfig, PlanningStatus, LearningData |
| planning_base/math | 12+ | Curve1d, PiecewiseJerk, ConstraintChecker |
| planners/public_road | 12 | PublicRoadPlanner, ScenarioManager |
| planners/lattice | 8+ | LatticePlanner |
| scenarios | 19种 | LaneFollowScenario, ValetParkingScenario等 |
| tasks | 35种 | LaneFollowPath, SpeedBoundsDecider等 |
| traffic_rules | 10种 | Crosswalk, TrafficLight, StopSign等 |
| planning_interface_base | 6+ | Planner, Scenario, Stage, Task基类 |
| planning_open_space | 7+ | OpenSpaceOptimizer |
| pnc_map | 8 | LaneFollowMap |
| park_data_center | 8 | ParkDataCenter, NudgeInfo |

---

## 十一、关键配置

| 配置项 | 值 | 说明 |
|--------|-----|------|
| 规划频率 | 10Hz | 100ms/周期 |
| 默认场景 | LANE_FOLLOW | 初始化时进入 |
| 默认规划器 | PublicRoadPlanner | 基于Scenario架构 |
| 参考线平滑 | 3种 | 离散点/QP样条/螺旋线 |
| 速度规划 | 分段Jerk优化 | DP粗优化 + QP精优化 |
| 路径规划 | Frenet坐标系优化 | 横向l边界内QP/NL优化 |
| 场景切换 | IsTransferable() | 遍历检查，STATUS_PROCESSING不切换 |
| 轨迹缝合 | TrajectoryStitcher | 连接历史轨迹与新规划 |
