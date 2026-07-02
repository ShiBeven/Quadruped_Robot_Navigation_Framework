# 项目完整参考文档

> **强制规则：后续代码无论进行何种修改、优化，本文档必须同步更新，保证文档与源码实时一致。**

---

## 目录

- [1. 项目概览](#1-项目概览)
- [2. 目录结构](#2-目录结构)
- [3. 包间数据流](#3-包间数据流)
- [4. src/navigation/nav_bringup — 启动入口与参数](#4-srcnavigationnav_bringup--启动入口与参数)
- [5. src/navigation/nav2_plugins — Nav2 行为树与代价地图插件](#5-srcnavigationnav2_plugins--nav2-行为树与代价地图插件)
- [6. src/navigation/omni_pid_pursuit_controller — 全向PID控制器](#6-srcnavigationomni_pid_pursuit_controller--全向pid控制器)
- [7. src/navigation/point_lio — LiDAR-惯性里程计](#7-srcnavigationpoint_lio--lidar-惯性里程计)
- [8. src/navigation/livox_ros_driver2 — Livox LiDAR 驱动](#8-srcnavigationlivox_ros_driver2--livox-lidar-驱动)
- [9. src/navigation/loam_interface — 帧适配器](#9-srcnavigationloam_interface--帧适配器)
- [10. src/navigation/sensor_scan_generation — 传感器同步](#10-srcnavigationsensor_scan_generation--传感器同步)
- [11. src/navigation/small_gicp_relocalization — 全局重定位](#11-srcnavigationsmall_gicp_relocalization--全局重定位)
- [12. src/navigation/terrain_analysis — 近场地形分析](#12-srcnavigationterrain_analysis--近场地形分析)
- [13. src/navigation/terrain_analysis_ext — 远场地形分析](#13-srcnavigationterrain_analysis_ext--远场地形分析)
- [14. src/navigation/pointcloud_to_laserscan — 点云转激光](#14-srcnavigationpointcloud_to_laserscan--点云转激光)
- [15. src/navigation/ign_sim_pointcloud_tool — 仿真点云转换](#15-srcnavigationign_sim_pointcloud_tool--仿真点云转换)
- [16. src/navigation/teleop_twist_joy — 手柄遥控](#16-srcnavigationteleop_twist_joy--手柄遥控)
- [17. src/simulation/nav2_loopback_sim — 回路仿真器](#17-srcsimulationnav2_loopback_sim--回路仿真器)
- [18. src/tools/pcd2pgm — PCD转PGM地图](#18-srctoolspcd2pgm--pcd转pgm地图)
- [19. src/tools/rosbag2_composable_recorder — 数据录制](#19-srctoolsrosbag2_composable_recorder--数据录制)
- [20. src/dependencies/ — 第三方依赖](#20-srcdependencies--第三方依赖)
- [21. 完整ROS话题索引](#21-完整ros话题索引)
- [22. 完整ROS参数索引](#22-完整ros参数索引)
- [23. TF坐标变换树](#23-tf坐标变换树)
- [24. 文件间依赖关系图谱](#24-文件间依赖关系图谱)

---

## 1. 项目概览

**项目名称：** Basic Navigation Framework for ROS 2
**ROS 版本：** Humble Hawksbill (Ubuntu 22.04)
**来源：** 从 RoboMaster 哨兵竞赛工程精简而来，移除视觉自瞄、串口/裁判系统通信、竞赛行为树，保留核心导航算法、通用底盘控制插件和仿真工具。
**编译命令：** `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`
**当前分支：** master
**包总数：** 16（原17个，已删除 `robot_interfaces`）

### 1.1 导航管线总览

```
Livox LiDAR (/livox/lidar + /livox/imu)
        ↓
  livox_ros_driver2 (LiDAR驱动 → CustomMsg/PointCloud2)
        ↓
  point_lio (LiDAR-惯性里程计, ESIKF + iVox)
        ↓  aft_mapped_to_init (里程计, lidar_odom帧)
  loam_interface (帧适配 → 标准odom帧)
        ↓  lidar_odometry + registered_scan
  sensor_scan_generation (时间同步 + TF生成)
        ↓  odometry (带速度), sensor_scan
  ┌─────┼─────┬──────────────────┐
  ↓     ↓     ↓                  ↓
terrain  terrain   small_gicp    pointcloud_to
_analysis _analysis_ext _reloc   _laserscan
  ↓       ↓         ↓              ↓
terrain  terrain   map→odom      obstacle_scan
_map     _map_ext  TF
  ↓       ↓
  └───┬───┘
      ↓  local_costmap/global_costmap (Nav2)
  Nav2 规划器 (SmacPlannerHybrid) + 控制器 (OmniPidPursuitController)
      ↓
  cmd_vel (速度指令 → 底盘)
```

---

## 2. 目录结构

```
Basic_Navigatio_Framework/
├── PROJECT_REFERENCE.md             ← 本文档
├── README.md
├── docs/
│   ├── AGENTS.md
│   ├── 阅读顺序指南.md
│   ├── slim_loopback_refactor.md
│   ├── 2026-07-03-wheeled-to-legged-adaptation-design.md
│   └── 后续工作指南.md
├── src/
│   ├── dependencies/                ← 第三方依赖 (3个)
│   │   ├── BehaviorTree.ROS2/
│   │   ├── joint_state_publisher/
│   │   └── sdformat_tools/
│   ├── navigation/                  ← 核心导航包 (13个)
│   │   ├── nav_bringup/             ← 启动入口+参数配置
│   │   ├── nav2_plugins/            ← Nav2扩展 (BT节点+代价地图层+行为)
│   │   ├── omni_pid_pursuit_controller/ ← 全向PID控制器
│   │   ├── point_lio/               ← LiDAR-惯性里程计
│   │   ├── livox_ros_driver2/       ← Livox LiDAR 驱动
│   │   ├── loam_interface/          ← 帧适配器
│   │   ├── sensor_scan_generation/  ← 传感器同步
│   │   ├── small_gicp_relocalization/ ← 全局重定位
│   │   ├── terrain_analysis/        ← 近场地形分析 (10m)
│   │   ├── terrain_analysis_ext/    ← 远场地形分析 (40m)
│   │   ├── pointcloud_to_laserscan/ ← 3D点云→2D激光
│   │   ├── ign_sim_pointcloud_tool/ ← 仿真点云格式转换
│   │   └── teleop_twist_joy/        ← 手柄遥控
│   ├── simulation/
│   │   └── nav2_loopback_sim/       ← 无物理引擎回路仿真器
│   └── tools/                       ← 辅助工具 (2个)
│       ├── pcd2pgm/                 ← PCD→PGM地图转换
│       └── rosbag2_composable_recorder/ ← 可组合rosbag录制
├── build/
├── install/
└── log/
```

### 2.1 包功能速查表

| 包名 | 类型 | 语言 | 一句话功能 | 修改频率 |
|------|------|------|-----------|:---:|
| `nav_bringup` | 配置 | Python/YAML/XML | 启动入口、参数配置 | ★★★★★ |
| `nav2_plugins` | 插件 | C++ | BT节点+代价地图层+恢复行为 | ★★★ |
| `omni_pid_pursuit_controller` | 控制器 | C++ | 全向PID纯追踪控制器 | ★★★★ |
| `point_lio` | 算法 | C++ | LiDAR-IMU融合里程计 (ESIKF+iVox) | ★★ |
| `livox_ros_driver2` | 驱动 | C++ | Livox雷达驱动 (3线程架构) | ★★ |
| `loam_interface` | 接口 | C++ | 坐标系桥接 (lidar_odom→odom) | ★ |
| `sensor_scan_generation` | 融合 | C++ | 传感器时间同步+TF发布 | ★ |
| `small_gicp_relocalization` | 算法 | C++ | 全局重定位 (GICP+KD-tree) | ★★ |
| `terrain_analysis` | 感知 | C++ | 近场10m地形分析 (682行单文件) | ★★★ |
| `terrain_analysis_ext` | 感知 | C++ | 远场40m地形分析+BFS连通性 (557行) | ★★★ |
| `pointcloud_to_laserscan` | 转换 | C++ | 3D→2D激光 | ★ |
| `ign_sim_pointcloud_tool` | 工具 | C++ | 仿真点云格式转换 | ★ |
| `teleop_twist_joy` | 控制 | C++ | 手柄遥控 (manual/auto模式) | ★★ |
| `nav2_loopback_sim` | 仿真 | Python | 无硬件回路仿真 | ★★ |
| `pcd2pgm` | 工具 | C++ | PCD→PGM地图 | ★★ |
| `rosbag2_composable_recorder` | 工具 | C++ | rosbag录制 | ★★ |

---

## 3. 包间数据流

### 3.1 完整话题数据流

```
[livox_ros_driver2]
  └→ /livox/lidar (CustomMsg/PointCloud2)
  └→ /livox/imu (sensor_msgs/Imu)

[point_lio]
  ← /livox/lidar, /livox/imu
  └→ /aft_mapped_to_init (Odometry, lidar_odom帧)
  └→ /cloud_registered (PointCloud2, lidar_odom帧)
  └→ /path (Path, 可选)
  └→ /cloud_undistorted (PointCloud2, 可选)

[loam_interface]
  ← /aft_mapped_to_init, /cloud_registered
  └→ /lidar_odometry (Odometry, odom帧)
  └→ /registered_scan (PointCloud2, odom帧)

[sensor_scan_generation]
  ← /lidar_odometry, /registered_scan (时间同步)
  ← /tf_static (lidar→chassis, lidar→gimbal)
  └→ /odometry (Odometry + twist速度)
  └→ /sensor_scan (PointCloud2, lidar本地帧)
  └→ TF: odom→chassis

[small_gicp_relocalization]
  ← /registered_scan
  ← /initialpose
  └→ TF: map→odom (20Hz)

[terrain_analysis]
  ← /lidar_odometry, /registered_scan
  └→ /terrain_map (PointCloud2, height-above-ground as intensity)

[terrain_analysis_ext]
  ← /lidar_odometry, /registered_scan
  ← /terrain_map (来自terrain_analysis的近场地图)
  └→ /terrain_map_ext (PointCloud2, 近场+远场融合)

[pointcloud_to_laserscan]
  ← /terrain_map_ext (as cloud_in → remapped)
  └→ /obstacle_scan (LaserScan, 2D)

[Nav2 Stack]
  ← /map (OccupancyGrid, 来自map_server)
  ← /odometry (来自sensor_scan_generation)
  ← /terrain_map, /terrain_map_ext (注入代价地图)
  ← /scan 或 /obstacle_scan (LaserScan)
  ← TF: map→odom→chassis→...
  └→ /cmd_vel (Twist, 经velocity_smoother)
  └→ /local_plan, /plan (Path)
  └→ /local_costmap/costmap, /global_costmap/costmap

[omni_pid_pursuit_controller] (Nav2内部)
  ← /plan (全局路径)
  ← /local_costmap/costmap
  └→ /cmd_vel_nav2_result (内部, 经velocity_smoother → /cmd_vel)
  └→ /local_plan, /lookahead_point

[velocity_smoother] (Nav2内部)
  ← /cmd_vel_nav2_result
  ← /odometry
  └→ /cmd_vel (最终底盘指令)
```

### 3.2 TF坐标变换链

```
map → odom → base_footprint → base_link → body → front_mid360
```

- `map→odom`: 由 small_gicp_relocalization 广播 (全局重定位修正)
- `odom→base_footprint`: 由 sensor_scan_generation 广播 (里程计)
- `base_footprint→base_link`: 静态TF (URDF, Z偏移)
- `base_link→body`: 静态TF (URDF)
- `body→front_mid360`: 静态TF (URDF, LiDAR外参)

---

## 4. src/navigation/nav_bringup — 启动入口与参数

**语言**: Python (launch) + YAML (config) + XML (BT) + URDF
**构建系统**: `ament_cmake` (仅安装数据文件)
**执行文件依赖**: 所有导航相关包

### 4.1 文件列表

| 文件 | 描述 |
|------|------|
| `CMakeLists.txt` | 安装 launch/map/config/behavior_trees/description 到 share |
| `package.xml` | v2.0.0, 四足导航启动包 |
| `config/nav2_params.legged.yaml` | 四足实机参数 (~380行) |
| `config/nav2_params.legged_sim.yaml` | 四足仿真参数 (~350行) |
| `description/quadruped.urdf` | 四足URDF模型 (TF链 + 4腿关节占位) |
| `launch/legged_navigation_launch.py` | 四足导航完整启动 |
| `launch/legged_localization_launch.py` | 四足定位管线启动 |
| `launch/legged_slam_launch.py` | 四足 SLAM 启动 |
| `behavior_trees/legged_navigate_w_replanning_and_recovery.xml` | 四足行为树 (Spin恢复) |
| `map/README.txt` | 放置 `.pgm` 和 `.yaml` 地图文件 |

### 4.2 启动脚本详解

#### legged_navigation_launch.py

**函数**: `generate_launch_description() -> LaunchDescription`

**Launch参数:**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `namespace` | `""` | 命名空间 |
| `use_sim_time` | `"false"` | 仿真时钟 |
| `autostart` | `"true"` | 自动激活生命周期节点 |
| `params_file` | `config/nav2_params.legged.yaml` | Nav2参数文件 |
| `use_composition` | `"False"` | 是否使用组合节点 |
| `container_name` | `"nav2_container"` | 组合容器名 |
| `use_respawn` | `"False"` | 节点崩溃后自动重启 |
| `log_level` | `"info"` | 日志级别 |
| `use_sensor_scan` | `"true"` | 是否启动sensor_scan_generation |

**启动的节点 (非组合模式):**

| 节点名 | 包/可执行文件 | 功能 |
|--------|-------------|------|
| `terrain_analysis` | terrain_analysis/terrainAnalysis | 近场地形分析 |
| `terrain_analysis_ext` | terrain_analysis_ext/terrainAnalysisExt | 远场地形分析 |
| `loam_interface` | loam_interface/loam_interface_node | 帧适配 |
| `sensor_scan_generation` | sensor_scan_generation/sensor_scan_generation_node | 传感器同步 (条件启动) |
| `controller_server` | nav2_controller | Nav2控制器 (remap cmd_vel→cmd_vel_nav2_result) |
| `smoother_server` | nav2_smoother | 路径平滑器 |
| `planner_server` | nav2_planner | 全局规划器 |
| `behavior_server` | nav2_behaviors | 行为服务器 (Spin/Backup/Wait) |
| `bt_navigator` | nav2_bt_navigator | 行为树导航器 |
| `waypoint_follower` | nav2_waypoint_follower | 途经点跟随 |
| `velocity_smoother` | nav2_velocity_smoother | 速度平滑器 |
| `lifecycle_manager_navigation` | nav2_lifecycle_manager | 生命周期管理器 |

**cmd_vel 流向:** 控制器→`cmd_vel_nav2_result`→velocity_smoother→`cmd_vel`→底盘

#### legged_localization_launch.py

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `map` | *(必填)* | 地图YAML文件路径 |
| `prior_pcd` | `""` | 先验PCD点云地图路径 |
| `params_file` | `config/nav2_params.legged.yaml` | 定位参数 |
| `use_sim_time` | `"false"` | 仿真时钟 |
| `autostart` | `"true"` | 自动启动 |

**启动的节点:** `point_lio`, `map_server`, `small_gicp_relocalization`, `lifecycle_manager_localization`

#### legged_slam_launch.py

**启动的节点:** `pointcloud_to_laserscan`, `slam_toolbox`, `point_lio` (prior_pcd.enable=False), `static_transform_publisher` (map→odom 恒等)

**remappings:** `cloud_in` ← `terrain_map_ext`; `scan` → `obstacle_scan`

### 4.3 行为树 XML

#### legged_navigate_w_replanning_and_recovery.xml

```xml
RecoveryNode (NavigateRecovery, 重试10次)
  PipelineSequence
    RateController (3Hz)
      RecoveryNode (ComputePathToPose, 重试1次)
        -- child: ComputePathToPose (planner_id="GridBased")
        -- recovery: ClearEntireCostmap (global)
    RecoveryNode (FollowPath, 重试10次)
      -- child: FollowPath (controller_id="FollowPath")
      -- recovery: ClearEntireCostmap (local)
  ReactiveFallback (RecoveryFallback)
    GoalUpdated
    RoundRobin
      Sequence: ClearLocalCostmap + ClearGlobalCostmap
      Spin (3.14 rad = 180°, is_recovery=true)   ← 四足: 原地旋转替代轮式后退
```

### 4.4 参数文件关键差异 (四足 vs 原轮式)

| 参数域 | 关键四足值 |
|--------|-----------|
| `controller_server.FollowPath` | kp=1.5, ki=0.05, kd=0.1, v_linear=±1.5, v_angular=±1.5 |
| `velocity_smoother` | max_vel=[1.2,1.2,1.2], max_accel=[0.8,0.8,1.0] |
| `local_costmap` | 6m×6m, robot_radius=0.45, inflation=0.6 |
| `global_costmap` | robot_radius=0.45, inflation=0.8 |
| `planner_server` | DUBIN, minimum_turning_radius=0.0, tolerance=0.25 |
| `behavior_server` | backup: nav2_behaviors/BackUp (标准,非轮式BackUpFreeSpace) |
| `sensor_scan_generation.robot_base_frame` | `"body"` (非 `"gimbal_yaw"`) |
| `small_gicp_relocalization.robot_base_frame` | `"body"` |

### 4.5 URDF 结构 (quadruped.urdf)

```
base_footprint → base_link (Z偏移 0.30m)
base_link → body (identity)
body → front_mid360 (外参: [0.3, 0, 0.25, 0, 0, 0])
body → fl_hip, fr_hip, rl_hip, rr_hip (四腿关节占位)
```

所有腿关节当前为 `fixed` 类型。TF 链通过 `robot_state_publisher` 加载。

---

## 5. src/navigation/nav2_plugins — Nav2 行为树与代价地图插件

**语言**: C++ | **构建系统**: `ament_cmake` | **依赖**: Nav2, BehaviorTree.CPP, TF2

### 5.1 构建产物 (14个独立共享库)

| 库名 | 源文件 | 注册名 | 类型 |
|------|--------|--------|------|
| `pb_back_up_frees_space_behavior` | back_up_free_space.cpp | `pb_nav2_behaviors/BackUpFreeSpace` | nav2_core Behavior |
| `layers` | intensity_voxel_layer.cpp | `pb_nav2_costmap_2d::IntensityVoxelLayer` | costmap Layer |
| `is_path_goal_reached_bt` | is_path_goal_reached.cpp | `IsPathGoalReached` | BT Condition |
| `send_nav2_goal_bt` | send_nav2_goal.cpp | `SendNav2Goal` | BT Action (RosActionNode) |
| `publish_nav_goal_bt` | publish_nav_goal.cpp | `PublishNavGoal` | BT Action |
| `pub_spin_speed_bt` | pub_spin_speed.cpp | `PublishSpinSpeed` | BT Action |
| `pub_twist_bt` | pub_twist.cpp | `PublishTwist` | BT Action |
| `select_patrol_path_bt` | select_patrol_path.cpp | `SelectPatrolPath` | BT Action |
| `select_fixed_path_bt` | select_fixed_path.cpp | `SelectFixedPath` | BT Action |
| `hold_stop_flag_bt` | hold_stop_flag.cpp | `HoldStopFlag` | BT Action |
| `send_nav_through_poses_bt` | send_nav_through_poses.cpp | `SendNavThroughPoses` | BT Action |
| `select_path_goal_pose_bt` | select_path_goal_pose.cpp | `SelectPathGoalPose` | BT Action |
| `recovery_node_bt` | recovery_node.cpp | `RecoveryNode` | BT Control |
| `rate_controller_bt` | rate_controller.cpp | `RateController` | BT Decorator |

### 5.2 共享工具函数

**文件**: `include/nav2_plugins/bt/nav_utils.hpp`
**命名空间**: `nav2_plugins`

| 函数 | 签名 | 功能 |
|------|------|------|
| `getNodeFromBlackboard` | `(tree_node) → Node::SharedPtr` | 从根黑板获取ROS节点, 失败抛 `std::runtime_error` |
| `makePoseStamped` | `(point, frame_id="map") → PoseStamped` | Point→PoseStamped (identity orientation) |
| `buildGoalPoints` | `(xs, ys, zs) → vector<Point>` | 三个并行向量构建目标点列表 |
| `validateIndex` | `(index, total, label) → size_t` | 索引边界检查, 失败抛 `std::runtime_error` |
| `buildPathFromIndices` | `(goal_points, indices, frame_id) → Path` | 从索引列表构建Path |
| `squaredDistance` | `(lhs, rhs) → double` | 两点间欧几里得距离平方 |
| `normalizeAngle` | `(angle) → double` | 标准化到[-pi, pi] |
| `isPathGoalReached` | `(pose, path, tolerance) → bool` | 检查current_pose到path末点距离≤tolerance |
| `findNearestIndex` | `(pose, goal_points, candidates) → size_t` | 找到最近目标点索引, O(n)遍历 |
| `computeNextPatrolState` | `(cursor, direction, count) → pair<int,int>` | 往返巡逻状态机: direction=1递增到count-1后反转 |
| `pathEquivalent` | `(lhs, rhs, tolerance) → bool` | 比较两条路径的poses是否长度相同且对应点距离<tolerance |
| `toSizeIndices` | `(raw, total, label) → vector<size_t>` | int64→size_t转换+范围验证 |

**文件**: `include/nav2_plugins/bt/custom_types.hpp`

提供 BehaviorTree.CPP 的 `convertFromString<PoseStamped>` 模板特化:
- 7元素格式: `x;y;z;qx;qy;qz;qw` → 完整位姿
- 3元素格式: `x;y;yaw` → 位置+yaw角 (自动转四元数)

### 5.3 BT Action 节点详解

#### 5.3.1 HoldStopFlag (`HoldStopFlagAction`)
- **继承**: `BT::StatefulActionNode`
- **参数**: `nav.decision_config.waypoint_stop_duration_s` (double) — 停留秒数
- **发布话题**: `stop_flag` (std_msgs/Bool)
- **行为**:
  - `onStart()`: 发布 `Bool(data=true)`, 记录开始时间 `start_ = now()`
  - `onRunning()`: 若 `elapsed >= duration_` → 发布 `Bool(data=false)`, 返回 SUCCESS; 否则 RUNNING
  - `onHalted()`: 发布 `Bool(data=false)` 确保停止标志被清除

#### 5.3.2 PublishSpinSpeed (`PublishSpinSpeedAction`)
- **继承**: `BT::RosTopicPubStatefulActionNode<Float32>`
- **输入端口**: `spin_speed` (double) — 角速度 rad/s
- **发布话题**: 可配置 (topic_name端口, 默认 `"spin_speed"`)
- **行为**: `setMessage()`→Float32.data=spin_speed; `setHaltMessage()`→data=0

#### 5.3.3 PublishTwist (`PublishTwistAction`)
- **继承**: `BT::RosTopicPubStatefulActionNode<Twist>`
- **输入端口**: `v_x`, `v_y`, `v_yaw` (double) — 各轴速度
- **发布话题**: 可配置 (topic_name端口)
- **行为**: setMessage()→Twist(linear.x=v_x, linear.y=v_y, angular.z=v_yaw); setHaltMessage()→全零

#### 5.3.4 PublishNavGoal (`PublishNavGoalAction`)
- **继承**: `BT::SyncActionNode`
- **输入端口**: `goal` (PoseStamped), `topic_name` (string, 默认 `"goal_pose"`)
- **参数**: `nav.decision_config.goal_position_tolerance` (double, 默认0.1)
- **行为**: 去重发布 — `position_distance < tolerance && yaw_diff < 0.05` 则跳过; 新目标额外多发1次 (共2次) 防丢包
- **话题切换**: topic_name变化时销毁旧publisher重建, 重置 `last_published_goal_`

#### 5.3.5 SelectFixedPath (`SelectFixedPathAction`)
- **继承**: `BT::SyncActionNode`
- **输入端口**: `target_index` (int)
- **输出端口**: `path` (Path, default=`{decision_path}`)
- **参数**: `nav.goal_points.x/y/z` (vector<double>)
- **行为**: 从 goal_points 中取 target_index 对应的单个点, 构建单点 Path

#### 5.3.6 SelectPatrolPath (`SelectPatrolPathAction`)
- **继承**: `BT::SyncActionNode`
- **输入端口**: `patrol_cursor` (int), `patrol_direction` (int)
- **输出端口**: `path` (Path), `next_cursor` (int), `next_direction` (int)
- **参数**:
  - `nav.goal_points.x/y/z` — 所有目标点
  - `nav.point_roles.patrol_indices` (vector<int64>) — 巡逻点子集
  - `nav.decision_config.patrol_preview_points` (int, 默认2) — 预览点数
- **行为**: 使用 `computeNextPatrolState()` 实现往返巡逻模式: cursor在patrol_indices范围内前后移动, 构建连续preview_points个途经点的Path, 输出新的cursor和direction

#### 5.3.7 SelectPathGoalPose (`SelectPathGoalPoseAction`)
- **继承**: `BT::SyncActionNode`
- **输入端口**: `path` (Path)
- **输出端口**: `goal` (PoseStamped)
- **行为**: 提取 Path 的最后一个位姿 (back()) 作为 goal

#### 5.3.8 SendNav2Goal (`SendNav2GoalAction`)
- **继承**: `BT::RosActionNode<NavigateToPose>`
- **输入端口**: `goal` (PoseStamped, 格式 "x;y;yaw")
- **ROS Action**: `nav2_msgs::action::NavigateToPose` (客户端)
- **行为**: `setGoal()`→填充 frame_id="map"; `onResultReceived()`→SUCCEEDED→SUCCESS

#### 5.3.9 SendNavThroughPoses (`SendNavThroughPosesAction`)
- **继承**: `BT::SyncActionNode` (手动管理Action客户端, 非RosActionNode)
- **输入端口**: `path` (Path)
- **输出端口**: `current_pose` (PoseStamped), `goal_succeeded` (bool)
- **参数**:
  - `nav.decision_config.nav2_action_server` (string, 默认"/navigate_through_poses")
  - `nav.decision_config.goal_position_tolerance` (double, 默认0.1)
  - `nav.decision_config.action_server_wait_timeout_s` (double, 默认2.0)
- **ROS Action**: `nav2_msgs::action::NavigateThroughPoses` (客户端)
- **行为**:
  - 路径去重: 使用 `pathEquivalent()` 比较, 相同路径不重发
  - 路径变更: 调用 `cancel_all_goals()` 取消旧goal, 然后 `async_send_goal()`
  - 反馈: 在回调中更新 `current_pose_`
  - 成功: 在回调中设置 `goal_succeeded_`
- **并发安全**: 全部共享状态经 `std::mutex` 保护, `request_id_` 防回调错乱

### 5.4 BT Condition 节点

#### 5.4.1 IsPathGoalReached (`IsPathGoalReachedCondition`)
- **继承**: `BT::SimpleConditionNode`
- **输入端口**: `path` (Path), `current_pose` (PoseStamped)
- **参数**: `nav.decision_config.path_tolerance` (double, 默认0.2)
- **行为**: `isPathGoalReached(current_pose, path, tolerance) → bool`

### 5.5 BT Control 节点

#### 5.5.1 RecoveryNode
- **继承**: `BT::ControlNode`
- **输入端口**: `num_attempts` (int, 默认999)
- **要求**: 恰好2个子节点 (child[0]=主行为, child[1]=恢复)
- **行为**:
  - tick child[0] → SUCCESS → 整体 SUCCESS
  - tick child[0] → FAILURE → tick child[1] → SUCCESS → 重置重试计数, tick child[0]
  - tick child[1] → FAILURE → 整体 FAILURE
  - 任一 RUNNING → 整体 RUNNING
  - 超过 num_attempts 次重试 → FAILURE

### 5.6 BT Decorator 节点

#### 5.6.1 RateController
- **继承**: `BT::DecoratorNode`
- **输入端口**: `hz` (double, 默认10.0)
- **行为**: 限频执行子节点 — 两次tick间隔 >= 1/hz 才透传, 否则返回 RUNNING; 子节点完成后返回其结果

### 5.7 Behavior 插件

#### 5.7.1 BackUpFreeSpace
- **继承**: `nav2_behaviors::DriveOnHeading<BackUp>`
- **注册**: `pb_nav2_behaviors/BackUpFreeSpace`
- **参数**: `global_frame` (map), `max_radius` (1.0m), `service_name` ("local_costmap/get_costmap"), `visualize` (false)
- **算法 `findBestDirection()`**:
  1. 调用 `local_costmap/get_costmap` 服务获取当前代价地图
  2. 扫描360度 (步长 pi/32 ≈ 5.6°), 每角度射线追踪至 max_radius
  3. 检测 costmap 值 ≥ 253 (INSCRIBED_INFLATED_OBSTACLE) 的占据格
  4. 累计各角度的安全距离, 找到最大连续安全扇区
  5. 返回扇区中点角度
  6. 后退方向: twist.x = cos(angle) × speed, twist.y = sin(angle) × speed
- **发布**: `back_up_free_space_markers` (MarkerArray, 条件当 visualize=true)

> **注意**: BackUpFreeSpace 在四足配置中未使用 (behavior_server.backup 使用标准 nav2_behaviors/BackUp)

### 5.8 代价地图层插件

#### 5.8.1 IntensityVoxelLayer
- **继承**: `nav2_costmap_2d::ObstacleLayer`
- **注册**: `pb_nav2_costmap_2d::IntensityVoxelLayer`
- **特性**: 在标准ObstacleLayer基础上增加3D体素网格 + point intensity 过滤
- **参数** (命名空间 `<name>.xxx`):
  - `z_voxels` (16) — Z方向体素数
  - `origin_z` (0.0) — Z原点
  - `min_obstacle_intensity` (0.1) — 障碍物最小强度 (低于为地面)
  - `max_obstacle_intensity` (2.0) — 障碍物最大强度
  - `z_resolution` (0.05) — Z分辨率
  - `unknown_threshold` (15+VOXEL_BITS-size_z)
  - `mark_threshold` (0)
  - `publish_voxel_map` (false)
- **发布**: `voxel_grid` (VoxelGrid, 条件当 publish_voxel_map=true)
- **算法 `updateBounds()`**:
  1. 通过消息过滤器获取观测点云 (继承自ObstacleLayer)
  2. 逐点过滤: 高度 → **强度** (min~max) → 距离 (min/max range)
  3. `markVoxelInMap()`: 标记3D体素
  4. 体素计数 ≥ mark_threshold → 2D代价地图设为 LETHAL_OBSTACLE
  5. `updateFootprint()`: 清除足迹区域

### 5.9 完整黑板键 (BB Key) 使用表

| 黑板键 | 类型 | 使用者 |
|--------|------|--------|
| `node` (root) | Node::SharedPtr | 全部节点 (getNodeFromBlackboard) |
| `{decision_path}` / `path` | Path | SelectFixedPath, SelectPatrolPath, SelectPathGoalPose, SendNavThroughPoses, IsPathGoalReached |
| `{decision_goal_pose}` / `goal` | PoseStamped | PublishNavGoal, SendNav2Goal, SelectPathGoalPose |
| `{decision_patrol_cursor}` / `patrol_cursor` | int | SelectPatrolPath |
| `{decision_patrol_direction}` / `patrol_direction` | int | SelectPatrolPath |
| `{decision_next_patrol_cursor}` / `next_cursor` | int | SelectPatrolPath |
| `{decision_next_patrol_direction}` / `next_direction` | int | SelectPatrolPath |
| `{decision_current_pose}` / `current_pose` | PoseStamped | SendNavThroughPoses, IsPathGoalReached |
| `{decision_nav_goal_succeeded}` / `goal_succeeded` | bool | SendNavThroughPoses, IsPathGoalReached |

---

## 6. src/navigation/omni_pid_pursuit_controller — 全向PID控制器

**语言**: C++ | **构建系统**: `ament_cmake`
**注册**: `nav2_core::Controller` 插件, class=`OmniPidPursuitController`

### 6.1 文件列表

| 文件 | 描述 |
|------|------|
| `include/pb_omni_pid_pursuit_controller/pid.hpp` | PID类声明 |
| `include/pb_omni_pid_pursuit_controller/omni_pid_pursuit_controller.hpp` | 主控制器类声明 |
| `src/pid.cpp` | PID实现 |
| `src/omni_pid_pursuit_controller.cpp` | 控制器核心实现 (~300行) |
| `pb_omni_pid_pursuit_controller.xml` | 插件注册文件 |

### 6.2 PID类 (`pid.hpp` + `pid.cpp`)

**成员变量** (全部 `double`, 全部 private):
| 变量 | 含义 |
|------|------|
| `dt_` | 循环间隔时间 (秒) |
| `max_` / `min_` | 输出限幅 |
| `kp_` / `kd_` / `ki_` | PID增益 |
| `pre_error_` | 前次误差 (用于微分) |
| `integral_` | 累积积分误差 |

**核心方法 `calculate(set_point, pv) → double`**:
```
1. error = set_point - pv
2. p_out = kp × error
3. integral += error × dt   (限幅到 [-1, 1])
4. i_out = ki × integral
5. derivative = (error - pre_error) / dt
6. d_out = kd × derivative
7. output = clamp(p_out + i_out + d_out, min, max)
8. pre_error = error
```

### 6.3 OmniPidPursuitController 类 (`omni_pid_pursuit_controller.hpp` + `.cpp`)

**nav2_core::Controller 接口实现:**

| 方法 | 功能 |
|------|------|
| `configure(parent, name, tf, costmap)` | 读取参数, 创建publisher, 初始化两个PID (translation + rotation) |
| `cleanup()` | 重置publisher shared_ptr |
| `activate()` | 激活publisher + 注册动态参数回调 |
| `deactivate()` | 停用publisher + 重置参数回调 |
| `computeVelocityCommands(pose, velocity, goal_checker) → TwistStamped` | **主控制循环** |
| `setPlan(path)` | 存储全局路径到 `global_plan_` |
| `setSpeedLimit(limit, percentage)` | 空实现 (全向不限制) |

#### `computeVelocityCommands()` 详细流程

**步骤1: `transformGlobalPlan(pose)`**
- 将全局路径从 map 帧变换到 `robot_base_frame_`
- 裁剪超出代价地图范围的位姿 (使用 `worldToMap` 边界检查)
- 去除重复位姿 (距离 < 0.01m)
- 如果没有有效点: 返回原地旋转 (如果 enable_rotation) 或 零速

**步骤2: 航向对齐检查**
- 如果 `use_rotate_to_heading_` 启用:
  - 计算当前偏航角 (从 pose 四元数提取) 与路径方向的角度差
  - 若 |角差| > `use_rotate_to_heading_treshold_` (0.1 rad):
    - 返回纯旋转指令: angular.z = PID(theta_diff, 0), linear.x=y=0

**步骤3: `getLookAheadPoint(lookahead_dist, transformed_plan)`**
- 在变换后的局部路径上, 找到第一个距离超过 lookahead_dist 的位姿
- 如果启用 `use_interpolation_`: 调用 `circleSegmentIntersection()` 在"圆内-圆外"两个位姿之间精确计算线段与圆的交点

**步骤4: PID控制**
- `dist` = 当前位置到预瞄点的距离
- `theta_dist` = atan2(prey.y, prey.x) (预瞄点相对于当前朝向的角度)
- `lin_vel = translation_pid_.calculate(dist, 0)` (目标距离=0)
- `angular_vel = rotation_pid_.calculate(theta_dist, 0)` (仅在 enable_rotation 时)
- 全向分解: `cmd_vel.linear.x = lin_vel × cos(theta_dist)`, `cmd_vel.linear.y = lin_vel × sin(theta_dist)`

**步骤5: `applyCurvatureLimitation()`**
- `calculateCurvature()`: 三圆法曲率半径计算
  - 取预瞄点 ± forward/backward_dist (0.7m/0.3m) 处的两个路径点
  - 计算三点的外接圆半径 R → 曲率 = 1/R
- 线性插值: curvature ∈ [curvature_min, curvature_max] → ratio ∈ [1.0, reduction_ratio]
- 速率限制: 缩放因子变化率受 `max_velocity_scaling_factor_rate_` 约束

**步骤6: `applyApproachVelocityScaling()`**
- 当 `dist_to_goal < approach_velocity_scaling_dist_` 时:
  - ratio = max(min_approach_velocity/v_max, dist_to_goal / approach_dist)
  - 线性缩放线速度和角速度

**步骤7: `isCollisionDetected()`**
- 检查 robot_pose 处的 costmap 值
- 如果 ≥ INSCRIBED_INFLATED_OBSTACLE (253): 返回零速并记录警告

**步骤8: 发布可视化**
- `local_plan`: 变换后的局部路径 (Path)
- `lookahead_point`: 预瞄点 (PointStamped)
- `curvature_points`: 曲率计算用的三个点 (Marker)

### 6.4 全部可动态调整参数 (27个)

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `translation_kp/ki/kd` | double | 1.5/0.05/0.1 | 平移PID增益 |
| `rotation_kp/ki/kd` | double | 1.0/0.05/0.1 | 旋转PID增益 |
| `enable_rotation` | bool | true | 启用旋转PID |
| `use_interpolation` | bool | false | 预瞄点插值 |
| `use_velocity_scaled_lookahead_dist` | bool | true | 速度缩放预瞄距离 |
| `use_rotate_to_heading` | bool | true | 先对齐航向再移动 |
| `use_rotate_to_heading_treshold` | double | 0.1 | 航向对齐阈值 (rad) |
| `lookahead_dist` | double | 1.0 | 预瞄距离 (m) |
| `min_lookahead_dist` | double | 0.3 | 最小预瞄距离 |
| `max_lookahead_dist` | double | 1.5 | 最大预瞄距离 |
| `lookahead_time` | double | 1.0 | 速度×time=动态预瞄距离 |
| `v_linear_min/max` | double | -1.5/1.5 | 线速度限幅 (m/s) |
| `v_angular_min/max` | double | -1.5/1.5 | 角速度限幅 (rad/s) |
| `min_approach_linear_velocity` | double | 0.1 | 接近阶段最小线速度 |
| `approach_velocity_scaling_dist` | double | 1.0 | 接近减速距离 (m) |
| `curvature_min` | double | 1.0 | 低曲率阈值 (不减速) |
| `curvature_max` | double | 3.0 | 高曲率阈值 (最大减速) |
| `reduction_ratio_at_high_curvature` | double | 0.4 | 高曲率减速比例 |
| `curvature_forward_dist` | double | 0.7 | 前向曲率计算距离 |
| `curvature_backward_dist` | double | 0.3 | 后向曲率计算距离 |
| `max_velocity_scaling_factor_rate` | double | 0.9 | 缩放因子变化率 |
| `max_robot_pose_search_dist` | double | 10.0 | 路径搜索最大距离 |
| `transform_tolerance` | double | 0.1 | TF变换容差 |
| `controller_frequency` | double | 20.0 | 控制频率 (从父节点继承) |

### 6.5 线程模型

单线程。`computeVelocityCommands()` 在 Nav2 controller_server 的定时器回调中调用 (20Hz)。无内部线程、无mutex。

### 6.6 依赖

- `nav2_core` — Controller 基类
- `nav2_costmap_2d` — 代价地图查询
- `tf2_ros` — TF 变换
- `pluginlib` — 插件注册

---

## 7. src/navigation/point_lio — LiDAR-惯性里程计

**语言**: C++17 | **构建系统**: CMake (OpenMP, Eigen3, PCL)
**可执行文件**: `pointlio_mapping` | **作者**: Dongjiao He (HKU)

### 7.1 文件列表

| 文件 | 描述 |
|------|------|
| `src/laserMapping.cpp` | 主入口: ROS节点 + 主循环 + 点云/里程计发布 |
| `src/li_initialization.cpp` / `.h` | LiDAR/IMU回调 + 时间同步 (message_filters) |
| `src/parameters.cpp` / `.h` | 参数加载 (从ROS param获取全部配置) |
| `src/preprocess.cpp` / `.h` | 点云预处理: 降采样, 去畸变, 点过滤器 |
| `src/Estimator.cpp` / `.h` | **ESIKF核心**: 状态转移, 观测模型, 雅可比, 迭代更新 |
| `src/IMU_Processing.cpp` / `.h` | IMU正向传播: 中值积分, 状态预测 |
| `include/common_lib.h` | 通用类型定义 (状态向量18D, 矩阵, 点结构) |
| `include/so3_math.h` | SO(3)流形数学运算 (exp, log, boxplus, boxminus) |
| `include/ivox/` | iVox数据结构 (增量体素, 自适应分辨率空间哈希) |
| `include/IKFoM/` | 流形上的迭代误差状态卡尔曼滤波 (ESIKF) 模板库 |
| `include/matplotlibcpp.h` | Python matplotlib绑定 (调试用, 非核心) |
| `config/mid360.yaml` | Livox MID-360 默认配置 |

### 7.2 核心算法架构

Point-LIO 是一种**逐点**更新的 LiDAR-惯性里程计。与经典 LOAM 系列 (按帧处理) 不同, 它在每个 LiDAR 点到达时立即更新状态估计。

**状态向量** (18维): `[p(3), v(3), R(SO(3)→3), ba(3), bg(3), g(3)]`

**算法流程:**

1. **IMU传播** (`imu_callback` → `IMU_Processing.cpp`):
   - 每次 IMU 数据到达时, 用 ESIKF 对状态进行前向传播 (预测步)
   - 中值积分: 使用当前和上一帧 IMU 的均值进行加速度/角速度积分
   - 传播频率 = IMU 频率 (通常 200+ Hz)

2. **LiDAR点处理** (`livox_pcl_callback` → `laserMapping.cpp` 主循环):
   - 每个 LiDAR 点到达时, 查找 iVox 地图中的最近体素
   - 计算残差: 点到局部拟合平面的距离 (`plane_thr=0.1m`)
   - ESIKF 更新步: 用该残差迭代更新状态 (每个点迭代最多3次)
   - 将新点加入 iVox 地图
   - 更新频率 = 点频率 (数万 Hz)

3. **输出** (`laserMapping.cpp` `publish_frame_world()`):
   - 发布 `/aft_mapped_to_init` 里程计 (lidar_odom 帧, 10-20Hz)
   - 发布 `/cloud_registered` 去畸变点云 (lidar_odom 帧)
   - 可选: `/path` (轨迹), `/cloud_undistorted`

### 7.3 iVox 数据结构 (`include/ivox/`)

自适应分辨率空间哈希网格:
- `ivox_grid_resolution`: 默认 0.5m 体素大小
- `ivox_nearby_type`: 18 邻域搜索模式 (可配置 0/6/18/26)
- 每个体素存储点云的均值/协方差, 用于快速平面拟合
- O(1) 增量插入 + O(1) 最近邻查询

### 7.4 ESIKF 详解 (`Estimator.cpp`)

**状态向量** (18维): `x = [p, v, R, ba, bg, g]`
- p: 位置 (3), v: 速度 (3), R: 姿态 SO(3), ba/bg: IMU偏置 (各3), g: 重力 (3)

**传播模型** (IMU预报):
- p' = v
- v' = R × (acc - ba) + g
- R' = R × (gyro - bg)^
- ba' = 0, bg' = 0, g' = 0

**观测模型** (LiDAR更新):
- 对每个 LiDAR 点, 在 iVox 地图中搜索邻近体素
- 拟合局部平面 (从体素的均值/协方差)
- 残差 = 点到平面距离 = |(p_point - p_plane) · n_plane|
- 雅可比 = ∂(点到平面距离)/∂状态 (用链式法则, 经 SO(3) 的 Baker-Campbell-Hausdorff 近似)

**迭代更新**: 每个点迭代更新状态 (max 3次), 使用 IKFoM 的迭代重加权最小二乘

### 7.5 ROS 接口

| 方向 | 话题名 | 类型 | 说明 |
|------|--------|------|------|
| Sub | `lid_topic` (默认 "livox/lidar") | CustomMsg 或 PointCloud2 | 可配置 lidar_type |
| Sub | `imu_topic` (默认 "livox/imu") | Imu | IMU 数据 |
| Pub | `aft_mapped_to_init` | Odometry | 里程计 (lidar_odom帧) |
| Pub | `cloud_registered` | PointCloud2 | 去畸变+注册点云 |
| Pub | `path` (可选) | Path | 轨迹路径 |
| Pub | `cloud_undistorted` (可选) | PointCloud2 | 无畸变点云 |

### 7.6 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `preprocess.lidar_type` | 1 | 1=Livox, 2=Velodyne, 3=Ouster |
| `preprocess.scan_line` | 4 | LiDAR线数 |
| `preprocess.blind` | 0.3 | 盲区距离 (m) |
| `filter_size_surf` | 0.05 | 表面点降采样 (m) |
| `filter_size_map` | 0.15 | 地图点降采样 (m) |
| `ivox_grid_resolution` | 0.5 | iVox体素大小 (m) |
| `mapping.plane_thr` | 0.1 | 平面拟合阈值 (m) |
| `mapping.gravity` | [0,0,-9.81] | 重力向量 |
| `mapping.extrinsic_T` | [0,0,0] | LiDAR→IMU 平移外参 |
| `mapping.extrinsic_R` | I | LiDAR→IMU 旋转外参 (行主序) |

### 7.7 线程模型

- **主线程**: ROS spin + 主循环 (发布)
- **IMU回调**: 在 ROS executor 线程, 触发 ESIKF 传播
- **LiDAR回调**: 在 ROS executor 线程, 触发 ESIKF 更新
- OpenMP 用于并行化点云处理 (降采样, 去畸变)

### 7.8 依赖

- `rclcpp`, `sensor_msgs`, `nav_msgs`, `pcl_conversions`, `tf2`
- `livox_ros_driver2` (CustomMsg 消息定义)
- Eigen3, PCL, OpenMP

---

## 8. src/navigation/livox_ros_driver2 — Livox LiDAR 驱动

**语言**: C++14 | **构建系统**: `ament_cmake` | **版本**: 1.2.4
**依赖**: Livox SDK2, PCL, rapidjson | **注册**: `rclcpp_components` 插件

### 8.1 线程架构 (4线程)

| 线程 | 运行位置 | 职责 |
|------|---------|------|
| **SDK回调线程** | Livox SDK2 内部 | 接收原始以太网UDP数据, 调用 `OnLivoxLidarPointCloudCallback` |
| **RawDataProcess 线程** | PubHandler | 原始数据→RawPacket→标准PointXyzlt, 外参变换, 帧组装 |
| **PointCloudDataPollThread** | DriverNode | 从 LidarDataQueue (无锁环形队列) 取数据→转ROS消息→publish |
| **IMUDataPollThread** | DriverNode | 从 LidarImuDataQueue 取数据→转 ROS Imu→publish |

### 8.2 数据流详解

```
以太网UDP包
  → Livox SDK2 (内部线程)
    → PubHandler::OnLivoxLidarPointCloudCallback()
      ├─ IMU数据: → extrinsic_global旋转 → LidarImuDataQueue → imu_semaphore
      │    → ImuDataPollThread → publish(Imu)
      └─ 点云数据: RawPacket → raw_packet_queue_ (mutex+condition_variable)
           → RawDataProcess线程:
             → LidarPubHandler::PointCloudProcess()
               → CartesianHigh/Low/Spherical 处理
               → 外参旋转+平移
               → CheckTimer() → 帧组装
                 → LidarCommonCallback::OnLidarPointClounCb()
                   → LidarDataQueue::QueuePushAny() (无锁环形队列)
                     → pcd_semaphore_.Signal()
                       → PointCloudDataPollThread:
                         → PublishPointcloud2/CustomMsg
```

### 8.3 无锁队列设计 (LidarDataQueue)

**结构** (`comm.h`):
```cpp
typedef struct {
    StoragePacket * storage_packet;   // 预分配数组
    volatile uint32_t rd_idx;         // 单调递增读指针
    volatile uint32_t wr_idx;         // 单调递增写指针
    uint32_t mask;                    // size - 1 (取模用)
    uint32_t size;                    // 2的幂
} LidarDataQueue;
```

- **SPSC**: 仅 RawDataProcess 线程写, PointCloudDataPollThread 读, 无锁
- **容量**: 根据发布频率计算, 约可存 0.5 秒数据
- **满条件**: `(wr_idx - rd_idx) > mask`
- **空条件**: `rd_idx == wr_idx`

### 8.4 消息格式

| 格式类型 | xfer_format值 | 消息类型 | 每点字节数 |
|----------|:---:|------|:---:|
| PointCloud2 | 0 | `sensor_msgs/PointCloud2` (7字段: x,y,z,intensity,tag,line,timestamp) | 26 |
| CustomMsg | 1 | `livox_ros_driver2/CustomMsg` (header,timebase,point_num,lidar_id,CustomPoint[]) | 18 |
| AllMsg | 4 | 同时发布上述两种 | — |
| PclPxyziMsg | 2 | **ROS2 不支持** (三个方法直接return+warning) | — |

### 8.5 IMU 数据路径

1. SDK 回调中以 `kLivoxLidarImuData` 类型到达
2. `PubHandler` 对 gyro/acc 应用 `extrinsic_global.rotation`
3. 立即调用 `LidarCommonCallback::LidarImuDataCallback()` → `Lds::StorageImuData()`
4. 推入 `LidarImuDataQueue` (mutex保护的 std::list)
5. `ImuDataPollThread` 被 `imu_semaphore` 唤醒
6. `Lddc::PublishImuData()` → `sensor_msgs::Imu` → publish

### 8.6 关键类

| 类 | 文件 | 职责 |
|----|------|------|
| `DriverNode` | driver_node.h/cpp | ROS节点, 启动/停止所有线程 |
| `Lddc` | lddc.h/cpp | 数据分发中心: 点云/IMU→ROS主题 |
| `Lds` | lds.h/cpp | LiDAR数据源抽象基类 (管理32个LiDAR设备) |
| `LdsLidar` | lds_lidar.h/cpp | 真实Livox LiDAR实现 (单例) |
| `PubHandler` | pub_handler.h/cpp | 原始数据→标准格式转换 (单例) |
| `LidarPubHandler` | pub_handler.h/cpp | 每LiDAR的点云累积+外参应用 |
| `LidarDataQueue` | comm.h | SPSC无锁环形队列 |
| `Semaphore` | semaphore.h/cpp | mutex+condition_variable 信号量 |
| `CacheIndex` | cache_index.h/cpp | LiDAR handle→数组索引映射 |
| `LidarImuDataQueue` | lidar_imu_data_queue.h/cpp | mutex保护的IMU队列 |
| `ParseCfgFile` | parse_cfg_file.h/cpp | JSON配置解析 (RapidJSON) |
| `LivoxLidarConfigParser` | parse_livox_lidar_cfg.h/cpp | LiDAR特定配置解析 |

### 8.7 ROS 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `xfer_format` | int | 0 | 0=PointCloud2, 1=CustomMsg, 4=AllMsg |
| `multi_topic` | int | 0 | 0=单话题, 1=每雷达独立话题 |
| `data_src` | int | 0 | 数据源 (0=raw, 1=hub, 2=lvx) |
| `publish_freq` | double | 10.0 | 发布频率 Hz |
| `frame_id` | string | "frame_default" | TF帧ID |
| `user_config_path` | string | "" | JSON配置文件路径 |

---

## 9. src/navigation/loam_interface — 帧适配器

**语言**: C++14 | **可执行文件**: `loam_interface_node` | **注册**: rclcpp_component

### 9.1 类 `LoamInterfaceNode` (public rclcpp::Node)

**成员变量:**

| 变量 | 类型 | 说明 |
|------|------|------|
| `pcd_sub_` | Subscription\<PointCloud2\> | 订阅 `registered_scan_topic_` |
| `odom_sub_` | Subscription\<Odometry\> | 订阅 `state_estimation_topic_` |
| `pcd_pub_` | Publisher\<PointCloud2\> | 发布 `"registered_scan"` |
| `odom_pub_` | Publisher\<Odometry\> | 发布 `"lidar_odometry"` |
| `tf_buffer_` | unique_ptr\<tf2_ros::Buffer\> | TF2查询缓冲 |
| `tf_listener_` | unique_ptr\<tf2_ros::TransformListener\> | TF2监听器 |
| `state_estimation_topic_` | string | ROS param: 输入里程计话题 (默认 `"aft_mapped_to_init"`) |
| `registered_scan_topic_` | string | ROS param: 输入点云话题 (默认 `"cloud_registered"`) |
| `odom_frame_` | string | ROS param: 输出坐标系 (默认 `"odom"`) |
| `lidar_frame_` | string | ROS param: LiDAR帧 (默认 `"front_mid360"`) |
| `base_frame_` | string | ROS param: 底盘帧 (默认 `"base_footprint"`) |
| `base_frame_to_lidar_initialized_` | bool | 延迟初始化标志 |
| `tf_odom_to_lidar_odom_` | tf2::Transform | 缓存的静态 base→lidar TF |

### 9.2 算法 (`odometryCallback`)

```
1. 首次调用: 查找 base_frame_→lidar_frame_ 静态TF (超时0.5s)
   → 缓存为 tf_odom_to_lidar_odom_ (即 base→lidar 的外参变换)
2. 将输入里程计 (lidar_odom帧) 变换到 odom 帧:
   tf_odom_to_lidar = tf_odom_to_lidar_odom × tf_lidar_odom_to_lidar
3. 发布变换后的里程计到 /lidar_odometry
4. 点云回调: 类似地变换点云 header.frame_id, 发布到 /registered_scan
```

**功能总结**: 将 point_lio 输出从 `lidar_odom` 坐标系桥接到标准 `odom` 坐标系。

### 9.3 ROS 接口

| 方向 | 话题 | 类型 |
|------|------|------|
| Sub | `state_estimation_topic_` (如 "aft_mapped_to_init") | Odometry |
| Sub | `registered_scan_topic_` (如 "cloud_registered") | PointCloud2 |
| Pub | `registered_scan` | PointCloud2 |
| Pub | `lidar_odometry` | Odometry |

### 9.4 线程模型

单线程。两个订阅回调在 ROS executor 线程串行执行。无内部线程、无mutex。

---

## 10. src/navigation/sensor_scan_generation — 传感器同步

**语言**: C++14 | **可执行文件**: `sensor_scan_generation_node`

### 10.1 类 `SensorScanGenerationNode` (public rclcpp::Node)

**成员变量:**

| 变量 | 类型 | 说明 |
|------|------|------|
| `odom_sub_` | Subscriber\<Odometry\> | 订阅 `"lidar_odometry"` (BEST_EFFORT QoS) |
| `pcd_sub_` | Subscriber\<PointCloud2\> | 订阅 `"registered_scan"` (BEST_EFFORT QoS) |
| `sync_` | message_filters::Synchronizer | **ApproximateTime** 策略 (队列=50, 容许~0.1s) |
| `pcd_pub_` | Publisher\<PointCloud2\> | 发布 `"sensor_scan"` |
| `odom_pub_` | Publisher\<Odometry\> | 发布 `"odometry"` (带速度) |
| `tf_broadcaster_` | TransformBroadcaster | 广播 `odom→base_footprint` |
| `lidar_frame_` / `base_frame_` / `robot_base_frame_` | string | ROS params: 坐标系名 |
| `prev_position_` / `prev_orientation_` | Point / Quaternion | 用于速度有限差分 |
| `prev_time_` | Time | 上次回调时间 |

### 10.2 算法 (`laserCloudAndOdometryHandler`)

```
1. 将输入 odometry 转为 tf_odom_to_lidar (odom→lidar 位姿)
2. 查找 lidar→robot_base (如 body) TF → 缓存
3. 查找 lidar→base_frame (如 base_footprint) TF → 缓存
4. 计算 odom→base_footprint = odom→lidar × lidar→base_footprint
5. 计算 odom→body = odom→lidar × lidar→body
6. 广播 odom→base_footprint TF
7. 发布 odom→body 里程计 (含有限差分速度):
   v_linear = (current_pos - prev_pos) / dt
   v_angular = quat_diff.axis × quat_diff.angle / dt
8. 将点云从 lidar_odom 帧反变换到 lidar 本地帧, 发布为 sensor_scan
```

### 10.3 ROS 接口

| 方向 | 话题 | 类型 | QoS |
|------|------|------|-----|
| Sub | `lidar_odometry` | Odometry | BEST_EFFORT |
| Sub | `registered_scan` | PointCloud2 | BEST_EFFORT |
| Pub | `sensor_scan` | PointCloud2 | 默认 |
| Pub | `odometry` | Odometry | 默认 |
| TF | odom→base_footprint | — | 按回调频率 |

### 10.4 线程模型

单线程。message_filters 的 Synchronizer 在 ApproximateTime 匹配后调用回调。无内部线程。

---

## 11. src/navigation/small_gicp_relocalization — 全局重定位

**语言**: C++14 + OpenMP | **可执行文件**: `small_gicp_relocalization_node`

### 11.1 类 `SmallGicpRelocalizationNode` (public rclcpp::Node)

**成员变量:**

| 变量 | 类型 | 说明 |
|------|------|------|
| `registered_scan_sub_` | Subscription\<PointCloud2\> | 订阅实时点云 |
| `initial_pose_sub_` | Subscription\<PoseWithCovarianceStamped\> | 订阅RViz初始位姿 |
| `tf_broadcaster_` | TransformBroadcaster | 广播 map→odom TF |
| `global_map_` | pcl::PointCloud\<PointXYZ\>::Ptr | 加载的先验全局地图 |
| `target_kdtree_` | pcl::KdTreeFLANN\<PointXYZ\> | 全局地图KD-tree |
| `accumulated_cloud_` | pcl::PointCloud\<PointXYZ\> | 累积的实时点云 |
| `result_t_` | Eigen::Isometry3d | GICP配准结果 |
| `is_initialized_` | bool | 全局地图是否加载成功 |

### 11.2 核心算法

**`loadGlobalMap(file_name)`**:
1. `pcl::io::loadPCDFile()` 加载 PCD
2. 应用 `lidar_odom→odom` 逆变换 (`odom_to_lidar_odom` param)
3. 降采样: VoxelGrid (leaf = `global_leaf_size`, 默认0.25m)
4. 构建 KD-tree

**`registeredPcdCallback(msg)`**: 将实时点云追加到 `accumulated_cloud_`

**`performRegistration()`** (2Hz 定时器):
1. 累积点云降采样 (leaf = `registered_leaf_size`, 默认0.25m)
2. `small_gicp::estimate_covariances_omp()` — 并行协方差估计 (OpenMP)
3. GICP 配准: 源→目标, 初始猜测 = 上次结果, 10次迭代
4. 更新 `result_t_`
5. 清空累积点云

**`publishTransform()`** (20Hz 定时器): 广播 `map→odom` TF = `result_t_`

**`initialPoseCallback(msg)`**: 处理 RViz `2D Pose Estimate` → 设置 `result_t_` 为手动位姿

### 11.3 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `num_threads` | int | 4 | OpenMP线程数 |
| `num_neighbors` | int | 20 | 协方差估计邻域数 |
| `global_leaf_size` | double | 0.25 | 全局地图降采样 (m) |
| `registered_leaf_size` | double | 0.25 | 实时点云降采样 (m) |
| `max_dist_sq` | double | 1.0 | 对应点最大距离平方 (m²) |
| `prior_pcd_file` | string | "" | 先验PCD文件路径 |
| `init_pose` | double[6] | zeros | 初始位姿 [x,y,z,roll,pitch,yaw] |

### 11.4 线程模型

单 ROS executor 线程 + OpenMP 并行 (协方差估计)。两个定时器 (2Hz + 20Hz) 共享同一线程。

---

## 12. src/navigation/terrain_analysis — 近场地形分析

**语言**: C++14 | **单文件实现**: `src/terrainAnalysis.cpp` (683行)
**可执行文件**: `terrainAnalysis` | **无类, 纯全局变量+函数**

### 12.1 全局常量

| 常量 | 值 | 含义 |
|------|-----|------|
| `terrainVoxelSize` | 1.0 m | 地形累积体素尺寸 |
| `terrainVoxelWidth` | 21 | 体素网格宽度 (奇数, 车辆居中) |
| `terrainVoxelHalfWidth` | 10 | (21-1)/2 |
| `kTerrainVoxelNum` | 441 | 21×21 |
| `planarVoxelSize` | 0.2 m | 输出平面体素尺寸 |
| `planarVoxelWidth` | 51 | 平面网格宽度 |
| `planarVoxelHalfWidth` | 25 | (51-1)/2 |
| `kPlanarVoxelNum` | 2601 | 51×51 |

**覆盖范围**: 21×1.0m = 21m 体素网格; 51×0.2m = 10.2m 输出平面

### 12.2 全局变量

**可配置参数** (30+, 从 ROS param 加载):

| 变量 | 类型 | 默认值 | 含义 |
|------|------|--------|------|
| `scanVoxelSize` | double | 0.02 | PCL VoxelGrid 降采样尺寸 |
| `decayTime` | double | 0.5 | 点云老化时间 (s), 超时丢弃 |
| `noDecayDis` | double | 0.0 | 此距离内点永不过期 |
| `clearingDis` | double | 15.0 | 此距离外点清除 |
| `useSorting` | bool | true | true=分位数地面, false=最低点地面 |
| `quantileZ` | double | 0.2 | 地面分位数 (useSorting=true) |
| `vehicleHeight` | double | 0.6 | 车辆高度, 高于此的点丢弃 |
| `minRelZ` / `maxRelZ` | double | -0.75/0.25 | 有效Z范围 (相对地面) |
| `clearDyObs` | bool | true | 动态障碍物检测 |
| `noDataObstacle` | bool | false | 无数据区域填障碍物 |
| `voxelPointUpdateThre` | int | 100 | 体素处理触发点数 |
| `voxelTimeUpdateThre` | double | 1.0 | 体素处理触发时间 (s) |

**点云数组** (全局, pcl::PointCloud\<pcl::PointXYZI\>):
- `laserCloud`, `laserCloudCrop`, `laserCloudDwz`
- `terrainCloud`, `terrainCloudElev`
- `terrainVoxelCloud[kTerrainVoxelNum]` (441个体素各存独立点云)

**状态数组**:
- `terrainVoxelUpdateNum[441]` — 各体素累积点数
- `terrainVoxelUpdateTime[441]` — 各体素更新时间
- `planarVoxelElev[2601]` — 平面体素地面高度
- `planarVoxelEdge[2601]` — 平面体素边缘标记
- `planarVoxelDyObs[2601]` — 动态障碍物标记
- `planarPointElev[2601]` — 平面点高程

### 12.3 主循环算法 (100Hz, `main()` 函数)

**Phase 1 - 体素网格滑动** (4个while循环):
- 当车辆移动超过1个体素边界时, 整行/整列移位 (X+, X-, Y+, Y-)
- 新进入的体素清零

**Phase 2 - 堆叠激光点**:
- 将裁剪后的点按车辆相对位置 floor 分配到 21×21 体素

**Phase 3 - 体素处理**:
- 触发条件: 点数≥voxelPointUpdateThre(100) 或 时间≥voxelTimeUpdateThre(2s) 或 clearingCloud
- PCL VoxelGrid 降采样
- 老化过滤: 保留 decayTime 内点, noDecayDis 内点永不过期

**Phase 4 - 组装中央地形云**:
- 取中央 11×11 体素 (~10.5m), 合并为 terrainCloud

**Phase 5 - 地面估计**:
- 点分配到 51×51 平面体素 (3×3邻域扩展)
- 动态障碍物检测: 计数投票 (坐标变换 yaw→pitch→roll) + 当前帧清除
- 地面高度: 排序取 quantileZ 分位数 (useSorting=true) 或最小值

**Phase 6 - 高程计算**:
- disZ = point.z - groundElevation
- 过滤 0 ≤ disZ < vehicleHeight → intensity = disZ

**Phase 7 - 无数据障碍物填充** (noDataObstacle=true):
- 低点数体素→边缘→迭代扩张→合成 obstacle point (vehicleHeight/2 高度)

**Phase 8 - 发布**: terrainCloudElev → `terrain_map` (frame="odom")

### 12.4 ROS 接口

| 方向 | 话题 | 类型 | 队列 |
|------|------|------|------|
| Sub | `lidar_odometry` | Odometry | 5 |
| Sub | `registered_scan` | PointCloud2 | 5 |
| Sub | `joy` | Joy | 5 (按钮[5]→重置) |
| Sub | `map_clearing` | Float32 | 5 |
| Pub | `terrain_map` | PointCloud2 | 2 |

### 12.5 线程模型

单线程。100Hz while 循环 + ros::spinOnce()。所有处理在 main() 主循环中同步完成。

---

## 13. src/navigation/terrain_analysis_ext — 远场地形分析

**语言**: C++14 | **单文件实现**: `src/terrainAnalysisExt.cpp` (557行)
**可执行文件**: `terrainAnalysisExt`

### 13.1 与 terrain_analysis 的关键差异

| 方面 | terrain_analysis (近场) | terrain_analysis_ext (远场) |
|------|------------------------|---------------------------|
| terrainVoxelSize | 1.0 m | **2.0 m** |
| terrainVoxelWidth | 21 (441 cells) | **41 (1681 cells)** |
| planarVoxelSize | 0.2 m | **0.4 m** |
| planarVoxelWidth | 51 (2601 cells) | **101 (10201 cells)** |
| 覆盖范围 | ~10.5m半径 | ~41m半径 |
| 中央装配区 | 11×11 (~10.5m) | 21×21 (~42m) |
| 降采样 | 0.05 m | **0.1 m** (scanVoxelSize=0.03 vs 0.02) |
| 老化时间 | 0.5 s | **0.2 s** (decayTime) |
| 无老化区 | 0.0 m | **0.0 m** |
| 高度边界 | minRelZ=-0.75, maxRelZ=0.25 | lowerBoundZ=-1.5, upperBoundZ=0.5 |
| 地面分位数 | 0.2 | **0.1** |
| 动态障碍物 | ✅ clearDyObs | ❌ 无 |
| 地面抬升限制 | ✅ limitGroundLift | ❌ 无 |
| 考虑下降 | 可选 considerDrop | ✅ 始终 fabs |
| 无数据填充 | ✅ noDataObstacle | ❌ 无 |
| 连通性分析 | ❌ 无 | ✅ **BFS checkTerrainConn** |
| 近场融合 | N/A (发布源) | ✅ 订阅 terrain_map |
| 输出 | terrain_map | terrain_map_ext (4m内近场，4m外远场) |

### 13.2 BFS 地形连通性 (核心新增功能)

**目的**: 过滤树冠/桥梁等悬垂结构 (上方点云与地面不连通 → 天花板, 不是可行走区域)

**算法** (BFS, `checkTerrainConn=true` 时生效):
1. 从车辆中心平面体素开始 BFS
2. 邻居体素 |高程差| < `terrainConnThre` (0.5m) → 标记为已连通 (2)
3. 体素 |高程差| > `ceilingFilteringThre` (2.0m) → 标记为天花板 (-1)
4. BFS 终止条件: 队列为空或所有可达体素已访问
5. 结果: 标记2=与地面连通 (保留), 标记-1=天花板 (丢弃), 标记0=未访问 (丢弃)

### 13.3 近场/远场融合

- 订阅 `terrain_analysis` 发布的 `terrain_map`
- 输出: distance ≤ `localTerrainMapRadius` (4.0m) → 直接复制近场点
- distance > 4.0m → 使用远场分析结果 (含 BFS 连通性过滤)

### 13.4 ROS 接口

| 方向 | 话题 | 类型 | 队列 |
|------|------|------|------|
| Sub | `lidar_odometry` | Odometry | 5 |
| Sub | `registered_scan` | PointCloud2 | 5 |
| Sub | `joy` | Joy | 5 |
| Sub | `cloud_clearing` | Float32 | 5 |
| **Sub** | **`terrain_map`** | **PointCloud2** | **2** |
| Pub | `terrain_map_ext` | PointCloud2 | 2 |

---

## 14. src/navigation/pointcloud_to_laserscan — 点云转激光

**语言**: C++14 | **版本**: 2.0.1 (ROS1 perception_pcl 移植版)

### 14.1 类 `PointCloudToLaserScanNode` (public rclcpp::Node)

**算法** (`cloudCallback`):
1. 创建 LaserScan 消息, 设置角度参数 (angle_min, angle_max, angle_increment)
2. `ranges_size = ceil((angle_max - angle_min) / angle_increment)`
3. 初始化 ranges = inf (use_inf=true) 或 range_max+epsilon
4. 可选 TF 变换到 target_frame (transform_tolerance=0.01s)
5. 逐点遍历:
   - NaN/高度/强度/距离/角度过滤
   - `index = (atan2(y, x) - angle_min) / angle_increment`
   - `ranges[index] = min(ranges[index], hypot(x, y))`
6. 发布 LaserScan

**延迟订阅机制**: `subscriptionListenerThreadLoop` 每100ms检查是否有 scan 订阅者, 无则退订 cloud_in (节省资源)

### 14.2 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `target_frame` | "" | 目标坐标系 |
| `min_height` | -inf | Z下界 |
| `max_height` | +inf | Z上界 |
| `angle_min` | -pi | 扫描起始角 |
| `angle_max` | pi | 扫描终止角 |
| `angle_increment` | pi/180 (1°) | 角度分辨率 |
| `range_min` | 0.0 | 最小距离 |
| `range_max` | +inf | 最大距离 |
| `use_inf` | true | 超范围用inf |

**订阅**: `cloud_in` (remappable) | **发布**: `scan` (remappable)

---

## 15. src/navigation/ign_sim_pointcloud_tool — 仿真点云转换

**语言**: C++14 | **可执行文件**: `ign_sim_pointcloud_tool_node`

### 15.1 类 `PointCloudConverter` (public rclcpp::Node)

**功能**: 将 Ignition Gazebo 通用 XYZ 点云转换为含 ring 和 time 字段的 Velodyne 兼容格式。

**算法** (`lidarHandle`):
1. `pcl::fromROSMsg` → PointCloud\<PointXYZ\>
2. 逐点计算:
   - `vertical_angle = atan2(z, sqrt(x²+y²)) × 180/π`
   - `row_id = (vertical_angle + ang_bottom) / ang_res_y`
   - 如果 0 ≤ row_id < n_scan: ring = row_id
   - `time = (point_id % horizon_scan) × 0.1 / horizon_scan` (模拟10Hz旋转)
3. 发布 PointCloud2 (含 x/y/z/intensity/ring/time)

**发布/订阅**: Sub `pcd_topic_` (PointCloud2) → Pub `velodyne_points` (PointCloud2, SensorDataQoS)

---

## 16. src/navigation/teleop_twist_joy — 手柄遥控

**语言**: C++14 | **可执行文件**: `teleop_twist_joy_node`

### 16.1 类 `TeleopTwistJoyNode` (public rclcpp::Node)

**两种控制模式:**

| 模式 | 行为 |
|------|------|
| `manual_control` | joy→Twist/TwistStamped→`cmd_vel` (全向: x, y, yaw 轴) |
| `auto_control` | joy x,y→base_frame→TF变换到map→NavigateToPose action goal (4Hz限速) |

**死手开关**: `enable_button` (默认5), 松开→立即发零速

**加速模式**: `enable_turbo_button` (默认-1禁用), 用 turbo 比例系数

### 16.2 `joyCallback(joy_msg)` 分支逻辑

```
if (turbo_button被按下 && enable_turbo_button_ >= 0):
    sendCmdVelMsg(joy_msg, "turbo")
elif (!require_enable_button || enable_button被按下):
    sendCmdVelMsg(joy_msg, "normal")
elif (enable_button刚松开):
    sendZeroCommand()
    sent_disable_msg_ = false
```

### 16.3 `sendCmdVelMsg()` 分支逻辑

```
if (control_mode == "manual_control"):
    if (publish_stamped_twist):
        构建 TwistStamped (frame_id=robot_base_frame)
    else:
        构建 Twist
    fillCmdVelMsg(joy_msg, which_map, cmd_vel_msg)
    publish(cmd_vel_msg)
else (auto_control):
    sendGoalPoseAction(joy_msg, which_map)
```

### 16.4 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `publish_stamped_twist` | false | 发 TwistStamped (含header) |
| `robot_base_frame` | `"base_link"` | 机器人帧 |
| `control_mode` | `"manual_control"` | manual/auto |
| `require_enable_button` | true | 死手开关 |
| `enable_button` | 5 | 使能按钮索引 |
| `enable_turbo_button` | -1 | 加速按钮 (-1禁用) |
| `inverted_reverse` | false | 倒车航向反转 |
| `axis_chassis` | {x:5, y:-1, yaw:-1} | 轴映射 (-1=未使用) |
| `scale_chassis` | {x:0.5, y:0.0, yaw:0.0} | 正常比例 |
| `scale_chassis_turbo` | {x:1.0, y:0.0, yaw:0.0} | 加速比例 |

### 16.5 线程模型

单线程。joyCallback 在 ROS executor 线程。无内部线程、无mutex。

---

## 17. src/simulation/nav2_loopback_sim — 回路仿真器

**语言**: Python | **版本**: 1.4.0 | **类**: `LoopbackSimulator(Node)`

### 17.1 核心闭环

```
Nav2控制器→cmd_vel
  → cmdVelCallback() 存储速度
  → timerCallback() (update_dur=0.01s间隔):
     积分速度→更新odom→base_link TF
     发布 Odometry + TF
  → publishLaserScan() (scan_publish_dur=0.1s间隔):
     获取laser世界位姿 (map→odom→base→laser TF链)
     对静态地图做射线追踪→生成LaserScan
  → Nav2接收/odom, /scan, TF→更新代价地图→规划→新cmd_vel
```

### 17.2 初始位姿状态机

1. **首次 `initialpose`**: 设置 map→odom, 重置 odom→base, 启动主 timer
2. **后续 `initialpose`** (重定位): 保持 odom→base 不变, 仅调 map→odom (通过矩阵分解: `T_map_odom = T_map_base × inv(T_odom_base)`)

### 17.3 射线追踪算法 (`getLaserScan`)

- 每条射线从 laser 世界坐标出发, 沿角度方向以 0.5 像素步长遍历
- Bresenham 线迭代 (LineIterator)
- 检测 OccupancyGrid 值 ≥ 60 → 记录距离 (Euclidean, 按分辨率缩放)
- 超地图边界 → skip 当前射线, ranges[i] = inf

### 17.4 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `update_duration` | 0.01 | 速度积分周期 (s) |
| `base_frame_id` | `"base_footprint"` | 底盘帧 |
| `map_frame_id` | `"map"` | 地图帧 |
| `odom_frame_id` | `"odom"` | 里程计帧 |
| `scan_frame_id` | `"base_scan"` | 激光帧 |
| `enable_stamped_cmd_vel` | true | 使用 TwistStamped |
| `scan_publish_dur` | 0.1 | 激光发布周期 (s) |
| `scan_range_min/max` | 0.05/30.0 | 激光距离范围 |
| `scan_angle_min/max/increment` | -π/π/0.0261 | 扫描角度参数 |
| `publish_map_odom_tf` | true | 发布 map→odom TF |
| `publish_clock` | true | 发布 /clock (10Hz) |

### 17.5 ROS 接口

| 类型 | 话题 | 消息类型 | QoS |
|------|------|---------|-----|
| Sub | `initialpose` | PoseWithCovarianceStamped | 默认 |
| Sub | `cmd_vel` | Twist/TwistStamped | 默认 |
| Pub | `odom` | Odometry | 默认 |
| Pub | `scan` | LaserScan | BEST_EFFORT, VOLATILE |
| Pub | `/clock` | Clock | 默认 (条件) |
| Client | `/map_server/map` | GetMap | — |

### 17.6 工具模块

**`tf_compat.py`**: tf_transformations 兼容层。`quaternion_from_euler`, `euler_from_quaternion`, `quaternion_multiply`, `quaternion_matrix` 等。

**`utils.py`**: 几何工具。`worldToMap`, `getMapOccupancy`, `transformStampedToMatrix`, `addYawToQuat`。

---

## 18. src/tools/pcd2pgm — PCD转PGM地图

**语言**: C++14 | **可执行文件**: `pcd2pgm_node`

### 18.1 类 `Pcd2PgmNode` (public rclcpp::Node)

**处理管道**:
```
PCD文件 → loadPCDFile
  → applyTransform (odom→lidar 逆变换)
  → passThroughFilter (Z轴过滤, flag_pass_through 控制方向)
  → radiusOutlierFilter (半径离群点移除)
  → setMapTopicMsg (点云→OccupancyGrid: x/y bounds→grid→occupied=100)
  → publishCallback (1Hz: PointCloud2 + OccupancyGrid)
```

**参数**: pcd_file, thre_z_min/max (0.5/2.0), flag_pass_through (false), thre_radius (0.5), map_resolution (0.05), thres_point_count (10), map_topic_name ("map"), odom_to_lidar_odom ([0,0,0,0,0,0])

**发布**: OccupancyGrid (TRANSIENT_LOCAL QoS), `pcd_cloud` (PointCloud2)

---

## 19. src/tools/rosbag2_composable_recorder — 数据录制

**语言**: C++14 | **类**: `ComposableRecorder` (extends `rosbag2_transport::Recorder`)

### 19.1 功能

可组合的 rosbag2 录制节点, 支持零拷贝进程内通信。

**服务**:
| 服务名 | 类型 | 功能 |
|--------|------|------|
| `/start_recording` | Trigger | 开始录制 |
| `/stop_recording` | Trigger | 停止录制 |

**参数**: bag_name (""), bag_prefix ("rosbag2_"), topics ([]), storage_id ("sqlite3"), max_cache_size (100MB), record_all (false), start_recording_immediately (false)

### 19.2 录制管理

- `startRecording()`: bag_name 非空→固定路径, 否则→bag_prefix+时间戳, 调用 `record()`
- `stopRecording()`: 调用 `stop()`, 设置 isRecording_=false

---

## 20. src/dependencies/ — 第三方依赖

| 目录 | 用途 |
|------|------|
| `BehaviorTree.ROS2/` | 行为树引擎 ROS2 封装 (btcpp_ros2), 提供 BT 节点基类 |
| `joint_state_publisher/` | 关节状态发布器 (URDF 可视化) |
| `sdformat_tools/` | SDFormat→URDF 转换工具 |

这些包不在核心导航管线中, 仅在仿真/可视化时使用。

---

## 21. 完整ROS话题索引

### 21.1 传感器话题

| 话题 | 类型 | 发布者 | 说明 |
|------|------|--------|------|
| `livox/lidar` | CustomMsg/PointCloud2 | livox_ros_driver2 | LiDAR 点云 |
| `livox/imu` | Imu | livox_ros_driver2 | 内置 IMU |
| `aft_mapped_to_init` | Odometry | point_lio | 里程计 (lidar_odom帧) |
| `cloud_registered` | PointCloud2 | point_lio | 去畸变点云 (lidar_odom帧) |
| `lidar_odometry` | Odometry | loam_interface | 里程计 (odom帧) |
| `registered_scan` | PointCloud2 | loam_interface | 点云 (odom帧) |
| `odometry` | Odometry | sensor_scan_generation | 带速度的里程计 |
| `sensor_scan` | PointCloud2 | sensor_scan_generation | LiDAR 本地帧点云 |

### 21.2 感知话题

| 话题 | 类型 | 发布者 | 说明 |
|------|------|--------|------|
| `terrain_map` | PointCloud2 | terrain_analysis | 近场地形高程图 (intensity=高度差) |
| `terrain_map_ext` | PointCloud2 | terrain_analysis_ext | 远场+近场地形 |
| `obstacle_scan` | LaserScan | pointcloud_to_laserscan | 2D 激光 (用于 slam_toolbox) |

### 21.3 导航话题

| 话题 | 类型 | 发布者 | 说明 |
|------|------|--------|------|
| `map` | OccupancyGrid | map_server | 静态地图 |
| `cmd_vel` | Twist | velocity_smoother | **最终底盘速度指令** |
| `cmd_vel_nav2_result` | Twist | controller_server | 控制器原始输出 (内部) |
| `plan` | Path | planner_server | 全局路径 |
| `local_plan` | Path | omni_pid_pursuit_controller | 局部变换后路径 |
| `lookahead_point` | PointStamped | omni_pid_pursuit_controller | 预瞄点 |
| `local_costmap/costmap` | OccupancyGrid | local_costmap | 局部代价地图 |
| `global_costmap/costmap` | OccupancyGrid | global_costmap | 全局代价地图 |

### 21.4 控制/工具话题

| 话题 | 类型 | 发布者 | 说明 |
|------|------|--------|------|
| `joy` | Joy | joy_node | 手柄原始数据 |
| `stop_flag` | Bool | HoldStopFlag (nav2_plugins) | 停止标志 |
| `goal_pose` | PoseStamped | PublishNavGoal (nav2_plugins) | 导航目标 |
| `initialpose` | PoseWithCovarianceStamped | RViz/用户 | 初始位姿 |
| `/clock` | Clock | loopback_simulator | 仿真时钟 |

### 21.5 ROS Action

| Action名 | 类型 | 客户端 | 服务端 |
|---------|------|--------|--------|
| `/navigate_through_poses` | NavigateThroughPoses | SendNavThroughPoses | bt_navigator |
| `/navigate_to_pose` | NavigateToPose | SendNav2Goal, teleop_twist_joy | bt_navigator |

### 21.6 ROS Service

| Service名 | 类型 | 客户端 | 服务端 |
|----------|------|--------|--------|
| `/start_recording` | Trigger | start_recording.py | rosbag2_composable_recorder |
| `/stop_recording` | Trigger | — | rosbag2_composable_recorder |
| `/map_server/map` | GetMap | loopback_simulator | map_server |
| `local_costmap/get_costmap` | GetCostmap | BackUpFreeSpace | local_costmap |

---

## 22. 完整ROS参数索引

### 22.1 导航参数 (Nav2)

| 命名空间 | 关键参数 |
|---------|---------|
| `controller_server.FollowPath` | kp=1.5, ki=0.05, kd=0.1, v_linear=±1.5, v_angular=±1.5, lookahead=1.0m, curvature_min=1.0, enable_rotation=true |
| `planner_server.GridBased` | SmacPlannerHybrid, DUBIN model, tolerance=0.25, min_turning_radius=0.0 |
| `local_costmap` | 6m×6m, 0.05m, robot_radius=0.45, inflation=0.6, intensity_voxel_layer |
| `global_costmap` | map帧, robot_radius=0.45, inflation=0.8 |
| `velocity_smoother` | OPEN_LOOP, max_vel=[1.2,1.2,1.2], max_accel=[0.8,0.8,1.0] |
| `behavior_server` | spin/backup (标准)/drive_on_heading/wait |

### 22.2 感知参数

| 命名空间 | 关键参数 |
|---------|---------|
| `point_lio` | lidar_type=1(Livox), scan_line=4, ivox_grid=0.5, plane_thr=0.1, extrinsic_T/I, gravity=[0,0,-9.81] |
| `loam_interface` | state_estimation="aft_mapped_to_init", registered_scan="cloud_registered", odom/base/lidar frames |
| `sensor_scan_generation` | lidar="front_mid360", base="base_footprint", robot_base="body" |
| `terrain_analysis` | scanVoxelSize=0.02, decayTime=0.5, vehicleHeight=0.6 |
| `terrain_analysis_ext` | scanVoxelSize=0.03, checkTerrainConn=false |
| `small_gicp_relocalization` | num_threads=4, global_leaf=0.15, registered_leaf=0.05 |

### 22.3 工具/仿真参数

| 命名空间 | 关键参数 |
|---------|---------|
| `loopback_simulator` | update_dur=0.01, scan_range=30m, frame IDs |
| `pcd2pgm` | pcd_file, thre_z, map_resolution=0.05 |
| `rosbag2_composable_recorder` | bag_prefix, topics, storage_id="sqlite3" |

---

## 23. TF坐标变换树

### 23.1 完整 TF 链 (四足)

```
map
  └→ odom                         ← small_gicp_relocalization (20Hz)
       └→ base_footprint           ← sensor_scan_generation (动态, 按里程计频率)
            └→ base_link           ← URDF 静态 (Z偏移 ~0.30m)
                 └→ body           ← URDF 静态 (identity)
                      └→ front_mid360  ← URDF 静态 (LiDAR 外参)
                      └→ fl_hip, fr_hip, rl_hip, rr_hip  ← URDF 静态 (腿关节占位)
```

### 23.2 各 TF 广播者

| TF变换 | 类型 | 广播者 | 更新频率 |
|--------|------|--------|---------|
| map → odom | 动态 | small_gicp_relocalization | 20 Hz |
| map → odom | 动态 | loopback_simulator (仿真) | ~100 Hz |
| odom → base_footprint | 动态 | sensor_scan_generation | 按里程计频率 |
| odom → base_footprint | 动态 | loopback_simulator (仿真) | ~100 Hz |
| base_footprint → base_link | 静态 | robot_state_publisher (URDF) | — |
| base_link → body | 静态 | robot_state_publisher | — |
| body → front_mid360 | 静态 | robot_state_publisher | — |

---

## 24. 文件间依赖关系图谱

### 24.1 包级依赖 (数据流方向)

```
nav2_plugins (BT 决策)
    ↓ depends on
nav2_controller, nav2_planner, nav2_behaviors (Nav2 框架)
    ↓ depends on
sensor_scan_generation (里程计+TF)
    ↓ depends on
loam_interface (帧适配)
    ↓ depends on
point_lio (SLAM 里程计)
    ↓ depends on
livox_ros_driver2 (LiDAR 驱动)
```

横向依赖 (同层, 仅通过 ROS 话题):
```
terrain_analysis ──→ terrain_analysis_ext (话题: terrain_map)
terrain_analysis_ext ──→ pointcloud_to_laserscan (话题: cloud_in→terrain_map_ext)
pointcloud_to_laserscan ──→ Nav2 costmap (话题: obstacle_scan)
small_gicp_relocalization ──→ TF: map→odom
```

### 24.2 关键文件→文件直接依赖

| 消费者文件 | 依赖项 |
|-----------|--------|
| `terrainAnalysisExt.cpp` | ← 订阅 `terrain_analysis` 发布的 `/terrain_map` |
| `sensor_scan_generation.cpp` | ← 订阅 `loam_interface` 发布的 `/lidar_odometry` + `/registered_scan` |
| `loam_interface.cpp` | ← 订阅 `point_lio` 发布的 `/aft_mapped_to_init` + `/cloud_registered` |
| `small_gicp_relocalization.cpp` | ← 订阅 `/registered_scan`, 广播 `map→odom` TF |
| `omni_pid_pursuit_controller.cpp` | ← 使用 `nav2_costmap_2d` 查询代价, `tf2` 变换路径 |
| `nav2_plugins/intensity_voxel_layer.cpp` | ← 继承 `nav2_costmap_2d::ObstacleLayer` |
| `nav2_plugins/back_up_free_space.cpp` | ← 继承 `nav2_behaviors::DriveOnHeading`, 调用 `local_costmap/get_costmap` 服务 |
| `nav2_loopback_sim/loopback_simulator.py` | ← 订阅 `cmd_vel`, 调用 `/map_server/map`, 发布 `odom`+`scan`+TF |

### 24.3 编译依赖链

```
nav_bringup (纯配置, 无编译)
  exec_depend→ 所有其他包

nav2_plugins (14个独立 .so)
  build_depend→ nav2_core, nav2_costmap_2d, BehaviorTree.CPP, tf2

omni_pid_pursuit_controller
  build_depend→ nav2_core, nav2_costmap_2d, tf2, pluginlib

point_lio
  build_depend→ rclcpp, pcl_conversions, tf2, livox_ros_driver2 (CustomMsg), Eigen3, OpenMP

livox_ros_driver2
  build_depend→ LivoxSDK2, PCL, rapidjson, rclcpp_components

loam_interface, sensor_scan_generation, small_gicp_relocalization
  build_depend→ rclcpp, tf2, pcl_ros

terrain_analysis, terrain_analysis_ext
  build_depend→ rclcpp, pcl_ros, tf2

nav2_loopback_sim (Python)
  exec_depend→ rclpy, tf2_ros, nav2_simple_commander
```

---

> **文档版本**: 2026-07-03
> **编译命令**: `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`
> **ROS版本**: Humble Hawksbill (Ubuntu 22.04)
> **包总数**: 16
> **适配状态**: 已完成四足底盘适配 (v2.0.0)，待仿真验证和参数调优
>
> **强制规则再次提醒：后续代码无论进行何种修改、优化，本文档必须同步更新，保证文档与源码实时一致。**
