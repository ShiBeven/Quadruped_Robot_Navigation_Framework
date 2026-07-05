# 四足机器人导航框架 — 分层索引

> 生成时间: 2026-07-05 | 基线提交: `5b3f214`
> 语言: C++ (62%) + Python (22%) + YAML/XML/Config (16%)
> 总文件数: 133 | Tier A: 12 / Tier B: 48 / Tier C: 73
> 源代码行数: ~21,844 (非空行、非注释行，不含第三方库)

> **本文档是分层索引，不替代源代码 — 源代码始终是唯一真理。** 每个条目都带有同步提交哈希，以便立即识别过期内容。

---

## 第一层：架构总览

### 1.1 概述

- **技术栈:** ROS2 Humble + Nav2 + BehaviorTree.CPP + Point-LIO (激光-惯性里程计) + PCL + Eigen + TF2
- **构建系统:** CMake (ament_cmake) + Python setuptools (ament_python)
- **项目目标:** 四足机器人自主导航框架，融合 3D 激光-惯性 SLAM、地形分析、自定义行为树决策和全向路径控制。

### 1.2 目录布局

```
Quadruped_Robot_Navigation_Framework/
├── src/
│   ├── navigation/                          — 所有导航与感知 ROS2 包
│   │   ├── nav_bringup/                     — [入口] 启动文件、配置、行为树
│   │   ├── point_lio/                       — [核心] 3D 激光-惯性里程计与建图
│   │   ├── nav2_plugins/                    — [核心] 自定义 BT 动作/条件/控制节点 + 代价地图图层
│   │   ├── omni_pid_pursuit_controller/     — [核心] 全向纯追踪 PID 控制器
│   │   ├── loam_interface/                  — [桥接] LOAM 到 Nav2 里程计转发器
│   │   ├── small_gicp_relocalization/       — [定位] GICP 扫描-地图重定位
│   │   ├── sensor_scan_generation/          — [桥接] 同步里程计 + 扫描帧变换
│   │   ├── terrain_analysis/                — [感知] 滚动网格地形高度估计与动态障碍物检测
│   │   ├── terrain_analysis_ext/            — [感知] 带连通性过滤的扩展地形分析
│   │   ├── pointcloud_to_laserscan/         — [转换] 3D 点云 → 2D 激光扫描投影
│   │   ├── livox_ros_driver2/              — [驱动] Livox 激光雷达 ROS2 驱动
│   │   ├── teleop_twist_joy/               — [遥控] 手柄手动控制
│   │   └── ign_sim_pointcloud_tool/         — [工具] Gazebo Ignition 点云转换
│   ├── simulation/                          — 仿真基础设施
│   │   └── nav2_loopback_sim/              — [仿真] 无摩擦回路仿真，带射线投射扫描
│   ├── tools/                               — 实用工具
│   │   ├── pcd2pgm/                        — [工具] PCD 点云 → 2D 占据栅格地图
│   │   └── rosbag2_composable_recorder/    — [工具] 可组合 rosbag2 录制器，服务控制
│   └── dependencies/                        — 第三方依赖包
│       ├── BehaviorTree.ROS2/              — BehaviorTree.CPP 的 ROS2 绑定
│       ├── joint_state_publisher/          — 机器人关节状态发布器
│       └── sdformat_tools/                 — SDF↔URDF 转换工具
├── docs/                                    — 项目文档（本索引）
├── scripts/                                 — （空 — 预留）
└── patterns/                                — （空 — 预留）
```

### 1.3 模块依赖拓扑图

```
                        ┌──────────────────────┐
                        │    nav_bringup        │ (启动入口)
                        └──────┬───┬───┬───────┘
                               │   │   │
              ┌────────────────┘   │   └──────────────────┐
              │                    │                      │
              ▼                    ▼                      ▼
    ┌─────────────────┐  ┌──────────────────┐  ┌──────────────────────┐
    │   point_lio      │  │  nav2_plugins    │  │  omni_pid_pursuit    │
    │  (SLAM 核心)     │  │  (BT + 代价地图) │  │  _controller         │
    │  激光-惯性里程计  │  │                  │  │  (路径跟踪)          │
    └────────┬─────────┘  └────────┬─────────┘  └──────────────────────┘
             │                     │
             │ 发布:               │ 使用: Nav2 BT 节点
             │ aft_mapped_to_init  │ intensity_voxel_layer
             │ cloud_registered    │ back_up_free_space
             │                     │
    ┌────────┴─────────┐           │
    │ terrain_analysis  │◄──────────┘ (订阅 cloud_registered)
    │ terrain_analysis_ext│
    └────────┬─────────┘
             │ 发布: terrain_map, terrain_map_ext
             │
    ┌────────┴──────────────┐
    │ pointcloud_to_laserscan│ (terrain_map_ext → obstacle_scan)
    └────────┬──────────────┘
             │
    ┌────────┴──────────┐
    │    slam_toolbox    │ (2D 占据栅格建图)
    └───────────────────┘

    ┌────────────────────────┐     ┌───────────────────────────┐
    │ small_gicp_relocalization│◄───│ point_lio (cloud_registered)│
    │ map→odom TF 广播        │     └───────────────────────────┘
    └────────────────────────┘

    loam_interface ──► sensor_scan_generation ──► terrain_analysis
    (里程计桥接)        (帧变换)                    (点云输入)
```

**双向依赖标注:** `terrain_analysis_ext` 依赖 `terrain_analysis`（订阅 `terrain_map`）；其他地方未检测到循环导入。

### 1.4 核心数据流

1. **SLAM 管线（建图模式）：**
   ```
   Livox 激光雷达 → point_lio (激光-惯性里程计)
     → cloud_registered → terrain_analysis → terrain_map
     → terrain_analysis_ext → terrain_map_ext
     → pointcloud_to_laserscan → obstacle_scan → slam_toolbox → /map
     → aft_mapped_to_init (里程计) → /tf (camera_init→aft_mapped)
   ```

2. **导航管线（自主模式）：**
   ```
   /map (slam_toolbox/map_server) + /odom (point_lio) + /tf
     → small_gicp_relocalization (map→odom 修正)
     → global_costmap (IntensityVoxelLayer, 10m 范围)
     → local_costmap (IntensityVoxelLayer, 5m 范围)
     → SmacPlannerHybrid (全局规划, DUBIN 运动模型)
     → OmniPidPursuitController (路径跟踪, 曲率限速)
     → velocity_smoother → cmd_vel → 机器人运动
   ```

3. **行为树决策循环（3 Hz）：**
   ```
   ComputePathToPose → FollowPath（含恢复: 清除代价地图 + 180° 旋转）
     → [通过 BT 插件] SelectPatrolPath/SendNavThroughPoses/PublishTwist/HoldStopFlag
   ```

### 1.5 入口点

| 场景 | 文件 | 描述 |
|---|---|---|
| 导航（硬件） | [nav_bringup](../modules/nav_bringup.md) `launch/legged_navigation_launch.py` | 完整 Nav2 栈 + 感知 |
| 导航（仿真） | [nav_bringup](../modules/nav_bringup.md) `config/nav2_params.legged_sim.yaml` | 仿真调优参数 |
| 定位（已知地图） | [nav_bringup](../modules/nav_bringup.md) `launch/legged_localization_launch.py` | Point-LIO + map_server + small_gicp |
| SLAM（建图） | [nav_bringup](../modules/nav_bringup.md) `launch/legged_slam_launch.py` | Point-LIO + slam_toolbox |
| 手动遥控 | [teleop_twist_joy](../modules/teleop_twist_joy.md) | 手柄 → cmd_vel |
| 回路仿真 | [nav2_loopback_sim](../modules/nav2_loopback_sim.md) `loopback_simulator.py` | 无摩擦仿真，带射线投射 |

### 1.6 测试覆盖率概览

> 跳过覆盖率 — 未检测到测试基础设施（未传递 `--with-coverage`）。该项目不含测试文件（源码目录中未发现 `*test*`、`*spec*` 模式）。

---

## 阅读路径

- **调试导航故障:** 从 [risk-map.md](risk-map.md) 查看复杂度热点（特别是 1052 行的 `laserMapping.cpp`），然后用 [symbol-index.md](symbol-index.md) 定位符号，最后在第二层模块文档中追踪调用链。
- **修改控制器:** 阅读 `omni_pid_pursuit_controller` 第二层文档，然后查看 `nav_bringup` 配置中的影响范围，以及 `nav2_plugins`（调用控制器的 BT 节点）。
- **添加新激光雷达传感器:** 阅读 `point_lio` 第二层文档（preprocess.h 支持 AVIA, VELO16, OUST64, HESAIxt32）、`livox_ros_driver2`（驱动模式）和 `nav_bringup` 配置。
- **新人上手:** 完整阅读第一层，然后阅读 `nav_bringup`、`point_lio`、`nav2_plugins` 和 `omni_pid_pursuit_controller` 的第二层文档。
