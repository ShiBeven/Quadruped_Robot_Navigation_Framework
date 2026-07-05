# 模块: nav_bringup

> 上级: [PROJECT_DOC.md](../PROJECT_DOC.md)
> 最后同步: 提交 `5b3f214` — 2026-07-05
> 层级分布: A: 3 / B: 2 / C: 5

## 职责
系统入口点：ROS2 启动文件、参数 YAML 配置和行为树 XML 定义，组合四足机器人的整个导航栈。包含三种启动场景（导航、定位、SLAM）、两套参数配置（硬件和仿真）以及机器人 URDF 描述。

## 关键文件

| 文件 | 层级 | 职责 |
|---|---|---|
| `launch/legged_navigation_launch.py` | A | 主入口：启动完整 Nav2 栈 + 3 个感知节点 + 生命周期管理器 |
| `launch/legged_localization_launch.py` | A | 定位入口：Point-LIO + map_server + small_gicp_relocalization |
| `launch/legged_slam_launch.py` | A | SLAM 入口：Point-LIO + slam_toolbox + pointcloud_to_laserscan |
| `config/nav2_params.legged.yaml` | A | 硬件参数配置（保守调优、Livox 激光、全向运动） |
| `config/nav2_params.legged_sim.yaml` | B | 仿真参数配置（Velodyne 模型、宽松限制、仿真时钟） |
| `behavior_trees/legged_navigate_w_replanning_and_recovery.xml` | A | 行为树：3Hz 规划、10 次重试路径跟踪、原地旋转恢复 |
| `description/quadruped.urdf` | C | 四足机器人 URDF 模型 |

## 启动架构

### `legged_navigation_launch.py` — 自主导航

**启动的节点（共 13 个）：**

| 节点 | 包 | 受管控? | 可组合? |
|---|---|---|---|
| `terrain_analysis` | terrain_analysis | 否 (独立) | 否 |
| `terrain_analysis_ext` | terrain_analysis_ext | 否 (独立) | 否 |
| `loam_interface` | loam_interface | 否 | 是 |
| `sensor_scan_generation` | sensor_scan_generation | 否 (条件) | 是 |
| `controller_server` | nav2_controller | 是 | 是 |
| `smoother_server` | nav2_smoother | 是 | 是 |
| `planner_server` | nav2_planner | 是 | 是 |
| `behavior_server` | nav2_behaviors | 是 | 是 |
| `bt_navigator` | nav2_bt_navigator | 是 | 是 |
| `waypoint_follower` | nav2_waypoint_follower | 是 | 是 |
| `velocity_smoother` | nav2_velocity_smoother | 是 | 是 |
| `lifecycle_manager_navigation` | nav2_lifecycle_manager | — | 否 |

**关键重映射:** `cmd_vel` → `cmd_vel_nav2_result`（控制器输出）→ `cmd_vel_smoothed` → `cmd_vel`（最终输出）。将原始控制器输出与机器人驱动隔离开。

### `legged_localization_launch.py` — 已知地图定位

| 节点 | 包 | 受管控? |
|---|---|---|
| `point_lio` | point_lio | 否 (独立) |
| `map_server` | nav2_map_server | 是 |
| `small_gicp_relocalization` | small_gicp_relocalization | 否 |

### `legged_slam_launch.py` — SLAM 建图

| 节点 | 包 |
|---|---|
| `point_lio` | point_lio |
| `pointcloud_to_laserscan` | pointcloud_to_laserscan |
| `slam_toolbox` | slam_toolbox |
| `static_transform_publisher` | tf2_ros (map→odom) |

## 参数配置对比

| 参数 | 硬件 (`legged`) | 仿真 (`legged_sim`) |
|---|---|---|
| `use_sim_time` | false | true |
| `point_lio.lidar_type` | 1 (Livox) | 2 (Velodyne) |
| `point_lio.scan_line` | 4 | 32 |
| `filter_size_surf` | 0.05 | 0.2 |
| `terrain_analysis.scanVoxelSize` | 0.02 | 0.05 |
| `v_linear_max` | 1.5 m/s | 2.5 m/s |
| `v_linear_min` | -1.5 m/s | -2.5 m/s |
| `velocity_smoother.max_velocity` | [1.2, 1.2, 1.2] | [2.0, 2.0, 2.0] |

## 行为树结构

```
MainTree
└── RecoveryNode "NavigateRecovery" [重试次数=10]
    ├── PipelineSequence "NavigateWithReplanning"
    │   ├── RateController [3 Hz] → ComputePathToPose → ClearGlobalCostmap (恢复)
    │   └── RecoveryNode "FollowPath" [重试次数=10]
    │       ├── FollowPath [controller_id="FollowPath"]
    │       └── ClearLocalCostmap (恢复)
    └── ReactiveFallback "RecoveryFallback"
        ├── GoalUpdated (逃生出口)
        └── RoundRobin
            ├── ClearLocalCostmap + ClearGlobalCostmap
            └── Spin [180°] ← 四足特化: 旋转代替后退
```

## 四足机器人特有设计决策

1. **旋转恢复代替后退**: 四足机器人不适合盲向后运动。恢复策略为原地旋转 180°。
2. **全向转向**: SmacPlannerHybrid 中 `minimum_turning_radius=0.0`（四足可原地转向）。
3. **`robot_base_frame="body"`** 代替 `base_link` — 反映四足机器人机身坐标系约定。
4. **保守的速度限制**: 约为典型轮式机器人限制的 50%（1.5 m/s vs 3.0+ m/s）。
5. **`IntensityVoxelLayer`** 替代标准 `ObstacleLayer` — 按激光强度过滤进行地形感知障碍物检测。

## 调用关系

- **依赖于:** point_lio, nav2_plugins, omni_pid_pursuit_controller, terrain_analysis, terrain_analysis_ext, loam_interface, sensor_scan_generation, small_gicp_relocalization, pointcloud_to_laserscan, nav2_controller, nav2_planner, nav2_behaviors, nav2_bt_navigator, nav2_waypoint_follower, nav2_velocity_smoother, nav2_lifecycle_manager, nav2_map_server, slam_toolbox, tf2_ros
- **被依赖:** (入口点 — 不被其他模块导入)
