# 模块: nav_bringup

> 同步: 2026-07-11 (非 git 仓库) | Tier 分布: A: 5 (launch+yaml) / C: URDF/BT xml | 语言: Python (launch) + YAML
> 路径: `src/navigation/nav_bringup/`
> **这是理解整个框架的入口** —— 它把所有节点接线成端到端管线。

## 职责
四足导航栈的顶层编排包: 通过三套 launch (localization / navigation / slam) 拉起 point_lio、loam_interface、sensor_scan、terrain_analysis×2、nav2 全家桶与定位节点, 并用 RewrittenYaml 注入统一参数与 topic remapping, 把 point_lio → loam_interface → nav2 → 控制器串成完整数据流。

## 关键文件
| 文件 | Tier | 职责 |
|---|---|---|
| launch/legged_navigation_launch.py | A | 拉起 nav2 导航栈 + loam_interface + sensor_scan + terrain×2; 支持 composition 双分支 (348 行) |
| launch/legged_localization_launch.py | A | 拉起 point_lio + map_server + small_gicp 重定位 + lifecycle_manager (207 行) |
| launch/legged_slam_launch.py | A | 拉起 point_lio + slam_toolbox + pointcloud_to_laserscan + map→odom 静态 TF (137 行) |
| config/nav2_params.legged.yaml | A | 实机全栈参数 (581 行) |
| config/nav2_params.legged_sim.yaml | A | 仿真参数 (use_sim_time=true, lidar_type=2 Velodyne, 546 行) |
| behavior_trees/legged_navigate_w_replanning_and_recovery.xml | C | 四足行为树, Spin 替代 BackUp (34 行) |
| description/quadruped.urdf | C | 机器人 TF 树定义 (177 行) |

## 各 launch 启动的节点 + 关键 remapping (胶水层)

### legged_navigation_launch.py
`use_composition` 决定 Node vs ComposableNode 分支 (节点集一致):
- `terrain_analysis` + `terrain_analysis_ext` — 始终启动
- GroupAction 内: `loam_interface`、`sensor_scan_generation` (受 use_sensor_scan 条件)
- nav2: `controller_server` (**remap cmd_vel → cmd_vel_nav2_result**)、`smoother_server`、`planner_server`、`behavior_server`、`waypoint_follower`、`bt_navigator` (remap cmd_vel → cmd_vel_nav2_result)、`velocity_smoother` (**remap cmd_vel→cmd_vel_nav2_result 且 cmd_vel_smoothed→cmd_vel**)
- `lifecycle_manager_navigation` 管理上述 7 个 nav2 节点

> 关键接线: controller/bt 输出到 `cmd_vel_nav2_result` → velocity_smoother 读入 → 平滑结果发到最终 `cmd_vel`。

### legged_localization_launch.py
- `point_lio` (追加 `prior_pcd.prior_pcd_map_path`)
- `map_server`、`small_gicp_relocalization` (追加 prior_pcd_file)、`lifecycle_manager_localization` (仅管 `map_server`; ⚠️ small_gicp 不在生命周期列表内)
- launch arg `map` (必填无默认)、`prior_pcd` (默认空)

### legged_slam_launch.py
- `pointcloud_to_laserscan` (**remap cloud_in→terrain_map_ext, scan→obstacle_scan**, 把全局地形点云打成 2D scan 供 slam_toolbox)
- `sync_slam_toolbox_node`、`point_lio` (prior_pcd.enable=False, pcd_save_en=True)
- `static_transform_publisher_map2odom` (发 map→odom 恒等 TF, SLAM 时无重定位)

## nav2 配置要点 (nav2_params.legged.yaml)
| 组件 | 配置 |
|---|---|
| **controller** | `FollowPath` = `omni_pid_pursuit_controller::OmniPidPursuitController` (全向PID纯追踪)。四足调低: translation_kp 1.5, v_linear ±1.5, use_rotate_to_heading true。goal_checker yaw_tolerance 6.28 (不约束朝向) |
| **local_costmap layers** | `[static_layer, intensity_voxel_layer, inflation_layer]`; `intensity_voxel_layer` = `nav2_plugins::IntensityVoxelLayer`, source=`terrain_map` (frame front_mid360), z_voxels 16。robot_radius 0.45, inflation 0.6 |
| **global_costmap layers** | 同三层; intensity_voxel_layer source=`terrain_map_ext`; inflation 0.8 |
| **planner** | `GridBased` = `nav2_smac_planner/SmacPlannerHybrid`, DUBIN, `minimum_turning_radius: 0.0` (四足原地转向), smooth_path True |
| **smoother** | `simple_smoother` = nav2_smoother::SimpleSmoother |
| **BT xml** | `$(find-pkg-share nav_bringup)/behavior_trees/legged_navigate_w_replanning_and_recovery.xml` |
| **behavior_server** | `[spin, backup, drive_on_heading, wait]` — 全标准 Nav2 行为, **未用 nav2_plugins 的 back_up_free_space** (注释明写用标准 BackUp; 恢复实际用 Spin) |
| **velocity_smoother** | max_velocity [1.2,1.2,1.2], max_accel [0.8,0.8,1.0] (四足大幅限速) |
| bt_navigator | robot_base_frame `base_footprint`, odom_topic `odometry` |

行为树使用节点: `RecoveryNode`、`PipelineSequence`、`RateController(hz=3.0)`、`ComputePathToPose(GridBased)`、`ClearEntireCostmap`、`FollowPath`、`ReactiveFallback`、`GoalUpdated`、`RoundRobin`、`Spin(spin_dist=3.14, is_recovery=true)`。**恢复用 Spin 替代 BackUp** (四足不宜后退)。

## URDF TF 树 (quadruped.urdf)
`base_footprint` → `base_link` → `body` → `front_mid360` (LiDAR); 四腿 `{fl,fr,rl,rr}_hip→knee→foot` 全 fixed。关键帧: `base_footprint` (costmap/bt robot_base_frame)、`body` (sensor_scan robot_base_frame)、`front_mid360` (lidar/sensor frame)。

## 核心数据流 (端到端, 本包编排)
```
Livox MID360 (livox/lidar + livox/imu)  [仿真: ign_sim_pointcloud_tool → velodyne_points]
  → point_lio → aft_mapped_to_init + cloud_registered
  → loam_interface → lidar_odometry + registered_scan (odom 帧)
  → sensor_scan_generation → TF odom→base_footprint + odometry + sensor_scan
      ├→ terrain_analysis → terrain_map (local_costmap source)
      └→ terrain_analysis_ext → terrain_map_ext (global_costmap source)
  → nav2: costmap(static+intensity_voxel_layer+inflation)
      → planner(SmacPlannerHybrid) → bt_navigator(BT, 恢复=Spin)
      → controller(omni_pid_pursuit) → cmd_vel_nav2_result
      → velocity_smoother → cmd_vel → 四足底盘
定位分支: map_server + small_gicp_relocalization → TF map→odom
```
TF 链: `map→odom`(small_gicp/slam/static) · `odom→base_footprint`(sensor_scan) · `base_footprint→base_link→body→front_mid360`(URDF/robot_state_publisher)。

## 实机 vs 仿真差异 (legged.yaml vs legged_sim.yaml)
`use_sim_time` false→true; point_lio `lidar_type` 1(Livox)→2(Velodyne), `scan_line` 4→32, `filter_size_surf` 0.05→0.2; terrain scanVoxelSize 0.02→0.05。帧名与拓扑不变。

## 调用关系 + 动态加载
- **依赖** (exec_depend): navigation2, nav2_common, nav2_smac_planner, nav2_plugins, livox_ros_driver2, point_lio, terrain_analysis(_ext), omni_pid_pursuit_controller, small_gicp_relocalization, pointcloud_to_laserscan, slam_toolbox, rviz2, robot_state_publisher, joint_state_publisher。
- **动态加载**: 用 `GroupAction`+`IfCondition(PythonExpression)` 做 Node vs `LoadComposableNodes` 双分支 (默认容器 `nav2_container`); nav2 lifecycle 管理器; `RewrittenYaml` 注入参数覆写 (`allow_substs=True`, 支持 yaml 内 `$(find-pkg-share)` 与 `<robot_namespace>` 占位符替换)。

> 详细风险 (BT xml 硬编码路径、`<robot_namespace>` 占位符依赖替换、SLAM map→odom 全 0 静态 TF、帧名与 URDF 强耦合、small_gicp 未纳入 lifecycle) 见 `PROJECT_DOC.md` Layer 3 注解 #43。
