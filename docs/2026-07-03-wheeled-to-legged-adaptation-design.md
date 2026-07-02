# 四舵轮→四足机器人导航框架适配设计

> **设计日期**: 2026-07-03
> **项目**: Basic Navigation Framework for ROS 2
> **变更类型**: 底盘替换 —— 轮式哨兵 → 自研四足机器人
> **核心原则**: `cmd_vel` 接口不变，步态解算由电控端闭环，导航端仅负责运动指令生成

---

## 目录

- [1. 设计目标与边界](#1-设计目标与边界)
- [2. 分层结构与模块职责重定义](#2-分层结构与模块职责重定义)
- [3. TF 坐标变换树](#3-tf-坐标变换树)
- [4. 参数配置改动清单](#4-参数配置改动清单)
- [5. 模块修改清单](#5-模块修改清单)
- [6. 接口兼容规则](#6-接口兼容规则)

---

## 1. 设计目标与边界

### 1.1 前置条件

| 项 | 决策 |
|----|------|
| 机器人类型 | 自研四足（12 自由度，控制器自研） |
| 运动模式 | 先按全向假设，仿真验证后迭代约束 |
| 传感器 | 沿用 Livox MID-360 + 内置 IMU |
| 地形分析 | 暂不动，后续有步态参数再升级 |
| 轮式兼容 | 不需要，直接替换，删除轮式配置和哨兵遗留 |

### 1.2 边界定义

```
导航框架（本项目职责）               电控端（不在本项目范围）

                        cmd_vel
  Nav2 导航栈 ─────────────────────→ 步态引擎 → WBC/MPC → 关节指令
  (Twist: vx, vy, vyaw) 
                        
                        odometry  
  sensor_scan_generation ←──────── 里程计反馈（也可由本框架 LiDAR-IMU 自给）
  (Odometry + TF)
```

**接口：** `cmd_vel` (geometry_msgs/Twist) 语义不变，电控端自行解算步态。导航端不输出关节指令。

### 1.3 设计约束

1. 上层决策/感知/通信模块代码零改动
2. 底层传感器驱动/定位管线零改动
3. 仅替换配置文件和启动文件
4. 删除所有轮式哨兵残留（云台消息、哨兵模型、手柄遥控映射）
5. 参数切换通过 `params_file` 一键完成

---

## 2. 分层结构与模块职责重定义

原项目 17 个包，适配后 16 个包（删除 1 个：`robot_interfaces`），按与底盘的耦合度分为三类：

### 2.1 🟢 完全保留（13 个包，代码零改动）

| 包 | 理由 |
|----|------|
| `livox_ros_driver2` | 传感器驱动，纯硬件 |
| `point_lio` | LiDAR-IMU 里程计，只依赖点云 + IMU |
| `loam_interface` | 帧适配，只依赖 TF 外参配置 |
| `sensor_scan_generation` | 时间同步 + TF 广播，只依赖 TF 配置 |
| `small_gicp_relocalization` | 全局重定位，只依赖点云 + 先验地图 |
| `terrain_analysis` | 近场地形分析（暂不动） |
| `terrain_analysis_ext` | 远场地形分析（暂不动） |
| `pointcloud_to_laserscan` | 3D→2D，纯数据转换 |
| `omni_pid_pursuit_controller` | 全向 PID 控制器（参数调整，代码不动） |
| `nav2_plugins` | BT 节点 + 代价地图层（恢复行为不加载即可，代码不动） |
| `nav2_loopback_sim` | 回路仿真器（代码不动，仿真参数单独配置） |
| `ign_sim_pointcloud_tool` | 仿真点云格式转换 |
| `pcd2pgm` | PCD→PGM 地图工具 |
| `rosbag2_composable_recorder` | rosbag 录制工具 |

### 2.2 🟡 参数级修改（`nav_bringup`）

| 改动项 | 内容 |
|--------|------|
| 新建 `config/nav2_params.legged.yaml` | 四足实机参数 |
| 新建 `config/nav2_params.legged_sim.yaml` | 四足仿真参数 |
| 新建 `description/quadruped.urdf` | 四足 URDF 模型 + TF 静态链 |
| 新建 `launch/legged_navigation_launch.py` | 四足导航启动 |
| 新建 `launch/legged_localization_launch.py` | 四足定位启动 |
| 新建 `launch/legged_slam_launch.py` | 四足 SLAM 启动 |
| 新建 `behavior_trees/legged_navigate_w_replanning_and_recovery.xml` | 四足行为树 |
| 删除全部轮式哨兵配置和 launch 文件 | 见模块修改清单 |

### 2.3 🔴 删除/废弃

| 模块 | 内容 | 理由 |
|------|------|------|
| `robot_interfaces`（整包） | 4 条 msg 全部作废 | 云台、哨兵模型、机器人状态均为轮式哨兵专用，四足不适用 |

---

## 3. TF 坐标变换树

### 3.1 轮式（旧） vs 四足（新）

```
轮式哨兵（旧）：                    四足（新）：
map                               map
 └→ odom                            └→ odom
      └→ chassis (动态TF)                 └→ base_footprint (动态TF)
           └→ gimbal_yaw (静态)                └→ base_link (静态)
                └→ gimbal_pitch (静态)              └→ body (静态)
                     └→ front_mid360 (静态)              └→ front_mid360 (静态)
```

### 3.2 四足 TF 链定义

| TF | 类型 | 广播者 | 频率 | 说明 |
|----|------|--------|------|------|
| `map → odom` | 动态 | `small_gicp_relocalization` | 20 Hz | 全局定位修正 |
| `odom → base_footprint` | 动态 | `sensor_scan_generation` | 按里程计频率 | 里程计位姿 |
| `base_footprint → base_link` | 静态 | URDF / `robot_state_publisher` | — | Z 轴偏移（地面 → 躯干中心） |
| `base_link → body` | 静态 | URDF | — | 可设 identity 或小偏移 |
| `body → front_mid360` | 静态 | URDF | — | LiDAR 安装位姿（外参） |

### 3.3 坐标系含义

| 坐标系 | 含义 |
|--------|------|
| `map` | 世界固定坐标系，由 `small_gicp_relocalization` 提供 |
| `odom` | 里程计局部坐标系，连续无跳变 |
| `base_footprint` | 机器人躯干中心在地面的垂直投影（Z=0 在地面） |
| `base_link` | 躯干几何中心（考虑 Z 偏移） |
| `body` | 躯干参考坐标系，里程计和 cmd_vel 的基准 |
| `front_mid360` | Livox MID-360 LiDAR 安装位姿 |

### 3.4 删除的 TF

- `chassis → gimbal_yaw → gimbal_pitch` 及下属帧
- `front_industrial_camera`

---

## 4. 参数配置改动清单

### 4.1 控制器参数 (`controller_server.ros__parameters.FollowPath`)

| 参数 | 轮式旧值 | 四足推荐值 | 变更理由 |
|------|----------|-----------|----------|
| `v_linear_min` | -4.5 | **-1.5** | 四足极限速度约 2 m/s |
| `v_linear_max` | 4.5 | **1.5** | 同上 |
| `v_angular_min` | -3.0 | **-1.5** | 原地转身也需步态调整 |
| `v_angular_max` | 3.0 | **1.5** | 同上 |
| `translation_kp` | 3.0 | **1.5** | 四足加速慢，高 P 导致超调/振荡 |
| `translation_ki` | 0.1 | **0.05** | 相应降低 |
| `translation_kd` | 0.3 | **0.1** | 相应降低 |
| `rotation_kp` | 未设 | **1.0** | 与 translation 保持一致 |
| `rotation_ki` | 未设 | **0.05** | — |
| `rotation_kd` | 未设 | **0.1** | — |
| `lookahead_dist` | 2.0 | **1.0** | 速度慢，预瞄距离缩短 |
| `min_lookahead_dist` | 0.5 | **0.3** | — |
| `max_lookahead_dist` | 1.0 | **1.5** | — |
| `curvature_min` | 2.5 | **1.0** | 四足转向能力弱于轮式，更早减速 |
| `curvature_max` | 5.0 | **3.0** | — |
| `reduction_ratio_at_high_curvature` | 0.5 | **0.4** | 急弯时更加保守 |
| `enable_rotation` | false | **true** | 四足可能需要先转身再走 |
| `use_rotate_to_heading` | false | **true** | 同上 |
| `use_velocity_scaled_lookahead_dist` | 未设 | **true** | 使预瞄距离随速度动态调整 |

### 4.2 规划器参数 (`planner_server.ros__parameters.GridBased`)

| 参数 | 轮式旧值 | 四足推荐值 | 变更理由 |
|------|----------|-----------|----------|
| `motion_model` | DUBIN | **DUBIN**（暂保持） | 全向假设下适用 |
| `tolerance` | 0.3 | **0.25** | 四足到位精度稍高 |
| `minimum_turning_radius` | 0.07 | **0.0** | 四足可原地转向 |
| `allow_unknown` | true | **true**（保持） | — |

### 4.3 速度平滑器 (`velocity_smoother.ros__parameters`)

| 参数 | 轮式旧值 | 四足推荐值 | 变更理由 |
|------|----------|-----------|----------|
| `max_velocity` | [3.5, 3.5, 5.0] | **[1.2, 1.2, 1.2]** | 平滑后留余量 |
| `min_velocity` | [-3.5, -3.5, -5.0] | **[-1.2, -1.2, -1.2]** | — |
| `max_accel` | [4.5, 4.5, 5.0] | **[0.8, 0.8, 1.0]** | 四足加速远慢于轮式 |
| `max_decel` | [-4.5, -4.5, -5.0] | **[-0.8, -0.8, -1.0]** | — |
| `feedback` | OPEN_LOOP | **OPEN_LOOP**（保持） | — |
| `frequency` | 20.0 | **20.0**（保持） | — |

### 4.4 代价地图参数

#### local_costmap

| 参数 | 轮式旧值 | 四足推荐值 | 变更理由 |
|------|----------|-----------|----------|
| `width` | 5.0 | **6.0** | 制动距离更长，需要更大观察窗 |
| `height` | 5.0 | **6.0** | 同上 |
| `resolution` | 0.05 | **0.05**（保持） | — |
| `robot_radius` | 0.3 | **0.45** | 四足躯干 + 腿部外展 |
| `inflation_layer.inflation_radius` | 0.4 | **0.6** | — |
| `inflation_layer.cost_scaling_factor` | 4.0 | **4.0**（保持） | — |
| `rolling_window` | true | **true**（保持） | — |

#### global_costmap

| 参数 | 轮式旧值 | 四足推荐值 | 变更理由 |
|------|----------|-----------|----------|
| `robot_radius` | 0.3 | **0.45** | 同上 |
| `inflation_layer.inflation_radius` | 0.6 | **0.8** | — |
| `inflation_layer.cost_scaling_factor` | 4.0 | **4.0**（保持） | — |

### 4.5 坐标系 frame_id 替换

| 参数（所属模块） | 旧值 | 新值 |
|------------------|------|------|
| `loam_interface.odom_frame` | `"odom"` | **`"odom"`**（保持） |
| `loam_interface.base_frame` | `"base_footprint"` | **`"base_footprint"`**（保持） |
| `loam_interface.lidar_frame` | `"front_mid360"` | **`"front_mid360"`**（保持） |
| `sensor_scan_generation.lidar_frame` | `"front_mid360"` | **`"front_mid360"`**（保持） |
| `sensor_scan_generation.base_frame` | `"base_footprint"` | **`"base_footprint"`**（保持） |
| `sensor_scan_generation.robot_base_frame` | `"gimbal_yaw"` | **`"body"`** ← **关键变化** |

### 4.6 四足仿真参数差异 (`nav2_params.legged_sim.yaml`)

| 参数 | 四足实机 | 四足仿真 | 理由 |
|------|----------|----------|------|
| `point_lio.lidar_type` | 1 (Livox) | 2 (Velodyne) | 仿真使用 Velodyne 格式 |
| `point_lio.scan_line` | 4 | 32 | 仿真高线数 |
| `point_lio.filter_size_surf` | 0.05 | 0.2 | 仿真稀疏点云 |
| `terrain_analysis.scanVoxelSize` | 0.02 | 0.05 | 仿真放宽 |
| `controller_server.v_linear_max` | 1.5 | 2.5 | 仿真放宽速度限制 |
| `use_sim_time` | false | true | 仿真时钟 |

### 4.7 删除的参数

- `point_lio` 外参 `extrinsic_T: [-0.011, -0.02329, 0.04412]` —— 轮式哨兵专用，四足需实测后填入新外参
- `teleop_twist_joy` 全部参数（`axis_chassis`, `scale_chassis`, `axis_gimbal` 等）—— 四足手柄遥控后续重写
- `robot_interfaces` 消息引用 —— Gimbal/GimbalCmd/Models/RobotStateInfo 全部作废
- 轮式哨兵的 `point_lio` 重力向量 `gravity: [-0.145, -9.168, -3.407]` —— 四足安装方向不同，需实测

---

## 5. 模块修改清单

### 5.1 新增文件（7 个）

| 文件 | 说明 |
|------|------|
| `src/navigation/nav_bringup/config/nav2_params.legged.yaml` | 四足实机参数 |
| `src/navigation/nav_bringup/config/nav2_params.legged_sim.yaml` | 四足仿真参数 |
| `src/navigation/nav_bringup/description/quadruped.urdf` | 四足 URDF：body + 四条腿 joint + LiDAR 安装位 |
| `src/navigation/nav_bringup/launch/legged_navigation_launch.py` | 四足导航启动 |
| `src/navigation/nav_bringup/launch/legged_localization_launch.py` | 四足定位启动 |
| `src/navigation/nav_bringup/launch/legged_slam_launch.py` | 四足 SLAM 启动 |
| `src/navigation/nav_bringup/behavior_trees/legged_navigate_w_replanning_and_recovery.xml` | 四足行为树（移除轮式 BackUp 恢复行为） |

### 5.2 修改文件（2 个）

| 文件 | 改动内容 | 改动量 |
|------|----------|:---:|
| `nav_bringup/CMakeLists.txt` | 新增 install 指令：`description/` 目录、`legged_*.py`、`nav2_params.legged*.yaml`、`legged_*.xml` | 小 |
| `nav_bringup/package.xml` | 版本号升至 2.0.0 | 1 行 |

### 5.3 删除文件（12 个）

| 文件 | 理由 |
|------|------|
| `nav_bringup/config/nav2_params.reality.yaml` | 轮式实机参数 |
| `nav_bringup/config/nav2_params.simulation.yaml` | 轮式仿真参数 |
| `nav_bringup/launch/navigation_launch.py` | 轮式导航启动 |
| `nav_bringup/launch/localization_launch.py` | 轮式定位启动 |
| `nav_bringup/launch/slam_launch.py` | 轮式 SLAM 启动 |
| `nav_bringup/launch/simulation.launch.py` | 轮式仿真启动 |
| `nav_bringup/launch/rviz_launch.py` | 轮式 RViz 配置 |
| `nav_bringup/launch/joy_teleop_launch.py` | 手柄遥控（后续重写四足版） |
| `nav_bringup/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml` | 轮式行为树 |
| `nav_bringup/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml` | 轮式行为树 |
| `nav_bringup/rviz/nav2_default_view.rviz` | 轮式 RViz 视图 |
| `robot_interfaces/`（整包） | 云台/哨兵消息全部作废 |

### 5.4 改动量统计

| 类别 | 数量 |
|------|:---:|
| 新增文件 | 7 |
| 修改文件 | 2 |
| 删除文件 | 12 |
| 不变包（代码零改动） | 13 / 16 |

---

## 6. 接口兼容规则

五条硬约束，保证后续代码优化和迭代不会破坏核心管线。

### 规则 1：`cmd_vel` 接口契约

```
发布者: velocity_smoother
话题:   /cmd_vel
类型:   geometry_msgs/Twist
帧:     base_footprint（必须）
```

| 字段 | 含义 | 单位 | 约束 |
|------|------|------|------|
| `linear.x` | 前进速度 | m/s | 范围见 `velocity_smoother` 配置 |
| `linear.y` | 侧移速度 | m/s | 同上 |
| `angular.z` | 转向角速度 | rad/s | 同上 |
| 其余字段 | 必须为 0 | — | 禁止填充非零值 |

**约束：**
- 任何底层模块不得改变此消息的语义
- 电控端若需要额外信号（如步态类型切换），使用独立话题，不污染 `cmd_vel`
- 帧必须为 `base_footprint`，Velocity Smoother 的 Twist 不包含 `header.frame_id` 时下游以 `base_footprint` 解析

### 规则 2：里程计接口契约

```
发布者: sensor_scan_generation
话题:   /odometry
类型:   nav_msgs/Odometry
```

| 字段 | 要求 |
|------|------|
| `header.frame_id` | `"odom"`（必须） |
| `child_frame_id` | `"body"`（必须，以 body 为基准） |
| `pose` | 机器人 body 在 odom 系下的 6-DOF 位姿 |
| `twist` | body 在 body 系下的速度，必须填充真实值 |

**约束：**
- `twist` 必须包含有限差分计算的真实速度（`sensor_scan_generation` 已实现），禁止零填充——Nav2 的 `velocity_smoother` 依赖它做开环反馈
- 如果后续电控端能提供更精确的里程计（关节编码器 + IMU 融合），可替换 `sensor_scan_generation` 的 odom 发布者，但话题名和消息结构不能变
- `child_frame_id` 必须是 `body`，下游读取此值做 TF 拼接时以此为准

### 规则 3：TF 树最小约定

```
必须提供（动态）:
  map → odom               ← small_gicp_relocalization（全局定位）
  odom → base_footprint    ← sensor_scan_generation（里程计）

必须提供（静态 URDF）:
  base_footprint → base_link  ← robot_state_publisher
  base_link → body            ← robot_state_publisher
  body → front_mid360         ← robot_state_publisher（LiDAR 外参）

可选（后续扩展）:
  body → {fl, fr, rl, rr}_foot  ← 四足足端坐标，用于地形分析
```

**约束：**
- `base_footprint` 必须位于地面（body 中心在地面的垂直投影），`base_link` 位于躯干几何中心
- 新增 TF 帧（如足端）只能挂在 `body` 下，不能改变现有 TF 链的 parent-child 关系
- 动态 TF 更新频率 ≥ 20 Hz
- 所有模块通过 TF 查找而非硬编码 frame_id——后续改 frame 名只需改 URDF，不改代码

### 规则 4：参数命名空间隔离

```
nav2_params.<chassis_type>.yaml:
  命名空间层次（结构必须一致，只改值）：
    controller_server.ros__parameters.FollowPath.*
    planner_server.ros__parameters.GridBased.*
    local_costmap.ros__parameters.*
    global_costmap.ros__parameters.*
    velocity_smoother.ros__parameters.*
    point_lio.ros__parameters.*
    loam_interface.ros__parameters.*
    sensor_scan_generation.ros__parameters.*
    terrain_analysis.ros__parameters.*
    terrain_analysis_ext.ros__parameters.*
```

**约束：**
- 新增底盘类型，在 `config/` 下新建 `nav2_params.<chassis_type>.yaml`，不改动已有文件
- namespace 结构必须一致（参数名不能变），只改值
- Launch 文件通过 `params_file:=config/nav2_params.<type>.yaml` 切换，不搞 if-else 分支

### 规则 5：模块依赖方向

```
决策/行为 (nav2_plugins BT nodes)
    ↓  (依赖 Nav2 Action/BT 接口)
导航 (planner_server, controller_server, smoother)
    ↓  (依赖 /odometry, TF, /scan)
感知 (terrain_analysis, pointcloud_to_laserscan)
    ↓  (依赖点云 + TF)
定位 (point_lio, loam_interface, sensor_scan)
    ↓  (依赖 LiDAR + IMU 原始数据)
驱动 (livox_ros_driver2)
```

**约束：**
- 上层可以依赖下层，下层不得依赖上层——定位层不知道导航层的存在
- 同层模块不得互相直接依赖——`terrain_analysis` 和 `terrain_analysis_ext` 通过话题通信，不直接 include
- 新增模块放入对应层级，违反依赖方向 = 设计错误
- 唯一例外：`nav_bringup` 可以跨层依赖（它只是编排器，无算法逻辑）

---

> **下一步**: 通过 writing-plans 技能生成实施计划，按阶段落实到代码。
