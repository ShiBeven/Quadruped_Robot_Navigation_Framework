# 设计文档：哨兵竞赛工程 → 日常导航框架

日期：2026-06-30
状态：已确认

## 目标

将 RoboMaster 哨兵竞赛工作区精简为纯净、可复用的机器人导航框架。移除全部视觉自瞄、串口/裁判系统通信、竞赛行为树及 RoboMaster 专用依赖，仅保留核心导航算法、通用底盘控制插件和仿真工具。重命名所有包以去除竞赛品牌标识。

## 非目标

- 不添加任何新功能或特性
- 不修改保留代码的行为逻辑
- 除依赖裁剪外不改变构建系统
- 不含机器人描述文件（用户后续自行添加）
- 不含硬件相关配置（用户自行提供地图/参数）

---

## 1. 最终目录结构

```
.
├── AGENTS.md
├── README.md
├── .gitignore                          # 更新：移除 sp_vision25 排除行
├── src/
│   ├── dependencies/
│   │   ├── BehaviorTree.ROS2/
│   │   ├── joint_state_publisher/
│   │   └── sdformat_tools/
│   │
│   ├── interfaces/
│   │   └── robot_interfaces/           # 新建：从 pb_rm_interfaces 提取的通用消息
│   │       ├── CMakeLists.txt
│   │       ├── package.xml
│   │       └── msg/
│   │           ├── Gimbal.msg
│   │           ├── GimbalCmd.msg
│   │           ├── Models.msg
│   │           └── RobotStateInfo.msg
│   │
│   ├── navigation/
│   │   ├── nav_bringup/
│   │   │   ├── behavior_trees/
│   │   │   ├── config/
│   │   │   ├── launch/
│   │   │   ├── map/
│   │   │   └── rviz/
│   │   ├── nav2_plugins/
│   │   │   ├── include/nav2_plugins/
│   │   │   │   ├── behaviors/
│   │   │   │   ├── layers/
│   │   │   │   └── bt/
│   │   │   └── src/
│   │   │       ├── behaviors/
│   │   │       ├── layers/
│   │   │       └── bt/
│   │   ├── omni_pid_pursuit_controller/
│   │   ├── point_lio/
│   │   ├── livox_ros_driver2/
│   │   ├── loam_interface/
│   │   ├── sensor_scan_generation/
│   │   ├── small_gicp_relocalization/
│   │   ├── pointcloud_to_laserscan/
│   │   ├── terrain_analysis/
│   │   ├── terrain_analysis_ext/
│   │   ├── ign_sim_pointcloud_tool/
│   │   └── teleop_twist_joy/
│   │
│   ├── simulation/
│   │   └── nav2_loopback_sim/
│   │
│   └── tools/
│       ├── pcd2pgm/
│       └── rosbag2_composable_recorder/
│
├── docs/
│   ├── superpowers/specs/
│   └── slim_loopback_refactor.md
```

---

## 2. 整包移除清单（10 个包）

| 包 | 移除原因 |
|----|---------|
| `src/sp_vision25/` 残留引用 | 视觉自瞄，已被 gitignore；清理文档和 .gitignore 中的引用 |
| `src/standard_robot_pp_ros2/` | 串口通信、裁判系统中继、竞赛指令下发 |
| `src/pb2025_sentry_behavior/` | 30 个 BT 插件服务于 RoboMaster 比赛策略（攻防/撤退/巡逻/视觉跟随） |
| `src/pb2025_sentry_bringup/` | 竞赛启动入口，引用串口/行为树/视觉包 |
| `src/pb2025_robot_description/` | 哨兵 URDF/SDF，含装甲板、发射器、竞赛传感器 |
| `src/dependencies/rmoss_core/` | RoboMasterOSS：上位机-下位机通信、弹道解算、RM 相机封装 |
| `src/dependencies/rmoss_gz_resources/` | RM 裁判系统组件、装甲板、弹丸、标准机器人模型 |
| `src/dependencies/rmoss_interfaces/` | RM 专用消息/服务（射击、底盘、裁判系统、队伍颜色） |
| `src/interfaces/pb_rm_interfaces/` | 7 个裁判系统消息 + 战队品牌包名 |
| `src/tools/teleop_gimbal_keyboard/` | 依赖 pb_rm_interfaces/GimbalCmd |

---

## 3. 包重命名

| 旧路径 | 新路径 |
|--------|--------|
| `src/pb2025_sentry_nav/` → 元包移除 | `src/navigation/`（仅作目录，不再设元包） |
| `src/pb2025_sentry_nav/pb2025_nav_bringup/` | `src/navigation/nav_bringup/` |
| `src/pb2025_sentry_nav/pb_nav2_plugins/` | `src/navigation/nav2_plugins/` |
| `src/pb2025_sentry_nav/pb_omni_pid_pursuit_controller/` | `src/navigation/omni_pid_pursuit_controller/` |
| `src/pb2025_sentry_nav/pb_teleop_twist_joy/` | `src/navigation/teleop_twist_joy/` |
| `src/loopback_sim/` | `src/simulation/nav2_loopback_sim/` |
| （新建） | `src/interfaces/robot_interfaces/` |

---

## 4. BT 插件迁移：12 个插件从 pb2025_sentry_behavior → nav2_plugins/bt/

### 保留（12 个）

| 旧 XML ID | 新类名 | 类型 | 用途 |
|-----------|--------|------|------|
| `PublishTwist` | `PubTwist` | Action | 发布 cmd_vel（vx/vy/vyaw），持续指定时长，停止时归零 |
| `PublishSpinSpeed` | `PubSpinSpeed` | Action | 发布底盘自旋角速度 |
| `HoldStopFlag` | `HoldStopFlag` | Action | 在路径点停留指定时长，保持 stop_flag |
| `SendNav2Goal` | `SendNav2Goal` | Action | Nav2 NavigateToPose 动作客户端 |
| `SendNavThroughPoses` | `SendNavThroughPoses` | Action | Nav2 NavigateThroughPoses 动作客户端，路径去重，进度追踪 |
| `PublishDecisionGoal` | `PublishNavGoal` | Action | 向 goal_pose 话题发布目标，带容差去重 |
| `SelectPathGoalPose` | `SelectPathGoalPose` | Action | 从 Path 中提取末位姿 → PoseStamped |
| `SelectFixedPath` | `SelectFixedPath` | Action | 从参数化目标点列表构建单点路径 |
| `SelectPatrolPath` | `SelectPatrolPath` | Action | 多点往返巡逻路径生成 |
| `IsPathGoalReached` | `IsPathGoalReached` | Condition | 当前位姿距路径末点是否在容差内 |
| `RecoveryNode` | `RecoveryNode` | Control | 双子节点重试恢复（子1失败 → 子2恢复 → 重试子1） |
| `RateController` | `RateController` | Decorator | 限制子节点执行频率 |

### 迁移时需要修改

- 去除 `pb_rm_interfaces` 依赖 — 所有插件仅使用标准消息类型 `nav_msgs`、`geometry_msgs`、`nav2_msgs`
- 从 `decision_utils.hpp` 提取通用路径工具到 `bt/nav_utils.hpp`：
  - `buildPathFromIndices()` — 从索引目标点构建 Path
  - `findNearestIndex()` — 找到距当前位姿最近的目标点索引
  - `computeNextPatrolState()` — 往返巡逻游标推进
  - 移除：`GameTimeStage`、`HpBand`、`TimeThresholds`、`HpThresholds` 等竞赛枚举/结构体
- 参数命名空间迁移：`decision.*` → `nav.*`（如 `nav.goal_points`、`nav.patrol_indices`）

### 移除（18 个）

| 插件 | 移除原因 |
|------|---------|
| `IsGameStatus`, `IsGameTimeStage`, `IsHpBand`, `IsStatusOK`, `IsAttacked`, `IsRfidDetected` | 依赖裁判系统消息 |
| `IsVisionTargetValid`, `SelectVisionTargetPath`, `SelectVisionFollowPath` | 依赖 sp_msgs 视觉消息 |
| `IsDecisionInputSource`, `IsDecisionMode` | 竞赛仿真/裁判模式切换 |
| `SelectNearestRetreatPath` | 血量触发的撤退策略 |
| `AdvancePatrolCursor`, `ResetLowHpTarget` | 竞赛状态机辅助 |
| `PublishGimbalAbsolute`, `PublishGimbalVelocity` | 依赖 pb_rm_interfaces/GimbalCmd |

---

## 5. 启动体系（nav_bringup/launch/ 下 4 个文件）

| Launch 文件 | 启动内容 | 对外参数 |
|------------|---------|---------|
| `navigation.launch.py` | controller_server, planner_server, smoother_server, behavior_server, bt_navigator, waypoint_follower, lifecycle_manager_navigation, terrain_analysis, terrain_analysis_ext, loam_interface, sensor_scan_generation | `params_file`, `use_sim_time`, `map` |
| `slam.launch.py` | point_lio, slam_toolbox, pointcloud_to_laserscan, 静态 TF (map→odom) | `params_file`, `use_sim_time` |
| `localization.launch.py` | point_lio, map_server, small_gicp_relocalization, lifecycle_manager_localization | `params_file`, `map`, `prior_pcd` |
| `simulation.launch.py` | nav2_loopback_sim, map_server, navigation.launch.py, rviz2 | `params_file`, `map`, `use_rviz` |

移除全部 RM 世界相关的 launch 文件：`rm_navigation_reality_launch.py`、`rm_navigation_simulation_launch.py`、`rm_multi_navigation_simulation_launch.py`、`robot_state_publisher_launch.py`，以及 `pb2025_sentry_bringup` 下的全部 launch 文件。

### 参数文件（nav_bringup/config/ 下 2 个）

- `nav2_params.reality.yaml` — 实机 Nav2 栈：controller（OmniPidPursuitController）、planner（SmacPlannerHybrid）、costmap、传感器（point_lio、terrain_analysis、loam_interface）
- `nav2_params.simulation.yaml` — 回环仿真：相同结构，sim_time=True，模拟 scan/costmap 话题

两个文件均从当前 `nav2_params.yaml` 提取，剔除 RM 专用段（无串口、无云台、无裁判系统话题）。

---

## 6. 接口：pb_rm_interfaces → robot_interfaces

从 `pb_rm_interfaces` 提取 4 个通用消息，移除 7 个裁判系统消息。

### 保留（4 个）

| 消息 | 用途 |
|------|------|
| `Gimbal.msg` | float64 pitch, yaw, pitch_range[2], yaw_range[2] |
| `GimbalCmd.msg` | Header, uint8 control_type (ABSOLUTE_ANGLE/VELOCITY), Gimbal position, Gimbal velocity |
| `Models.msg` | string[5] models: chassis, gimbal, shoot, arm, custom_controller |
| `RobotStateInfo.msg` | Header, Models robot_models |

### 移除（7 个）

`GameStatus`、`GameRobotHP`、`EventData`、`RobotStatus`、`Buff`、`GroundRobotPosition`、`RfidStatus` — 全部为 RoboMaster 裁判系统串口协议专用消息。

包名：`robot_interfaces`（替代战队品牌名 `pb_rm_interfaces`）。

---

## 7. 导航包内部变更

### 从 pb2025_sentry_nav 移除

| 子包 | 移除原因 |
|------|---------|
| `fake_vel_transform/` | 哨兵云台旋转速度补偿 |
| `sp_msgs/` | 视觉-导航互通消息（VisionTargetMsg 等） |
| `pb2025_sentry_nav/`（元包） | 竞赛品牌元包，不再需要 |

### 修改

| 子包 | 变更内容 |
|------|---------|
| `teleop_twist_joy/` | 移除云台关节状态发布、射击指令、云台轴映射。保留：cmd_vel + Nav2 NavigateToPose 动作目标 |

### 不变（11 个）

`point_lio`、`livox_ros_driver2`、`loam_interface`、`sensor_scan_generation`、`small_gicp_relocalization`、`pointcloud_to_laserscan`、`terrain_analysis`、`terrain_analysis_ext`、`ign_sim_pointcloud_tool`、`pb_nav2_plugins`（behaviors + layers）、`pb_omni_pid_pursuit_controller`。

---

## 8. 文档与顶层文件清理

### 移除（12 项）

| 文件 | 移除原因 |
|------|---------|
| `nav_README.md` | pb2025_sentry_nav 竞赛导航说明 |
| `ws_README.md` | 竞赛哨兵工作区说明 |
| `de_README.md` | pb2025_robot_description 说明 |
| `NAV2.sh` | 硬编码竞赛启动命令的看门狗脚本 |
| `mapping.sh` | 同上，建图模式 |
| `pp_ros2.sh` | 串口节点看门狗（对应包已移除） |
| `rosbag.sh` | 竞赛 rosbag 录制看门狗 |
| `docs/融合.md` | 视觉-导航融合架构文档 |
| `docs/移植.md` | 竞赛决策系统迁移笔记 |
| `docs/视觉跟随仿真调试.md` | 视觉跟随调试指南 |
| `docs/navigate_through_poses_migration_checklist.md` | 竞赛特定迁移清单 |
| `docs/sentry_bt_decision_checklist.md` | 竞赛决策树迁移清单 |

### 更新（3 项）

| 文件 | 变更 |
|------|------|
| `README.md` | 全文重写：去除视觉/串口/裁判系统章节，改为日常导航框架说明 |
| `AGENTS.md` | 更新包名、作用域、依赖 |
| `.gitignore` | 移除 `src/sp_vision25/` 行 |

### 保留不变（1 项）

| 文件 | 保留原因 |
|------|---------|
| `docs/slim_loopback_refactor.md` | 回环仿真架构模式可复用 |

---

## 9. 实施顺序

1. **移除** 竞赛整包目录（10 个包）
2. **移除** 竞赛文档、README 和 Shell 脚本（12 项）
3. **移除** `pb2025_sentry_nav/` 中竞赛子包（fake_vel_transform、sp_msgs、元包）
4. **移除** 竞赛依赖（rmoss_core、rmoss_gz_resources、rmoss_interfaces）
5. **重命名** 保留包的目录结构
6. **创建** `robot_interfaces` 包，含 4 个通用消息
7. **迁移** 12 个 BT 通用插件至 `nav2_plugins/bt/` 并完成清理
8. **简化** `teleop_twist_joy`（剔除云台/射击）
9. **重写** 4 个 launch 文件
10. **重写** `README.md` 和 `AGENTS.md`
11. **更新** `.gitignore`

---

## 10. 验证标准

- 移除后 `find src/ -name "package.xml" | wc -l` 应显示约 18 个包（原约 35 个）
- 无残留引用：`sp_vision25`、`pb_rm_interfaces`、`rmoss`、`pb2025_sentry_behavior`、`pb2025_sentry_bringup`、`standard_robot_pp_ros2`、`pb2025_robot_description`
- 无残留竞赛相关 CMake 依赖
- `README.md` 不含任何竞赛术语
- `AGENTS.md` 仅引用保留的包
- `grep -r "referee\|sentry\|rmul\|rmuc\|rmoss\|sp_vision" src/` 返回零结果
