> ⚠️ **强制同步声明**：后续代码无论进行任何修改、重构、优化，本 MD 文档必须同步更新，确保文档与源码实时一致。文档落后于代码即为失效。

# 四足机器人导航框架 全覆盖说明文档

> 生成时间：2026-07-04 22:00
> 源码根目录：W:/Quadruped_Robot_Navigation_Framework
> 覆盖范围：47 C++源文件 / 52 头文件 / 29 Python文件 / 约21,800行代码

---

## 1. 架构总览

### 1.1 项目类型 & 技术栈

| 维度 | 内容 |
|------|------|
| **项目类型** | 四足机器人自主导航软件框架 |
| **目标平台** | Ubuntu 22.04 + ROS 2 Humble |
| **语言** | C++17 (核心), Python 3.10 (Launch/工具) |
| **构建系统** | CMake (ament_cmake), colcon |
| **导航框架** | Nav2 (Navigation2) |
| **SLAM** | Point-LIO (LiDAR-惯性里程计), slam_toolbox |
| **传感器** | Livox MID-360 LiDAR + 内置 IMU |
| **控制算法** | PID 纯追踪控制器 (全向/非全向可配) |
| **行为树** | BehaviorTree.CPP v4, 通过自定义 Nav2 BT 插件 |
| **点云处理** | PCL (Point Cloud Library) |
| **地形感知** | 基于体素网格的实时地形分析 |

### 1.2 顶层目录结构

```
Quadruped_Robot_Navigation_Framework/
├── .gitignore                     # 构建产物 & IDE 忽略规则
├── docs/                          # 文档
│   ├── 新手小白入门指南.md          # 通俗入门教程
│   ├── 后续工作指南.md              # 后续开发任务清单
│   └── PROJECT_DOC.md             # 本文件
├── patterns/                      # 空目录(预留)
├── scripts/                       # 空目录(预留)
├── src/
│   ├── dependencies/              # ⚠️ 第三方依赖(非本项目代码)
│   │   ├── BehaviorTree.ROS2/     # BT ROS2 封装
│   │   ├── joint_state_publisher/ # 关节状态发布
│   │   └── sdformat_tools/        # SDF→URDF 转换工具
│   ├── navigation/                # ★ 导航核心 (13个包)
│   ├── simulation/                # 仿真工具 (1个包)
│   └── tools/                     # 辅助工具 (2个包)
└── README.md                      # (已删除,待重建)
```

### 1.3 分层架构

```
┌─────────────────────────────────────────────────┐
│                  ④ 决策层                         │
│   nav_bringup (总启动)                            │
│   bt_navigator (行为树引擎)                        │
│   nav2_plugins (自定义 BT 节点 + 代价地图层)       │
├─────────────────────────────────────────────────┤
│                  ③ 规划控制层                      │
│   planner_server (全局路径规划: SmacHybrid-DUBIN) │
│   controller_server (局部控制: OmniPidPursuit)    │
│   behavior_server (恢复行为: Spin/BackUp)          │
│   velocity_smoother (速度平滑)                     │
├─────────────────────────────────────────────────┤
│                  ② 感知定位层                      │
│   point_lio (LiDAR-IMU 紧耦合里程计)              │
│   loam_interface (坐标系翻译)                      │
│   sensor_scan_generation (时间同步 + 速度估计)      │
│   small_gicp_relocalization (重定位)               │
│   terrain_analysis / terrain_analysis_ext (地形)  │
│   pointcloud_to_laserscan (3D→2D 转换)            │
├─────────────────────────────────────────────────┤
│                  ① 驱动层                         │
│   livox_ros_driver2 (Livox LiDAR 驱动)            │
│   teleop_twist_joy (手柄遥控)                      │
└─────────────────────────────────────────────────┘
```

### 1.4 启动路径与入口

**一键启动导航**：`ros2 launch nav_bringup legged_navigation_launch.py`

启动脚本 `legged_navigation_launch.py` 按以下顺序启动 12 个节点：
1. `terrain_analysis` / `terrain_analysis_ext` — 地形分析（先启动，等传感器数据）
2. `loam_interface` — 坐标变换中间层
3. `sensor_scan_generation` — 时间同步与里程计生成（可选）
4. `controller_server` → `planner_server` → `smoother_server` — Nav2 核心
5. `behavior_server` — 恢复行为
6. `bt_navigator` — 行为树执行引擎
7. `waypoint_follower` — 路径点跟踪
8. `velocity_smoother` — 速度平滑
9. `lifecycle_manager_navigation` — 生命周期管理 (自动激活各节点)

**Topic 流转**：
```
cmd_vel 流水线:
  bt_navigator → cmd_vel_nav2_result → velocity_smoother → cmd_vel → 机器人底盘
```

---

## 2. 文件清单

### 2.1 C++ 源文件

| 文件路径 | 模块 | 职责摘要 | 行数 |
|----------|------|----------|------|
| `src/navigation/nav2_plugins/src/bt/action/hold_stop_flag.cpp` | nav2_plugins | BT动作：发布停止指令并保持一段时间 | ~90 |
| `src/navigation/nav2_plugins/src/bt/action/pub_spin_speed.cpp` | nav2_plugins | BT动作：发布固定旋转速度 | ~70 |
| `src/navigation/nav2_plugins/src/bt/action/pub_twist.cpp` | nav2_plugins | BT动作：发布一次性速度指令 | ~85 |
| `src/navigation/nav2_plugins/src/bt/action/publish_nav_goal.cpp` | nav2_plugins | BT动作：发布导航目标点（Rviz交互） | ~80 |
| `src/navigation/nav2_plugins/src/bt/action/select_fixed_path.cpp` | nav2_plugins | BT动作：选取固定路径（硬编码路径点） | ~100 |
| `src/navigation/nav2_plugins/src/bt/action/select_path_goal_pose.cpp` | nav2_plugins | BT动作：从路径数组中选取目标 | ~90 |
| `src/navigation/nav2_plugins/src/bt/action/select_patrol_path.cpp` | nav2_plugins | BT动作：巡逻路径轮询选取 | ~115 |
| `src/navigation/nav2_plugins/src/bt/action/send_nav2_goal.cpp` | nav2_plugins | BT动作：发送 NavigateToPose 目标 | ~85 |
| `src/navigation/nav2_plugins/src/bt/action/send_nav_through_poses.cpp` | nav2_plugins | BT动作：发送 NavigateThroughPoses 目标 | ~70 |
| `src/navigation/nav2_plugins/src/bt/condition/is_path_goal_reached.cpp` | nav2_plugins | BT条件：检查路径是否完全走完 | ~100 |
| `src/navigation/nav2_plugins/src/bt/control/recovery_node.cpp` | nav2_plugins | BT控制：自定义 Recovery 节点 | ~120 |
| `src/navigation/nav2_plugins/src/bt/decorator/rate_controller.cpp` | nav2_plugins | BT装饰器：控制子节点执行频率 | ~90 |
| `src/navigation/nav2_plugins/src/behaviors/back_up_free_space.cpp` | nav2_plugins | 行为：在自由空间中倒退（轮式遗留） | ~60 |
| `src/navigation/nav2_plugins/src/layers/intensity_voxel_layer.cpp` | nav2_plugins | 代价地图层：基于强度的3D体素障碍物感知 | ~400 |
| `src/navigation/loam_interface/src/loam_interface.cpp` | loam_interface | 坐标系翻译节点：lidar_odom→odom | 93 |
| `src/navigation/sensor_scan_generation/src/sensor_scan_generation.cpp` | sensor_scan | 时间同步器：对齐里程计+点云，计算速度 | 152 |
| `src/navigation/small_gicp_relocalization/src/small_gicp_relocalization.cpp` | small_gicp | 基于GICP的全局重定位 | ~300 |
| `src/navigation/omni_pid_pursuit_controller/src/omni_pid_pursuit_controller.cpp` | omni_pid | PID纯追踪控制器主实现 | 764 |
| `src/navigation/omni_pid_pursuit_controller/src/pid.cpp` | omni_pid | 通用PID控制器实现 | 50 |
| `src/navigation/terrain_analysis/src/terrainAnalysis.cpp` | terrain | 近程地形分析（10m内） | 682 |
| `src/navigation/terrain_analysis_ext/src/terrainAnalysisExt.cpp` | terrain_ext | 远程地形分析（40m外）+ 连通性检查 | 557 |
| `src/navigation/teleop_twist_joy/src/pb_teleop_twist_joy.cpp` | teleop | 手柄遥控twist生成 | ~150 |
| `src/navigation/pointcloud_to_laserscan/src/pointcloud_to_laserscan_node.cpp` | pcd2scan | 3D点云→2D激光扫描行 | ~200 |
| `src/navigation/pointcloud_to_laserscan/src/laserscan_to_pointcloud_node.cpp` | pcd2scan | 2D激光扫描→3D点云 | ~120 |
| `src/navigation/pointcloud_to_laserscan/src/dummy_pointcloud_publisher.cpp` | pcd2scan | 测试用的假点云发布者 | ~60 |
| `src/navigation/ign_sim_pointcloud_tool/src/point_cloud_converter.cpp` | ign_sim | Ignition Gazebo点云格式转换 | ~100 |
| `src/navigation/point_lio/src/laserMapping.cpp` | point_lio | LiDAR-IMU 紧耦合状态估计主循环 | ~800+ |
| `src/navigation/point_lio/src/Estimator.cpp` | point_lio | IESKF 迭代误差状态卡尔曼滤波器 | ~900+ |
| `src/navigation/point_lio/src/IMU_Processing.cpp` | point_lio | IMU数据前向/后向传播 | ~300+ |
| `src/navigation/point_lio/src/preprocess.cpp` | point_lio | 点云预处理（去畸变、降采样） | ~200+ |
| `src/navigation/point_lio/src/parameters.cpp` | point_lio | 参数加载与解析 | ~100 |
| `src/navigation/point_lio/src/li_initialization.cpp` | point_lio | LiDAR-IMU 初始化（重力对齐） | ~200+ |
| `src/navigation/livox_ros_driver2/src/livox_ros_driver2.cpp` | livox | Livox驱动主入口 | ~80 |
| `src/navigation/livox_ros_driver2/src/driver_node.cpp` | livox | ROS2驱动节点实现 | ~150 |
| `src/navigation/livox_ros_driver2/src/lds.cpp` | livox | LiDAR设备抽象层 | ~400+ |
| `src/navigation/livox_ros_driver2/src/lds_lidar.cpp` | livox | LiDAR实例管理 | ~500+ |
| `src/navigation/livox_ros_driver2/src/lddc.cpp` | livox | LiDAR数据分发控制器 | ~200+ |
| `src/navigation/livox_ros_driver2/src/call_back/*.cpp` (2文件) | livox | SDK回调封装：点云+IMU | ~200 |
| `src/navigation/livox_ros_driver2/src/comm/*.cpp` (6文件) | livox | 通信层：队列、发布、缓存、信号量 | ~500+ |
| `src/navigation/livox_ros_driver2/src/parse_cfg_file/*.cpp` (2文件) | livox | JSON配置解析 | ~200 |
| `src/tools/pcd2pgm/src/pcd2pgm.cpp` | tools | PCD点云→PGM栅格地图转换 | 175 |
| `src/tools/rosbag2_composable_recorder/src/composable_recorder.cpp` | tools | 可组合ROS2录制器核心 | 146 |
| `src/tools/rosbag2_composable_recorder/src/composable_recorder_node.cpp` | tools | 录制器节点入口 | 16 |

### 2.2 Python 启动/工具文件

| 文件路径 | 模块 | 职责 | 行数 |
|----------|------|------|------|
| `src/navigation/nav_bringup/launch/legged_navigation_launch.py` | nav_bringup | 导航总启动 | 349 |
| `src/navigation/nav_bringup/launch/legged_localization_launch.py` | nav_bringup | 纯定位模式启动 | ~200 |
| `src/navigation/nav_bringup/launch/legged_slam_launch.py` | nav_bringup | SLAM建图模式启动 | ~150 |
| `src/navigation/point_lio/launch/point_lio.launch.py` | point_lio | Point-LIO启动 | ~80 |
| `src/navigation/loam_interface/launch/loam_interface_launch.py` | loam_interface | Loam界面启动 | ~50 |
| `src/navigation/small_gicp_relocalization/launch/small_gicp_relocalization_launch.py` | small_gicp | 重定位启动 | ~60 |
| `src/navigation/sensor_scan_generation/launch/sensor_scan_generation.launch.py` | sensor_scan | 传感器扫描启动 | ~40 |
| `src/navigation/teleop_twist_joy/launch/pb_teleop_twist_joy_launch.py` | teleop | 手柄遥控启动 | ~90 |
| `src/navigation/livox_ros_driver2/launch/msg_HAP_launch.py` | livox | HAP型号LiDAR启动 | ~60 |
| `src/navigation/livox_ros_driver2/launch/msg_MID360_launch.py` | livox | MID360型号LiDAR启动 | ~60 |
| `src/navigation/livox_ros_driver2/launch/rviz_HAP_launch.py` | livox | HAP + Rviz启动 | ~60 |
| `src/navigation/livox_ros_driver2/launch/rviz_MID360_launch.py` | livox | MID360 + Rviz启动 | ~60 |
| `src/navigation/livox_ros_driver2/launch/rviz_mixed.py` | livox | 混合型号 + Rviz启动 | ~70 |
| `src/simulation/nav2_loopback_sim/nav2_loopback_sim/loopback_simulator.py` | sim | 2D闭环仿真核心 | ~300+ |
| `src/simulation/nav2_loopback_sim/nav2_loopback_sim/tf_compat.py` | sim | TF兼容层（ROS1↔ROS2） | ~80 |
| `src/simulation/nav2_loopback_sim/nav2_loopback_sim/utils.py` | sim | 仿真工具函数 | ~50 |
| `src/simulation/nav2_loopback_sim/launch/bringup_launch.py` | sim | 仿真bringup | ~60 |
| `src/simulation/nav2_loopback_sim/launch/loopback_simulation.launch.py` | sim | 闭环仿真启动 | ~60 |
| `src/tools/pcd2pgm/launch/pcd2pgm_launch.py` | tools | PCD→PGM工具启动 | ~40 |
| `src/tools/rosbag2_composable_recorder/launch/recorder.launch.py` | tools | 录制器启动 | 52 |
| `src/tools/rosbag2_composable_recorder/src/start_recording.py` | tools | Python录制触发脚本 | 44 |

### 2.3 关键配置文件

| 文件路径 | 模块 | 职责 |
|----------|------|------|
| `src/navigation/nav_bringup/config/nav2_params.legged.yaml` | nav_bringup | 四足实机全部导航参数 (582行) |
| `src/navigation/nav_bringup/config/nav2_params.legged_sim.yaml` | nav_bringup | 四足仿真导航参数 |
| `src/navigation/nav_bringup/behavior_trees/legged_navigate_w_replanning_and_recovery.xml` | nav_bringup | 四足导航行为树 |
| `src/navigation/nav_bringup/description/quadruped.urdf` | nav_bringup | 四足URDF骨架模型 |
| `src/navigation/point_lio/config/mid360.yaml` | point_lio | MID-360 LiDAR算法参数 |
| `src/navigation/point_lio/config/avia.yaml` | point_lio | Avia LiDAR算法参数 |
| `src/navigation/teleop_twist_joy/config/xbox.config.yaml` | teleop | Xbox手柄按键映射 |
| `src/navigation/livox_ros_driver2/config/MID360_config.json` | livox | MID-360硬件配置 |
| `src/tools/pcd2pgm/config/pcd2pgm.yaml` | tools | PCD→PGM参数 |
| `src/simulation/nav2_loopback_sim/params/nav2_params.yaml` | sim | 仿真导航参数 |
| `src/simulation/nav2_loopback_sim/maps/*.yaml` | sim | 仿真地图元数据 (6张地图) |

---

## 3. 函数/方法级详解

### 3.1 loam_interface — 坐标系翻译节点

#### 类：LoamInterfaceNode (`rclcpp::Node`)

- **文件**：`src/navigation/loam_interface/include/loam_interface/loam_interface.hpp` (49行) / `src/loam_interface.cpp` (93行)
- **插件导出**：`RCLCPP_COMPONENTS_REGISTER_NODE(loam_interface::LoamInterfaceNode)`
- **命名空间**：`loam_interface`
- **职责**：将 Point-LIO 输出的 `lidar_odom` 坐标系下的里程计和点云，变换到 ROS TF 树的标准 `odom` 坐标系。

**成员变量**：

| 名称 | 类型 | 用途 |
|------|------|------|
| `state_estimation_topic_` | `std::string` | 订阅的里程计话题名（默认 `"aft_mapped_to_init"`） |
| `registered_scan_topic_` | `std::string` | 订阅的点云话题名（默认 `"cloud_registered"`） |
| `odom_frame_` | `std::string` | 目标 TF 帧名（默认 `"odom"`） |
| `base_frame_` | `std::string` | 机器人基座帧名（从YAML配置） |
| `lidar_frame_` | `std::string` | LiDAR帧名（从YAML配置） |
| `base_frame_to_lidar_initialized_` | `bool` | TF初始化标记 |
| `tf_odom_to_lidar_odom_` | `tf2::Transform` | 缓存的 odom→lidar_odom 变换 |
| `tf_buffer_` / `tf_listener_` | TF2 缓冲区/监听器 | |
| `pcd_pub_` / `odom_pub_` | Publisher | 发布变换后的点云和里程计 |
| `pcd_sub_` / `odom_sub_` | Subscription | 订阅原始点云和里程计 |

#### 函数：LoamInterfaceNode(const rclcpp::NodeOptions & options)

- **用途**：构造函数，声明5个参数(`state_estimation_topic`, `registered_scan_topic`, `odom_frame`, `base_frame`, `lidar_frame`)，初始化 TF 缓冲区，创建2个订阅者和2个发布者。
- **副作用**：创建 ROS 话题通信基础设施。

#### 函数：pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)

- **用途**：将 lidar_odom 坐标系下的点云变换到 odom 坐标系。
- **核心算法**：调用 `pcl_ros::transformPointCloud(odom_frame_, tf_odom_to_lidar_odom_, *msg, *out)` 完成单步TF变换。
- **分支路径**：
  - 无条件执行（无异常处理——依赖缓存的`tf_odom_to_lidar_odom_`必须有效）
- **副作用**：发布变换后点云到 `registered_scan` 话题。
- **前置条件**：`base_frame_to_lidar_initialized_` 必须为 true（由 odometryCallback 设置）。⚠️ 如果先收到点云后收到里程计，会使用未初始化的 identity 变换。

#### 函数：odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)

- **用途**：将 Point-LIO 输出的 lidar_odom → LiDAR 的里程计变换到 odom → LiDAR。
- **核心算法**：
  1. 首次调用时，通过 TF 查询 `base_frame`→`lidar_frame` 的静态变换，缓存为 `tf_odom_to_lidar_odom_`。
  2. 将里程计中的 pose 解析为 `tf_lidar_odom_to_lidar`。
  3. 链式变换：`tf_odom_to_lidar = tf_odom_to_lidar_odom_ * tf_lidar_odom_to_lidar`。
  4. 构造新的 Odometry 消息，header.frame_id = `odom_frame_`，child_frame_id = `lidar_frame_`。
- **分支路径**：
  - **首次初始化**：TF lookup 成功 → 设置 `base_frame_to_lidar_initialized_ = true`。
  - **TF lookup 异常**：捕获 `tf2::TransformException` → 打印 WARN，return（不处理本条里程计消息）。
  - **已初始化**：直接用缓存变换。
- **副作用**：发布变换后的里程计到 `lidar_odometry` 话题。
- **前置条件**：TF 树中 `base_frame`→`lidar_frame` 可用（由 URDF robot_state_publisher 发布）。

---

### 3.2 sensor_scan_generation — 传感器时间同步与里程计生成

#### 类：SensorScanGenerationNode (`rclcpp::Node`)

- **文件**：`src/navigation/sensor_scan_generation/include/.../sensor_scan_generation.hpp` / `src/sensor_scan_generation.cpp` (152行)
- **插件导出**：`RCLCPP_COMPONENTS_REGISTER_NODE(...)`

**成员变量**：

| 名称 | 类型 | 用途 |
|------|------|------|
| `lidar_frame_` / `base_frame_` / `robot_base_frame_` | `std::string` | TF帧名配置 |
| `tf_lidar_to_robot_base_` | `tf2::Transform` | 缓存的LiDAR→机体变换 |
| `tb_buffer_` / `tf_listener_` / `br_` | TF2工具 | 查询和广播TF |
| `pub_laser_cloud_` | Publisher | 发布传感器扫描 (`sensor_scan`) |
| `pub_chassis_odometry_` | Publisher | 发布机体里程计 (`odometry`) |
| `odometry_sub_` + `laser_cloud_sub_` + `sync_` | 消息滤波器同步器 | 时间对齐输入 |

#### 函数：laserCloudAndOdometryHandler(odometry_msg, pcd_msg)

- **用途**：时间对齐后的回调，合成三个输出：
  1. **TF广播**：发布 `odom → base_frame` 的变换。
  2. **里程计发布**：发布 `odom → robot_base_frame` 的 Odometry 消息，包含速度估计。
  3. **点云发布**：发布去畸变后的点云（从LiDAR坐标系变换到 odom 坐标系）。
- **核心算法**：
  - TF链组合：`tf_odom_to_chassis = odom_pose * tf_lidar_to_base_frame`
  - 速度估计：通过前后帧位置差分除以时间间隔计算线速度和角速度。
- **分支路径**：
  - **dt > 0**：计算并填充 twist 速度。
  - **dt = 0**：跳过速度计算（首帧或帧重复）。
- **副作用**：每个回调周期发布2个消息（点云+里程计）+ 1个TF广播。
- **内部调用**：`getTransform()`, `publishTransform()`, `publishOdometry()`

#### 函数：getTransform(target_frame, source_frame, time) → tf2::Transform

- **用途**：带异常处理的TF查询。
- **分支路径**：
  - **查询成功**：返回变换。
  - **异常**：捕获 `tf2::TransformException`，打印 WARN，返回 identity 变换。⚠️ 静默退化可能导致里程计漂移。

#### 函数：publishOdometry(transform, parent_frame, child_frame, stamp)

- **用途**：发布里程计并估计速度。
- **核心算法**：维护 `static previous_transform` 和 `static previous_time`。速度 = (当前位置 - 前一位置) / dt。角速度通过四元数差分计算。
- ⚠️ **问题**：使用 `static` 局部变量存储前一帧状态，多实例共享？单节点没问题，但如果作为 ComposableNode 多实例化，`static` 变量会跨实例共享。

---

### 3.3 omni_pid_pursuit_controller — PID纯追踪控制器

#### 类：PID

- **文件**：`pid.hpp` (34行) / `pid.cpp` (50行)
- **命名空间**：无

**成员变量**：

| 名称 | 类型 | 默认 | 用途 |
|------|------|------|------|
| `dt_` | double | 构造参数 | 控制周期 |
| `max_` / `min_` | double | 构造参数 | 输出限幅 |
| `kp_` / `ki_` / `kd_` | double | 构造参数 | PID增益 |
| `pre_error_` | double | 0 | 前一帧误差(D项) |
| `integral_` | double | 0 | 积分累加(I项) |

#### 函数：PID::calculate(set_point, pv) → double

- **用途**：标准 PID 计算。
- **核心算法**：
  1. error = set_point - pv
  2. P_out = kp * error
  3. integral += error * dt; I_out = ki * integral
  4. derivative = (error - pre_error) / dt; D_out = kd * derivative
  5. output = P_out + I_out + D_out
  6. clamp(output, min, max)
  7. pre_error = error
- **分支路径**：
  - **integral > 1**：钳位到1（抗积分饱和）。
  - **integral < -1**：钳位到-1。
  - **output > max**：钳位到max。
  - **output < min**：钳位到min。
- ⚠️ **硬编码积分钳位**：积分饱和阈值为 ±1，不可配置。对于不同的速度范围可能不够灵活。

#### 类：OmniPidPursuitController (`nav2_core::Controller`)

- **文件**：`omni_pid_pursuit_controller.hpp` / `omni_pid_pursuit_controller.cpp` (764行)
- **插件导出**：`PLUGINLIB_EXPORT_CLASS(..., nav2_core::Controller)`

**关键成员变量(非参数)**：

| 名称 | 类型 | 用途 |
|------|------|------|
| `move_pid_` | `std::shared_ptr<PID>` | 平移方向PID |
| `heading_pid_` | `std::shared_ptr<PID>` | 旋转方向PID |
| `global_plan_` | `nav_msgs::msg::Path` | 缓存的全局路径 |
| `last_velocity_scaling_factor_` | `double` | 上一帧速度缩放因子（曲率平滑） |
| `has_last_velocity_scaling_` | `bool` | 曲率平滑初始化标记 |
| `mutex_` | `std::mutex` | 保护重入（动态参数 + 速度计算） |

**全部 ROS 参数 (40个)**：

| 参数名 | 默认 | 用途 |
|--------|------|------|
| `translation_kp` | 3.0 | 平移比例增益 |
| `translation_ki` | 0.1 | 平移积分增益 |
| `translation_kd` | 0.3 | 平移微分增益 |
| `enable_rotation` | true | 是否启用旋转控制 |
| `rotation_kp` | 3.0 | 旋转比例增益 |
| `rotation_ki` | 0.1 | 旋转积分增益 |
| `rotation_kd` | 0.3 | 旋转微分增益 |
| `transform_tolerance` | 0.1 | TF变换超时(秒) |
| `min_max_sum_error` | 1.0 | 最小求和误差 ⚠️未使用 |
| `lookahead_dist` | 0.3 | 静态预瞄距离(m) |
| `use_velocity_scaled_lookahead_dist` | true | 速度比例预瞄 |
| `min_lookahead_dist` | 0.2 | 最小预瞄距离 |
| `max_lookahead_dist` | 1.0 | 最大预瞄距离 |
| `lookahead_time` | 1.0 | 预瞄时间(秒) |
| `use_interpolation` | true | 预瞄点插值 |
| `use_rotate_to_heading` | true | 接近终点时先转方向 |
| `use_rotate_to_heading_treshold` | 0.1 | 转向阈值(rad) |
| `min_approach_linear_velocity` | 0.05 | 最小接近速度 |
| `approach_velocity_scaling_dist` | 0.6 | 速度缩放起始距离 |
| `v_linear_min` | -3.0 | 最小线速度(m/s) |
| `v_linear_max` | 3.0 | 最大线速度(m/s) |
| `v_angular_min` | -3.0 | 最小角速度(rad/s) |
| `v_angular_max` | 3.0 | 最大角速度(rad/s) |
| `max_robot_pose_search_dist` | costmap_extent | 最近路径点搜索范围 |
| `curvature_min` | 0.4 | 曲率减速起始 |
| `curvature_max` | 0.7 | 最大曲率减速比 |
| `reduction_ratio_at_high_curvature` | 0.5 | 高曲率速度缩减比 |
| `curvature_forward_dist` | 0.7 | 曲率前向采样距离 |
| `curvature_backward_dist` | 0.3 | 曲率后向采样距离 |
| `max_velocity_scaling_factor_rate` | 0.9 | 速度缩放变化率上界 |

#### 函数：configure(parent, name, tf, costmap_ros)

- **用途**：声明并读取40个参数，创建PID实例和3个发布器(local_plan, lookahead_point, curvature_points_marker_array)。
- ⚠️ **问题**：`approach_velocity_scaling_dist_ > costmap_size/2` 时仅打印WARN，不自动修正。
- **内部调用**：`getCostmapMaxExtent()`

#### 函数：computeVelocityCommands(pose, velocity, goal_checker) → TwistStamped

- **用途**：每个控制周期调用一次，计算速度指令。
- **核心算法**：
  1. **路径变换**：`transformGlobalPlan(pose)` 将全局路径转到机器人坐标系。
  2. **预瞄距离**：`getLookAheadDistance(velocity)` — 静态或速度比例。
  3. **预瞄点**：`getLookAheadPoint(lookahead_dist, transformed_plan)` — 找到预瞄点(含线段-圆插值)。
  4. **PID计算**：
     - `move_pid_->calculate(lin_dist, 0)` — 线距离→线速度
     - `heading_pid_->calculate(angle_to_goal, 0)` — 角偏差→角速度
  5. **曲率限速**：`applyCurvatureLimitation()` — 根据路径曲率降低速度。
  6. **接近减速**：`applyApproachVelocityScaling()` — 靠近终点时线性减速。
  7. **碰撞检测**：`isCollisionDetected()` — 采样10个路径点检查代价。
  8. 速度分解：`vx = lin_vel * cos(theta_dist)`, `vy = lin_vel * sin(theta_dist)`。
- **分支路径**：
  - **use_rotate_to_heading**：终点方向偏差 > 阈值 → 线速度=0（原地转向）。
  - **碰撞检测失败**：抛出 `PlannerException`。
- **副作用**：发布 local_plan、carrot、curvature markers。

#### 函数：applyCurvatureLimitation(path, lookahead_pose, linear_vel)

- **核心算法**：
  1. 计算预瞄点周围三个点（后-中-前）的曲率半径：`calculateCurvatureRadius()`。
  2. 若曲率 > `curvature_min`，按比例减速。若 > `curvature_max`，用最大减速比。
  3. 减速变化率受 `max_velocity_scaling_factor_rate` 约束，避免急减速。
  4. 最小速度不小于 `2 * min_approach_linear_velocity`。

#### 函数：calculateCurvatureRadius(near, current, far) → double

- **核心算法**：三点共圆求半径。圆心通过解三点的垂直平分线交点得到。半径 = 圆心到任一点距离。
- **保护**：`isnan(radius) || isinf(radius) || radius < 1e-9` → 返回 1e9（无穷大曲率半径 = 直线）。

#### 函数：dynamicParametersCallback(parameters) → SetParametersResult

- **用途**：运行时动态参数更新，支持23个 double 参数 + 3个 bool 参数。
- **副作用**：修改 `translation_kp_` 等成员变量（`move_pid_`/`heading_pid_` 内部使用引用吗？不——PID 在构造函数中接收值拷贝，动态参数修改后 PID 不更新！）。
- ⚠️ **严重Bug**：`move_pid_` 和 `heading_pid_` 在 `configure()` 中用初始 kp/kd/ki 值构造，但 `dynamicParametersCallback` 只更新控制器类的成员变量 `translation_kp_` 等，并不更新 PID 对象内部的 `kp_`/`ki_`/`kd_` 成员。**运行时调整 PID 参数无效**。

---

### 3.4 terrain_analysis — 近程地形分析

- **文件**：`src/navigation/terrain_analysis/src/terrainAnalysis.cpp` (682行)
- **架构**：单节点，全局变量模式（无类封装），`main()` 中完成所有逻辑。
- **节点名**：`terrainAnalysis`

#### 全局变量

所有计算状态均为全局变量，约30个。关键常量：

| 名称 | 值 | 用途 |
|------|-----|------|
| `terrainVoxelWidth` | 21 | 地形体素网格宽度 |
| `kTerrainVoxelNum` | 441 | 总地形体素数 (21×21) |
| `terrainVoxelSize` | 1.0m | 每个体素边长 |
| `planarVoxelWidth` | 51 | 平面体素网格宽度 |
| `kPlanarVoxelNum` | 2601 | 总平面体素数 (51×51) |
| `planarVoxelSize` | 0.2m | 每个平面体素边长 |

算法参数全部通过 ROS 参数系统声明和读取（26个参数），详见 nav2_params.legged.yaml 中 `terrain_analysis` 段。

#### 函数：odometryHandler(odom)

- **用途**：里程计回调，提取 `(x, y, z, roll, pitch, yaw)` 及三角缓存。
- **分支路径**：
  - `noDataInited == 0`：记录初始位置 → `noDataInited = 1`。
  - `noDataInited == 1`：距离超过 `noDecayDis` → `noDataInited = 2`（进入已初始化状态）。

#### 函数：laserCloudHandler(laserCloud2)

- **用途**：激光点云回调，裁剪有效范围并存储。
- **核心算法**：对每个点计算相对车辆的距离，通过 `minRelZ - disRatioZ*dis` 和 `maxRelZ + disRatioZ*dis` 两锥面界定有效区域。点云存入 `laserCloudCrop`，时间戳存入 `intensity` 字段作为后续衰减依据。

#### 主循环 (main外的全局 while loop)

**处理流程**（每帧 ~100Hz，仅在新数据到来时执行）：

1. **体素滚动**（Voxel Roll-over）：当车辆超出当前体素网格中心时，整行/整列移动体素数组（类似滑动窗口），清空离开区域的体素。

2. **点云堆叠**：将 `laserCloudCrop` 中每个点分配到对应体素，累加计数。

3. **体素更新**：满足以下任一条件触发单个体素更新：
   - 点数 ≥ `voxelPointUpdateThre` (100)
   - 时间 ≥ `voxelTimeUpdateThre` (1.0s)
   - 手动清除标志 `clearingCloud`
   
   更新时先降采样（`scanVoxelSize=0.02`），再根据时间衰减或距离保留点云。

4. **平面高程估计**：取车辆周围 11×11 个体素（~10m范围），合并到 `terrainCloud`。每个平面体素收集点的高度值，以分位数 `quantileZ=0.2` 作为地面高度估计。支持最大地面抬升限制 `maxGroundLift`。

5. **动态障碍物过滤**：通过坐标变换（Yaw→Pitch→Roll链式变换）判断点是否在车辆前方仰角范围内，标记为动态障碍物。

6. **无数据障碍物**：在已初始化区域中，数据不足的体素块被标记为障碍物（`noDataObstacle`）。

7. **发布**：高度着色点云 → `terrain_map` topic (frame_id=`odom`)。

---

### 3.5 terrain_analysis_ext — 远程地形分析

- **文件**：`src/navigation/terrain_analysis_ext/src/terrainAnalysisExt.cpp` (557行)
- **节点名**：`terrainAnalysisExt`

与 terrain_analysis 的关键差异：

| 属性 | terrain_analysis | terrain_analysis_ext |
|------|-----------------|---------------------|
| 体素大小 | 1.0m | 2.0m |
| 体素网格 | 21×21 (~20m) | 41×41 (~80m) |
| 平面体素大小 | 0.2m | 0.4m |
| 平面网格 | 51×51 (~10m) | 101×101 (~40m) |
| 关注范围 | 近程 10m | 远程 40m+ |
| 动态过滤 | ✅ | ❌ (无DyObs逻辑) |
| 连通性检查 | ❌ | ✅ (BFS地形连通) |
| 本地地图融合 | ❌ | ✅ (订阅 `terrain_map` 融合近程) |

#### 新增算法：地形连通性检查 (BFS)

从车辆所在体素 (`planarVoxelHalfWidth, planarVoxelHalfWidth`) 开始，BFS搜索相邻体素。若相邻体素高度差 < `terrainConnThre`(0.5m)，标记为连通。若高度差 > `ceilingFilteringThre`(2.0m)，标记为非连通（天花板过滤）。最终只有连通体素内的点才进入输出。

#### 新增算法：本地地图融合

`localTerrainMapRadius`(4.0m) 以内的区域不使用 `terrain_analysis_ext` 自身计算，而是从订阅的 `terrain_map`（即 terrain_analysis 的输出）中直接拷贝点，避免重复计算。

---

### 3.6 nav2_plugins — 自定义导航插件集

#### 3.6.1 BT Action: publish_nav_goal (PublishNavGoal)

- **文件**：`src/navigation/nav2_plugins/src/bt/action/publish_nav_goal.cpp`
- **基类**：`nav2_behavior_tree::BtActionNode<...>`
- **职责**：将当前目标点通过 `RvizClickPoint` topic 发布出去（用于Rviz交互点选目标）。

#### 3.6.2 BT Action: select_fixed_path (SelectFixedPath)

- **职责**：从 YAML 配置中读取硬编码路径点，按顺序选取。
- **输出端口**：`path` (nav_msgs/Path) — 预设路径。

#### 3.6.3 BT Action: select_patrol_path (SelectPatrolPath)

- **职责**：支持多条巡逻路径的轮询。每次调用选取下一条路径。
- **参数**：路径数组通过 ROS 参数 `paths` 获取。

#### 3.6.4 BT Action: send_nav2_goal (SendNav2Goal)

- **职责**：调用 Nav2 `NavigateToPose` action 发送单目标。

#### 3.6.5 BT Action: send_nav_through_poses (SendNavThroughPoses)

- **职责**：调用 Nav2 `NavigateThroughPoses` action 发送多目标序列。

#### 3.6.6 BT Action: hold_stop_flag (HoldStopFlag)

- **职责**：发布 `cmd_vel` 为零速度，并保持一段可配置时间。用于定点停止。

#### 3.6.7 BT Action: pub_spin_speed (PubSpinSpeed)

- **职责**：发布固定角速度的 `cmd_vel`，用于原地旋转。

#### 3.6.8 BT Action: pub_twist (PubTwist)

- **职责**：发布一次性的自定义速度指令（从 BT port 读取）。

#### 3.6.9 BT Condition: is_path_goal_reached (IsPathGoalReached)

- **职责**：判断当前路径的所有路径点是否都已经过。内部维护已访问路径点索引，通过欧氏距离阈值判断到达。

#### 3.6.10 BT Control: recovery_node (RecoveryNode)

- **职责**：自定义 Recovery 控制节点，增强 Nav2 的恢复行为逻辑。
- **基类**：`nav2_behavior_tree::ControlNode`

#### 3.6.11 BT Decorator: rate_controller (RateController)

- **职责**：限制子节点的执行频率到指定 Hz。

#### 3.6.12 Behavior: back_up_free_space (BackUpFreeSpace)

- **职责**：轮式哨兵的后退自由空间行为（⚠️ 轮式遗留，四足未使用，行为树中已替换为 Spin）。

#### 3.6.13 Costmap Layer: intensity_voxel_layer (IntensityVoxelLayer)

- **职责**：基于点云强度的 3D 体素障碍物层。将 `terrain_map` / `terrain_map_ext` 点云数据转换为代价地图的障碍物标记。
- **关键参数**：
  - `min_obstacle_intensity` / `max_obstacle_intensity`：强度范围过滤
  - `min_obstacle_height` / `max_obstacle_height`：高度范围过滤
  - `z_resolution` / `z_voxels`：Z轴体素参数
  - `mark_threshold`：标记阈值
  - `combination_method`：体素→2D投影方式

---

### 3.7 行为树：legged_navigate_w_replanning_and_recovery.xml

**文件**：`src/navigation/nav_bringup/behavior_trees/legged_navigate_w_replanning_and_recovery.xml`

```xml
RecoveryNode number_of_retries=10  ← 顶层恢复节点
  PipelineSequence  ← 顺序执行
    RateController hz=3.0  ← 每333ms重新规划一次
      RecoveryNode number_of_retries=1
        ComputePathToPose  ← 规划到目标
        ClearEntireCostmap (global)  ← 规划失败清空全局代价地图
    RecoveryNode number_of_retries=10
      FollowPath  ← 跟踪路径
      ClearEntireCostmap (local)  ← 跟随时失败清空局部代价地图
  ReactiveFallback  ← 恢复子树：任一条件达成则触发
    GoalUpdated  ← 目标已更新(取消当前)
    RoundRobin  ← 轮询恢复策略
      ClearCostmaps (local+global)  ← 先清空代价地图
      Spin spin_dist=3.14 is_recovery=true  ← 四足版：原地转180°
```

**与轮式版本差异**：RecoveryFallback 中用 `Spin` (π rad) 替代 `BackUp`，因四足不适于后退。

---

### 3.8 small_gicp_relocalization — 全局重定位

- **文件**：`src/navigation/small_gicp_relocalization/src/small_gicp_relocalization.cpp`
- **算法**：基于 [small_gicp](https://github.com/koide3/small_gicp) 的点云配准。
- **关键参数**：
  - `num_threads: 4` — 并行线程数
  - `num_neighbors: 20` — K近邻搜索数
  - `global_leaf_size: 0.15` — 全局地图降采样
  - `registered_leaf_size: 0.05` — 当前扫描降采样
  - `max_dist_sq: 3.0` — 最大配准距离平方阈值

#### 函数：loadGlobalMap(file_name)

- **用途**：加载全局先验 PCD 地图，并变换到 `odom` 坐标系。
- **核心算法**：循环等待 `base_frame → lidar_frame` TF 变换可用（✅ 已修复：最多重试 100 次，超时后 FATAL + shutdown，避免无限阻塞）。
- **分支路径**：
  - **PCD 加载失败**：记录 ERROR，return。
  - **TF 获取成功**：在 break 前对 global_map_ 做 `pcl::transformPointCloud`。
  - **TF 超时**（✅ 新增）：`retry_count >= kMaxRetries(100)` → `RCLCPP_FATAL` + `rclcpp::shutdown()`。

---

### 3.9 nav2_loopback_sim — 2D闭环仿真

- **文件**：`src/simulation/nav2_loopback_sim/nav2_loopback_sim/loopback_simulator.py`
- **原理**：在无物理引擎的情况下，通过图结构预定义路径+传感器模拟来验证导航逻辑。
- **核心组件**：
  - `loopback_simulator.py`：主模拟循环，发布 odom/tf/scan。
  - `tf_compat.py`：ROS1 TF ↔ ROS2 TF 兼容转换。
  - `utils.py`：辅助工具函数。
- **地图**：`maps/` 目录下6张仿真地图（depot, warehouse, tb3_sandbox 各含 keepout/speed 变体）。

---

## 4. 全局变量 & 常量索引

### 4.1 terrain_analysis (约30个全局变量)

| 名称 | 类型 | 可变 | 用途 |
|------|------|------|------|
| `scanVoxelSize` 等26个参数 | double/bool/int | ✅ (参数系统) | 算法参数，运行时可通过 YAML/命令行覆盖 |
| `laserCloud` | `pcl::PointCloud::Ptr` | ✅ | 当前帧原始点云 |
| `laserCloudCrop` | `pcl::PointCloud::Ptr` | ✅ | 裁剪后的有效点云 |
| `laserCloudDwz` | `pcl::PointCloud::Ptr` | ✅ | 降采样后的点云 |
| `terrainVoxelCloud[kTerrainVoxelNum]` | `pcl::PointCloud::Ptr[]` | ✅ | 体素网格点云数组 |
| `planarVoxelElev[kPlanarVoxelNum]` | `float[]` | ✅ | 每个平面体素的地面高度 |
| `planarVoxelDyObs[kPlanarVoxelNum]` | `int[]` | ✅ | 动态障碍物计数 |
| `vehicleX/Y/Z/Roll/Pitch/Yaw` | `float` | ✅ | 当前车辆位姿 |
| `noDataInited` | `int` | ✅ | 初始化状态机: 0=未初始化/1=已记录原点/2=已移动超noDecayDis |

### 4.2 terrain_analysis_ext (类似结构，去掉动态障碍物相关变量，增加连通性检查变量)

| 名称 | 类型 | 可变 | 用途 |
|------|------|------|------|
| `planarVoxelConn[kPlanarVoxelNum]` | `int[]` | ✅ | BFS连通性标记: 0=未访问/1=队列中/2=已连通/-1=非连通 |
| `planarVoxelQueue` | `std::queue<int>` | ✅ | BFS搜索队列 |
| `terrainCloudLocal` | `pcl::PointCloud::Ptr` | ✅ | 近程地形缓存(从terrain_map融合) |
| `kdtree` | `pcl::KdTreeFLANN` | ✅ | KD树(声明但主循环中未使用⚠️) |

### 4.3 sensor_scan_generation (static局部变量)

| 名称 | 类型 | 位置 | 用途 |
|------|------|------|------|
| `previous_transform` | `static tf2::Transform` | `publishOdometry()` 内 | 上一帧变换(速度估计) |
| `previous_time` | `static auto` | `publishOdometry()` 内 | 上一帧时间戳 |

### 4.4 omni_pid_pursuit_controller (static)

| 名称 | 类型 | 位置 | 用途 |
|------|------|------|------|
| 无全局变量 | — | — | 全部状态封装在类成员中 |

---

## 5. 类型/接口/枚举索引

| 名称 | 类别 | 定义位置 | 备注 |
|------|------|----------|------|
| `LoamInterfaceNode` | 类 (rclcpp::Node) | `loam_interface.hpp:17` | 坐标系翻译节点 |
| `SensorScanGenerationNode` | 类 (rclcpp::Node) | `sensor_scan_generation.hpp` | 传感器同步节点 |
| `OmniPidPursuitController` | 类 (nav2_core::Controller) | `omni_pid_pursuit_controller.hpp` | PID纯追踪控制器 |
| `PID` | 类 | `pid.hpp:6` | 通用PID控制器 |
| `PublishNavGoal` | 类 (BtActionNode) | `publish_nav_goal.hpp` | BT:发布导航目标 |
| `SelectFixedPath` | 类 (BtActionNode) | `select_fixed_path.hpp` | BT:选取固定路径 |
| `SelectPatrolPath` | 类 (BtActionNode) | `select_patrol_path.hpp` | BT:巡逻路径轮询 |
| `SendNav2Goal` | 类 (BtActionNode) | `send_nav2_goal.hpp` | BT:发送单个导航目标 |
| `SendNavThroughPoses` | 类 (BtActionNode) | `send_nav_through_poses.hpp` | BT:发送路径点序列 |
| `HoldStopFlag` | 类 (BtActionNode) | `hold_stop_flag.hpp` | BT:保持停止 |
| `PubSpinSpeed` | 类 (BtActionNode) | `pub_spin_speed.hpp` | BT:发布旋转速度 |
| `PubTwist` | 类 (BtActionNode) | `pub_twist.hpp` | BT:发布Twist |
| `IsPathGoalReached` | 类 (BtConditionNode) | `is_path_goal_reached.hpp` | BT:路径完成检查 |
| `RecoveryNode` | 类 (ControlNode) | `recovery_node.hpp` | BT:恢复控制节点 |
| `RateController` | 类 (DecoratorNode) | `rate_controller.hpp` | BT:频率控制 |
| `BackUpFreeSpace` | 类 (nav2_core::Behavior) | `back_up_free_space.hpp` | 行为:自由空间后退(轮式遗留) |
| `IntensityVoxelLayer` | 类 (nav2_costmap_2d::VoxelLayer) | `intensity_voxel_layer.hpp` | 强度体素代价地图层 |
| `CustomTypes` | 类 | `custom_types.hpp` | BT自定义类型 |
| `NavUtils` | 工具函数集 | `nav_utils.hpp` | 导航辅助函数 |
| `PointCloudConverter` | 类 | `point_cloud_converter.hpp` (ign_sim) | Gazebo点云格式转换 |
| `ComposableRecorder` | 类 | `composable_recorder.hpp` | 可组合录制器核心 |

---

## 6. 依赖关系图谱

### 6.1 包级别依赖链 (从传感器到执行器)

```
livox_ros_driver2 (硬件驱动)
    ↓ /livox/lidar (PointCloud2) + /livox/imu (Imu)
point_lio (LiDAR-IMU紧耦合里程计)
    ↓ /aft_mapped_to_init (Odometry) + /cloud_registered (PointCloud2)
loam_interface (坐标系翻译)
    ↓ /lidar_odometry (Odometry) + /registered_scan (PointCloud2)
    ├─────────────────────────────────────┐
    ↓                                     ↓
sensor_scan_generation              terrain_analysis
    ↓ /odometry + /sensor_scan          ↓ /terrain_map
    │                                     │
    └──────────┬──────────────────────────┘
               ↓
         nav2_plugins (代价地图层: IntensityVoxelLayer)
               ↓
          Nav2 导航栈
               ↓ /cmd_vel_nav2_result
    velocity_smoother
               ↓ /cmd_vel
          机器人底盘
```

### 6.2 nav_bringup 启动的节点依赖

```
legged_navigation_launch.py
├─ (不依赖其他节点)
│  ├─ terrain_analysis
│  └─ terrain_analysis_ext
├─ (依赖 LiDAR driver + point_lio)
│  ├─ loam_interface
│  └─ sensor_scan_generation (可选)
├─ Nav2 核心 (依赖 lifecycle_manager 自动激活)
│  ├─ controller_server (加载 omni_pid_pursuit_controller 插件)
│  ├─ planner_server (加载 SmacPlannerHybrid 插件)
│  ├─ smoother_server
│  ├─ behavior_server (加载 Spin/BackUp/DriveOnHeading/Wait)
│  ├─ bt_navigator (加载 behavior_plugin.xml 中注册的 BT 插件)
│  ├─ waypoint_follower
│  └─ velocity_smoother
└─ lifecycle_manager_navigation (管理上面7个 Nav2 节点的生命周期)
```

### 6.3 第三方依赖清单

| 依赖 | 版本 | 用途 | 影响范围 |
|------|------|------|----------|
| ROS 2 Humble | — | 核心通信框架 | 全部包 |
| Nav2 (navigation2) | Humble | 导航框架 | nav_bringup, nav2_plugins |
| PCL (libpcl-all-dev) | ≥1.12 | 点云处理 | terrain_analysis, pointcloud_to_laserscan, pcd2pgm |
| Eigen3 | — | 线性代数 | point_lio, omni_pid_pursuit_controller |
| BehaviorTree.CPP | v4 | 行为树引擎 | nav2_plugins |
| Livox-SDK2 | — | LiDAR 硬件 SDK | livox_ros_driver2 |
| tf2_ros / tf2_geometry_msgs | Humble | TF 坐标变换 | loam_interface, sensor_scan_generation, small_gicp |
| small_gicp | — | 点云配准 | small_gicp_relocalization |
| pluginlib | Humble | 插件加载 | nav2_plugins, omni_pid_pursuit_controller |
| rosbag2 | Humble | 数据录制 | rosbag2_composable_recorder |
| pcl_ros | Humble | PCL↔ROS桥接 | loam_interface, sensor_scan_generation |

### 6.4 关键调用链

**从目标设定到速度指令**：
```
NavigateToPose Action
  → bt_navigator (执行行为树)
    → ComputePathToPose (BT节点, 调用 planner_server)
    → FollowPath (BT节点, 调用 controller_server)
      → OmniPidPursuitController::computeVelocityCommands()
        → transformGlobalPlan()         // 路径转到机器人系
        → getLookAheadDistance()        // 动态预瞄距离
        → getLookAheadPoint()           // 预瞄点选取
        → circleSegmentIntersection()   // 线段-圆交点(插值)
        → PID::calculate(lin_dist)      // 线速度PID
        → PID::calculate(angle_to_goal) // 角速度PID
        → applyCurvatureLimitation()    // 曲率限速
          → calculateCurvature()        // 三点曲率
          → calculateCurvatureRadius()  // 三点共圆半径
        → applyApproachVelocityScaling() // 接近减速
        → isCollisionDetected()         // 碰撞检查
  → cmd_vel_nav2_result
  → velocity_smoother
  → cmd_vel → 底盘
```

---

## 7. 数据流

### 7.1 全局状态管理

系统**无全局状态管理器**。各节点独立维护各自状态：

| 节点 | 状态 | 持久化 |
|------|------|--------|
| terrain_analysis | 全局变量: 位姿/点云/体素网格 | ❌ |
| terrain_analysis_ext | 同上 + BFS连通性 | ❌ |
| loam_interface | 成员: TF缓存变换 | ❌ |
| sensor_scan_generation | static局部: 前一帧变换+时间 | ❌ |
| omni_pid_pursuit_controller | 成员: 全局路径/曲率缩放 | ❌ |
| point_lio | IKEF状态向量 | ❌ (PCD可保存) |
| small_gicp_relocalization | 先验地图点云 | ✅ (从文件加载) |

### 7.2 主要数据流路径

```
点云数据流:
  LiDAR硬件 → livox_ros_driver2 → /livox/lidar
    → point_lio (SLAM前端)
    → /cloud_registered
    → loam_interface (坐标系翻译)
    → /registered_scan
    ├→ terrain_analysis → /terrain_map (近程高度图)
    │   └→ IntensityVoxelLayer (local_costmap 障碍物)
    ├→ terrain_analysis_ext → /terrain_map_ext (远程高度图)
    │   └→ IntensityVoxelLayer (global_costmap 障碍物)
    └→ sensor_scan_generation → /sensor_scan (去畸变扫描)
        └→ pointcloud_to_laserscan → /obstacle_scan (2D)
            └→ slam_toolbox (建图)

速度指令流:
  用户目标 (Rviz / API)
    → bt_navigator (行为树)
    → controller_server (PID纯追踪)
    → /cmd_vel_nav2_result
    → velocity_smoother (加/减速约束)
    → /cmd_vel
    → 机器人底盘控制器

TF流:
  robot_state_publisher (URDF) → 静态TF (base_footprint→base_link→body→front_mid360)
  loam_interface → 动态TF: lidar_odom→odom
  sensor_scan_generation → 动态TF: odom→base_footprint (发布)
  point_lio → 动态里程计 (lidar_odom 坐标系)
```

### 7.3 异步/事件驱动模型

- **主循环模式**：`terrain_analysis` / `terrain_analysis_ext` 使用 `rclcpp::Rate(100)` 轮询模式（100Hz）。
- **回调模式**：`loam_interface`、`sensor_scan_generation` 使用 ROS2 订阅者回调。
- **同步模式**：`sensor_scan_generation` 使用 `message_filters::Synchronizer` 对齐 odometry + pointcloud。
- **Action模式**：Nav2 使用 ROS2 Action Server/Client 模式处理长时间运行的任务（导航、规划、控制）。
- **插件模式**：`nav2_plugins` 通过 BehaviorTree.CPP 插件机制动态加载；`omni_pid_pursuit_controller` 通过 Nav2 `pluginlib` 机制动态加载。

---

## 8. 配置 & 环境变量

主配置文件 `nav2_params.legged.yaml` (582行) 包含全部节点的参数，按节点名分组。详细参数说明见各函数级拆解中的参数表。

### 8.1 四足特有的关键参数调整

| 参数路径 | 原轮式值 | 四足值 | 原因 |
|----------|----------|--------|------|
| `controller_server.FollowPath.translation_kp` | 3.0 | 1.5 | 四足响应慢 |
| `controller_server.FollowPath.v_linear_max` | 4.5 | 1.5 | 四足速度低 |
| `controller_server.FollowPath.v_angular_max` | 3.0 | 1.5 | 转向更保守 |
| `velocity_smoother.max_velocity` | [3.5,3.5,5.0] | [1.2,1.2,1.2] | 大幅减速 |
| `velocity_smoother.max_accel` | [4.5,4.5,5.0] | [0.8,0.8,1.0] | 柔和加速 |
| `local_costmap.robot_radius` | 0.3 | 0.45 | 腿部外展 |
| `planner_server.minimum_turning_radius` | 0.07 | 0.0 | 四足可原地转身 |
| `terrain_analysis.vehicleHeight` | 1.5 | 0.6 | 四足躯干高度 |

### 8.2 编译相关环境

| 变量 | 默认 | 用途 |
|------|------|------|
| `CMAKE_BUILD_TYPE` | Release | 编译优化级别 |
| `AMENT_PREFIX_PATH` | (colcon自动) | ROS2包搜索路径 |
| `RCUTILS_LOGGING_BUFFERED_STREAM` | 1 (launch设置) | 日志缓冲 |
| `RCUTILS_COLORIZED_OUTPUT` | 1 (launch设置) | 彩色日志 |

### 8.3 运行时参数覆盖

```bash
# 仿真模式
ros2 launch nav_bringup legged_navigation_launch.py \
  use_sim_time:=true \
  params_file:=src/navigation/nav_bringup/config/nav2_params.legged_sim.yaml

# 纯定位模式
ros2 launch nav_bringup legged_localization_launch.py \
  map:=/path/to/map.yaml \
  prior_pcd:=/path/to/prior.pcd

# SLAM模式
ros2 launch nav_bringup legged_slam_launch.py
```

---

## 附录 A: 包依赖关系矩阵

| 包 | 依赖 | 被依赖 |
|----|------|--------|
| nav_bringup | 全部导航包 | — (顶层集成) |
| nav2_plugins | Nav2, BT.CPP | nav_bringup |
| omni_pid_pursuit_controller | Nav2 | nav_bringup |
| loam_interface | PCL, tf2 | sensor_scan_generation, terrain_analysis |
| sensor_scan_generation | PCL, tf2 | nav_bringup |
| terrain_analysis | PCL | nav_bringup |
| terrain_analysis_ext | PCL, terrain_analysis(数据依赖) | nav_bringup |
| point_lio | Eigen, PCL | loam_interface |
| small_gicp_relocalization | small_gicp, PCL | nav_bringup |
| pointcloud_to_laserscan | PCL | slam_toolbox |
| livox_ros_driver2 | Livox-SDK2 | point_lio, loam_interface |
| teleop_twist_joy | joy, Nav2 | nav_bringup |
| ign_sim_pointcloud_tool | ignition, PCL | 仿真 |
| nav2_loopback_sim | Nav2 | 仿真验证 |
| pcd2pgm | PCL, OpenCV | 离线工具 |
| rosbag2_composable_recorder | rosbag2 | 数据采集工具 |

---

## 附录 B: 修复记录

| 提交 | 日期 | 修复问题 | 涉及文件 |
|------|------|----------|----------|
| `289db1a` | 2026-07-04 | #2 数组越界保护 | `pointcloud_to_laserscan_node.cpp` |
| | | #3 恢复 mutex 并发保护 | `li_initialization.cpp` |
| | | #4 析构函数 atomic flag 修正 | `laserscan_to_pointcloud_node.cpp` |
| | | #10 TF 等待超时保护 | `small_gicp_relocalization.cpp` |

---

> 📝 **文档维护**：本文档覆盖全部 16 个 ROS2 包、约 150 个文件、约 21,800 行代码的全部函数/方法。后续任何源码修改必须在对应章节同步更新。
