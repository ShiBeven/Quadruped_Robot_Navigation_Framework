# Quadruped Robot Navigation Framework

面向四足机器人的自主导航框架,基于 ROS 2 与 Navigation2 构建。以固态激光雷达为核心传感器,通过紧耦合的激光-惯性里程计完成状态估计,结合地形可通行性分析生成代价地图,交由 Nav2 完成路径规划与运动控制,形成从感知到底盘指令的完整闭环。

框架采用标准 colcon 工作空间组织,各功能包通过 ROS 2 话题与 TF 松耦合,可整体部署,也可按需抽取单个功能包复用。

## 特性

- **激光-惯性里程计**:集成 Point-LIO,基于 IKFoM 流形迭代扩展卡尔曼滤波,逐点更新,输出高频里程计与配准点云。
- **地形可通行性分析**:局部与扩展尺度双层地形分析,基于体素网格估计地面高程,输出可通行代价点云,并支持动态障碍剔除与地形连通性(天花板)过滤。
- **全向纯追踪控制器**:面向全向底盘的 Nav2 控制器插件,纯追踪选点结合双路 PID,叠加曲率限速与接近减速。
- **Nav2 扩展插件**:强度过滤的三维体素代价层、行为树决策/巡逻节点、恢复行为等,通过 pluginlib 动态加载。
- **多场景启动**:定位、导航、建图(SLAM)三套启动配置,并提供无物理引擎的回环仿真器用于快速验证。
- **传感器驱动与工具**:Livox 激光雷达 ROS 2 驱动、点云与栅格地图转换、可组合数据录制等配套工具。

## 系统架构

数据流为单向管线,自传感器输入至底盘速度指令:

```
Livox 激光雷达 (点云 + IMU)
        │
        ▼
   point_lio ──────────────── 激光-惯性里程计
        │  里程计 + 配准点云
        ▼
   loam_interface ─────────── 坐标系桥接 (输出 odom 帧)
        │
        ▼
   sensor_scan_generation ─── 时间同步, 生成底盘里程计与 TF
        │
        ├──► terrain_analysis ──────► terrain_map      (局部代价)
        └──► terrain_analysis_ext ──► terrain_map_ext  (全局代价)
        │
        ▼
   Nav2
     代价地图 (静态层 + 强度体素层 + 膨胀层)
     规划器 (SmacPlannerHybrid)
     行为树导航 + 恢复行为
     控制器 (omni_pid_pursuit_controller)
        │
        ▼
   速度平滑 ──► cmd_vel ──► 四足底盘
```

定位由 `small_gicp_relocalization` 基于先验点云地图配准提供 `map → odom` 变换;建图模式下由 slam_toolbox 承担。

完整的模块契约、数据流与设计说明见 [`docs/`](docs/) 目录下的分层理解文档。

## 功能包一览

### 导航 (`src/navigation/`)

| 功能包 | 说明 |
| --- | --- |
| `point_lio` | 激光-惯性里程计,输出高频位姿与配准点云 |
| `livox_ros_driver2` | Livox 三维激光雷达 ROS 2 驱动 |
| `loam_interface` | 里程计坐标系桥接适配层 |
| `sensor_scan_generation` | 时间同步,生成底盘里程计、TF 与局部扫描 |
| `terrain_analysis` | 局部地形可通行性分析 |
| `terrain_analysis_ext` | 扩展尺度地形分析,含地形连通性过滤 |
| `nav2_plugins` | Nav2 扩展:强度体素代价层、行为树节点、恢复行为 |
| `omni_pid_pursuit_controller` | 全向纯追踪 + PID 控制器插件 |
| `pointcloud_to_laserscan` | 点云与二维激光扫描互转 |
| `small_gicp_relocalization` | 基于先验地图的 GICP 重定位 |
| `ign_sim_pointcloud_tool` | 仿真点云格式转换 |
| `teleop_twist_joy` | 手柄遥操作 |
| `nav_bringup` | 顶层编排:启动文件、参数与机器人描述 |

### 仿真 (`src/simulation/`)

| 功能包 | 说明 |
| --- | --- |
| `nav2_loopback_sim` | 无物理引擎的回环仿真器,由速度指令积分伪造里程计、TF 与激光扫描 |

### 工具 (`src/tools/`)

| 功能包 | 说明 |
| --- | --- |
| `pcd2pgm` | 点云地图转占据栅格地图 |
| `rosbag2_composable_recorder` | 可组合的 rosbag2 录制节点 |

## 依赖

- ROS 2 与 Navigation2(含 `nav2_smac_planner`、`nav2_costmap_2d`、`nav2_behavior_tree` 等)
- PCL、Eigen
- BehaviorTree.CPP(经 `dependencies/BehaviorTree.ROS2` 提供 ROS 2 桥接)
- Livox-SDK2(随 `livox_ros_driver2` 提供)
- slam_toolbox(建图模式)

`dependencies/` 目录内置了部分第三方功能包(BehaviorTree.ROS2、joint_state_publisher、sdformat_tools)以简化依赖获取。其余系统依赖建议通过 `rosdep` 安装。

## 构建

```bash
# 在工作空间根目录
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash
```

如仅需构建部分功能包:

```bash
colcon build --symlink-install --packages-up-to nav_bringup
```

> 注意:`nav2_plugins` 启用了 `-Werror`,任何编译警告都会中断构建。

## 使用

三套启动配置均位于 `nav_bringup`。

建图(SLAM):

```bash
ros2 launch nav_bringup legged_slam_launch.py
```

基于先验地图定位:

```bash
ros2 launch nav_bringup legged_localization_launch.py map:=/path/to/map.yaml prior_pcd:=/path/to/map.pcd
```

导航:

```bash
ros2 launch nav_bringup legged_navigation_launch.py
```

实机与仿真通过不同的参数文件区分(`nav2_params.legged.yaml` 与 `nav2_params.legged_sim.yaml`),二者在 `use_sim_time`、激光雷达类型与部分感知参数上有所差异,拓扑结构一致。

## 配置

关键参数集中在 `nav_bringup/config/`:

- **控制器**:`omni_pid_pursuit_controller` 的 PID 增益、前瞻距离、曲率限速。
- **代价地图**:强度体素层的观测源、高度与强度过滤区间、膨胀半径。
- **规划器**:`SmacPlannerHybrid` 的运动模型与转弯半径(四足支持原地转向)。
- **行为树**:导航与恢复行为的行为树 XML。

坐标系命名(世界系、本体系、里程计子帧)在里程计与地形分析功能包中均已参数化,默认值保持通用,可按目标机器人的 TF 规范覆盖,无需修改源码。

## 复用说明

框架以跨项目复用为设计目标。功能包之间不存在编译期强耦合,连接关系通过 ROS 2 话题、TF 与启动文件的重映射建立。抽取单个功能包时,通常只需提供其订阅的话题与所需的坐标变换即可独立运行。

对外暴露的绑定点(坐标系名、话题名、文件路径、插件参数)尽量以 ROS 参数形式提供,并保留向后兼容的默认值。各功能包的公共契约、参数清单与已知约束见 `docs/modules/` 下对应文档。

## 文档

`docs/` 目录提供分层理解文档,便于在不通读全部源码的前提下掌握框架:

- [`docs/PROJECT_DOC.md`](docs/PROJECT_DOC.md) — 架构总览、数据流、风险图与符号索引。
- [`docs/modules/`](docs/modules/) — 各功能包的职责、公共接口契约、依赖关系与关键类型。

建议阅读顺序:先读 `PROJECT_DOC.md` 的架构总览,再按数据流顺序查阅各模块文档。

## 许可与归属

本仓库为多个功能包的集成,各功能包沿用其**原始许可证与版权声明**,并未重新授权。集成与适配工作在保留原始许可的前提下进行。使用或分发前,请以各功能包目录下的 `LICENSE` / `NOTICE` 文件及源码文件头为准。

各功能包许可概览:

| 功能包 | 许可证 | 版权 / 原始来源 |
| --- | --- | --- |
| `point_lio` | BSD | Point-LIO(Ji Zhang, Carnegie Mellon University 及原作者) |
| `terrain_analysis` / `terrain_analysis_ext` | BSD | Ji Zhang, CMU(自主探索系列地形分析) |
| `pointcloud_to_laserscan` | BSD | Willow Garage, Inc.;Paul Bovbel、Michel Hidalgo |
| `livox_ros_driver2` | MIT | Copyright (c) 2022 Livox |
| `nav2_plugins` | Apache-2.0 | Copyright 2025 Lihan Chen;部分组件(`BackUpFreeSpace`、`IntensityVoxelLayer`)派生自 @PolarisXQ SCURM_SentryNavigation 与 @ros-navigation navigation2,详见该包 `NOTICE` |
| `omni_pid_pursuit_controller` | Apache-2.0 | Lihan Chen |
| `nav2_loopback_sim` | Apache-2.0 | Steve Macenski / Nav2 |
| `rosbag2_composable_recorder` | Apache-2.0 | Bernd Pfrommer |
| `loam_interface`、`sensor_scan_generation`、`small_gicp_relocalization`、`teleop_twist_joy`、`ign_sim_pointcloud_tool`、`nav_bringup`、`pcd2pgm` | Apache-2.0 | Lihan Chen |

内置的第三方依赖(`dependencies/`):

| 功能包 | 许可证 | 来源 |
| --- | --- | --- |
| `BehaviorTree.ROS2`(`behaviortree_ros2` 等) | MIT | BehaviorTree.CPP |
| `joint_state_publisher` / `_gui` | BSD | ROS 社区 |
| `sdformat_tools` | Apache-2.0 | 上游项目 |

上表为便于查阅的概览,不构成对原始许可条款的修改或补充;若与各包内许可文件存在出入,以许可文件为准。

## 致谢

框架集成并适配了多个开源项目,包括 Point-LIO、Navigation2、Livox ROS Driver 2、BehaviorTree.CPP,以及 CMU 自主探索系列的地形分析工作。感谢上述项目及其作者的贡献。
