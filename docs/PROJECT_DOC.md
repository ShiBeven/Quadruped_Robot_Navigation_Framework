# Quadruped_Robot_Navigation_Framework — 分层理解层文档

> 生成: 2026-07-11 | 更新: 2026-07-11 (复用性改造) | 基线 commit: 无 (该目录非 git 仓库, 版本标记以生成日期为准)
> 语言: C++ (~10,500 行) + Python (~2,900 行) | 一方源码文件: 106 | Tier A: 12 / B: 20 / C: 74 | 一方源码行数: ~16,300 (非空)
> 本文档承载理解代码所需的语义 (契约 / 数据流 / 不变量 / 意图)。落地编辑时回源码确认行号。
> 采用 split 模式: 各模块 Layer 2 拆分至 `docs/modules/<name>.md`; 本文件含 Layer 1 / Layer 3 / 符号索引。
> 项目 MD 文档: 0 份 (无 README/docs)。全部内容由源码静态推断得出, 无权威文档校准, 风险标注置信度已相应下调。
> 每个模块段落带同步标记, 过时立即可见。

---

## Layer 1: 架构总览

### 1.1 概要
- 技术栈: ROS2 (colcon/ament 工作空间) + C++17 + Python (rclpy) | Nav2 导航栈 | PCL / Eigen (点云与线代) | BehaviorTree.CPP (行为树) | pluginlib (插件反射加载) | Livox-SDK2 (激光雷达)。
- 一句话用途: 面向**四足 (legged) 机器人**的自主导航框架 —— 以 Livox MID360 激光雷达为核心传感器, 通过 Point-LIO 做激光-惯性里程计, 经地形分析生成可通行代价, 交由 Nav2 (Smac 规划器 + 全向 PID 纯追踪控制器 + 行为树) 完成建图 / 定位 / 导航闭环。

### 1.2 目录布局
```
src/
├── navigation/                         导航主战场 (14 个 ROS2 package)
│   ├── point_lio/                      激光-惯性里程计 (LIO), 输出高频位姿+配准点云 [核心, 4695 行]
│   ├── livox_ros_driver2/              Livox 3D 激光雷达 ROS2 驱动 [核心, 3906 行]
│   ├── nav2_plugins/                   Nav2 扩展: BT 节点 + 强度体素代价层 + 后退恢复行为 [1919 行]
│   ├── omni_pid_pursuit_controller/    Nav2 全向纯追踪+PID 控制器插件 [1014 行]
│   ├── terrain_analysis/               局部地形可通行性分析 → terrain_map [682 行]
│   ├── terrain_analysis_ext/           扩展尺度地形分析 → terrain_map_ext [557 行]
│   ├── pointcloud_to_laserscan/        点云↔2D激光互转 [761 行]
│   ├── nav_bringup/                    顶层编排: launch + nav2 参数 + URDF [619 行]
│   ├── loam_interface/                 LIO里程计→odom帧 桥接适配 [144 行]
│   ├── sensor_scan_generation/         时间同步生成底盘里程计+TF+局部扫描 [152 行]
│   ├── small_gicp_relocalization/      基于先验PCD地图的 GICP 重定位 → map→odom TF [238 行]
│   ├── ign_sim_pointcloud_tool/        仿真点云→Velodyne格式转换 [104 行]
│   └── teleop_twist_joy/               手柄遥操作 (底盘 cmd_vel / 导航目标点)
├── simulation/
│   └── nav2_loopback_sim/              无物理引擎回环仿真器 (速度积分伪造 odom/TF/scan) [863 行]
├── tools/
│   ├── pcd2pgm/                        .pcd 点云地图 → .pgm 占据栅格 [244 行]
│   └── rosbag2_composable_recorder/    可组合 rosbag2 录制器 [257 行]
└── dependencies/                       第三方原样引入 (不深读)
    ├── BehaviorTree.ROS2/              BT 与 ROS2 的桥接 (nav2_plugins 依赖)
    ├── joint_state_publisher/          关节状态发布 (标准 ROS2 包)
    └── sdformat_tools/                 SDF/URDF 转换工具 (Python)
```

### 1.3 模块依赖拓扑
```
                       [硬件/仿真]
        Livox MID360           ign_sim_pointcloud_tool (仿真时)
         │ livox/lidar+imu      │ velodyne_points
         ▼                      ▼
   ┌──────────────────────────────────┐
   │           point_lio              │  激光-惯性里程计
   └──────────────────────────────────┘
         │ aft_mapped_to_init + cloud_registered
         ▼
   ┌──────────────┐
   │ loam_interface│  帧变换适配 (lidar_odom → odom)
   └──────────────┘
         │ lidar_odometry + registered_scan
         ▼
   ┌────────────────────────┐        ┌──────────────────────────┐
   │ sensor_scan_generation │──────► │ small_gicp_relocalization │ (定位分支)
   └────────────────────────┘  TF    └──────────────────────────┘
         │ odometry + sensor_scan              │ map→odom TF
         ▼
   ┌──────────────────┐  ┌──────────────────────┐
   │ terrain_analysis │  │ terrain_analysis_ext │  (级联: ext 消费 analysis 的 terrain_map)
   └──────────────────┘  └──────────────────────┘
         │ terrain_map          │ terrain_map_ext
         ▼                      ▼
   ┌────────────────────────────────────────────────┐
   │                    Nav2                          │
   │  costmap(intensity_voxel_layer ← nav2_plugins)   │
   │  planner(SmacPlannerHybrid)                      │
   │  controller(omni_pid_pursuit_controller)         │
   │  bt_navigator(BT ← nav2_plugins) + behaviors     │
   └────────────────────────────────────────────────┘
         │ cmd_vel_nav2_result → velocity_smoother → cmd_vel
         ▼
      [四足底盘]

   nav_bringup: 编排以上全部 (三套 launch: localization / navigation / slam)
   teleop_twist_joy / nav2_loopback_sim / pcd2pgm / rosbag2_recorder: 旁路工具
```
> **无双向循环依赖** (数据流为单向管线)。模块间几乎全部通过 ROS2 话题/TF 松耦合, 无编译期强依赖环。

### 1.4 核心数据流
1. **感知→里程计流**: Livox `livox/lidar`(PointCloud2)+`livox/imu`(Imu) → **point_lio** (预处理→IMU初始化→IKFoM EKF predict/update) → `aft_mapped_to_init`(Odometry, `camera_init`帧)+`cloud_registered`(PointCloud2)。仿真时 `ign_sim_pointcloud_tool` 先把 Gazebo 点云转 Velodyne 格式。
2. **里程计→代价地图流**: point_lio 输出 → **loam_interface** (变换到 `odom` 帧) → `lidar_odometry`+`registered_scan` → **sensor_scan_generation** (时间同步, 广播 `odom→base_footprint` TF, 发 `odometry`) → **terrain_analysis**/`_ext` (体素累积+地面估计+可通行代价) → `terrain_map`(局部)/`terrain_map_ext`(全局) → Nav2 costmap 的 `intensity_voxel_layer`。
3. **规划→控制流**: Nav2 `bt_navigator` (行为树) → `planner_server`(SmacPlannerHybrid/DUBIN, `GridBased`) 出全局路径 → `controller_server`(`FollowPath`=**omni_pid_pursuit_controller**, 全向双PID纯追踪) 出 `cmd_vel` → remap 到 `cmd_vel_nav2_result` → `velocity_smoother` 平滑 → 最终 `cmd_vel`。恢复行为用 `Spin` (四足不宜后退)。
4. **定位/建图分支**: 定位时 **small_gicp_relocalization** 配 `registered_scan` 与先验 PCD, 广播 `map→odom` TF; 建图 (SLAM) 时 `pointcloud_to_laserscan` 把 `terrain_map_ext` 转 `obstacle_scan` 喂 `slam_toolbox`。

### 1.5 入口点
| 场景 | 入口 |
|---|---|
| 实机导航 | `nav_bringup/launch/legged_navigation_launch.py` (启动 loam_interface + sensor_scan + terrain×2 + nav2 全栈) |
| 定位 | `nav_bringup/launch/legged_localization_launch.py` (启动 point_lio + map_server + small_gicp + lifecycle_manager) |
| 建图 (SLAM) | `nav_bringup/launch/legged_slam_launch.py` (启动 point_lio + slam_toolbox + pointcloud_to_laserscan) |
| 激光雷达驱动 | `livox_ros_driver2/launch/msg_MID360_launch.py` |
| LIO 里程计 | `point_lio/launch/point_lio.launch.py` |
| 纯仿真闭环 | `simulation/nav2_loopback_sim/launch/loopback_simulation.launch.py` |
| 地图转换工具 | `tools/pcd2pgm/launch/pcd2pgm_launch.py` |
| C++ 组件注册 | point_lio: `pointlio_mapping`; 多数节点经 `RCLCPP_COMPONENTS_REGISTER_NODE` 注册为可组合组件 |

---

## 阅读路径
- **排查 Bug**: 先看本文件 Layer 3 风险图定位热点 → 查文末符号索引找文件 → 打开对应 `docs/modules/<name>.md` 追调用链与契约 → 回源码确认行号。
- **修改功能**: 先看 1.4 核心数据流确认影响面 (改一处话题/帧名会波及整条管线) → 查目标模块文档的"调用关系/被谁依赖" → 注意 1.3 拓扑中该模块的上下游。
- **新人上手**: 通读 Layer 1 → 读 `modules/nav_bringup.md` (理解编排与端到端连接) → 再按数据流顺序读 `point_lio` → `terrain_analysis` → `nav2_stack`。
- **改参数/调参**: nav2 行为看 `modules/nav_bringup.md` 的 yaml 配置要点; 里程计调参看 `modules/point_lio.md` 参数节 (注意实机 legged.yaml vs 仿真 legged_sim.yaml 差异)。

## Layer 3: 风险图

> 以下标注基于代码的结构特征, 不断言"这是 bug", 每条需人工确认。
> 置信度: 高 = 可量化事实 (行数/引用数/明确的空 catch); 中 = 模式匹配需复核; 低 = 大概率无害但列出以求完整。
> 本项目无 git 历史与作者文档, 涉及"意图/理解缺口"的判断无法用 blame 佐证, 置信度普遍偏保守。

### 摘要
| 类别 | 数量 | 已修复 (2026-07-11) |
|---|---|---|
| 复杂度热点 | 7 | — |
| 高耦合 / 全局状态 | 5 | — |
| 隐式/动态依赖 | 6 | — |
| 异常处理缺口 | 8 | — |
| 理解缺口 | 6 | — |
| 配置/代码不一致 | 5 | #35 部分, #36 ✅ |
| 硬编码 / 魔数 | 7 | #39 ✅, #46 ✅ (新增记录) |

> **本轮复用性改造已处理**: #36 #39 (PID 抗饱和+参数化)、#35 (teleop 死读取移除)、#46 (point_lio/terrain 帧名与 PCD 路径参数化)。均默认值向后兼容, 无破坏性变更。未处理的架构级项 (全局状态挡多实例 #8/#9、巨型 main #1/#4) 留作演进。

### 注解
| # | 文件:行 | 类型 | 置信度 | 说明 / 建议 |
|---|---|---|---|---|
| 1 | point_lio/src/laserMapping.cpp:400-1032 | 复杂度热点 | 高 | 巨型 main (~630 行); `use_imu_as_input` 两大分支近乎重复, 各 5-7 层嵌套。重构前先补特征测试; 提取 input/output 两条路径为独立函数。 |
| 2 | point_lio/src/preprocess.cpp:全文 | 复杂度热点 | 高 | 935 行; 各型号 LiDAR handler + `give_feature`/`plane_judge` 特征提取多层 for+if。修改雷达支持前隔离目标 handler。 |
| 3 | point_lio/src/Estimator.cpp:112-322 | 复杂度热点 | 高 | `h_model_input` 与 `h_model_output` 逐行几乎相同 (复制粘贴)。改量测模型须同步两处, 易漏改。 |
| 4 | terrain_analysis/src/terrainAnalysis.cpp:182-682 | 复杂度热点 | 高 | ~500 行全塞进 main; 体素滚动/动态障碍剔除/无数据补障三处嵌套 6-7 层。头号单函数复杂度。(注: 输出帧已参数化, 见 #46) |
| 5 | terrain_analysis_ext/src/terrainAnalysisExt.cpp:159-557 | 复杂度热点 | 高 | ~400 行 main; BFS 连通性天花板过滤中每队列元素扫 21×21=441 邻域, 大网格性能热点。 |
| 6 | livox_ros_driver2/src/lddc.cpp / pub_handler.cpp | 复杂度热点 | 中 | `GetCurrentPublisher`/`GetCurrentPublisher2` (lddc.cpp:450-514) 重复代码; `CheckTimer` (pub_handler.cpp:186-249) 双分支 4 层嵌套。 |
| 7 | omni_pid_pursuit_controller/src/omni_pid_pursuit_controller.cpp | 复杂度热点 | 低 | 770 行但函数拆分良好, 复杂度分散。作为单文件核心插件, 修改时通读 hpp 契约。 |
| 8 | point_lio/src/parameters.h + Estimator.h + li_initialization.h | 高耦合/全局状态 | 高 | 状态几乎全为跨文件 `extern` 全局 (`kf_input/kf_output`,`ivox_`,`k`,`idx`,`imu_deque`...)。量测模型经全局 `k`/`idx` 与主循环耦合; MultiThreadedExecutor 下主循环访问这些全局无锁。改动前确认无并发访问路径。 |
| 9 | terrain_analysis(_ext)/src/*.cpp | 高耦合/全局状态 | 高 | 各 40+/30+ 全局变量 + 大数组; 回调与 main 靠全局标志 `newlaserCloud` 通信, 依赖单线程 spin_some。勿改成多线程执行器。 |
| 10 | livox_ros_driver2/src/comm/comm.h:213 (LidarDataQueue) | 高耦合/并发 | 中 | 无锁 SPSC 环形队列, 依赖"单生产单消费"假设 + volatile 索引 (非 atomic)。队列满分支 (lds.cpp:200-211) 丢包。多核可见性/重排需复核。 |
| 11 | livox_ros_driver2/src/comm/pub_handler.cpp:43 | 高耦合/并发 | 中 | `extrinsic_global` 文件级全局; 多雷达外参 last-writer-wins 相互覆盖, 多雷达 IMU 补偿隐患。 |
| 12 | shared/utils 无单点 | 高耦合 | 低 | 本框架无跨包共享工具库, 耦合主要通过 ROS 话题/TF, 编译期耦合低 (优点)。 |
| 13 | nav2_plugins (全部 Behavior/Layer/BT 节点) | 隐式/动态依赖 | 高 | 经 `pluginlib` (`PLUGINLIB_EXPORT_CLASS`) 与 BT 工厂按字符串名反射加载, 编译期无调用点。排障须结合 nav2 的 costmap yaml 与 bt_navigator 的 BT XML。搜索 `PLUGINLIB_EXPORT_CLASS` / `BT_REGISTER_NODES` 定位注册点。 |
| 14 | omni_pid_pursuit_controller.cpp:768 | 隐式/动态依赖 | 高 | Controller 经 pluginlib 加载 (`nav2_core::Controller`), 无静态调用点; 由 `nav2_params` 的 `FollowPath.plugin` 字符串决定。 |
| 15 | point_lio: laserMapping.cpp:354-356, 374-382 | 隐式/动态依赖 | 中 | IKFoM 经 `init_dyn_share_modified` 注入 `h_model` 函数指针; `lidar_type` 运行时决定订阅消息类型与 handler 分派 (lambda 包装函数指针)。 |
| 16 | 多个 C++ 包 (pcd2pgm, teleop_twist_joy, rosbag2_recorder, loam_interface, sensor_scan_generation) | 隐式/动态依赖 | 中 | 均 `RCLCPP_COMPONENTS_REGISTER_NODE` 注册为可组合组件; 但仅 rosbag2_composable_recorder 实际经 ComposableNodeContainer + intra-process 加载, 余者 launch 中仍以独立进程 Node 启动。 |
| 17 | livox_ros_driver2 (SDK 回调链) | 隐式/动态依赖 | 中 | 依赖 Livox-SDK2 注册回调 (`SetLivoxLidarInfoChangeCallback` 等) 反向驱动数据流; 静态调用图不完整, 数据入口在 SDK 线程。 |
| 18 | point_lio: preprocess.h:52-98 | 隐式/动态依赖 | 低 | PCL `POINT_CLOUD_REGISTER_POINT_STRUCT` 与 MTK `MTK_BUILD_MANIFOLD` 为编译期元编程"反射", 生成点结构/流形状态类型。 |
| 19 | point_lio/src/parameters.cpp:231-234 | 异常处理缺口 | 高 | `catch(ParameterTypeException / std::exception)` 仅打日志后继续; 参数读取失败会带默认/未初始化值 (parameters.cpp 顶部多个变量无初值) 静默运行。 |
| 20 | omni_pid_pursuit_controller.cpp:264 | 异常处理缺口 | 中 | 检测到碰撞直接 `throw PlannerException` (激进停车); 且 isCollisionDetected 中路径点越出 costmap 时 return false 视为无碰撞 (cpp:456), 漏检风险。 |
| 21 | sensor_scan_generation.cpp:85-88 | 异常处理缺口 | 中 | `getTransform` TF 失败返回 identity 变换 (静默降级), 会污染下游位姿。至少应告警并跳过该帧。 |
| 22 | livox_ros_driver2/src/lddc.cpp:321-322,371-372,428-429 | 异常处理缺口 | 中 | `output_type != kOutputToRos` 时 `else{}` 空分支静默丢弃点云/IMU; PclMsg 路径 (375-394) 仅 warn 后 return 丢全部数据; `CreateBagFile` 空实现 (RosBag 未实现)。 |
| 23 | pointcloud_to_laserscan_node.cpp:181-182 | 异常处理缺口 | 中 | 强制访问点云 `intensity` 字段, 输入无该字段将抛异常/崩溃。转换前应校验字段存在。 |
| 24 | small_gicp_relocalization.cpp:97-100 | 异常处理缺口 | 中 | PCD 读取失败仅 return, `global_map_` 为空则后续配准无意义, 无致命处理 (与 :111-128 的 TF FATAL 逻辑不对称)。 |
| 25 | pcd2pgm.cpp:26-29 / :164-167 | 异常处理缺口 | 中 | PCD 加载失败仅 return, 节点空转无输出; `odom_to_lidar_odom_[0..5]` 无长度校验, 参数数组 <6 元素越界崩溃。 |
| 26 | loopback_simulator.py:174 / start_recording.py:31 | 异常处理缺口 | 低 | 宽泛 `except Exception` (非裸 except); loopback 吞 TF 异常仅 debug 日志; recorder 客户端记录后 break, 行为可接受。 |
| 27 | terrain_analysis(_ext): fromROSMsg / 索引计算 | 异常处理缺口 | 低 | 完全无 try/catch; `joystickHandler` 直接读 `joy->buttons[5]`, 手柄按钮少于 6 个越界 (terrainAnalysis.cpp:169; Ext:148)。 |
| 28 | point_lio/src/laserMapping.cpp:633,721,900 | 理解缺口 | 中 | 作者自留注释 `// big problem` / `// >= ?` 标注的可疑时序逻辑 (`time_predict_last_const` 等)。改时序前需实测验证。 |
| 29 | point_lio/src/laserMapping.cpp:734-737 | 理解缺口 | 中 | `is_first_frame` 分支内 `break` 之后仍有 angvel/acc 赋值, 疑似死代码。 |
| 30 | point_lio/src/Estimator.cpp:143-161,251-268 | 理解缺口 | 中 | 大段被注释的鲁棒核自适应加权; 现版本仅保留硬阈值筛选, 是否应启用不明。 |
| 31 | point_lio/src/laserMapping.cpp:526-527 | 理解缺口/潜在崩溃 | 中 | `loadPointcloudFromPcd` 失败返回 nullptr 后直接解引用 `map_cloud->points`, 未判空; `enable_prior_pcd` 且加载失败会崩溃。 |
| 32 | nav2_plugins/src/behaviors/back_up_free_space.cpp:171-224 | 理解缺口 | 中 | `findBestDirection` 全阻塞判定用 `final_safe==0 && final_unsafe==0`, 与合法 0 角度无法区分 (潜在逻辑缺陷); :227-245 `gatherFreePoints` 死代码。 |
| 33 | ign_sim_pointcloud_tool/src/point_cloud_converter.cpp:62 | 理解缺口 | 低 | `time` 由点索引估算而非真实时间戳, 假设固定 0.1s 帧周期与均匀扫描; 点序非扫描序时 ring/time 失真。 |
| 34 | nav2_plugins/.../is_path_goal_reached.cpp:32 | 配置/代码不一致 | 低 | `goal_succeeded` 输入端口声明但 tickCondition 未使用 (遗留)。 |
| 35 | teleop_twist_joy: pb_teleop_twist_joy.cpp:129-144 | 配置/代码不一致 | 高 | ✅ **已部分修复 (2026-07-11)**: 移除了 fillCmdVelMsg 中对不存在键 `z`/`pitch`/`roll` 的死读取 (原恒返回 0)。⚠️ 仍待办: yaml 的 `axis_gimbal`/`scale_gimbal` 依旧无人读取, 真正的云台支持需新增独立话题/消息 (功能新增, 待产品决定)。 |
| 36 | omni_pid_pursuit_controller.cpp:112 (min_max_sum_error) | 配置/代码不一致 | 中 | ✅ **已修复 (2026-07-11)**: `min_max_sum_error_` 现经 `PID::setSumErrorLimit` 接入 PID 积分限幅 (构造 cpp:166-167 + 动态更新 718-719)。剩余: `setSpeedLimit()` (cpp:272) 仍空实现仅告警。 |
| 37 | terrain_analysis(_ext): 默认值 vs launch | 配置/代码不一致 | 中 | 多个参数 cpp 默认值与 launch 覆盖值冲突 (clearDyObs/maxRelZ/checkTerrainConn 等); `checkTerrainConn` cpp 默认 true 而 launch arg 默认 false, 实际生效以 launch 为准。理解行为须看 launch。 |
| 38 | 多处 (nav2_params.yaml / loopback / teleop) | 配置/代码不一致 | 低 | yaml `enable_stamped_cmd_vel: false` vs 代码默认 True; teleop enable_button 默认 5 vs yaml 4。行为依配置来源。 |
| 39 | omni_pid_pursuit_controller/src/pid.cpp:18-25 | 硬编码/逻辑次序 | 中 | ✅ **已修复 (2026-07-11)**: 积分钳位移到使用 `i_out` **之前** (抗饱和当拍生效); 限幅值改用可配的 `i_max_` (由 min_max_sum_error 注入), 不再硬编码 ±1。 |
| 40 | omni_pid_pursuit_controller/src/pid.hpp:15 | 硬编码/API 缺陷 | 低 | PID 构造参数顺序 `(dt,max,min,kp,kd,ki)` (kd 在 ki 前), 反直觉易误用; 参数名拼写 `use_rotate_to_heading_treshold` (应 threshold)。 |
| 41 | point_lio: laserMapping.cpp / Estimator.cpp | 硬编码/魔数 | 中 | 固定大小数组 `point_selected_surf[100000]`,`pcl_wait_pub(500000)`; 点数超限越界隐患。`MOV_THRESHOLD=1.5f`,`PUBFRAME_PERIOD=20`,`MAX_INI_COUNT=100` 等魔数。 |
| 42 | livox_ros_driver2: lddc.cpp:398 / MID360_config.json | 硬编码/魔数 | 中 | IMU frame_id 硬编码 "livox_frame" (忽略 frame_id 参数); 雷达 IP `192.168.1.177`/host `192.168.1.50`; poll 线程固定 `sleep 3s`。 |
| 43 | nav_bringup: nav2_params.legged.yaml + legged_slam_launch.py:107-122 | 硬编码/魔数 | 中 | 帧名 `front_mid360`/`base_footprint`/`body` 与 URDF 强耦合散落 yaml; SLAM 模式 map→odom 静态 TF 硬编码全 0 (假设无重定位漂移)。 |
| 46 | point_lio (laserMapping.cpp) / terrain_analysis(_ext) | 硬编码/帧名 | 高 | ✅ **已修复 (2026-07-11)**: point_lio 输出帧 (`camera_init`/`body`/`aft_mapped`) 提为 `publish.world_frame`/`body_frame`/`aft_mapped_frame` 参数; PCD 存盘路径提为 `pcd_save.pcd_save_dir` (原编译期 ROOT_DIR, 存进源码树); terrain 两节点输出帧提为 `mapFrame`。默认值全部向后兼容。这是跨项目复用的关键解绑。 |
| 44 | small_gicp_relocalization.cpp:188 / sensor_scan_generation.cpp:118 | 硬编码/时序 | 中 | small_gicp 发布 TF 时 stamp 硬编码 +0.1s 未来外推; sensor_scan 速度用 `steady_clock` 墙钟而非 ROS 时钟, `use_sim_time` 下速度失真。 |
| 45 | loopback_simulator.py:23-41 | 硬编码/脆弱兼容 | 低 | 运行时 monkey-patch `np.float`/`np.maximum_sctype` 兼容 NumPy 2.x, 隐式全局副作用; occupancy 阈值 60、LineIterator 步长 0.5 硬编码。 |

---

## 附录: 符号索引

> 字母序 (按类型)。用 `grep <符号名>` 在对应 `docs/modules/<module>.md` 定位契约, 或直接在源码搜索。
> 完整逐符号清单见各模块文档的"公共 API 契约"节。此处收录跨模块关键符号与公共接口类。

### 核心类 / 节点 (C++)
| 符号 | 种类 | 文件 | 模块 |
|---|---|---|---|
| `DriverNode` | class (rclcpp::Node) | livox_ros_driver2/src/driver_node.h | livox_ros_driver2 |
| `Lddc` | class | livox_ros_driver2/src/lddc.h | livox_ros_driver2 |
| `Lds` / `LdsLidar` | class | livox_ros_driver2/src/lds.h / lds_lidar.h | livox_ros_driver2 |
| `PubHandler` | class | livox_ros_driver2/src/comm/pub_handler.h | livox_ros_driver2 |
| `ImuProcess` | class | point_lio/src/IMU_Processing.h | point_lio |
| `Preprocess` | class | point_lio/src/preprocess.h | point_lio |
| `OmniPidPursuitController` | class (nav2_core::Controller) | omni_pid_pursuit_controller/src/omni_pid_pursuit_controller.cpp | omni_pid_pursuit_controller |
| `PID` | class | omni_pid_pursuit_controller/src/pid.cpp | omni_pid_pursuit_controller |
| `BackUpFreeSpace` | class (nav2_core::Behavior) | nav2_plugins/src/behaviors/back_up_free_space.cpp | nav2_plugins |
| `IntensityVoxelLayer` | class (nav2_costmap_2d::Layer) | nav2_plugins/src/layers/intensity_voxel_layer.cpp | nav2_plugins |
| `RecoveryNode` | class (BT::ControlNode) | nav2_plugins/src/bt/control/recovery_node.cpp | nav2_plugins |
| `RateController` | class (BT::DecoratorNode) | nav2_plugins/src/bt/decorator/rate_controller.cpp | nav2_plugins |
| `LoamInterfaceNode` | class (component) | loam_interface/src/loam_interface.cpp | loam_interface |
| `SensorScanGenerationNode` | class (component) | sensor_scan_generation/src/sensor_scan_generation.cpp | sensor_scan_generation |
| `Pcd2PgmNode` | class (component) | tools/pcd2pgm/src/pcd2pgm.cpp | pcd2pgm |
| `ComposableRecorder` | class (rosbag2 Recorder) | tools/rosbag2_composable_recorder/src/composable_recorder.cpp | rosbag2_composable_recorder |
| `TeleopTwistJoyNode` | class (component) | teleop_twist_joy/src/pb_teleop_twist_joy.cpp | teleop_twist_joy |
| `LoopbackSimulator` | class (rclpy Node) | simulation/nav2_loopback_sim/.../loopback_simulator.py | nav2_loopback_sim |

### BT 节点 (XML 名 → 类, nav2_plugins)
| XML 名 | 种类 | 文件 |
|---|---|---|
| `SelectFixedPath` / `SelectPatrolPath` / `SelectPathGoalPose` | BT action | nav2_plugins/src/bt/action/select_*.cpp |
| `PublishNavGoal` / `SendNav2Goal` / `SendNavThroughPoses` | BT action | nav2_plugins/src/bt/action/*.cpp |
| `HoldStopFlag` / `PublishTwist` / `PublishSpinSpeed` | BT action | nav2_plugins/src/bt/action/*.cpp |
| `IsPathGoalReached` | BT condition | nav2_plugins/src/bt/condition/is_path_goal_reached.cpp |
| `RecoveryNode` | BT control | nav2_plugins/src/bt/control/recovery_node.cpp |
| `RateController` | BT decorator | nav2_plugins/src/bt/decorator/rate_controller.cpp |

### 关键函数 (C++)
| 符号 | 种类 | 文件 | 模块 |
|---|---|---|---|
| `main` (LIO 主循环) | function | point_lio/src/laserMapping.cpp:324 | point_lio |
| `sync_packages` | function | point_lio/src/li_initialization.cpp:177 | point_lio |
| `h_model_input` / `h_model_output` | function | point_lio/src/Estimator.cpp:112/218 | point_lio |
| `readParameters` | function | point_lio/src/parameters.cpp | point_lio |
| `computeVelocityCommands` | method | omni_pid_pursuit_controller.cpp:209 | omni_pid_pursuit_controller |
| `getLookAheadPoint` / `circleSegmentIntersection` | method | omni_pid_pursuit_controller.cpp:353/384 | omni_pid_pursuit_controller |
| `PID::calculate` | method | omni_pid_pursuit_controller/src/pid.cpp:10 | omni_pid_pursuit_controller |
| `odometryCallback` / `pointCloudCallback` | method | loam_interface/src/loam_interface.cpp | loam_interface |
| `laserCloudAndOdometryHandler` | method | sensor_scan_generation/src/sensor_scan_generation.cpp | sensor_scan_generation |
| `performRegistration` / `publishTransform` | method | small_gicp_relocalization/src/small_gicp_relocalization.cpp | small_gicp_relocalization |
| `computeNextPatrolState` (巡逻状态机) | inline fn | nav2_plugins/include/nav2_plugins/bt/nav_utils.hpp:144 | nav2_plugins |
| `lidarHandle` | method | ign_sim_pointcloud_tool/src/point_cloud_converter.cpp | ign_sim_pointcloud_tool |

### 关键类型 (C++)
| 符号 | 种类 | 文件 | 模块 |
|---|---|---|---|
| `CustomMsg` / `CustomPoint` | msg | livox_ros_driver2/msg/*.msg | livox_ros_driver2 |
| `LivoxPointXyzrtlt` / `PointXyzlt` | struct (packed) | livox_ros_driver2/src/comm/comm.h:152/163 | livox_ros_driver2 |
| `LidarDataQueue` | struct (SPSC 环形) | livox_ros_driver2/src/comm/comm.h:213 | livox_ros_driver2 |
| `state_input` (24维) / `state_output` (30维) | MTK 流形状态 | point_lio/include/common_lib.h:24 | point_lio |
| `MeasureGroup` | struct | point_lio/include/common_lib.h:85 | point_lio |
| `PointType` = pcl::PointXYZINormal | typedef | point_lio/src/preprocess.h:12 | point_lio |
| `IVoxType` | typedef (iVox 地图) | point_lio/src/parameters.h:34 | point_lio |
| `convertFromString<PoseStamped>` | BT 端口解析 | nav2_plugins/include/nav2_plugins/bt/custom_types.hpp | nav2_plugins |
| `PointXYZIRT` | struct | ign_sim_pointcloud_tool/include/.../point_cloud_converter.hpp | ign_sim_pointcloud_tool |

### 关键话题 (跨模块数据总线)
| 话题 | 类型 | 生产者 → 消费者 |
|---|---|---|
| `livox/lidar` / `livox/imu` | PointCloud2 / Imu | livox_ros_driver2 → point_lio |
| `aft_mapped_to_init` | nav_msgs/Odometry | point_lio → loam_interface |
| `cloud_registered` | PointCloud2 | point_lio → loam_interface |
| `lidar_odometry` | nav_msgs/Odometry | loam_interface → sensor_scan_generation, terrain_analysis |
| `registered_scan` | PointCloud2 | loam_interface → sensor_scan/terrain/small_gicp |
| `odometry` | nav_msgs/Odometry | sensor_scan_generation → nav2 (odom_topic) |
| `terrain_map` | PointCloud2 | terrain_analysis → local_costmap, terrain_analysis_ext |
| `terrain_map_ext` | PointCloud2 | terrain_analysis_ext → global_costmap, pointcloud_to_laserscan |
| `cmd_vel_nav2_result` | Twist | controller/bt_navigator → velocity_smoother |
| `cmd_vel` | Twist | velocity_smoother / teleop → 底盘 |
| TF `map→odom` | tf2 | small_gicp_relocalization (或 slam/static) |
| TF `odom→base_footprint` | tf2 | sensor_scan_generation |

