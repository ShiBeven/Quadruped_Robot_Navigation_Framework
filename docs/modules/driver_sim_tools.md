# 模块: livox_ros_driver2

> 上级: [PROJECT_DOC.md](../PROJECT_DOC.md)
> 最后同步: 提交 `5b3f214` — 2026-07-05
> 层级分布: A: 0 / B: 7 / C: 30

## 职责
Livox 激光雷达 ROS2 驱动：与 Livox SDK2 接口对接，为 AVIA、HAP 和 MID360 激光雷达型号发布 `CustomMsg` 点云数据。包含通信层 (comm)、配置解析器以及多种激光雷达配置的启动文件。

## 关键文件

| 文件 | 层级 | 职责 |
|---|---|---|
| `src/driver_node.cpp` | B | 主驱动节点入口 |
| `src/lddc.cpp` | B | 激光雷达设备发现与控制（543 行） |
| `src/lds.cpp` | B | 激光雷达数据流（216 行） |
| `src/lds_lidar.cpp` | B | LDS 激光雷达接口（219 行） |
| `src/comm/comm.cpp` | B | 通信协议实现 |
| `src/comm/pub_handler.cpp` | B | 点云发布处理器（504 行） |
| `src/call_back/livox_lidar_callback.cpp` | B | 激光雷达数据回调处理（334 行） |
| `src/livox_ros_driver2.cpp` | C | ROS2 节点设置 |
| `src/parse_cfg_file/*` | B | JSON 配置解析器 |
| 各种 `*.h`, config, launch | C | 头文件、JSON 配置、启动文件 |

## ROS2 接口

| 方向 | 话题 | 类型 | 用途 |
|---|---|---|---|
| **发布** | (可配置) | `livox_ros_driver2::msg::CustomMsg` | 原始 Livox 点云数据 |

## 支持的激光雷达型号

- **HAP** (TX): 短距离密集点云
- **MID360**: 中距离 360° 视场
- **混合 HAP + MID360**: 双激光雷达配置

## 调用关系

- **依赖于:** Livox-SDK2, ROS2 (rclcpp, sensor_msgs), rapidjson
- **被依赖:** point_lio (订阅 CustomMsg), nav_bringup (启动文件)

---

# 模块: simulation (nav2_loopback_sim)

> 上级: [PROJECT_DOC.md](../PROJECT_DOC.md)
> 最后同步: 提交 `5b3f214` — 2026-07-05
> 层级分布: A: 0 / B: 1 / C: 2

## 职责
无摩擦/无惯性/无碰撞的 Python 仿真：将 `cmd_vel` 积分为里程计，对来自 `map_server` 的全局代价地图进行射线投射生成虚拟 LaserScan，支持 `initialpose` 重定位。用于无需完整物理模拟器的快速 Nav2 测试。

## 关键文件

| 文件 | 层级 | 职责 |
|---|---|---|
| `nav2_loopback_sim/loopback_simulator.py` | B | 主仿真节点（LoopbackSimulator 类） |
| `nav2_loopback_sim/utils.py` | C | 工具函数 |
| `nav2_loopback_sim/tf_compat.py` | C | TF 兼容层 |
| `launch/loopback_simulation.launch.py` | C | 仿真启动文件 |
| `launch/bringup_launch.py` | C | 组合启动 |
| `params/nav2_params.yaml` | C | 仿真 Nav2 参数 |

## ROS2 接口

| 方向 | 话题 | 类型 | 用途 |
|---|---|---|---|
| **订阅** | `cmd_vel` | `geometry_msgs::Twist` (或 `TwistStamped`) | 运动命令积分 |
| **订阅** | `initialpose` | `geometry_msgs::PoseWithCovarianceStamped` | 设置/重定位位姿 |
| **发布** | `odom` | `nav_msgs::Odometry` | 仿真的里程计 |
| **发布** | `scan` | `sensor_msgs::LaserScan` | 对 /map 做射线投射 |
| **发布** | `/clock` | `rosgraph_msgs::Clock` | 仿真时钟（可选） |
| **TF** | `map → odom`, `odom → base_footprint` | — | 仿真的变换 |

## 调用关系

- **依赖于:** rclpy, nav_msgs, sensor_msgs, tf2_ros, NumPy（射线投射数学）
- **被依赖:** (独立 — 仅仿真使用，不在硬件部署中使用)

---

# 模块: tools (pcd2pgm + rosbag2_composable_recorder)

> 上级: [PROJECT_DOC.md](../PROJECT_DOC.md)
> 最后同步: 提交 `5b3f214` — 2026-07-05
> 层级分布: A: 0 / B: 2 / C: 4

## pcd2pgm

**职责:** 加载 PCD 点云，经过 Z 轴直通滤波和半径异常过滤，投影为 2D 占据栅格，发布为 `OccupancyGrid`。

**ROS2 接口:**

| 方向 | 话题 | 类型 |
|---|---|---|
| **发布** | 可配置 | `nav_msgs::OccupancyGrid` |
| **发布** | 可配置 | `sensor_msgs::PointCloud2` |

**处理管线:** `passThroughFilter → radiusOutlierFilter → applyTransform → setMapTopicMsg → publishCallback`（定时器驱动）。

**主要用例:** 将 Point-LIO 保存的 PCD 地图转换为 Nav2 兼容的占据栅格地图。

## rosbag2_composable_recorder

**职责:** 可组合 rosbag2 录制器，带 start/stop ROS2 服务，支持在组件容器内按需录制。

| 特性 | 详情 |
|---|---|
| **父类** | `rosbag2_transport::Recorder` |
| **服务: `~/start`** | `std_srvs::Trigger` — 开始新录制 |
| **服务: `~/stop`** | `std_srvs::Trigger` — 停止当前录制 |
| **参数** | `bag_name`（显式路径）、`bag_prefix`（自动命名） |

## 调用关系

- **被依赖:** (独立工具 — 不是运行时导航的一部分)
