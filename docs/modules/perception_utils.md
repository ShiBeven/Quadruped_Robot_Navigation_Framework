# 模块组: 感知转换工具 (pointcloud_to_laserscan / ign_sim_pointcloud_tool)

> 同步: 2026-07-11 (非 git 仓库) | 语言: C++ | 点云格式/维度转换工具
> 路径: `src/navigation/{pointcloud_to_laserscan, ign_sim_pointcloud_tool}/`

---

## pointcloud_to_laserscan — 点云↔激光互转

### 职责
提供两个反向节点: 点云投影为 2D LaserScan (含高度/强度/距离/角度过滤), 及 LaserScan 反投影为点云。本框架 SLAM 中把 `terrain_map_ext` 点云转成 `obstacle_scan` 喂 slam_toolbox。

### 关键文件
| 文件 | Tier | 职责 |
|---|---|---|
| src/pointcloud_to_laserscan_node.cpp | A | PointCloud2→LaserScan (244 行) |
| src/laserscan_to_pointcloud_node.cpp | B | LaserScan→PointCloud2 (用 laser_geometry projector, 150 行) |

### ROS2 接口
- **pointcloud_to_laserscan**: 订阅 `cloud_in`(PointCloud2, SLAM launch remap 自 `terrain_map_ext`) → 发布 `scan`(LaserScan, remap 到 `obstacle_scan`)。参数: target_frame(base_footprint)、min/max_height(-0.3/4.0)、min/max_intensity(0.1/2.0)、angle_min/max/increment、range_min/max(0.15/10.0)、use_inf、queue_size
- **laserscan_to_pointcloud**: 订阅 `scan_in`(LaserScan) → 发布 `cloud`(PointCloud2)
- 两者均用后台线程 `subscriptionListenerThreadLoop` 按下游订阅数惰性启停上游订阅

### 公共 API 契约
| 符号 | 用途 | 契约 / 不变量 |
|---|---|---|
| `cloudCallback` | 点云→scan | ⚠️ 读 `x,y,z,intensity` 迭代器 (**要求输入点云必含 intensity 字段**); 按 nan/height/intensity/range/angle 逐点过滤, 取每角度 bin 最小 range; TF 失败 return 丢帧。 |
| `scanCallback` | scan→点云 | `projector_.projectLaser`; target_frame 非空则 tf2 变换, 失败 ERROR+return。 |

> **风险**: 强制访问 intensity 字段 (无则崩溃); min/max_height 默认用 `numeric_limits<double>::min()` (正极小值非负无穷, 语义陷阱, 本框架 yaml 已覆盖); TF 失败长期丢帧无告警。见 Layer 3 注解 #23。

---

## ign_sim_pointcloud_tool — 仿真点云格式转换

### 职责
把 Ignition/Gazebo 仿真发出的无 ring/time 字段点云 (如 `livox/lidar`) 按垂直角计算 ring、按索引计算 time, 转换为带 `PointXYZIRT` 的 Velodyne 兼容格式 (`velodyne_points`), 供 point_lio 在仿真下以 lidar_type=2 消费。

### 关键文件
| 文件 | Tier | 职责 |
|---|---|---|
| src/point_cloud_converter.cpp | A | 转换实现 (86 行) |
| include/ign_sim_pointcloud_tool/point_cloud_converter.hpp | B | `PointXYZIRT` 定义 + 类声明 (52 行) |

### ROS2 接口 (node `point_cloud_converter`)
- **订阅**: `<pcd_topic>`(PointCloud2, 默认 `livox/lidar`)
- **发布**: `velodyne_points`(PointCloud2, SensorDataQoS)
- **参数**: pcd_topic、n_scan(32)、horizon_scan(1875)、ang_bottom(7.0)、ang_res_y(1.0)

### 公共 API 契约
| 符号 | 用途 | 契约 / 不变量 |
|---|---|---|
| `lidarHandle` | 点云转换 | 输入按 `PointXYZ` 解析 (⚠️ 丢弃原 intensity, 硬置 0); `row_id=(vertical_angle+ang_bottom)/ang_res_y`, 越界 [0,n_scan) 丢点; `time=(idx%horizon_scan)*0.1/horizon_scan` (假设 10Hz)。 |
| `publishPoints<T>` | 发布 | is_dense=false, 复制原 header。 |

> **风险**: intensity 恒置 0 (下游强度过滤 min 0.1 会全滤除, 但仿真 costmap 走 terrain_map); time 由索引估算假设固定帧周期; ring 由几何反推, 参数错配静默丢点。见 Layer 3 注解 #33。

---

## 数据流定位
- **pointcloud_to_laserscan**: 仅 SLAM 分支使用 (terrain_map_ext → obstacle_scan → slam_toolbox)。
- **ign_sim_pointcloud_tool**: 仅纯仿真链路使用 (Gazebo 点云 → velodyne_points → point_lio, lidar_type=2)。
- **依赖**: pcl_ros/pcl_conversions、laser_geometry (p2l)、tf2、sensor_msgs、rclcpp。
