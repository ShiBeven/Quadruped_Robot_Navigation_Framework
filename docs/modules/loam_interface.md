# 模块: loam_interface

> 上级: [PROJECT_DOC.md](../PROJECT_DOC.md)
> 最后同步: 提交 `5b3f214` — 2026-07-05
> 层级分布: A: 0 / B: 1 / C: 2

## 职责
桥接节点：将 LOAM 激光里程计和配准点云转发到目标里程计坐标系 (`odom`)。在首次收到消息时计算并缓存 LOAM 里程计坐标系到目标坐标系的静态变换。

## 关键文件

| 文件 | 层级 | 职责 |
|---|---|---|
| `include/loam_interface/loam_interface.hpp` | B | 节点类：订阅者、变换缓存、转发器 |
| `src/loam_interface.cpp` | C | 实现 |
| `launch/loam_interface_launch.py` | C | 启动文件 |

## ROS2 接口

| 方向 | 话题 (可配置) | 类型 | 用途 |
|---|---|---|---|
| **订阅** | `registered_scan` | `sensor_msgs::PointCloud2` | LOAM 配准点云 |
| **订阅** | `state_estimation_topic_` | `nav_msgs::Odometry` | LOAM 里程计（话题名可配置） |
| **发布** | (可配置) | `sensor_msgs::PointCloud2` | 目标坐标系转发的点云 |
| **发布** | (可配置) | `nav_msgs::Odometry` | 目标坐标系转发的里程计 |

## 变换帧链

```
camera_init (LOAM 原点)
    ↓ (静态变换，计算一次)
odom (目标坐标系)
    ↓ (来自里程计的动态变换)
base_footprint
    ↓ (来自 URDF 的静态变换)
front_mid360 (激光雷达传感器坐标系)
```

**变换缓存:** 首次收到消息时，节点通过 TF2 查找 `camera_init → odom` 变换并缓存。所有后续消息使用此缓存变换进行变换。

## 关键参数

| 参数 | 描述 |
|---|---|
| `odom_frame` | 目标里程计坐标系（默认: `odom`） |
| `lidar_frame` | 激光雷达传感器坐标系（默认: `front_mid360`） |
| `base_frame` | 机器人主体坐标系（默认: `base_footprint`） |

## 调用关系

- **依赖于:** ROS2 (sensor_msgs, nav_msgs, tf2_ros), TF2
- **被依赖:** nav_bringup (由 legged_navigation_launch.py 启动)

---

# 模块: sensor_scan_generation

> 上级: [PROJECT_DOC.md](../PROJECT_DOC.md)
> 最后同步: 提交 `5b3f214` — 2026-07-05
> 层级分布: A: 0 / B: 1 / C: 2

## 职责
通过 `message_filters::ApproximateTime` 同步 LOAM 里程计和激光雷达点云，计算静态激光-机器人主体坐标变换，并将两者转发为机器人主体坐标系下的里程计和点云。

## 关键文件

| 文件 | 层级 | 职责 |
|---|---|---|
| `include/sensor_scan_generation/sensor_scan_generation.hpp` | B | 带同步订阅者的节点类 |
| `src/sensor_scan_generation.cpp` | C | 实现 |
| `launch/sensor_scan_generation.launch.py` | C | 启动文件 |

## ROS2 接口

| 方向 | 话题 | 类型 | 用途 |
|---|---|---|---|
| **订阅** | LOAM 里程计 | `nav_msgs::Odometry` | 同步里程计输入 |
| **订阅** | LOAM 配准点云 | `sensor_msgs::PointCloud2` | 同步点云输入 |
| **发布** | 变换后点云 | `sensor_msgs::PointCloud2` | `robot_base_frame_` 坐标系点云 |
| **发布** | 底盘里程计 | `nav_msgs::Odometry` | `robot_base_frame_ → odom` 里程计 |
| **TF** | `lidar_frame_ → robot_base_frame_` | — | 静态变换（计算一次） |

## 调用关系

- **依赖于:** ROS2 (sensor_msgs, nav_msgs, tf2_ros, message_filters)
- **被依赖:** nav_bringup (由 legged_navigation_launch.py 启动，受 `use_sensor_scan` 条件决定)
