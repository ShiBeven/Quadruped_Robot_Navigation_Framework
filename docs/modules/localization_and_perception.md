# 模块: small_gicp_relocalization

> 上级: [PROJECT_DOC.md](../PROJECT_DOC.md)
> 最后同步: 提交 `5b3f214` — 2026-07-05
> 层级分布: A: 0 / B: 1 / C: 3

## 职责
重定位节点：加载先验全局 PCD 地图，累积配准激光雷达扫描，定期运行 `small_gicp`（GICP）配准以估计并广播 `map → odom` 变换。在局部激光里程计之上提供全局定位修正。

## 关键文件

| 文件 | 层级 | 职责 |
|---|---|---|
| `include/small_gicp_relocalization/small_gicp_relocalization.hpp` | B | 节点类（238 行实现） |
| `src/small_gicp_relocalization.cpp` | C | 实现 |
| `launch/small_gicp_relocalization_launch.py` | C | 启动文件 |

## ROS2 接口

| 方向 | 话题 | 类型 | 用途 |
|---|---|---|---|
| **订阅** | 配准点云 | `sensor_msgs::PointCloud2` | 累积用于扫描-地图匹配 |
| **订阅** | `initialpose` | `geometry_msgs::PoseWithCovarianceStamped` | 手动重定位种子 |
| **TF 广播** | `map → odom` | — | 估计的全局定位修正 |

## 算法

```
先验 PCD 地图（启动时加载）
    +
累积配准扫描（来自回调）
    ↓
定时器: small_gicp::Registration<GICPFactor, ParallelReductionOMP>
    ├── KdTreeOMP: 目标 = 先验地图, 源 = 累积扫描
    ├── 全局 + 配准降采样分辨率
    └── 多线程（N 线程）
    ↓
Eigen::Isometry3d（map → odom 修正）
    ↓
定时器: tf2_ros::TransformBroadcaster → /tf（map→odom）
```

## 关键参数

| 参数 | 默认值 | 描述 |
|---|---|---|
| `num_threads` | 4 | KdTree + 配准的 OpenMP 线程数 |
| `global_leaf_size` | 0.15 | 先验地图降采样分辨率 |
| `registered_leaf_size` | 0.05 | 实时扫描降采样分辨率 |
| `max_dist_sq` | 3.0 | 最大对应距离平方 |
| `robot_base_frame` | `body` | 机器人主体坐标系 |
| `prior_pcd_file` | (用户提供) | 全局先验 PCD 地图路径 |

## 调用关系

- **依赖于:** small_gicp (GICPFactor, ParallelReductionOMP, KdTreeOMP), PCL, Eigen, tf2_ros, nav_msgs
- **被依赖:** nav_bringup (由 legged_localization_launch.py 启动)

---

# 模块: pointcloud_to_laserscan

> 上级: [PROJECT_DOC.md](../PROJECT_DOC.md)
> 最后同步: 提交 `5b3f214` — 2026-07-05
> 层级分布: A: 0 / B: 1 / C: 3

## 职责
将 3D 点云转换为 2D LaserScan 消息，方法是将可配置高度/强度范围内的点投影到平面扫描中。使用 `tf2_ros::MessageFilter` 进行帧同步。

## 关键文件

| 文件 | 层级 | 职责 |
|---|---|---|
| `include/pointcloud_to_laserscan/pointcloud_to_laserscan_node.hpp` | B | 节点类（244 行实现） |
| `src/pointcloud_to_laserscan_node.cpp` | C | 实现 |
| `include/pointcloud_to_laserscan/laserscan_to_pointcloud_node.hpp` | C | 反向转换器 |
| `launch/*` | C | 启动文件 |

## ROS2 接口

| 方向 | 话题 | 类型 | 用途 |
|---|---|---|---|
| **订阅** | (可配置) | `sensor_msgs::PointCloud2` | 输入 3D 点云（TF 同步） |
| **发布** | `scan` | `sensor_msgs::LaserScan` | 输出 2D 激光扫描 |

**SLAM 模式下:** `cloud_in` 重映射到 `terrain_map_ext`，`scan` 重映射到 `obstacle_scan`。

## 关键参数

| 参数 | 描述 |
|---|---|
| `target_frame` | 输出扫描坐标系（默认: `base_footprint`） |
| `min_height` / `max_height` | Z 轴点过滤 |
| `range_min` / `range_max` | 0.15–10.0 m（SLAM 配置） |
| `angle_min` / `angle_max` | 全圆（-π 到 π） |
| `angle_increment` | 角度分辨率 |

## 调用关系

- **依赖于:** tf2_ros (MessageFilter), sensor_msgs, PCL
- **被依赖:** nav_bringup (由 legged_slam_launch.py 启动，供给 slam_toolbox)

---

# 模块: teleop_twist_joy

> 上级: [PROJECT_DOC.md](../PROJECT_DOC.md)
> 最后同步: 提交 `5b3f214` — 2026-07-05
> 层级分布: A: 0 / B: 1 / C: 3

## 职责
手柄遥控：将手柄轴值转换为 `geometry_msgs/Twist` 速度命令，支持涡轮/使能按钮、倒车反转切换，以及可选的 Nav2 `NavigateToPose` 目标分发。

## 关键文件

| 文件 | 层级 | 职责 |
|---|---|---|
| `include/teleop_twist_joy/teleop_twist_joy.hpp` | B | TeleopTwistJoyNode 类 |
| `src/pb_teleop_twist_joy.cpp` | C | 实现（199 行） |
| `config/xbox.config.yaml` | C | Xbox 手柄映射 |

## ROS2 接口

| 方向 | 话题 | 类型 | 用途 |
|---|---|---|---|
| **订阅** | 手柄 | `sensor_msgs::Joy` | 原始手柄输入 |
| **发布** | `cmd_vel` | `geometry_msgs::Twist` (或 `TwistStamped`) | 速度输出 |
| **动作客户端** | `navigate_to_pose` | `nav2_msgs::action::NavigateToPose` | Nav2 目标分发 |

## 调用关系

- **依赖于:** ROS2 (sensor_msgs, geometry_msgs, nav2_msgs)
- **被依赖:** (独立模块 — 手动操作入口点)

---

# 模块: ign_sim_pointcloud_tool

> 上级: [PROJECT_DOC.md](../PROJECT_DOC.md)
> 最后同步: 提交 `5b3f214` — 2026-07-05
> 层级分布: A: 0 / B: 0 / C: 2

## 职责
Gazebo Ignition 仿真工具：在不同格式之间转换仿真点云数据。

## 调用关系

- **依赖于:** ROS2, PCL
- **被依赖:** (仿真工具 — 不在运行时导航中使用)
