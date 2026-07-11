# 模块组: 里程计桥接与定位 (loam_interface / sensor_scan_generation / small_gicp_relocalization)

> 同步: 2026-07-11 (非 git 仓库) | 语言: C++ | 均为 point_lio 与 nav2 之间的适配/定位层
> 路径: `src/navigation/{loam_interface, sensor_scan_generation, small_gicp_relocalization}/`

---

## loam_interface — LIO 里程计桥接

### 职责
把 point_lio (基于 `lidar_odom` 帧) 输出的里程计与配准点云, 经 `base_frame→lidar_frame` 外参变换转换到 `odom` 帧后重新发布, 作为 point_lio 与 nav2/sensor_scan 之间的适配层。

### 关键文件
| 文件 | Tier | 职责 |
|---|---|---|
| src/loam_interface.cpp | A | 节点实现 (92 行) |
| include/loam_interface/loam_interface.hpp | B | 类声明 (48 行) |

### ROS2 接口 (node `loam_interface`, 可组合组件)
- **订阅**: `<state_estimation_topic>`(Odometry, 配置=`aft_mapped_to_init`)、`<registered_scan_topic>`(PointCloud2, 配置=`cloud_registered`)
- **发布**: `lidar_odometry`(Odometry)、`registered_scan`(PointCloud2)
- **参数**: `state_estimation_topic`、`registered_scan_topic`、`odom_frame`(代码默认 "odom")、`base_frame`、`lidar_frame` (后两者代码默认 `""`, 实际值 base_footprint/front_mid360 由 nav2_params.legged.yaml 注入)

### 公共 API 契约
| 符号 | 用途 | 契约 / 不变量 |
|---|---|---|
| `odometryCallback` | 里程计帧变换 | 首帧惰性查 `base_frame→lidar_frame` TF (0.5s 超时) 初始化 `tf_odom_to_lidar_odom_`, 失败 return 重试; 之后输出 frame_id=`odom`, child=`lidar_frame`。 |
| `pointCloudCallback` | 点云帧变换 | 用 `pcl_ros::transformPointCloud` 把点云搬到 odom 帧。⚠️ 依赖 odom 回调先成功初始化外参, 否则用未初始化 Transform (风险 #见下)。 |

> **风险**: 外参只初始化一次 (假设 base↔lidar 静态), 四足腿部运动若影响该 TF 则错误; 点云回调在初始化前收到会用未定义 Transform。见 Layer 3 相关。

---

## sensor_scan_generation — 传感器扫描 + 底盘里程计生成

### 职责
用 message_filters 时间同步 `lidar_odometry` 与 `registered_scan`, 计算并广播 `odom→base_frame` TF、发布 `robot_base_frame` 底盘里程计 (含数值微分速度), 并把点云变换回传感器局部帧输出。

### 关键文件
| 文件 | Tier | 职责 |
|---|---|---|
| src/sensor_scan_generation.cpp | A | 节点实现 (152 行) |

### ROS2 接口 (node `sensor_scan_generation`, 可组合组件)
- **订阅** (ApproxTime 同步, 队列100, BEST_EFFORT): `lidar_odometry`(Odometry)、`registered_scan`(PointCloud2)
- **发布**: `sensor_scan`(PointCloud2)、`odometry`(Odometry) + TF broadcast `odom→base_frame`
- **参数**: `lidar_frame`(front_mid360)、`base_frame`(base_footprint)、`robot_base_frame`(body)

### 公共 API 契约
| 符号 | 用途 | 契约 / 不变量 |
|---|---|---|
| `laserCloudAndOdometryHandler` | 主同步回调 | `tf_odom_to_lidar` 来自 odom 消息 pose; 乘 lidar→base/robot_base TF 得底盘/本体位姿; 广播 TF + 发 odom; 点云用 inverse 变换到 lidar_frame 发 `sensor_scan`。 |
| `publishOdometry` | 里程计+速度 | 用 `steady_clock` 差分算速度 (首帧跳过)。⚠️ 用墙钟非 ROS 时钟, use_sim_time 下失真。 |
| `getTransform` | TF 查询 | ⚠️ 失败返回 **identity** (仅 WARN), 静默降级污染下游位姿。 |

> **风险**: TF 失败返回 identity; 速度用 steady_clock; 角速度跨 π 无归一化。见 Layer 3 注解 #21。

---

## small_gicp_relocalization — 先验地图重定位

### 职责
加载先验 PCD 全局地图, 累积 `registered_scan` 用 small_gicp (GICP+OMP) 配准, 以 2Hz 求解、20Hz 广播 `map→odom` TF, 实现相对先验地图的重定位; 支持 `initialpose` 手动重置。

### 关键文件
| 文件 | Tier | 职责 |
|---|---|---|
| src/small_gicp_relocalization.cpp | A | 节点实现 (238 行) |

### ROS2 接口 (node `small_gicp_relocalization`)
- **订阅**: `registered_scan`(PointCloud2)、`initialpose`(PoseWithCovarianceStamped)
- **发布**: TF broadcast `map→odom`
- **参数**: num_threads(4)、num_neighbors(20)、global/registered_leaf_size、max_dist_sq、map/odom/base/robot_base/lidar_frame、prior_pcd_file、init_pose[6]

### 公共 API 契约
| 符号 | 用途 | 契约 / 不变量 |
|---|---|---|
| `loadGlobalMap` | 加载先验地图 | ⚠️ 读 PCD 失败仅 ERROR return (不退出); 查 base→lidar TF 最多 100 次全失败则 FATAL + `rclcpp::shutdown()` (构造期调用, 副作用重)。 |
| `performRegistration` | 配准 (500ms timer) | 空累积 return; voxel 下采样→估协方差→KdTree→align; 收敛更新 result_t_ 否则 WARN 保留上次; max_iter 10; 末尾清空累积。 |
| `publishTransform` | 发 TF (50ms timer) | result_t_ 全零跳过; ⚠️ stamp = `last_scan_time_ + 0.1s` (未来外推)。 |
| `initialPoseCallback` | 手动重置 | 用 robot_base→current_scan_frame TF 组合出 map→odom 重置。 |

> **风险**: PCD 失败处理与 TF FATAL 不对称; 构造期 shutdown; +0.1s 未来外推; 未纳入 lifecycle_manager。见 Layer 3 注解 #24, #44。

---

## 三包核心数据流
```
point_lio (aft_mapped_to_init + cloud_registered, lidar_odom 帧)
  → loam_interface (帧变换) → lidar_odometry + registered_scan (odom 帧)
      → sensor_scan_generation (时间同步) → TF odom→base_footprint + odometry + sensor_scan
      → small_gicp_relocalization (配 registered_scan vs 先验PCD) → TF map→odom
```
- **依赖**: pcl_ros/pcl_conversions、tf2、message_filters (sensor_scan)、small_gicp 库 (relocalization)、rclcpp。
- **被依赖**: sensor_scan 的 `odometry` 是 nav2 的 odom_topic; terrain_analysis 消费 lidar_odometry/registered_scan; small_gicp 的 map→odom TF 供 nav2 全局定位。
