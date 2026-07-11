# 模块组: 仿真与工具 (nav2_loopback_sim / pcd2pgm / rosbag2_composable_recorder / teleop_twist_joy)

> 同步: 2026-07-11 (非 git 仓库) | 语言: Python + C++ | 旁路工具, 非核心数据流
> 路径: `src/simulation/nav2_loopback_sim/`, `src/tools/{pcd2pgm, rosbag2_composable_recorder}/`, `src/navigation/teleop_twist_joy/`

---

## nav2_loopback_sim — 无物理引擎回环仿真器

### 职责
订阅 `cmd_vel` 累积积分出位姿, 伪造 odom/TF/laser scan, 替代 Gazebo 等物理仿真为 Nav2 提供最小闭环环境。

### 关键文件
| 文件 | Tier | 职责 |
|---|---|---|
| nav2_loopback_sim/loopback_simulator.py | A | 主节点 `LoopbackSimulator`: 速度积分→TF/odom/scan (422 行) |
| nav2_loopback_sim/utils.py | B | 位姿/四元数/矩阵/地图坐标转换工具 (65 行) |
| nav2_loopback_sim/tf_compat.py | C | tf_transformations 缺失时回退 transforms3d 的 shim (55 行) |
| launch/loopback_simulation.launch.py, bringup_launch.py | B/C | 启动 |
| params/nav2_params.yaml | C | Nav2 全栈参数 |

### ROS2 接口 (node `loopback_simulator`)
- **订阅**: `initialpose`(PoseWithCovarianceStamped, 首次触发才启动积分)、`cmd_vel`(Twist 或 TwistStamped, 依 enable_stamped_cmd_vel)
- **发布**: `odom`(Odometry)、`scan`(LaserScan, BEST_EFFORT)、`/clock`(Clock, 条件)、TF `map→odom`(可选)+`odom→base_footprint`
- **服务客户端**: `/map_server/map`(GetMap)
- **关键参数**: update_duration(0.01/yaml 0.02)、base/odom/map/scan_frame_id、enable_stamped_cmd_vel(True/yaml false)、scan_range/angle_*、publish_map_odom_tf、publish_clock

### 核心机制 (如何伪造 odom/TF/scan)
- `map→odom` 由 initialpose 一次性设定; `odom→base_link` 从单位阵起
- `timerCallback`: 一阶欧拉积分 `dx=v_x·dt` 按 yaw 旋转累加 (loopback_simulator.py:255-266), 无摩擦/惯性/碰撞
- 收新 initialpose (非首次) 保持 odom→base_link, 反解 `map→odom=(map→base)·(odom→base)⁻¹`
- `cmd_vel` 超 1s 未更新则只复发旧 TF 不积分
- scan 由 `getLaserScan` 用 LineIterator 在 OccupancyGrid 光线投射 (遇 occupancy≥60 记距离)

### 公共 API 契约
| 符号 | 用途 | 契约 / 不变量 |
|---|---|---|
| `cmdVelCallback` / `cmdVelStampedCallback` | 缓存速度 | `initial_pose is None` 时丢弃速度 |
| `initialPoseCallback` | 启动开关 | 首次调用 cancel setupTimer, 创建积分 timer 与 scan timer; 只有首次设 map→odom |
| `timerCallback` | 积分主循环 | 发布 TF + odom |
| `publishTransforms` | 发 TF | map→odom stamp 前移 update_dur (未来化防外推报错) |

> **风险**: 运行时 monkey-patch np.float 兼容 NumPy 2.x (隐式全局副作用); 宽泛 except; occupancy 阈值/LineIterator 步长硬编码; enable_stamped_cmd_vel 默认与 yaml 不一致。见 Layer 3 注解 #26, #45。

---

## pcd2pgm — PCD 点云转 PGM 占据栅格

### 职责
离线工具节点: 加载 `.pcd` 点云地图, 经坐标变换 + Z 直通滤波 + 半径离群滤波后投影为 2D `OccupancyGrid` 并周期发布 (供保存为 .pgm)。

### 关键文件
| 文件 | Tier | 职责 |
|---|---|---|
| src/pcd2pgm.cpp | A | `Pcd2PgmNode` 实现 (175 行) |
| include/pcd2pgm/pcd2pgm.hpp | B | 类声明 (60 行) |

### ROS2 接口 (node `pcd2pgm`, 可组合但 launch 独立进程启动)
- **订阅**: 无
- **发布**: `map`(OccupancyGrid, transient_local)、`pcd_cloud`(PointCloud2); 1s 周期重复发布
- **参数**: pcd_file、thre_z_min/max(0.5/2.0)、flag_pass_through、thre_radius、map_resolution(0.05)、thres_point_count(10)、map_topic_name、odom_to_lidar_odom[6]

### 公共 API 契约
| 符号 | 用途 | 契约 / 不变量 |
|---|---|---|
| `Pcd2PgmNode` (构造) | 完整管线 | declare→loadPCD→applyTransform→passThrough→radiusOutlier→setMapTopicMsg→1s timer。⚠️ 加载失败 return, 节点存活无输出 |
| `applyTransform` | 坐标变换 | 用 `odom_to_lidar_odom_` 构 Affine3f 对点云施加**逆变换** |
| `setMapTopicMsg` | 栅格化 | 求 xy 包围盒→定 origin/尺寸→占据格置 100 (空点云 0, 非 nav2 惯例的 -1) |

> **风险**: `odom_to_lidar_odom_[0..5]` 无长度校验 (<6 越界崩溃); PCD 失败仅 return; frame_id "map" 硬编码。见 Layer 3 注解 #25。

---

## rosbag2_composable_recorder — 可组合 rosbag2 录制器

### 职责
继承 `rosbag2_transport::Recorder` 的可组合录制节点, 提供 start/stop 触发服务, 可与驱动同进程 intra-process 通信降低录制开销。

### 关键文件
| 文件 | Tier | 职责 |
|---|---|---|
| src/composable_recorder.cpp | A | `ComposableRecorder` 实现 + 组件注册 (146 行) |
| include/.../composable_recorder.hpp | B | 类声明 (37 行) |
| src/composable_recorder_node.cpp | C | 独立进程 main() 包装 (16 行) |
| src/start_recording.py | C | 调用 start_recording 服务的脚本 (44 行) |

### ROS2 接口 (node `recorder`)
- **服务**: `start_recording`(Trigger, 仅非立即录制时创建)、`stop_recording`(Trigger)
- **录制话题**: 由 `topics` 参数或 `record_all` 决定, 运行时动态订阅
- **参数**: bag_name/prefix、topics、storage_id(sqlite3)、max_cache_size、record_all、start_recording_immediately

### 公共 API 契约
| 符号 | 用途 | 契约 / 不变量 |
|---|---|---|
| `startRecording` | 触发录制 | 已录则拒绝; uri=bag_name 或 prefix+时间戳; 捕获 runtime_error; 始终 return true |
| `stopRecording` | 停止 | 未录制则拒绝; 调用 stop() |

> ⚠️ **动态加载 (FLAG)**: 这是全项目**唯一实际经 ComposableNodeContainer + intra-process 加载**的可组合节点 (设计核心, `use_intra_process_comms: True` 零拷贝录制)。`RCLCPP_COMPONENTS_REGISTER_NODE` (composable_recorder.cpp:146)。
> **风险**: 多个 `#ifdef` 跨 rosbag2 版本兼容; 仅捕获 runtime_error; 服务等待无超时。

---

## teleop_twist_joy — 手柄遥操作

### 职责
通用手柄遥操作节点: 将 `joy` 摇杆轴映射为底盘 `cmd_vel` (manual_control), 或在 auto_control 模式下映射为 `NavigateToPose` 目标点动作。

### 关键文件
| 文件 | Tier | 职责 |
|---|---|---|
| src/pb_teleop_twist_joy.cpp | A | `TeleopTwistJoyNode` 实现 (199 行) |
| include/teleop_twist_joy/teleop_twist_joy.hpp | B | 类声明 (64 行) |
| config/xbox.config.yaml | C | Xbox 轴/按钮/缩放映射 |

### ROS2 接口 (node `teleop_twist_joy_node`, 可组合但 launch 独立启动)
- **订阅**: `joy`(Joy)
- **发布**: `cmd_vel`(Twist 或 TwistStamped, 依 publish_stamped_twist; launch 可 remap 为 joy_vel)
- **动作客户端**: `navigate_to_pose`(NavigateToPose, 仅 auto_control)
- **参数**: publish_stamped_twist、robot_base_frame、control_mode、enable_button(5/yaml 4)、enable_turbo_button、axis_chassis{x,y,yaw}、scale_chassis(_turbo)

### 公共 API 契约
| 符号 | 用途 | 契约 / 不变量 |
|---|---|---|
| `joyCallback` | 主回调 | turbo 按下→turbo 速度; enable 按下→normal; 松开时发一次零命令停车 (边沿触发) |
| `getVal` | 取轴值 | axis=-1/越界/字段缺失返回 0.0 (安全默认) |
| `sendGoalPoseAction` | auto 目标点 | \|x\|,\|y\|≤0.1 死区跳过; TF map 变换失败 warn+return; static 节流 0.25s |

### 可复用性/正确性改造 (2026-07-11)
- ✅ **移除失效的死读取**: `fillCmdVelMsg` 原先从 `axis_chassis_map_` 取 `"z"/"pitch"/"roll"` 三个键填 linear.z/angular.y/angular.x, 但该 map 只有 x/y/yaw, `getVal` 找不到字段恒返回 0 —— 这三个自由度**永远输出 0**。现只填 chassis 实际定义的 x/y/yaw, 并加注释说明云台 (gimbal) 需独立话题/消息, 不应挤进 chassis 的 cmd_vel Twist。
- ⚠️ **仍待办**: yaml 里的 `axis_gimbal`/`scale_gimbal` 依旧无人读取。若要真正支持云台, 需新增独立的 gimbal 发布路径 (专用话题 + 消息类型), 属功能新增而非 bug 修复, 需产品决定。当前包描述"chassis+gimbal"仍不准确。

> 详见 Layer 3 注解 #35 (已部分处理: 死读取移除; 云台功能仍未实现)。

---

## 汇总
- **可组合节点 (rclcpp_components)**: pcd2pgm、rosbag2_composable_recorder、teleop_twist_joy 均注册为 component, 但仅 rosbag2_composable_recorder 实际经容器 intra-process 加载。
- **TF 使用**: loopback_sim (广播 map→odom→base + 监听 base→scan)、teleop (监听 map→base 做目标变换)。
- **依赖**: rclpy(loopback)、rclcpp/rclcpp_components、nav2_simple_commander(loopback line_iterator)、PCL+Eigen(pcd2pgm)、rosbag2_transport(recorder)、tf2/nav2_msgs(teleop)。
