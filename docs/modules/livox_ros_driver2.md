# 模块: livox_ros_driver2

> 同步: 2026-07-11 (非 git 仓库) | Tier 分布: A: 4 / B: 6 / C: 多 (含第三方) | 语言: C++
> 路径: `src/navigation/livox_ros_driver2/`

## 职责
Livox 3D 激光雷达的 ROS2 设备驱动: 通过 Livox-SDK2 接收雷达原始以太网点云/IMU 数据, 经解析、外参补偿、按频率组帧后, 以 PointCloud2 / 自定义 CustomMsg / Imu 消息发布到 ROS2 话题。

## 关键文件
| 文件 | Tier | 职责 |
|---|---|---|
| src/lddc.cpp/.h | A | Lidar Data Distribute Component: 从队列取数据、构造 ROS 消息并 publish (543 行, 发布核心) |
| src/comm/pub_handler.cpp/.h | A | SDK 点云观察者: 原始包→标准点、外参补偿、按发布间隔组帧回调 (504 行, 并发热点) |
| src/livox_ros_driver2.cpp | A | 组件节点入口: 声明参数, 构造 Lddc + LdsLidar, 启动两个轮询线程 (132 行) |
| src/driver_node.cpp/.h | A | `DriverNode` (rclcpp::Node 子类); 点云/IMU 轮询线程体与析构清理 |
| src/lds.cpp/.h | B | 雷达数据源抽象基类: 管理 32 路 LidarDevice、点云/IMU 入队、信号量通知 |
| src/lds_lidar.cpp/.h | B | `LdsLidar` (Lds 子类, 单例): 解析配置、初始化 SDK、注册回调 |
| src/call_back/livox_lidar_callback.cpp/.h | B | 雷达信息变更回调: 下发数据类型/扫描模式/盲区/外参/使能 IMU |
| src/call_back/lidar_common_callback.cpp/.h | B | 点云/IMU 通用回调 → `LdsLidar::StoragePointData`/`StorageImuData` |
| src/comm/comm.h | B | 核心类型/常量集中定义 (PointXyzlt、StoragePacket、LidarDataQueue、LidarDevice) |
| src/comm/ldq.cpp/.h | B | 无锁环形点云队列 (LidarDataQueue) 操作 |
| src/comm/{semaphore,cache_index,lidar_imu_data_queue}.* | C | 信号量、handle→index 映射、IMU 队列 |
| src/parse_cfg_file/*.cpp/.h | C | 用 rapidjson 解析配置 JSON |
| msg/CustomMsg.msg, CustomPoint.msg | A | 自定义点云接口消息 |
| config/MID360_config.json | B | 网络参数、雷达 IP、pcl_data_type、外参 |
| **Livox-SDK2/include/**, **3rdparty/rapidjson/** | **第三方** | 厂商 SDK (设备通信/回调注册) + JSON 库 |

## ROS2 接口
- **节点名**: 代码固定 `livox_driver_node`, launch 覆盖为 `livox_lidar_publisher`; 经 `RCLCPP_COMPONENTS_REGISTER_NODE(livox_ros::DriverNode)` 注册为可组合组件; executable `livox_ros_driver2_node`。
- **发布** (由 `Lddc::GetCurrentPublisher*` 决定, 取决于 multi_topic 与 xfer_format):
  - 单话题 (multi_topic=0): `livox/lidar` (随 xfer_format: PointCloud2/CustomMsg), `livox/lidar/pointcloud` (仅 kAllMsg 额外), `livox/imu` (Imu)
  - 多话题 (multi_topic=1): `livox/lidar_<ip>`、`livox/imu_<ip>`
- **关键参数** (livox_ros_driver2.cpp:56-71):
  - `xfer_format`: 0=PointCloud2, 1=CustomMsg, 2=Pcl(ROS2不支持), 3=Imu, 4=All (launch 默认 **4**)
  - `multi_topic` (默认 0)、`data_src` (0=kSourceRawLidar)、`publish_freq` (默认 10Hz, clamp 到 [0.5,100])
  - `frame_id` (launch "livox_frame")、`user_config_path` (JSON 路径)
  - ⚠️ IMU 消息 frame_id 硬编码 "livox_frame" (lddc.cpp:398), 忽略参数

## 公共 API 契约
| 符号 | 签名 | 用途 | 契约 / 不变量 |
|---|---|---|---|
| `DriverNode` | `(const rclcpp::NodeOptions&)` | 节点构造 | 声明读参→构造 Lddc→`LdsLidar::GetInstance(freq)`→RegisterLds→InitLdsLidar→启动两轮询线程。析构: `RequestExit`→`exit_signal_.set_value()`→join (前置线程可被 future 唤醒否则死锁)。 |
| `Lddc::RegisterLds` | `(Lds*) → int` | 绑定数据源 | 仅 `lds_==nullptr` 成功返回 0, 重复返回 -1 (单一数据源不变量)。 |
| `Lddc::DistributePointCloudData` | `()` | 消费+发布点云 | `pcd_semaphore_.Wait()` 阻塞等数据; 仅对 `connect_state==kConnectStateSampling` 的雷达发布。 |
| `Lddc::InitPointcloud2Msg` | `(pkg, cloud, timestamp)` | 组装 PointCloud2 | 7 字段布局 (x/y/z/intensity FLOAT32, tag/line UINT8, timestamp FLOAT64); `point_step=sizeof(LivoxPointXyzrtlt)`, 字段 offset 与 packed 结构逐字节对应 (零拷贝 memcpy 契约)。 |
| `Lds::PushLidarData` | `(PointPacket*, index, base_time)` | 点云入队 | 首次按 `CalculatePacketQueueSize` InitQueue; 入环形队列后 Signal 信号量; 队列满时也 Signal 但丢弃当前包 (lds.cpp:200-211)。 |
| `LdsLidar::InitLdsLidar` | `(const std::string& path) → bool` | 初始化 SDK | InitLidars→SetLidarPubHandle→Start; 重复调用返回 false。 |

## 调用关系
- **依赖**: Livox-SDK2 (第三方, 设备通信+回调)、rapidjson (第三方, 仅配置)、rclcpp/rclcpp_components、sensor_msgs、PCL (Pcl 路径为占位)、自定义 CustomMsg/CustomPoint。
- **被依赖**: point_lio (订阅 `livox/lidar`+`livox/imu`); nav_bringup 实机 launch。

> **注意**: 数据入口在 SDK 线程, 静态调用图不完整。数据流由 SDK 反向回调驱动 (`LivoxLidarAddPointCloudObserver`, `SetLivoxLidarInfoChangeCallback`)。

### 并发模型 (关键)
至少 4 类线程:
1. **SDK 内部网络线程** → `PubHandler::OnLivoxLidarPointCloudCallback` (生产 RawPacket 到 deque, `packet_mutex_`+`packet_condition_`)
2. **`PubHandler::RawDataProcess` 处理线程** (pub_handler.cpp:71): 消费 raw 队列, 做点云换算/外参补偿/组帧, 回调入 LidarDataQueue
3. **`PointCloudDataPollThread`**: `pcd_semaphore_.Wait()` 阻塞, 消费 LidarDataQueue → publish
4. **`ImuDataPollThread`**: `imu_semaphore_.Wait()`, 消费 IMU list → publish

同步原语: 自实现 `Semaphore` (mutex+condition_variable); `LidarDataQueue` 为 volatile rd/wr 索引的 **SPSC 无锁环形队列** (依赖单生产单消费假设); IMU 队列/cache_index 用 `std::mutex`; 退出用 `promise/shared_future` + `volatile bool request_exit_` (非 atomic)。

## 核心数据流
```
LiDAR 硬件 (UDP) → SDK 网络线程
  → OnLivoxLidarPointCloudCallback (pub_handler.cpp:104)
     ├─ IMU: 直接构造 ImuData → LidarImuDataCallback
     └─ 点云: RawPacket → raw_packet_queue_ (deque)
          → RawDataProcess 线程: 原始点→PointXyzlt (换算米+外参补偿)
          → CheckTimer 按 publish_interval 组帧 → OnLidarPointClounCb
               → LdsLidar::StoragePointData → Lds::PushLidarData
                  → QueuePushAny 入 LidarDataQueue → pcd_semaphore_.Signal()
  ── 线程边界 (信号量) ──
  PollThread: Lddc::DistributePointCloudData: Wait()
     → PollingLidarPointCloudData → QueuePop → InitMsg+FillPoints → publish()
        → ROS2 话题 (livox/lidar, livox/imu)
```

## 关键类型
| 类型 | 定义位置 | 用途 |
|---|---|---|
| `CustomMsg` | msg/CustomMsg.msg | Header, timebase, point_num, lidar_id, CustomPoint[] |
| `CustomPoint` | msg/CustomPoint.msg | offset_time, x/y/z, reflectivity, tag, line |
| `LivoxPointXyzrtlt` | comm.h:152 (#pragma pack(1)) | PointCloud2 缓冲区二进制布局单元 (零拷贝 memcpy 契约) |
| `PointXyzlt` | comm.h:163 | 内部标准点 (pub_handler 产出) |
| `LidarDataQueue` | comm.h:213 | SPSC 无锁环形缓冲 (volatile rd/wr_idx) |
| `LidarDevice` | comm.h:288 | 每路雷达聚合 (handle, connect_state, 队列, config) |

> 详细风险 (SPSC 队列多核可见性、`extrinsic_global` 多雷达覆盖、空 else 丢数据、裸指针生命周期、硬编码 IP/frame_id) 见 `PROJECT_DOC.md` Layer 3 注解 #6, #10-11, #17, #22, #42。
