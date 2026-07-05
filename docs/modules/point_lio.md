# 模块: point_lio

> 上级: [PROJECT_DOC.md](../PROJECT_DOC.md)
> 最后同步: 提交 `5b3f214` — 2026-07-05
> 层级分布: A: 4 / B: 5 / C: 10

## 职责
快速激光-惯性里程计，通过流形上的迭代误差状态卡尔曼滤波器（ESKF）实现。通过融合激光扫描-地图配准（点-面残差）和 IMU 传播提供实时 6 自由度姿态估计。支持多种激光雷达型号（Livox AVIA、Velodyne VLP-16、Ouster OS-64、Hesai XT32），使用 ivox 空间哈希实现高效 KNN 查询。

## 关键文件

| 文件 | 层级 | 职责 |
|---|---|---|
| `src/laserMapping.cpp` | A | 主入口：ROS2 节点、ESKF 循环、地图增量更新、所有发布器（1052 行） |
| `src/preprocess.h` | A | 激光预处理：点-帧标记、多厂商处理程序分发 |
| `src/IMU_Processing.h` | A | IMU 前向/反向传播、重力初始化、点云去畸变 |
| `src/Estimator.h` | A | 卡尔曼滤波器模型：状态转移、激光观测（点-面残差） |
| `include/common_lib.h` | A | MTK 流形定义（24D/30D 状态）、MeasureGroup 结构体、工具函数 |
| `include/so3_math.h` | B | SO(3) 李群数学：Exp/Log 映射、反对称矩阵、雅可比矩阵 |
| `src/li_initialization.h/.cpp` | B | ROS2 传感器回调、sync_packages() 测量同步 |
| `src/parameters.h/.cpp` | B | ~100+ 个 ROS2 参数声明和全局状态 |
| `src/preprocess.cpp` | C | Preprocess 实现（935 行，标题逻辑的实现） |
| `src/IMU_Processing.cpp` | B | IMU 处理实现 |
| `src/Estimator.cpp` | B | 估计器模型实现 |
| `include/ivox/*` | C | 3D 空间哈希 ivox 地图（PHC/DEFAULT 节点类型） |
| `include/matplotlibcpp.h` | C | 仅头文件的 C++ matplotlib 封装（调试可视化） |
| config, launch, msg, rviz | C | 配置和启动文件 |

## 公共 API 参考

| 符号 | 签名 | 用途 |
|---|---|---|
| `main()` | `int main(int argc, char **argv)` | 节点入口：运行 ESKF 循环 |
| `Preprocess::process()` | `void process(const CustomMsg::SharedPtr&, PointCloudXYZI::Ptr&)` | Livox 扫描 → 结构化点云 |
| `Preprocess::process()` | `void process(const PointCloud2::SharedPtr&, PointCloudXYZI::Ptr&)` | 标准扫描 → 结构化点云 |
| `Preprocess::avia_handler()` | `void avia_handler(const CustomMsg::SharedPtr&)` | Livox AVIA 原始数据 → 内部格式 |
| `Preprocess::give_feature()` | `void give_feature(PointCloudXYZI&, vector<orgtype>&)` | 逐环点分类（平面/边缘） |
| `ImuProcess::Process()` | `void Process(const MeasureGroup&, PointCloudXYZI::Ptr)` | IMU 传播 + 点去畸变 |
| `ImuProcess::IMU_init()` | `void IMU_init(const MeasureGroup&, int&)` | 静止 IMU 偏置/重力标定 |
| `get_f_input()` | `Eigen::Matrix<double,24,1> get_f_input(state_input&, const input_ikfom&)` | 24D 状态转移（IMU 作为输入） |
| `get_f_output()` | `Eigen::Matrix<double,30,1> get_f_output(state_output&, const input_ikfom&)` | 30D 状态转移（IMU 作为观测） |
| `h_model_input()` | `void h_model_input(state_input&, ...)` | 点-面观测模型（24D 滤波器） |
| `h_model_output()` | `void h_model_output(state_output&, ...)` | 点-面观测模型（30D 滤波器） |
| `pointBodyToWorld()` | `void pointBodyToWorld(PointType const*, PointType*)` | 将点从载体坐标系变换到世界坐标系 |
| `MapIncremental()` | `void MapIncremental()` | 将降采样世界坐标点插入 ivox 地图 |
| `sync_packages()` | `bool sync_packages(MeasureGroup&)` | 将一帧激光扫描与其 IMU 数据同步 |

## ROS2 接口

**订阅：**

| 话题 (可配置) | 类型 | 回调 |
|---|---|---|
| `lid_topic` (Livox) | `livox_ros_driver2::msg::CustomMsg` | `livox_pcl_cbk` |
| `lid_topic` (标准) | `sensor_msgs::msg::PointCloud2` | `standard_pcl_cbk` |
| `imu_topic` | `sensor_msgs::msg::Imu` | `imu_cbk` |

**发布：**

| 话题 | 类型 | 内容 |
|---|---|---|
| `cloud_registered` | `sensor_msgs::msg::PointCloud2` | 世界坐标系去畸变扫描 |
| `cloud_registered_body` | `sensor_msgs::msg::PointCloud2` | 载体坐标系去畸变扫描 |
| `cloud_effected` | `sensor_msgs::msg::PointCloud2` | 用于配准的降采样扫描 |
| `Laser_map` | `sensor_msgs::msg::PointCloud2` | 实时局部地图 (ivox → 点云) |
| `aft_mapped_to_init` | `nav_msgs::msg::Odometry` | 6 自由度里程计输出 |
| `path` | `nav_msgs::msg::Path` | 累积轨迹 |

**TF 广播:** `camera_init → aft_mapped`（受 `tf_send_en` 条件控制）

## 状态估计架构

```
传感器输入              预处理                     状态估计                   输出
───────────           ──────────               ────────────              ────
Livox/标准激光雷达 →   Preprocess::process() →  ESKF 更新               里程计
                     (帧切割、                  (对 ivox 做点-面配准)    TF
IMU                  → 特征提取)                                    点云
                     ImuProcess::Process() →  ESKF 预测
                     (前向/反向传播、去畸变)    (IMU 积分)
```

**两种 ESKF 变体（由 `use_imu_as_input` 参数选择）：**
- `kf_input` (24D): pos, rot, offset_R_L_I, offset_T_L_I, vel, bg, ba, gravity
- `kf_output` (30D): 同上 + omg, acc (IMU 作为观测，额外自由度)

## 关键类型

| 类型 | 定义位置 | 用途 |
|---|---|---|
| `PointType` | `common_lib.h` | `pcl::PointXYZINormal` — `curvature` 字段重用于逐点时间戳 |
| `PointCloudXYZI` | `common_lib.h` | `pcl::PointCloud<PointType>` |
| `state_input` | `common_lib.h` | 24D MTK 流形（IMU-作为-输入模式） |
| `state_output` | `common_lib.h` | 30D MTK 流形（IMU-作为-观测模式） |
| `input_ikfom` | `common_lib.h` | 6D IMU 输入流形 (acc, gyro) |
| `MeasureGroup` | `common_lib.h` | 1 帧激光扫描 + IMU 消息双端队列 |
| `IVoxType` | `parameters.h` | PHC 或 DEFAULT 3D 空间哈希地图（用于 KNN） |
| `LID_TYPE` | `preprocess.h` | AVIA=1, VELO16=2, OUST64=3, HESAIxt32 |
| `Feature` | `preprocess.h` | Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint |

## 调用关系

- **依赖于:** PCL, Eigen, livox_ros_driver2 (CustomMsg), ROS2 (sensor_msgs, nav_msgs, tf2)
- **被依赖:** nav_bringup (启动), loam_interface (里程计转发), terrain_analysis (cloud_registered → terrain_map), small_gicp_relocalization (cloud_registered → 扫描-地图配准)

> **注意:** 本模块使用了 ivox 空间哈希库（内置于 `include/ivox/`）和 IKFoM/esekfom（内置于 `include/IKFoM/`）。`common_lib.h` 中的 MTK（流形工具包）宏在编译时生成流形类型 — 这些不是运行时依赖。
