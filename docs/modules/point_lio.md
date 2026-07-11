# 模块: point_lio

> 同步: 2026-07-11 (非 git 仓库, 无 commit hash) | Tier 分布: A: 5 / B: 4 / C: 若干 (含第三方头) | 语言: C++
> 路径: `src/navigation/point_lio/`
> 变更 (2026-07-11): 输出 TF 帧名与 PCD 存盘路径已参数化 (见下方"可复用性改造"), 消除硬编码, 默认值向后兼容。

## 职责
Point-LIO 紧耦合、逐点更新 (point-by-point) 的激光雷达-惯性里程计 (LiDAR-Inertial Odometry) ROS2 节点; 基于 IKFoM 流形迭代扩展卡尔曼滤波融合 LiDAR 点云与 IMU, 输出高频位姿 (odometry)、路径与配准点云地图。

## 关键文件
| 文件 | Tier | 职责 |
|---|---|---|
| src/laserMapping.cpp | A | ROS2 节点 main 入口; 订阅/发布、主循环、EKF predict/update 编排、地图增量与发布 (1062 行) |
| src/Estimator.cpp/.h | A | EKF 过程模型/雅可比 (`get_f_*`/`df_dx_*`) 与量测模型 (`h_model_*`)、点体到世界变换; 持大量全局状态 (391 行) |
| src/preprocess.cpp/.h | A | 各型号 LiDAR 点云解析 (Avia/Ouster/Velodyne/Hesai)、降采样、切帧、特征提取 (935 行) |
| src/li_initialization.cpp/.h | A | 传感器回调 (LiDAR/Livox/IMU)、缓冲队列与 `sync_packages` 时间同步打包 (293 行) |
| src/parameters.cpp/.h | A | `readParameters` 声明读取所有 ROS 参数、`SO3ToEuler`、协方差初始化; 全部全局配置的 extern 声明 |
| src/IMU_Processing.cpp/.h | B | IMU 初始化 (静止求均值)、重力对齐求初始旋转、LiDAR 帧透传为去畸变点云 (114 行) |
| include/common_lib.h | B | MTK 流形状态类型定义、`MeasureGroup`、`esti_plane`、时间/宏工具 |
| launch/point_lio.launch.py | B | 启动 `pointlio_mapping` 节点 (加载 mid360.yaml) + 可选 RViz |
| config/mid360.yaml | B | Livox Mid360 参数集 (topics、外参、协方差、重力) |
| include/IKFoM/, include/ivox/, include/matplotlibcpp.h | C (第三方) | IKFoM 流形 ESEKF 工具、iVox 增量体素近邻地图、matplotlib 绘图 (不深读) |

## ROS2 接口
- **节点名**: `laserMapping` (代码), executable `pointlio_mapping` (launch)。用 `MultiThreadedExecutor` + `SensorDataQoS`。
- **订阅**:
  - `lid_topic` (代码默认字符串 `".livox.lidar"` — 畸形点分格式, 实际值 `livox/lidar` 由 mid360.yaml 注入) — `lidar_type==AVIA` 时为 `livox_ros_driver2/msg/CustomMsg` → `livox_pcl_cbk`; 否则 `sensor_msgs/PointCloud2` → `standard_pcl_cbk`
  - `imu_topic` (代码默认 `".livox.imu"`, 实际 `livox/imu` 由 yaml 注入) — `sensor_msgs/Imu` → `imu_cbk`
- **发布**:
  - `aft_mapped_to_init` (nav_msgs/Odometry) — 主里程计输出
  - `cloud_registered` (PointCloud2, 世界系配准点云) / `cloud_registered_body` (IMU body 系)
  - `path` (nav_msgs/Path) / `Laser_map` (初始地图) / `cloud_effected` (声明但未见发布)
  - TF: `camera_init` → `aft_mapped` (受 `tf_send_en` 控制); world=`camera_init`, body=`body`
- **关键参数** (parameters.cpp:62-248): `use_imu_as_input` (输入/输出模型切换)、`prop_at_freq_of_imu`、`check_satu`、`space_down_sample`、`mapping.imu_en`、`preprocess.lidar_type`/`scan_line`/`blind`、`filter_size_surf`/`filter_size_map`、`mapping.ivox_grid_resolution`、各类协方差 (`lidar_meas_cov`→`laser_point_cov`, `acc/gyr_cov_*`)、`mapping.extrinsic_T`/`extrinsic_R`、`gravity`、`prior_pcd.enable`/`prior_pcd_map_path`。
  - **帧名/路径参数** (2026-07-11 新增, 见"可复用性改造"): `publish.world_frame`(camera_init)、`publish.body_frame`(body)、`publish.aft_mapped_frame`(aft_mapped)、`pcd_save.pcd_save_dir`("")。
  - ⚠️ 参数默认值 (parameters.cpp) 与 yaml 值、parameters.cpp 顶部初值三处并存, 可能不一致 (如 `init_map_size` 默认 100 vs yaml 10; `ivox_nearby_type` 18/6/6)。

## 公共 API 契约
| 符号 | 签名 | 用途 | 契约 / 不变量 |
|---|---|---|---|
| `main` | `(int,char**)` laserMapping.cpp:324 | 初始化 + 500Hz 主循环 | 前置: `readParameters` 先执行填充全局, `ivox_` 循环前构建。双分支互斥: `use_imu_as_input` 走 `kf_input`(24维) 否则 `kf_output`(30维), 两路径大量重复。首帧设 `first_lidar_time`/重力; 累计到 `init_map_size` 点才建图, 之前 continue。SIGINT→`flg_exit`。 |
| `sync_packages` | `(MeasureGroup&) → bool` li_initialization.cpp:177 | 打包一个测量组 | 需 `last_timestamp_imu >= lidar_end_time` 才返回 true (IMU 未覆盖帧尾则等待); `imu_en==false` 仅打包 LiDAR。副作用: pop 缓冲、置 lidar_pushed/imu_pushed。 |
| `ImuProcess::Process` | `(const MeasureGroup&, PointCloudXYZI::Ptr)` | IMU 初始化 + 点云透传 | 累计 `MAX_INI_COUNT(100)` 帧做初始化; 期间不产出可用点云。注意: 不做逐点运动去畸变 (在主循环逐点 predict 隐式完成)。 |
| `h_model_input` / `h_model_output` | `(state&, ..., dyn_share_modified&)` Estimator.cpp:112/218 | LiDAR 平面量测模型 | 对 `time_seq[k]` 内点 `pointBodyToWorld` 后 `ivox_->GetClosestPoint` 取 5 近邻, `esti_plane` 拟合, 点面距筛选。依赖全局 `k`/`idx`/`feats_down_*`/`Nearest_Points`; 无有效点则 `valid=false`。`extrinsic_est_en` 决定外参是否入雅可比。**两函数逐行几乎相同**。 |
| `h_model_IMU_output` | `(state_output&, dyn_share_modified&)` Estimator.cpp:324 | IMU 量测 (输出模型) | 残差 = 角速度/加速度观测减状态; `check_satu` 时对饱和轴置零残差。 |
| `pointBodyToWorld` | `(PointType const*, PointType*)` Estimator.cpp:364 | LiDAR 系点→世界系 | 依赖对应 kf 已 predict 到当前时刻; 按 `extrinsic_est_en`/`use_imu_as_input` 选 rot/pos。 |
| `kf_*.predict` | `(dt, Q, input, predict_state, prop_cov)` | EKF 传播 (IKFoM) | 主循环两种调用: `(…,true,false)` 只传播状态, `(…,false,true)` 只传播协方差。 |

## 调用关系
- **依赖**: livox_ros_driver2 (CustomMsg 接口)、PCL、Eigen、IKFoM (MTK 流形+ESEKF)、ivox3d (近邻地图)、OpenMP、glog、Python(matplotlibcpp)。
- **被依赖**: loam_interface (订阅 `aft_mapped_to_init`+`cloud_registered`); nav_bringup 三套 launch 均启动本节点。

> **注意**: 本模块含动态/反射模式, 上方静态依赖不完整:
> - MTK 宏 `MTK_BUILD_MANIFOLD` 元编程生成流形状态类型 (编译期反射)。
> - PCL `POINT_CLOUD_REGISTER_POINT_STRUCT` 注册 velodyne/hesai/ouster 点结构 (preprocess.h:52-98)。
> - `lidar_type` 运行时决定订阅消息类型与 handler 分派; 回调用 lambda 包装函数指针注入。
> - IKFoM 经 `init_dyn_share_modified_2h/3h` 注入 `get_f`/`df_dx`/`h_model` 函数指针 (laserMapping.cpp:364-366)。

## 关键类型
| 类型 | 定义位置 | 用途 |
|---|---|---|
| `PointType` = pcl::PointXYZINormal | preprocess.h:12 | 点类型; `curvature` 字段被复用存相对帧起点时间戳 (ms), 逐点更新时序核心 |
| `state_input` (24维) | common_lib.h:24 | IMU 作输入模型: pos,rot,offset_R/T_L_I,vel,bg,ba,gravity |
| `state_output` (30维) | common_lib.h | IMU 作量测模型: 额外显式估计 omg/acc |
| `MeasureGroup` | common_lib.h:85 | lidar_beg_time, lidar, deque<Imu> imu |
| `esekf<...> kf_input/kf_output` | common_lib.h:40 | 两套 EKF (全局) |
| `IVoxType` | parameters.h:34 | `faster_lio::IVox<3,...>` 增量体素近邻地图 |

## 可复用性改造 (2026-07-11)
为使 point_lio 作为通用底层框架跨项目复用, 消除两处硬编码绑定 (默认值保持原行为, 完全向后兼容):

| 项 | 旧行为 | 新参数 (默认值) | 位置 |
|---|---|---|---|
| 世界系帧名 | 硬编码 `"camera_init"` | `publish.world_frame` ("camera_init") | parameters.cpp; laserMapping.cpp 全部 frame_id 引用改用 `world_frame` |
| 本体系帧名 | 硬编码 `"body"` | `publish.body_frame` ("body") | 同上, `body_frame` |
| odom 子帧名 | 硬编码 `"aft_mapped"` | `publish.aft_mapped_frame` ("aft_mapped") | TF broadcast, `aft_mapped_frame` |
| PCD 存盘目录 | 编译期 `ROOT_DIR + "PCD/"` (存进源码树, 只读环境崩溃) | `pcd_save.pcd_save_dir` ("" → 回退 ROOT_DIR) | 新增 `get_pcd_save_dir()` 辅助函数 (laserMapping.cpp:30), 两处存盘点统一调用 |

- 全局 extern 声明: parameters.h 新增 `world_frame`/`body_frame`/`aft_mapped_frame`/`pcd_save_dir`。
- ⚠️ 注意: 帧名参数化后, `loam_interface` 把 `camera_init`→`odom` 的桥接职责在多数场景下可简化甚至去掉 (直接让 point_lio 输出 `odom` 帧)。若做此简化需同步更新 `nav_bringup` launch 与 `odometry_bridge.md`。
- 未改动: 巨型 main、全局可变状态 (挡多实例)、量测模型重复 —— 属架构级重构, 见 Layer 3 #1-3, #8。

> 详细风险 (巨型 main、全局状态、量测模型重复、时序注释 `// big problem`、固定大小数组越界隐患等) 见 `PROJECT_DOC.md` Layer 3 注解 #1-3, #8, #28-31, #41。
