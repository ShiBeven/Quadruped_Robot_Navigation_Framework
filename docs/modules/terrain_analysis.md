# 模块: terrain_analysis / terrain_analysis_ext

> 同步: 2026-07-11 (非 git 仓库) | Tier 分布: 各 A: 1 (单文件节点) / C: 配置 | 语言: C++
> 路径: `src/navigation/terrain_analysis/` 与 `terrain_analysis_ext/`
> 血统: CMU 自主探索 `terrain_analysis` 系列 (maintainer Ji Zhang)。两节点级联使用。
> 变更 (2026-07-11): 两节点输出帧名由硬编码 `"odom"` 改为 `mapFrame` 参数 (默认 "odom", 向后兼容)。

## 职责
- **terrain_analysis**: 单文件 ROS2 节点, 将配准点云按滚动体素网格累积, 估计各平面体素地面高程, 输出每点相对地面抬升 (intensity=可通行代价) 的 `terrain_map` 点云; 含动态障碍剔除与无数据补障。
- **terrain_analysis_ext**: 大尺度扩展版, 用更大体素网格 (2m×41×41) 累积远距地形, 基于 BFS 地形连通性剔除天花板, 并在 `localTerrainMapRadius` 内融合局部 `terrain_map`, 输出 `terrain_map_ext`。

## 关键文件
| 文件 | Tier | 职责 |
|---|---|---|
| terrain_analysis/src/terrainAnalysis.cpp | A | 单文件节点: 全局状态 + 4 回调 + main 内巨型处理循环 (682 行) |
| terrain_analysis_ext/src/terrainAnalysisExt.cpp | A | 单文件节点: 大尺度体素累积 + BFS 连通性天花板过滤 + 局部图融合 (557 行) |
| */launch/*.launch | C | 启动并覆盖参数 (以 launch 值为准) |

## ROS2 接口

### terrain_analysis (节点名 `terrainAnalysis`)
- **订阅**: `lidar_odometry`(Odometry)→odometryHandler; `registered_scan`(PointCloud2)→laserCloudHandler; `joy`(Joy, 按钮[5]清云); `map_clearing`(Float32)
- **发布**: `terrain_map`(PointCloud2, frame_id=`mapFrame` 参数, 默认 "odom", intensity=相对地面高度)
- **关键参数** (默认/launch 覆盖): `scanVoxelSize`(0.05)、`decayTime`(2.0/1.0)、`noDecayDis`(4.0/1.75)、`quantileZ`(0.25)、`clearDyObs`(false/**true**)、`minDyObsDis`(0.3)、`minDyObsRelZ`(-0.5/-0.3)、`maxRelZ`(0.2/**0.3**)、`vehicleHeight`(1.5)、`minBlockPointNum`(10)、`noDataObstacle`(false)

### terrain_analysis_ext (节点名 `terrainAnalysisExt`)
- **订阅**: `lidar_odometry`(Odometry); `registered_scan`(PointCloud2); `joy`(Joy); `cloud_clearing`(Float32, 注意名与上不同); `terrain_map`(PointCloud2)→terrainCloudLocalHandler (消费 analysis 输出作局部图)
- **发布**: `terrain_map_ext`(PointCloud2, frame_id=`mapFrame` 参数, 默认 "odom")
- **关键参数**: `scanVoxelSize`(0.1)、`decayTime`(10.0/4.0)、`useSorting`(false/**true**)、`quantileZ`(0.25/0.1)、`checkTerrainConn`(cpp true/**launch arg false**)、`terrainConnThre`(0.5)、`ceilingFilteringThre`(2.0)、`terrainUnderVehicle`(-0.75)、`localTerrainMapRadius`(4.0)、`lowerBoundZ`(-1.5/-2.5)、`upperBoundZ`(1.0)

## 核心处理管线 (无类, 全局函数 + 全局变量)

### terrain_analysis (main cpp:263-679, 仅 newlaserCloud 时执行)
1. **地形体素滚动** (268-336): 车移动超一个 `terrainVoxelSize` 时 21×21 网格整体移位, 清空新入边行/列
2. **累积配准点** (338-363): 点入 `terrainVoxelCloud[indX*W+indY]`, 计数++
3. **体素降采样与时间/数量衰减** (365-396): 达阈值时 VoxelGrid 下采样, 按 decayTime/noDecayDis/clearingDis 过滤旧点
4. **聚合中心 11×11 体素** 到 terrainCloud (399-405)
5. **地面估计** (408-559): 点散布到 3×3 平面体素; `clearDyObs` 分支旋转到车体系按 VFOV/角度判动态障碍; `useSorting` 用 `quantileZ` 分位数作地面高程否则取最小 Z
6. **计算高程/可通行代价** (561-601): `disZ=point.z-groundElev`, 满足 `0<=disZ<vehicleHeight` 且体素点数≥minBlockPointNum 则入 terrainCloudElev (intensity=disZ)
7. **无数据补障** (603-663): `noDataObstacle && noDataInited==2` 时 BFS 式边缘扩散把无点体素标障碍
8. **发布** terrain_map (668-673)

### terrain_analysis_ext (main cpp:229-554)
1-3. 体素滚动 (41×41 @2m) + 累积 + 下采样衰减 (同上逻辑)
4. 聚合中心 21×21 体素
5. 地面估计 (无 clearDyObs/limitGroundLift 分支)
6. **地形连通性天花板过滤** (450-486, `checkTerrainConn`): 从车体中心体素 BFS (std::queue), 相邻高程差 <terrainConnThre(0.5) 判连通(conn=2), >ceilingFilteringThre(2.0) 判天花板(conn=-1); 空体素初始化为 `vehicleZ+terrainUnderVehicle`。**该文件独有算法**
7. **远处地形代价** (489-528): 仅 dis>localTerrainMapRadius 的点计代价, 要求 conn==2
8. **融合局部图** (530-539): terrainCloudLocal 中 dis<=localTerrainMapRadius 的点直接并入
9. **发布** terrain_map_ext

## 核心数据流
```
registered_scan + lidar_odometry
  → [analysis] 裁剪 → 滚动 terrainVoxelCloud[21×21] → VoxelGrid下采样+衰减
      → 中心 terrainCloud → 平面体素(51×51)地面分位数估计 (+动态障碍剔除)
      → 逐点 disZ 代价 (+无数据补障) → terrain_map (intensity=可通行代价)
  → [ext] 滚动[41×41 @2m] → 中心terrainCloud → 平面体素(101×101 @0.4m)地面估计
      → BFS 连通性剔天花板 → 远距(>4m)代价点 + 融合近距(<=4m) terrain_map → terrain_map_ext
```
> 级联关系: analysis 供精细近场 (terrain_map), ext 供扩展远场 (terrain_map_ext); 两者通过话题松耦合无握手。均订阅共享 `lidar_odometry`+`registered_scan`, 均输出 `odom` 系。

## 关键类型 / 参数
- analysis: terrain voxel `1.0m×21×21` (±10m); planar voxel `0.2m×51×51`; disZ 有效区间 [0,1.5)。
- ext: terrain voxel `2.0m×41×41` (±40m); planar voxel `0.4m×101×101`; 连通阈值 0.5/2.0m。
- 点类型 `pcl::PointXYZI`, intensity 字段**复用** (先存时间戳差, 最终存 disZ 代价, 语义隐晦)。

## 调用关系
- **依赖**: rclcpp、sensor_msgs/nav_msgs/std_msgs/geometry_msgs、pcl_ros/pcl_conversions (VoxelGrid)、tf2 (四元数→RPY)。ext 额外 include kdtree (**声明但未使用, 死代码**)。未用 Eigen/pluginlib。
- **被依赖**: nav2 costmap 的 `intensity_voxel_layer` (local 用 terrain_map, global 用 terrain_map_ext); SLAM 时 pointcloud_to_laserscan 消费 terrain_map_ext。上游: sensor_scan_generation/loam_interface (lidar_odometry, registered_scan)。

## 可复用性改造 (2026-07-11)
- ✅ **输出帧参数化**: 新增 `mapFrame` 参数 (默认 "odom"), 两节点发布点云时用之替代硬编码 `"odom"` (terrainAnalysis.cpp / terrainAnalysisExt.cpp 的 publish 处)。多机器人 namespace 或非标准 odom 命名下可正确对接 costmap。
- 未改动: 巨型 main、全局可变状态 (挡多实例, 且回调↔main 靠全局标志通信依赖单线程 spin_some)、intensity 语义复用 —— 属架构级重构。

> 详细风险 (巨型 main 6-7 层嵌套、40+ 全局变量无同步、手柄越界、intensity 语义复用、BFS 性能、参数默认与 launch 冲突) 见 `PROJECT_DOC.md` Layer 3 注解 #4-5, #9, #27, #37。
