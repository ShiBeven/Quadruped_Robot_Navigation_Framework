# 模块: terrain_analysis (×2: terrain_analysis + terrain_analysis_ext)

> 上级: [PROJECT_DOC.md](../PROJECT_DOC.md)
> 最后同步: 提交 `5b3f214` — 2026-07-05
> 层级分布: A: 0 / B: 2 / C: 2

## 职责
感知管线：将配准后的激光雷达扫描转换为滚动网格地形高度图，分类地面与障碍物，并可选择过滤动态障碍物。`terrain_analysis` 生成主要的 `terrain_map`；`terrain_analysis_ext` 通过连通性过滤（BFS 洪泛）和局部地形合并扩展输出。

## 关键文件

| 文件 | 层级 | 职责 |
|---|---|---|
| `terrain_analysis/src/terrainAnalysis.cpp` | B | 主地形节点：滚动网格、地面估计、动态障碍物过滤（682 行） |
| `terrain_analysis_ext/src/terrainAnalysisExt.cpp` | B | 扩展地形节点：连通性 BFS、天花板去除、局部地图合并（557 行） |

> 两者都是过程式 `main()` 节点 — 无类定义。

## ROS2 接口

### terrain_analysis

| 方向 | 话题 | 类型 | 用途 |
|---|---|---|---|
| **订阅** | `registered_scan` | `sensor_msgs::PointCloud2` | 地形累积用激光数据 |
| **订阅** | `lidar_odometry` | `nav_msgs::Odometry` | 机器人 6 自由度位姿 |
| **订阅** | `joy` | `sensor_msgs::Joy` | 按钮 5：清除地形地图 |
| **订阅** | `map_clearing` | `std_msgs::Float32` | 清除机器人周围半径 |
| **发布** | `terrain_map` | `sensor_msgs::PointCloud2` | 标注点云（Z=高程, intensity=离地高度），100 Hz，odom 坐标系 |

### terrain_analysis_ext

| 方向 | 话题 | 类型 | 用途 |
|---|---|---|---|
| **订阅** | `registered_scan` | `sensor_msgs::PointCloud2` | 激光数据 |
| **订阅** | `lidar_odometry` | `nav_msgs::Odometry` | 机器人位姿 |
| **订阅** | `terrain_map` | `sensor_msgs::PointCloud2` | 主地形（在 `localTerrainMapRadius` 内作为局部高分辨率源） |
| **订阅** | `joy` | `sensor_msgs::Joy` | 按钮 5：清除 |
| **订阅** | `cloud_clearing` | `std_msgs::Float32` | 清除半径 |
| **发布** | `terrain_map_ext` | `sensor_msgs::PointCloud2` | 扩展地形（100 Hz，odom 坐标系） |

## 算法概述

### terrain_analysis 管线

```
激光扫描 + 里程计
  → 滚动地形体素网格（1.0m 单元，21×21 网格）
  → 平面体素网格（0.2m 单元，51×51 网格）
  → 逐单元分位数地面高度估计
  → 过期点的时域衰减
  → 可选：动态障碍物检测（载体坐标系角度阈值）
  → 可选：盲区障碍物插入（未见区域标记为障碍物）
  → 发布 terrain_map
```

### terrain_analysis_ext 扩展

```
与 terrain_analysis 相同管线，此外：
  → 从机器人单元通过地形体素 BFS 洪泛
  → 移除在 terrainConnThre 高度差内不可达的单元
  → 移除天花板/悬垂点（高度超过可达最大高度的 ceilingFilteringThre）
  → 在 localTerrainMapRadius 内：合并来自 terrain_map 的高分辨率点
  → 发布 terrain_map_ext
```

## 关键参数

| 参数 | terrain_analysis | terrain_analysis_ext | 描述 |
|---|---|---|---|
| `vehicleHeight` | 0.6 m | 1.0 m | 机器人离地高度 |
| `terrainVoxelSize` | 1.0 m | 2.0 m | 地形网格单元尺寸 |
| `planarVoxelSize` | 0.2 m | 0.4 m | 地面估计单元尺寸 |
| `terrainGridSize` | 21×21 | 41×41 | 滚动网格尺寸 |
| `planarGridSize` | 51×51 | 101×101 | 地面网格尺寸 |
| `scanVoxelSize` | 0.02 | (继承) | 点降采样率 |
| `clearingDis` | 15.0 m | 20.0 m | 最大地形范围 |
| `localTerrainMapRadius` | — | 4.0 m | 高分辨率合并区半径 |
| `terrainConnThre` | — | 0.1 m | BFS 连通性阈值 |
| `ceilingFilteringThre` | — | 0.2 m | 天花板点移除 |

## 调用关系

- **依赖于:** PCL, ROS2 (sensor_msgs, nav_msgs, std_msgs), Eigen
- **被依赖:**
  - 两者: nav_bringup (启动文件启动两个节点)
  - `terrain_analysis`: terrain_analysis_ext (订阅 `terrain_map`)
  - `terrain_analysis_ext` → pointcloud_to_laserscan (重映射 `terrain_map_ext` → `obstacle_scan`)
  - `terrain_map` / `terrain_map_ext` → nav2_plugins IntensityVoxelLayer (local_costmap / global_costmap 观测源)
