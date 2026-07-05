# 风险地图

> [返回 PROJECT_DOC.md](../PROJECT_DOC.md)
> 以下标注基于代码的结构特征。
> 它们不声称"这是一个 bug"。每一项都需要人工确认。
> 置信度等级: **高** = 可量化事实；**中** = 需要人工审查的模式匹配；
> **低** = 大概率无害但为完整性而标记。

## 概览

| 类别 | 数量 |
|---|---|
| 复杂度热点 | 4 |
| 高耦合 | 1 |
| 隐式依赖 | 2 |
| 异常处理缺口 | 3 |
| 意图不明 | 2 |
| 硬编码密钥 | 0 |

---

### 复杂度热点（行数 >300 / 嵌套深度 >4 / 函数参数 >5）

#### 标注 #1
| 字段 | 详情 |
|---|---|
| 文件 | [src/navigation/point_lio/src/laserMapping.cpp](../modules/point_lio.md) |
| 类型 | 复杂度热点 — 1052 行；`main` 循环编排 ESKF 预测-更新-发布周期，嵌套层次深 |
| 置信度 | **高**（可量化的结构指标） |
| 覆盖率 | 无覆盖率数据 |
| 影响 | 核心 SLAM 管线。所有状态估计、地图更新和发布集中在一个函数中。修改风险极高 — 循环逻辑的任何更改都可能破坏整个里程计管线。 |
| 建议 | 将子步骤（ESKF 迭代、地图增量更新、发布）提取为具名函数以降低认知负荷。为 SLAM 管线添加集成级特征测试。 |

#### 标注 #2
| 字段 | 详情 |
|---|---|
| 文件 | [src/navigation/point_lio/src/preprocess.cpp](../modules/point_lio.md) |
| 类型 | 复杂度热点 — 935 行；四个激光雷达处理程序 + `give_feature()` 逐环分类，含多个边界情况分支 |
| 置信度 | **高**（可量化的结构指标） |
| 覆盖率 | 无覆盖率数据 |
| 影响 | 支持新激光雷达型号需要阅读全部四个处理程序以理解转换模式。`give_feature()` 有多个分类阈值（`edgea/edgeb`、`smallp_intersect`、`plane_judge`）非平凡地相互作用。 |
| 建议 | 文档化特征提取参数的含义。各处理程序应可提取到各自的编译单元以提高可读性。 |

#### 标注 #3
| 字段 | 详情 |
|---|---|
| 文件 | [src/navigation/omni_pid_pursuit_controller/src/omni_pid_pursuit_controller.cpp](../modules/omni_pid_pursuit_controller.md) |
| 类型 | 复杂度热点 — 769 行；`computeVelocityCommands()` 在单个方法中处理预瞄点查找、双 PID、曲率限制、接近缩放和碰撞检查 |
| 置信度 | **高**（可量化的结构指标） |
| 覆盖率 | 无覆盖率数据 |
| 影响 | 控制器是规划与执行之间的关键路径。此处的逻辑错误直接影响机器人运动安全。 |
| 建议 | 将 `computeVelocityCommands()` 分解为：(1) 预瞄点选择、(2) 误差计算、(3) PID 输出、(4) 曲率限制、(5) 接近缩放。每一步可独立测试。 |

#### 标注 #4
| 字段 | 详情 |
|---|---|
| 文件 | [src/navigation/terrain_analysis/src/terrainAnalysis.cpp](../modules/terrain_analysis.md) |
| 类型 | 复杂度热点 — 682 行；单个 `main()` 内联包含所有 ROS2 回调、网格管理、地面估计、动态障碍物过滤 |
| 置信度 | **高**（可量化的结构指标） |
| 覆盖率 | 无覆盖率数据 |
| 影响 | 无类封装 — 所有状态为全局变量。添加第二个地形分析变体（terrain_analysis_ext）需要复制整个文件。缺陷修复必须在两个地方分别应用。 |
| 建议 | 提取一个由两个节点共享的 `TerrainAnalysis` 类。terrain_analysis 与 terrain_analysis_ext（557 行）之间的重复逻辑是维护上的负债。 |

---

### 高耦合（被 ≥10 个模块引用）

#### 标注 #5
| 字段 | 详情 |
|---|---|
| 文件 | [src/navigation/point_lio/](../modules/point_lio.md) |
| 类型 | 高耦合 — `aft_mapped_to_init`（里程计）和 `cloud_registered`（点云）被 terrain_analysis、terrain_analysis_ext、loam_interface、small_gicp_relocalization 和 sensor_scan_generation 消费 |
| 置信度 | **高**（统计性的 — 5 个下游消费者，全为关键路径） |
| 影响 | 里程计消息格式、坐标帧或发布频率在 point_lio 中的任何更改都会级联影响整个感知和定位栈。 |
| 建议 | 将里程计消息契约（坐标帧、频率、协方差语义）文档化为模块级接口规范。修改 point_lio 输出话题时运行集成冒烟测试。 |

---

### 隐式依赖（全局变量 / 模块级可变状态）

#### 标注 #6
| 字段 | 详情 |
|---|---|
| 文件 | [src/navigation/point_lio/src/parameters.h](../modules/point_lio.md) |
| 类型 | 隐式依赖 — ~100+ 个全局 `extern` 变量（所有配置、状态和共享指针）在模块的每个编译单元中均可访问 |
| 置信度 | **中**（此模式在 ROS2 节点中是惯例，但增加了耦合风险） |
| 建议 | 考虑迁移到 `Config` 结构体或 ROS2 参数客户端模式。至少应文档化哪些全局变量在初始化后只读，哪些在运行时被修改。 |

#### 标注 #7
| 字段 | 详情 |
|---|---|
| 文件 | [src/navigation/point_lio/src/laserMapping.cpp](../modules/point_lio.md) `laserMapping.cpp` + `li_initialization.cpp` |
| 类型 | 隐式依赖 — `lidar_buffer`、`time_buffer`、`imu_deque` 是由单个 `mtx_buffer` 保护的全局双端队列，被 3 个回调和主循环访问 |
| 置信度 | **中**（生产者-消费者队列 + 单个互斥锁 — 正确但修改时脆弱） |
| 建议 | 替换为适当的线程安全队列（如 `readerwriterqueue`）。当前设计可行，但添加新的传感器订阅者需要理解跨 4 个编译单元的互斥锁规则。 |

---

### 异常处理缺口

#### 层级：高置信度（确定性的缺口）

#### 标注 #8
| 字段 | 详情 |
|---|---|
| 文件 | [src/navigation/nav2_plugins/src/bt/action/send_nav_through_poses.cpp](../modules/nav2_plugins.md) |
| 类型 | 异常处理缺口 — 动作服务器调用（`async_send_goal`、`async_cancel_goal`）使用原始 rclcpp_action 客户端，无 try-catch 或错误处理程序链 |
| 置信度 | **高**（ROS2 动作客户端操作在传输/网络故障时可能抛出异常） |
| 建议 | 注册结果回调、反馈回调和目标响应回调。将 `async_send_goal` 包装在 try-catch 中，向行为树返回有意义的错误信息。 |

#### 层级：中等置信度（需要人工审查）

#### 标注 #9
| 字段 | 详情 |
|---|---|
| 文件 | [src/navigation/small_gicp_relocalization/src/small_gicp_relocalization.cpp](../modules/localization_and_perception.md) |
| 类型 | 异常处理缺口 — PCD 文件加载和 GICP 配准可能失败（空地图、退化几何体），但头文件中未见显式错误处理 |
| 置信度 | **中**（实现未深度阅读；头文件显示配准回调无错误码返回） |
| 建议 | 审查 PCD 加载失败和配准收敛失败路径的实现。如果配准在失败时静默产生恒等变换，机器人将累积无界漂移。 |

#### 层级：低置信度（大概率无害）

#### 标注 #10
| 字段 | 详情 |
|---|---|
| 文件 | [src/navigation/point_lio/src/laserMapping.cpp](../modules/point_lio.md) `SigHandle()` 信号处理器 |
| 类型 | 异常处理缺口 — 信号处理器仅设置 `flg_exit = true`，不刷新调试日志文件 |
| 置信度 | **低**（调试日志完整性对运行非关键；`ofstream` 析构函数在正常退出时刷新） |
| 建议 | 验证 `SIGINT` 触发的退出是否正确销毁 `fout_out` 和 `fout_imu_pbp` ofstream 对象。如未，在信号处理器中添加显式 `flush()` 和 `close()`。 |

---

### 意图不明（LLM 无法确定意图的代码）

#### 标注 #11
| 字段 | 详情 |
|---|---|
| 文件 | [src/navigation/point_lio/src/preprocess.cpp](../modules/point_lio.md) `give_feature()` — 多个魔法阈值 |
| 类型 | 意图不明 — 特征分类使用硬编码常量（`cos160`、`edgea=2`、`edgeb=0.1`、`smallp_intersect=1.0`、`smallp_ratio=1.0`），没有文档说明其几何含义 |
| 置信度 | **高**（无注释解释阈值的物理意义） |
| 建议 | 对 `give_feature()` 运行 `git blame` 并咨询原始 Point-LIO 作者。文档化每个参数的几何解释。这些值很可能是经验调优的结果；不理解激光雷达光束模型的情况下更改它们将降低特征质量。 |

#### 标注 #12
| 字段 | 详情 |
|---|---|
| 文件 | [src/navigation/point_lio/src/laserMapping.cpp](../modules/point_lio.md) `use_imu_as_input` 标志 |
| 类型 | 意图不明 — 两个并行的 ESKF 变体（`kf_input` 24D 与 `kf_output` 30D），在主循环中代码路径几乎相同。`if (!use_imu_as_input)` 条件重复了约 200 行代码。 |
| 置信度 | **中**（意图 — 支持不同传感器配置 — 是清晰的，但重复模式暗示逐步演化而未重构） |
| 建议 | 将公共 ESKF 迭代提取为参数化于滤波器类型的模板。当前的重复使得难以判断哪个代码路径是"主要"的，哪个是实验变体。 |

---

### 硬编码密钥

代码库中未检测到硬编码的 API 密钥、令牌、密码或证书。
