# 模块: nav2_plugins

> 同步: 2026-07-11 (非 git 仓库) | Tier 分布: A: 5 / B: 4 / C: 5 | 语言: C++
> 路径: `src/navigation/nav2_plugins/`

## 职责
为 Navigation2 提供扩展插件: 一个朝自由空间后退的恢复行为、一个带强度过滤的 3D 体素代价地图层、以及一整套决策/巡逻/导航行为树 (BT) 节点 (action/condition/control/decorator), 全部通过 pluginlib / BT 工厂**动态加载**进 nav2 的 BT 导航器与 costmap 管线。

> ⚠️ **注意**: 本框架 `nav2_params.legged.yaml` 实际启用的是 `IntensityVoxelLayer` (costmap) 与部分 BT 节点; behavior_server 用标准 Nav2 行为 (spin/backup/drive_on_heading/wait), **未启用本包的 `back_up_free_space`** (四足用 Spin 恢复)。

## 关键文件
| 文件 | Tier | 职责 |
|---|---|---|
| src/behaviors/back_up_free_space.cpp | A | 扇形扫描找最大自由角度区间, 朝自由空间后退 (继承 DriveOnHeading, 335 行) |
| src/layers/intensity_voxel_layer.cpp | A | 带强度过滤的 3D 体素障碍层, 从点云标记体素并投影为 2D 致命障碍 (210 行) |
| src/bt/action/send_nav_through_poses.cpp | A | 异步调用 NavigateThroughPoses, request_id+互斥锁管理并发目标 (200 行) |
| src/bt/action/select_patrol_path.cpp | A | 从固定路点按巡逻游标/方向生成往返式预览路径 (115 行) |
| src/bt/control/recovery_node.cpp | A | 二子节点恢复控制: 子0失败则跑子1恢复并重试 (107 行) |
| include/nav2_plugins/bt/nav_utils.hpp | A | 纯 inline 工具库: 黑板取 node、构造路点、巡逻状态机、路径等价/到达判断 (198 行) |
| src/bt/action/publish_nav_goal.cpp | B | 向 `goal_pose` 发布目标, 带去重与一次性补发 |
| src/bt/action/send_nav2_goal.cpp | B | ROS action 客户端 → NavigateToPose |
| src/bt/action/hold_stop_flag.cpp | B | 发布 `stop_flag`(Bool) 保持时长后释放 |
| src/bt/decorator/rate_controller.cpp | B | 按 hz 限制子节点 tick 频率 |
| src/bt/action/{pub_twist,pub_spin_speed,select_fixed_path,select_path_goal_pose}.cpp | C | 薄封装 BT 节点 |
| src/bt/condition/is_path_goal_reached.cpp | C | 判断当前位姿到路径终点距离在容差内 |
| include/nav2_plugins/bt/custom_types.hpp | B | `convertFromString<PoseStamped>` (7段完整位姿 / 3段 x;y;yaw) |

## 插件注册 (pluginlib — 反射/动态加载, 重点标注)
三种独立动态加载机制并存, 均运行时按字符串名反射实例化 (编译期无调用点):
- **nav2_core::Behavior** (behavior_plugin.xml): `pb_nav2_behaviors::BackUpFreeSpace` (back_up_free_space.cpp:335 `PLUGINLIB_EXPORT_CLASS`)
- **nav2_costmap_2d::Layer** (costmap_plugins.xml): `pb_nav2_costmap_2d::IntensityVoxelLayer` (intensity_voxel_layer.cpp:210)
- **BT 节点工厂注册** (每 .so 一注册入口): 普通节点 `BT_REGISTER_NODES` 注册 IsPathGoalReached/SelectFixedPath/SelectPatrolPath/HoldStopFlag/PublishNavGoal/SendNavThroughPoses/RecoveryNode/RateController; ROS 感知节点用 `CreateRosNodePlugin` 注册 SendNav2Goal/PublishTwist/PublishSpinSpeed。由 BT 引擎运行时 `registerFromPlugin` 加载。

## 公共 API 契约 (核心节点)
| 符号 | 端口 / 签名 | 用途 | 契约 / 不变量 |
|---|---|---|---|
| `SelectPatrolPathAction` | in `patrol_cursor`/`patrol_direction`; out `path`/`next_cursor`/`next_direction` | 生成往返巡逻路径 | 游标越界回 0, 方向 0 归 1; 用 `computeNextPatrolState` 预览 `max(2,preview_points)` 点。next_* 是预览末状态供回填黑板。patrol_indices 空→FAILURE。 |
| `SendNavThroughPosesAction` | in `path`; out `current_pose`/`goal_succeeded` | 异步多点导航 | 自持 rclcpp_action 客户端 + `mutex_`。回调用 `request_id != goal_request_id_` 丢弃过期响应; `cancelCurrentGoal` 先自增 request_id 使旧回调失效。SyncActionNode 却做长期异步, 完成判定隐式靠黑板 `goal_succeeded` 轮询 (非显然)。 |
| `PublishNavGoalAction` | in `goal`/`topic_name` | 发布导航目标 | `isSameGoal` 用位置容差+偏航 0.05rad 判重; 相同目标仅 `burst_republish_remaining_>0` 补发一次; 新目标发布后置 =1 (保证晚订阅者也收到)。 |
| `RecoveryNode` | in `num_attempts` (默认 999) | 恢复控制 | 必须恰好 2 子节点否则抛异常。子0 SUCCESS→整体 SUCCESS; 子0 FAILURE 未超重试→跑子1; 子1 SUCCESS→retry++回子0。子节点绝不返回 IDLE。nav2 原版移植。 |
| `RateController` | in `hz` (端口默认 10.0; cpp:12 局部回退值 1.0) | 限频装饰器 | 首次/子在 RUNNING/已过 period 才 tick 子; 子 SUCCESS 重置计时器。nav2 原版移植。 |
| `BackUpFreeSpace::findBestDirection` | — | 求最佳后退角 | `angle_increment=π/32` 从 -π 扫到 π, 沿半径采样 costmap, cell≥253 判不安全, 记最长连续安全区间返回中点。全阻塞报错返回 0 (与合法 0 角度无法区分, 潜在缺陷)。 |
| `IntensityVoxelLayer::updateBounds` | — | 更新代价层 | rolling 时重置 origin; 遍历点云按高度/强度区间 [min,max]/距离过滤, markVoxelInMap 成功则写 `LETHAL_OBSTACLE`。强度过滤是与标准 ObstacleLayer 的核心区别。 |

## 调用关系
- **依赖**: nav2_core、nav2_costmap_2d (ObstacleLayer/VoxelGrid)、nav2_behaviors (DriveOnHeading)、nav2_behavior_tree/behaviortree_cpp、behaviortree_ros2 (ROS 感知 BT 节点)、pluginlib、rclcpp/rclcpp_action、nav2_util、消息 (nav2_msgs/nav_msgs/geometry_msgs)、tf2。
- **被依赖**: nav_bringup 的 nav2 配置 (costmap 层引用 `IntensityVoxelLayer`; bt_navigator 加载引用这些 BT 节点的 XML)。

## 关键类型 / 黑板键
- `convertFromString<PoseStamped>` (custom_types.hpp): `;` 分隔, 7段=完整位姿, 3段=(x,y,yaw)。
- 核心黑板键 (约定命名): `node` (根黑板 ROS 句柄, 所有非 Ros 派生节点依赖 `getNodeFromBlackboard`)、`{decision_path}`、`{decision_goal_pose}`、`{decision_current_pose}`、`{decision_nav_goal_succeeded}`、`{decision_patrol_cursor}`/`{decision_patrol_direction}`。
- `computeNextPatrolState(cursor,direction,count)→pair<int,int>` (nav_utils.hpp:144): 往返状态机。

> 详细风险 (pluginlib 动态加载、findBestDirection 边界缺陷、gatherFreePoints 死代码、参数返回值忽略、send_nav_through_poses 并发) 见 `PROJECT_DOC.md` Layer 3 注解 #13, #32, #34。
