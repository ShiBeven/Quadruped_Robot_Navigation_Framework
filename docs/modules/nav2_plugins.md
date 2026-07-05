# 模块: nav2_plugins

> 上级: [PROJECT_DOC.md](../PROJECT_DOC.md)
> 最后同步: 提交 `5b3f214` — 2026-07-05
> 层级分布: A: 5 / B: 8 / C: 5

## 职责
自定义 Nav2 扩展：用于高层决策的行为树动作/条件/控制节点、基于 3D 强度过滤的体素代价地图图层，以及寻找自由空间的增强后退行为。作为 ROS2 Nav2 与四足机器人特有导航逻辑之间的粘合剂。

## 关键文件

| 文件 | 层级 | 职责 |
|---|---|---|
| `include/nav2_plugins/bt/action/send_nav_through_poses.hpp` | A | 路径点序列动作客户端，带去重和反馈转发 |
| `include/nav2_plugins/bt/action/send_nav2_goal.hpp` | A | 单点 NavigateToPose 动作封装 |
| `include/nav2_plugins/bt/action/select_patrol_path.hpp` | A | 构建带反弹逻辑的巡逻预览路径 |
| `include/nav2_plugins/bt/control/recovery_node.hpp` | A | 可配置重试次数 RecoveryNode（主行为 → 恢复 → 重试） |
| `include/pb_nav2_plugins/layers/intensity_voxel_layer.hpp` | A | 带强度过滤的 3D 体素层 |
| `include/nav2_plugins/bt/action/hold_stop_flag.hpp` | B | 在航点处按停留时长发布 stop_flag Bool |
| `include/nav2_plugins/bt/action/pub_twist.hpp` | B | 从 BT 输入端口发布原始 Twist |
| `include/nav2_plugins/bt/action/select_fixed_path.hpp` | B | 从目标点索引构建单点路径 |
| `include/nav2_plugins/bt/action/pub_spin_speed.hpp` | B | 发布纯旋转速度 |
| `include/nav2_plugins/bt/action/publish_nav_goal.hpp` | B | 在话题上发布 Nav2 目标位姿 |
| `include/nav2_plugins/bt/condition/is_path_goal_reached.hpp` | B | 检查当前位姿是否在路径目标容差内 |
| `include/nav2_plugins/bt/decorator/rate_controller.hpp` | B | 按频率节流子节点执行 |
| `include/nav2_plugins/bt/custom_types.hpp` | B | PoseStamped 字符串反序列化 |
| `include/pb_nav2_plugins/behaviors/back_up_free_space.hpp` | B | 感知代价地图的后退：扫描寻找最优自由方向 |
| `include/nav2_plugins/bt/nav_utils.hpp` | B | 共享工具：路径构建、距离、巡逻状态、黑板访问 |
| `src/*.cpp` (全部) | C | 实现文件 |

## 公共 API 参考

### BT 动作节点

| 符号 | 父类 | 用途 |
|---|---|---|
| `SendNavThroughPosesAction` | `BT::SyncActionNode` | 将 Path 发送到 NavigateThroughPoses 动作服务器；对相同路径去重 |
| `SendNav2GoalAction` | `BT::RosActionNode<NavigateToPose>` | 将单个 PoseStamped 目标发送到 NavigateToPose |
| `SelectPatrolPathAction` | `BT::SyncActionNode` | 沿巡逻路线构建 N 点预览路径（带反弹） |
| `SelectFixedPathAction` | `BT::SyncActionNode` | 从目标点数组索引构建单点路径 |
| `HoldStopFlagAction` | `BT::StatefulActionNode` | 发布 `true` 持续指定秒数，然后释放 `false` |
| `PublishTwistAction` | `BT::RosTopicPubStatefulActionNode<Twist>` | 从 BT 端口发布 Twist (v_x, v_y, v_yaw)，停止时归零 |
| `PublishSpinSpeedAction` | `BT::RosTopicPubStatefulActionNode<Twist>` | 发布纯旋转 Twist |
| `PublishNavGoalAction` | `BT::SyncActionNode` | 在可配置话题上发布 PoseStamped |

### BT 控制/条件/装饰器节点

| 符号 | 父类 | 用途 |
|---|---|---|
| `RecoveryNode` | `BT::ControlNode` | 运行主节点；失败时运行恢复节点，最多重试 N 次 |
| `IsPathGoalReached` | `BT::ConditionNode` | 当前位姿在路径最终目标的容差内返回 SUCCESS |
| `RateController` | `BT::DecoratorNode` | 以最高指定频率（Hz）执行子节点 |

### 代价地图图层

| 符号 | 父类 | 用途 |
|---|---|---|
| `IntensityVoxelLayer` | `nav2_costmap_2d::ObstacleLayer` | 仅当激光强度在 [min, max] 内时标记障碍物，使用 3D 体素网格 |

### 行为插件

| 符号 | 父类 | 用途 |
|---|---|---|
| `BackUpFreeSpace` | `nav2_behaviors::DriveOnHeading<BackUp>` | 分析代价地图寻找最优自由方向，朝该方向后退 |

## 黑板键

| 键 | 类型 | 生产者 | 消费者 | 用途 |
|---|---|---|---|---|
| `{decision_path}` | `nav_msgs::msg::Path` | SelectPatrolPath, SelectFixedPath | SendNavThroughPoses | 当前导航路径 |
| `{decision_current_pose}` | `PoseStamped` | SendNavThroughPoses | IsPathGoalReached | 最新动作反馈位姿 |
| `{decision_nav_goal_succeeded}` | `bool` | SendNavThroughPoses | (下游 BT 节点) | 路径是否已完成 |
| `{decision_patrol_cursor}` | `int` | (持久化) | SelectPatrolPath | 当前巡逻索引 |
| `{decision_patrol_direction}` | `int` | (持久化) | SelectPatrolPath | 1 前进 / -1 后退 |
| `node` | `rclcpp::Node::SharedPtr` | (根黑板) | 所有节点通过 `getNodeFromBlackboard` | ROS2 节点句柄 |

## 插件注册

- `behavior_plugin.xml`: 将 `BackUpFreeSpace` 注册为 `nav2_core::Behavior`（插件名: `pb_nav2_behaviors/BackUpFreeSpace`）
- `costmap_plugins.xml`: 将 `IntensityVoxelLayer` 注册为 `nav2_costmap_2d::Layer`（插件名: `pb_nav2_costmap_2d::IntensityVoxelLayer`）
- BT 节点通过在各 `*.cpp` 文件中使用 `BT_REGISTER_NODES` 宏注册，命名空间 `nav2_plugins`

## 调用关系

- **依赖于:** nav2_core, nav2_behaviors, nav2_costmap_2d, nav2_msgs, nav_msgs, BehaviorTree.CPP, tf2, PCL
- **被依赖:** nav_bringup (BT XML 使用所有注册的 BT 节点; YAML 配置使用 IntensityVoxelLayer)
