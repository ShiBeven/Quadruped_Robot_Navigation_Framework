# 模块: omni_pid_pursuit_controller

> 上级: [PROJECT_DOC.md](../PROJECT_DOC.md)
> 最后同步: 提交 `5b3f214` — 2026-07-05
> 层级分布: A: 2 / B: 0 / C: 3

## 职责
Nav2 控制器插件：带双 PID 回路（平移 + 航向）和曲率限速的带约束纯追踪算法，专为全向机器人设计。计算 `cmd_vel` 命令以跟踪全局路径，同时遵守动态速度限制和接近段速度缩放。

## 关键文件

| 文件 | 层级 | 职责 |
|---|---|---|
| `include/pb_omni_pid_pursuit_controller/omni_pid_pursuit_controller.hpp` | A | 控制器类：configure/activate/setPlan/computeVelocityCommands（769 行实现） |
| `include/pb_omni_pid_pursuit_controller/pid.hpp` | A | 独立 PID 控制器工具类 |
| `src/omni_pid_pursuit_controller.cpp` | C | 实现 |
| `src/pid.cpp` | C | PID 实现 |
| `pb_omni_pid_pursuit_controller.xml` | C | Pluginlib 注册 |

## 公共 API 参考

| 符号 | 签名 | 用途 |
|---|---|---|
| `OmniPidPursuitController::configure()` | `void configure(Node::SharedPtr, string, TF buffer, costmap)` | 生命周期：加载参数、创建发布器 |
| `OmniPidPursuitController::activate()` | `void activate()` | 生命周期：激活发布器 |
| `OmniPidPursuitController::deactivate()` | `void deactivate()` | 生命周期：停用发布器 |
| `OmniPidPursuitController::cleanup()` | `void cleanup()` | 生命周期：重置所有状态 |
| `OmniPidPursuitController::setPlan()` | `void setPlan(const Path&)` | 接收新全局路径；重置预瞄弧 |
| `OmniPidPursuitController::computeVelocityCommands()` | `TwistStamped computeVelocityCommands(pose, velocity, goal_checker)` | 主循环：寻找预瞄点、计算 PID 输出、施加曲率限制 |
| `OmniPidPursuitController::setSpeedLimit()` | `void setSpeedLimit(bool, double)` | 施加外部速度限制 |
| `PID::calculate()` | `double calculate(double set_point, double pv)` | 给定设定值和过程变量，返回 PID 操作变量 |
| `PID::setGains()` | `void setGains(double kp, double kd, double ki)` | 运行时增益更新 |

## 控制架构

```
全局路径 → findLookaheadPoint() → 计算平移误差 → PID(linear) → 速度命令
                                → 计算航向误差 → PID(angular) → 角速度命令
                                → 预瞄点曲率 → 速度降低因子

速度命令 = PID(linear) × 曲率因子 × 接近因子
角速度命令 = PID(angular)

最终输出: TwistStamped → velocity_smoother → cmd_vel
```

**双 PID 回路：**
- 平移 PID: 基于到预瞄点距离控制前进/后退速度
- 旋转 PID: 基于航向误差控制角速度（支持 `rotate_to_heading`）

**基于曲率的速度限制：**
- 从路径计算预瞄点处曲率
- 在高曲率段施加 `curvature_reduction_ratio`
- 通过 `min_curvature`/`max_curvature` 阈值配置

**接近段速度缩放：**
- 在 `approach_distance_threshold` 范围内按比例降低速度
- 从当前速度降至 `min_approach_linear_velocity`

## 关键参数

| 参数 | 硬件默认值 | 描述 |
|---|---|---|
| `kp_linear` / `ki_linear` / `kd_linear` | 1.5 / 0.05 / 0.1 | 平移 PID 增益 |
| `kp_angular` / `ki_angular` / `kd_angular` | 1.0 / 0.0 / 0.0 | 旋转 PID 增益 |
| `v_linear_min` / `v_linear_max` | -1.5 / 1.5 m/s | 线速度范围 |
| `v_angular_min` / `v_angular_max` | -1.5 / 1.5 rad/s | 角速度范围 |
| `lookahead_distance` / `min` / `max` | 1.0 / 0.3 / 1.5 m | 纯追踪预瞄距离 |
| `rotate_to_heading` | true | 转向预瞄点航向 |
| `min_curvature` / `max_curvature` | 1.0 / 3.0 | 曲率速度降低阈值 |
| `curvature_reduction_ratio` | 0.4 | 高曲率处速度倍率 |
| `transform_tolerance` | 0.5 s | TF 查找超时 |

**发布的话题:** `local_plan` (Path)、`lookahead_point` (PointStamped)、`curvature_points` (MarkerArray) — 仅用于可视化和调试。

## 调用关系

- **依赖于:** nav2_core (Controller 接口), nav2_costmap_2d, tf2_ros, nav_msgs
- **被依赖:** nav_bringup (YAML 配置: `controller_plugin: "omni_pid_pursuit_controller::OmniPidPursuitController"`), bt_navigator (由 FollowPath BT 动作调用)
