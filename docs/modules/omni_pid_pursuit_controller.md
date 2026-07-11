# 模块: omni_pid_pursuit_controller

> 同步: 2026-07-11 (非 git 仓库) | Tier 分布: A: 2 (cpp+hpp 核心) / C: 2 (配置) | 语言: C++
> 路径: `src/navigation/omni_pid_pursuit_controller/`
> 变更 (2026-07-11): 修复 PID 积分抗饱和 —— 钳位改为使用前生效, 且限幅接入 `min_max_sum_error` 参数 (原硬编码 ±1)。

## 职责
通过 pluginlib 动态加载的 `nav2_core::Controller` 插件: 用纯追踪 (lookahead carrot) 选目标点, 配合两路 PID (平移/朝向) 输出**全向** `Twist` 速度, 并叠加曲率限速与接近减速。适配四足全向底盘。

## 关键文件
| 文件 | Tier | 职责 |
|---|---|---|
| src/omni_pid_pursuit_controller.cpp | A | 控制器全部逻辑: 生命周期、速度计算、纯追踪、曲率/接近限速、碰撞检测、动态参数 (770 行) |
| include/pb_omni_pid_pursuit_controller/omni_pid_pursuit_controller.hpp | A | 类声明、成员参数、方法契约 (含 doxygen, 305 行) |
| src/pid.cpp | A | 通用位置式 PID (P/I/D + 积分钳位 + 输出饱和, 50 行) |
| include/pb_omni_pid_pursuit_controller/pid.hpp | A | PID 类声明 (含内联 setGains) |
| pb_omni_pid_pursuit_controller.xml | C | pluginlib 清单: `OmniPidPursuitController` → `nav2_core::Controller` |
| package.xml | C | 依赖 + `<nav2_core plugin>` 注册 |

## ROS2 接口
- **非独立节点**, 被 nav2 controller_server 的 LifecycleNode 宿主。⚠️ **动态加载**: `PLUGINLIB_EXPORT_CLASS(...,nav2_core::Controller)` (cpp:768), 由 pluginlib 按 `FollowPath.plugin` 参数字符串运行时加载, 无静态调用点。
- **发布** (宿主命名空间下):
  - `local_plan` (nav_msgs/Path) — 变换到 base 系的局部路径
  - `lookahead_point` (geometry_msgs/PointStamped) — carrot 可视化
  - `curvature_points_marker_array` (visualization_msgs/MarkerArray)
- **订阅**: 无显式订阅; 经 nav2 接口 `setPlan()` 收全局路径, `computeVelocityCommands()` 收 pose/velocity。
- **关键参数** (前缀 `<plugin_name>.`, 默认值 cpp:42-102, 全支持运行时动态更新):
  - PID: `translation_kp/ki/kd`(3.0/0.1/0.3)、`rotation_kp/ki/kd`(3.0/0.1/0.3, 实机 legged.yaml 调低 translation_kp=1.5)
  - lookahead: `lookahead_dist`(0.3)、`use_velocity_scaled_lookahead_dist`(true)、`min/max_lookahead_dist`(0.2/1.0)、`lookahead_time`(1.0)、`use_interpolation`(true)
  - 转向: `use_rotate_to_heading`(true)、`use_rotate_to_heading_treshold`(0.1, 注意拼写)
  - 限速: `v_linear_min/max`(-3/3)、`v_angular_min/max`(-3/3)、`curvature_min/max`(0.4/0.7)、`reduction_ratio_at_high_curvature`(0.5)、`approach_velocity_scaling_dist`(0.6)、`min_approach_linear_velocity`(0.05)

## 公共 API 契约
| 符号 | 签名 | 用途 | 契约 / 不变量 |
|---|---|---|---|
| `configure` | `(parent,name,tf,costmap_ros)` cpp:21 | 生命周期配置 | `parent.lock()` 失败抛 PlannerException; 声明读全部参数, 构造两个 PID; `control_duration_=1/control_frequency`。 |
| `computeVelocityCommands` | `(pose,velocity,goal_checker) → TwistStamped` cpp:209 | 主控制循环 | 持 mutex+costmap 锁 → transformGlobalPlan(剪枝) → getLookAheadPoint → 双 PID → 曲率限速 → 接近减速 → 抽样 10 点碰撞检测 (命中**抛异常停车** cpp:264) → `linear.x=v*cos(θ)`,`linear.y=v*sin(θ)` 全向拆分 + angular.z。`goal_checker` 未使用。 |
| `setPlan` | `(const Path&)` cpp:270 | 存全局路径 | 仅存 `global_plan_`; 必须先于 computeVelocityCommands 否则 transformGlobalPlan 抛 "zero length"。 |
| `getLookAheadPoint` | `(dist,plan)` cpp:353 | 取 carrot 点 | 找第一个距原点 ≥dist 的位姿, 找不到取末点; `use_interpolation` 时用 `circleSegmentIntersection` 求精确交点。依赖 prev 在圆内/goal 在圆外 (find_if 顺序)。插值分支返回 pose 无 orientation。 |
| `circleSegmentIntersection` | `(p1,p2,r)` cpp:384 | 圆-线段交点 | 以机器人为原点的闭式解 (Wolfram); `copysign(1,dd)` 保证取段内点。 |
| `PID::PID` | `(dt,max,min,kp,kd,ki)` pid.cpp:5 | 构造 | ⚠️ 参数顺序 kp,**kd,ki** (与常见 kp,ki,kd 不同), 易误用。构造时 `i_max_` 默认 1.0, 之后由 controller 覆盖。 |
| `PID::calculate` | `(set_point,pv) → double` pid.cpp:10 | 位置式 PID | ✅ (已修复) 积分先钳位到 [-i_max_, i_max_] **再**乘 ki_ 用于本周期, 抗饱和当拍生效; 输出饱和到 [min,max]。 |
| `setSumError` | `(double)` pid.cpp:47 | 重置积分 | 无独立 reset(), 重置靠传 0。 |
| `setSumErrorLimit` | `(double)` pid.hpp:27 | 设积分限幅 | ✅ (新增) 设 `i_max_` (对称抗饱和上限); 由 controller 用 `min_max_sum_error_` 在构造与动态参数更新时注入 (cpp:166-167, 718-719)。 |

## 核心数据流
```
setPlan(全局路径) 存 global_plan_
  → 每控制周期 computeVelocityCommands:
      全局路径 → transformGlobalPlan (变换到 base_frame + 裁剪到 costmap + pruning, 发 local_plan)
      → 选 lookahead carrot (发 lookahead_point)
      → 双 PID (平移 lin_dist / 朝向 angle_to_goal)
      → 曲率限速 (三点圆拟合 calculateCurvatureRadius, 发 marker)
      → 接近减速 → 碰撞检测 (10 点)
      → TwistStamped (全向 x/y + angular.z) → controller_server
```

## 关键类型 / 参数
- 全向拆分: `linear.x=v*cos(θ)`, `linear.y=v*sin(θ)` (cpp:260-261), 区别于差速纯追踪。
- 曲率限速: curvature=1/半径; >0.4 起降速, >0.7 降到 0.5 倍。
- lookahead: 静态 0.3m 或 速度×lookahead_time 钳到 [0.2,1.0]m。

## 调用关系
- **依赖**: nav2_core (Controller 基类/异常/GoalChecker)、nav2_util、nav2_costmap_2d (碰撞代价)、pluginlib (**动态加载**)、tf2、geometry_msgs/nav_msgs/visualization_msgs、rclcpp。未用 PCL/Eigen。
- **被依赖**: nav_bringup 的 `nav2_params.legged.yaml` 中 `FollowPath.plugin = omni_pid_pursuit_controller::OmniPidPursuitController`。

## 可复用性/正确性改造 (2026-07-11)
- ✅ **PID 抗积分饱和修复**: `calculate()` 现在先钳位 `integral_` 再计算 `i_out`, 抗饱和当拍生效 (原先延迟一拍)。
- ✅ **积分限幅参数化**: 新增 `PID::setSumErrorLimit`; controller 在 `configure()` (cpp:166-167) 与 `dynamicParametersCallback` (cpp:718-719) 用已有参数 `min_max_sum_error_` 注入, 该参数不再是死参数。
- 未改动: 碰撞抛异常 (#20)、参数名拼写 `treshold` (#40, 改名属破坏性 API 变更, 需单独决定)、`setSpeedLimit` 空实现。

> 详细风险 (碰撞抛异常、参数拼写、构造顺序) 见 `PROJECT_DOC.md` Layer 3 注解 #7, #14, #20, #40。#36 (min_max_sum_error 未用) 与 #39 (PID 钳位次序) 已修复。
