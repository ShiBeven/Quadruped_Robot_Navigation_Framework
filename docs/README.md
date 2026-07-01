# 机器人导航框架 — 从零开始完全指南

> **这份文档写给谁？** 写给零基础的你。不需要任何机器人学或 ROS 2 的先验知识，我们会从最基础的概念讲起，每一步都有解释和可复制的终端命令。看完这份文档，你将能够独立安装、运行、修改和扩展这套导航系统。

---

## 目录

- [第一部分：概念速通](#第一部分概念速通)
  - [什么是 ROS 2](#11-什么是-ros-2)
  - [什么是导航](#12-什么是导航)
  - [本项目的导航管线](#13-本项目的导航管线)
- [第二部分：工程地图](#第二部分工程地图)
  - [完整目录结构](#21-完整目录结构)
  - [包功能速查表](#22-包功能速查表)
- [第三部分：环境搭建](#第三部分环境搭建)
  - [系统依赖安装](#31-系统依赖安装)
  - [ROS 2 Humble 安装](#32-ros-2-humble-安装)
  - [额外依赖安装](#33-额外依赖安装)
- [第四部分：构建工程](#第四部分构建工程)
- [第五部分：运行](#第五部分运行)
  - [5.1 仿真模式（推荐新手首选）](#51-仿真模式推荐新手首选)
  - [5.2 手柄遥控](#52-手柄遥控)
  - [5.3 SLAM 建图](#53-slam-建图)
  - [5.4 纯定位导航](#54-纯定位导航)
  - [5.5 仅导航（已有定位）](#55-仅导航已有定位)
- [第六部分：参数调优指南](#第六部分参数调优指南)
- [第七部分：扩展指南](#第七部分扩展指南)
- [第八部分：常见问题排查](#第八部分常见问题排查)

---

# 第一部分：概念速通

## 1.1 什么是 ROS 2

ROS 2（Robot Operating System 2）是一个用于开发机器人软件的开源框架。你可以把它理解为"机器人领域的操作系统"——它提供了进程间通信、设备驱动、算法库和可视化工具。

**核心概念：**

| 概念 | 通俗解释 |
|------|---------|
| **节点 (Node)** | 一个独立的程序。比如"处理 LiDAR 数据的程序"就是一个节点 |
| **话题 (Topic)** | 节点之间传递消息的通道。比如节点 A 把速度指令发到 `/cmd_vel` 话题，节点 B 订阅这个话题来接收指令 |
| **消息 (Message)** | 话题上传输的数据格式。比如 `Twist` 消息包含 `linear.x`（线速度）和 `angular.z`（角速度） |
| **包 (Package)** | 代码的组织单元。每个包是一个独立的 ROS 2 项目 |
| **工作区 (Workspace)** | 多个包的集合。本工程就是一个工作区 |
| **Launch 文件** | 一键启动多个节点的配置文件。不需要逐个手动运行节点 |
| **TF2** | 坐标变换系统。告诉每个节点"传感器装在机器人哪个位置" |
| **colcon** | ROS 2 的编译工具。类比 C/C++ 中的 CMake 或 Python 中的 pip |

一个典型的 ROS 2 命令长这样：
```bash
ros2 launch 包名 launch文件名
```

## 1.2 什么是导航

机器人导航就是让机器人从 **当前位置** 安全地移动到 **目标位置**，同时避开障碍物。

**导航三要素：**

```
┌──────────────┐    ┌──────────────┐    ┌──────────────┐
│   定位        │    │   建图        │    │   路径规划     │
│ (Localization)│    │ (Mapping)    │    │ (Planning)   │
│              │    │              │    │              │
│ "我在哪？"    │    │ "周围长什么样？"│    │ "怎么到那里去？"│
└──────────────┘    └──────────────┘    └──────────────┘
```

**常见导航术语：**

| 术语 | 全称 | 含义 |
|------|------|------|
| **SLAM** | Simultaneous Localization And Mapping | 同时定位与建图——机器人在未知环境中一边移动一边构建地图并估算自己的位置 |
| **LIO** | LiDAR-Inertial Odometry | 激光雷达+惯性里程计——融合 LiDAR 和 IMU 数据来估计机器人的运动轨迹 |
| **Odometry** | — | 里程计——对机器人"走了多远、转了多少角度"的估计 |
| **Costmap** | — | 代价地图——把环境划分成一个个小格子，每个格子有一个"代价"值（0=自由空间，255=障碍物） |
| **Global Planner** | — | 全局规划器——在整张地图上规划出一条从起点到终点的粗略路径 |
| **Local Controller** | — | 局部控制器——跟踪全局路径的同时实时避障，输出速度指令 |
| **PCD** | Point Cloud Data | 点云数据——LiDAR 采集的三维空间点集合 |
| **PGM** | Portable Gray Map | 便携式灰度图——用图片格式存储的二维栅格地图 |
| **IMU** | Inertial Measurement Unit | 惯性测量单元——测量机器人的加速度和角速度 |

## 1.3 本项目的导航管线

本项目的导航管线如同一条流水线，传感器数据从底部流入，经过层层处理，最终变成驱动机器人的速度指令：

```
                         ┌──────────────────────────┐
                         │    Livox LiDAR           │  ← 激光雷达扫描环境
                         │    (非重复扫描)            │
                         └────────────┬─────────────┘
                                      │ /livox/lidar  (CustomMsg)
                                      │ /livox/imu    (IMU数据)
                         ┌────────────▼─────────────┐
      状态估计            │  point_lio                │  ← 融合LiDAR+IMU，估算位姿
      (前端)              │  (IEKF + iVox)            │
                         └────────────┬─────────────┘
                                      │ aft_mapped_to_init  (里程计, lidar_odom帧)
                                      │ cloud_registered    (去畸变点云)
                         ┌────────────▼─────────────┐
      帧适配              │  loam_interface            │  ← 把SLAM帧转换为标准odom帧
                         └────────────┬─────────────┘
                                      │ lidar_odometry     (里程计, odom帧)
                                      │ registered_scan    (点云, odom帧)
              ┌───────────────────────┼───────────────────────┐
              │                       │                       │
     ┌────────▼────────┐   ┌─────────▼─────────┐   ┌─────────▼──────────┐
感知 │sensor_scan_gen   │   │terrain_analysis    │   │small_gicp_reloc... │
     │(数据同步 + TF)    │   │(近场10m地形)        │   │(加载PCD全局重定位)   │
     └────────┬────────┘   └─────────┬─────────┘   └─────────┬──────────┘
              │                      │                        │
              │           ┌──────────▼──────────┐             │
              │           │terrain_analysis_ext  │             │
              │           │(远场40m+连通性分析)    │             │
              │           └──────────┬──────────┘             │
              │                      │ terrain_map_ext        │
              │                      │ (最终可通行地图)         │
              │                      │                        │
     ┌────────▼──────────────────────▼────────────────────────▼──┐
     │                       Nav2 导航栈                           │
     │                                                            │
导航 │  Global Planner (SmacPlannerHybrid)  ←  规划全局路径          │
     │         │                                                  │
     │         ▼                                                  │
     │  Local Controller (omni_pid_pursuit) ←  跟踪路径+实时避障     │
     │         │                                                  │
     │         ▼                                                  │
     │  cmd_vel (vx, vy, vyaw)            ←  输出全向底盘速度指令    │
     └────────────────────────────────────────────────────────────┘
```

**管线中每个环节的直观理解：**

| 环节 | 类比 |
|------|------|
| LiDAR | 你用眼睛看周围 |
| point_lio | 你的大脑根据看到的东西判断自己走了多远 |
| loam_interface | 把大脑的判断转换成 GPS 坐标 |
| terrain_analysis | 画一张周围 10 米的"可通行/障碍物"地图 |
| terrain_analysis_ext | 画一张周围 40 米的"可通行/障碍物"地图 |
| small_gicp_relocalization | 拿着手里的地图纠正"GPS漂移" |
| Nav2 Planner | 在地图上规划最短安全路径 |
| Nav2 Controller | 握着方向盘沿路径开，碰到障碍物就绕开 |

---

# 第二部分：工程地图

## 2.1 完整目录结构

```
ATS_2026_snetry_test-main/                ← 工作区根目录
│
├── README.md                             ← 你正在看的这份文档
├── AGENTS.md                             ← AI 开发助手指南
│
├── src/                                  ← 源码目录
│   │
│   ├── dependencies/                     ← 第三方依赖（不用管内部细节）
│   │   ├── BehaviorTree.ROS2/            ←   行为树引擎的 ROS2 封装
│   │   ├── joint_state_publisher/        ←   关节状态发布器
│   │   └── sdformat_tools/               ←   仿真模型格式转换工具
│   │
│   ├── interfaces/                       ← 自定义消息定义
│   │   └── robot_interfaces/             ←   云台/模型/状态消息（4条）
│   │
│   ├── navigation/                       ← ★ 核心导航包集合（13个子包）
│   │   ├── nav_bringup/                  ←   ★ 启动入口+参数配置（最重要的包！）
│   │   ├── nav2_plugins/                 ←   Nav2扩展插件（代价地图层+行为+BT节点）
│   │   ├── omni_pid_pursuit_controller/  ←   全向PID纯追踪控制器
│   │   ├── point_lio/                    ←   LiDAR-惯性里程计
│   │   ├── livox_ros_driver2/            ←   Livox LiDAR 驱动
│   │   ├── loam_interface/               ←   帧适配器
│   │   ├── sensor_scan_generation/       ←   传感器同步
│   │   ├── small_gicp_relocalization/    ←   全局重定位
│   │   ├── terrain_analysis/             ←   近场地形分析（10m）
│   │   ├── terrain_analysis_ext/         ←   远场地形分析（40m）
│   │   ├── pointcloud_to_laserscan/      ←   3D点云转2D激光
│   │   ├── ign_sim_pointcloud_tool/      ←   仿真点云格式桥接
│   │   └── teleop_twist_joy/             ←   手柄遥控
│   │
│   ├── simulation/                       ← 仿真工具
│   │   └── nav2_loopback_sim/            ←   无物理引擎回环仿真器
│   │
│   └── tools/                            ← 辅助工具
│       ├── pcd2pgm/                      ←   PCD→PGM 地图转换
│       └── rosbag2_composable_recorder/  ←   可组合rosbag录制
│
├── docs/                                 ← 参考文档
│   └── slim_loopback_refactor.md
│
├── build/                                ← colcon 编译产物（自动生成，不要手动修改）
├── install/                              ← colcon 安装产物（自动生成，不要手动修改）
└── log/                                  ← 编译日志（自动生成，不要手动修改）
```

## 2.2 包功能速查表

| 包名 | 作用（一句话） | 需要修改的概率 |
|------|--------------|:---:|
| `nav_bringup` | 启动入口、参数配置、地图存放 | ★★★★★ 非常频繁 |
| `nav2_plugins` | 自定义 BT 节点、代价地图层、恢复行为 | ★★★ 扩展功能时 |
| `omni_pid_pursuit_controller` | 控制机器人沿路径行驶，输出速度指令 | ★★★★ 调PID参数时 |
| `point_lio` | 融合 LiDAR + IMU 估算机器人位置 | ★★ 换雷达时 |
| `livox_ros_driver2` | 读取 Livox LiDAR 原始数据 | ★★ 换雷达时 |
| `loam_interface` | 坐标系转换桥接 | ★ 几乎不动 |
| `sensor_scan_generation` | 传感器数据时间同步 | ★ 几乎不动 |
| `small_gicp_relocalization` | 用先验 PCD 修正累积漂移 | ★★ 换场地时 |
| `terrain_analysis` | 近场（10m）可通行性分析 | ★★★ 调过滤参数时 |
| `terrain_analysis_ext` | 远场（40m）可通行性分析 | ★★★ 调过滤参数时 |
| `pointcloud_to_laserscan` | 3D 点云 → 2D 激光扫描 | ★ 几乎不动 |
| `ign_sim_pointcloud_tool` | 仿真点云格式转换 | ★ 几乎不动 |
| `teleop_twist_joy` | 手柄遥控机器人 | ★★ 换手柄时 |
| `nav2_loopback_sim` | 不需要硬件的仿真 | ★★ 测试时 |
| `robot_interfaces` | 自定义 ROS 消息定义 | ★ 几乎不动 |
| `pcd2pgm` | PCD 转 PGM 地图 | ★★ 处理地图时 |
| `rosbag2_composable_recorder` | 录制数据包 | ★★ 录数据时 |

---

# 第三部分：环境搭建

> **注意：** 以下所有命令都在 Ubuntu 22.04 的终端中执行。`$` 开头表示这是一条命令，复制时不要包含 `$` 符号。

## 3.1 系统依赖安装

打开终端 (Ctrl+Alt+T)，逐条执行：

```bash
# 更新软件包列表
sudo apt update

# 安装编译工具链和基础库
sudo apt install -y \
  git git-lfs curl wget python3-pip python3-vcstool python3-rosdep \
  build-essential cmake pkg-config \
  libopencv-dev libfmt-dev libeigen3-dev libyaml-cpp-dev libomp-dev
```

> **知识点：** `sudo` 表示以管理员权限执行；`apt install -y` 表示自动确认安装；`\` 表示命令换行。

## 3.2 ROS 2 Humble 安装

如果你还没有安装 ROS 2 Humble，执行以下步骤：

```bash
# 1. 添加 ROS 2 软件源
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 2. 安装 ROS 2 Humble 桌面版（含 RViz、rqt 等可视化工具）
sudo apt update
sudo apt install -y ros-humble-desktop

# 3. 安装导航相关包
sudo apt install -y ros-humble-slam-toolbox ros-humble-nav2-bringup ros-humble-joy
```

> **知识点：** ROS 2 每个版本有一个代号，Humble Hawksbill 对应 Ubuntu 22.04。`ros-humble-*` 是 ROS 2 官方提供的预编译包。

### 配置 ROS 2 环境（每次打开终端都要做）

```bash
# 将 ROS 2 环境变量加载到当前终端
source /opt/ros/humble/setup.bash

# （推荐）自动加载：把上面这行加到 ~/.bashrc 末尾，以后每次打开终端自动生效
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

> **知识点：** `source` 命令把指定脚本中的环境变量导入当前终端。`~/.bashrc` 是每次打开新终端时自动执行的脚本。

### 初始化 rosdep

```bash
sudo rosdep init
rosdep update
```

> **知识点：** `rosdep` 是 ROS 的依赖管理工具。当你编译一个包时，它能自动帮你安装这个包依赖的系统库。`rosdep init` 只需执行一次。

### 验证安装

```bash
# 打开一个终端，运行：
ros2 run demo_nodes_cpp talker

# 再打开另一个终端，运行：
ros2 run demo_nodes_py listener
```

如果第二个终端能看到 `Hello World` 消息，说明 ROS 2 安装成功。

## 3.3 额外依赖安装

### Ceres Solver（非线性优化库，point_lio 需要）

```bash
sudo apt install -y libceres-dev
```

### small_gicp（高效点云配准库，重定位需要）

```bash
cd ~
git clone https://github.com/koide3/small_gicp.git
cd small_gicp
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
sudo cmake --install build
```

> **知识点：** `$(nproc)` 返回 CPU 核心数；`-j8` 表示用 8 个核心并行编译（数字越大越快，但占内存越多）。如果编译时出现 "killed" 错误，改成 `-j1` 单核编译。

### Livox SDK2（仅当你使用 Livox 激光雷达时需要）

```bash
# 进入工程目录后执行
cd ~/ATS_2026_snetry_test-main
./src/navigation/livox_ros_driver2/scripts/setup_livox_sdk2.sh
```

> **知识点：** Livox SDK2 是览沃科技为 MID-360 / HAP 等激光雷达提供的官方软件开发包。不用 Livox 雷达可以跳过。

### PCL（点云库，地形分析需要）

```bash
sudo apt install -y libpcl-dev
```

---

# 第四部分：构建工程

### 4.1 获取代码

```bash
cd ~
git clone https://github.com/liukong1220/ATS_2026_snetry_test.git
cd ATS_2026_snetry_test
```

> **知识点：** `git clone` 从 GitHub 下载代码到本地。下载后 `cd` 进入工作区根目录。

### 4.2 安装 ROS 依赖

```bash
# 确保 ROS 2 环境已加载（没加 ~/.bashrc 的话需要手动执行）
source /opt/ros/humble/setup.bash

# 自动安装所有包的 ROS 依赖
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
```

> **知识点：** `--from-paths src` 表示扫描 `src/` 下所有包的 `package.xml`，找出依赖项；`--ignore-src` 表示忽略源码中已存在的包；`-r` 表示即使个别包找不到依赖也不中断。

### 4.3 编译

```bash
# 标准编译（推荐）
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

> **知识点：**
> - `colcon build` 是 ROS 2 的编译命令，会编译 `src/` 下所有包
> - `--symlink-install` 表示用符号链接而不是复制文件（修改 Python 脚本不用重新编译）
> - `-DCMAKE_BUILD_TYPE=Release` 表示编译优化版本（运行更快）
> - 编译产物放在 `build/` 和 `install/` 目录

**内存不够怎么办？** 如果你的电脑内存 ≤ 8GB，可能会编译到一半被系统杀掉：

```bash
# 限制并行编译数量
export CMAKE_BUILD_PARALLEL_LEVEL=1
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 1
```

**只想编译某一个包？** （修改代码后快速验证）

```bash
colcon build --packages-select nav_bringup --symlink-install
```

### 4.4 加载编译产物

```bash
# 每次打开新终端编译后都要执行
source install/setup.bash

# （推荐）加到 ~/.bashrc
echo "source ~/ATS_2026_snetry_test/install/setup.bash" >> ~/.bashrc
```

> **知识点：** `install/setup.bash` 让 ROS 2 能找到你编译的包。如果你两个工作区都 source 了，后面的会覆盖前面的（这叫 overlay 机制）。

### 4.5 验证编译成功

```bash
# 检查 nav_bringup 包是否能被找到
ros2 pkg prefix nav_bringup
```

如果输出了 `nav_bringup` 包的路径（而不是报错），说明编译成功。

---

# 第五部分：运行

## 使用前必读

每次打开新终端，执行：

```bash
source ~/ATS_2026_snetry_test/install/setup.bash
```

或直接把这个命令加到 `~/.bashrc` 末尾（推荐）。

## 5.1 仿真模式（推荐新手首选）

**不需要任何硬件！** 只需要一张静态地图，就可以在虚拟环境中跑完整的导航流程。

### 你需要准备什么

一张 Nav2 格式的栅格地图，包含两个文件：
- `xxx.yaml` — 地图描述文件（文本）
- `xxx.pgm` — 地图图片文件（黑白二值图）

> **知识点：** PGM 是 Portable Gray Map 的缩写，一种简单的灰度图片格式。白色（255）= 自由空间，黑色（0）= 障碍物，灰色（128）= 未知区域。

**示例地图文件 `my_map.yaml`：**

```yaml
image: my_map.pgm          # 指向同目录下的图片文件
mode: trinary               # 三值模式：自由/障碍/未知
resolution: 0.05            # 每个像素代表 0.05 米（5厘米）
origin: [-10.0, -10.0, 0.0] # 地图左下角在世界坐标系中的位置
negate: 0                   # 0=白色是自由空间，1=黑色是自由空间
occupied_thresh: 0.65       # 灰度值 > 0.65 视为障碍物
free_thresh: 0.196          # 灰度值 < 0.196 视为自由空间
```

把 `my_map.yaml` 和 `my_map.pgm` 放进 `src/navigation/nav_bringup/map/` 目录。

### 启动仿真

```bash
ros2 launch nav_bringup simulation.launch.py \
    map:=src/navigation/nav_bringup/map/my_map.yaml \
    use_rviz:=true
```

> **知识点：** `:=` 是 ROS 2 launch 的传参语法。`map:=路径` 把地图文件路径传给 launch 文件。

**启动后你会看到：**
1. RViz 窗口打开（3D 可视化工具）
2. 地图显示在 RViz 中
3. 左侧面板有一系列可视化选项

### 在仿真中导航

1. **设置初始位姿**：点击 RViz 顶部工具栏的 `2D Pose Estimate` 按钮，在地图上机器人实际所在位置点击并拖拽方向箭头
2. **发送导航目标**：点击 `Nav2 Goal` 按钮，在地图上目标位置点击并拖拽方向箭头
3. 机器人（绿色轨迹）将从起点规划路径并沿着路径驶向目标

> **知识点：** 在仿真中，"机器人"是一个虚拟的点。没有物理引擎，没有碰撞检测，robot 会精确地沿着你发的速度指令移动。

### 仿真是如何工作的？

`nav2_loopback_sim` 替代了 Gazebo 这种重量级物理引擎。它的原理很简单：

```
你发 cmd_vel (速度指令)
       │
       ▼
loopback_sim 对速度积分 → 算出新位姿
       │
       ├─→ 发布 odom (里程计)
       ├─→ 发布 map→odom→base_link TF链
       └─→ 在地图上做射线追踪 → 生成假 scan (激光扫描)
```

这样 Nav2 的代价地图和控制器就能正常工作了，完全不需要真实传感器。

### 仿真启动参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `map:=` | (空) | **必填**。你的地图 YAML 文件路径 |
| `use_rviz:=` | `true` | 是否打开 RViz 可视化。设为 `false` 可节省资源 |
| `params_file:=` | `...simulation.yaml` | Nav2 参数文件路径 |
| `use_sim_time:=` | `true` | 使用仿真时钟。仿真模式下必须为 `true` |
| `autostart:=` | `true` | 是否自动激活 Nav2 生命周期节点 |

**示例：** 不用 RViz，只跑仿真：

```bash
ros2 launch nav_bringup simulation.launch.py \
    map:=src/navigation/nav_bringup/map/my_map.yaml \
    use_rviz:=false
```

## 5.2 手柄遥控

连接手柄后，用手柄直接控制机器人移动：

```bash
ros2 launch nav_bringup joy_teleop_launch.py
```

> **知识点：** 手柄（Joystick）通常是 Xbox 或 PS 手柄，Linux 下插上就能识别为 `/dev/input/js0` 设备。

**手柄默认操作：**
- 左摇杆：前后左右移动
- 右摇杆：旋转
- L1 / LB：启用/禁用遥控
- R1 / RB：加速模式

**参数说明：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `joy_dev:=` | `0` | 手柄设备编号（第一个手柄是 0） |
| `joy_vel:=` | `cmd_vel` | 速度指令发到哪个话题 |
| `joy_config_file:=` | Nav2 参数文件 | 手柄按键映射配置 |

## 5.3 SLAM 建图

当你在一个未知环境中首次运行时，需要先用 SLAM 建图：

```bash
ros2 launch nav_bringup slam_launch.py \
    params_file:=src/navigation/nav_bringup/config/nav2_params.reality.yaml
```

> **知识点：** 这个 launch 会启动 point_lio（定位）+ slam_toolbox（建图）+ pointcloud_to_laserscan（点云转激光）。你需要用遥控或手柄让机器人走遍整个环境。

**建图完成后保存地图：**

```bash
# 保存 slam_toolbox 的建图结果
ros2 run nav2_map_server map_saver_cli -f ~/my_saved_map
```

这会在你的 home 目录生成 `my_saved_map.yaml` 和 `my_saved_map.pgm`。

> **知识点：** `map_saver_cli` 是 Nav2 提供的地图保存工具。`-f` 指定输出文件前缀。

**启动参数说明：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `params_file:=` | `...simulation.yaml` | 参数文件。实机用 `reality.yaml` |
| `use_sim_time:=` | `false` | 实机为 `false`，仿真才用 `true` |

## 5.4 纯定位导航

当你已有地图（从 SLAM 或别处获得），想加载地图后导航：

```bash
ros2 launch nav_bringup localization_launch.py \
    map:=/path/to/my_map.yaml \
    prior_pcd:=/path/to/prior_pointcloud.pcd
```

> **知识点：**
> - `map` 是你的二维栅格地图（pgm+yaml）
> - `prior_pcd` 是先验三维点云地图——提前在场地用 LiDAR 采集的三维整体地图。small_gicp_relocalization 用它来做全局重定位，纠正 point_lio 的累积漂移

**如果你没有 PCD 先验地图**（只想用里程计+二维地图定位）：

把 `prior_pcd` 设为空字符串（或直接不传）：

```bash
ros2 launch nav_bringup localization_launch.py \
    map:=/path/to/my_map.yaml
```

然后在 RViz 中手动用 `2D Pose Estimate` 设初始位姿，Nav2 的 AMCL 替代方案（small_gicp + prior PCD）会从那里开始工作。

**启动参数说明：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `map:=` | (无) | **必填**。二维栅格地图 YAML 路径 |
| `prior_pcd:=` | `""` | 三维先验点云地图 PCD 路径（可选但强烈推荐） |
| `params_file:=` | `...reality.yaml` | 实机参数文件 |
| `use_sim_time:=` | `false` | 实机必须为 `false` |
| `use_composition:=` | `false` | 是否用组合节点（减少内存拷贝，提高性能） |
| `autostart:=` | `true` | 是否自动激活生命周期节点 |

## 5.5 仅导航（已有定位）

当定位由外部系统提供（或者你已经提前启动了 `slam_launch` 或 `localization_launch`）时：

```bash
ros2 launch nav_bringup navigation_launch.py \
    params_file:=src/navigation/nav_bringup/config/nav2_params.reality.yaml
```

> **知识点：** 这个 launch 只启动导航栈本体——控制器、规划器、地形分析等——不启动任何定位或 SLAM 节点。它假设已有 `/odometry` 话题和 TF 变换。

**启动参数说明：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `params_file:=` | `...simulation.yaml` | 实机用 `reality.yaml` |
| `use_sim_time:=` | `false` | 实机必须为 `false` |
| `use_sensor_scan:=` | `true` | 是否启动 sensor_scan_generation 节点 |

---

# 第六部分：参数调优指南

所有 Nav2 相关参数都在两个 YAML 文件中：
- `src/navigation/nav_bringup/config/nav2_params.reality.yaml` — 实机参数
- `src/navigation/nav_bringup/config/nav2_params.simulation.yaml` — 仿真参数

> **知识点：** YAML 是一种人类可读的配置文件格式，用缩进表示层级。修改 YAML 后**不需要重新编译**，重新 launch 即可生效。

## 6.1 控制器调优（最重要）

如果你发现机器人"撞墙"、"追不上路径"、"震荡"，首先调控制器参数。

在 YAML 中找到 `controller_server` 段落（reality.yaml 约第 310 行）：

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0        # 控制频率（Hz），越高越精细
    FollowPath:
      plugin: "omni_pid_pursuit_controller::OmniPidPursuitController"
      
      # 速度限制
      v_linear_min: -4.5              # 最大后退速度 (m/s)
      v_linear_max: 4.5               # 最大前进速度 (m/s)
      v_angular_min: -3.0             # 最大顺时针角速度 (rad/s)
      v_angular_max: 3.0              # 最大逆时针角速度 (rad/s)
      
      # 预瞄距离
      lookahead_dist: 0.8             # 预瞄距离 (m)。越大路径越平滑但越不精确
      min_lookahead_dist: 0.5         # 最小预瞄距离
      max_lookahead_dist: 2.0         # 最大预瞄距离
      
      # 目标容差
      xy_goal_tolerance: 0.15         # 到达目标的 XY 容差 (m)
      yaw_goal_tolerance: 6.28        # 到达目标的航向容差 (rad)。6.28=不限航向
      
      # 曲率控制
      curvature_min: 0.5              # 最小转弯曲率
      curvature_max: 0.9              # 最大转弯曲率
```

**常见调优场景：**

| 问题 | 可能的解决方案 |
|------|---------------|
| 机器人震荡（左右摇摆） | 减小 `lookahead_dist`，降低 `v_angular_max` |
| 机器人转弯太急 | 增大转弯半径 → 减小 `curvature_max` |
| 到达目标了还在微调位置 | 增大 `xy_goal_tolerance`（比如改为 0.2） |
| 机器人跟不上路径 | 增大 `lookahead_dist`，增大 `v_linear_max` |
| 机器人撞障碍物 | 增大 `inflation_radius`（代价地图膨胀半径） |
| 控制延迟感 | 增大 `controller_frequency` |

> **知识点：**
> - **预瞄距离** (Lookahead Distance)：纯追踪算法在路径上挑选一个"往前看"的目标点，机器人追着这个点走。预瞄距离越远，路径越平滑但拐弯越"钝"；越近越精确但容易震荡。
> - **纯追踪** (Pure Pursuit)：一种经典的路径跟踪算法。想象你在骑自行车追前面的人——你看的不是脚下，而是前方某个距离的目标。

## 6.2 代价地图调优

代价地图决定"机器人认为哪里能走，哪里不能走"。

在 YAML 中找到 `local_costmap` 和 `global_costmap` 段落：

```yaml
local_costmap:
  ros__parameters:
    robot_radius: 0.3                 # 机器人外接圆半径 (m)
    inflation_radius: 0.4             # 障碍物膨胀半径 (m)
    cost_scaling_factor: 4.0          # 代价衰减系数（越大衰减越快）
    # ...其他参数

global_costmap:
  ros__parameters:
    robot_radius: 0.3                 # 同上
    inflation_radius: 0.6             # 全局的膨胀可以大一些
```

**常见调优：**

| 问题 | 操作 |
|------|------|
| 机器人离障碍物太近 | 增大 `robot_radius` 和 `inflation_radius` |
| 机器人过不去窄通道 | 减小 `robot_radius` 和 `inflation_radius` |
| 全局规划路径太靠近墙 | 增大 `global_costmap` 的 `inflation_radius` |

> **知识点：**
> - **局部代价地图** (Local Costmap)：机器人周围一个方形窗口（默认 5m×5m），随机器人移动而滚动。用于实时避障。
> - **全局代价地图** (Global Costmap)：覆盖整张静态地图，用于全局路径规划。
> - **膨胀层** (Inflation Layer)：在障碍物周围"吹"出一圈禁区。这个圈越大，机器人越安全但也越保守。

## 6.3 地形分析调优

如果你的场地有斜坡、草坡、不平整地面，需要调地形分析参数。

在 YAML 中找到 `terrain_analysis` 段落：

```yaml
terrain_analysis:
  ros__parameters:
    scanVoxelSize: 0.05               # 点云降采样分辨率 (m)。越小越精细但越吃算力
    decayTime: 2.0                    # 体素过期时间 (s)。大于2秒没新数据就丢弃
    vehicleHeight: 1.5                # 车体高度 (m)。高于此的点视为悬垂障碍物
    minRelZ: -1.5                     # 车体下方裁剪高度 (m)
    maxRelZ: 0.2                      # 车体上方裁剪高度 (m)
    quantileZ: 0.25                   # 地面估计分位数。0.25=取最低25%点的高度
```

> **知识点：**
> - **分位数过滤** (Quantile Filter)：一堆点的高度排序后取第 Q 百分位的值作为地面高度。Q=0.25 意味着"大部分点都在这个高度以上"——这对于地面上的草、小石子等噪声很有效。
> - **体素** (Voxel)：三维空间的最小单元，类似二维图片的像素。

---

# 第七部分：扩展指南

## 7.1 添加你自己的地图

1. 把 `.pgm` 和 `.yaml` 文件放入：
   ```
   src/navigation/nav_bringup/map/
   ```

2. 启动时指定地图：
   ```bash
   ros2 launch nav_bringup simulation.launch.py \
       map:=src/navigation/nav_bringup/map/你的地图.yaml
   ```

> **提示：** 如果没有自己的地图，可以用 loopback_sim 自带的地图做测试：
> ```bash
> ros2 launch nav_bringup simulation.launch.py \
>     map:=src/simulation/nav2_loopback_sim/maps/tb3_sandbox.yaml
> ```

## 7.2 接入你自己的激光雷达

如果你用的不是 Livox 雷达：

1. **确保你的雷达驱动发布 `sensor_msgs/msg/PointCloud2`**

2. **修改参数文件**（复制一份 reality.yaml）：
   ```yaml
   point_lio:
     ros__parameters:
       lid_topic: "你的雷达话题名"       # 默认是 livox/lidar
       imu_topic: "你的IMU话题名"        # 默认是 livox/imu
       lidar_type: 2                     # 1=Livox, 2=Velodyne/其他
       scan_line: 32                     # 你的雷达线数
   ```

3. **如果不用 Livox，跳过 SDK 安装**：编译时可以排除 livox_ros_driver2：
   ```bash
   colcon build --packages-skip livox_ros_driver2 --symlink-install
   ```

## 7.3 接入你自己的机器人底盘

1. **确保你的底盘接受 `geometry_msgs/msg/Twist` 或 `TwistStamped` 速度指令**

2. **修改控制器速度范围**（在参数 YAML 中）：
   ```yaml
   controller_server:
     ros__parameters:
       FollowPath:
         plugin: "omni_pid_pursuit_controller::OmniPidPursuitController"
         v_linear_min: -2.0             # 最大后退速度
         v_linear_max: 2.0              # 最大前进速度
         v_angular_min: -1.5            # 最大旋转速度
         v_angular_max: 1.5
   ```

3. **如果你的底盘不是全向的**（只能前进+旋转，不能横向移动）：
   把 `omni_pid_pursuit_controller` 替换为 Nav2 标准控制器。在参数 YAML 中：
   ```yaml
   controller_server:
     ros__parameters:
       FollowPath:
         plugin: "dwb_core::DWBLocalPlanner"   # 或 regulated_pure_pursuit::RegulatedPurePursuit
   ```

4. **修改 TF 帧名称**：确保参数 YAML 中的 `base_frame`、`odom_frame` 与你的底盘发布的 TF 一致。

## 7.4 录制和分析数据

```bash
# 录制所有话题
ros2 bag record -a -o ~/my_recording

# 只录制特定话题
ros2 bag record /odometry /cmd_vel /scan -o ~/my_recording

# 回放录制的数据
ros2 bag play ~/my_recording
```

> **知识点：** `ros2 bag` 是 ROS 2 的数据录制/回放工具。录制文件以 `.db3`（sqlite3）格式存储。回放时其他节点可以像实时数据一样接收消息，非常方便调试。

---

# 第八部分：常见问题排查

## 8.1 编译问题

| 错误 | 原因 | 解决方法 |
|------|------|---------|
| `cannot find -lMvCameraControl` | 旧 README 的遗留依赖，已删除 | 确认你用的是最新的精简版工程 |
| `cc1plus: fatal error: Killed` | 内存不足 | `export CMAKE_BUILD_PARALLEL_LEVEL=1`，然后用 `-j1` 编译 |
| `package 'xxx' not found` | 缺少系统包 | 运行 `rosdep install -r --from-paths src --ignore-src --rosdistro humble -y` |
| `Could not find a package configuration file provided by "xxx"` | 缺少 ROS 依赖 | `sudo apt install ros-humble-xxx` |

## 8.2 运行问题

| 问题 | 原因 | 解决方法 |
|------|------|---------|
| `package 'nav_bringup' not found` | 没 source install | 执行 `source install/setup.bash` |
| RViz 没有显示地图 | map 路径写错了 | 用绝对路径 `map:=/home/你的用户名/.../my_map.yaml` |
| 机器人在仿真中不动 | 没有设置初始位姿 | 在 RViz 用 `2D Pose Estimate` 点击机器人位置 |
| `/scan` 话题没有数据 | loopback_sim 没加载到地图 | 确认 `map:=` 参数传了正确的 YAML |
| 控制器不跟踪路径 | 参数中的控制器插件名写错了 | 确认是 `omni_pid_pursuit_controller::OmniPidPursuitController` |
| 仿真中机器人撞墙 | 代价地图膨胀不够 | 增大 `inflation_radius` |

## 8.3 如何获取帮助

```bash
# 查看当前有哪些节点在运行
ros2 node list

# 查看当前有哪些话题在发布
ros2 topic list

# 查看某个话题的实时数据
ros2 topic echo /odometry

# 查看 TF 变换树
ros2 run tf2_tools view_frames

# 查看某个包的详细信息
ros2 pkg xml nav_bringup
```

> **知识点：** `ros2 topic echo` 是最常用的调试命令，能让你看到某个话题上正在传输的数据。`ros2 node list` 让你知道当前哪些节点在运行。

---

## 附录：命令速查表

```bash
# ===== 环境 =====
source /opt/ros/humble/setup.bash                         # 加载 ROS 2 环境
source install/setup.bash                                  # 加载本工程环境

# ===== 编译 =====
colcon build --symlink-install                             # 全量编译
colcon build --packages-select 包名 --symlink-install       # 只编译某个包
colcon build --packages-skip 包名 --symlink-install         # 跳过某个包编译

# ===== 启动 =====
ros2 launch nav_bringup simulation.launch.py map:=xxx.yaml use_rviz:=true      # 仿真
ros2 launch nav_bringup slam_launch.py                                         # SLAM建图
ros2 launch nav_bringup localization_launch.py map:=xxx.yaml prior_pcd:=x.pcd  # 定位导航
ros2 launch nav_bringup navigation_launch.py                                   # 仅导航
ros2 launch nav_bringup joy_teleop_launch.py                                   # 手柄遥控

# ===== 调试 =====
ros2 node list                                              # 查看运行中的节点
ros2 topic list                                             # 查看所有话题
ros2 topic echo /话题名                                      # 查看话题数据
ros2 topic hz /话题名                                        # 查看话题发布频率
ros2 run rqt_reconfigure rqt_reconfigure                     # 动态参数调节（GUI）
ros2 run tf2_tools view_frames                              # 查看 TF 树
```

---

> **文档版本：** 2026-06-30  
> **工程项目：** ATS 2026 Sentry → 日常导航框架  
> **ROS 版本：** Humble Hawksbill  
> **维护者：** 是你  
> **下一步：** 看完这份文档后，建议从 [5.1 仿真模式](#51-仿真模式推荐新手首选) 开始，把 loopback 仿真跑通，感受完整导航流程。然后逐步尝试实机部署和参数调优。
