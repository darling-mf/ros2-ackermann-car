# ROS 2 Ackermann Car

基于 **ROS 2 Humble + Gazebo Classic 11** 的 TARKBOT R5 阿克曼小车项目：
从 URDF/`ros2_control` 底盘执行，到 Nav2 全局/局部规划、2D 雷达建图与
AMCL 定位，再到 X-Protocol 串口桥与 STM32 固件模块。

> 当前主线是 **2D 雷达 + 阿克曼控制/导航**。3D 激光方案
> （FAST-LIO / FAST-LIO-SAM-QN / livox）已弃用，源码保留但不参与构建。

## 目录

- [架构总览](#架构总览)
- [快速开始](#快速开始)
- [运行模式](#运行模式)
- [仓库结构](#仓库结构)
- [文档](#文档)
- [验证与测试](#验证与测试)
- [已知边界](#已知边界)

## 当前状态

- 仿真主链约 **85%**：控制、前向 Nav2、2D 建图、串口协议已验证；
- 前进目标 `(2.0,0)` **SUCCEEDED、0 次恢复**；AMCL 误差 3–5 cm；
- 短距倒车可用；**左房间长路径仍 ABORTED**；动态障碍/窄通道/融合默认接入未完成；
- 实车 S7~S9（HW 插件/标定/集成）未开始。详见 [docs/PROJECT_STATE.md](docs/PROJECT_STATE.md)。

## 架构总览

```
任务层   bt_navigator + BehaviorTree
规划层   planner_server + Smac Hybrid（Reeds-Shepp）
控制层   controller_server + MPPI（Ackermann）
整形层   velocity_smoother → /cmd_vel
执行层   r5_steering_controller（50Hz，阿克曼 IK）
边界层   Gazebo ros2_control / 实车 SystemInterface
感知层   2D lidar → /scan → AMCL / costmap；/odom + /imu → 融合实验
```

- **全局规划**：`SmacPlannerHybrid`，`REEDS_SHEPP`，最小转弯半径 0.35 m。
- **局部控制**：`MPPIController`，Ackermann 模型，20 Hz。
- **LQR / Pure Pursuit / Stanley**：`path_tracker.py` 独立实验台，
  与 Nav2 互斥，不是主链。
- 详细分层、接口、数据流见 **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)**。

## 快速开始

### 依赖

- Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11
- `ros-humble-gazebo-ros2-control`、`ros-humble-nav2-bringup`
- 控制器源码：`src/ros2_controllers`（工作区 overlay）或
  `ros-humble-ackermann-steering-controller`
- 2D 建图：`ros-humble-slam-toolbox`

### 构建

```bash
cd ~/car_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-up-to car_bringup
source install/setup.bash
```

### 启动

```bash
# 仿真 + 控制器（带/不带界面）
ros2 launch car_bringup gazebo.launch.py
ros2 launch car_bringup gazebo.launch.py gui:=false rviz:=false

# 一键：Gazebo + Nav2 + RViz（推荐；Nav2 只能起一套）
ros2 launch car_bringup sim_nav2.launch.py

# 2D 建图（与 Nav2 互斥）
ros2 launch car_bringup mapping.launch.py

# 传感器融合实验（不启动 Nav2）
ros2 launch car_bringup fusion.launch.py

# 清理残留（重复 Nav2/goal 无响应/RTF 异常时先用）
ros2 run car_bringup cleanup_sim.sh
```

键盘遥控：`ros2 run teleop_twist_keyboard teleop_twist_keyboard`（Twist → `/cmd_vel`）。

## 运行模式

| 模式 | 命令 | 说明 |
|---|---|---|
| 仅仿真 | `gazebo.launch.py` | Gazebo + 控制器，供遥控/调试 |
| 导航 | `sim_nav2.launch.py` | 一条命令；AMCL 自动初始位姿 |
| 建图 | `mapping.launch.py` | slam_toolbox；与 Nav2 互斥 |
| 融合 | `fusion.launch.py` | `/odometry/fused` 实验；未接管默认 odom |
| 跟踪实验 | `path_tracker.py --tracker lqr` | PP/Stanley/LQR 对比；与 Nav2 互斥 |

> 同一时间只能有一套 Nav2/建图/跟踪控制源发布 `/cmd_vel`；
> 出现 TF 两棵树、`Send goal call failed`、RTF 异常时，先跑 `cleanup_sim.sh`。

## 仓库结构

```
.
├── README.md                  # 本文件
├── docs/                      # 架构、状态、路线图、实验/排障报告
│   ├── README.md              # 文档索引（先看这里）
│   ├── ARCHITECTURE.md        # 分层架构 / 包职责 / 数据流
│   ├── PROJECT_MAP.md         # 文件树 + 进度总账
│   ├── PROJECT_STATE.md       # 压缩记忆：已验证数据/缺口
│   ├── SIM_TODO.md            # 仿真验证清单
│   ├── ENGINEERING_HANDOVER.md
│   ├── READING_GUIDE.md
│   └── reports/NAV2_DEBUG_REPORT.md
├── src/
│   ├── car_description/       # URDF / Gazebo world / RViz
│   ├── car_bringup/           # launch / config / maps / scripts
│   ├── r5_steering_controller/# 自研 C++ 阿克曼控制器
│   ├── r5_serial/             # X-Protocol 串口桥 + 虚拟 STM32
│   └── car_navigation/        # Nav2 阿克曼行为树
├── firmware/openctr_b20/      # STM32 纯 C 模块与 PC 烧录文档
├── patches/                   # 上游 ros2_controllers 补丁
├── scripts/                   # 仓库级脚本（sync-to-pi）
├── vendor/                    # 厂家固件源码/手册（第三方）
├── build/ install/ log/       # colcon 产物（不提交）
└── .gitignore
```

`ros2_control`、`ros2_controllers`、`gz_ros2_control` 作为外部 overlay
放在 `src/` 但不进本仓库；对 `ros2_controllers` 的修改归档在 `patches/`。

## 文档

| 文档 | 内容 |
|---|---|
| [docs/README.md](docs/README.md) | **文档总索引** |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | 分层/接口/数据流/启动模式 |
| [docs/PROJECT_MAP.md](docs/PROJECT_MAP.md) | 文件树 + 命令速查 |
| [docs/PROJECT_STATE.md](docs/PROJECT_STATE.md) | 当前状态、已验证数据、缺口 |
| [docs/SIM_TODO.md](docs/SIM_TODO.md) | 仿真验证清单与 RTF/建图复盘 |
| [src/r5_steering_controller/STAGE1_GUIDE.md](src/r5_steering_controller/STAGE1_GUIDE.md) | 控制器逐行/修改预测 |
| [src/r5_steering_controller/EXPERIMENT_LOG.md](src/r5_steering_controller/EXPERIMENT_LOG.md) | 实验逻辑链（Round 1~17） |
| [src/r5_serial/docs/S5_SERIAL_BRIDGE.md](src/r5_serial/docs/S5_SERIAL_BRIDGE.md) | 串口协议/真机 Checklist |
| [firmware/openctr_b20/README.md](firmware/openctr_b20/README.md) | STM32 引脚/集成/PC 烧录 |
| [patches/README.md](patches/README.md) | 上游补丁应用方法 |

## 验证与测试

```bash
# 控制器三项验收
ros2 run car_bringup acceptance_test.py --vx 0.5

# 方向/转角/轮速闭环
ros2 run car_bringup drive_test.py --vx 0.1 --wz 0.3

# PP/Stanley/LQR 对比（先停 Nav2）
ros2 run car_bringup path_tracker.py --tracker lqr --path line --offset 0.3

# 融合量化（需先 fusion.launch.py）
ros2 run car_bringup fusion_test.py --loops 3

# 串口虚拟联调
ros2 run r5_serial virtual_stm32
ros2 run r5_serial serial_bridge --port /dev/pts/N

# 上游控制器单测
colcon test --packages-select steering_controllers_library
colcon test-result
```

已验证数据、失败项与下一步见 [docs/PROJECT_STATE.md](docs/PROJECT_STATE.md)
和 [docs/SIM_TODO.md](docs/SIM_TODO.md)。

## 已知边界

- 3D 激光方案已弃用，源码保留在 `src/`，`COLCON_IGNORE` 不参与构建；
- 融合节点未接管默认 `/odom`；
- 动态障碍可停车但绕行未通过；窄通道/死胡同未压测；
- 实车 S7~S9（HW 插件/标定/集成）未完成，需硬件；
- 厂家固件无速度看门狗；`serial_bridge` 已做上位机兜底，S6 固件需补下位机看门狗。

## 参与开发

构建、测试、补丁与提交约定见 [CONTRIBUTING.md](CONTRIBUTING.md)。

## 第三方与许可

- `vendor/` 为厂家 TARKBOT/OpenCTR 固件与手册，遵循其原始许可；
- `src/ros2_control`、`src/ros2_controllers` 为 ROS Control 上游代码；
- 本仓库自研代码的许可证尚未指定，使用前请先确认。
