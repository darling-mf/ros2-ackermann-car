# ROS 2 阿克曼小车

基于 **ROS 2 Humble + Gazebo Classic** 的 TARKBOT R5 阿克曼小车项目，覆盖从
URDF、`ros2_control` 底盘执行，到 Nav2 全局/局部规划、2D 激光雷达建图、
AMCL 定位，以及 STM32 下位机串口桥的完整链路。

公开仓库聚焦 **2D 激光雷达导航栈**。早期 3D 激光雷达实验不参与构建。

## 工程亮点

- **自研 C++ `ros2_control` 控制器**：后轮驱动、前轮转向的阿克曼底盘，
  带加速度限幅、转向速率限幅和随速度变化的转向限幅。
- **Nav2 阿克曼导航**：Smac Hybrid 全局规划器（Reeds-Shepp）、MPPI 局部
  控制器，以及不使用原地 `Spin` 恢复的自定义行为树。
- **轮速 + IMU 融合里程计**：作为默认 odom 源。控制器只发布
  `/wheel/odom`，融合节点发布 `/odom` 和唯一的
  `odom -> base_footprint` 变换。
- **2D 建图与定位**：`slam_toolbox`、`map_saver`、AMCL，以及带车体
  footprint 的代价地图配置。
- **嵌入式链路**：X-Protocol 串口桥、用于无硬件联调的虚拟 STM32，以及
  可移植的 C 模块（协议解析、300 ms 安全看门狗、阿克曼几何）。

## 实测结果

以下数据全部来自 Gazebo 仿真，使用
`src/car_bringup/maps/my_room_map.yaml` 地图和融合里程计。

| 场景 | 结果 | 证据 |
|---|---|---|
| 控制器验收 | PASS | `acceptance_test.py`（直行、转向、倒车） |
| 前进目标 `(2.0, 0)` | `SUCCEEDED`，0 次恢复 | AMCL 与真值误差 3–5 cm |
| 1.0 m/s 目标 `(4.0, 0)` | `SUCCEEDED`，8.0 s | 定位误差约 5.2 cm |
| 1.0 m/s 目标 `(6.0, 0)` | `SUCCEEDED`，4.8 s | 定位误差约 6.0 cm |
| 障碍物 `(4.0, 0)`，目标 `(6.0, 0)` | `SUCCEEDED`，12 s，0 次恢复 | 静态圆柱；定位误差约 6.7 cm |
| 左房间 `(-3.0, 0.5)`，yaw −π/2 | `SUCCEEDED`，54 s，0 次恢复 | 地图与真值差约 19 cm；对 yaw 敏感 |
| 2D 建图 | 地图保存成功 | 376 × 222 栅格，分辨率 0.05 m |
| 串口桥 + 虚拟 STM32 | 协议自检 + 虚拟设备联调 | 示例帧、回环、坏帧重同步 |
| STM32 C 模块 | 主机测试 5/5 | `firmware/openctr_b20/test/test_host.c` |

已知边界见 [docs/VALIDATION.md](docs/VALIDATION.md)：左房间仍对目标 yaw
敏感，贴墙目标可能不可达，窄通道压力测试尚未完成。

## 系统架构

```text
任务层        bt_navigator + 自定义行为树
              src/car_navigation/behavior_trees/akm_navigate_to_pose.xml
       |
全局规划      planner_server + SmacPlannerHybrid（Reeds-Shepp）
              src/car_bringup/config/nav2_params.yaml
       |
局部控制      controller_server + MPPI（Ackermann 模型）
              src/car_bringup/config/nav2_params.yaml
       |
速度整形      velocity_smoother -> /cmd_vel
       |
底盘执行      r5_steering_controller（50 Hz，阿克曼逆解）
              src/r5_steering_controller/src/r5_steering_controller.cpp
       |
硬件边界      gazebo_ros2_control（仿真）/ SystemInterface（实车）
```

感知与定位支路：

```text
/scan（2D 激光雷达）--> AMCL：map -> odom
                     --> 局部/全局代价地图

/wheel/odom + /imu --> sensor_fusion.py --> /odom + odom -> base_footprint
```

设计边界是刻意划分的：`r5_steering_controller` 只做阿克曼逆解和安全限幅，
不决定“往哪走”；`MPPI` 跟踪全局路径；`path_tracker.py` 里的
Pure Pursuit / Stanley / LQR 是控制器对比实验，不属于 Nav2 主链。

## 仓库结构

```text
.
├── README.md
├── CONTRIBUTING.md
├── docs/
│   ├── README.md                  # 公开文档索引
│   ├── ARCHITECTURE.md            # 分层、接口、数据流、文件定位
│   ├── VALIDATION.md              # 测试方法、实测数据、已知边界
│   └── TROUBLESHOOTING.md         # Nav2/TF/代价地图/AMCL 排障案例
├── src/
│   ├── car_description/           # URDF/xacro、Gazebo 世界、RViz 配置
│   ├── car_bringup/               # launch、参数、地图、测试脚本
│   ├── r5_steering_controller/    # 自研 C++ 阿克曼控制器
│   ├── r5_serial/                 # X-Protocol 串口桥 + 虚拟 STM32
│   └── car_navigation/            # 阿克曼行为树
├── firmware/openctr_b20/          # STM32 可移植 C 模块 + 主机测试
├── patches/                       # 上游 ros2_controllers 补丁
└── LICENSE
```

每个包都有自己的 `README.md`，里面写明具体改哪个文件、暴露什么接口。

## 构建

### 依赖

- Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11
- `ros-humble-gazebo-ros2-control`
- `ros-humble-nav2-bringup`
- `ros-humble-slam-toolbox`
- `ros-humble-robot-state-publisher`、`ros-humble-xacro`

自研控制器基于 `ackermann_steering_controller`。对上游源码的修改归档在
`patches/`；如果需要转向限幅和里程计协方差修复，不要只依赖二进制包。

### 构建工作区

```bash
cd ~/car_ws
source /opt/ros/humble/setup.bash

# 可选但推荐：带补丁的上游控制器源码
git clone -b humble https://github.com/ros-controls/ros2_controllers.git \
  src/ros2_controllers
git -C src/ros2_controllers checkout 65db7362
git -C src/ros2_controllers apply \
  ../../patches/ros2_controllers-r5.patch

colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-up-to car_bringup
source install/setup.bash
```

如果工作区里已经安装并打好补丁，可以跳过 `git clone`。上游版本和补丁范围
见 [patches/README.md](patches/README.md)。

## 运行

```bash
# Gazebo + 控制器
ros2 launch car_bringup gazebo.launch.py

# Gazebo + Nav2 + RViz（只能起一套 Nav2）
ros2 launch car_bringup sim_nav2.launch.py

# 2D 建图（与 Nav2 互斥）
ros2 launch car_bringup mapping.launch.py

# 虚拟 STM32 + 串口桥（无需硬件）
ros2 run r5_serial virtual_stm32
ros2 run r5_serial serial_bridge --port /dev/pts/N
```

发送导航目标：

```bash
ros2 action send_goal /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

同一时间只能有一个进程发布 `/cmd_vel`。在 Nav2、建图和跟踪实验之间切换时，
先运行 `ros2 run car_bringup cleanup_sim.sh`。

## 文档

| 文档 | 说明 |
|---|---|
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | 每层由哪个包/文件负责，数据和 TF 怎么流 |
| [docs/VALIDATION.md](docs/VALIDATION.md) | 测了什么、用什么参数、结果是多少 |
| [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) | Nav2、TF、代价地图、AMCL 的可复用排查路径 |
| [src/r5_steering_controller/README.md](src/r5_steering_controller/README.md) | 控制器限幅、参数和测试入口 |
| [src/car_bringup/README.md](src/car_bringup/README.md) | 启动模式、参数、地图和脚本 |
| [src/r5_serial/README.md](src/r5_serial/README.md) | 串口协议和虚拟 STM32 |
| [firmware/openctr_b20/README.md](firmware/openctr_b20/README.md) | STM32 集成点和主机测试 |
| [patches/README.md](patches/README.md) | 上游控制器补丁和应用步骤 |

## 设计说明

- **命令层后处理**：父控制器每个周期都会覆盖 reference 接口，因此加速度和
  转向限幅放在父类输出之后的 `update()` 里。这样补丁小，也不用复制上游
  运动学代码。
- **速度相关转向限幅**：用 `a_y = v^2 * tan(phi) / L` 约束侧向加速度。
  低速保留完整转向范围，高速在进入舵机模型前就限制指令。
- **只有一个 TF 发布者**：控制器和融合节点同时打开 `enable_odom_tf` 会产生
  两套 `odom -> base_footprint`。控制器只发布 `/wheel/odom`；融合节点同时
  估计静止陀螺零偏，避免停车时航向漂移。
- **不做原地旋转**：阿克曼底盘不能零速自转。自定义行为树去掉 `Spin`，
  改用有界的 `BackUp` / `DriveOnHeading` 恢复。
- **数据说话**：控制器验收、AMCL 与真值误差、代价地图栅格值都记录在
  [docs/VALIDATION.md](docs/VALIDATION.md)。

## 当前状态与路线

- 仿真已验证：控制、前进导航、1.0 m/s 导航、2D 建图、融合里程计、串口协议。
- 仿真待完善：任意 yaw 的左房间路径（yaw=0 仍会绕远）、贴墙目标、窄通道、
  移动障碍预测。
- 实车待完成：硬件接口、轮径/IMU 标定、整机集成。当前仓库是仿真验证过的
  基线，不是完成品。

## 参与开发

构建/测试命令、补丁规范和提交约定见 [CONTRIBUTING.md](CONTRIBUTING.md)。

## 许可证

本仓库自研代码使用 Apache License 2.0，见 [LICENSE](LICENSE)。
第三方组件遵循各自原始许可证；厂家固件和手册不在本仓库分发。
