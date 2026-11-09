# 系统架构

本文说明公开、可构建的 2D 激光雷达阿克曼栈：每层由哪个包负责、由哪些文件
实现、数据和 TF 怎么流动。

## 1. 范围

机器人是后轮驱动、前轮转向的阿克曼底盘，基于 TARKBOT R5 级别车体。
整套栈在 ROS 2 Humble + Gazebo Classic 11 上开发和测试。实车边界放在
`ros2_control` 后面，仿真换成硬件时上层不需要改。

```text
任务层        bt_navigator + 行为树
全局规划      SmacPlannerHybrid（Reeds-Shepp）
局部控制      MPPI（Ackermann 运动模型）
速度整形      velocity_smoother
底盘执行      r5_steering_controller（ros2_control）
硬件边界      gazebo_ros2_control / SystemInterface
```

## 2. 包职责

| 包 | 职责 | 关键文件 |
|---|---|---|
| `car_description` | URDF/xacro、Gazebo 世界、RViz 配置 | `urdf/car.urdf.xacro`、`urdf/car/control.xacro`、`urdf/car/wheels.xacro`、`world/custom_room.world` |
| `car_bringup` | 启动文件、Nav2/控制器参数、地图、诊断脚本 | `launch/*.launch.py`、`config/controllers.yaml`、`config/nav2_params.yaml`、`config/slam_toolbox.yaml` |
| `r5_steering_controller` | 自研 C++ 阿克曼控制器和安全限幅 | `src/r5_steering_controller.cpp`、`include/r5_steering_controller/r5_steering_controller.hpp` |
| `car_navigation` | 阿克曼专用 Nav2 行为树 | `behavior_trees/akm_navigate_to_pose.xml`、`akm_navigate_through_poses.xml` |
| `r5_serial` | X-Protocol 串口桥和虚拟 STM32 | `r5_serial/protocol.py`、`serial_bridge.py`、`virtual_stm32.py`、`serial_port.py` |
| `firmware/openctr_b20` | STM32 可移植 C 模块和主机测试 | `src/xprotocol.c/.h`、`src/safety.c/.h`、`src/ackermann.c/.h`、`test/test_host.c` |
| `patches` | 上游 `ros2_controllers` 修改归档 | `ros2_controllers-r5.patch` |

## 3. 控制链

### 3.1 从速度指令到关节

```text
/cmd_vel（geometry_msgs/Twist）
   |
   v
r5_steering_controller::update()
   |-- 加速度斜坡
   |-- 速度相关转向限幅
   |-- 转向速率限幅
   |-- 阿克曼逆解
   v
后轮速度 x2 + 前轮转角 x2
   |
   v
gazebo_ros2_control / 实车 SystemInterface
```

控制器 fork 自上游 `ackermann_steering_controller`。上游每个周期都会写
reference 接口，所以自研层在 `update()` 里对父类输出做后处理，而不是复制
一套上游运动学。

### 3.2 为什么需要这些限幅

| 限幅 | 原因 | 位置 |
|---|---|---|
| 加速度斜坡 | 保护电机驱动和电流预算；避免起步打滑 | `r5_steering_controller.cpp` |
| 转向速率限幅 | 舵机指标约 9.5 rad/s | `controllers.yaml`（`r5.max_steer_rate`） |
| 速度相关转向 | 约束侧向加速度 `a_y = v^2 * tan(phi) / L`，防止高速侧滑 | `r5_steering_controller.cpp` |
| 转向角钳位 | 上游没有物理转向限位 | 控制器源码 / 补丁 |

对应参数在 `src/car_bringup/config/controllers.yaml`：

```yaml
wheelbase: 0.112
front_wheel_track: 0.164
rear_wheel_track: 0.164
front_wheels_radius: 0.0325
rear_wheels_radius: 0.0325
max_steering_angle: 0.235
r5.max_steer_rate: 9.5
r5.max_accel: 0.8
r5.max_decel: 1.5
r5.a_lat_max: 2.0
```

这些值来自 R5 手册和实车实测：轴距 112 mm、轮距 164 mm、轮胎直径 65 mm。
URDF 和 `controllers.yaml` 使用同一套参数。

### 3.3 里程计归属

```text
r5_steering_controller  -> /wheel/odom（enable_odom_tf: false）
sensor_fusion.py        -> /odom + odom -> base_footprint
AMCL                    -> map -> odom
```

`odom -> base_footprint` 只能有一个广播者。两个发布者会形成两棵 TF 树，
导致 AMCL 和代价地图跳变。详见
[TROUBLESHOOTING.md](TROUBLESHOOTING.md)。

`sensor_fusion.py` 还会用前 100 个静止样本估计陀螺零偏，并加一个小死区，
防止停车时融合航向缓慢旋转。启动后请保持车体静止约 2 秒。

## 4. 导航栈

### 4.1 任务层

`src/car_navigation/behavior_trees/akm_navigate_to_pose.xml` 和
`akm_navigate_through_poses.xml` 定义恢复行为。行为树不使用原地 `Spin`，
因为阿克曼底盘不能零速自转；恢复动作使用有界的 `BackUp` 和
`DriveOnHeading`。

`src/car_bringup/launch/nav2.launch.py` 加载 `config/nav2_params.yaml`，并把
自定义行为树注入进去。行为树是启动契约的一部分，不需要在 RViz 里手选。

### 4.2 全局规划

`planner_server` 使用 `SmacPlannerHybrid` 和 `REEDS_SHEPP` 运动基元：

```yaml
planner_server:
  ros__parameters:
    GridBased:
      plugin: nav2_smac_planner/SmacPlannerHybrid
      motion_model_for_search: REEDS_SHEPP
      minimum_turning_radius: 0.55
      reverse_penalty: 2.0
```

车辆有最小转弯半径，而且有些目标在车后，所以规划器允许倒车；但会惩罚倒车，
优先前进路径。0.55 m 与手册、0.235 rad 转向限位和实测车体一致。

### 4.3 局部控制

`controller_server` 运行 `nav2_mppi_controller::MPPIController`：

```yaml
motion_model: Ackermann
AckermannConstraints:
  min_turning_r: 0.55
vx_max: 1.0
vx_min: -0.15
```

MPPI 跟踪全局路径并做局部避障。Pure Pursuit、Stanley、LQR 在
`src/car_bringup/scripts/path_tracker.py` 里单独实现，属于控制器对比实验；
Nav2 运行时它们不能同时发布 `/cmd_vel`。

### 4.4 速度整形

`velocity_smoother` 在底盘控制器之前限制最终指令。它的 `max_velocity`
必须和 MPPI 的 `vx_max` 一致，否则规划器和执行层对“实际能跑多快”理解不同。

## 5. 定位与感知

### 5.1 2D 激光雷达

```text
Gazebo ray 传感器 / 实车 RPLIDAR 或 LD19
        |
        v
      /scan
        |-- AMCL（likelihood field）
        |-- 局部代价地图障碍层
        |-- 全局代价地图障碍层
```

仿真传感器定义在 `src/car_description/urdf/car/laser.xacro`；建图参数在
`src/car_bringup/config/slam_toolbox.yaml`。

### 5.2 AMCL

AMCL 使用 `/scan`、`/odom` 和静态地图，发布 `map -> odom`。关键参数在
`src/car_bringup/config/nav2_params.yaml`：

- `max_beams: 90`
- `update_min_d: 0.10`
- `update_min_a: 0.10`
- `laser_min_range: 0.25`

1.0 m/s 时需要更紧的更新阈值；默认 0.25 m 的距离阈值会让滤波器明显滞后。

### 5.3 代价地图

局部和全局代价地图都由障碍层 + 膨胀层组成。车体 footprint 是矩形，
`raytrace_min_range` / `obstacle_min_range` 都设为 0.25 m，避免激光打到
前轮/车体后又被膨胀到机器人中心。详见
[TROUBLESHOOTING.md](TROUBLESHOOTING.md)。

## 6. 串口与固件边界

实车链路是 UART 上的 X-Protocol 帧流：

```text
AA 55 LEN ID DATA CHECK
```

`r5_serial` 实现 ROS 侧串口桥，并通过 pty 提供虚拟 STM32，方便无硬件联调。
`firmware/openctr_b20` 包含可移植 C 模块：

- `xprotocol.c/.h`：帧解析和打包；
- `safety.c/.h`：300 ms 指令看门狗；
- `ackermann.c/.h`：虚拟转角、内轮角、舵机映射，几何参数可标定。

固件模块用 `gcc` 做主机测试；厂家固件和手册不在本仓库分发。

## 7. 启动模式

| 命令 | 启动内容 | 互斥关系 |
|---|---|---|
| `gazebo.launch.py` | Gazebo + 控制器 + 融合 | 可以与 Nav2 叠加 |
| `sim_nav2.launch.py` | Gazebo + Nav2 + RViz | 不要再起第二套 Nav2 |
| `nav2.launch.py` | 只启动 Nav2 | 不要与建图同时 |
| `mapping.launch.py` | Gazebo + `slam_toolbox` | 不要与 Nav2/AMCL 同时 |
| `fusion.launch.py` | Gazebo + 一个显式融合节点 | 不启动 Nav2 |

同一时间只能有一个进程发布 `/cmd_vel`。`cleanup_sim.sh` 会清理残留的
Nav2 容器、Gazebo、RViz、建图和融合进程。

## 8. 设计取舍

| 选择 | 替代方案 | 为什么这样选 |
|---|---|---|
| 自研控制器做命令层后处理 | 复制整套上游控制器 | 补丁小，保留上游运动学和测试 |
| Smac Hybrid + Reeds-Shepp | 纯 Dubin | 能处理车后目标和室内窄弯 |
| MPPI 局部控制器 | DWB / TEB | Humble 下阿克曼支持好，矩形 footprint 行为可预测 |
| 轮速 + 陀螺互补滤波 | 纯轮式里程计 | 陀螺提供航向，轮速提供位移，简单可测 |
| 融合节点独占 TF | 控制器和融合都发 TF | 去掉重复 authority，避免 AMCL 跳变 |
| 自定义 BT，去掉 `Spin` | 默认 Nav2 行为树 | 阿克曼不能原地旋转 |

## 9. 已知边界

- 左房间任意 yaw 仍可能绕远或超时；yaw=−π/2 已实测 54 s 成功。量产场景
  建议用 Waypoint/BT 分段，而不是一条长目标。
- 目标离墙小于约 0.5 m 时可能不可达，因为终点落在膨胀后的代价地图里。
- 窄通道和移动障碍压力测试尚未完成。
- 实车 `SystemInterface`、IMU 标定和整机集成尚未完成。仿真数值不能替代
  实车测量。
