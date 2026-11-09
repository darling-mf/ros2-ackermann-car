# 验证记录

本文记录测了什么、怎么测、哪些还没有验证。结论保持保守：只有动作结果和
实测状态一致，才记为通过。

## 1. 测试环境

| 项目 | 值 |
|---|---|
| 系统 / ROS | Ubuntu 22.04、ROS 2 Humble |
| 仿真器 | Gazebo Classic 11.10.2 |
| 地图 | `src/car_bringup/maps/my_room_map.yaml`（376 × 222，0.05 m） |
| 控制器 | `r5_steering_controller`，50 Hz |
| 定位 | 融合 `/odom` + AMCL |
| 真值 | Gazebo p3d `/ground_truth`（仅仿真） |
| 起点 | `(0, 0, yaw 0)` |
| 测试入口 | `ros2 launch car_bringup sim_nav2.launch.py gui:=false rviz:=false` |

真值话题只用于测量，Nav2 和 AMCL 不订阅它。

## 2. 测试总表

| 模块 | 场景 | 结果 | 证据 |
|---|---|---|---|
| 底盘 | 直行 / 转向 / 倒车验收 | 通过 | `acceptance_test.py` |
| 底盘 | 方向、转角、轮速闭环 | 通过 | `drive_test.py` |
| 上游控制器 | `steering_controllers_library` 测试 | 38 项通过 | `colcon test` |
| 导航 | 目标 `(2.0, 0)`，0.3 m/s | 成功，0 次恢复 | AMCL 与真值差 3–5 cm |
| 导航 | 目标 `(4.0, 0)`，1.0 m/s | 成功，8.0 s | 定位误差 5.2 cm |
| 导航 | 目标 `(6.0, 0)`，1.0 m/s | 成功，4.8 s | 定位误差 6.0 cm |
| 里程计 | 轮速 + IMU 融合作为默认 `/odom` | 通过 | `/odom` 只有一个发布者，TF 只有一个 authority |
| 里程计 | 静止陀螺零偏标定 | 通过 | 100 个静止样本；静止漂移 < 0.1 m |
| 建图 | `slam_toolbox` + `map_saver` | 通过 | 地图 376 × 222，0.05 m |
| 障碍 | 静态圆柱 `(4.0, 0)`，目标 `(6.0, 0)` | 成功，12 s，0 次恢复 | 定位误差 6.7 cm |
| 障碍 | 圆柱 `(6.5, 0)`，目标 `(8.0, 0)` 贴墙 | 60 s 超时 | 目标落在膨胀区 |
| 长路径 | 左房间 `(-3.0, 0.5)`，yaw −π/2 | 成功，54 s，0 次恢复 | 地图与真值差约 19 cm；对 yaw 敏感 |
| 串口 | 示例帧、回环、坏帧重同步 | 通过 | `python3 -m r5_serial.protocol` |
| 串口 | 虚拟 STM32 + 串口桥 | 通过 | pty 联调 |
| 固件 | 协议、看门狗、阿克曼几何 | 5/5 | `firmware/openctr_b20/test/test_host.c` |

## 3. 测试方法

### 3.1 动作结果与耗时

Nav2 目标用 `ros2 action send_goal ... --feedback` 发送。只有动作返回
`SUCCEEDED` 才记为成功；最终位姿、恢复次数和仿真耗时从反馈/结果中读取。

### 3.2 定位误差

车辆停稳后，用 `tf2_echo map base_footprint` 读取当前地图位姿，再与 Gazebo
真值比较：

```text
error_xy = norm(map_xy - ground_truth_xy)
```

这里用地图变换，而不是最后一条 `/amcl_pose`。AMCL 只在滤波器更新时发布
位姿，停车后最后一条消息可能滞后一个更新周期。表中记录的是稳态误差，
不是运行过程中的最大误差。障碍绕行结束后实测约 6.7 cm。

### 3.3 代价地图检查

障碍测试直接读取全局代价地图：

```text
(4.0, 0.0)  cost 88   （障碍膨胀区）
(4.0, +0.4) cost 0    （可绕行自由区）
```

这证明障碍确实进入了代价地图，成功路径不可能是直接穿过障碍。测试对象是
Gazebo 里的**静态圆柱**，不能验证行人/车辆的运动预测。

### 3.4 TF 与里程计检查

```bash
ros2 topic info /odom -v
# 发布者数量：1，节点：sensor_fusion

ros2 run tf2_ros tf2_echo odom base_footprint
```

`odom -> base_footprint` 必须只有一个 authority。控制器配置为
`enable_odom_tf: false`，只发布 `/wheel/odom`。

融合节点用前 100 个静止 IMU 样本估计陀螺零偏，并加一个小死区。修复前，
仿真静止一段时间后航向会漂到约 18°；修复后静止时融合位姿保持在原点
0.1 m 以内。

融合 odom 按轮速 odom 的更新频率发布，不按更高的 IMU 频率发布。用 IMU
频率重复复制 50 Hz 轮速会产生重复样本和差分加速度尖峰；现在 `/odom`
保持轮速频率，同时仍给 Nav2 足够的 TF 更新。

## 4. 1.0 m/s 结果

1.0 m/s 配置使用手册/实车实测几何（轴距 0.112 m、轮距 0.164 m、轮半径
0.0325 m），并设置：

```yaml
# src/car_bringup/config/nav2_params.yaml
amcl:
  max_beams: 90
  update_min_d: 0.10
  update_min_a: 0.10

controller_server:
  FollowPath:
    vx_max: 1.0

velocity_smoother:
  max_velocity: [1.0, 0.0, 1.0]
```

最初 1.0 m/s 时 AMCL 误差约 16 cm。主要原因是 AMCL 更新阈值：1.0 m/s 下
`update_min_d: 0.25` 意味着每 0.25 s 才更新一次。把距离阈值降到 0.10 m、
把 `max_beams` 提到 90 后，稳态误差降到约 5–6 cm。几何校准和融合发布
频率修复后，实测 4 m 为 5.2 cm，6 m 为 6.0 cm。

速度链必须一致：

- MPPI 的 `vx_max`；
- `velocity_smoother.max_velocity`；
- 控制器加速度/减速度限制。

只改其中一个，会出现“规划器承诺的速度底盘做不到”或“控制器永远到不了
规划速度”。

## 5. 建图

```bash
ros2 launch car_bringup mapping.launch.py
ros2 run nav2_map_server map_saver_cli \
  -f ~/car_ws/src/car_bringup/maps/my_room_map
```

当前地图是 `src/car_bringup/maps/my_room_map.pgm|yaml`。建图和 Nav2 互斥：
两者都会发布 TF 和 `/cmd_vel`，同时启动会冲突。

## 6. 已知失败与边界

### 6.1 左房间长路径

到 `(-3.0, 0.5)` 的最短自由路径为 17.7 m；按 0.22 m 膨胀后为 18.1 m。
直线在 `(-2.0, 0.34)` 附近被墙挡住。目标 yaw 仍会改变结果：

- yaw 0：规划出 15–16 m 环线，110 s 后仍在运行；
- yaw π：能到房间附近，但最终 yaw 超出 0.8 rad 容差；
- yaw −π/2：几何校准后，从原点出发**成功，54 s，0 次恢复**。

yaw −π/2 是真实通过，但地图与真值 19 cm 已接近 20 cm 的终点容差，而且
路径仍对 yaw 敏感。正式场景应选择可行的最终朝向，并把长路径拆成
Waypoint/BT 分段，而不是发一条长目标。

### 6.2 贴墙目标

目标离墙约 0.5 m 时可能落在膨胀后的代价地图里。车体可以靠近，但规划器
认为目标不可达。应把接近点放在离墙至少 1 m 的位置，再用短距离低速动作
完成最后一段。

### 6.3 窄通道和移动障碍

窄通道压力测试尚未完成。障碍测试使用的是静态圆柱，没有验证行人运动、
传感器延迟或预测。

### 6.4 实车

目前没有实车导航结论。硬件接口、IMU 零偏标定和急停集成尚未完成。几何
参数来自手册和实车测量，但仿真仍然只是基线，不是标定证书。

URDF、`controllers.yaml`、Nav2 参数和固件主机测试现在统一使用一套实测
几何：轴距 0.112 m、轮距 0.164 m、轮半径 0.0325 m、最小转弯半径 0.55 m。
厂家固件的 0.146/0.160/0.35 已不再使用。

## 7. 复现

```bash
# 清理之前的仿真进程
ros2 run car_bringup cleanup_sim.sh

# 干净启动
ros2 launch car_bringup sim_nav2.launch.py gui:=false rviz:=false

# 前进目标
ros2 action send_goal /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback

# 控制器检查
ros2 run car_bringup acceptance_test.py --vx 0.5
ros2 run car_bringup drive_test.py --vx 0.1 --wz 0.3

# 融合与串口
ros2 run car_bringup fusion_test.py --loops 3
ros2 run r5_serial virtual_stm32
ros2 run r5_serial serial_bridge --port /dev/pts/N
```
