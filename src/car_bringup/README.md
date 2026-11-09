# car_bringup

R5 阿克曼小车的启动文件、参数、地图和主机侧测试工具。这个包是仿真和导航的
入口。

## 启动文件

| 文件 | 用途 |
|---|---|
| `launch/gazebo.launch.py` | Gazebo + `ros2_control` + 传感器融合；可选 RViz |
| `launch/sim_nav2.launch.py` | 一条命令启动 Gazebo + Nav2 + RViz |
| `launch/nav2.launch.py` | 只启动 Nav2，用于已经在运行的 Gazebo/实车 |
| `launch/mapping.launch.py` | Gazebo + `slam_toolbox`；与 Nav2 互斥 |
| `launch/fusion.launch.py` | 独立融合验证；只启动一个显式融合节点 |

同一时间只能有一个进程发布 `/cmd_vel`。切换模式前先运行
`ros2 run car_bringup cleanup_sim.sh`。

## 配置

| 文件 | 内容 |
|---|---|
| `config/controllers.yaml` | 轮距/轴距、关节顺序、`ros2_control` 限幅、里程计协方差、融合 TF 归属 |
| `config/nav2_params.yaml` | AMCL、代价地图、Smac Hybrid、MPPI、速度平滑器 |
| `config/slam_toolbox.yaml` | 2D 建图参数 |
| `maps/my_room_map.yaml` | 默认仿真地图（376 × 222，0.05 m） |

关键关系：

- `controllers.yaml` 设置 `enable_odom_tf: false`；`sensor_fusion.py` 是
  唯一的 `odom -> base_footprint` 广播者。
- `nav2_params.yaml` 使用 `SmacPlannerHybrid` + `REEDS_SHEPP`，
  `minimum_turning_radius: 0.55`；MPPI 使用 `motion_model: Ackermann`。
- MPPI 的 `vx_max`、`velocity_smoother.max_velocity` 和控制器加减速度限制
  属于同一条速度链，必须一起改。
- 1.0 m/s 配置下，AMCL 使用 `max_beams: 90`、`update_min_d: 0.10`、
  `update_min_a: 0.10`。
- `sensor_fusion.py` 用 100 个静止样本估计陀螺零偏并加死区；启动后保持
  车体静止约 2 秒。
- 融合 `/odom` 按轮速 odom 的更新频率发布。不要用更高的 IMU 频率重复
  复制轮速 twist，否则差分会出现虚假的加速度尖峰。
- 几何参数为实测 R5 数据：轴距 0.112 m、轮距 0.164 m、轮半径 0.0325 m、
  最小转弯半径 0.55 m。

## 脚本

| 脚本 | 用途 |
|---|---|
| `scripts/acceptance_test.py` | 直行 / 转向 / 倒车控制器验收 |
| `scripts/drive_test.py` | 方向、转角、轮速检查 |
| `scripts/path_tracker.py` | Pure Pursuit / Stanley / LQR 对比；不属于 Nav2 |
| `scripts/sensor_fusion.py` | 轮速 + IMU 融合；发布 `/odom` 和 TF |
| `scripts/fusion_test.py` | 与 `/ground_truth` 对比的融合误差 |
| `scripts/plan_inspector.py` | 检查 Nav2 路径里的倒车段 |
| `scripts/cleanup_sim.sh` | 停止残留的 Gazebo/Nav2/RViz/建图/融合进程 |

## 快速检查

```bash
ros2 run car_bringup acceptance_test.py --vx 0.5
ros2 run car_bringup drive_test.py --vx 0.1 --wz 0.3
ros2 run car_bringup fusion_test.py --loops 3
```

如果 Nav2 接受了目标但车不动，先检查实例数量和 TF：

```bash
ros2 node list | grep -E "amcl|bt_navigator|controller_server"
ros2 run tf2_ros tf2_echo map base_footprint
```

系统层面的说明见仓库根目录的
[架构文档](../../docs/ARCHITECTURE.md)、
[验证记录](../../docs/VALIDATION.md) 和
[排障手册](../../docs/TROUBLESHOOTING.md)。
