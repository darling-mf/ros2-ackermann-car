# 排障手册

下面几类问题在阿克曼 Nav2 栈里很容易误判。每个案例都按“现象 → 先收集什么
证据 → 根因 → 修复 → 验证”组织。命令默认已经执行
`source install/setup.bash`。

## 1. 重复启动 Nav2，导致两棵 TF 树

**现象**

- RViz 报 `No transform from [base_footprint] to [map]`；
- `Send goal call failed`，或者目标被接受但车一直不动；
- 同一条路径这次能走、下次闪烁或跳变；
- `/odom`、`/scan` 或 `/cmd_vel` 出现多个发布者。

**先收集证据**

```bash
ros2 node list | grep -E "amcl|bt_navigator|controller_server|planner_server"
ros2 topic info /cmd_vel -v
ros2 topic info /odom -v
ros2 run tf2_ros tf2_monitor map base_footprint
```

每个节点名应该只出现一次；`tf2_monitor` 中每个变换应该只有一个 authority。

**根因**

同时启动两套 Nav2（例如 `sim_nav2.launch.py` 之外又起了一个
`nav2.launch.py`）会有两个 AMCL/控制器容器。它们抢 TF 和 `/cmd_vel`，
AMCL 还会发布两套不同的 `map -> odom` 修正。即使关闭了 launch 终端，
崩溃的容器也可能残留。

**修复**

```bash
ros2 run car_bringup cleanup_sim.sh
ros2 node list | grep -E "amcl|bt_navigator|controller_server" || echo clean
```

只启动一套：

```bash
ros2 launch car_bringup sim_nav2.launch.py
```

`src/car_bringup/scripts/cleanup_sim.sh` 会清理 Gazebo、Nav2 容器、RViz、
建图和融合进程。它必须单独执行，不要和 launch 命令写在同一行。

**验证**

```bash
ros2 topic info /odom -v      # 只应有一个 sensor_fusion
ros2 run tf2_ros tf2_echo map base_footprint
```

然后发一个短距离前进目标。干净的栈应该无恢复、无 TF 警告地到点。

## 2. 激光雷达扫到车体自身

**现象**

- 空旷区域车也不走，或者走一下就停；
- RViz 在 footprint 内部出现一圈点；
- 局部代价地图在机器人中心出现 lethal/inflated 栅格。

**证据**

原始 `/scan` 检查发现 360 束里有 148 束小于 0.35 m，最近 0.100 m，集中在
前轮和后车体附近。这些点被膨胀到机器人中心，导致 MPPI 的每条轨迹都像在
碰撞。

**根因**

激光雷达安装位置能扫到转向机构和车体。只开 `footprint_clearing` 不够：
它会清理 footprint 内部已标记的栅格，但位于 footprint 边界上的点仍可能被
膨胀到中心。

**修复**

在 `src/car_bringup/config/nav2_params.yaml` 中，让两个代价地图和 AMCL
都忽略车体内部的回波：

```yaml
amcl:
  laser_min_range: 0.25

local_costmap:
  local_costmap:
    ros__parameters:
      voxel_layer:
        footprint_clearing_enabled: true
        scan:
          raytrace_min_range: 0.25
          obstacle_min_range: 0.25

global_costmap:
  global_costmap:
    ros__parameters:
      obstacle_layer:
        footprint_clearing_enabled: true
        scan:
          raytrace_min_range: 0.25
          obstacle_min_range: 0.25
```

原始传感器的 `range_min` 可以保持 0.1 m，只让消费端忽略自击点。0.25 m 大于
footprint 半对角线（`sqrt(0.14^2 + 0.11^2) ≈ 0.178 m`），同时仍能检测车体
外部的真实障碍。

**验证**

读取机器人中心所在的代价地图栅格。它应该是 `0`（free），不是 `254`
（lethal）或 `253`（inscribed）。原始 `/scan` 仍可能显示自击点，只要在进入
代价地图前被过滤掉即可。

## 3. MPPI 因 `consider_footprint` 配置崩溃

**现象**

`CostCritic.consider_footprint: true` 时，`controller_server` 在
configure/启动阶段崩溃。

**根因**

`CostCritic` 需要代价地图 footprint。如果代价地图只配置了 `robot_radius`，
没有 `footprint`，critic 拿到空多边形，进程可能段错误。

**修复**

在 `src/car_bringup/config/nav2_params.yaml` 定义一个矩形 footprint，并让
两个代价地图都使用它：

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      footprint: "[[-0.14,-0.11],[-0.14,0.11],[0.14,0.11],[0.14,-0.11]]"
```

全局代价地图保持同一个多边形。尺寸必须按实车测量。

**验证**

```bash
ros2 node list | grep controller_server
ros2 param get /controller_server FollowPath.CostCritic.consider_footprint
```

控制器应保持存活，并成功加载 CostCritic 插件。

## 4. 1.0 m/s 时 AMCL 滞后

**现象**

1.0 m/s 时车能到目标，但 AMCL 与 Gazebo 真值偏差 10–16 cm；同样地图和
控制器在 0.3 m/s 时是准的。

**根因**

AMCL 只有在移动超过 `update_min_d` 或旋转超过 `update_min_a` 时才更新。
默认 `0.25 m / 0.2 rad` 下，1.0 m/s 的车每 0.25 s 才更新一次，滤波器
永远落后于运动；60 束激光对高速修正也偏稀。

**修复**

在 `src/car_bringup/config/nav2_params.yaml`：

```yaml
amcl:
  ros__parameters:
    max_beams: 90
    update_min_d: 0.10
    update_min_a: 0.10
```

倒车仍保持 `vx_min: -0.15`。1.0 m/s 时同时确认速度链一致：MPPI 的
`vx_max`、`velocity_smoother.max_velocity` 和控制器加减速度限制要匹配。

**验证**

| 目标 | 结果 | 定位误差 |
|---|---|---|
| `(4.0, 0)` | 成功，8.0 s | 5.2 cm |
| `(6.0, 0)` | 成功，4.8 s | 6.0 cm |

转弯绕行仍可能产生更大的瞬态误差；应在车辆停稳后比较稳态值。

## 5. 倒车导致里程计和 AMCL 发散

**现象**

目标在车后时 AMCL 大幅跳变：里程计误差从厘米级涨到接近 1 m，粒子云饱和。

**根因**

旧配置允许 `vx_min: -0.4` 和长距离倒车弧线。倒车打滑时的轮式里程计误差
叠加较大的 `map -> odom` 修正，形成反馈回路。规划器还会用纯 Dubin 基元
生成很长的大环。

**修复**

使用 Reeds-Shepp、惩罚倒车，并限制倒车速度：

```yaml
planner_server:
  GridBased:
    motion_model_for_search: REEDS_SHEPP
    reverse_penalty: 2.0

controller_server:
  FollowPath:
    vx_min: -0.15
```

阿克曼行为树去掉原地 `Spin`，改用有界的 `BackUp` 或
`DriveOnHeading`。

**验证**

- 短距离倒车目标不应引起 AMCL 跳变；
- `/odom` 与 `/ground_truth` 的漂移应在预期范围内；
- 倒车后必须还能继续前进。已单独测试：倒车约 −0.8 m，再前进到 +0.5 m，
  8 s 成功。

## 6. 静止时融合里程计航向漂移

**现象**

车没动，但 `/odom` 缓慢旋转，`y` 位置也漂。AMCL 被迫吸收这部分误差，
地图位姿看起来比轮式里程计还差。

**根因**

陀螺存在零偏。直接积分 `angular_velocity.z` 而不估计零偏，即使车静止也会
累积航向误差。仿真测试中，长时间静止后融合航向漂了约 18°。

**修复**

`sensor_fusion.py` 用前 `gyro_bias_samples` 个静止样本（默认 100）估计
陀螺零偏，并加一个小死区 `gyro_deadband`（默认 0.0005 rad/s）：

```python
if not bias_ready and abs(v) < stationary_speed and abs(wz) < 0.05:
    bias_sum += raw_wz
    bias_count += 1
    if bias_count >= bias_samples:
        gyro_bias = bias_sum / bias_count
        bias_ready = True
```

启动后车体需要静止约 2 秒；实车在这个窗口内不要推车。

**验证**

静止时 `/odom` 应保持在原点几厘米内，yaw 接近 0。开启零偏标定后，仿真
停车漂移小于 0.1 m。

另一个坑是发布频率：用 IMU 频率发布融合 `/odom`，却复制较慢的轮速
twist，会产生重复值然后跳变，差分出虚假的加速度尖峰。应每个轮速 odom
更新只发布一次融合样本，或者对 twist 做插值。

## 7. 通用排查顺序

1. 数据链：`/clock`、`/scan`、`/odom`、`/tf`、`/cmd_vel` 的发布者数量。
2. TF：`tf2_echo map base_footprint`；区分“数据缺失”和“RViz 显示问题”。
3. 单实例：只有一个 AMCL、一个控制器、一个规划器。
4. 定位：随时间对比 `/amcl_pose` 和 `/ground_truth`。
5. 代价地图：直接读机器人中心栅格值，不要只看颜色。
6. 目标可达性：检查目标是否落在膨胀区内部。
7. 控制器：检查 `vx_min`、footprint 和 MPPI critic 配置。
8. 地图：某个房间持续失败时，先重建地图，再改权重。

## 8. 常用命令

```bash
# 节点和话题
ros2 node list
ros2 topic info /cmd_vel -v
ros2 topic info /odom -v

# TF
ros2 run tf2_ros tf2_echo map base_footprint
ros2 run tf2_ros tf2_monitor map base_footprint

# 代价地图中心（用 Python/rclpy；方法见 docs/VALIDATION.md）
# 中心栅格 0 表示 free；253/254 表示 inscribed/lethal。

# 清理
ros2 run car_bringup cleanup_sim.sh
```
