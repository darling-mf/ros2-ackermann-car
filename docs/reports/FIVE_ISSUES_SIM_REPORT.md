# 五项问题复测报告：原因、修复与工程常见度

> 日期：2026-11-08
> 平台：TARKBOT R5 阿克曼车，ROS 2 Humble + Gazebo Classic 11
> 地图：`src/car_bringup/maps/my_room_map.yaml`（376×222 @ 0.05m）
> 导航：Smac Hybrid（Reeds-Shepp）+ MPPI（Ackermann）+ 自定义阿克曼 BT
> 定位：融合 odom（`/wheel/odom` + `/imu` → `/odom`）+ AMCL
> 测试入口：`ros2 launch car_bringup sim_nav2.launch.py gui:=false rviz:=false`

## 0. 结论先行

| # | 问题 | 仿真判定 | 根因层级 | 实机是否常见 |
|---|---|---|---|---|
| 1 | 左房间/长路径 | 🟡 留实机 | 地图连通性 + 18m 长路径 + 目标 yaw | 很常见 |
| 2 | 动态障碍/窄通道 | 🟡 部分通过 | 障碍层 + 终点可达性 + 局部极小 | 很常见 |
| 3 | 1 m/s 高速 | ✅ 通过 | AMCL 更新率/波束 + 速度链 | 很常见 |
| 4 | 融合默认 odom | ✅ 通过 | TF 唯一性 + 轮速/陀螺融合 | 标准做法 |
| 5 | 新图→Nav2 闭环 | 🟡 留实机 | 地图质量/初始位姿/场景 | 很常见 |

**一句话**：这五件事里，真正需要改代码的是第 3、4 项；第 1、2、5 项主要是
**地图、目标设计和行为树**问题，继续在仿真里拧 MPPI 参数收益很低。

> 说明：本项目说的“动态障碍”测试，目前是在 Gazebo 里生成一个**静止圆柱**
> 来模拟障碍物，不是会移动的行人/车辆。它能验证“感知→costmap→绕行”的
> 最小闭环，但不能验证运动预测。这个边界必须写清楚，避免把“静态绕障”
> 误当成“动态避障”。

---

## 1. 左房间/长路径：不是控制器坏了，是“路太长 + 终点朝向不可达”

### 1.1 现象

目标 `(-3.0, 0.5)`，从原点出发：

- yaw=0：规划出 15–16m 环线，110s 仍在执行，车跑到 x≈−9；
- yaw=π：能回到 `(-3.36,0.48)` 附近，但终态 yaw≈−1.37，超过 0.8 容差，超时；
- yaw=−π/2：从目标附近 `(-3.36,0.48)` 出发，短距离 **SUCCEEDED**；
- 从原点直接发 yaw=−π/2：180s 仍在绕行，车到 `(-9.16,0.245,yaw≈1.74)`。

### 1.2 数据链

1. 对地图做 BFS：原始 free 最短路径 **17.7m**；按 0.22m 膨胀后 **18.1m**。
2. 起点到目标的直线在 **(-2.0,0.34)** 附近被墙挡住，必须绕到左侧房间。
3. Nav2 的全局路径本身是通的，所以不是“规划器找不到路”；问题是
   **路径长度 × 局部控制器跟踪误差 × 终点姿态** 的累积。
4. 目标 yaw=0 时，进入左房间后要掉头/原地转向，而阿克曼不能原地转；
   BT 里已去掉 `Spin`，只能靠前进/倒车组合，末端很容易在目标附近打转。
5. yaw=−π/2 时，目标朝向与房间入口方向一致，短距离验证就成功；
   说明“目标朝向选对”比继续调 MPPI 更有效。

### 1.3 根因

- **地图连通性**：左房间与主区域之间只有特定门/通道，最短路径 18m；
  地图如果有缺失/重影，SMAC 会生成更长的绕行。
- **目标姿态不可达**：`yaw_goal_tolerance=0.80` 已经比较宽，但目标点
  本身在狭窄区域，阿克曼的最小转弯半径 0.35m 使末端调整空间不足。
- **长路径累积误差**：AMCL/odom 的航向误差沿 18m 放大，MPPI 的局部
  预测时域只有约 2.8s，无法“看见”整条长路径。
- **单条长目标缺少中间约束**：一条 `NavigateToPose` 从原点直达左房间，
  中间没有“先到门口、再进房间”的分段，失败后恢复行为也难判断该退哪。

### 1.4 方案（与你的实机计划一致）

优先用 **BT/Waypoint 分段 + 可行 yaw**，不要继续在仿真里硬调 MPPI：

1. **重建/确认地图**：覆盖左右房间、门洞完整；用 `map_server` 确认
   `resolution/origin/width/height`；在 RViz 里检查门洞是否被膨胀层封死。
2. **分段**：把 18m 拆成 1–3m 的中间点（倒车段建议 ≤0.6m），每段一个
   `NavigateToPose` 或 `FollowWaypoints`；中间点放在通道中心，不要贴墙。
3. **每段给可行 yaw**：进左房间时让车头朝入口方向（例如 −π/2），
   不要在死胡同里要求 yaw=0；末端容差可先用 0.8，稳定后再收紧。
4. **BT 层处理失败**：每段失败只重试该段，不要整条 18m 重规划；
   恢复用 `BackUp`/`DriveOnHeading`，不要用 `Spin`（阿克曼不能原地转）。
5. **验证顺序**：原点→门口（1–2m）→门内（1–2m）→目标；每段记录
   `SUCCEEDED/耗时/终态 yaw`，确认哪一段开始失败。

示例（概念，不是现成 BT 文件）：

```text
Sequence
├── NavigateToPose(门口, yaw=入口方向)
├── NavigateToPose(房间内中间点, yaw=入口方向)
└── NavigateToPose(目标, yaw=-π/2)
```

Humble 里可以直接用 `waypoint_follower` 的 `FollowWaypoints` 做同样的事；
如果要条件恢复/重试，再把它包进自定义 BT。

### 1.5 复测判据

- 每段独立成功，整条链路不依赖一次 18m 规划；
- 到点 yaw 在容差内，且不需要 `Spin`；
- 失败时能指出“哪一段、为什么”，而不是只看到整条 ABORTED。

### 1.6 工程常见度

**很常见。** 室内阿克曼在长走廊 + 窄门口最容易出现：

- 全局路径通、局部控制器在拐角/门口抖；
- 目标 yaw 不可达，末端反复调整；
- 地图门洞被膨胀层封死，规划器绕远路。

真实机器人通常比仿真更容易解决，因为可以人工录 waypoint、现场调整
目标朝向；但地图噪声和轮速滑移会更明显。你的“BT 分段 + yaw”方向是对的。

---

## 2. 动态障碍/窄通道：能绕，但“终点贴墙”会把成功变成超时

### 2.1 两个场景，两种结果

| 场景 | 障碍 | 目标 | 结果 | 终态 |
|---|---|---|---|---|
| A 近墙终点 | (6.5,0)，r=0.20m | (8.0,0) | 60s 超时 | (7.633,0.386,yaw −0.502) |
| B 终点离墙 | (4.0,0)，r=0.20m | (6.0,0) | **SUCCEEDED，12s，0 恢复** | (5.779,−0.028) |

场景 B 的 costmap 证据：全局代价地图在 `(4.0,0)` 的 cost=88，
`(4.0,+0.4)` 为 0；说明障碍确实进入了代价地图，路径必须从左侧绕行。
12s 是从静止加速、绕行到 6m 的总时间，不能和无障碍时 2.2m 后段的
4.7s 直接比较；但“障碍在 costmap 内 + 动作成功 + 0 恢复”已能证明
不是直穿。

### 2.2 根因链

**场景 A 为什么超时：**

1. 目标 `(8.0,0)` 距右墙（约 x=8.47）只有约 0.5m；
2. 车体 footprint + `inflation_radius=0.22` 后，目标附近几乎没有
   free cell，终点本身就在膨胀区/不可达区；
3. MPPI 到了 `(7.63,0.39)` 后，前方是墙+膨胀区，局部轨迹大量碰撞，
   进入局部极小；BT 的恢复也难把车“推”进一个不可达终点；
4. 这不是“不会避障”——车已经绕过了 (6.5,0) 的圆柱，失败点是
   **终点不可达 + 末端姿态收敛**。

**场景 B 为什么成功：**

1. 目标离墙 ≥1m，目标周围有足够 free space；
2. 障碍在 (4.0,0)，左右都有绕行空间；
3. 全局规划器生成绕行路径，MPPI 跟踪；12s 完成，0 恢复。

**窄通道为什么更难：**

- 车宽 + 两侧膨胀后，free corridor 可能只剩几十厘米；
- MPPI 的采样轨迹在窄通道里容易全部碰撞，进入“无可行解”；
- 阿克曼不能横移，窄通道内没有倒车/转向余量时，局部极小更严重。

### 2.3 方案

1. **先保证终点可达**：目标离墙 ≥1m；实在要贴墙，先到“接近点”，
   再用低速/倒车/专用行为完成最后 0.5m。
2. **分段**：把“绕过障碍”和“到终点”拆成两个 waypoint/BT 段；
   绕行段用较宽的中间点，终点段再收敛。
3. **代价地图**：确认障碍层（`obstacle_layer`/`voxel_layer`）真的把
   障碍写进 global/local costmap；检查 `inflation_radius` 与 footprint；
   窄通道不要为了通过而无限缩小膨胀，先降速。
4. **局部控制器**：适当调 `CostCritic`、`PathAlignCritic`、`PathFollowCritic`；
   如果 MPPI 在窄通道持续无解，换 RPP/TEB 做对比，而不是只调一个权重。
5. **恢复行为**：用 `BackUp`/`DriveOnHeading`/`Wait`，不要 `Spin`；
   对“终点不可达”要能早失败，而不是耗到 60s。
6. **真正的动态障碍**：本次只是静态圆柱。要验证移动障碍，需要
   障碍物会动 + 感知延迟 + 预测；可加 `collision_monitor` 做安全层，
   或使用带障碍预测的局部规划器。

### 2.4 复测判据

- 终点离墙 ≥1m 的静态障碍：`SUCCEEDED`、无碰撞、耗时合理；
- 近墙终点：要么成功，要么在合理时间内明确 ABORTED，不能 60s 无结论；
- 窄通道：能通过或明确判定“几何不可通过”，并给出最小宽度数据。

### 2.5 工程常见度

**非常常见。** 这是 Nav2 现场调试的高频问题：

- 目标点贴墙/贴障碍，用户以为“到了”，规划器认为“不可达”；
- 全局路径绕障成功，局部控制器在障碍前进入局部极小；
- 窄通道里 footprint/膨胀参数把通道封死。

真实机器人通常**更难**：雷达噪声、盲区、延迟、轮速滑移、行人运动都会
放大问题；仿真里能绕，不代表实车能绕。实车应先降速、加安全层，再逐步提速。

---

## 3. 1 m/s 高速：不是只改 `vx_max`，AMCL 更新率才是定位误差主因

### 3.1 原始问题

把 Nav2 限速从 0.3 m/s 提到 1.0 m/s 后，车能跑，但 AMCL 与 Gazebo 真值
偏差约 16cm；感觉“高速时定位跟不上”。

### 3.2 数据链

| 阶段 | 目标 | 结果 | 用时 | AMCL vs GT |
|---|---|---|---|---|
| 旧 AMCL 参数 | (4.0,0) | SUCCEEDED | 11s | ≈16cm |
| 运行时调参后 | (6.0,0) | SUCCEEDED | 7s | ≈7.7cm |
| 参数写回 YAML 后干净重启 | (4.0,0) | SUCCEEDED | 8.2s | ≈5.4cm |
| 同上 | (6.0,0) | SUCCEEDED | 4.7s | ≈5.5cm |

复测末态（停车后读数）：

- 4m：AMCL `(3.793,0.011)` vs GT `(3.827,0.053)`；
- 6m：AMCL `(5.822,0.012)` vs GT `(5.828,0.067)`。

### 3.3 根因

1. **AMCL 更新阈值**：默认 `update_min_d=0.25m`、`update_min_a=0.2rad`。
   1 m/s 时每 0.25s 才更新一次滤波器；0.3 m/s 时是 0.83s 一次。
   速度越快，同样的“距离阈值”对应的时间越短，但粒子更新仍落后，
   表现为定位滞后/误差大。
2. **波束数**：`max_beams=60` 在高速下采样太稀；增加到 90 后，
   每次更新的观测更充分，误差下降。
3. **速度链不一致**：`vx_max`、`velocity_smoother.max_velocity`、
   MPPI `vx_max` 必须一起改；只改一个会出现“规划 1.0、执行 0.3”
   或“控制器突变”。
4. **融合 odom 的质量**：轮速+陀螺融合比纯轮式航向好，但陀螺零偏/
   轮径误差仍会进入 AMCL 运动模型。
5. **不是控制器 bug**：控制器能跟上 1 m/s；问题在定位更新频率和
   高速下的观测/运动模型匹配。

### 3.4 修复

已写回 `src/car_bringup/config/nav2_params.yaml`：

```yaml
amcl:
  ros__parameters:
    max_beams: 90
    update_min_a: 0.10
    update_min_d: 0.10

controller_server:
  ros__parameters:
    FollowPath:
      vx_max: 1.0

velocity_smoother:
  ros__parameters:
    max_velocity: [1.0, 0.0, 1.0]
```

同时保留：

- `vx_min: -0.15`（倒车仍慢速，避免 odom/AMCL 发散）；
- `r5_steering_controller` 的加速度斜坡（`max_accel=0.80`、
  `max_decel=1.50`）和横向加速度限制；
- `min_turning_r=0.35` 与 Smac 的 `minimum_turning_radius` 一致。

### 3.5 复测判据

- 4m/6m 直线目标 `SUCCEEDED`、0 恢复；
- 停车后 AMCL 与真值差 <10cm；
- 速度平滑器输出不超 1.0 m/s，转向不抖。

### 3.6 工程常见度

**很常见。** 高速调试的典型顺序是：

1. 先确认底盘能跟上（控制器/电机/转向速率）；
2. 再确认定位更新率（AMCL 的 `update_min_d/a`、`max_beams`）；
3. 再调局部控制器（MPPI 采样、critic 权重）；
4. 最后才谈“高速绕障”。

实车注意：

- 1 m/s 的刹车距离、雷达盲区、控制延迟都比仿真大；
- 先 0.5–0.7 m/s 验证，再逐级到 1.0；
- 保留硬件急停/软件 `collision_monitor`；不要只依赖 Nav2 的恢复行为；
- 轮径/IMU 标定不准时，高速误差会被放大，先标定再提速。

---

## 4. 融合默认 odom：三处改动 + 唯一 TF，仿真已闭环

### 4.1 为什么必须做

实车只有轮速 + IMU，没有 Gazebo 的完美轮式里程计。融合节点把：

- 轮速 → 位置增量（不受陀螺零偏影响）；
- 陀螺 → 航向（不受前轮刮地/轮径误差影响）；

组合成 `/odom`，并成为唯一的 `odom→base_footprint` 广播者。
这样 Nav2/AMCL 只认一个 odom 源，不会出现两棵 TF 树。

### 4.2 改动

1. `src/car_description/urdf/car/control.xacro`：

```xml
<remapping>/r5_steering_controller/tf_odometry:=/tf_wheel</remapping>
<remapping>/r5_steering_controller/odometry:=/wheel/odom</remapping>
```

2. `src/car_bringup/config/controllers.yaml`：

```yaml
enable_odom_tf: false   # 由 sensor_fusion 发布唯一 odom→base_footprint
```

3. `src/car_bringup/scripts/sensor_fusion.py`：

- 订阅 `/wheel/odom` + `/imu`；
- 发布 `/odom`、`/odometry/fused`、`/odometry/gyro_only`；
- 发布 `odom→base_footprint` TF。

4. `src/car_bringup/launch/gazebo.launch.py`：

```python
declare_use_fusion = DeclareLaunchArgument(
    'use_fusion', default_value='true',
    description='Start sensor fusion (default odom source)')
```

`fusion.launch.py` 仍保留为独立验证入口：它包含的 Gazebo 用
`use_fusion:='false'`，再显式启动一个融合节点，避免重复。

### 4.3 验证

```bash
ros2 topic info /odom -v
# Publisher count: 1, Node name: sensor_fusion

ros2 run tf2_ros tf2_echo odom base_footprint
# 有连续变换，无重复 authority
```

- Nav2 前向 `(2.0,0)`：8s `SUCCEEDED`；
- 1 m/s `(4.0,0)/(6.0,0)`：8.2s/4.7s `SUCCEEDED`；
- AMCL 误差 5.4–5.5cm。

### 4.4 风险与实机注意

- **陀螺零偏**：本次干净启动后，车静止约 1500s（仿真时间），融合航向
  从 0 漂到约 0.122rad（7°）。短时间导航影响小，但长时间停车/实车
  MPU6050 零偏会更明显。实车必须做静置零偏标定；可加“静止死区”或
  启动时平均零偏。
- **轮径/轮距**：轮速积分的位置误差直接进入 odom；实车先标定。
- **协方差**：当前融合节点的协方差比较简单；如果要给 Nav2/AMCL
  更规范的置信度，换 `robot_localization` EKF。
- **TF 唯一性**：任何时候只能有一个节点发 `odom→base_footprint`；
  改 launch 后先 `tf2_monitor`，再看 Nav2 是否正常。

### 4.5 工程常见度

**标准做法。** 移动机器人通常都是“轮速 + IMU → EKF/互补滤波 →
`/odom` + TF”；关键不是算法多复杂，而是：

- 坐标系/时间戳统一；
- TF 只有一个广播者；
- 轮径/IMU 标定；
- 断流/超时保护。

---

## 5. 新图→Nav2 闭环：你的判断是对的，剩余问题留实机

### 5.1 现状

你已经用自己的地图跑过 Nav2：除一个房间外基本正常。这个信息很重要：
说明 **map_server、AMCL、TF、Nav2 主链没有系统性问题**；剩余是
“某个房间”的地图/场景问题。

### 5.2 为什么只剩一个房间不稳定

按可能性排序：

1. **地图质量**：该房间门洞缺失/重影/家具动态残留，导致全局路径绕远
   或被膨胀层封死；
2. **初始位姿**：AMCL 初始位姿与新图原点不一致，车一开始就“以为自己在
   别的位置”，进房间后误差累积；
3. **长路径/目标 yaw**：和左房间相同，单条长目标 + 不可达末端姿态；
4. **窄通道**：门宽接近车宽+膨胀，MPPI 局部无解；
5. **动态障碍**：房间内有人/家具移动，costmap 短时被占。

这些都不是“Nav2 坏了”，所以继续在仿真里换图/换参数，收益有限。

### 5.3 建议的实机闭环顺序

1. 用实车雷达建图，保存 `new_map.pgm/yaml`；
2. 检查 `map_server` 加载的尺寸/origin，和建图时一致；
3. 在 RViz 用 2D Pose Estimate 给初始位姿，确认 `map→odom` 合理；
4. 先发房间外短目标，再发房间内短目标，最后才发跨房间长目标；
5. 失败时记录：全局路径是否存在、costmap 门洞是否 free、AMCL 粒子
   是否收敛、目标 yaw 是否可达；
6. 需要时用 Waypoint/BT 分段，不要用一条长目标硬闯。

### 5.4 工程常见度

**很常见。** “换一张图就有一个房间不行”是建图/定位/场景问题，
不是导航栈的普遍 bug。实机复测比仿真更有意义，因为实车地图才是最终地图。

---

## 6. 五个问题之间的共同规律

1. **先分层，再调参**：控制、定位、规划、地图、行为树是不同层；
   用错层的参数会“看起来有效、换场景就坏”。
2. **终点可达性是 Nav2 的第一性问题**：目标贴墙、贴障碍、yaw 不可达，
   会让一个正常的规划/控制栈表现为“不会动”。
3. **阿克曼没有原地转向**：`Spin` 类恢复不要用；长路径/窄空间要用
   前进+倒车组合或分段。
4. **TF 唯一性是定位的生命线**：融合默认 odom 后，控制器必须停止发 TF。
5. **仿真能过 ≠ 实车能过**：雷达噪声、延迟、轮速滑移、行人运动、
   刹车距离都要在实车重新标定。

## 7. 本次实际改动清单

| 文件 | 改动 |
|---|---|
| `src/car_bringup/config/nav2_params.yaml` | `vx_max=1.0`、`max_velocity=[1.0,0,1.0]`；AMCL `max_beams=90`、`update_min_d/a=0.10` |
| `src/car_bringup/config/controllers.yaml` | `enable_odom_tf: false`；确认几何/限幅 |
| `src/car_description/urdf/car/control.xacro` | odom/TF remap 到 `/wheel/odom`、`/tf_wheel` |
| `src/car_bringup/scripts/sensor_fusion.py` | 订阅 `/wheel/odom`；发布 `/odom` + 唯一 TF |
| `src/car_bringup/launch/gazebo.launch.py` | `use_fusion` 默认 true |
| `src/car_bringup/launch/fusion.launch.py` | 避免重复融合节点 |
| `src/car_bringup/scripts/fusion_test.py` | 测试源改为 `/wheel/odom` |
| `docs/SIM_TODO.md`、`docs/PROJECT_STATE.md`、`docs/ARCHITECTURE.md`、`README.md` | 状态/数据/边界同步 |

## 8. 尚未完成 / 下一步

- 左房间：按你的计划实机用 BT 分段 + yaw，不在仿真继续硬调；
- 动态障碍：本次是静态圆柱；下一步用会移动的障碍 + 真实延迟复测；
- 窄通道/死胡同：需要构造场景，记录最小可通行宽度；
- 新图闭环：实机用实车地图复测；
- 实车：S6 固件烧录 → S8 标定（轮径/轴距/IMU）→ S7 `SystemInterface`
  → S9 集成；融合 odom 需在实车重新标定后再判定。

---

## 附：复现命令

```bash
# 清理
ros2 run car_bringup cleanup_sim.sh

# 干净启动（融合默认、1 m/s 参数）
ros2 launch car_bringup sim_nav2.launch.py gui:=false rviz:=false

# 1 m/s 前向
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 4.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" --feedback

# 动态障碍最小场景（先 spawn 圆柱到 (4,0)，再发目标 (6,0)）
# 注意：这是静态障碍，不是移动行人
```
