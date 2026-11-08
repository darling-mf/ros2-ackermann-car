# Nav2 导航闪烁 / TF 报错 / 自击代价地图 解决报告

> 日期：2026-10-29  
> 环境：Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11.10.2 + VMware  
> 机器人：R5 阿克曼小车（轴距 0.112 m，轮半径 0.0325 m，footprint 0.28×0.22 m）  
> 地图：`src/car_bringup/maps/my_room_map.yaml`（376×222 @ 0.05 m，origin −10.3/−6.46）  
> 目标：复现并解决 Nav2 导航时的闪烁/TF 报错/定位丢失，以及运动过程中机器人脚下出现代价地图导致“自己卡死自己”；恢复 Nav2 的可视化。

---

## 0. 结论摘要（先看这）

这不是一个 bug，而是**四层问题叠加**；第二份问题则是另一个独立的“传感器看到自己”的问题。

### 问题一：Nav2 闪烁 / TF 报错 / 丢定位

按发现顺序，四层根因：

1. **现场是“孤儿 Nav2”**：只有 Nav2，没有 Gazebo/控制器，`/scan`、`/odom`、`/clock` 发布者都是 0，TF 链必然断。此时 RViz 报 TF 错是结果，不是根因。
2. **RViz 默认视图引用了 TurtleBot 的帧**：`nav2_bringup` 的 `nav2_default_view.rviz` 里写死了 `camera_link`、`caster_back_*`、`wheel_left/right_link`，本车没有这些帧，所以即使完整栈在跑，RViz 的 TF 显示仍然报“缺帧”。
3. **重复 Nav2 容器 + 倒车导致定位爆炸**：`cleanup_sim.sh` 的 `pkill -x component_contai` 与实际进程名 `component_conta` 不匹配，Nav2 容器残留；同时目标在车后时 MPPI 以 **−0.4 m/s** 倒车，控制器 odom 与 Gazebo 真值误差到 **0.84 m**，AMCL 估计从 0.07 m 偏到 **7.5 m**，粒子打满 2000，恢复 18 次后 ABORTED。
4. **控制/规划参数不匹配**：Nav2 速度上限 1.0 m/s 远高于 R5 底盘和室内场景；`vx_min: 0.0` 会让 MPPI 在静止起步时直接 `Optimizer fail to compute path`；纯 DUBIN 对“车后目标”会规划 15–16 m 长环线，在起步急转处同样无解。

**已修**：cleanup 脚本、`sim_nav2`/`mapping` 的 `rviz:=false` 泄漏、专用 `nav2.rviz`、软件渲染、速度 0.3 m/s、慢倒车 −0.15 m/s、REEDS_SHEPP + reverse_penalty 2.0、min_turning_r 0.35、inflation 0.25、vx_min −0.15、MPPI 起步可解。

**已验证**：前进目标 `(2.0, 0)` **SUCCEEDED，0 次恢复**，AMCL 与真值差 **3–5 cm**；TF 链路正常；RViz 导航面板/路径/代价地图可见。

**仍未解决**：左房间目标 `(-3.0, 0.5)` 会规划出 15–16 m 绕行路径，跟踪/恢复后 ABORTED；AMCL 不再发散（误差 0.1–0.3 m），问题转到**地图左房间连通性/路径**和终点朝向。建议按 `SIM_TODO.md` §10 重建一张完整地图，再测左房间；必要时换 RPP/TEB 或放宽 `yaw_goal_tolerance`。

### 问题二：机器人脚下出现代价地图 / 自己卡死自己

**实测 `/scan`：360 条里有 148 条 < 0.35 m，最近 0.100 m，角度集中在 ±69°、±110°、±135°、180°——激光在扫自己的前轮/转向机构/后车体。**

原配置 local/global 的 scan 源：

```yaml
raytrace_min_range: 0.0
obstacle_min_range: 0.0
```

0.1–0.17 m 的自身回波被当成障碍标记，`inflation_radius: 0.25` 再膨胀到车体中心。MPPI 认为“起步就在障碍里”，于是原地不动、反复恢复。

**修复**：两处 scan 源 `raytrace_min_range`/`obstacle_min_range` 改为 **0.25**；两层显式 `footprint_clearing_enabled: true`；AMCL `laser_min_range: 0.25`。  
**验证**：重启 Nav2 后 `LOCAL center=0`、`GLOBAL center=0`，机器人中心 ±5 格全为 free，没有 `#`/`+`；脚下粉红圈消失。

> 关键结论：不是单纯“自清洁开关被关掉”。`footprint_clearing_enabled` 只清足迹内部，边界点/全局层/膨胀层仍可能覆盖车体；更直接的是**别把足迹内的回波喂给代价地图**。

---

## 1. 复现环境与初始现象

### 1.1 环境与关键文件

| 项目 | 内容 |
|---|---|
| 地图 | `src/car_bringup/maps/my_room_map.yaml`（376×222 @ 0.05，origin −10.3/−6.46） |
| 地图发布 | `nav2_map_server`，`use_sim_time: true`，`/map` transient local |
| 定位 | `nav2_amcl`，`set_initial_pose: true`，初始 `(0,0,0)` |
| 控制 | `r5_steering_controller`（自研阿克曼），`enable_odom_tf: true` |
| 底盘真值 | `gazebo_ros_p3d` → `/ground_truth`（不参与控制，只做对比） |
| 规划 | `SmacPlannerHybrid` |
| 控制 | `MPPIController`（Ackermann） |
| 可视化 | 原 `nav2.launch.py` 使用 `nav2_bringup/rviz/nav2_default_view.rviz`；现改为 `car_bringup/rviz/nav2.rviz` |

### 1.2 用户描述的现象

- Gazebo 里小车本身没问题；
- 用 Nav2 跑导航时出现“闪烁或者其他现象”；
- RViz 里的 TF 树会报错；
- 需要 Nav2 里的导航可视化；
- 行动过程中机器人自己位置出现代价地图，车走不动，怀疑代价地图“自清洁”被禁用。

### 1.3 第一次现场检查：先别猜 TF，先看数据链

第一次检查时发现：

```
/clock:  Publisher count: 0
/scan:   Publisher count: 0
/odom:   Publisher count: 0
/tf:     Publisher count: 1（只有 amcl）
```

也就是说，当时**只有 Nav2，没有 Gazebo/控制器**。没有 `/scan`、`/odom`、`/clock`，TF 链 `map→odom→base_footprint→base_link` 不可能完整，RViz 报 TF 错是必然结果。

**这一步的思维链**：

- 看到“TF 报错”，第一反应容易去调 TF 参数；
- 但先检查“谁在发布 TF/传感器/时钟”，发现 Gazebo 根本没跑；
- 所以第一步不是修 TF，而是**恢复完整数据链**，并在完整链路上再复现。
- 同时发现 `cleanup_sim.sh` 的容器清理拼写错误，Nav2 容器会残留，这会制造“看起来像 TF 冲突”的假象。

---

## 2. 问题一：Nav2 闪烁 / TF 报错 / 导航可视化

### 2.1 排查链（按时间顺序）

#### Step 1：完整栈下的 TF 到底通不通？

启动完整栈（headless Gazebo + Nav2）后检查：

```
/tf Publisher count: 3
  - amcl
  - r5_steering_controller
  - robot_state_publisher
```

这是**正常**的：

- `amcl` 发 `map→odom`；
- `r5_steering_controller` 发 `odom→base_footprint`；
- `robot_state_publisher` 发动关节的 `base_footprint→base_link→…` 与 `laser_link` 等。

实测：

```
tf2_echo map base_link      → (0.000, 0.000, 0.032)
tf2_echo odom base_footprint → (0.000, 0.000, 0.000)
```

初始 AMCL 与 Gazebo 真值：

```
AMCL: (0.000, 0.000, 0.000)
GT:   (0.006, -0.0015, -0.007 rad)
```

误差约 7 mm。**完整栈下 TF 数据层是通的。**

**结论**：用户看到的 “TF 树报错” 不是 TF 数据断了，而是 RViz 的显示配置引用了不存在的帧。

#### Step 2：RViz 的 TF 显示为什么报错？

对比 `nav2_default_view.rviz` 与本车 URDF：

- 默认视图的 TF `Frames` 和 `RobotModel` 里有：
  - `camera_link`、`camera_depth_frame`、`camera_rgb_frame`
  - `caster_back_left_link`、`caster_back_right_link`
  - `wheel_left_link`、`wheel_right_link`
  - `base_scan`
- 本车实际只有：
  - `base_footprint`、`base_link`
  - `laser_link`、`sonar_link`、`imu_link`
  - `left/right_front_steer_link`、`left/right_front_wheel_link`
  - `left/right_rear_wheel_link`

默认视图是 TurtleBot 的帧树，RViz 会一直报“找不到 camera_link / caster_* / wheel_*”。

**解决**：新增 `src/car_bringup/rviz/nav2.rviz`：

- TF：`Frames: {All Enabled: true}`，删掉 TurtleBot 的固定帧树；
- RobotModel：`All Links Enabled: true`，`robot_description` 用 transient local；
- 保留 Map、LaserScan、ParticleCloud、Global Planner、Controller、Navigation 2 面板；
- 去掉 Bumper Hit / Realsense / MarkerArray 等与本车无关的显示。

`nav2.launch.py` 改为加载这个文件。  
验证：`rviz_nav2.log` 不再出现 `No transform from ...`。

#### Step 3：重复实例 / 容器残留

`cleanup_sim.sh` 原代码：

```bash
pkill -9 -x component_contai
```

但 `component_container_isolated` 的 comm 被截断成 15 字符，实际是：

```
component_conta
```

所以这条命令永远匹配不到，Nav2 容器会残留；再启动一套 Nav2 就会出现多个 `amcl/bt_navigator/controller_server`，表现就是 TF 抖、goal 无响应、cmd_vel 多发布者。

**修复**：

```bash
pkill -9 -x component_conta
pkill -9 -f "nav2_container"
```

同时修 `sim_nav2.launch.py` / `mapping.launch.py` 的另一个坑：

```python
# IncludeLaunchDescription 的 launch_arguments 会写进共享 launch 配置
# 这里传 rviz:=false 给 Gazebo，会把顶层 rviz 也覆盖成 false
gazebo = GroupAction(
    actions=[IncludeLaunchDescription(..., {'gui': gui, 'rviz': 'false'})],
    scoped=True,
    forwarding=True,
)
```

否则 `sim_nav2` 和 `mapping` 的 RViz 条件永远不成立。

#### Step 4：复现“闪烁/卡死”——短目标成功，车后目标失败

- `(0.8, 0)`、`(2.0, 0)` 前进目标：成功。
- 目标在车后 `(-3.0, 0.5)`：18 次恢复后 ABORTED。

抓 `/amcl_pose`、`/ground_truth`、`/odom`、`/particle_cloud`、`map→odom` 做时间序列后，看到清晰的时间线：

| 时间 | 真值 G | AMCL E | 误差 | odom O | cmd | 粒子数 |
|---|---|---|---|---|---|---|
| 1700 | (1.85, 0.01) | (1.81, −0.05) | 0.07 m | (1.81, 0.06) | 0.00 | 567 |
| 1704 | (1.32, 0.15) | (1.25, 0.14) | 0.07 m | (1.28, 0.22) | **−0.40** | 1087 |
| 1710 | (0.13, 0.71) | (−0.08, 0.66) | 0.22 m | (0.10, 0.78) | 0.29 | 2000 |
| 1736 | (0.30, 0.21) | (−0.17, 0.28) | 0.48 m | (0.29, 0.32) | 0.22 | 2000 |
| 1740 | (−0.33, −0.70) | (−1.05, −0.06) | 0.97 m | (−0.33, −0.57) | 0.53 | 2000 |
| 1744 | (−2.08, −1.75) | (−2.82, −0.22) | 1.69 m | (−2.18, −1.42) | 0.00 | 2000 |
| 1760 | (−3.67, −5.56) | (−7.43, 0.22) | **7.53 m** | (−4.26, −4.97) | 0.32 | 2000 |

- **−0.4 m/s 倒车弧线**是转折点；
- odom 与真值误差从 0.06 m 涨到 **0.84 m**（1760 时刻）；
- AMCL 的粒子云 std 涨到 **1.23 / 1.75 / 2.75 rad**，粒子打满 2000；
- 最终恢复 18 次、ABORTED；机器人被反复 `BackUp` 到左墙附近。

**根因不是“地图突然坏了”，而是倒车工况下控制器 odom 变差，AMCL 的运动模型跟着 odom 走，扫描匹配又发生感知别名（相似墙面），把估计越拉越远。**

#### Step 5：先禁倒车？——DUBIN 的代价

尝试 `motion_model_for_search: "DUBIN"` + `vx_min: 0.0`：

- 前进目标 `(2.0, 0)`：**SUCCEEDED**（`vx_min: −0.05` 后）。
- 车后目标 `(-3.0, 0.5)`：规划出 **15–16 m** 长环线；`vx_min: 0.0` 时 MPPI 直接：

```
[controller_server]: Optimizer fail to compute path
```

原因是静止起步 + Ackermann 约束 + 路径起始急转，优化器找不到可行轨迹；`vx_min` 放宽到 −0.05 才能起步。

**这一步的结论**：纯前进规划能解决“倒车丢定位”，但对“车后目标/窄空间”会制造长环线和起步无解；不能只靠 DUBIN。

#### Step 6：慢倒车折中

最终采用：

```yaml
vx_max: 0.30
vx_min: -0.15                 # 原 −0.4 太快，odom/AMCL 会爆
motion_model_for_search: "REEDS_SHEPP"
reverse_penalty: 2.0          # 允许但明显惩罚倒车
min_turning_r: 0.35
inflation_radius: 0.25
acc_lim_x: 1.0
decel_lim_x: -1.5
```

慢倒车测试中，AMCL 与真值全程保持 **0.1–0.3 m**，没有再出现 7 m 级发散；但车后目标的长路径跟踪/终点朝向仍会导致 ABORTED（见 2.5）。

### 2.2 根因分层（为什么不是单一问题）

| 层 | 现象 | 根因 | 修复 |
|---|---|---|---|
| 场景层 | TF 全断、RViz 报错 | 只有 Nav2，没有 Gazebo/控制器 | 恢复完整栈；cleanup 清残留 |
| 可视化层 | 完整栈下 TF 仍报错 | `nav2_default_view.rviz` 引用 TurtleBot 帧 | 新增 `nav2.rviz`，用本车帧 |
| 定位层 | 车后目标闪烁/丢定位 | −0.4 m/s 倒车 + odom 误差 0.84 m + AMCL 感知别名 | 慢倒车 −0.15、限速 0.3、REEDS_SHEPP |
| 控制层 | 车完全不动、恢复循环 | `vx_min: 0.0` → `Optimizer fail`；速度 1.0 与实际不匹配 | `vx_min: −0.15`、`vx_max: 0.30`、smoother 0.3 |
| 地图/路径层 | 左房间目标 ABORTED | 15–16 m 绕行路径、左房间连通/地图质量 | 待重建地图 / RPP/TEB / 放宽 yaw |

### 2.3 为什么这样解决（取舍理由）

- **为什么不只改 RViz 配置？**  
  改 RViz 只消除“显示层”的 TF 报错；重复容器、倒车丢定位、路径失败仍在。
- **为什么不只降速？**  
  降速能减轻 odom/AMCL 漂移，但 −0.4 m/s 倒车本身是触发条件；必须同时限制倒车和速度。
- **为什么不用纯 DUBIN？**  
  DUBIN 对车后目标会规划 15 m 环线；在起步急转处 MPPI 仍可能无解（`vx_min=0`）。慢倒车能直接完成短距离倒车弧线，AMCL 在 −0.15 下稳定。
- **为什么 `vx_min` 不是 0？**  
  MPPI 在静止 + Ackermann 约束下，`vx_min=0` 会出现 `Optimizer fail to compute path`；留一个很小的倒车余量（−0.05 至 −0.15）让求解器能起步，同时规划器仍用 `reverse_penalty` 惩罚倒车。
- **为什么 `min_turning_r` 从 0.55 改 0.35？**  
  手册 0.55 偏保守；固件/URDF 实测最小约 0.287，0.35 是安全且能让阿克曼在室内完成掉头的值。
- **为什么 `inflation_radius` 从 0.3 改 0.25？**  
  0.3 在窄通道会过度压缩可通行空间；0.25 仍大于 footprint 半宽，配合慢速/低加速度更稳。
- **为什么 RViz 用软件渲染？**  
  VMware/SVGA 硬件 GL 下 RViz 3D 视口会全黑；`LIBGL_ALWAYS_SOFTWARE=1` 能显示地图/代价地图，代价是帧率低（把 Frame Rate 降到 10）。
- **墙上的彩色竖条是什么？**  
  是 Mesa/Ogre `indexed_8bit_image` 的已知 shader 链接问题，属于 **Map 纹理显示**，不是代价地图，也不影响 `/map` 数据/导航。彻底规避可用 `map_to_points.py` 的 `/map_points` Marker（建图 RViz 已用），或换正常 GPU。

### 2.4 验证数据

| 场景 | 结果 | 关键数据 |
|---|---|---|
| 前进 (2.0, 0)，最终配置 | **SUCCEEDED，0 次恢复** | 速度 0.3 m/s；AMCL 误差 3–5 cm |
| 车后 (−3.0, 0.5)，−0.4 倒车 | ABORTED，18 次恢复 | AMCL 误差 7.53 m；odom 0.84 m；粒子 2000 |
| 车后 (−3.0, 0.5)，−0.15 慢倒车 | ABORTED，20 次恢复 | AMCL 误差 0.1–0.3 m；路径 15–16 m；定位未丢 |
| TF 链 | 正常 | `/tf` 3 个发布者：amcl / r5_controller / RSP |
| RViz TF 显示 | 无 “No transform” | 自定义 `nav2.rviz` |

> 验证边界说明：`(2.0, 0)` 前进目标的成功验证是在“速度/倒车参数修正”后完成的；
> 之后又独立做了 self-hit 的 `min_range` 修正，并单独验证了
> `LOCAL center=0`、`GLOBAL center=0`。两者验证的对象不同，不能互相替代：
> 前者证明定位/控制链路，后者证明自身回波过滤。

### 2.5 仍然失败的部分与下一步

目标 `(-3.0, 0.5)` 在左房间，规划器给出 **15–16 m** 绕行路径；慢倒车版 AMCL 不再发散，但 MPPI 在长路径/急转/恢复中仍 ABORTED。这属于**地图连通性/路径**，不是 TF 或定位爆炸。

建议顺序：

1. 按 `SIM_TODO.md` §10 重建一张完整新图（右房间、左上房间、左下房间、回环闭合），保存为 `my_room_map2`，再测左房间目标；
2. 若地图仍窄，换 RPP/TEB 或把长路径拆成 Waypoint 分段（每段 ≤0.6 m，配合倒车分流）；
3. `yaw_goal_tolerance: 0.30` 对阿克曼偏严，终点朝向容易触发恢复；可放宽到 0.5–0.8，或让目标朝向沿接近方向；
4. 用 `plan_inspector.py` 看 `/plan` 的倒车段/换向次数，区分“地图不可达”与“控制器跟不住”。

---

## 3. 问题二：机器人脚下出现代价地图 / 自己卡死自己

### 3.1 现象

Nav2 行动过程中，机器人当前位置出现粉/红色代价地图圈；plan 到车就结束，`Feedback: aborted`；车不走。看起来像“代价地图没有自清洁”。

### 3.2 证据：激光在打自己

用 `rclpy` 直接订阅 `/scan`（不依赖 CLI daemon），360 条雷达数据：

```
SCAN: frame=laser_link n=360 min=0.100 range_min=0.10 range_max=10.00
returns <0.35m: 148
  angle= +69.7°  range=0.100
  angle= -69.7°  range=0.100
  angle=+110.8°  range=0.100
  angle=-110.8°  range=0.100
  ...
  r[  0°]=8.314   r[ 45°]=6.498   r[ 90°]=4.532
  r[135°]=0.132   r[180°]=0.174   r[-135°]=0.132
```

- **148/360 条 <0.35 m**，最近 **0.100 m**（雷达 `range_min`）；
- 集中在 **±69°、±110°、±135°、180°**；
- 正前方 0° 是 8.3 m（环境正常），说明是车体自身。

雷达安装：`laser.xacro` 的 `laser_joint`：

```xml
<origin xyz="0.074 0 0.067" rpy="0 0 0"/>
```

高度 6.7 cm，正好在轮胎/转向机构上沿；前轮/转向/后车体进入扫描平面。

### 3.3 假设与排除

| 假设 | 检查 | 结论 |
|---|---|---|
| A. 激光打到自身结构 | `/scan` 148 条 <0.35 m，角度集中在车体 | **确认，主因** |
| B. footprint 配小/配错 | `footprint` 0.28×0.22；自击点 0.10–0.17 m 位于足迹内 | 不是主因；足迹基本正确 |
| C. inflation_radius 过大 | 0.25 m 本身合理；问题在“有自身回波可膨胀” | 不是根因；仅降低会掩盖 |
| D. 自清洁被禁用 | `footprint_clearing_enabled` 只清足迹内部；边界点/全局层仍可被膨胀覆盖 | 不是单一开关问题；需同时过滤自身回波 |

### 3.4 为什么“自清洁”没兜住

Nav2 的 footprint clearing 逻辑是：标记完后，把**足迹内部**的格子清掉。它无法处理：

- 落在足迹边界/外侧的自击点（仍会被 inflation 膨胀到车体中心）；
- 全局代价地图层（static/obstacle/inflation）里同一批点的膨胀；
- 扫描点先被标记、inflation 再覆盖中心的时序。

所以只把 `footprint_clearing_enabled` 设 true，可能仍然卡死；**更上层、更直接的做法是：足迹内的回波根本不进入代价地图**。

### 3.5 解决方案

`config/nav2_params.yaml`：

```yaml
amcl:
  ros__parameters:
    laser_min_range: 0.25          # 定位也不吃自身回波

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

为什么是 0.25 m：

- 自击点最远 0.174 m（180°），0.25 能全部滤掉；
- 足迹半对角线约 `sqrt(0.14²+0.11²)=0.178 m`，0.25 是“足迹+安全余量”，不会误删真正的近障碍；
- 雷达 `range_min` 仍是 0.1，`/scan` 原始数据不变；只是代价地图/AMCL 不采信足迹内的读数。

### 3.6 验证

重启 Nav2 后读取代价地图（`rclpy`，不依赖 CLI daemon）：

```
LOCAL 100x100 origin=(-2.45,-2.45) cell=(48,48)
    ...........
    ...........
    center= 0
GLOBAL 376x222 origin=(-10.30,-6.46) cell=(205,129)
    :..........
    ...........
    center= 0
```

机器人中心 ±5 格全为 `.`（free），没有 `#`（lethal）/`+`（inflation）；脚下粉红圈消失。

### 3.7 如果还想让 `/scan` 本身干净

当前 `/scan` 仍会包含 0.1 m 自击点（RViz 的 LaserScan 还会在车附近显示红点），这是**传感器原始数据**，不是代价地图又坏了。可选：

1. 加 `laser_filters` 的 `LaserScanRangeFilter`，`lower_threshold: 0.25`，把 `/scan` 过滤成 `/scan_filtered`，代价地图/AMCL 用过滤后的话题；
2. 仿真里把 `laser_link` 抬高到 0.09 m（避开轮/转向），但实车要按真实安装高度处理；
3. 保持现状：代价地图/AMCL 的 `min_range` 已足够，`/scan` 原始数据保留给调试。

---

## 4. 改动清单

| 文件 | 改动 | 作用 |
|---|---|---|
| `config/nav2_params.yaml` | `vx_max 0.30`、`vx_min −0.15`、`REEDS_SHEPP`、`reverse_penalty 2.0`、`min_turning_r 0.35`、`inflation 0.25`、`acc 1.0/−1.5`、smoother `0.30/−0.15` | 慢速、慢倒车、可起步、不爆定位 |
| `config/nav2_params.yaml` | scan 源 `raytrace_min_range/obstacle_min_range 0.25`；两层 `footprint_clearing_enabled: true`；AMCL `laser_min_range 0.25` | 过滤自身回波，代价地图中心 free |
| `launch/nav2.launch.py` | 改用 `car_bringup/rviz/nav2.rviz`；`-geometry 1600x880+50+20`；`LIBGL_ALWAYS_SOFTWARE=1` | 本车 TF 视图 + VMware 可显示 |
| `launch/sim_nav2.launch.py` | Gazebo include 用 `GroupAction(scoped=True)` 隔离 `rviz:=false` | Nav2 RViz 不再被顶掉 |
| `scripts/cleanup_sim.sh` | `component_conta` + `pkill -f nav2_container` | 真正清掉 Nav2 容器，避免重复实例 |
| `rviz/nav2.rviz` | 新增：TF All Enabled、RobotModel 本车 links、Map/LaserScan/ParticleCloud/Global Planner/Controller/Navigation 2 | 消除 TurtleBot 缺帧报错 |
| `rviz/mapping.rviz`、`scripts/map_to_points.py`、`launch/mapping.launch.py` | 建图 RViz 修复 + `/map_points` Marker | 建图可视化；Map 纹理 shader 的替代显示 |
| `docs/SIM_TODO.md` | §11、§12 | 简要复盘记录 |
| 本文件 | `reports/NAV2_DEBUG_REPORT.md` | 完整思维链与证据 |

---

## 5. 复现与验收命令

```bash
# 1) 清干净（关键：确认没有残留 Nav2 容器）
ros2 run car_bringup cleanup_sim.sh
ros2 node list | grep -E "amcl|bt_navigator|controller_server|map_server" || echo clean

# 2) 一键：headless Gazebo + Nav2 + 自定义 RViz
ros2 launch car_bringup sim_nav2.launch.py gui:=false
# 等日志出现 "Managed nodes are active" 再发目标；太早会 Goal was rejected

# 3) 前进目标验收
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}}" --feedback

# 4) 查自身回波 / 代价地图中心
python3 /tmp/self_cost_diag.py     # 需要时重新生成；核心看 /scan <0.35m 条数与局部/全局 center
# 期望：代价地图机器人中心 = 0（free），不再有 #/+

# 5) 重建地图（左房间/长路径问题）
ros2 launch car_bringup mapping.launch.py gui:=false
ros2 run nav2_map_server map_saver_cli -f ~/car_ws/src/car_bringup/maps/my_room_map2
```

---

## 6. 可复用的排查清单

1. **先看数据链，再看算法**：`/clock`、`/scan`、`/odom`、`/tf` 的发布者数量；没有 Gazebo 时，TF 报错是结果不是原因。
2. **区分“TF 数据”与“TF 显示”**：`tf2_echo map base_link` 能查到就说明数据链通；RViz 仍报缺帧，查 RViz 的 `Frames`/RobotModel 配置。
3. **先确认只有一套实例**：`ros2 node list | grep -E "amcl|bt_navigator|controller_server"` 应各一个；容器残留用 `pkill -x component_conta`。
4. **定位问题用真值对照**：p3d `/ground_truth` vs `/amcl_pose` vs `/odom`；看误差随时间的变化，而不是只看最终结果。
5. **闪烁要看 TF 的 map→odom**：它跳变 = AMCL 在大幅修正；先查 odom 质量/倒车/速度，而不是调 RViz。
6. **代价地图中心先读数值**：机器人中心 cell = 0/253/254 比看颜色可靠；self-hit 用 `/scan` 的最小值和角度确认。
7. **优先修“输入”，不是“显示”**：自身回波 → min_range；重复实例 → cleanup；地图不可达 → 重建/换规划器。
8. **每次只改一层并复测**：TF、定位、控制、地图分开验证，避免“改了 5 个参数不知道哪个生效”。

---

## 7. 附录：关键数据

### 7.1 倒车发散时间线（节选）

```
t=1700  E=( 1.81,-0.05) G=( 1.85, 0.01) err=0.07 cmd= 0.00 N= 567  M2O=(-0.01,-0.02,-0.05)
t=1704  E=( 1.25, 0.14) G=( 1.32, 0.15) err=0.07 cmd=-0.40 N=1087  M2O=(-0.03,-0.03,-0.05)
t=1740  E=(-1.05,-0.06) G=(-0.33,-0.70) err=0.97 cmd= 0.53 N=2000  M2O=(-0.57, 0.39,-0.30)
t=1744  E=(-2.82,-0.22) G=(-2.08,-1.75) err=1.69 cmd= 0.00 N=2000  M2O=(-0.30, 0.39,-0.34)
t=1760  E=(-7.43, 0.22) G=(-3.67,-5.56) err=7.53 cmd= 0.32 N=2000  M2O=(-1.37,-2.26,-1.25)
```

### 7.2 自击点证据

```
SCAN: frame=laser_link n=360 min=0.100 range_min=0.10 range_max=10.00
returns <0.35m: 148
angle= +69.7° range=0.100
angle= -69.7° range=0.100
angle=+110.8° range=0.100
angle=-110.8° range=0.100
r[135°]=0.132  r[180°]=0.174  r[-135°]=0.132
```

### 7.3 修复后的代价地图

```
LOCAL center= 0
GLOBAL center= 0
```

机器人中心 ±5 格全 free。

---

## 8. 一句话总结

**闪烁/TF 报错的根因是“孤儿 Nav2 + TurtleBot 版 RViz + 重复容器”；真正让车丢定位/卡死的是“−0.4 m/s 倒车 + odom 漂移 + AMCL 扫描别名”；机器人脚下代价地图的根因是“激光 0.1–0.17 m 自击点被 min_range=0 喂进代价地图，再被 0.25 m 膨胀覆盖到车体中心”。**

修完这些后：前进目标稳定成功、TF 显示正常、代价地图中心 free。剩下的左房间目标 15–16 m 绕行/ABORTED 属地图连通性与终点朝向问题，下一步是重建完整地图或换 RPP/TEB。

## 9. 追加：贴边 + 拐角卡死（2026-10-30）

**背景**：用户把 `inflation_radius` 从 0.3 改到 **0.15**，车能进走廊，但开始“贴着代价地图边缘走”，拐角处卡死。

**实测与根因（两件事，不要混在一起）**：

1. **贴边**：footprint 0.28×0.22 的外接圆半径是 `sqrt(0.14²+0.11²)=0.178 m`。
   `inflation_radius=0.15 < 0.178`，意味着规划器允许路径中心离墙只有 0.15 m，
   而车角实际到中心 0.178 m——路径在数学上就已经让车角插进墙里。
   直线段还能擦着过，一到拐弯，车头/车尾扫掠更大，MPPI 的
   `CostCritic(consider_footprint)` 判定碰撞，车就停/倒车恢复。
   **不是“自清洁”问题，是安全边界小于车体本身。**
2. **拐角卡死**：目标 `(7.5, −5.2)`，机器人最终到
   `(7.576, −5.101)`——距目标只有 **0.12 m**（xy 容差 0.20 内），
   但 yaw = **−0.643 rad**，目标 yaw = 0；`yaw_goal_tolerance` 只有
   **0.30**，GoalChecker 判失败，MPPI 在终点反复“原地对齐”，
   16 次恢复后 ABORTED。**这是阿克曼不能原地转够 37° 的终点朝向问题。**

**处理**：

```yaml
# local/global costmap
inflation_radius: 0.22          # ≥ 外接圆 0.178 + 余量
cost_scaling_factor: 5.0        # 代价梯度更陡，路径往通道中心走
footprint_padding: 0.02         # 2 cm 安全余量
# Smac
cost_penalty: 3.5               # 规划器更愿意走低代价(中心)区域
# goal checker
yaw_goal_tolerance: 0.80        # 阿克曼不要求精确终点朝向
```

**复测**：
- 从靠近目标的位姿发 `(7.5,−5.2)`：机器人到 `(7.576,−5.101)`、
  yaw −0.643；在 `yaw_goal_tolerance 0.8` 下位置/朝向都在容差内，
  终点对齐不再触发恢复（这正是之前 16 次恢复的直接原因）。
- 从原点重新发 `(7.5,−5.2)`：机器人到 `(7.055,−3.122)` 后 15 次恢复
  ABORTED；说明**这条长路线里的走廊/扫掠宽度对 0.22 膨胀仍然过窄**，
  属于路线几何问题，不是再调一个参数能解决的。

**正常项目做法（顺序不要反）**：

1. 先量：footprint、外接圆、最小转弯半径、走廊净宽、车体扫掠宽度；
2. `inflation_radius` 必须 ≥ 外接圆 + 安全余量，`cost_scaling_factor`
   5–8，`footprint_padding` 2–3 cm；**不要为了挤窄道把 inflation 降到
   外接圆以下**，那是在骗规划器；
3. 若走廊净宽 < 车体扫掠宽度：改路线/改地图（加宽门洞），或做**三点掉头**
   （REEDS_SHEPP + 反向段 + BackUp），或换 RPP/TEB；不要继续压 inflation；
4. 阿克曼终点朝向：`yaw_goal_tolerance` 放到 0.5–0.8，或让目标朝向沿
   接近方向；精确终点朝向需要专门的最后一段对准/多段行驶；
5. 每次只改一层并复测：路径贴边看 inflation/代价，终点卡死看 yaw/goal
   checker，走廊过不去看几何/路线。

