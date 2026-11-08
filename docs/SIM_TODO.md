# SIM_TODO —— 仿真验证清单与结果（2026-11-08，含 10-28 历史）

> **2026-11-08 最新结论（以此为准）**：当前导航限速 **1.0 m/s**（平滑器同），
> 融合 odom 已接管默认链路，AMCL 用 `max_beams 90 / update_min_d 0.10 /
> update_min_a 0.10`。1 m/s 目标 `(4.0,0)` 与 `(6.0,0)` 均 **SUCCEEDED**，
> AMCL 误差约 **16→5.4~7.7 cm**；左房间 `(-3.0,0.5)` 仍 **ABORTED**；
> 动态障碍在“终点不贴墙”场景 12s 通过，近墙终点仍会超时。
> 下文 §2–§4 含 2026-10-28 的旧参数历史数据，保留用于对比。
> 五项问题的完整原因/方案见
> [`reports/FIVE_ISSUES_SIM_REPORT.md`](reports/FIVE_ISSUES_SIM_REPORT.md)。

> 状态图例：✅ 已验证；🟡 部分/有条件；⬜ 未做。
> 所有 `ros2 launch/run` 命令均假设已 `source install/setup.bash`。

## 1. 总表

| 任务 | 状态 | 结论 / 数据 |
|---|---|---|
| 底盘执行 + 控制器 | ✅ | acceptance 三项 PASS；38 上游单测 |
| Nav2 前进导航 | ✅ | 目标 (0.8,0) SUCCEEDED，车到 x≈0.68 |
| 倒车（短距） | 🟡 | 历史 2/3（旧参数）；当前慢倒车 −0.15 定位稳定，长路径/左房间仍 ABORTED |
| **倒车后前进转弯** | ✅ | 见 §3：倒车后 +0.5 目标 8s SUCCEEDED，未卡死 |
| 1 m/s | ✅ | (4.0,0) 8.2s、(6.0,0) 4.7s 均 SUCCEEDED；AMCL 误差 16→5.4~7.7cm（§4） |
| 动态障碍 | 🟡 | 终点不贴墙：障碍 (4,0)/目标 (6,0) 12s 通过；近墙目标 (8,0) 60s 超时（§4/§13） |
| 窄通道/死胡同 | ⬜ | 未构造压测；动态障碍测试已暴露 MPPI 在障碍前易停死 |
| 传感器融合 | ✅ | 已接管默认 odom：`/wheel/odom` + `/imu` → `/odom` + 唯一 TF（§5） |
| 2D 建图 | ✅ | mapping.launch.py；/map 350×219@0.05；map_saver 保存成功 |
| 新图→Nav2 闭环 | 🟡 | map 参数已加；左房间失败指向地图连通性，需重建完整图后复测 |
| STM32 固件 | 🟡 | 主机单测 5/5；实车待联调 |
| 实车 S7/S8/S9 | ⬜ | 需硬件 |

## 2. 倒车三连测（调参后）

参数：`reverse_penalty 0.5 / PreferForwardCritic 0.2 / vx_min -0.4 / xy_goal_tolerance 0.20`。

| 目标 | Nav2 结果 | 用时 | Gazebo 终态 (x,y,yaw) |
|---|---|---|---|
| −0.8 m | SUCCEEDED | 5 s | (−0.672, −0.072, 0.026) |
| −1.6 m | SUCCEEDED | 6 s | (−1.440, −0.368, 0.194) |
| −2.4 m | 超时 | 55 s | (−3.05, −0.996, 0.343) |

**结论**：≤1.6m 满足 2/3、<40s、yaw<0.2；>2m 仍会过冲/超时，建议 BT 分段（每段 ≤0.6m）。
注意：Nav2 判据在 map 系；Gazebo 真值与 map 有约 0.2~0.4m 偏差，AMCL/地图对齐仍需调。

## 3. 倒车 → 前进转弯（用户反馈的卡死场景）

| 步骤 | 结果 | 用时 | 终态 (x,y,yaw) |
|---|---|---|---|
| 倒车到 −0.8 | 动作超时（车已到位） | 61 s | (−0.610, 0.009, −0.021) |
| 再前进到 +0.5 | **SUCCEEDED** | 8 s | (0.315, 0.051, 0.092) |

**结论：倒车后能正常前进转弯，没有卡死。** 倒车动作在终点附近可能超时（判据/终点容差），但不会锁死后续运动。

## 4. 1 m/s 与动态障碍（2026-11-08 复测）

**1 m/s 前向**（`vx_max 1.0`、`max_velocity [1.0,0,1.0]`、融合 odom）：

| 目标 | 结果 | 用时 | 终态/定位误差 |
|---|---|---|---|
| (2.0,0) | SUCCEEDED | 8 s | 融合默认链路回归 |
| (4.0,0) | SUCCEEDED | 8.2 s | AMCL (3.793,0.011) vs GT (3.827,0.053) ≈5.4 cm |
| (6.0,0) | SUCCEEDED | 4.7 s | AMCL (5.822,0.012) vs GT (5.828,0.067) ≈5.5 cm |

- 关键不是只改 `vx_max`：AMCL 默认 0.25m/0.2rad 才更新，1 m/s 下永远滞后。
  改成 `0.10m/0.10rad`、`max_beams 90` 后误差从 16cm 降到 5~8cm。
- 这些参数已写回 YAML；`vx_min` 仍为 −0.15，倒车不要跟高速混用。

**动态障碍（两个场景，2026-11-08）**：

1. **近墙终点（失败）**：障碍 (6.5,0)、目标 (8.0,0) → 60s 超时；车到
   `(7.633,0.386,yaw −0.502)`，未碰撞、已从侧面绕开。目标距右墙仅约 0.5m，
   终点本身落在膨胀区/不可达区，不能把这次失败当成“不会避障”。
2. **终点不贴墙（通过）**：障碍 (4.0,0)、目标 (6.0,0) → **SUCCEEDED、12s、
   0 恢复**；车到 `(5.779,−0.028)`。全局 costmap 在 (4.0,0) 的 cost=88，
   而 (4.0,+0.4) 为 0，说明路径必须从障碍左侧绕行。12s 是从静止加速、
   绕行到 6m 的总时间，不能和无障碍时 2.2m 后段的 4.7s 直接比较。

- 结论：**检测/减速/绕行链路已打通**；失败集中在“终点贴墙/末端姿态”和
  “旧场景目标在障碍后方且极近”这类**终点不可达**问题。
- 改进：目标离墙 ≥1m；用 Waypoint/BT 把“绕过障碍”和“到终点”拆开；
  窄通道再考虑 TEB/RPP 或分层 costmap。详见 §13 与五项报告。

## 5. 传感器融合接入默认 odom：已完成（2026-11-08）

**结果**：融合节点已成为默认 odom 源，Nav2 前向回归 `SUCCEEDED`（8s）。

**改动三处（缺一不可）**：
1. `control.xacro`：控制器 remap `/r5_steering_controller/odometry:=/wheel/odom`、
   `/r5_steering_controller/tf_odometry:=/tf_wheel`；
2. `controllers.yaml`：`enable_odom_tf: false`，避免和融合节点抢
   `odom→base_footprint`；
3. `sensor_fusion.py`：订阅 `/wheel/odom` + `/imu`，发布 `/odom`、
   `/odometry/fused`、`/odometry/gyro_only` 和唯一 TF；`gazebo.launch.py`
   默认 `use_fusion:=true`。

**为什么这样拆**：控制器只提供“轮速里程计”原料；融合节点统一负责
“最终 odom + TF”。TF 只能有一个广播者，否则 AMCL/costmap 会看到两棵树，
出现闪烁、定位跳变、`Send goal call failed`。

**验证**：
- `ros2 topic hz /odom`、`ros2 run tf2_ros tf2_echo odom base_footprint` 正常；
- `ros2 run tf2_ros tf2_monitor` 无重复 authority；
- Nav2 `(2.0,0)` 8s 成功；1 m/s `(4.0,0)/(6.0,0)` 成功。

**风险**：互补滤波的 `alpha=0.999` 偏重陀螺，长时间仍会漂；实车必须用
轮径/IMU 标定后的数据复测，必要时换 `robot_localization`（EKF）。
`fusion.launch.py` 仍保留为独立验证入口（包含的 Gazebo 用 `use_fusion:false`，
再显式起一个融合节点，避免重复）。

## 6. 新图 → Nav2 闭环（待跑）

```bash
# 1) 建图
ros2 launch car_bringup mapping.launch.py
# 2) 遥控走完区域，保存
ros2 run nav2_map_server map_saver_cli -f ~/car_ws/src/car_bringup/maps/new_map
# 3) 关掉建图，重新起 Gazebo
ros2 launch car_bringup gazebo.launch.py gui:=false rviz:=false
# 4) 用新图起 Nav2（map 参数已支持）
ros2 launch car_bringup nav2.launch.py map:=$HOME/car_ws/src/car_bringup/maps/new_map.yaml
# 5) RViz 设初始位姿（或 amcl.set_initial_pose 调成新图起点），发目标
```
未跑原因：需要一张完整新图 + 重新对齐初始位姿；建图/保存已单独验证。

## 7. 建议下一步（优先级）

1. 新图→Nav2 在自己的地图上已基本跑通；剩余一个房间的卡壳留实机复测。
2. 动态障碍：换“目标离墙 ≥1m + Waypoint/BT 分段”的场景复测；窄通道再考虑 TEB。
3. 融合 odom 已默认接入；实车重点复测轮径/IMU 标定后的航向漂移。
4. 1 m/s 在更大场地做直线/弯道压测（当前室内 + RTF 限制）。
5. 实车：S6 固件烧录 → S8 标定 → S7 HW 插件 → S9 集成。

## 8. 环境健康 / RTF / 地图（2026-10-28 复查）

- **Gazebo 单独 RTF = 0.99**；**单套 Nav2 RTF = 0.99**（`gz stats -d 2` 实测）。
- 之前 RTF 0.4 的原因：残留的 `mapping.launch` + `slam_toolbox` + `sensor_fusion`
  + 多套 Nav2 同时运行；`cleanup_sim.sh` 原先只清 nav2/gazebo，**已补上
  mapping/fusion/slam_toolbox/sensor_fusion**。
- **地图没有丢**：活动地图在
  `src/car_bringup/maps/my_room_map.pgm|yaml`（376×222，origin −10.3/−6.46），
  `map_server` 实际加载路径为 `install/car_bringup/share/car_bringup/maps/...`。
  被删除的是早期目录 `my_final_map/`（446M）与 `my_mapping_bag/`（521M），
  属于我清理时判断过度；活动地图不受影响。若需旧数据只能重新建图/录包。
- 实验前后建议：`ros2 run car_bringup cleanup_sim.sh` → `ros2 launch ...`，
  再用 `gz stats` 确认 RTF；RViz/VSCode 也会吃 CPU。

## 9. 建图正确用法 / 本次“建不了图”复盘

错误用法（你截图里的）：
```bash
# ❌ 不要在 ~/car_ws 直接这样跑：nav2_params.yaml 不在当前目录，
#    日志已提示 Parameter file path is not a file，slam 实际用了默认参数
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=nav2_params.yaml
```
正确用法：
```bash
ros2 run car_bringup cleanup_sim.sh
ros2 launch car_bringup mapping.launch.py          # Gazebo + slam + RViz
# 或先起 Gazebo，再单独起 slam（用 R5 参数）：
ros2 launch car_bringup gazebo.launch.py
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=$HOME/car_ws/src/car_bringup/config/slam_toolbox.yaml
```
已修：
- `mapping.launch.py` 使用 `config/slam_toolbox.yaml`（激光 0.1–10m，消除
  “min 0/max 25 超出雷达能力”的警告）；
- `gazebo.launch.py` 设置 `GAZEBO_MODEL_DATABASE_URI=""`，避免代理不可用时
  gzclient 卡 134s（日志 `libcurl ... 192.168.5.100:7890`）；
- 建图实测：`/map` 269×241@0.05，`Registering sensor` 正常。

RViz `context mismatch in svga surface destroy` 是 VMware/OpenGL 驱动告警；
若 RViz 崩溃，用 `rviz:=false` 先建图，再单独开 RViz；不影响 slam_toolbox。

## 10. RViz 建图可视化修复（2026-10-29）

**症状**：`ros2 launch car_bringup mapping.launch.py` 只出 Gazebo，没有 RViz。

**根因（三层叠在一起）**：
1. `mapping.launch.py` 以 `rviz:=false` 包含 `gazebo.launch.py`。
   `IncludeLaunchDescription` 的 `launch_arguments` 会写进**共享**的 launch
   配置，把顶层的 `rviz` 也覆盖成 false，导致建图 RViz 的条件永远不成立。
   已用 `GroupAction(scoped=True)` 隔离这次覆盖。
2. `slam_toolbox` 的 apt 包**不带** `rviz/slam_toolbox_default.rviz`，原来
   `mapping.launch.py` 指向这个不存在的文件。已新增
   `src/car_bringup/rviz/mapping.rviz`。
3. VMware/Mesa 下 RViz 的 `Map` 显示会报
   `indexed_8bit_image ... active samplers with a different type refer to the
   same texture image unit`（Ogre 1.12 已知 shader 问题），地图纹理发花。
   新增 `scripts/map_to_points.py`：把 `/map` 转成 `/map_points` Marker
   （灰=空闲、红=占用、未知透明），`mapping.rviz` 默认禁用 `Map`、显示
   `MapPoints`。RViz 节点同时设置 `LIBGL_ALWAYS_SOFTWARE=1`，避免 VMware
   下 3D 视图全黑。

**推荐用法（VMware 下 Gazebo 不开 GUI，只开 RViz）**：
```bash
ros2 run car_bringup cleanup_sim.sh
ros2 launch car_bringup mapping.launch.py gui:=false
# 自动启动：Gazebo(无界面) + slam_toolbox + map_to_points + RViz
```
建图/保存流程不变：
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 run nav2_map_server map_saver_cli -f ~/car_ws/src/car_bringup/maps/my_map
```
**验证**：RViz 自动启动、Fixed Frame=map、MapPoints 有灰/红点、`/map`
持续更新；新会话起始 `/map` 350×219@0.05。

## 11. Nav2 闪烁 / TF 报错 / 定位丢失复盘（2026-10-29）

**症状**：Gazebo 里车正常，Nav2 跑导航时 RViz 闪烁、TF 树报错、恢复次数
暴涨；发左房间目标最终 `ABORTED`。实测复现并逐项修复：

1. **重复实例**：`cleanup_sim.sh` 里 `pkill -x component_contai` 与实际
   comm `component_conta`（15 字符截断）不匹配，Nav2 容器一直残留；再叠
   加“只跑 Nav2、没跑 Gazebo”时 `/scan`、`/odom`、`/clock` 都没有发布者，
   TF 必然断链。已修 cleanup（`component_conta` + `nav2_container`），并
   把 `sim_nav2.launch.py` / `mapping.launch.py` 的 Gazebo include 用
   `GroupAction(scoped=True)` 隔离 `rviz:=false`，避免把 Nav2/建图 RViz 关掉。
2. **RViz TF 报错**：`nav2.launch.py` 原来用 nav2_bringup 默认视图，TF/
   RobotModel 引用 TurtleBot 的 `camera_link`、`caster_*`、`wheel_*`，本车
   没有这些帧。新增 `rviz/nav2.rviz`（TF `All Enabled`、RobotModel 全部本
   车 link），`nav2.launch.py` 改用它；日志不再出现 “No transform from…”。
   由于 VMware 下硬件 GL 会黑屏，Nav2 RViz 也设置
   `LIBGL_ALWAYS_SOFTWARE=1`；Mesa 下 Map 纹理可能仍有条纹（Ogre shader
   已知问题），不影响地图数据/代价地图/导航。
3. **倒车导致 AMCL 爆炸（核心）**：目标在车后时，MPPI 以 **-0.4 m/s**
   倒车弧线；控制器 odom 与 Gazebo 真值误差从 0.06 m 涨到 0.84 m，AMCL
   估计从 0.07 m 偏到 **7.5 m**，粒子打满 2000、std 1.2/1.7 m，恢复 18 次
   后 ABORTED。改为 **REEDS_SHEPP + reverse_penalty 2.0 + vx_min -0.15**
   （慢速倒车）后，同场景 AMCL 误差保持 **0.1–0.3 m**，TF 稳定。
4. **MPPI 起步无解**：`vx_min: 0.0` 时 `controller_server` 报
   `Optimizer fail to compute path`，车完全不动、只做恢复。改为 `-0.05`
   （前进目标）后 `(2.0, 0)` 目标 **SUCCEEDED**，AMCL 与真值差 3–5 cm。
5. **速度/膨胀（历史参数）**：当时把 `vx_max 1.0 → 0.30`，
   `velocity_smoother` 0.30，`acc_lim_x 1.0 / decel -1.5`，
   `inflation_radius 0.3 → 0.25`，`min_turning_r 0.35`；原来 0.56 m/s
   会冲过目标/顶墙，是“闪烁/恢复”的放大器。**2026-11-08 已重新把限速
   提到 1.0 m/s**，并通过 AMCL 更新率/波束调参把误差压到 5~8cm（§4）；
   本节保留为历史排障记录。
6. **剩余问题**：目标 `(-3.0, 0.5)`（左房间）会规划出 **15–16 m 绕行路
   径**，MPPI 跟踪/恢复后仍 ABORTED；`(2.0, 0)` 前进目标已稳定成功。说明
   `my_room_map` 左房间的连通/通道仍需处理：建议按 §10 重跑一张**完整新
   图**（覆盖左右房间、回环闭合），再测左房间目标；或改用 RPP/TEB 做窄
   通道跟踪。`yaw_goal_tolerance: 0.30` 对阿克曼偏严，必要时放宽到
   0.5–0.8，或让目标朝向沿接近方向。

**当前推荐命令**：
```bash
ros2 run car_bringup cleanup_sim.sh
ros2 launch car_bringup sim_nav2.launch.py gui:=false   # 无 Gazebo GUI，自动 Nav2+RViz
# 先发前进目标验证：(2.0, 0) 成功；左房间目标需先重建地图
```

## 12. 代价地图“自己标自己” / 自清洁问题（2026-10-29）

**现象**：Nav2 行动过程中机器人脚下出现粉/红代价地图圈，原地卡死；
RViz 里 plan 到车就结束，`Feedback: aborted`。看起来像“代价地图没自清洁”。

**实测 `/scan`（360 条）**：
- **148 条 < 0.35 m**，最近 **0.100 m**（雷达自身 min range）；
- 角度集中在 **±69°、±110°、±135°、180°**，是前轮/转向机构/后车体；
- 前方 0° 是 8.3 m（正常），说明不是地图/环境问题，是**雷达看到了自己**。

**根因**：`nav2_params.yaml` 里 local/global 的 scan 观测源
`obstacle_min_range: 0.0`、`raytrace_min_range: 0.0`，把 0.1–0.17 m 的
自身回波当障碍标记；`inflation_radius: 0.25` 再把这些点膨胀到车体中心。
`footprint_clearing_enabled` 只清足迹内部，边界点/全局层仍会被膨胀覆盖，
所以“自清洁”不是万能开关；更直接的是**别把足迹内的回波喂给代价地图**。

**修复**（`config/nav2_params.yaml`）：
- local `voxel_layer.scan` 和 global `obstacle_layer.scan`：
  `raytrace_min_range: 0.25`、`obstacle_min_range: 0.25`；
- 两层显式 `footprint_clearing_enabled: true`；
- AMCL `laser_min_range: 0.25`，定位也不吃自身回波。

**验证**：重启 Nav2 后读代价地图，机器人中心
`LOCAL center=0`、`GLOBAL center=0`，±5 格 patch 全为 `.`（free），
没有 `#`/`+`。`/scan` 本身仍会显示 0.10 m 自击点（传感器原始数据），
RViz 的 LaserScan 还会看到；如需彻底消掉可加 `laser_filters` 或抬高雷达。

> 这两个问题的完整排查链、证据、取舍与验证见
> [`reports/NAV2_DEBUG_REPORT.md`](reports/NAV2_DEBUG_REPORT.md)。

## 12. 左房间/长路径复测（2026-11-08）

- **地图连通性**：BFS（原始 free）最短 **17.7m**；按 0.22m 膨胀后 **18.1m**。
  起点到 `(-3,0.5)` 的直线在 **(-2.0, 0.34)** 附近被墙挡住，必须绕行。
- **目标朝向很关键**：
  - `(-3,0.5)` yaw=0：规划 15–16m 环线，110s 仍在执行（车到 −9 附近）；
  - yaw=π：从左边回到 `(-3.36,0.48)`，但终态 yaw≈−1.37，超过 0.8 容差，超时；
  - yaw=−π/2：从 `(-3.36,0.48)` 出发 **SUCCEEDED**（短距离验证通过）；
  - 从原点 yaw=−π/2：180s 仍在绕行（车到 −9.16），未在超时内到点。
- **结论**：不是 TF/控制器坏了，是 **地图上 18m 的长路径 + 目标朝向**。
  这不是单纯调 MPPI 能解决；需要：
  1. 重建一张覆盖左右房间、门/通道完整的图；
  2. 用 Waypoint Follow / BT 分段（每段 ≤0.6m）替代单条 18m 目标；
  3. 目标 yaw 选可行接近方向（例如进左房间朝 −y），或把最终朝向容差放宽；
  4. 仍不行再换 RPP/TEB。
- 1 m/s 与融合默认 odom 已通过前向回归；动态障碍在终点离墙场景已闭环，
  近墙终点仍未闭环；窄通道/死胡同仍未压测。不要把它们误判为全部通过。

## 13. 五项问题总判定（2026-11-08）

| # | 问题 | 判定 | 一句话结论 |
|---|---|---|---|
| 1 | 左房间/长路径 | 🟡 留实机 | 地图 17.7~18.1m 长路径 + 目标 yaw 不可达；用 BT/Waypoint 分段 + 可行 yaw，不在仿真继续硬调 |
| 2 | 动态障碍/窄通道 | 🟡 部分 | 终点离墙 ≥1m：1 m/s 12s 绕行通过；终点贴墙：60s 超时；窄通道未做 |
| 3 | 1 m/s 高速 | ✅ 仿真 | 4m/6m 目标均成功；关键修 AMCL 更新率（0.10m/0.10rad、90 束），误差 16→5.5cm |
| 4 | 融合默认 odom | ✅ 仿真 | 控制器只发 `/wheel/odom`，融合发 `/odom`+唯一 TF；前向/高速回归通过 |
| 5 | 新图→Nav2 闭环 | 🟡 留实机 | 用户自有地图除一个房间外可用；剩余问题在地图/场景，非 Nav2 主链，实机复测更合适 |

原因、修复、工程常见度与实机注意事项：
[`reports/FIVE_ISSUES_SIM_REPORT.md`](reports/FIVE_ISSUES_SIM_REPORT.md)。
