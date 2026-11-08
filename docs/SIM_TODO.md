# SIM_TODO —— 仿真验证清单与结果（2026-10-28）

> **2026-10-29 最新结论（以此为准）**：当前导航参数为速度 0.3 m/s、
> 慢倒车 −0.15 m/s、`reverse_penalty 2.0`、`min_turning_r 0.35`。
> 前进目标 `(2.0,0)` **SUCCEEDED、0 次恢复**；左房间 `(-3.0,0.5)` 仍
> **ABORTED**（15–16m 绕行/地图连通性）；倒车发散已用慢倒车控制住。
> 下文 §2–§4 是 2026-10-28 的旧参数历史数据，保留用于对比，不要当成当前配置。
> 详细证据见 [`reports/NAV2_DEBUG_REPORT.md`](reports/NAV2_DEBUG_REPORT.md)。

> 状态图例：✅ 已验证；🟡 部分/有条件；⬜ 未做。
> 所有 `ros2 launch/run` 命令均假设已 `source install/setup.bash`。

## 1. 总表

| 任务 | 状态 | 结论 / 数据 |
|---|---|---|
| 底盘执行 + 控制器 | ✅ | acceptance 三项 PASS；38 上游单测 |
| Nav2 前进导航 | ✅ | 目标 (0.8,0) SUCCEEDED，车到 x≈0.68 |
| 倒车（短距） | 🟡 | 历史 2/3（旧参数）；当前慢倒车 −0.15 定位稳定，长路径/左房间仍 ABORTED |
| **倒车后前进转弯** | ✅ | 见 §3：倒车后 +0.5 目标 8s SUCCEEDED，未卡死 |
| 1 m/s | ⬜ | 当前 Nav2 限速 0.3；控制器直发 1.0 曾由 acceptance 验证，但未做高速导航压测 |
| 动态障碍 | 🟡 | 旧参数下能检测停车、不能绕行；新参数下未复测 |
| 窄通道/死胡同 | ⬜ | 未构造压测；动态障碍测试已暴露 MPPI 在障碍前易停死 |
| 传感器融合 | 🟡 | 算法/launch/量化已完成，未接入默认 odom（见 §5） |
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

## 4. 1 m/s 与动态障碍

- **1 m/s**：控制器直发 1.0 m/s 已在 acceptance 中验证；Nav2 目标 (2.0,0) SUCCEEDED，终态 (1.987,−0.069,0.047)。墙钟 25s 主要受 Gazebo RTF/MPPI 2000 采样计算影响，不代表实车 1 m/s 用时。
- **动态障碍**：在 (1.0,0) 生成半径 0.2m、高 0.6m 圆柱；目标 (0.2,0) → **ABORTED**，车停在 (1.379,−0.093)，距障碍 0.38m，**未碰撞但未绕行**。
  - 说明：激光/costmap 能检测到并停车；当前 footprint/inflation/CostCritic 组合使车进入膨胀区后“无可行轨迹”。
  - 改进：增大 `inflation_radius` 或在更远处开始避让；调 `CostCritic`/`PathAlign`；必要时换 TEB/RPP。窄通道/死胡同需同样调参后再压测。

## 5. 传感器融合接入默认 odom：难度与方案

**现状**：`fusion.launch.py` → `/odometry/fused` 正常；Nav2 仍用控制器 `/odom`。
**接入不难（约 1 小时），但要改三处，且必须回归测试**：
1. 控制器 `enable_odom_tf: false`；把 `~/odometry` remap 到 `/wheel/odom`（control.xacro）；
2. `sensor_fusion.py` 订阅 `/wheel/odom` + `/imu`，发布 `/odom` + TF `odom→base_footprint`；
3. 启动融合节点，确认 TF 只有一套；再跑 AMCL/Nav2 回归（定位、倒车、前向）。
**风险**：AMCL 的运动模型依赖 odom 质量；融合节点当前是互补滤波，需先验证航向/位置漂移再接管默认 odom。建议单独分支/launch 做，不要直接改默认链路。

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

1. 新图→Nav2 完整跑一遍（验证 map 参数/初始位姿/AMCL）。
2. 动态障碍/窄通道：先调大 `inflation_radius`、降 `CostCritic` 激进参数，再压测；或换 TEB。
3. 融合 odom 接入（按 §5，独立 launch 验证后再默认）。
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
5. **速度/膨胀**：`vx_max 1.0 → 0.30`，`velocity_smoother` 0.30，
   `acc_lim_x 1.0 / decel -1.5`，`inflation_radius 0.3 → 0.25`，
   `min_turning_r 0.35`。原来 0.56 m/s 会冲过目标/顶墙，是“闪烁/恢复”的
   放大器。
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
- 动态障碍、窄通道、1 m/s、融合默认 odom 仍未完成；不要把它们误判为“已通过”。
