# SIM_TODO —— 仿真验证清单与结果（2026-10-28）

> 状态图例：✅ 已验证；🟡 部分/有条件；⬜ 未做。
> 所有 `ros2 launch/run` 命令均假设已 `source install/setup.bash`。

## 1. 总表

| 任务 | 状态 | 结论 / 数据 |
|---|---|---|
| 底盘执行 + 控制器 | ✅ | acceptance 三项 PASS；38 上游单测 |
| Nav2 前进导航 | ✅ | 目标 (0.8,0) SUCCEEDED，车到 x≈0.68 |
| 倒车 ≤1.6m（三连测） | ✅ 2/3 | 见 §2；长距离 >2m 需分段 |
| **倒车后前进转弯** | ✅ | 见 §3：倒车后 +0.5 目标 8s SUCCEEDED，未卡死 |
| 1 m/s | 🟡 | 控制器直发 1.0 已由 acceptance 验证；Nav2 目标 (2.0,0) SUCCEEDED（25s 墙钟，受仿真 RTF/MPPI 计算影响） |
| 动态障碍 | 🟡 | 生成 0.4m 圆柱后：**能检测停车，不能绕行**（ABORTED，未碰撞） |
| 窄通道/死胡同 | ⬜ | 未构造压测；动态障碍测试已暴露 MPPI 在障碍前易停死 |
| 传感器融合 | 🟡 | 算法/launch/量化已完成，未接入默认 odom（见 §5） |
| 2D 建图 | ✅ | mapping.launch.py；/map 350×219@0.05；map_saver 保存成功 |
| 新图→Nav2 闭环 | 🟡 | map 参数已加；尚未用新建完整图跑一遍导航 |
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
