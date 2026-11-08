# 系统架构

> 目标：一页看懂“谁负责什么、数据怎么流、每个包从哪读起”。
> 更细的逐行/改动指南见 [ENGINEERING_HANDOVER.md](ENGINEERING_HANDOVER.md)；
> 文件级地图见 [PROJECT_MAP.md](PROJECT_MAP.md)。

## 1. 一句话

一台 **TARKBOT R5 阿克曼小车**：Gazebo Classic 仿真（或实车）→
`ros2_control` 底盘执行 → Nav2 全局/局部规划 → 2D 雷达建图/AMCL 定位；
另有一条 **X-Protocol 串口桥 + STM32 固件** 的实车链路。

## 2. 分层架构

```
┌──────────────────────────────────────────────────────────────┐
│ 任务层   bt_navigator + BehaviorTree（akm_navigate_*.xml）    │
│          输入：/navigate_to_pose    输出：调规划/控制/恢复     │
├──────────────────────────────────────────────────────────────┤
│ 全局规划 planner_server + SmacPlannerHybrid (Reeds-Shepp)     │
│          输入：map + costmap + goal   输出：/plan            │
├──────────────────────────────────────────────────────────────┤
│ 局部控制 controller_server + MPPI (Ackermann)                │
│          输入：/plan + local costmap 输出：/cmd_vel_nav      │
├──────────────────────────────────────────────────────────────┤
│ 速度整形 velocity_smoother（加速度/限速）→ /cmd_vel          │
├──────────────────────────────────────────────────────────────┤
│ 底盘执行 r5_steering_controller（50Hz）                       │
│          /cmd_vel → 后轮 velocity + 前轮 position            │
├──────────────────────────────────────────────────────────────┤
│ 仿真/硬件边界 gazebo_ros2_control（现在）/ SystemInterface（实车）│
└──────────────────────────────────────────────────────────────┘

感知/定位支路：
/scan (2D lidar) ─┬─ AMCL：map→odom
                  └─ costmap：局部/全局障碍
/odom + /imu ── sensor_fusion.py：互补滤波（实验，未接管默认 odom）
Gazebo p3d ── /ground_truth：仅用于量化对比
```

**关键边界**：

- `MPPI` 是 Nav2 的**局部控制器**，负责跟踪 + 局部避障；
- `LQR / Pure Pursuit / Stanley` 是 `path_tracker.py` 的**独立实验**，
  不参与 Nav2 主链，直接发 `/cmd_vel`；与 Nav2 **互斥**；
- `r5_steering_controller` 只做阿克曼 IK/限幅，不决定“往哪走”。

## 3. 包职责

| 包 | 职责 | 关键文件 |
|---|---|---|
| `car_description` | URDF/xacro、Gazebo 世界、RViz 配置 | `urdf/car.urdf.xacro`、`urdf/car/control.xacro`、`wheels.xacro` |
| `car_bringup` | 启动、参数、地图、实验脚本、RViz | `launch/*.launch.py`、`config/{controllers,nav2_params}.yaml`、`scripts/` |
| `r5_steering_controller` | 自研 C++ 阿克曼控制器（三限幅） | `src/r5_steering_controller.cpp` |
| `r5_serial` | X-Protocol 串口桥、虚拟 STM32 | `r5_serial/{protocol,serial_bridge,virtual_stm32}.py` |
| `car_navigation` | 阿克曼行为树（无 Spin、大 BackUp） | `behavior_trees/akm_navigate_*.xml` |
| `firmware/openctr_b20` | STM32 侧纯 C 模块（协议/看门狗/几何） | `src/{xprotocol,safety,ackermann}.c` |
| `patches` | 上游 ros2_controllers 修改归档 | `ros2_controllers-r5.patch` |
| `scripts` | 仓库级运维脚本 | `sync-to-pi.sh` |

## 4. 一次导航的数据流

1. RViz/脚本发 `/navigate_to_pose`；
2. `bt_navigator` 执行 `akm_navigate_to_pose.xml`；
3. `planner_server`（Smac Hybrid）发布 `/plan`；
4. `controller_server`（MPPI）输出 `/cmd_vel_nav`；
5. `velocity_smoother` 输出 `/cmd_vel`；
6. `r5_steering_controller` 做阿克曼 IK，写关节命令；
7. Gazebo/实车执行，关节状态回到控制器；
8. 控制器发布 `/odom` + `odom→base_footprint`；
9. AMCL 用 `/scan` + `/odom` 发布 `map→odom`；
10. costmap 更新，回到 2/3 闭环。

## 5. 启动模式（互斥关系）

| 命令 | 内容 | 与谁互斥 |
|---|---|---|
| `ros2 launch car_bringup gazebo.launch.py` | Gazebo + 控制器 | 可与 Nav2/建图叠加 |
| `ros2 launch car_bringup sim_nav2.launch.py` | Gazebo + Nav2 + RViz（一条命令） | 不要重复启动 |
| `ros2 launch car_bringup nav2.launch.py` | Nav2（需已有 Gazebo/实车） | 不要与建图同时 |
| `ros2 launch car_bringup mapping.launch.py` | Gazebo + slam_toolbox | 不与 Nav2/AMCL 同时 |
| `ros2 launch car_bringup fusion.launch.py` | Gazebo + 互补滤波 | 不启动 Nav2 |
| `ros2 run car_bringup path_tracker.py` | PP/Stanley/LQR 实验 | 不与 Nav2 同时发 `/cmd_vel` |

## 6. 2D 雷达/定位链路

- 仿真：`laser.xacro` 的 Gazebo ray → `/scan`；
- 实车：RPLIDAR A1M8（apt 驱动）或 LD19（UART），统一 `/scan`；
- 建图：`slam_toolbox`（`config/slam_toolbox.yaml`，0.1–10m）；
- 定位：AMCL（`nav2_params.yaml` 的 `amcl`），输出 `map→odom`；
- 避障：local/global costmap 的 `obstacle/voxel_layer` + `inflation_layer`；
- 已知坑：激光扫到车体（min_range 0.25）、TF 多实例、地图/初始位姿不一致。

## 7. 实车边界

仿真用 `gazebo_ros2_control/GazeboSystem`；实车需写
`hardware_interface::SystemInterface`，导出同样的
`后轮 velocity×2 + 前轮 position×2` 命令接口与轮速/IMU 状态接口。
上层 `r5_steering_controller` 不改。串口协议与固件模块见
[../src/r5_serial/docs/S5_SERIAL_BRIDGE.md](../src/r5_serial/docs/S5_SERIAL_BRIDGE.md)
与 [../firmware/openctr_b20/README.md](../firmware/openctr_b20/README.md)。

## 8. 已知边界

- 3D 激光（FAST-LIO / FAST-LIO-SAM-QN / livox）已弃用，源码保留在
  `src/`，不参与构建；
- 融合节点尚未接管默认 `/odom`；
- 动态障碍可停车但未验证绕行；窄通道/死胡同未压测；
- 实车 S7~S9 未开始（需要硬件）。
