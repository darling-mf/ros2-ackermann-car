# PROJECT_STATE —— 压缩记忆（2026-10-28）

> 用途：上下文丢失时，先读这一页。详细展开见文末索引。

## 一句话状态

**仿真主链路完成约 80~85%**：控制/前向导航/建图/融合实验已验证；
**倒车能执行但不稳**（3 连测仅 1 次 40s 内成功）；实车 S7/S8/S9 未开始。

## 已验证（有数据）

| 模块 | 状态 | 证据 |
|---|---|---|
| 底盘执行 + R5 控制器 | ✅ | acceptance 三项 PASS；38 上游单测 |
| Nav2 前向导航 | ✅ | 目标 (0.8,0) `SUCCEEDED`，车到 x≈0.68 |
| Nav2 倒车 | 🟡 | −0.8m 70s 超时但到位(y=0.15)；−1.6m 47s **SUCCEEDED**；−2.4m **ABORTED** 过冲 0.45m |
| 2D 建图 | ✅ | `mapping.launch.py`；`/map` 350×219@0.05；`map_saver_cli` 保存成功 |
| 传感器融合 | 🟡 | `fusion.launch.py` → `/odometry/fused` 正常；**未替换默认 `/odom`** |
| 串口桥 + 虚拟 STM32 | ✅ | 七项测试（坏帧/看门狗） |
| STM32 固件 | 🟡 | 主机单测 5/5；实车待联调 |

## 关键根因（已修）

- **“Nav2 导航不了”头号原因 = 重复启动**：多套 nav2/sim_nav2 + respawn 容器，
  amcl/bt_navigator/controller_server 多实例抢 TF//cmd_vel。
  用 `cleanup_sim.sh` 清理，确认每个节点名只出现一次。
- Nav2 参数曾未加载：`nav2.launch.py` 现在真正传 `params_file` 并注入阿克曼 BT。
- MPPI `CostCritic.consider_footprint` 必须配 costmap footprint，否则 configure SIGSEGV。
- 厂家固件无速度看门狗；串口桥已在上位机兜底，S6 固件需加 300ms 看门狗。
- 厂家 `ax_kinematics.c:107` 单位 bug；S6 改为可标定舵机映射。

## 必记命令

```bash
# 清理一切残留（单独一条命令，勿与 launch 串在同一行）
ros2 run car_bringup cleanup_sim.sh

# 仿真 + Nav2 + RViz（AMCL 自动初始位姿 0,0）
ros2 launch car_bringup sim_nav2.launch.py

# 只仿真 / 只 Nav2（先 Gazebo，后 Nav2；Nav2 只能一套）
ros2 launch car_bringup gazebo.launch.py
ros2 launch car_bringup nav2.launch.py rviz:=false

# 2D 建图（与 Nav2 互斥）
ros2 launch car_bringup mapping.launch.py
ros2 run nav2_map_server map_saver_cli -f ~/car_ws/src/car_bringup/maps/my_map

# 融合实验（不启动 Nav2）
ros2 launch car_bringup fusion.launch.py
ros2 run car_bringup fusion_test.py --loops 3

# 控制/跟踪/验收
ros2 run car_bringup drive_test.py --vx 0.1 --wz 0.3
ros2 run car_bringup acceptance_test.py --vx 0.5
ros2 run car_bringup path_tracker.py --tracker lqr --path line --offset 0.3
```

## 关键文件（改代码先看这）

- 执行/控制：`src/r5_steering_controller/src/r5_steering_controller.cpp`、
  `src/car_bringup/config/controllers.yaml`、`src/car_description/urdf/car/control.xacro`
- 规划/控制参数：`src/car_bringup/config/nav2_params.yaml`
- 启动：`src/car_bringup/launch/{sim_nav2,nav2,gazebo,mapping,fusion}.launch.py`
- 行为树：`src/car_navigation/behavior_trees/akm_navigate_*.xml`
- 融合：`src/car_bringup/scripts/sensor_fusion.py`
- 串口：`src/r5_serial/r5_serial/{protocol,serial_bridge,virtual_stm32}.py`
- 固件：`firmware/openctr_b20/`（vendor 源码在 `vendor/xtark_fw/`）
- 上游补丁：`patches/ros2_controllers-r5.patch`

## 已知缺口（按优先级）

1. **倒车稳定性**：MPPI 倒车终点横向纠偏慢；长距离（>1.5m）易过冲。
   先试 `reverse_penalty 0.5`、`PreferForwardCritic 0.2`、`vx_min -0.4`；
   仍不行用 BT 分段（每段 ≤0.6m）或换 RPP/TEB。
2. **融合未接入默认导航**：需让控制器停止发 `/odom`+TF，融合节点成为唯一
   odom 源；或换 `robot_localization`。当前只是实验节点。
3. **建图未进主流程**：`mapping.launch.py` 已可跑、可保存；Nav2 用保存后的地图。
4. 实车：S6 固件烧录/看门狗/标定 → S7 `SystemInterface` → S8 标定 → S9 集成。
5. 压力测试：窄通道、动态障碍、1 m/s 高速未做。

## 下一步建议

先做“倒车三连测”的参数迭代（用同一套 `sim_nav2`，目标 −0.8/−1.6/−2.4，
记录成功/用时/终态），把 3 次成功率提到 ≥2/3、40s 内；再考虑融合接入。
实车到货后直接按 `firmware/openctr_b20/README.md` 烧录 + 标定。

## 文档索引

- `docs/ENGINEERING_HANDOVER.md`：全栈分层/接口/改动指南（新人必读）
- `docs/READING_GUIDE.md`：文件定位/阅读顺序/S4-S6 报告/雷达/轴距
- `PROJECT_MAP.md`：文件树 + 进度总账 + 命令
- `src/r5_steering_controller/EXPERIMENT_LOG.md`：Round 1~16 排错逻辑链
- `src/r5_steering_controller/STAGE1_GUIDE.md`：控制器逐行/预测表
- `src/r5_serial/docs/S5_SERIAL_BRIDGE.md`：协议/真机 Checklist
- `firmware/openctr_b20/README.md`：S6 引脚/集成/PC 烧录
