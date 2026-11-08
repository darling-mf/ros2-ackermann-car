# PROJECT_STATE —— 压缩记忆（2026-11-08）

> 用途：上下文丢失时，先读这一页。详细展开见文末索引。

## 一句话状态

**仿真主链基本闭合**：控制/前向导航/2D 建图/串口已验证，融合 odom 已接管默认链路；
1 m/s 前向目标成功、AMCL 误差约 5–8 cm。剩余：左房间长路径 ABORTED、
动态障碍近墙终点会超时、窄通道未压测；实车 S7/S8/S9 未开始。

## 已验证（有数据）

| 模块 | 状态 | 证据 |
|---|---|---|
| 底盘执行 + R5 控制器 | ✅ | acceptance 三项 PASS；38 上游单测 |
| Nav2 前向导航 | ✅ | 目标 (2.0,0) `SUCCEEDED`、0 次恢复；AMCL 误差 3–5cm |
| Nav2 1 m/s 高速 | ✅ | (4.0,0) 8.2s、(6.0,0) 4.7s 均 `SUCCEEDED`；AMCL 误差 16→5.4~7.7cm |
| 动态障碍（终点离墙） | ✅ | 障碍 (4,0)、目标 (6,0)：12s `SUCCEEDED`、0 恢复；近墙 (8,0) 仍超时 |
| Nav2 倒车 | 🟡 | 短距旧参数 2/3；当前慢倒车 −0.15 定位稳定，但左房间/长路径仍 ABORTED |
| 2D 建图 | ✅ | `mapping.launch.py`；`/map` 376×222@0.05；`map_saver_cli` 保存成功 |
| 传感器融合 | ✅ | 默认链路：控制器 `/wheel/odom` 无 TF → 融合 `/odom` + 唯一 TF；前向回归 8s `SUCCEEDED` |
| 环境/RTF | ✅ | Gazebo/单套 Nav2 RTF≈0.99；残留进程清理后恢复 |
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

# 融合默认链路已包含在 sim_nav2/gazebo 中；单独验证时：
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

1. **倒车长距离**：≤1.6m 已稳定（2/3 标准达成）；>2m 仍会超时/过冲，
   用 BT 分段（每段 ≤0.6m）或换 RPP/TEB。当前参数：
   `reverse_penalty 0.5 / PreferForward 0.2 / vx_min -0.4 / 到点 0.20m`。
2. **动态障碍/近墙终点**：障碍 (4,0)/目标 (6,0) 已 12s 通过；但障碍 (6.5,0)/
   目标 (8.0,0) 贴墙时 60s 未收敛（车到 7.63,0.39）；需要更远的终点、
   分层恢复或 TEB 复测。
3. **建图未进主流程**：`mapping.launch.py` 已可跑、可保存；Nav2 用保存后的地图。
4. 实车：S6 固件烧录/看门狗/标定 → S7 `SystemInterface` → S8 标定 → S9 集成。
5. 压力测试：窄通道、死胡同、动态障碍系统化场景未做。

## 下一步建议

仿真侧优先做“窄通道 + 动态障碍 + 近墙终点”的场景化回归，并把左房间改成
Waypoint/BT 分段；这些都属于场景与行为树问题，不应再靠单一 MPPI 调参硬扛。
实车到货后直接按 `firmware/openctr_b20/README.md` 烧录 + 标定，再复测融合 odom。

## 文档索引

- `docs/ENGINEERING_HANDOVER.md`：全栈分层/接口/改动指南（新人必读）
- `docs/READING_GUIDE.md`：文件定位/阅读顺序/S4-S6 报告/雷达/轴距
- `reports/NAV2_DEBUG_REPORT.md`：Nav2 闪烁/TF 报错/自击代价地图完整排查链
- `PROJECT_MAP.md`：文件树 + 进度总账 + 命令
- `src/r5_steering_controller/EXPERIMENT_LOG.md`：Round 1~16 排错逻辑链
- `src/r5_steering_controller/STAGE1_GUIDE.md`：控制器逐行/预测表
- `src/r5_serial/docs/S5_SERIAL_BRIDGE.md`：协议/真机 Checklist
- `firmware/openctr_b20/README.md`：S6 引脚/集成/PC 烧录
