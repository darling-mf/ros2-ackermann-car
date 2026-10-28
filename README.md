# ROS 2 阿克曼小车（TARKBOT R5）

Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11。
当前主线：**2D 雷达 + 阿克曼运动控制/导航闭环**；3D 激光方案（FAST-LIO /
FAST-LIO-SAM-QN / livox）已弃用，源码保留但不参与构建。
完整文件树、里程碑与命令速查见 **[PROJECT_MAP.md](PROJECT_MAP.md)**。

## 当前状态

| 模块 | 状态 | 验证方式 |
|---|---|---|
| 底盘执行链（ros2_control + R5 阿克曼控制器） | 完成 | `acceptance_test.py` 三项 PASS，38 个上游单测全绿 |
| 跟踪算法（Pure Pursuit / Stanley / LQR + 增益调度） | 完成 | `path_tracker.py` 六组对比数据 |
| 传感器融合（/odom + /imu 互补滤波） | 完成 | 圆周实验：融合 11.7° vs 轮式 85° |
| 串口桥（X-Protocol 与 ROS 2，含虚拟 STM32） | 完成 | 七项测试（3% 坏帧注入、看门狗证据） |
| Nav2 倒车分流（S4） | 完成 | 正后方目标实测：后退 0.93m、航向不变 |
| STM32 固件（S6） | 模块完成 | `firmware/openctr_b20` 主机单测 5/5 通过；待实车 |
| HW 插件 / 标定 / 实机（S7~S9） | 待做 | 见 `src/car_bringup/ROADMAP_TO_REAL.md` |

## 构建与运行

```bash
cd ~/car_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-up-to car_bringup
source install/setup.bash

# 仿真（带界面 / 无界面）
ros2 launch car_bringup gazebo.launch.py
ros2 launch car_bringup gazebo.launch.py gui:=false rviz:=false

# 一键仿真 + Nav2 + RViz（默认；Nav2 只能起一套）
ros2 launch car_bringup sim_nav2.launch.py
ros2 run car_bringup cleanup_sim.sh          # 出现重复 Nav2/goal 无响应时先清理

# 2D 建图（与 Nav2 互斥）/ 传感器融合实验
ros2 launch car_bringup mapping.launch.py
ros2 launch car_bringup fusion.launch.py
ros2 control list_controllers        # joint_state_broadcaster + r5_steering_controller

# 控制自检 / 验收 / 跟踪对比
ros2 run car_bringup drive_test.py --vx 0.1 --wz 0.3
ros2 run car_bringup acceptance_test.py --vx 0.5
ros2 run car_bringup path_tracker.py --tracker lqr --path line --offset 0.3

# 串口链（无硬件联调）
ros2 run r5_serial virtual_stm32                 # 记下 /dev/pts/N
ros2 run r5_serial serial_bridge --port /dev/pts/N
```

键盘遥控：`ros2 run teleop_twist_keyboard teleop_twist_keyboard`（Twist → `/cmd_vel`）。

## 仓库结构（自研部分）

```
├── docs/READING_GUIDE.md      # 阅读顺序 + S4/S6 报告 + 雷达/轴距指南
├── patches/                   # 上游 ros2_controllers 补丁
├── firmware/openctr_b20/      # S6 固件模块（纯 C，主机单测通过）
├── hardware/openctr_b20/      # 厂家固件源码（本地，不提交）
└── src/
    ├── car_bringup/           # 启动、配置、全部实验脚本、路线图
    ├── car_description/       # URDF + Gazebo 世界（R5 实车尺寸/限位）
    ├── r5_steering_controller/# 自研 ros2_control 阿克曼插件（限幅三件套）
    ├── r5_serial/             # X-Protocol 串口桥 + 虚拟 STM32
    └── car_navigation/        # Nav2 阿克曼行为树（无 Spin，大 BackUp）
```

## 外部依赖与补丁

`ros2_control`、`ros2_controllers`、`gz_ros2_control` 以及 3D 雷达仓库
**不在本仓库内**（见 `.gitignore`）。对 `ros2_controllers` 的 4 处修复
（协方差 bug、转向限幅、FK 往返一致性、CMake 头文件顺序）归档在
**`patches/ros2_controllers-r5.patch`**，应用方式见 `patches/README.md`。

## 关键文档

| 文档 | 内容 |
|---|---|
| `docs/PROJECT_STATE.md` | **压缩记忆**：当前状态/已验证数据/缺口/命令 |
| `docs/ENGINEERING_HANDOVER.md` | **全栈架构交接**：分层/接口/运动控制/规划/雷达/学习路线 |
| `docs/READING_GUIDE.md` | 文件定位 + 阅读顺序 + S4/S6 报告 + 雷达选型 + 轴距实测 |
| `PROJECT_MAP.md` | 文件树全景 + 进度总账 + 命令速查 |
| `src/car_bringup/CONTROL_DEBUG_NOTES.md` | S0 八大 bug 的根因切割与方法论 |
| `src/car_bringup/ROADMAP_TO_REAL.md` | S0~S9 实机路线图与完成判据 |
| `src/r5_steering_controller/STAGE1_GUIDE.md` | 控制器数据流/逐行解析/修改预测表 |
| `src/r5_steering_controller/EXPERIMENT_LOG.md` | Round 1~14 实验逻辑链（最有价值的复盘） |
| `src/r5_serial/docs/S5_SERIAL_BRIDGE.md` | 固件源码提取的协议规格 + 真机 Checklist |

## 安全提醒

厂家固件**没有速度看门狗**（ROS 断流会保持最后速度）。`serial_bridge`
已在上位机侧实现 0.5 s 看门狗兜底；实车调试前必须架空车轮，S6 固件改造
时补下位机 300 ms 看门狗。固件参数（轴距 0.146 / 轮距 0.160 / R_min 0.35）
与手册（0.112 / 0.164 / 0.55）存在冲突，**以实车卷尺实测为准**（S8）。
