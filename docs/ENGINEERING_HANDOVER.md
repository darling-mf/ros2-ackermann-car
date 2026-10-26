# 工程交接文档（Engineering Handover）

> 面向：第一次接手本项目的新人 / 未来的自己。
> 目标：**从上到下看懂每一层、知道每层核心文件与接口、明确改什么该动哪里**。
> 日期：2026-10-26。配套文档见 §12。
> 本文只讲“架构与改动方法”；具体 bug 复盘见 `src/r5_steering_controller/EXPERIMENT_LOG.md`。

---

## 0. 先回答：仿真层面做完了吗？

**主链路已经闭环可用**（控制 / 定位 / 全局规划 / 局部控制 / 倒车 / 串口虚拟联调
全部有实测），但作为“工程级交付”大约 **80%**，还差三块——它们不阻塞你现在学习，
但要知道边界：

| 能力 | 状态 | 证据 | 缺口 |
|---|---|---|---|
| 底盘执行（ros2_control + R5 控制器） | ✅ | 三项验收 PASS；38 上游单测 | 实车硬件接口（S7） |
| 跟踪算法（PP/Stanley/LQR） | ✅ | 六组对比 + 增益调度 | 未接入 Nav2 主链路（是实验台） |
| 状态估计（AMCL + 融合实验） | 🟡 | AMCL 导航可用；融合 11.7° | **融合节点未接入默认启动**，`/odom` 仍是轮式 |
| 全局规划（Smac Hybrid + Reeds-Shepp） | ✅ | 正后方目标生成倒车段 | 窄通道/动态障碍未压测 |
| 局部控制（MPPI Ackermann） | ✅ | 倒车 0.93m 到达、航向不变 | Humble MPPI 对长倒车段跟踪一般 |
| 行为树（无 Spin、大 BackUp） | ✅ | `akm_navigate_*.xml` 已接入 | 复杂恢复策略未做 |
| 2D 建图（slam_toolbox） | 🟡 | 已有地图 `my_room_map` | 建图 launch 未纳入仓库主流程 |
| 串口桥 + 虚拟 STM32 | ✅ | 七项测试（坏帧/看门狗） | 真实串口/硬件待联调 |
| STM32 固件模块 | 🟡 | 主机单测 5/5 | 实车烧录/标定 |
| 实车（S7/S8/S9） | ⬜ | — | 需硬件 |

**一句话**：**仿真作为一个“2D 雷达阿克曼导航实验台”已经完成**；
作为一个可交付产品，还差“融合默认接入 + 压力测试 + 真实硬件替换”。

---

## 1. 系统总览（从上到下）

```
┌──────────────────────────────────────────────────────────────────────┐
│ L5 任务/行为层   bt_navigator + BehaviorTree XML                     │
│    核心：src/car_navigation/behavior_trees/akm_navigate_*.xml        │
│    接口：Nav2 Action（NavigateToPose / ComputePathToPose / FollowPath）│
├──────────────────────────────────────────────────────────────────────┤
│ L4 全局规划层   planner_server + SmacPlannerHybrid（Reeds-Shepp）     │
│    核心：src/car_bringup/config/nav2_params.yaml 的 planner_server    │
│    接口：/plan (nav_msgs/Path)                                        │
├──────────────────────────────────────────────────────────────────────┤
│ L3 局部控制层   controller_server + MPPI（motion_model=Ackermann）     │
│    核心：nav2_params.yaml 的 controller_server.FollowPath             │
│    接口：/cmd_vel_nav (geometry_msgs/Twist)                           │
├──────────────────────────────────────────────────────────────────────┤
│ L2 速度整形/安全 velocity_smoother → /cmd_vel；控制器超时；串口看门狗  │
│    核心：nav2_params.yaml 的 velocity_smoother；r5 控制器 timeout      │
│    接口：/cmd_vel (Twist)                                             │
├──────────────────────────────────────────────────────────────────────┤
│ L1 状态估计层   AMCL(map→odom) + 轮式/IMU 融合实验 + p3d 真值          │
│    核心：nav2_params.yaml 的 amcl；scripts/sensor_fusion.py            │
│    接口：/scan、/imu、/odom、TF(map→odom→base_footprint)              │
├──────────────────────────────────────────────────────────────────────┤
│ L0 底盘执行层   ros2_control + R5 阿克曼控制器                         │
│    核心：src/r5_steering_controller/src/r5_steering_controller.cpp     │
│    配置：src/car_bringup/config/controllers.yaml                      │
│    接口：/cmd_vel → 关节指令；关节状态 → /odom + TF                    │
├──────────────────────────────────────────────────────────────────────┤
│ L-1 仿真/硬件边界 Gazebo Classic + gazebo_ros2_control（现在）         │
│    核心：src/car_description/urdf/car/control.xacro（总接线板）        │
│    实车替换：S7 写 hardware_interface::SystemInterface                 │
├──────────────────────────────────────────────────────────────────────┤
│ L-2 验证/工具   drive_test / acceptance_test / path_tracker /          │
│                fusion_test / plan_inspector / virtual_stm32            │
└──────────────────────────────────────────────────────────────────────┘
```

**读图方法**：每层只关心“我收到什么、我发什么”；层与层之间只通过 ROS2 话题/
Action/TF 通信。改代码时先定位层，再找核心文件，避免在错误的层里改参数。

---

## 2. 每一层：职责、核心文件、接口、改动点

### 2.1 L5 任务/行为层（bt_navigator + BehaviorTree）

- **职责**：把“去某个目标”拆成 规划→跟踪→失败恢复 的流程；决定何时重规划、
  何时倒车/等待/清 costmap。
- **核心文件**：
  - `src/car_navigation/behavior_trees/akm_navigate_to_pose.xml`
  - `.../akm_navigate_through_poses.xml`
  - 注入逻辑：`src/car_bringup/launch/nav2.launch.py`
- **接口（承上启下）**：
  - 对上：`/navigate_to_pose`（`nav2_msgs/action/NavigateToPose`，RViz/脚本发）；
  - 对下：`ComputePathToPose`（调 planner_server）、`FollowPath`（调 controller_server）、
    `BackUp/Wait/ClearEntireCostmap`（调 behavior_server/costmap 服务）。
- **为什么删 Spin**：阿克曼不能原地转，默认 BT 的 `Spin` 会打满舵、轮胎死锁。
- **怎么改**：
  - 改恢复策略：编辑 XML（BackUp 距离/速度、Wait 时长、重试次数）；
  - 换 BT：修改 `nav2.launch.py` 注入的 `default_nav_to_pose_bt_xml`；
  - 加新节点：需要在 `bt_navigator.plugin_lib_names` 注册对应 BT 插件。
- **测试**：发正后方目标，看是否用倒车完成（见 §6.3）。

### 2.2 L4 全局规划层（planner_server + Smac Hybrid）

- **职责**：在 (x,y,yaw) 状态空间里找一条**运动学可行**的全局路径（含倒车段）。
- **核心文件**：`src/car_bringup/config/nav2_params.yaml` 的 `planner_server:`
- **接口**：`/plan`（`nav_msgs/Path`）；Action `/compute_path_to_pose`。
- **关键参数**：
  - `GridBased.plugin: nav2_smac_planner/SmacPlannerHybrid`
  - `motion_model_for_search: REEDS_SHEPP`（允许前进+倒车）
  - `minimum_turning_radius: 0.55`（与 MPPI `min_turning_r` 必须一致）
  - `reverse_penalty: 0.8`（越小越愿意倒车；原 2.0 导致"宁绕圈不倒车"）
  - `angle_quantization_bins: 72`（朝向离散，72=5°一格）
  - `reverse_penalty / change_penalty / non_straight_penalty / cost_penalty`
- **怎么改**：
  - 更愿意倒车 → 降 `reverse_penalty`（0.5~0.8），同时保证 MPPI `vx_min` 允许负速度；
  - 更贴障碍/更安全 → 调 costmap 的 `inflation_radius`，不是调 planner 的 cost_penalty；
  - 换规划器 → `NavFn`（差速/全向，**不尊重阿克曼**）不推荐；`SmacPlanner2D` 也不尊重
    最小半径；`StateLattice` 计算量大。当前 Hybrid 是正确选择。
- **测试工具**：`ros2 run car_bringup plan_inspector.py` 监听 `/plan`，统计前进/倒车
  里程与换向次数——这是判断"规划器到底给没给倒车段"的唯一直接证据。

### 2.3 L3 局部控制层（controller_server + MPPI）

- **职责**：跟随 `/plan`，在局部 costmap 上避障，输出 `(vx, wz)`。
- **核心文件**：`nav2_params.yaml` 的 `controller_server.FollowPath`
- **接口**：输入 `/plan` + 局部 costmap + TF；输出 `/cmd_vel_nav`。
- **关键参数**：
  - `plugin: nav2_mppi_controller/MPPIController`
  - `motion_model: Ackermann`、`AckermannConstraints.min_turning_r: 0.55`
  - `vx_max/vx_min/wz_max`、`acc_lim_x/acc_lim_theta`
  - `time_steps: 56`、`model_dt: 0.05`（视界 2.8s）、`batch_size: 2000`（CPU 大户）
  - Critics：`PreferForwardCritic`（**倒车意愿的关键**，0.5 已很低）、
    `PathAlign/PathFollow/PathAngle`（贴线）、`CostCritic`（避障）、
    `ConstraintCritic`（运动学约束）
- **怎么改**：
  - 倒车不执行 → 降 `PreferForwardCritic.cost_weight`（先确认 plan 里有 R 段）；
  - 更贴线 → 提高 PathAlign/PathFollow；更平顺 → 提高 `batch_size`/`time_steps`（费 CPU）；
  - 窄通道 → 改 `local_costmap` footprint/inflation，不是改 MPPI 权重；
  - 换控制器 → RPP 已随 Nav2 安装（`nav2_regulated_pure_pursuit_controller`），可直接在 `controller_plugins` 切换；TEB 当前未安装，需源码编译。
- **注意**：Humble 的 MPPI 对**长距离倒车段**跟踪一般；若失败，优先用 BT 分段
  或降低倒车距离，而不是无限调权重。

### 2.4 L2 速度整形与安全

- **velocity_smoother**：`nav2_params.yaml` → 输入 `cmd_vel_nav`、输出 `/cmd_vel`；
  限制加速度与最大速度。它是 L3 与 L0 之间的"最后一道理性"。
- **控制器超时**：`controllers.yaml` 的 `reference_timeout: 0.5`——0.5s 没
  `/cmd_vel` 就把牵引速度置零（转向保持）。
- **串口看门狗**：`src/r5_serial/r5_serial/serial_bridge.py`——0.5s 断流发零速帧；
  这是实车安全的**上位机兜底**（厂家固件没有看门狗）。
- **怎么改**：限速改 `velocity_smoother.max_velocity`；断流时间改
  `reference_timeout`/`--watchdog`；不要把加速度限幅写到轮子层（会破坏阿克曼比例）。

### 2.5 L1 状态估计层

- **AMCL**：`nav2_params.yaml` 的 `amcl:`；输入 `/scan` + `/odom` + map，
  输出 `map→odom` TF。核心参数：粒子数、`update_min_d/a`、`max_beams`、
  `laser_max_range`、`likelihood_field`。
- **轮式里程计**：由 L0 控制器内部 FK 产生 `/odom` + `odom→base_footprint`。
  **不是**传感器融合结果（融合节点默认没启动）。
- **融合实验**：`src/car_bringup/scripts/sensor_fusion.py` +
  `/odometry/fused`；`fusion_test.py` 量化。**尚未接入默认 launch**——
  这是仿真层最大的缺口之一。
- **真值**：`control.xacro` 里的 p3d 插件 → `/ground_truth`（仅仿真，用于量化）。
- **怎么改**：
  - 要接融合：把 `sensor_fusion.py` 加入 launch，并把 `/odom` 的消费者
    （Nav2/控制器）改为订阅 `/odometry/fused`；或直接换 `robot_localization` EKF；
  - 定位漂移 → 先查 TF 时间戳/初始位姿/雷达量程，再调 AMCL 粒子参数。

### 2.6 L0 底盘执行层（ros2_control）

- **职责**：把 `(vx, wz)` 变成 4 个关节指令；把关节状态变成 `/odom`。
- **核心文件**：
  - `src/r5_steering_controller/src/r5_steering_controller.cpp`（**自研核心**）
  - `src/ros2_controllers/steering_controllers_library/src/steering_odometry.cpp`
    （上游基类，补丁见 `patches/ros2_controllers-r5.patch`）
  - `src/car_bringup/config/controllers.yaml`（参数）
  - `src/car_description/urdf/car/control.xacro`（接口声明 + Gazebo 插件 + remap）
  - `src/car_description/urdf/car/wheels.xacro`（关节限位/摩擦/轴符号）
- **接口（承上启下）**：
  - 对上：订阅 `/cmd_vel`；发布 `/odom` + `odom→base_footprint` TF + `~controller_state`；
  - 对下：`command_interfaces_`（后轮 velocity×2、前轮 position×2）→
    `gazebo_ros2_control` → Gazebo 关节；`state_interfaces_` 反向读取。
- **R5 控制器三功能**（都在 `update_and_write_commands` 的命令层后处理）：
  1. 驱动轮速度斜坡（保护 AT8236 电流）；
  2. 速度相关转向限幅（`a_y=v²tanφ/L ≤ a_lat_max`）；
  3. 转向速率限幅（舵机 0.11s/60°）。
  **为什么在命令层**：非链式模式下父类每周期开头从订阅者重载
  `reference_interfaces_`、结尾清 NaN；子类在调用父类前写的限幅值会被覆盖
  （详见 EXPERIMENT_LOG Round 1）。命令接口在 update 返回后才被硬件读取，
  所以“读出→限幅→写回”确定生效。
- **怎么改**：见 §5。

### 2.7 L-1 仿真/硬件边界

- **当前**：`control.xacro` 的 `<ros2_control>` 用
  `gazebo_ros2_control/GazeboSystem` 插件；Gazebo Classic 提供物理与传感器。
- **实车替换**：S7 写 `hardware_interface::SystemInterface` 插件，导出同样的
  command/state 接口（后轮速度×2 + 舵角；轮速 + IMU），上层 R5 控制器**一行不改**。
  实车前轮是**单舵机+曲柄**，两个 steer 指令要在 HW 层合并成一个舵角——这正是
  ros2_control 分层的意义。
- **关键文件**：`control.xacro`（接口契约）、`firmware/openctr_b20/`（下位机）。

### 2.8 L-2 验证与工具

| 工具 | 证明什么 |
|---|---|
| `acceptance_test.py` | 加速度/转向限幅/转向速率三项 PASS/FAIL |
| `drive_test.py` | cmd_vel→odom 方向是否正确 |
| `path_tracker.py` | PP/Stanley/LQR 跟踪误差对比（独立实验台） |
| `fusion_test.py` | 轮式/陀螺/融合对 p3d 真值的误差 |
| `plan_inspector.py` | `/plan` 是否含倒车段 |
| `virtual_stm32.py` | 无硬件联调串口协议/看门狗/坏帧 |

---

## 3. 一次导航的完整数据流（按时间）

```
① RViz/脚本发 NavigateToPose 目标
        ↓ (Action)
② bt_navigator 载入 akm_navigate_to_pose.xml
        ↓ ComputePathToPose (Action)
③ planner_server（Smac Hybrid + Reeds-Shepp）
   输入：global_costmap（map + /scan）、起点/目标
        ↓ /plan (nav_msgs/Path)
④ controller_server（MPPI, 20Hz）
   输入：/plan、local_costmap（odom + /scan）、TF
        ↓ /cmd_vel_nav
⑤ velocity_smoother（20Hz）
        ↓ /cmd_vel
⑥ r5_steering_controller（50Hz）
   读 /cmd_vel → 阿克曼 IK → command_interfaces_
        ↓ 关节指令
⑦ gazebo_ros2_control → Gazebo 物理
        ↓ 关节状态
⑧ r5 控制器 FK → /odom + TF(odom→base_footprint)
   joint_state_broadcaster → /joint_states → robot_state_publisher → 其余 TF
        ↓
⑨ AMCL：/scan + /odom + map → TF(map→odom)
        ↓
⑩ costmap 更新 → 回到 ③/④（闭环）
```

**排查顺序**：目标没动 → 看 ②③（有 /plan 吗）；有 plan 不动 → 看 ④⑤（/cmd_vel_nav、
/cmd_vel）；有 cmd_vel 不动 → 看 ⑥⑦（控制器/关节/限位）；动但歪 → 看 ⑧⑨（TF/定位）。

---

## 4. TF 树与坐标系（最容易搞错）

```
map ──(AMCL 发布)──> odom ──(r5 控制器发布)──> base_footprint
                                                    │
                                        (robot_state_publisher 固定变换)
                                                    ↓
                                                 base_link
                                        ┌───────────┼───────────┐
                                     laser_link   imu_link   steering/wheel links
```

- **map→odom**：AMCL 发布，需要初始位姿（RViz 2D Pose Estimate 或 `/initialpose`）；
- **odom→base_footprint**：控制器发布，`enable_odom_tf: true`；
- **base_footprint→base_link**：URDF 固定关节，robot_state_publisher 发布；
- **Gazebo world 不是 TF 帧**：p3d 的 `/ground_truth` 是世界系数值，**不要**
  把它和 `map` 直接比较；实验前要按 t0 做刚体对齐（Round 13 的教训）；
- **base_footprint 在两轴中点**：控制器里程计按后轴模型积分，急转弯会有
  几厘米杆臂误差（已知限制，AMCL 会修正）；
- **雷达**：`laser_link` 在 base_link 上前方 0.074m、高 0.067m；`/scan.frame_id`
  必须是 `laser_link`。

---

## 5. 运动控制：看什么、改什么

### 5.1 必读顺序（约 1 天）

1. `src/car_bringup/config/controllers.yaml`——参数与关节顺序契约；
2. `src/car_description/urdf/car/control.xacro`——接口声明/remap/插件；
3. `src/r5_steering_controller/src/r5_steering_controller.cpp`——三功能；
4. 上游 `steering_odometry.cpp`（`get_commands`/`update_from_velocity`）+ `patches/`；
5. `acceptance_test.py`——三项判据。

### 5.2 常见改动 recipe

| 想改什么 | 改哪里 | 注意 |
|---|---|---|
| 加速/减速更猛/更柔 | `controllers.yaml` 的 `r5.max_accel/max_decel` | 重启仿真；`drive_test` 看斜坡 |
| 高速转向更保守 | `r5.a_lat_max`（默认 2.0） | 1.2 m/s 时 φ_cap≈0.155；调小更保守 |
| 转向更快/更慢 | `r5.max_steer_rate` + URDF steer joint `velocity` | 两处要一致（舵机 9.5 rad/s） |
| 机械转角上限 | **四处同步**：URDF `lower/upper`、`controllers.yaml max_steering_angle`、固件 `akm_cfg`、Nav2 `min_turning_r` | 漏一处就会出现"指令/实际/规划"三方矛盾 |
| 轮径/轴距/轮距 | **四处**：`common.xacro`、`controllers.yaml`、固件 `akm_cfg`、Nav2 最小半径 | 轮径直接影响里程计与最大车速 |
| 新增控制功能 | `r5_steering_controller.cpp` 的 `update_and_write_commands` 命令层后处理 | 不要在参考层写（会被父类覆盖） |
| 换控制频率 | `controllers.yaml update_rate` + Nav2 `model_dt`/`time_steps` | 50Hz 与 Gazebo 1000Hz 物理步长无冲突 |
| 换底盘类型 | 换控制器插件（diff_drive/bicycle/tricycle），改 URDF/参数 | 阿克曼不要用 diff_drive |

### 5.3 调试与验收

```bash
ros2 run car_bringup drive_test.py --vx 0.1 --wz 0.3     # 方向/转角/轮速
ros2 run car_bringup acceptance_test.py --vx 0.5         # 三项限幅
ros2 run car_bringup acceptance_test.py --vx 1.0         # 高速 a_lat 生效区间
```

**预测习惯**：改参数前先写预测（收敛时间/RMS/饱和占比），跑完对答案；
详见 `STAGE1_GUIDE.md` 的 12 条修改预测表。

### 5.4 不要做的事

- 不要直接改 `/opt/ros/humble` 的二进制；源码 overlay 的修改要进 `patches/`；
- 不要在轮子层单独夹速度（破坏内外轮比例，车会画龙）；
- 不要用加大阻尼/摩擦去治“接线错误”（先验证左右顺序与轴符号）；
- 不要改基类 `SteeringOdometry` 的**类布局**（与 `/opt` 预编译控制器 ABI 冲突）。

---

## 6. 路径规划：看什么、改什么

### 6.1 必读

1. `nav2_params.yaml` 的 `planner_server`、`controller_server`、两个 costmap；
2. `nav2.launch.py`（怎么加载参数、注入 BT）；
3. `akm_navigate_to_pose.xml`（恢复策略）；
4. `plan_inspector.py`（判断有没有倒车段）。

### 6.2 recipe

| 想改什么 | 改哪里 | 注意 |
|---|---|---|
| 允许/更愿意倒车 | `reverse_penalty` 0.8→0.5；`vx_min` -0.3；`PreferForwardCritic` 0.5 | 先确认 plan 有 R 段；MPPI Humble 对长倒车段一般 |
| 掉头更利索 | `minimum_turning_radius`/`min_turning_r` 与实车 R_min 一致（0.35~0.55 待实测） | 两处必须一致，否则规划与执行打架 |
| 更贴线/更平滑 | MPPI 的 PathAlign/PathFollow 权重；`batch_size`/`time_steps` | 增加计算量；`model_dt` 与 `time_steps` 决定视界 |
| 窄通道 | costmap `footprint`/`inflation_radius`；必要时减小车速 | `CostCritic.consider_footprint` 必须与 footprint 配套 |
| 换全局规划器 | `planner_plugins` | NavFn/2D 不尊重最小半径；Hybrid 才是阿克曼正确解 |
| 换局部控制器 | `FollowPath.plugin` | RPP 已装可直接切；TEB 未装需编译；MPPI 当前可用 |
| 卡住恢复 | BT XML 的 BackUp/Wait/清 costmap | 阿克曼禁止 Spin |

### 6.3 测试与指标

```bash
# 先清理旧进程（Nav2 只能一套）
ros2 run car_bringup cleanup_sim.sh
# 一条命令启动（Gazebo + Nav2 + RViz，AMCL 自动初始位姿）
ros2 launch car_bringup sim_nav2.launch.py
# 或：Gazebo 稳定后单独起 Nav2
# 发正后方目标
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
 "{pose: {header: {frame_id: map}, pose: {position: {x: -1.0, y: 0.0}, orientation: {w: 1.0}}}}"
ros2 run car_bringup plan_inspector.py      # 看 R 段与换向次数
gz model -m ackermann_car -p                # 终态（yaw 不变→倒车）
```

指标：是否成功、用时、路径长度、倒车里程占比、终态位置/朝向误差、Recoveries 次数。

### 6.4 常见失败表

| 症状 | 第一动作 |
|---|---|
| 参数改了没效果 | `ros2 param get` 确认；检查 launch 是否传 `params_file` |
| controller configure abort | 看类型错误（`wz_max: 1` vs `1.0`）或 footprint/CostCritic 不一致 |
| 有 plan 但车不倒 | 看 `/cmd_vel` 是否出现负 vx；降 PreferForward |
| plan 全是 F | 降 `reverse_penalty`；确认 `REEDS_SHEPP` |
| 到点不结束 | `yaw_goal_tolerance` 太严 / 空间不够画弧 |
| 定位漂移 | AMCL 初始位姿、粒子数、`update_min_d/a`、TF 时间戳 |

---

## 7. 2D 激光雷达：学什么、改什么、核心在哪

### 7.1 仿真里的链路（先看懂这个）

```
src/car_description/urdf/car/laser.xacro
   └─ Gazebo ray sensor（360 samples, 10m, 10Hz, frame=laser_link）
        └─ libgazebo_ros_ray_sensor.so → /scan (LaserScan)
             ├─ AMCL（map→odom）
             ├─ local_costmap/voxel_layer
             └─ global_costmap/obstacle_layer
```

核心文件：`laser.xacro`（传感器参数）、`nav2_params.yaml` 的 `amcl`/costmap 段、
`maps/my_room_map.*`。仿真雷达是理想射线模型（无噪声、无玻璃穿透问题）。

### 7.2 实车雷达选型与接入

| 型号 | 接口 | 量程/频率 | 驱动 | 建议 |
|---|---|---|---|---|
| **RPLIDAR A1M8** | USB，5V | 12m/5.5Hz | `ros-humble-rplidar-ros`（apt 有） | **首选**，最省事 |
| LD19/LD06 | UART 230400 | 12m/10Hz | 社区 `ldlidar_stl_ros2` | 便宜小巧，需调 UART |
| RPLIDAR S2 | USB | 30m/10Hz | 同上 | 预算足/室外 |

接入步骤：
1. 装车顶（与 `laser_link` 同高），USB 接 Pi5；
2. 启动驱动，`frame_id=laser_link`，`/scan` 频率 ≥10Hz；
3. 改 `laser.xacro` 的 `samples/range/update_rate`（仿真与实车对齐）；
4. 实车加 `laser_filters` 去掉车体自遮挡（如需要）；
5. Nav2 的 `scan_topic`/costmap `observation_sources` 保持 `/scan`。

### 7.3 AMCL（定位核心）

- **原理**：粒子滤波；每个粒子是一个位姿假设，用**似然场模型**给 `/scan` 打分，
  重采样后收敛。输出 `map→odom`。
- **关键参数**：`min/max_particles`、`update_min_d/a`、`max_beams`、
  `laser_max_range`、`z_hit/z_rand`、`sigma_hit`、`transform_tolerance`。
- **失败模式**：初始位姿没给、对称走廊、动态人群、雷达被遮挡、TF 时间不同步、
  地图与现场不一致（家具移动）。
- **改法**：先设初始位姿；再调 `update_min_d/a`（越小越敏感但费 CPU）；
  粒子数越大越稳但越费 CPU；`transform_tolerance` 要大于 TF 延迟。

### 7.4 costmap（避障核心）

- **三层**：`static_layer`（地图）、`obstacle/voxel_layer`（雷达实时障碍）、
  `inflation_layer`（膨胀）。
- **关键参数**：`footprint`（必须与 MPPI CostCritic 配套）、`inflation_radius`、
  `cost_scaling_factor`、`obstacle_max_range`、`raytrace_max_range`、
  `marking/clearing`。
- **改法**：窄通道先减 inflation；别把 `obstacle_max_range` 设得比雷达量程还大；
  动态障碍需要 `voxel_layer` + 合理 `z` 范围。

### 7.5 建图（slam_toolbox）

- 已安装 `slam_toolbox`；流程：起 Gazebo → 起 slam_toolbox（online_async）→
  遥控走一圈 → `map_saver_cli` 保存到 `car_bringup/maps/`。
- 关键：雷达频率/时间戳、TF 完整、速度慢（≤0.3 m/s）、回环（原地转一圈再走）。
- 地图分辨率 0.05m 已够；地图坐标系原点会影响 AMCL 初始位姿。

### 7.6 需要学习的内容（按优先级）

1. **传感器模型**：似然场 vs 射线模型（Probabilistic Robotics 第 6 章）；
2. **粒子滤波/AMCL**（第 8 章）：重采样、粒子耗尽、 kidnapped robot；
3. **scan matching**：ICP/NDT 的基本思想（为 3D 回补打底）；
4. **costmap/膨胀**：为什么膨胀半径 = 车半径 + 安全裕度；
5. **TF 与时间同步**：`use_sim_time`、消息头时间、`transform_tolerance`；
6. **ROS2 QoS**：sensor data 常用 best effort；`/scan` 订阅端 QoS 不匹配会“看不到数据”；
7. **实车工程**：雷达安装角/外参、`laser_filters`、电磁干扰/丢帧。

### 7.7 本仓库里最值得改的激光雷达相关点

| 文件 | 可改什么 | 影响 |
|---|---|---|
| `laser.xacro` | 采样数/量程/频率/frame/安装位置 | 仿真保真度 |
| `nav2_params.yaml amcl` | 粒子数/更新阈值/量程 | 定位稳定性 |
| `nav2_params.yaml costmap` | footprint/inflation/观察源 | 避障与通行性 |
| `maps/*.yaml` | 分辨率/原点 | 定位与规划精度 |
| `nav2.launch.py` | map 路径/初始位姿流程 | 启动体验 |

---

## 8. 新人 7 天学习路线（每天有可验收动作）

| 天 | 读什么 | 动手 | 交付 |
|---|---|---|---|
| 1 | `README`/`PROJECT_MAP`/本文 §1-3 | 构建、起 Gazebo、`list_controllers`、`drive_test` | 能跑通并解释数据流 |
| 2 | `controllers.yaml` + `control.xacro` + `r5 cpp` | 改 `r5.max_accel`，预测并跑 `acceptance_test` | 一份参数-行为对照 |
| 3 | `steering_odometry.cpp` + `patches/` | 手算一次左转的 α_r/α_l，对比实测 | 公式-代码-数据三方对齐 |
| 4 | `nav2_params` planner/MPPI + BT | 发正后方目标，`plan_inspector` 看 R 段 | 能解释倒车为何成功 |
| 5 | `amcl`/costmap/`laser.xacro` | 设初始位姿、看粒子云、改 inflation 观察 | 定位/避障参数笔记 |
| 6 | `sensor_fusion.py`/`fusion_test.py` | 跑圆周实验，复现 11.7° | 融合误差表 |
| 7 | `firmware/.../README` + 主机测试 | 编译/跑 5 组 C 测试，读 5 个集成点 | 能讲清 S6 上实车步骤 |

---

## 9. 调试 Runbook（症状 → 第一动作）

| 症状 | 第一动作 | 文件/命令 |
|---|---|---|
| **Nav2 点目标无反应 / `Send goal call failed` / TF 两棵树** | **先查是否重复启动**：`ros2 node list \| grep -E "amcl\|bt_navigator"` 应各一个；`ros2 run car_bringup cleanup_sim.sh` 后重来 | §2.1/§6.4 |
| Nav2 里 planner 说 `Starting point in lethal space` | 初始位姿与实际位置不符：仿真用 `amcl.set_initial_pose`；实车用 RViz 2D Pose Estimate | nav2_params |
| 控制器不加载 | 读 `Available classes`；`ros2 pkg prefix` | §2.6 |
| `controller configure` abort | 查参数类型/footprint 一致性 | nav2 日志 |
| 车不动但控制器 active | `ros2 topic info -v /cmd_vel` 看订阅者 | §2.4 |
| 方向反 | `gz model -p` + joint_states 符号 | wheels.xacro |
| 画龙/不转弯 | 先验内外轮不等式，再查限位/摩擦 | §2.6 |
| Nav2 参数无效 | 确认 launch 传了 `params_file` | nav2.launch.py |
| 有 plan 不倒车 | `plan_inspector` + `PreferForward`/`reverse_penalty` | §6 |
| AMCL 不发布 map→odom | 设初始位姿；查 `/scan`/TF | §7.3 |
| 定位漂移 | 检查地图/雷达/时间戳/粒子数 | §7.3 |
| 仿真整体怪 | 车是否撞进家具/位姿漂了 | `gz model -p` 复位 |

---

## 10. 变更影响矩阵（改一处要同步哪些）

| 改动 | 必须同步 |
|---|---|
| 轴距/轮距/轮径 | URDF `common.xacro`、`controllers.yaml`、固件 `akm_cfg_t`、Nav2 `minimum_turning_radius/min_turning_r` |
| 转向限位 | URDF `wheels.xacro`、`controllers.yaml max_steering_angle`、固件、Nav2 最小半径 |
| 关节名/顺序 | URDF、`controllers.yaml`（右前左后）、`control.xacro` 无需改但 remap 名要对应 |
| 控制器名 | `controllers.yaml`、`gazebo.launch.py` spawner、`control.xacro` remap |
| cmd_vel 类型 | `controllers.yaml use_stamped_vel`、`control.xacro` remap、发布者 |
| 控制频率 | `controllers.yaml update_rate`、MPPI `model_dt/time_steps` |
| 地图 | `nav2.launch.py map`、`maps/*.yaml`、AMCL 初始位姿 |
| 雷达 | `laser.xacro`、`nav2_params scan_topic/observation_sources`、`amcl.laser_*` |
| BT | XML、`nav2.launch.py` 注入路径、`bt_navigator.plugin_lib_names` |
| 上游控制器代码 | `patches/`、重新 `colcon build`、跑 38 单测 |

---

## 11. 术语表

| 术语 | 含义 |
|---|---|
| ros2_control | 实时控制器框架：Controller（算法）↔ Hardware（执行） |
| chainable controller | 可串联的控制器（reference 接口），本项目 R5 控制器是 |
| FK/IK | 正/逆运动学：轮速转角→(v,ω)；反之 |
| AMCL | 自适应蒙特卡洛定位（粒子滤波） |
| MPPI | 采样式模型预测控制（Nav2 局部控制器） |
| Smac Hybrid | 状态格 A* + Reeds-Shepp，支持阿克曼/倒车 |
| BT | BehaviorTree，Nav2 的任务编排 |
| footprint | 机器人多边形轮廓，避障用 |
| TF | 坐标变换树 |
| X-Protocol | 厂家串口协议：`AA 55 LEN ID DATA CK` |

---

## 12. 配套文档索引

| 文档 | 用途 |
|---|---|
| `docs/READING_GUIDE.md` | 文件定位/阅读顺序/S4/S6 报告/雷达/轴距 |
| `PROJECT_MAP.md` | 文件树 + 进度总账 + 命令速查 |
| `src/car_bringup/ARCHITECTURE.md` | 控制层内部细节（类继承/control.xacro/controllers.yaml） |
| `src/r5_steering_controller/STAGE1_GUIDE.md` | 控制器数据流/逐行解析/修改预测表 |
| `src/r5_steering_controller/EXPERIMENT_LOG.md` | Round 1~15 实验逻辑链（排错方法论） |
| `src/r5_serial/docs/S5_SERIAL_BRIDGE.md` | 串口协议/固件差异/真机 Checklist |
| `firmware/openctr_b20/README.md` | S6 引脚/集成点/PC 烧录/标定 |
| `src/car_bringup/CONTROL_DEBUG_NOTES.md` | S0 八大 bug 根因切割 |
| `src/car_bringup/ROADMAP_TO_REAL.md` | S0~S9 实机路线图 |

---

*交接原则：先跑通，再画图，再改一个参数并预测结果。看不懂某一层时，先用
`ros2 topic echo/info` 看它的输入输出，再回代码。*
