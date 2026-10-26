# 阅读指南与 S4/S6 交付报告

> 更新：2026-10-26。回答三件事：**截图里的文件在哪、按什么顺序读；
> S4/S6 做了什么、怎么用；2D 雷达怎么选、轴距怎么定**。
> 配套总账：根目录 `PROJECT_MAP.md`。

---

## 一、截图“优先级总表”的文件位置（全部核对过）

> 厂家源码在 `vendor/xtark_fw/OpenCTR_B20V11_AKM_V1.00.241015/`（已提交，UTF-8）；
> 原始 GBK 副本在 `vendor/xtark_fw_original_gbk/`，PDF 在 `vendor/xtark_docs/`。

| 优先级 | 文件 | 实际路径 | 读法 |
|---|---|---|---|
| ① 逐行 | `ax_kinematics.c` | `vendor/xtark_fw/OpenCTR_B20V11_AKM_V1.00.241015/Robot/ax_kinematics.c`（核心 62-140 行） | 拿计算器逐行算，重点 `ROBOT_AKM_Kinematics` 的 IK/FK 与舵机映射 |
| ① 逐行 | `ax_robot.c` 的 `Robot_Task` + `ROBOT_SendDataToRos` | 同目录 `Robot/ax_robot.c`（任务 92-131 行；发送 194-232 行） | 主循环骨架：20ms 周期、0x10 载荷布局 |
| ① 逐行 | `ax_uart_pi.c` 中断 ↔ `protocol.py` | 同目录 `Driver/ax_uart_pi.c`（114-240 行）↔ `src/r5_serial/r5_serial/protocol.py` | **镜像读**：两边状态机逐分支对照 |
| ① 逐行 | `ax_robot.h` 148-166 参数区 | `Robot/ax_robot.h` | 每个数溯源（轴距/轮距/轮径/分辨率/限速） |
| ① 逐行 | `steering_odometry.cpp`（我们打过补丁） | `src/ros2_controllers/steering_controllers_library/src/steering_odometry.cpp`（本地 overlay，不提交；补丁见 `patches/ros2_controllers-r5.patch`） | 补丁处重点：`update_from_velocity` 的 FK 反变换、`get_commands` 的限幅 |
| ① 逐行 | `r5_steering_controller.cpp`（自研） | `src/r5_steering_controller/src/r5_steering_controller.cpp` | 对照 `STAGE1_GUIDE.md`：命令层三功能 |
| ② 函数级 | `path_tracker.py` | `src/car_bringup/scripts/path_tracker.py` | 每个函数看懂即可（DARE/PP/Stanley） |
| ② 函数级 | `sensor_fusion.py` | `src/car_bringup/scripts/sensor_fusion.py` | 互补滤波 v2、sim-time dt |
| ② 函数级 | `serial_bridge.py` | `src/r5_serial/r5_serial/serial_bridge.py` | 50Hz 收发、看门狗、话题映射 |
| ③ 浏览 | `gazebo.launch.py` / `nav2.launch.py` | `src/car_bringup/launch/` | 知道启动顺序与参数注入即可 |
| ③ 浏览 | `acceptance_test.py` / `virtual_stm32.py` | `src/car_bringup/scripts/` / `src/r5_serial/r5_serial/` | 测试与模拟器行为 |
| ③ 浏览 | `ax_control.c` | `vendor/xtark_fw/OpenCTR_B20V11_AKM_V1.00.241015/Robot/ax_control.c` | PS2/APP/SBUS 控制源；ROS 模式不走这里 |
| ✕ 跳过 | ros2_control 框架内部、OLED/RGB/按键/PS2/SBUS、R10 宏块、vendor 其余 | — | 与当前主线无关 |

**推荐阅读顺序（约 12~14 小时）**：
1. `ax_robot.h` 参数区（0.5h）→ 建立“实车到底多大”的锚点；
2. `ax_kinematics.c`（3h，计算器）→ 自己推出 β、内外轮角，再对照我们的 `ackermann.c`；
3. `ax_uart_pi.c` ↔ `protocol.py`（2h，镜像）→ 协议吃透；
4. `ax_robot.c` 两个函数（1.5h）→ 50Hz 数据帧；
5. `steering_odometry.cpp`（3h）→ 上游 FK/IK，对照 `patches/`；
6. `r5_steering_controller.cpp`（1.5h）→ 我们的约束层；
7. 三个 Python（各 1~2h）→ 算法与桥；
8. launch/测试（半天）→ 会跑即可。

---

## 二、S4：Nav2 倒车分流（已完成并实测）

### 改了什么 / 为什么

| 文件 | 修改 | 原因 |
|---|---|---|
| `launch/nav2.launch.py` | **把 `nav2_params.yaml` 真正传给 Nav2**（原来没传，一直在用 Nav2 默认参数！）；注入阿克曼行为树路径；新增 `rviz` 开关 | 不传 `params_file`，之前所有 Nav2 调参都没生效 |
| `config/nav2_params.yaml` | `reverse_penalty: 2.0 → 0.8` | 让 Smac-Hybrid 愿意生成 Reeds-Shepp 倒车段 |
| 同上 | `vx_min: -1 → -0.3`；`velocity_smoother` 同步 | 倒车限速，室内安全 |
| 同上 | `progress_checker 0.5m/10s → 0.2m/15s` | 小车 + 倒车机动，原判据太苛刻 |
| 同上 | `xy_goal_tolerance 0.25 → 0.15`，`yaw 0.25 → 0.3` | 到点更准，但朝向放宽（避免原地较劲） |
| 同上 | 两个 costmap 用**矩形 footprint**（0.28×0.22）替代 `robot_radius` | MPPI `CostCritic.consider_footprint: true` 与 `robot_radius` 不匹配会在 configure 时段错误（实测 SIGSEGV） |
| `behavior_trees/akm_navigate_*.xml` | 基于官方 BT：**删除 Spin**，BackUp 0.30m/0.05 → **0.60m/0.12** | 阿克曼不能原地转；恢复时用倒车更有用 |

### 实测证据

- 目标：正后方 `(-1.0, 0.0)`，期望航向不变；
- 结果：`Goal finished with status: SUCCEEDED`；
- 最终 Gazebo 位姿：**(-0.906, -0.251, yaw≈0.016 rad)**——后退约 0.93 m、航向几乎未变，证明是**倒车**而非掉头；
- BT 参数核对：`/bt_navigator default_nav_to_pose_bt_xml` 指向 `.../akm_navigate_to_pose.xml`；`FollowPath.vx_min=-0.3`。

### 怎么复现

```bash
ros2 launch car_bringup gazebo.launch.py gui:=false rviz:=false
ros2 launch car_bringup nav2.launch.py rviz:=false
# 另一个终端：设初始位姿
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
 "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.07]}}"
# 发正后方目标
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
 "{pose: {header: {frame_id: map}, pose: {position: {x: -1.0, y: 0.0}, orientation: {w: 1.0}}}}"
# 看最终位姿（yaw 应仍≈0）
gz model -m ackermann_car -p
```

> 注意：`wz_max` 等 MPPI 参数必须是**浮点**（`1.0` 而非 `1`），否则
> controller_server configure 报类型错误并 abort。

---

## 三、S6：STM32 固件改造（模块完成，等硬件联调）

### 已交付（可在 PC 上验证）

目录 `firmware/openctr_b20/`：

| 文件 | 内容 | 验证 |
|---|---|---|
| `src/xprotocol.c/.h` | X-Protocol 解析/打包（与厂家中断逐分支一致） | 主机单测：官方示例帧、20 字节回环、坏帧重同步 |
| `src/safety.c/.h` | **下位机 300ms 速度看门狗** | 主机单测：上电禁止、300ms 边界、恢复 |
| `src/ackermann.c/.h` | β/内轮角/舵机指令（修厂家单位 bug，舵机参数化可标定） | 主机单测：β=0.284、内轮 19.2°、最小半径钳位 |
| `test/test_host.c` | 5 组测试 | `gcc` 全 PASS（已跑） |
| `README.md` | 引脚表、5 个集成点、PC 烧录三种方式、S8 标定清单 | — |

运行主机测试：

```bash
cd ~/car_ws/firmware/openctr_b20/test
gcc -std=c99 -Wall -Wextra -Werror -I../src test_host.c \
    ../src/xprotocol.c ../src/safety.c ../src/ackermann.c -lm -o /tmp/test_host && /tmp/test_host
```

### 硬件事实（从厂家源码抽取，直接可用）

| 项目 | 值 |
|---|---|
| MCU | **STM32F103RC**（256KB/48KB，Keil MDK 工程） |
| 电机 PWM | TIM4 CH1-4 → PB6/PB7/PB8/PB9（20kHz） |
| 编码器 | A: TIM2(PA15/PB3)；B: TIM3(PB4/PB5)；1560 counts/rev |
| 转向舵机 | TIM8_CH2 → **PC7**，1500±1.111×angle（0.1°） |
| MPU6050 | 软件 I2C：PA11(SDA)/PA12(SCL) |
| 树莓派串口 | UART4 PC11(RX)/PC10(TX)，**230400 8N1** |
| USB 调试 | UART1 PA10/PA9 → CH340，230400，与 UART4 同时发 0x10 |
| 固件参数 | 轴距 0.146 / 轮距 0.160 / R_min 0.35 / 轮径 65mm（**与手册冲突，待实测**） |

### PC 端烧录（按推荐排序）

1. **ST-Link V2 + STM32CubeProgrammer**（最稳）：SWDIO=PA13、SWCLK=PA14、GND、3.3V，BOOT0=0；Keil 工程 `Project/xproject.uvprojx` 可直接编译。
2. **USB 串口 ISP（板载 CH340，无需调试器）**：BOOT0=1 → 复位 → PC 上 `stm32flash -w firmware.hex -v -b 115200 /dev/ttyUSB0` → BOOT0=0 → 复位。前提是板上 BOOT0 可操作。
3. **PlatformIO**：不建议整体迁移厂家 SPL+FreeRTOS+Keil 启动工程；我们三个模块已用 `platform = native` 做主机测试，固件仍推荐 Keil 编译 + 上述烧录。

### 实车联调顺序（安全）

架空车轮 → 上电 0.5s 内不发指令（电机应零输出）→ 发 0x50 后断流 1s（车必须停）→ 发小速度验证方向 → 落地。

---

## 四、2D 雷达怎么选（适配你的 Pi5 + Nav2）

| 型号 | 接口/供电 | 量程/频率 | ROS 2 Humble 驱动 | 评价 |
|---|---|---|---|---|
| **RPLIDAR A1M8**（推荐） | USB 转接板，5V | 12m / 5.5Hz | `ros-humble-rplidar-ros` **apt 可直接装**（已确认仓库有 2.1.4） | 最省事，资料多，适合室内建图/AMCL |
| RPLIDAR S2 | USB，5V | 30m / 10Hz，IP65 | 同上 | 更贵，抗光强，室外也能用 |
| LD19 / LD06 | UART TTL 3.3~5V，230400 | 12m / 10Hz | 社区 `ldlidar_stl_ros2`（需源码编译） | 便宜小巧，需自己接 UART、调驱动 |
| YDLIDAR X4 | USB，5V | 10m / 7Hz | `ydlidar_ros2_driver`（源码） | 可用，驱动配置略繁琐 |

**选型建议**：优先 **RPLIDAR A1M8**（apt 一键装、USB 即插即用、与 Pi5 兼容），
预算紧再考虑 LD19。指标底线：360°、≥10Hz（A1 的 5.5Hz 用于 AMCL 也可，
但高速建图建议 10Hz）、量程 ≥10m、接口 USB 或 UART 230400。

**接入步骤**：
1. 雷达装车顶（与 `laser_link` 同高），USB 接 Pi5；
2. `sudo apt install ros-humble-rplidar-ros`，改 launch 的 `serial_port`/`frame_id=laser_link`；
3. 更新 `car_description/urdf/car/laser.xacro` 的 `samples/range/update_rate`；
4. 确认 `/scan` 频率与 Nav2 `scan_topic` 一致；建图用 `slam_toolbox`（已装），导航用 AMCL。

---

## 五、轴距还没定：一次实测裁决 0.112 / 0.146 之争

厂家固件写 0.146 m，图纸/手册写 0.112 m，差 30%——**必须实测**：

1. 车放平，用粉笔标记**前轴两轮中心连线**与**后轴两轮中心连线**；
2. 卷尺量两线垂直距离 = 真实轴距 H；同法量轮距 W、轮胎外径 D；
3. 满舵低速画圈，标记**后轴中心**在地面的轨迹，量半径 R_min；
4. 写入 4 处：`firmware/openctr_b20/src/ackermann.h` 的 `akm_cfg_t`、
   `car_description/urdf/common.xacro`、`car_bringup/config/controllers.yaml`
   （`wheelbase/front_wheel_track/rear_wheel_track/wheels_radius`）、
   `nav2_params.yaml` 的 `minimum_turning_radius/min_turning_r`。

---

## 六、目录整理结果（删了什么、留了什么）

删除（约 967MB + 陈旧文档）：
- `my_final_map/`（446M）、`my_mapping_bag/`（521M）——早期 2D 建图残留；
- 根目录 `ROADMAP.md`、`TROUBLESHOOTING.md`——已被 `PROJECT_MAP.md` /
  `ROADMAP_TO_REAL.md` 取代；
- `src/PROJECT_MAP.md` 重复副本、`r5_steering_controller/image.png`、
  旧 `my_custom_bt.xml`（被两个阿克曼专用 BT 取代）。

保留 / 新增：
- **3D 雷达相关全部保留**（FAST_LIO / FAST-LIO-SAM-QN / livox_ros_driver2 / 3D xacro）；
- `firmware/openctr_b20/`（S6 模块 + 文档）；`vendor/`（厂家源码/PDF，已提交）；
- `patches/`（上游补丁）、`docs/READING_GUIDE.md`（本文）、`PROJECT_MAP.md`。

最终结构：

```
car_ws/
├── README.md                 # 项目入口
├── PROJECT_MAP.md            # 文件树 + 进度总账 + 命令速查
├── docs/READING_GUIDE.md     # 本文（阅读顺序/S4/S6/雷达/轴距）
├── patches/                  # ros2_controllers 补丁
├── firmware/openctr_b20/     # S6 固件模块（主机测试通过）
├── vendor/                    # 厂家源码（UTF-8 + 原始 GBK）与 PDF 手册
└── src/                      # car_bringup / description / r5_* / navigation
```

---

## 七、下一步

1. **S8 标定**（有车就能做）：轴距/轮距/轮径/R_min/舵机 gain-offset；
2. **S7 HW 插件**：把 `r5_serial` 的串口层包成 `SystemInterface`，上层控制器零改动；
3. **S4 进阶**：窄通道 + 动态障碍压力测试，按 `EXPERIMENT_LOG` 的循环记录数据；
4. 雷达到货后：先建图（slam_toolbox）→ 存图 → AMCL + Nav2 闭环。
