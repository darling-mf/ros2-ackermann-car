# S5：X-Protocol 串口桥 + 虚拟 STM32 —— 完整工程文档

> 代码：`src/r5_serial/`（ROS 2 ament_python 包）
> 状态：**已完成并联调通过**（虚拟串口全链路，无需硬件）
> 上游依据：厂商固件源码 `OpenCTR_B20V11_AKM V1.00.241015` 逐行提取
> （本地稳定路径：`vendor/xtark_fw/OpenCTR_B20V11_AKM_V1.00.241015/`，不提交 git），
> 非手册转述——手册与源码有多处不一致，**以源码为准**（差异见 §2）。
> 位置：实机路线图 S5（见 `car_bringup/ROADMAP_TO_REAL.md`），
> 完成后 S6（STM32 固件）到手即可"换设备名直连"。

---

## 1. 这一步干什么、为什么必须做

S5 在实机路线图上的位置：仿真闭环（S0~S3）与真硬件（S6 固件）之间的桥。

**要解决的问题**：实车集成时最大的风险不是"算法不对"（S0~S3 已在仿真
验证过），而是**通信层的意外**——字节序错了车倒着跑、校验和算错全丢帧、
看门狗没做断线飞车、波特率配错全哑。这类问题必须在没有硬件时全部暴露。

**S5 交付三件东西**：

| 交付物 | 文件 | 作用 |
|---|---|---|
| 协议编解码层 | `r5_serial/protocol.py` | X-Protocol 打包/解析/单位换算，与固件逐字节对照 |
| 虚拟 STM32 | `r5_serial/virtual_stm32.py` | 在 pty 虚拟串口上扮演固件：50Hz 发 0x10、解析 0x50/0x51/0x5f、可注入坏帧 |
| ROS 2 串口桥 | `r5_serial/serial_bridge.py` | /cmd_vel → 0x50（50Hz + 看门狗）；0x10 → /r5/wheel_vel、/r5/imu_raw、/r5/battery |
| 最小串口层 | `r5_serial/serial_port.py` | 纯标准库（os/termios/select）实现，**零第三方依赖**，pty 与真实 tty 同一代码路径 |

**设计原则**：虚拟与真实**零差别切换**——联调时 `--port /dev/pts/N`（pty），
实车时 `--port /dev/ttyUSB0`（CH340），代码一行不改。这是"先虚拟后实车"
策略的核心：把通信层的所有 bug 留在虚拟阶段。

---

## 2. 协议精确规格（从固件源码提取，含与手册的差异）

### 2.1 帧结构（X-Protocol 变帧长）

```
AA 55 | LEN | ID | DATA...(大端) | CK
  LEN = len(DATA) + 5     （总帧字节数，含帧头/长度/校验）
  CK  = sum(AA,55,LEN,ID,DATA...) & 0xFF
```

- 与固件 `AX_UART_PI_SendPacket` 逐行对照：`tx_buf[2] = len+5`，
  校验和对前 `4+len` 字节累加——与手册示例帧
  `AA 55 0B 01 03 E8 FC 18 00 0A 14`（数据 1000,−1000,10）逐字节一致，
  `protocol.py` 的 `__main__` 自检固化了这一断言。
- 接收状态机（固件 `USART2_IRQHandler`）：AA→55→LEN→(LEN−5 字节数据，
  ID 在偏移 3)→校验末字节；校验失败**静默丢帧**（无 NACK），所以链路
  质量只能靠上位机统计（桥里已做：好帧/坏帧/重同步计数）。

### 2.2 帧号总表

| 帧号 | 方向 | 含义 | 载荷 |
|---|---|---|---|
| **0x10** | STM32→ROS | 50Hz 综合数据 | 20 字节，见 §2.3 |
| **0x50** | ROS→STM32 | 目标速度 vx,vy,vw | 6 字节，3×int16 大端，m/s×1000 |
| **0x51** | ROS→STM32 | IMU 零偏校准指令 | 1 字节 flag —— **手册没写，源码发现** |
| **0x5f** | ROS→STM32 | 舵机零偏写值 | 2 字节 int16（0.1°）—— **手册没写，源码发现** |

桥里已实现全部四个（0x51/0x5f 作为标定通道，S8 标定时直接用）。

### 2.3 0x10 综合帧载荷（20 字节 = 10×int16 大端）

| 偏移 | 字段 | 换算 | 说明 |
|---|---|---|---|
| 0-1 | ACC_X | raw/32768×2 g ×9.81 → m/s² | 量程 2g |
| 2-3 | ACC_Y | 同上 | |
| 4-5 | ACC_Z | 同上 | 静止 ≈ 16384 |
| 6-7 | GYRO_X | raw/32768×500 °/s → rad/s | **已做上电零偏校准** |
| 8-9 | GYRO_Y | 同上 | |
| 10-11 | GYRO_Z | 同上 | |
| 12-13 | VEL_X | /1000 → m/s | **轮式里程计 FK**（固件只做差速 FK，不含转向角） |
| 14-15 | VEL_Y | /1000 | 阿克曼恒 0 |
| 16-17 | VEL_W | /1000 → rad/s | =(−vA+vB)/W，**W 用固件值 0.160** |
| 18-19 | BAT | /100 → V | 1120 = 11.2V |

固件已把 MPU6050 轴重映射到 ROS 右手系（`ROS.X = IMU.Y, ROS.Y = −IMU.X`），
桥收到的就是 ROS 系，无需二次旋转。

### 2.4 发送频率与串口参数（源码 main.c）

- `AX_UART4_Init(230400)`（EXP2 TTL，接树莓派）与 `AX_UART1_Init(230400)`
  （CH340 USB 调试口）**同时发 0x10**——实车调试可只用 USB 线，非常方便；
- 8N1，无流控；0x10 帧 50Hz（Robot_Task 20ms 周期）；
- 桥的 0x50 发送频率对齐 50Hz。

### 2.5 固件源码 vs 手册的差异（以源码为准）

| 项 | 手册/图纸 | **固件 ax_robot.h（代码在用）** | 影响 |
|---|---|---|---|
| 轴距 H | 112 mm | **146 mm** | 仿真 URDF 与实车不一致，S8 实测裁决 |
| 轮距 W | 164 mm | **160 mm** | 同上 |
| 最小转弯半径 | 0.55 m | **0.35 m**（L·cot30°−W/2 的另一取值） | Nav2 参数可能偏保守 |
| 轮径 | 65 mm | **65 mm** ✓ | 仿真已同步 |
| 编码器分辨率 | 390 | **1560**（13线×30×**四倍频**） | S8 标定项 |
| ROS 速度帧 | 只写了 0x50 | 还有 **0x51 / 0x5f** 校准帧 | 桥已实现 |

### 2.6 重大安全发现：固件没有速度看门狗 ⚠️

`ax_uart_pi.c` 的中断里收到 0x50 直接写 `RobotV_TG`，**没有任何超时
逻辑**；`Robot_Task` 只是持续执行最后的目标速度。也就是说：

> **ROS 断线/崩溃后，实车会保持最后的速度一直跑，直到撞墙或没电。**

对策（两层）：
1. **本桥已实现上位机看门狗**（默认 0.5s，参数可调）：cmd_vel 断流即发
   零速帧，断电也不受影响的是下位机——所以这只是"尽力兜底"；
2. **S6 固件改造必须加下位机看门狗**（Robot_Task 里检查"距最后 0x50 的
   时间 > 300ms → 目标速度清零"），这才是安全闭环。已列入 S6 清单。

---

## 3. 架构与数据流

```
ROS 2 侧                                虚拟/真实 STM32 侧
──────────────                          ──────────────────
/cmd_vel (Twist, Nav2/teleop)
   │ 50Hz 定时器 + 看门狗
   ▼
serial_bridge ── pack_velocity ──►  [0x50]  ──►  固件中断 → RobotV_TG
   ▲                                            → AX_AKM_WToAngle(转向角mrad)
   │                                            → 逆运动学 → 轮速目标
   │                                            → 电机 PID(PWM) → 编码器
[0x10] ◄── pack_frame ◄── Robot_Task(50Hz) ◄── IMU/轮速/电压
   │ FrameParser（坏帧统计）
   ▼
   ├─► /r5/wheel_vel (TwistStamped)   ← 轮式里程计速度
   ├─► /r5/imu_raw   (Imu)            ← 陀螺 rad/s / 加速度 m/s²
   └─► /r5/battery   (BatteryState)   ← 电压 V
```

**与 S3 融合节点的衔接**：`/r5/wheel_vel` + `/r5/imu_raw` 就是
`sensor_fusion.py` 需要的 two inputs（轮式速度 + 陀螺）。实车上
`/odom` 由融合节点产出，链路：0x10 → 桥 → 融合 → /odom → 控制器/Nav2。
**这就是上下位机闭环的全景**——S7 只是把 serial_bridge 后面的
`SerialPort` 换成 `hardware_interface` 插件，上层全复用。

**与仿真链路的对照**：Gazebo 里 gazebo_ros2_control 直接驱动关节；
实车上 ros2_control 的 ackermann 控制器还在，但 Hardware 换成 S7 的
串口插件——所以桥的话题命名故意不带 `/odom`（那是 S3 融合的产出）。

---

## 4. 代码逐文件解析（为什么这样写）

### 4.1 `protocol.py` —— 协议即数据，不是代码

- **常量即文档**：帧号/限幅/换算系数全部按固件源码命名并注释出处
  （`R_VX_LIMIT=1500` 来自 ax_robot.h:186），固件升级时 diff 这几行
  就知道协议有没有变；
- `pack_velocity` 内置固件同款限幅（±1.5 m/s / ±6.28 rad/s）——
  **限幅要在发送端做**，因为固件把超限值当 int16 直接截断，发了也白发；
- `FrameParser` 与固件中断状态机**对称实现**（同样的 AA/55/LEN/CK 状态
  流转），坏帧静默丢弃但**计数暴露**——固件静默，上位机不能也静默；
- `decode_utx_data` 单位换算集中在这一个函数，其余代码只见物理单位
  （rad/s、m/s、V），杜绝"raw 和物理量混用"这类量纲 bug（Round 2 的教训）。

### 4.2 `serial_port.py` —— 30 行看懂串口

逐项解释 termios 配置为什么这么写：
- `O_NOCTTY`：串口不是控制终端，防 Ctrl-C 串扰；
- raw 模式四连关（ECHO/ICANON/ISIG/IEXTEN）：**协议是二进制流**，
  0xAA/0x55 里可能出现任何控制字符，行缓冲/转义必须全关；
- `IXON/ICRNL` 关闭：0x13(XON)/0x0D(CR) 会触发流控/换行替换，毁帧；
- `CS8 + PARENB/CSTOPB 清零`：8N1；
- 波特率查表 `B230400`（termios 常量），pty 下无意义但不报错——
  **这就是虚拟/真实同一代码路径的代价为零的原因**。

### 4.3 `virtual_stm32.py` —— 扮演固件，而不是扮演一辆车

- **只模拟协议与一阶惯性**：速度一阶逼近目标（τ=0.15s，模拟电机 PID），
  不做轮胎/转向几何——联调目标是通信层，不是动力学（那是 Gazebo 的活）；
- **陀螺零偏默认 55 raw ≈ 0.84°/s**：故意比真实 MPU6050 更差，
  让"上位机是否正确处理零偏"在数据上可见；
- `--corrupt N`（千分率坏帧注入）：**解析器的鲁棒性必须主动攻击测试**，
  不能等实车电磁环境来教；
- 看门狗证据采集：虚拟端打印"收到零速指令（距上次非零 x.xx s）"，
  把上位机看门狗的行为变成可检查的日志。

### 4.4 `serial_bridge.py` —— 桥的四个纪律

1. **看门狗在发送定时器里**（50Hz tick 判断断流），不在订阅回调里——
   回调只在有消息时触发，断流时永远等不到它；
2. **线程模型**：读线程（阻塞串口）+ ROS 定时器（发送），帧解析在读线程，
   发布也在读线程（rclpy 发布器线程安全）；50Hz 发送不做任何阻塞调用；
3. **打开端口先 `drain()`**：CH340 打开瞬间会残留上电乱码/旧帧，
   不清空则解析器从半帧开始（同步耗时）；
4. **退出时发零速**：Ctrl-C 桥之后车必须停——最后的安全垫。

---

## 5. 测试与结果（虚拟串口全链路）

### 5.1 测试方法

```bash
# 终端 1：起虚拟 STM32（打印 pty 路径，如 /dev/pts/3）
ros2 run r5_serial virtual_stm32

# 终端 2：起串口桥（接 pty）
ros2 run r5_serial serial_bridge --port /dev/pts/3

# 终端 3：发指令
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.4}}"
```

### 5.2 测试项与判据（全部通过）

| # | 测试 | 判据 | 结果 |
|---|---|---|---|
| 1 | 协议自检 | `pack_frame` 输出 ≡ 固件手册示例帧 | ✅ 逐字节一致 |
| 2 | 速度闭环 | /r5/wheel_vel 在 ~0.5s 内收敛到指令 (0.2, 0.4) | ✅ 一阶惯性收敛 |
| 3 | IMU 直通 | /r5/imu_raw gyro.z ≈ 0.4 rad/s + 零偏 + 噪声 | ✅ |
| 4 | 看门狗 | 停发 /cmd_vel 后 0.5s，虚拟端打印看门狗触发、速度归零 | ✅ |
| 5 | 坏帧鲁棒性 | `--corrupt 30` 时桥的 bad_checksum 计数增长、好帧持续、无崩溃 | ✅ |
| 6 | 0x51/0x5f | 虚拟端正确打印校准指令与零偏值 | ✅ |
| 7 | 退出安全 | Ctrl-C 桥 → 虚拟端收到零速帧 | ✅ |

### 5.3 复现命令（测试完成后可随时重跑）

```bash
colcon build --symlink-install --packages-select r5_serial
source install/setup.bash
ros2 run r5_serial virtual_stm32                     # 记下 pty 路径
ros2 run r5_serial serial_bridge --port /dev/pts/X   # 换成实际路径
ros2 topic echo /r5/wheel_vel
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.4}}"
# 看门狗测试：Ctrl-C 停掉 topic pub，观察 serial_bridge 的 warning 和虚拟端日志
# 坏帧测试：virtual_stm32 --corrupt 30 重启，看 serial_bridge 统计里的坏帧计数
```

---

## 6. 对接真硬件的 Checklist（S6 到手后照此执行）

1. **接线**（精简方式，一根线解决供电+通信）：
   EXP2(J5)：GND→Pi GND、5V→Pi 5V、PC11(UART4_RX)→Pi TXD、PC10(UART4_TX)→Pi RXD；
   **树莓派 5 的 5V 引脚直供有风险**（峰值电流），建议 B20 的 5V@5A 走
   GPIO 5V 且实测电压跌落；USB 调试口（CH340）留作备用通道；
2. **树莓派串口使能**：`/boot/firmware/config.txt` 加
   `enable_uart=1`，**关闭蓝牙**（释放 PL011：`dtoverlay=disable-bt`），
   使用 `/dev/ttyAMA0`（板载 UART 的蓝牙 UART0 性能差）；
3. **权限**：`sudo usermod -aG dialout $USER`；
4. **首连测试**：`ros2 run r5_serial serial_bridge --port /dev/ttyAMA0`
   → 观察 `/r5/wheel_vel` 有 50Hz 数据即链路通；
5. **验证转向**：发 `vw=600`（0.6 rad/s），**观察舵机是否明显偏转**
   ——这是裁决固件舵机映射疑点（§7）的关键实验；
6. **静态漂移**：车架空上电静置 5 分钟，记录 gyro.z 零偏 → 决定
   sensor_fusion 的 alpha 与零偏补偿策略；
7. **安全**：车架空！固件无看门狗，桥的看门狗是唯一防线（S6 改造前）。

## 7. 从源码挖出的疑点/风险清单（诚实记录）

| # | 发现 | 状态 | 裁决方式 |
|---|---|---|---|
| 1 | **固件无速度看门狗**（断流保持最后速度） | 确认（源码无超时逻辑） | 桥已兜底；S6 固件加下位机看门狗 |
| 2 | 舵机映射链疑似单位 bug：`ax_kinematics.c:107` 的 `tan(akm_angle*0.01745)`（弧度又乘 π/180）会让右前轮角恒 ≈0.2°，舵机角近乎回中——**与"商品固件能转弯"矛盾**，疑似还有未读到的补偿，或 WToAngle 单位约定与我理解不同 | **待实测裁决** | S6 上真机后：串口发 0x50(vw=600)，看舵机偏角。若不转，按手册公式（θ=atan(H/(H/tanβ+W/2))，β 为弧度）重写该函数 |
| 3 | 固件里程计 FK 只用轮速差分（不含转向角） | 确认（ax_kinematics.c:139） | 优于 ros2_control 的 steer-FK？不——转弯刮地时差速更可信，直行两者一致。S3 的融合实验（刮地 6% 率误差）说明**实车 FK 也会有系统误差**，靠融合兜底 |
| 4 | 固件参数（轴距 0.146/轮距 0.160/R_min 0.35）与手册（0.112/0.164/0.55）冲突 | 确认 | S8 卷尺实测 + 满舵画圆量半径，**以实测为准**，同步改 URDF/Nav2 |
| 5 | 加速度计 z 静止值会因安装角 ≠1g | 预期 | S8 静置六面法标定 |
| 6 | 厂商有自己的 ROS 驱动（xtark_robot，ROS1 风格） | 未获取 | 不阻塞：我们的桥+ros2_control 方案自主可控且已验证；若拿到《智能小车 ROS 扩展资料》可对照参考 |

## 8. 需要用户提供的清单（"需要什么再跟我说"）

**现在就可以（不影响 S5/S7）**：
- [ ] 《智能小车 ROS 扩展资料》《智能小车基础控制资料》两个文件夹
      （对照厂商官方 ROS 驱动与 STM32 注释版源码，查漏补缺）；
- [ ] 确认手上的 OpenCTR B20 板版本（V1.1？）与是否已焊 EXP2 连接线；

**S6 前必须**：
- [ ] 实车（R5 整车 + OpenCTR B20 + 电池）；DAPLINK 或 ST-LINK（烧固件）；
- [ ] 树莓派 5 + 已装 ROS 2 Humble 的 Ubuntu（22.04/24.04）；
- [ ] 卷尺/直尺（S8 标定：轴距、轮距、满舵画圆半径）；
- [ ] optional：逻辑分析仪或 USB 转串口小板（抓 0x10 帧波形，排错利器）。

**S6 时需要明确的一个决策**：固件是否由我们改（加看门狗 + 修舵机映射
疑点）？改则需 DAPLINK + MDK5 环境（手册 5.6 节），或转投
PlatformIO/STM32CubeIDE（MDK 工程可迁移）。

---

## 9. 下一步衔接

- **S6（STM32 固件）**：按 §6 checklist 接线 + 本文档 §2 协议表实现
  收发；重点：下位机看门狗（300ms）、舵机映射疑点实测、零偏校准流程；
- **S7（HW interface）**：把 serial_bridge 的 SerialPort 换成
  `hardware_interface::SystemInterface`，导出 command（轮速×2 + 舵角）
  与 state（轮速×2 + gyro）接口，R5 控制器零改动；
- **S4（Nav2）**：与本步并行，仿真内做倒车分流与 BT 定制。

*文档版本：2026-10-26，随 `r5_serial 0.1.0` 发布。协议如有固件更新，
以 diff ax_robot.h/ax_uart_pi.c 为准。*
