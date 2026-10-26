# OpenCTR B20 固件（R5 阿克曼）研读标注版

这份目录是**厂商固件源码 + 研读注释**，服务于两件事：读懂控制链（学习）、S6 改造固件（加看门狗等）。所有注释只是新增，**代码一行未改**，已用"剥掉注释后逐字节比对"验证。

## 目录结构

```
~/car_ws/vendor/
├── xtark_fw/
│   ├── OpenCTR_B20V11_AKM_V1.00.241015/   # ★ 标注版（UTF-8，VS Code 直接可读）
│   ├── README_ANNOTATED.md                # 本文件
│   └── tools/annotate.py                  # 标注脚本（幂等，可重跑）
├── xtark_fw_original_gbk/                 # 厂商原始树（GBK，未改动，Keil 编译烧录用它）
└── xtark_docs/                            # 6 份 PDF 手册
```

- **读代码 → 用 `xtark_fw/`**（UTF-8，中文注释正常显示）
- **编译/烧录 → 用 `xtark_fw_original_gbk/`**（保持厂商原始编码，避免 Keil 对 UTF-8 中文注释的兼容问题）
- 两份代码内容完全一致，只有注释和编码不同。

> 如果 VS Code 里仍显示乱码：关闭文件重开（旧缓冲区有缓存）。若打开的是 `xtark_fw_original_gbk/`，点右下角编码 → Reopen with Encoding → GB18030。

## 已标注的 7 个文件（共 51 处）

| 文件 | 标注处数 | 内容 |
|---|---|---|
| `Robot/ax_kinematics.c` | 12 | 阿克曼 IK、舵机映射、轮速 PID、FK 回算；P1/P2/P3 缺陷现场 |
| `Driver/ax_uart4.c` | 12 | **真正收发上位机数据的地方**；状态机、校验和、安全缺陷 |
| `Robot/ax_robot.c` | 10 | 50Hz 主循环、IMU 轴变换、0x10 帧组装、电池保护 |
| `Robot/ax_akm.c` | 7 | `AX_AKM_WToAngle`：vx/vw → 前轮角（单位 0.001 rad） |
| `Robot/ax_robot.h` | 4 | 几何参数三方对照、速度限幅、帧号表 |
| `Driver/ax_uart_pi.c` | 3 | 旧版镜像实现（B20 不调用），用于对照读状态机 |
| `Robot/ax_speed.c` | 3 | 增量式"PID"的真实数学结构（Kp 实为积分） |

其余文件（OLED/RGB/按键/蜂鸣器/PS2/SBUS/MPU6050 驱动等）未标注，按 PROJECT_MAP 的"跳过清单"处理。

新增注释都以 `/* @...` 开头。用 `grep -rn "研读标注\|@L[0-9]"` 或 VS Code 搜索 `@L` 可定位。

## 标注里的行号规则

所有 `@Lxx` 与文中提到的行号，**均指厂商原始文件**（`xtark_fw_original_gbk/`）。标注版因为插入了注释，行号已经下移。需要引用行号给他人时，说"原始文件第 xx 行"。

## 一条完整控制链（读代码时对照）

```
上位机 /cmd_vel (m/s, rad/s)
  │  serial_bridge.py ×1000，50Hz
  ▼
0x50 帧  AA 55 | LEN | 50 | vx(2B) vy(2B) vw(2B) | CK      （大端）
  │  ax_uart4.c:168~176  中断里解析
  ▼
AX_AKM_WToAngle(vx, vw)  →  ax_akm_angle（单位 0.001 rad）
  │  50Hz Robot_Task
  ▼
ROBOT_AKM_Kinematics():
  编码器读后清零 → 轮速 m/s → R = L/tan(δ) → 左右轮差速
  → 舵机角（梯形连杆二次拟合）→ 舵机 PWM
  → 每轮增量式 PID → 电机 PWM
  → FK 回算 vx/wz
  ▼
0x10 帧（20 字节）：acc×3, gyro×3, vx, vy, wz, 电池电压
  │  ax_uart4.c:229 每 50Hz 发送
  ▼
上位机 /r5/imu_raw, /r5/wheel_vel, /r5/battery
```

## 单位总表（全固件唯一的"陷阱集中区"）

| 量 | 单位 | 转换点 |
|---|---|---|
| `ax_akm_angle` | 0.001 rad | `ax_akm.c:66` ×1000；`ax_kinematics.c:93` ÷1000 |
| `R_Vel.TG_IX/IY` | 0.001 m/s | `ax_uart4.c:170` 解包；`ax_kinematics.c:92` ÷1000 |
| `R_Vel.TG_IW` | 0.001 rad/s | `ax_uart4.c:172` 解包 |
| `R_Wheel_*.RT` | m/s | 编码器 20ms 增量 × `AKM_WHEEL_SCALE`(0.006545) |
| `servo_angle` | 0.1 度 | `ax_kinematics.c:123` ×10 |
| IMU acc | 原始 LSB | 上位机 ×2g/32768 |
| IMU gyro | 原始 LSB | 上位机 ×500dps/32768 |
| 电池电压 | 0.1 V | 上位机 ÷10 |

## 缺陷清单（读代码时要能自己挑出来）

### 代码缺陷（影响控制质量）

| 编号 | 位置 | 问题 | 影响 | 建议 |
|---|---|---|---|---|
| **P1** | `ax_kinematics.c:107` + `:120` | 弧度当度数（`*0.01745`），行尾又 `×180/π`，120 行再 `×180/π` | 两次换算近似抵消，净效果 **约 +12% 舵机增益** + 梯形二次补偿项失效；**不是 57 倍错误**，车能正常开 | 改为 `atan(L/(L/tan(δ)+w/2))`（弧度），120 行保持不动。旁证：老版 `robot.c:214` 同款写法，属遗传缺陷 |
| **P2** | `ax_kinematics.c:88~89` | 转向角限幅整段被注释掉 | 0x50 路径有 `ax_akm.c:48` 的最小半径兜底；其它写 `ax_akm_angle` 的路径无保护 | 恢复为 ±0.4 rad 左右的限幅 |
| **P3** | `ax_kinematics.c:137~139` | wz 用左右轮速差估算 | 轮胎侧滑导致**系统性高估约 6%**（你们的圆周实验数据） | 不是代码 bug，是物理误差 → 必须 IMU 融合（S3 已做） |

### 安全隐患（S6 要修的）

| 编号 | 位置 | 问题 | 后果 |
|---|---|---|---|
| **S1** | `ax_uart4.c:149` | LEN 无上限，`uart4_rx_buf[40]` 可被写到 index 242 | 线路噪声可越界改写相邻全局变量；LEN=0 时状态机死等（uint8 回绕） |
| **S2** | `ax_uart4.c:165` | 无通信看门狗，最后一帧目标速度永久生效 | 树莓派死机/USB 掉线 → **车保持原速直行**。上位机 0.5s 看门狗只是软件兜底。S6 加 300ms 下位机看门狗 |
| **S3** | `ax_uart4.c:165` | 每个合法帧强制 `CTL_ROS` | 上位机一发包即夺走 PS2/SBUS 控制权；实车调试时"手柄突然失灵" |
| **S4** | `ax_uart4.c:229` | `len<=50` 但发送缓冲只有 40 字节 | 当前唯一调用者传 20，安全；改协议先修这里 |

### 与手册/仿真的不一致（S8 实测裁决）

| 量 | 固件 | 手册 | 仿真 URDF | 结论 |
|---|---|---|---|---|
| 轴距 | 0.146 m | 0.112 m | 0.112 m | 卷尺实测 |
| 轮距 | 0.160 m | 0.164 m | 0.164 m | 卷尺实测 |
| 最小转弯半径 | 0.35 m（钳位） | 0.55 m | — | 满舵画圆实测 |
| 固件注释自称 R_min | 0.173 m | — | — | 连厂商注释和宏值都对不上 |

## 行为特征（不是 bug，但决定实车表现）

1. **`vx == 0` 时前轮角强制为 0**（`ax_akm.c:42`）：静止打方向无效 → Nav2 的 Spin / RotateToGoal 在这台车上必定失败，必须靠"前进/倒车 + 转向"三点掉头（S4）。
2. **转角只在收到 0x50 那一刻更新**，不在 50Hz 循环里重算：上位机 vx 变了而 vw 没变时，曲率会跟着变（减速时车头轻微偏一下的机制）。
3. **IMU 轴变换与老版固件差 180°**：`ax_robot.c:161~187` 是 `ACC_X=+imu[1], ACC_Y=-imu[0]`，老 `robot.c:116~131` 是反的。实车方向不对先查这里。
4. **0x51/0x5f 在 B20 上无效**：只有老版 `ax_uart_pi.c` 处理陀螺校准和舵机零偏，`ax_uart4.c` 只认 0x50/0x60。别照抄老上位机代码。
5. **IMU 零偏只能靠上位机自校准**：下位机的 `Imu_Task` 由 0x51 触发，但 B20 收不到 0x51 → `sensor_fusion.py` 的上电静止校准是必需的。

## 重新生成 / 恢复

```bash
# 标注（幂等：已含"[研读标注]"的文件会自动跳过）
python3 ~/car_ws/vendor/xtark_fw/tools/annotate.py

# 恢复某个文件为厂商原样（GBK）
cp ~/car_ws/vendor/xtark_fw_original_gbk/Robot/ax_kinematics.c \
   ~/car_ws/vendor/xtark_fw/OpenCTR_B20V11_AKM_V1.00.241015/Robot/

# 整棵树恢复
rsync -a --delete ~/car_ws/vendor/xtark_fw_original_gbk/ \
   ~/car_ws/vendor/xtark_fw/OpenCTR_B20V11_AKM_V1.00.241015/
```

## 建议阅读顺序（约 2 天）

1. `ax_robot.c` 的 `Robot_Task`（92 行起）→ 建立 50Hz 节拍感
2. `ax_uart4.c` 的 ISR（106 行起）→ 数据怎么进来
3. `ax_akm.c` 全文（69 行）→ vx/vw 怎么变成转角
4. `ax_kinematics.c` 全文（143 行）→ 转角怎么变成舵机/轮速，P1/P2/P3
5. `ax_speed.c`（92 行）→ 轮速怎么变成 PWM
6. `ax_robot.h` 参数区 → 所有常量溯源
7. `ax_uart_pi.c` → 只做镜像对照，确认两者差异
