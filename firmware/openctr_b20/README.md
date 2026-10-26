# S6：OpenCTR B20（STM32F103RC）固件改造 + PC 端烧录

> 目标：给厂家固件补上**下位机速度看门狗**、修正**转向几何单位问题**、
> 把手感未知的**舵机映射参数化**，并建立 PC 端编译/烧录流程。
> 本目录只包含**我们自己的、可在 PC 上单元测试的纯 C 模块**；厂家源码在
> 仓库 `vendor/xtark_fw/OpenCTR_B20V11_AKM_V1.00.241015/`（UTF-8，见 §1）。
> 主机测试：`test/test_host.c`，`gcc` 即可跑，无需硬件。

---

## 1. 厂家源码与硬件事实（来源：OpenCTR_B20V11_AKM V1.00.241015）

厂家固件在仓库 `vendor/xtark_fw/OpenCTR_B20V11_AKM_V1.00.241015/`
（`vendor/xtark_fw_original_gbk/` 为原始 GBK 副本）。关键事实：

| 项目 | 值 | 来源 |
|---|---|---|
| MCU | **STM32F103RC**（256KB Flash / 48KB RAM，LQFP64） | `Project/xproject.uvprojx` |
| 工程/编译 | Keil MDK5（RVDS 启动文件 + StdPeriph + FreeRTOS） | 同上 |
| 电机 PWM | TIM4 CH1-4 → **PB6/PB7/PB8/PB9**（20 kHz） | `Driver/ax_motor.c` |
| 编码器 A | TIM2 全重映射 → **PA15 / PB3** | `Driver/ax_encoder.c` |
| 编码器 B | TIM3 部分重映射 → **PB4 / PB5** | 同上 |
| 转向舵机 S3 | TIM8 CH2 → **PC7**（脉宽 1500±1.111×angle，angle 单位 0.1°） | `Driver/ax_servo.c` |
| 云台舵机 S1/S2/S4 | TIM8 CH4/CH3/CH1 → PC9/PC8/PC6 | 同上 |
| MPU6050 | 软件 I2C：**PA11=SDA / PA12=SCL** | `Driver/ax_mpu6050.c` |
| 树莓派串口 | UART4（USART2）**PC11=RX / PC10=TX**，**230400 8N1** | `Driver/ax_uart_pi.c`, `main.c` |
| USB 调试口 | UART1（USART1）PA10/PA9 → CH340，**230400**，与 UART4 **同时发 0x10** | `main.c`, `ax_robot.c` |
| 蓝牙 | UART2 PA3/PA2，115200 | `main.c` |
| 编码器分辨率 | 1560 counts/rev（13 线 × 30 减速比 × 4 倍频） | `Robot/ax_robot.h` |
| 固件几何（**与手册冲突**） | 轴距 0.146 / 轮距 0.160 / R_min 0.35 / 轮径 65mm | `Robot/ax_robot.h:150-157` |

**待实测裁决**：轴距 0.146（固件）vs 0.112（手册图纸）、轮距 0.160 vs 0.164、
R_min 0.35 vs 0.55。卷尺 + 满舵画圆一次定案，然后同步改本目录
`akm_cfg_t`、URDF、`controllers.yaml`、Nav2 参数。

---

## 2. 本目录交付的模块（纯 C，已通过主机单测）

| 文件 | 作用 |
|---|---|
| `src/xprotocol.c/.h` | X-Protocol 打包/解析状态机（与固件一致，含统计） |
| `src/safety.c/.h` | 速度看门狗（300ms 无 0x50 → 禁止运动） |
| `src/ackermann.c/.h` | 虚拟转角 β、内轮角、舵机指令；含最小半径钳位与可标定舵机参数 |
| `test/test_host.c` | 5 组主机测试：官方示例帧/回环/坏帧重同步/看门狗/阿克曼几何 |

主机测试（PC，无需硬件）：

```bash
cd firmware/openctr_b20/test
gcc -std=c99 -Wall -Wextra -Werror -I../src test_host.c \
    ../src/xprotocol.c ../src/safety.c ../src/ackermann.c -lm -o test_host
./test_host
```

预期输出五行 PASS + “全部 S6 主机测试通过”。

---

## 3. 集成到厂家固件的 5 个改动点（精确到函数）

> 厂家工程是 Keil；把 `src/*.c/.h` 复制进 `Robot/` 并加入 Keil 工程即可。
> 下面按“改哪个文件、改哪个函数、加哪几行”给出。

### 3.1 `main.c`：初始化
```c
#include "xprotocol.h"
#include "safety.h"
static xproto_t g_xp;
static safety_t g_safety;

/* main() 外设初始化之后、启动任务之前（厂家工程为 FreeRTOS + StdPeriph，不是 HAL） */
xproto_init(&g_xp);
safety_init(&g_safety, 300);              /* 300 ms 看门狗 */
```

### 3.2 `Driver/ax_uart4.c` 的 `USART4_IRQHandler`：解析 0x50/0x51/0x5f
在原来的 `Res = USART_ReceiveData(UART4);` 之后，**用状态机替换**原
`uartpi_rx_*` 逻辑（或保留原逻辑只加喂狗），核心：

```c
uint8_t id, data[50], len;
if (xproto_feed(&g_xp, Res, &id, data, &len))
{
  if (id == 0x50 && len >= 6)
  {
    int16_t vx = (int16_t)((data[0] << 8) | data[1]);   /* mm/s */
    int16_t vy = (int16_t)((data[2] << 8) | data[3]);   /* 忽略 */
    int16_t wz = (int16_t)((data[4] << 8) | data[5]);   /* mrad/s */
    RobotV_TG.I_X = vx; RobotV_TG.I_Y = vy; RobotV_TG.I_W = wz;
    safety_feed(&g_safety, (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS));              /* ← 关键：喂狗 */
  }
  else if (id == 0x51) { ax_imu_calibrate_flag = data[0]; }
  else if (id == 0x5f && len >= 2)
  {
    ax_servo_offset = (int16_t)((data[0] << 8) | data[1]);
  }
}
```

> 原固件在 `ax_uart_pi.c` 里自己维护 `uartpi_rx_*` 状态机；建议整段替换
> 为 `xproto_feed`，避免维护两份协议代码（S5 的 Python 桥已与它对齐）。

### 3.3 `Robot/ax_robot.c` 的 `Robot_Task`（20ms）：看门狗 + 新运动学
```c
#include "safety.h"
#include "ackermann.h"

static const akm_cfg_t cfg = {
  .axle_base = 0.146f,    /* ← S8 实测后修改 */
  .wheel_base = 0.160f,   /* ← S8 实测后修改 */
  .min_radius = 0.35f,    /* ← S8 实测后修改 */
  .servo_gain = 1.0f,     /* ← S8 标定：舵机角/内轮角 */
  .servo_offset = 0.0f,   /* ← S8 标定：机械零位 */
};

/* Robot_Task 循环内：先安全判定，再执行运动学 */
if (!safety_allow(&g_safety, (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS)))
{
  R_Vel.TG_IX = 0; R_Vel.TG_IW = 0;
  AX_SERVO_S3_SetAngle((int16_t)(cfg.servo_offset * 10));  /* 回中 */
  AX_MOTOR_A_SetSpeed(0); AX_MOTOR_B_SetSpeed(0);
}
else
{
  int16_t beta_mrad = akm_w_to_beta_mrad(R_Vel.TG_IX, R_Vel.TG_IW, &cfg);
  float beta = beta_mrad / 1000.0f;
  float inner_deg = akm_inner_wheel_deg(beta, &cfg);
  AX_SERVO_S3_SetAngle(akm_servo_cmd_d10(inner_deg, &cfg));
  /* 轮速仍可用厂家 ROBOT_AKM_Kinematics 的差速部分 */
}
```

**为什么把看门狗放在 Robot_Task 而不是中断里**：中断只负责“收到就喂”，
判定必须在固定周期任务里做——这样即使 UART 中断被异常刷屏，也不会
绕过超时语义。

### 3.4 舵机映射：修掉单位 bug，改为可标定
厂家 `ax_kinematics.c:107`：
```c
atan(AKM_ACLE_BASE / ((AKM_ACLE_BASE / tan(akm_angle*0.01745)) + 0.5*AKM_WHEEL_BASE))
```
`akm_angle` 本身是弧度，再乘 `0.01745`（π/180）等于把弧度当角度又转一次，
导致右前轮角恒 ≈0.2°（与“商品固件能转弯”矛盾，疑有未读到的补偿）。
**我们的做法**：不再使用这段多项式，改为
`inner_deg = akm_inner_wheel_deg(β)` + `servo_deg = gain·inner_deg + offset`，
`gain/offset` 由 S8 实测拟合。这比盲目沿用厂家系数可控，也便于换舵机。

### 3.5 上电流程
1. 保持车轮架空；
2. 上电后 0.5s 内不发指令 → 看门狗应保持电机零输出（验证安全默认）；
3. 用 S5 的 `serial_bridge` 发一条 0x50，再断流 1s → 车必须停；
4. 只有以上通过，才允许落地测试。

---

## 4. PC 端编译与烧录（三种方式，按推荐排序）

### 方式 A：ST-Link V2 + STM32CubeProgrammer（推荐，最稳）
- 接线：SWDIO→PA13、SWCLK→PA14、GND、3.3V；BOOT0=0；
- PC 安装 STM32CubeProgrammer，选 SWD，Connect → 打开厂家 `.hex`
  （或 Keil 编译产物 `Project/Objects/*.hex`）→ Download；
- Keil 工程：`Project/xproject.uvprojx`，Options→Debug 选 ST-Link，
  即可一键编译+烧录。

### 方式 B：USB 串口 ISP（板载 CH340，无需调试器）
STM32F103 的 ROM bootloader 支持 USART1；B20 的 CH340 正好接 USART1
（PA9/PA10）。步骤：
1. 找到板上 **BOOT0** 跳线/焊盘，置 1（若未引出，只能用方式 A）；
2. 复位，PC 上确认串口设备（`ls /dev/ttyUSB*`）；
3. `stm32flash -w firmware.hex -v -b 115200 /dev/ttyUSB0`；
4. BOOT0 置 0，复位运行。
> 注意：ISP 只用于烧录；运行期 USART1 被固件用于 230400 调试输出，
> 与 ISP 的 115200 不冲突（不同阶段）。

### 方式 C：PlatformIO（跨平台、可 CI）
厂家工程是 StdPeriph + FreeRTOS + Keil 启动文件，**不建议整体迁移**；
推荐只把我们这三个模块用 `platform = native` 做主机测试（已可跑），
固件仍用 Keil 编译烧录。若坚持 PlatformIO，参考
`platformio.ini.example`（仅示例，需按实际 SPL 源/链接脚本调整）。

---

## 5. S8 标定清单（上实车第一件事）

| 参数 | 方法 | 写入位置 |
|---|---|---|
| 轮径 | 卷尺量外径（65mm 待确认） | URDF `common.xacro`、`controllers.yaml` |
| 轴距 H | 前后轮轴中心距（0.146 vs 0.112） | `akm_cfg_t.axle_base`、URDF、Nav2 |
| 轮距 W | 左右轮中心距（0.160 vs 0.164） | 同上 |
| 最小转弯半径 | 满舵低速画圆，量后轴中心轨迹半径 | `akm_cfg_t.min_radius`、Nav2 `min_turning_r` |
| 舵机 gain/offset | 给若干舵角，量前轮实际角，线性拟合 | `akm_cfg_t.servo_gain/offset` |
| IMU 零偏 | 静置 5min 取 gyro.z 均值 | 0x51 校准或固件上电采样 |
| 编码器 1560 | 推车 1m，读 counts/轮转 | 固件 `AKM_WHEEL_RESOLUTION` |

---

## 6. 安全红线

- **不要信任厂家固件的默认行为**：它没有看门狗，断流保持最后速度；
- 首次上电必须架空；先验证看门狗，再验证方向，最后落地；
- 任何固件改动后，先跑主机单测（`test_host`），再烧录；
- 烧录前备份原厂 `.hex`（出问题可回滚）。
