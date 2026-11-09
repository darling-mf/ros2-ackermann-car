# OpenCTR B20 固件模块

R5 阿克曼平台使用的 STM32F103RC OpenCTR B20 下位机可移植 C 模块。模块按
厂家 StdPeriph/FreeRTOS 工程编写，但不依赖硬件，可以直接在 PC 上编译和测试。

厂家固件和手册不在本仓库分发。

## 模块

| 文件 | 用途 |
|---|---|
| `src/xprotocol.c/.h` | X-Protocol 帧打包/解析，带重同步和统计 |
| `src/safety.c/.h` | 指令看门狗：300 ms 没有合法速度帧就禁止运动 |
| `src/ackermann.c/.h` | 虚拟转角、内轮角、舵机映射、最小半径钳位 |
| `test/test_host.c` | 主机测试：示例帧、回环、坏帧重同步、看门狗、阿克曼几何 |

## 主机测试

```bash
cd firmware/openctr_b20/test
gcc -std=c99 -Wall -Wextra -Werror -I../src \
  test_host.c ../src/xprotocol.c ../src/safety.c ../src/ackermann.c -lm \
  -o test_host
./test_host
```

预期输出五行 `PASS`。

## 集成点

厂家工程是 Keil 工程。把 `src/*.c/.h` 复制进去，并加入 Keil target。集成点：

1. **初始化**：启动控制任务前，初始化协议解析器和安全看门狗。
2. **UART 接收**：把收到的字节喂给 `xproto_feed()`；处理 `0x50` 速度帧，
   拒绝长度/校验和错误的帧。
3. **安全超时**：每收到一个合法 `0x50` 帧调用 `safety_feed()`；控制循环里
   调用 `safety_allow()`。300 ms 内没有合法指令就强制零轮速。
4. **阿克曼换算**：用 `akm_w_to_beta_mrad()` 和
   `akm_inner_wheel_deg()` 把 `(vx, wz)` 转成内轮角；最小半径在配置里钳位。
5. **舵机输出**：用实测的 `servo_gain`/`servo_offset` 调用
   `akm_servo_cmd_d10()`；机械限位仍要在软件里守住。

## 几何参数

C 模块用配置结构体，而不是把几何写死：

```c
typedef struct {
    float axle_base;      /* 轴距 [m] */
    float wheel_base;     /* 轮距 [m] */
    float min_radius;     /* 最小转弯半径 [m] */
    float servo_gain;     /* 舵机角 = gain * 内轮角 + offset */
    float servo_offset;   /* [deg]，含机械零位 */
} akm_cfg_t;
```

仿真和主机测试使用手册/实车实测几何：轴距 0.112 m、轮距 0.164 m、最小
转弯半径 0.55 m。厂家固件里的常量不同，不要直接照抄。

## 协议摘要

| 项目 | 值 |
|---|---|
| 帧格式 | `AA 55 LEN ID DATA CHECK` |
| 波特率 | 230400，8N1 |
| STM32 -> 主机 | `0x10` 综合数据帧 |
| 主机 -> STM32 | `0x50` 速度帧（`vx`、`vy`、`wz`，m/s × 1000，大端） |
| 转向 | `0x60` |

主机侧对应实现见 `src/r5_serial/`。
