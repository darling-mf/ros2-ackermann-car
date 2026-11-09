# r5_serial

通过 UART 上的 X-Protocol，把 `/cmd_vel` 转到 OpenCTR B20 下位机的 ROS 2
串口桥，并提供虚拟 STM32 用于无硬件联调。

## 协议

| 项目 | 值 |
|---|---|
| 帧格式 | `AA 55 LEN ID DATA CHECK` |
| 波特率 | 230400，8N1 |
| STM32 -> ROS | `0x10` 综合帧（IMU、轮速、电压），50 Hz |
| ROS -> STM32 | `0x50` 速度帧（`vx`、`vy`、`wz`，m/s × 1000，大端） |
| 转角 | `0x60`；`0x51` / `0x5F` 属于旧固件，B20 未实现 |

协议常量在 `r5_serial/protocol.py`；帧解析器是带重同步的状态机，会拒绝
长度或校验和错误的帧。

## 组成

| 文件 | 用途 |
|---|---|
| `r5_serial/protocol.py` | 帧打包/解析、`0x10` 解码、`0x50` 编码 |
| `r5_serial/serial_port.py` | 串口抽象 |
| `r5_serial/serial_bridge.py` | ROS 节点：`/cmd_vel` -> UART，UART -> `/r5/imu_raw`、`/r5/wheel_vel`、`/r5/battery` |
| `r5_serial/virtual_stm32.py` | 基于 pty 的虚拟 STM32，无硬件时使用 |

串口桥有上位机看门狗：`/cmd_vel` 断流超过 `watchdog` 秒（默认 0.5 s）时，
会发送零速帧。这只是上位机兜底；下位机也应该实现自己的指令超时。

## 使用

```bash
# 终端 1：创建虚拟串口
ros2 run r5_serial virtual_stm32
# 记下打印的 /dev/pts/N

# 终端 2：启动串口桥
ros2 run r5_serial serial_bridge --port /dev/pts/N

# 终端 3：遥控
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

相关参数：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `port` | 空（必填） | 串口设备，通过 `--port` 传入 |
| `baud` | `230400` | 波特率 |
| `watchdog` | `0.5` | 零速超时 [s] |
| `send_imu` | `true` | 是否发布解码后的 IMU 数据 |

`0x50` 固定 50 Hz 发送（`serial_bridge.py` 中的 `CMD_RATE`），与固件控制
任务对齐。

## 测试

协议模块自带示例帧、`0x10` 打包/解析回环和坏校验重同步测试：

```bash
cd src/r5_serial
python3 -m r5_serial.protocol
```

串口桥联调：启动 `virtual_stm32`，记下打印的 `/dev/pts/N`，再运行
`serial_bridge --port /dev/pts/N`。用 teleop 发布 `/cmd_vel`，即可验证
`/cmd_vel` -> `0x50` -> `/r5/imu_raw`/`/r5/wheel_vel` 链路。对应的 C 实现
在 `firmware/openctr_b20/`。
