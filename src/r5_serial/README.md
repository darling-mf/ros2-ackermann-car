# r5_serial

ROS 2 bridge between `/cmd_vel` and an OpenCTR B20 lower controller using the
X-Protocol over UART, plus a virtual STM32 for hardware-free integration tests.

## Protocol

| Item | Value |
|---|---|
| Frame | `AA 55 LEN ID DATA CHECK` |
| Baud rate | 230400, 8N1 |
| STM32 -> ROS | `0x10` combined frame (IMU, wheel speed, voltage) at 50 Hz |
| ROS -> STM32 | `0x50` velocity frame (`vx`, `vy`, `wz`, m/s × 1000, big-endian) |
| Steering angle | `0x60`; `0x51` / `0x5F` exist in older firmware and are not implemented on B20 |

The protocol constants are in `r5_serial/protocol.py`; the frame parser is a
resynchronizing state machine that rejects bad length/checksum frames.

## Components

| File | Purpose |
|---|---|
| `r5_serial/protocol.py` | Frame builder/parser, `0x10` decoding, `0x50` encoding |
| `r5_serial/serial_port.py` | Serial port abstraction |
| `r5_serial/serial_bridge.py` | ROS node: `/cmd_vel` -> UART, UART -> `/r5/imu_raw`, `/r5/wheel_vel`, `/r5/battery` |
| `r5_serial/virtual_stm32.py` | Virtual STM32 over a pty; useful without hardware |

The bridge has a host-side watchdog: if `/cmd_vel` stops for `watchdog` seconds
(default 0.5 s), it sends a zero-velocity frame. This is an upper-computer
fallback; the lower controller should also implement its own command timeout.

## Usage

```bash
# Terminal 1: create a virtual serial device
ros2 run r5_serial virtual_stm32
# Note the printed /dev/pts/N

# Terminal 2: bridge
ros2 run r5_serial serial_bridge --port /dev/pts/N

# Terminal 3: drive
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Relevant parameters:

| Parameter | Default | Meaning |
|---|---|---|
| `port` | empty (required) | Serial device, passed as `--port` |
| `baud` | `230400` | Baud rate |
| `watchdog` | `0.5` | Zero-velocity timeout [s] |
| `send_imu` | `true` | Publish decoded IMU data |

The `0x50` send rate is a fixed 50 Hz (`CMD_RATE` in `serial_bridge.py`), aligned
with the firmware control task.

## Tests

The protocol module has a self-test for the documented example frame, a
`0x10` pack/parse round-trip, and bad-checksum resynchronization:

```bash
cd src/r5_serial
python3 -m r5_serial.protocol
```

For the serial bridge, start `virtual_stm32`, note the printed `/dev/pts/N`,
and run `serial_bridge --port /dev/pts/N`; a teleop publisher then exercises the
`/cmd_vel` -> `0x50` -> `/r5/imu_raw`/`/r5/wheel_vel` path. The companion C
implementation is in `firmware/openctr_b20/`.
