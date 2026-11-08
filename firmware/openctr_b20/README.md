# OpenCTR B20 Firmware Modules

Portable C modules for the STM32F103RC-based OpenCTR B20 lower controller used
on the R5 Ackermann platform. The modules are written for the vendor's
StdPeriph/FreeRTOS project but have no hardware dependency, so they can be
compiled and tested on a PC.

The vendor firmware and manuals are not redistributed in this repository.

## Modules

| File | Purpose |
|---|---|
| `src/xprotocol.c/.h` | X-Protocol frame builder/parser with resynchronization and statistics |
| `src/safety.c/.h` | Command watchdog: 300 ms without a valid velocity frame disables motion |
| `src/ackermann.c/.h` | Virtual steering angle, inner-wheel angle, servo mapping, minimum-radius clamp |
| `test/test_host.c` | Host tests for the example frame, round-trip, bad-frame resync, watchdog, and Ackermann geometry |

## Host Tests

```bash
cd firmware/openctr_b20/test
gcc -std=c99 -Wall -Wextra -Werror -I../src \
  test_host.c ../src/xprotocol.c ../src/safety.c ../src/ackermann.c -lm \
  -o test_host
./test_host
```

Expected output: five PASS lines.

## Integration Points

The vendor project is Keil-based. Copy `src/*.c/.h` into the project and add
them to the Keil target. The integration points are:

1. **Initialization** — initialize the protocol parser and safety watchdog
   before starting the control task.
2. **UART receive** — feed received bytes into `xproto_feed()`; handle
   `0x50` velocity frames and reject bad length/checksum frames.
3. **Safety timeout** — call `safety_feed()` for each valid `0x50` frame and
   call `safety_allow()` from the control loop. If no valid command arrives
   within 300 ms, force zero wheel speed.
4. **Ackermann conversion** — use `akm_w_to_beta_mrad()` and
   `akm_inner_wheel_deg()` to convert `(vx, wz)` to the inner-wheel angle;
   clamp the minimum radius in the configuration.
5. **Servo output** — use `akm_servo_cmd_d10()` with a measured
   `servo_gain`/`servo_offset`; keep the mechanical limit enforced in software.

## Geometry Parameters

The C module exposes a configuration struct rather than hard-coded geometry:

```c
typedef struct {
    float axle_base;      /* wheelbase [m] */
    float wheel_base;     /* track [m] */
    float min_radius;     /* minimum turning radius [m] */
    float servo_gain;     /* servo angle = gain * inner wheel angle + offset */
    float servo_offset;   /* [deg], includes mechanical zero */
} akm_cfg_t;
```

Measure the real chassis before choosing values. The simulation uses a 0.146 m
wheelbase and 0.160 m track; the vendor manual and firmware differ on some
dimensions, so a tape measure and a full-lock circle test are required.

## Protocol Summary

| Item | Value |
|---|---|
| Frame | `AA 55 LEN ID DATA CHECK` |
| Baud rate | 230400, 8N1 |
| STM32 -> host | `0x10` combined data frame |
| Host -> STM32 | `0x50` velocity frame (`vx`, `vy`, `wz`, m/s × 1000, big-endian) |
| Steering | `0x60` |

The host-side counterpart is in `src/r5_serial/`.
