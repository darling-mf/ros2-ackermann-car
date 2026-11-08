# r5_steering_controller

A `ros2_control` controller for a rear-drive / front-steer Ackermann chassis.
It extends the upstream `ackermann_steering_controller` with three limits that
the upstream controller does not provide.

## Features

| Feature | Why it exists |
|---|---|
| Acceleration / deceleration ramp | Protects the motor driver and current budget; prevents launch wheel slip |
| Speed-dependent steering limit | Bounds lateral acceleration `a_y = v^2 * tan(phi) / L` to avoid high-speed side slip |
| Steering-rate limit | Respects the steering servo's maximum rate (about 9.5 rad/s) |
| Mechanical steering clamp | Upstream has no physical steering-angle limit; the R5 single-wheel limit is 0.235 rad |

The command post-processing runs in `update()` after the parent controller has
written its reference interfaces. This is required because the parent
overwrites those interfaces every control cycle.

## Files

| File | Purpose |
|---|---|
| `src/r5_steering_controller.cpp` | Controller implementation and limit layer |
| `include/r5_steering_controller/r5_steering_controller.hpp` | Class declaration |
| `r5_steering_controller.xml` | pluginlib export |
| `CMakeLists.txt` | Build and install rules |
| `../patches/ros2_controllers-r5.patch` | Upstream changes required by this controller |

## Configuration

The controller is configured in
`src/car_bringup/config/controllers.yaml`:

```yaml
r5_steering_controller:
  ros__parameters:
    wheelbase: 0.146
    front_wheel_track: 0.160
    rear_wheel_track: 0.160
    front_wheels_radius: 0.0325
    rear_wheels_radius: 0.0325

    max_steering_angle: 0.235
    r5.max_steer_mech: 0.235
    r5.max_steer_rate: 9.5
    r5.max_accel: 0.8
    r5.max_decel: 1.5
    r5.a_lat_max: 2.0
    r5.speed_dependent_limit: true
```

Geometry and limits are simulation defaults. Measure the real chassis before
using these values on hardware.

## Build

The controller depends on the patched upstream sources. See
[patches/README.md](../../patches/README.md) for the upstream commit and apply
steps.

```bash
cd ~/car_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to car_bringup
source install/setup.bash
```

## Verification

```bash
# Controller acceptance: straight, turn, reverse
ros2 run car_bringup acceptance_test.py --vx 0.5

# Direction / steering / wheel-speed loop
ros2 run car_bringup drive_test.py --vx 0.1 --wz 0.3

# Upstream controller tests
colcon test --packages-select steering_controllers_library
colcon test-result --verbose
```

The controller publishes `/wheel/odom`; TF is owned by `sensor_fusion.py` in
`car_bringup`. Do not enable `enable_odom_tf` here while fusion is running.
