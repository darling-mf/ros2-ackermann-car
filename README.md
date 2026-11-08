# ROS 2 Ackermann Car

A ROS 2 Humble + Gazebo Classic Ackermann platform for a TARKBOT R5-class
vehicle. The stack covers the full path from URDF and `ros2_control` execution
to Nav2 global/local planning, 2D LiDAR mapping, AMCL localization, and a
serial bridge to an STM32 lower controller.

The public repository focuses on the **2D LiDAR navigation stack**. The earlier
3D LiDAR experiments are not part of the build.

## Highlights

- **Custom C++ `ros2_control` controller** for a rear-drive / front-steer
  Ackermann chassis, with acceleration, steering-rate, and speed-dependent
  steering limits.
- **Nav2 Ackermann navigation**: Smac Hybrid planner (Reeds-Shepp), MPPI local
  controller, and a custom behavior tree without in-place `Spin` recovery.
- **Wheel + IMU odometry fusion** as the default odometry source. The controller
  publishes `/wheel/odom`; one fusion node publishes `/odom` and the single
  `odom -> base_footprint` transform.
- **2D mapping and localization**: `slam_toolbox`, `map_saver`, AMCL, and a
  footprint-aware costmap configuration.
- **Embedded path**: X-Protocol serial bridge, a virtual STM32 for host tests,
  and portable C modules for protocol parsing, a 300 ms safety watchdog, and
  Ackermann geometry.

## Validated Results

All numbers below are from the Gazebo simulation with the map in
`src/car_bringup/maps/my_room_map.yaml` and fused odometry.

| Scenario | Result | Evidence |
|---|---|---|
| Controller acceptance | PASS | `acceptance_test.py` (straight, turn, reverse) |
| Forward goal `(2.0, 0)` | `SUCCEEDED`, 0 recoveries | AMCL vs ground truth: 3–5 cm |
| High-speed `(4.0, 0)` at 1.0 m/s | `SUCCEEDED`, 8.0 s | localization error ≈ 5.2 cm |
| High-speed `(6.0, 0)` at 1.0 m/s | `SUCCEEDED`, 4.8 s | localization error ≈ 6.0 cm |
| Obstacle at `(4.0, 0)`, goal `(6.0, 0)` | `SUCCEEDED`, 12 s, 0 recoveries | static cylinder; localization error ≈ 6.7 cm |
| Left room `(-3.0, 0.5)`, yaw −π/2 | `SUCCEEDED`, 54 s, 0 recoveries | map vs GT ≈ 19 cm; yaw-sensitive |
| 2D mapping | map saved | 376 × 222 cells at 0.05 m |
| Serial bridge + virtual STM32 | protocol self-checks + virtual-device run | example frame, round-trip, bad-frame resync |
| STM32 C modules | 5/5 host tests | `firmware/openctr_b20/test/test_host.c` |

Known limitations are listed in [docs/VALIDATION.md](docs/VALIDATION.md):
left-room navigation is still yaw-sensitive, near-wall goals can be
unreachable, and narrow-passage stress tests are not complete.

## System Architecture

```text
Task layer        bt_navigator + custom behavior tree
                  src/car_navigation/behavior_trees/akm_navigate_to_pose.xml
       |
Planning          planner_server + SmacPlannerHybrid (Reeds-Shepp)
                  src/car_bringup/config/nav2_params.yaml
       |
Local control     controller_server + MPPI (Ackermann model)
                  src/car_bringup/config/nav2_params.yaml
       |
Velocity shaping  velocity_smoother -> /cmd_vel
       |
Chassis           r5_steering_controller (50 Hz, Ackermann IK)
                  src/r5_steering_controller/src/r5_steering_controller.cpp
       |
Hardware boundary gazebo_ros2_control (simulation) / SystemInterface (real car)
```

Perception and localization:

```text
/scan (2D LiDAR) --> AMCL: map -> odom
                 --> local/global costmaps

/wheel/odom + /imu --> sensor_fusion.py --> /odom + odom -> base_footprint
```

The main design boundary is intentional: `r5_steering_controller` performs
Ackermann inverse kinematics and safety limiting, but does not decide where to
drive. `MPPI` tracks the global plan; the separate `path_tracker.py` implements
Pure Pursuit / Stanley / LQR for controller comparison and is not part of the
Nav2 chain.

## Repository Layout

```text
.
├── README.md
├── CONTRIBUTING.md
├── docs/
│   ├── README.md                  # public documentation index
│   ├── ARCHITECTURE.md            # layers, interfaces, data flow, file map
│   ├── VALIDATION.md              # test method, measured results, limits
│   └── TROUBLESHOOTING.md         # reproducible Nav2/TF/costmap/AMCL cases
├── src/
│   ├── car_description/           # URDF/xacro, Gazebo world, RViz configs
│   ├── car_bringup/               # launch, params, maps, test scripts
│   ├── r5_steering_controller/    # custom C++ Ackermann controller
│   ├── r5_serial/                 # X-Protocol bridge + virtual STM32
│   └── car_navigation/            # Ackermann behavior trees
├── firmware/openctr_b20/          # portable STM32 C modules + host tests
├── patches/                       # upstream ros2_controllers patch
└── LICENSE
```

Each package contains its own `README.md` with the exact files to edit and the
interfaces it exposes.

## Build

### Dependencies

- Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11
- `ros-humble-gazebo-ros2-control`
- `ros-humble-nav2-bringup`
- `ros-humble-slam-toolbox`
- `ros-humble-robot-state-publisher`, `ros-humble-xacro`

The custom controller builds on top of `ackermann_steering_controller`. The
patched upstream sources are archived in `patches/`; do not rely on a binary
package if you need the steering-limit and odometry-covariance fixes.

### Build the workspace

```bash
cd ~/car_ws
source /opt/ros/humble/setup.bash

# Optional but recommended: patched controller sources
git clone -b humble https://github.com/ros-controls/ros2_controllers.git \
  src/ros2_controllers
git -C src/ros2_controllers checkout 65db7362
git -C src/ros2_controllers apply \
  ../../patches/ros2_controllers-r5.patch

colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-up-to car_bringup
source install/setup.bash
```

If the upstream sources are already installed and patched in your workspace,
the `git clone` step can be skipped. See [patches/README.md](patches/README.md)
for the exact upstream commit and patch scope.

## Run

```bash
# Gazebo + controller
ros2 launch car_bringup gazebo.launch.py

# Gazebo + Nav2 + RViz (single Nav2 instance)
ros2 launch car_bringup sim_nav2.launch.py

# 2D mapping (mutually exclusive with Nav2)
ros2 launch car_bringup mapping.launch.py

# Serial bridge with a virtual STM32 (no hardware)
ros2 run r5_serial virtual_stm32
ros2 run r5_serial serial_bridge --port /dev/pts/N
```

Send a navigation goal:

```bash
ros2 action send_goal /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

Only one process may publish `/cmd_vel` at a time. When switching between Nav2,
mapping, and the tracker, run `ros2 run car_bringup cleanup_sim.sh` first.

## Documentation

| Document | What it answers |
|---|---|
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Which package/file owns each layer, how data and TF flow |
| [docs/VALIDATION.md](docs/VALIDATION.md) | What was tested, with which parameters and results |
| [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) | Reusable diagnosis paths for Nav2, TF, costmap, and AMCL |
| [src/r5_steering_controller/README.md](src/r5_steering_controller/README.md) | Controller limits, parameters, and test hooks |
| [src/car_bringup/README.md](src/car_bringup/README.md) | Launch modes, params, maps, and scripts |
| [src/r5_serial/README.md](src/r5_serial/README.md) | Serial protocol and virtual STM32 |
| [firmware/openctr_b20/README.md](firmware/openctr_b20/README.md) | STM32 integration and host tests |
| [patches/README.md](patches/README.md) | Upstream controller patch and apply steps |

## Design Notes

- **Command-layer post-processing.** The parent controller overwrites reference
  interfaces every cycle, so acceleration and steering limits are applied in
  `update()` after the parent output. This keeps the patch small and avoids
  copying the upstream kinematics.
- **Speed-dependent steering limit.** The lateral acceleration is bounded by
  `a_y = v^2 * tan(phi) / L`. At low speed the steering range is preserved; at
  high speed the command is limited before it reaches the servo model.
- **One TF owner.** Enabling `enable_odom_tf` in both the controller and the
  fusion node creates two `odom -> base_footprint` authorities. The controller
  is configured to publish `/wheel/odom` only. The fusion node also estimates
  the stationary gyro bias so the fused heading does not drift while parked.
- **No in-place spin.** An Ackermann chassis cannot rotate at zero speed. The
  custom behavior tree removes `Spin` and uses bounded `BackUp` /
  `DriveOnHeading` recovery instead.
- **Measured, not guessed.** Controller tests, AMCL-vs-ground-truth error, and
  costmap cell values are recorded in [docs/VALIDATION.md](docs/VALIDATION.md).

## Status and Roadmap

- Simulation: control, forward navigation, 1.0 m/s navigation, 2D mapping,
  fused odometry, and serial protocol are validated.
- Open simulation work: left-room paths with arbitrary yaw (yaw=0 still loops),
  near-wall goal handling, narrow passages, and moving-obstacle prediction.
- Real vehicle: hardware interface, wheel/IMU calibration, and full integration
  remain. The current repository is a simulation-validated baseline, not a
  finished product.

## Contributing

Build/test commands, patch policy, and commit conventions are in
[CONTRIBUTING.md](CONTRIBUTING.md).

## License

Original code in this repository is released under the Apache License 2.0; see
[LICENSE](LICENSE). Third-party components retain their original licenses.
Vendor firmware and manuals are not redistributed in this repository.
