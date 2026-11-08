# System Architecture

This document describes the public, buildable 2D LiDAR Ackermann stack: which
package owns each layer, which files implement it, and how data and TF flow.

## 1. Scope

The robot is a rear-drive / front-steer Ackermann platform based on a TARKBOT
R5-class chassis. The stack is developed and measured in Gazebo Classic 11 on
ROS 2 Humble. The real-vehicle boundary is kept behind `ros2_control` so the
upper layers do not change when the simulation is replaced by hardware.

```text
Task layer        bt_navigator + behavior trees
Planning          SmacPlannerHybrid (Reeds-Shepp)
Local control     MPPI (Ackermann motion model)
Velocity shaping  velocity_smoother
Chassis           r5_steering_controller (ros2_control)
Hardware boundary gazebo_ros2_control / SystemInterface
```

## 2. Package Responsibilities

| Package | Responsibility | Key files |
|---|---|---|
| `car_description` | URDF/xacro, Gazebo world, RViz configs | `urdf/car.urdf.xacro`, `urdf/car/control.xacro`, `urdf/car/wheels.xacro`, `world/custom_room.world` |
| `car_bringup` | Launch files, Nav2/controller params, maps, diagnostics | `launch/*.launch.py`, `config/controllers.yaml`, `config/nav2_params.yaml`, `config/slam_toolbox.yaml` |
| `r5_steering_controller` | Custom C++ Ackermann controller and safety limits | `src/r5_steering_controller.cpp`, `include/r5_steering_controller/r5_steering_controller.hpp` |
| `car_navigation` | Ackermann-specific Nav2 behavior trees | `behavior_trees/akm_navigate_to_pose.xml`, `akm_navigate_through_poses.xml` |
| `r5_serial` | X-Protocol serial bridge and virtual STM32 | `r5_serial/protocol.py`, `serial_bridge.py`, `virtual_stm32.py`, `serial_port.py` |
| `firmware/openctr_b20` | Portable STM32 C modules and host tests | `src/xprotocol.c/.h`, `src/safety.c/.h`, `src/ackermann.c/.h`, `test/test_host.c` |
| `patches` | Upstream `ros2_controllers` changes | `ros2_controllers-r5.patch` |

## 3. Control Chain

### 3.1 From command to joints

```text
/cmd_vel (geometry_msgs/Twist)
   |
   v
r5_steering_controller::update()
   |-- acceleration ramp
   |-- speed-dependent steering limit
   |-- steering-rate limit
   |-- Ackermann inverse kinematics
   v
rear_wheel_joint velocity x2 + front_steer_joint position x2
   |
   v
gazebo_ros2_control / real SystemInterface
```

The controller is a fork of the upstream
`ackermann_steering_controller`. The upstream controller writes reference
interfaces every cycle; the custom layer therefore post-processes the final
commands in `update()` instead of duplicating the upstream kinematics.

### 3.2 Why the limits exist

| Limit | Reason | Location |
|---|---|---|
| Acceleration ramp | Protects the motor driver and current budget; avoids wheel slip at launch | `r5_steering_controller.cpp` |
| Steering-rate limit | Servo specification: about 9.5 rad/s at the steering joint | `controllers.yaml` (`max_steer_rate`) |
| Speed-dependent steering | Bounds lateral acceleration `a_y = v^2 * tan(phi) / L`; prevents high-speed side slip | `r5_steering_controller.cpp` |
| Steering-angle clamp | Upstream controller does not implement a physical steering limit | controller source / patch |

Relevant parameters live in
`src/car_bringup/config/controllers.yaml`:

```yaml
wheelbase: 0.146
front_wheel_track: 0.160
rear_wheel_track: 0.160
front_wheels_radius: 0.0325
rear_wheels_radius: 0.0325
max_steering_angle: 0.235
r5.max_steer_rate: 9.5
r5.max_accel: 0.8
r5.max_decel: 1.5
r5.a_lat_max: 2.0
```

The values are simulation defaults and must be re-measured on the real chassis.
The URDF currently carries the drawing geometry (`0.112 m` wheelbase,
`0.164 m` track), while `controllers.yaml` carries the firmware/controller
values (`0.146 m`, `0.160 m`). This is a known calibration item: do not treat
either set as final until the chassis is measured.

### 3.3 Odometry ownership

```text
r5_steering_controller  -> /wheel/odom   (enable_odom_tf: false)
sensor_fusion.py        -> /odom + odom -> base_footprint
AMCL                    -> map -> odom
```

There is exactly one `odom -> base_footprint` broadcaster. This is deliberate:
two publishers create two TF trees, which makes AMCL and the costmaps jump.
See [TROUBLESHOOTING.md](TROUBLESHOOTING.md#1-duplicate-nav2-instances-and-two-tf-trees).

## 4. Navigation Stack

### 4.1 Task layer

`src/car_navigation/behavior_trees/akm_navigate_to_pose.xml` and
`akm_navigate_through_poses.xml` define the recovery behavior. The trees do not
use in-place `Spin`; an Ackermann chassis cannot rotate at zero speed. Recovery
uses bounded `BackUp` and `DriveOnHeading` instead.

`src/car_bringup/launch/nav2.launch.py` loads `config/nav2_params.yaml` and
injects the custom trees, so the behavior-tree selection is part of the launch
contract rather than a manual RViz setting.

### 4.2 Global planning

`planner_server` uses `SmacPlannerHybrid` with `REEDS_SHEPP` motion primitives:

```yaml
planner_server:
  ros__parameters:
    GridBased:
      plugin: nav2_smac_planner/SmacPlannerHybrid
      motion_model_for_search: REEDS_SHEPP
      minimum_turning_radius: 0.35
      reverse_penalty: 2.0
```

The planner is allowed to reverse because the vehicle has a minimum turning
radius and some goals are behind the current heading. Reverse motion is
penalized so forward paths are preferred.

### 4.3 Local control

`controller_server` runs `nav2_mppi_controller::MPPIController` with:

```yaml
motion_model: Ackermann
AckermannConstraints:
  min_turning_r: 0.35
vx_max: 1.0
vx_min: -0.15
```

MPPI tracks the global path and performs local obstacle avoidance. Pure
Pursuit, Stanley, and LQR are implemented separately in
`src/car_bringup/scripts/path_tracker.py`; they are controller experiments and
must not publish `/cmd_vel` while Nav2 is active.

### 4.4 Velocity shaping

`velocity_smoother` limits the final command before the chassis controller.
Its `max_velocity` must match the MPPI `vx_max`; otherwise the planner and the
execution layer disagree about the achievable speed.

## 5. Localization and Perception

### 5.1 2D LiDAR

```text
Gazebo ray sensor / real RPLIDAR or LD19
        |
        v
      /scan
        |-- AMCL (likelihood field)
        |-- local costmap obstacle layer
        |-- global costmap obstacle layer
```

The simulation sensor is defined in
`src/car_description/urdf/car/laser.xacro`; the mapping parameters are in
`src/car_bringup/config/slam_toolbox.yaml`.

### 5.2 AMCL

AMCL consumes `/scan`, `/odom`, and the static map, and publishes
`map -> odom`. The important parameters are in
`src/car_bringup/config/nav2_params.yaml`:

- `max_beams: 90`
- `update_min_d: 0.10`
- `update_min_a: 0.10`
- `laser_min_range: 0.25`

The tighter update thresholds are needed at 1.0 m/s; the default 0.25 m
distance threshold makes the filter lag behind the vehicle.

### 5.3 Costmaps

Both local and global costmaps use an obstacle layer plus an inflation layer.
The robot footprint is a rectangle, and `raytrace_min_range` /
`obstacle_min_range` are set to 0.25 m so that LiDAR returns from the vehicle's
own front wheels do not inflate into the robot center. See
[TROUBLESHOOTING.md](TROUBLESHOOTING.md#2-lidar-sees-the-robot-itself).

## 6. Serial and Firmware Boundary

The real-vehicle link is an X-Protocol frame stream over UART:

```text
AA 55 LEN ID DATA CHECK
```

`r5_serial` implements the ROS-side bridge and a virtual STM32 over a pty for
hardware-free integration tests. `firmware/openctr_b20` contains the portable C
modules:

- `xprotocol.c/.h`: frame parser and builder;
- `safety.c/.h`: 300 ms command watchdog;
- `ackermann.c/.h`: virtual steering angle, inner-wheel angle, and servo
  mapping with a configurable steering geometry.

The firmware modules are host-tested with `gcc`; vendor firmware and manuals
are not redistributed in this repository.

## 7. Launch Modes

| Command | Starts | Mutual exclusion |
|---|---|---|
| `gazebo.launch.py` | Gazebo + controller + fusion | Can be combined with Nav2 |
| `sim_nav2.launch.py` | Gazebo + Nav2 + RViz | Do not start a second Nav2 |
| `nav2.launch.py` | Nav2 only | Not with mapping |
| `mapping.launch.py` | Gazebo + `slam_toolbox` | Not with Nav2/AMCL |
| `fusion.launch.py` | Gazebo + one explicit fusion node | Does not start Nav2 |

Only one process may publish `/cmd_vel` at a time. `cleanup_sim.sh` stops
leftover Nav2 containers, Gazebo, RViz, mapping, and fusion processes.

## 8. Design Decisions and Trade-offs

| Decision | Alternative | Why this choice |
|---|---|---|
| Custom controller post-processing | Copy the upstream controller | Small patch, preserves upstream kinematics and tests |
| Smac Hybrid + Reeds-Shepp | Pure Dubin | Handles goals behind the vehicle and tight indoor turns |
| MPPI local controller | DWB / TEB | Good Ackermann support in Humble; predictable behavior with a rectangular footprint |
| Wheel + gyro complementary filter | Pure wheel odometry | Gyro supplies heading; wheel speed supplies translation; simple and testable |
| One fusion-owned TF | Controller and fusion both publish TF | Removes duplicate authority and AMCL jumps |
| Custom BT without `Spin` | Default Nav2 BT | An Ackermann vehicle cannot rotate in place |

## 9. Known Boundaries

- Long paths into the left room can exceed the local controller's effective
  horizon and time out; segmented waypoints are the intended next step.
- Goals closer than about 0.5 m to a wall may be unreachable because the goal
  lies inside the inflated costmap.
- Narrow-passage and moving-obstacle stress tests are not complete.
- The real-vehicle `SystemInterface`, wheel/IMU calibration, and integration
  are not finished. Simulation values are not a substitute for measurement.
