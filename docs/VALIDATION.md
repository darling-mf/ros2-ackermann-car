# Validation

This page records what was measured, how it was measured, and what is still
unverified. It is intentionally conservative: a scenario is marked passed only
when the action result and the measured state agree.

## 1. Test Environment

| Item | Value |
|---|---|
| OS / ROS | Ubuntu 22.04, ROS 2 Humble |
| Simulator | Gazebo Classic 11.10.2 |
| Map | `src/car_bringup/maps/my_room_map.yaml` (376 × 222 at 0.05 m) |
| Controller | `r5_steering_controller` at 50 Hz |
| Localization | fused `/odom` + AMCL |
| Ground truth | Gazebo p3d `/ground_truth` (simulation only) |
| Start pose | `(0, 0, yaw 0)` |
| Test entry | `ros2 launch car_bringup sim_nav2.launch.py gui:=false rviz:=false` |

The ground-truth topic is used only for measurement. Nav2 and AMCL do not
consume it.

## 2. Test Matrix

| Area | Scenario | Result | Evidence |
|---|---|---|---|
| Chassis | Straight / turn / reverse acceptance | PASS | `acceptance_test.py` |
| Chassis | Direction, steering angle, wheel speed loop | PASS | `drive_test.py` |
| Upstream controller | `steering_controllers_library` tests | 38 passed | `colcon test` |
| Navigation | Goal `(2.0, 0)`, 0.3 m/s | `SUCCEEDED`, 0 recoveries | AMCL vs GT 3–5 cm |
| Navigation | Goal `(4.0, 0)`, 1.0 m/s | `SUCCEEDED`, 8.0 s | localization error 5.2 cm |
| Navigation | Goal `(6.0, 0)`, 1.0 m/s | `SUCCEEDED`, 4.8 s | localization error 6.0 cm |
| Odometry | Wheel + IMU fusion as default `/odom` | PASS | one `/odom` publisher; one TF authority |
| Odometry | Static gyro-bias calibration | PASS | 100 stationary samples; drift < 0.1 m at rest |
| Mapping | `slam_toolbox` + `map_saver` | PASS | map 376 × 222 at 0.05 m |
| Obstacle | Static cylinder at `(4.0, 0)`, goal `(6.0, 0)` | `SUCCEEDED`, 12 s, 0 recoveries | localization error 6.7 cm |
| Obstacle | Cylinder at `(6.5, 0)`, goal `(8.0, 0)` near wall | timeout at 60 s | goal inside inflated region |
| Long path | Left room `(-3.0, 0.5)`, yaw −π/2 | `SUCCEEDED`, 54 s, 0 recoveries | map vs GT ≈ 19 cm; yaw-sensitive |
| Serial | Example frame, round-trip, bad-frame resync | PASS | `python3 -m r5_serial.protocol` |
| Serial | Virtual STM32 + bridge | PASS | pty integration run |
| Firmware | Protocol, watchdog, Ackermann geometry | 5/5 | `firmware/openctr_b20/test/test_host.c` |

## 3. Method

### 3.1 Action result and runtime

Nav2 goals were sent with `ros2 action send_goal ... --feedback`. A scenario is
marked `SUCCEEDED` only when the action returns `SUCCEEDED`; final pose,
recovery count, and elapsed simulation time were read from the feedback/result.

### 3.2 Localization error

The current map pose is read from `tf2_echo map base_footprint` and compared
with Gazebo ground truth after the vehicle stops:

```text
error_xy = norm(map_xy - ground_truth_xy)
```

The map transform is used instead of the last `/amcl_pose` message because
AMCL publishes a pose only when it updates; at rest the last message can be
stale by one update interval. The reported values are steady-state errors, not
the maximum error during the run. The obstacle detour measured about 6.7 cm at
the end of the maneuver.

### 3.3 Costmap checks

For the obstacle test, the global costmap was read directly:

```text
(4.0, 0.0)  cost 88   (inflated obstacle)
(4.0, +0.4) cost 0    (free detour side)
```

This confirms that the obstacle entered the costmap and the successful path
could not have been a straight line through the obstacle. The test object is a
**static cylinder** in Gazebo; it does not validate prediction of a moving
pedestrian or vehicle.

### 3.4 TF and odometry checks

```bash
ros2 topic info /odom -v
# Publisher count: 1, node: sensor_fusion

ros2 run tf2_ros tf2_echo odom base_footprint
```

A single `odom -> base_footprint` authority is required. The controller is
configured with `enable_odom_tf: false`; it publishes `/wheel/odom` only.

The fusion node estimates the gyro bias from the first 100 stationary IMU
samples and applies a small deadband. Before this fix, a static Gazebo run
drifted to about 18° of heading; after the fix, the fused pose stayed within
0.1 m of the origin while stationary.

The fused odometry is published once per wheel-odom update, not at the higher
IMU rate. Publishing at IMU rate while holding the 50 Hz wheel twist caused
duplicate samples and finite-difference acceleration spikes; the fix keeps
`/odom` at the wheel-odom rate and still gives Nav2 enough TF updates.

## 4. High-Speed Result

The 1.0 m/s configuration uses the measured/manual geometry (0.112 m wheelbase,
0.164 m track, 0.0325 m wheel radius) and:

```yaml
# src/car_bringup/config/nav2_params.yaml
amcl:
  max_beams: 90
  update_min_d: 0.10
  update_min_a: 0.10

controller_server:
  FollowPath:
    vx_max: 1.0

velocity_smoother:
  max_velocity: [1.0, 0.0, 1.0]
```

The initial 1.0 m/s run had about 16 cm AMCL error. The main cause was the AMCL
update threshold: at 1.0 m/s, `update_min_d: 0.25` means an update only every
0.25 s. Reducing it to 0.10 m and increasing `max_beams` to 90 reduced the
steady-state error to about 5.4–5.5 cm.

The speed chain must stay consistent:

- MPPI `vx_max`;
- `velocity_smoother.max_velocity`;
- controller acceleration/deceleration limits.

Changing only one of them produces either a planner that promises more than
the chassis can execute or a controller that never reaches the planned speed.

## 5. Mapping

```bash
ros2 launch car_bringup mapping.launch.py
ros2 run nav2_map_server map_saver_cli \
  -f ~/car_ws/src/car_bringup/maps/my_room_map
```

The active map is `src/car_bringup/maps/my_room_map.pgm|yaml`. Mapping and Nav2
are mutually exclusive: both publish TF and `/cmd_vel` in ways that conflict if
started together.

## 6. Known Failures and Boundaries

### 6.1 Left-room long path

The shortest free path to `(-3.0, 0.5)` is 17.7 m; after 0.22 m inflation it is
18.1 m. The straight line is blocked near `(-2.0, 0.34)`. Goal yaw still
changes the result:

- yaw 0: a 15–16 m loop, still running after 110 s;
- yaw π: reaches the room but ends outside the 0.8 rad tolerance;
- yaw −π/2: after the geometry calibration, a full run from the origin
  **SUCCEEDED in 54 s with 0 recoveries**.

The yaw −π/2 result is a real pass, but the 19 cm map-vs-ground-truth error is
close to the 20 cm goal tolerance, and the path remains yaw-sensitive. Use a
feasible final yaw and, for production, segmented waypoints or a behavior tree
rather than one long goal.

### 6.2 Near-wall goals

A goal about 0.5 m from a wall can lie inside the inflated costmap. The vehicle
can physically approach the area but the goal is not reachable for the planner.
Use an approach pose at least 1 m from the wall, then finish with a short
low-speed maneuver.

### 6.3 Narrow passages and moving obstacles

Narrow-passage stress tests are not complete. The obstacle test uses a static
cylinder; it does not test a moving person, sensor latency, or prediction.

### 6.4 Real vehicle

No real-vehicle navigation result is claimed. The hardware interface, IMU
bias calibration, and emergency-stop integration remain. The geometry values
are from the manual and the measured chassis, but simulation is still a
baseline, not a calibration certificate.

The URDF, `controllers.yaml`, Nav2 parameters, and firmware host tests now use
one measured geometry set: 0.112 m wheelbase, 0.164 m track, 0.0325 m wheel
radius, and 0.55 m minimum turning radius. The vendor firmware constants
(0.146/0.160/0.35) are not used.

## 7. Reproduce

```bash
# Clean any previous simulation
ros2 run car_bringup cleanup_sim.sh

# Fresh stack
ros2 launch car_bringup sim_nav2.launch.py gui:=false rviz:=false

# Forward goal
ros2 action send_goal /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback

# Controller checks
ros2 run car_bringup acceptance_test.py --vx 0.5
ros2 run car_bringup drive_test.py --vx 0.1 --wz 0.3

# Fusion and serial
ros2 run car_bringup fusion_test.py --loops 3
ros2 run r5_serial virtual_stm32
ros2 run r5_serial serial_bridge --port /dev/pts/N
```
