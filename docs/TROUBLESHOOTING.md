# Troubleshooting

Four issues that are easy to misdiagnose on an Ackermann Nav2 stack. Each case
lists the symptom, the evidence to collect first, the root cause, the fix, and
the verification. Commands assume `source install/setup.bash`.

## 1. Duplicate Nav2 instances and two TF trees

**Symptom**

- RViz shows `No transform from [base_footprint] to [map]`;
- `Send goal call failed`, or a goal is accepted but never makes progress;
- the robot moves in one run and jumps/flickers in the next;
- `/odom`, `/scan`, or `/cmd_vel` has more than one publisher.

**Collect first**

```bash
ros2 node list | grep -E "amcl|bt_navigator|controller_server|planner_server"
ros2 topic info /cmd_vel -v
ros2 topic info /odom -v
ros2 run tf2_ros tf2_monitor map base_footprint
```

Each node name should appear once. `tf2_monitor` should show one authority for
each transform, not two.

**Root cause**

Launching Nav2 twice (for example `sim_nav2.launch.py` and a separate
`nav2.launch.py`) starts two AMCL/controller containers. They compete for TF
and `/cmd_vel`; AMCL also publishes two different `map -> odom` corrections.
A crashed container can be left behind even after the launch terminal is
closed.

**Fix**

```bash
ros2 run car_bringup cleanup_sim.sh
ros2 node list | grep -E "amcl|bt_navigator|controller_server" || echo clean
```

Start exactly one stack:

```bash
ros2 launch car_bringup sim_nav2.launch.py
```

`src/car_bringup/scripts/cleanup_sim.sh` stops Gazebo, Nav2 containers, RViz,
mapping, and fusion processes. It must be run as its own command, not in the
same shell line as a launch command.

**Verification**

```bash
ros2 topic info /odom -v      # one publisher: sensor_fusion
ros2 run tf2_ros tf2_echo map base_footprint
```

Then send a short forward goal. A clean stack should reach it without recovery
and without TF warnings.

## 2. LiDAR sees the robot itself

**Symptom**

- the robot stops or refuses to move even in open space;
- RViz shows a ring of points inside the footprint;
- the local costmap has lethal/inflated cells at the robot center.

**Evidence**

A raw `/scan` check found 148 of 360 beams below 0.35 m, with the nearest at
0.100 m, concentrated around the front wheels and rear body. Those points were
inflated into the robot center, so every MPPI trajectory looked colliding.

**Root cause**

The LiDAR is mounted where it can see the steering mechanism and chassis.
`footprint_clearing` alone is not sufficient: it clears cells inside the
footprint after marking, but points on the footprint boundary can still be
inflated into the center.

**Fix**

In `src/car_bringup/config/nav2_params.yaml`, both costmaps and AMCL ignore
returns inside the footprint:

```yaml
amcl:
  laser_min_range: 0.25

local_costmap:
  local_costmap:
    ros__parameters:
      voxel_layer:
        footprint_clearing_enabled: true
        scan:
          raytrace_min_range: 0.25
          obstacle_min_range: 0.25

global_costmap:
  global_costmap:
    ros__parameters:
      obstacle_layer:
        footprint_clearing_enabled: true
        scan:
          raytrace_min_range: 0.25
          obstacle_min_range: 0.25
```

The raw sensor `range_min` can stay at 0.1 m; only the consumers ignore the
self-returns. 0.25 m is larger than the footprint half-diagonal
(`sqrt(0.14^2 + 0.11^2) ≈ 0.178 m`) and still detects real obstacles outside
the body.

**Verification**

Read the costmap cell at the robot center. It should be `0` (free), not `254`
(lethal) or `253` (inscribed). The raw `/scan` may still show self-hits; that is
acceptable if they are filtered before reaching the costmap.

## 3. MPPI configure crash with `consider_footprint`

**Symptom**

`controller_server` crashes during configure/startup when
`CostCritic.consider_footprint: true` is enabled.

**Root cause**

`CostCritic` requires a costmap footprint. If the costmap is configured with
only `robot_radius` and no `footprint`, the critic receives an empty polygon and
the process can segfault.

**Fix**

Define one rectangular footprint in `src/car_bringup/config/nav2_params.yaml`
and use it in both costmaps:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      footprint: "[[-0.14,-0.11],[-0.14,0.11],[0.14,0.11],[0.14,-0.11]]"
```

Keep the same polygon in the global costmap. The exact dimensions must be
measured on the real chassis.

**Verification**

```bash
ros2 node list | grep controller_server
ros2 param get /controller_server FollowPath.CostCritic.consider_footprint
```

The controller should stay alive and load the CostCritic plugin.

## 4. AMCL lags at 1.0 m/s

**Symptom**

The robot reaches the goal but AMCL drifts 10–16 cm from the Gazebo ground
truth at 1.0 m/s. At 0.3 m/s the same map and controller are accurate.

**Root cause**

AMCL updates only after the robot has moved `update_min_d` or rotated
`update_min_a`. With the default `0.25 m / 0.2 rad`, a 1.0 m/s vehicle gets an
update every 0.25 s, so the filter is always behind the motion. The 60-beam
scan is also sparse for high-speed correction.

**Fix**

In `src/car_bringup/config/nav2_params.yaml`:

```yaml
amcl:
  ros__parameters:
    max_beams: 90
    update_min_d: 0.10
    update_min_a: 0.10
```

Keep `vx_min: -0.15` for reverse. At 1.0 m/s, also make the speed chain
consistent: MPPI `vx_max`, `velocity_smoother.max_velocity`, and the controller
acceleration limits must agree.

**Verification**

| Goal | Result | AMCL vs ground truth |
|---|---|---|
| `(4.0, 0)` | `SUCCEEDED`, 8.2 s | 5.4 cm |
| `(6.0, 0)` | `SUCCEEDED`, 4.7 s | 5.5 cm |

A turning detour can still produce a larger transient error; compare final
steady-state values after the vehicle stops.

## 5. Reverse motion diverges odometry and AMCL

**Symptom**

A goal behind the robot causes a large AMCL jump: odometry error grows from
centimeters to nearly a meter, and the particle cloud saturates.

**Root cause**

The previous configuration allowed `vx_min: -0.4` and a long reverse arc.
Wheel odometry under reverse slip, combined with a large `map -> odom`
correction, produced a feedback loop. The planner also generated a long loop
with pure Dubin primitives.

**Fix**

Use Reeds-Shepp planning, penalize reverse, and limit reverse speed:

```yaml
planner_server:
  GridBased:
    motion_model_for_search: REEDS_SHEPP
    reverse_penalty: 2.0

controller_server:
  FollowPath:
    vx_min: -0.15
```

Remove in-place `Spin` from the Ackermann behavior tree; use bounded `BackUp`
or `DriveOnHeading` instead.

**Verification**

- short reverse goals should finish without an AMCL jump;
- `/odom` and `/ground_truth` should stay within the expected drift;
- after reverse, a forward goal must still succeed. This was tested explicitly:
  reverse to about −0.8 m, then forward to +0.5 m, `SUCCEEDED` in 8 s.

## 6. General Debug Order

1. Data chain: `/clock`, `/scan`, `/odom`, `/tf`, `/cmd_vel` publisher counts.
2. TF: `tf2_echo map base_footprint`; distinguish missing data from RViz display.
3. Single instance: one AMCL, one controller, one planner.
4. Localization: compare `/amcl_pose` with `/ground_truth` over time.
5. Costmap: read the cell at the robot center; do not judge only by color.
6. Goal reachability: check whether the goal is inside the inflated region.
7. Controller: check `vx_min`, footprint, and MPPI critic configuration.
8. Map: if a room fails consistently, rebuild the map before changing weights.

## 7. Useful Commands

```bash
# Nodes and topics
ros2 node list
ros2 topic info /cmd_vel -v
ros2 topic info /odom -v

# TF
ros2 run tf2_ros tf2_echo map base_footprint
ros2 run tf2_ros tf2_monitor map base_footprint

# Costmap center (Python/rclpy; see docs/VALIDATION.md for method)
# A center cell of 0 is free; 253/254 indicates inscribed/lethal.

# Cleanup
ros2 run car_bringup cleanup_sim.sh
```
