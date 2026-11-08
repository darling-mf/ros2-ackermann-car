# car_bringup

Launch files, parameters, maps, and host-side test tools for the R5 Ackermann
robot. This package is the entry point for simulation and navigation.

## Launch Files

| File | Purpose |
|---|---|
| `launch/gazebo.launch.py` | Gazebo + `ros2_control` + sensor fusion; optional RViz |
| `launch/sim_nav2.launch.py` | Gazebo + Nav2 + RViz in one command |
| `launch/nav2.launch.py` | Nav2 only, for an already running Gazebo/real robot |
| `launch/mapping.launch.py` | Gazebo + `slam_toolbox`; mutually exclusive with Nav2 |
| `launch/fusion.launch.py` | Standalone fusion validation; starts one explicit fusion node |

Only one process may publish `/cmd_vel` at a time. Run
`ros2 run car_bringup cleanup_sim.sh` before switching modes.

## Configuration

| File | Contents |
|---|---|
| `config/controllers.yaml` | Wheel geometry, joint order, `ros2_control` limits, odometry covariance, fusion TF ownership |
| `config/nav2_params.yaml` | AMCL, costmaps, Smac Hybrid, MPPI, velocity smoother |
| `config/slam_toolbox.yaml` | 2D mapping parameters |
| `maps/my_room_map.yaml` | Default simulation map (376 × 222 at 0.05 m) |

Key relationships:

- `controllers.yaml` sets `enable_odom_tf: false`; `sensor_fusion.py` is the
  only `odom -> base_footprint` broadcaster.
- `nav2_params.yaml` uses `SmacPlannerHybrid` with `REEDS_SHEPP`,
  `minimum_turning_radius: 0.55`, and `MPPI` with `motion_model: Ackermann`.
- The MPPI `vx_max`, `velocity_smoother.max_velocity`, and controller
  acceleration limits are a single speed chain and must be changed together.
- AMCL uses `max_beams: 90`, `update_min_d: 0.10`, and `update_min_a: 0.10`
  for the 1.0 m/s configuration.
- `sensor_fusion.py` estimates the gyro bias from 100 stationary samples and
  applies a deadband; keep the robot still for about two seconds after startup.
- Fused `/odom` is published once per wheel-odom update. Do not publish it at
  the higher IMU rate with a held wheel twist; finite differences produce a
  false acceleration spike.
- Geometry is the measured R5 set: 0.112 m wheelbase, 0.164 m track, 0.0325 m
  wheel radius, 0.55 m minimum turning radius.

## Scripts

| Script | Purpose |
|---|---|
| `scripts/acceptance_test.py` | Straight / turn / reverse controller acceptance |
| `scripts/drive_test.py` | Direction, steering angle, wheel-speed checks |
| `scripts/path_tracker.py` | Pure Pursuit / Stanley / LQR comparison; not part of Nav2 |
| `scripts/sensor_fusion.py` | Wheel + IMU fusion; publishes `/odom` and TF |
| `scripts/fusion_test.py` | Fusion error against `/ground_truth` |
| `scripts/plan_inspector.py` | Inspect reverse segments in a Nav2 plan |
| `scripts/cleanup_sim.sh` | Stop stale Gazebo/Nav2/RViz/mapping/fusion processes |

## Quick Checks

```bash
ros2 run car_bringup acceptance_test.py --vx 0.5
ros2 run car_bringup drive_test.py --vx 0.1 --wz 0.3
ros2 run car_bringup fusion_test.py --loops 3
```

If Nav2 accepts a goal but the robot does not move, check the instance count and
TF first:

```bash
ros2 node list | grep -E "amcl|bt_navigator|controller_server"
ros2 run tf2_ros tf2_echo map base_footprint
```

See the repository-level [architecture](../../docs/ARCHITECTURE.md),
[validation](../../docs/VALIDATION.md), and
[troubleshooting](../../docs/TROUBLESHOOTING.md) documents for the system view.
