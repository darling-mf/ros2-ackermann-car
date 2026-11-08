# car_navigation

Nav2 behavior trees tuned for an Ackermann chassis.

## Files

| File | Action type |
|---|---|
| `behavior_trees/akm_navigate_to_pose.xml` | `NavigateToPose` |
| `behavior_trees/akm_navigate_through_poses.xml` | `NavigateThroughPoses` |

Both trees are injected by `car_bringup/launch/nav2.launch.py`; they are not
selected manually in RViz.

## Ackermann-Specific Changes

- **No `Spin` recovery.** The chassis cannot rotate at zero speed. A `Spin`
  action would command the steering servo to its limit without moving the car.
- **Bounded reverse.** Recovery uses `BackUp` with `backup_dist="0.60"` and
  `backup_speed="0.12"`; this is slow enough to keep odometry stable.
- **Costmap clearing first.** Recovery starts by clearing local/global
  costmaps, then waits, then backs up. This order avoids immediately repeating a
  failed local plan.
- **Replanning rate.** `NavigateToPose` replans at 1 Hz;
  `NavigateThroughPoses` uses 0.333 Hz because it handles a pose array.

## Usage

```bash
ros2 launch car_bringup sim_nav2.launch.py
```

Send a goal with the standard Nav2 action:

```bash
ros2 action send_goal /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

For multi-waypoint routes, use the `NavigateThroughPoses` action or the Nav2
waypoint follower. Segmenting a long route into short poses is preferred over
one long goal when the map has narrow doorways or the final yaw is constrained.
