# car_navigation

面向阿克曼底盘调过的 Nav2 行为树。

## 文件

| 文件 | 动作类型 |
|---|---|
| `behavior_trees/akm_navigate_to_pose.xml` | `NavigateToPose` |
| `behavior_trees/akm_navigate_through_poses.xml` | `NavigateThroughPoses` |

两棵树都由 `car_bringup/launch/nav2.launch.py` 注入，不需要在 RViz 里手动选。

## 阿克曼专用改动

- **不使用 `Spin` 恢复。** 底盘不能零速自转；`Spin` 只会把舵机打到限位，
  车本身不动。
- **限制倒车。** 恢复动作使用 `BackUp`，参数为
  `backup_dist="0.60"`、`backup_speed="0.12"`，速度足够慢，里程计不易发散。
- **先清代价地图。** 恢复顺序是先清局部/全局代价地图，再等待，再倒车。
  这样不会立刻重复上一轮失败的局部规划。
- **重规划频率。** `NavigateToPose` 以 1 Hz 重规划；
  `NavigateThroughPoses` 处理位姿数组，使用 0.333 Hz。

## 使用

```bash
ros2 launch car_bringup sim_nav2.launch.py
```

用标准 Nav2 action 发送目标：

```bash
ros2 action send_goal /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

多路点路线使用 `NavigateThroughPoses` 或 Nav2 waypoint follower。当地图门洞
较窄、或最终 yaw 受限时，把长路径拆成多个短位姿，比发一条长目标更稳。
