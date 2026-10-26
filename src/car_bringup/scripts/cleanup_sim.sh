#!/bin/bash
# 清理仿真/Nav2 残留（重复启动会导致 amcl/bt_navigator/controller_server 多实例，
# 表现为 TF 冲突、goal 无响应、Send goal call failed）。
echo "[cleanup] 停止 launch 进程..."
pkill -9 -f "car_bringup (nav2|sim_nav2|gazebo)\.launch\.py" 2>/dev/null
echo "[cleanup] 停止容器/Gazebo/RViz..."
pkill -9 -x component_contai 2>/dev/null
pkill -9 -x gzserver 2>/dev/null
pkill -9 -x gzclient 2>/dev/null
pkill -9 -x rviz2 2>/dev/null
pkill -9 -x robot_state_pub 2>/dev/null
pkill -9 -x spawner 2>/dev/null
pkill -9 -f "auto_initial_pose[.]py" 2>/dev/null
sleep 1
ros2 daemon stop >/dev/null 2>&1 || true
echo "[cleanup] 完成。用 ros2 node list 确认没有 bt_navigator/amcl 残留。"
