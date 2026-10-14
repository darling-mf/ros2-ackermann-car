# ROS 2 阿克曼小车建模 (Ackermann Car)

基于 ROS 2 Humble 和 Xacro 的阿克曼小车模型，包含底盘、驱动轮和转向轮。

## 📦 依赖
- Ubuntu 22.04
- ROS 2 Humble
- 安装依赖：`sudo apt install ros-humble-joint-state-publisher-gui ros-humble-xacro`

## 🚀 编译与运行
```bash
cd ~/car_ws
rm -rf build install log  # 清理缓存
colcon build
source install/setup.bash
ros2 launch car_description display.launch.py

如果你在启动过程中遇到报错（如 RViz 不显示模型、xacro 崩溃等），请查阅 TROUBLESHOOTING.md。