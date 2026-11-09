# 上游补丁

本目录归档 R5 控制器依赖的本地修改。上游仓库不直接放进本仓库，而是单独
clone 后打补丁。

## ros2_controllers-r5.patch

| 项目 | 值 |
|---|---|
| 上游 | https://github.com/ros-controls/ros2_controllers |
| 分支 | `humble` |
| 基线 commit | `65db7362`（2.54.1） |
| 目标目录 | colcon 工作区里的 `src/ros2_controllers` |

应用方式：

```bash
cd ~/car_ws/src/ros2_controllers
git checkout humble
git apply --check ../../patches/ros2_controllers-r5.patch
git apply ../../patches/ros2_controllers-r5.patch
```

## 补丁内容

| 文件 | 修改 | 原因 |
|---|---|---|
| `steering_odometry.{hpp,cpp}` | 增加带 `max_steering_angle` 的 `get_commands` 重载；限制虚拟中心角和单轮角；在轮角到虚拟角的逆变换中正确选择内/外轮 | 上游没有转向限位；纯旋转指令会算出不合理的转角 |
| `steering_controllers_library.cpp` | 修复里程计协方差：pose 和 twist 分开写；从节点参数读取 `max_steering_angle` | 上游把 pose 协方差写进 twist 数组，导致 pose 协方差恒为 0 |
| `steering_controllers_library.yaml` | 改成量纲合理的协方差默认值 | 上游默认值把矩阵对角下标当成了数值 |
| `ackermann_steering_controller/CMakeLists.txt` | 强制 overlay 头文件目录排在 `/opt` 之前 | 混用两个版本的头文件会改变类布局，可能崩溃 |
| `ackermann_steering_controller.yaml` | 声明 `max_steering_angle` | 未声明的 ROS 参数会被静默忽略 |

补丁保持 `SteeringControllersLibrary::Params` 的字段顺序与已安装二进制包
兼容；新增的方法重载不改变类布局。

## 重新构建

```bash
cd ~/car_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to car_bringup
source install/setup.bash
```
