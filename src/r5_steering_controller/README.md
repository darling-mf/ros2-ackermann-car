# r5_steering_controller

面向后轮驱动、前轮转向阿克曼底盘的 `ros2_control` 控制器。它在上游
`ackermann_steering_controller` 基础上增加了三类上游没有的限幅。

## 功能

| 功能 | 存在原因 |
|---|---|
| 加速度/减速度斜坡 | 保护电机驱动和电流预算，避免起步打滑 |
| 速度相关转向限幅 | 约束侧向加速度 `a_y = v^2 * tan(phi) / L`，防止高速侧滑 |
| 转向速率限幅 | 遵守舵机最大速度（约 9.5 rad/s） |
| 机械转角钳位 | 上游没有物理转角限制；R5 单前轮限位 0.235 rad |

命令后处理放在 `update()` 里，在父控制器写完 reference 接口之后执行。
父类每个控制周期都会覆盖这些接口，所以必须放在这个位置。

## 文件

| 文件 | 用途 |
|---|---|
| `src/r5_steering_controller.cpp` | 控制器实现和限幅层 |
| `include/r5_steering_controller/r5_steering_controller.hpp` | 类声明 |
| `r5_steering_controller.xml` | pluginlib 导出 |
| `CMakeLists.txt` | 构建和安装规则 |
| `../patches/ros2_controllers-r5.patch` | 本控制器依赖的上游修改 |

## 配置

控制器参数在 `src/car_bringup/config/controllers.yaml`：

```yaml
r5_steering_controller:
  ros__parameters:
    wheelbase: 0.112
    front_wheel_track: 0.164
    rear_wheel_track: 0.164
    front_wheels_radius: 0.0325
    rear_wheels_radius: 0.0325

    max_steering_angle: 0.235
    r5.max_steer_mech: 0.235
    r5.max_steer_rate: 9.5
    r5.max_accel: 0.8
    r5.max_decel: 1.5
    r5.a_lat_max: 2.0
    r5.speed_dependent_limit: true
```

几何参数与 R5 手册和实车实测一致：轴距 112 mm、轮距 164 mm、轮胎直径
65 mm、最小转弯半径 0.55 m。底盘机械结构有改动后要重新测量。

## 构建

控制器依赖打过补丁的上游源码。上游 commit 和应用步骤见
[patches/README.md](../../patches/README.md)。

```bash
cd ~/car_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to car_bringup
source install/setup.bash
```

## 验证

```bash
# 控制器验收：直行、转向、倒车
ros2 run car_bringup acceptance_test.py --vx 0.5

# 方向 / 转角 / 轮速闭环
ros2 run car_bringup drive_test.py --vx 0.1 --wz 0.3

# 上游控制器测试
colcon test --packages-select steering_controllers_library
colcon test-result --verbose
```

控制器发布 `/wheel/odom`；TF 由 `car_bringup` 里的 `sensor_fusion.py` 负责。
融合节点运行时，不要在这里打开 `enable_odom_tf`。
