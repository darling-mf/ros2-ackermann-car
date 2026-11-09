# car_description

R5 阿克曼平台的机器人描述和仿真资源。

## 内容

| 路径 | 用途 |
|---|---|
| `urdf/car.urdf.xacro` | 机器人顶层描述 |
| `urdf/common.xacro` | 共用尺寸和材质 |
| `urdf/car/control.xacro` | `ros2_control` 插件、关节接口、里程计重映射 |
| `urdf/car/wheels.xacro` | 车轮 link、关节、摩擦和驱动轴 |
| `urdf/car/laser.xacro` | 2D 激光雷达 link 和 Gazebo ray 传感器 |
| `urdf/car/imu.xacro` | IMU link 和 Gazebo 传感器 |
| `urdf/car/sonar.xacro` | 超声波传感器模型 |
| `world/custom_room.world` | 室内仿真世界 |
| `rviz/display.rviz` | 基础机器人视图 |

## 坐标帧

```text
map -> odom -> base_footprint -> base_link
                                  |-- laser_link
                                  |-- imu_link
                                  |-- 前/后轮 link
```

`map -> odom` 由 AMCL 发布；`odom -> base_footprint` 由 `car_bringup` 里的
`sensor_fusion.py` 发布，不是底盘控制器发布。

## 控制接口

`control.xacro` 加载 `gazebo_ros2_control`，并重映射自研控制器的里程计输出：

```xml
<remapping>/r5_steering_controller/tf_odometry:=/tf_wheel</remapping>
<remapping>/r5_steering_controller/odometry:=/wheel/odom</remapping>
```

控制器写四路关节指令：两个后轮速度、两个前轮转角。实车硬件接口必须暴露
同样的接口。

## 使用

机器人描述由 `car_bringup/launch/gazebo.launch.py` 加载。只查看模型、不启动
Nav2：

```bash
ros2 launch car_bringup gazebo.launch.py
```

xacro 中的尺寸与 R5 手册和实车实测一致：轴距 0.112 m、轮距 0.164 m、轮
半径 0.0325 m。机械结构有改动后，实车测试前要重新测量。
