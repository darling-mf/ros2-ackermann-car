# car_description

Robot description and simulation assets for the R5 Ackermann platform.

## Contents

| Path | Purpose |
|---|---|
| `urdf/car.urdf.xacro` | Top-level robot description |
| `urdf/common.xacro` | Shared dimensions and materials |
| `urdf/car/control.xacro` | `ros2_control` plugin, joint interfaces, odometry remaps |
| `urdf/car/wheels.xacro` | Wheel links, joints, friction, and drive axes |
| `urdf/car/laser.xacro` | 2D LiDAR link and Gazebo ray sensor |
| `urdf/car/imu.xacro` | IMU link and Gazebo sensor |
| `urdf/car/sonar.xacro` | Ultrasonic sensor model |
| `world/custom_room.world` | Indoor simulation world |
| `rviz/display.rviz` | Basic robot view |

## Frames

```text
map -> odom -> base_footprint -> base_link
                                  |-- laser_link
                                  |-- imu_link
                                  |-- front/rear wheel links
```

`map -> odom` is published by AMCL. `odom -> base_footprint` is published by
`sensor_fusion.py` in `car_bringup`, not by the chassis controller.

## Control Interfaces

`control.xacro` loads `gazebo_ros2_control` and remaps the custom controller's
odometry outputs:

```xml
<remapping>/r5_steering_controller/tf_odometry:=/tf_wheel</remapping>
<remapping>/r5_steering_controller/odometry:=/wheel/odom</remapping>
```

The controller writes four joint commands: two rear-wheel velocities and two
front-steering positions. The hardware interface on the real vehicle must
expose the same interfaces.

## Usage

The description is loaded by `car_bringup/launch/gazebo.launch.py`. To inspect
the model without starting Nav2:

```bash
ros2 launch car_bringup gazebo.launch.py
```

Dimensions in the xacro match the R5 manual and the measured chassis:
0.112 m wheelbase, 0.164 m track, 0.0325 m wheel radius. Re-measure after any
mechanical change before hardware tests.
