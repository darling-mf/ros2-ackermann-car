# Upstream Patch

This directory archives the local changes required by the R5 controller. The
upstream repository is not vendored; it is cloned separately and patched.

## ros2_controllers-r5.patch

| Item | Value |
|---|---|
| Upstream | https://github.com/ros-controls/ros2_controllers |
| Branch | `humble` |
| Base commit | `65db7362` (2.54.1) |
| Target | `src/ros2_controllers` in the colcon workspace |

Apply:

```bash
cd ~/car_ws/src/ros2_controllers
git checkout humble
git apply --check ../../patches/ros2_controllers-r5.patch
git apply ../../patches/ros2_controllers-r5.patch
```

## What the Patch Changes

| File | Change | Reason |
|---|---|---|
| `steering_odometry.{hpp,cpp}` | Add a `get_commands` overload with `max_steering_angle`; clamp the virtual center angle and the individual wheel angle; select the inner/outer wheel consistently in the wheel-to-virtual inverse transform | Upstream has no steering limit; a pure rotation command can produce an unrealistically large angle |
| `steering_controllers_library.cpp` | Fix odometry covariance: write pose and twist covariance separately; read `max_steering_angle` from the node parameters | Upstream writes pose covariance into the twist array, leaving pose covariance at zero |
| `steering_controllers_library.yaml` | Use dimensionally reasonable covariance defaults | The upstream defaults put matrix diagonal indices where values belong |
| `ackermann_steering_controller/CMakeLists.txt` | Force the overlay include directory before `/opt` | Mixing headers from two installed versions changes class layout and can crash |
| `ackermann_steering_controller.yaml` | Declare `max_steering_angle` | An undeclared ROS parameter is silently ignored |

The patch keeps the `SteeringControllersLibrary::Params` field order compatible
with the installed binary packages; the added method overload does not change
the class layout.

## Rebuild

```bash
cd ~/car_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to car_bringup
source install/setup.bash
```
