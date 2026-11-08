# Contributing

## Build

```bash
cd ~/car_ws
source /opt/ros/humble/setup.bash

# If the patched controller sources are not already in the workspace:
git clone -b humble https://github.com/ros-controls/ros2_controllers.git \
  src/ros2_controllers
git -C src/ros2_controllers checkout 65db7362
git -C src/ros2_controllers apply \
  ../../patches/ros2_controllers-r5.patch

colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-up-to car_bringup
source install/setup.bash
```

## Tests

```bash
# Controller acceptance (straight / turn / reverse)
ros2 run car_bringup acceptance_test.py --vx 0.5

# Direction, steering angle, wheel-speed loop
ros2 run car_bringup drive_test.py --vx 0.1 --wz 0.3

# Upstream controller unit tests
colcon test --packages-select steering_controllers_library
colcon test-result --verbose

# Firmware C modules (no hardware required)
cd firmware/openctr_b20/test
gcc -std=c99 -Wall -Wextra -Werror -I../src \
  test_host.c ../src/xprotocol.c ../src/safety.c ../src/ackermann.c -lm \
  -o test_host
./test_host
```

## Change Rules

1. **Keep the layer boundary.** Chassis safety and Ackermann kinematics belong
   in `r5_steering_controller`; path tracking belongs in the Nav2 controller;
   task sequencing belongs in the behavior tree.
2. **One TF owner.** Do not enable `enable_odom_tf` in both the controller and
   `sensor_fusion.py`.
3. **Keep speed limits consistent.** MPPI `vx_max`, `velocity_smoother`
   `max_velocity`, and the controller acceleration limits must agree.
4. **Test before changing a parameter.** Record the scenario, the value, and the
   measured effect. Negative results are useful and should be kept.
5. **Patch upstream code through `patches/`.** Do not leave changes only in a
   local overlay; the patch must be reproducible.
6. **Do not commit generated or third-party material.** `build/`, `install/`,
   `log/`, `vendor/`, point clouds, rosbags, and vendor manuals are excluded.
7. **Update the matching README.** A change to launch files, parameters, or
   interfaces must update the package README and, when relevant,
   `docs/ARCHITECTURE.md`.

## Commit Messages

Use a scoped prefix:

```text
feat(control): add steering-rate limiting
fix(nav2): use a single AMCL instance
docs(arch): document the serial boundary
test(firmware): add bad-frame resync case
chore(repo): remove generated files
```

## Pull Requests

- State the problem, the root cause, and the verification command.
- Include measured before/after values when changing control or navigation
  parameters.
- Keep unrelated formatting changes out of functional commits.
- Do not add personal notes, interview material, or vendor source to the public
  repository.
