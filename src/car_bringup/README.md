# car_bringup —— 阿克曼小车 Gazebo 控制

> 深度复盘（每个 bug 的根因链/为什么错/怎么改/验证/举一反三）见
> [CONTROL_DEBUG_NOTES.md](CONTROL_DEBUG_NOTES.md)。

本包负责把 `car_description` 里的 URDF 放进 Gazebo Classic，并加载
`ros2_control` 的阿克曼控制器。当前控制链路已经实测通过：

- 直行：`/cmd_vel` 的 `linear.x` 与 Gazebo 真实位姿方向一致
- 左/右转：外侧轮转角小、内侧轮转角大，后轮差速方向正确
- 倒车：`linear.x < 0` 时车向后退
- 纯旋转（`linear.x = 0`，`angular.z != 0`）：阿克曼车不能原地转，
  前轮会打到机械限位、车轮不转，车不漂移
- `/odom` 与 `/tf`（`odom -> base_footprint`）正常发布

---

## 1. 构建

```bash
cd ~/car_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to car_bringup
source install/setup.bash
```

`ackermann_steering_controller` 在当前机器上原本没有安装（所以之前
`ros2 control list_controllers` 只有 `joint_state_broadcaster`），现在从
`src/ros2_controllers` 源码编译。上面的 `packages-up-to` 会把它和
`steering_controllers_library` 一起编译。

如果只想编译最小集合：

```bash
colcon build --symlink-install --packages-select \
  steering_controllers_library ackermann_steering_controller \
  car_description car_bringup
```

> 说明：`src/FAST_LIO`、`src/FAST-LIO-SAM-QN`、`src/livox_ros_driver2`
> 和 `src/gz_ros2_control` 已放 `COLCON_IGNORE`（3D 雷达已舍弃；
> `gz_ros2_control` 是新 Gazebo 的插件，本项目用的是 Gazebo Classic）。
> 以后要用，删掉对应目录下的 `COLCON_IGNORE` 即可。

---

## 2. 启动

带界面：

```bash
source ~/car_ws/install/setup.bash
ros2 launch car_bringup gazebo.launch.py
```

无界面（服务器/远程调试推荐）：

```bash
ros2 launch car_bringup gazebo.launch.py gui:=false rviz:=false
```

启动后先确认两个控制器都是 `active`：

```bash
ros2 control list_controllers
```

期望输出：

```
joint_state_broadcaster        joint_state_broadcaster/JointStateBroadcaster        active
ackermann_steering_controller  ackermann_steering_controller/AckermannSteeringController  active
```

---

## 2.5 自动导航（S4）

```bash
ros2 launch car_bringup nav2.launch.py            # 带 RViz
ros2 launch car_bringup nav2.launch.py rviz:=false
```

`nav2.launch.py` 会把 `config/nav2_params.yaml` 真正传给 Nav2，并注入
`car_navigation/behavior_trees/akm_navigate_*.xml`（无 Spin、BackUp 0.6m）。
首次启动后需在 RViz 用 2D Pose Estimate 设初始位姿（或发 `/initialpose`）。

## 3. 手动控制

控制器订阅标准的 `geometry_msgs/msg/Twist` 话题 `/cmd_vel`
（`controllers.yaml` 中 `use_stamped_vel: false`）。

键盘遥控：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

注意：

- 阿克曼车不能原地转，尽量不要只按左右方向键；先给一点前进速度再转向。
- 建议线速度 `0.05 ~ 0.20 m/s`，角速度 `0.2 ~ 0.8 rad/s`。
- 车轮关节的 velocity limit 是 `10 rad/s`，对应约 `0.245 m/s` 的车速上限。

命令行直接发速度：

```bash
# 前进 0.1 m/s
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"

# 左转（曲率 0.3/0.1 = 3 1/m）
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.3}}"
```

---

## 4. 控制链路自检脚本

`drive_test.py` 会持续发布 `/cmd_vel`，同时读取 `/odom` 和
`/joint_states`，打印前轮转角、后轮速度，并判断方向是否正确。

```bash
# 直行
ros2 run car_bringup drive_test.py --vx 0.10 --wz 0.0 --duration 2.0

# 左转
ros2 run car_bringup drive_test.py --vx 0.10 --wz 0.3 --duration 2.0

# 倒车
ros2 run car_bringup drive_test.py --vx -0.10 --wz 0.0 --duration 1.5

# 纯旋转测试：车不动，前轮打到 ±0.5 限位，属于正常现象
ros2 run car_bringup drive_test.py --vx 0.0 --wz 0.5 --duration 1.5
```

### 全局路径倒车段检查

判断"目标在正后方/朝向难达到时小车不倒车"该归咎规划器还是控制器：

```bash
ros2 run car_bringup plan_inspector.py
```

持续监听 `/plan`，输出前进/倒车里程与换向次数。倒车段 = pose 朝向与路径
切线夹角大于 90°（nav2 的路径约定，MPPI/RPP 靠它识别倒车）。全 F 说明
规划器没给倒车段，先查 Smac 的 `reverse_penalty`；有 R 段则去查 MPPI
是否肯执行（看 `/cmd_vel` 有没有负速度）。

### R5 控制器三项验收

```bash
ros2 run car_bringup acceptance_test.py            # 默认 vx=0.5
ros2 run car_bringup acceptance_test.py --vx 1.0   # 高速场景（a_lat 限幅生效区间）
```

验收加速度限幅（斜坡）、速度相关转向限幅（ω 上限）、转向速率限幅
（dφ/dt 上限）三项，输出 PASS/FAIL 报告。逐行代码解析、踩坑实录与
"修改预测表"见 `src/r5_steering_controller/STAGE1_GUIDE.md`。

正常左转时（`vx=0.1, wz=0.3`）应看到：

- `right_front_steer_joint` ≈ `+0.263 rad`（外侧轮，转角小）
- `left_front_steer_joint` ≈ `+0.419 rad`（内侧轮，转角大）
- `right_rear_wheel_joint` 速度 > `left_rear_wheel_joint`（外侧快）
- `/odom` 的 `yaw` 增大，车向左画弧

---

## 5. 关键约定（不要随意改）

| 项目 | 约定 |
| --- | --- |
| 关节顺序 | 控制器内部按 **右轮、左轮** 生成指令；`controllers.yaml` 里两个数组也必须是“右、左” |
| 转向关节 | `front_wheels_names` 填的是 **转向关节**（`*_front_steer_joint`），不是前轮滚动关节 |
| 驱动关节 | `rear_wheels_names` 填后轮滚动关节，命令接口是 `velocity` |
| 前轮滚动 | 只读状态，不参与控制 |
| 坐标系 | `odom -> base_footprint -> base_link`；`base_footprint` 在两轴中点 |
| 转向限位 | 单个前轮 ±0.5 rad；控制器把它换算成虚拟中心角约 ±0.372 rad，最小转弯半径约 0.287 m |
| cmd_vel 类型 | `geometry_msgs/msg/Twist`（Nav2 Humble 默认） |

`car_description/urdf/car/wheels.xacro` 里把四个轮子的 `mu1/mu2`
都设为 1.0；前轮如果侧向摩擦太小（比如 0.1），车头会侧滑、转不了弯。
后轮 `axis` 用 `0 0 -1`，与 `rpy="1.5707 0 0"` 配合后正速度才对应
车向前。

---

## 6. 这次修掉的 bug / 坑

1. **控制器根本没加载**：系统里只有 bicycle/tricycle，没有
   `ackermann_steering_controller`。从源码编译后解决。
2. **左右轮顺序反了**：上游控制器约定“右轮在前”，原配置写成
   `[left, right]`，导致内侧/外侧转角和后轮差速全反，车会画龙/不转弯。
3. **`cmd_vel` 收不到**：原配置 `use_stamped_vel: true`，但 URDF 里只
   重映射了 `reference_unstamped`。改成 `use_stamped_vel: false` 后，
   `teleop_twist_keyboard` 和 Nav2 的 `Twist` 才能直接进控制器。
4. **后轮转轴方向反了**：`rpy="1.5707 0 0"` + `axis="0 0 1"` 时，
   正速度会让车后退。改为 `axis="0 0 -1"`。
5. **前轮侧向摩擦太小**：`mu2=0.1` 只适合差速/麦轮，阿克曼前轮需要
   侧向抓地，改为 1.0。
6. **URDF 注释导致 gazebo_ros2_control 崩溃**：插件会把整段
   `robot_description` 拼成 `--param robot_description:=...` 再交给 rcl
   解析，注释里出现冒号/URL 等字符会报
   `Couldn't parse parameter override rule`。launch 里现在会先用正则
   去掉 XML 注释再发布；同时控制文件里也不再写 ASCII 的“冒号+空格”。
7. **`max_steering_angle` 等参数是摆设**：官方 2.54.1 没有这些参数。
   已在 `steering_controllers_library` 源码里实现
   `max_steering_angle`（含虚拟中心角换算 + 单轮兜底限幅），
   `controllers.yaml` 中设为 0.5。`max_velocity` / `max_wheel_velocity`
   仍然不是官方参数，速度上限由 URDF 关节 limit 约束。
8. **里程计协方差 bug**：原代码把 pose 协方差写进了 twist 数组，且
   pose 协方差永远是 0。已修正为分别写入，并给出合理的默认值。
9. **前/后轮距**：官方 2.54.1 用后轮距 `wheel_track_` 计算前轮阿克曼转角。
   本车前后轮距都是 0.164 m，所以没有影响。为了和 `/opt` 里预编译的
   bicycle/tricycle 控制器保持 ABI 兼容，这次没有改 `SteeringOdometry`
   的类布局；以后如果前后轮距不同，再单独处理。
10. **launch 细节**：补上 `GAZEBO_MODEL_PATH`，增加 `gui`/`rviz` 开关，
    出生高度改为 0.02 m；RViz 固定坐标系改为 `odom`，能直接看到车在动。

修改的控制器源码文件：

```
src/ros2_controllers/steering_controllers_library/include/steering_controllers_library/steering_odometry.hpp
src/ros2_controllers/steering_controllers_library/src/steering_odometry.cpp
src/ros2_controllers/steering_controllers_library/src/steering_controllers_library.cpp
src/ros2_controllers/steering_controllers_library/src/steering_controllers_library.yaml
src/ros2_controllers/ackermann_steering_controller/src/ackermann_steering_controller.cpp
src/ros2_controllers/ackermann_steering_controller/src/ackermann_steering_controller.yaml
src/ros2_controllers/ackermann_steering_controller/CMakeLists.txt
```

> 当前机器上同时存在 `/opt` 的旧版 `steering_controllers_library 2.53.1`
> 和工作区源码的 2.54.1。ackermann 包的 CMakeLists 已强制优先使用
> overlay 头文件，避免“旧头文件 + 新库”导致的类布局不一致；基类参数
> 结构体也保持与旧版相同的字段顺序，方便和 `ros2_control` 二进制混用。

---

## 7. 已知限制 / 下一步

- **里程计是运动学推算**（后轮实际速度 + 前轮实际转角），急转弯时轮胎
  侧滑会让航向/位置有几厘米、几度的漂移。Nav2 用 AMCL + 雷达会纠正；
  如果要更准，可以把 `base_footprint` 挪到后轴，或接
  `robot_localization` 做融合。
- **Nav2 参数**：`car_bringup/config/nav2_params.yaml` 里 MPPI 的
  `AckermannConstraints.min_turning_r: 0.55` 比较保守（本车实际约
  0.287 m）。想让车掉头更利索，可改到 `0.30` 左右再实测；
  `velocity_smoother.max_velocity` 目前 0.26 m/s，略高于车轮极限
  0.245 m/s，建议改成 0.20。`behavior_server` 已没有启用 `spin`
  （阿克曼不能原地自转），保持这样。
- **实车**：当前 `control.xacro` 的硬件插件是
  `gazebo_ros2_control/GazeboSystem`。上实车时需要在 `<ros2_control>`
  里换成一个 `hardware_interface::SystemInterface` 插件，把
  “前轮位置 + 后轮速度”映射到舵机/电机。
- `ros2_control` 的 `ros2controlcli` / 其他控制器包没有全部从源码编译，
  当前用的是 `/opt/ros/humble` 的二进制，与源码 overlay 的
  `controller_manager 2.54.3` 兼容。
