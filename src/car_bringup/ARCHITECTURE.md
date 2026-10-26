# R5 小车全栈架构详解 —— 谁是谁、谁管谁、指令怎么流

> 说明：本文是**控制层内部深读**（类继承/control.xacro/controllers.yaml）。
> 全栈分层、导航/规划/雷达与改动方法见 `docs/ENGINEERING_HANDOVER.md`。

> 本文回答的问题：每个文件夹/文件是干嘛的？谁是上层谁是下层？谁发指令、
> 谁收指令、谁执行？为什么既要写 .yaml 又要写 .cpp？消息接口改没改？
> ChainableControllerInterface 到底在哪个文件里？
> 所有路径均为本机真实路径（已逐一核实）。

---

## 1. 一张总图：四个层级

```
┌─────────────────────────────────────────────────────────────┐
│ L3 决策层（导航）    Nav2 / teleop / path_tracker.py          │  发 "我想以 (v,ω) 运动"
│                      ↓ /cmd_vel (geometry_msgs/Twist)         │
├─────────────────────────────────────────────────────────────┤
│ L2 控制层（ros2_control）                                     │  翻译成 4 个关节指令
│   R5SteeringController → Ackermann... → SteeringControllers   │  (右后轮速, 左后轮速,
│   Library → ChainableControllerInterface（框架基类）           │   右前轮角, 左前轮角)
│                      ↓ command_interfaces_ (4 个句柄)          │
├─────────────────────────────────────────────────────────────┤
│ L1 硬件抽象层    controller_manager + ResourceManager          │  搬运工+调度员
│                  + GazeboSystem (gazebo_ros2_control 插件)     │
│                      ↓ ODE 关节 API                            │
├─────────────────────────────────────────────────────────────┤
│ L0 物理层          Gazebo Classic 11 (gzserver 物理引擎)       │  真正"执行"的地方
│                  ← state_interfaces_ (12 个句柄) 回传          │
└─────────────────────────────────────────────────────────────┘
     状态流自下而上：关节真实状态 → GazeboSystem::read() → state_interfaces_
     → R5 控制器 FK 算里程计 → /odom + /tf；joint_state_broadcaster → /joint_states
```

**一句话记层级**：L3 说"想去哪"，L2 说"每个轮子该转多快/转多大角"，
L1 只是搬运工（把数字送到关节、把状态带回来），L0 是物理世界本身。

---

## 2. 三个包族：你改的、你写的、官方的

### 2.1 你自己写的包（车）

```
src/car_description/          车"长什么样"——纯模型，没有任何行为代码
├── urdf/car.urdf.xacro       总装入口：include 下面所有子模块
├── urdf/common.xacro         尺寸/质量参数（轮径 0.0325、轴距 0.112 在这）
├── urdf/car/chassis.xacro    底盘的 link/joint 定义
├── urdf/car/wheels.xacro     四个轮子：关节 axis/限位/摩擦（±0.235、9.5、45 在这）
├── urdf/car/control.xacro    ★ 全项目最关键的接线文件，见第 4 节
├── urdf/car/laser|imu|sonar.xacro   传感器模型 + Gazebo 传感器插件
└── world/custom_room.world   Gazebo 房间（墙/家具）

src/car_bringup/              车"怎么启动、参数是多少"——装配层
├── launch/gazebo.launch.py   把下面所有东西按顺序拉起来（含剥 XML 注释的补丁）
├── config/controllers.yaml   ★ 控制器的运行参数（改参数只动这里）
├── config/nav2_params.yaml   Nav2 导航参数（当前对比实验没用 Nav2）
├── maps/                     之前建好的 2D 栅格地图
└── scripts/                  实验工具：drive_test.py / acceptance_test.py /
                              path_tracker.py / plan_inspector.py

src/r5_steering_controller/   车"大脑的行为"——我们自己写的控制器插件
├── r5_steering_controller.xml        pluginlib 注册表（"电话簿"，见 5.2）
├── include/r5_steering_controller/
│   └── r5_steering_controller.hpp    类声明：成员变量、三个功能的参数
├── src/r5_steering_controller.cpp    类实现：on_init 读参数 + 50Hz 的四步流水线
├── STAGE1_GUIDE.md / EXPERIMENT_LOG.md   阶段一文档
└── CMakeLists.txt                    怎么把 cpp 编译成 libr5_steering_controller.so
```

### 2.2 官方框架源码（克隆下来只编译了需要的部分）

```
src/ros2_control/             ros2_control 框架本体（humble 分支 2.54.3）
├── controller_manager/       ★"中枢神经"：加载控制器、跑 50Hz update 循环、
│                             管理接口资源。它运行在 gzserver 进程内部！
├── controller_interface/     ★ 控制器的基类库：
│   include/controller_interface/
│   ├── controller_interface_base.hpp    ← 类树最顶上那个
│   ├── controller_interface.hpp         ← 普通控制器基类
│   └── chainable_controller_interface.hpp  ← ★ 你问的那个类就在这
└── hardware_interface/       StateInterface/CommandInterface 句柄的定义、
                              ResourceManager（分配接口）、SystemInterface
                              （硬件插件基类，GazeboSystem 实现了它）

src/ros2_controllers/         官方控制器集合（20+ 个，我们只编译了 2 个）
├── steering_controllers_library/     阿克曼/自行车/三轮的公共运动学库
│   ├── include/.../steering_odometry.hpp       FK/IK 数学（纯算法，无 ROS）
│   ├── src/steering_controllers_library.cpp    生命周期+主循环（我们改过协方差）
│   └── src/steering_controllers_library.yaml   ★ 它的参数"声明表"
└── ackermann_steering_controller/    官方阿克曼控制器（R5 的父类）
    ├── include/.../ackermann_steering_controller.hpp   子类声明
    ├── src/ackermann_steering_controller.cpp           只做：把 ACKERMANN_CONFIG 传给里程计
    └── src/ackermann_steering_controller.yaml          ★ 它的参数"声明表"
```

注意这些包**同时存在于两处**：`/opt/ros/humble`（apt 装的旧二进制，
如 steering_controllers_library 2.53.1）和 `src/` + `install/`（我们源码
编译的新版 2.54.1，覆盖旧版）。终端里 `source install/setup.bash` 之后
用的是新版——这就是 colcon "overriding" 警告的含义。

### 2.3 官方二进制（没源码，直接用的）

- `/opt/ros/humble/lib/libgazebo_ros2_control.so`：Gazebo↔ros2_control
  的桥。它实现了 hardware_interface::SystemInterface（GazeboSystem 类），
  负责 read()/write() 把 state/command 接口与 ODE 关节互转。
- Nav2 全家、teleop_twist_keyboard 等 L3 组件。

---

## 3. 你问的类继承树——每个类在哪个文件

（STAGE1_GUIDE 3.1 节那张图的"文件落点"）

```
ControllerInterfaceBase
│   文件：src/ros2_control/controller_interface/include/
│         controller_interface/controller_interface_base.hpp
│   职责：生命周期状态机(unconfigured→inactive→active)、持有
│         command_interfaces_/state_interfaces_ 句柄数组、update() 入口
│
└─ ChainableControllerInterface          ← 你问的类
│   文件：src/ros2_control/controller_interface/include/
│         controller_interface/chainable_controller_interface.hpp
│         （/opt/ros/humble/include/controller_interface/controller_interface/
│           chainable_controller_interface.hpp 是同一个文件的 apt 版）
│   职责：提供 reference_interfaces_ 机制——允许上游控制器不经过话题、
│         直接把输出写进下游的参考接口（"链式"）。我们的 R5 继承它
│         纯粹因为父类继承它；我们没用链式功能。
│
└─ SteeringControllersLibrary（抽象基类，不能直接用）
    文件：src/ros2_controllers/steering_controllers_library/
          include/steering_controllers_library/steering_controllers_library.hpp
          src/steering_controllers_library.cpp
          src/steering_odometry.cpp / include/.../steering_odometry.hpp
    职责：干所有脏活——创建订阅/发布、管理 4+4 个接口的顺序契约、
          50Hz 主循环（重载参考→FK 里程计→IK 出轮指令→写接口→发 /odom）、
          超时停车。运动学数学全在 steering_odometry.cpp。
          我们改过它：协方差 bug 修复、max_steering_angle 限幅。
    ★ 它是抽象的：订阅哪个话题(use_stamped_vel)、速度上限等由子类决定
      的部分留成了纯虚函数。
│
└─ AckermannSteeringController
    文件：src/ros2_controllers/ackermann_steering_controller/
          include/ackermann_steering_controller/ackermann_steering_controller.hpp
          src/ackermann_steering_controller.cpp
    职责：小到只有 80 行——告诉基类"我是阿克曼构型"(ACKERMANN_CONFIG)、
          把轮距/轴距/轮径塞给里程计。轮距用后轮距、转角计算等具体几何
          由基类模板完成。它的参数声明在 src/ackermann_steering_controller.yaml。
│
└─ R5SteeringController（我们写的）
    文件：src/r5_steering_controller/include/r5_steering_controller/
          r5_steering_controller.hpp
          src/r5_steering_controller.cpp
    职责：不碰任何运动学。只在父类写完 4 个关节指令之后、硬件读取之前，
          做"斜坡+限幅+限速率"三步后处理。
```

**为什么一层层继承**：每一层只回答一个问题——框架层回答"控制器怎么
被管理"，库层回答"转向机器人运动学怎么算"，ackermann 层回答"参数
怎么填进运动学"，R5 层回答"这台实车还需要什么额外约束"。改哪层就
只动哪层的文件。

---

## 4. control.xacro——全项目的"总接线板"（最该读懂的文件）

`src/car_description/urdf/car/control.xacro` 做两件事：

### 4.1 `<ros2_control>` 标签：声明"硬件有什么接口"

```xml
<ros2_control name="GazeboSimSystem" type="system">
  <hardware><plugin>gazebo_ros2_control/GazeboSystem</plugin></hardware>
  <joint name="left_rear_wheel_joint">
    <command_interface name="velocity"/>      ← 声明：这个关节可被命令速度
    <state_interface name="position"/>        ← 声明：可读它的位置
    <state_interface name="velocity"/>
  </joint>
  ...（4 个关节 × 各自接口，共 4 命令 + 12 状态）
</ros2_control>
```

这是**模型侧的声明**："我（这台车）能接受什么命令、能提供什么状态"。

### 4.2 `<gazebo><plugin>` 标签：告诉 Gazebo 加载桥

```xml
<plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
  <parameters>$(find car_bringup)/config/controllers.yaml</parameters>  ← 指到配置
  <ros>
    <remapping>/r5_steering_controller/odometry:=/odom</remapping>       ← 改话题名
    <remapping>/r5_steering_controller/reference_unstamped:=/cmd_vel</remapping>
  </ros>
</plugin>
```

gzserver 加载模型时读到这段 → 加载 .so → 在 Gazebo 进程内部创建
controller_manager 和全部控制器。**这就是为什么 controller_manager
不在独立进程里，`ps` 看不到它**。

### 4.3 上实车时只改这一个文件

把 `<plugin>gazebo_ros2_control/GazeboSystem</plugin>` 换成你自己写的
SystemInterface 插件（串口收发），上面所有控制器代码**一行不改**——
这就是 ros2_control 分层的全部意义。

---

## 5. controllers.yaml 深度解析（三个名字的关系 + 两种 yaml 的区别）

### 5.1 你问的"是不是改了官方的 controllers.yaml"

**纠正一个概念：controllers.yaml 从来不是官方文件，它是我们 car_bringup
自己的配置。** 官方仓库里那个长得像的 yaml 是另一个东西。正确区分：

| 文件 | 角色 | 谁消费它 |
|---|---|---|
| `src/ros2_controllers/ackermann_steering_controller/src/ackermann_steering_controller.yaml` | **参数声明表**（schema）：有哪些参数、类型、默认值、校验规则。由 generate_parameter_library 在**编译期**变成 C++ 代码编进插件 | 编译器 |
| `src/car_bringup/config/controllers.yaml` | **参数赋值表**（config）：给上面声明的参数（和 r5.* 参数）填运行值 | controller_manager 加载控制器时 |

类比：声明表是"表格模板"（编译进插件），controllers.yaml 是"填好的表"。
在 controllers.yaml 里写一个声明表里不存在的键（如曾经的
max_velocity、min_steering_angle）→ **静默忽略，不报错**——这是我们付
过学费的教训。`r5.*` 六个参数不在官方声明表里，它们由我们自己的
`on_init` 用 `has_parameter + declare_parameter` 声明（轻量路线）。

### 5.2 controllers.yaml 里三个名字的关系（最易懵的点）

```yaml
controller_manager:
  ros__parameters:
    r5_steering_controller:                              # ① 实例名
      type: r5_steering_controller/R5SteeringController  # ② 类名

r5_steering_controller:        # ③ 参数段名（必须=①）
  ros__parameters: ...
```

- **① 实例名**：spawner 在 launch 里喊的名字（`arguments=["r5_steering_controller", ...]`）。
  它成为控制器的 **ROS 节点名**，所有话题挂它下面：
  `/r5_steering_controller/odometry`、`.../reference_unstamped`。
  control.xacro 里的 remap 写的正是这些带①前缀的名字。
- **② 类名**：告诉 controller_manager 去 pluginlib 查这个类。
  查询路径：② → `r5_steering_controller.xml`（`<class name="r5_steering_controller/R5SteeringController"
  type="r5_steering_controller::R5SteeringController" ...>`）→
  `<library path="r5_steering_controller">` → `install/r5_steering_controller/lib/libr5_steering_controller.so`
  → 加载 .so，实例化 C++ 类。
- **③ 参数段名**：controller_manager 加载时按节点名（=①）去 yaml 里找参数段。
  所以 **①和③必须一字不差**，②只和 xml 里的注册名对上即可。

改名换控制器时三处联动：spawner 参数（launch）、①+②（yaml）、
remap 前缀（control.xacro）。漏任何一处的故障模式见 STAGE1_GUIDE 第 2 节。

### 5.3 为什么既要 yaml 又要 cpp

- **cpp = 行为**（算法逻辑）：斜坡公式、限幅算法、四步流水线。
  编译进 .so，改它要 `colcon build`。
- **yaml = 参数**（行为的具体数值）：限幅值 0.8 还是 0.2、轮径 0.0325、
  关节叫什么名字。symlink-install 下改完**重启 launch 即生效**。
- 意义：同一套 .so 适配任何阿克曼车——换车只改 yaml/URDF，不改代码。
  这是"代码与配置分离"，也是 ROS 生态用 yaml 做参数的原因。

---

## 6. 指令与数据的完整旅程（带文件名）

### 启动序列（launch 干的事，按时间顺序）

1. `car_bringup/launch/gazebo.launch.py`：
   `xacro.process_file(car.urdf.xacro)` → 剥 XML 注释 → 存进
   `robot_description` 参数 → 启动 `robot_state_publisher`；
2. 启动 gzserver（读 `car_description/world/custom_room.world`）；
3. `spawn_entity.py` 把 URDF 塞给 Gazebo 生成车模；Gazebo 解析 URDF 时
   读到 control.xacro 里的 `<plugin filename="libgazebo_ros2_control.so">`
   → **在 gzserver 进程内**加载桥插件；
4. 桥读 `<ros2_control>` 标签 → 创建 GazeboSystem 硬件 +
   controller_manager 节点（从 `<parameters>` 指的 controllers.yaml 读
   update_rate=50）；
5. launch 里的两个 spawner 依次调 controller_manager 的服务：
   加载并激活 `joint_state_broadcaster` 和 `r5_steering_controller`
   （后者带着 `--param-file controllers.yaml`）。

### 运行时（每 20ms 一个循环，全在 gzserver 进程里）

```
【下行：指令】
/cmd_vel (Twist)                        ← path_tracker.py / teleop / Nav2 发布
  ↓ remap 后被订阅（订阅者在 steering_controllers_library.cpp on_configure 里创建）
input_ref_ 实时缓冲
  ↓ [50Hz] R5SteeringController::update_and_write_commands()   (r5_steering_controller.cpp)
父类 SteeringControllersLibrary::update_and_write_commands()
  (steering_controllers_library.cpp)
  ├─ 重载 reference_interfaces_ = (vx, wz)
  ├─ update_odometry() → SteeringOdometry::update_from_velocity()
  │    (steering_odometry.cpp)      ← FK：实测轮速+转角 → 里程计
  ├─ odometry_.get_commands()       ← IK：(vx,wz) → {右后ω, 左后ω, 右前δ, 左前δ}
  └─ 写 command_interfaces_[0..3]   ← 4 个"命名通道"的值
R5 后处理：读句柄 → 斜坡/限幅/限速率 → 写回句柄        (r5_steering_controller.cpp 步骤①②③)
  ↓ update() 返回
ResourceManager 收集所有句柄值
  ↓
GazeboSystem::write() (libgazebo_ros2_control.so)
  ↓ ODE：后轮关节 SetVelocity、前舵关节 SetPosition

【上行：状态】
物理引擎推进 → 关节真实位置/速度
GazeboSystem::read() → 填 state_interfaces_[0..11]
  ├→ R5/父类下周期读 → FK → 发布 /odom + /tf (odom→base_footprint)
  └→ joint_state_broadcaster 读 → 发布 /joint_states
```

**谁发/谁收/谁执行，一句话版**：L3 发意图（cmd_vel）；R5 控制器收意图、
发关节指令（command interfaces）；GazeboSystem 是快递员（送指令、取状态）；
物理引擎执行；state interfaces 把结果带回来给控制器算里程计。

---

## 7. "接口"的两种含义（你问"消息接口改没改"）

| 概念 | 是什么 | 我们改了吗 |
|---|---|---|
| **消息接口**（ROS 层） | 话题上跑的数据类型：`/cmd_vel` 用 geometry_msgs/Twist、`/odom` 用 nav_msgs/Odometry、`/joint_states` 用 sensor_msgs/JointState | **没改**。全是标准包的现成消息，没自定义任何 msg/srv |
| **硬件接口**（ros2_control 层） | 不是消息！是带名字的**内存通道**：`left_rear_wheel_joint/velocity`、`right_front_steer_joint/position`……声明在 control.xacro 的 `<ros2_control>` 里，由控制器的 `command_interface_configuration()`/`state_interface_configuration()` **申请**，ResourceManager 负责把"申请"和"声明"对上号（对不上 configure 直接失败） | **没加新类型**，只用标准的 position/velocity；数量和名字是我们自己定的 |

硬件接口和消息的区别：它们不经过网络/DDS，是同进程内共享内存的
实时通道（realtime 安全），这也是 ros2_control 能跑 1kHz 的原因。

---

## 8. 一页速查：改 X → 动哪个文件

| 想改什么 | 动哪个文件 | 要重编译吗 |
|---|---|---|
| 限幅值/超时/轮径/关节名 | `car_bringup/config/controllers.yaml` | 否（重启 launch） |
| 车的尺寸/轮距/质量/限位 | `car_description/urdf/common.xacro` / `wheels.xacro` | 否（symlink，重启） |
| 传感器/世界 | `car_description/urdf/car/*.xacro` / `world/` | 否 |
| 限幅算法本身 | `r5_steering_controller/src/r5_steering_controller.cpp` | **是** |
| 参数声明（新增 r5.xxx） | `r5_steering_controller.cpp` 的 on_init + controllers.yaml | **是** |
| 启动顺序/加载哪个控制器 | `car_bringup/launch/gazebo.launch.py` | 否 |
| 话题名/remap | `car_description/urdf/car/control.xacro` | 否 |
| 里程计/IK 公式 | `ros2_controllers/steering_controllers_library/src/steering_odometry.cpp` | **是** |
| 导航参数 | `car_bringup/config/nav2_params.yaml` | 否 |
