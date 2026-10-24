# 阿克曼小车控制调试 —— 深度复盘笔记

> 对应 2026-10-23 的控制链路修复。本文是"问题 → 根因 → 我错在哪 → 怎么改 → 验证 → 举一反三"的完整切割，比 README 更深一层。
>
> 阅读前提：三个分层视角。所有 bug 都可以归到其中一层，**分层定位是这次排查最快的方法论**：
>
> | 层 | 内容 | 出错时的典型症状 |
> |---|---|---|
> | A. 包/环境层 | 插件是否存在、overlay 是否 source | spawner 报 `Loader not found`，控制器列表里根本没有 |
> | B. 配置/接线层 | yaml 参数、关节名顺序、话题类型、remap | 控制器 active 但车不动 / 动得诡异、话题无订阅者 |
> | C. 物理/模型层 | 关节轴符号、摩擦、限位 | 车动但方向反、画龙、侧滑、打不出转角 |
>
> 一个反直觉的事实：**C 层症状（车画龙、抖动）完全可能由 B 层错误（左右轮接反）引起**。
> 这次就走过了这条弯路——先在物理层加阻尼、调摩擦"治抖"，后来才发现是接线反了。

---

## 问题 1：ackermann 控制器缺失（A 层）

### 现象
- 截图里 `ros2 control list_controllers` 只有 `joint_state_broadcaster`。
- 启动日志里 spawner 进程 exit code 1：
  ```
  [ERROR] [controller_manager]: Loader for controller 'ackermann_steering_controller'
  (type 'ackermann_steering_controller/AckermannSteeringController') not found.
  [INFO] [controller_manager]: Available classes:
    ... bicycle_steering_controller/BicycleSteeringController
    ... tricycle_steering_controller/TricycleSteeringController
    （26 个类里唯独没有 ackermann）
  ```

### 根因链
1. `controller_manager` 通过 **pluginlib** 按类名加载控制器；类描述来自各个包安装时注册的 plugin XML（经 ament index）。
2. 本机 `dpkg -l` 显示只装了 `ros-humble-ackermann-msgs` 和 `ros-humble-steering-controllers-library 2.53.1`，
   **没有** `ros-humble-ackermann-steering-controller`（bicycle/tricycle 装了，唯独漏了它）。
3. `src/ros2_controllers`（humble 分支，2.54.1）克隆了但从未编译：`colcon list` 认识它，
   `build/` 目录里却没有它的产物；当时的构建命令是
   `--packages-up-to car_bringup car_description car_navigation`，而 `car_bringup/package.xml`
   当时只 `<depend>controller_manager</depend>`，**构建图根本不会把 ackermann 拉进来**。
4. 于是 gzserver 里的 controller_manager 既在 `/opt` 找不到、也在 `install/` 找不到 → 类加载失败。

### 我错在哪
- **把"克隆了源码"当成"装好了"**。ROS2 的运行时查找看的是 ament index（`install/` 或 `/opt`），
  不是 `src/` 里有没有目录。
- 出问题后第一反应去查 yaml / URDF（B、C 层），没先读全错误日志。
  其实那条错误下面紧跟的 `Available classes` 列表**已经把答案写出来了**——列表里没有 ackermann，
  这不是配置错，是插件不存在。

### 怎么改
- 源码编译（后来还要打补丁，所以没用 apt）：
  ```bash
  colcon build --symlink-install --packages-select \
    steering_controllers_library ackermann_steering_controller
  ```
- 在 `car_bringup/package.xml` 加 `<depend>ackermann_steering_controller</depend>`，
  让 `--packages-up-to car_bringup` 以后自动带上它（colcon graph 从 3 包变 11 包）。

### 举一反三
- 快速判断一个包在当前环境是否存在：`ros2 pkg prefix <包名>`（"Package not found" 即不存在）。
- `Loader not found` + `Available classes` 是 pluginlib 的注册表转储，**先读列表再查配置**。
- 克隆了第三方源码后，确认 `build/`、`install/` 里真的有它，且终端 source 的是这份 `install/setup.bash`。
- 另一个小坑：重复启动仿真时，上一次的 `gzserver` 残留会让两次日志 interleaved，
  出现"新启动却报旧的 FATAL"。重启前 `pkill -9 -x gzserver` 一类清理是必要动作。

---

## 问题 2：左右轮顺序反了（B 层，最核心的一个）

### 现象（修复前推断 + 用户历史描述）
车画龙、不按指令转弯；此前用户通过给转向关节加 `damping=2.0 friction=0.5`
（wheels.xacro 里注释写着"防止转向轮自发抖动"）来压制症状。

### 根因链（源码级）
上游 `steering_odometry.cpp` 的 ACKERMANN 分支**硬编码"右轮在前"**：

```cpp
traction_commands = {Wr, Wl};          // [0]=右轮速度, [1]=左轮速度
steering_commands = {alpha_r, alpha_l}; // [0]=右轮转角, [1]=左轮转角
```

`steering_controllers_library.cpp` 按数组下标写入指令接口：

```cpp
for (i in rear_wheels_names)  command_interfaces_[i]        = traction_commands[i];
for (i in front_wheels_names) command_interfaces_[i + n_rear] = steering_commands[i];
```

也就是说：**`rear_wheels_names[0]` 拿到的是"右轮"的指令**。原配置写
`["left_rear_wheel_joint", "right_rear_wheel_joint"]`，于是：

| 数组位置 | 应该放 | 实际放了 | 后果 |
|---|---|---|---|
| `[0]` ← Wr / α_r | 右轮 | 左轮 | 左轮拿到右轮的指令 |
| `[1]` ← Wl / α_l | 左轮 | 右轮 | 右轮拿到左轮的指令 |

具体到一次左转（vx=0.1, wz=0.3，几何解 α_r=0.263 < α_l=0.419，Wr≈5.1 > Wl≈2.7 rad/s）：

- **转向**：左关节被命令 0.263（外侧角）、右关节被命令 0.419（内侧角）。
  两前轮转角交换后，各自延长线与后轴交于不同点——不存在公共瞬时转向中心，前轮互相较劲（轮胎刮擦）。
- **驱动**：右后轮拿到慢速 Wl、左后轮拿到快速 Wr。左转时右轮本该是**外侧快轮**，
  现在后轴的差速方向整个反过来，产生的偏航力矩**与前轮转向方向相反**——后轴在"拆台"。
- **里程计**：`update_from_velocity` 同样按下标读 STATE 接口，正反运动学一起错。

这就是"画龙/不转弯"的力学解释：不是缺 PID、不是阻尼不够，是**运动学接线反了**。
加 damping/friction 只是给错误的系统增加了耗散，症状从"剧烈画龙"变成"迟钝画龙"。

### 我错在哪
- 凭直觉按 `[left, right]`（从左往右读的惯性）写数组，没有核对控制器的**顺序契约**。
- 这个契约在 README 里不显眼，但它白纸黑字写在**控制器自己的测试参数**里：
  `test/ackermann_steering_controller_params.yaml`：
  ```yaml
  rear_wheels_names:  [rear_right_wheel_joint, rear_left_wheel_joint]
  front_wheels_names: [front_right_steering_joint, front_left_steering_joint]
  ```
  测试文件就是最可靠的接口文档，当时没有去看。

### 怎么改
```yaml
# controllers.yaml（改后）
rear_wheels_names:  ["right_rear_wheel_joint", "left_rear_wheel_joint"]
front_wheels_names: ["right_front_steer_joint", "left_front_steer_joint"]
```

### 验证（数字对得上几何解）
左转 vx=0.1、wz=0.3，理论 vs 实测：

| 量 | 理论 | 实测 |
|---|---|---|
| 虚拟中心转角 φ=atan(wz·L/vx) | 0.3245 rad | —（闭环用实测转角） |
| 右前轮转角 α_r | 0.265 rad | 0.2634 ✓ |
| 左前轮转角 α_l | 0.419 rad | 0.4192 ✓ |
| 右后轮 ω（外侧快） | ≈5.1 rad/s | 5.13 ✓ |
| 左后轮 ω（内侧慢） | ≈2.7 rad/s | 2.74 ✓ |
| yaw 变化 | 左转增大 | odom +0.544 / Gazebo +0.549 ✓ |

### 举一反三
- 接任何 ros2_control 控制器，先翻它的 `test/*_params.yaml`——那是对接契约。
- 顺序类错误**不会有任何报错**（名字都对，只是位置互换），唯一暴露方式是低速曲线实测：
  内侧轮转角必须大于外侧轮、外侧轮转速必须快于内侧轮。这两个不等式一验，接没接反立刻现形。
- 症状归因纪律：**运动学接线的错误，不要用物理层的补丁（阻尼/摩擦/PID）去治**。
  先验证指令到关节的映射，再谈参数整定。

---

## 问题 3：`use_stamped_vel` 与 remap 不匹配 —— /cmd_vel 静默失效（B 层）

### 现象
控制器 active，发布 `/cmd_vel` 车毫无反应，也没有任何报错。

### 根因链
`steering_controllers_library.cpp` 的 `on_configure`：

```cpp
if (params_.use_stamped_vel)
  ref_subscriber_twist_ = create_subscription<TwistStamped>("~/reference", ...);
else
  ref_subscriber_unstamped_ = create_subscription<Twist>("~/reference_unstamped", ...);
```

URDF 里的 remap 是：

```xml
<remapping>/ackermann_steering_controller/reference_unstamped:=/cmd_vel</remapping>
```

配置却是 `use_stamped_vel: true`。于是：

1. 控制器实际创建的订阅者是 `~/reference`（**TwistStamped** 类型）；
2. remap 规则针对的是 `reference_unstamped`——一个**根本没被创建的话题**，remap 空转；
3. `teleop_twist_keyboard` / Nav2 的 velocity_smoother 发布的是 `/cmd_vel`（**Twist** 类型），
   即使 remap 匹配上了，类型也不同——ROS2 里同名不同型的发布/订阅**互不相连，且不报错**。

三层静默：订阅者不存在 → remap 不命中 → 类型不匹配。每一层都不产生日志。

### 我错在哪
- 以为"remap 写了就生效"。**remap 只重命名节点真实创建的那个名字**；
  用 A 参数关掉订阅者、却 remap B 名字，等于什么都没做。
- 配了 `cmd_vel_timeout: 0.5` 想做超时停车——这个参数在本控制器里**不存在**
  （真正生效的叫 `reference_timeout`，写错的键会被静默忽略）。

### 怎么改
- `use_stamped_vel: false`，保留 `reference_unstamped:=/cmd_vel` 的 remap；
- `reference_timeout: 0.5`（0.5 s 收不到指令自动置零，比默认 1.0 更安全）。

### 验证
`ros2 topic list` 出现 `/cmd_vel`；`teleop_twist_keyboard` 与 `drive_test.py` 均能驱动车辆；
`reference_timeout` 生效表现为停发指令后 ~0.5 s 车停、转角回零。

### 举一反三
- 排查"发指令没反应"的标准三板斧：
  `ros2 topic info -v /cmd_vel`（看有没有订阅者）、
  `ros2 node info`（看控制器节点实际订阅了哪个话题）、
  `ros2 topic echo` 确认类型。
- 写参数前对照 `generate_parameter_library` 生成的参数声明
  （`src/*.yaml` 是参数的"字典"）：不存在的键不报错、不生效，是最阴的一类配置错误。

---

## 问题 4：后轮转轴符号反了（C 层）

### 现象
（与问题 2、5 修复前混在一起呈现为"方向混乱"。单独隔离后：正速度指令 → 车后退。）

### 根因链（完整的几何推导）
后轮关节定义：

```xml
<origin xyz="..." rpy="1.5707 0 0"/>   <!-- 子坐标系绕 x 转 +90° -->
<axis xyz="0 0 1"/>                    <!-- 轴在子坐标系里 -->
```

- 旋转矩阵 R_x(π/2) 把子系 z 轴映射到**世界 −y**：`R_x(π/2)·ẑ = −ŷ`。
  所以 `axis="0 0 1"` 的世界方向是 −ŷ。
- 无滑滚动约束：轮毂以 +x 前进时，接触点速度为零
  ⇒ `ω = +(v/r)·ŷ`（对 +ŷ 取正）。
  推导：`v_x·x̂ + ω ŷ × (−r·ẑ) = 0 ⇒ v_x − ωr = 0`。
- 结论：**要"+速度 = 前进"，关节轴的世界方向必须是 +ŷ**，
  即子系里写 `axis="0 0 -1"`。原来的 `0 0 1` 对应 −ŷ，正速度 = 倒车。

**git 考古**还发现更深一层：`git diff` 显示原代码（gazebo_ros_ackermann_drive 插件时代）是
`axis="0 0 ${-y_reflect}"`（左右镜像：左轮 −1、右轮 +1），迁移 ros2_control 时被改成了
`0 0 1`。两个版本都不对：

| 写法 | 左轮世界轴 | 右轮世界轴 | ros2_control 下行为 |
|---|---|---|---|
| `0 0 ${-y_reflect}`（原） | +ŷ ✓ | −ŷ ✗ | 直行指令 → 原地打转（一正一反） |
| `0 0 1`（迁移时改的） | −ŷ ✗ | −ŷ ✗ | 直行指令 → 倒车（一致但反了） |
| `0 0 -1`（修复后） | +ŷ ✓ | +ŷ ✓ | 直行指令 → 前进 ✓ |

镜像写法是旧 Gazebo 插件生态的惯例（插件内部对左右轮做了符号处理）；
**换驱动栈等于换了符号契约**，迁移时必须对每个被驱动关节重新推导轴的符号。

### 我错在哪
- 迁移时"顺手简化"了轴定义，没有重新推导符号——把上一套驱动的隐含约定带进了下一套。
- 前轮被动关节保留了镜像写法。虽然前轮不接收指令、不影响运动，
  但 joint_states 里左右前轮的速度符号会相反，看数据时容易被误导，一并统一成 `-1`。

### 验证
直行 +0.1 m/s、2 s：
- Gazebo 真值 Δx = 0.2005 m（期望 0.2）✓ 方向正确；
- /odom Δx = 0.1966 m（与真值差 ~2 mm/0.2 m，来自轮胎滑移与滚动窗口平滑）✓；
- 后轮关节速度 ≈ +4.05 rad/s = 0.1/0.0245 ✓（符号、幅值都对）。
倒车 −0.1 m/s、1.5 s：Gazebo 位移 0.147 m 沿反方向 ✓。

### 举一反三
- 验证轴符号不需要推公式，一条命令闭环：
  发 `linear.x=+0.05`，看 `gz model -m <car> -p` 的 x 是否增大、
  `/joint_states` 里驱动轮 velocity 是否为正。三处符号一致才算接对。
- URDF 里 `axis` 是**子坐标系**的量，`origin rpy` 会旋转它——先算 rpy 对轴的映射，再定符号。

---

## 问题 5：前轮侧向摩擦 mu2=0.1（C 层）

### 现象
（与其他问题叠加表现为）转向迟钝、车头"推头"画大圈。

### 根因链
Gazebo ODE 摩擦模型两个系数：
- `mu1`：第一摩擦方向（对轮子≈滚动/周向）；
- `mu2`：正交方向（≈侧向/滑移方向）。

前轮原来的 `mu2=0.1` 是从差速/履带教程里抄来的设定——那类底盘转弯时轮子**需要**侧滑，调低 mu2 是减少刮擦阻力。

阿克曼正相反：**前轮的侧向摩擦力就是提供向心力的东西**。转弯时前轮平面与车速方向夹 φ，
轮胎靠侧向附着力把车"拽"进弯。mu2=0.1 意味着侧向力很早就饱和 → 前轮横向打滑 → 严重不足转向。
配合问题 2（差速反向），整车表现就是"不转弯/乱漂"。

### 我错在哪
- 把摩擦参数当"视觉/手感"参数抄模板，没意识到**摩擦系数就是简化的轮胎模型**，
  不同底盘构型对 mu1/mu2 的要求有本质差别：
  - 差速/履带：轮子必须有侧向自由度（低 mu2 合理）；
  - 阿克曼：四轮都要求高侧向附着（mu1=mu2=1.0）。

### 验证
mu2 修复后，前轮实际转角能稳定跟住指令（0.2634/0.4192 vs 指令 0.265/0.419），
实测 yaw 速率 0.27 rad/s ≈ 指令 0.3（差值来自小轮胎+滚动窗口平滑），不再推头。

### 举一反三
- 症状指纹：**转角指令正确、关节也到位，但 yaw 速率远小于指令** → 查侧向摩擦，不是查控制器。
- 换底盘构型时，URDF 里所有"物理含义"标签（摩擦、阻尼、限位）都要按新构型重审，
  不能整体照搬旧车的。

---

## 问题 6：URDF 注释让 gazebo_ros2_control 崩溃（A/B 层之间，最隐蔽）

### 现象
gzserver 正常、车正常 spawn，但 controller_manager **从未创建**：
```
[ERROR] [gazebo_ros2_control]: parser error Couldn't parse parameter override rule:
'--param robot_description:=<?xml version="1.0" ?><!-- ... -->...'
```
随后 spawner 陆续超时：`Could not contact service /controller_manager/list_controllers`。

### 根因链（这个 bug 的传导路径很长，值得完整展开）
1. `robot_state_publisher` 把 URDF 存在参数 `robot_description` 里。
2. `gazebo_ros2_control` 插件通过 AsyncParametersClient 从 RSP 取回这段字符串。
3. 插件要把 URDF **塞给它在 gzserver 内部创建的 controller_manager 节点**。
   它的做法非常"土"：拼一条命令行参数
   `--param robot_description:=<整段URDF>`，
   连同 `--params-file controllers.yaml`、`--remap ...` 一起调用
   `rcl_parse_arguments()` 解析进 rcl 全局参数。
4. rcl 的参数覆盖规则用 YAML 风格的小语法解析 `name:=value`；
   URDF 文本（尤其 XML 注释）里出现 `冒号+空格`、URL 等字符时，这个迷你解析器报错。
5. 解析失败 → 插件直接 return → **controller_manager 没有诞生** →
   一切下游症状（spawner 超时、无 /odom、无 /cmd_vel 订阅）都是尸检结论，不是病因。

社区同类案例：gazebo_ros2_control#295、gz_ros2_control#503、launch_ros#214
（上游后来的修复就是在插件里用正则删掉 XML 注释再传参）。

**一个诚实的备注**：我用 `rcl_parse_arguments` 单独复现时，带注释的完整参数串竟然能解析通过，
说明插件内部真实走的解析路径与我的简化复现有差异（失败点没能缩到单个字符）。
但对照实验是成立的：同样的 URDF，只在 launch 里去掉注释后插件即刻成功。
工程结论不受影响——**注释里不要留 ASCII 冒号/URL，或干脆在 launch 层统一剥掉注释**（现行方案）。

### 我错在哪（双重）
- 第一层：我自己在 `control.xacro` 新加的注释里写了 `use_stamped_vel: false`（ASCII 冒号+空格），
  亲手引爆了这个雷——说明这个坑**极其容易在无意识中踩中**。
- 第二层：历史注释（中文全角"："安全，但 `prefix:部件名字前缀` 这类半角冒号一直存在），
  之前碰巧没炸是运气（旧注释里没有"冒号+空格"组合）。

### 怎么改
launch 里发布前统一剥注释（与上游后来的修复同思路）：

```python
robot_desc = doc.toxml()
robot_desc = re.sub(r'<!--[\s\S]*?-->', '', robot_desc)   # 去 XML 注释
```

### 举一反三
- 症状指纹：**gzserver 活着、车 spawn 成功、但 `/controller_manager` 节点不存在** →
  先 grep 插件日志里的 `parser error`，八成是 robot_description 里的特殊字符。
- `robot_description` 不只是给 RSP 用的字符串，它会被多个组件（gazebo_ros2_control、
  controller_manager）反复当命令行参数转手——把它当"代码"对待：注释也要讲卫生。
- 错误串里 URDF 被截断在 ~200 字符是 rcl 错误信息的截断，别误以为问题在前 200 字符里。

---

## 问题 7：`max_steering_angle` 是摆设 + 上游限位缺陷（源码层修复）

### 现象/风险
配置里写了 `max_steering_angle: 0.5`、`max_velocity: 1.0`、`max_wheel_velocity: 10.0`——
官方 2.54.1 的 ackermann 控制器**根本没有这些参数**（`generate_parameter_library`
只认 `src/ackermann_steering_controller.yaml` 里声明的键），多余键被静默忽略。

### 根因链（为什么这是真 bug，不只是参数无效）
失控场景：`vx=0, wz≠0`（纯旋转指令，Nav2 recovery、手柄误按都会发出）：

```
φ = atan(wz·L/vx) = atan(±∞) = ±π/2
α_l = atan2(2L·sinφ, 2L·cosφ − w·sinφ) = atan2(0.224, −0.164) ≈ 2.206 rad ≈ 126°
```

左前轮被命令转到 2.2 rad，而 URDF 机械限位是 ±0.5。后果链：
关节持续顶死在限位（力矩饱和、物理抖动）→ 里程计 FK 拿到顶死后的奇怪角度算位姿 →
指令语义与实际能力脱节。这是**上游的功能缺陷**：转向限位只存在于 URDF，
控制器计算指令时完全不知道。

### 怎么改（设计要点）
在 `steering_odometry.cpp` 的 `get_commands` 里加限幅，关键设计决策：

1. **限"虚拟中心角 φ"而不是逐轮角度**。因为后轮差速是由 φ 经转弯半径算的
   （`R = L/tanφ; Wr = Ws(R+w/2)/R`），只夹 α 不夹 φ 会导致速度按一个
   物理上达不到的半径计算——指令内部自相矛盾。
2. **单轮限位 → 虚拟角限位的换算**用内侧轮（先到限位的轮）：
   ```
   φ_max = atan( L·tan(δ_max) / (L + w/2·tan(δ_max)) )
         = atan(0.112·0.546 / (0.112 + 0.082·0.546)) ≈ 0.374 rad   (δ_max=0.5)
   ```
   等效最小转弯半径 R_min = L/tan(0.374) ≈ 0.285 m。
3. 逐轮 `clamp` 作为兜底（防数值越界），乘 0.999 防边界振荡。
4. 参数声明在 **ackermann 自己的 yaml** 里而不是基类——原因见问题 9（ABI）。

### 验证
纯旋转 `vx=0, wz=0.5`：
- 右前轮 0.294 rad、左前轮 0.499 rad（正好顶在限位内侧）✓
- 车轮不转（Ws = vx/r = 0）、车不漂移、odom 不动 ✓
对照理论：φ 被夹到 0.374 后 α_r=atan2(0.0818, 0.2684)=0.296、α_l=atan2(0.0818, 0.1486)=0.503→clamp 0.5，
与实测 0.294/0.499 完全吻合。
另外回归：正常左转数值不变（问题 2 的表格）。

### 举一反三
- "参数写了但车行为没变"→ 先确认参数真的被声明了：
  `ros2 param get /ackermann_steering_controller max_steering_angle` 必须能取到值。
- 阿克曼车的指挥系统必须知道自己的机械限位；`vx=0 && wz≠0` 这类物理上不可达的指令
  要在控制器层被"翻译"成可达的最接近指令（打满轮、不驱动），而不是送进运动学里发散。

---

## 问题 8：里程计协方差 bug（源码层修复）

### 根因链
上游 `steering_controllers_library.cpp`：

```cpp
auto & covariance = msg_.twist.covariance;      // ← 只取了 twist 的数组
for (index = 0..5) {
  covariance[diagonal_index] = params_.pose_covariance_diagonal[index];
  covariance[diagonal_index] = params_.twist_covariance_diagonal[index];  // ← 立刻覆盖上一行
}
```

一箭双雕的 bug：
1. pose 协方差被写进 **twist** 的数组，`pose.covariance` 永远全零；
2. 第二行赋值覆盖第一行，pose 的值在任何地方都没留下。

下游影响：AMCL/robot_localization 融合时把"全零协方差"理解为"完全可信的完美观测"，
或者干脆使滤波器权重失衡——静默地毁掉定位质量。

顺带一个上游彩蛋：默认值 `[0.0, 7.0, 14.0, 21.0, 28.0, 35.0]` 看形状就是
把**协方差矩阵的对角下标**（0,7,14,21,28,35）当成了数值抄进去——原作者自己也没逃过复制粘贴。

### 怎么改
```cpp
auto & pose_covariance  = msg_.pose.covariance;
auto & twist_covariance = msg_.twist.covariance;
for (...) {
  pose_covariance[diagonal_index]  = params_.pose_covariance_diagonal[index];
  twist_covariance[diagonal_index] = params_.twist_covariance_diagonal[index];
}
```
默认值改为量纲合理的小量：`[1e-4, 1e-4, 1e-6, 1e-6, 1e-6, 1e-3]`。

### 验证
`ros2 topic echo /odom --field pose.covariance` 对角线非零、
twist 协方差与 pose 协方差各归各位。

### 举一反三
- review 代码时对"循环内对同一变量连续赋值两次"保持敏感——几乎必是复制粘贴残留。
- 里程计消息不是只给自己看的，`covariance` 是给定位栈的契约字段，
  全零协方差是融合系统的慢性毒药。

---

## 问题 9：ABI 兼容陷阱（修改 ros2_control 源码时的隐藏战场）

这一条不属于原始 bug，是**修复过程中踩出来的**，对"改上游源码"极有参考价值。

### 现象
给 `SteeringOdometry` 加了两个成员（`steering_wheel_track_`、`max_steering_angle_`）后，
`ackermann_steering_controller` 的两个单元测试**段错误**（SIGSEGV），其余 36 个测试全过。

### 根因链（层层剥开）
1. 本机 `/opt` 装着 **steering_controllers_library 2.53.1（二进制）**，
   工作区源码是 **2.54.1（overlay）**。colcon 构建时警告过 "overriding package"。
2. 编译 ackermann 测试时，头文件查找路径混了两套：
   - `steering_odometry.hpp` 命中**源码**（含新成员，类布局变大）；
   - `steering_controllers_library_parameters.hpp` 命中 **/opt 的旧版**（`Params` 少一个字段，结构体更小）。
   原因很细：deb 包把头平铺在 `/opt/ros/humble/include/` 下，
   而 overlay 的导出路径是 `install/.../include/steering_controllers_library`，
   两种布局对 `#include <steering_controllers_library/xxx.hpp>` 的命中顺序不同。
3. 测试二进制按旧 `Params` 布局分配内存、库按新布局写入 → 越界写 → 堆损坏 →
   在第一个 `ElementsAreArray`（比较 vector<string>）处崩掉。
   **崩溃点离案发现场很远**，backtrace 全是 gmock 内部帧，极具迷惑性。

### 怎么改（两道保险）
1. **把新参数挪到 ackermann 自己的 yaml**，基类 `Params` 字段顺序与 /opt 2.53.1
   完全一致（逐字段 diff 验证过）→ 即便头文件混用，布局也兼容，不再越界。
2. `ackermann_steering_controller/CMakeLists.txt` 里强制 overlay 头文件优先：
   ```cmake
   get_target_property(STEERING_CONTROLLERS_LIBRARY_INCLUDE_DIRS
     steering_controllers_library::steering_controllers_library
     INTERFACE_INCLUDE_DIRECTORIES)
   include_directories(BEFORE ${STEERING_CONTROLLERS_LIBRARY_INCLUDE_DIRS})
   ```
3. 限幅值不作为类成员存储，改为 `get_commands(...)` 的**新增重载参数**逐次传入
   ——基类零布局变更，与 /opt 里预编译的 bicycle/tricycle 控制器保持二进制兼容。

### 验证
38 个测试全部通过（含此前崩溃的 2 个）；仿真行为（限幅、直行、左转）回归不变。

### 举一反三
- **在 overlay 里改"被 /opt 预编译包依赖"的公共基类 = ABI 地雷**。
  规则：要么改子类/新增重载（零布局变更），要么接受全量重编所有相关包。
- "少数测试段错误、其余全过"且崩溃点在 STL 容器比较里 → 第一反应查**头文件混用/布局不一致**，
  而不是查测试逻辑。`.d` 依赖文件（`grep -o "[^ ]*\.hpp" *.o.d`）能直接暴露头来自哪套。
- colcon 那句 "overriding ... must be API and ABI compatible" 警告不是废话，这次全部应验。

---

## 其余修改的为什么（简剖）

| 修改 | 原因 |
|---|---|
| launch 补上 `GAZEBO_MODEL_PATH` | 原文件**定义了**这个变量却没放进 `LaunchDescription`（死代码）。当前 world 自包含（房间模型内联）所以侥幸能跑；一旦 world 用 `model://` 就会现原形。已加入返回列表并指向 `world/` 目录。 |
| `gui`/`rviz` launch 开关 | 无头测试刚需：服务器/CI 里起 GUI 会拖垮启动甚至失败。`gui:=false rviz:=false` 一条命令进纯仿真。 |
| spawn z: 0.05 → 0.02 | 轮半径 0.0245，出生高度 0.05 意味着 2.5 cm 自由落体——落地冲击会给接触/里程计带来初始瞬态。降到 0.02（略高于贴地，防初始穿插）。 |
| RViz Fixed Frame: base_footprint → odom | 固定在 base_footprint 时机器人永远"不动"、世界在转，肉眼无法判断车是否真的在走。改 odom 后一眼看到轨迹。 |
| `car_description/package.xml` 删掉 rosidl_default_generator/runtime | 该包不定义任何消息接口，这两条依赖纯属误加，只会污染依赖图。 |
| `car_bringup/package.xml` 补 `<depend>ackermann_steering_controller</depend>` 等 | 让 `--packages-up-to car_bringup` 的构建图自动包含控制器（问题 1 的长效修复）；同时补 gazebo_ros2_control、rviz2、nav2_bringup、消息包，杜绝"换台机器少依赖"。 |
| 4 个包加 `COLCON_IGNORE` | FAST_LIO / FAST-LIO-SAM-QN / livox_ros_driver2：3D 雷达方案已弃，留着只会拖慢构建、拉进重依赖；gz_ros2_control：面向**新 Gazebo（gz-sim）**，本项目用 Gazebo Classic 11，编译它纯属浪费。要用时删掉对应文件即可。 |
| 新增 `scripts/drive_test.py` | 把本次排查的"验证动作"固化成一等公民：自动发指令、采 odom/joint_states、输出方向判定（✅/❌）。以后改任何控制相关的东西，跑一遍就知道有没有退步。 |

---

## 参数语义速查（当前 controllers.yaml 里每个关键参数为什么是这个值）

| 参数 | 值 | 理由 |
|---|---|---|
| `rear_wheels_names` | `[right, left]` | 控制器硬编码右前左后顺序（问题 2） |
| `front_wheels_names` | `[right_steer, left_steer]` | 同上；且这里填的是**转向关节**，不是前轮滚动关节——滚动关节只读状态 |
| `open_loop` | false | 用实测轮速+转角算里程计；与 Gazebo 真值差 ~1.5%（0.1966 vs 0.2005 m） |
| `position_feedback` | false | 后轮反馈的是 velocity 接口 |
| `use_stamped_vel` | false | 匹配 teleop/Nav2 的 Twist（问题 3） |
| `reference_timeout` | 0.5 | 断流 0.5 s 自动停车，比默认 1.0 保守 |
| `max_steering_angle` | 0.5 | 单前轮机械限位，控制器换算成虚拟角 ±0.374 rad（问题 7） |
| `update_rate` | 50 | 低于 Gazebo 1000 Hz 物理步长（日志有 warning 但无害）；再高收益趋零 |
| （不写 `max_velocity` 等） | — | 控制器无此参数；速度上限由 URDF 轮关节 `velocity=10 rad/s ≈ 0.245 m/s` 决定 |

---

## 诊断决策树（下次从症状出发）

```
spawner 报 Loader not found
 └→ A 层：ros2 pkg prefix 查包在不在；读 Available classes 列表
controller active 但 /cmd_vel 无反应
 └→ B 层：ros2 topic info -v /cmd_vel 看订阅者；ros2 node info 看实际订阅话题名与类型
gzserver 正常、车 spawn 成功，但 controller_manager 不存在
 └→ A/B 层：grep "parser error"——robot_description 里的注释/特殊字符
车动但方向反
 └→ C 层：三处对符号（指令 / joint_states / gz model 真值），推 rpy→axis 映射
车画龙 / 不转弯 / 推头
 └→ B+C 层：先验左右顺序（内外轮不等式），再查前轮 mu2、阻尼
"参数改了没反应"
 └→ B 层：ros2 param get 确认参数真实存在；对照上游 src/*.yaml 的声明
改了上游源码后诡异的段错误
 └→ ABI：头文件是否混用（查 .d 文件）；公共基类是否改了布局
```

---

## 一句话总结每个教训

1. **包不存在就是不存在**——错误日志的 `Available classes` 列表里已经有答案。
2. **顺序是契约**——测试文件即文档；接线错误永远不要用物理补丁去治。
3. **remap 只作用于真实存在的名字**——三层静默失效不产生任何日志，必须主动探测。
4. **axis 在子坐标系里**——rpy 会转走它；换驱动栈等于换符号契约，逐关节重推导。
5. **摩擦就是轮胎模型**——构型一换，mu1/mu2 的物理要求整个反过来。
6. **robot_description 会被当命令行参数转手**——注释里的半角冒号是雷，launch 层剥掉最稳。
7. **不可达指令要在控制器层翻译**——限虚拟角保证速度与转角自洽。
8. **协方差是给定位栈的契约**——全零协方差是慢性毒药；"循环里连赋两次值"必是 bug。
9. **overlay 改基类 = ABI 地雷**——优先加子类参数/重载，保持公共布局不变。
