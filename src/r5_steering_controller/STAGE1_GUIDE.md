# R5 阶段一实战指南：控制器插件从接入到验收

> 配套代码：`src/r5_steering_controller/`（本包）、`src/car_bringup/config/controllers.yaml`、
> `src/car_bringup/scripts/acceptance_test.py`。
> 本文回答四个问题：**代码在干什么？为什么这样干？怎么改？改了会发生什么？**
> 所有结论都有实测数据背书（2026-10-24，本机 Humble + Gazebo Classic 11）。

---

## 0. 读完后你应能做到

1. 画出一条 `/cmd_vel` 从发布到轮子转动的**完整数据流**，并说出每一站的处理者；
2. 解释为什么三个限幅功能放在**命令层后处理**而不是参考层（本文最重要的教训）；
3. 独立修改任何一个 `r5.*` 参数，并在跑测试**之前**预测验收数据会怎么变；
4. 看懂验收报告里每个数字为什么是这个值（包括为什么略超限值仍算 PASS）。

---

## 1. 全景：一条 cmd_vel 的旅程

```
/cmd_vel (geometry_msgs/Twist, 20Hz, teleop/Nav2 发布)
   │
   ▼ ① 订阅回调写 RealtimeBuffer input_ref_
┌────────────────────────────────────────────────────────────┐
│ controller_manager 50Hz update 循环（gzserver 进程内）        │
│                                                            │
│ R5SteeringController::update_and_write_commands()          │
│   ├─ 父类开头：非链式模式【无条件】从 input_ref_ 重载          │
│   │   reference_interfaces_ = (vx, wz)                      │
│   ├─ update_odometry：FK——由实测轮速+实测转角反推 (v,ω)，     │
│   │   积分出位姿 → 发布 /odom 和 /tf (odom→base_footprint)   │
│   ├─ get_commands：IK——(v,ω) → {Wr,Wl 后轮速, αr,αl 前轮角}  │
│   │   （含父类的 max_steering_angle 虚拟中心角限幅）           │
│   ├─ 写 command_interfaces_（此刻父类的工作结束）              │
│   └─ ★ R5 命令层后处理（本包的全部价值所在）★                 │
│       ① 驱动轮速度斜坡（加速度限幅）                          │
│       ② 转向幅度帽（速度相关 + 机械限位）                     │
│       ③ 转向速率帽（舵机响应）                               │
└────────────────────────────────────────────────────────────┘
   │
   ▼ ② update 返回后，resource_manager 读取 command_interfaces_
GazeboSystem 插件 → ODE 关节：轮 SetVelocity / 舵 SetPosition
   │
   ▼ ③ 物理引擎推进世界
joint_states → state_interfaces → 回到 FK（闭环）
```

**记忆锚点**：父类负责"几何正确"（IK/FK），R5 层负责"物理可执行"
（限加速度/限转角/限转速）。两层各司其职，谁也不越界。

---

## 2. 三处接入的深度解析

把 `ackermann_steering_controller` 换成 `r5_steering_controller` 动了三个文件。
**每一处都必须改，且各自对应一种"漏改故障"**——这三种故障在之前的调试中全部真实踩过。

### 2.1 `controllers.yaml`：类型键 + 参数段改名

```yaml
controller_manager:
  ros__parameters:
    r5_steering_controller:
      type: r5_steering_controller/R5SteeringController   # ← 类型键：pluginlib 类名

r5_steering_controller:        # ← 参数段名 = 控制器节点名（必须与上面一致）
  ros__parameters:
    wheelbase: 0.112           # 父类 ackermann 的参数……
    ...
    r5.max_accel: 0.8          # ……加 R5 专属参数
```

- **类型键为什么重要**：`controller_manager` 靠它去 pluginlib 查类。
  写错/漏写 → `Loader for controller ... not found`（排查法：看报错下面的
  `Available classes` 列表里有没有你的类名）。
- **参数段为什么跟着改名**：控制器的 ROS 节点名 = 控制器名。
  参数段名必须等于节点名，否则参数全都加载不到（静默，无报错）。
- **`front_wheels_radius: 0.0325` 的联动**：轮径改了，这里必须同步——
  它直接乘进里程计和轮速指令。**半径差 30%（49→65mm）等于里程计虚报 30%**。

### 2.2 `gazebo.launch.py`：spawner 的控制器名

```python
arguments=["r5_steering_controller", "--controller-manager-timeout", "60",
           "--param-file", ".../controllers.yaml", "--activate"]
```

漏改的故障：spawner 去加载旧名，旧控制器又没有对应类型键 → 加载失败 →
车完全不动但仿真看起来正常。**spawner 名、类型键、参数段名三者必须同串**。

### 2.3 `control.xacro`：remap 三连

```xml
<remapping>/r5_steering_controller/tf_odometry:=/tf</remapping>
<remapping>/r5_steering_controller/odometry:=/odom</remapping>
<remapping>/r5_steering_controller/reference_unstamped:=/cmd_vel</remapping>
```

漏改的故障是**三层静默**（都曾在历史上真实发生）：
1. `/odom`、`/tf` 消失（话题还在旧名下）→ Nav2/RViz 找不到定位；
2. `/cmd_vel` 没有订阅者 → 发指令车不动，`ros2 topic info /cmd_vel`
   会显示 `Subscription count: 0`——**这就是唯一的线索**；
3. remap 不命中不会报任何错。ROS2 的教训：**凡是"应该存在却没有"的东西，
   先查话题图（`ros2 node info` / `ros2 topic info -v`），再查代码**。

---

## 3. 代码逐段精讲

### 3.1 类结构与生命周期

```
ControllerInterfaceBase
 └─ ChainableControllerInterface          ← 提供 reference_interfaces_ 机制
     └─ SteeringControllersLibrary        ← 运动学 + 接口管理（我们改过它的源码）
         └─ AckermannSteeringController    ← 只做一件事：把 ACKERMANN_CONFIG 传给里程计
             └─ R5SteeringController       ← 本包：命令层后处理
```

生命周期时序（每个函数被谁、什么时候调用）：

| 阶段 | 回调 | 我们做什么 |
|---|---|---|
| load | `on_init()` | 调父类 on_init → 声明并读 `r5.*` 参数 → 合法性检查 |
| configure | `on_configure()`（父类实现） | 创建订阅/发布、配置里程计、锁定接口列表 |
| activate | `on_activate()`（父类实现） | 清参考、接接口 |
| **每周期 50Hz** | `update_and_write_commands()` | 见 3.3 |

**为什么用继承而不是复制改**：父类带 38 个单元测试和已验证的运动学；
复制改意味着这份测试资产全部作废。继承只叠加差异，测试资产继续生效。

### 3.2 参数读取与"ParameterAlreadyDeclared 炸掉 gzserver"事件

第一版代码直白地写：

```cpp
node->declare_parameter<double>("r5.max_accel", max_accel_);  // ❌ 会炸
```

启动即崩，日志：

```
terminate called after throwing an instance of
'rclcpp::exceptions::ParameterAlreadyDeclaredException'
  what(): parameter 'r5.max_accel' has already been declared
[ERROR] [gzserver-1]: process has died [exit code -6]
```

**破案链**：
1. 本机 controller_manager 2.54.3 在加载控制器时，把 YAML 参数段里的键
   **预先声明**到了控制器节点上；
2. 我们再 `declare_parameter` 同名参数 → 抛异常；
3. 异常发生在 pluginlib `createSharedInstance` 的调用栈里，一路穿出
   gazebo 插件边界没有任何 catch → `std::terminate` → **整个 gzserver
   abort**（不是"这个控制器加载失败"，是仿真全死）。

**修复**（标准防御式写法，先查再声明）：

```cpp
auto ensure_double = [node](const std::string & name, double fallback) -> double
{
  if (!node->has_parameter(name))
    node->declare_parameter<double>(name, fallback);
  return node->get_parameter(name).as_double();
};
```

**两条通用教训**：
- 在 controller_manager 管理的节点里声明参数，永远用 `has_parameter` 守卫；
- 插件构造/init 路径上的异常会**杀死宿主进程**（gzserver），所以 init 里
  对一切可能失败的操作（参数、文件、接口）都要显式处理并返回 ERROR，
  让生命周期状态机优雅拒绝，而不是让异常逃逸。

另外注意：`r5.*` 参数在 `on_init` **读一次存成员**，之后改 YAML 不重启
不会生效——这是"配置快照"模式（父类参数同理）。想运行时改要加
`on_set_parameter` 回调（留作练习）。

### 3.3 `update_and_write_commands`：四步流水线

#### 为什么全部在命令层（本包最重要的设计决策，用事故说明）

第一版把加速度限幅写在**参考层**：调用父类之前先夹
`reference_interfaces_[0]`。验收测试当场揭穿：峰值加速度 2.48 m/s²，
限幅形同虚设。原因是父类的数据流：

| 时刻（一个 20ms 周期内） | reference_interfaces_[0] 的值 | 谁写的 |
|---|---|---|
| update 进入时 | NaN | 父类上一周期**结尾**清的 |
| 父类开头重载后 | 订阅者最新 vx（未限幅） | 父类 |
| （旧方案）我们限幅后 | 限幅值 | 子类 |
| 调用父类后父类再次重载 | **又变回未限幅值** ← 覆盖 | 父类 |
| update 结尾 | NaN | 父类 |

结论：**非链式模式下父类对 reference_interfaces_ 拥有完全的写权
（开头重载 + 结尾清 NaN），子类在它之前写的一切都会被覆盖。**
而命令接口的生命周期完全不同：父类写 → update 返回 → resource_manager
读 → 硬件。所以 update 线程内"读出→限幅→写回"确定生效。

> 这就是"先写测试再写功能"的价值：如果验收测试后写，这个 bug 可能
> 一直潜伏——限幅"看起来存在"，车"看起来在跑"。

#### ① 驱动轮速度斜坡

```cpp
const double r_wheel = std::max(ackermann_params_.rear_wheels_radius, 1e-4);
const double dw_acc = max_accel_ / r_wheel * dt;   // 底盘 a → 轮角加速度
const double dw_dec = max_decel_ / r_wheel * dt;
for (size_t i = 0; i < n_rear; ++i) {
  double raw = command_interfaces_[i].get_value();
  raw = std::clamp(raw, last_traction_cmds_[i] - dw_dec, last_traction_cmds_[i] + dw_acc);
  command_interfaces_[i].set_value(raw);
  last_traction_cmds_[i] = raw;
}
```

- **换算公式**：v = ω·r ⇒ dω/dt = a / r（不是 a×r！第一版写反，
  斜坡慢了 r² ≈ 947 倍，车几乎不动——又是验收测试抓的）。
  **量纲注释写在每个换算行旁边**，这不是可读性装饰，是防错栏。
- **为什么按轮限而不是按底盘限**：直行时左右轮同速，两者完全等价；
  转弯进入的瞬态按轮独立整形，恰好和实车一致——实车就是每个电机
  一个独立速度环。代价是瞬态内外轮比例略有偏差（见 6 节数据解读）。
- **超时停车也变"物理"了**：父类超时把牵引指令写 0，我们的斜坡把它
  变成以 max_decel 减速的刹车曲线，而不是瞬间归零——对 AT8236 更友好。

#### ② 速度相关转向幅度帽

```cpp
double phi_cap = max_steer_mech_;                       // 机械限位兜底
if (speed_dependent_limit_ && v_cmd > 0.05) {
  phi_cap = std::min(phi_cap, std::atan(a_lat_max_ * L / (v_cmd * v_cmd)));
}
```

- **公式**：a_y = v²·tanφ/L ≤ a_lat_max ⇒ tanφ_cap = a_lat_max·L/v²。
  数值直觉（L=0.112, a_lat=2.0）：

  | v (m/s) | φ_cap (rad) | 等效转弯半径 R=L/tanφ | 绑定约束 |
  |---|---|---|---|
  | 0.2 | 0.235（封顶） | 0.55 | 机械 |
  | 0.5 | 0.235（封顶） | 0.55 | 机械 |
  | 1.0 | 0.220 | 0.59 | **a_lat** |
  | 1.2 | 0.155 | 0.84 | **a_lat** |

- **v_cmd 用限幅后的指令车速**（`mean(轮指令)×r`）而不是参考车速：
  限幅器的输入必须是"实际会被执行的量"，用未限幅参考等于自欺欺人。
- **近似声明**：公式是自行车模型的虚拟中心角，直接作用在轮角上偏保守
  （真实内轮角对同一半径更大 ~17%）。保护性限幅宁严勿松，可接受；
  几何精确性由父类 `max_steering_angle`（已做虚拟角换算）保证。
- **为什么 >0.05 才启用**：v→0 时 atan(∞) 发散，不设阈值则该项永远
  不绑定，行为一样；设阈值是**表达意图**（低速不受 a_lat 管）。

#### ③ 转向速率帽

```cpp
const double max_delta = max_steer_rate_ * dt;
raw = std::clamp(raw, last_steer_cmds_[i] - max_delta, last_steer_cmds_[i] + max_delta);
```

- 9.5 rad/s 来自舵机手册（0.11s/60°）。默认下几乎不约束（0.235 rad 的
  满舵 25ms 打完），它的意义是**给实车留一个真实的动态模型**：
  调小后仿真轨迹的转向瞬态与实车一致（见 6 节 1.5 实测）。
- 顺序：先夹幅度（②）再夹速率（③），最后存基准。

#### 状态初始化与复位

- 首周期从 **state_interfaces_ 反馈**初始化斜坡基准（而不是 0）——
  如果控制器在车运动中被 reload，从 0 开始夹会把指令钳在一周期步长内，
  车会突然失去驱动；从反馈出发则无扰接管。
- NaN 命令（deactivate 等）：跳过并把基准重置为反馈，防止旧基准污染。

---

## 4. 踩坑实录：三个真 bug 的完整破案过程

这三段是本文最有价值的部分——**每个 bug 都是被验收测试或启动日志
当场抓获的，破案路径可复用**。

### 案 1：参考层限幅被父类覆盖
- **现象**：验收测试 1 报 `峰值 2.484 <= 限值 0.80 FAIL`。
- **假设**：限幅没跑？参数没读进去？
- **验证**：读父类源码确认 reference_interfaces_ 的"开头重载+结尾清 NaN"
  生命周期（3.3 节表格）；另两个测试的 PASS 恰好是父类自身的
  max_steering_angle 在兜底，掩盖了部分症状。
- **修复**：全部功能重构到命令层后处理。
- **方法论**：交叉引用（功能 A 失效但 B 正常）往往说明"你以为在控制的
  变量"和"实际被读的变量"不是同一个。

### 案 2：量纲写反
- **现象**：修复后峰值加速度 0.005，车几乎不动（稳态 vx 0.003）。
- **验证**：手算 dw_acc = 0.8 × 0.0325 × 0.02 = 0.00052 rad/s/周期，
  到 15.4 rad/s 需要 591 秒——数值本身就在喊"斜率错了"。
- **根因**：dω/dt = a/**r**，写成 a×r，差 r² = 947 倍。
- **修复**：`/ r_wheel`，并在注释里写量纲。
- **方法论**：**一切物理换算必须带单位注释 + 幅值 sanity check**。
  符号对了幅值错 1000 倍的 bug，只有测试能抓。

### 案 3：被测环境本身坏了（假故障）
- **现象**：vx=1.0 验收双 FAIL，vx 曲线在 0.3~0.7 间振荡，dv/dt 出现 ±5。
- **验证**：`gz model -m ackermann_car -p` → 车在 (-1.26, 1.01) yaw −0.79
  ——之前几轮测试把它开进了房间家具堆，顶住墙轮子打滑。
- **修复**：`gz model -m ackermann_car -x 0 -y 0 -z 0.05 -R 0 -P 0 -Y 0`
  复位后复测全 PASS。
- **方法论**：**被测对象的状态是实验变量**。连续实验之间必须复位
  （位姿/速度/里程计），否则你在测"上一次实验的残留"。

---

## 5. 修改预测表（改之前先写预测，跑完对答案）

> 这是训练"对代码行为做出预测"的核心练习。每条先自己预测，再跑
> `acceptance_test.py` 对答案。

| # | 修改 | 预测 | 原理 |
|---|---|---|---|
| 1 | `r5.max_accel` 0.8→0.2 | 测试1峰值≈0.2~0.3；0→0.5m/s 用时 0.63s→2.5s | 斜坡斜率=dw/dt，与 dt 无关总时长=a⁻¹ |
| 2 | `r5.max_decel` 1.5→0.5 | 超时刹车距离 0.083→0.25m；测试1不受影响（它只测加速段） | 刹车距离 v²/2a |
| 3 | `r5.a_lat_max` 2.0→1.0 | vx=1.0 时 φ_cap 0.220→0.112，ω_cap 2.00→1.00；vx=0.5 不变（机械限位仍在绑定） | atan(a·L/v²) 随 a 线性缩 |
| 4 | `r5.speed_dependent_limit` false | vx=1.0 大 ω 不再被压到 2.0，只剩机械限（ω 上限 2.14）；高速急转侧滑风险↑ | 跳过 min 分支 |
| 5 | `r5.max_steer_mech` 0.235→0.3 | **行为不变**（URDF 限位 0.235 先顶死），但暴露"三处限位不一致"的隐患 | 约束链取最严者 |
| 6 | `r5.max_steer_rate` 9.5→1.5 | 大转角阶跃 0.235rad 需 0.16s 到位；实测峰值 \|dφ/dt\| ≈1.58；转向瞬态车先直行后弯（外甩） | **已实测验证** |
| 7 | `update_rate` 50→20 | 每周期斜坡步长 ×2.5（dt 变大），阶梯更粗；测得峰值加速度抖动更大 | dt 出现在每个 clamp |
| 8 | `use_stamped_vel` true | /cmd_vel 突然无人订阅，车不动，零报错 | 订阅者换成 TwistStamped（老教训） |
| 9 | 关节顺序换回 [left,right] | 画龙回归；测试2 的稳态 ω 崩掉 | 老教训（内外轮互换） |
| 10 | `reference_timeout` 0.5→0 | 车停走停走几乎不动 | ref_timeout==0 时每周期清参考 |
| 11 | 删掉三个功能（只剩父类） | 得到"基线"：阶跃直通，acc 峰值≈∞（被 Gazebo 平滑后仍 >2） | 对照实验 |
| 12 | `r5.max_accel` 运行时 `ros2 param set` | **无效果** | 参数在 on_init 快照，改 YAML 需重启仿真 |

---

## 6. 验收测试：真实数据与解读

运行方式：

```bash
ros2 launch car_bringup gazebo.launch.py gui:=false rviz:=false   # 等 active
ros2 run car_bringup acceptance_test.py --vx 0.5 --wz-big 3.0 \
    --max-accel 0.8 --max-steer-rate 9.5 --a-lat-max 2.0 \
    --max-steer-mech 0.235 --wheelbase 0.112
```

### 三组实测（2026-10-24）

| 场景 | 测试1 峰值加速度 | 测试2 稳态ω / 理论上限 | 测试3 峰值\|dφ/dt\| | 结果 |
|---|---|---|---|---|
| vx=0.5, rate=9.5 | 0.889 / 0.80 | 0.904 / 1.069（机械绑定） | 9.63 / 9.5 | **PASS** |
| vx=1.0, rate=9.5 | 0.953 / 0.80 | 1.754 / **2.000（a_lat 绑定）** | 8.61 / 9.5 | **PASS** |
| vx=0.5, rate=1.5 | 0.941 / 0.80 | 0.898 / 1.069 | **1.577 / 1.5** | **PASS** |

### 每个数字为什么是这个值（学会解读比看到 PASS 更重要）

- **0.889 > 0.80 为什么算 PASS**：测的是 `/odom` 的 vx（FK 由轮速+滚动
  均窗平滑 + 20Hz 采样差分），含测量噪声与轮地耦合动态；限幅器限的是
  **指令**。判据留 25% 余量就是给这条"指令↔测量"链路的传递误差。
  若想验证指令本身，直接 echo command_interfaces（`ros2 control
  list_hardware_interfaces` 看不到值，需 controller_state 话题）。
- **0.904 < 1.069 为什么不满**：稳态转弯时轮胎刮地（scrub）+ 转弯阻力
  让实际车速低于指令车速，ω=v·tanφ/L 随 v 一起缩水。**这是物理，不是 bug**
  ——真车同样如此（所以真车跑轨迹要靠反馈控制而非开环速度指令）。
- **1.754/2.000 vs 0.904/1.069 的对比是 a_lat 功能生效的直接证据**：
  vx=0.5 时 `min(机械, a_lat)` 取机械（上限 1.069）；vx=1.0 时切到
  a_lat（上限 2.000）。同一参数在两个速度下换了绑定约束——这就是
  "速度相关"三个字的实测含义。
- **9.63 / 1.577 vs 各自限值**：命令层每周期台阶 = rate×dt（9.5→0.19 rad，
  1.5→0.03 rad），关节跟踪 + 采样差分让测量值围绕限值±5% 抖动。

### 验收判据的哲学

限幅器的验收标准不是"测量值 ≤ 限值"，而是
**"测量值 ≤ 限值 × (1+传递误差)"且"明显低于未限幅基线"**。
前者防误报，后者防"限幅器在但没用"（案 1 就是后者）。

---

## 7. 排错速查表

| 症状 | 第一动作 | 对应章节 |
|---|---|---|
| gzserver 整个 abort + `ParameterAlreadyDeclaredException` | 参数声明加 `has_parameter` 守卫 | 3.2 |
| `Loader for controller ... not found` | 读 Available classes 列表；查类型键/包是否编译 | 2.1 |
| 车不动、无报错 | `ros2 topic info -v /cmd_vel` 看订阅者数 | 2.3 |
| `/odom` `/tf` 消失 | 查 control.xacro remap 是否同步改名 | 2.3 |
| 验收加速度远超限值 | 先 `gz model -p` 查是否顶障碍物，再查限幅层 | 4/案3 |
| 速度爬坡极慢 | 查换算量纲（a/r 不是 a·r） | 4/案2 |
| 改 `r5.*` 参数无效果 | 参数是 on_init 快照，重启仿真 | 3.2 |
| 转弯时稳态 ω 低于理论 | 正常（scrub），看 6 节解读 | 6 |

---

## 8. 下一步衔接（阶段一剩余）

1. **转向速率限幅的动态参数化**（练习）：加 `on_set_parameter` 回调让
   `r5.*` 可运行时调，配预测表 #12 的对照。
2. **PP → Stanley → LQR 三连**（2 周）：独立 tracker 节点订阅
   `/plan`+`/odom` 发 `/cmd_vel`，绕过 Nav2 controller_server，
   用 `acceptance_test.py` 的采样思路量化三者横向误差。
3. **EKF 融合**（1 周）：`robot_localization` 融 `/odom`+`/imu`，
   `gz model -p` 真值量化漂移改善。
4. 阶段三预埋：本包的命令层后处理结构就是未来
   `hardware_interface::SystemInterface` 的镜像——实车 HW 插件里
   "两个 steer 指令 → 一个舵角查表" 的映射逻辑与这里同一思想：
   **上层几何不动，执行层翻译**。

---

## 附：当前验收命令速记

```bash
# 标准三项验收（默认参数）
ros2 run car_bringup acceptance_test.py

# 高速 a_lat 验证（看 ω 上限从 1.069 变 2.000）
ros2 run car_bringup acceptance_test.py --vx 1.0 --wz-big 3.0

# 速率限幅验证（先把 YAML 里 r5.max_steer_rate 改小并重启）
ros2 run car_bringup acceptance_test.py --max-steer-rate 1.5

# 基线对照（手动把三个 r5.* 功能关掉后跑）
```
