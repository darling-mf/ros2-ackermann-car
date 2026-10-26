# PROJECT_MAP —— 项目文件树全景 + 进度总账

> 更新：2026-10-26。回答三个问题：**每个文件在哪、是干嘛的；已经做完了什么；
> 还差什么才到实车**。配套深读文档见各目录内的 *.md（本文件只做索引）。

---

## 一、一句话现状

**仿真侧全部闭环 + S4 倒车实测通过**（底盘执行 / 跟踪算法 / 传感器融合 /
串口协议栈 / Nav2 倒车分流）。**S6 固件模块已完成并在 PC 上通过单测**，
剩下实车联调；再往后是 S7（hardware_interface 插件）→ S8（标定）→ S9
（实机集成）。乐观 3~4 周（取决于硬件与雷达到货）。

---

## 二、文件树（自研部分，逐文件注解）

```
~/car_ws/                              # colcon 工作区根
├── README.md                        # 项目入口（状态/命令/文档索引）
├── PROJECT_MAP.md                   # ← 本文件（总账）
├── docs/READING_GUIDE.md            # 阅读顺序 + S4/S6 报告 + 雷达/轴距指南
├── patches/                         # ros2_controllers 补丁（上游修改归档）
├── firmware/openctr_b20/            # S6 固件模块（纯 C，主机单测通过）
├── vendor/                          # 厂家源码（UTF-8/GBK 两版）与 PDF 手册
├── build/ install/ log/             # colcon 产物（可删重建，命令见 §四）
└── src/
    ├── car_bringup/                   # 【总装包】启动、配置、全部实验脚本
    │   ├── launch/gazebo.launch.py    #   仿真总启动：Gazebo+控制器加载（含 URDF 去注释
    │   │                              #   修复、GAZEBO_MODEL_PATH、gui/rviz 开关）
    │   ├── launch/nav2.launch.py      #   Nav2 导航启动（读取 maps/my_room_map）
    │   ├── config/controllers.yaml    #   ros2_control 控制器参数（R5 控制器参数、
    │   │                              #   右前左后关节顺序、use_stamped_vel、限幅、协方差）
    │   ├── config/nav2_params.yaml    #   Nav2 全套参数（Smac Hybrid+MPPI+AMCL+costmap）
    │   ├── maps/my_room_map.*         #   2D 栅格地图（建图产物）
    │   ├── scripts/drive_test.py      #   【自检】cmd_vel→odom 方向闭环自检（✅/❌ 判定）
    │   ├── scripts/acceptance_test.py #   【验收】R5 控制器三项验收：加速度限幅/
    │   │                              #   速度相关转向限幅/转向速率限幅，PASS/FAIL 报告
    │   ├── scripts/path_tracker.py    #   【实验台】PP/Stanley/LQR 三合一跟踪器对比
    │   │                              #   （LQR 带增益调度 K(v)，--no-schedule 对照）
    │   ├── scripts/sensor_fusion.py   #   【融合】互补滤波 v2：陀螺主+轮式锚定航向，
    │   │                              #   轮速沿融合航向积分位置（"轮子管里程陀螺管方向"）
    │   ├── scripts/fusion_test.py     #   【实验】圆周三源航向量化（对 p3d 真值，
    │   │                              #   含坐标系对齐修正）→ 融合 11.7° 最优
    │   ├── scripts/plan_inspector.py  #   【工具】检查 /plan 倒车段（Nav2 倒车分流用）
    │   ├── README.md                  #   使用手册（构建/启动/遥控/自检）
    │   ├── CONTROL_DEBUG_NOTES.md     #   【复盘】S0 八大 bug 深度切割（左右顺序/
    │   │                              #   cmd_vel 类型/轴符号/摩擦/注释炸插件/协方差/ABI）
    │   ├── ROADMAP_TO_REAL.md         #   【路线图】S0~S9 实机里程碑与完成判据
    │   └── ARCHITECTURE.md            #   早期架构笔记
    │
    ├── car_description/               # 【机器人模型】URDF + Gazebo 世界
    │   ├── urdf/car.urdf.xacro        #   顶层 xacro（组装全部模块）
    │   ├── urdf/common.xacro          #   尺寸/质量/材质参数（轮径 0.0325=65mm 实车值）
    │   ├── urdf/car/chassis.xacro     #   base_footprint/base_link/车体
    │   ├── urdf/car/wheels.xacro      #   四轮：后轮 axis=-1（符号修复）、转向限位
    │   │                              #   ±0.235（R=0.55 实车值）、舵机速率 9.5、四轮 mu=1.0
    │   ├── urdf/car/control.xacro     #   ros2_control 硬件接口 + p3d 真值插件
    │   │                              #   （两个实测教训写在注释里：p3d 默认话题 odom、
    │   │                              #   固定关节合并后连杆名是 base_footprint）
    │   ├── urdf/car/imu.xacro         #   IMU 传感器（噪声按实车 MPU6050 量级：零偏 17°/min）
    │   ├── urdf/car/laser.xacro       #   2D 雷达（/scan）
    │   ├── urdf/car/sonar.xacro       #   超声波（/ultrasonic）
    │   ├── urdf/car/3D_*.xacro, mapping_car.xacro  # 3D 雷达遗留（已弃用，未挂载）
    │   ├── world/custom_room.world    #   测试房间（含家具，倒车/掉头压力测试场）
    │   └── rviz/display.rviz          #   RViz 配置（Fixed Frame=odom）
    │
    ├── r5_steering_controller/        # 【自研控制器】ros2_control 阿克曼插件（阶段一核心）
    │   ├── include/.../r5_steering_controller.hpp
    │   ├── src/r5_steering_controller.cpp  # 继承 ackermann 控制器，命令层后处理三功能：
    │   │                              #   ①驱动轮速度斜坡(AT8236 限流保护) ②速度相关
    │   │                              #   转向限幅(a_y=v²tanφ/L≤a_lat) ③转向速率限幅(舵机)
    │   ├── r5_steering_controller.xml #   pluginlib 注册
    │   ├── STAGE1_GUIDE.md            #   【教材】数据流全景/逐行解析/12 条修改预测表/排错树
    │   └── EXPERIMENT_LOG.md          #   【实验日志】Round 1~14 全循环（功能→预测→测试→
    │                                  #   数据→破案→修复→复测），项目最有价值的文档
    │
    ├── r5_serial/                     # 【串口桥】ROS2 ↔ OpenCTR B20（S5，无硬件联调通过）
    │   ├── r5_serial/protocol.py      #   X-Protocol 编解码（帧号 0x10/0x50/0x51/0x5f，
    │   │                              #   校验和/限幅/单位换算，与固件源码逐行对照，
    │   │                              #   __main__ 含三条自检断言含坏帧重同步）
    │   ├── r5_serial/serial_port.py   #   最小串口层（纯标准库 termios/select，pty/真实 tty 同路径）
    │   ├── r5_serial/virtual_stm32.py #   虚拟 STM32：pty 上扮演固件（50Hz 0x10、速度闭环、
    │   │                              #   坏帧注入 --corrupt、看门狗证据日志）
    │   ├── r5_serial/serial_bridge.py #   ROS2 桥：/cmd_vel→0x50(50Hz+看门狗)，
    │   │                              #   0x10→/r5/wheel_vel + /r5/imu_raw + /r5/battery
    │   ├── docs/S5_SERIAL_BRIDGE.md   #   【文档】协议精确规格（固件源码提取）/固件 vs 手册
    │   │                              #   差异表/七项测试/真机 Checklist/疑点清单/所需硬件
    │   └── setup.py, package.xml      #   ament_python 打包（entry_points: virtual_stm32/serial_bridge）
    │
    ├── car_navigation/                # Nav2 行为树（akm_navigate_*.xml：无 Spin、BackUp 0.6m）
    │
    ├── ros2_control/ ros2_controllers/  # 上游源码 overlay（humble 分支）
    │   ├── steering_controllers_library/  # 被我们修改：里程计协方差 bug 修复、
    │   │   │                            #   max_steering_angle 虚拟角限幅、
    │   │   │                            #   FK 往返一致性修复（右转 9% 系统偏差）
    │   │   └── src/steering_odometry.cpp #  ↑ 三处补丁都在这个文件，ABI 兼容保持
    │   └── ackermann_steering_controller/ # 被我们修改：CMake 头文件搜索顺序修复
    │
    ├── gz_ros2_control/               # 新 Gazebo 插件（本项目用 Classic，未使用）
    ├── FAST_LIO/ FAST-LIO-SAM-QN/ livox_ros_driver2/  # 3D 雷达方案（已弃，COLCON_IGNORE；
    │                                                  #  恢复：删除对应 COLCON_IGNORE）
    └── .git                           # 工作区 git
```

**补丁归档（本仓库跟踪）**
```
~/car_ws/patches/
├── README.md                        # 上游版本/应用方式/4 处补丁说明
└── ros2_controllers-r5.patch        # 对 ros2_controllers 2.54.1 的全部修改
```
外部仓库 `src/ros2_control|ros2_controllers|gz_ros2_control` **不进 git**
（.gitignore），按 `patches/README.md` 重新 clone + `git apply` 即可复现。

**厂商资料（不在 git，本地留存）**
```
~/car_ws/vendor/
├── xtark_fw/                                  # ★ 研读标注版（UTF-8，VS Code 直接可读）
│   ├── OpenCTR_B20V11_AKM_V1.00.241015/
│   │   ├── Robot/ax_robot.c/.h    # 50Hz 主任务、0x10 帧载荷、帧号表、R5 参数(轴距0.146!)
│   │   ├── Robot/ax_kinematics.c  # 阿克曼 IK/FK + 舵机映射（P1 缺陷现场，见 README_ANNOTATED）
│   │   ├── Robot/ax_akm.c         # AX_AKM_WToAngle: vx,vw -> ax_akm_angle（单位 0.001 rad）
│   │   ├── Robot/ax_speed.c       # 增量式"PID"（Kp 实为积分、Kd 实为比例）
│   │   ├── Driver/ax_uart4.c      # ★ 真正接收 0x50 的地方（168~176 行）；S1/S2/S3 隐患
│   │   └── Driver/ax_uart_pi.c    # 旧版镜像实现（B20 不调用，仅对照）
│   ├── README_ANNOTATED.md        # ★ 导读：目录约定/单位总表/缺陷清单/阅读顺序
│   └── tools/annotate.py          # 标注脚本（幂等，可重跑）
├── xtark_fw_original_gbk/                     # 厂商原始树（GBK，Keil 编译烧录用这个）
└── xtark_docs/                                # 6 份 PDF 手册
```
7 个核心文件共 51 处标注，已验证"剥掉注释后代码与厂商原文件逐字节一致"。


---

## 三、进度总账（S=里程碑，详见 ROADMAP_TO_REAL.md）

| 里程碑 | 状态 | 验收证据 |
|---|---|---|
| S0 仿真执行链闭环（ros2_control+阿克曼控制器） | ✅ | 三项验收 PASS，38 上游单测全绿 |
| S1 跟踪算法（PP/Stanley/LQR+增益调度+限幅） | ✅ | 六组对比数据 + Round 10 修正循环 |
| S2 量化基建（p3d 真值/自检/验收/实验脚本） | ✅ | Round 1~13 实验日志 |
| S3 传感器融合（互补滤波 v2，实车级 IMU 噪声） | ✅ | 圆周实验融合 11.7° 最优（vs 陀螺 17.4°/轮式 85°） |
| S5 串口桥+虚拟 STM32 | ✅ | 七项测试（含 3% 坏帧注入、看门狗证据） |
| S4 Nav2 调优+倒车分流 | ✅ | 正后方目标 (-1,0) 导航成功；终态 (-0.906,-0.251,yaw≈0.016)，后退 0.93m 不倒头 |
| **S6 STM32 固件改造** | 🟡 模块完成 | `firmware/openctr_b20` 的协议/看门狗/阿克曼模块 + 主机单测；实车集成待硬件 |
| **S7 hardware_interface 插件** | ⬜ | 实车跑 drive_test.py，方向与仿真一致；上层零改动 |
| **S8 标定三件套** | ⬜ | 轮径/轴距/轮距实测裁决（固件 0.146/0.160 vs 手册 0.112/0.164 之争）、编码器 1560 验证、满舵半径≈0.35~0.55 实测 |
| **S9 实机集成** | ⬜ | 遥控→融合里程计→tracker→Nav2 逐级点亮 |

## 四、常用命令速查

```bash
# 全量重建（删 build/install 后）
cd ~/car_ws && source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to car_bringup

# 仿真 + 控制器
source install/setup.bash && ros2 launch car_bringup gazebo.launch.py [gui:=false rviz:=false]
ros2 control list_controllers        # 应见 joint_state_broadcaster + r5_steering_controller 均 active

# 自检三连
ros2 run car_bringup drive_test.py --vx 0.1 --wz 0.3
ros2 run car_bringup acceptance_test.py [--vx 1.0]
ros2 run car_bringup path_tracker.py --tracker lqr --path line --offset 0.3

# 融合实验（需先 ros2 run car_bringup sensor_fusion.py）
ros2 run car_bringup fusion_test.py --loops 3

# 串口链（无硬件联调）
ros2 run r5_serial virtual_stm32                    # 记下 /dev/pts/N
ros2 run r5_serial serial_bridge --port /dev/pts/N
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.4}}"
```

## 五、git 与外部依赖

- 外部上游仓库 `src/ros2_control`、`src/ros2_controllers`、`src/gz_ros2_control`
  **不进本仓库**（已加入 `.gitignore`，并从索引移除 gitlink）；
- 对 `ros2_controllers` 的修改（协方差/转向限幅/FK 一致性/CMake 头文件顺序）
  归档为 `patches/ros2_controllers-r5.patch`，基于上游 humble `65db7362`（2.54.1）；
- 新环境复现：先 clone 三个外部仓库（见 `patches/README.md`）→ `git apply`
  补丁 → `colcon build`。

## 六、当前最大的三个未知数（都是"实测裁决"型，不阻塞开发）

1. **固件舵机映射增益偏差**（ax_kinematics.c:107）：读码已定位——`ax_akm_angle`
   单位是 0.001 rad（ax_akm.c:66 返回 `akm_angle*1000`），107 行把弧度当度数
   （`*0.01745`）换算一次，120 行又乘 `180/π` 换回一次，两次近似抵消，实际
   只剩约 **+12% 增益偏差 + 梯形补偿项失效**（并非 57 倍错误）。上真机发
   vw=600 量舵机偏角即可确认，按 `atan(L/(L/tan δ + w/2))`（弧度）重写为正确版;
2. **实车几何参数**：轴距 0.146 vs 0.112、轮距 0.160 vs 0.164、R_min 0.35 vs
   0.55——卷尺+满舵画圆一次裁决，同步改 URDF/Nav2/融合节点；
3. **固件无看门狗**：桥已兜底，S6 加下位机 300ms 看门狗是安全闭环的最后一块。
