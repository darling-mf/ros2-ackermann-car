# ROS2 智能小车开发进度与完成次序表 (ROADMAP)

> 本项目严格遵循 ROS2 官方推荐的自底向上（Bottom-Up）开发范式。所有模块遵循“先仿真验证，后实车部署”的原则。

## 📊 总体进度概览
- [x] 已完成  [ ] 进行中  [ ] 未开始

---

## 🗺️ 开发阶段与完成次序

| 阶段 | 模块/任务 | 核心内容 | 产出/验证标准 | 状态 |
| :--- | :--- | :--- | :--- | :---: |
| **第一阶段：基础建模** | URDF/Xacro 建模 | 底盘、车轮、传感器等Xacro文件的编写与拆分 | `common.xacro`, `chassis.xacro`, `wheels.xacro` 等文件完成 | [x] |
| | TF 坐标树 | 建立 `base_link`, `base_footprint`, `odom` 等坐标系 | `rviz2` 中显示模型且TF树无报错 | [x] |
| **第二阶段：可视化与仿真** | RViz 基础显示 | 编写 `display.launch.py`，加载模型 | 模型能在 RViz 中正常显示，且能通过 GUI 控制关节 | [x] |
| | Gazebo 仿真环境 | 编写 `gazebo.launch.py`，加载 `car.urdf.xacro` | 模型成功生成在 Gazebo 中，未坠落 | [x] |
| | 传感器仿真插件 | 在 Xacro 中配置 IMU、激光雷达、摄像头的 Gazebo 插件 | Gazebo 中能输出 `/scan`, `/imu`, `/camera` 话题 | [ ] |
| **第三阶段：运动控制** | ros2_control | 配置差速驱动控制器（DiffDriveController） | `ros2_control` 节点成功加载 | [ ] |
| | 运动控制验证 | 发布 `/cmd_vel` 话题 | 小车在 Gazebo 中能按预期前后左右移动 | [ ] |
| **第四阶段：建图与导航** | SLAM 建图 | 引入 `slam_toolbox` 或 `cartographer` | 控制小车运动，能在 RViz 中生成 2D 栅格地图 | [ ] |
| | Nav2 导航 | 配置导航参数，引入 Nav2 栈 | 小车能实现自主定位与路径规划（Point-to-Point） | [ ] |
| **第五阶段：实车部署与优化** | 硬件驱动对接 | 替换仿真话题为真实硬件驱动（如STM32/Arduino串口） | 实车能接收 `/cmd_vel` 并反馈 `/odom` | [ ] |
| | 高级应用 | 视觉巡线、目标检测、语音控制等 | 具体业务功能上线 | [ ] |

---

## 📝 更新记录
- 2026-10-15: 完成 URDF 拆分，建立基础 TF 树；成功在RVIZ正常显示。
- 2026-10-16: 在URDF配置gazembo属性，引入阿克曼驱动插件，成功在GAZEBO正常显示。
