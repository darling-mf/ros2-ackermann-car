# ROS 2 阿克曼小车建模与 3D SLAM 探索 (Ackermann Car & FAST-LIO2)

基于 **Ubuntu 22.04** + **ROS 2 Humble** 的阿克曼小车项目。目前已完成 3D 雷达建模、Gazebo 物理仿真，并成功编译集成了 FAST-LIO2 激光雷达惯性里程计，用于 3D 建图与导航算法的学习。

## 📦 依赖
- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic
- 安装依赖：
```bash
    sudo apt install ros-humble-joint-state-publisher-gui ros-humble-xacro
    sudo apt install ros-humble-joint-state-publisher-gui \
                 ros-humble-xacro \
                 ros-humble-gazebo-ros-pkgs \
                 ros-humble-teleop-twist-keyboard
```

## 🚀 编译与运行
```bash
cd ~/car_ws
rm -rf build install log  # 清理缓存
colcon build
source install/setup.bash

纯udrf可视化（rviz2）
ros2 launch car_description display.launch.py

Gazebo物理仿真
ros2 launch car_description gazebo.launch.py

键盘测试工具
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard

FAST-LIO2 3D 建图仿真
ros2 launch fast_lio mapping.launch.py \
    config_file:=/home/mjf/car_ws/src/FAST_LIO/config/velodyne.yaml \
    use_sim_time:=true
```
## 📂 项目结构
car_ws/
├── src/
│   ├── car_description/      # 阿克曼小车描述包
│   │   ├── urdf/
│   │   │   └── car/         # 拆分后的 Xacro 模型文件
│   │   │       ├── chassis.xacro
│   │   │       ├── wheels.xacro
│   │   │       ├── laser.xacro
│   │   │       └── imu.xacro
│   │   ├── launch/          # 启动文件
│   │   ├── rviz/            # RViz 配置文件
│   │   └── world/           # Gazebo 地图文件
│   └── FAST_LIO/            # 3D SLAM 算法包 (已去除非必要的 Livox 依赖)
├── .gitignore                # 忽略 build/install/log
├── README.md                 # 本文件
├── ROADMAP.md                # 开发进度规划
└── TROUBLESHOOTING.md        # 踩坑与报错记录

## 🛠️ 开发与编译工作流 (Development Workflow)

```bash
本项目推荐使用带有符号链接（symlink）的编译方式，避免每次修改简单文件都要重新编译。

首次全量编译	cd ~/car_ws && colcon build --symlink-install
日常单包编译	colcon build --symlink-install --packages-select car_description
刷新环境变量    source install/setup.bash
```
> **💡 极简法则（顺口溜）：**
> **改文本、改配置，直接跑，不用编。**
> **碰 C++、加节点、动结构，必须编。**

| 文件类型                | 具体举例                            | 修改后是否需要重新 `colcon build`？ | 软链接是否生效？ |
| **URDF / Xacro**       | `car.urdf.xacro`, `chassis.xacro`  | **不需要**                       | ✅ 生效 |
| **Launch 文件**         | `display.launch.py`               | **不需要**                        | ✅ 生效 |
| **Rviz / YAML 配置**    | `display.rviz`, `params.yaml`     | **不需要**                        | ✅ 生效|
| **Python 脚本（逻辑）**  | 已有节点内 `.py` 代码修改             | **不需要**                       | ✅ 生效 |
| **Python 新节点**       | 在 `setup.py` 里新增 `entry_points` | **必须重新编译**                   | ❌ 不生效 |
| **C++ 源码**            | `.cpp`, `.hpp`, `.cc`              | **必须重新编译**                  | ❌ 不生效 |
| **构建配置**            | `CMakeLists.txt`, `package.xml`     | **必须重新编译**                  | ❌ 不生效 |
| **消息/服务定义**       | `.msg`, `.srv`, `.action`           | **必须重新编译**                   | ❌ 不生效 |


> **⚠️ 避坑提示（断头路与移植）：**
> 1. **断头路报错**：如果在 `src` 里删除或重命名了文件，`install` 里的软链接会变成“断链”指向虚无。必须清理 `install` 重新 build。
> 2. **移植灾难**：软链接里存的是你电脑的绝对路径。如果要打包发给别人，必须进行一次普通的 `colcon build`（不加 `--symlink-install`），生成纯拷贝版。

> **🚑 玄学报错急救包：**
> 但凡遇到“我明明改对了，为什么它没生效”或者“莫名其妙的路径报错”，直接执行终极三连：
> ```bash
> cd ~/car_ws
> rm -rf build/ install/ log/
> colcon build --symlink-install
> source install/setup.bash
> ```


## 🐛 报错排查
如果你在启动过程中遇到报错（如 RViz 不显示模型、xacro 崩溃等），请查阅 TROUBLESHOOTING.md。