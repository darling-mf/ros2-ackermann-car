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
ros2 launch fast_lio mapping.launch.py
```

## 📂 项目结构
car_ws/
├── src/
│   ├── car_description/      # 阿克曼小车描述包
│   │   ├── urdf/            # 拆分后的 Xacro 模型文件
│   │   ├── launch/          # 启动文件
│   │   └── rviz/            # RViz 配置文件
│   └── FAST_LIO/            # 3D SLAM 算法包 (已去除非必要的 Livox 依赖)
├── .gitignore                # 忽略 build/install/log
├── README.md                 # 本文件
├── ROADMAP.md                # 开发进度规划
└── TROUBLESHOOTING.md        # 踩坑与报错记录

## 🐛 报错排查
如果你在启动过程中遇到报错（如 RViz 不显示模型、xacro 崩溃等），请查阅 TROUBLESHOOTING.md。