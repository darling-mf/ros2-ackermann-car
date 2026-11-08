# Documentation Index

本目录是项目的文档入口。按“先跑通、再看架构、最后查专题”的顺序阅读。

## 1. 入门与总览

| 文档 | 用途 |
|---|---|
| [../README.md](../README.md) | 项目入口：构建、启动、目录、状态 |
| [PROJECT_MAP.md](PROJECT_MAP.md) | 文件树全景 + 进度总账 + 命令速查 |
| [ARCHITECTURE.md](ARCHITECTURE.md) | 分层架构、包职责、数据流、接口 |
| [PROJECT_STATE.md](PROJECT_STATE.md) | 压缩记忆：已验证数据、缺口、下一步 |
| [SIM_TODO.md](SIM_TODO.md) | 仿真验证清单与环境/RTF/建图复盘 |

## 2. 子系统文档

| 子系统 | 文档 |
|---|---|
| 底盘控制 / ros2_control | [../src/r5_steering_controller/STAGE1_GUIDE.md](../src/r5_steering_controller/STAGE1_GUIDE.md) |
| 控制调试方法论 | [../src/car_bringup/CONTROL_DEBUG_NOTES.md](../src/car_bringup/CONTROL_DEBUG_NOTES.md) |
| 跟踪算法（PP/Stanley/LQR） | [../src/car_bringup/scripts/path_tracker.py](../src/car_bringup/scripts/path_tracker.py) 与 STAGE1_GUIDE |
| Nav2 / 规划 / 行为树 | [../src/car_bringup/config/nav2_params.yaml](../src/car_bringup/config/nav2_params.yaml)、[../src/car_navigation/behavior_trees](../src/car_navigation/behavior_trees) |
| 建图 / 定位 / 2D 雷达 | [ARCHITECTURE.md](ARCHITECTURE.md) §6、[../src/car_bringup/launch/mapping.launch.py](../src/car_bringup/launch/mapping.launch.py) |
| 串口协议 / 虚拟 STM32 | [../src/r5_serial/docs/S5_SERIAL_BRIDGE.md](../src/r5_serial/docs/S5_SERIAL_BRIDGE.md) |
| STM32 固件 / PC 烧录 | [../firmware/openctr_b20/README.md](../firmware/openctr_b20/README.md) |
| 实机路线图 | [../src/car_bringup/ROADMAP_TO_REAL.md](../src/car_bringup/ROADMAP_TO_REAL.md) |

## 3. 实验与排障报告

| 文档 | 内容 |
|---|---|
| [../src/r5_steering_controller/EXPERIMENT_LOG.md](../src/r5_steering_controller/EXPERIMENT_LOG.md) | Round 1~17：功能→预测→测试→破案→修复→复测 |
| [reports/NAV2_DEBUG_REPORT.md](reports/NAV2_DEBUG_REPORT.md) | Nav2 闪烁/TF/自击 costmap/RTF 的完整排查 |
| [READING_GUIDE.md](READING_GUIDE.md) | 文件定位、阅读顺序、S4/S6、雷达选型、轴距实测 |
| [../patches/README.md](../patches/README.md) | 上游 ros2_controllers 补丁与应用方法 |
| [面试题-ROS2阿克曼小车-全栈75题.docx](面试题-ROS2阿克曼小车-全栈75题.docx) | 面试准备题库 |

## 4. 维护约定

- 新增功能：先在 `docs/SIM_TODO.md` 或对应专题文档写“预测/验收标准”，再改代码；
- 改上游源码：补丁必须进 `patches/`，不要只留在本地 `src/ros2_controllers`；
- 新增 launch/参数：同步更新 [ARCHITECTURE.md](ARCHITECTURE.md) 的接口/数据流；
- 排障记录：写到 `docs/reports/`，包含现象、根因、修复、复测数据。
