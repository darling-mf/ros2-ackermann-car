# 文档索引

公开仓库的文档刻意保持精简：一篇架构说明、一篇验证记录、一篇排障手册，
再给每个包配一份 README。详细的个人工程笔记不进入公开树。

## 推荐阅读顺序

1. [README](../README.md)：项目范围、构建、运行、实测结果。
2. [ARCHITECTURE.md](ARCHITECTURE.md)：分层、包职责、接口、数据流，以及
   每个子系统具体改哪个文件。
3. [VALIDATION.md](VALIDATION.md)：测试方法、实测数字、仍然失败的场景。
4. [TROUBLESHOOTING.md](TROUBLESHOOTING.md)：Nav2、TF、代价地图、AMCL
   的可复用排查路径。

## 包级文档

| 包 / 模块 | 文档 | 内容 |
|---|---|---|
| 底盘控制器 | [src/r5_steering_controller/README.md](../src/r5_steering_controller/README.md) | 阿克曼控制器、限幅、参数、测试 |
| 启动与 Nav2 | [src/car_bringup/README.md](../src/car_bringup/README.md) | 启动模式、配置文件、地图、脚本 |
| 机器人模型 | [src/car_description/README.md](../src/car_description/README.md) | URDF/xacro、坐标帧、Gazebo 世界、RViz |
| 导航行为 | [src/car_navigation/README.md](../src/car_navigation/README.md) | 阿克曼行为树和恢复策略 |
| 串口桥 | [src/r5_serial/README.md](../src/r5_serial/README.md) | X-Protocol、虚拟 STM32、主机测试 |
| STM32 模块 | [firmware/openctr_b20/README.md](../firmware/openctr_b20/README.md) | C 模块、集成点、主机测试 |
| 上游补丁 | [patches/README.md](../patches/README.md) | `ros2_controllers` 补丁和应用步骤 |

## 文档约定

- 描述某个子系统时，必须指向真正实现它的文件。
- 测试结果必须写清场景、参数和证据。
- 已知失败保留在 [VALIDATION.md](VALIDATION.md)，不要藏进变更日志。
- 不上传厂家固件、手册、个人笔记和面试材料。
