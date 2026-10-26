# patches/ —— 上游源码补丁归档

本目录保存**对外部上游仓库的本地修改**。外部仓库本身不进本 git
（体积大、且是第三方代码），用补丁文件复现我们的修改。

## ros2_controllers-r5.patch

- **上游**：https://github.com/ros-controls/ros2_controllers.git
- **分支/版本**：`humble`，本补丁基于 commit `65db7362`（2.54.1）
- **应用方式**（先 clone 到 `src/ros2_controllers`）：

  ```bash
  cd ~/car_ws/src/ros2_controllers
  git checkout humble
  git apply --check ../../patches/ros2_controllers-r5.patch   # 先干跑
  git apply ../../patches/ros2_controllers-r5.patch
  ```

- **补丁内容**（4 处修复，全部有实验依据，见
  `src/r5_steering_controller/EXPERIMENT_LOG.md`）：

  | 文件 | 修改 | 原因 |
  |---|---|---|
  | `steering_odometry.{hpp,cpp}` | 新增带 `max_steering_angle` 的 `get_commands` 重载；Ackermann 虚拟中心角限幅 + 单轮兜底；`wheel→virtual` 反变换按转弯方向选内外轮 | 上游无转向限位（纯旋转指令会算出 2.2 rad）；上游 FK 往返不一致（右转高估 ~9%） |
  | `steering_controllers_library.cpp` | 修复里程计协方差：pose/twist 分开写入；读取节点参数 `max_steering_angle` | 上游把 pose 协方差写进 twist 数组，pose 恒为 0 |
  | `steering_controllers_library.yaml` | 协方差默认值改为量纲合理的小量 | 上游默认值 `[0,7,14,...]` 是把矩阵对角下标当成了数值 |
  | `ackermann_steering_controller/CMakeLists.txt` | 强制 overlay 头文件优先（`include_directories(BEFORE ...)`） | /opt 有旧版 2.53.1，头文件混用会导致类布局不一致（测试段错误） |
  | `ackermann_steering_controller.yaml` | 新增 `max_steering_angle` 参数声明 | 参数必须先声明才生效 |

  **ABI 注意**：补丁保持 `SteeringControllersLibrary` 的基类 `Params`
  字段顺序与 /opt 2.53.1 一致，`SteeringOdometry` 只加方法重载不改布局
  以外的东西——目的是与 `/opt` 里预编译的 bicycle/tricycle 控制器共存。

## 其他外部仓库（无修改，无需补丁）

| 目录 | 上游 | 版本 | 说明 |
|---|---|---|---|
| `src/ros2_control` | ros-controls/ros2_control | `7711a43d` (2.54.3) | controller_manager 等基础包，未修改 |
| `src/gz_ros2_control` | ros-controls/gz_ros2_control | `3921125` | 新 Gazebo 插件，本项目用 Gazebo Classic，未使用 |

三个外部仓库已加入根 `.gitignore`。重建工作区时按上表 clone 到对应
目录即可（或用 apt 二进制包，本仓库控制器补丁仍可单独应用）。
