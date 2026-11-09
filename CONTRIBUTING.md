# 参与开发

## 构建

```bash
cd ~/car_ws
source /opt/ros/humble/setup.bash

# 如果工作区里还没有带补丁的上游控制器源码
git clone -b humble https://github.com/ros-controls/ros2_controllers.git \
  src/ros2_controllers
git -C src/ros2_controllers checkout 65db7362
git -C src/ros2_controllers apply \
  ../../patches/ros2_controllers-r5.patch

colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-up-to car_bringup
source install/setup.bash
```

## 测试

```bash
# 控制器验收（直行 / 转向 / 倒车）
ros2 run car_bringup acceptance_test.py --vx 0.5

# 方向、转角、轮速闭环
ros2 run car_bringup drive_test.py --vx 0.1 --wz 0.3

# 上游控制器单元测试
colcon test --packages-select steering_controllers_library
colcon test-result --verbose

# 固件 C 模块（无需硬件）
cd firmware/openctr_b20/test
gcc -std=c99 -Wall -Wextra -Werror -I../src \
  test_host.c ../src/xprotocol.c ../src/safety.c ../src/ackermann.c -lm \
  -o test_host
./test_host
```

## 修改约定

1. **守住分层边界。** 底盘安全和阿克曼运动学放在
   `r5_steering_controller`；路径跟踪放在 Nav2 控制器；任务顺序放在
   行为树。
2. **TF 只能有一个发布者。** 不要同时让控制器和 `sensor_fusion.py` 都
   打开 `enable_odom_tf`。
3. **速度链保持一致。** MPPI 的 `vx_max`、`velocity_smoother` 的
   `max_velocity` 和控制器加减速度限制必须匹配。
4. **改参数前先做预测。** 记录场景、参数值和实测效果。负结果也有价值，
   应该保留。
5. **改上游代码必须落到 `patches/`。** 不要只留在本地 overlay，补丁要能
   复现。
6. **不要提交生成物和第三方材料。** `build/`、`install/`、`log/`、
   `vendor/`、点云、rosbag、厂家手册都应排除。
7. **同步更新对应 README。** 改 launch、参数或接口时，更新包级 README；
   涉及系统结构时同时更新 `docs/ARCHITECTURE.md`。

## 提交信息

使用“类型(范围): 摘要”的格式。摘要用中文。

```text
feat(control): 增加转向速率限幅
fix(nav2): 只保留一套 AMCL 实例
docs(arch): 补充串口边界说明
test(firmware): 增加坏帧重同步用例
chore(repo): 移除生成文件
```

## 提交 PR 时

- 写清问题、根因和验证命令。
- 修改控制/导航参数时，附上修改前后的实测值。
- 不要把无关格式化改动混进功能提交。
- 不要把个人笔记、面试材料或厂家源码加进公开仓库。
