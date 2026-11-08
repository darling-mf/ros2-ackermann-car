# Contributing

## 构建与验证

```bash
cd ~/car_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-up-to car_bringup

# 控制器单测
colcon test --packages-select steering_controllers_library
colcon test-result --verbose

# 控制链路自检
source install/setup.bash
ros2 run car_bringup acceptance_test.py --vx 0.5
ros2 run car_bringup drive_test.py --vx 0.1 --wz 0.3
```

## 改动约定

1. **先写验收**：在 `docs/SIM_TODO.md` 或对应专题文档写清“预测/判据”，再改代码；
2. **上游补丁**：修改 `src/ros2_control` / `src/ros2_controllers` 后，导出到
   `patches/`，不要只留在本地 overlay；
3. **文档同步**：改 launch/参数/接口时更新 `docs/ARCHITECTURE.md` 与
   `docs/PROJECT_MAP.md`；
4. **排障记录**：写到 `docs/reports/`，包含现象、根因、修复、复测数据；
5. 不提交 `build/`、`install/`、`log/`、`vendor/` 与点云/录包大文件（见 `.gitignore`）。

## 提交信息

使用 `feat|fix|docs|refactor|test|chore(scope): 摘要`，例如：

```
fix(nav2): 修正重复实例导致的 TF 冲突
feat(control): 增加转向速率限幅
docs(arch): 重整仓库文档索引
```

## 实验记录风格

推荐沿用 `src/r5_steering_controller/EXPERIMENT_LOG.md` 的循环：

```
功能 → 预测 → 测试 → 数据 →（失败）破案 → 修复 → 复测
```

负结果也要记录；能解释“为什么失败”比只保留成功数据更有价值。
