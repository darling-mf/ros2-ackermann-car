cmake_minimum_required(VERSION 3.8)
project(car_bringup)

find_package(ament_cmake REQUIRED)

# 安装所有需要被 launch 加载的文件夹
install(
  DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
)

# 控制链路自检脚本：ros2 run car_bringup drive_test.py --vx 0.1 --wz 0.3
# 全局路径倒车段检查：ros2 run car_bringup plan_inspector.py
# R5 控制器三项验收：ros2 run car_bringup acceptance_test.py
# PP/Stanley/LQR 跟踪对比：ros2 run car_bringup path_tracker.py --tracker lqr
# 互补滤波融合节点：ros2 run car_bringup sensor_fusion.py
# 融合量化实验：ros2 run car_bringup fusion_test.py --loops 3
install(
  PROGRAMS scripts/drive_test.py scripts/plan_inspector.py scripts/acceptance_test.py
           scripts/path_tracker.py scripts/sensor_fusion.py scripts/fusion_test.py
           scripts/cleanup_sim.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES README.md CONTROL_DEBUG_NOTES.md ARCHITECTURE.md ROADMAP_TO_REAL.md
  DESTINATION share/${PROJECT_NAME})

ament_package()