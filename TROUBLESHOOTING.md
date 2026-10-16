# ROS 2 项目踩坑与报错记录 (TROUBLESHOOTING)

本文件用于记录在阿克曼小车建模、Gazebo 仿真及 FAST-LIO2 集成过程中遇到的各种报错及解决方案。

## 一、 RViz2 显示问题

### 1. 简单建模也显示不出实体
- **现象**：RViz 里只有坐标轴，没有模型实体。
- **解决方案**：
    1. 将 `Fixed Frame` 从 `map` 改为 `base_link` 或 `base_footprint`。
    2. 确保 URDF 中有关节连接（没有关节，`robot_state_publisher` 不会发布 TF）。
    3. 在 RViz 中右键 `RobotModel` -> Remove，重新 Add，并手动将 `Description Topic` 选为 `/robot_description`。
    4. 如果仍不行，在终端运行 `export LIBGL_ALWAYS_SOFTWARE=1` 强制软件渲染（虚拟机下非常有效）。
    5. 修复 xacro 潜在崩溃：`sudo apt install --reinstall ros-humble-xacro`。

### 2. 实际小车建模显示不出
- **URDF 标签顺序陷阱**：`<origin>` 标签必须放在最前面，然后是 `<geometry>`，最后才是 `<material>`。
- **Xacro 模块化重复包含**：子模块文件（如 `chassis.xacro`）绝不能包含公共文件（如 `common.xacro`），否则会引发“材质重复定义”报错。`common.xacro` 必须只在主文件 `car.urdf.xacro` 中包含一次。

## 二、 Gazebo 仿真问题

### 1. 阿克曼驱动插件缺失
- **现象**：ROS 2 Humble 并未原生内置可直接用的 `libgazebo_ros_ackermann_drive.so`。
- **解决方案**：在仿真阶段暂时使用 `libgazebo_ros_diff_drive.so` 差速驱动插件，将后轮作为驱动轮，前轮设置为被动关节。

### 2. TF 树混乱/轮子闪烁或消失
- **现象**：Gazebo 中一运动，轮子就消失或 RViz 中 TF 疯狂闪烁（多个发布者打架）。
- **解决方案**：
    1. 在差速插件中设置 `<publish_wheel_tf>false</publish_wheel_tf>`，让插件闭嘴。
    2. 在 `car.urdf.xacro` 中加入 `libgazebo_ros_joint_state_publisher.so` 插件，向 `/joint_states` 发布关节状态。
    3. 在 `gazebo.launch.py` 中彻底删除 `joint_state_publisher` 节点，让 `robot_state_publisher` 全权接管 TF 发布。
    4. 发现驱动轮转向相反时，在宏中利用 `y_reflect` 镜像设置旋转轴 `axis xyz="0 0 ${-y_reflect}"`。

## 三、 FAST-LIO2 编译与集成问题

### 1. 源码硬编码 Livox 依赖导致编译失败
- **现象**：`CMake Generate step failed`，找不到 `livox_ros_driver2`。
- **原因**：FAST-LIO2 默认强绑定 Livox 雷达消息类型，但 Gazebo 仿真输出的是标准 `PointCloud2`。
- **解决方案（彻底切除 Livox 依赖）**：
    - `CMakeLists.txt`：注释掉 `find_package(livox_ros_driver2 REQUIRED)`，并在 `set(dependencies ...)` 中注释掉 `# livox_ros_driver2`。
    - `package.xml`：删除或注释掉 `<depend>livox_ros_driver2</depend>`。
    - `include/preprocess.h`：注释掉 Livox 头文件、`process(...)` 和 `avia_handler(...)` 函数声明。
    - `src/preprocess.cpp`：注释掉 Livox 专属的 `process(...)` 和 `avia_handler(...)` 函数体，注释掉 `if (lidar_type == AVIA)` 代码块。
    - `src/laserMapping.cpp`：注释掉 `#include <livox...>`，注释掉 `livox_pcl_cbk` 函数，注释掉 `sub_pcl_livox_` 成员变量，并在构造函数中注释掉订阅 Livox 的逻辑。

### 2. `ikd-Tree` 文件夹为空导致找不到源文件
- **现象**：`find . -name "ikd_Tree.cpp"` 无输出，编译报错 `Cannot find source file: include/ikd-Tree/ikd_Tree.cpp`。
- **原因**：`ikd-Tree` 是 Git 子模块（Submodule），直接 `git clone` 主仓库不会自动拉取子模块内容。
- **解决方案**：执行 `git submodule update --init --recursive` 拉取子模块内容。

### 3. Boost 库满屏警告（非致命）
- **现象**：编译时满屏 `BOOST_PRAGMA_MESSAGE` 黄色警告。
- **原因**：Ubuntu 22.04 自带 Boost 1.74+ 对旧式占位符写法的兼容性提醒，不影响编译与运行。
- **解决方案（强迫症消除法）**：在 `CMakeLists.txt` 中 `add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")` 下方添加 `add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)`，重新编译即可。

### 4. CMake 策略警告（无需理会）
- 类似 `Policy CMP0074 is not set` 和 `PCL_ROOT is set to /usr` 的警告，属于 CMake 3.25+ 对老项目的例行提示，无需理会，修改反而易引发 PCL 找不到的连锁报错。