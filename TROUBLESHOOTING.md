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

## 四、 FAST-LIO2 运行与数据流问题

### 1. `No point, skip this scan!` 报错
- **现象**：FAST-LIO2 启动后终端反复打印此警告，RViz 黑屏无点云。
- **原因与解决方案**：
    1. **点云格式不匹配**：将 `velodyne.yaml` 中 `lidar_type` 改为 `0`（通用处理器），因为 Gazebo 的 `libgazebo_ros_ray_sensor.so` 输出的是基础 `PointCloud2` 格式（无 `ring` 和 `time` 字段）。
    2. **过滤参数过严**：将 `point_filter_num` 改为 `1`，`blind` 改为 `0.1`，确保点云不被过滤光。
    3. **Gazebo 仿真时间未同步**：启动 FAST-LIO2 时必须加上 `use_sim_time:=true` 参数。
    4. **Gazebo 处于暂停状态**：启动 Gazebo 后点击窗口按空格键，确保左下角时间在跳动。

### 2. IMU 无数据 / `Get noise index not valid`
- **现象**：`/imu` 话题有 Publisher 但无数据，Gazebo 终端报 `Get noise index not valid`。
- **原因**：Gazebo Classic 中，`libgazebo_ros_imu_sensor.so` 插件如果只写了 `<imu>` 标签但没有配置 `<noise>` 噪声模型，会导致传感器初始化不合法，拒绝输出数据。
- **解决方案**：在 `imu.xacro` 的 `<imu>` 标签内补全 `angular_velocity` 和 `linear_acceleration` 的标准高斯噪声参数，并设置 `<always_on>true</always_on>`。

### 4. Xacro 文件结构规范（包含 vs 宏）
- **现象**：使用 `<xacro:include>` 包含子文件时报 `ExpatError: junk after document element`。
- **原因**：`xacro:include` 相当于将子文件内容原封不动地复制粘贴到主文件。如果子文件包含 `<?xml version="1.0"?>` 或 `<robot>` 标签，就会导致主文件出现嵌套的根节点，XML 解析崩溃。
- **解决方案**：
    - 使用 `xacro:include` 时，子文件（如 `laser.xacro`、`imu.xacro`）绝对不能有 `<?xml>` 声明和 `<robot>` 标签。
    - 使用 `xacro:macro` 定义时，宏内部可以包裹在 `<robot>` 标签中，因为 Xacro 引擎在调用宏时会自动剥离外层标签。
|        文件类型        | `<?xml?>` 声明   | `<robot>` 根标签 | `xmlns:xacro` 声明 | include 其他文件 |
| 主文件`car.urdf.xacro` |    ✅ 必须有     |     ✅ 必须有    |       ✅ 必须有    | ✅ 统一在此 include |
| 子文件`common.xacro`   |    ❌ 不能有     |     ✅ 必须有    |       ✅ 必须有    | ❌ 不 include |
| 子文件`chassis.xacro`  |    ❌ 不能有     |     ✅ 必须有    |       ✅ 必须有    | ❌ 不 include |
| 子文件`wheels.xacro`   |    ❌ 不能有     |     ✅ 必须有    |       ✅ 必须有    | ❌ 不 include |
| 子文件`laser.xacro`    |    ❌ 不能有     |     ✅ 必须有    |       ✅ 必须有    | ❌ 不 include |
| 子文件`imu.xacro`      |    ❌ 不能有     |     ✅ 必须有    |       ✅ 必须有    | ❌ 不 include |

