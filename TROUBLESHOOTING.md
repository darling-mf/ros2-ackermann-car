#rviz2显示不出实体处理方法
    ##简单建模也显示不出
        1. 将 RViz 的 `Fixed Frame` 从 `map` 改为 `base_link` 或 `base_footprint`。
        2. 确保 URDF 中有关节连接（没有关节，`robot_state_publisher` 不会发布 TF）。
        3. 在 RViz 中右键 `RobotModel` -> Remove，重新 Add，并手动将 `Description Topic` 选为 `/robot_description`。
        4. 如果仍不行，在终端运行 `export LIBGL_ALWAYS_SOFTWARE=1` 强制软件渲染
        5. xacro出问题了，修复xacro:`sudo apt install --reinstall ros-humble-xacro`

    ##实际小车建模显示不出
        1. URDF 标签顺序陷阱---<origin> 标签必须放在最前面，然后是 <geometry>，最后才是 <material>
        2. Xacro 模块化重复包含引发“材质重复定义”----子模块文件不包含公共文件
            <xacro:include filename="common.xacro"/>只能在主文件car.urdf.xacro中包含
