import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """仿真 2D 建图：Gazebo + slam_toolbox（online_async）+ RViz。

    与导航模式互斥：slam_toolbox 发布 map→odom，不能再启动 nav2 的
    map_server/AMCL（会抢同一个 TF）。建图完成后用 map_saver 保存：
      ros2 run nav2_map_server map_saver_cli -f ~/car_ws/src/car_bringup/maps/my_map
    然后把 nav2.launch.py 的 map 参数指向新 yaml 即可。
    """
    pkg = get_package_share_directory('car_bringup')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')

    declare_gui = DeclareLaunchArgument('gui', default_value='true')
    declare_rviz = DeclareLaunchArgument('rviz', default_value='true')

    # 注意：IncludeLaunchDescription 的 launch_arguments 会写进共享的 launch
    # 配置。Gazebo 这里传 rviz:=false 会把顶层的 rviz 也覆盖成 false，导致
    # 本文件的建图 RViz 永远不启动。用 scoped GroupAction 隔离这次覆盖。
    gazebo = GroupAction(
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch',
                                                      'gazebo.launch.py')),
            launch_arguments={'gui': gui, 'rviz': 'false'}.items(),
        )],
        scoped=True,
        forwarding=True,
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch',
            'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': os.path.join(
                pkg, 'config', 'slam_toolbox.yaml'),
        }.items(),
    )

    # slam_toolbox 的 apt 包不带 rviz/slam_toolbox_default.rviz，
    # 用本包专用的建图视图（MapPoints + LaserScan + RobotModel + TF）。
    rviz_config = os.path.join(pkg, 'rviz', 'mapping.rviz')
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config, '-geometry', '1600x900+50+30'],
        parameters=[{'use_sim_time': True}],
        output='screen',
        # VMware/SVGA 下用硬件 GL 时 RViz 3D 视图可能是全黑；
        # 软件渲染慢一些，但地图/激光能正常显示。
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
        condition=IfCondition(rviz),
    )

    # RViz 的 Map 显示在 VMware/Mesa 下会触发 indexed_8bit_image 的
    # GLSL 链接错误（地图纹理发花）。这个节点把 /map 转成普通 Marker
    # 点，rviz/mapping.rviz 里默认显示 /map_points。
    map_points = Node(
        package='car_bringup', executable='map_to_points.py',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        declare_gui,
        declare_rviz,
        gazebo,
        slam,
        map_points,
        rviz_node,
    ])
