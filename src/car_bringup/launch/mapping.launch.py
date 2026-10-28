import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)
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

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch',
                                                  'gazebo.launch.py')),
        launch_arguments={'gui': gui, 'rviz': 'false'}.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch',
            'online_async_launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    rviz_config = os.path.join(
        get_package_share_directory('slam_toolbox'), 'rviz',
        'slam_toolbox_default.rviz')
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        declare_gui,
        declare_rviz,
        gazebo,
        slam,
        rviz_node,
    ])
