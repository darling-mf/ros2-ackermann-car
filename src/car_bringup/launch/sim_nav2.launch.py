import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """一条命令启动仿真 + Nav2（Gazebo 先起，12s 后 Nav2；自动设 AMCL 初始位姿）。

    为什么延迟 20s：Nav2 的 global_costmap 需要 /map、/scan、odom→base_footprint；
    Gazebo 拉起控制器/传感器约 8~10s，留足余量。Nav2 自己也会等，但错开启动日志更干净。
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

    nav2 = TimerAction(
        period=20.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch',
                                                      'nav2.launch.py')),
            launch_arguments={'rviz': rviz}.items(),
        )],
    )

    return LaunchDescription([
        declare_gui,
        declare_rviz,
        gazebo,
        nav2,
    ])
