from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import launch

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('car_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'car.urdf.xacro')

    # 将 xacro 解析为 URDF
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)


    return launch.LaunchDescription([
        # 1. 发布机器人状态
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen', # 加上这个，方便看 URDF 报错
            parameters=[{'robot_description': robot_description}]
        ),
        # 2. 关节状态 GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),
        # 3. 启动 RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            # 如果以后保存了 rviz 配置，可以取消注释下面这行：
            arguments=['-d', os.path.join(pkg_path, 'rviz', 'display.rviz')]
        )
    ])