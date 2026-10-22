import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    pkg_path = get_package_share_directory('car_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'car', 'mapping_car.xacro')
    world_path = os.path.join(pkg_path, 'world', 'custom_room.world')
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'display.rviz')

    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toxml()

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('gazebo_ros'),
                             'launch', 'gazebo.launch.py')
            ]),
            launch_arguments=[('world', world_path)]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': True
            }]
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'mapping_car',
                '-z', '0.02'
            ],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': True}]
        )
    ])