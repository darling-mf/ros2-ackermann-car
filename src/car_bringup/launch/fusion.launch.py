import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Standalone sensor-fusion validation: Gazebo + one explicit fusion node.

    Outputs:
      /odometry/fused    wheel position + gyro heading
      /odometry/gyro_only pure gyro integration (reference)

    This launch does not start Nav2. The default chain already includes fusion
    through gazebo.launch.py; see docs/ARCHITECTURE.md for the TF ownership.
    """
    pkg = get_package_share_directory('car_bringup')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    declare_gui = DeclareLaunchArgument('gui', default_value='true')
    declare_rviz = DeclareLaunchArgument('rviz', default_value='true')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch',
                                                  'gazebo.launch.py')),
        launch_arguments={'gui': gui, 'rviz': rviz, 'use_fusion': 'false'}.items(),
    )
    fusion = Node(
        package='car_bringup', executable='sensor_fusion.py',
        name='sensor_fusion', output='screen',
        parameters=[{'use_sim_time': True}],
    )
    return LaunchDescription([declare_gui, declare_rviz, gazebo, fusion])
