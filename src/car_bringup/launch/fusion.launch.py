import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """仿真传感器融合实验：Gazebo + 互补滤波节点。

    输出：
      /odometry/fused    轮式位置 + 陀螺航向（融合）
      /odometry/gyro_only 纯陀螺积分（对照）
    注意：本 launch 不启动 Nav2；融合里程计尚未替换默认 /odom。
    要用它做导航，需把控制器的 odom/TF 发布关掉并让融合节点成为唯一
    里程计源（见 docs/ENGINEERING_HANDOVER.md §2.5）。
    """
    pkg = get_package_share_directory('car_bringup')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    declare_gui = DeclareLaunchArgument('gui', default_value='true')
    declare_rviz = DeclareLaunchArgument('rviz', default_value='true')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch',
                                                  'gazebo.launch.py')),
        launch_arguments={'gui': gui, 'rviz': rviz}.items(),
    )
    fusion = Node(
        package='car_bringup', executable='sensor_fusion.py',
        name='sensor_fusion', output='screen',
        parameters=[{'use_sim_time': True}],
    )
    return LaunchDescription([declare_gui, declare_rviz, gazebo, fusion])
