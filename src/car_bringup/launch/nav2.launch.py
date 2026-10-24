import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('car_bringup')
    map_yaml_file = os.path.join(pkg_path, 'maps', 'my_room_map.yaml')

    # 1. 启动 Nav2 导航栈
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ]),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': 'true',
            'slam': 'False' # 禁用建图，因为我们是用保存好的地图导航
        }.items()
    )

    # 2. 启动统一的 RViz (配置好 Nav2 默认视图)
    rviz_config = os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        nav2_bringup,
        rviz_node
    ])