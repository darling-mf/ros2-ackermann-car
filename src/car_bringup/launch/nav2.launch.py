import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory('car_bringup')
    map_yaml_file = os.path.join(pkg_path, 'maps', 'my_room_map.yaml')
    params_path = os.path.join(pkg_path, 'config', 'nav2_params.yaml')
    bt_dir = os.path.join(
        get_package_share_directory('car_navigation'), 'behavior_trees')

    # ---- 关键修复：原 launch 没有把 nav2_params.yaml 传给 Nav2（一直在用默认参数）。
    # 这里读取 YAML，注入阿克曼专用行为树路径，再写成临时 params_file。----
    with open(params_path, 'r', encoding='utf-8') as f:
        params = yaml.safe_load(f)
    bt_params = params.setdefault('bt_navigator', {}).setdefault(
        'ros__parameters', {})
    bt_params['default_nav_to_pose_bt_xml'] = os.path.join(
        bt_dir, 'akm_navigate_to_pose.xml')
    bt_params['default_nav_through_poses_bt_xml'] = os.path.join(
        bt_dir, 'akm_navigate_through_poses.xml')

    merged = tempfile.NamedTemporaryFile(
        mode='w', suffix='_nav2_params.yaml', delete=False)
    yaml.safe_dump(params, merged, allow_unicode=True)
    merged.close()
    params_file = merged.name

    rviz_enabled = LaunchConfiguration('rviz')
    map_yaml = LaunchConfiguration('map')
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true', description='Start RViz2')
    declare_map = DeclareLaunchArgument(
        'map', default_value=map_yaml_file,
        description='地图 yaml 路径（建图保存后可换）')

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch', 'bringup_launch.py')
        ]),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': 'true',
            'slam': 'False',
            'params_file': params_file,
            'autostart': 'true',
        }.items()
    )

    rviz_config = os.path.join(
        get_package_share_directory('nav2_bringup'), 'rviz',
        'nav2_default_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(rviz_enabled),
    )

    return LaunchDescription([
        declare_rviz,
        declare_map,
        nav2_bringup,
        rviz_node,
    ])
