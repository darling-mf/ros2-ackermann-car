import os
import re
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit
import xacro


def generate_launch_description():
    pkg_path = get_package_share_directory('car_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'car.urdf.xacro')
    default_world_path = os.path.join(pkg_path, 'world', 'custom_room.world')
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'display.rviz')

    # Gazebo 模型目录（world 里若使用 model:// 需要它）
    model_path = os.path.join(pkg_path, 'world')

    gui = LaunchConfiguration('gui')
    rviz_enabled = LaunchConfiguration('rviz')

    # 解析 xacro
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toxml()

    # gazebo_ros2_control 会把整段 robot_description 拼成 "--param robot_description:=..."
    # 再交给 rcl 解析；URDF 注释里的特殊字符（冒号、URL 等）会让解析器报
    # "Couldn't parse parameter override rule"。这里统一去掉 XML 注释。
    robot_desc = re.sub(r'<!--[\s\S]*?-->', '', robot_desc)

    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Start Gazebo client (GUI)')
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Start RViz2')

    # 清理并设置 Gazebo 模型路径
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=model_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    # 1. 启动 Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments=[('world', default_world_path), ('verbose', 'true'), ('gui', gui)]
    )

    # 2. 发布机器人状态
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # 3. 在 Gazebo 中生成小车
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'ackermann_car', '-z', '0.02'],
        output='screen'
    )

    # 4. 加载并激活 joint_state_broadcaster
    load_joint_state_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "60"],
        output="screen",
    )

    # 5. 加载并激活 R5 阿克曼控制器（fork 自 ackermann_steering_controller）
    load_ackermann_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "r5_steering_controller",
            "--controller-manager-timeout", "60",
            "--param-file", os.path.join(get_package_share_directory('car_bringup'), 'config', 'controllers.yaml'),
            "--activate",
        ],
        output="screen",
    )

    # 6. 等车生成完，再加载控制器
    delay_load_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_controller],
        )
    )

    delay_load_ackermann = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_ackermann_controller],
        )
    )

    # 7. RViz2（可关：rviz:=false）
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz_enabled),
    )

    return LaunchDescription([
        declare_gui,
        declare_rviz,
        set_gazebo_model_path,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        delay_load_jsb,
        delay_load_ackermann,
        rviz,
    ])
