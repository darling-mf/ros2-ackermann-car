import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('car_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'car.urdf.xacro')
    default_world_path = os.path.join(pkg_path, 'world', 'custom_room.world')
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'display.rviz')

    #模型
    model_path = os.path.join(pkg_path, 'world', 'room')

    # 解析 xacro
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toxml()
    
     # 清理并设置 Gazebo 模型路径
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=model_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    # 1. 启动 Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        # 传递参数
        launch_arguments=[('world', default_world_path),('verbose','true')]
    )

    # 2. 发布机器人状态
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}]
    )

    # 3. 在 Gazebo 中生成小车
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'ackermann_car'],
        output='screen'
    )

    # 4. 启动 RViz2（可选，方便看里程计数据）
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]  # 加载你保存的 RViz 配置文件
    )

    return LaunchDescription([
        set_gazebo_model_path,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        rviz
    ])