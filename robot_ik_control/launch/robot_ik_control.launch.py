import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('robot_ik_control'),
        'config',
        'robot_ik_control.yaml'
    )

    robot_ik_control_node = Node(
        package='robot_ik_control',
        executable='robot_ik_control_node',
        name='robot_ik_control_node',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        robot_ik_control_node
    ])
