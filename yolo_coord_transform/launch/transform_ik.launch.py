import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('yolo_coord_transform')
    robot_ik_pkg_dir = get_package_share_directory('robot_ik_control')

    # Path to the parameters YAML file
    default_config_file = os.path.join(pkg_dir, 'config', 'transform_params.yaml')
    default_robot_ik_config_file = os.path.join(
        robot_ik_pkg_dir,
        'config',
        'robot_ik_control.yaml',
    )
    config_file = LaunchConfiguration('config_file')
    robot_ik_config_file = LaunchConfiguration('robot_ik_config_file')

    yolo_coord_transform_node = Node(
        package='yolo_coord_transform',
        executable='yolo_coord_transform_node',
        name='yolo_coord_transform_node',
        output='screen',
        parameters=[config_file],
    )

    robot_ik_control_node = Node(
        package='robot_ik_control',
        executable='robot_ik_control_node',
        name='robot_ik_control_node',
        output='screen',
        parameters=[robot_ik_config_file],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_file,
            description='Path to the yolo_coord_transform parameter YAML file',
        ),
        DeclareLaunchArgument(
            'robot_ik_config_file',
            default_value=default_robot_ik_config_file,
            description='Path to the robot_ik_control parameter YAML file',
        ),
        yolo_coord_transform_node,
        robot_ik_control_node,
    ])
