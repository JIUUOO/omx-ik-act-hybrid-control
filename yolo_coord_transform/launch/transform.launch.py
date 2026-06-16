import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('yolo_coord_transform')
    
    # Path to the parameters YAML file
    default_config_file = os.path.join(pkg_dir, 'config', 'transform_params.yaml')
    config_file = LaunchConfiguration('config_file')

    node = Node(
        package='yolo_coord_transform',
        executable='yolo_coord_transform_node',
        name='yolo_coord_transform_node',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_file,
            description='Path to the yolo_coord_transform parameter YAML file',
        ),
        node
    ])
