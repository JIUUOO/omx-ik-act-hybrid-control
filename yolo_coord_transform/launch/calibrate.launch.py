import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('yolo_coord_transform')
    config_file = os.path.join(pkg_dir, 'config', 'calibration_params.yaml')

    node = Node(
        package='yolo_coord_transform',
        executable='calibration_node',
        name='calibration_node',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        SetEnvironmentVariable('PYTHONNOUSERSITE', '1'),
        node
    ])
