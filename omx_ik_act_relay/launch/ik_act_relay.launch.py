import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('omx_ik_act_relay')
    default_config_file = os.path.join(
        pkg_dir,
        'config',
        'ik_act_relay.yaml',
    )

    config_file = LaunchConfiguration('config_file')

    ik_act_relay_node = Node(
        package='omx_ik_act_relay',
        executable='ik_act_relay',
        name='ik_act_relay',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_file,
            description='Path to the IK/ACT relay parameter YAML file',
        ),
        ik_act_relay_node,
    ])
