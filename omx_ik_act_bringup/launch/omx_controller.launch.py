#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    controller_type = LaunchConfiguration('controller_type')
    start_interactive_marker = LaunchConfiguration('start_interactive_marker')

    default_config_file = PathJoinSubstitution([
        FindPackageShare('omx_ik_act_bringup'),
        'config',
        'omx_config.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_file,
            description='Path to the OMX IK/ACT controller config file.',
        ),
        DeclareLaunchArgument(
            'controller_type',
            default_value='movel',
            description='Controller type passed to cyclo OMX launch.',
        ),
        DeclareLaunchArgument(
            'start_interactive_marker',
            default_value='false',
            description='Start interactive marker for marker-follow mode.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('cyclo_motion_controller_ros'),
                    'launch',
                    'omx_controller.launch.py',
                ])
            ),
            launch_arguments={
                'config_file': config_file,
                'controller_type': controller_type,
                'start_interactive_marker': start_interactive_marker,
            }.items(),
        ),
    ])
