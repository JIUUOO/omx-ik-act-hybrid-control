#!/usr/bin/env python3

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _load_launch_arguments(config_path):
    with open(config_path, 'r', encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file) or {}

    args = config.get('yolo', {}).get('launch_arguments', {})
    return {key: str(value) for key, value in args.items()}


def generate_launch_description():
    def include_yolo(context: LaunchContext):
        resolved_config_file = context.perform_substitution(config_file)
        yolo_args = _load_launch_arguments(resolved_config_file)

        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('yolo_bringup'),
                        'launch',
                        'yolo.launch.py',
                    ])
                ),
                launch_arguments=yolo_args.items(),
            )
        ]

    default_config_file = PathJoinSubstitution([
        FindPackageShare('omx_ik_act_bringup'),
        'config',
        'yolo.yaml',
    ])

    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_file,
            description='Path to the OMX IK/ACT YOLO launch argument YAML file.',
        ),
        OpaqueFunction(function=include_yolo),
    ])
