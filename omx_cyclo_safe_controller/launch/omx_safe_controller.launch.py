#!/usr/bin/env python3

"""Launch the OMX startup-relaxed MoveL controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Create the safe MoveL launch description."""
    declared_arguments = [
        DeclareLaunchArgument(
            'base_frame',
            default_value='link0',
            description='Base frame used by the OMX controller.',
        ),
        DeclareLaunchArgument(
            'controlled_link',
            default_value='end_effector_link',
            description='Controlled end-effector link.',
        ),
        DeclareLaunchArgument(
            'joint_states_topic',
            default_value='/joint_states',
            description='Joint-state feedback topic.',
        ),
        DeclareLaunchArgument(
            'joint_command_topic',
            default_value='/leader/joint_trajectory',
            description='Arm trajectory command topic.',
        ),
        DeclareLaunchArgument(
            'movel_topic',
            default_value='/omx_movel_controller/movel',
            description='MoveL command topic.',
        ),
        DeclareLaunchArgument(
            'urdf_path',
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare('cyclo_motion_controller_models'),
                    'models',
                    'omx',
                    'omx_f.urdf',
                ]
            ),
            description='Cyclo collision URDF path.',
        ),
        DeclareLaunchArgument(
            'srdf_path',
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare('cyclo_motion_controller_models'),
                    'models',
                    'omx',
                    'omx_f.srdf',
                ]
            ),
            description='Cyclo collision SRDF path.',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare('omx_cyclo_safe_controller'),
                    'config',
                    'omx_safe_config.yaml',
                ]
            ),
            description='Safe MoveL controller parameter file.',
        ),
    ]

    node = Node(
        package='omx_cyclo_safe_controller',
        executable='omx_safe_movel_controller_node',
        name='omx_movel_controller',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'urdf_path': LaunchConfiguration('urdf_path'),
                'srdf_path': LaunchConfiguration('srdf_path'),
                'base_frame': LaunchConfiguration('base_frame'),
                'controlled_link': LaunchConfiguration('controlled_link'),
                'joint_states_topic': LaunchConfiguration('joint_states_topic'),
                'joint_command_topic': LaunchConfiguration('joint_command_topic'),
                'movel_topic': LaunchConfiguration('movel_topic'),
            },
        ],
        output='screen',
    )

    return LaunchDescription(declared_arguments + [node])
