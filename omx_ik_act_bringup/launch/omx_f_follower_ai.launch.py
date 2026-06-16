#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'start_rviz',
            default_value='false',
            description='Whether to execute rviz2.',
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='Prefix of the joint and link names.',
        ),
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Start robot in Gazebo simulation.',
        ),
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='Use mock hardware mirroring command.',
        ),
        DeclareLaunchArgument(
            'mock_sensor_commands',
            default_value='false',
            description='Enable mock sensor commands.',
        ),
        DeclareLaunchArgument(
            'init_position',
            default_value='true',
            description='Whether to launch the init_position node.',
        ),
        DeclareLaunchArgument(
            'ros2_control_type',
            default_value='omx_f',
            description='Type of ros2_control.',
        ),
        DeclareLaunchArgument(
            'port_name',
            default_value=(
                '/dev/serial/by-id/'
                'usb-ROBOTIS_OpenRB-150_BA2D16185157375037202020FF121710-if00'
            ),
            description='Port name for the hardware interface.',
        ),
        DeclareLaunchArgument(
            'initial_positions_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('omx_ik_act_bringup'),
                'config',
                'omx_f_follower_ai_initial_positions.yaml',
            ]),
            description='Path to the OMX IK/ACT initial positions file.',
        ),
    ]

    start_rviz = LaunchConfiguration('start_rviz')
    prefix = LaunchConfiguration('prefix')
    use_sim = LaunchConfiguration('use_sim')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    mock_sensor_commands = LaunchConfiguration('mock_sensor_commands')
    init_position = LaunchConfiguration('init_position')
    ros2_control_type = LaunchConfiguration('ros2_control_type')
    port_name = LaunchConfiguration('port_name')
    initial_positions_file = LaunchConfiguration('initial_positions_file')

    urdf_file = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('open_manipulator_description'),
            'urdf',
            'omx_f',
            'omx_f.urdf.xacro',
        ]),
        ' ',
        'prefix:=',
        prefix,
        ' ',
        'use_sim:=',
        use_sim,
        ' ',
        'use_mock_hardware:=',
        use_mock_hardware,
        ' ',
        'mock_sensor_commands:=',
        mock_sensor_commands,
        ' ',
        'ros2_control_type:=',
        ros2_control_type,
        ' ',
        'port_name:=',
        port_name,
    ])

    controller_manager_config = PathJoinSubstitution([
        FindPackageShare('open_manipulator_bringup'),
        'config',
        'omx_f_follower_ai',
        'hardware_controller_manager.yaml',
    ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('open_manipulator_description'),
        'rviz',
        'open_manipulator.rviz',
    ])

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': urdf_file}, controller_manager_config],
        output='both',
        condition=UnlessCondition(use_sim),
        remappings=[('/arm_controller/joint_trajectory', '/leader/joint_trajectory')],
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            'joint_state_broadcaster',
        ],
        output='both',
        parameters=[{'robot_description': urdf_file}],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf_file, 'use_sim_time': use_sim}],
        output='both',
    )

    joint_trajectory_executor = Node(
        package='open_manipulator_bringup',
        executable='joint_trajectory_executor',
        parameters=[initial_positions_file],
        output='both',
        condition=IfCondition(init_position),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='both',
        condition=IfCondition(start_rviz),
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[rviz_node],
        )
    )

    delay_joint_trajectory_executor_after_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_trajectory_executor],
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            control_node,
            robot_controller_spawner,
            robot_state_publisher_node,
            delay_rviz_after_joint_state_broadcaster_spawner,
            delay_joint_trajectory_executor_after_controllers,
        ]
    )
