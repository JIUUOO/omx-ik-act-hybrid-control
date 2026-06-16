#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def usb_camera_node(name, params_file):
    return Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name=name,
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('omx_ik_act_bringup'),
                'config',
                params_file,
            ]),
        ],
        remappings=[
            ('image_raw', f'{name}/image_raw'),
            ('image_raw/compressed', f'{name}/image_raw/compressed'),
            ('image_raw/compressedDepth', f'{name}/image_raw/compressedDepth'),
            ('image_raw/theora', f'{name}/image_raw/theora'),
            ('camera_info', f'{name}/camera_info'),
        ],
    )


def generate_launch_description():
    return LaunchDescription([
        usb_camera_node('camera1', 'params_1.yaml'),
        usb_camera_node('camera2', 'params_2.yaml'),
        usb_camera_node('camera3', 'params_3.yaml'),
    ])
