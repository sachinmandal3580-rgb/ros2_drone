#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('drone_delivery_system'),
        'config', 'delivery_params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('drone_namespace', default_value='simple_drone'),

        Node(
            package='drone_delivery_system',
            executable='dual_camera_yolo_detector',
            name='dual_camera_yolo_detector',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='drone_delivery_system',
            executable='payload_manager',
            name='payload_manager',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='drone_delivery_system',
            executable='enhanced_delivery_controller',
            name='delivery_controller',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='drone_delivery_system',
            executable='mission_planner',
            name='mission_planner',
            output='screen',
        ),
    ])