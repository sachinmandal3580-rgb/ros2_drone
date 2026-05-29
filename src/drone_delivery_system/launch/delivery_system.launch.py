#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ns = LaunchConfiguration('drone_namespace', default='simple_drone')
    return LaunchDescription([
        DeclareLaunchArgument('drone_namespace', default_value='simple_drone'),

        Node(
            package='drone_delivery_system',
            executable='dual_camera_yolo_detector',
            name='dual_camera_yolo_detector',
            output='screen',
            parameters=[{
                'front_camera_topic':  '/simple_drone/front/image_raw',
                'bottom_camera_topic': '/simple_drone/bottom/image_raw',
                'confidence_threshold': 0.45,
                'target_class':        'person',
                'publish_annotated':   True,
            }],
        ),

        Node(
            package='drone_delivery_system',
            executable='enhanced_delivery_controller',
            name='delivery_controller',
            output='screen',
            parameters=[{
                'drone_namespace': ns,
                'cube_x':          2.0,
                'cube_y':          1.5,
                'fly_alt':         3.0,
                'search_alt':      2.0,
                'drop_alt':        0.5,
                'sonar_topic':    '/simple_drone/sonar',
                'odom_topic':     '/simple_drone/odom',
            }],
        ),

        Node(
            package='drone_delivery_system',
            executable='mission_planner',
            name='mission_planner',
            output='screen',
        ),
    ])