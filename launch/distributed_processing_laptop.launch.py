"""
Distributed Processing - Laptop Side Launch File
Laptop handles processing and visualization of data from robot
Robot side: Motor control + Camera data streaming to laptop
Laptop side: Teleop control + RTAB-Map processing & visualization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    robot_base_dir = get_package_share_directory('robot_base')
    map_builder_dir = get_package_share_directory('map_builder')
    
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'camera_name',
            default_value='oak',
            description='Name of the camera node (must match robot side)'
        ),
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.1.100',
            description='IP address of the robot for ROS communication'
        ),
        DeclareLaunchArgument(
            'use_rtabmap_viz',
            default_value='true',
            description='Enable RTAB-Map visualization'
        ),
        DeclareLaunchArgument(
            'laptop_namespace',
            default_value='laptop',
            description='Namespace for laptop nodes'
        )
    ]

    return LaunchDescription(declared_arguments + [
        # Keyboard teleop (publishes cmd_vel)
        Node(
            package='robot_base',
            executable='teleop_wasd',
            name='teleop_wasd',
            namespace=LaunchConfiguration('laptop_namespace'),
            output='screen',
            remappings=[
                ('cmd_vel', '/cmd_vel')  # Publish to global cmd_vel topic
            ]
        ),
        
        # RTAB-Map SLAM processing and visualization
        # This will process the camera data received from the robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(map_builder_dir, 'launch', 'rtabmap.launch.py')
            ),
            launch_arguments={
                'name': LaunchConfiguration('camera_name')
            }.items()
        )
    ])