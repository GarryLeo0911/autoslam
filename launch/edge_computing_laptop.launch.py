"""
Edge Computing - Laptop Side Launch File
Minimal laptop role - only teleop control and optional monitoring
Robot side: Motor control + Camera + Complete RTAB-Map processing
Laptop side: Teleop control only (+ optional map viewing)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.1.100',
            description='IP address of the robot for ROS communication'
        ),
        DeclareLaunchArgument(
            'laptop_namespace',
            default_value='laptop',
            description='Namespace for laptop nodes'
        ),
        DeclareLaunchArgument(
            'enable_map_viewer',
            default_value='true',
            description='Enable simple map viewer to see the map built by robot'
        )
    ]

    nodes = [
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
        )
    ]
    
    # Optional: Simple RViz configuration to view the map and robot status
    # This is much lighter than full RTAB-Map visualization
    nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_map_viewer',
            namespace=LaunchConfiguration('laptop_namespace'),
            output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory('autoslam'), 
                'config', 
                'edge_computing_laptop.rviz'
            )],
            condition=lambda context: LaunchConfiguration('enable_map_viewer').perform(context) == 'true'
        )
    )

    return LaunchDescription(declared_arguments + nodes)