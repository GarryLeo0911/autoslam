"""
Edge Computing - Laptop Side Launch File
Minimal laptop role - only optional monitoring
Robot side: Motor control + Camera + Complete RTAB-Map processing
Laptop side: Optional map viewing (run teleop separately)
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

    nodes = []
    
    # Optional: Simple RViz configuration to view the map and robot status
    # This is much lighter than full RTAB-Map visualization
    # Run teleop separately: ros2 run robot_base teleop_wasd --ros-args -r cmd_vel:=/cmd_vel
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