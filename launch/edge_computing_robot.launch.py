"""
Edge Computing - Robot Side Launch File
Robot handles all processing autonomously, minimal laptop dependency
Robot side: Motor control + Camera + Complete RTAB-Map processing
Laptop side: Teleop control only (+ optional map viewing)
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
    depthai_ros_driver_dir = get_package_share_directory('depthai_ros_driver')
    
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'camera_name',
            default_value='oak',
            description='Name of the camera node'
        ),
        DeclareLaunchArgument(
            'max_duty',
            default_value='1000',
            description='Maximum duty cycle for motor control'
        ),
        DeclareLaunchArgument(
            'camera_params_file',
            default_value=os.path.join(depthai_ros_driver_dir, 'config', 'rgbd.yaml'),
            description='Camera configuration file path optimized for RTAB-Map'
        ),
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='robot',
            description='Namespace for robot nodes'
        ),
        DeclareLaunchArgument(
            'enable_rtabmap_viz',
            default_value='false',
            description='Enable RTAB-Map visualization on robot (usually false for headless robots)'
        )
    ]

    return LaunchDescription(declared_arguments + [
        # Motor control node (listens to cmd_vel)
        Node(
            package='robot_base',
            executable='motor_node',
            name='motor_node',
            namespace=LaunchConfiguration('robot_namespace'),
            output='screen',
            parameters=[{
                'max_duty': LaunchConfiguration('max_duty')
            }],
            remappings=[
                ('cmd_vel', '/cmd_vel')  # Listen to global cmd_vel topic
            ]
        ),
        
        # Complete RTAB-Map SLAM processing with camera
        # This includes camera driver + RTAB-Map odometry + RTAB-Map SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_ros_driver_dir, 'launch', 'rtabmap.launch.py')
            ),
            launch_arguments={
                'name': LaunchConfiguration('camera_name'),
                'params_file': LaunchConfiguration('camera_params_file')
            }.items()
        )
    ])