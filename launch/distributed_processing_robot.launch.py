"""
Distributed Processing - Robot Side Launch File
Robot handles data capture and streaming, laptop handles processing
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
    oakd_driver_dir = get_package_share_directory('oakd_driver')
    
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
            default_value=os.path.join(oakd_driver_dir, 'config', 'camera.yaml'),
            description='Camera configuration file path'
        ),
        DeclareLaunchArgument(
            'enable_pointcloud',
            default_value='false',
            description='Enable point cloud generation'
        ),
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='robot',
            description='Namespace for robot nodes'
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
        
        # Camera node for data streaming
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(oakd_driver_dir, 'launch', 'camera.launch.py')
            ),
            launch_arguments={
                'name': LaunchConfiguration('camera_name'),
                'params_file': LaunchConfiguration('camera_params_file'),
                'use_rviz': 'false',  # No visualization on robot side
                'pointcloud.enable': LaunchConfiguration('enable_pointcloud'),
                'rectify_rgb': 'true',
                'publish_tf_from_calibration': 'true',
                'rs_compat': 'false'
            }.items()
        )
    ])