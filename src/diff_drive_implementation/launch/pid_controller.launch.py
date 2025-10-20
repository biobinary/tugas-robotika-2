#!/usr/bin/env python3
"""
Launch file for PID controller (single target navigation) node.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for PID controller."""
    
    # Get package directory
    pkg_share = get_package_share_directory('diff_drive_implementation')
    default_config_path = os.path.join(pkg_share, 'config', 'pid_params.yaml')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Path to PID config file'
    )
    
    target_x_arg = DeclareLaunchArgument(
        'target_x',
        default_value='3.0',
        description='Target X position'
    )
    
    target_y_arg = DeclareLaunchArgument(
        'target_y',
        default_value='4.0',
        description='Target Y position'
    )
    
    target_yaw_arg = DeclareLaunchArgument(
        'target_yaw',
        default_value='-1.56',
        description='Target yaw orientation (radians)'
    )
    
    # PID controller node
    pid_controller_node = Node(
        package='diff_drive_implementation',
        executable='pid_controller',
        name='pid_controller',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'target_x': LaunchConfiguration('target_x'),
                'target_y': LaunchConfiguration('target_y'),
                'target_yaw': LaunchConfiguration('target_yaw'),
            }
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        target_x_arg,
        target_y_arg,
        target_yaw_arg,
        pid_controller_node,
    ])

