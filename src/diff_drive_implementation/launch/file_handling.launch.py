#!/usr/bin/env python3
"""
Launch file for file handling (multi-waypoint navigation) node.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for file handling navigator."""
    
    # Get package directory
    pkg_share = get_package_share_directory('diff_drive_implementation')
    default_config_path = os.path.join(pkg_share, 'config', 'navigator_params.yaml')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Path to navigator config file'
    )
    
    target_file_arg = DeclareLaunchArgument(
        'target_file',
        default_value='data/target.txt',
        description='Path to target waypoints file'
    )
    
    # File handling navigator node
    file_handling_node = Node(
        package='diff_drive_implementation',
        executable='file_handling',
        name='file_handling_navigator',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'target_file': LaunchConfiguration('target_file'),
            }
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        target_file_arg,
        file_handling_node,
    ])

