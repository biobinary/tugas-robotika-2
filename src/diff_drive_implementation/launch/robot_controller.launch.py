#!/usr/bin/env python3
"""
Launch file for robot controller (keyboard teleop) node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for robot controller."""
    
    robot_controller_node = Node(
        package='diff_drive_implementation',
        executable='robot_controller',
        name='robot_controller',
        output='screen',
        emulate_tty=True,
        prefix='xterm -e',  # Run in separate terminal for keyboard input
    )
    
    return LaunchDescription([
        robot_controller_node,
    ])

