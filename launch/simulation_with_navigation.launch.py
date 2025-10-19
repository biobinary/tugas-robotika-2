#!/usr/bin/env python3
"""
All-in-one launch file: Simulation + Navigation
Launches Gazebo, RViz, robot localization, and navigation controller.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for complete simulation with navigation."""
    
    pkg_share = get_package_share_directory('tugas_robotika')
    
    # Paths
    default_model_path = os.path.join(pkg_share, 'description/diff_drive_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/navigation.rviz')
    world_path = os.path.join(pkg_share, 'worlds/my_world.sdf')
    models_path = os.path.join(pkg_share, 'models')
    default_pid_config = os.path.join(pkg_share, 'config/pid_params.yaml')
    default_nav_config = os.path.join(pkg_share, 'config/navigator_params.yaml')
    
    # Set environment variable for Gazebo to find models
    env = os.environ.copy()
    if 'IGN_GAZEBO_RESOURCE_PATH' in env:
        env['IGN_GAZEBO_RESOURCE_PATH'] = env['IGN_GAZEBO_RESOURCE_PATH'] + ':' + models_path
    else:
        env['IGN_GAZEBO_RESOURCE_PATH'] = models_path
    
    # Launch Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time'
    )
    
    navigation_mode_arg = DeclareLaunchArgument(
        'navigation_mode',
        default_value='single',
        choices=['single', 'multi', 'none'],
        description='Navigation mode: single (PID controller), multi (waypoint navigator), none (no auto navigation)'
    )
    
    target_x_arg = DeclareLaunchArgument(
        'target_x',
        default_value='3.0',
        description='Target X position (single mode only)'
    )
    
    target_y_arg = DeclareLaunchArgument(
        'target_y',
        default_value='4.0',
        description='Target Y position (single mode only)'
    )
    
    target_yaw_arg = DeclareLaunchArgument(
        'target_yaw',
        default_value='-1.56',
        description='Target yaw orientation (single mode only)'
    )
    
    target_file_arg = DeclareLaunchArgument(
        'target_file',
        default_value='data/target.txt',
        description='Target waypoints file (multi mode only)'
    )
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', default_model_path]),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Gazebo
    gazebo_process = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path],
        output='screen',
        additional_env=env
    )
    
    # Spawn Entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'bot',
            '-topic', 'robot_description',
            '-x', '5.5',
            '-y', '5.7',
            '-z', '0.15',
            '-Y', '-1.57'
        ],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Robot Localization
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config/ekf.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # Gazebo Bridge
    bridge_params = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/depth_camera@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        ],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # PID Controller (single target mode)
    pid_controller_node = Node(
        package='tugas_robotika',
        executable='pid_controller',
        name='pid_controller',
        output='screen',
        condition=IfCondition("$(eval '\'$(var navigation_mode)\' == \'single\'')"),
        parameters=[
            default_pid_config,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'target_x': LaunchConfiguration('target_x'),
                'target_y': LaunchConfiguration('target_y'),
                'target_yaw': LaunchConfiguration('target_yaw'),
            }
        ]
    )
    
    # Waypoint Navigator (multi target mode)
    navigator_node = Node(
        package='tugas_robotika',
        executable='file_handling',
        name='file_handling_navigator',
        output='screen',
        condition=IfCondition("$(eval '\'$(var navigation_mode)\' == \'multi\'')"),
        parameters=[
            default_nav_config,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'target_file': LaunchConfiguration('target_file'),
            }
        ]
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        navigation_mode_arg,
        target_x_arg,
        target_y_arg,
        target_yaw_arg,
        target_file_arg,
        
        # Simulation nodes
        gazebo_process,
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity,
        bridge_params,
        robot_localization_node,
        rviz_node,
        
        # Navigation nodes (conditional)
        pid_controller_node,
        navigator_node,
    ])

