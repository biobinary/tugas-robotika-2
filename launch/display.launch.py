import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('tugas-robotika')
    default_model_path = os.path.join(pkg_share, 'description/diff_drive_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/navigation.rviz')
    world_path=os.path.join(pkg_share, 'worlds/my_world.sdf')
    models_path = os.path.join(pkg_share, 'models')
    
    # Set environment variable for Gazebo to find models
    env = os.environ.copy()
    if 'IGN_GAZEBO_RESOURCE_PATH' in env:
        env['IGN_GAZEBO_RESOURCE_PATH'] = env['IGN_GAZEBO_RESOURCE_PATH'] + ':' + models_path
    else:
        env['IGN_GAZEBO_RESOURCE_PATH'] = models_path
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
 
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'bot', 
                   '-topic', 'robot_description', 
                   '-x', '5.5', 
                   '-y', '5.7',
                   '-z', '0.15',
                   '-Y', '-1.57'
                  ],
        output='screen'
    )
    
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Bridge for sensor topics from Gazebo to ROS2
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
        output='screen'
    )
    
    # Bridge for joint_states
    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        ExecuteProcess(cmd=['ign', 'gazebo', '-r', world_path], output='screen', additional_env=env),
        robot_state_publisher_node,
        spawn_entity,
        bridge_params,
        joint_state_bridge,
        robot_localization_node,
        rviz_node,
    ])
