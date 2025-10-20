# ROS 2 Assignment 1

## 👥 Team Members

| Name                          | NRP        |
| ----------------------------- | ---------- |
| Muhammad Ammar Ghifari        | 5025231109 |
| Glenn Muhammad Rooney         | 5025231112 |
| Muhamad Baihaqi Dawanis       | 5025231177 |
| Muhammad Risyad Himawan Putra | 5025231205 |
| Naswan Nashir Ramadhan        | 5025231246 |
| Faizal Aldy Armiriawan        | 5025231266 |

## 📋 Features

- **PID Controller**: Single target navigation with proportional-integral control
- **Multi-Waypoint Navigator**: Sequential waypoint navigation from file
- **Keyboard Teleop**: Manual robot control via keyboard
- **Gazebo Simulation**: Full physics simulation with sensors (LiDAR, IMU, Depth Camera)
- **Robot Localization**: EKF-based sensor fusion for accurate odometry

## 🏗️ Project Structure

```
tugas-robotika-2/
├── config/              # YAML configuration files
│   ├── pid_params.yaml
│   ├── navigator_params.yaml
│   └── ekf.yaml
├── data/                # Target waypoint files
├── description/         # URDF robot description
├── launch/              # Launch files
│   ├── display.launch.py          # Main simulation
│   ├── pid_controller.launch.py   # Single target navigation
│   ├── file_handling.launch.py    # Multi-waypoint navigation
│   └── robot_controller.launch.py # Keyboard teleop
├── maps/                # Map files for navigation
├── models/              # Gazebo models
├── rviz/                # RViz configurations
├── src/                 # Python source code
│   └── diff_drive_implementation/
│       ├── __init__.py
│       ├── pid_controller.py
│       ├── file_handling.py
│       └── robot_controller.py
└── worlds/              # Gazebo world files
```

## 🔧 Installation

### Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Ignition

### Install Dependencies

```bash
sudo rosdep init  # Skip if already initialized
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Build Package

```bash
cd ~/ros_projects/tugas-robotika-2
colcon build --symlink-install
source install/setup.bash
```

## 🚀 Usage

### 1. Launch Simulation Environment

Start Gazebo, RViz, and robot localization:

```bash
ros2 launch diff_drive_implementation display.launch.py
```

### 2. Navigation Modes

#### A. Single Target Navigation (PID Controller)

Navigate to a single target position with specific orientation:

```bash
# Default target (x=3.0, y=4.0, yaw=-1.56)
ros2 launch diff_drive_implementation pid_controller.launch.py

# Custom target
ros2 launch diff_drive_implementation pid_controller.launch.py \
    target_x:=5.0 target_y:=3.0 target_yaw:=1.57
```

#### B. Multi-Waypoint Navigation

Navigate through multiple waypoints from a file:

```bash
# Default waypoints (data/target.txt)
ros2 launch diff_drive_implementation file_handling.launch.py

# Custom waypoint file
ros2 launch diff_drive_implementation file_handling.launch.py \
    target_file:=data/my_waypoints.txt
```

**Waypoint File Format:**

```
# target_x  target_y  target_yaw
1  2.0  3.0  0.0
2  5.0  5.0  1.57
3  8.0  2.0  -1.57
```

#### C. Manual Control (Keyboard Teleop)

Control the robot manually using keyboard:

```bash
ros2 launch diff_drive_implementation robot_controller.launch.py
```

**Keyboard Controls:**

```
   u    i    o       Movement:
   j    k    l       u/i/o: Forward left/straight/right
   m    ,    .       j/k/l: Rotate left/stop/rotate right
                     m/,/.: Backward left/straight/right

Speed Control:
  q/z: Increase/decrease linear speed
  w/x: Increase/decrease angular speed
```

## ⚙️ Configuration

### PID Parameters

Edit `config/pid_params.yaml` or `config/navigator_params.yaml`:

```yaml
/**:
  ros__parameters:
    # PID Gains
    kp: 1.0              # Proportional gain
    ki: 0.1              # Integral gain
    kd: 0.05             # Derivative gain
  
    # Velocity Limits
    max_linear_vel: 0.8
    max_angular_vel: 1.5
  
    # Tolerances
    angle_tolerance: 0.05         # radians
    distance_tolerance: 0.5       # meters
    final_angle_tolerance: 0.1    # radians
  
    # State Transition
    transition_delay: 1.0         # seconds
```

### Custom Configuration File

```bash
ros2 launch diff_drive_implementation pid_controller.launch.py \
    config_file:=/path/to/custom_config.yaml
```

## 🤖 Robot Specifications

- **Type**: Differential Drive
- **Dimensions**: 0.42m (L) × 0.31m (W) × 0.18m (H)
- **Wheel Radius**: 0.10m
- **Wheel Separation**: 0.36m

**Sensors:**

- LiDAR: 180° FOV, 20m range
- IMU: 100Hz update rate
- Depth Camera: 640×480, 30Hz

## 🐛 Troubleshooting

### Robot doesn't move

- Check if `use_sim_time` is properly set
- Verify cmd_vel topic: `ros2 topic echo /cmd_vel`
- Check Gazebo bridge: `ros2 topic list | grep cmd_vel`

### Navigation overshoots target

- Decrease PID gains (especially `kp`)
- Increase `distance_tolerance`
- Check for wheel slippage (adjust friction in URDF)

### Angle wrap-around issues

- Ensure using latest version with normalized angle fixes
- Check `angle_tolerance` values

## 📊 Topics

| Topic        | Type                      | Description       |
| ------------ | ------------------------- | ----------------- |
| `/cmd_vel` | `geometry_msgs/Twist`   | Velocity commands |
| `/odom`    | `nav_msgs/Odometry`     | Odometry data     |
| `/scan`    | `sensor_msgs/LaserScan` | LiDAR data        |
| `/imu`     | `sensor_msgs/Imu`       | IMU data          |
| `/tf`      | `tf2_msgs/TFMessage`    | Transform tree    |

## 📝 License

MIT License - see LICENSE file for details.

## 🙏 Acknowledgments

- ROS2 Humble Documentation
- Gazebo Ignition
- Robot Localization Package
