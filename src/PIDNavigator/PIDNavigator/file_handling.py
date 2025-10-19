#!/usr/bin/env python3
"""
File Handling Node - Multi-Waypoint Navigation
Reads multiple target waypoints from file and navigates sequentially.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory
import math
import time
import os


class Target:
    """Data class to store target waypoint information."""
    def __init__(self, no, pos_x, pos_y, pos_yaw):
        self.no = no
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.pos_yaw = pos_yaw
    
    def __repr__(self):
        return f"Target({self.no}, x={self.pos_x}, y={self.pos_y}, yaw={self.pos_yaw})"


class PIDNavigator(Node):
    """
    Multi-waypoint navigator with PID control.
    
    Reads targets from file and navigates through them sequentially.
    Uses three-stage control for each waypoint:
    1. Rotate to face target
    2. Move to target position
    3. Adjust final orientation
    """
    
    def __init__(self):
        super().__init__('file_handling')
        
        # Declare parameters
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.01)
        self.declare_parameter('max_linear_vel', 0.8)
        self.declare_parameter('angle_tolerance', 0.1)
        self.declare_parameter('distance_tolerance', 0.5)
        self.declare_parameter('final_angle_tolerance', 0.1)
        self.declare_parameter('target_file', 'data/target.txt')
        self.declare_parameter('transition_delay', 1.0)
        
        # Get parameters
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value
        self.final_angle_tolerance = self.get_parameter('final_angle_tolerance').value
        self.transition_delay = self.get_parameter('transition_delay').value
        
        # Load targets from file
        target_file = self.get_parameter('target_file').value
        self.targets = self.load_targets(target_file)
        
        if not self.targets:
            self.get_logger().error('No targets loaded! Exiting...')
            return
        
        # Navigation state
        self.current_target_idx = 0
        self.current_target = self.targets[self.current_target_idx]
        
        # State flags
        self.first_pose = False
        self.end_controller = False
        self.final_pose = False
        
        # PID variables
        self.sum_error = 0.0
        
        # State transition timer
        self.state_transition_time = None
        
        # Publisher and Subscriber
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 30)
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.movement_callback,
            30
        )
        
        self.twist_msg = Twist()
        self.twist_msg.linear.x = 0.0
        self.twist_msg.angular.z = 0.0
        
        self.get_logger().info(f'File Handling Navigator initialized with {len(self.targets)} targets')
    
    def load_targets(self, filename):
        """Load target waypoints from file."""
        try:
            # Get package share directory
            package_share = get_package_share_directory('tugas-robotika')
            target_path = os.path.join(package_share, filename)
            
            targets = []
            
            self.get_logger().info(f'Loading targets from: {target_path}')
            
            with open(target_path, 'r') as f:
                # Skip header line
                header = f.readline()
                
                print(f"\n{'='*60}")
                print(f"{'Target':<10}{'X':<10}{'Y':<10}{'Yaw':<10}")
                print(f"{'='*60}")
                
                for line in f:
                    parts = line.strip().split()
                    if len(parts) >= 4:
                        no = int(parts[0])
                        x = float(parts[1])
                        y = float(parts[2])
                        yaw = float(parts[3])
                        
                        target = Target(no, x, y, yaw)
                        targets.append(target)
                        
                        print(f"{no:<10}{x:<10.2f}{y:<10.2f}{yaw:<10.2f}")
                
                print(f"{'='*60}")
                print(f"Total targets loaded: {len(targets)}\n")
            
            return targets
            
        except FileNotFoundError:
            self.get_logger().error(f'Target file not found: {target_path}')
            return []
        except Exception as e:
            self.get_logger().error(f'Error loading targets: {e}')
            return []
    
    def quaternion_to_yaw(self, x, y, z, w):
        """Convert quaternion to yaw angle."""
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi] range."""
        angle = math.fmod(angle, 2 * math.pi)
        if angle > math.pi:
            angle -= 2 * math.pi
        elif angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def movement_callback(self, msg):
        """Main odometry callback for navigation control."""
        if self.current_target_idx >= len(self.targets):
            self.get_logger().info('All targets reached! Mission complete.')
            return
        
        # Extract position
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        
        # Extract orientation (quaternion to yaw)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = self.quaternion_to_yaw(qx, qy, qz, qw)
        
        # Stage 1: Rotate to face target
        if not self.first_pose:
            self.rotate_to_target(pos_x, pos_y, yaw)
        
        # Stage 2: Move to target position
        elif self.first_pose and not self.end_controller:
            self.move_to_target(pos_x, pos_y)
        
        # Stage 3: Final orientation adjustment
        elif self.first_pose and self.end_controller and not self.final_pose:
            self.adjust_final_orientation(pos_x, pos_y, yaw)
    
    def rotate_to_target(self, pos_x, pos_y, current_yaw):
        """Rotate robot to face the target position."""
        target = self.current_target
        target_yaw = math.atan2(target.pos_y - pos_y, target.pos_x - pos_x)
        yaw_error = self.normalize_angle(target_yaw - current_yaw)
        
        # Check if within tolerance (fixed: use normalized error)
        if abs(yaw_error) <= self.angle_tolerance:
            # Use state timer instead of blocking sleep
            if self.state_transition_time is None:
                self.state_transition_time = time.time()
                self.twist_msg.linear.x = 0.0
                self.twist_msg.angular.z = 0.0
                self.publisher_.publish(self.twist_msg)
                self.get_logger().info(f'Berhasil berbelok ke arah tujuan {target.no}, yaw = {current_yaw:.2f}, menunggu stabilisasi...')
            elif time.time() - self.state_transition_time >= self.transition_delay:
                self.first_pose = True
                self.state_transition_time = None
                self.get_logger().info(f'Transisi ke fase movement untuk target {target.no}')
        else:
            # Reset timer if not within tolerance
            self.state_transition_time = None
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = yaw_error
            self.publisher_.publish(self.twist_msg)
    
    def move_to_target(self, pos_x, pos_y):
        """Move robot to target position using PID control."""
        target = self.current_target
        distance = math.sqrt((target.pos_y - pos_y)**2 + (target.pos_x - pos_x)**2)
        self.get_logger().info(f'Target {target.no}: Jarak sampai tujuan: {distance:.3f} m')
        
        # PID control (P only for stability)
        error = distance
        self.sum_error += error
        
        p = self.kp * error
        
        velocity = p  # + i
        
        # Limit velocity
        if abs(velocity) > self.max_linear_vel:
            velocity = self.max_linear_vel if velocity > 0 else -self.max_linear_vel
        
        self.twist_msg.linear.x = velocity
        self.twist_msg.angular.z = 0.0
        self.publisher_.publish(self.twist_msg)
        
        if distance < self.distance_tolerance:
            # Use state timer instead of blocking sleep
            if self.state_transition_time is None:
                self.state_transition_time = time.time()
                self.twist_msg.linear.x = 0.0
                self.twist_msg.angular.z = 0.0
                self.publisher_.publish(self.twist_msg)
                self.get_logger().info(f'Robot telah sampai target {target.no}, menunggu stabilisasi...')
            elif time.time() - self.state_transition_time >= self.transition_delay:
                self.end_controller = True
                self.sum_error = 0.0
                self.state_transition_time = None
                self.get_logger().info(f'Transisi ke fase orientasi final untuk target {target.no}')
        else:
            # Reset timer if not within tolerance
            self.state_transition_time = None
    
    def adjust_final_orientation(self, pos_x, pos_y, current_yaw):
        """Adjust robot to final target orientation."""
        target = self.current_target
        yaw_error = self.normalize_angle(target.pos_yaw - current_yaw)
        
        # Check if within tolerance (fixed: use normalized error)
        if abs(yaw_error) <= self.final_angle_tolerance:
            # Use state timer instead of blocking sleep
            if self.state_transition_time is None:
                self.state_transition_time = time.time()
                self.twist_msg.linear.x = 0.0
                self.twist_msg.angular.z = 0.0
                self.publisher_.publish(self.twist_msg)
                self.get_logger().info(
                    f'Target {target.no} reached - x: {pos_x:.2f}, y: {pos_y:.2f}, yaw: {current_yaw:.2f}, menunggu stabilisasi...'
                )
            elif time.time() - self.state_transition_time >= self.transition_delay:
                self.final_pose = True
                self.state_transition_time = None
                self.get_logger().info(f'Target {target.no} selesai, pindah ke target berikutnya')
                self.next_target()
        else:
            # Reset timer if not within tolerance
            self.state_transition_time = None
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = yaw_error
            self.publisher_.publish(self.twist_msg)
    
    def next_target(self):
        """Move to the next target waypoint."""
        self.current_target_idx += 1
        
        if self.current_target_idx < len(self.targets):
            self.current_target = self.targets[self.current_target_idx]
            self.first_pose = False
            self.end_controller = False
            self.final_pose = False
            self.sum_error = 0.0
            self.get_logger().info(f'Moving to next target: {self.current_target}')
        else:
            self.get_logger().info('All waypoints completed!')
            # Stop the robot
            stop_msg = Twist()
            self.publisher_.publish(stop_msg)
    
    def destroy_node(self):
        """Cleanup on node shutdown."""
        # Stop the robot
        stop_msg = Twist()
        self.publisher_.publish(stop_msg)
        super().destroy_node()


def main(args=None):
    """Main entry point for file handling navigator node."""
    rclpy.init(args=args)
    
    try:
        navigator = PIDNavigator()
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            navigator.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

