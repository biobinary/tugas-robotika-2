#!/usr/bin/env python3
"""
PID Controller Node - Single Target Navigation
Navigates robot to a single target position using PID control.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time


class PIDController(Node):
    """
    PID-based navigation controller for differential drive robot.
    
    Three-stage control:
    1. Rotate to face target
    2. Move to target position
    3. Adjust final orientation
    """
    
    def __init__(self):
        super().__init__('pid_controller')
        
        # Declare and get parameters
        self.declare_parameter('target_x', 3.0)
        self.declare_parameter('target_y', 4.0)
        self.declare_parameter('target_yaw', -1.56)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.1)
        self.declare_parameter('max_linear_vel', 0.8)
        self.declare_parameter('angle_tolerance', 0.05)
        self.declare_parameter('distance_tolerance', 0.5)
        self.declare_parameter('final_angle_tolerance', 0.1)
        self.declare_parameter('transition_delay', 1.0)
        
        # Get parameters
        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value
        self.target_yaw = self.get_parameter('target_yaw').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value
        self.final_angle_tolerance = self.get_parameter('final_angle_tolerance').value
        self.transition_delay = self.get_parameter('transition_delay').value
        
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
        
        self.get_logger().info(f'PID Controller initialized - Target: ({self.target_x}, {self.target_y}, {self.target_yaw})')
    
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
        target_yaw = math.atan2(self.target_y - pos_y, self.target_x - pos_x)
        yaw_error = self.normalize_angle(target_yaw - current_yaw)
        
        # Check if within tolerance (fixed: use normalized error)
        if abs(yaw_error) <= self.angle_tolerance:
            # Use state timer instead of blocking sleep
            if self.state_transition_time is None:
                self.state_transition_time = time.time()
                self.twist_msg.linear.x = 0.0
                self.twist_msg.angular.z = 0.0
                self.publisher_.publish(self.twist_msg)
                self.get_logger().info('Berhasil berbelok ke arah tujuan, menunggu stabilisasi...')
            elif time.time() - self.state_transition_time >= self.transition_delay:
                self.first_pose = True
                self.state_transition_time = None
                self.get_logger().info('Transisi ke fase movement')
        else:
            # Reset timer if not within tolerance
            self.state_transition_time = None
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = yaw_error
            self.publisher_.publish(self.twist_msg)
    
    def move_to_target(self, pos_x, pos_y):
        """Move robot to target position using PID control."""
        distance = math.sqrt((self.target_y - pos_y)**2 + (self.target_x - pos_x)**2)
        self.get_logger().info(f'Jarak sampai tujuan: {distance:.3f} m')
        
        # PID control
        error = distance
        self.sum_error += error
        
        p = self.kp * error
        i = self.ki * self.sum_error
        
        velocity = p + i
        
        # Limit velocity
        if abs(velocity) > self.max_linear_vel:
            velocity = self.max_linear_vel if velocity > 0 else -self.max_linear_vel
        
        self.twist_msg.linear.x = velocity
        self.twist_msg.angular.z = 0.0
        self.publisher_.publish(self.twist_msg)
        
        if distance < self.distance_tolerance:
            self.get_logger().info('Robot telah sampai tujuan')
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = 0.0
            self.end_controller = True
            self.sum_error = 0.0
    
    def adjust_final_orientation(self, pos_x, pos_y, current_yaw):
        """Adjust robot to final target orientation."""
        yaw_error = self.normalize_angle(self.target_yaw - current_yaw)
        
        # Check if within tolerance (fixed: use normalized error)
        if abs(yaw_error) <= self.final_angle_tolerance:
            # Use state timer instead of blocking sleep
            if self.state_transition_time is None:
                self.state_transition_time = time.time()
                self.twist_msg.linear.x = 0.0
                self.twist_msg.angular.z = 0.0
                self.publisher_.publish(self.twist_msg)
                self.get_logger().info('Orientasi final tercapai, menunggu stabilisasi...')
            elif time.time() - self.state_transition_time >= self.transition_delay:
                self.final_pose = True
                self.state_transition_time = None
                self.get_logger().info(
                    f'Final pose reached - x: {pos_x:.2f}, y: {pos_y:.2f}, yaw: {current_yaw:.2f}'
                )
        else:
            # Reset timer if not within tolerance
            self.state_transition_time = None
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = yaw_error
            self.publisher_.publish(self.twist_msg)
    
    def destroy_node(self):
        """Cleanup on node shutdown."""
        # Stop the robot
        stop_msg = Twist()
        self.publisher_.publish(stop_msg)
        super().destroy_node()


def main(args=None):
    """Main entry point for PID controller node."""
    rclpy.init(args=args)
    
    try:
        pid_controller = PIDController()
        rclpy.spin(pid_controller)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            pid_controller.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

