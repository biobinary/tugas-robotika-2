#!/usr/bin/env python3
"""
Robot Controller Node - Keyboard Teleop
Allows manual control of the robot using keyboard input.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import threading


class RobotController(Node):
    """
    Keyboard teleoperation node for differential drive robot.
    
    Controls:
        u/i/o - Forward left/straight/right
        j/k/l - Rotate left/stop/rotate right
        m/,/. - Backward left/straight/right
        q/z - Increase/decrease linear speed
        w/x - Increase/decrease angular speed
    """
    
    def __init__(self):
        super().__init__('robot_controller')
        
        # Publisher
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Initial velocities
        self.speed_x = 0.4
        self.speed_z = 0.4
        
        # Twist message
        self.twist_msg = Twist()
        self.twist_msg.linear.x = 0.0
        self.twist_msg.angular.z = 0.0
        
        # Print instructions
        self.print_instructions()
        
        # Start keyboard loop in separate thread
        self.running = True
        self.keyboard_thread = threading.Thread(target=self.keyboard_loop)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        self.get_logger().info('Robot Controller Node started')
    
    def print_instructions(self):
        """Print keyboard control instructions."""
        print("\n" + "="*50)
        print("Panduan tombol keyboard untuk menggerakkan robot!")
        print("="*50)
        print("Gerakan:")
        print("   u    i    o")
        print("   j    k    l")
        print("   m    ,    .")
        print("-"*50)
        print("u : maju ke kiri")
        print("i : maju ke depan")
        print("o : maju ke kanan")
        print("j : putar kiri")
        print("k : berhenti")
        print("l : putar kanan")
        print("m : mundur ke kiri")
        print(", : mundur ke belakang")
        print(". : mundur ke kanan")
        print("-"*50)
        print("q/z : menambah/mengurangi kecepatan linier (0.1)")
        print("w/x : menambah/mengurangi kecepatan angular (0.1)")
        print("-"*50)
        print("Tekan CTRL-C untuk keluar")
        print("="*50 + "\n")
    
    def get_key(self):
        """Get single keypress from terminal."""
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def keyboard_loop(self):
        """Main keyboard input loop."""
        self.settings = termios.tcgetattr(sys.stdin)
        
        try:
            while self.running and rclpy.ok():
                key = self.get_key()
                
                if key == 'u':
                    self.twist_msg.linear.x = self.speed_x
                    self.twist_msg.angular.z = self.speed_z
                elif key == 'i':
                    self.twist_msg.linear.x = self.speed_x
                    self.twist_msg.angular.z = 0.0
                elif key == 'o':
                    self.twist_msg.linear.x = self.speed_x
                    self.twist_msg.angular.z = -self.speed_z
                elif key == 'j':
                    self.twist_msg.linear.x = 0.0
                    self.twist_msg.angular.z = self.speed_z
                elif key == 'k':
                    self.twist_msg.linear.x = 0.0
                    self.twist_msg.angular.z = 0.0
                elif key == 'l':
                    self.twist_msg.linear.x = 0.0
                    self.twist_msg.angular.z = -self.speed_z
                elif key == 'm':
                    self.twist_msg.linear.x = -self.speed_x
                    self.twist_msg.angular.z = self.speed_z
                elif key == ',':
                    self.twist_msg.linear.x = -self.speed_x
                    self.twist_msg.angular.z = 0.0
                elif key == '.':
                    self.twist_msg.linear.x = -self.speed_x
                    self.twist_msg.angular.z = -self.speed_z
                elif key == 'q':
                    self.speed_x = min(self.speed_x + 0.1, 2.0)
                    self.get_logger().info(f'Linear speed: {self.speed_x:.2f}')
                elif key == 'z':
                    self.speed_x = max(self.speed_x - 0.1, 0.0)
                    self.get_logger().info(f'Linear speed: {self.speed_x:.2f}')
                elif key == 'w':
                    self.speed_z = min(self.speed_z + 0.1, 2.0)
                    self.get_logger().info(f'Angular speed: {self.speed_z:.2f}')
                elif key == 'x':
                    self.speed_z = max(self.speed_z - 0.1, 0.0)
                    self.get_logger().info(f'Angular speed: {self.speed_z:.2f}')
                elif key == '\x03':  # Ctrl+C
                    break
                
                # Publish velocity command
                self.publisher_.publish(self.twist_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error in keyboard loop: {e}')
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    
    def destroy_node(self):
        """Cleanup on node shutdown."""
        self.running = False
        # Stop the robot
        stop_msg = Twist()
        self.publisher_.publish(stop_msg)
        super().destroy_node()


def main(args=None):
    """Main entry point for robot controller node."""
    rclpy.init(args=args)
    
    try:
        robot_controller = RobotController()
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            robot_controller.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()