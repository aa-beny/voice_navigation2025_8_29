#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class ForwardReturnNode(Node):
    def __init__(self):
        super().__init__('forward_return_node')
        
        # Create publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        # Robot movement parameters
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 1.0  # rad/s
        self.forward_time = 2.0  # seconds
        
        self.get_logger().info('Forward Return Node initialized')
        
        # Start the movement sequence
        self.start_movement_sequence()
    
    def start_movement_sequence(self):
        """Execute the complete movement sequence"""
        self.get_logger().info('Starting movement sequence...')
        
        # Step 1: Move forward for 2 seconds
        self.move_forward()
        
        # Step 2: Turn 180 degrees
        self.turn_180()
        
        # Step 3: Move forward again (return path)
        self.move_forward()
        
        # Step 4: Stop
        self.stop_robot()
        
        self.get_logger().info('Movement sequence completed')
    
    def move_forward(self):
        """Move robot forward for specified time"""
        self.get_logger().info(f'Moving forward for {self.forward_time} seconds...')
        
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = 0.0
        
        # Publish forward command
        start_time = time.time()
        while time.time() - start_time < self.forward_time:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)  # 10Hz publishing rate
        
        # Stop after forward movement
        self.stop_robot()
        time.sleep(0.5)  # Brief pause
    
    def turn_180(self):
        """Turn robot 180 degrees"""
        self.get_logger().info('Turning 180 degrees...')
        
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.angular_speed
        
        # Calculate time needed for 180 degree turn (π radians)
        turn_time = 3.14159 / self.angular_speed
        
        # Publish turn command
        start_time = time.time()
        while time.time() - start_time < turn_time:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)  # 10Hz publishing rate
        
        # Stop after turning
        self.stop_robot()
        time.sleep(0.5)  # Brief pause
    
    def stop_robot(self):
        """Stop the robot by publishing zero velocities"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        
        # Publish stop command multiple times to ensure it's received
        for _ in range(5):
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    
    node = ForwardReturnNode()
    
    try:
        # Keep the node alive for a short time to complete the sequence
        time.sleep(10)  # Adjust based on total movement time needed
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()