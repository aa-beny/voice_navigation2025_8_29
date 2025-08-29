#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class ColorSequencePublisher(Node):
    def __init__(self):
        super().__init__('color_sequence_publisher')
        
        self.publisher = self.create_publisher(
            String,
            'color_sequence',
            10)
        
        self.get_logger().info('Color sequence publisher started')
        
    def publish_sequence(self, colors):
        """Publish a color sequence"""
        msg = String()
        msg.data = ','.join(colors)
        
        self.get_logger().info(f'Publishing color sequence: {msg.data}')
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ColorSequencePublisher()
    
    # Example color sequence: yellow, green, blue, home
    # Wait to ensure subscriber is ready
    time.sleep(2)
    node.publish_sequence(['yellow', 'green', 'blue', 'home'])
    
    # Keep node alive for a bit to ensure message is sent
    time.sleep(2)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 