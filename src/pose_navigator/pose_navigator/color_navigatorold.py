#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import String
import json
import os
import time
from ament_index_python.packages import get_package_share_directory
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from pose_navigator.pose_navigator_node import Direction, PoseNavigator

# Define TaskResult enum to match the navigation result values

class colorNavigator(PoseNavigator):
    def __init__(self):
        super().__init__()
        
        # Create additional subscriber for color_start
        self.color_start_sub = self.create_subscription(
            String,
            'color_start',
            self.color_start_callback,
            10)
        
        # Create additional subscriber for color_sort
        self.color_sort_sub = self.create_subscription(
            String,
            'color_sort',
            self.color_sort_callback,
            10)
        
        # Parent class already creates nav_done_pub and navigator
        self.color = []
        
        # Publish home position as initial pose if it exists


    def center_to_color(self,color):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        if color == "Yellow":
            self.get_logger().info('執行向黃導航序列')
        # 先移動到up_center並面向左方
            self.navigate_to_pose("up_center", Direction.trunLEFT)
            self.navigate_to_pose('Yellow', Direction.trunLEFT)
            self.navigate_to_pose('Yellow', Direction.trunRIGHT)
            self.navigate_to_pose('up_center', Direction.trunRIGHT)
            self.get_logger().info('向黃導航序列完成')
        if color == "Green":
            self.get_logger().info('執行向綠導航序列')
            # 先移動到up_center並面向右方
            self.navigate_to_pose("up_center", Direction.trunRIGHT)
            self.navigate_to_pose('Green', Direction.trunRIGHT)
            self.navigate_to_pose('Green', Direction.trunLEFT)
            self.navigate_to_pose('up_center', Direction.trunLEFT)
            self.get_logger().info('向綠導航序列完成')
        if color == "Blue":
            self.get_logger().info('執行向藍導航序列')
            # 先移動到up_center並面向上方
            self.navigate_to_pose("up_center", Direction.FORWARD)
            self.navigate_to_pose('Blue', Direction.FORWARD)
            self.navigate_to_pose('Blue', Direction.BACKWARD)
            self.navigate_to_pose('up_center', Direction.BACKWARD)
            self.get_logger().info('向藍導航序列完成')
        if color == "home":
            """執行回到家的導航序列"""
            self.get_logger().info('執行回到家的導航序列')
        # 先移動到up_center並面向上方
            self.navigate_to_pose("up_center", Direction.trunLEFT)
            self.navigate_to_pose('base', Direction.trunLEFT)
            self.navigate_to_pose('base', Direction.BACKWARD)
            self.navigate_to_pose('home', Direction.BACKWARD)
            self.get_logger().info('回到家導航序列完成')

    def center_to_home(self):
        """執行回到家的導航序列"""
        self.get_logger().info('執行回到家的導航序列')
        # 先移動到up_center並面向上方
        self.navigate_to_pose("up_center", Direction.trunLEFT)
        self.navigate_to_pose('base', Direction.trunLEFT)
        self.navigate_to_pose('base', Direction.BACKWARD)
        self.navigate_to_pose('home', Direction.BACKWARD)
        self.get_logger().info('回到家導航序列完成')
    
    def pose_callback(self, msg):
        """Handle incoming pose name"""
        self.navigate_to_pose(msg.data)
    def color_start_callback(self, msg):
        self.home_to_center()
    def color_sort_callback(self, msg):
        self.color = msg.data.split(',')
        if len(self.color) == 3:
            self.get_logger().error('Color sequence must contain exactly 3 colors')
            self.center_to_color(self.color[0])
            self.center_to_color(self.color[1])
            self.center_to_color(self.color[2])
            self.center_to_home()
            self.get_logger().error('down')
            self.destroy_node()
            rclpy.shutdown()
            return

def main(args=None):
    rclpy.init(args=args)
    node = colorNavigator()   
    node.start()   
    node.center_to_color("Blue")
    node.center_to_color("Green")
    node.center_to_color("Yellow")
 
    node.center_to_home()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 