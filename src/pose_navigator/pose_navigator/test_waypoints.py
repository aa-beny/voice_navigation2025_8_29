#!/usr/bin/env python3

import rclpy
from std_msgs.msg import String
from pose_navigator_node import PoseNavigator, Waypoints, Direction

def main():

    rclpy.init()
    
    # 創建節點
    
    
   
    navigator = PoseNavigator()
    rclpy.sleep(1.0)

    
    print("\n開始導航測試序列:")
    navigator.start(),
    navigator.gotoleft(),
    navigator.gotoright(),
    navigator.gotoup(),
    navigator.gotohome(),
    rclpy.shutdown()

if __name__ == '__main__':
    main()