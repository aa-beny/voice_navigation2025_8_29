#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import time
from pose_navigator.pose_navigator_node import Direction, PoseNavigator

class ColorNavigator(PoseNavigator):
    def __init__(self):
        super().__init__()

        self.command_sub = self.create_subscription(String, '/car_command', self.command_callback, 10)
        self.arrival_pub = self.create_publisher(Bool, '/car_arrived', 10)

        self.get_logger().info('✅  Navigator Ready (with up_center insert sequence)')
        self.last_zone = None  # 🔄 記錄上一個區域

    def command_callback(self, msg):
        zone = msg.data.strip().upper()

        self.get_logger().info(f"🚗 Received navigation request to zone {zone}")
        
        if zone == 'B':
            self.get_logger().warn(f"111 zone: {zone}")
            self.navigate_to_pose("base", Direction.trunLEFT)
            self.navigate_to_pose("left", Direction.trunLEFT)

        elif zone == 'C':
            self.navigate_to_pose("up_center", Direction.FORWARD)
            self.navigate_to_pose("up", Direction.FORWARD)
         

        elif zone == 'D':
           
   
        
            self.navigate_to_pose("right", Direction.trunRIGHT)
           
        elif zone == 'E':
            if self.last_zone == 'B':
                self.get_logger().info(f"🚗 Received navigation last_zone {self.last_zone}")
                self.navigate_to_pose("left", Direction.trunRIGHT)
                self.navigate_to_pose("up_center", Direction.trunRIGHT)
            elif self.last_zone == 'C':
                self.get_logger().info(f"🚗 Received navigation last_zone {self.last_zone}")
                self.navigate_to_pose("up", Direction.BACKWARD)
                self.navigate_to_pose("up_center", Direction.BACKWARD)
            elif self.last_zone == 'D':
                self.get_logger().info(f"🚗 Received navigation last_zone {self.last_zone}")
                self.navigate_to_pose("right", Direction.trunLEFT)
          
            else:
                self.get_logger().warn("⚠️ Unknown last zone before E, skipping up_center alignment.")
            self.navigate_to_pose("collection", Direction.BACKWARD)
        elif zone == 'A':
            self.navigate_to_pose("up_center", Direction.trunLEFT)
            self.navigate_to_pose("base", Direction.trunLEFT)
            self.navigate_to_pose("base", Direction.BACKWARD)
            self.navigate_to_pose("home", Direction.BACKWARD)
        else:
            self.get_logger().warn(f"❌ Unknown zone: {zone}")
            return

        # 等待導航完成
        while not self.navigator.isTaskComplete():
            time.sleep(0.5)

        result = self.navigator.getResult()
        self.get_logger().info(f"📤 Navigation ended with result: {result} → sending /car_arrived")
        self.arrival_pub.publish(Bool(data=True))
        self.last_zone = zone


        # print(result)
        # if result == "TaskResult.SUCCEEDED":
        #     self.get_logger().info(f"📍 Arrived at zone {zone}, sending /car_arrived")
        #     self.arrival_pub.publish(Bool(data=True))
        # else:
        #     self.get_logger().warn(f"⚠️ Failed to reach zone {zone}")


def main(args=None):
    rclpy.init(args=args)
    node = ColorNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
