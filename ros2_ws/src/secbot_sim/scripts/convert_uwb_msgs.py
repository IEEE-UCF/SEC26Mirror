"""
converts sim uwb messages into secbot_uwb messages
"""

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from uwb_interfaces.msg import UWBData
from mcu_msgs.msg import UWBRanging, UWBRange

class UWBBridgeNode(Node):
    def __init__(self):
        super().__init__('uwb_sim_bridge')
        self.sub = self.create_subscription(UWBData, '/uwbData/my_robot', self.callback, 10) # change my_robot to robot name if it changes idk
        self.pub = self.create_publisher(UWBRanging, 'mcu_uwb/ranging', 10)
        self.tag_id = 12 

    def callback(self, msg):
        out = UWBRanging()
        out.header.stamp = self.get_clock().now().to_msg()
        out.tag_id = self.tag_id
        
        for sim_range in msg.distances:
            r = UWBRange()
            r.anchor_id = sim_range.anchor_id
            r.distance = sim_range.distance * 100.0  # Convert meters to cm
            r.valid = True
            r.error_code = 0
            out.ranges.append(r)
            
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    bridge = UWBBridgeNode()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()