#!/usr/bin/env python3
"""
drive_test.py — Simple empirical drive test for Gazebo.
Publishes cmd_vel commands to test basic robot motion.

Usage (from inside the container, with sim running):
  python3 drive_test.py

Controls what gets published:
  PHASE 1: Forward  (linear.x=+0.3, angular.z=0)  — 3 sec
  PHASE 2: Stop                                     — 1 sec
  PHASE 3: Backward (linear.x=-0.3, angular.z=0)   — 3 sec
  PHASE 4: Stop                                     — 1 sec
  PHASE 5: Turn left (linear.x=0, angular.z=+0.5)  — 3 sec
  PHASE 6: Stop                                     — 1 sec
  PHASE 7: Turn right (linear.x=0, angular.z=-0.5) — 3 sec
  PHASE 8: Stop
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class DriveTest(Node):
    def __init__(self):
        super().__init__('drive_test')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Wait for pub to connect
        time.sleep(0.5)

    def send(self, lin_x: float, ang_z: float, duration: float, label: str):
        msg = Twist()
        msg.linear.x = lin_x
        msg.angular.z = ang_z
        self.get_logger().info(
            f'--- {label}: linear.x={lin_x:+.2f}  angular.z={ang_z:+.2f} ---'
        )
        end_time = time.time() + duration
        while time.time() < end_time:
            self.pub.publish(msg)
            time.sleep(0.1)

    def stop(self, duration: float = 1.0):
        self.send(0.0, 0.0, duration, 'STOP')

    def run(self):
        self.send(+0.3,  0.0, 3.0, 'FORWARD')
        self.stop()
        self.send(-0.3,  0.0, 3.0, 'BACKWARD')
        self.stop()
        self.send( 0.0, +0.5, 3.0, 'TURN LEFT (angular.z positive)')
        self.stop()
        self.send( 0.0, -0.5, 3.0, 'TURN RIGHT (angular.z negative)')
        self.stop()
        self.get_logger().info('=== Drive test complete ===')


def main():
    rclpy.init()
    node = DriveTest()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
