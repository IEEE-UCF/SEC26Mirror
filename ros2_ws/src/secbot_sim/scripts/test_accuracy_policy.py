#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from mcu_msgs.msg import DriveBase
import math
import time
from dataclasses import dataclass
from typing import List, Tuple

@dataclass
class TestPoint:
    name: str
    target_x: float
    target_y: float
    target_yaw: float

class AccuracyPolicyTester(Node):
    def __init__(self):
        super().__init__('accuracy_policy_tester')
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_in', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        self.gt_sub = self.create_subscription(Odometry, '/odom/ground_truth', self._gt_callback, 10)
        self.mcu_sub = self.create_subscription(DriveBase, '/drive_base/status', self._mcu_callback, 10)
        
        self.odom_pose = None
        self.gt_pose = None
        self.mcu_pose = None
        
        self.get_logger().info("Accuracy Policy Tester Initialized")

    def _odom_callback(self, msg: Odometry):
        self.odom_pose = msg.pose.pose

    def _gt_callback(self, msg: Odometry):
        self.gt_pose = msg.pose.pose

    def _mcu_callback(self, msg: DriveBase):
        self.mcu_pose = msg.transform.transform

    def get_pose_2d(self, pose):
        if pose is None: return None
        # Handle both Pose (msg) and Transform (mcu status)
        if hasattr(pose, 'position'):
            x = pose.position.x
            y = pose.position.y
            qz = pose.orientation.z
            qw = pose.orientation.w
        else:
            x = pose.translation.x
            y = pose.translation.y
            qz = pose.rotation.z
            qw = pose.rotation.w
        
        yaw = 2.0 * math.atan2(qz, qw)
        return (x, y, yaw)

    def drive(self, v, w, duration):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        
        end_time = time.time() + duration
        while rclpy.ok() and time.time() < end_time:
            self.cmd_pub.publish(msg)
            time.sleep(0.05)
            rclpy.spin_once(self, timeout_sec=0)
            
        self.cmd_pub.publish(Twist()) # Stop

    def run_policy(self):
        self.get_logger().info("Starting Accuracy Policy Test...")
        
        # Robust wait for sensors
        timeout = 10.0
        start_wait = time.time()
        while rclpy.ok() and (time.time() - start_wait) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            missing = []
            if self.odom_pose is None: missing.append("/odom")
            if self.gt_pose is None: missing.append("/odom/ground_truth")
            if self.mcu_pose is None: missing.append("/drive_base/status")
            
            if not missing:
                break
            self.get_logger().info(f"Waiting for topics: {', '.join(missing)}...", throttle_duration_sec=2.0)

        if self.gt_pose is None:
            self.get_logger().error("TIMEOUT: No Ground Truth received! Ensure Gazebo is running and OdometryPublisher is enabled.")
            return
        if self.mcu_pose is None:
            self.get_logger().error("TIMEOUT: No MCU Status received! Ensure mcu_subsystem_sim is running.")
            return

        self.get_logger().info("All sensors active. Starting square test...")

        def run_leg(d, ang):
            self.drive(0.2, 0.0, d/0.2)
            time.sleep(0.5)
            self.drive(0.0, 0.5, ang/0.5)
            time.sleep(0.5)

        for i in range(4):
            self.get_logger().info(f"Running Leg {i+1}/4")
            run_leg(1.0, math.pi/2.0)

        end_gt = self.get_pose_2d(self.gt_pose)
        end_mcu = self.get_pose_2d(self.mcu_pose)
        
        dist_error = math.sqrt((end_gt[0] - start_gt[0])**2 + (end_gt[1] - start_gt[1])**2)
        yaw_error = abs(end_gt[2] - start_gt[2])
        mcu_drift = math.sqrt((end_gt[0] - end_mcu[0])**2 + (end_gt[1] - end_mcu[1])**2)

        self.get_logger().info("\n" + "="*40)
        self.get_logger().info("POLICY TEST RESULTS")
        self.get_logger().info("="*40)
        self.get_logger().info(f"Final GT Position Error: {dist_error:.4f} m")
        self.get_logger().info(f"Final GT Yaw Error:      {math.degrees(yaw_error):.2f} deg")
        self.get_logger().info(f"MCU vs GT Drift:         {mcu_drift:.4f} m")
        
        score = max(0, 100 - (dist_error * 100) - (mcu_drift * 100))
        self.get_logger().info(f"ACCURACY SCORE: {score:.1f}/100")
        self.get_logger().info("="*40)

def main():
    rclpy.init()
    tester = AccuracyPolicyTester()
    try:
        tester.run_policy()
    except KeyboardInterrupt:
        pass
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
