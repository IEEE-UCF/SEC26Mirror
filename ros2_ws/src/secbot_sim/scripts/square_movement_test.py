#!/usr/bin/env python3
"""
square_movement_test.py
=======================
A primitive navigation script that drives the robot in a 1m x 1m square pattern.
Sequences linear motion and closed-loop angular rotations, with detailed error reporting.
"""

import math
import sys
import os
import rclpy

# Add the scripts directory to path to import MotionCalibrationNode
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from motion_calibration_test import MotionCalibrationNode, CalibResult

def main():
    rclpy.init()
    
    # Initialize the node with closed-loop enabled for precise corners
    node = MotionCalibrationNode(
        speed=0.2, 
        turn_speed=0.5, 
        closed_loop=True,
        settle_time=1.5
    )
    
    results = []
    
    try:
        # 1. Wait for Gazebo topics to be ready
        if not node._wait_for_topics():
            node.get_logger().error("Topics not ready. Is Gazebo running?")
            return

        node.get_logger().info("------------------------------------------------")
        node.get_logger().info("--- STARTING SQUARE MOVEMENT TEST (1m x 1m) ---")
        node.get_logger().info("------------------------------------------------")
        
        # Capture starting pose
        start_snapshot = node._snapshot()
        
        # 2. Perform 4 iterations (Side -> Corner)
        for i in range(1, 5):
            node.get_logger().info(f"\n>>> SEQUENCE {i}/4")
            
            # Step A: Move Forward 1.0 meter
            node.get_logger().info(f" [Side {i}] Driving 1.0m Forward...")
            res_lin = node._run_linear_test(f"Side {i}", 1.0)
            results.append(res_lin)
            
            # Log step error
            node.get_logger().info(f"   -> Result: {res_lin.gz_measured:.4f}m (Err: {res_lin.gz_error:+.4f}m | {res_lin.gz_pct:+.1f}%)")
            node._settle()
            
            # Step B: Turn 90 Degrees CCW (Positive Radians)
            node.get_logger().info(f" [Corner {i}] Turning 90° CCW...")
            res_turn = node._run_turn_test(f"Corner {i}", math.pi / 2)
            results.append(res_turn)
            
            # Log step error
            node.get_logger().info(f"   -> Result: {res_turn.imu_measured:.4f}rad (Err: {res_turn.imu_error:+.4f}rad | {res_turn.imu_pct:+.1f}%)")
            node._settle()
            
        # Capture final pose
        end_snapshot = node._snapshot()
        
        # 3. Final Report
        node.get_logger().info("\n" + "="*50)
        node.get_logger().info(" FINAL CUMULATIVE ERROR SUMMARY")
        node.get_logger().info("="*50)
        
        # Calc final displacement from start (should be 0 for a closed square)
        dx = end_snapshot["gt_x"] - start_snapshot["gt_x"]
        dy = end_snapshot["gt_y"] - start_snapshot["gt_y"]
        final_drift = math.hypot(dx, dy)
        
        # Calc orientation drift (should be 0 or dynamic multiple of 2pi)
        # gt_cum_yaw tracks total rotation. After 4x90=360, it should be start + 2pi.
        total_rot = end_snapshot["gt_cum_yaw"] - start_snapshot["gt_cum_yaw"]
        rot_error = total_rot - (2.0 * math.pi)
        
        node.get_logger().info(f" Final X-Drift:   {dx:+.4f} m")
        node.get_logger().info(f" Final Y-Drift:   {dy:+.4f} m")
        node.get_logger().info(f" TOTAL DRIFT:     {final_drift:.4f} m (Final distance from start)")
        node.get_logger().info(f" ROTATION DRIFT:  {rot_error:+.4f} rad ({math.degrees(rot_error):+.2f}°)")
        node.get_logger().info("="*50)

    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
        node._stop()
    except Exception as e:
        node.get_logger().error(f"An error occurred: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
