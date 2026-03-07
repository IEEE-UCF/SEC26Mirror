#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess
import time
import math
import sys

class RobotParamTuner(Node):
    def __init__(self):
        super().__init__('robot_param_tuner')
        self.get_logger().info("Automated Parameter Tuner Initialized")

    def set_param(self, node_name, param_name, value):
        cmd = f"ros2 param set {node_name} {param_name} {value}"
        subprocess.run(cmd, shell=True, check=True, capture_output=True)

    def run_test(self):
        # Run the accuracy policy script and capture output
        # In a real environment, we'd use a more robust IPC, 
        # but for a quick tool, parsing output or checking status works.
        cmd = "python3 scripts/test_accuracy_policy.py"
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        
        # Simple parser for the accuracy score
        for line in result.stdout.split('\n'):
            if "ACCURACY SCORE:" in line:
                try:
                    return float(line.split(":")[1].split("/")[0].strip())
                except:
                    return 0.0
        return 0.0

    def tune(self):
        self.get_logger().info("Starting Automated Tuning...")
        
        best_score = 0
        best_track_width = 7.95276
        best_imu_scaler = 1.0
        
        # Coordinate Descent Optimization (Simplified)
        # Search track_width around 7.95 +/- 0.5
        for tw in [7.4, 7.7, 7.95, 8.2, 8.5]:
            self.get_logger().info(f"Testing track_width={tw}")
            self.set_param('mcu_subsystem_sim', 'track_width', float(tw))
            score = self.run_test()
            self.get_logger().info(f"Score: {score}")
            if score > best_score:
                best_score = score
                best_track_width = tw

        # Search imu_scaler around 1.0 +/- 0.1
        for sc in [0.9, 0.95, 1.0, 1.05, 1.1]:
            self.get_logger().info(f"Testing imu_scaler={sc}")
            self.set_param('mcu_subsystem_sim', 'imu_scaler', float(sc))
            score = self.run_test()
            self.get_logger().info(f"Score: {score}")
            if score > best_score:
                best_score = score
                best_imu_scaler = sc

        self.get_logger().info("\n" + "*"*40)
        self.get_logger().info("TUNING COMPLETE")
        self.get_logger().info(f"Best Score: {best_score}")
        self.get_logger().info(f"Optimized track_width: {best_track_width}")
        self.get_logger().info(f"Optimized imu_scaler:  {best_imu_scaler}")
        self.get_logger().info("*"*40)
        
        # Apply best params
        self.set_param('mcu_subsystem_sim', 'track_width', float(best_track_width))
        self.set_param('mcu_subsystem_sim', 'imu_scaler', float(best_imu_scaler))

def main():
    rclpy.init()
    tuner = RobotParamTuner()
    tuner.tune()
    tuner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
