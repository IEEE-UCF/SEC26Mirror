#!/usr/bin/env python3
import math
import sys
import rclpy
from rclpy.node import Node
import argparse

# Import from the existing calibration script
# Note: In a ROS2 workspace, we might need to append the scripts dir to path if not installed as a module
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from motion_calibration_test import MotionCalibrationNode, CalibResult

def print_sweep_report(all_results: dict):
    """
    all_results: { speed: [CalibResult, ...], ... }
    """
    print("\n" + "="*80)
    print(" SPEED CALIBRATION SWEEP SUMMARY")
    print("="*80)
    
    # Linear results table
    print("\nLINEAR ACCURACY (GT vs Command)")
    print("-" * 60)
    print(f"{'Speed (m/s)':<15} | {'Measured (m)':<15} | {'Error %':<10} | {'Status'}")
    print("-" * 60)
    
    for speed, results in all_results.items():
        lin_results = [r for r in results if r.units == "m"]
        if not lin_results: continue
        
        # Take the average Error% for linear tests at this speed
        avg_err = sum(abs(r.gz_pct) for r in lin_results) / len(lin_results)
        avg_dist = sum(r.gz_measured for r in lin_results) / len(lin_results)
        status = "PASS" if avg_err < 5.0 else "FAIL"
        
        print(f"{speed:<15.2f} | {avg_dist:<15.4f} | {avg_err:<9.1f}% | {status}")

    # Angular results table
    print("\nANGULAR ACCURACY (IMU Feedback vs Command)")
    print("-" * 60)
    print(f"{'Speed (rad/s)':<15} | {'Measured (rad)':<15} | {'IMU Err %':<10} | {'Status'}")
    print("-" * 60)
    
    for speed, results in all_results.items():
        ang_results = [r for r in results if r.units == "rad"]
        if not ang_results: continue
        
        # Use turn_speed from node config associated with these results
        # For simplicity, we'll just display based on the iteration speed
        avg_err = sum(abs(r.imu_pct) for r in ang_results) / len(ang_results)
        avg_ang = sum(r.imu_measured for r in ang_results) / len(ang_results)
        status = "PASS" if avg_err < 5.0 else "FAIL"
        
        print(f"{speed:<15.2f} | {avg_ang:<15.4f} | {avg_err:<9.1f}% | {status}")
        
    print("\n" + "="*80)

def load_sweep_defaults():
    # Final hardcoded fallbacks
    lin_speeds = ",".join([f"{x/100:.2f}" for x in range(5, 105, 5)])
    ang_speeds = ",".join([f"{x/100:.2f}" for x in range(5, 105, 5)])
    closed_loop = True
    
    try:
        from ament_index_python.packages import get_package_share_directory
        import yaml
        pkg_path = get_package_share_directory('my_robot_description')
        config_path = os.path.join(pkg_path, 'config', 'motion_config.yaml')
        
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                cfg = yaml.safe_load(f)
                cal = cfg.get('calibration_defaults', {})
                # Note: If we wanted specific sweep ranges in YAML, we could add them there.
                # For now, we use the same 0.05-1.0 logic unless overridden by user.
                # But we can at least pull the closed-loop preference.
                closed_loop = cal.get('closed_loop_sweep', True) # Assume default if missing
    except:
        pass
    return lin_speeds, ang_speeds, closed_loop

def main():
    default_lin, default_ang, default_cl = load_sweep_defaults()
    
    parser = argparse.ArgumentParser(description="SECBot Speed Sweep Calibration")
    parser.add_argument("--speeds", type=str, default=default_lin, 
                        help=f"List of linear speeds (default: {default_lin})")
    parser.add_argument("--turn-speeds", type=str, default=default_ang, 
                        help=f"List of angular speeds (default: {default_ang})")
    parser.add_argument("--closed-loop", action="store_true", default=default_cl,
                        help=f"Use IMU feedback for turns (default: {default_cl})")


    
    args = parser.parse_args()
    
    lin_speeds = [float(s) for s in args.speeds.split(",")]
    ang_speeds = [float(s) for s in args.turn_speeds.split(",")]
    
    rclpy.init()
    
    sweep_results = {}
    
    try:
        # 1. Linear Sweep
        for speed in lin_speeds:
            print(f"\n>>> TESTING LINEAR SPEED: {speed} m/s")
            node = MotionCalibrationNode(speed=speed, settle_time=1.0)
            
            if not node._wait_for_topics():
                print(f"!!! Skipping speed {speed} - topics timeout")
                node.destroy_node()
                continue

            # Run a subset of linear tests for efficiency
            results = []
            results.append(node._run_linear_test(f"Fwd {speed}m/s", 1.0))
            node._settle()
            
            # Return to start point to stay within arena bounds (approx 16x8m)
            # This allows the full sweep to run without hitting a wall.
            node._run_linear_test(f"Reset {speed}m/s", -1.0)
            node._settle()
            
            sweep_results[speed] = results
            node.destroy_node()


        # 2. Angular Sweep
        for t_speed in ang_speeds:
            print(f"\n>>> TESTING TURN SPEED: {t_speed} rad/s")
            # We use a dummy linear speed
            node = MotionCalibrationNode(turn_speed=t_speed, closed_loop=args.closed_loop, settle_time=1.0)
            
            if not node._wait_for_topics():
                print(f"!!! Skipping turn speed {t_speed} - topics timeout")
                node.destroy_node()
                continue

            results = []
            results.append(node._run_turn_test(f"Turn {t_speed}rad/s", math.pi/2)) # 90 deg
            node._settle()

            
            if t_speed in sweep_results:
                sweep_results[t_speed].extend(results)
            else:
                sweep_results[t_speed] = results
            node.destroy_node()
            
        print_sweep_report(sweep_results)
        
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
