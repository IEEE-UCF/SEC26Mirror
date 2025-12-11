#!/usr/bin/env python3

import argparse
import os
import subprocess
import sys
from pathlib import Path

# Container paths (match docker-compose mounts)
SCRIPTS_DIR = Path("/home/ubuntu/scripts")
MCU_WS = Path("/home/ubuntu/mcu_workspaces/sec26mcu")
ROS_WS = Path("/home/ubuntu/ros2_workspaces")
ROS_INSTALL_SETUP = ROS_WS / "install" / "setup.bash"
TEST_PKG_LAUNCH = Path("/home/ubuntu/ros2_workspaces/src/sec26ros/test_package/launch/test_node.launch.py")


def run(cmd, env=None, cwd=None, shell=False):
    print(f"$ {' '.join(cmd) if not shell else cmd}")
    try:
        subprocess.run(cmd, check=True, env=env, cwd=cwd, shell=shell)
    except subprocess.CalledProcessError as e:
        print(f"\n❌ Command failed: {e}")
        sys.exit(e.returncode or 1)


def ensure_in_container():
    # Best-effort check to ensure paths exist in the container
    missing = []
    for p in [SCRIPTS_DIR, MCU_WS, ROS_WS]:
        if not p.exists():
            missing.append(str(p))
    if missing:
        print("❌ Required container paths not found:")
        for m in missing:
            print(f"   - {m}")
        print("This script is intended to run inside the devcontainer.")
        print("Start the container and run it there (docker exec or VS Code).")
        sys.exit(1)


def flash_mcu():
    script = SCRIPTS_DIR / "flash_mcu.sh"
    if not script.exists():
        print(f"❌ Missing script: {script}")
        sys.exit(1)
    print("🚀 Flashing MCU (Teensy + Comm ESP32)...")
    run(["bash", str(script)])
    print("✅ MCU flashing completed")


def build_ros():
    script = SCRIPTS_DIR / "start_robot.sh"
    if not script.exists():
        print(f"❌ Missing script: {script}")
        sys.exit(1)
    print("🛠️ Building ROS 2 workspace with colcon...")
    run(["bash", str(script)])
    print("✅ ROS 2 build completed")


def launch_test_node():
    if not ROS_INSTALL_SETUP.exists():
        print(f"❌ ROS install setup not found: {ROS_INSTALL_SETUP}")
        print("Run the build step first or investigate build errors.")
        sys.exit(1)

    print("▶️ Launching test node from test_package...")
    # Prefer a Python launch file if available; fallback to ros2 run
    if TEST_PKG_LAUNCH.exists():
        # Use ros2 launch with the exact file path to avoid naming mismatch
        cmd = (
            f"source {ROS_INSTALL_SETUP} && "
            f"ros2 launch {TEST_PKG_LAUNCH}"
        )
        # Use bash -lc so that 'source' and env persist within the shell
        run(["bash", "-lc", cmd])
    else:
        cmd = (
            f"source {ROS_INSTALL_SETUP} && "
            f"ros2 run test_package test_node"
        )
        # Use bash -lc to ensure ROS environment is active for the command
        run(["bash", "-lc", cmd])
    print("✅ Test node launched")


def main():
    parser = argparse.ArgumentParser(description="Deploy all components: MCU + ROS + test node")
    # Flags:
    # --skip-mcu: Skip flashing Teensy/ESP32 using scripts/flash_mcu.sh
    # --skip-ros: Skip colcon build using scripts/start_robot.sh
    # --skip-launch: Skip launching the test node (ros2 launch/run)
    parser.add_argument("--skip-mcu", action="store_true", help="Skip MCU flashing")
    parser.add_argument("--skip-ros", action="store_true", help="Skip ROS build")
    parser.add_argument("--skip-launch", action="store_true", help="Skip launching test node")
    args = parser.parse_args()

    ensure_in_container()

    if not args.skip_mcu:
        flash_mcu()
    else:
        print("⏭️ Skipping MCU flashing by request")

    if not args.skip_ros:
        build_ros()
    else:
        print("⏭️ Skipping ROS build by request")

    if not args.skip_launch:
        launch_test_node()
    else:
        print("⏭️ Skipping test node launch by request")

    print("\n🎉 Deployment sequence completed")


if __name__ == "__main__":
    main()
