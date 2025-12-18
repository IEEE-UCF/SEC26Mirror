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
PREBUILT_DIR = MCU_WS / "prebuilt"


def stage_prebuilt_artifacts():
    # Copy prebuilt firmware into PlatformIO build folders so upload can run without compiling.
    if not PREBUILT_DIR.exists():
        print(f"ℹ️ No prebuilt directory found at {PREBUILT_DIR}; will fall back to normal build")
        return

    robot_build_dir = MCU_WS / ".pio" / "build" / "robot"
    robotcomms_build_dir = MCU_WS / ".pio" / "build" / "robotcomms"
    robot_build_dir.mkdir(parents=True, exist_ok=True)
    robotcomms_build_dir.mkdir(parents=True, exist_ok=True)

    # Search broadly in PREBUILT_DIR to handle nested paths from artifact download
    robot_candidates = []
    robotcomms_candidates = []
    for pattern in ["**/robot/firmware.*", "**/robot/**/*.hex", "**/robot/**/*.elf"]:
        robot_candidates.extend(PREBUILT_DIR.glob(pattern))
    for pattern in ["**/robotcomms/firmware.*", "**/robotcomms/**/*.bin", "**/robotcomms/**/*.elf"]:
        robotcomms_candidates.extend(PREBUILT_DIR.glob(pattern))

    staged = False
    if robot_candidates:
        # Prefer firmware.hex if present; else take first match
        preferred = next((p for p in robot_candidates if p.name == "firmware.hex"), robot_candidates[0])
        target = robot_build_dir / "firmware.hex"
        print(f"📦 Staging Teensy artifact: {preferred} -> {target}")
        target.write_bytes(preferred.read_bytes())
        staged = True

    if robotcomms_candidates:
        # Prefer firmware.bin for ESP32; fall back to firmware.elf if needed
        preferred = next((p for p in robotcomms_candidates if p.suffix == ".bin" and p.name.startswith("firmware")), None)
        if preferred is None:
            preferred = next((p for p in robotcomms_candidates if p.name == "firmware.elf"), robotcomms_candidates[0])
        # Keep original extension
        target = robotcomms_build_dir / preferred.name
        print(f"📦 Staging Comm ESP32 artifact: {preferred} -> {target}")
        target.write_bytes(preferred.read_bytes())
        staged = True

    if staged:
        print("✅ Prebuilt artifacts staged into .pio/build/{robot,robotcomms}")
    else:
        print("ℹ️ No matching prebuilt artifacts found; will fall back to normal build")


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
    # Stage artifacts (if any) before flashing, to enable upload without rebuild
    stage_prebuilt_artifacts()
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
    try:
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
    except SystemExit as e:
        raise


if __name__ == "__main__":
    main()
