#!/usr/bin/env python3
"""
In-container deployment script: flash MCU + build ROS2 + launch.

This script runs INSIDE the Docker container. For the main automated
deployment pipeline, see deploy-orchestrator.py (runs on the host).
"""

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

# Container paths (match docker-compose mounts)
SCRIPTS_DIR = Path("/home/ubuntu/scripts")
MCU_WS = Path("/home/ubuntu/mcu_workspaces/sec26mcu")
ROS_WS = Path("/home/ubuntu/ros2_workspaces")
ROS_INSTALL_SETUP = ROS_WS / "install" / "setup.bash"


def run(cmd, env=None, cwd=None, shell=False):
    print(f"$ {' '.join(cmd) if not shell else cmd}")
    try:
        subprocess.run(cmd, check=True, env=env, cwd=cwd, shell=shell)
    except subprocess.CalledProcessError as e:
        print(f"\n❌ Command failed: {e}")
        sys.exit(e.returncode or 1)


def try_run(cmd, env=None, cwd=None, shell=False) -> bool:
    """Run a command and return True/False instead of exiting on error."""
    print(f"$ {' '.join(cmd) if not shell else cmd}")
    try:
        subprocess.run(cmd, check=True, env=env, cwd=cwd, shell=shell)
        return True
    except subprocess.CalledProcessError as e:
        print(f"⚠️ Command failed (will handle): {e}")
        return False


def ensure_in_container():
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


def get_ros_distro() -> str:
    dist = os.environ.get("ROS_DISTRO")
    if not dist and Path("/opt/ros").exists():
        dists = [p.name for p in Path("/opt/ros").iterdir() if p.is_dir()]
        if dists:
            dist = sorted(dists)[-1]
    return dist or "jazzy"


def rosdep_update_and_install():
    """Ensure rosdep is available, updated, and install workspace deps."""
    distro = get_ros_distro()
    src_path = ROS_WS / "src"

    print("🔄 Updating apt-get package lists...")
    if not try_run(["bash", "-lc", "sudo apt-get update"]):
        print("⚠️ apt-get update had some issues, continuing...")

    if not try_run(["bash", "-lc", "command -v rosdep >/dev/null 2>&1"]):
        print("ℹ️ rosdep not found; installing via apt")
        if not try_run(["bash", "-lc", "sudo apt-get update && sudo apt-get install -y python3-rosdep"]):
            print("❌ Failed to install rosdep (python3-rosdep)")
            sys.exit(1)

    default_list = Path("/etc/ros/rosdep/sources.list.d/20-default.list")
    if not default_list.exists():
        print("ℹ️ rosdep not initialized; running 'sudo rosdep init'")
        if not try_run(["bash", "-lc", "sudo rosdep init"]):
            print("❌ Failed to initialize rosdep")
            sys.exit(1)

    updated = False
    for attempt in range(1, 4):
        print(f"🔄 rosdep update attempt {attempt}/3")
        if try_run(["bash", "-lc", "rosdep update || true"]):
            updated = True
            break
        time.sleep(2)

    if not updated:
        print("⚠️ rosdep update had issues, continuing anyway...")

    install_cmd = (
        f"sudo -n rosdep install --rosdistro {distro} --from-paths {src_path} -y --ignore-src --skip-keys ament_python "
        f"|| rosdep install --rosdistro {distro} --from-paths {src_path} -y --ignore-src --skip-keys ament_python"
    )
    if not try_run(["bash", "-lc", install_cmd], shell=False):
        print("❌ rosdep install failed for workspace dependencies")
        sys.exit(1)
    print("✅ rosdep dependencies installed")


def flash_mcu():
    script = SCRIPTS_DIR / "flash_mcu.sh"
    if not script.exists():
        print(f"❌ Missing script: {script}")
        sys.exit(1)
    print("🚀 Flashing MCU (Teensy)...")
    run(["bash", str(script)])
    print("✅ MCU flashing completed")


def build_ros():
    script = SCRIPTS_DIR / "start_robot.sh"
    if not script.exists():
        print(f"❌ Missing script: {script}")
        sys.exit(1)
    print("🛠️ Resolving dependencies with rosdep...")
    rosdep_update_and_install()
    print("🛠️ Building ROS 2 workspace with colcon...")
    run(["bash", str(script)])
    print("✅ ROS 2 build completed")


def main():
    parser = argparse.ArgumentParser(
        description="In-container deployment: MCU flash + ROS build")
    parser.add_argument("--skip-mcu", action="store_true",
                        help="Skip MCU flashing")
    parser.add_argument("--skip-ros", action="store_true",
                        help="Skip ROS build")
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

        print("\n🎉 Deployment sequence completed")
    except SystemExit:
        raise


if __name__ == "__main__":
    main()
