#!/usr/bin/env bash
set -euo pipefail

# (Optional) if you're in a shared environment and want stricter isolation:
# export ROS_LOCALHOST_ONLY=1

SHUTTING_DOWN=0
cleanup() {
  if [ "$SHUTTING_DOWN" -eq 1 ]; then return; fi
  SHUTTING_DOWN=1
  set +e
  echo ""
  echo "Shutting down..."

  # Kill the whole launch session/process group if we started it
  if [ -n "${PGID:-}" ]; then
    kill -INT  "-$PGID" 2>/dev/null || true
    sleep 0.5
    kill -TERM "-$PGID" 2>/dev/null || true
    sleep 0.5
    kill -KILL "-$PGID" 2>/dev/null || true
  fi

  # Last-resort cleanup for stragglers (container-only)
  pkill -f ros_gz_bridge/parameter_bridge 2>/dev/null || true
  pkill -f "gz sim" 2>/dev/null || true
  pkill -f "gz-gui" 2>/dev/null || true
}
trap cleanup INT TERM EXIT

# ---- (Optional) pre-clean old stuff from previous runs ----
pkill -f ros_gz_bridge/parameter_bridge 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
pkill -f "gz-gui" 2>/dev/null || true

# ---- Build ----
colcon build --packages-select secbot_msgs
colcon build --packages-select secbot_sim
colcon build --packages-select secbot_navigation
colcon build --packages-select secbot_fusion
colcon build --packages-select secbot_vision
set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

# ---- Start sim in background, but in its OWN session ----
# setsid makes PID == PGID so kill -PGID reliably kills everything under it
ros2 launch secbot_sim mcu_sim.launch.py 