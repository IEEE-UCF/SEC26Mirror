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
# ---- Clean stale msg build artifacts (symlink vs directory conflict) ----
rm -rf build/mcu_msgs install/mcu_msgs
rm -rf build/secbot_msgs install/secbot_msgs
# ---- Build ----
colcon build --packages-select mcu_msgs secbot_msgs my_robot_description secbot_sim secbot_navigation secbot_fusion secbot_vision
# ---- Ensure image_tools is available ----
if ! ros2 pkg prefix image_tools > /dev/null 2>&1; then
  echo "Installing image_tools..."
  if command -v sudo > /dev/null 2>&1; then
    sudo apt-get update && sudo apt-get install -y ros-jazzy-image-tools
  else
    apt-get update && apt-get install -y ros-jazzy-image-tools
  fi
fi
# ---- Start sim in the background ----
echo "Launching mcu_sim_secbot..."
ros2 launch secbot_sim sim_viz_secbot.launch.py &
LAUNCH_PID=$!
# Capture process group so Ctrl+C kills everything
PGID="$(ps -o pgid= "$LAUNCH_PID" | tr -d ' ')"
# ---- Wait for sim to initialise then open camera views ----
echo "Waiting for sim to start..."
sleep 5
echo "Opening camera views..."
ros2 run image_tools showimage --ros-args -r image:=/camera1/image  &
ros2 run image_tools showimage --ros-args -r image:=/vision/debug_image &
ros2 run image_tools showimage --ros-args -r image:=/camera2/image &
echo "---------------------------------------------------------"
echo "SECbot sim is running!"
echo "  Camera 1: /camera1"
echo "  Debug Image: /vision/debug_image"
echo "  Camera 2: /camera2"
echo "Press Ctrl+C to stop everything."
echo "---------------------------------------------------------"
# Wait for the launch process (and camera windows) to finish
wait