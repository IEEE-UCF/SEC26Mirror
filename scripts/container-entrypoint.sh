#!/bin/bash
# Container entrypoint — starts micro-ROS agents, deploy node, and keeps container alive.
# Serial agent reconnects automatically when Teensy reboots during flashing.
# Guards against duplicate processes (safe if user also docker exec's in).

source /opt/ros/jazzy/setup.bash

# Source workspace install if it exists (may not on first boot before colcon build)
if [ -f /home/ubuntu/ros2_workspaces/install/setup.bash ]; then
  source /home/ubuntu/ros2_workspaces/install/setup.bash
fi

# Build secbot_deploy on first boot if not already built
if ! ros2 pkg list 2>/dev/null | grep -q secbot_deploy; then
  echo "[entrypoint] Building secbot_deploy (first boot)..."
  (
    cd /home/ubuntu/ros2_workspaces
    colcon build --packages-select secbot_deploy 2>&1 || true
    if [ -f install/setup.bash ]; then
      source install/setup.bash
    fi
  )
fi

# Re-source after potential build
if [ -f /home/ubuntu/ros2_workspaces/install/setup.bash ]; then
  source /home/ubuntu/ros2_workspaces/install/setup.bash
fi

# Helper: check if a live (non-zombie) process matching a pattern exists
is_running() {
  local pids
  pids=$(pgrep -f "$1" 2>/dev/null) || return 1
  for pid in $pids; do
    local state
    state=$(awk '{print $3}' /proc/"$pid"/stat 2>/dev/null)
    [ "$state" != "Z" ] && return 0
  done
  return 1
}

# Serial agent for Teensy (auto-reconnect loop)
(
  while true; do
    if [ -e /dev/ttyACM0 ] && ! is_running "micro_ros_agent serial"; then
      echo "[entrypoint] Starting serial micro-ROS agent on /dev/ttyACM0"
      ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 921600 || true
      wait $! 2>/dev/null  # reap child
    fi
    sleep 2
  done
) &

# UDP agent for ESP32s (auto-reconnect loop)
(
  while true; do
    if ! is_running "micro_ros_agent udp4"; then
      echo "[entrypoint] Starting UDP micro-ROS agent on port 8888"
      ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 || true
      wait $! 2>/dev/null  # reap child
    fi
    sleep 2
  done
) &

# Deploy bridge node (auto-restart loop)
(
  sleep 10  # Wait for micro-ROS agents to start first
  while true; do
    if ! is_running "secbot_deploy" && ros2 pkg list 2>/dev/null | grep -q secbot_deploy; then
      echo "[entrypoint] Starting secbot_deploy node"
      ros2 launch secbot_deploy deploy.launch.py || true
      wait $! 2>/dev/null
    fi
    sleep 5
  done
) &

# Keep container alive
exec sleep infinity
