#!/bin/bash
# Builds secbot_autonomy, launches the FSM, and auto-triggers the mission start

WS_DIR="/home/ubuntu/ros2_workspaces"
source /opt/ros/jazzy/setup.bash
cd "$WS_DIR" || exit 1

colcon build --packages-select secbot_autonomy mcu_msgs secbot_msgs
source install/setup.bash

# Kill background launch and all its children on Ctrl+C
cleanup() {
  kill -INT "$LAUNCH_PID" 2>/dev/null
  wait "$LAUNCH_PID" 2>/dev/null
  exit 0
}
trap cleanup SIGINT SIGTERM

# Launch nodes in background, call the start service, then wait
ros2 launch secbot_autonomy mission.launch.py &
LAUNCH_PID=$!

# Wait for nodes to spin up, then trigger the mission
sleep 3
ros2 service call /mission/start std_srvs/srv/Trigger

# Keep running until Ctrl+C
wait $LAUNCH_PID
