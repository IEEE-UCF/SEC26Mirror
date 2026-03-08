#!/bin/bash
# Builds secbot_autonomy and runs the forward direction integration test

WS_DIR="/home/ubuntu/ros2_workspaces"
source /opt/ros/jazzy/setup.bash
cd "$WS_DIR" || exit 1

colcon build --packages-select secbot_autonomy mcu_msgs
source install/setup.bash

ros2 run secbot_autonomy drive_test_node --ros-args \
  -p speed:=0.2 \
  -p leg_time:=3.0
