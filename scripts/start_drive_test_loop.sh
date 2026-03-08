#!/bin/bash
# Builds secbot_autonomy and runs the back-and-forth encoder/IMU accuracy test

WS_DIR="/home/ubuntu/ros2_workspaces"
source /opt/ros/jazzy/setup.bash
cd "$WS_DIR" || exit 1

colcon build --packages-select secbot_autonomy mcu_msgs
source install/setup.bash

ros2 run secbot_autonomy drive_test_node --ros-args \
  -p loop:=true \
  -p with_turn:=true \
  -p distance:=0.5 \
  -p turn_angle:=1.5708 \
  -p goal_tolerance:=0.03 \
  -p goal_timeout:=10.0 \
  -p calibrate_time:=3.0
