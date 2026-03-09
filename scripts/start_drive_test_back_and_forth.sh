#!/bin/bash
# Tares IMU heading, then runs back-and-forth drive test (no turns)

WS_DIR="/home/ubuntu/ros2_workspaces"
source /opt/ros/jazzy/setup.bash
cd "$WS_DIR" || exit 1

colcon build --packages-select secbot_autonomy mcu_msgs
source install/setup.bash

# Zero IMU heading before test
echo "Taring IMU..."
ros2 service call /mcu_robot/imu/tare mcu_msgs/srv/Reset --once
sleep 1

ros2 run secbot_autonomy drive_test_node --ros-args \
  -p loop:=true \
  -p with_turn:=false \
  -p distance:=0.5 \
  -p goal_tolerance:=0.03 \
  -p goal_timeout:=10.0 \
  -p calibrate_time:=3.0
