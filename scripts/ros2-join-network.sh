#!/bin/bash
# Source this script to join the robot's ROS2 graph from an external device.
#
# Usage:
#   source scripts/ros2-join-network.sh              # defaults to WiFi AP (192.168.4.1)
#   source scripts/ros2-join-network.sh 192.168.4.1  # Pi via WiFi AP
#   source scripts/ros2-join-network.sh 10.0.0.5     # Pi via Ethernet
#
# After sourcing, run: ros2 topic list

PI_IP="${1:-192.168.4.1}"

export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_STATIC_PEERS="${PI_IP}"

echo "[ros2-network] Configured to discover via ${PI_IP}"
echo "[ros2-network] Run: ros2 topic list"
