#!/bin/bash

# 1. Define Workspace Path
WS_DIR="/home/rosdev/ros2_workspaces"

# 2. Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# 3. Build the Workspace
echo "🐢 Starting Colcon Build..."
cd $WS_DIR

# Using symlink-install is faster and allows python editing without rebuilding
colcon build --symlink-install --executor sequential

# 4. Announce Success
echo "--------------------------------"
echo "✅ colcon built"
echo "✅ Robot Started"
echo "--------------------------------"

# (Optional) If you want to verify the environment is ready for future commands:
source install/setup.bash