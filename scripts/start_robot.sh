#!/bin/bash

# 1. Define Workspace Path
WS_DIR="/home/ubuntu/ros2_workspaces"

# 2. Source ROS 2 Jazzy
# Check if the setup file exists to prevent confusing errors later
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "❌ Error: ROS 2 Jazzy setup file not found at /opt/ros/jazzy/setup.bash"
    exit 1
fi

# 3. Build the Workspace
echo "🐢 Starting Colcon Build..."

# Verify the directory exists before entering
if [ ! -d "$WS_DIR" ]; then
    echo "❌ Error: Workspace directory $WS_DIR does not exist."
    exit 1
fi

cd "$WS_DIR" || exit 1

# Do NOT use symlink install as it provides no benefit on a prod environment
if colcon build --executor sequential; then
    # 4. Announce Success
    echo "--------------------------------"
    echo "✅ colcon built successfully"
    echo "✅ Robot Started"
    echo "--------------------------------"

    # Source the new environment
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
    fi
    
    # Explicitly exit with success code
    exit 0
else
    # 5. Handle Failure
    echo "--------------------------------"
    echo "❌ Error: colcon build failed"
    echo "--------------------------------"
    
    # Explicitly exit with error code
    exit 1
fi