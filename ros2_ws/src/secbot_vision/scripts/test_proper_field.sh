#!/usr/bin/env bash
# Readme: To Run: 
# sudo chmod +x src/sec26ros/secbot_vision/scripts/test_proper_field.sh
# then Run 
# src/sec26ros/secbot_vision/scripts/test_proper_field.sh

set -e

source /opt/ros/jazzy/setup.bash

# ---- Source ROS ----
echo "Building secbot_vision..."
if [ -d "$HOME/ros2_ws" ]; then
    cd "$HOME/ros2_ws"
elif [ -d "$HOME/ros2_workspaces" ]; then
    cd "$HOME/ros2_workspaces"
else
    echo "Warning: Could not find ~/ros2_ws or ~/ros2_workspaces. Assuming current directory is workspace root."
fi

colcon build --packages-select secbot_vision secbot_msgs --symlink-install

echo "Sourcing workspace..."
source install/setup.bash

# Ensure image_tools and vision_msgs are installed
if ! ros2 pkg prefix image_tools >/dev/null 2>&1 || ! ros2 pkg prefix vision_msgs >/dev/null 2>&1; then
  echo "Installing dependencies..."
  if command -v sudo >/dev/null 2>&1; then
    sudo apt-get update
    sudo apt-get install -y ros-jazzy-image-tools ros-jazzy-vision-msgs
  else
    apt-get update
    apt-get install -y ros-jazzy-image-tools ros-jazzy-vision-msgs
  fi
fi

# Export resource path for Gazebo to find the field mesh
# Points to .../share/secbot_vision/worlds so model://proper_field resolves correctly if needed
export GZ_SIM_RESOURCE_PATH="$(ros2 pkg prefix --share secbot_vision)/worlds:$GZ_SIM_RESOURCE_PATH"

# ---- Start sim in background ----
echo "Launching Proper Field simulation..."
# You can add num_blocks:=5 or x_offset:=1.0 here if desired
ros2 launch secbot_vision proper_field_sim.launch.py &
LAUNCH_PID=$!

# Get the process group ID for the launch (so we can kill the whole tree)
PGID="$(ps -o pgid= "$LAUNCH_PID" | tr -d ' ')"

SHUTTING_DOWN=0
cleanup() {
  if [ "$SHUTTING_DOWN" -eq 1 ]; then return; fi
  SHUTTING_DOWN=1
  set +e
  echo ""
  echo "Shutting down..."

  # Graceful first
  kill -INT "-$PGID" 2>/dev/null || true
  sleep 0.5

  # Then harder if needed
  kill -TERM "-$PGID" 2>/dev/null || true
  sleep 0.5

  # Last resort
  kill -KILL "-$PGID" 2>/dev/null || true
}
trap cleanup INT TERM EXIT

# ---- Give sim time ----
sleep 5

# Open debug view
echo "Showing /vision/debug_image "
ros2 run image_tools showimage --ros-args -r image:=/vision/debug_image &
echo "Showing /camera/image_raw "
ros2 run image_tools showimage --ros-args -r image:=/camera/image_raw &

echo "---------------------------------------------------------"
echo "Proper Field Test is running!"
echo "Configuration is loaded from config/spawn_locations.yaml"
echo "To verify detections, open receive /detected_objects"
echo "---------------------------------------------------------"

# Wait for background processes to finish
wait
