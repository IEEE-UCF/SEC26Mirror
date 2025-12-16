#!/usr/bin/env bash
# Readme: To Run: 
# sudo chmod +x ros2_ws/src/secbot_vision/scripts/test_field_world.sh
# then Run 
# ros2_ws/src/secbot_vision/scripts/test_field_world.sh

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

# Export resource path for Gazebo to find the field mesh
# Points to .../share/secbot_vision/worlds so model://field resolves to .../worlds/field
export GZ_SIM_RESOURCE_PATH="$(ros2 pkg prefix --share secbot_vision)/worlds:$GZ_SIM_RESOURCE_PATH"

# ---- Start sim in background ----
echo "Launching field simulation..."
ros2 launch secbot_vision field_sim.launch.py &
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
sleep 8

# -------- Spawn test object (Mustard Bottle) --------
# Use ros2 pkg prefix to find the share directory dynamically
SHARE_DIR=$(ros2 pkg prefix --share secbot_vision)
MODEL="$SHARE_DIR/worlds/default/model.sdf"
WORLD="field_world"  # This must match the name in field.world

echo "Spawning mustard bottle for testing..."
ros2 run ros_gz_sim create -world "$WORLD" -name mustard_bottle_test -file "$MODEL" -x 1.0 -y 0.0 -z 0.5 -Y 0.0

# Open debug view
echo "Showing /vision/debug_image "
ros2 run image_tools showimage --ros-args -r image:=/vision/debug_image &

echo "---------------------------------------------------------"
echo "Field Test is running!"
echo "To verify detections, open a new terminal and run:"
echo "  source install/setup.bash"
echo "  ros2 topic echo /detected_objects"
echo "---------------------------------------------------------"

# Wait for background processes to finish
wait
