# Readme: To Run: 
# sudo chmod +x ros2_workspaces/src/sec26ros/src/robot_workshop/scripts/test_duck_detector.sh
# then Run 
# ros2_workspaces/src/sec26ros/src/robot_workshop/scripts/test_duck_detector.sh

#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash

# ---- Source ROS ----
echo "Building robot_workshop..."
cd ~/ros2_workspaces
colcon build --packages-select robot_workshop robot_workshop_msgs --symlink-install

echo "Sourcing workspace..."
source install/setup.bash

echo "Verifying interfaces..."
if ! ros2 interface show robot_workshop_msgs/msg/DuckDetections >/dev/null; then
    echo "ERROR: Could not find DuckDetections message interface! Build might have failed."
    exit 1
fi
echo "Interface Verified: robot_workshop_msgs/msg/DuckDetections"


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


# ---- Start sim in background ----
echo "Launching simulation..."
ros2 launch robot_workshop robot_sim.launch.py image_topic:=/camera2/image_raw &
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
sleep 2

# -------- Spawn mustard bottles --------
MODEL="/home/ubuntu/ros2_workspaces/src/sec26ros/src/robot_workshop/worlds/model.sdf"

echo "Spawning mustard bottles..."
ros2 run ros_gz_sim create -world "$WORLD" -name mustard_bottle_0 -file "$MODEL" -x 1.0 -y 0.0 -z 0.15 -Y 0.0
ros2 run ros_gz_sim create -world "$WORLD" -name mustard_bottle_1 -file "$MODEL" -x 1.0 -y 0.3 -z 0.15 -Y 0.0
ros2 run ros_gz_sim create -world "$WORLD" -name mustard_bottle_2 -file "$MODEL" -x 1.0 -y -0.6 -z 0.15 -Y 0.0

# echo "Showing /vision/debug_image "
ros2 run image_tools showimage --ros-args -r image:=/vision/debug_image &
echo "Showing /camera/image_raw "
ros2 run image_tools showimage --ros-args -r image:=/camera/image_raw &

echo "---------------------------------------------------------"
echo "Test is running!"
echo "To verify detections, open a new terminal and run:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source install/setup.bash"
echo "  ros2 topic echo /duck_detections"
echo "  ros2 topic echo /detected_objects"
echo "---------------------------------------------------------"

# Wait for background processes to finish (optional, or just keep script running)
wait