# Go to the package folder
cd ~/ros2_workspaces/src/sec26ros/secbot_jh_test

# Delete the build artifacts that shouldn't be here
rm -rf build install log

cd ~/ros2_workspaces

# This puts the build files in ~/ros2_workspaces/install (where they belong)
colcon build --packages-select secbot_jh_test --symlink-install

# This updates your path so ROS can find the new package
source install/setup.bash

ros2 run secbot_jh_test keystroke_subscribe_node