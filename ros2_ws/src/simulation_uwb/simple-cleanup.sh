cd ~/ros2_workspaces

# 1. Delete the old cached build files
rm -rf build/uwb_interfaces build/uwb_simulation

# 2. Delete the old installation files
rm -rf install/uwb_interfaces install/uwb_simulation

# 3. Rebuild!
colcon build --packages-select uwb_interfaces uwb_simulation secbot_jh_test