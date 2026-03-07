source ~/ros2_workspaces/install/setup.bash
ros2 run secbot_jh_test keystroke_publish_node
# # Go to the package folder
# cd ~/ros2_workspaces/src/sec26ros/secbot_jh_test

# # Delete the build artifacts that shouldn't be here
# rm -rf build install log

# cd ~/ros2_workspaces

# # This puts the build files in ~/ros2_workspaces/install (where they belong)
# colcon build --packages-select secbot_jh_test --symlink-install

# # This updates your path so ROS can find the new package
# source install/setup.bash

# ros2 run secbot_jh_test keystroke_publish_node
#must be the name of the cmake executable you made

#now you might be asking as to why we don't use a launch file in this scenario....
#well since we specifically are using this executable as a typing node all it needs to 
#do is PUBLISH...its not really interacting with GazebO(other than just sending msgs to a topic which the gazebo node we will now implement)