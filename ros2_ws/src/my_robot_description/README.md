# my_robot_description

## Export from Onshape
```bash
/home/ubuntu/scripts/export_urdf.sh
```

## Build
```bash
cd /home/ubuntu/ros2_workspaces
colcon build --packages-select my_robot_description
source install/setup.bash
```

## View
```bash
ros2 launch my_robot_description display.launch.py
```
