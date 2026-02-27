"""
UWB Simulation Launch File

Tier 3: Includes MCU sim (Tier 1) and adds the full sensor-fusion autonomy stack:
  - UWB simulation + bridge + positioning node
  - Sensor fusion (fusion_node + ekf_node) -> /odometry/filtered
  - Vision + pathing (remapped to use /odometry/filtered)
  - RViz2
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_secbot_sim    = get_package_share_directory('secbot_sim')
    pkg_secbot_fusion = get_package_share_directory('secbot_fusion')
    pkg_secbot_nav    = get_package_share_directory('secbot_navigation')
    pkg_secbot_vision = get_package_share_directory('secbot_vision')
    pkg_my_robot      = get_package_share_directory('my_robot_description')

    rviz_config       = os.path.join(pkg_my_robot,      'rviz',   'secbot_sim.rviz')
    mcu_sim_launch    = os.path.join(pkg_secbot_sim,    'launch', 'mcu_sim_secbot.launch.py')
    ekf_config_path   = os.path.join(pkg_secbot_fusion, 'config', 'ekf.yaml')
    vision_config     = os.path.join(pkg_secbot_vision, 'config', 'vision.yaml')
    nav_config_path   = os.path.join(pkg_secbot_nav,    'config', 'nav_sim.yaml')
    arena_config_path = os.path.join(pkg_secbot_nav,    'config', 'arena_layout.yaml')

    return LaunchDescription([
        # 1. Tier 1: Gazebo + robot + bridge + mcu_subsystem_sim
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mcu_sim_launch)
        ),

        # 2. RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}]
        ),

        # --- UWB stack ---

        # 3. Vacabun UWB Simulation
        ExecuteProcess(
            cmd=[
                '/home/ubuntu/ros2_workspaces/install/uwb_simulation/lib/uwb_simulation/uwb_simulation_node',
                '--ros-args', '-p', 'label_name:=my_robot'
            ],
            cwd='/home/ubuntu/',
            output='screen'
        ),

        # 4. UWB message bridge
        ExecuteProcess(
            cmd=[
                'python3',
                '/home/ubuntu/ros2_workspaces/src/sec26ros/secbot_sim/scripts/convert_uwb_msgs.py'
            ],
            output='screen'
        ),

        # 5. UWB Positioning Node
        Node(
            package='secbot_uwb',
            executable='positioning_node',
            name='uwb_positioning_node',
            output='screen',
            parameters=['/home/ubuntu/ros2_workspaces/src/sec26ros/secbot_uwb/config/beacons.yaml'],
            # This helps debug what the node actually "sees"
            arguments=['--ros-args', '--log-level', 'info'] 
        )
    ])