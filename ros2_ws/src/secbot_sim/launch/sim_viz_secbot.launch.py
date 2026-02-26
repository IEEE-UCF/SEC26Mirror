"""
sim_viz_secbot.launch.py — Full MCU+nav simulation + RViz visualization.

Mirrors sim_viz.launch.py but uses the new robot_description package:
  - Gazebo  (from mcu_sim_secbot, which also starts RSP)
  - MCU subsystem simulator + pathing + vision nodes
  - ROS-Gazebo bridge
  - RViz2  (my_robot_description/rviz/urdf.rviz)
  - use this for our simulation testing since we can then actually check what is being shown via rviz
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_secbot_sim = get_package_share_directory('secbot_sim')
    pkg_my_robot   = get_package_share_directory('my_robot_description')

    # Rich RViz config with odom, global path, local trajectory, goal, junctions
    rviz_config     = os.path.join(pkg_my_robot,   'rviz',   'secbot_sim.rviz')
    mcu_sim_launch  = os.path.join(pkg_secbot_sim, 'launch', 'mcu_sim_secbot.launch.py')

    return LaunchDescription([
        # 1. Full simulation backend (Gazebo + MCU + pathing + vision + RSP + bridge)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mcu_sim_launch)
        ),

        # 2. RViz2 — visualise robot model, planned path, TF frames
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}]
        ),
    ])
