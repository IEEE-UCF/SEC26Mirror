"""
sim_viz_draw_secbot.launch.py — Draw mode (empty world) + pathing + RViz.

Includes Tier 1 MCU sim and adds pathing_node with empty arena config
for 2D Goal Pose testing. No vision nodes.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_secbot_sim = get_package_share_directory('secbot_sim')
    pkg_secbot_nav = get_package_share_directory('secbot_navigation')
    pkg_my_robot   = get_package_share_directory('my_robot_description')

    rviz_config       = os.path.join(pkg_my_robot,   'rviz',   'urdf.rviz')
    mcu_sim_launch    = os.path.join(pkg_secbot_sim, 'launch', 'mcu_sim_secbot.launch.py')
    nav_config_path   = os.path.join(pkg_secbot_nav, 'config', 'nav_sim.yaml')
    arena_config_path = os.path.join(pkg_secbot_nav, 'config', 'empty_arena.yaml')

    return LaunchDescription([
        # 1. Tier 1: Gazebo + robot + bridge + mcu_subsystem_sim
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mcu_sim_launch)
        ),

        # 2. Pathing node (draw mode — empty arena, no vision)
        Node(
            package='secbot_navigation',
            executable='pathing_node',
            name='pathing_node',
            output='screen',
            parameters=[{
                'use_sim':      True,
                'use_sim_time': True,
                'config_file':  nav_config_path,
                'arena_file':   arena_config_path,
            }],
        ),

        # 3. RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}]
        ),
    ])
