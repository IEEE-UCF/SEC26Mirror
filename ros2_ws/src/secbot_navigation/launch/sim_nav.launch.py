#!/usr/bin/env python3
"""
Launch file for proper field world in Gazebo Harmonic with ROS 2 bridge
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_secbot_nav = get_package_share_directory('secbot_navigation')
    config_file_path = os.path.join(pkg_secbot_nav, 'config', 'nav_sim.yaml')
    arena_file_path = os.path.join(pkg_secbot_nav, 'config', 'arena_layout.yaml')

    # run the pathing node
    pathing_node = Node(
        package='secbot_navigation',
        executable='pathing_node',
        name='pathing_node',
        output='screen',
        parameters=[{
            'use_sim': True,
            'config_file': config_file_path,
            'arena_file': arena_file_path,
        }]
    )


    return LaunchDescription([
        pathing_node
    ])
