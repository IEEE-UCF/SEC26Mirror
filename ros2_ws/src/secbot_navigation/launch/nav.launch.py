#!/usr/bin/env python3
"""
Launch file for Real Robot Navigation
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('secbot_navigation')

    # Config Files
    nav_config = 'nav.yaml'
    arena_config = 'arena_layout.yaml'

    # Pathing Node
    pathing_node = Node(
        package='secbot_navigation',
        executable='pathing_node',
        name='pathing_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'config_file': nav_config,
            'arena_file': arena_config
        }]
    )

    return LaunchDescription([
        pathing_node
    ])
