#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav_share = get_package_share_directory('secbot_navigation')

    nav_params = os.path.join(nav_share, 'config', 'nav_sim.yaml')
    arena_file = os.path.join(nav_share, 'config', 'arena_layout.yaml')

    pathing_node = Node(
        package='secbot_navigation',
        executable='pathing_node',
        name='pathing_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'config_file': 'nav_sim.yaml',
            'arena_file': 'arena_layout.yaml',
        }]
    )


    return LaunchDescription([
        pathing_node
    ])
