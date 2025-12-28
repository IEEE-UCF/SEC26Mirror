#!/usr/bin/env python3
"""
Launch file for UWB positioning system (C++ version)

This launch file starts the UWB positioning node with beacon configuration.

Usage:
    ros2 launch secbot_uwb uwb_positioning.launch.py
    ros2 launch secbot_uwb uwb_positioning.launch.py config_file:=/path/to/custom_config.yaml

@author SEC26 Team
@date 12/27/2025
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('secbot_uwb')

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'beacons.yaml'),
        description='Path to beacon configuration YAML file'
    )

    # UWB Positioning Node (C++ executable)
    positioning_node = Node(
        package='secbot_uwb',
        executable='positioning_node',
        name='uwb_positioning_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        emulate_tty=True,
    )

    return LaunchDescription([
        config_file_arg,
        positioning_node,
    ])
