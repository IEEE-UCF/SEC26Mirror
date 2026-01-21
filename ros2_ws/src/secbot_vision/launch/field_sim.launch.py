#!/usr/bin/env python3
"""
Launch file for field world in Gazebo Harmonic with ROS 2 bridge
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('secbot_vision')
    
    # Path to World file
    world_file = os.path.join(
        pkg_share,
        'worlds',
        'field',
        'field.world'
    )
    
    # Path to Robot SDF for spawning
    robot_sdf_file = os.path.join(
        pkg_share,
        'worlds',
        'default',
        'my_bot.sdf'
    )

    # Path to bridge config
    bridge_config = os.path.join(
        pkg_share,
        'config',
        'ros_gz_bridge.yaml'
    )

    # Gazebo Sim (Harmonic)
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '-r'],
        output='screen'
    )

    # ROS-Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_config}'
        ],
        output='screen'
    )

    # Spawn Robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', robot_sdf_file,
            '-name', 'my_bot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2'
        ],
        output='screen'
    )

    vision_config = os.path.join(
        pkg_share, 
        'config', 
        'vision.yaml')
        
    vision = Node(
        package = 'secbot_vision',
        executable='detector_node',
        name='detector_node',
        output='screen',
        parameters=[vision_config]
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        spawn_robot,
        vision,
    ])
