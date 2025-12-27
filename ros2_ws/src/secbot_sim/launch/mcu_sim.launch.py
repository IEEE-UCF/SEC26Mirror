"""
MCU Subsystem Simulator Launch File

Launches the mcu_subsystem_sim node that simulates Drive, Battery, and Heartbeat subsystems
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='secbot_sim',
            executable='mcu_subsystem_sim',
            name='mcu_subsystem_sim',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }],
        ),
    ])
