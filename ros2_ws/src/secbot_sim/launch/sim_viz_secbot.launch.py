"""
sim_viz_secbot.launch.py — Full autonomy simulation + RViz visualization.

Includes sim_autonomy_secbot (Tier 2: Gazebo + MCU sim + vision + pathing)
and adds RViz2 for visualization.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_secbot_sim = get_package_share_directory('secbot_sim')
    pkg_my_robot   = get_package_share_directory('my_robot_description')

    rviz_config    = os.path.join(pkg_my_robot,   'rviz',   'secbot_sim.rviz')
    autonomy_launch = os.path.join(pkg_secbot_sim, 'launch', 'sim_autonomy_secbot.launch.py')

    return LaunchDescription([
        # 1. Tier 2: Gazebo + MCU sim + vision + pathing
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(autonomy_launch)
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
    ])
