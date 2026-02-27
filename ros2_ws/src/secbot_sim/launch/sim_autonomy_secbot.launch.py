"""
Autonomy Simulation Launch File — Secbot (URDF) Variant

Tier 2: Includes the MCU sim (Tier 1) and adds the autonomy stack:
  - Vision (detector_node)
  - Vision-to-goal converter
  - Pathing node (navigation)

Uses raw /odom from Gazebo. For filtered odometry via sensor fusion,
use uwb_sim.launch.py instead.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # package directories =====================================
    pkg_secbot_sim    = get_package_share_directory('secbot_sim')
    pkg_secbot_nav    = get_package_share_directory('secbot_navigation')
    pkg_secbot_vision = get_package_share_directory('secbot_vision')

    # configs =================================================
    vision_config     = os.path.join(pkg_secbot_vision, 'config', 'vision.yaml')
    nav_config_path   = os.path.join(pkg_secbot_nav,    'config', 'nav_sim.yaml')
    arena_config_path = os.path.join(pkg_secbot_nav,    'config', 'arena_layout.yaml')

    # Tier 1: MCU sim backend =================================
    mcu_sim_launch = os.path.join(pkg_secbot_sim, 'launch', 'mcu_sim_secbot.launch.py')

    # Vision node =============================================
    vision = Node(
        package='secbot_vision',
        executable='detector_node',
        name='detector_node',
        output='screen',
        parameters=[vision_config, {'use_sim_time': True,
                                    'image_topic': '/camera1/image'}],
    )

    # Vision -> Goal converter ================================
    vision_to_goal = Node(
        package='secbot_sim',
        executable='convert_vision_to_goal.py',
        name='vision_to_goal',
        output='screen',
        parameters=[{
            'detections_topic':    '/duck_detections',
            'camera_info_topic':   '/camera1/camera_info',
            'use_sim_time':        True,
            'odom_topic':          '/odom',
            'camera_height':       0.34,
            'camera_tilt_deg':     0,
            'goal_standoff':       0.3,
            'min_confidence':      60.0,
            'use_largest_area':    True,
            'priority_ids':        [3, 1, 7, 2],
            'republish_hz':        1.0,
            'target_timeout_sec':  1.0,
        }],
    )

    # Pathing node ============================================
    pathing_node = Node(
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
    )

    return LaunchDescription([
        # Tier 1: Gazebo + robot + bridge + mcu_subsystem_sim
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mcu_sim_launch)
        ),

        # Autonomy stack
        vision,
        vision_to_goal,
        pathing_node,
    ])
