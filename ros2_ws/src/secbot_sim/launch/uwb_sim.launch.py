"""
UWB Simulation Launch File

Tier 3: Includes MCU sim (Tier 1) and adds the full sensor-fusion + autonomy stack:
  - UWB simulation + bridge + positioning node
  - Sensor fusion (fusion_node + ekf_node) -> /odometry/filtered
  - Vision + pathing (on raw /odom for now, switch to /odometry/filtered once EKF is confirmed)
  - RViz2
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_secbot_sim    = get_package_share_directory('secbot_sim')
    pkg_secbot_fusion = get_package_share_directory('secbot_fusion')
    pkg_secbot_nav    = get_package_share_directory('secbot_navigation')
    pkg_secbot_vision = get_package_share_directory('secbot_vision')
    pkg_my_robot      = get_package_share_directory('my_robot_description')

    rviz_config       = os.path.join(pkg_my_robot,      'rviz',   'secbot_sim.rviz')
    mcu_sim_launch    = os.path.join(pkg_secbot_sim,    'launch', 'mcu_sim_secbot.launch.py')
    ekf_config_path   = os.path.join(pkg_secbot_fusion, 'config', 'ekf.yaml')
    vision_config     = os.path.join(pkg_secbot_vision, 'config', 'vision.yaml')
    nav_config_path   = os.path.join(pkg_secbot_nav,    'config', 'nav_sim.yaml')
    arena_config_path = os.path.join(pkg_secbot_nav,    'config', 'arena_layout.yaml')

    return LaunchDescription([
        # 1. Tier 1: Gazebo + robot + bridge + mcu_subsystem_sim
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mcu_sim_launch)
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

        # --- UWB stack ---

        # 3. Vacabun UWB Simulation
        ExecuteProcess(
            cmd=[
                '/home/ubuntu/ros2_workspaces/install/uwb_simulation/lib/uwb_simulation/uwb_simulation_node',
                '--ros-args', '-p', 'label_name:=my_robot'
            ],
            cwd='/home/ubuntu/',
            output='screen'
        ),

        # 4. UWB message bridge
        ExecuteProcess(
            cmd=[
                'python3',
                '/home/ubuntu/ros2_workspaces/src/sec26ros/secbot_sim/scripts/convert_uwb_msgs.py'
            ],
            output='screen'
        ),

        # 5. UWB Positioning Node
        Node(
            package='secbot_uwb',
            executable='positioning_node',
            name='uwb_positioning_node',
            output='screen',
            parameters=['/home/ubuntu/ros2_workspaces/src/sec26ros/secbot_uwb/config/beacons.yaml'],
            arguments=['--ros-args', '--log-level', 'info']
        ),

        # --- Sensor fusion (runs in parallel, publishing /odometry/filtered) ---

        # 6. Fusion node: /drive_base/status -> /odom/unfiltered
        Node(
            package='secbot_fusion',
            executable='fusion_node',
            name='fusion_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        # 7. EKF: /odom/unfiltered + /mcu_robot/imu/data + /uwb/pose/robot -> /odometry/filtered
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[ekf_config_path, {'use_sim_time': True, 'publish_tf': True}],
        ),

        # --- Autonomy (using raw /odom for now; switch to /odometry/filtered once EKF is confirmed) ---

        # 8. Vision
        Node(
            package='secbot_vision',
            executable='detector_node',
            name='detector_node',
            output='screen',
            parameters=[vision_config, {'use_sim_time': True,
                                        'image_topic': '/camera1/image'}],
        ),

        # 9. Vision -> Goal converter
        Node(
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
        ),

        # 10. Pathing node
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
    ])
