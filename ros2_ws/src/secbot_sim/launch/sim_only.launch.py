"""
sim_only.launch.py — Minimal Gazebo launch for drive testing.

Starts ONLY:
  - Gazebo (world + robot spawned)
  - ROS-Gazebo bridge (/cmd_vel, /odom, /tf, /joint_states)
  - Robot State Publisher

NO navigation, NO vision, NO pathing — so drive_test.py has
exclusive control of /cmd_vel.
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_secbot_sim = get_package_share_directory('secbot_sim')
    pkg_my_robot   = get_package_share_directory('my_robot_description')

    world_file      = os.path.join(pkg_secbot_sim, 'worlds', 'bot', 'my_bot_harmonic.sdf')
    robot_urdf_file = os.path.join(pkg_my_robot,   'urdf',   'robot.urdf')
    bridge_config   = os.path.join(pkg_my_robot,   'config', 'ros_gz_bridge.yaml')

    # Mesh resolution: include my_robot_description share parent
    my_robot_share_parent = os.path.dirname(pkg_my_robot)
    secbot_sim_worlds     = os.path.join(pkg_secbot_sim, 'worlds')
    gz_resource_path      = secbot_sim_worlds + os.pathsep + my_robot_share_parent
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        gz_resource_path += os.pathsep + os.environ['GZ_SIM_RESOURCE_PATH']

    with open(robot_urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        # 1. Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', world_file, '-r'],
            output='screen',
            additional_env={'GZ_SIM_RESOURCE_PATH': gz_resource_path}
        ),

        # 2. Spawn robot
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-file', robot_urdf_file, '-name', 'my_robot',
                       '-x', '0', '-y', '0', '-z', '0.2'],
            output='screen'
        ),

        # 3. ROS↔Gazebo bridge (/cmd_vel, /odom, /tf, /joint_states, cameras)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # 4. Robot State Publisher (for RViz / TF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': True}]
        ),
    ])
