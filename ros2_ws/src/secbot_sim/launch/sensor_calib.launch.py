"""
sensor_calib.launch.py
======================
Minimal launch file for encoder & IMU calibration testing.

Starts ONLY what is needed for the calibration script:
  - Gazebo Harmonic (empty world with robot)
  - ROS↔Gazebo bridge  (/cmd_vel, /odom, /tf, /joint_states, /imu)
  - Robot State Publisher
  - MCU Subsystem Sim  (publishes /drive_base/status, bridges /cmd_vel_in)

NO vision, NO pathing, NO navigation, NO yellow-box spawning.
The calibration script (motion_calibration_test.py) has exclusive
control of /cmd_vel_in and will not be interrupted by any other node.

Usage (inside Docker container):
  ros2 launch secbot_sim sensor_calib.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Package paths ─────────────────────────────────────────
    pkg_secbot_sim = get_package_share_directory('secbot_sim')
    pkg_my_robot   = get_package_share_directory('my_robot_description')

    # ── Files ────────────────────────────────────────────────
    world_file      = os.path.join(pkg_secbot_sim, 'worlds', 'bot', 'my_bot_harmonic.sdf')
    robot_urdf_file = os.path.join(pkg_my_robot,   'urdf',   'robot.urdf')
    bridge_config   = os.path.join(pkg_my_robot,   'config', 'ros_gz_bridge.yaml')

    # ── GZ resource path (so meshes resolve correctly) ───────
    my_robot_share_parent = os.path.dirname(pkg_my_robot)   # …/share
    secbot_sim_worlds     = os.path.join(pkg_secbot_sim, 'worlds')
    gz_resource_path      = secbot_sim_worlds + os.pathsep + my_robot_share_parent
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        gz_resource_path += os.pathsep + os.environ['GZ_SIM_RESOURCE_PATH']

    # ── Read URDF for robot_state_publisher ──────────────────
    with open(robot_urdf_file, 'r') as f:
        robot_description = f.read()

    # ── Nodes ────────────────────────────────────────────────

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '-r'],
        output='screen',
        additional_env={'GZ_SIM_RESOURCE_PATH': gz_resource_path}
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', robot_urdf_file,
            '-name', 'my_robot',
            '-x', '0', '-y', '0', '-z', '0.2',
        ],
        output='screen'
    )

    # Bridge — includes /imu, /joint_states, /odom, /tf, /cmd_vel
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }]
    )

    return LaunchDescription([
        gz_sim,
        spawn_robot,
        bridge,
        robot_state_publisher,
    ])
