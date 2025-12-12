#!/usr/bin/env python3
"""
Launch file for my_bot in Gazebo Harmonic with ROS 2 bridge
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to SDF file
    sdf_file = os.path.join(
        get_package_share_directory('robot_workshop'),
        'worlds',
        'my_bot_harmonic.sdf'
    )

    # Path to bridge config
    bridge_config = os.path.join(
        get_package_share_directory('robot_workshop'),
        'config',
        'ros_gz_bridge.yaml'
    )

    nav_sim_cofig = os.path.join(
        get_package_share_directory('robot_workshop'),
        'config',
        'nav_sim.yaml'
    )

    arena_config = os.path.join(
        get_package_share_directory('robot_workshop'),
        'config',
        'arena_layout.yaml'
    )

    # Gazebo Sim (Harmonic)
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', sdf_file, '-r'],
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

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=arena_config,
        description='Full path to map yaml file'
    )

    map_yaml = LaunchConfiguration('map')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server_node',
        output='screen',
        parameters=[
            nav_sim_cofig,                     # if you need other params
            {'yaml_filename': arena_config},       # THIS tells it which map
        ],
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav_sim_cofig],
    )


    # === Lifecycle manager (does configure + activate for map_server_node) ===
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,                # this is what runs the lifecycle steps
            'node_names': ['map_server_node', 'amcl']
        }],
    )

    rviz_config = os.path.join(
        get_package_share_directory('robot_workshop'),
        'rviz',
        'nav2_map.rviz'
    )


    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    odom_tf_broadcaster = Node(
        package='robot_workshop',
        executable='odom_tf_broadcaster',
        name='odom_tf_broadcaster',
        output='screen',
    )


    # === Nav2 Core Nodes (planner, controller, etc.) ===
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav_sim_cofig],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav_sim_cofig],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav_sim_cofig],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav_sim_cofig],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav_sim_cofig],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav_sim_cofig],
    )

    # Lifecycle manager for NAVIGATION (does configure+activate on all these)
    nav_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
            ]
        }],
    )


    return LaunchDescription([
        gz_sim,
        bridge,
        map_arg,
        map_server,
        static_tf,
        amcl,
        lifecycle_manager,

        odom_tf_broadcaster,

        planner_server,
        controller_server,
        smoother_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        nav_lifecycle_manager,

        rviz,
        # turtlesim,
        # pathing_node,
        # arena_cofig,
        # nav_sim_cofig,
    ])