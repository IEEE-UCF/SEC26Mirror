"""
MCU Subsystem Simulator Launch File

Launches the mcu_subsystem_sim node that simulates Drive, Battery, and Heartbeat subsystems
"""
# TO START RUN chmod +x /home/ubuntu/ros2_workspaces/src/sec26ros/secbot_sim/scripts/run_sim.sh 
# this initializes the .sh file

# this file is used to make it easier to run mcu_sim.launch.py since making sure all nodes are updated is super tedious

# FINALLY RUN /home/ubuntu/ros2_workspaces/src/sec26ros/secbot_sim/scripts/run_sim.sh
# this is to run the actual file itself

import os
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory




def generate_launch_description():

    # package directories =====================================
    pkg = get_package_share_directory('build_robot_world')

    
    # configs ===============================
    bridge_config = os.path.join(pkg,'controllers','ros2_control.yaml')# Path to bridge config
    
    # world info ===================================
    world_file = os.path.join(pkg, 'worlds','bot', 'my_bot_harmonic.sdf') # Path to World file
    beacons_sdf_file1 = os.path.join(pkg, 'worlds', 'beacons', 'beacons1.sdf')    # Path to Robot SDF for spawning
    beacons_sdf_file2 = os.path.join(pkg, 'worlds', 'beacons', 'beacons2.sdf')
    gz_resource_path = os.path.join(pkg, 'worlds') # Ensure Gazebo can find the models
    

    # THIS IS WHAT IS USED TO START GAZEBO ===========================
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        gz_resource_path += os.pathsep + os.environ['GZ_SIM_RESOURCE_PATH']
        
    env = {'GZ_SIM_RESOURCE_PATH': gz_resource_path}

    # Gazebo Sim (Harmonic)
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '-r'],
        output='screen',
        additional_env=env
    )


    # NODES TIME ================================================
    # Spawn Robot
    spawn_beacon1 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', beacons_sdf_file1,
            '-name', 'black_block1',
            '-x', '1',
            '-y', '2',
            '-z', '0.5',
        ],
        output='screen'
    )

    spawn_beacon2 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', beacons_sdf_file2,
            '-name', 'black_block2',
            '-x', '-5',
            '-y', '5',
            '-z', '0.5',
        ],
        output='screen'
    )


    # # ROS-Gazebo Bridge
    # bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=[
    #         '--ros-args',
    #         '-p',
    #         f'config_file:={bridge_config}'
    #         ],
    #     output='screen',
    #     parameters=[{'use_sim_time': True,}]
    # )

    return LaunchDescription([
        # sim ==============
        gz_sim,

        # nodes ============
        spawn_beacon1,
        spawn_beacon2,
        # bridge,
    ])
