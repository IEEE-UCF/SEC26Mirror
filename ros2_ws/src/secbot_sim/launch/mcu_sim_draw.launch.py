"""
MCU Subsystem Simulator Launch File (DRAW MODE)

Launches the mcu_subsystem_sim node in an empty world for 2D Goal Pose testing.
"""
import os
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # package directories =====================================
    pkg_secbot_sim = get_package_share_directory('secbot_sim')
    pkg_secbot_fusion = get_package_share_directory('secbot_fusion')
    pkg_secbot_navigation = get_package_share_directory('secbot_navigation')
    pkg_secbot_vision = get_package_share_directory('secbot_vision')

    
    # configs ===============================
    vision_config = os.path.join(pkg_secbot_vision, 'config', 'vision.yaml')
    ekf_config_path = os.path.join(pkg_secbot_fusion, 'config', 'ekf.yaml')
    nav_config_path = os.path.join(pkg_secbot_navigation, 'config', 'nav_sim.yaml')
    # DRAW MODE: Use empty arena
    arena_config_path = os.path.join(pkg_secbot_navigation, 'config', 'empty_arena.yaml') 
    bridge_config = os.path.join(pkg_secbot_sim,'controllers','ros2_control.yaml')# Path to bridge config
    
    # world info ===================================
    # DRAW MODE: Use empty world
    world_file = os.path.join(pkg_secbot_sim, 'worlds', 'empty.sdf') 
    robot_sdf_file = os.path.join(pkg_secbot_sim, 'worlds', 'bot', 'my_bot.sdf')    # Path to Robot SDF for spawning
    gz_resource_path = os.path.join(pkg_secbot_sim, 'worlds') # Ensure Gazebo can find the models
    
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
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', robot_sdf_file,
            '-name', 'my_bot',
            '-z', '0.1', # spawn slightly above ground
        ],
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
        output='screen',
        parameters=[{'use_sim_time': True,}]
    )

    # mcu dummy node
    mcu_subsystem = Node(
            package='secbot_sim',
            executable='mcu_subsystem_sim',
            name='mcu_subsystem_sim',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'num_tof_sensors': 4,

                # Drive config
                'track_width': 12.0,
                'wheel_diameter': 4.0,
                'encoder_ticks_per_rev': 2048,
                'gear_ratio': 1,
                'max_velocity': 24.0,        # in/s
                'max_angular_velocity': 3.0,  # rad/s
                'max_wheel_velocity': 48.0,   # in/s at PWM=255

                # PID
                'pid_kp': 0.8,
                'pid_ki': 0.1,
                'pid_kd': 0.05,
                'pid_out_min': -255.0,
                'pid_out_max': 255.0,
                'pid_i_min': -50.0,
                'pid_i_max': 50.0,
                'pid_d_filter_alpha': 0.1,

                # Linear S-curve
                'linear_v_max': 24.0,   
                'linear_a_max': 12.0,   
                'linear_d_max': 12.0,   
                'linear_j_max': 48.0,   

                # Angular S-curve
                'angular_v_max': 3.0,   
                'angular_a_max': 6.0,   
                'angular_d_max': 6.0,   
                'angular_j_max': 24.0,  
            }],
            remappings=[
                ('/cmd_vel_out', '/cmd_vel'),   # MCU drives Gazebo
            ]
        )
    
    # pathing to goal
    pathing_node = Node(
            package='secbot_navigation',
            executable='pathing_node',
            name='pathing_node',
            output='screen',
            parameters=[{
                'use_sim': True,
                'use_sim_time': True,
                'config_file': nav_config_path,
                'arena_file': arena_config_path,
            }],
            remappings=[
                ('/cmd_vel', '/cmd_vel_in'),
                ('/odom','/odom')
            ]
        )


    return LaunchDescription([
        gz_sim,
        mcu_subsystem,
        spawn_robot,
        bridge,
        pathing_node,
    ])
