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
    pkg_secbot_sim = get_package_share_directory('secbot_sim')
    pkg_secbot_fusion = get_package_share_directory('secbot_fusion')
    pkg_secbot_navigation = get_package_share_directory('secbot_navigation')
    pkg_secbot_vision = get_package_share_directory('secbot_vision')

    
    # configs ===============================
    vision_config = os.path.join(pkg_secbot_vision, 'config', 'vision.yaml')
    ekf_config_path = os.path.join(pkg_secbot_fusion, 'config', 'ekf.yaml')
    nav_config_path = os.path.join(pkg_secbot_navigation, 'config', 'nav_sim.yaml')
    arena_config_path = os.path.join(pkg_secbot_navigation, 'config', 'arena_layout.yaml')
    bridge_config = os.path.join(pkg_secbot_sim,'controllers','ros2_control.yaml')# Path to bridge config
    
    # world info ===================================
    world_file = os.path.join(pkg_secbot_sim, 'worlds','bot', 'my_bot_harmonic.sdf') # Path to World file
    robot_sdf_file = os.path.join(pkg_secbot_sim, 'worlds', 'bot', 'my_bot.sdf')    # Path to Robot SDF for spawning
    gz_resource_path = os.path.join(pkg_secbot_sim, 'worlds') # Ensure Gazebo can find the models
    spawn_yaml_path = os.path.join(pkg_secbot_vision,'controllers', 'spawn_locations.yaml')# Path to Spawn Locations YAML
    yellow_box_sdf = os.path.join(pkg_secbot_sim,'worlds','proper_field','yellow_box.sdf')# Path to Yellow Box SDF
    

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


    # ARGS ==========================================
    num_blocks_arg = DeclareLaunchArgument('num_blocks', default_value='3', description='Number of yellow boxes to spawn (max 6)')# Declare num_blocks argument
    block_scale_arg = DeclareLaunchArgument('block_scale',default_value='1.0',description='Uniform scale factor for yellow boxes (e.g., 0.5, 2.0)')# Declare num_blocks argument

    def spawn_yellow_boxes(context, *args, **kwargs):
        num_blocks_str = LaunchConfiguration('num_blocks').perform(context)
        try:
            num_blocks = int(num_blocks_str)
        except ValueError:
            num_blocks = 1
            
        # Load spawn locations from YAML (re-read to ensure we get locations)
        spawn_poses = []
        if os.path.exists(spawn_yaml_path):
            with open(spawn_yaml_path, 'r') as f:
                y_data = yaml.safe_load(f) or {}
                if 'locations' in y_data:
                    # Convert values to strings for Launch arguments
                    spawn_poses = [
                        {k: str(v) for k, v in loc.items()} 
                        for loc in y_data['locations']
                    ]
        
        # Fallback if YAML fails or is empty
        if not spawn_poses:
            spawn_poses = [
                {'x': f'7.0', 'y': f'0.0', 'z': '0.1'},
                {'x': f'3.0', 'y': f'2.0', 'z': '0.1'},
                {'x': f'7.0', 'y': f'-1.0', 'z': '0.1'},
            ]
        
        # Clamp number of blocks
        num_blocks = max(0, min(num_blocks, len(spawn_poses)))
        nodes = []
        for i in range(num_blocks):
            pose = spawn_poses[i]


            scale = LaunchConfiguration('block_scale').perform(context)

            node = Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-file', yellow_box_sdf,
                    '-name', f'yellow_box_{i+1}',
                    '-x', pose['x'],
                    '-y', pose['y'],
                    '-z', pose['z'],
                    '-s', scale,   # <-- add this
                ],
                output='screen'
            )
            nodes.append(node)
        return nodes

    spawn_boxes_opaque = OpaqueFunction(function=spawn_yellow_boxes)

    # NODES TIME ================================================
    # Spawn Robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', robot_sdf_file,
            '-name', 'my_bot',
        ],
        output='screen'
    )

    # vision
    vision = Node(
        package = 'secbot_vision',
        executable='detector_node',
        name='detector_node',
        output='screen',
        parameters=[vision_config, {'use_sim_time': True,}]
    )

    # compute goal based on info from vision
    vision_to_goal = Node(
        package ='secbot_sim',
        executable='convert_vision_to_goal.py',
        name='vision_to_goal',
        output='screen',
        parameters=[{
            'detections_topic': '/duck_detections',
            'camera_info_topic': '/camera/camera_info',
            'use_sim_time': True,
            'odom_topic': '/odom',
            'camera_height': 0.30,
            'camera_tilt_deg': 0,
            'goal_standoff': 0.0,
            'min_confidence': 60.0,
            'use_largest_area': True,
            "priority_ids": [3, 1, 7, 2],
            "republish_hz": 1.0,
            "target_timeout_sec": 1.0,
        }]
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

    # Load central motion config
    motion_config_path = os.path.join(get_package_share_directory('my_robot_description'), 'config', 'motion_config.yaml')
    motion_params = {}
    if os.path.exists(motion_config_path):
        with open(motion_config_path, 'r') as f:
            cfg = yaml.safe_load(f)
            rob = cfg.get('robot_parameters', {})
            # Map SI units back to the expected (partially inches) simulation params
            # Note: Simulation code still uses inches internally for some things, 
            # so we convert BACK if necessary, or better yet, update node to SI eventually.
            # Convert m -> in
            M_TO_IN = 1.0 / 0.0254
            motion_params = {
                'track_width': float(rob.get('track_width', 0.303022) * M_TO_IN),
                'wheel_diameter': float(rob.get('wheel_diameter', 0.08255) * M_TO_IN),
                'encoder_ticks_per_rev': int(rob.get('encoder_ticks_per_rev', 104)),
                'gear_ratio': float(rob.get('gear_ratio', 0.6)),
                'max_velocity': float(rob.get('max_velocity', 0.6096) * M_TO_IN),
                'max_angular_velocity': float(rob.get('max_angular_velocity', 3.0)),
                'linear_v_max': float(rob.get('max_velocity', 0.6096) * M_TO_IN),
                'linear_a_max': float(rob.get('max_acceleration', 0.3048) * M_TO_IN),
                'linear_j_max': float(rob.get('max_jerk', 1.2192) * M_TO_IN),
                'angular_v_max': float(rob.get('max_angular_velocity', 3.0)),
                'angular_a_max': float(rob.get('angular_a_max', 6.0)), # Fallback to default if missing
                'angular_j_max': float(rob.get('angular_j_max', 24.0)),
            }

    # mcu dummy node
    mcu_subsystem = Node(
            package='secbot_sim',
            executable='mcu_subsystem_sim',
            name='mcu_subsystem_sim',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'num_tof_sensors': 4,
                **motion_params,
                # PID (same for both wheels)
                'pid_kp': 0.8,
                'pid_ki': 0.1,
                'pid_kd': 0.05,
                'pid_out_min': -255.0,
                'pid_out_max': 255.0,
                'pid_i_min': -50.0,
                'pid_i_max': 50.0,
                'pid_d_filter_alpha': 0.1,
            }],
            remappings=[
                ('/cmd_vel_out', '/cmd_vel'),   # MCU drives Gazebo
            ]
        )

    
    # ekf 
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': True}],
        remappings=[
            ('/odom/unfiltered', '/odom'),
        ],
    )

    # fusion
    fusion_node = Node(
        package='secbot_fusion',
        executable='fusion_node',
        name='fusion_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
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
                # ('/odom','/odometry/filtered'),
                # ('/odom_raw','/odom'),
                ('/cmd_vel', '/cmd_vel_in'),
                ('/odom','/odom')
            ]
        )


    return LaunchDescription([
        # sim ==============
        gz_sim,

        # args =============
        num_blocks_arg,
        block_scale_arg,
        spawn_boxes_opaque,

        # nodes ============
        mcu_subsystem,
        spawn_robot,
        vision,
        bridge,
        # ekf_node,
        # fusion_node,
        vision_to_goal,
        pathing_node,
    ])
