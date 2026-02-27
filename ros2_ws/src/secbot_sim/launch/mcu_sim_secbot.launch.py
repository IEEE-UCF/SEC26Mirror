"""
MCU Subsystem Simulator Launch File — Secbot (URDF) Variant

Same as mcu_sim.launch.py but spawns and publishes robot_description
from my_robot_description/urdf/robot.urdf instead of my_bot.sdf.
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
    pkg_secbot_sim      = get_package_share_directory('secbot_sim')
    pkg_secbot_fusion   = get_package_share_directory('secbot_fusion')
    pkg_secbot_nav      = get_package_share_directory('secbot_navigation')
    pkg_secbot_vision   = get_package_share_directory('secbot_vision')
    pkg_my_robot        = get_package_share_directory('my_robot_description')

    # configs ===============================
    vision_config    = os.path.join(pkg_secbot_vision,  'config', 'vision.yaml')
    ekf_config_path  = os.path.join(pkg_secbot_fusion,  'config', 'ekf.yaml')
    nav_config_path  = os.path.join(pkg_secbot_nav,     'config', 'nav_sim.yaml')
    arena_config_path= os.path.join(pkg_secbot_nav,     'config', 'arena_layout.yaml')
    bridge_config    = os.path.join(pkg_my_robot,       'config', 'ros_gz_bridge.yaml')

    # world / robot files ===================================
    world_file        = os.path.join(pkg_secbot_sim,  'worlds', 'bot', 'my_bot_harmonic.sdf')
    robot_urdf_file   = os.path.join(pkg_my_robot,    'urdf', 'robot.urdf')
    gz_resource_path  = os.path.join(pkg_secbot_sim,  'worlds')
    spawn_yaml_path   = os.path.join(pkg_secbot_vision,'controllers', 'spawn_locations.yaml')
    yellow_box_sdf    = os.path.join(pkg_secbot_sim,  'worlds', 'proper_field', 'yellow_box.sdf')

    # Read URDF for robot_state_publisher
    with open(robot_urdf_file, 'r') as f:
        robot_description = f.read()

    # GZ resource path env ===========================
    # We need the *parent* of each package's share dir so that
    # model://my_robot_description/meshes/... resolves correctly.
    my_robot_share_parent = os.path.dirname(pkg_my_robot)  # …/share
    secbot_sim_worlds     = os.path.join(pkg_secbot_sim, 'worlds')

    gz_resource_path = secbot_sim_worlds + os.pathsep + my_robot_share_parent
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        gz_resource_path += os.pathsep + os.environ['GZ_SIM_RESOURCE_PATH']
    env = {'GZ_SIM_RESOURCE_PATH': gz_resource_path}

    # Gazebo Sim (Harmonic) ===========================
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '-r'],
        output='screen',
        additional_env=env
    )

    # ARGS ==========================================
    num_blocks_arg = DeclareLaunchArgument(
        'num_blocks', default_value='1',
        description='Number of yellow boxes to spawn (max 6)')
    block_scale_arg = DeclareLaunchArgument(
        'block_scale', default_value='1.0',
        description='Uniform scale factor for yellow boxes')

    def spawn_yellow_boxes(context, *args, **kwargs):
        num_blocks_str = LaunchConfiguration('num_blocks').perform(context)
        try:
            num_blocks = int(num_blocks_str)
        except ValueError:
            num_blocks = 1

        spawn_poses = []
        if os.path.exists(spawn_yaml_path):
            with open(spawn_yaml_path, 'r') as f:
                y_data = yaml.safe_load(f) or {}
                if 'locations' in y_data:
                    spawn_poses = [{k: str(v) for k, v in loc.items()}
                                   for loc in y_data['locations']]

        if not spawn_poses:
            spawn_poses = [
                {'x': '5.0',  'y':  '0.0', 'z': '0.1'},
                {'x': '3.0',  'y':  '2.0', 'z': '0.1'},
                {'x': '4.0',  'y': '-1.0', 'z': '0.1'},
            ]

        num_blocks = max(0, min(num_blocks, len(spawn_poses)))
        nodes = []
        for i in range(num_blocks):
            pose  = spawn_poses[i]
            scale = LaunchConfiguration('block_scale').perform(context)
            node  = Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-file', yellow_box_sdf,
                    '-name', f'yellow_box_{i+1}',
                    '-x', pose['x'],
                    '-y', pose['y'],
                    '-z', pose['z'],
                    '-s', scale,
                ],
                output='screen'
            )
            nodes.append(node)
        return nodes

    spawn_boxes_opaque = OpaqueFunction(function=spawn_yellow_boxes)

    # Spawn Robot from URDF =============================
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

    # Robot State Publisher =============================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    # Vision node =======================================
    vision = Node(
        package='secbot_vision',
        executable='detector_node',
        name='detector_node',
        output='screen',
        parameters=[vision_config, {'use_sim_time': True,
                                    'image_topic': '/camera1/image'}]  # uniquely namespaced
    )

    # Vision → Goal converter ===========================
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
        }]
    )

    # ROS-Gazebo Bridge =================================
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args', '-p',
            f'config_file:={bridge_config}'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Load central motion config
    # FIX GEAR_RATIO LATER CANNOT BE FLOAT/DOUBLE, ONLY TAKES INT
    motion_config_path = os.path.join(get_package_share_directory('my_robot_description'), 'config', 'motion_config.yaml')
    motion_params = {}
    if os.path.exists(motion_config_path):
        with open(motion_config_path, 'r') as f:
            cfg = yaml.safe_load(f)
            rob = cfg.get('robot_parameters', {})
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
                'angular_a_max': float(rob.get('angular_a_max', 6.0)),
                'angular_j_max': float(rob.get('angular_j_max', 24.0)),
            }

    # MCU Subsystem Sim =================================
    mcu_subsystem = Node(
        package='secbot_sim',
        executable='mcu_subsystem_sim',
        name='mcu_subsystem_sim',
        output='screen',
        parameters=[{
            'use_sim_time':            True,
            'num_tof_sensors':         4,
            **motion_params,
            'pid_kp':                  0.8,
            'pid_ki':                  0.1,
            'pid_kd':                  0.05,
            'pid_out_min':             -255.0,
            'pid_out_max':             255.0,
            'pid_i_min':               -50.0,
            'pid_i_max':               50.0,
            'pid_d_filter_alpha':      0.1,
        }],
        remappings=[
            ('/cmd_vel_out', '/cmd_vel'),
        ]
    )


    # EKF node (commented out until tuned) ==============
    # ekf_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_node',
    #     output='screen',
    #     parameters=[ekf_config_path, {'use_sim_time': True}],
    #     remappings=[('/odom/unfiltered', '/odom')],
    # )

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
        # # Pipeline: pathing_node → /cmd_vel_in → mcu_subsystem (PID+S-curve)
        # #                        → /cmd_vel_out → /cmd_vel → Gazebo DiffDrive
        # remappings=[
        #     ('/cmd_vel', '/cmd_vel_in'),
        # ]
    )

    return LaunchDescription([
        # simulation
        gz_sim,

        # args
        num_blocks_arg,
        block_scale_arg,
        spawn_boxes_opaque,

        # nodes
        spawn_robot,
        robot_state_publisher,
        bridge,
        mcu_subsystem,
        vision,
        vision_to_goal,
        pathing_node,
    ])
