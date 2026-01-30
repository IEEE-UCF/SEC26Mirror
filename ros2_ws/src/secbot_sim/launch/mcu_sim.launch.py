"""
MCU Subsystem Simulator Launch File

Launches the mcu_subsystem_sim node that simulates Drive, Battery, and Heartbeat subsystems
"""
import os
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('secbot_sim')
    
    # Path to World file
    world_file = os.path.join(
        pkg_share,
        'worlds',
        # 'field',
        'bot',
        # 'proper_field.world'
        'my_bot_harmonic.sdf'
        # 'field.world'
        
    )
    
    # Path to Robot SDF for spawning
    robot_sdf_file = os.path.join(
        pkg_share,
        'worlds',
        'bot',
        'my_bot.sdf'
    )

    # Ensure Gazebo can find the models
    gz_resource_path = os.path.join(pkg_share, 'worlds')
    
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        gz_resource_path += os.pathsep + os.environ['GZ_SIM_RESOURCE_PATH']
        
    env = {'GZ_SIM_RESOURCE_PATH': gz_resource_path}

    # Gazebo Sim (Harmonic)
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '-r'],
        output='screen',
        additional_env=env
    )

    # Spawn Robot
    # Path to Spawn Locations YAML
    spawn_yaml_path = os.path.join(
        pkg_share,
        'controllers',
        'spawn_locations.yaml'
    )
    
    # Load YAML if exists
    yaml_data = {}
    if os.path.exists(spawn_yaml_path):
        with open(spawn_yaml_path, 'r') as f:
            yaml_data = yaml.safe_load(f) or {}

    # Spawn Robot
    # Default Spawn Location from YAML or Fallback
    robot_cfg = yaml_data.get('robot', {})
    default_x = str(robot_cfg.get('x', '0.25'))
    default_y = str(robot_cfg.get('y', '-1.0'))
    default_z = str(robot_cfg.get('z', '0.2'))
    default_yaw = str(robot_cfg.get('yaw', '0.0'))

    # Declare launch arguments for offsets
    # x_offset_arg = DeclareLaunchArgument('x_offset', default_value='0.0', description='X offset for robot spawn')
    # y_offset_arg = DeclareLaunchArgument('y_offset', default_value='0.0', description='Y offset for robot spawn')
    # z_offset_arg = DeclareLaunchArgument('z_offset', default_value='0.0', description='Z offset for robot spawn')
    # yaw_offset_arg = DeclareLaunchArgument('yaw_offset', default_value='0.0', description='Yaw offset for robot spawn')

    # Calculate final spawn position
    # Calculate final spawn position
    x_pos = PythonExpression([default_x, " + ", LaunchConfiguration('x_offset')])
    y_pos = PythonExpression([default_y, " + ", LaunchConfiguration('y_offset')])
    z_pos = PythonExpression([default_z, " + ", LaunchConfiguration('z_offset')])
    yaw_pos = PythonExpression([default_yaw, " + ", LaunchConfiguration('yaw_offset')])

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

    # Path to bridge config
    bridge_config = os.path.join(
        pkg_share,
        'controllers',
        'ros2_control.yaml'
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


    mcu_subsystem = Node(
            package='secbot_sim',
            executable='mcu_subsystem_sim',
            name='mcu_subsystem_sim',
            output='screen',
            parameters=[{
                'use_sim_time': True,
            }],
        )
    
    pathing_node = Node(
            package='secbot_navigation',
            executable='pathing_node',
            name='pathing_node',
            output='screen',
            parameters=[{
                'use_sim': True,
                'config_file': 'nav_sim.yaml',
                'arena_file': 'arena_layout.yaml',
            }],
            # If needed, remap topics here (see notes below)
            # remappings=[
            #   ('/odom', '/odom'), 
            #   ('/cmd_vel', '/cmd_vel'),
            # ]
        )


    return LaunchDescription([
        # x_offset_arg,
        # y_offset_arg,
        # z_offset_arg,
        # yaw_offset_arg,
        gz_sim,
        spawn_robot,
        bridge,
        mcu_subsystem,
        pathing_node,
    ])
