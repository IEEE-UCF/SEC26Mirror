#!/usr/bin/env python3
"""
Launch file for proper field world in Gazebo Harmonic with ROS 2 bridge
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('secbot_vision')
    
    # Path to World file
    world_file = os.path.join(
        pkg_share,
        'worlds',
        'proper_field',
        'proper_field.world'
    )
    
    # Path to Robot SDF for spawning
    robot_sdf_file = os.path.join(
        pkg_share,
        'worlds',
        'default',
        'my_bot.sdf'
    )

    # Path to bridge config
    bridge_config = os.path.join(
        pkg_share,
        'config',
        'ros_gz_bridge.yaml'
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

    # Spawn Robot
    # Path to Spawn Locations YAML
    spawn_yaml_path = os.path.join(
        pkg_share,
        'config',
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
    x_offset_arg = DeclareLaunchArgument('x_offset', default_value='0.0', description='X offset for robot spawn')
    y_offset_arg = DeclareLaunchArgument('y_offset', default_value='0.0', description='Y offset for robot spawn')
    z_offset_arg = DeclareLaunchArgument('z_offset', default_value='0.0', description='Z offset for robot spawn')
    yaw_offset_arg = DeclareLaunchArgument('yaw_offset', default_value='0.0', description='Yaw offset for robot spawn')

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
            '-x', x_pos,
            '-y', y_pos,
            '-z', z_pos,
            '-Y', yaw_pos
        ],
        output='screen'
    )

    # Path to Yellow Box SDF
    yellow_box_sdf = os.path.join(
        pkg_share,
        'worlds',
        'proper_field',
        'yellow_box.sdf'
    )
    
    # Declare num_blocks argument
    num_blocks_arg = DeclareLaunchArgument(
        'num_blocks', 
        default_value='3', 
        description='Number of yellow boxes to spawn (max 6)'
    )

    def spawn_yellow_boxes(context, *args, **kwargs):
        num_blocks_str = LaunchConfiguration('num_blocks').perform(context)
        try:
            num_blocks = int(num_blocks_str)
        except ValueError:
            num_blocks = 3
            
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
                {'x': '1.0', 'y': '0.0', 'z': '0.1'},
                {'x': '1.0', 'y': '1.0', 'z': '0.1'},
                {'x': '0.0', 'y': '1.0', 'z': '0.1'}
            ]
        
        # Clamp number of blocks
        num_blocks = max(0, min(num_blocks, len(spawn_poses)))
        
        nodes = []
        for i in range(num_blocks):
            pose = spawn_poses[i]
            node = Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-file', yellow_box_sdf,
                    '-name', f'yellow_box_{i+1}',
                    '-x', pose['x'],
                    '-y', pose['y'],
                    '-z', pose['z']
                ],
                output='screen'
            )
            nodes.append(node)
        return nodes

    spawn_boxes_opaque = OpaqueFunction(function=spawn_yellow_boxes)

    vision_config = os.path.join(
        pkg_share, 
        'config', 
        'vision.yaml')
        
    vision = Node(
        package = 'secbot_vision',
        executable='detector_node',
        name='detector_node',
        output='screen',
        parameters=[vision_config]
    )

    return LaunchDescription([
        x_offset_arg,
        y_offset_arg,
        z_offset_arg,
        yaw_offset_arg,
        gz_sim,
        bridge,
        spawn_robot,
        vision,
        num_blocks_arg,
        spawn_boxes_opaque
    ])
