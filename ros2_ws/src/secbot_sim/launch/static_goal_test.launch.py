"""
static_goal_test.launch.py — Robot stays still, vision pipeline only.

Starts: Gazebo + bridge + RSP + detector + vision_to_goal
Does NOT start: mcu_subsystem_sim or pathing_node
→ Robot cannot move; used to verify goal calculation in isolation.
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_secbot_sim    = get_package_share_directory('secbot_sim')
    pkg_secbot_vision = get_package_share_directory('secbot_vision')
    pkg_my_robot      = get_package_share_directory('my_robot_description')

    vision_config   = os.path.join(pkg_secbot_vision, 'config', 'vision.yaml')
    bridge_config   = os.path.join(pkg_my_robot,      'config', 'ros_gz_bridge.yaml')
    world_file      = os.path.join(pkg_secbot_sim, 'worlds', 'bot', 'my_bot_harmonic.sdf')
    robot_urdf_file = os.path.join(pkg_my_robot,   'urdf',   'robot.urdf')
    spawn_yaml_path = os.path.join(pkg_secbot_vision, 'controllers', 'spawn_locations.yaml')
    yellow_box_sdf  = os.path.join(pkg_secbot_sim, 'worlds', 'proper_field', 'yellow_box.sdf')

    with open(robot_urdf_file, 'r') as f:
        robot_description = f.read()

    my_robot_share_parent = os.path.dirname(pkg_my_robot)
    secbot_sim_worlds     = os.path.join(pkg_secbot_sim, 'worlds')
    gz_resource_path = secbot_sim_worlds + os.pathsep + my_robot_share_parent
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        gz_resource_path += os.pathsep + os.environ['GZ_SIM_RESOURCE_PATH']
    env = {'GZ_SIM_RESOURCE_PATH': gz_resource_path}

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
                {'x': '7.0',  'y':  '0.0', 'z': '0.1'},
                {'x': '3.0',  'y':  '2.0', 'z': '0.1'},
                {'x': '7.0',  'y': '-1.0', 'z': '0.1'},
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

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', robot_urdf_file, '-name', 'my_robot',
                   '-x', '0', '-y', '0', '-z', '0.2'],
        output='screen'
    )

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
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # detector_node — subscribes /camera, publishes /duck_detections
    vision = Node(
        package='secbot_vision',
        executable='detector_node',
        name='detector_node',
        output='screen',
        parameters=[vision_config, {
            'use_sim_time': True,
            'image_topic': '/camera1/image',
        }]
    )

    # vision_to_goal — converts detections to /goal_pose
    vision_to_goal = Node(
        package='secbot_sim',
        executable='convert_vision_to_goal.py',
        name='vision_to_goal',
        output='screen',
        parameters=[{
            'detections_topic':  '/duck_detections',
            'camera_info_topic': '/camera1/camera_info',
            'use_sim_time':      True,
            'odom_topic':        '/odom',
            'camera_height':     0.34,
            'camera_tilt_deg':   0,
            'goal_standoff':     0.3,
            'min_confidence':    60.0,
            'use_largest_area':  True,
            # Fallback intrinsics so we don't wait for CameraInfo
            'fallback_fx':       528.0,
            'fallback_fy':       528.0,
            'fallback_cx':       320.0,
            'fallback_cy':       240.0,
        }]
    )

    return LaunchDescription([
        num_blocks_arg,
        block_scale_arg,
        ExecuteProcess(
            cmd=['gz', 'sim', world_file, '-r'],
            output='screen',
            additional_env=env
        ),
        spawn_robot,
        bridge,
        robot_state_publisher,
        vision,
        vision_to_goal,
        OpaqueFunction(function=spawn_yellow_boxes),
    ])
