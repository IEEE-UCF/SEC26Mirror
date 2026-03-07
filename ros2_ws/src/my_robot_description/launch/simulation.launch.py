import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_my_robot = get_package_share_directory('my_robot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Path to URDF
    urdf_file = os.path.join(pkg_my_robot, 'urdf', 'robot.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Path to Bridge Config
    bridge_config = os.path.join(
        pkg_my_robot,
        'config',
        'ros_gz_bridge.yaml'
    )

    # Set GZ_SIM_RESOURCE_PATH to include the share directory so meshes can be found
    install_dir = get_package_share_directory('my_robot_description')
    gz_resource_path = os.path.dirname(install_dir)
    
    # Set the environment variable to find meshes
    set_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_path,
        separator=os.pathsep
    )

    # Gazebo Sim (Harmonic)
    # Launch with an empty world
    # -r runs the simulation immediately on start
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
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
    # Use -file instead of -string to avoid "Argument list too long" error
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', urdf_file,
            '-name', 'my_robot',
            '-x', '0',
            '-y', '0',
            '-z', '0.2'
        ],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    return LaunchDescription([
        set_gz_resource_path,
        gz_sim,
        bridge,
        robot_state_publisher,
        spawn_entity,
    ])
