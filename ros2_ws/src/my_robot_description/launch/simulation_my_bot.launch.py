import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_my_robot = get_package_share_directory('my_robot_description')
    pkg_secbot_sim = get_package_share_directory('secbot_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Paths
    sdf_file = os.path.join(pkg_secbot_sim, 'worlds', 'bot', 'my_bot.sdf')
    urdf_file = os.path.join(pkg_my_robot, 'urdf', 'my_bot_viz.urdf')
    bridge_config = os.path.join(pkg_my_robot, 'config', 'ros_gz_bridge.yaml')
    rviz_config = os.path.join(pkg_my_robot, 'rviz', 'sdf_config.rviz') # New config

    # Resource Path for Meshes
    install_dir = get_package_share_directory('my_robot_description')
    gz_resource_path = os.path.dirname(install_dir)
    
    set_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_path,
        separator=os.pathsep
    )

    # Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # Spawn SDF
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', sdf_file,
            '-name', 'my_robot',
            '-x', '0', '-y', '0', '-z', '0.5'
        ],
        output='screen'
    )

    # ROS-Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p', f'config_file:={bridge_config}'
        ],
        output='screen'
    )

    # Robot State Publisher
    # Use sdformat_urdf to convert SDF to URDF on the fly
    # allowing us to use my_bot.sdf as single source of truth
    from launch.substitutions import Command
    
    robot_description_content = Command(['ros2', 'run', 'sdformat_urdf', 'sdformat_to_urdf', sdf_file])
        
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': True}],
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        set_gz_resource_path,
        gz_sim,
        spawn_entity,
        bridge,
        robot_state_publisher,
        rviz_node
    ])
