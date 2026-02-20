import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    pkg_my_robot = get_package_share_directory('my_robot_description')
    pkg_secbot_nav = get_package_share_directory('secbot_navigation')

    # Simulation Launch
    # This launches the world, spawns the robot, and starts the bridge
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_my_robot, 'launch', 'simulation.launch.py')
        )
    )

    # Navigation Launch
    # Launches the existing secbot_navigation stack
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_secbot_nav, 'launch', 'sim_nav.launch.py')
        )
    )

    # RViz
    rviz_config_file = os.path.join(pkg_my_robot, 'rviz', 'urdf.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        simulation_launch,
        navigation_launch,
        rviz_node
    ])
