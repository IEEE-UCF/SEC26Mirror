import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_secbot_sim = get_package_share_directory('secbot_sim')
    
    # URDF file (same for both modes)
    urdf_file = os.path.join(pkg_secbot_sim, 'urdf', 'my_bot.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # RViz config (same for both modes)
    rviz_config = os.path.join(pkg_secbot_sim, 'rviz', 'sim.rviz')

    return LaunchDescription([
        # 1. Start Simulation (DRAW MODE)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_secbot_sim, 'launch', 'mcu_sim_draw.launch.py')
            )
        ),

        # 2. Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # 3. Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_gui': False}]
        ),

        # 4. RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])
