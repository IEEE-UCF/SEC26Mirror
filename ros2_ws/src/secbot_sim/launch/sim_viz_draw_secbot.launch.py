import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_secbot_sim = get_package_share_directory('secbot_sim')
    pkg_my_robot   = get_package_share_directory('my_robot_description')

    # RViz config from my_robot_description
    rviz_config = os.path.join(pkg_my_robot, 'rviz', 'urdf.rviz')

    return LaunchDescription([
        # 1. Full sim + navigation (includes robot_state_publisher already)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_secbot_sim, 'launch', 'mcu_sim_secbot.launch.py')
            )
        ),

        # 2. RViz2 — visualise the URDF robot and planned path
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}]
        ),
    ])
