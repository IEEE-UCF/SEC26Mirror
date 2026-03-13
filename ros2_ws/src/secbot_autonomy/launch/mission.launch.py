from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('secbot_autonomy')

    return LaunchDescription([
        Node(
            package='secbot_autonomy',
            executable='autonomy_node',
            name='autonomy_node',
            output='screen',
        ),
        Node(
            package='secbot_autonomy',
            executable='mission_sequencer',
            name='mission_sequencer',
            parameters=[
                os.path.join(pkg_share, 'config', 'mission_waypoints.yaml'),
            ],
            output='screen',
        ),
    ])
