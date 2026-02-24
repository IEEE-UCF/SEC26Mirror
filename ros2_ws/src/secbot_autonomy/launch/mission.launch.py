from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='secbot_autonomy',
            executable='autonomy_node',
            name='autonomy_node',
            output='screen',
        ),
        Node(
            package='secbot_autonomy',
            executable='mission_node',
            name='mission_node',
            output='screen',
        ),
    ])
