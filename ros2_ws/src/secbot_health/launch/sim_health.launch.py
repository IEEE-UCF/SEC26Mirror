from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='secbot_health',
            executable='health_node',
            name='health_node',
            output='screen',
            parameters=[{
                'use_sim': True,
                'odom_timeout_sec': 5.0,
                'autonomy_timeout_sec': 5.0,
                'publish_rate_hz': 1.0,
            }],
        ),
    ])
