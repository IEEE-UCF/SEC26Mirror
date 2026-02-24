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
                'use_sim': False,
                'heartbeat_timeout_sec': 1.0,
                'mcu_state_timeout_sec': 2.0,
                'battery_timeout_sec': 5.0,
                'battery_warn_voltage': 11.0,
                'battery_error_voltage': 10.5,
                'odom_timeout_sec': 3.0,
                'autonomy_timeout_sec': 3.0,
                'publish_rate_hz': 2.0,
            }],
        ),
    ])
