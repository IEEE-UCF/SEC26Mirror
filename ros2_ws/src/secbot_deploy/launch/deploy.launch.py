from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='secbot_deploy',
            executable='deploy_node',
            name='deploy_node',
            output='screen',
            parameters=[{
                'deploy_dir': '/home/ubuntu/scripts/.deploy',
            }],
        ),
    ])
