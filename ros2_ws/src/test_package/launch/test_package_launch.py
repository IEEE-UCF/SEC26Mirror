from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch description for test_package.

    Launches the `test_node` executable from the `test_package` package.
    """
    return LaunchDescription([
        Node(
            package='test_package',
            executable='test_node',
            name='test_node',
            output='screen',
            emulate_tty=True,
        )
    ])
