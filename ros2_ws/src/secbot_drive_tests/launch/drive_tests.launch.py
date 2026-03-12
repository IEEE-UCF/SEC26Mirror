"""
Launch file for drive base integration tests.

Usage:
  # Run a specific test suite:
  ros2 launch secbot_drive_tests drive_tests.launch.py test:=velocity
  ros2 launch secbot_drive_tests drive_tests.launch.py test:=goal
  ros2 launch secbot_drive_tests drive_tests.launch.py test:=trajectory
  ros2 launch secbot_drive_tests drive_tests.launch.py test:=stress

  # With custom parameters:
  ros2 launch secbot_drive_tests drive_tests.launch.py test:=goal distance:=0.3
  ros2 launch secbot_drive_tests drive_tests.launch.py test:=velocity velocity:=0.2
  ros2 launch secbot_drive_tests drive_tests.launch.py test:=trajectory scale:=0.5
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    test_arg = DeclareLaunchArgument(
        'test', default_value='velocity',
        description='Test suite to run: velocity, goal, trajectory, stress'
    )

    # Common parameters
    goal_tolerance_arg = DeclareLaunchArgument(
        'goal_tolerance', default_value='0.03',
        description='Position tolerance in meters'
    )
    heading_tolerance_arg = DeclareLaunchArgument(
        'heading_tolerance', default_value='0.1',
        description='Heading tolerance in radians'
    )
    goal_timeout_arg = DeclareLaunchArgument(
        'goal_timeout', default_value='10.0',
        description='Timeout per goal in seconds'
    )
    pause_time_arg = DeclareLaunchArgument(
        'pause_time', default_value='1.0',
        description='Pause between test steps in seconds'
    )

    # Test-specific parameters
    distance_arg = DeclareLaunchArgument(
        'distance', default_value='0.5',
        description='[goal] Drive distance in meters'
    )
    velocity_arg = DeclareLaunchArgument(
        'velocity', default_value='0.3',
        description='[velocity] Test velocity in m/s'
    )
    scale_arg = DeclareLaunchArgument(
        'scale', default_value='1.0',
        description='[trajectory] Scale factor for path sizes'
    )

    test_name = LaunchConfiguration('test')

    # We launch based on the 'test' argument
    # Since ROS2 launch doesn't easily support conditional node selection,
    # the recommended approach is to use ros2 run directly:
    #   ros2 run secbot_drive_tests test_velocity
    # This launch file provides a convenient wrapper with all parameters.

    # Launch all test types as separate nodes (only the selected one runs)
    # Use the test arg to construct the executable name
    test_node = Node(
        package='secbot_drive_tests',
        executable=['test_', test_name],
        name='drive_test',
        output='screen',
        parameters=[{
            'goal_tolerance': LaunchConfiguration('goal_tolerance'),
            'heading_tolerance': LaunchConfiguration('heading_tolerance'),
            'goal_timeout': LaunchConfiguration('goal_timeout'),
            'pause_time': LaunchConfiguration('pause_time'),
            'distance': LaunchConfiguration('distance'),
            'velocity': LaunchConfiguration('velocity'),
            'scale': LaunchConfiguration('scale'),
        }],
    )

    return LaunchDescription([
        test_arg,
        goal_tolerance_arg,
        heading_tolerance_arg,
        goal_timeout_arg,
        pause_time_arg,
        distance_arg,
        velocity_arg,
        scale_arg,
        test_node,
    ])
