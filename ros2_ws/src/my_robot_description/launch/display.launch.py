import os
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('my_robot_description')
    urdf_file = os.path.join(pkg, 'urdf', 'robot.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Parse URDF to find the base link name
    tree = ET.parse(urdf_file)
    root = tree.getroot()
    # The first link in strict URDF is often the base, or we can just pick the first one found if not specified.
    # In this specific file, the first link is "1_x_1_x___23___aluminum_angle..."
    base_link = root.find('link').attrib['name']

    rviz_config_file = os.path.join(pkg, 'rviz', 'urdf.rviz')

    return LaunchDescription([
        # Publishes TF transforms from the URDF joint states
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        # GUI sliders for any joints (no-op if URDF has no joints, but harmless)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
        ),
        # RViz2 viewer
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file, '-f', base_link],
        ),
    ])
