
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_secbot_sim = get_package_share_directory('secbot_sim')
    
    # Paths
    urdf_file = os.path.join(pkg_secbot_sim, 'urdf', 'my_bot.urdf')
    rviz_config = os.path.join(pkg_secbot_sim, 'rviz', 'sim.rviz')
    mcu_sim_launch = os.path.join(pkg_secbot_sim, 'launch', 'mcu_sim.launch.py')

    # Read URDF for Robot State Publisher
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        # 1. Start the complete Simulation Backend (Gazebo + MCU + Pathing Node)
        # This brings up everything needed for the robot to actually work.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mcu_sim_launch)
        ),

        # 2. Robot State Publisher
        # Publishes semantic link information (TF) from the URDF so RViz knows what the robot looks like
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # 3. Joint State Publisher
        # Publishes default joint states (0) so RSP can publish transforms for the continuous wheel joints
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_gui': False}]
        ),

        # 4. RViz2
        # Visualizes the robot's state, path, and sensor data
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )



    ])
