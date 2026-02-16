from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_secbot_fusion = get_package_share_directory('secbot_fusion')
    ekf_config_path = os.path.join(pkg_secbot_fusion,'config','ekf.yaml')

    ekf_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            parameters=[ekf_config_path]
        )
    
    fusion_node = Node(
            package='secbot_fusion',
            executable='fusion_node',
            name='fusion_node'
        )

    return LaunchDescription([
        ekf_node,
        fusion_node
    ])