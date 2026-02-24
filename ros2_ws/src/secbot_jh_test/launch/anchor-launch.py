import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from ros_gz_bridge.actions import RosGzBridge
def generate_launch_description():
    pkg_share = get_package_share_directory('secbot_jh_test')
,.kmjlhuihiuhuuihuik