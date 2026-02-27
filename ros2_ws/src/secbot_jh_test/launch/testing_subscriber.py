import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription #always needed for any ros2 setup basically anytime you wanna run
#a launch file using the ros2 distro blah blah blah you get it.... 
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
def generate_launch_description():
    pkg_share = get_package_share_directory('secbot_jh_test')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')#probably not needed either???
    subscriber_node = Node(
        package='secbot_jh_test',
        name = 'subscriber_node',
        executable='subscriber_node_cmake',
        output = 'screen'
    )
    return LaunchDescription([
        subscriber_node
    ])
"""
okay seems like we got what we came for just be super weary of the way you build your components together
now that we got node interactions ITS TIME TO SIMULATE THE DRONE!!!
"""