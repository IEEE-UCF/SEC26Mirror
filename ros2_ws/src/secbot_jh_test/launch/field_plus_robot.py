"""
with everything you have just been taught in mind we
can make two nodes where one focuses on making the field and the other the robot...
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
def generate_launch_description():
    pkg_share = get_package_share_directory('secbot_jh_test')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    proper_field_path = os.path.join(pkg_share,'worlds','proper_field','proper_field.world')
    gz_sim_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim,'launch','gz_sim.launch.py')
            ),
            #-r runs the simulation immediately on startup
            launch_arguments={'gz_args': f'-r {proper_field_path}'}.items(),
        )
    setup =  SetEnvironmentVariable(
            name = 'GZ_SIM_RESOURCE_PATH',
            value=os.path.join(pkg_share,'worlds')# pkg_share points to .../share/secbot_jh_test, which contains 'worlds'
            )
    spawn_field = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name','field',
            '-file',os.path.join(pkg_share,'worlds','proper_field','proper_field_model.sdf'),
            '-x','1.0',#you could probably take it even farther and make the position 
            '-y','0.0',#properties in a YAML file!!
            '-z','0.5'
        ],
        output='screen',
    )
    return LaunchDescription([
        setup,
        gz_sim_launch
        # spawn_field
    ])
# for now the world file appears to not work!s