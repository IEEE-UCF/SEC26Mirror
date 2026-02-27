import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_secbot_sim = get_package_share_directory('secbot_sim')
    pkg_my_robot   = get_package_share_directory('my_robot_description')
    pkg_secbot_fusion = get_package_share_directory('secbot_fusion')
    ekf_config_path = os.path.join(pkg_secbot_fusion,'config','ekf.yaml')

    rviz_config    = os.path.join(pkg_my_robot,   'rviz',   'secbot_sim.rviz')
    mcu_sim_launch = os.path.join(pkg_secbot_sim, 'launch', 'mcu_sim_secbot.launch.py')
    
    # Direct path to bypass CMake installation for the bridge
    bridge_script = os.path.join(os.path.expanduser('~'), 'ros2_workspaces', 'src', 'sec26ros', 'secbot_sim', 'scripts', 'convert_uwb_msgs.py')

    return LaunchDescription([
        # 1. Full simulation backend
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mcu_sim_launch)
        ),

        # 2. RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}]
        ),
        
        # 3. Vacabun UWB Simulation - Direct path to the built binary
        ExecuteProcess(
            cmd=[
                '/home/ubuntu/ros2_workspaces/install/uwb_simulation/lib/uwb_simulation/uwb_simulation_node',
                '--ros-args', '-p', 'label_name:=my_robot'
            ],
            cwd='/home/ubuntu/',
            output='screen'
        ),

        # 4. Python Bridge - Use 'python3' and the direct path to the script
        ExecuteProcess(
            cmd=[
                'python3', 
                '/home/ubuntu/ros2_workspaces/src/sec26ros/secbot_sim/scripts/convert_uwb_msgs.py'
            ],
            output='screen'
        ),

        # 5. Your C++ Positioning Node
        Node(
            package='secbot_uwb',
            executable='positioning_node',
            name='uwb_positioning_node',
            output='screen',
            parameters=['/home/ubuntu/ros2_workspaces/src/sec26ros/secbot_uwb/config/beacons.yaml'],
            # This helps debug what the node actually "sees"
            arguments=['--ros-args', '--log-level', 'info'] 
        ),

        # EKF node (commented out until tuned) ==============
        Node(
             package='robot_localization',
             executable='ekf_node',
             name='ekf_node',
             output='screen',
             parameters=[ekf_config_path, {'use_sim_time': True}],
             remappings=[('/odom/unfiltered', '/odom')],
         ),

         Node(
             package='secbot_fusion',
             executable='fusion_node',
             name='fusion_node',
             output='screen',
             parameters=[{'use_sime_time':True}]
         )
    ])