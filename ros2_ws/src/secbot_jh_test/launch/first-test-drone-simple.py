import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from ros_gz_bridge.actions import RosGzBridge
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
def generate_launch_description():#big side note apparently the ROS2 system DOES specifically looks for this function
    #AND looks for the return launch description so be super weary of that....
    pkg_share = get_package_share_directory('secbot_jh_test')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    urdf_file = os.path.join(pkg_share,'worlds','cf2x.urdf')
    enviornemnt_set = SetEnvironmentVariable(name = 'GZ_SIM_RESOURCE_PATH',value=pkg_share)# pkg_share points to .../share/secbot_jh_test, which contains 'Coke)    
    #world path
    world_path = os.path.join(pkg_share,'worlds','empty_world.sdf')
    #world spawn
    world_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim,'launch','gz_sim.launch.py')
            ),
            #-r runs the simulation immediately on startup
            launch_arguments={'gz_args': f'-r {world_path}'}.items(),
        )
    #read the urdf file
    with open(urdf_file, 'r') as infp:
        drone_description = infp.read()
    """
    the code below(just below) this comment is made for the purposes of connecting the topics of ROS2 and GAZEBO
    the drone_state_publisher read the URDF text, checks movements, and tells you locations of EVERY PART (RViz, SLAM?)
    
    once the launch is running you can type this in another terminal...
    ros2 topic echo/robot_description
    ros2 topic echp/tf <-- this is used for positions...we'll see more on this

    for better viewing of the nodes and their topics use the 
    rqt_graph
    """
    drone_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher', #no need to include this in the cmake file seems like ROS2 has this already??
        name = 'drone_state_publisher',
        output = 'both',
        parameters=[
            {'use_sim_time': True}, #to keep everything stable time will be counted from the Gazebo clock NOT your personal system clock!
            {'robot_description':drone_description},#funny...this might be the topic but for who?? ros2 or gazebo??
        ]
    )
    # drone spawn
    drone_spawn_node = Node(
        package='ros_gz_sim',
        executable='create', #YES you do NEED this....don't believe me?...https://gazebosim.org/docs/latest/spawn_urdf/
        arguments=[
            '-topic', 'robot_description',
            '-name','first_drone',
            '-z','0.5'
        ],
        output = 'screen'
    )

    #bridge node--optional for now...we are simply SPAWNING...once we start doing interactions like rotating motors and whatnot
    #using keystrokes
    bridge_config   = os.path.join(
        pkg_share,
        'config','ros-gz-bridge.yaml',
    )
    bridge = Node (
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_config}'
        ],
        output = 'screen'
    )
    return LaunchDescription([
        enviornemnt_set,
        world_launch,
        drone_state_publisher,
        drone_spawn_node,
        bridge
        # bridge
        # drone_node
    ])

"""
ubuntu@76bfefec964b:~/ros2_workspaces/src/sec26ros/secbot_jh_test$ cd ~/ros2_workspaces/src/sec26ros
colcon build --symlink-install
source install/setup.bash
Starting >>> mcu_msgs
Starting >>> secbot_msgs
Starting >>> secbot_bridge_i2c
Starting >>> secbot_fusion
Starting >>> secbot_health
Starting >>> secbot_navigation
Starting >>> secbot_sim
Starting >>> secbot_tf
Starting >>> test_package
Finished <<< secbot_fusion [18.6s]
Finished <<< secbot_health [20.6s]
Finished <<< secbot_bridge_i2c [22.4s]
Finished <<< secbot_sim [24.4s]
Finished <<< secbot_tf [26.2s]
Finished <<< test_package [33.6s]
Finished <<< secbot_navigation [58.1s]
[Processing: mcu_msgs, secbot_msgs]
[Processing: mcu_msgs, secbot_msgs]
Finished <<< secbot_msgs [2min 21s]
Starting >>> secbot_vision
Finished <<< secbot_vision [11.8s]
Finished <<< mcu_msgs [3min 5s]
Starting >>> secbot_autonomy
Starting >>> secbot_jh_test
Starting >>> secbot_uwb
[Processing: secbot_autonomy, secbot_jh_test, secbot_uwb]
Finished <<< secbot_jh_test [45.2s]
[Processing: secbot_autonomy, secbot_uwb]
Finished <<< secbot_uwb [1min 27s]
Finished <<< secbot_autonomy [1min 50s]

Summary: 13 packages finished [4min 56s]
ubuntu@76bfefec964b:~/ros2_workspaces/src/sec26ros$ cd secbot_jh_test
ubuntu@76bfefec964b:~/ros2_workspaces/src/sec26ros/secbot_jh_test$ ./first_drone_launch.sh
Starting >>> secbot_jh_test
Finished <<< secbot_jh_test [4.26s]

Summary: 1 package finished [4.55s]
[INFO] [launch]: All log files can be found below /home/ubuntu/.ros/log/2026-02-12-02-42-06-104840-76bfefec964b-9401
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [gazebo-1]: process started with pid [9545]
[INFO] [robot_state_publisher-2]: process started with pid [9546]
[INFO] [create-3]: process started with pid [9547]
[INFO] [parameter_bridge-4]: process started with pid [9548]
[create-3] [INFO] [1770864140.349421699] [ros_gz_sim]: Requesting list of world names.
[parameter_bridge-4] [INFO] [1770864140.440555229] [ros_gz_bridge]: Creating GZ->ROS Bridge: [/clock (gz.msgs.Clock) -> /clock (rosgraph_msgs/msg/Clock)] (Lazy 0)
[robot_state_publisher-2] [WARN] [1770864140.654818695] [kdl_parser]: The root link base_link has an inertia specified in the URDF, but KDL does not support a root link with an inertia.  As a workaround, you can add an extra dummy link to your URDF.
[robot_state_publisher-2] [INFO] [1770864140.654919648] [drone_state_publisher]: Robot initialized
[gazebo-1] QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-ubuntu'
[create-3] [INFO] [1770864141.473504832] [ros_gz_sim]: Waiting messages on topic [robot_description].
[create-3] [INFO] [1770864141.554018096] [ros_gz_sim]: Entity creation successful.
[INFO] [create-3]: process has finished cleanly [pid 9547]
[gazebo-1] [GUI] [Err] [SystemPaths.cc:425] Unable to find file with URI [model://secbot_jh_test/worlds/cf2.dae]
[gazebo-1] [GUI] [Err] [SystemPaths.cc:525] Could not resolve file [model://secbot_jh_test/worlds/cf2.dae]
[gazebo-1] [GUI] [Err] [MeshManager.cc:211] Unable to find file[model://secbot_jh_test/worlds/cf2.dae]
[gazebo-1] [GUI] [Err] [SceneManager.cc:426] Failed to load geometry for visual: base_link_visual

NEGATE THESE COMMENTS!!!
"""