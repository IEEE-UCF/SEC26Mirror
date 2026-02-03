# heloo
#cool feature you can do "shift+enter" to run code for python???
# def fjfj():
#     print("hhelo");
"""
The launch system in ROS 2 is responsible for helping the user describe the configuration of their
associate simulation(in this case GAZEBO!) and then execuete it as described in the file(in this case the python file).
The configuration of the system inlcudes what programs to run, WHERE to run them, what arguments to pass them, and ROS-specific 
conventions which make it easy to reuse components throughout the system by giving them each a different configuration. It is also responsible
for monitoring the state of the processes launched, and reporting and/or reacting to changes in the state of those processes.
Launch files written in XML, YAML, or python can START and STOP different nodes(which is probably the equivalent of Tasks in FreeRTOS) as well as
trigger and act on various events.
1.You must create a directory(folder) to store your launch files(just make a "launch" folder)
here is a sample launch python file
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LanuchDescription([
        Node(
            package = 'turtlesim', // can be replaced with YOUR OWN personal package which is what you make with the package.XML file + the CMAKELIST(this is where you include the executables, launch files and what not/...)
            namespace = 'turtlesim1', // not really necessary
            executable = 'turtlesim_node', //can be replaced with a cpp file...you'll see
            name = 'sim',
            arguments = ['--ros-args', ''--log-level', 'info'])
            ])
let us see what we can implemt over here...
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
    # Path to your uploaded world file
    """
    I mean just look at that wozaa!
    pkg_share is like the parent directory housing the worlds folder holding the empty_world.sdf...
    """
    world_path = os.path.join(pkg_share,'worlds','empty_world.sdf')
    
    #launch action for the "empty_world" file
    gz_sim_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim,'launch','gz_sim.launch.py')
            ),
            #-r runs the simulation immediately on startup
            launch_arguments={'gz_args': f'-r {world_path}'}.items(),
        )


    #spawner Node for coke can
    spawn_model = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name','coke_can',
            '-file',os.path.join(pkg_share,'Coke','model.sdf'),
            '-x','1.0',#you could probably take it even farther and make the position 
            '-y','0.0',#properties in a YAML file!!
            '-z','0.5'
        ],
        output='screen',
    )
    return LaunchDescription([
        SetEnvironmentVariable(
            name = 'GZ_SIM_RESOURCE_PATH',
            value=pkg_share# pkg_share points to .../share/secbot_jh_test, which contains 'Coke
            ),
        gz_sim_launch,
        spawn_model
    ])
"""
Instead of typing a massive command in the terminal every time, this script automates the entire build process.
It finds the installation path of your package(get_package_share_directory)<--check cmaklist.txt comments
locates your world file, and tells the ros_gz_sim package to start Gazebo using that specific world


Now if you take real notice you'll see that there is NO model in this FILE! NOR IN THE empty_world.sdf FILE!!!
I may have created the world BUT i have not PLACED ANY OBJECTS in there I can either do two things..either I go into the
empty_world.sdf file and delibrately say
<include> <--you can say this since package.xml handles all the pathing stuff
  <uri>model://Coke</uri>
  <name>coke_can_1</name>
  <pose>1 0 0.5 0 0 0</pose> 
</include>
OR
I MAKE A NODE in this LAUNCH file that care of adding that coke obejct...remember NODES are tasks(almost the same concept of TASKS in FreeRTOS!) you can make
em spawn objects, compute a number, make objects move OR all 3 it is completely up to you to decide the functionality of the NODE!

once all set and done 
1.Build: Run colcon build --packages-select secbot_jh_test
2.Source Run source install/setup.bash
3.Launch Run ros2 launch secbot_jh_test first_launch.py

and if you are super lazy just make a little script those three lines!!
just make sure to run that script EVERYTIME you make any changes whatsoEVER!
"""