import os
# from turtle import position
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
def generate_launch_description():
    pkg_share = get_package_share_directory('secbot_jh_test')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    enviornemnt_set = SetEnvironmentVariable(name = 'GZ_SIM_RESOURCE_PATH',value=pkg_share)
    anchors_config_file_path = os.path.join(pkg_share,'config', 'anchors.yaml')
    anchor_model_path = os.path.join(pkg_share, 'worlds', 'beacon1.sdf')
    moving_anchor_model_path = os.path.join(pkg_share, 'worlds', 'beacon2.sdf')
    tag_model_path = os.path.join(pkg_share,'worlds', 'tag.sdf')
    empty_world_path = os.path.join(pkg_share, 'worlds', 'empty_world.sdf')
    world_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim,'launch','gz_sim.launch.py')
            ),
            #-r runs the simulation immediately on startup
            launch_arguments={'gz_args': f'-r {empty_world_path}'}.items(),
    )

    with open(anchors_config_file_path, 'r') as f:
        params_dict = yaml.safe_load(f)
    beacons_data = params_dict['/**']['ros__parameters']['beacons']
    spawn_beacons = []
    for beacon_id, data in beacons_data.items():
        pos = data['position']
        beacon_type = data['type']
        if beacon_type =='stationary':
            beacon_name = f"anchor_{beacon_id}"
            spawn_beacon = Node(
                package='ros_gz_sim',
                executable='create',
                arguments = [
                    '-name', beacon_name,
                    '-file', anchor_model_path,
                    '-x', str(pos['x']),
                    '-y', str(pos['y']),
                    '-z', str(pos['z'])
                ],
                output = 'screen'
            )
            spawn_beacons.append(spawn_beacon)
        elif beacon_type == 'moving':
            moving_beacon_name = f"moving_anchor{beacon_id}"
            spawn_moving_beacon = Node(
                package= 'ros_gz_sim',
                executable='create',
                arguments=[
                '-name', moving_beacon_name,
                '-file', moving_anchor_model_path,
                '-x', str(pos['x']),
                '-y', str(pos['y']),
                '-z', str(pos['z'])
                ],
                output = 'screen'
            )
            spawn_beacons.append(spawn_moving_beacon)
    spawn_tags = []
    tag_data = params_dict['/**']['ros__parameters']['tags']
    for tag_id, data_tag in tag_data.items():
        pos = data_tag['position']
        tag_name = f"tag_{tag_id}"
        spawn_tag_node = Node(
            package='ros_gz_sim',
            executable= 'create',
            arguments=[
                '-name',tag_name,
                '-file',tag_model_path,
                '-x', str(pos['x']),
                '-y', str(pos['y']),
                '-z', str(pos['z'])
            ],            
            output='screen'
        )
        spawn_tags.append(spawn_tag_node)
    drone_tag_node = Node(
        package='secbot_jh_test',
        executable='uwb_tag_node_cmake',
        namespace='drone',
        parameters=[{'tag_name': 'tag_20'}]
    )
    # drone_tag_node = Node(
    #     package='secbot_jh_test',
    #     executable='uwb_tag_node_cmake',
    #     namespace='drone',
    #     parameters=[{'tag_name': 'Drone'}]
    # )
    final_launch_list = [
        enviornemnt_set,
        world_launch
    ]
    final_launch_list.extend(spawn_beacons)
    final_launch_list.extend(spawn_tags)
    final_launch_list.append(drone_tag_node)
    return LaunchDescription(
        final_launch_list
)
