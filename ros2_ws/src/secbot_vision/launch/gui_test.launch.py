# Real camera — launches camera driver + detector.
#
# Requires rpicam-vid running on the Pi host (outside Docker):
#   rpicam-vid -t 0 --width 640 --height 480 --framerate 30 \
#       --codec mjpeg --inline -l -o tcp://0.0.0.0:8554
#
# Or enable the rpicam-stream systemd service:
#   sudo systemctl enable --now rpicam-stream
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('secbot_vision')

    debug_filter_arg = DeclareLaunchArgument(
        'debug_filter',
        default_value='',
        description=(
            'When non-empty, the detector runs only this filter and reads '
            'its HSV ranges live from /color_tunning (rgb_slider_gui). '
            'Example values: red, green, blue, purple, yellow'
        ),
    )

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='Camera image topic the detector subscribes to',
    )

    rgb_slider_node = Node(
            package='secbot_vision',
            executable='rgb_slider_gui',
            name='rgb_slider_gui',
            output='screen',
            env={'DISPLAY': os.environ.get('DISPLAY', ':0')},
        )
    
    detector_node = Node(
        package='secbot_vision',
        executable='detector_node',
        name='detector_node',
        output='screen',
        parameters=[{
            'image_topic':  LaunchConfiguration('image_topic'),
            'debug_filter': LaunchConfiguration('debug_filter'),
            'debug_viz':    True,
        }],
    )

    return LaunchDescription([
        debug_filter_arg,
        image_topic_arg,
        rgb_slider_node,
        detector_node
    ])
