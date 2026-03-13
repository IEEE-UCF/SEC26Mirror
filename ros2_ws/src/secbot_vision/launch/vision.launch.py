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

    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='tcp',
                              description='Camera backend: auto, picamera2, tcp, or opencv'),
        DeclareLaunchArgument('device_id', default_value='2',
                              description='Video device index for OpenCV backend'),
        DeclareLaunchArgument('tcp_host', default_value='127.0.0.1',
                              description='TCP stream host (host networking = 127.0.0.1)'),
        DeclareLaunchArgument('tcp_port', default_value='8554',
                              description='TCP stream port'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('frame_rate', default_value='30.0'),

        # Camera driver
        Node(
            package='secbot_vision',
            executable='camera_node',
            name='camera_node',
            parameters=[{
                'image_topic': '/camera/image_raw',
                'backend': LaunchConfiguration('backend'),
                'device_id': LaunchConfiguration('device_id'),
                'tcp_host': LaunchConfiguration('tcp_host'),
                'tcp_port': LaunchConfiguration('tcp_port'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'frame_rate': LaunchConfiguration('frame_rate'),
            }],
            output='screen',
        ),

        # Detector (yellow object detection + bounding box debug image)
        Node(
            package='secbot_vision',
            executable='detector_node',
            name='detector_node',
            parameters=[
                os.path.join(pkg_share, 'config', 'vision.yaml'),
            ],
            output='screen',
        ),
    ])
