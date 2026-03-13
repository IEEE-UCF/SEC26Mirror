# Real camera — launches camera driver + detector.
#
# Requires rpicam-vid running on the Pi host (outside Docker):
#   rpicam-vid -t 0 --width 640 --height 480 --framerate 30 \
#       --codec mjpeg --inline -l -o tcp://0.0.0.0:8554
#
# Or enable the rpicam-stream systemd service:
#   sudo systemctl enable --now rpicam-stream
#
# Camera profile is read from the environment (set via .env + launch_vision.sh):
#   CAMERA_PROFILE=rpicam  →  TCP stream from rpicam-vid
#   CAMERA_PROFILE=webcam  →  USB/laptop webcam via OpenCV
#
# Individual settings can be overridden via env vars or ros2 launch args:
#   ros2 launch secbot_vision vision.launch.py backend:=opencv device_id:=2
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

# ── profile → default settings ───────────────────────────────────────────────
_PROFILE_DEFAULTS = {
    'rpicam': {
        'backend':    'tcp',
        'tcp_host':   os.environ.get('RPICAM_TCP_HOST', '127.0.0.1'),
        'tcp_port':   os.environ.get('RPICAM_TCP_PORT', '8554'),
        'device_id':  '0',
    },
    'webcam': {
        'backend':    'opencv',
        'tcp_host':   '127.0.0.1',
        'tcp_port':   '8554',
        'device_id':  os.environ.get('WEBCAM_DEVICE_ID', '0'),
    },
}

_profile  = os.environ.get('CAMERA_PROFILE', 'rpicam')
_defaults = _PROFILE_DEFAULTS.get(_profile, _PROFILE_DEFAULTS['rpicam'])

# CAMERA_BACKEND env var overrides the profile's backend when set
_backend_override = os.environ.get('CAMERA_BACKEND', '').strip()

_default_backend    = _backend_override if _backend_override else _defaults['backend']
_default_tcp_host   = _defaults['tcp_host']
_default_tcp_port   = _defaults['tcp_port']
_default_device_id  = _defaults['device_id']
_default_width      = os.environ.get('CAMERA_WIDTH',  '640')
_default_height     = os.environ.get('CAMERA_HEIGHT', '480')
_default_fps        = os.environ.get('CAMERA_FPS',    '30.0')


def generate_launch_description():
    pkg_share = get_package_share_directory('secbot_vision')

    return LaunchDescription([
        DeclareLaunchArgument('backend',    default_value=_default_backend,
                              description='Camera backend: auto, picamera2, tcp, or opencv'),
        DeclareLaunchArgument('device_id',  default_value=_default_device_id,
                              description='Video device index for OpenCV backend'),
        DeclareLaunchArgument('tcp_host',   default_value=_default_tcp_host,
                              description='TCP stream host'),
        DeclareLaunchArgument('tcp_port',   default_value=_default_tcp_port,
                              description='TCP stream port'),
        DeclareLaunchArgument('width',      default_value=_default_width),
        DeclareLaunchArgument('height',     default_value=_default_height),
        DeclareLaunchArgument('frame_rate', default_value=_default_fps),

        # Camera driver
        Node(
            package='secbot_vision',
            executable='camera_node',
            name='camera_node',
            parameters=[{
                'image_topic': '/camera/image_raw',
                'backend':     LaunchConfiguration('backend'),
                'device_id':   LaunchConfiguration('device_id'),
                'tcp_host':    LaunchConfiguration('tcp_host'),
                'tcp_port':    LaunchConfiguration('tcp_port'),
                'width':       LaunchConfiguration('width'),
                'height':      LaunchConfiguration('height'),
                'frame_rate':  LaunchConfiguration('frame_rate'),
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
