"""Camera publisher node — captures frames and publishes to /camera/image_raw.

Backends (in 'auto' order):
  1. picamera2  — Pi CSI cameras (requires picamera2 package)
  2. tcp        — MJPEG stream from rpicam-vid over TCP (for Docker)
  3. opencv     — /dev/video<id> via V4L2 (USB cameras)

For Pi 5 CSI cameras inside Docker, run on the host:
  rpicam-vid -t 0 --width 640 --height 480 --framerate 30 --codec mjpeg \
      --inline -l -o tcp://0.0.0.0:8554
Then launch this node with backend:=tcp.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('device_id', 2)
        self.declare_parameter('backend', 'auto')  # 'auto', 'picamera2', 'tcp', or 'opencv'
        self.declare_parameter('tcp_host', '192.168.4.1')  # host IP from container's perspective
        self.declare_parameter('tcp_port', 8554)

        topic = self.get_parameter('image_topic').value
        self.fps = self.get_parameter('frame_rate').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.device_id = self.get_parameter('device_id').value
        self.tcp_host = self.get_parameter('tcp_host').value
        self.tcp_port = self.get_parameter('tcp_port').value
        backend = self.get_parameter('backend').value

        self.publisher_ = self.create_publisher(Image, topic, 10)
        self.bridge = CvBridge()
        self.picam = None
        self.cap = None
        self._backend = None

        backends_to_try = {
            'auto': ['picamera2', 'tcp', 'opencv'],
            'picamera2': ['picamera2'],
            'tcp': ['tcp'],
            'opencv': ['opencv'],
        }.get(backend, ['picamera2', 'tcp', 'opencv'])

        for b in backends_to_try:
            if b == 'picamera2' and self._try_picamera2():
                self._backend = 'picamera2'
                break
            elif b == 'tcp' and self._try_tcp():
                self._backend = 'tcp'
                break
            elif b == 'opencv' and self._try_opencv():
                self._backend = 'opencv'
                break

        if self._backend is None:
            self.get_logger().fatal('No camera backend available')
            raise RuntimeError('No camera backend available')

        self.get_logger().info(
            f'Using {self._backend} backend — '
            f'{self.width}x{self.height} @ {self.fps}Hz → {topic}'
        )
        period = 1.0 / self.fps
        self.timer = self.create_timer(period, self._timer_cb)

    def _try_picamera2(self) -> bool:
        try:
            from picamera2 import Picamera2
            cam_list = Picamera2.global_camera_info()
            if not cam_list:
                self.get_logger().info('picamera2: no cameras found')
                return False
            self.picam = Picamera2()
            config = self.picam.create_still_configuration(
                main={'size': (self.width, self.height), 'format': 'RGB888'}
            )
            self.picam.configure(config)
            self.picam.start()
            return True
        except Exception as e:
            self.get_logger().info(f'picamera2 init failed: {e}')
            self.picam = None
            return False

    def _try_tcp(self) -> bool:
        try:
            import cv2
            url = f'tcp://{self.tcp_host}:{self.tcp_port}'
            self.get_logger().info(f'Trying TCP stream: {url}')
            self.cap = cv2.VideoCapture(url)
            if not self.cap.isOpened():
                self.get_logger().info(f'TCP stream not available at {url}')
                self.cap = None
                return False
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().info(f'TCP stream opened but no frames from {url}')
                self.cap.release()
                self.cap = None
                return False
            return True
        except Exception as e:
            self.get_logger().info(f'TCP init failed: {e}')
            self.cap = None
            return False

    def _try_opencv(self) -> bool:
        try:
            import cv2
            self.cap = cv2.VideoCapture(self.device_id)
            if not self.cap.isOpened():
                self.get_logger().info(f'Cannot open /dev/video{self.device_id}')
                self.cap = None
                return False
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            ret, _ = self.cap.read()
            if not ret:
                self.get_logger().info(
                    f'/dev/video{self.device_id} opened but no frames'
                )
                self.cap.release()
                self.cap = None
                return False
            return True
        except Exception as e:
            self.get_logger().info(f'OpenCV init failed: {e}')
            self.cap = None
            return False

    def _timer_cb(self):
        if self.picam is not None:
            frame = self.picam.capture_array()
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='rgb8')
        else:
            import cv2
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn('Frame capture failed', throttle_duration_sec=5.0)
                return
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        self.publisher_.publish(msg)

    def destroy_node(self):
        if self.picam is not None:
            self.picam.stop()
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
