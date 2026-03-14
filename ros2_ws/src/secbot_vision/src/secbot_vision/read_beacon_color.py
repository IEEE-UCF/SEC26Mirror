# action server for beacon reading

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from secbot_msgs.action import ReadBeaconColor as ReadBeaconColorAction
from mcu_msgs.srv import SetCameraAngle, SetServo
from secbot_msgs.msg import AntennaDetections
import time

class ReadBeaconColorServer(Node):
    def __init__(self):
        super().__init__('read_beacon_color')
        self.declare_parameter('servo_settle_ms',500)
        self.declare_parameter('camera_servo_index', 0)
        self.declare_parameter('read_timeout_s', 5.0)
        self.servo_settle_ms = self.get_parameter('servo_settle_ms').value
        self.camera_servo_index = self.get_parameter('camera_servo_index').value
        self.read_timeout_s = self.get_parameter('read_timeout_s').value

        self._cb_group = ReentrantCallbackGroup()

        # SetServo client (MCU servo control)
        self.servo_client = self.create_client(
            SetServo, '/mcu_robot/servo/set',
            callback_group=self._cb_group)

        # SetCameraAngle service server (calls SetServo under the hood)
        self.create_service(
            SetCameraAngle, 'set_camera_angle',
            self._set_camera_angle_cb,
            callback_group=self._cb_group)

        # SetCameraAngle client (used by action callback)
        self.camera_client = self.create_client(
            SetCameraAngle, 'set_camera_angle',
            callback_group=self._cb_group)

        self.action_server = ActionServer(self, ReadBeaconColorAction,'read_beacon_color', self.execute_callback,
            callback_group=self._cb_group)
        self.latest_detection = None
        self.create_subscription(AntennaDetections,'/antenna_detections', self.detect_callback,10,
            callback_group=self._cb_group)

    def detect_callback(self, msg):
        self.latest_detection = msg

    def _set_camera_angle_cb(self, request, response):
        """SetCameraAngle service handler -- forwards to SetServo."""
        if not self.servo_client.wait_for_service(timeout_sec=2.0):
            response.status = False
            return response

        req = SetServo.Request()
        req.index = self.camera_servo_index
        req.angle = float(request.camera_angle)
        future = self.servo_client.call_async(req)

        while not future.done():
            time.sleep(0.01)

        if future.result() is not None:
            response.status = future.result().success
        else:
            response.status = False
        return response

    def execute_callback(self, goal_handle):
        angle = goal_handle.request.camera_angle_goal

        req = SetCameraAngle.Request()
        feedback = ReadBeaconColorAction.Feedback()

        req.camera_angle = angle

        feedback.status = 'rotating'
        goal_handle.publish_feedback(feedback)
        future = self.camera_client.call_async(req)

        while not future.done():
            time.sleep(0.01)

        time.sleep(self.servo_settle_ms / 1000.0)

        feedback.status = 'reading'
        goal_handle.publish_feedback(feedback)
        # Clear so any new detection is guaranteed post-settle
        self.latest_detection = None

        deadline = time.monotonic() + self.read_timeout_s
        while time.monotonic() < deadline:
            det = self.latest_detection
            if det is not None:
                if det.detections:
                    result = ReadBeaconColorAction.Result()
                    result.color = det.detections[0].color
                    self.get_logger().info(f'Detected color: {result.color}')
                    goal_handle.succeed()
                    return result
                else:
                    # Got a detection msg but no colors — clear and keep waiting
                    self.latest_detection = None
            time.sleep(0.02)

        self.get_logger().warn('Timed out waiting for antenna detection — is the antenna detector running?')
        result = ReadBeaconColorAction.Result()
        result.color = 'unknown'
        goal_handle.abort()
        return result

def main(args=None):
    rclpy.init(args=args)
    node = ReadBeaconColorServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt,ExternalShutdownException):
        node.get_logger().info('[read_beacon_color_action]: Shutting down')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
