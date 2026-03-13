# action server for beacon reading

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.action import ActionServer
from secbot_msgs.action import ReadBeaconColor as ReadBeaconColorAction
from mcu_msgs.srv import SetCameraAngle
from secbot_msgs.msg import DuckDetections
import time

class ReadBeaconColorServer(Node):
    def __init__(self):
        super().__init__('read_beacon_color')
        self.declare_parameter('servo_settle_ms',500)
        self.servo_settle_ms = self.get_parameter('servo_settle_ms').value
        self.camera_client = self.create_client(SetCameraAngle, 'set_camera_angle')
        self.action_server = ActionServer(self, ReadBeaconColorAction,'read_beacon_color', self.execute_callback)
        self.latest_detection = None
        self.create_subscription(DuckDetections,'/beacon_detections', self.detect_callback,10)

    def detect_callback(self, msg: DuckDetections):
        self.latest_detection = msg

    def execute_callback(self, goal_handle):
        angle = goal_handle.request.camera_angle_goal

        req = SetCameraAngle.Request()
        feedback = ReadBeaconColorAction.Feedback()
        
        req.camera_angle = angle

        feedback.status = 'rotating'
        goal_handle.publish_feedback(feedback)
        future = self.camera_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)  

        command_time = self.get_clock().now()
        time.sleep(self.servo_settle_ms / 1000.0)

        feedback.status = 'reading'
        goal_handle.publish_feedback(feedback)
        while(self.latest_detection is None or 
            self.latest_detection.header.stamp < command_time):
                rclpy.spin_once(self, timeout_sec=0.01)

        result = ReadBeaconColorAction.Result()
        result.color = self.latest_detection.detections[0].color if self.latest_detection.detections else 'unknown'
        goal_handle.succeed()
        return result
            
def main(args=None):
    rclpy.init(args=args)
    node = ReadBeaconColorServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        node.get_logger().info('[read_beacon_color_action]: Shutting down')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
