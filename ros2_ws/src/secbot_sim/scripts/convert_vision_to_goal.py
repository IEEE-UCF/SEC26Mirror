#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
from secbot_msgs.msg import DuckDetections

class ConvertVisionToGoal(Node):
    def __init__(self):
        super().__init__("convert_vision_to_goal")

        # subs and pubs DECLARE ==========================
        self.declare_parameter("detections_topic", "/duck_detections")
        self.declare_parameter("camera_info_topic", "/camera_info")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("goal_topic", "/goal_pose")

        # subs and pubs ==============================
        self.detection_topic = self.get_parameter("detections_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.goal_topic = self.get_parameter("goal_topic").value

        # camera info DECLARE ======================================
        self.declare_parameter("camera_height", 0.30)
        self.declare_parameter("camera_tilt_deg", 0)
        self.declare_parameter("goal_standoff", 0.50)
        self.declare_parameter("min_confidence", 60.0)
        self.declare_parameter("use_largest_area", True)

        # camera info =================================
        self.camera_height = float(self.get_parameter("camera_height").value)
        self.camera_tilt_deg = float(self.get_parameter("camera_tilt_deg").value)
        self.goal_standoff = float(self.get_parameter("goal_standoff").value)
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.use_largest_area = bool(self.get_parameter("use_largest_area").value)
        self.tilt_rad = math.radians(self.camera_tilt_deg)

        # used to convert math to gazebo points ============================================
        self.declare_parameter("grid_origin_x", -8.0)
        self.declare_parameter("grid_origin_y", -4.0)
        self.declare_parameter("grid_res", 0.25)
        self.declare_parameter("grid_width", 64)
        self.declare_parameter("grid_height", 32)


        # state =============================
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None
        


        # QOS (SUBSCRIBE TO RELIABLE MESSAGES) =======================================
        reliable_qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.RELIABLE)

        # subscriptions and publishes ========================

        self.create_subscription(CameraInfo, self.camera_info_topic, self.on_camera_info, reliable_qos)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, reliable_qos)
        self.create_subscription(DuckDetections, self.detection_topic, self.on_detections, reliable_qos)

        # publisher ===============================
        self.goal_publish = self.create_publisher(PoseStamped, self.goal_topic, 10)

        self.get_logger().info(f"Listening detections: {self.detection_topic}")
        self.get_logger().info(f"Listening camera_info: {self.camera_info_topic}")
        self.get_logger().info(f"Listening odom: {self.odom_topic}")
        self.get_logger().info(f"Publishing goal: {self.goal_topic}")
        self.get_logger().info(f"camera_height={self.camera_height}m tilt={self.camera_tilt_deg:.1f}deg standoff={self.goal_standoff}m")


    # camera info including where it's located on robot and center x center y =================================================
    def on_camera_info(self, msg: CameraInfo):
        if self.fx is not None:
            return
        
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

        self.get_logger().info(f"GOT INFO fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}")

    # after recieving odom information find where robot is and where it's facing ==================================
    def on_odom(self, msg: Odometry):
        def convert_yaw(yaw):
            return math.atan2(2.0 * (yaw.w * yaw.z + yaw.x * yaw.y), 1.0 - 2.0 * (yaw.y * yaw.y + yaw.z * yaw.z))
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = convert_yaw(msg.pose.pose.orientation)


    # simple enough, pick which duck to detect ==============================
    def pick_detection(self, detections_msg: DuckDetections):
        candidates = [d for d in detections_msg.detections if float(d.confidence) >= self.min_confidence]
        if not candidates:
            return None

        if self.use_largest_area:
            return max(candidates, key=lambda d: float(d.area))
        else:
            return max(candidates, key=lambda d: float(d.confidence))
        
    # convert the world value to cell value for pathing_node to understand the grid coordinates ============================
    def world_to_cell(self, xw, yw):
        ox = float(self.get_parameter("grid_origin_x").value)
        oy = float(self.get_parameter("grid_origin_y").value)
        res = float(self.get_parameter("grid_res").value)
        w = int(self.get_parameter("grid_width").value)
        h = int(self.get_parameter("grid_height").value)

        col = int(math.floor((xw - ox) / res))
        row = int(math.floor((yw - oy) / res))

        # clamp
        col = max(0, min(w - 1, col))
        row = max(0, min(h - 1, row))
        return row, col


    # WHERE THE REAL STUFF HAPPENS ===========================
    def on_detections(self, msg: DuckDetections):
        if self.fx is None: # check if camera is publishing
            self.get_logger().warn("No CameraInfo yet (fx is None).")
            return
        if self.robot_x is None: # check if odom is set
            self.get_logger().warn("No /odom yet.")
            return

        detection = self.pick_detection(msg)
        if detection is None: # check if any ducks
            return
        

        # u = dx + dw * 0.5             center of duck
        u = float(detection.x + detection.w * 0.5)
        # v = dy + dh                   top of duck
        v = float(detection.y + detection.h)


        # phi = -arctan((u - center_camera_x) / camera x)
        bearing = -math.atan2((u - self.cx), self.fx)
        # theta = arctan((v - center_camera_y) / camera y)
        vertical_ray_angle = math.atan2((v-  self.cy), self.fy)

        # alpha = tilt + theta
        tot_down_angle = self.tilt_rad + vertical_ray_angle

        if tot_down_angle <= math.radians(1.0): # looking too far down
            self.get_logger().warn(f"tot_down_angle too small {math.degrees(tot_down_angle):.2f}")
            return

        # dist = cam_height / tan(alpha)
        dist_forward = self.camera_height / math.tan(tot_down_angle)

        if (not math.isfinite(dist_forward)) or dist_forward <= 0.0 or dist_forward > 12.0: # return if distance forward is non existent or too far
            return

        # goal = max(0.0, dist - amount away from goal)
        d_goal = max(0.0, dist_forward - self.goal_standoff)

        if d_goal <= 0.0: # return if robot is basically on top of object
            self.get_logger().info("too close, not publishing")
            return

        # rotate into odom/world
        cyaw = math.cos(self.robot_yaw)
        syaw = math.sin(self.robot_yaw)

        # rotate gazebo
        x_rel = d_goal * math.cos(bearing)
        y_rel = d_goal * math.sin(bearing)

        # compute (goal_x, goal_y) -> /goal_pose
        goal_x = self.robot_x + (cyaw * x_rel - syaw * y_rel)
        goal_y = self.robot_y + (syaw * x_rel + cyaw * y_rel)


        out = PoseStamped()
        out.header = msg.header
        out.header.frame_id = "odom"
        out.pose.position.x = float(goal_x)
        out.pose.position.y = float(goal_y)
        out.pose.position.z = 0.0
        out.pose.orientation.w = 1.0

        self.goal_publish.publish(out)

        self.get_logger().info(f"detection id = {detection.id} confidence={detection.confidence:.1f}% u={u:.1f} v={v:.1f} bearing={math.degrees(bearing):.1f}deg d={dist_forward:.2f}m -> goal=(x={goal_x:.2f}, y{goal_y:.2f})")



def main():
    rclpy.init()
    node = ConvertVisionToGoal()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()