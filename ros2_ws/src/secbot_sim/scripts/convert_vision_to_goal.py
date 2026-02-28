#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from secbot_msgs.msg import DuckDetections

class ConvertVisionToGoal(Node):
    def __init__(self):
        super().__init__("convert_vision_to_goal")

        # subs and pubs DECLARE ==========================
        self.declare_parameter("detections_topic", "/duck_detections")
        self.declare_parameter("camera_info_topic", "/camera_info")
        self.declare_parameter("odom_topic", "/odometry/global")
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
        self.declare_parameter("visited_radius", 0.4)
        self.declare_parameter("queue_merge_radius", 0.5)

        # camera info =================================
        self.camera_height = float(self.get_parameter("camera_height").value)
        self.camera_tilt_deg = float(self.get_parameter("camera_tilt_deg").value)
        self.goal_standoff = float(self.get_parameter("goal_standoff").value)
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.use_largest_area = bool(self.get_parameter("use_largest_area").value)
        self.visited_radius = float(self.get_parameter("visited_radius").value)
        self.queue_merge_radius = float(self.get_parameter("queue_merge_radius").value)
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

        # Goal queue: remembers ALL detected goal positions ====
        self.goal_queue = []          # list of (x, y) — goals waiting to be visited
        self.active_goal = None       # (x, y) — the goal we're currently navigating to
        self.visited_positions = []   # list of (x, y) — completed goals


        # QOS (SUBSCRIBE TO RELIABLE MESSAGES) =======================================
        reliable_qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.RELIABLE)

        # subscriptions and publishes ========================

        self.create_subscription(CameraInfo, self.camera_info_topic, self.on_camera_info, reliable_qos)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, reliable_qos)
        self.create_subscription(DuckDetections, self.detection_topic, self.on_detections, reliable_qos)
        self.create_subscription(Bool, "/nav/goal_reached", self.on_goal_reached, reliable_qos)

        # publisher ===============================
        self.goal_publish = self.create_publisher(PoseStamped, self.goal_topic, 10)

        # Republish active goal at 2Hz so pathing_node always has it
        self.create_timer(0.5, self.republish_active_goal)

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


    # when pathing_node says we reached a goal, mark visited and advance queue ====
    def on_goal_reached(self, msg: Bool):
        if not msg.data or self.active_goal is None:
            return

        self.visited_positions.append(self.active_goal)
        self.get_logger().info(
            f"GOAL REACHED — marked visited at ({self.active_goal[0]:.2f}, {self.active_goal[1]:.2f}), "
            f"total visited: {len(self.visited_positions)}, queue remaining: {len(self.goal_queue)}")
        self.active_goal = None

        # advance to next queued goal
        self.advance_queue()

    def is_near_visited(self, gx, gy):
        for vx, vy in self.visited_positions:
            if math.hypot(gx - vx, gy - vy) < self.visited_radius:
                return True
        return False

    def is_near_queued_or_active(self, gx, gy):
        """Check if a goal is already in the queue or is the active goal."""
        for qx, qy in self.goal_queue:
            if math.hypot(gx - qx, gy - qy) < self.queue_merge_radius:
                return True
        if self.active_goal is not None:
            ax, ay = self.active_goal
            if math.hypot(gx - ax, gy - ay) < self.queue_merge_radius:
                return True
        return False

    def advance_queue(self):
        """Pop the closest queued goal (that isn't visited) and make it active."""
        while self.goal_queue:
            # pick closest goal to robot
            if self.robot_x is not None:
                self.goal_queue.sort(
                    key=lambda g: math.hypot(g[0] - self.robot_x, g[1] - self.robot_y))

            next_goal = self.goal_queue.pop(0)

            if self.is_near_visited(next_goal[0], next_goal[1]):
                continue  # skip, already visited

            # if robot is already on top of this goal, mark visited and skip
            if self.robot_x is not None:
                d = math.hypot(next_goal[0] - self.robot_x, next_goal[1] - self.robot_y)
                if d < 0.30:
                    self.visited_positions.append(next_goal)
                    self.get_logger().info(
                        f"SKIP — already at goal ({next_goal[0]:.2f}, {next_goal[1]:.2f}), d={d:.2f}m")
                    continue

            self.active_goal = next_goal
            self.publish_goal_msg(next_goal[0], next_goal[1])
            self.get_logger().info(
                f"QUEUE ADVANCE — now targeting ({next_goal[0]:.2f}, {next_goal[1]:.2f}), "
                f"remaining in queue: {len(self.goal_queue)}")
            return

        self.get_logger().info("Goal queue empty — waiting for new detections")

    def publish_goal_msg(self, gx, gy):
        """Publish a PoseStamped goal."""
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"
        out.pose.position.x = float(gx)
        out.pose.position.y = float(gy)
        out.pose.position.z = 0.0
        out.pose.orientation.w = 1.0
        self.goal_publish.publish(out)

    def republish_active_goal(self):
        """Timer callback: keep republishing active goal so pathing_node tracks it."""
        if self.active_goal is not None:
            self.publish_goal_msg(self.active_goal[0], self.active_goal[1])

    def compute_goal(self, detection):
        """Compute world-frame goal (x, y) for a detection. Returns (goal_x, goal_y, dist_forward, bearing) or None."""
        u = float(detection.x + detection.w * 0.5)
        v = float(detection.y + detection.h)

        bearing = -math.atan2((u - self.cx), self.fx)
        vertical_ray_angle = math.atan2((v - self.cy), self.fy)
        tot_down_angle = self.tilt_rad + vertical_ray_angle

        if tot_down_angle <= math.radians(1.0):
            return None

        dist_forward = self.camera_height / math.tan(tot_down_angle)

        if (not math.isfinite(dist_forward)) or dist_forward <= 0.0 or dist_forward > 8.0:
            return None

        d_goal = max(0.0, dist_forward - self.goal_standoff)
        if d_goal <= 0.0:
            return None

        cyaw = math.cos(self.robot_yaw)
        syaw = math.sin(self.robot_yaw)
        x_rel = d_goal * math.cos(bearing)
        y_rel = d_goal * math.sin(bearing)
        goal_x = self.robot_x + (cyaw * x_rel - syaw * y_rel)
        goal_y = self.robot_y + (syaw * x_rel + cyaw * y_rel)

        # reject goals outside the field
        ox = float(self.get_parameter("grid_origin_x").value)
        oy = float(self.get_parameter("grid_origin_y").value)
        res = float(self.get_parameter("grid_res").value)
        field_max_x = ox + int(self.get_parameter("grid_width").value) * res
        field_max_y = oy + int(self.get_parameter("grid_height").value) * res
        if goal_x < ox or goal_x > field_max_x or goal_y < oy or goal_y > field_max_y:
            return None

        return (goal_x, goal_y, dist_forward, bearing)

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

        # get all valid candidates sorted by area (largest first)
        candidates = [d for d in msg.detections if float(d.confidence) >= self.min_confidence]
        if not candidates:
            return

        if self.use_largest_area:
            candidates.sort(key=lambda d: float(d.area), reverse=True)
        else:
            candidates.sort(key=lambda d: float(d.confidence), reverse=True)

        # compute goals for ALL detections and queue new ones ====
        for detection in candidates:
            result = self.compute_goal(detection)
            if result is None:
                continue

            goal_x, goal_y, dist_forward, bearing = result

            # skip if already visited, already queued, or is the active goal
            if self.is_near_visited(goal_x, goal_y):
                continue
            if self.is_near_queued_or_active(goal_x, goal_y):
                continue

            # new goal — add to queue
            self.goal_queue.append((goal_x, goal_y))
            u = float(detection.x + detection.w * 0.5)
            v = float(detection.y + detection.h)
            self.get_logger().info(
                f"QUEUED detection id={detection.id} conf={detection.confidence:.1f}% "
                f"bearing={math.degrees(bearing):.1f}deg d={dist_forward:.2f}m "
                f"-> goal=({goal_x:.2f}, {goal_y:.2f})  queue size: {len(self.goal_queue)}")

        # if no active goal, pick the closest from queue
        if self.active_goal is None and self.goal_queue:
            self.advance_queue()



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
