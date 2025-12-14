#!/usr/bin/env python3

# MAKE SURE YOU HAVE ALL INFO INSTALLED:
# source /opt/ros/jazzy/setup.bash
# colcon build --symlink-install --packages-select robot_workshop
# source install/local_setup.bash
# ros2 pkg executables robot_workshop


# TO RUN: 
# 1) IMPORT MODEL (Mustard bottle):
# ros2 run ros_gz_sim create   -world default   -name mustard_bottle   -file 'ros2_workspaces/src/sec26ros/src/robot_workshop/worlds/model.sdf'   -x 0.7 -y 0.0 -z 0.15 -Y 0.0

# 2) RUN GAZEBO (SEPERATE TERMINAL):
# ros2 launch robot_workshop robot_sim.launch.py image_topic:=/camera2/image_raw

# 3) SEE WHAT ROBOT SEES (OPTIONAL)
# ros2 run image_tools showimage --ros-args -r image:=/camera2/image_raw
# ros2 run image_tools showimage --ros-args -r image:=/vision/debug_image


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
from .duck_detector import process_frame
import cv2, os, yaml, numpy as np
from ament_index_python.packages import get_package_share_directory

class DetectorNode(Node):
    """DetectorNode class"""
    def __init__(self):
        """initalize all parameters, logging info and inputs and outputs"""
        super().__init__('detector_node')

        # Parameters
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('class_name', 'yellow_object')
        self.declare_parameter('pub_topic', '/detected_objects')
        self.declare_parameter('pub_debug_image', '/vision/debug_image')
        self.declare_parameter('debug_viz', True)
        self.declare_parameter('color_name', 'yellow')
        self.declare_parameter('target_hue', 25)
        self.declare_parameter('hue_change', 20)
        self.declare_parameter('kernel_size', 15)
        self.declare_parameter('min_area_ratio', 0.0002)

        # read parameters
        self.class_name = self.get_parameter('class_name').value
        image_topic     = self.get_parameter('image_topic').value
        self.pub_topic  = self.get_parameter('pub_topic').value
        self.debug_viz  = self.get_parameter('debug_viz').value
        self.pub_dbg_name = self.get_parameter('pub_debug_image').value

        # ROS Inputs and outputs
        qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.bridge  = CvBridge()
        self.sub     = self.create_subscription(Image, image_topic, self.on_image, qos)
        self.pub_det = self.create_publisher(Detection2DArray, self.pub_topic, 10)
        self.pub_dbg = self.create_publisher(Image, self.pub_dbg_name, 10) if self.debug_viz else None

        # Logging info
        self.get_logger().info(f"Using: {image_topic}")
        self.get_logger().info(f"Detections: {self.pub_topic}")
        if self.debug_viz:
            self.get_logger().info(f"Debug image: {self.pub_dbg_name}")

        pkg_share = get_package_share_directory('robot_workshop')
        colors_path = os.path.join(pkg_share, 'config', 'colors.yaml')

        with open(colors_path, 'r') as f:
            colors_cfg = yaml.safe_load(f)

        color_name = self.get_parameter('color_name').value
        color_entry = colors_cfg['colors'][color_name]
        self.hsv_low  = np.array(color_entry['hsv_low'], dtype=np.uint8)
        self.hsv_high = np.array(color_entry['hsv_high'], dtype=np.uint8)

        self.target_hue = int(self.get_parameter('target_hue').value)
        self.hue_change = int(self.get_parameter('hue_change').value)
        self.kernel_size = int(self.get_parameter('kernel_size').value)
        self.min_area_ratio = float(self.get_parameter('min_area_ratio').value)


    def on_image(self, msg: Image):
        """Call function and read the metadata/debuging details"""
        # Use OpenCV in ROS2
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # detector
        metadata, debug = process_frame(frame,
                                        hsv_low=self.hsv_low,
                                        hsv_high=self.hsv_high,
                                        target_hue=self.target_hue,
                                        hue_change=self.hue_change,
                                        kernel_size=self.kernel_size,
                                        min_area_ratio=self.min_area_ratio,
                                        )
        # build detection array
        arr = Detection2DArray() 
        # time stames of camera
        arr.header = msg.header
        for meta in metadata:
            x, y = meta["Position"]
            w, h = meta["Size"]
            score = meta["Score"]
            area = meta["Area"]
            id = meta["Id"]

            # --- draw bbox ---
            x1, y1 = int(x), int(y)
            x2, y2 = int(x + w), int(y + h)
            cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # --- draw label text ---
            text = f"{self.class_name}#{id} {score:.1f}%"
            cv2.putText(debug, text, (x1, max(0, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            self.get_logger().info(
                f"[Detected - Duck #{id}]"
                f"  Center position=(x={(x + w / 2.0):.1f}, y={(y + h / 2.0):.1f})"
                f"  bounding box=({x:.1f}, {y:.1f}, {x+w:.1f}, {y+h:.1f})"
                f"  size/area={area}" 
                f"  confidence={score:.2f}%"
            )

        if self.pub_dbg and debug is not None:
            self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))
            self.get_logger().info(
                f"[DEBUG IMG] Published annotated frame with {len(metadata)} detections to: {self.pub_dbg_name}"
            )

        

def main():
    rclpy.init()
    node = DetectorNode() 

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info('Detector interrupted, shutting down')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
