#!/usr/bin/env python3

# MAKE SURE YOU HAVE ALL INFO INSTALLED:
# source /opt/ros/jazzy/setup.bash
# colcon build --symlink-install --packages-select secbot_vision
# source install/local_setup.bash
# ros2 pkg executables secbot_vision


# TO RUN: 
# 1) IMPORT MODEL (Mustard bottle):
# ros2 run ros_gz_sim create   -world default   -name mustard_bottle   -file 'ros2_workspaces/src/sec26ros/src/secbot_vision/worlds/model.sdf'   -x 0.7 -y 0.0 -z 0.15 -Y 0.0

# 2) RUN GAZEBO (SEPERATE TERMINAL):
# ros2 launch secbot_vision robot_sim.launch.py image_topic:=/camera2/image_raw

# 3) SEE WHAT ROBOT SEES (OPTIONAL)
# ros2 run image_tools showimage --ros-args -r image:=/camera2/image_raw
# ros2 run image_tools showimage --ros-args -r image:=/vision/debug_image


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2, os, yaml, numpy as np
from ament_index_python.packages import get_package_share_directory
from secbot_msgs.msg import DuckDetection, DuckDetections, AntennaDetection, AntennaDetections

class DetectorNode(Node):
    """DetectorNode class"""
    
    def _isDuck(self):
        yellow_detection_duck = self.get_parameter('yellow_detection_duck').value
        self.filters_array = [yellow_detection_duck]
        
    def _isAntenna(self):
        red_antenna = self.get_parameter('red_antenna').value
        green_antenna = self.get_parameter('green_antenna').value
        blue_antenna = self.get_parameter('blue_antenna').value
        purple_antenna = self.get_parameter('purple_antenna').value
            
        self.filters_array = [red_antenna,green_antenna,blue_antenna,purple_antenna]
        
    
    def __init__(self, duck_or_antenna):
        """initalize all parameters, logging info and inputs and outputs"""
        
        super().__init__('detector_node')
        self.duck_or_antenna = duck_or_antenna
        # Parameters
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('class_name', 'yellow_object')
        self.declare_parameter('pub_topic', '/detected_objects')
        self.declare_parameter('pub_debug_image', '/vision/debug_image')
        self.declare_parameter('pub_mask_image', '/vision/mask')
        self.declare_parameter('debug_viz', True)
        self.declare_parameter('yellow_detection_duck', 'yellow')
        self.declare_parameter('red_antenna', 'red')
        self.declare_parameter('green_antenna', 'green')
        self.declare_parameter('blue_antenna', 'blue')
        self.declare_parameter('purple_antenna', 'purple')
        self.declare_parameter('min_area_ratio', 0.0002)

        # read parameters
        self.class_name = self.get_parameter('class_name').value
        image_topic     = self.get_parameter('image_topic').value
        self.pub_topic  = self.get_parameter('pub_topic').value
        self.debug_viz  = self.get_parameter('debug_viz').value
        self.pub_dbg_name = self.get_parameter('pub_debug_image').value
        self.pub_mask_name = self.get_parameter('pub_mask_image').value

        # ROS Inputs and outputs
        qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.bridge  = CvBridge()
        self.sub = self.create_subscription(Image, image_topic, self.on_image, qos)
        self.pub_dbg = self.create_publisher(Image, self.pub_dbg_name, 10) if self.debug_viz else None
        self.pub_mask = self.create_publisher(Image, self.pub_mask_name, 10)
        
        
        self.pub_ducks = self.create_publisher(
            DuckDetections,     # Type of Message it can send
            '/duck_detections', # Name
            10                  # Queue Size
        )
        
        self.pub_ant = self.create_publisher(
            AntennaDetections,
            '/antenna_detections',
            10
        )

        # Logging info
        self.get_logger().info(f"Using: {image_topic}")
        self.get_logger().info(f"Detections: {self.pub_topic}")
        if self.debug_viz:
            self.get_logger().info(f"Debug image: {self.pub_dbg_name}")
            self.get_logger().info(f"Mask image: {self.pub_mask_name}")
        pkg_share = get_package_share_directory('secbot_vision')
        self.colors_path = os.path.join(pkg_share, 'config', 'colors.yaml')

        with open(self.colors_path, 'r') as f:
            colors_cfg = yaml.safe_load(f)
        
        
        
        # DUCK STATE IS 1, ANTENNA STATE IS 0
        self._isDuck() if duck_or_antenna else self._isAntenna()
        
        
        self.color_configs_for_objects = []
        for filter_name in self.filters_array:
            color_cfg = colors_cfg['colors'][filter_name]
            self.color_configs_for_objects.append({'filter': filter_name,
                                                   'color_low': np.array(color_cfg['color_low'], dtype=np.uint8), 
                                                   'color_high': np.array(color_cfg['color_high'], dtype=np.uint8),
                                                   'hsv_value': np.array(color_cfg['hsv_values'], dtype=np.uint8)})
        
        self.min_area_ratio = float(self.get_parameter('min_area_ratio').value)


    def on_image(self, msg: Image):
        """Call function and read the metadata/debuging details"""
        # Use OpenCV in ROS2
        
        
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        debug = frame.copy()
        best_mask = np.zeros(frame.shape[:2], dtype=np.unit8)
        
        # do it for all items in the array
        # detector
        
        best_filter_name = None
        best_filter_score = 0.0
        all_metadata = []
        
        for filter_cfgs in self.color_configs_for_objects:            
            metadata, debug_out, mask_out = process_frame(frame,
                                            color_low=filter_cfgs['color_low'],
                                            color_high=filter_cfgs['color_high'],
                                            hsv_values=filter_cfgs['hsv_value'],
                                            min_area_ratio=self.min_area_ratio)
            
            
            if not metadata: continue
            for meta in metadata:
                x, y = meta["Position"]
                w, h = meta["Size"]
                score = meta["Score"]
                det_id = meta["Id"]

                # --- draw bbox ---
                x1, y1 = int(x), int(y)
                x2, y2 = int(x + w), int(y + h)
                cv2.rectangle(debug_out, (x1, y1), (x2, y2), (0, 255, 0), 2)

                # --- draw label text ---
                text = f"{filter_cfgs['filter']}#{det_id} {score:.1f}%"
                cv2.putText(debug_out, text, (x1, max(0, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            filter_best = max(meta["Score"] for meta in metadata)
            if filter_best > best_filter_score:
                best_filter_score = filter_best
                best_filter_name = filter_cfgs
                debug = debug_out
                best_mask = mask_out
                all_metadata = metadata
            

        if self.pub_dbg and debug is not None:
            self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))
            self.get_logger().info(f"[DEBUG IMG] {len(all_metadata)} detections")
            self.get_logger().info(f"[DEBUG IMG] filter: {best_filter_name['filter'] if best_filter_name else 'none'}")
            self.get_logger().info(f"[DEBUG IMG] INFO SENT -> {self.pub_dbg_name}")
            
        self.pub_mask.publish(self.bridge.cv2_to_imgmsg(best_mask, encoding='bgr8'))
        
        self._publish_duck_detections(msg, all_metadata) if self.duck_or_antenna else self._publish_antenna_detections(msg, all_metadata, best_filter_name)
    

    def _publish_duck_detections(self, msg, metadata):
        # Create and Publish Custom Duck Detections
        duck_msg = DuckDetections()
        duck_msg.header = msg.header # re-use the image header

        for meta in metadata:
            d = DuckDetection()

            d.x, d.y = float(meta["Position"][0]), float(meta["Position"][1])
            d.w, d.h = float(meta["Size"][0]), float(meta["Size"][1])
            d.confidence = float(meta["Score"])
            d.area = float(meta["Area"])
            d.id = int(meta["Id"])

            d.center_x = float(d.x + d.w / 2.0)
            d.center_y = float(d.y + d.h / 2.0)

            duck_msg.detections.append(d)
        
        self.pub_ducks.publish(duck_msg)
    
    def _publish_antenna_detections(self, msg, metadata, best_filter_name):
        # Create and Publish Custom Duck Detections
        antenna_msg = AntennaDetections()
        antenna_msg.header = msg.header # re-use the image header

        for meta in metadata:
            d = AntennaDetection()

            d.x, d.y = float(meta["Position"][0]), float(meta["Position"][1])
            d.w, d.h = float(meta["Size"][0]), float(meta["Size"][1])
            d.confidence = float(meta["Score"])
            d.area = float(meta["Area"])
            d.id = int(meta["Id"])

            d.center_x = float(d.x + d.w / 2.0)
            d.center_y = float(d.y + d.h / 2.0)

            d.color = best_filter_name['filter'] if best_filter_name else ''
            antenna_msg.detections.append(d)
        
        self.pub_ant.publish(antenna_msg)

def process_frame(frame, color_low, color_high, hsv_value, min_area_ratio=0.0002):
    """Return metadata array of an image:
      Center position(x,y)
      SIZE(w,h)
      LABEL(confidence)
      debugging"""
      
    hue, saturation, value = hsv_value
    
    MIN_AREA = int(min_area_ratio * frame.shape[0] * frame.shape[1])
    # MIN_AREA = 600

    value = int(value)
    if value % 2 == 0:
        value += 1  # keep it odd-ish
    KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (value, value))

    # convert to hue saturation value
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # create mask for yellow objects
    mask = cv2.inRange(hsv, color_low, color_high)

    # clean up specks & fill small holes
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,  KERNEL)

    # see the mask in color
    result = cv2.bitwise_and(frame, frame, mask=mask)

    # find the contours(object's boundaries) of the mask
    contours, __ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    H = hsv[:,:,0]
    S = hsv[:,:,1]
    V = hsv[:,:,2]
    debug = frame.copy()
    metadata  = []

    for contour in contours:
        area = cv2.contourArea(contour)
        x, y, w, h = cv2.boundingRect(contour)

        if area < MIN_AREA and (w < 50 or h < 50):
            continue

        # create a mask for the object itself
        roi_mask = mask[y:y+h, x:x+w] == 255
        roi_H    = H[y:y+h, x:x+w]
        roi_S    = S[y:y+h, x:x+w]
        roi_V    = V[y:y+h, x:x+w]

        # --- Fill Ratio ---
        # N_mask / N_bbox
        # N_mask = number of pixels with mask value 255 inside box
        # N_bbox = w * h
        n_mask = int(np.sum(roi_mask))
        n_bbox = w * h
        fill_ratio = n_mask / n_bbox

        yy = roi_H[roi_mask>0]
        if yy.size > 0:

            # --- Hue Accuracy ---

            # compute circular hue distance to target hue H_t
            # d_i = min(|H_i - H_t|, 180 - |H_i - H_t|)
            d = abs(yy.astype(np.int16) - hue)
            d_i = np.minimum(d, 180 - d).astype(np.float32)
            # convert to [0-1] match score
            # hue_i = max(0, 1 - d_i / H_CHANGE)          H_CHANGE = maxium acceptable hue devation
            hue_i = np.maximum(0.0, 1.0 - (d_i / saturation)).mean()

            # Average over all mask pixels
            # hue_acc = 1 / (n_mask) * sum(hue_i)
            # hue_acc = (1 / n_mask) * hue_i

            # --- Saturation and brightness ---
            sat = roi_S[roi_mask>0].astype(np.float32) / 255
            bri = roi_V[roi_mask>0].astype(np.float32) / 255
            sv_boost = ((sat + bri)/2).mean()

            # --- Confidence ---

            confidence = ((0.0 * fill_ratio) + (0.9 * hue_i) + (0.1 * sv_boost)) * 100

            # Append the metadata
            metadata.append({
                    "Position": (float(x), float(y)),
                    "Size": (w, h),
                    "Score": float(confidence),
                    "Area": float(area),
                })

    metadata.sort(key=lambda d: d["Position"][0])   # x
    for i, d in enumerate(metadata):
        d["Id"] = i

    return metadata, debug, mask


def main():
    rclpy.init()
    import sys
    mode = not (len(sys.argv) > 1 and sys.argv[1].lower() == 'antenna')
    node = DetectorNode(duck_or_antenna=mode) 

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
