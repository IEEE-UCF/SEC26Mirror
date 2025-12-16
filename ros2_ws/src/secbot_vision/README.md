# secbot_vision

This package handles the computer vision tasks for the SecBot, specifically detecting ducks (yellow objects) in the environment. It acts as the "eyes" of the robot, processing camera feeds and publishing detection metadata.

## Overview

The core component is the `detector_node`, which uses OpenCV to process images. It subscribes to a camera topic, filters for yellow objects (Mustard Bottles representing ducks) in HSV color space, and publishes both standard `vision_msgs/Detection2DArray` and custom `secbot_msgs/DuckDetections`.

## Nodes

### `detector_node`

Performs object detection using color thresholding and contour analysis.

#### Subscribed Topics
- **`/camera/image_raw`** (`sensor_msgs/Image`): Raw camera stream. Topic name configurable via `image_topic` parameter.

#### Published Topics
- **`/detected_objects`** (`vision_msgs/Detection2DArray`): Standard ROS 2 detection array containing bounding boxes and scores.
- **`/duck_detections`** (`secbot_msgs/DuckDetections`): Custom message containing a list of detected ducks with position (x, y), size (w, h), area, and confidence.
- **`/vision/debug_image`** (`sensor_msgs/Image`): Debug image with drawn bounding boxes and labels. Published only if `debug_viz` is true.

#### Parameters
- **`image_topic`** (string, default: `'/camera/image_raw'`): The topic to subscribe to for image data.
- **`class_name`** (string, default: `'yellow_object'`): Label to use for detections.
- **`pub_topic`** (string, default: `'/detected_objects'`): Topic name for standard detections.
- **`pub_debug_image`** (string, default: `'/vision/debug_image'`): Topic name for the debug visualization.
- **`debug_viz`** (bool, default: `True`): Whether to publish the debug image.
- **`color_name`** (string, default: `'yellow'`): Color profile to use from `colors.yaml`.
- **`target_hue`** (int, default: `25`): Center HSV hue for detection.
- **`hue_change`** (int, default: `20`): Acceptable deviation from target hue.
- **`kernel_size`** (int, default: `15`): Kernel size for morphological operations.
- **`min_area_ratio`** (double, default: `0.0002`): Minimum object area ratio to be considered a valid detection.

## Message Definitions

The package publishes messages defined in `secbot_msgs`.

### `secbot_msgs/DuckDetections`
The top-level message published to `/duck_detections`.

| Field | Type | Description |
| :--- | :--- | :--- |
| `header` | `std_msgs/Header` | Standard ROS header with timestamp and frame ID (usually same as camera frame). |
| `detections` | `secbot_msgs/DuckDetection[]` | List of individual detected objects. |

### `secbot_msgs/DuckDetection`
Describes a single detected object.

| Field | Type | Description |
| :--- | :--- | :--- |
| `id` | `int32` | Tracking ID for the object (resets per frame currently). |
| `label` | `string` | Class label (e.g., "yellow_object"). |
| `x`, `y` | `float32` | Top-left corner coordinates of the bounding box (pixels). |
| `w`, `h` | `float32` | Width and height of the bounding box (pixels). |
| `center_x`, `center_y` | `float32` | Center coordinates of the object (pixels). |
| `area` | `float32` | Area of the detected contour (pixels²). |
| `confidence` | `float32` | Detection confidence score (0-100%). |

## Usage Example (Python)

Here is a simple example of how to subscribe to the detections in Python:

```python
import rclpy
from rclpy.node import Node
from secbot_msgs.msg import DuckDetections

class DuckSubscriber(Node):
    def __init__(self):
        super().__init__('duck_subscriber')
        self.subscription = self.create_subscription(
            DuckDetections,
            '/duck_detections',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received {len(msg.detections)} ducks')
        for duck in msg.detections:
            self.get_logger().info(
                f'  Duck #{duck.id}: Center({duck.center_x:.1f}, {duck.center_y:.1f}) Area={duck.area:.0f}'
            )

def main(args=None):
    rclpy.init(args=args)
    duck_subscriber = DuckSubscriber()
    rclpy.spin(duck_subscriber)
    duck_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Launch Files

### `robot_sim.launch.py`
Launches the full simulation environment locally with the detector node.

**Functionality:**
1. Spawns the Gazebo Harmonic simulation with `my_bot_harmonic.sdf`.
2. Starts the `ros_gz_bridge` to bridge Gazebo topics to ROS 2.
3. Starts the `detector_node` with parameters loaded from `config/vision.yaml`.

**Usage:**
```bash
ros2 launch secbot_vision robot_sim.launch.py
```

## Configuration

- **`config/vision.yaml`**: Contains default parameter overrides for the detector node.
- **`config/colors.yaml`**: Defines HSV ranges for different colors (e.g., yellow, red, green).
- **`config/ros_gz_bridge.yaml`**: Configures the mapping between Gazebo topics and ROS 2 topics.

## Testing

A test script is provided to verify the setup:

```bash
ros2 run secbot_vision test_duck_detector.sh
```
Or directly from the scripts directory:
```bash
./src/secbot_vision/scripts/test_duck_detector.sh
```

This script:
1. Builds the package.
2. Launches `robot_sim.launch.py`.
3. Spawns test objects (Mustard Bottles) in the simulation.
4. Opens image viewers for the raw feed and debug output.
