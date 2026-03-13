# secbot_vision

Camera input and HSV-based object detection for rubber ducks (yellow objects) and antenna LEDs. Acts as the "eyes" of the robot, processing camera feeds and publishing detection metadata.

## Nodes

### `camera_node` (Python)

Captures video frames and publishes to ROS2. Supports multiple camera backends.

#### Published Topics

| Topic | Message Type | QoS | Rate | Description |
|-------|-------------|-----|------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | 10 | 30 Hz | Raw video frames |

#### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `image_topic` | string | `/camera/image_raw` | Output topic |
| `frame_rate` | double | 30.0 | FPS |
| `width` | int | 640 | Frame width |
| `height` | int | 480 | Frame height |
| `device_id` | int | 2 | Video device for OpenCV backend |
| `backend` | string | `auto` | `auto`, `picamera2`, `tcp`, `opencv` |
| `tcp_host` | string | `192.168.4.1` | TCP stream host |
| `tcp_port` | int | 8554 | TCP stream port |

### `detector_node` (Python)

Performs object detection using HSV color thresholding and contour analysis. Detects both ducks and antenna LEDs.

#### Subscribed Topics

| Topic | Message Type | QoS | Description |
|-------|-------------|-----|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | BEST_EFFORT | Raw camera stream (topic configurable) |

#### Published Topics

| Topic | Message Type | QoS | Description |
|-------|-------------|-----|-------------|
| `/detected_objects` | `vision_msgs/Detection2DArray` | 10 | Standard ROS2 detection array |
| `/duck_detections` | `secbot_msgs/DuckDetections` | 10 | Custom duck detections with position, area, confidence |
| `/antenna_detections` | `secbot_msgs/AntennaDetections` | 10 | Antenna LED colors and bounding boxes |
| `/vision/debug_image` | `sensor_msgs/Image` | 10 | Annotated image with bounding boxes (if `debug_viz` is true) |
| `/vision/mask` | `sensor_msgs/Image` | 10 | HSV mask visualization |

#### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `image_topic` | string | `/camera/image_raw` | Input image topic |
| `class_name` | string | `yellow_object` | Label for detections |
| `pub_topic` | string | `/detected_objects` | Standard detections topic |
| `pub_debug_image` | string | `/vision/debug_image` | Debug visualization topic |
| `pub_mask_image` | string | `/vision/mask` | Mask output topic |
| `debug_viz` | bool | true | Enable debug visualization |
| `color_name` | string | `yellow` | Color profile from `colors.yaml` |
| `target_hue` | int | 25 | Center HSV hue |
| `hue_change` | int | 20 | Acceptable hue deviation |
| `kernel_size` | int | 15 | Morphological kernel size |
| `min_area_ratio` | double | 0.0002 | Min detection area ratio |

## Quick ROS2 Testing

```bash
# Launch full vision pipeline (TCP camera backend)
ros2 launch secbot_vision vision.launch.py backend:=tcp

# Launch with USB camera
ros2 launch secbot_vision vision.launch.py backend:=opencv device_id:=0

# Run nodes standalone
ros2 run secbot_vision camera_node --ros-args -p backend:=opencv -p device_id:=0
ros2 run secbot_vision detector_node

# Monitor detections
ros2 topic echo /duck_detections
ros2 topic echo /antenna_detections
ros2 topic hz /duck_detections

# View debug image (requires display)
ros2 run image_tools showimage --ros-args -r image:=/vision/debug_image

# View HSV mask
ros2 run image_tools showimage --ros-args -r image:=/vision/mask
```

## Detection Algorithm

1. Convert BGR to HSV
2. Threshold by hue range (from `colors.yaml`)
3. Morphological close (fill holes)
4. Contour detection
5. Score = 0.9 x hue_accuracy + 0.1 x saturation_brightness
6. Publish bounding boxes with confidence

## Message Definitions

The package publishes messages defined in `secbot_msgs`.

### `secbot_msgs/DuckDetections`

The top-level message published to `/duck_detections`.

| Field        | Type                          | Description                                                                     |
| :----------- | :---------------------------- | :------------------------------------------------------------------------------ |
| `header`     | `std_msgs/Header`             | Standard ROS header with timestamp and frame ID (usually same as camera frame). |
| `detections` | `secbot_msgs/DuckDetection[]` | List of individual detected objects.                                            |

### `secbot_msgs/DuckDetection`

Describes a single detected object.

| Field                  | Type      | Description                                               |
| :--------------------- | :-------- | :-------------------------------------------------------- |
| `id`                   | `int32`   | Tracking ID for the object (resets per frame currently).  |
| `label`                | `string`  | Class label (e.g., "yellow_object").                      |
| `x`, `y`               | `float32` | Top-left corner coordinates of the bounding box (pixels). |
| `w`, `h`               | `float32` | Width and height of the bounding box (pixels).            |
| `center_x`, `center_y` | `float32` | Center coordinates of the object (pixels).                |
| `area`                 | `float32` | Area of the detected contour (pixels²).                   |
| `confidence`           | `float32` | Detection confidence score (0-100%).                      |

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

### `field_sim.launch.py`

Launches the custom field environment with the standalone robot.

**Functionality:**

1. Loads the `field.world` which uses the custom STL environment.
2. Spawns `my_bot` (standalone SDF) into the field.
3. Starts the bridge and detector node.

**Usage:**

```bash
ros2 launch secbot_vision field_sim.launch.py
```

## Configuration

- **`config/vision.yaml`**: Contains default parameter overrides for the detector node.
- **`config/colors.yaml`**: Defines HSV ranges for different colors (e.g., yellow, red, green).
- **`config/ros_gz_bridge.yaml`**: Configures the mapping between Gazebo topics and ROS 2 topics.

## Simulation Worlds

The package supports multiple simulation environments located in `src/secbot_vision/worlds/`:

### Default World (`worlds/default/`)

Contains the standard testing ground with simple geometry and the original `my_bot_harmonic.sdf` model.

### Field World (`worlds/field/`)

Contains a custom environment based on `robot_field.stl`.

- **Scale**: The STL is scaled by `0.001` to convert millimeters to meters.
- **Robot**: Uses a standalone 1ft³ (0.3048m) version of the robot spawned at runtime.

### Proper Field World (`worlds/proper_field/`)

The primary testing environment using the full field model (`proper_field.obj`).

- **Configurable Spawns**: Uses `config/spawn_locations.yaml` for robot and item positions.
- **Launch**: `proper_field_sim.launch.py`

## Testing

A test script is provided to verify the setup:

```bash
ros2 run secbot_vision test_duck_detector.sh
```

Or directly from the scripts directory:

```bash
src/sec26ros/secbot_vision/scripts/test_duck_detector.sh
```

This script:

1. Builds the package.
2. Launches `robot_sim.launch.py`.
3. Spawns test objects (Mustard Bottles) in the simulation.
4. Opens image viewers for the raw feed and debug output.

### Proper Field Test -> use this one for better testing

To test the full `proper_field` environment with configurable spawns:

```bash
src/sec26ros/secbot_vision/scripts/test_proper_field.sh
```

### Field World Test

To test the custom field environment:

```bash
ros2 run secbot_vision test_field_world.sh
```

This script:

1. Builds the package.
2. Launches `field_sim.launch.py`.
3. Spawns the robot (1ft³) and a test mustard bottle.
4. Sets up the necessary resource paths for Gazebo to find the field mesh.
