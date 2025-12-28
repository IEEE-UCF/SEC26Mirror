# secbot_uwb

Ultra-Wideband (UWB) positioning and ranging package for the SEC26 robot.

## Overview

This package provides UWB-based positioning for the SEC26 robot system using the DW3000 UWB modules. The system supports:

- **Stationary beacons** with known absolute positions (for field reference)
- **Moving beacons** with odometry fusion (robot-mounted beacon for enhanced 3D positioning)
- **Tags** that are tracked in 2D or 3D space (drone, mini robot, etc.)
- **Configurable axes** to specify which dimensions are known absolutely

The UWB system uses trilateration to calculate positions based on distance measurements between beacons and tags.

## System Architecture

### Hardware Setup

The SEC26 UWB system consists of 5 UWB modules:

| Device | Beacon/Tag ID | Type | MCU | Description |
|--------|---------------|------|-----|-------------|
| Stationary Beacon 1 | 10 | Beacon | ESP32 | Fixed reference point (origin) |
| Stationary Beacon 2 | 11 | Beacon | ESP32 | Fixed reference point (far corner) |
| Main Robot Beacon | 12 | Beacon (Moving) | Teensy 4.1 | Mounted on main robot |
| Drone | 20 | Tag | ESP32 | UWB tag for 3D tracking |
| Mini Robot | 21 | Tag | ESP32 | UWB tag for 2D tracking (optional) |

### Architecture Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    Field / Environment                      │
│                                                             │
│  ┌──────────┐                           ┌──────────┐       │
│  │ Beacon 10│                           │ Beacon 11│       │
│  │(ESP32)   │                           │(ESP32)   │       │
│  │Stationary│                           │Stationary│       │
│  └─────┬────┘                           └─────┬────┘       │
│        │                                      │             │
│        │          ┌──────────┐               │             │
│        │          │ Beacon 12│               │             │
│        │          │(Teensy)  │               │             │
│        │          │ Moving   │               │             │
│        │          │(on robot)│               │             │
│        │          └─────┬────┘               │             │
│        │                │                    │             │
│        └────UWB Range───┴────────UWB Range──┘             │
│                         │                                   │
│                  ┌──────▼───────┐                          │
│                  │  Tag 20      │                          │
│                  │  (Drone)     │                          │
│                  └──────────────┘                          │
└─────────────────────────────────────────────────────────────┘
                          │
                          │ UWB Ranging Data
                          │ (via micro-ROS)
                          ▼
            ┌─────────────────────────┐
            │   Raspberry Pi          │
            │                         │
            │  ┌───────────────────┐  │
            │  │ UWB Positioning   │  │
            │  │ Node (ROS2)       │  │
            │  │                   │  │
            │  │ - Trilateration   │  │
            │  │ - Odometry Fusion │  │
            │  │ - Position Pub    │  │
            │  └───────────────────┘  │
            │           │             │
            └───────────┼─────────────┘
                        │
            ┌───────────▼──────────────┐
            │  /uwb/pose/drone         │
            │  (PoseWithCovariance)    │
            └──────────────────────────┘
```

### Communication Flow

1. **MCU firmware** (beacons and tags) perform UWB ranging using the DW3000 library
2. **Beacon MCUs** publish ranging data to Raspberry Pi via micro-ROS
3. **Positioning node** on Raspberry Pi:
   - Receives ranging measurements from `/mcu_uwb/ranging`
   - Receives robot odometry from `/robot/pose` (for moving beacon)
   - Performs trilateration to calculate tag positions
   - Publishes estimated poses to `/uwb/pose/<tag_name>`
4. **Other ROS2 nodes** can subscribe to position estimates for navigation, tracking, etc.

## Configuration

### Beacon Configuration File

The system is configured via `config/beacons.yaml`. This file defines:

- **Beacons**: Stationary and moving reference points
- **Tags**: Devices to be tracked
- **Positioning parameters**: Algorithm tuning
- **Covariance settings**: Uncertainty estimation

#### Example Configuration

```yaml
uwb_positioning_node:
  ros__parameters:
    # Beacon configuration
    beacons:
      10:
        type: "stationary"
        position: {x: 0.0, y: 0.0, z: 0.15}
        known_axes: ["x", "y", "z"]
        description: "Stationary beacon at origin"

      11:
        type: "stationary"
        position: {x: 3.0, y: 3.0, z: 0.15}
        known_axes: ["x", "y", "z"]
        description: "Stationary beacon at far corner"

      12:
        type: "moving"
        position: {x: 0.0, y: 0.0, z: 0.20}
        known_axes: ["z"]  # Only Z is fixed (robot height)
        odometry_topic: "/robot/pose"
        use_odometry_fusion: true
        description: "Moving beacon on main robot"

    # Tag configuration
    tags:
      20:
        description: "Drone UWB tag"
        publish_topic: "/uwb/pose/drone"
        enable_3d_positioning: true

    # Positioning algorithm parameters
    positioning:
      min_beacons_2d: 3
      min_beacons_3d: 4
      max_residual: 0.5
      outlier_threshold: 2.0
```

### Configuration Parameters

#### Beacon Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `type` | string | `"stationary"` or `"moving"` |
| `position` | dict | Initial position `{x, y, z}` in meters |
| `known_axes` | list | Axes with known absolute values (e.g., `["x", "y", "z"]` or `["z"]`) |
| `odometry_topic` | string | Topic for moving beacon odometry (moving beacons only) |
| `use_odometry_fusion` | bool | Enable odometry fusion for moving beacons |
| `description` | string | Human-readable description |

#### Tag Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `description` | string | Human-readable description |
| `publish_topic` | string | Topic to publish estimated pose |
| `enable_3d_positioning` | bool | Enable 3D positioning (requires 4+ beacons) |

#### Positioning Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `min_beacons_2d` | 3 | Minimum beacons for 2D trilateration |
| `min_beacons_3d` | 4 | Minimum beacons for 3D trilateration |
| `max_residual` | 0.5 m | Maximum acceptable residual error |
| `outlier_threshold` | 2.0 m | Distance threshold for outlier rejection |

#### Covariance Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `base_xy_variance` | 0.05 m² | Base variance for X/Y position |
| `base_z_variance` | 0.10 m² | Base variance for Z position |
| `scale_with_residual` | true | Scale covariance with positioning residual |

## Usage

### Building

The package is part of the ROS2 workspace and is built with:

```bash
# Inside Docker container
cd /home/ubuntu/ros2_workspaces
source /opt/ros/jazzy/setup.bash
colcon build --packages-select secbot_uwb
source install/setup.bash
```

### Running

Launch the UWB positioning node:

```bash
# Using default configuration
ros2 launch secbot_uwb uwb_positioning.launch.py

# Using custom configuration
ros2 launch secbot_uwb uwb_positioning.launch.py config_file:=/path/to/custom.yaml
```

### Subscribing to Position Estimates

The positioning node publishes `geometry_msgs/PoseWithCovarianceStamped` messages:

```bash
# View drone position
ros2 topic echo /uwb/pose/drone

# View all UWB topics
ros2 topic list | grep uwb
```

### Example: Using in Python

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class DroneTracker(Node):
    def __init__(self):
        super().__init__('drone_tracker')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/uwb/pose/drone',
            self.position_callback,
            10
        )

    def position_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        self.get_logger().info(f'Drone at ({x:.2f}, {y:.2f}, {z:.2f})')

def main():
    rclpy.init()
    node = DroneTracker()
    rclpy.spin(node)
    rclpy.shutdown()
```

## Positioning Algorithms

### Trilateration

The positioning system uses **least-squares trilateration** to estimate tag positions from distance measurements.

Given:
- Beacon positions: **a₁**, **a₂**, ..., **aₙ**
- Measured distances: d₁, d₂, ..., dₙ

We solve for tag position **p** that minimizes:
```
Σ (||p - aᵢ||² - dᵢ²)²
```

The system linearizes this problem and solves using `np.linalg.lstsq`.

### 2D vs 3D Positioning

- **2D Positioning**: Requires 3+ beacons, estimates (x, y), sets z=0
- **3D Positioning**: Requires 4+ beacons, estimates (x, y, z)

With the moving beacon on the robot (beacon 12), the drone can achieve 3D positioning:
- 2 stationary beacons (known x, y, z)
- 1 moving beacon (known z, x/y from odometry)
- = 3 beacons with full position → enough for 3D positioning of the drone

### Known Axes

The `known_axes` parameter allows specifying which dimensions are fixed:

- **Stationary beacons**: `["x", "y", "z"]` - all axes known
- **Moving beacon on robot**: `["z"]` - only height is fixed
- **Future extensions**: Could support `["x", "y"]` for ground-based tags

### Odometry Fusion

For moving beacons with `use_odometry_fusion: true`:

1. Subscribe to odometry topic (e.g., `/robot/pose`)
2. Extract position from `PoseWithCovarianceStamped`
3. Update beacon position for unknown axes (x, y)
4. Keep known axes fixed (z = 0.20m for robot height)

This allows the robot-mounted beacon to serve as a reference point with accurate height and odometry-based x/y position, enabling 3D drone positioning.

**Future enhancement**: Implement Kalman filter fusion to combine UWB and odometry with proper uncertainty handling.

## ROS2 Interface

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/mcu_uwb/ranging` | `mcu_msgs/UWBRanging` | UWB range measurements from tags |
| `/robot/pose` (configurable) | `geometry_msgs/PoseWithCovarianceStamped` | Moving beacon odometry |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/uwb/pose/<tag_name>` | `geometry_msgs/PoseWithCovarianceStamped` | Estimated tag pose (configurable per tag) |

### Message Definitions

#### `mcu_msgs/UWBRanging`

```
std_msgs/Header header
uint8 tag_id              # ID of the tag device
UWBRange[] ranges         # Array of ranges to beacons
float32 temperature       # DW3000 chip temperature
uint8 num_anchors         # Number of anchors ranged
```

#### `mcu_msgs/UWBRange`

```
std_msgs/Header header
uint8 tag_id              # Tag ID
uint8 anchor_id           # Beacon/anchor ID
float32 distance          # Distance in cm
float32 signal_strength   # Signal quality
int32 clock_offset        # Clock offset
uint64 tx_timestamp       # TX timestamp
uint64 rx_timestamp       # RX timestamp
bool valid                # Valid measurement flag
uint8 error_code          # Error code (0 = no error)
```

## Troubleshooting

### Insufficient Beacons Warning

```
Tag 20: Insufficient beacons (2/3 min) for positioning
```

**Cause**: Not enough beacons are visible to the tag.

**Solutions**:
- Check UWB antenna connections
- Verify beacons are powered and running
- Reduce `min_beacons_2d` or `min_beacons_3d` in config (not recommended)
- Check for physical obstructions blocking UWB signals

### High Residual Error Warning

```
Tag 20: High residual error (0.85m), position may be inaccurate
```

**Cause**: Position solution doesn't fit the measurements well.

**Solutions**:
- Verify beacon positions in config are accurate
- Check for multipath interference (reflective surfaces)
- Increase `max_residual` threshold in config
- Improve beacon geometry (spread beacons farther apart)

### Moving Beacon Not Updating

**Cause**: Odometry topic not publishing or misconfigured.

**Solutions**:
- Verify odometry topic is publishing: `ros2 topic echo /robot/pose`
- Check `odometry_topic` in beacon configuration
- Ensure `use_odometry_fusion: true` is set

## Development

### Adding a New Tag

1. Edit `config/beacons.yaml`:
   ```yaml
   tags:
     21:
       description: "Mini robot UWB tag"
       publish_topic: "/uwb/pose/mini_robot"
       enable_3d_positioning: false
   ```

2. Flash MCU firmware with tag ID 21

3. Restart the positioning node

### Adding a New Beacon

1. Edit `config/beacons.yaml`:
   ```yaml
   beacons:
     13:
       type: "stationary"
       position: {x: 1.5, y: 1.5, z: 0.15}
       known_axes: ["x", "y", "z"]
       description: "Center beacon"
   ```

2. Flash MCU firmware with beacon ID 13

3. Restart the positioning node

### Testing

```bash
# Check if node is running
ros2 node list | grep uwb

# View node parameters
ros2 param list /uwb_positioning_node

# Monitor ranging data
ros2 topic echo /mcu_uwb/ranging

# Monitor position output
ros2 topic echo /uwb/pose/drone

# Check positioning frequency
ros2 topic hz /uwb/pose/drone
```

## Future Enhancements

- [ ] Implement Kalman filter for odometry fusion
- [ ] Add GDOP (Geometric Dilution of Precision) calculation
- [ ] Implement outlier rejection for noisy measurements
- [ ] Add diagnostics publishing (`/uwb/diagnostics`)
- [ ] Support dynamic beacon reconfiguration
- [ ] Add visualization tools (RViz markers)
- [ ] Implement time-synchronization monitoring
- [ ] Add support for different positioning algorithms (e.g., Extended Kalman Filter)

## References

- **DW3000 Library**: [https://github.com/Fhilb/DW3000_Arduino.git](https://github.com/Fhilb/DW3000_Arduino.git)
- **UWB Technology**: IEEE 802.15.4a/z standards
- **Trilateration Algorithm**: [Wikipedia - Trilateration](https://en.wikipedia.org/wiki/Trilateration)
- **ROS2 Jazzy**: [https://docs.ros.org/en/jazzy/](https://docs.ros.org/en/jazzy/)

## License

MIT License

## Authors

SEC26 Team - IEEE UCF SoutheastCon 2026 Hardware Competition
