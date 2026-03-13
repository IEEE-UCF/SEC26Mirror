# secbot_fusion

Sensor fusion: converts MCU DriveBase odometry to standard ROS2 Odometry messages and optionally runs an EKF for state estimation.

## Nodes

### fusion_node (C++)

Pass-through converter from `mcu_msgs/DriveBase` to `nav_msgs/Odometry`.

### ekf_node (robot_localization, external)

Extended Kalman Filter for filtered state estimation. Configured via `ekf.yaml`.

## ROS2 Interface

### fusion_node

#### Publishers

| Topic | Message Type | QoS | Rate | Description |
|-------|-------------|-----|------|-------------|
| `/odom/unfiltered` | `nav_msgs/Odometry` | 10 | ~50 Hz | Raw drive_base conversion |

#### Subscribers

| Topic | Message Type | QoS | Description |
|-------|-------------|-----|-------------|
| `/drive_base/status` | `mcu_msgs/DriveBase` | 10 | Raw odometry from Teensy |

### ekf_node

#### Publishers

| Topic | Message Type | Rate | Description |
|-------|-------------|------|-------------|
| `/odom/filtered` | `nav_msgs/Odometry` | ~50 Hz | EKF-filtered odometry |

#### Subscribers

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/odom/unfiltered` | `nav_msgs/Odometry` | Unfiltered odom input from fusion_node |

### Message Conversion

```
DriveBase.transform    -> Odometry.pose (position + orientation)
DriveBase.twist        -> Odometry.twist (linear.x, angular.z)
Frame IDs: parent="odom", child="base_link"
```

Hardcoded covariances:
- Position (x, y, z): 0.1
- Velocity (vx, vy, vz): 0.01

## Config Files

| File | Purpose |
|------|---------|
| `config/ekf.yaml` | EKF parameters (inputs, covariances, frequency) |

## Launch Files

| File | Description |
|------|-------------|
| `fusion.launch.py` | Both fusion_node and ekf_node |

## Usage Examples

```bash
# Launch fusion pipeline
ros2 launch secbot_fusion fusion.launch.py

# Monitor filtered odometry
ros2 topic echo /odom/filtered

# Monitor raw (unfiltered) odometry
ros2 topic echo /odom/unfiltered

# Check rates
ros2 topic hz /odom/filtered
ros2 topic hz /odom/unfiltered

# View TF tree
ros2 run tf2_tools view_frames
```
