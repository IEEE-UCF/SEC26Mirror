# mcu_msgs

Shared message and service definitions for MCU-to-ROS2 communication. Used by both Teensy/ESP32 firmware (via micro-ROS) and ROS2 nodes on the Raspberry Pi.

**Source of truth:** `ros2_ws/src/mcu_msgs`
**Symlinked to:** `mcu_ws/extra_packages/mcu_msgs` (for micro-ROS code generation)

## Messages (20)

### Drive & Odometry

| Message | Description |
|---------|-------------|
| `DriveBase` | Full drive state & command: pose (TransformStamped), twist, drive_mode (VECTOR=0, GOAL=1, TRAJ=2), goal_transform, goal_velocity, goal_path |
| `DriveCommand` | Deprecated (use DriveBase) |

### Sensors

| Message | Description |
|---------|-------------|
| `BatteryHealth` | voltage (V), shunt_voltage (V), current (A), temperature, power (W), energy, charge_use |
| `RC` | 10-channel RC receiver values (-255 to 255) |
| `UWBRanging` | tag_id, UWBRange[] ranges, temperature (C), num_anchors |
| `UWBRange` | tag_id, anchor_id, distance (cm), signal_strength, timestamps, valid, error_code |
| `UWBAnchorInfo` | Anchor metadata |

### State

| Message | Description |
|---------|-------------|
| `McuState` | MCU lifecycle: PRE_INIT=0, INIT=1, INIT_FAIL=2, INIT_SUCCESS=3, ARMED=4, RUNNING=5, STOPPED=6, RESET=7 |
| `MiniRobotState` | State (9 values: PRE_INIT through EMERGENCY_STOP), current_task (7 values) |
| `IntakeState` | Intake mechanism state, motor speed |
| `IntakeCommand` | rail_command, intake_speed (-1.0 to 1.0) |
| `RobotInputs` | Aggregated button/DIP switch inputs |

### Actuators

| Message | Description |
|---------|-------------|
| `ArmCommand` | Arm servo joint commands |
| `ArmSusbsytem` | Arm subsystem state |
| `LedColor` | RGB LED color: r, g, b (0-255) |
| `IRCommand` | IR NEC transmission: address, command |
| `AntennaMarker` | Antenna LED position + color |

### Drone

| Message | Description |
|---------|-------------|
| `DroneState` | state (8 values), current_task, altitude, pos_x, pos_y, roll, pitch, yaw |
| `DroneControl` | Drone motor/param commands |
| `MiniRobotControl` | Minibot control commands |

## Services (18)

### Core MCU

| Service | Request | Response | Description |
|---------|---------|----------|-------------|
| `Reset` | (empty) | `bool success` | Reset all subsystems |
| `SetServo` | `uint8 index, float32 angle` | `bool success` | Set PCA9685 servo (0-180 deg) |
| `SetMotor` | `uint8 index, float32 speed` | `bool success` | Set PCA9685 motor (-1.0 to 1.0) |
| `ArmControl` | (arm params) | (response) | Move arm to position |
| `LCDAppend` | `string text` | `bool accepted` | Append text to OLED |
| `OLEDControl` | (params) | (response) | Control OLED display |
| `SetCameraAngle` | (params) | (response) | Pan/tilt camera servo |
| `SetDriveConfig` | (params) | (response) | Configure drive parameters |

### UWB

| Service | Request | Response | Description |
|---------|---------|----------|-------------|
| `UWBCalibrationStatus` | (empty) | state, positions | Query beacon calibration |

### Drone

| Service | Request | Response | Description |
|---------|---------|----------|-------------|
| `DroneArm` | `bool arm` | `success, message` | Arm/disarm motors |
| `DroneTakeoff` | `float32 target_altitude` | `success, message` | Initiate takeoff |
| `DroneLand` | (empty) | `success, message` | Land |
| `DroneTare` | (empty) | `success, message` | Zero IMU reference |
| `DroneSetAnchors` | `ids, positions` | `success` | Configure UWB anchors |
| `DroneSetMotors` | `float32[4] speeds` | `success, message` | Direct motor override |
| `DroneTransmitIR` | `uint8[4] colors` | `success` | Send IR to antennas |
| `DroneSetParam` | `string name, float32 value` | `success, message` | Runtime PID tuning |

## Topic Namespace Reference

All robot firmware topics use `/mcu_robot/` prefix:

```
/mcu_robot/battery_health      BatteryHealth        0.5 Hz
/mcu_robot/imu/data            sensor_msgs/Imu      100 Hz
/mcu_robot/rc                  RC                   20 Hz
/mcu_robot/tof_distances       Float32MultiArray    2 Hz
/mcu_robot/servo/state         Float32MultiArray    5 Hz
/mcu_robot/motor/state         Float32MultiArray    5 Hz
/mcu_robot/encoders            Float32MultiArray    20 Hz
/mcu_robot/dip_switches        UInt8                0.5 Hz
/mcu_robot/buttons             UInt8                10 Hz
/mcu_robot/crank/state         UInt8                5 Hz
/mcu_robot/crank/command       UInt8 (sub)
/mcu_robot/keypad/state        UInt8                5 Hz
/mcu_robot/keypad/command      UInt8 (sub)
/mcu_robot/intake/state        IntakeState          20 Hz
/mcu_robot/intake/command      IntakeCommand (sub)
/mcu_robot/led/set_all         LedColor (sub)
/mcu_robot/lcd/append          String (sub)
/mcu_robot/lcd/scroll          Int8 (sub)
/mcu_robot/deploy/trigger      String               event
/mcu_robot/deploy/status       String (sub)
/mcu_robot/uwb/ranging         UWBRanging           5 Hz
/mcu_robot/heartbeat           String               1 Hz
/mcu_robot/debug               String               on-demand
/mcu_robot/servo/set           SetServo (service)
/mcu_robot/motor/set           SetMotor (service)
/mcu_robot/imu/tare            Reset (service)
/mcu_robot/reset               Reset (service)
drive_base/status               DriveBase            50 Hz
drive_base/command              DriveBase (sub)
drive_base/reset_pose           Pose (sub)
```

Minibot: `/mcu_minibot/{state, heartbeat, debug, enter_crater, exit_crater}`
Drone: `/mcu_drone/{state, heartbeat, debug, uwb/ranging, cmd_vel, arm, takeoff, land, ...}`
Beacons: `mcu_uwb/range_{a}_{b}`, `/mcu_robot/heartbeat`

---

## OLED Display — Serial Terminal Interface

The SSD1306 128x64 OLED on the robot behaves like a serial terminal.
Text is appended to a 128-line ring buffer; the display shows 8 lines at a time.

### Interface

| Name | Type | Description |
|------|------|-------------|
| `/mcu_robot/lcd/append` | topic `std_msgs/String` | Append text; `\n` splits into lines |
| `/mcu_robot/lcd/scroll` | topic `std_msgs/Int8` | `-1` = scroll up (older), `+1` = scroll down (newer) |

### Limits

| Parameter | Value | Notes |
|-----------|-------|-------|
| Lines visible | 8 | 64 px / 8 px per row (text size 1) |
| Max line len | 21 | 128 px / 6 px per char (truncated) |
| Ring depth | 128 | Oldest lines overwritten when full |

### OLED Examples

```bash
# Append text
ros2 topic pub --once /mcu_robot/lcd/append std_msgs/msg/String "data: 'Hello from ROS2'"

# Multi-line append
ros2 topic pub --once /mcu_robot/lcd/append std_msgs/msg/String \
  "data: 'A1:RED A2:BLU\nA3:GRN A4:PUR'"

# Scroll up (older lines)
ros2 topic pub --once /mcu_robot/lcd/scroll std_msgs/msg/Int8 "data: -1"

# Scroll down (newer lines)
ros2 topic pub --once /mcu_robot/lcd/scroll std_msgs/msg/Int8 "data: 1"

# Jump back to live view
ros2 topic pub --once /mcu_robot/lcd/scroll std_msgs/msg/Int8 "data: 127"
```

---

## Building

After modifying `.msg` or `.srv` files:

```bash
# ROS2 side
cd /home/ubuntu/ros2_workspaces
colcon build --packages-select mcu_msgs
source install/setup.bash

# MCU side (clean micro-ROS to regenerate)
cd /home/ubuntu/mcu_workspaces/sec26mcu
pio run -e robot -t clean_microros && pio run -e robot
```

## Usage Examples

```bash
# Show message definition
ros2 interface show mcu_msgs/msg/DriveBase
ros2 interface show mcu_msgs/srv/SetServo

# Monitor battery
ros2 topic echo /mcu_robot/battery_health

# Set servo
ros2 service call /mcu_robot/servo/set mcu_msgs/srv/SetServo "{index: 0, angle: 90.0}"

# Set motor
ros2 service call /mcu_robot/motor/set mcu_msgs/srv/SetMotor "{index: 0, speed: 0.5}"

# Reset MCU subsystems
ros2 service call /mcu_robot/reset mcu_msgs/srv/Reset

# Set LEDs to red
ros2 topic pub --once /mcu_robot/led/set_all mcu_msgs/msg/LedColor "{r: 255, g: 0, b: 0}"

# Send drive command (velocity mode)
ros2 topic pub --once drive_base/command mcu_msgs/msg/DriveBase \
  "{drive_mode: 0, goal_velocity: {linear: {x: 0.3}, angular: {z: 0.0}}}"
```
