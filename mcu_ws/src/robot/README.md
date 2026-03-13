# Robot Firmware

Main firmware for the SEC26 competition robot running on Teensy 4.1. Manages 20+ subsystems via TeensyThreads with micro-ROS for ROS2 communication.

## Hardware

- **MCU:** Teensy 4.1 (600 MHz ARM Cortex-M7)
- **Transport:** USB Serial micro-ROS to Raspberry Pi
- **Baud:** 921600

### I2C Bus Layout

```
Wire0 (pins 18/19): TCA9548A mux (0x70), TCA9555 GPIO (0x20), INA219 power (0x40 via mux ch0)
Wire1 (pins 17/16): BNO085 IMU (0x4B), INT=41, RST=40
Wire2 (pins 24/25): PCA9685 #0 servos (0x40, OE=28), PCA9685 #1 motors (0x41, OE=29)
```

### Key Pin Assignments

| Function | Pins | Notes |
|----------|------|-------|
| Motor FG encoders | 2–9 | QTimer3/4 pulse inputs (pin 8 disabled) |
| UWB SPI | CS=10, MOSI=11, MISO=12, CLK=13 | DW3000 tag |
| OLED SPI1 | MOSI=26, SCK=27, CS=38, DC=37, RST=33 | Hardware SPI1 |
| RC receiver | 34 (Serial8) | IBusBM NOTIMER |
| WS2812B LEDs | 35 | 5 addressable RGB LEDs |
| Button INT | 36 | TCA9555 interrupt |
| PCA9685 OE | Servo=28, Motor=29 | Output enable |
| Mux Reset | 23 | TCA9548A |

See `RobotPins.h` for full assignments.

## Building and Flashing

All commands run inside the Docker container.

```bash
cd /home/ubuntu/mcu_workspaces/sec26mcu

# Build
pio run -e robot

# Flash
pio run -e robot --target upload
# or automated:
/home/ubuntu/scripts/flash_mcu.sh

# Serial monitor
pio device monitor -e robot

# Clean micro-ROS (after changing mcu_msgs)
pio run -e robot -t clean_microros && pio run -e robot
```

## DIP Switch Configuration

| DIP | Bit | Name | ON | OFF |
|-----|-----|------|----|-----|
| 1 | 0 | ROS2_ENABLE | ROS2 joystick control | RC manual control |
| 2 | 1 | UWB_ENABLE | DW3000 tag active | UWB disabled |
| 3 | 2 | VISION_ENABLE | Duck detection | Vision disabled |
| 4 | 3 | SPEED_PROFILE | Half speed | Full speed |
| 5 | 4 | (unused) | — | — |
| 6 | 5 | OLED_DEBUG | Debug dashboard | Terminal scroll |
| 7 | 6 | FORCE_REFLASH | Force OTA reflash | Normal OTA |
| 8 | 7 | DEPLOY_TARGET | Target: `robot` | Target: `teensy-test-all-subsystems` |

## ROS2 Interface

### Publishers

| Topic | Message Type | Rate | Description |
|-------|-------------|------|-------------|
| `/mcu_robot/battery_health` | `mcu_msgs/BatteryHealth` | 0.5 Hz | Voltage (V), current (A), power (W) |
| `/mcu_robot/imu/data` | `sensor_msgs/Imu` | 100 Hz | Quaternion, angular velocity, acceleration |
| `/mcu_robot/rc` | `mcu_msgs/RC` | 20 Hz | 10 RC channel values (-255 to 255) |
| `/mcu_robot/tof_distances` | `std_msgs/Float32MultiArray` | 2 Hz | TOF distance array (meters) |
| `/mcu_robot/servo/state` | `std_msgs/Float32MultiArray` | 5 Hz | 8 servo angles (degrees) |
| `/mcu_robot/motor/state` | `std_msgs/Float32MultiArray` | 5 Hz | 8 motor speeds (-1.0 to 1.0) |
| `/mcu_robot/encoders` | `std_msgs/Float32MultiArray` | 20 Hz | 8 encoder tick rates (ticks/sec) |
| `/mcu_robot/dip_switches` | `std_msgs/UInt8` | 0.5 Hz | 8-bit DIP switch bitmask |
| `/mcu_robot/buttons` | `std_msgs/UInt8` | 10 Hz | 8-bit button bitmask (1=pressed) |
| `/mcu_robot/crank/state` | `std_msgs/UInt8` | 5 Hz | 0=IDLE, 1=CRANKING, 2=RESETTING |
| `/mcu_robot/keypad/state` | `std_msgs/UInt8` | 5 Hz | 0=IDLE, 1=SWITCHING, 2=PRESSING, 3=RETREATING |
| `/mcu_robot/intake/state` | `mcu_msgs/IntakeState` | 20 Hz | State, rail position (0–1), intake speed |
| `/mcu_robot/deploy/trigger` | `std_msgs/String` | Event | "start:robot", "start:*:force", "cancel" |
| `/mcu_robot/uwb/ranging` | `mcu_msgs/UWBRanging` | 5 Hz | Tag → anchor ranges (conditional on DIP 2) |
| `/mcu_robot/heartbeat` | `std_msgs/String` | 1 Hz | "HEARTBEAT" |
| `/mcu_robot/debug` | `std_msgs/String` | On-demand | Manager debug messages |
| `drive_base/status` | `mcu_msgs/DriveBase` | 50 Hz | Pose, velocity, drive mode |

### Subscribers

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/mcu_robot/lcd/append` | `std_msgs/String` | Append text to OLED (max 2816 bytes) |
| `/mcu_robot/lcd/scroll` | `std_msgs/Int8` | Scroll OLED: -1=up, +1=down |
| `/mcu_robot/led/set_all` | `mcu_msgs/LedColor` | Set all 5 WS2812B LEDs (r, g, b) |
| `/mcu_robot/crank/command` | `std_msgs/UInt8` | 1=SPIN, 2=RESET |
| `/mcu_robot/keypad/command` | `std_msgs/UInt8` | 1–4=KEY1–KEY4, 5=PRESS |
| `/mcu_robot/intake/command` | `mcu_msgs/IntakeCommand` | rail_command (0–3), intake_speed (-1 to 1) |
| `/mcu_robot/deploy/status` | `std_msgs/String` | Orchestrator phase feedback |
| `drive_base/command` | `mcu_msgs/DriveBase` | Velocity/goal/trajectory command |
| `drive_base/reset_pose` | `geometry_msgs/Pose` | Override odometry pose |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/mcu_robot/servo/set` | `mcu_msgs/srv/SetServo` | Set servo angle: `index` (0–7), `angle` (0–180°) |
| `/mcu_robot/motor/set` | `mcu_msgs/srv/SetMotor` | Set motor speed: `index` (0–7), `speed` (-1.0 to 1.0) |
| `/mcu_robot/imu/tare` | `mcu_msgs/srv/Reset` | Zero IMU heading via BNO085 hardware tare |
| `/mcu_robot/reset` | `mcu_msgs/srv/Reset` | Reset all registered subsystems |

## Usage Examples

### Monitor sensor data

```bash
ros2 topic list | grep mcu_robot
ros2 topic echo /mcu_robot/battery_health
ros2 topic echo /mcu_robot/imu/data
ros2 topic echo /mcu_robot/encoders
ros2 topic echo /mcu_robot/dip_switches
ros2 topic echo /mcu_robot/buttons
ros2 topic hz /mcu_robot/imu/data       # Expect ~100 Hz
```

### Motor control

```bash
# Set right drive motor (0) to 50% forward
ros2 service call /mcu_robot/motor/set mcu_msgs/srv/SetMotor "{index: 0, speed: 0.5}"

# Set left drive motor (1) to 50% forward
ros2 service call /mcu_robot/motor/set mcu_msgs/srv/SetMotor "{index: 1, speed: 0.5}"

# Stop both
ros2 service call /mcu_robot/motor/set mcu_msgs/srv/SetMotor "{index: 0, speed: 0.0}"
ros2 service call /mcu_robot/motor/set mcu_msgs/srv/SetMotor "{index: 1, speed: 0.0}"
```

### Servo control

```bash
# Set crank servo (ch 0) to 90°
ros2 service call /mcu_robot/servo/set mcu_msgs/srv/SetServo "{index: 0, angle: 90.0}"

# Set keypad servo (ch 1) to 60°
ros2 service call /mcu_robot/servo/set mcu_msgs/srv/SetServo "{index: 1, angle: 60.0}"
```

### Drive commands

```bash
# Velocity mode: drive forward 0.3 m/s
ros2 topic pub --once drive_base/command mcu_msgs/msg/DriveBase \
  "{drive_mode: 0, goal_velocity: {linear: {x: 0.3}, angular: {z: 0.0}}}"

# Goal mode: drive to (1.0, 0.5) at heading 0
ros2 topic pub --once drive_base/command mcu_msgs/msg/DriveBase \
  "{drive_mode: 1, goal_transform: {transform: {translation: {x: 1.0, y: 0.5}}}}"

# Reset odometry
ros2 topic pub --once drive_base/reset_pose geometry_msgs/msg/Pose \
  "{position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}"
```

### LED control

```bash
ros2 topic pub --once /mcu_robot/led/set_all mcu_msgs/msg/LedColor "{r: 255, g: 0, b: 0}"
ros2 topic pub --once /mcu_robot/led/set_all mcu_msgs/msg/LedColor "{r: 0, g: 255, b: 0}"
ros2 topic pub --once /mcu_robot/led/set_all mcu_msgs/msg/LedColor "{r: 0, g: 0, b: 0}"
```

### OLED display

```bash
ros2 topic pub --once /mcu_robot/lcd/append std_msgs/msg/String "{data: 'Hello from ROS2'}"
ros2 topic pub --once /mcu_robot/lcd/scroll std_msgs/msg/Int8 "{data: 1}"   # scroll down
ros2 topic pub --once /mcu_robot/lcd/scroll std_msgs/msg/Int8 "{data: -1}"  # scroll up
```

### Crank task

```bash
ros2 topic pub --once /mcu_robot/crank/command std_msgs/msg/UInt8 "{data: 1}"  # spin
ros2 topic pub --once /mcu_robot/crank/command std_msgs/msg/UInt8 "{data: 2}"  # reset
ros2 topic echo /mcu_robot/crank/state
```

### Keypad task

```bash
ros2 topic pub --once /mcu_robot/keypad/command std_msgs/msg/UInt8 "{data: 1}"  # key 1
ros2 topic pub --once /mcu_robot/keypad/command std_msgs/msg/UInt8 "{data: 2}"  # key 2
ros2 topic pub --once /mcu_robot/keypad/command std_msgs/msg/UInt8 "{data: 5}"  # press
ros2 topic echo /mcu_robot/keypad/state
```

### Intake control

```bash
# Extend rail
ros2 topic pub --once /mcu_robot/intake/command mcu_msgs/msg/IntakeCommand \
  "{rail_command: 1, intake_speed: 0.0}"

# Retract rail
ros2 topic pub --once /mcu_robot/intake/command mcu_msgs/msg/IntakeCommand \
  "{rail_command: 0, intake_speed: 0.0}"

# Spin intake motor at 50%
ros2 topic pub --once /mcu_robot/intake/command mcu_msgs/msg/IntakeCommand \
  "{rail_command: 2, intake_speed: 0.5}"
```

### Reset all subsystems

```bash
ros2 service call /mcu_robot/reset mcu_msgs/srv/Reset
```

### Tare IMU heading

```bash
ros2 service call /mcu_robot/imu/tare mcu_msgs/srv/Reset
```

## Architecture

### Threading Model

All subsystems run as independent TeensyThreads tasks:

| Priority | Subsystem | Rate | Stack | Notes |
|----------|-----------|------|-------|-------|
| 4 | IMU (BNO085) | 100 Hz | 8192 | INT pin optimization |
| 4 | Drive | 50 Hz | 4096 | S-curve + PID + localization |
| 3 | Servo | 10 Hz | 2048 | PCA9685 #0 |
| 3 | Motor Manager | 1000 Hz | 2048 | NFPShop reverse-pulse handling |
| 3 | Encoder | 20 Hz | 2048 | QTimer FG pulse counting |
| 3 | UWB | 5 Hz | 2048 | DW3000 tag (DIP 2 conditional) |
| 3 | Arm, Intake | 20 Hz | 2048 | Field mechanism control |
| 2 | OLED | 10 Hz | 2048 | SSD1306 128×64 |
| 2 | Battery | 0.5 Hz | 2048 | INA219 voltage/current |
| 2 | Sensor (TOF) | 2 Hz | 2048 | VL53L0X distance |
| 2 | DIP Switch | 0.5 Hz | 2048 | Mode configuration |
| 2 | Button | 10 Hz | 2048 | TCA9555 edge detection |
| 2 | LED | 5 Hz | 2048 | WS2812B updates |
| 2 | Heartbeat | 1 Hz | 2048 | Alive pulse |
| 2 | Deploy | 10 Hz | 2048 | Button 4 hold + LED feedback |
| 2 | Crank, Keypad | 5 Hz | 2048 | Servo time-based control |
| 2 | RC + Manual Drive | 200 Hz | 512 | IBusBM from main loop |
| 1 | micro-ROS Manager | ~1000 Hz | 8192 | Deferred publish + executor spin |

### Deferred Publishing

Subsystem threads cache messages under `data_mutex_` and set `data_ready_ = true`. The MicrorosManager calls `publishAll()` under `g_microros_mutex`, which acquires each subsystem's `data_mutex_` and publishes if ready. Lock ordering: `g_microros_mutex` (outer) > `data_mutex_` (inner).

### micro-ROS Entity Limits

- MAX_PARTICIPANTS: 32 (20 used)
- Executor handles: 24
- Publishers: 22 max
- Subscriptions: 14 max
- Services: 6 max

### Drive Control Pipeline

```
Command → [S-Curve Motion Profile] → [Inverse Kinematics] → [Wheel PID] → Motors
                                                                    ↑
                                                              Encoder feedback
                                                                    ↑
Odometry ← [Tank Drive Localization] ← Encoders + IMU yaw
```

**Drive modes:** VELOCITY (twist), GOAL (pose), TRAJECTORY (waypoints), MANUAL (RC passthrough)

**Physical constants:**
- Track width: 10.0" (0.254 m)
- Wheel diameter: 3.25" (0.0825 m)
- Gear ratio: 56.67:1
- Ticks per rev: 170
- Max wheel velocity: 0.732 m/s
