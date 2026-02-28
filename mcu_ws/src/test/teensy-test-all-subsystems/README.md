# teensy-test-all-subsystems

Wires **every** robot subsystem together in a single firmware image. Useful for verifying the full robot hardware stack before running `env:robot`.

| | |
|---|---|
| **Environment** | `teensy-test-all-subsystems` |
| **Hardware** | Full robot hardware (see below) |
| **micro-ROS** | Yes (serial) |

## Known Issues

- **LED subsystem**: WS2812B LEDs not working properly — to be investigated
- **RC subsystem (IBUS)**: FlySky receiver not reading correctly — to be investigated
- **IMU**: BNO085 not initializing correctly

## Required Hardware

| Bus | Device | Address / Pin |
|-----|--------|---------------|
| Wire0 | TCA9548A I2C mux | 0x70 |
| Wire0 | TCA9555 GPIO expander | 0x20 |
| Wire0 (mux ch0) | INA219 power monitor | 0x40 |
| Wire0 | VL53L0X TOF sensor | default |
| Wire1 | BNO085 IMU | 0x4A (RST=40, INT=41) |
| Wire2 | PCA9685 servo board | 0x40 (OE=28) |
| Wire2 | PCA9685 motor board | 0x41 (OE=29) |
| Software SPI | SSD1306 OLED 128x64 | MOSI=26, CLK=27, CS=38, DC=37, RST=33 |
| SPI0 | DW3000 UWB tag (ID=13) | CS=10, MOSI=11, MISO=12, CLK=13 |
| GPIO | WS2812B LED strip (x5) | pin 35 |
| Serial8 | FlySky RC receiver | pin 34 |

## Build & Flash

```bash
pio run -e teensy-test-all-subsystems --target upload
pio device monitor -e teensy-test-all-subsystems
```

Start the micro-ROS agent (separate terminal):

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 921600
```

**Expected serial output:**

```
SEC26 Robot — All Subsystems Test (TeensyThreads)
setup(): all subsystem threads started.
```

LEDs flash green on startup. All topics and services become available once the micro-ROS agent connects.

## ROS2 Interface

### Published Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/mcu_robot/heartbeat` | `std_msgs/msg/String` | 5 Hz | Uptime heartbeat |
| `/mcu_robot/battery_health` | `mcu_msgs/msg/BatteryHealth` | 1 Hz | Voltage, current, power |
| `/mcu_robot/imu/data` | `sensor_msgs/msg/Imu` | 50 Hz | Quaternion + accel + gyro |
| `/mcu_robot/rc` | `mcu_msgs/msg/RC` | 20 Hz | RC joystick/switch channels |
| `/mcu_robot/tof_distances` | `std_msgs/msg/Float32MultiArray` | 10 Hz | TOF distance readings (mm) |
| `/mcu_robot/dip_switches` | `std_msgs/msg/UInt8` | 2 Hz | 8-bit DIP switch bitmask |
| `/mcu_robot/buttons` | `std_msgs/msg/UInt8` | 10 Hz | 8-bit push button bitmask |
| `/mcu_robot/servo/state` | `std_msgs/msg/Float32MultiArray` | 5 Hz | Current servo angles (deg) |
| `/mcu_robot/motor/state` | `std_msgs/msg/Float32MultiArray` | 5 Hz | Current motor speeds (-1..1) |
| `mcu_uwb/ranging` | `mcu_msgs/msg/UWBRanging` | 10 Hz | UWB distance measurements to beacons |
| `/mcu_robot/crank/state` | `std_msgs/msg/UInt8` | 5 Hz | Crank state (0=IDLE, 1=CRANKING, 2=RESETTING) |
| `/mcu_robot/deploy/trigger` | `std_msgs/msg/String` | on event | Deploy trigger command |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/mcu_robot/servo/set` | `mcu_msgs/srv/SetServo` | Set servo angle (0-180 deg) |
| `/mcu_robot/motor/set` | `mcu_msgs/srv/SetMotor` | Set motor speed (-1.0 to 1.0) |

### Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `/mcu_robot/lcd/append` | `std_msgs/msg/String` | Append text to OLED (newlines split into lines) |
| `/mcu_robot/lcd/scroll` | `std_msgs/msg/Int8` | Scroll OLED display (-1=up, 1=down) |
| `/mcu_robot/led/set_all` | `mcu_msgs/msg/LedColor` | Set all WS2812B LEDs to RGB color |
| `/mcu_robot/crank/command` | `std_msgs/msg/UInt8` | Crank command (1=SPIN, 2=RESET) |
| `/mcu_robot/deploy/status` | `std_msgs/msg/String` | Deploy status feedback |

## Testing Commands

### Prerequisites

Make sure `mcu_msgs` is built on the ROS2 side:

```bash
cd /home/ubuntu/ros2_workspaces
colcon build --packages-select mcu_msgs
source install/setup.bash
```

### Verify Everything Is Up

```bash
# List all topics (expect 12 topics under /mcu_robot/ + 1 under mcu_uwb/)
ros2 topic list | grep -E "mcu_robot|mcu_uwb"

# List all services (expect 2 services)
ros2 service list | grep mcu_robot
```

### Heartbeat

```bash
# Should print string data every ~200ms
ros2 topic echo /mcu_robot/heartbeat
```

### Battery Health

```bash
# Voltage, current, power from INA219
ros2 topic echo /mcu_robot/battery_health

# Expected fields:
#   voltage: 12.3        (V)
#   current: 0.15        (A)
#   power: 1.85          (W)
#   shunt_voltage: ...   (V)
#   temperature: ...     (C)
#   energy: ...          (Wh)
#   charge_use: ...      (Ah)
```

### IMU

```bash
# Quaternion orientation + linear acceleration + angular velocity
ros2 topic echo /mcu_robot/imu/data

# Check just the orientation quaternion
ros2 topic echo /mcu_robot/imu/data --field orientation

# Monitor at reduced rate
ros2 topic hz /mcu_robot/imu/data
# Expected: ~50 Hz
```

### RC Receiver

```bash
# Raw channel data + parsed switches/joysticks
ros2 topic echo /mcu_robot/rc

# Expected fields:
#   channels: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]   (range -255 to 255)
#   swa: false   swb: false   swc: false   swd: false
#   knobl: 0     knobr: 0
#   lx: 0   ly: 0   rx: 0   ry: 0
#
# Move RC sticks and flip switches to see values change.
```

### TOF Distance Sensors

```bash
# Distance readings in mm
ros2 topic echo /mcu_robot/tof_distances

# Expected: data array with one float per TOF sensor
```

### DIP Switches

```bash
# 8-bit bitmask of DIP switch positions (bit N = switch N)
ros2 topic echo /mcu_robot/dip_switches

# Expected: data: 0 (all off) through data: 255 (all on)
# Toggle DIP switches on the TCA9555 Port 0 to see changes.
```

### Push Buttons

```bash
# 8-bit bitmask of button states (bit N = button N pressed)
ros2 topic echo /mcu_robot/buttons

# Expected: data: 0 (none pressed)
# Press buttons on TCA9555 Port 1 to see bits go high.
```

### OLED Display

The first line always shows battery status: voltage, current, power (2 decimal places), and estimated battery percent.

```bash
# Append a line of text to the OLED terminal (appears below battery status line)
ros2 topic pub --once /mcu_robot/lcd/append std_msgs/msg/String "{data: 'Hello from ROS2'}"

# Append multi-line text (newlines split into separate lines)
ros2 topic pub --once /mcu_robot/lcd/append std_msgs/msg/String "{data: 'Line 1\nLine 2\nLine 3'}"

# Scroll up (view older lines)
ros2 topic pub --once /mcu_robot/lcd/scroll std_msgs/msg/Int8 "{data: -1}"

# Scroll down (back toward live output)
ros2 topic pub --once /mcu_robot/lcd/scroll std_msgs/msg/Int8 "{data: 1}"
```

### Servos

```bash
# Monitor current servo angles (8 floats, degrees)
ros2 topic echo /mcu_robot/servo/state

# Set servo 0 to 90 degrees (center)
ros2 service call /mcu_robot/servo/set mcu_msgs/srv/SetServo "{index: 0, angle: 90.0}"

# Set servo 1 to 0 degrees (min)
ros2 service call /mcu_robot/servo/set mcu_msgs/srv/SetServo "{index: 1, angle: 0.0}"

# Set servo 2 to 180 degrees (max)
ros2 service call /mcu_robot/servo/set mcu_msgs/srv/SetServo "{index: 2, angle: 180.0}"

# Sweep all servos to center
for i in 0 1 2 3 4 5 6 7; do
  ros2 service call /mcu_robot/servo/set mcu_msgs/srv/SetServo "{index: $i, angle: 90.0}"
done
```

### Motors

```bash
# Monitor current motor speeds (8 floats, -1.0 to 1.0)
ros2 topic echo /mcu_robot/motor/state

# Set motor 0 to 50% forward
ros2 service call /mcu_robot/motor/set mcu_msgs/srv/SetMotor "{index: 0, speed: 0.5}"

# Set motor 1 to 30% reverse
ros2 service call /mcu_robot/motor/set mcu_msgs/srv/SetMotor "{index: 1, speed: -0.3}"

# Stop motor 0
ros2 service call /mcu_robot/motor/set mcu_msgs/srv/SetMotor "{index: 0, speed: 0.0}"

# Stop all motors
for i in 0 1 2 3 4 5 6 7; do
  ros2 service call /mcu_robot/motor/set mcu_msgs/srv/SetMotor "{index: $i, speed: 0.0}"
done
```

### UWB Ranging

Requires UWB beacons (ID=10, 11, 12) to be powered on and within range.

```bash
# View ranging data (distances to each beacon)
ros2 topic echo mcu_uwb/ranging

# Check publish rate (~10 Hz)
ros2 topic hz mcu_uwb/ranging
```

### LEDs

```bash
# Set all WS2812B LEDs to red (r=255, g=0, b=0)
ros2 topic pub --once /mcu_robot/led/set_all mcu_msgs/msg/LedColor "{r: 255, g: 0, b: 0}"

# Set all LEDs to green
ros2 topic pub --once /mcu_robot/led/set_all mcu_msgs/msg/LedColor "{r: 0, g: 255, b: 0}"

# Set all LEDs to blue
ros2 topic pub --once /mcu_robot/led/set_all mcu_msgs/msg/LedColor "{r: 0, g: 0, b: 255}"

# Set all LEDs to white (dim)
ros2 topic pub --once /mcu_robot/led/set_all mcu_msgs/msg/LedColor "{r: 32, g: 32, b: 32}"

# Turn off all LEDs
ros2 topic pub --once /mcu_robot/led/set_all mcu_msgs/msg/LedColor "{r: 0, g: 0, b: 0}"
```

### Crank

The crank servo is on PCA9685 #0, channel 0. It does **not** move at startup — an explicit command is required.

```bash
# Monitor crank state (0=IDLE, 1=CRANKING, 2=RESETTING)
ros2 topic echo /mcu_robot/crank/state

# Spin crank (servo moves 0° → 180°, holds CRANKING for ~1.5s, returns to IDLE)
ros2 topic pub --once /mcu_robot/crank/command std_msgs/msg/UInt8 "{data: 1}"

# Reset crank (servo moves 180° → 0°, holds RESETTING for ~1.5s, returns to IDLE)
ros2 topic pub --once /mcu_robot/crank/command std_msgs/msg/UInt8 "{data: 2}"
```

---

## Quick Sanity Check (copy-paste block)

Run these in sequence to verify all subsystems:

```bash
# 1. Check topics and services exist
ros2 topic list | grep mcu_robot
ros2 service list | grep mcu_robot

# 2. Heartbeat (Ctrl+C after a few messages)
ros2 topic echo /mcu_robot/heartbeat --once

# 3. IMU alive
ros2 topic echo /mcu_robot/imu/data --once

# 4. Battery readings
ros2 topic echo /mcu_robot/battery_health --once

# 5. OLED write
ros2 topic pub --once /mcu_robot/lcd/append std_msgs/msg/String "{data: 'Sanity check OK'}"

# 6. Servo test (moves servo 0 to 90 deg)
ros2 service call /mcu_robot/servo/set mcu_msgs/srv/SetServo "{index: 0, angle: 90.0}"

# 7. Motor test (spins motor 0 at 20% — careful!)
ros2 service call /mcu_robot/motor/set mcu_msgs/srv/SetMotor "{index: 0, speed: 0.2}"
sleep 2
ros2 service call /mcu_robot/motor/set mcu_msgs/srv/SetMotor "{index: 0, speed: 0.0}"

# 8. LED test (flash green then off)
ros2 topic pub --once /mcu_robot/led/set_all mcu_msgs/msg/LedColor "{r: 0, g: 64, b: 0}"
sleep 1
ros2 topic pub --once /mcu_robot/led/set_all mcu_msgs/msg/LedColor "{r: 0, g: 0, b: 0}"

# 9. RC (move sticks, Ctrl+C to stop)
ros2 topic echo /mcu_robot/rc --once

# 10. UWB ranging (requires beacons powered on)
ros2 topic echo mcu_uwb/ranging --once

# 11. Buttons and DIP switches
ros2 topic echo /mcu_robot/buttons --once
ros2 topic echo /mcu_robot/dip_switches --once

# 12. Crank spin test (moves servo 0 to 180 deg)
ros2 topic pub --once /mcu_robot/crank/command std_msgs/msg/UInt8 "{data: 1}"
sleep 2
ros2 topic pub --once /mcu_robot/crank/command std_msgs/msg/UInt8 "{data: 2}"
```

## Thread Configuration

| Priority | Subsystem | Rate | Stack |
|----------|-----------|------|-------|
| 4 (highest) | micro-ROS Manager | 10ms | 8192 |
| 3 | IMU (BNO085) | 20ms (50 Hz) | 2048 |
| 3 | RC Receiver (IBUS) | 5ms | 1024 |
| 2 | Servo state publisher | 50ms (20 Hz) | 1024 |
| 2 | Motor state publisher | 50ms (20 Hz) | 1024 |
| 2 | UWB tag (DW3000) | 50ms (20 Hz) | 2048 |
| 1 | OLED display | 50ms (20 Hz) | 2048 |
| 1 | Battery monitor | 100ms (10 Hz) | 1024 |
| 1 | TOF sensors | 100ms (10 Hz) | 1024 |
| 1 | DIP switches | 500ms (2 Hz) | 1024 |
| 1 | Buttons | 20ms (50 Hz) | 1024 |
| 1 | LEDs | 50ms (20 Hz) | 1024 |
| 1 | Heartbeat | 200ms (5 Hz) | 1024 |
| 1 | Deploy | 20ms (50 Hz) | 1024 |
| 1 | Crank | 50ms (20 Hz) | 1024 |
| — | PCA9685 PWM flush | 20ms | 1024 |

## Troubleshooting

**micro-ROS agent connects then immediately disconnects:**
- Check that `custom_microros.meta` is present and `board_microros_user_meta = custom_microros.meta` is set in `platformio.ini`. The default entity limits are too low for this test (needs 3 services, default allows 1).
- After changing meta, run: `pio run -e teensy-test-all-subsystems -t clean_microros && pio run -e teensy-test-all-subsystems`

**CrashReport on startup (DACCVIOL / null pointer):**
- Usually means a hardware device (BNO085, TOF, INA219) is not wired or not responding. The drivers have `initSuccess_` guards to prevent crashes, but check that all I2C devices are connected and addresses match `RobotPins.h`.

**`mcu_msgs/msg/LedColor` type invalid:**
- You need to build `mcu_msgs` on the ROS2 side: `colcon build --packages-select mcu_msgs && source install/setup.bash`

**Topics show but no data:**
- Make sure the micro-ROS agent is running and the Teensy serial port is not held by another process (e.g., `pio device monitor`). The agent and serial monitor cannot share the port.

**I2C device not found:**
- Check wiring, pull-ups (4.7k on SDA/SCL), and that the correct Wire bus is used (Wire0=pins 18/19, Wire1=pins 16/17, Wire2=pins 24/25).
- For devices behind the TCA9548A mux, ensure `PIN_MUX_RESET` (pin 23) is wired HIGH.
