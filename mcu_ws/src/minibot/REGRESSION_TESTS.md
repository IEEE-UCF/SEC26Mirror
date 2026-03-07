# SEC26 Minibot Regression Tests

Manual hardware regression tests for the `minibot` firmware target.

## Overview

The minibot runs on a Seeed XIAO ESP32-S3 with WiFi micro-ROS (UDP transport).
It has UWB tag positioning, differential drive motors, and an MPU6050 IMU.

- **MCU:** ESP32-S3 (Seeed XIAO Plus)
- **Transport:** WiFi UDP micro-ROS to Pi agent (port 8888)
- **Static IP:** 192.168.4.24
- **OTA hostname:** `sec26-minibot`
- **UWB Tag ID:** 2 (ranges to anchors 10, 11, 12, 13)
- **Participants:** 3 (UWBSubsystem, MinibotDriveSubsystem, MiniBotIMUSubsystem)
- **Entry point:** `src/minibot/machines/MinibotLogic.h`

**Note:** Pin assignments are currently TODOs — this target is in scaffolding state.
Regression tests should be run after pins are assigned and hardware is wired.

### Pin Assignments (TODO)

| Function | Pin | Status |
|----------|-----|--------|
| I2C SDA (MPU6050) | 5 | TODO |
| I2C SCL (MPU6050) | 6 | TODO |
| UWB SPI CS | 7 | TODO |
| UWB Reset | 8 | TODO |
| Motor A IN1 | 1 | TODO |
| Motor A IN2 | 2 | TODO |
| Motor A PWM | 3 | TODO |
| Motor B IN1 | 4 | TODO |
| Motor B IN2 | 9 | TODO |
| Motor B PWM | 10 | TODO |

## Prerequisites

- Seeed XIAO ESP32-S3 board powered on
- Pi WiFi AP running (SSID: `UCFIEEEBot`)
- Docker container running with UDP micro-ROS agent (auto-started by entrypoint)
- MPU6050 wired to I2C, DW3000 wired to SPI, motors wired
- All commands run inside the container

## Quick Reference

```bash
# Inside Docker container
cd /home/ubuntu/mcu_workspaces/sec26mcu

# Build minibot firmware
pio run -e minibot

# Flash via USB
pio run -e minibot --target upload

# Flash via OTA (if minibot is on WiFi)
pio run -e minibot --target upload --upload-port 192.168.4.24

# Monitor serial (USB)
pio device monitor -e minibot
```

## Test Procedure

### 1. Flash and Boot

```bash
pio run -e minibot --target upload
```

**Verify on serial monitor:**
- [ ] `Minibot starting...` message appears
- [ ] `WiFi connected: 192.168.4.24` prints (correct IP)
- [ ] `[OTA] Ready` appears
- [ ] No `ERROR: One or more subsystems failed init!` (or identify which failed)
- [ ] `Minibot initialized!` prints
- [ ] No crash or reboot loop

### 2. WiFi Connection

- [ ] Minibot connects to `UCFIEEEBot` AP
- [ ] Ping from Pi: `ping 192.168.4.24` responds
- [ ] WiFi auto-reconnects after AP reboot

### 3. micro-ROS Connection

```bash
# Verify UDP agent is running
pgrep -f "micro_ros_agent udp4" && echo "Agent running"

# Check for minibot topics
ros2 topic list | grep minibot
```

**Verify:**
- [ ] Agent shows session established
- [ ] All expected topics appear (see topic table)

### 4. Topic Publish Rates

| Topic | Expected Hz | Acceptable Range | Notes |
|-------|------------|-----------------|-------|
| `/mcu_minibot/state` | TBD | TBD | Drive subsystem state |
| UWB ranging | 10 | 8 - 12 | Tag mode, ranges to 4 anchors |
| IMU data | TBD | TBD | MPU6050 orientation |

```bash
ros2 topic hz /mcu_minibot/state
```

### 5. Topic Data Validation

#### Drive State
```bash
ros2 topic echo /mcu_minibot/state --once
```
- [ ] Message fields are populated
- [ ] Values are reasonable

#### UWB Ranging
```bash
ros2 topic echo <uwb_topic> --once
```
- [ ] Ranges include anchor IDs (10, 11, 12, 13)
- [ ] Distance values are reasonable

#### IMU
- [ ] Orientation quaternion is normalized
- [ ] Angular velocity near zero when stationary

### 6. Drive Control

```bash
# Send velocity command (differential drive)
ros2 topic pub --rate 10 /mcu_minibot/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

- [ ] Both motors spin forward
- [ ] Stopping the publisher stops motors
- [ ] Turning commands (angular.z != 0) cause differential speeds

### 7. OTA Update

```bash
pio run -e minibot --target upload --upload-port 192.168.4.24
```

- [ ] OTA flash completes without error
- [ ] Minibot reboots and reconnects to WiFi
- [ ] All topics resume publishing after OTA

### 8. Reconnection

```bash
pkill -9 -f "micro_ros_agent udp4"
sleep 5
# Agent auto-restarts via entrypoint
```

- [ ] Minibot reconnects to agent
- [ ] All topics resume at correct rates
- [ ] No crash or hang on ESP32

### 9. Stability

Run for 60+ seconds:

- [ ] No serial error messages
- [ ] WiFi stays connected
- [ ] All topics continue publishing
- [ ] No watchdog resets
- [ ] Motors stop cleanly on command timeout

## Common Failures and Causes

| Symptom | Likely Cause |
|---------|-------------|
| WiFi won't connect | AP not running, wrong SSID/password, IP conflict |
| Subsystem init fails | Pin assignment TODO not updated, hardware not wired |
| No UWB ranging | DW3000 not wired, SPI pins wrong, anchors not powered |
| Motors don't spin | Motor driver pins wrong, power supply issue |
| IMU reads zero | MPU6050 not on I2C bus, wrong address |
| OTA fails | Not on WiFi, wrong IP |

## Checklist Template

```
Date: _______________
Git commit: _______________
Tester: _______________

Boot:
  [ ] Serial messages correct
  [ ] WiFi connected (192.168.4.24)
  [ ] All subsystems initialized
  [ ] OTA ready

micro-ROS:
  [ ] Agent connects
  [ ] All topics visible

Rates:
  drive state:  _____ Hz
  UWB ranging:  _____ Hz (expect 10)
  IMU:          _____ Hz

Drive:
  [ ] Forward drive works
  [ ] Turning works
  [ ] Command timeout stops motors

OTA:
  [ ] OTA flash works

Stability (60s+):
  [ ] No crashes
  [ ] WiFi stable
  [ ] Topics steady

Issues Found:
  _______________________________________________
```
