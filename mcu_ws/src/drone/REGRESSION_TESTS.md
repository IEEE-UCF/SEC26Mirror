# SEC26 Drone Regression Tests

Manual hardware regression tests for the `drone` and `esp32-test-all-drone-subsystems` firmware targets.

## Overview

The drone runs on a Seeed XIAO ESP32-S3 with FreeRTOS multi-tasking and WiFi micro-ROS (UDP transport).

- **MCU:** ESP32-S3 (Seeed XIAO)
- **Transport:** WiFi UDP micro-ROS to Pi agent (port 8888)
- **Static IP:** 192.168.4.25
- **OTA hostname:** `sec26-drone` (production), `sec26-drone-test` (test)
- **Participants:** 3-4 (Heartbeat, DroneState, DroneIR, DroneUWB)
- **Entry point:** `src/drone/main.cpp` (production), `src/test/esp32-test-all-drone-subsystems.cpp` (test)

### Test Environment Differences

| Aspect | `drone` | `esp32-test-all-drone-subsystems` |
|--------|---------|-----------------------------------|
| Entry point | `src/drone/main.cpp` | `src/test/esp32-test-all-drone-subsystems.cpp` |
| DEBUG_STAGE | Configurable 0-2 (default 2) | Configurable 1-7 (default 7) |
| UWB | Build flag (default 0) | Always enabled (flag=1) |
| Height | Build flag (default 1) | Always enabled (flag=1) |
| Serial debug | Configurable (add `-DDRONE_SERIAL_DEBUG`) | ON (`-DDRONE_SERIAL_DEBUG` in build flags) |
| OTA hostname | `sec26-drone` | `sec26-drone-test` |

### Production Debug Stages (`drone` env)

Set via `-DDRONE_DEBUG_STAGE=N` in platformio.ini:

| Stage | Subsystems Active | Purpose |
|-------|-------------------|---------|
| 0 | WiFi + micro-ROS + Heartbeat | Isolate micro-ROS connectivity |
| 1 | + Gyro/IMU (BNO085) | Isolate I2C bus issues |
| 2 | Full system (all subsystems) | Normal operation |

## Prerequisites

- ESP32 drone board powered on
- Pi WiFi AP running (SSID: `UCFIEEEBot`)
- Docker container running with UDP micro-ROS agent
- All commands run inside the container

## Quick Reference

```bash
# Inside Docker container
cd /home/ubuntu/mcu_workspaces/sec26mcu

# Build production drone
pio run -e drone

# Build test-all-subsystems
pio run -e esp32-test-all-drone-subsystems

# Flash via USB
pio run -e esp32-test-all-drone-subsystems --target upload

# Flash via OTA
pio run -e esp32-test-all-drone-subsystems --target upload --upload-port 192.168.4.25

# Monitor serial (USB, 921600 baud)
pio device monitor -e esp32-test-all-drone-subsystems
```

## Topic Publish Rates

| Topic | Expected Hz | Acceptable Range | Source |
|-------|------------|-----------------|--------|
| `/mcu_drone/heartbeat` | 1 | 0.9 - 1.1 | HeartbeatSubsystem (main loop) |
| `/mcu_drone/state` | 10 | 8 - 12 | DroneStateSubsystem (RTOSSubsystem, 10Hz) |
| `/mcu_drone/uwb/ranging` | 20 | 15 - 25 | DroneUWBSubsystem (RTOSSubsystem, 20Hz) |

## RTOSSubsystem Task Rates

All subsystems now use RTOSSubsystem (`beginThreaded` / `beginThreadedPinned`).

| Task (RTOSSubsystem) | Hz | Core | Priority | Timing | Stack |
|---|---|---|---|---|---|
| GyroSubsystem | 250 | 1 | 5 | Precise (vTaskDelayUntil) | 2048 |
| DroneFlightSubsystem | 250 | 1 | 4 | Precise (vTaskDelayUntil) | 2048 |
| HeightSubsystem | 50 | 1 | 3 | Precise (vTaskDelayUntil) | 2048 |
| DroneUWBSubsystem | 20 | 0 | 3 | Precise (vTaskDelayUntil) | 2048 |
| DroneSafetySubsystem | 10 | any | 3 | Standard (vTaskDelay) | 2048 |
| DroneStateSubsystem | 10 | any | 2 | Standard (vTaskDelay) | 2048 |
| HeartbeatSubsystem | 1 | loop | - | Loop update | - |
| Arduino `loop()` | ~100 | any | 1 | - | - |

## RTOSSubsystem Refactor Regression

Additional checks after converting all subsystems to `RTOSSubsystem::beginThreadedPinned()`:

- [ ] IMU init: 3000ms delay completes, BNO085 initializes reliably
- [ ] All RTOSSubsystem threads create successfully (check `[RTOS] Task created OK` messages)
- [ ] Flight rate stays at ~250Hz after refactoring
- [ ] Height rate stays at ~50Hz
- [ ] No I2C bus contention errors (gyro + height share mutex correctly)
- [ ] Serial debug OFF: build `drone` env, verify no serial output (clean production)
- [ ] Serial debug ON: build test env with `-DDRONE_SERIAL_DEBUG`, verify debug output over USB CDC
- [ ] EKF predict runs at flight rate (250Hz)
- [ ] UWB EKF update runs at 20Hz
- [ ] Heap stable after 60+ seconds (check `ESP.getMinFreeHeap()`)

## Test Procedure

### 1. Flash and Boot

```bash
pio run -e esp32-test-all-drone-subsystems --target upload
pio device monitor -e esp32-test-all-drone-subsystems
```

**Verify on serial monitor:**
- [ ] `SEC26 Drone — All Subsystems Test` banner appears
- [ ] `DEBUG_STAGE = 7` confirmed
- [ ] WiFi connected with correct IP (192.168.4.25)
- [ ] OTA ready message
- [ ] No crash or reboot loop

### 2. IMU Validation (BNO085)

**Verify initialization:**
- [ ] `IMU (BNO085 @ 0x4A): OK` prints
- [ ] Rotation/gyro/accel report rates print correctly

**Verify data (read serial debug output):**
- [ ] Euler angles (roll/pitch/yaw) update at ~200Hz+ (check "IMU rate" line)
- [ ] With drone flat: roll ~ 0, pitch ~ 0 (within +/- 3 deg)
- [ ] Rotate drone 90 degrees: yaw changes ~90 degrees
- [ ] Tilt drone: roll/pitch respond correctly
- [ ] Gyro rates near zero when stationary (< 1 deg/s)
- [ ] Accel near zero when stationary (gravity-free linear accel)
- [ ] Heading direction label matches physical orientation

**Orientation check — determine BNO085 facing direction:**
- [ ] Power on drone flat, note yaw reading (this is the "forward" reference, ~0 deg)
- [ ] Rotate drone 90 CW → yaw should increase to ~90
- [ ] Record which physical direction corresponds to yaw=0 (this is the BNO's X-axis forward)
- [ ] Record BNO mounting orientation: _____________________

```bash
# Monitor IMU from ROS2 (state includes roll/pitch/yaw)
ros2 topic echo /mcu_drone/state --field roll --field pitch --field yaw
```

### 3. Height Sensor (VL53L0X)

**Verify initialization:**
- [ ] `Height sensor: OK` prints

**Verify data:**
- [ ] Altitude reads ~0.0m when on desk/ground
- [ ] Altitude increases when lifting drone up
- [ ] Valid flag is `YES` for readings in range (0.01m - 4.0m)
- [ ] Valid flag is `NO` for readings out of range (too close or too far)
- [ ] Height rate shows ~50 Hz
- [ ] `last_valid` timestamp is recent (not stale)

### 4. Flight Controller (PID + Motor Mixer)

**Verify initialization:**
- [ ] `Flight controller: OK` prints
- [ ] PWM freq and resolution correct (20kHz, 10-bit)

**Verify default state:**
- [ ] Motors show 0.000 when unarmed
- [ ] Armed flag is `NO`
- [ ] Override flag is `NO`

**Verify arm/motor override via ROS2 service:**
```bash
# Arm the drone (keep props off or secured!)
ros2 service call /mcu_drone/arm mcu_msgs/srv/DroneArm "{arm: true}"
# Verify: State changes to ARMED, arm flag YES

# Test individual motors (ONLY with props removed)
ros2 service call /mcu_drone/set_motors mcu_msgs/srv/DroneSetMotors \
  "{motor_speeds: [0.1, 0.0, 0.0, 0.0]}"
# Verify: FL motor spins, others off

ros2 service call /mcu_drone/set_motors mcu_msgs/srv/DroneSetMotors \
  "{motor_speeds: [0.0, 0.1, 0.0, 0.0]}"
# Verify: FR motor spins, others off

ros2 service call /mcu_drone/set_motors mcu_msgs/srv/DroneSetMotors \
  "{motor_speeds: [0.0, 0.0, 0.1, 0.0]}"
# Verify: BR motor spins, others off

ros2 service call /mcu_drone/set_motors mcu_msgs/srv/DroneSetMotors \
  "{motor_speeds: [0.0, 0.0, 0.0, 0.1]}"
# Verify: BL motor spins, others off

# Stop all motors
ros2 service call /mcu_drone/set_motors mcu_msgs/srv/DroneSetMotors \
  "{motor_speeds: [0.0, 0.0, 0.0, 0.0]}"

# Disarm
ros2 service call /mcu_drone/arm mcu_msgs/srv/DroneArm "{arm: false}"
```

- [ ] Arm service succeeds from UNARMED state
- [ ] Arm service fails if IMU not ready
- [ ] Each motor spins when individually commanded (props removed!)
- [ ] Motor direction matches expected (FL=CCW, FR=CW, BR=CCW, BL=CW)
- [ ] Disarm service succeeds from ARMED state
- [ ] Cannot disarm while flying ("use land" message)
- [ ] Motors go to zero on disarm

### 5. EKF

**Verify initialization:**
- [ ] `EKF: OK` prints

**Verify operation:**
- [ ] Position (x, y) values shown in debug output
- [ ] Velocity (vx, vy) near zero when stationary
- [ ] Anchor count shows configured value when set_anchors is called

```bash
# Configure UWB anchors
ros2 service call /mcu_drone/set_anchors mcu_msgs/srv/DroneSetAnchors \
  "{num_anchors: 3, anchor_ids: [10, 11, 12], anchor_x: [0.0, 3.66, 0.0], anchor_y: [0.0, 0.0, 3.66]}"
```

- [ ] Anchor count updates to 3

### 6. IR Transmitter

**Verify initialization:**
- [ ] `IR LED: OK` prints

**Verify transmission (requires IR receiver for full test):**
```bash
# Must be in VELOCITY_CONTROL state to transmit
# (arm → takeoff → wait → then transmit)
# For bench test, verify service exists:
ros2 service list | grep transmit_ir
```
- [ ] `/mcu_drone/transmit_ir` service exists
- [ ] Service rejects calls when not in VELOCITY_CONTROL state

### 7. UWB

**Verify initialization:**
- [ ] `UWB DW3000: OK` prints (if hardware connected)
- [ ] Target anchors set to 10, 11, 12

**Verify operation (requires beacons):**
- [ ] UWB rate shows ~20 Hz
- [ ] Valid ranges appear for each beacon
- [ ] Range values are reasonable (in cm)

```bash
ros2 topic echo /mcu_drone/uwb/ranging
ros2 topic hz /mcu_drone/uwb/ranging
```
- [ ] `/mcu_drone/uwb/ranging` publishes at ~20 Hz
- [ ] Range data includes peer_id matching beacon IDs

### 8. State Machine Transitions

```bash
# 1. Check initial state
ros2 topic echo /mcu_drone/state --field state --once
# Should be 0 (INIT) or 1 (UNARMED)

# 2. Arm
ros2 service call /mcu_drone/arm mcu_msgs/srv/DroneArm "{arm: true}"
# State → 2 (ARMED)

# 3. Takeoff (HEIGHT SENSOR REQUIRED)
ros2 service call /mcu_drone/takeoff mcu_msgs/srv/DroneTakeoff "{target_altitude: 0.5}"
# State → 3 (LAUNCHING) → 4 (VELOCITY_CONTROL) when alt ≥ 95% target

# 4. Land
ros2 service call /mcu_drone/land mcu_msgs/srv/DroneLand "{}"
# State → 5 (LANDING) → 1 (UNARMED) when alt < 0.05m
```

- [ ] INIT → UNARMED on sensor ready
- [ ] UNARMED → ARMED on arm service
- [ ] ARMED → LAUNCHING on takeoff service
- [ ] LAUNCHING → VELOCITY_CONTROL at 95% target altitude
- [ ] VELOCITY_CONTROL → LANDING on land service
- [ ] LANDING → UNARMED when altitude < 0.05m
- [ ] cmd_vel timeout (500ms) → auto-hover (zero attitude, hold altitude)

### 9. Safety Monitor

**Verify safety triggers:**
- [ ] IMU failure → EMERGENCY_LAND (hard to test without disconnecting hardware)
- [ ] micro-ROS disconnect > 3s → EMERGENCY_LAND

```bash
# Kill UDP agent to test disconnect
pkill -9 -f "micro_ros_agent udp4"
# Wait 3+ seconds
# Verify: state transitions to EMERGENCY_LAND (if flying)
```

- [ ] Safety task count increases at ~10 Hz
- [ ] Sensor readiness correctly reflects hardware state

### 10. WiFi & Reconnection

- [ ] Drone connects to `UCFIEEEBot` AP
- [ ] Ping from Pi: `ping 192.168.4.25`
- [ ] WiFi auto-reconnects after AP reboot (wait ~15s)
- [ ] micro-ROS reconnects after agent restart
- [ ] Heartbeat resumes after reconnection

### 11. OTA Update

```bash
pio run -e esp32-test-all-drone-subsystems --target upload --upload-port 192.168.4.25
```

- [ ] OTA flash completes without error
- [ ] Drone reboots and reconnects
- [ ] All subsystems re-initialize correctly

### 12. Stability (60s+ run)

- [ ] No serial error messages
- [ ] WiFi stays connected
- [ ] Heartbeat continues publishing
- [ ] No watchdog resets
- [ ] Heap not decreasing (check "Heap" and "min" lines in debug output)
- [ ] No I2C bus contention (no error messages from gyro or height)
- [ ] EKF predict runs at flight rate
- [ ] All RTOSSubsystem tasks remain running

### 13. Staged Debug Bring-up

If issues are found, set `DRONE_DEBUG_STAGE` in platformio.ini to isolate:

```bash
# In platformio.ini [env:drone] build_flags, set:
#   -DDRONE_DEBUG_STAGE=0   WiFi + micro-ROS + Heartbeat only
#   -DDRONE_DEBUG_STAGE=1   + Gyro/IMU (I2C)
#   -DDRONE_DEBUG_STAGE=2   Full system (default)
#
# Also add -DDRONE_SERIAL_DEBUG and -DSERIAL_DEBUG for verbose output.

pio run -e drone --target upload
pio device monitor -p /dev/ttyACM2
```

At each stage, verify:
- [ ] No crash on boot
- [ ] Heap is stable
- [ ] micro-ROS connects and stays connected
- [ ] Previous-stage subsystems still work
- [ ] New subsystem initializes correctly

### 14. RC Teleop Test

```bash
# In Docker container:
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run secbot_drone_teleop drone_teleop_node
```

- [ ] Node starts, prints "Drone teleop ready"
- [ ] RC data received (requires robot Teensy publishing `/mcu_robot/rc`)
- [ ] SWA toggle → arm/disarm service called
- [ ] SWB toggle (while armed) → takeoff/land service called
- [ ] Sticks produce cmd_vel when flying (right stick = roll/pitch, left stick = alt/yaw)
- [ ] Deadzone: sticks near center produce zero cmd_vel
- [ ] cmd_vel stops publishing when RC signal lost (>500ms stale)

## Common Failures and Causes

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| WiFi won't connect | AP not running, wrong SSID/password, IP conflict | Check AP, verify credentials |
| No heartbeat topic | UDP agent not running, firewall on port 8888 | Start agent, check port |
| OTA fails | Drone not on WiFi, wrong IP | Verify WiFi, use USB |
| Reboot loop | Power supply issue, flash corruption, I2C hang | Re-flash USB, check voltage |
| IMU FAIL | BNO085 not found on I2C (0x4A), bad wiring, no reset pulse | Check wiring, verify I2C with scanner |
| IMU yaw drifts | BNO085 not calibrated, magnetic interference | Calibrate (figure-8 motion), move away from motors |
| IMU rate low | I2C mutex contention, bus speed too slow | Check 400kHz, verify no other I2C devices |
| IMU hangs bus | Unbounded getSensorEvent() loop draining SHTP backlog | Fixed: capped at 10 events per update (GyroSubsystem.cpp) |
| Height FAIL | VL53L0X not found on I2C (0x29), bad wiring | Check wiring, verify I2C with scanner |
| Height reads 0 | Surface too close/far, reflective surface | Aim at non-reflective surface 3-400cm away (30mm deadzone) |
| Service init FAIL | `esp32_microros_wifi` missing `board_microros_user_meta` | Fixed: added `custom_microros.meta` to esp32_microros_wifi base. After changing, delete `libs_external/esp32/micro_ros_platformio/libmicroros/` and rebuild |
| uROS stuck in WAIT | micro-ROS entity limits too low (default MAX_SERVICES) | Ensure `custom_microros.meta` is applied to ESP32 WiFi builds; delete prebuilt lib and rebuild |
| Motors don't spin | ESC not powered, wrong GPIO pin, PWM config | Check ESC power, verify pin assignments |
| Motor wrong direction | ESC wiring swapped, motor phase | Swap any 2 motor wires |
| UWB FAIL | DW3000 not found on SPI, wrong CS pin | Check SPI wiring, verify CS=GPIO21 |
| UWB no ranges | Beacons not running, too far away | Start beacons, move closer |
| EKF position wrong | Anchors not configured, bad trilateration geometry | Call set_anchors, ensure non-collinear placement |
| Agent won't connect | Wrong port, zombie agent | Kill and restart agent |
| Heap decreasing | micro-ROS leak, malloc without free | Monitor with debug output, check g_mr.update() |
| Emergency land unexpectedly | Height timeout, micro-ROS timeout | Check sensor wiring, agent connection |

## Checklist Template

```
Date: _______________
Git commit: _______________
Firmware target: [ ] drone  [ ] esp32-test-all-drone-subsystems
Tester: _______________

Boot:
  [ ] Serial messages correct
  [ ] WiFi connected (192.168.4.25)
  [ ] OTA ready

Subsystem Init:
  [ ] IMU: OK / FAIL  (yaw at power-on: _____ deg)
  [ ] IMU 3000ms delay completed
  [ ] Height: OK / FAIL / DISABLED
  [ ] Flight: OK / FAIL
  [ ] EKF: OK / FAIL
  [ ] IR: OK / FAIL
  [ ] UWB: OK / FAIL / DISABLED

RTOSSubsystem Tasks:
  [ ] All "[RTOS] Task created OK" messages present
  [ ] No "[RTOS] FAIL" messages

micro-ROS:
  [ ] Agent connects
  [ ] Heartbeat topic visible

Rates:
  heartbeat:    _____ Hz (expect 1)
  state:        _____ Hz (expect 10)
  uwb/ranging:  _____ Hz (expect 20)

Serial Debug:
  [ ] Production build (drone env): no serial output
  [ ] Test build: debug dashboard prints every 2s

IMU Orientation:
  [ ] Flat: roll~0, pitch~0
  [ ] 90 CW rotation: yaw changes ~90
  [ ] BNO X-axis forward direction: _______________

Motor Test (props removed!):
  [ ] FR spins (M1, pin D0, CW)
  [ ] BR spins (M2, pin D1, CCW)
  [ ] FL spins (M3, pin D2, CCW)
  [ ] BL spins (M4, pin D3, CW)

State Machine:
  [ ] INIT → UNARMED
  [ ] UNARMED → ARMED
  [ ] Disarm works

Safety:
  [ ] Sensor readiness correct

OTA:
  [ ] OTA flash works
  [ ] Reconnects after OTA

Stability (60s+):
  [ ] No crashes
  [ ] WiFi stable
  [ ] Heap stable
  [ ] Rates stable

Issues Found:
  _______________________________________________
  _______________________________________________
```

## Native Algorithm Tests

The drone control algorithms have native test suites that run without hardware:

```bash
# Motor mixer (X-quad mixing formula)
pio test -e test-drone-motor-mixer

# EKF (predict, trilateration, update, outlier rejection)
pio test -e test-drone-ekf

# IMU math (quaternion→Euler, world-frame rotation)
pio test -e test-drone-imu-math

# PID controller (shared with robot)
pio test -e test-control-pid

# Filters (LowPass1P used by altitude velocity)
pio test -e test-utils-filters
```

Run all native tests to verify algorithm correctness before flashing hardware.
