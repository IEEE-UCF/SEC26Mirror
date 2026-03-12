# Drone Firmware Documentation

## Hardware Overview

**MCU**: Seeed XIAO ESP32-S3
**Sensors**: BNO085 IMU (I2C), VL53L0X ToF altimeter (I2C), DW3000 UWB tag (SPI)
**Actuators**: 4x brushless ESC via PWM (20kHz, 10-bit)
**Communication**: IR NEC transmitter (38kHz)
**Network**: WiFi 192.168.4.25, micro-ROS UDP to Pi agent

### Pinout

| Pin | Function |
|-----|----------|
| D0 | Motor FL (CCW) |
| D1 | Motor FR (CW) |
| D2 | Motor BR (CCW) |
| D3 | Motor BL (CW) |
| D4 | I2C SDA (BNO085 + VL53L0X) |
| D5 | I2C SCL |
| D6 | IR LED |
| D7 | BNO085 RST |
| GPIO22 | BNO085 INT |
| GPIO21 | DW3000 CS (SPI) |

### I2C Bus

Both BNO085 (0x4A) and VL53L0X (0x29) share the same I2C bus (Wire on D4/D5).
Access is protected by a FreeRTOS mutex (`g_i2c_mutex`).

## State Machine

```
INIT ──(sensors OK)──> UNARMED
UNARMED ──(/arm)──> ARMED
ARMED ──(/takeoff)──> LAUNCHING
LAUNCHING ──(alt ≥ target)──> VELOCITY_CONTROL
VELOCITY_CONTROL ──(/land)──> LANDING
LANDING ──(alt < 0.05m)──> ARMED ──(/disarm)──> UNARMED

Any flying state ──(safety failure)──> EMERGENCY_LAND ──(on ground)──> UNARMED
```

### State Descriptions

| State | Description |
|-------|-------------|
| INIT | Waiting for all sensors to initialize |
| UNARMED | Motors off, safe to handle |
| ARMED | PID ready, motors can be tested via `/set_motors` |
| LAUNCHING | Ascending to target altitude with altitude hold |
| VELOCITY_CONTROL | Full cmd_vel control with altitude hold |
| LANDING | Controlled descent at 0.15 m/s |
| EMERGENCY_LAND | Safety-triggered descent, auto-disarm |

## ROS2 Interface

### Topics

| Topic | Type | Dir | Rate | Description |
|-------|------|-----|------|-------------|
| `/mcu_drone/state` | DroneState | pub | 10Hz | State + position + attitude |
| `/mcu_drone/heartbeat` | String | pub | 5Hz | Connection monitor |
| `/mcu_drone/debug` | String | pub | — | Debug messages |
| `/mcu_drone/cmd_vel` | Twist | sub | — | Velocity commands |
| `/mcu_drone/uwb/ranging` | UWBRanging | pub | 10Hz | UWB range data |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/mcu_drone/arm` | DroneArm | Arm/disarm (only from UNARMED) |
| `/mcu_drone/takeoff` | DroneTakeoff | Launch to target altitude (from ARMED) |
| `/mcu_drone/land` | DroneLand | Initiate landing (from any flying state) |
| `/mcu_drone/transmit_ir` | DroneTransmitIR | Send IR antenna colors (VELOCITY_CONTROL only) |
| `/mcu_drone/set_anchors` | DroneSetAnchors | Configure UWB anchor positions |
| `/mcu_drone/set_motors` | DroneSetMotors | Direct motor PWM 0-1 (ARMED only, bench test) |

### cmd_vel Mapping

| Twist Field | Drone Effect | Scale |
|-------------|-------------|-------|
| `linear.x` | Pitch angle (forward) | x15 deg |
| `linear.y` | Roll angle (lateral) | x15 deg |
| `linear.z` | Altitude rate | x0.5 m/s |
| `angular.z` | Yaw rate | x90 deg/s |

All zeros = hover. 500ms timeout = auto-hover.

## FreeRTOS Task Architecture

| Task | Core | Priority | Rate | Stack | Function |
|------|------|----------|------|-------|----------|
| flightControlTask | 1 | 5 | 250Hz | 4096 | IMU read, PID, motors, EKF predict |
| safetyTask | 1 | 4 | 10Hz | 2048 | Failure detection |
| heightSensorTask | 1 | 3 | 50Hz | 2048 | VL53L0X read |
| uwbTask | 0 | 3 | 20Hz | 4096 | DW3000 ranging, EKF update |
| Arduino loop() | any | 1 | — | default | WiFi, OTA, micro-ROS, state machine |

## EKF Design

**State vector**: `[x, y, vx, vy]` (4 states, 2D position + velocity)

**Predict** (250Hz, flight task):
- IMU linear acceleration (BNO085, gravity-free) rotated to world frame by yaw
- Standard kinematic model: `x += vx*dt + 0.5*ax*dt²`

**Update** (~20Hz, UWB task):
- UWB ranges trilaterated to (x,y) via linearized circle equation pairs
- 2×2 linear solve (Cramer's rule), requires ≥3 valid anchors
- Outlier gate: reject if residual > 1.0m
- Standard Kalman update with H = [I₂ | 0₂]

**Height**: Handled separately by VL53L0X + altitude PID (not in EKF).

### Tuning

EKF noise parameters in `DroneConfig.h`:
- `EKF_PROCESS_NOISE_POS`: Position process noise (m², default 0.01)
- `EKF_PROCESS_NOISE_VEL`: Velocity process noise ((m/s)², default 0.1)
- `EKF_MEASURE_NOISE_UWB`: UWB measurement noise (m², default 0.15)
- `EKF_OUTLIER_GATE_M`: Outlier rejection threshold (m, default 1.0)

## Safety System

| Failure | Detection | Response |
|---------|-----------|----------|
| VL53L0X timeout | `lastValidMs()` > 200ms | Emergency land |
| IMU failure | `isInitialized() == false` | Immediate motor disarm |
| micro-ROS disconnect | Not connected > 3s | Emergency land |
| Altitude ceiling | altitude > 2.0m | Altitude PID reduces setpoint |
| cmd_vel timeout | No cmd_vel > 500ms | Auto-hover (not emergency) |

### Emergency Land Procedure
1. Zero roll/pitch/yaw commands
2. If height sensor valid: controlled descent at 0.15 m/s
3. If height sensor invalid: fixed throttle 0.25 (below hover)
4. Auto-disarm after 5s timeout or when altitude < 0.05m

## Build Environments

```bash
# Full drone (all sensors)
pio run -e drone

# Without UWB module
pio run -e drone-no-uwb

# Bench test (no UWB, no height sensor)
pio run -e drone-bench
```

### Build Flags

| Flag | Default | Description |
|------|---------|-------------|
| `DRONE_ENABLE_UWB` | 1 | Enable UWB ranging + EKF updates |
| `DRONE_ENABLE_HEIGHT` | 1 | Enable VL53L0X altitude sensor |
| `SERIAL_DEBUG` | defined | Enable debug serial output |

### After Changing mcu_msgs

```bash
cd /home/ubuntu/ros2_workspaces && colcon build --packages-select mcu_msgs
cd /home/ubuntu/mcu_workspaces/sec26mcu
pio run -e drone -t clean_microros && pio run -e drone
```

## Testing Procedures

### Prerequisites

- micro-ROS agent running on the Pi: `ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888`
- Drone powered and connected to `UCFIEEEBot` WiFi
- Serial monitor (optional): `pio device monitor -e drone`

---

### 1. Connectivity Check

Flash and verify basic connectivity:

```bash
pio run -e drone --target upload
```

**Verify WiFi + micro-ROS:**
```bash
# Should see state publishing at 10Hz
ros2 topic echo /mcu_drone/state

# Should see "HEARTBEAT" at 5Hz
ros2 topic echo /mcu_drone/heartbeat
```

**Verify state is UNARMED (state=1):**
```bash
ros2 topic echo /mcu_drone/state --field state
# Expected: 1 (UNARMED) — if 0 (INIT), a sensor failed to initialize
```

**List all drone topics:**
```bash
ros2 topic list | grep mcu_drone
# /mcu_drone/state
# /mcu_drone/heartbeat
# /mcu_drone/debug
# /mcu_drone/cmd_vel
# /mcu_drone/uwb/ranging
```

**List all drone services:**
```bash
ros2 service list | grep mcu_drone
# /mcu_drone/arm
# /mcu_drone/takeoff
# /mcu_drone/land
# /mcu_drone/transmit_ir
# /mcu_drone/set_anchors
# /mcu_drone/set_motors
```

---

### 2. Service Reference

#### `/mcu_drone/arm` — Arm / Disarm

Arms the flight controller (enables PID, resets integrators). Only works from UNARMED state. Checks that IMU and height sensor (if enabled) are initialized.

```bash
# Arm
ros2 service call /mcu_drone/arm mcu_msgs/srv/DroneArm "{arm: true}"
# Expected: success=true, message="Armed"
# State changes: UNARMED → ARMED

# Disarm (only from ARMED, not while flying)
ros2 service call /mcu_drone/arm mcu_msgs/srv/DroneArm "{arm: false}"
# Expected: success=true, message="Disarmed"
# State changes: ARMED → UNARMED

# Error cases:
# - Arm when not UNARMED: success=false, "Not in UNARMED state"
# - Arm when IMU failed: success=false, "IMU not ready"
# - Arm when height sensor failed (DRONE_ENABLE_HEIGHT=1): success=false, "Height sensor not ready"
# - Disarm while flying: success=false, "Cannot disarm while flying, use land"
```

#### `/mcu_drone/takeoff` — Launch to Target Altitude

Initiates takeoff with altitude hold. Only works from ARMED state. Target altitude is clamped to 0.3m–2.0m.

```bash
# Takeoff to 1 meter
ros2 service call /mcu_drone/takeoff mcu_msgs/srv/DroneTakeoff "{target_altitude: 1.0}"
# Expected: success=true, message="Launching"
# State changes: ARMED → LAUNCHING → VELOCITY_CONTROL (when altitude reached)

# Takeoff to 0.5m (safer for testing)
ros2 service call /mcu_drone/takeoff mcu_msgs/srv/DroneTakeoff "{target_altitude: 0.5}"

# Error cases:
# - Not armed: success=false, "Not in ARMED state"
```

#### `/mcu_drone/land` — Initiate Landing

Starts a controlled descent at 0.15 m/s. Works from any flying state (LAUNCHING, VELOCITY_CONTROL, LANDING).

```bash
ros2 service call /mcu_drone/land mcu_msgs/srv/DroneLand "{}"
# Expected: success=true, message="Landing"
# State changes: (any flying) → LANDING → UNARMED (when alt < 0.05m)

# Error cases:
# - Not flying: success=false, "Not flying"
```

#### `/mcu_drone/set_motors` — Direct Motor PWM Override

Directly sets motor PWM values (0.0–1.0) for bench testing. **Only works in ARMED state** (not while flying). Bypasses all PID control. Use this to verify motor direction and spin-up.

```bash
# Spin front-left motor at 10%
ros2 service call /mcu_drone/set_motors mcu_msgs/srv/DroneSetMotors \
  "{motor_speeds: [0.1, 0.0, 0.0, 0.0]}"
# Motor order: [FL, FR, BR, BL]

# Spin all motors at 5%
ros2 service call /mcu_drone/set_motors mcu_msgs/srv/DroneSetMotors \
  "{motor_speeds: [0.05, 0.05, 0.05, 0.05]}"

# Spin front-right only
ros2 service call /mcu_drone/set_motors mcu_msgs/srv/DroneSetMotors \
  "{motor_speeds: [0.0, 0.1, 0.0, 0.0]}"

# Spin back-right only
ros2 service call /mcu_drone/set_motors mcu_msgs/srv/DroneSetMotors \
  "{motor_speeds: [0.0, 0.0, 0.1, 0.0]}"

# Spin back-left only
ros2 service call /mcu_drone/set_motors mcu_msgs/srv/DroneSetMotors \
  "{motor_speeds: [0.0, 0.0, 0.0, 0.1]}"

# Stop override (disarm and re-arm, or takeoff clears it)
ros2 service call /mcu_drone/arm mcu_msgs/srv/DroneArm "{arm: false}"

# Error cases:
# - Not in ARMED state: success=false, "Motor override only in ARMED (not flying)"
```

#### `/mcu_drone/transmit_ir` — Send IR Antenna Colors

Transmits NEC IR codes for 4 antenna colors to the Earth receiver. **Only works in VELOCITY_CONTROL state** (must be hovering).

```bash
# Color codes: 0x09=Red, 0x0A=Green, 0x0C=Blue, 0x0F=Purple, 0x00=skip
# Array: [antenna1, antenna2, antenna3, antenna4]

# All red
ros2 service call /mcu_drone/transmit_ir mcu_msgs/srv/DroneTransmitIR \
  "{antenna_colors: [9, 9, 9, 9]}"

# Mixed colors (Red, Green, Blue, Purple)
ros2 service call /mcu_drone/transmit_ir mcu_msgs/srv/DroneTransmitIR \
  "{antenna_colors: [9, 10, 12, 15]}"

# Skip antenna 3 (set to 0)
ros2 service call /mcu_drone/transmit_ir mcu_msgs/srv/DroneTransmitIR \
  "{antenna_colors: [9, 10, 0, 15]}"

# Error cases:
# - Not in VELOCITY_CONTROL: success=false
```

#### `/mcu_drone/set_anchors` — Configure UWB Anchor Positions

Sets UWB anchor positions for the EKF trilateration. Call this before takeoff to configure the field layout.

```bash
# 3 anchors at field corners (positions in meters)
ros2 service call /mcu_drone/set_anchors mcu_msgs/srv/DroneSetAnchors \
  "{anchor_ids: [10, 11, 12, 0], num_anchors: 3, anchor_x: [0.0, 3.66, 0.0, 0.0], anchor_y: [0.0, 0.0, 3.66, 0.0]}"

# 4 anchors
ros2 service call /mcu_drone/set_anchors mcu_msgs/srv/DroneSetAnchors \
  "{anchor_ids: [10, 11, 12, 13], num_anchors: 4, anchor_x: [0.0, 3.66, 3.66, 0.0], anchor_y: [0.0, 0.0, 3.66, 3.66]}"
```

---

### 3. Topic Reference

#### `/mcu_drone/cmd_vel` — Velocity Commands

Publish `geometry_msgs/Twist` to control the drone in VELOCITY_CONTROL state. All zeros = hover. Stops after 500ms without a message (auto-hover).

```bash
# Hover in place (all zeros)
ros2 topic pub /mcu_drone/cmd_vel geometry_msgs/msg/Twist "{}" --rate 10

# Move forward (pitch)
ros2 topic pub /mcu_drone/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {z: 0.0}}" --rate 10
# linear.x=0.5 → pitch = 0.5 * 15 = 7.5 degrees forward

# Move right (roll)
ros2 topic pub /mcu_drone/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.5, z: 0.0}, angular: {z: 0.0}}" --rate 10
# linear.y=0.5 → roll = 0.5 * 15 = 7.5 degrees right

# Ascend
ros2 topic pub /mcu_drone/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.5}, angular: {z: 0.0}}" --rate 10
# linear.z=0.5 → altitude rate = 0.5 * 0.5 = 0.25 m/s up

# Yaw left (CCW)
ros2 topic pub /mcu_drone/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.5}}" --rate 10
# angular.z=0.5 → yaw rate = 0.5 * 90 = 45 deg/s CCW

# STOP publishing to trigger auto-hover (500ms timeout)
# Press Ctrl+C
```

#### `/mcu_drone/state` — State Feedback

```bash
# Full state message
ros2 topic echo /mcu_drone/state

# Just the state enum
ros2 topic echo /mcu_drone/state --field state
# 0=INIT, 1=UNARMED, 2=ARMED, 3=LAUNCHING, 4=VELOCITY_CONTROL, 5=LANDING, 6=EMERGENCY_LAND

# Just altitude
ros2 topic echo /mcu_drone/state --field altitude

# Just position (EKF)
ros2 topic echo /mcu_drone/state --field pos_x
ros2 topic echo /mcu_drone/state --field pos_y

# Attitude
ros2 topic echo /mcu_drone/state --field roll
ros2 topic echo /mcu_drone/state --field pitch
ros2 topic echo /mcu_drone/state --field yaw
```

#### `/mcu_drone/uwb/ranging` — UWB Range Data

```bash
ros2 topic echo /mcu_drone/uwb/ranging
# Shows tag_id, num_anchors, temperature, and array of ranges (anchor_id, distance in cm, valid)
```

---

### 4. Bench Test Procedure (No Props!)

For motor and sensor verification without flying. Use `drone-bench` env to skip sensor init checks.

```bash
pio run -e drone-bench --target upload
```

**Step-by-step:**

```bash
# 1. Verify connectivity
ros2 topic echo /mcu_drone/state --once
# Expect state=1 (UNARMED)

# 2. Arm
ros2 service call /mcu_drone/arm mcu_msgs/srv/DroneArm "{arm: true}"
# Expect success=true, state changes to 2 (ARMED)

# 3. Test each motor individually (REMOVE PROPS FIRST!)
ros2 service call /mcu_drone/set_motors mcu_msgs/srv/DroneSetMotors \
  "{motor_speeds: [0.08, 0.0, 0.0, 0.0]}"
# Verify FL spins. Wait 2 seconds, then:

ros2 service call /mcu_drone/set_motors mcu_msgs/srv/DroneSetMotors \
  "{motor_speeds: [0.0, 0.08, 0.0, 0.0]}"
# Verify FR spins. Then:

ros2 service call /mcu_drone/set_motors mcu_msgs/srv/DroneSetMotors \
  "{motor_speeds: [0.0, 0.0, 0.08, 0.0]}"
# Verify BR spins. Then:

ros2 service call /mcu_drone/set_motors mcu_msgs/srv/DroneSetMotors \
  "{motor_speeds: [0.0, 0.0, 0.0, 0.08]}"
# Verify BL spins.

# 4. Verify motor directions:
#    FL = CCW, FR = CW, BR = CCW, BL = CW
#    If wrong, swap motor wires or ESC direction setting

# 5. Disarm
ros2 service call /mcu_drone/arm mcu_msgs/srv/DroneArm "{arm: false}"
```

---

### 5. Sensor Verification

```bash
# Use full drone build for sensor checks
pio run -e drone --target upload
```

**IMU check:**
```bash
# Watch attitude angles while tilting the drone by hand
ros2 topic echo /mcu_drone/state --field roll
ros2 topic echo /mcu_drone/state --field pitch
ros2 topic echo /mcu_drone/state --field yaw
# Should respond smoothly to tilting
```

**Height sensor check:**
```bash
# Hold drone at various heights, watch altitude
ros2 topic echo /mcu_drone/state --field altitude
# Should read 0.0-4.0m accurately. Move hand under sensor to verify.
```

**UWB check:**
```bash
# With beacons powered and running
ros2 topic echo /mcu_drone/uwb/ranging
# Should show ranges to beacon IDs 10, 11, 12

# Set anchor positions and check EKF
ros2 service call /mcu_drone/set_anchors mcu_msgs/srv/DroneSetAnchors \
  "{anchor_ids: [10, 11, 12, 0], num_anchors: 3, anchor_x: [0.0, 3.66, 0.0, 0.0], anchor_y: [0.0, 0.0, 3.66, 0.0]}"

# Watch EKF position estimate
ros2 topic echo /mcu_drone/state --field pos_x
ros2 topic echo /mcu_drone/state --field pos_y
```

---

### 6. Tethered Flight Test

**Setup**: Secure drone on a tether that limits altitude to ~1m. Props ON.

```bash
pio run -e drone --target upload
```

```bash
# 1. Verify UNARMED
ros2 topic echo /mcu_drone/state --field state --once

# 2. Arm
ros2 service call /mcu_drone/arm mcu_msgs/srv/DroneArm "{arm: true}"

# 3. Takeoff to 0.5m
ros2 service call /mcu_drone/takeoff mcu_msgs/srv/DroneTakeoff "{target_altitude: 0.5}"
# Watch state: 3 (LAUNCHING) → 4 (VELOCITY_CONTROL)

# 4. Monitor altitude hold
ros2 topic echo /mcu_drone/state --field altitude
# Should hover at ~0.5m

# 5. Test hover (all zeros cmd_vel)
ros2 topic pub /mcu_drone/cmd_vel geometry_msgs/msg/Twist "{}" --rate 10

# 6. Gentle forward pitch
ros2 topic pub /mcu_drone/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}}" --rate 10
# Ctrl+C to auto-hover

# 7. Land
ros2 service call /mcu_drone/land mcu_msgs/srv/DroneLand "{}"
# Watch state: 5 (LANDING) → 1 (UNARMED)
```

---

### 7. Safety System Tests

**micro-ROS disconnect test:**
```bash
# 1. Takeoff and hover
# 2. Kill the micro-ROS agent: Ctrl+C on the agent process
# 3. Watch serial monitor: should see EMERGENCY_LAND after 3 seconds
# 4. Drone descends and disarms automatically
# 5. Restart agent to reconnect
```

**cmd_vel timeout test:**
```bash
# 1. Takeoff and hover
# 2. Publish cmd_vel with forward pitch
ros2 topic pub /mcu_drone/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}}" --rate 10
# 3. Ctrl+C to stop publishing
# 4. After 500ms, drone should return to hover (not emergency land)
```

**Altitude ceiling test:**
```bash
# 1. Takeoff and hover
# 2. Command ascent
ros2 topic pub /mcu_drone/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {z: 1.0}}" --rate 10
# 3. Altitude should cap at 2.0m (ALTITUDE_CEILING_M in DroneConfig.h)
```

---

### 8. Full Mission Test (IR Transmit)

```bash
# 1. Takeoff
ros2 service call /mcu_drone/arm mcu_msgs/srv/DroneArm "{arm: true}"
ros2 service call /mcu_drone/takeoff mcu_msgs/srv/DroneTakeoff "{target_altitude: 1.0}"

# 2. Wait for VELOCITY_CONTROL (state=4)
ros2 topic echo /mcu_drone/state --field state
# Wait until it shows 4

# 3. Navigate to Earth element (via cmd_vel or autonomous)

# 4. Transmit IR colors
ros2 service call /mcu_drone/transmit_ir mcu_msgs/srv/DroneTransmitIR \
  "{antenna_colors: [9, 10, 12, 15]}"
# Red, Green, Blue, Purple

# 5. Land
ros2 service call /mcu_drone/land mcu_msgs/srv/DroneLand "{}"
```

---

### Quick Reference: Complete Flight Sequence

```bash
# Setup anchors (once)
ros2 service call /mcu_drone/set_anchors mcu_msgs/srv/DroneSetAnchors \
  "{anchor_ids: [10, 11, 12, 0], num_anchors: 3, anchor_x: [0.0, 3.66, 0.0, 0.0], anchor_y: [0.0, 0.0, 3.66, 0.0]}"

# Arm → Takeoff → Hover → IR → Land
ros2 service call /mcu_drone/arm mcu_msgs/srv/DroneArm "{arm: true}"
ros2 service call /mcu_drone/takeoff mcu_msgs/srv/DroneTakeoff "{target_altitude: 1.0}"
ros2 topic pub /mcu_drone/cmd_vel geometry_msgs/msg/Twist "{}" --rate 10 &
# ... fly to position ...
ros2 service call /mcu_drone/transmit_ir mcu_msgs/srv/DroneTransmitIR "{antenna_colors: [9, 10, 12, 15]}"
ros2 service call /mcu_drone/land mcu_msgs/srv/DroneLand "{}"
```

## File Structure

```
mcu_ws/src/drone/
├── main.cpp                           Entry point (includes DroneLogic.h)
├── DronePins.h                        GPIO pin assignments
├── DroneConfig.h                      PID gains, rates, thresholds
├── machines/
│   └── DroneLogic.h                   Globals, FreeRTOS tasks, setup/loop
└── subsystems/
    ├── GyroSubsystem.h/.cpp           BNO085 IMU (angles + rates + accel)
    ├── HeightSubsystem.h/.cpp         VL53L0X altitude
    ├── IRSubsystem.h/.cpp             IR NEC transmitter
    ├── DroneFlightSubsystem.h/.cpp    Cascaded PID flight controller
    ├── DroneEKFSubsystem.h/.cpp       4-state EKF + trilateration
    ├── DroneStateSubsystem.h/.cpp     State machine + ROS2 services
    ├── DroneSafetySubsystem.h         Safety monitor
    ├── DroneIRSubsystem.h             IR ROS2 service wrapper
    └── DroneUWBSubsystem.h            UWB ROS2 publisher wrapper
```

## Entity Budget

| Entity | Used | Limit |
|--------|------|-------|
| Publishers | 4 (state, heartbeat, UWB, debug) | 22 |
| Subscriptions | 1 (cmd_vel) | 16 |
| Services | 6 (arm, takeoff, land, ir, anchors, motors) | 12 |
| Executor handles | 7 (1 sub + 6 srv) | 24 |
