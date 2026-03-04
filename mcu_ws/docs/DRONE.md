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

### Bench Test (no props)
1. Flash: `pio run -e drone-bench --target upload`
2. Monitor: `pio device monitor -e drone-bench`
3. Verify WiFi + OTA + micro-ROS connect
4. `ros2 topic echo /mcu_drone/state` — verify state publishing
5. `ros2 service call /mcu_drone/arm mcu_msgs/srv/DroneArm "{arm: true}"`
6. `ros2 service call /mcu_drone/set_motors mcu_msgs/srv/DroneSetMotors "{motor_speeds: [0.1, 0.0, 0.0, 0.0]}"` — verify single motor
7. Disarm: `ros2 service call /mcu_drone/arm mcu_msgs/srv/DroneArm "{arm: false}"`

### Tethered Flight Test
1. Flash: `pio run -e drone --target upload`
2. Secure drone on tether
3. Arm and takeoff to 0.5m: `/mcu_drone/takeoff {target_altitude: 0.5}`
4. Verify altitude hold
5. Test cmd_vel: `ros2 topic pub /mcu_drone/cmd_vel geometry_msgs/Twist "{}"`
6. Land: `/mcu_drone/land`

### Safety Test
1. While hovering, disconnect micro-ROS agent
2. Verify emergency land after 3s timeout
3. Verify motors disarm on ground

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
