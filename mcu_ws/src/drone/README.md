# Drone Firmware

Firmware for the SEC26 quadcopter drone. Implements cascaded PID flight control, UWB positioning, altitude hold, and IR transmission for the antenna color task.

## Hardware

- **MCU:** Seeed XIAO ESP32-S3 (dual-core 240 MHz, 8 MB PSRAM)
- **Transport:** WiFi UDP micro-ROS to Pi agent (port 8888)
- **Static IP:** 192.168.4.25
- **OTA hostname:** `sec26-drone`

### Peripherals

| Component | Model | Protocol | Address/Pin |
|-----------|-------|----------|-------------|
| Flight motors (x4) | Brushless ESC | LEDC PWM 20kHz 10-bit | D0(1), D1(2), D2(42), D3(41) |
| IMU | BNO085 (9-axis) | I2C 400kHz | 0x4A on D4/D5, RST=D7, INT=GPIO22 |
| Altitude | VL53L0X (ToF) | I2C (shared bus) | 0x29 on D4/D5 |
| UWB Tag | DW3000 | SPI | CS=GPIO21 |
| IR Transmitter | NEC LED | GPIO | D6 (38kHz carrier) |

### Motor Layout (X-Quad)

```
  FL (CCW, M3, D2)    FR (CW, M1, D0)
            \          /
             [DRONE]
            /          \
  BL (CW, M4, D3)    BR (CCW, M2, D1)
```

## Building and Flashing

All commands run inside the Docker container.

```bash
cd /home/ubuntu/mcu_workspaces/sec26mcu

# Build
pio run -e drone

# Flash via USB
pio run -e drone --target upload

# Flash via OTA
pio run -e drone --target upload --upload-port 192.168.4.25

# Serial monitor
pio device monitor -e drone

# Bench test build (no height/UWB)
pio run -e drone-bench
```

### Build Flags

| Flag | Default | Description |
|------|---------|-------------|
| `DRONE_ENABLE_UWB` | 0 | Set to 1 if DW3000 hardware installed |
| `DRONE_ENABLE_HEIGHT` | 1 | Set to 0 to disable VL53L0X |
| `DRONE_DEBUG_STAGE` | 2 | 0=WiFi only, 1=+IMU, 2=full system |
| `DRONE_SERIAL_DEBUG` | (unset) | Uncomment for USB CDC debug dashboard |

## ROS2 Interface

### Publishers

| Topic | Message Type | QoS | Rate | Description |
|-------|-------------|-----|------|-------------|
| `/mcu_drone/state` | `mcu_msgs/DroneState` | Reliable | 10 Hz | Flight state, attitude (roll/pitch/yaw deg), altitude (m), position (m) |
| `/mcu_drone/heartbeat` | `std_msgs/String` | Best-effort | 1 Hz | Alive signal with reset reason |
| `/mcu_drone/uwb/ranging` | `mcu_msgs/UWBRanging` | Best-effort | 20 Hz | UWB ranges to beacons (tag ID=15) |
| `/mcu_drone/debug` | `std_msgs/String` | Best-effort | On-demand | Debug messages, config dumps |

### Subscribers

| Topic | Message Type | Rate | Description |
|-------|-------------|------|-------------|
| `/mcu_drone/cmd_vel` | `geometry_msgs/Twist` | 10+ Hz | Velocity command. `linear.x`=pitch, `linear.y`=roll, `linear.z`=alt_rate, `angular.z`=yaw_rate. Timeout >500ms triggers auto-hover. Only processed in VELOCITY_CONTROL state. |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/mcu_drone/arm` | `DroneArm` | Arm (`arm: true`) or disarm. Only from UNARMED state. |
| `/mcu_drone/ready_for_takeoff` | `DroneTare` | Pre-spin motors at 35%. ARMED -> READY_FOR_TAKEOFF. |
| `/mcu_drone/takeoff` | `DroneTakeoff` | Takeoff to altitude (0.3m-2.0m ceiling). |
| `/mcu_drone/land` | `DroneLand` | Descend at 0.15 m/s, disarm at <5cm. |
| `/mcu_drone/tare` | `DroneTare` | Zero roll/pitch/yaw IMU reference. |
| `/mcu_drone/set_anchors` | `DroneSetAnchors` | Configure UWB anchor 2D positions (max 4). |
| `/mcu_drone/set_motors` | `DroneSetMotors` | Direct motor override [0.0-1.0] (ARMED only, bypasses PID). |
| `/mcu_drone/transmit_ir` | `DroneTransmitIR` | Send NEC IR codes to antennas (VELOCITY_CONTROL only). |
| `/mcu_drone/set_param` | `DroneSetParam` | Runtime PID/config tuning by name. |
| `/mcu_drone/save_config` | `DroneTare` | Save all tunable params to ESP32 NVS flash. |
| `/mcu_drone/get_config` | `DroneTare` | Publish all params via debug topic. |

All service types are in the `mcu_msgs/srv/` namespace.

## State Machine

```
INIT -> UNARMED (sensors ready)
  -> ARMED (arm service)
    -> READY_FOR_TAKEOFF (ready service, motors pre-spin 35%)
      -> LAUNCHING (takeoff service, ramp at 0.3 m/s)
        -> VELOCITY_CONTROL (at 95% target altitude)
          -> LANDING (land service, descend 0.15 m/s)
            -> UNARMED (alt < 5cm)
Any flying state -> EMERGENCY_LAND (safety trigger)
```

## Usage Examples

### Full flight sequence

```bash
# 1. Tare IMU (zero reference)
ros2 service call /mcu_drone/tare mcu_msgs/srv/DroneTare

# 2. Arm motors
ros2 service call /mcu_drone/arm mcu_msgs/srv/DroneArm "{arm: true}"

# 3. Pre-spin (optional, smooths takeoff)
ros2 service call /mcu_drone/ready_for_takeoff mcu_msgs/srv/DroneTare

# 4. Takeoff to 0.5m
ros2 service call /mcu_drone/takeoff mcu_msgs/srv/DroneTakeoff "{target_altitude: 0.5}"

# 5. Send velocity commands (hold rate >2 Hz to avoid timeout)
ros2 topic pub --rate 10 /mcu_drone/cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}"

# 6. Land
ros2 service call /mcu_drone/land mcu_msgs/srv/DroneLand

# Emergency disarm (any time)
ros2 service call /mcu_drone/arm mcu_msgs/srv/DroneArm "{arm: false}"
```

### Monitor state

```bash
ros2 topic echo /mcu_drone/state
ros2 topic hz /mcu_drone/state    # Expect ~10 Hz

# Watch state transitions
ros2 topic echo /mcu_drone/state --field state

# Watch attitude
ros2 topic echo /mcu_drone/state --field roll --field pitch --field yaw
```

### UWB anchor configuration

```bash
ros2 service call /mcu_drone/set_anchors mcu_msgs/srv/DroneSetAnchors \
  "{anchor_ids: [10, 11, 12], anchor_x: [0.0, 3.0, 0.0], anchor_y: [0.0, 0.0, 3.0], num_anchors: 3}"
```

### Direct motor test (bench only)

```bash
ros2 service call /mcu_drone/arm mcu_msgs/srv/DroneArm "{arm: true}"
ros2 service call /mcu_drone/set_motors mcu_msgs/srv/DroneSetMotors \
  "{motor_speeds: [0.1, 0.1, 0.1, 0.1]}"
```

### IR transmission

```bash
# Colors: 1=RED, 2=GREEN, 3=BLUE, 4=PURPLE
ros2 service call /mcu_drone/transmit_ir mcu_msgs/srv/DroneTransmitIR \
  "{antenna_colors: [1, 2, 3, 1]}"
```

## PID Tuning

### Runtime parameter adjustment

```bash
ros2 service call /mcu_drone/set_param mcu_msgs/srv/DroneSetParam \
  "{param_name: 'hover_throttle', value: 0.50}"

# Save to flash (persists across reboots)
ros2 service call /mcu_drone/save_config mcu_msgs/srv/DroneTare

# Dump current config to debug topic
ros2 service call /mcu_drone/get_config mcu_msgs/srv/DroneTare
ros2 topic echo /mcu_drone/debug --once
```

### Available parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `roll_angle_kp/ki/kd` | 5.0 / 0.5 / 0.0 | Roll angle PID (outer loop) |
| `pitch_angle_kp/ki/kd` | 5.0 / 0.5 / 0.0 | Pitch angle PID (outer loop) |
| `roll_rate_kp/ki/kd` | 0.003 / 0.001 / 0.00003 | Roll rate PID (inner loop) |
| `pitch_rate_kp/ki/kd` | 0.003 / 0.001 / 0.00003 | Pitch rate PID (inner loop) |
| `yaw_rate_kp/ki/kd` | 0.005 / 0.001 / 0.0 | Yaw rate PID |
| `altitude_kp/ki/kd` | 1.2 / 0.15 / 0.0 | Altitude position PID |
| `alt_vel_kp/ki/kd` | 0.5 / 0.0 / 0.0 | Altitude velocity PID |
| `hover_throttle` | 0.45 | Baseline throttle for hover |
| `motor_max_slew` | 0.5 | Max motor change per second |
| `max_roll_deg` | 30.0 | Maximum roll angle limit |
| `max_pitch_deg` | 30.0 | Maximum pitch angle limit |
| `max_yaw_rate_dps` | 160.0 | Maximum yaw rate limit |
| `motor_min_output` | 0.045 | ESC dead zone threshold (~4.5% duty) |
| `ready_throttle` | 0.35 | Throttle in READY_FOR_TAKEOFF state |

### Tuning order (always inner loops first)

1. **Rate PIDs** (`roll_rate_kp/ki/kd`, `pitch_rate_kp/ki/kd`)
2. **Angle PIDs** (`roll_angle_kp/ki/kd`, `pitch_angle_kp/ki/kd`)
3. **Yaw rate** (`yaw_rate_kp/ki/kd`)
4. **Altitude velocity** (`alt_vel_kp`) then **altitude position** (`altitude_kp/ki`)

### PID Tuner TUI

```bash
ros2 run secbot_drone_teleop drone_pid_tuner.py
```

| Key | Action |
|-----|--------|
| `j`/`k` | Navigate params |
| `h`/`l` | Fine adjust (1 step) |
| `[`/`]` | Coarse adjust (5x step) |
| `Enter` | Type exact value |
| `c` | Copy roll gains to pitch |
| `S` | Save config to EEPROM |
| `a`/`d` | Arm / Disarm |
| `t`/`L` | Takeoff / Land |
| `T` | Tare IMU yaw |
| `q` | Quit |

### RC Teleop

```bash
ros2 run secbot_drone_teleop drone_teleop_node
```

| RC Input | Action |
|----------|--------|
| SWA (rising edge) | Toggle arm/disarm |
| SWB (rising edge) | Toggle takeoff/land |
| SWC (rising edge) | Tare IMU yaw |
| Right stick X/Y | Roll / Pitch |
| Left stick Y/X | Altitude rate / Yaw rate |

## Architecture

### FreeRTOS Tasks

| Task | Priority | Core | Rate | Stack | Description |
|------|----------|------|------|-------|-------------|
| GyroSubsystem | 5 | 1 | 250 Hz | 2048 | BNO085 I2C read, Euler extraction |
| DroneFlightSubsystem | 4 | 1 | 250 Hz | 4096 | Cascaded PID, motor mixer, PWM write |
| HeightSubsystem | 3 | 1 | 50 Hz | 2048 | VL53L0X altitude read |
| DroneUWBSubsystem | 3 | 0 | 20 Hz | 2048 | UWB SPI ranging, EKF update |
| DroneSafetySubsystem | 3 | any | 10 Hz | 2048 | Crash/stall/timeout detection |
| DroneStateSubsystem | 2 | any | 10 Hz | 2048 | State machine, ROS2 pub/sub |
| MicrorosManager | 2 | any | 100 Hz | 8192 | micro-ROS lifecycle, deferred publish |
| HeartbeatSubsystem | 1 | any | 1 Hz | 2048 | Heartbeat publisher |
| WiFi + OTA | 1 | any | 10 Hz | 4096 | WiFi reconnect, ArduinoOTA |

### Cascaded PID Control

```
cmd_vel -> [Angle PID] -> desired rate -> [Rate PID] -> motor correction
                                                              |
target altitude -> [Altitude PID] -> alt rate -> [Alt Vel PID] -> throttle
                                                                     |
                                                       [X-Quad Mixer] -> 4x motor PWM
```

### Safety Checks (10 Hz)

| Condition | Action |
|-----------|--------|
| IMU not initialized | Disarm |
| Height sensor stale >200ms | Emergency land |
| Roll/pitch > 60 deg or inverted | Disarm |
| Angular rate > 400 deg/s | Disarm (tumbling) |
| Motor >30% + rate <5 deg/s for 500ms | Disarm (stalled) |
| micro-ROS disconnect >3s | Emergency land |
| cmd_vel timeout >500ms | Auto-hover |
| Launch timeout >10s | Emergency land |

### EKF (4-State Position Estimator)

State: `[x, y, vx, vy]`
- Predict: world-frame accelerometer integration
- Update: per-range UWB scalar Kalman filter
- Process noise: position 0.01 m2, velocity 0.1 (m/s)2
- Measurement noise: 0.15 m2 per UWB range
- Outlier gate: Mahalanobis chi2 = 9.0

### NVS Parameter Persistence

All 28 tunable parameters are stored in ESP32 NVS flash under namespace `"drone_cfg"` with index-based keys (`"p0"` through `"p27"`). On boot, saved values auto-load before tasks start. On first micro-ROS connect, current config publishes via debug topic in `"CONFIG:name=val,..."` format.
