# Drone ROS2 Interface

All drone topics and services use the `/mcu_drone/` namespace.
Transport: WiFi UDP to Pi (192.168.4.1:8888).

## State Machine

```
INIT → UNARMED → ARMED → LAUNCHING → VELOCITY_CONTROL → LANDING → UNARMED
                    ↓                        ↓
               EMERGENCY_LAND          EMERGENCY_LAND
```

## Services

```bash
# Arm / disarm (only from UNARMED)
ros2 service call /mcu_drone/arm mcu_msgs/srv/DroneArm "{arm: true}"
ros2 service call /mcu_drone/arm mcu_msgs/srv/DroneArm "{arm: false}"

# Takeoff to altitude in meters (only from ARMED, clamped 0.3m - ceiling)
ros2 service call /mcu_drone/takeoff mcu_msgs/srv/DroneTakeoff "{target_altitude: 0.5}"

# Land (from LAUNCHING, VELOCITY_CONTROL, or LANDING)
ros2 service call /mcu_drone/land mcu_msgs/srv/DroneLand

# Tare IMU yaw to current heading (any state, requires IMU ready)
ros2 service call /mcu_drone/tare mcu_msgs/srv/DroneTare

# Set UWB anchor positions for EKF (any state)
ros2 service call /mcu_drone/set_anchors mcu_msgs/srv/DroneSetAnchors \
  "{anchor_ids: [10, 11, 12], anchor_x: [0.0, 3.0, 0.0], anchor_y: [0.0, 0.0, 3.0], num_anchors: 3}"

# Direct motor override for bench testing (ARMED only, not flying)
ros2 service call /mcu_drone/set_motors mcu_msgs/srv/DroneSetMotors \
  "{motor_speeds: [0.1, 0.1, 0.1, 0.1]}"

# Transmit IR to antennas (VELOCITY_CONTROL only)
ros2 service call /mcu_drone/transmit_ir mcu_msgs/srv/DroneTransmitIR \
  "{antenna_colors: [1, 2, 3, 1]}"

# Set flight parameter at runtime (PID gains, hover throttle, limits, etc.)
ros2 service call /mcu_drone/set_param mcu_msgs/srv/DroneSetParam \
  "{param_name: 'hover_throttle', value: 0.50}"
```

### Available Parameters for set_param

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
| `motor_max_slew` | 0.5 | Max motor change per second (brownout protection) |
| `max_roll_deg` | 30.0 | Maximum roll angle limit |
| `max_pitch_deg` | 30.0 | Maximum pitch angle limit |
| `max_yaw_rate_dps` | 160.0 | Maximum yaw rate limit |

### Apply All Defaults

```bash
# Edit values and paste into terminal:
ros2 service call /mcu_drone/set_param mcu_msgs/srv/DroneSetParam "{param_name: 'roll_angle_kp', value: 1.0}"
ros2 service call /mcu_drone/set_param mcu_msgs/srv/DroneSetParam "{param_name: 'roll_angle_ki', value: 0.5}"
ros2 service call /mcu_drone/set_param mcu_msgs/srv/DroneSetParam "{param_name: 'roll_angle_kd', value: 0.0}"
ros2 service call /mcu_drone/set_param mcu_msgs/srv/DroneSetParam "{param_name: 'pitch_angle_kp', value: 1.0}"
ros2 service call /mcu_drone/set_param mcu_msgs/srv/DroneSetParam "{param_name: 'pitch_angle_ki', value: 0.5}"
ros2 service call /mcu_drone/set_param mcu_msgs/srv/DroneSetParam "{param_name: 'pitch_angle_kd', value: 0.0}"
ros2 service call /mcu_drone/set_param mcu_msgs/srv/DroneSetParam "{param_name: 'roll_rate_kp', value: 0.003}"
ros2 service call /mcu_drone/set_param mcu_msgs/srv/DroneSetParam "{param_name: 'roll_rate_ki', value: 0.001}"
ros2 service call /mcu_drone/set_param mcu_msgs/srv/DroneSetParam "{param_name: 'roll_rate_kd', value: 0.00003}"
ros2 service call /mcu_drone/set_param mcu_msgs/srv/DroneSetParam "{param_name: 'pitch_rate_kp', value: 0.003}"
ros2 service call /mcu_drone/set_param mcu_msgs/srv/DroneSetParam "{param_name: 'pitch_rate_ki', value: 0.001}"
ros2 service call /mcu_drone/set_param mcu_msgs/srv/DroneSetParam "{param_name: 'pitch_rate_kd', value: 0.00003}"
ros2 service call /mcu_drone/set_param mcu_msgs/srv/DroneSetParam "{param_name: 'yaw_rate_kp', value: 0.005}"
ros2 service call /mcu_drone/set_param mcu_msgs/srv/DroneSetParam "{param_name: 'yaw_rate_ki', value: 0.001}"
ros2 service call /mcu_drone/set_param mcu_msgs/srv/DroneSetParam "{param_name: 'yaw_rate_kd', value: 0.0}"
ros2 service call /mcu_drone/set_param mcu_msgs/srv/DroneSetParam "{param_name: 'altitude_kp', value: 0.7}"
ros2 service call /mcu_drone/set_param mcu_msgs/srv/DroneSetParam "{param_name: 'altitude_ki', value: 0.15}"
ros2 service call /mcu_drone/set_param mcu_msgs/srv/DroneSetParam "{param_name: 'altitude_kd', value: 0.0}"
ros2 service call /mcu_drone/set_param mcu_msgs/srv/DroneSetParam "{param_name: 'alt_vel_kp', value: 0.5}"
ros2 service call /mcu_drone/set_param mcu_msgs/srv/DroneSetParam "{param_name: 'alt_vel_ki', value: 0.0}"
ros2 service call /mcu_drone/set_param mcu_msgs/srv/DroneSetParam "{param_name: 'alt_vel_kd', value: 0.0}"
ros2 service call /mcu_drone/set_param mcu_msgs/srv/DroneSetParam "{param_name: 'hover_throttle', value: 0.45}"
ros2 service call /mcu_drone/set_param mcu_msgs/srv/DroneSetParam "{param_name: 'motor_max_slew', value: 0.5}"
ros2 service call /mcu_drone/set_param mcu_msgs/srv/DroneSetParam "{param_name: 'max_roll_deg', value: 30.0}"
ros2 service call /mcu_drone/set_param mcu_msgs/srv/DroneSetParam "{param_name: 'max_pitch_deg', value: 30.0}"
ros2 service call /mcu_drone/set_param mcu_msgs/srv/DroneSetParam "{param_name: 'max_yaw_rate_dps', value: 160.0}"
```

## PID Tuning Guide

### Tuning Order (always inner loops first)

1. **Rate PIDs** (`roll_rate_kp/ki/kd`, `pitch_rate_kp/ki/kd`)
2. **Angle PIDs** (`roll_angle_kp/ki/kd`, `pitch_angle_kp/ki/kd`)
3. **Yaw rate** (`yaw_rate_kp/ki/kd`)
4. **Altitude velocity** (`alt_vel_kp`) then **altitude position** (`altitude_kp/ki`)

### Method

```bash
# Shorthand
alias dp='ros2 service call /mcu_drone/set_param mcu_msgs/srv/DroneSetParam'
```

1. Arm the drone, hold by hand or tether loosely
2. Zero ki and kd, start with kp only:
   ```bash
   dp "{param_name: 'roll_rate_ki', value: 0.0}"
   dp "{param_name: 'roll_rate_kd', value: 0.0}"
   ```
3. Increase `roll_rate_kp` until the drone resists rotation when tilted by hand, back off when it oscillates
4. Add `roll_rate_kd` in small increments to dampen oscillation
5. Add `roll_rate_ki` last, very small — only if there's steady-state error
6. Copy roll gains to pitch (symmetric frame)
7. Move to outer angle loop — same process

### Diagnostics

| Symptom | Cause | Fix |
|---------|-------|-----|
| Oscillation | kp too high or kd too low | Reduce kp or increase kd |
| Sluggish response | kp too low | Increase kp |
| Slow drift / won't hold angle | ki too low | Increase ki |
| High-frequency vibration | kd too high (amplifying noise) | Reduce kd |
| Motors saturating (0 or 1.0) | Gains too aggressive | Reduce output limits |

### Monitoring While Tuning

```bash
# Attitude and state
ros2 topic echo /mcu_drone/state

# Loop rates and debug info (every 5s)
ros2 topic echo /mcu_drone/debug
```

## Safety

- **Crash detection**: If roll or pitch exceeds 2x the max angle limits (default 60 deg) or goes past 90 deg (upside down) for 200ms while flying, motors are immediately killed.
- **Height timeout**: If VL53L0X data is stale for >200ms (except during LAUNCHING), triggers emergency land.
- **micro-ROS disconnect**: If agent is unreachable for >3s, triggers emergency land.
- **IMU failure**: Immediate disarm.

## Published Topics

```bash
# Drone state (10 Hz) — flight state, attitude, altitude, EKF position
ros2 topic echo /mcu_drone/state

# Heartbeat (1 Hz)
ros2 topic echo /mcu_drone/heartbeat

# UWB ranging (20 Hz, when UWB enabled)
ros2 topic echo /mcu_drone/uwb/ranging
```

## Subscribed Topics

```bash
# Velocity command (while in VELOCITY_CONTROL state)
#   linear.x  → pitch      linear.y  → roll
#   linear.z  → alt rate   angular.z → yaw rate
# Auto-hovers if no message received for >200ms
ros2 topic pub /mcu_drone/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
```

## RC Teleop

The `secbot_drone_teleop` node bridges RC input to drone commands:

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

## Monitoring

```bash
# Check all drone topics are publishing
for t in $(ros2 topic list | grep mcu_drone); do
  echo "--- $t ---"
  timeout 5 ros2 topic hz "$t" 2>&1 | tail -2 &
done; wait

# Watch state transitions
ros2 topic echo /mcu_drone/state --field state

# Watch attitude
ros2 topic echo /mcu_drone/state --field roll --field pitch --field yaw
```

## Motor Layout

```
  FL (CCW, M3, D2)    FR (CW, M1, D0)
            \          /
             [DRONE]
            /          \
  BL (CW, M4, D3)    BR (CCW, M2, D1)
```
