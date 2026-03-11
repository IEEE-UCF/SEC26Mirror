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
| `motor_max_slew` | 5.0 | Max motor change per second (brownout protection) |
| `max_roll_deg` | 30.0 | Maximum roll angle limit |
| `max_pitch_deg` | 30.0 | Maximum pitch angle limit |
| `max_yaw_rate_dps` | 160.0 | Maximum yaw rate limit |

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
