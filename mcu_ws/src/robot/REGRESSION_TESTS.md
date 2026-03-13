# SEC26 MCU Regression Tests

Manual hardware regression tests for `teensy-test-all-subsystems` and `robot` firmware.
Both share the same subsystem codebase — the test environment is a superset used for
isolated validation before flashing production firmware.

## Quick Reference

```bash
# Inside Docker container
cd /home/ubuntu/mcu_workspaces/sec26mcu

# Build + flash test firmware (full system)
pio run -e teensy-test-all-subsystems --target upload

# Build + flash production firmware
pio run -e robot --target upload

# Start micro-ROS agent (if entrypoint isn't running it)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyTeensyROS -b 921600

# Debug serial (Teensy SerialUSB1)
cat /dev/ttyTeensyDebug   # or screen /dev/ttyTeensyDebug

# Check all topic rates at once
for t in $(ros2 topic list | grep mcu_robot); do
  echo "--- $t ---"
  timeout 8 ros2 topic hz "$t" 2>&1 | tail -2 &
done; wait
```

## Environment Differences

| | `teensy-test-all-subsystems` | `robot` |
|---|---|---|
| Entry point | `src/test/teensy-test-all-subsystems.cpp` | `src/robot/main.cpp` -> `RobotLogic.h` |
| DEBUG_STAGE | Configurable (1-7), default 7 | Always full system |
| IntakeSubsystem | Not included | Included (20 Hz) |
| ArmSubsystem | Not included | Included (20 Hz, stub) |
| Participants | 18 (stage 7, UWB conditional) | 20 (UWB conditional) |
| RC polling | `loop()` at 5ms delay | `loop()` at 5ms delay |
| Serial debug | `SERIAL_DEBUG` defined (SerialUSB1) | `SERIAL_DEBUG` defined |

Both environments use identical subsystem code, thread priorities, stack sizes, and
update rates. If a regression passes on `teensy-test-all-subsystems`, it will pass on
`robot` (the two extra subsystems — Intake and Arm — are independent).

## Prerequisites

- Teensy 4.1 connected via USB
- Docker container running (`docker compose up -d`)
- All commands run inside the container
- Physical hardware connected (I2C buses, PCA9685 boards, BNO085, etc.)
- Kill the micro-ROS agent before flashing: `pkill -9 -f "micro_ros_agent serial"`
- First flash attempt often fails ("error writing") — retry after 3s

## Test Procedure

### 1. Flash and Boot

```bash
pkill -9 -f "micro_ros_agent serial" 2>/dev/null
sleep 3
pio run -e teensy-test-all-subsystems --target upload
```

**Verify on debug serial** (`/dev/ttyTeensyDebug`):
- [ ] No `CrashReport` output
- [ ] All `[INIT]` stages complete without errors
- [ ] `[IMU] Calibration done` message appears
- [ ] `[INIT] All threads started` message appears
- [ ] Periodic status prints show `uROS state : WAITING`

### 2. micro-ROS Connection

Start the agent and wait ~10s for the Teensy to connect:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyTeensyROS -b 921600
```

**Verify:**
- [ ] Agent log shows `session established` and all `publisher created` / `subscriber created` / `replier created` messages
- [ ] Debug serial transitions from `WAITING` to `CONNECTED`
- [ ] `ros2 topic list` shows all expected topics (see topic table below)
- [ ] No repeated `create_client` / `delete_session` cycling (indicates connection instability)

### 3. Topic Publish Rates

Run `ros2 topic hz <topic>` for each publisher and verify the rate falls within
the acceptable range. Rates depend on thread update rate and internal publish throttles.

| Topic | Expected Hz | Acceptable Range | Thread Rate | Internal Throttle |
|-------|------------|-----------------|-------------|-------------------|
| `/mcu_robot/heartbeat` | 1 | 0.9 - 1.1 | 1000ms | none |
| `/mcu_robot/imu/data` | ~25 | 18 - 33 | 30ms | none |
| `/mcu_robot/rc` | 20 | 15 - 25 | main loop 5ms | 50ms |
| `/mcu_robot/battery_health` | 0.5 | 0.4 - 0.6 | 2000ms | 1000ms |
| `/mcu_robot/tof_distances` | 2 | 1.5 - 2.5 | 500ms | none |
| `/mcu_robot/dip_switches` | 0.5 | 0.4 - 0.6 | 2000ms | 1000ms |
| `/mcu_robot/buttons` | 10 | 7 - 12 | 100ms | 100ms |
| `/mcu_robot/servo/state` | 5 | 4 - 6 | 100ms | 200ms |
| `/mcu_robot/motor/state` | 5 | 4 - 6 | 1ms | 200ms |
| `/mcu_robot/encoders` | 20 | 14 - 22 | 50ms | none |
| `/mcu_robot/crank/state` | 5 | 4 - 6 | 200ms | 200ms |
| `/mcu_robot/keypad/state` | 5 | 4 - 6 | 200ms | 200ms |
| `/mcu_robot/deploy/trigger` | on event | N/A | 100ms | event-driven |
| `drive_base/status` | 50 | 25 - 50 | 20ms | divider=1 |
| `/mcu_robot/uwb/ranging` | 10 | 8 - 12 | 200ms | 100ms |

**Notes:**
- UWB topic only appears if DIP switch 2 is ON
- `drive_base/status` effective rate is limited by serial bandwidth (~32 Hz measured)
- RC topic requires FlySky receiver powered on and bound
- Deploy trigger only publishes on button 4 hold event

**Additional topics for `robot` environment only:**

| Topic | Expected Hz | Thread Rate |
|-------|------------|-------------|
| `/mcu_robot/intake/state` | 20 | 50ms (everyMs 50) |

### 4. Topic Data Validation

Spot-check message content with `ros2 topic echo <topic> --once`:

#### IMU
```bash
ros2 topic echo /mcu_robot/imu/data --once
```
- [ ] `frame_id` is `"imu_link"`
- [ ] `orientation` quaternion is normalized (w^2 + x^2 + y^2 + z^2 ~ 1.0)
- [ ] Quaternion is non-zero (not `0,0,0,0`)
- [ ] `orientation_covariance[0]` is `0.001` (not zero)
- [ ] With robot stationary, `angular_velocity` values are near zero (< 0.05 rad/s)

#### Heartbeat
```bash
ros2 topic echo /mcu_robot/heartbeat --once
```
- [ ] `data` field is `"HEARTBEAT"`

#### Encoders
```bash
ros2 topic echo /mcu_robot/encoders --once
```
- [ ] `data` array has 8 elements (one per QTimer channel)
- [ ] Values are 0.0 when motors are stationary

#### Battery
```bash
ros2 topic echo /mcu_robot/battery_health --once
```
- [ ] `voltage` is reasonable (0V if unpowered, 10-14V if powered)
- [ ] `current_ma` is non-negative
- [ ] `power_mw` is non-negative

#### Drive Base
```bash
ros2 topic echo /drive_base/status --once
```
- [ ] `header.stamp` has non-zero seconds (rmw_uros_epoch_nanos)
- [ ] `transform.transform.rotation.w` is near 1.0 at startup (identity quaternion)
- [ ] `drive_mode` is `DRIVE_VECTOR` when idle
- [ ] `twist.linear.x` and `twist.angular.z` are ~0 when stationary

#### DIP Switches
```bash
ros2 topic echo /mcu_robot/dip_switches --once
```
- [ ] `data` value changes when toggling physical DIP switches

#### Buttons
```bash
ros2 topic echo /mcu_robot/buttons --once
```
- [ ] `data` value changes when pressing physical buttons

### 5. Service Calls

```bash
# Set servo 0 to 90 degrees
ros2 service call /mcu_robot/servo/set mcu_msgs/srv/SetServo "{channel: 0, angle: 90.0}"

# Set motor 0 to 50% speed
ros2 service call /mcu_robot/motor/set mcu_msgs/srv/SetMotor "{channel: 0, speed: 0.5}"

# Reset all subsystems
ros2 service call /mcu_robot/reset mcu_msgs/srv/Reset "{}"
```

- [ ] SetServo returns success, servo physically moves
- [ ] SetMotor returns success, motor spins
- [ ] Reset returns success, all subsystems re-initialize (motors stop, PIDs reset)

### 6. Subscription Callbacks

```bash
# Send text to OLED
ros2 topic pub --once /mcu_robot/lcd/append std_msgs/msg/String "{data: 'Hello from ROS2'}"

# Set all LEDs to green
ros2 topic pub --once /mcu_robot/led/set_all mcu_msgs/msg/LedColor "{r: 0, g: 32, b: 0}"
```

- [ ] OLED displays the published text
- [ ] LEDs change to the published color

### 7. Drive Control

**RC drive (DIP 1 OFF):**
- [ ] FlySky SWA high -> RC sticks drive motors
- [ ] FlySky SWA low -> motors stop
- [ ] Smooth throttle/steering response (low-pass filter working)
- [ ] Command timeout (500ms) stops motors when RC signal lost

**ROS2 drive (DIP 1 ON):**
```bash
# Send velocity command
ros2 topic pub --rate 10 drive_base/command mcu_msgs/msg/DriveBase \
  "{drive_mode: 1, goal_velocity: {linear: {x: 0.3}, angular: {z: 0.0}}}"
```
- [ ] Robot drives forward at ~0.3 m/s
- [ ] Stopping the publisher triggers command timeout -> motors stop
- [ ] `drive_base/status` reflects current pose and velocity

### 8. Reconnection

Test micro-ROS agent disconnect/reconnect:

```bash
# Kill agent
pkill -9 -f "micro_ros_agent serial"
sleep 5

# Restart agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyTeensyROS -b 921600
```

- [ ] Debug serial shows `CONNECTED` -> `WAITING` -> `CONNECTED` cycle
- [ ] All topics resume publishing at correct rates after reconnection
- [ ] No crash or hang on the Teensy (no CrashReport on next status print)
- [ ] Motors stop during disconnect (safety timeout)

### 9. Thread Health

After running for 60+ seconds with all subsystems active, check debug serial:

```bash
timeout 5 cat /dev/ttyTeensyDebug
```

- [ ] IMU `updates` count increases steadily (no stalls)
- [ ] IMU `pubs` count tracks `updates` at ~1:1 ratio (deferred publishing working)
- [ ] No `[FAULT]` or `[ERROR]` messages
- [ ] No watchdog resets or CrashReport
- [ ] Uptime counter advances normally
- [ ] Encoder tick values are stable when motors off (no drift)
- [ ] Battery voltage reading is stable (no I2C errors)

### 10. Staged Bring-up (Debugging)

If a full-system test fails, use `DEBUG_STAGE` to isolate the problem.
Change the define in `teensy-test-all-subsystems.cpp` and reflash:

```cpp
#define DEBUG_STAGE 2  // Change to desired stage
```

| Stage | Subsystems Added | New Topics |
|-------|-----------------|------------|
| 1 | micro-ROS, Heartbeat | heartbeat |
| 2 | + IMU | imu/data |
| 3 | + Servo, Motor, Encoder (Wire2) | servo/state, motor/state, encoders |
| 4 | + Battery, TOF, DIP, Buttons (Wire0) | battery_health, tof_distances, dip_switches, buttons |
| 5 | + LED, OLED, Deploy, Crank, Keypad | crank/state, keypad/state, deploy/trigger |
| 6 | + Drive, RC, Reset | drive_base/status, rc |
| 7 | + UWB (if DIP 2 ON) | uwb/ranging |

At each stage, verify:
1. Build succeeds: `pio run -e teensy-test-all-subsystems`
2. Flash succeeds (retry once if "error writing")
3. All previous-stage topics still publish at correct rates
4. New-stage topics appear and publish at correct rates
5. No CrashReport or thread starvation

## Common Failures and Causes

| Symptom | Likely Cause |
|---------|-------------|
| `uROS state: WAITING` indefinitely | Agent not running, wrong serial port, or stale agent session (kill and restart) |
| IMU shows `0,0,0,0` quaternion | BNO085 not connected on Wire1, or I2C address conflict |
| IMU rate drops below 15 Hz | I2C bus contention, thread starvation, or BNO085 report rate misconfigured |
| Battery voltage always 0.0V | INA219 not behind correct mux channel, or mux not initialized |
| TOF distance always 0.0 | VL53L0X not connected, or I2C mux channel wrong |
| Encoders drift when motors off | QTimer noise on floating encoder pins |
| Drive pose drifts with no motion | Encoder inversion flags wrong, or IMU yaw offset |
| Agent cycles connect/disconnect | Too many executor handles (max 24), or entity limit exceeded (22 pub, 14 sub, 6 srv) |
| Flash fails "error writing" | Normal — retry after 3s. Teensy bootloader timing issue |
| Zombie agent processes | Pre-existing entrypoint issue. `pkill -9` and restart |
| All topics stop publishing | Serial transport hung. Kill agent, power-cycle Teensy, restart agent |
| Motors don't respond to RC | DIP 1 must be OFF for RC mode. Check SWA switch position |
| OLED blank | SPI1 pins (26/27) conflict with other peripherals, or OE pin not set |

## Checklist Template

Copy this for each regression run:

```
Date: _______________
Firmware: teensy-test-all-subsystems / robot (circle one)
Git commit: _______________
Tester: _______________
DEBUG_STAGE: _______________

Boot & Init:
  [ ] No CrashReport
  [ ] All INIT stages pass
  [ ] IMU calibration completes

micro-ROS:
  [ ] Agent connects
  [ ] All topics appear in ros2 topic list
  [ ] Reconnect after agent kill works

Publish Rates (fill in measured Hz):
  heartbeat:       _____ Hz (expect 1)
  imu/data:        _____ Hz (expect ~25)
  rc:              _____ Hz (expect 20)
  battery_health:  _____ Hz (expect 0.5)
  tof_distances:   _____ Hz (expect 2)
  dip_switches:    _____ Hz (expect 0.5)
  buttons:         _____ Hz (expect 10)
  servo/state:     _____ Hz (expect 5)
  motor/state:     _____ Hz (expect 5)
  encoders:        _____ Hz (expect 20)
  crank/state:     _____ Hz (expect 5)
  keypad/state:    _____ Hz (expect 5)
  drive_base:      _____ Hz (expect 25-50)
  uwb/ranging:     _____ Hz (expect 10, or N/A)

Data Validation:
  [ ] IMU quaternion valid
  [ ] Heartbeat string correct
  [ ] Encoder values stable at rest
  [ ] DIP switch reflects physical state

Services:
  [ ] SetServo works
  [ ] SetMotor works
  [ ] Reset works

Subscriptions:
  [ ] OLED text display
  [ ] LED color change

Drive:
  [ ] RC drive (DIP 1 OFF)
  [ ] ROS2 drive (DIP 1 ON)
  [ ] Command timeout stops motors

Stability (60s+ runtime):
  [ ] No crashes
  [ ] IMU update/pub counts healthy
  [ ] No error messages on debug serial

Issues Found:
  _______________________________________________
  _______________________________________________
```
