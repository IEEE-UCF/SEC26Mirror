# Minibot Firmware

Firmware for the SEC26 minibot — a small differential-drive robot deployed from the main robot to navigate into and out of the crater field element.

## Hardware

- **MCU:** Seeed XIAO ESP32-S3
- **Transport:** WiFi UDP micro-ROS to Pi agent (port 8888)
- **Static IP:** 192.168.4.24
- **OTA hostname:** `sec26-minibot`
- **Motor driver:** H-bridge (DRV8833 / L298N style) with truth-table direction + PWM speed

### Pin Assignments

| Function       | Pin |
|----------------|-----|
| Motor A IN1    | 5   |
| Motor A IN2    | 15  |
| Motor A PWM    | 2   |
| Motor B IN1    | 18  |
| Motor B IN2    | 19  |
| Motor B PWM    | 4   |

## Building and Flashing

All commands run inside the Docker container.

```bash
cd /home/ubuntu/mcu_workspaces/sec26mcu

# Build
pio run -e minibot

# Flash via USB
pio run -e minibot --target upload

# Flash via OTA (minibot must be on WiFi)
pio run -e minibot --target upload --upload-port 192.168.4.24

# Serial monitor
pio device monitor -e minibot
```

## ROS2 Interface

### Publishers

| Topic | Message Type | QoS | Rate | Description |
|-------|-------------|-----|------|-------------|
| `/mcu_minibot/state` | `mcu_msgs/msg/MiniRobotState` | Best-effort | 10 Hz | Current mission state and task |
| `/mcu_minibot/heartbeat` | `std_msgs/msg/String` | Default | 1 Hz | Heartbeat with uptime |
| `/mcu_minibot/debug` | `std_msgs/msg/String` | Default | On-demand | Debug messages (e.g., reset reason on boot) |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/mcu_minibot/enter_crater` | `mcu_msgs/srv/Reset` | Trigger the enter-crater mission sequence |
| `/mcu_minibot/exit_crater` | `mcu_msgs/srv/Reset` | Trigger the exit-crater mission sequence |

Both services return `success: true` if the command was accepted, or `success: false` if the minibot is already running a mission.

### Subscribers

None. The minibot is controlled entirely through service calls.

## Message Definitions

### `mcu_msgs/msg/MiniRobotState`

```
std_msgs/Header header

# State constants
uint8 PRE_INIT=0
uint8 INIT=1
uint8 INIT_FAIL=2
uint8 INIT_SUCCESS=3
uint8 ARMED=4        # Idle, ready for commands
uint8 RUNNING=5      # Executing a mission
uint8 STOPPED=6
uint8 RESET=7
uint8 EMERGENCY_STOP=8
uint8 state

# Task constants
uint8 TASK_NONE=0
uint8 TASK_ENTER_CRATER=1
uint8 TASK_CIRCLE_CRATER=2
uint8 TASK_EXIT_CRATER=3
uint8 TASK_PARK=4
uint8 TASK_COMPLETE=5
uint8 TASK_BOARD_ROBOT=6
uint8 current_task
```

### `mcu_msgs/srv/Reset`

```
---
bool success
```

## Usage Examples

### Check minibot topics are visible

```bash
ros2 topic list | grep minibot
```

Expected output:
```
/mcu_minibot/debug
/mcu_minibot/heartbeat
/mcu_minibot/state
```

### Monitor minibot state

```bash
ros2 topic echo /mcu_minibot/state
```

When idle, you'll see `state: 4` (ARMED) and `current_task: 0` (TASK_NONE).
During a mission, `state: 5` (RUNNING) with the appropriate task constant.

### Check publish rate

```bash
ros2 topic hz /mcu_minibot/state
```

Expected: ~10 Hz.

### Trigger enter-crater mission

```bash
ros2 service call /mcu_minibot/enter_crater mcu_msgs/srv/Reset
```

Response: `success: true` if accepted.

The enter-crater sequence:
1. **Wait** 5 seconds (allows main robot to back away)
2. **Drive forward** at speed 90 for 3 seconds
3. **Turn left** (left wheel at 50% speed) for 2 seconds
4. **Stop** — returns to IDLE

### Trigger exit-crater mission

```bash
ros2 service call /mcu_minibot/exit_crater mcu_msgs/srv/Reset
```

The exit-crater sequence:
1. **Drive forward** at speed 220 for 3 seconds
2. **Stop** — returns to IDLE

### Check heartbeat

```bash
ros2 topic echo /mcu_minibot/heartbeat
```

### List available services

```bash
ros2 service list | grep minibot
```

## Architecture

### FreeRTOS Tasks

| Task | Priority | Rate | Stack | Description |
|------|----------|------|-------|-------------|
| MicrorosManager | 2 | 100 Hz | 8192 | micro-ROS agent lifecycle and publishing |
| MinibotMission | 2 | 20 Hz | 2048 | Mission state machine + motor control |
| Heartbeat | 1 | 1 Hz | 2048 | Heartbeat publisher |
| WiFi + OTA | 1 | 10 Hz | 4096 | WiFi reconnect and ArduinoOTA handler |

### Mission State Machine

```
          enter_crater service
                │
                v
IDLE ──► ENTER_WAIT (5s) ──► ENTER_FORWARD (3s) ──► ENTER_TURN_LEFT (2s) ──► IDLE
  │
  │     exit_crater service
  │           │
  └─────────► EXIT_FORWARD (3s) ──► IDLE
```

Service callbacks set a `pending_cmd_` flag. The mission subsystem's `update()` loop (running at 20 Hz in its own FreeRTOS task) consumes the flag and drives the state machine. This avoids motor writes from the service callback thread.

### Deferred Publishing

The mission subsystem caches state messages under a `data_mutex_` and sets `data_ready_ = true`. The MicrorosManager thread calls `publishAll()` which acquires the mutex and publishes if data is ready. This follows the same deferred publishing pattern as the main robot firmware.

## Network

| Parameter | Value |
|-----------|-------|
| WiFi SSID | UCFIEEEBot |
| WiFi Password | goodlife |
| Static IP | 192.168.4.24 |
| Gateway | 192.168.4.1 (Pi) |
| micro-ROS Agent | UDP port 8888 |
| OTA Port | Default (3232) |
