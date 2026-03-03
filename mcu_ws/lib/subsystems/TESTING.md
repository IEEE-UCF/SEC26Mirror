# ESP32 WiFi Subsystem — Testing Procedure

This document covers how to test the `ESP32WifiSubsystem` and related changes introduced in the `feat/esp32-wifi-subsystem` branch.

## Prerequisites

All PlatformIO commands must be run **inside the Docker container**:

```bash
docker compose up -d
docker compose exec devcontainer bash
cd /home/ubuntu/mcu_workspaces/sec26mcu
```

## 1. Build Verification (No Hardware)

Confirm all affected environments compile without errors.

### All default targets (robot, drone, minibot, beacons, test-all-subsystems)

```bash
pio run
```

### WiFi test environment

```bash
pio run -e esp32-test-microros-wifi
```

### Individual ESP32 targets that use ESP32WifiSubsystem

```bash
pio run -e beacon1
pio run -e beacon2
pio run -e beacon3
pio run -e minibot
pio run -e drone
```

### ROS2 deploy node

```bash
cd /home/ubuntu/ros2_workspaces
source /opt/ros/jazzy/setup.bash
colcon build --packages-select secbot_deploy
```

## 2. Native Unit Tests (No Hardware)

Run all existing unit tests to check for regressions:

```bash
./scripts/run_all_mcu_tests.sh
```

Or run individually:

```bash
pio test -e test-math-pose2d
pio test -e test-math-vector2d
pio test -e test-math-pose3d
pio test -e test-control-pid
pio test -e test-control-arm-kinematics
pio test -e test-control-trapezoidal-motion-profile
pio test -e test-control-scurve-motion-profile
pio test -e test-control-trajectory-controller
pio test -e test-drive-tankdrivelocalization
pio test -e test-utils-filters
pio test -e test-utils-units
```

All 253+ tests should pass.

## 3. WiFi Subsystem Hardware Test (Requires ESP32 + AP)

### Setup

1. A WiFi access point broadcasting SSID `UCFIEEEBot` (password: `goodlife`)
2. An ESP32 dev board connected via USB
3. A micro-ROS agent running on the network (for full test)

### Flash and monitor

```bash
pio run -e esp32-test-microros-wifi --target upload
pio device monitor -e esp32-test-microros-wifi
```

### Expected serial output

```
=== ESP32 micro-ROS WiFi Transport Test ===
Agent: 192.168.4.1:8888
SSID:  UCFIEEEBot
Waiting for WiFi...
WiFi connected: 192.168.4.100
Setup complete. Waiting for agent...
[OK] Agent connected - publishing heartbeat
```

### What to verify

| Check | Expected |
|-------|----------|
| WiFi connects | State transitions: `DISCONNECTED → CONNECTING → CONNECTED` |
| Static IP assigned | `192.168.4.100` shown in serial output |
| Agent heartbeat | `[OK] Agent connected` prints every second (requires agent running) |
| Auto-reconnect | Power-cycle AP briefly; ESP32 should reconnect within ~10s |
| Retry limit | Set AP to wrong SSID; should enter `FAILED` after 5 retries (default `max_retries`) |

### Verify ROS2 topic (with agent running)

On the Pi or a machine running the micro-ROS agent:

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

In another terminal:

```bash
ros2 topic echo /esp32_wifi_test/status std_msgs/msg/String
```

You should see `data: "OK"` messages at 1 Hz.

## 4. Per-Device Hardware Tests (Requires ESP32 + Pi)

Each ESP32 target needs the Pi running as AP with a micro-ROS agent on `192.168.4.1:8888`.

### Start the agent on the Pi

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

### Beacon (repeat for beacon1, beacon2, beacon3)

```bash
pio run -e beacon1 --target upload
pio device monitor -e beacon1
```

Expected:
- WiFi connects at `192.168.4.20` (beacon1), `.21` (beacon2), `.22` (beacon3)
- micro-ROS agent connection established
- UWB ranging data published to `mcu_uwb/ranging`

### Minibot

```bash
pio run -e minibot --target upload
pio device monitor -e minibot
```

Expected:
- WiFi connects at `192.168.4.24`
- micro-ROS agent connection established
- Subscribes to `/mcu_minibot/cmd_vel`

### Drone

```bash
pio run -e drone --target upload
pio device monitor -e drone
```

Expected:
- WiFi connects at `192.168.4.25`
- micro-ROS agent connection established

## 5. Full Integration Test

With all devices powered and the Pi as AP + agent:

1. Flash robot, all beacons, minibot
2. Start micro-ROS agent on Pi
3. Verify all nodes appear:
   ```bash
   ros2 topic list | grep mcu
   ```
4. Confirm UWB ranging:
   ```bash
   ros2 topic echo /mcu_uwb/ranging
   ```
5. Test WiFi resilience: reboot an ESP32 and confirm it re-joins and resumes publishing

## Checklist

| # | Test | Hardware | Pass? |
|---|------|----------|-------|
| 1 | `pio run` (all default targets) | None | |
| 2 | `pio run -e esp32-test-microros-wifi` | None | |
| 3 | Native unit tests (`run_all_mcu_tests.sh`) | None | |
| 4 | `colcon build --packages-select secbot_deploy` | None | |
| 5 | WiFi test on ESP32 (connect + heartbeat) | ESP32 + AP | |
| 6 | WiFi auto-reconnect | ESP32 + AP | |
| 7 | Beacon connectivity (×3) | ESP32 + Pi | |
| 8 | Minibot connectivity | ESP32 + Pi | |
| 9 | Drone connectivity | ESP32 + Pi | |
| 10 | Full integration (all devices) | All | |
