# SEC26 Drone Regression Tests

Manual hardware regression tests for the `drone` firmware target.

## Overview

The drone runs on an ESP32 with WiFi micro-ROS (UDP transport). Currently it only
publishes a heartbeat — flight controller, IMU, height, IR, and UWB subsystems are
planned but not yet wired in.

- **MCU:** ESP32
- **Transport:** WiFi UDP micro-ROS to Pi agent (port 8888)
- **Static IP:** 192.168.4.25
- **OTA hostname:** `sec26-drone`
- **Participants:** 1 (HeartbeatSubsystem)
- **Entry point:** `src/drone/main.cpp`

## Prerequisites

- ESP32 drone board powered on
- Pi WiFi AP running (SSID: `UCFIEEEBot`)
- Docker container running with UDP micro-ROS agent (auto-started by entrypoint)
- All commands run inside the container

## Quick Reference

```bash
# Inside Docker container
cd /home/ubuntu/mcu_workspaces/sec26mcu

# Build drone firmware
pio run -e drone

# Flash via USB (if connected)
pio run -e drone --target upload

# Flash via OTA (if drone is on WiFi)
pio run -e drone --target upload --upload-port 192.168.4.25

# Monitor serial (USB)
pio device monitor -e drone
```

## Test Procedure

### 1. Flash and Boot

```bash
pio run -e drone --target upload
```

**Verify on serial monitor:**
- [ ] `Drone starting...` message appears
- [ ] `WiFi connected: 192.168.4.25` prints (correct IP)
- [ ] `[OTA] Ready as sec26-drone` appears
- [ ] `Drone initialized!` prints
- [ ] No crash or reboot loop

### 2. WiFi Connection

- [ ] Drone connects to `UCFIEEEBot` AP
- [ ] Ping from Pi: `ping 192.168.4.25` responds
- [ ] WiFi auto-reconnects after AP reboot (wait ~15s)

### 3. micro-ROS Connection

The UDP agent should already be running (started by container entrypoint).

```bash
# Verify agent is running
pgrep -f "micro_ros_agent udp4" && echo "Agent running"

# Check for drone's heartbeat topic
ros2 topic list | grep drone
```

**Verify:**
- [ ] Agent log shows session established for drone
- [ ] Heartbeat topic appears (uses default namespace — check `ros2 topic list`)

### 4. Topic Validation

```bash
ros2 topic hz /heartbeat    # or whatever namespace the drone uses
ros2 topic echo /heartbeat --once
```

| Topic | Expected Hz | Acceptable Range |
|-------|------------|-----------------|
| Heartbeat | 1 | 0.9 - 1.1 |

- [ ] Heartbeat publishes at ~1 Hz
- [ ] `data` field is `"HEARTBEAT"`

### 5. OTA Update

```bash
pio run -e drone --target upload --upload-port 192.168.4.25
```

- [ ] OTA flash completes without error
- [ ] Drone reboots and reconnects to WiFi
- [ ] Heartbeat resumes publishing after OTA

### 6. Reconnection

```bash
# Kill UDP agent
pkill -9 -f "micro_ros_agent udp4"
sleep 5

# Agent auto-restarts via entrypoint (wait ~5s)
# Or manually:
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

- [ ] Drone reconnects to agent after agent restart
- [ ] Heartbeat resumes at correct rate
- [ ] No crash or hang on ESP32

### 7. Stability

Run for 60+ seconds:

- [ ] No serial error messages
- [ ] WiFi stays connected
- [ ] Heartbeat continues publishing
- [ ] No watchdog resets

## Common Failures and Causes

| Symptom | Likely Cause |
|---------|-------------|
| WiFi won't connect | AP not running, wrong SSID/password, IP conflict |
| No heartbeat topic | UDP agent not running, firewall blocking port 8888 |
| OTA fails | Drone not on WiFi, wrong IP, mDNS not resolving |
| Reboot loop | Power supply issue, flash corruption (re-flash via USB) |
| Agent won't connect | Wrong port, zombie agent process (kill and restart) |

## Checklist Template

```
Date: _______________
Git commit: _______________
Tester: _______________

Boot:
  [ ] Serial messages correct
  [ ] WiFi connected (192.168.4.25)
  [ ] OTA ready

micro-ROS:
  [ ] Agent connects
  [ ] Heartbeat topic visible

Rates:
  heartbeat: _____ Hz (expect 1)

OTA:
  [ ] OTA flash works
  [ ] Reconnects after OTA

Stability (60s+):
  [ ] No crashes
  [ ] WiFi stable
  [ ] Heartbeat steady

Issues Found:
  _______________________________________________
```
