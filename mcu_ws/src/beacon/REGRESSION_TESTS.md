# SEC26 Beacon Regression Tests

Manual hardware regression tests for the `beacon1`, `beacon2`, and `beacon3` firmware targets.

## Overview

Each beacon is an ESP32 + DW3000 running in UWB ANCHOR mode with WiFi micro-ROS
(UDP transport). Beacons respond to ranging requests from tags (robot, minibot, drone)
and perform inter-beacon peer ranging (lower ID initiates to higher IDs).

- **MCU:** ESP32
- **Transport:** WiFi UDP micro-ROS to Pi agent (port 8888)
- **Participants:** 2 (HeartbeatSubsystem, UWBSubsystem)
- **Entry point:** `src/beacon/machines/BeaconLogic.h`

| Target | BEACON_ID | Static IP | OTA Hostname | Peer Targets |
|--------|-----------|-----------|--------------|-------------|
| `beacon1` | 10 | 192.168.4.20 | `sec26-beacon-10` | ranges to 11, 12 |
| `beacon2` | 11 | 192.168.4.21 | `sec26-beacon-11` | ranges to 12 |
| `beacon3` | 12 | 192.168.4.22 | `sec26-beacon-12` | none (highest ID) |

### UWB Pin Wiring (ESP32)

| Function | Pin |
|----------|-----|
| SPI CS | 5 |
| Reset | 4 |
| IRQ | 17 (unused in polling mode) |

## Prerequisites

- All 3 beacon ESP32 boards powered on and placed at known field positions
- Pi WiFi AP running (SSID: `UCFIEEEBot`)
- Docker container running with UDP micro-ROS agent (auto-started by entrypoint)
- DW3000 modules properly wired to SPI
- All commands run inside the container

## Quick Reference

```bash
# Inside Docker container
cd /home/ubuntu/mcu_workspaces/sec26mcu

# Build all beacons
pio run -e beacon1 && pio run -e beacon2 && pio run -e beacon3

# Flash individual beacon via USB
pio run -e beacon1 --target upload

# Flash via OTA (beacon must be on WiFi)
pio run -e beacon1 --target upload --upload-port 192.168.4.20
pio run -e beacon2 --target upload --upload-port 192.168.4.21
pio run -e beacon3 --target upload --upload-port 192.168.4.22

# Monitor serial (USB)
pio device monitor -e beacon1
```

## Test Procedure

### 1. Flash and Boot (repeat for each beacon)

```bash
pio run -e beacon1 --target upload
```

**Verify on serial monitor:**
- [ ] `UWB Beacon 10 starting...` (correct ID)
- [ ] `WiFi connected: 192.168.4.20` (correct IP)
- [ ] `[OTA] Ready as sec26-beacon-10`
- [ ] No `ERROR: Failed to initialize subsystems!`
- [ ] `Beacon 10 initialized and waiting for ranging requests!`
- [ ] No crash or reboot loop

### 2. WiFi Connection (all 3 beacons)

- [ ] beacon1 at 192.168.4.20: `ping 192.168.4.20`
- [ ] beacon2 at 192.168.4.21: `ping 192.168.4.21`
- [ ] beacon3 at 192.168.4.22: `ping 192.168.4.22`
- [ ] WiFi auto-reconnects after AP reboot

### 3. micro-ROS Connection

```bash
# Verify UDP agent is running
pgrep -f "micro_ros_agent udp4" && echo "Agent running"

# Check for beacon topics
ros2 topic list
```

**Verify for each beacon:**
- [ ] Agent shows session established
- [ ] Heartbeat topic appears
- [ ] UWB ranging topic(s) appear (see topic table)

### 4. Topic Publish Rates

| Topic | Source | Expected Hz | Acceptable Range |
|-------|--------|------------|-----------------|
| Heartbeat (per beacon) | All beacons | 1 | 0.9 - 1.1 |
| `mcu_uwb/ranging` | Tag mode (not beacons) | N/A | N/A |
| Inter-beacon peer range | beacon1 -> beacon2 | 2 | 1.5 - 2.5 |
| Inter-beacon peer range | beacon1 -> beacon3 | 2 | 1.5 - 2.5 |
| Inter-beacon peer range | beacon2 -> beacon3 | 2 | 1.5 - 2.5 |

**Notes:**
- Beacons are in ANCHOR mode — they don't publish tag ranging topics
- Inter-beacon peer ranging only publishes if peers exist (lower ID ranges to higher)
- beacon3 (ID=12) has no peers to range to, so it only publishes heartbeat

```bash
# Check heartbeat rate for each beacon
ros2 topic hz /heartbeat  # namespace depends on beacon configuration
```

### 5. UWB Ranging Validation

With the robot tag (ID=13) powered on and ranging:

```bash
ros2 topic echo /mcu_robot/uwb/ranging --once
```

- [ ] `ranges` array includes entries from all 3 beacons (IDs 10, 11, 12)
- [ ] `distance_m` values are reasonable for known beacon placement
- [ ] `range_count` increments over time
- [ ] No large outliers (> 2m error from known distances)

### 6. Inter-beacon Peer Ranging

With all 3 beacons running:

- [ ] beacon1 serial shows ranging to beacon2 and beacon3
- [ ] beacon2 serial shows ranging to beacon3
- [ ] beacon3 serial shows no peer ranging activity (highest ID)
- [ ] Peer range values are stable and match physical placement

### 7. OTA Update

```bash
pio run -e beacon1 --target upload --upload-port 192.168.4.20
```

- [ ] OTA flash completes without error
- [ ] Beacon reboots and reconnects to WiFi
- [ ] UWB ranging resumes after reboot
- [ ] Heartbeat resumes publishing

### 8. Power Savings

After initialization, beacons drop to 80 MHz CPU and reduce WiFi TX power:

- [ ] Serial shows no errors after power savings applied
- [ ] micro-ROS connection remains stable at 80 MHz
- [ ] No spurious agent disconnect/reconnect cycles

### 9. Reconnection

```bash
# Kill UDP agent
pkill -9 -f "micro_ros_agent udp4"
sleep 5

# Agent auto-restarts via entrypoint
```

- [ ] All beacons reconnect to agent
- [ ] Heartbeat resumes for all 3
- [ ] UWB ranging resumes
- [ ] No crash or hang on any ESP32

### 10. Stability

Run all 3 beacons for 5+ minutes:

- [ ] No serial error messages on any beacon
- [ ] WiFi stays connected on all 3
- [ ] Heartbeat steady on all 3
- [ ] Ranging values stable (no drift without movement)
- [ ] No watchdog resets or reboots

## Common Failures and Causes

| Symptom | Likely Cause |
|---------|-------------|
| WiFi won't connect | AP not running, IP conflict with another beacon |
| `ERROR: Failed to initialize subsystems!` | DW3000 not wired correctly, SPI pins wrong |
| No ranging data from beacon | Beacon not in range of tag, DW3000 not initialized |
| Peer ranging fails | Both beacons must be powered, lower ID initiates |
| Agent disconnect cycling | `WIFI_PS_MAX_MODEM` accidentally enabled (causes missed UDP pings) |
| OTA fails | Wrong IP, beacon not on WiFi, mDNS not resolving |
| Reboot loop | Power supply issue, flash corruption |
| Ranging distance wrong | Antenna orientation, multipath interference, calibration needed |

## Checklist Template

```
Date: _______________
Git commit: _______________
Tester: _______________

Beacon 1 (ID=10, 192.168.4.20):
  [ ] Boot OK
  [ ] WiFi connected
  [ ] Heartbeat publishing at _____ Hz (expect 1)
  [ ] Peer ranging to beacon2: _____ m
  [ ] Peer ranging to beacon3: _____ m

Beacon 2 (ID=11, 192.168.4.21):
  [ ] Boot OK
  [ ] WiFi connected
  [ ] Heartbeat publishing at _____ Hz (expect 1)
  [ ] Peer ranging to beacon3: _____ m

Beacon 3 (ID=12, 192.168.4.22):
  [ ] Boot OK
  [ ] WiFi connected
  [ ] Heartbeat publishing at _____ Hz (expect 1)

Robot Tag Ranging:
  [ ] Ranges from all 3 beacons visible
  [ ] Distance values reasonable

OTA:
  [ ] OTA flash works for at least 1 beacon

Stability (5+ min):
  [ ] No crashes on any beacon
  [ ] WiFi stable on all 3
  [ ] Ranging stable

Issues Found:
  _______________________________________________
```
