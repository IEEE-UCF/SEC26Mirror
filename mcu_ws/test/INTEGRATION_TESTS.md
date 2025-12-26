# SEC26 MCU Integration Tests

This document describes integration tests that require multiple MCUs working together. These tests verify system-level functionality and communication between components.

## Overview

Integration tests are **not automated** and require manual setup and execution. They verify end-to-end functionality across multiple microcontrollers and hardware components.

## Prerequisites

- All required MCUs flashed with appropriate firmware
- Hardware properly connected and powered
- Serial monitors/debugging tools available
- Test environment (field setup for some tests)

## UWB Tag-Anchor Communication Test

### Purpose
Verify ultra-wideband (UWB) ranging and communication between tag and anchor nodes for robot localization.

### Hardware Required
- 1x ESP32 configured as UWB tag
- 3+ ESP32 configured as UWB anchors
- DW3000 UWB modules on each ESP32
- Power supply for all devices

### Setup Procedure

1. **Flash Firmware**:
   ```bash
   # Inside Docker container
   cd /home/ubuntu/mcu_workspaces/sec26mcu

   # Flash tag
   pio run -e esp32-test-uwb-tag --target upload

   # Flash each anchor (repeat for each anchor)
   pio run -e esp32-test-uwb-anchor --target upload
   ```

2. **Physical Setup**:
   - Place anchors at known positions forming a coverage area
   - Record anchor positions (coordinates)
   - Power on all anchors first
   - Power on tag last

3. **Anchor Configuration**:
   - Each anchor needs unique ID (configured via build flags or runtime)
   - Anchor positions should be at least 2m apart
   - Anchors should have clear line-of-sight to tag operating area

### Test Execution

1. Open serial monitors for tag and at least one anchor:
   ```bash
   # Tag monitor
   pio device monitor -e esp32-test-uwb-tag

   # Anchor monitor (separate terminal)
   pio device monitor -e esp32-test-uwb-anchor
   ```

2. **Verify Initialization**:
   - All devices should print "UWB initialized" messages
   - No error messages during startup
   - Anchor IDs should be displayed correctly

3. **Test Ranging**:
   - Move tag to known position
   - Observe distance measurements from each anchor
   - Record distances for at least 5 positions

4. **Expected Results**:
   - Distance measurements within ±10cm of true distance
   - Update rate: 5-10 Hz minimum
   - No packet loss >5%
   - Stable measurements (standard deviation <5cm when stationary)

### Troubleshooting

- **No communication**: Check SPI wiring, ensure DW3000 powered
- **Large errors**: Verify antenna orientation, check for obstacles
- **Intermittent ranging**: Check power supply stability, reduce distance
- **Anchor not responding**: Verify unique IDs, check serial output for errors

## Robot-Field Element Communication Test

### Purpose
Verify IR communication between robot (Teensy) and field elements (ESP32) for task detection and scoring.

### Hardware Required
- 1x Teensy41 (robot)
- 1x ESP32 (field element - button, crank, or keypad)
- IR transmitter/receiver pairs
- Test field element hardware

### Setup Procedure

1. **Flash Firmware**:
   ```bash
   # Robot
   pio run -e robot --target upload

   # Field element (example: button)
   pio run -e field-button --target upload
   ```

2. **Physical Setup**:
   - Connect IR hardware to both MCUs
   - Position robot within 20cm of field element
   - Ensure IR LEDs/receivers aligned
   - Power both devices

### Test Execution

1. **Verify IR Detection**:
   - Robot should detect field element presence
   - Check robot serial output for detection messages
   - Verify LED indicators (if present)

2. **Test Task Initiation**:
   - Trigger task start signal from robot
   - Field element should acknowledge
   - Monitor both serial outputs

3. **Test Task Completion**:
   - Complete the task (press button, turn crank, etc.)
   - Field element should send completion signal
   - Robot should acknowledge and record completion

4. **Expected Results**:
   - Detection range: 5-20cm
   - Communication latency: <100ms
   - No missed messages during task execution
   - Proper state synchronization

### Troubleshooting

- **No detection**: Check IR alignment, verify transmitter power
- **Missed messages**: Reduce distance, check for IR interference
- **Wrong state**: Verify protocol implementation on both sides

## Multi-Robot Coordination Test

### Purpose
Verify communication and coordination between multiple robots (if applicable).

### Hardware Required
- 2+ Teensy41 (robots)
- Communication hardware (WiFi/BLE modules if used)

### Setup
*(To be defined based on final robot architecture)*

## Micro-ROS Integration Test

### Purpose
Verify micro-ROS communication between MCUs and ROS2 nodes.

### Hardware Required
- 1x Teensy41 (robot) or ESP32 (robotcomms)
- Computer running ROS2 agent
- USB or network connection

### Setup Procedure

1. **Start micro-ROS Agent**:
   ```bash
   # On host or inside Docker
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
   ```

2. **Flash micro-ROS Firmware**:
   ```bash
   pio run -e teensy-test-microros-subsystem --target upload
   ```

3. **Verify Connection**:
   ```bash
   # List topics
   ros2 topic list

   # Echo heartbeat
   ros2 topic echo /heartbeat
   ```

### Expected Results
- Micro-ROS agent connects successfully
- Topics appear in ROS2 topic list
- Messages publish at expected rates
- No disconnections or transport errors

## Test Results Template

Use this template to record integration test results:

```
Test Name: _______________
Date: _______________
Tester: _______________

Hardware Configuration:
- Device 1: _______________
- Device 2: _______________
- Device N: _______________

Test Results:
[ ] Pass / [ ] Fail

Measurements:
- Metric 1: _______________ (expected: _______________)
- Metric 2: _______________ (expected: _______________)

Issues Encountered:
- _______________
- _______________

Notes:
- _______________
```

## Continuous Integration

Currently, integration tests are **manual only**. Future work may include:
- Hardware-in-the-loop (HIL) test rig
- Automated field element simulation
- Self-hosted runners with connected MCUs

## Adding New Integration Tests

When adding new integration tests:

1. Document hardware requirements
2. Provide step-by-step setup instructions
3. Define clear pass/fail criteria
4. Include troubleshooting section
5. Update this file with new test procedure

## References

- UWB Datasheet: DW3000 User Manual
- micro-ROS Documentation: https://micro.ros.org/
- PlatformIO Testing: https://docs.platformio.org/en/latest/advanced/unit-testing/
