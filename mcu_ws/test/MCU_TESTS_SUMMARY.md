# MCU Hardware Tests Summary

This document summarizes the MCU hardware tests that run on physical Teensy and ESP32 boards.

## Overview

MCU tests are Arduino programs that run on actual hardware to verify subsystem functionality. Unlike native tests (which run on your computer), MCU tests require the actual microcontroller hardware and connected sensors/peripherals.

## Test Organization

### Native Tests (Run on PC - No Hardware Required)
Located in `mcu_ws/test/`:
- `test_math_pose2d/` - 2D pose math operations (19 tests)
- `test_math_vector2d/` - 2D vector math operations (33 tests)
- `test_math_pose3d/` - 3D pose and quaternion math (18 tests)
- `test_control_pid/` - PID controller (16 tests)
- `test_control_arm_kinematics/` - Forward/inverse kinematics (20 tests)
- `test_control_trapezoidal_motion_profile/` - Trapezoidal motion profiles (20 tests)
- `test_control_scurve_motion_profile/` - S-curve motion profiles (25 tests)
- `test_control_trajectory_controller/` - Path following controller (22 tests)
- `test_utils_filters/` - Signal filtering utilities (37 tests)
- `test_utils_units/` - Unit conversion functions (43 tests)

**Total Native Tests: 253 tests**

### MCU Hardware Tests (Run on Teensy/ESP32 - Requires Hardware)
Located in `mcu_ws/src/test/`:

#### Teensy 4.1 Tests

**teensy-test-microros-subsystem**
- Tests: micro-ROS integration, publisher creation, connection management
- Hardware: Teensy 4.1, micro-ROS agent (on host)
- Verification: Publishes test messages to ROS2 topics

**teensy-test-battery-subsystem**
- Tests: I2CPowerDriver initialization, battery monitoring, data publishing
- Hardware: Teensy 4.1, INA228 power sensor, battery
- Verification:
  - init() returns true
  - Voltage readings in valid range (0-30V)
  - Current readings in valid range (±20A)
  - Temperature readings valid (-40-125°C)
  - update() cycles run without crashing

**teensy-test-sensor-subsystem**
- Tests: TOF sensor initialization, distance measurements, multi-sensor management
- Hardware: Teensy 4.1, VL53L0X/VL53L1X TOF sensors, I2C multiplexer
- Verification:
  - init() returns true for all sensors
  - Distance readings in valid range (0-5000mm)
  - update() cycles run without crashing
  - Multiple sensors operate independently

## Running MCU Tests

### Prerequisites
All MCU tests must be run inside the Docker container:

```bash
# Start container
docker compose up -d

# Enter container
docker compose exec devcontainer bash

# Navigate to MCU workspace
cd /home/ubuntu/mcu_workspaces/sec26mcu
```

### Building MCU Tests

```bash
# Build specific test
pio run -e teensy-test-battery-subsystem

# Build all MCU tests
pio run -e teensy-test-microros-subsystem \
        -e teensy-test-battery-subsystem \
        -e teensy-test-sensor-subsystem
```

### Uploading to Hardware

```bash
# Upload specific test
pio run -e teensy-test-battery-subsystem --target upload

# Monitor serial output
pio device monitor -e teensy-test-battery-subsystem
```

### Expected Output

MCU tests print results to Serial (115200 baud):

```
================================================
BATTERY SUBSYSTEM MCU TEST
Teensy 4.1
================================================

================================================
TEST: Subsystem Existence
================================================
[PASS] I2CPowerDriver instantiated
[PASS] BatterySubsystem instantiated
[PASS] getInfo() returns valid string
  Info: BatterySubsystem

================================================
TEST: Subsystem Initialization
================================================
[PASS] init() returns true
  Sensor initialized successfully
  I2C Address: 0x40

...

================================================
TEST SUMMARY
================================================
Total Updates: 0
Test Duration: 2.5 seconds

✓ ALL TESTS PASSED
================================================
```

## Adding New MCU Tests

### 1. Create Test File

Create file in `mcu_ws/src/test/` with naming convention:
- Teensy tests: `teensy-test-<subsystem-name>.cpp`
- ESP32 tests: `esp32-test-<subsystem-name>.cpp`

Example structure:
```cpp
#include <Arduino.h>
#include <YourSubsystem.h>

// Test state
bool testsPassed = true;

void printTestResult(const char* testName, bool passed) {
  Serial.print(passed ? "[PASS]" : "[FAIL]");
  Serial.print(" ");
  Serial.println(testName);
  if (!passed) testsPassed = false;
}

void testSubsystemExists() {
  // Create subsystem
  // Verify it exists
}

void testSubsystemInit() {
  // Test init() returns true
  // Verify hardware responds
}

void testSubsystemFunctionality() {
  // Test core functionality
  // Verify readings/outputs
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  // Run all tests
  testSubsystemExists();
  testSubsystemInit();
  testSubsystemFunctionality();

  // Print summary
  if (testsPassed) {
    Serial.println("✓ ALL TESTS PASSED");
  } else {
    Serial.println("✗ SOME TESTS FAILED");
  }
}

void loop() {
  // Optional: continuous monitoring
}
```

### 2. Add Environment to platformio.ini

Add test environment in the `; --- Tests ---` section:

```ini
[env:teensy-test-your-subsystem]
extends = teensy_base
build_src_filter = +<test/teensy-test-your-subsystem.cpp>, +<robot/subsystems/YourSubsystem.cpp>
```

For tests requiring micro-ROS:
```ini
[env:teensy-test-your-subsystem]
extends = teensy_microros
build_src_filter = +<test/teensy-test-your-subsystem.cpp>, +<robot/subsystems/YourSubsystem.cpp>, +<robot/microros>
```

### 3. Build and Test

```bash
pio run -e teensy-test-your-subsystem
pio run -e teensy-test-your-subsystem --target upload
pio device monitor -e teensy-test-your-subsystem
```

## Test Requirements Matrix

| Test | MCU | Sensors/Hardware Required | Dependencies |
|------|-----|--------------------------|--------------|
| microros-subsystem | Teensy 4.1 | None | micro-ROS agent running |
| battery-subsystem | Teensy 4.1 | INA228 power sensor, Battery | I2C connection |
| sensor-subsystem | Teensy 4.1 | VL53L0X/VL53L1X TOF sensors, I2C mux | I2C connection |

## Future MCU Tests to Implement

### Teensy 4.1 Tests
- [ ] teensy-test-rc-subsystem (FlySky RC receiver, IBUS)
- [ ] teensy-test-arm-subsystem (PCA9685 servo driver, encoders)
- [ ] teensy-test-drive-subsystem (Motor controllers, encoders, IMU)
- [ ] teensy-test-heartbeat-subsystem (LED indicators)

### ESP32 Tests
- [ ] esp32-test-beacon-uwb (UWB DW1000 beacon)
- [ ] esp32-test-drone-controller
- [ ] esp32-test-field-button
- [ ] esp32-test-field-crank
- [ ] esp32-test-field-keypad
- [ ] esp32-test-field-pressure

### Integration Tests (Multi-MCU)
- [ ] test-uwb-tag-anchor (Teensy + ESP32 UWB communication)
- [ ] test-robot-field-ir (Teensy robot + ESP32 field element IR)
- [ ] test-microros-end-to-end (Full ROS2 stack)

## Troubleshooting

### Test Won't Build
- **Check build_src_filter**: Ensure all required source files are included
- **Check dependencies**: Verify lib_deps includes necessary libraries
- **Check extends**: Ensure correct base (teensy_base, teensy_microros, etc.)

### Test Crashes on Hardware
- **Check Serial**: Use `Serial.begin(115200)` and monitor output
- **Check init()**: Ensure all hardware initialized before use
- **Check nullptr**: Verify pointers before dereferencing
- **Check I2C**: Verify SDA/SCL connections and pull-up resistors

### Sensor Not Responding
- **Check power**: Ensure sensor has power (3.3V or 5V)
- **Check I2C address**: Use I2C scanner to verify address
- **Check wiring**: Verify SDA, SCL, GND, VCC connections
- **Check I2C speed**: Some sensors require slower I2C speeds

### micro-ROS Connection Fails
- **Check agent**: Ensure `ros2 run micro_ros_agent...` is running
- **Check network**: For WiFi transport, verify network connection
- **Check serial**: For serial transport, verify baud rate and port

## CI/CD Integration

MCU tests can be integrated into CI/CD with hardware-in-the-loop:

```yaml
- name: Run MCU Tests
  run: |
    # Build tests
    pio run -e teensy-test-battery-subsystem

    # Upload to connected hardware (requires USB passthrough)
    pio run -e teensy-test-battery-subsystem --target upload

    # Monitor and capture results
    timeout 30 pio device monitor -e teensy-test-battery-subsystem > test_output.txt

    # Parse results
    grep "ALL TESTS PASSED" test_output.txt
```

## Additional Resources

- [PlatformIO Testing Documentation](https://docs.platformio.org/en/latest/advanced/unit-testing/index.html)
- [Teensy 4.1 Datasheet](https://www.pjrc.com/store/teensy41.html)
- [micro-ROS Documentation](https://micro.ros.org/)
- [Project CLAUDE.md](../../CLAUDE.md) - Development guidelines
- [Native Tests README](README.md) - Native test documentation
