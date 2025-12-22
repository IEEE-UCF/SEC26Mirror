# MCU Unit Tests

This directory contains unit tests for the MCU firmware components.

## Overview

The test suite validates core mathematical and control components used throughout the robot firmware. All tests run **natively on your computer** without requiring physical hardware.

## Prerequisites

All commands must be run **inside the Docker container**:

```bash
# Start the container (run on host)
docker compose up -d

# Enter the container (run on host)
docker compose exec devcontainer bash

# Navigate to MCU workspace (inside container)
cd /home/ubuntu/mcu_workspaces/sec26mcu
```

## Running Tests

### Run All Native Tests

```bash
# Run all native test suites (math + control)
pio test \
  -e test-math-pose2d \
  -e test-math-vector2d \
  -e test-math-pose3d \
  -e test-control-pid \
  -e test-control-arm-kinematics \
  -e test-control-trapezoidal-motion-profile \
  -e test-control-scurve-motion-profile \
  -e test-control-trajectory-controller

# Or use the convenience script
cd scripts && ./run_all_tests.sh
```

### Run Individual Test Suites

```bash
# Pose2D tests (19 tests)
pio test -e test-math-pose2d

# Vector2D tests (33 tests)
pio test -e test-math-vector2d

# Pose3D tests (18 tests)
pio test -e test-math-pose3d

# Control PID tests
pio test -e test-control-pid

# Arm kinematics tests (19 tests)
pio test -e test-control-arm-kinematics

# Trapezoidal motion profile tests (22 tests)
pio test -e test-control-trapezoidal-motion-profile

# S-curve motion profile tests (25 tests)
pio test -e test-control-scurve-motion-profile

# Trajectory controller tests (22 tests)
pio test -e test-control-trajectory-controller
```

### Run Tests with Verbose Output

```bash
# Single verbose flag
pio test -e test-math-pose2d -v

# Double verbose (more details)
pio test -e test-math-pose2d -vv

# Triple verbose (maximum details)
pio test -e test-math-pose2d -vvv
```

### Run Specific Test by Name

PlatformIO runs all tests in a suite, but you can filter by modifying the test file temporarily or by checking the output for specific test results.

## Test Coverage

### Pose2D Tests (`test_math_pose2d/`)
- Constructors and getters
- Angle normalization (handles ±π, overflow, underflow)
- Addition and rotation operations
- Edge cases (zero angles, large angles, small precision values)

### Vector2D Tests (`test_math_vector2d/`)
- Constructors and getters
- Magnitude calculations
- Addition and subtraction
- Rotation (90°, 180°, 360°, negative angles)
- Scalar multiplication (including division by zero)
- Element-wise multiplication
- Theta constraints (clamping)

### Pose3D Tests (`test_math_pose3d/`)
- Position constructors (x, y, z)
- Quaternion orientation (qx, qy, qz, qw)
- Rotation representations (X, Y, Z axes)
- Edge cases (identity quaternion, zero quaternion, very small/large values)

### PID Controller Tests (`test_control_pid/`)
- Proportional, integral, derivative components
- Output limits and anti-windup
- Derivative filtering
- Feedforward control

### Arm Kinematics Tests (`test_control_arm_kinematics/`)
- Forward kinematics (joint angles → end effector position)
- Inverse kinematics (position → joint angles)
- Joint limits and workspace boundaries
- Elbow-up/elbow-down configurations

### Trapezoidal Motion Profile Tests (`test_control_trapezoidal_motion_profile/`)
- Configuration validation
- Goal setting (forward, backward, zero distance)
- Profile generation (acceleration, cruise, deceleration phases)
- Edge cases (instant arrival, dt clamping, negative goals)
- Consistency over multiple updates

### S-Curve Motion Profile Tests (`test_control_scurve_motion_profile/`)
- Jerk-limited acceleration/deceleration
- S-curve phase transitions (7 phases)
- Velocity/acceleration continuity
- Comparison with trapezoidal (smoother transitions)
- Maximum jerk compliance

### Trajectory Controller Tests (`test_control_trajectory_controller/`)
- Waypoint following (straight and curved paths)
- Pure pursuit lookahead distance
- Segment transitions
- chassisToWheelSpeeds() utility function
- Goal detection and tolerance
- Edge cases (empty trajectory, single waypoint, duplicate points)

## Expected Output

Successful test run output:
```
Testing...
test/test_math_pose2d/test_pose2d.cpp:180: test_pose2d_default_constructor	[PASSED]
test/test_math_pose2d/test_pose2d.cpp:181: test_pose2d_parameterized_constructor	[PASSED]
...
================= 19 test cases: 19 succeeded in 00:00:06.184 =================
```

## Test Framework

- **Platform**: Native (runs on your computer's CPU, no hardware required)
- **Framework**: Unity Test Framework
- **Build System**: PlatformIO
- **Language**: C++17

## Troubleshooting

### Tests Won't Run

**Problem**: `Please specify 'board' in platformio.ini`
**Solution**: Make sure you're using the correct test environment names (`test-math-pose2d`, not `test_math_pose2d`)

**Problem**: `framework = arduino` error
**Solution**: The global `[env]` section should NOT contain `framework = arduino`. This should only be in `[esp32_base]` and `[teensy_base]`.

### Compilation Errors

**Problem**: `math.h: template with C linkage`
**Solution**: Ensure `lib/utils/math.h` has been renamed to `lib/utils/math_utils.h`

**Problem**: Missing includes
**Solution**: Ensure the `lib/math/` directory contains:
- `Pose2D.h` / `Pose2D.cpp`
- `Pose3D.h` / `Pose3D.cpp`
- `Vector2D.h` / `Vector2D.cpp`

### Clean Build

If tests fail unexpectedly, try a clean build:

```bash
# Clean the test environment
pio test -e test-math-pose2d --clean

# Clean all build artifacts
pio run --target clean
```

## Adding New Tests

### 1. Create Test Directory

```bash
mkdir test/test_new_component
```

### 2. Create Test File

```cpp
// test/test_new_component/test_new_component.cpp
#include <unity.h>
#include <YourComponent.h>

void setUp(void) {
    // Runs before each test
}

void tearDown(void) {
    // Runs after each test
}

void test_your_feature() {
    TEST_ASSERT_EQUAL(expected, actual);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_your_feature);
    return UNITY_END();
}
```

### 3. Add Test Environment to `platformio.ini`

```ini
[env:test-new-component]
platform = ${native_test_base.platform}
test_filter = test_new_component
build_flags = ${native_test_base.build_flags}
lib_deps = ${native_test_base.lib_deps}
```

### 4. Run Your New Test

```bash
pio test -e test-new-component
```

## Unity Assertions Reference

```cpp
// Equality
TEST_ASSERT_EQUAL(expected, actual)
TEST_ASSERT_EQUAL_FLOAT(expected, actual)

// Boolean
TEST_ASSERT_TRUE(condition)
TEST_ASSERT_FALSE(condition)

// Comparison
TEST_ASSERT_GREATER_THAN(threshold, actual)
TEST_ASSERT_LESS_THAN(threshold, actual)

// Floating point with delta
TEST_ASSERT_FLOAT_WITHIN(delta, expected, actual)

// Arrays
TEST_ASSERT_EQUAL_FLOAT_ARRAY(expected, actual, num_elements)
```

## CI/CD Integration

These tests can be integrated into your CI/CD pipeline:

```yaml
# Example GitHub Actions / Gitea Actions
- name: Run Unit Tests
  run: |
    docker compose exec -T devcontainer bash -c "
      cd /home/ubuntu/mcu_workspaces/sec26mcu && 
      pio test -e test-math-pose2d -e test-math-vector2d -e test-math-pose3d
    "
```

## Performance

Typical test execution times (on native platform):
- **Math tests** (Pose2D, Vector2D, Pose3D): ~10 seconds
- **Control tests** (PID, Arm Kinematics): ~8 seconds
- **Motion Profile tests** (Trapezoidal, S-Curve): ~12 seconds
- **Trajectory Controller tests**: ~6 seconds

**Total runtime**: ~30-40 seconds for all 139+ tests

## Hardware Tests

Some tests require physical hardware and are located in `src/test/`:

### Teensy Hardware Tests
- `teensy-test-microros-subsystem`: micro-ROS integration test (requires Teensy41 + micro-ROS agent)

Run hardware tests:
```bash
pio test -e teensy-test-microros-subsystem
```

**Note**: Hardware tests require:
- Connected MCU via USB
- Proper drivers installed
- micro-ROS agent running (for micro-ROS tests)

## Integration Tests

Multi-MCU integration tests are documented in `INTEGRATION_TESTS.md`. These tests verify:
- UWB tag-anchor communication
- Robot-field element IR communication
- Multi-robot coordination
- Micro-ROS end-to-end integration

See `INTEGRATION_TESTS.md` for detailed procedures.

## Additional Resources

- [PlatformIO Testing Documentation](https://docs.platformio.org/en/latest/advanced/unit-testing/index.html)
- [Unity Test Framework](https://github.com/ThrowTheSwitch/Unity)
- [Project CLAUDE.md](../../CLAUDE.md) - Development guidelines
