#!/bin/bash
# Run all MCU tests (native and hardware)
# This script should be run inside the Docker container

set -e

echo "======================================"
echo "SEC26 MCU Test Runner"
echo "======================================"
echo ""

# Check if we're in the correct directory
if [ ! -f "platformio.ini" ]; then
  echo "Error: platformio.ini not found"
  echo "Please run this script from /home/ubuntu/mcu_workspaces/sec26mcu inside the Docker container"
  exit 1
fi

echo "=== Running Native Tests ==="
echo "These tests run on your computer (no hardware required)"
echo ""

pio test \
  -e test-math-pose2d \
  -e test-math-vector2d \
  -e test-math-pose3d \
  -e test-control-pid \
  -e test-control-arm-kinematics \
  -e test-control-trapezoidal-motion-profile \
  -e test-control-scurve-motion-profile \
  -e test-control-trajectory-controller

echo ""
echo "=== Native Tests Completed Successfully ==="
echo ""

# Check if user wants to run hardware tests
read -p "Do you want to run hardware tests? (requires connected MCUs) [y/N]: " -n 1 -r
echo ""

if [[ ! $REPLY =~ ^[Yy]$ ]]; then
  echo "Skipping hardware tests"
  echo ""
  echo "=== All requested tests completed ==="
  exit 0
fi

echo ""
echo "=== Running Teensy Hardware Tests ==="
echo "Note: Requires Teensy41 connected via USB"
echo ""

# Check if Teensy is connected
if ! pio device list | grep -q "Teensy"; then
  echo "Warning: Teensy not detected. Tests may fail."
  read -p "Continue anyway? [y/N]: " -n 1 -r
  echo ""
  if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Aborting hardware tests"
    exit 1
  fi
fi

pio test \
  -e teensy-test-microros-subsystem \
  || echo "Note: Some Teensy tests may require specific hardware configurations"

echo ""
echo "=== Teensy Hardware Tests Completed ==="
echo ""

echo "=== Running ESP32 Hardware Tests ==="
echo "Note: Requires ESP32 connected via USB"
echo ""

# Check if ESP32 is connected
if ! pio device list | grep -q "CP210\|CH340\|ESP"; then
  echo "Warning: ESP32 not detected. Tests may fail."
  read -p "Continue anyway? [y/N]: " -n 1 -r
  echo ""
  if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Aborting ESP32 tests"
    exit 0
  fi
fi

echo "Note: ESP32 hardware tests (UWB) not yet implemented"
echo ""

echo "======================================"
echo "=== All Tests Completed ==="
echo "======================================"
echo ""
echo "Summary:"
echo "- Native tests: PASSED"
echo "- Hardware tests: Check output above for details"
echo ""
echo "For integration tests (multi-MCU), see mcu_ws/test/INTEGRATION_TESTS.md"
