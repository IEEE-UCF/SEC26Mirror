#!/bin/bash
# run_all_mcu_tests.sh
# Comprehensive test script for all MCU tests (native + hardware)
# Run from: /home/ubuntu/mcu_workspaces/sec26mcu

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Test counters
TOTAL_TESTS=0
PASSED_TESTS=0
FAILED_TESTS=0

# Arrays to track results
declare -a FAILED_TEST_NAMES
declare -a PASSED_TEST_NAMES

print_header() {
    echo -e "\n${BLUE}================================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}================================================${NC}\n"
}

print_section() {
    echo -e "\n${YELLOW}--- $1 ---${NC}\n"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ $1${NC}"
}

run_native_test() {
    local test_env=$1
    local test_name=$2

    print_section "Running: $test_name"
    TOTAL_TESTS=$((TOTAL_TESTS + 1))

    if pio test -e "$test_env" > /tmp/test_output.txt 2>&1; then
        PASSED_TESTS=$((PASSED_TESTS + 1))
        PASSED_TEST_NAMES+=("$test_name")
        print_success "$test_name passed"

        # Extract test count
        if grep -q "test cases:" /tmp/test_output.txt; then
            grep "test cases:" /tmp/test_output.txt | tail -1
        fi
    else
        FAILED_TESTS=$((FAILED_TESTS + 1))
        FAILED_TEST_NAMES+=("$test_name")
        print_error "$test_name failed"

        # Show failure details
        echo "Error output:"
        cat /tmp/test_output.txt | tail -20
    fi
}

build_mcu_test() {
    local test_env=$1
    local test_name=$2

    print_section "Building: $test_name"
    TOTAL_TESTS=$((TOTAL_TESTS + 1))

    if pio run -e "$test_env" > /tmp/build_output.txt 2>&1; then
        PASSED_TESTS=$((PASSED_TESTS + 1))
        PASSED_TEST_NAMES+=("$test_name (build)")
        print_success "$test_name build passed"
    else
        FAILED_TESTS=$((FAILED_TESTS + 1))
        FAILED_TEST_NAMES+=("$test_name (build)")
        print_error "$test_name build failed"

        echo "Error output:"
        cat /tmp/build_output.txt | tail -20
    fi
}

print_summary() {
    print_header "TEST SUMMARY"

    echo "Total Tests Run: $TOTAL_TESTS"
    echo -e "${GREEN}Passed: $PASSED_TESTS${NC}"
    echo -e "${RED}Failed: $FAILED_TESTS${NC}"
    echo ""

    if [ $FAILED_TESTS -gt 0 ]; then
        print_error "Failed Tests:"
        for test in "${FAILED_TEST_NAMES[@]}"; do
            echo "  - $test"
        done
        echo ""
    fi

    if [ $PASSED_TESTS -gt 0 ]; then
        print_success "Passed Tests:"
        for test in "${PASSED_TEST_NAMES[@]}"; do
            echo "  - $test"
        done
        echo ""
    fi

    if [ $FAILED_TESTS -eq 0 ]; then
        print_success "ALL TESTS PASSED!"
        return 0
    else
        print_error "SOME TESTS FAILED"
        return 1
    fi
}

# Main execution
main() {
    print_header "SEC26 MCU Test Suite"
    print_info "Running all native and hardware tests"
    print_info "Started at: $(date)"

    # Clean previous build artifacts
    print_section "Cleaning previous builds"
    pio run --target clean > /dev/null 2>&1 || true

    # ========================================
    # NATIVE TESTS (Math)
    # ========================================
    print_header "NATIVE TESTS - MATH"

    run_native_test "test-math-pose2d" "Pose2D Math"
    run_native_test "test-math-vector2d" "Vector2D Math"
    run_native_test "test-math-pose3d" "Pose3D Math"

    # ========================================
    # NATIVE TESTS (Control)
    # ========================================
    print_header "NATIVE TESTS - CONTROL"

    run_native_test "test-control-pid" "PID Controller"
    run_native_test "test-control-arm-kinematics" "Arm Kinematics"
    run_native_test "test-control-trapezoidal-motion-profile" "Trapezoidal Motion Profile"
    run_native_test "test-control-scurve-motion-profile" "S-Curve Motion Profile"
    run_native_test "test-control-trajectory-controller" "Trajectory Controller"

    # ========================================
    # NATIVE TESTS (Utils)
    # ========================================
    print_header "NATIVE TESTS - UTILITIES"

    run_native_test "test-utils-filters" "Signal Filters"
    run_native_test "test-utils-units" "Unit Conversions"

    # ========================================
    # MCU HARDWARE TESTS (Build Only)
    # ========================================
    print_header "MCU HARDWARE TESTS - BUILD VERIFICATION"
    print_info "Note: These tests only verify builds. Upload and run manually on hardware."

    build_mcu_test "teensy-test-microros-subsystem" "Teensy micro-ROS Subsystem"
    build_mcu_test "teensy-test-battery-subsystem" "Teensy Battery Subsystem"
    build_mcu_test "teensy-test-sensor-subsystem" "Teensy Sensor Subsystem"

    # ========================================
    # PRODUCTION BUILDS (Sanity Check)
    # ========================================
    print_header "PRODUCTION BUILD VERIFICATION"
    print_info "Verifying main production builds compile"

    build_mcu_test "robot" "Robot (Teensy41)"

    # Print final summary
    print_summary
}

# Run main function
main

# Exit with appropriate code
if [ $FAILED_TESTS -eq 0 ]; then
    exit 0
else
    exit 1
fi
