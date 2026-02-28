/**
 * @file RobotConfig.h
 * @author Trevor Cannon
 * @brief Robot configuration constants for drive kinematics and control.
 * @date 12/11/2025
 *
 * Physical measurements are defined in inches (what you measure with a ruler).
 * Metric equivalents are computed at compile time via constexpr converters.
 * All control loops operate in meters/radians/seconds internally.
 */

#ifndef ROBOTCONFIG_H
#define ROBOTCONFIG_H

#define _USE_MATH_DEFINES
#include <cmath>

#include <units.h>

namespace RobotConfig {

using namespace secbot::utils::units;

// ═══════════════════════════════════════════════════════════════════════════
//  Physical dimensions (inches — update these when measured!)
// ═══════════════════════════════════════════════════════════════════════════

// TODO: MEASURE AND UPDATE THESE VALUES
constexpr float TRACK_WIDTH_IN = 10.0f;           // center-to-center wheel dist
constexpr float WHEEL_DIAMETER_IN = 3.25f;         // Vex omniwheel outer diameter

// Metric equivalents (used by control loops)
constexpr float TRACK_WIDTH_M = in_to_m(TRACK_WIDTH_IN);
constexpr float WHEEL_DIAMETER_M = in_to_m(WHEEL_DIAMETER_IN);
constexpr float WHEEL_RADIUS_M = WHEEL_DIAMETER_M * 0.5f;
constexpr float WHEEL_CIRCUMFERENCE_M =
    static_cast<float>(M_PI) * WHEEL_DIAMETER_M;

// ═══════════════════════════════════════════════════════════════════════════
//  Encoder configuration
// ═══════════════════════════════════════════════════════════════════════════

constexpr int RAW_TICKS_PER_REVOLUTION = 3;
constexpr int GEAR_RATIO = 34;
constexpr long TICKS_PER_REVOLUTION =
    RAW_TICKS_PER_REVOLUTION * GEAR_RATIO;  // 102

constexpr float DIST_PER_TICK_M =
    WHEEL_CIRCUMFERENCE_M / TICKS_PER_REVOLUTION;

// ═══════════════════════════════════════════════════════════════════════════
//  Motor / encoder channel mapping (PCA9685 motor board indices)
// ═══════════════════════════════════════════════════════════════════════════

constexpr uint8_t LEFT_MOTOR_IDX = 1;
constexpr uint8_t RIGHT_MOTOR_IDX = 0;
constexpr uint8_t LEFT_ENCODER_IDX = 1;   // QTimer channel for left motor FG
constexpr uint8_t RIGHT_ENCODER_IDX = 0;  // QTimer channel for right motor FG

// Set to true if positive encoder ticks correspond to forward motion
// for each side. Flip if a side reads backwards.
constexpr bool LEFT_ENCODER_INVERTED = false;
constexpr bool RIGHT_ENCODER_INVERTED = false;

// ═══════════════════════════════════════════════════════════════════════════
//  Velocity / acceleration limits (in m/s, m/s^2, rad/s, rad/s^2)
// ═══════════════════════════════════════════════════════════════════════════

constexpr float MAX_LINEAR_VEL_MPS = inps_to_mps(30.0f);        // ~0.76 m/s
constexpr float MAX_LINEAR_ACCEL_MPS2 = inps_to_mps(30.0f);     // ~0.76 m/s^2
constexpr float MAX_LINEAR_JERK_MPS3 = inps_to_mps(120.0f);     // ~3.05 m/s^3
constexpr float MAX_ANGULAR_VEL_RADPS = 4.0f;                   // rad/s
constexpr float MAX_ANGULAR_ACCEL_RADPS2 = 10.0f;               // rad/s^2
constexpr float MAX_ANGULAR_JERK_RADPS3 = 40.0f;                // rad/s^3

// ═══════════════════════════════════════════════════════════════════════════
//  Wheel velocity PID (tuning constants)
// ═══════════════════════════════════════════════════════════════════════════

// These control individual wheel speed tracking.
// Output range maps to MotorManagerSubsystem::setSpeed() range [-1.0, 1.0].
constexpr float WHEEL_PID_KP = 0.8f;
constexpr float WHEEL_PID_KI = 0.1f;
constexpr float WHEEL_PID_KD = 0.05f;
constexpr float WHEEL_PID_OUT_MIN = -1.0f;
constexpr float WHEEL_PID_OUT_MAX = 1.0f;
constexpr float WHEEL_PID_I_MIN = -0.3f;
constexpr float WHEEL_PID_I_MAX = 0.3f;
constexpr float WHEEL_PID_D_FILTER = 0.1f;

// ═══════════════════════════════════════════════════════════════════════════
//  Pose drive gains
// ═══════════════════════════════════════════════════════════════════════════

constexpr float POSE_K_LINEAR = 2.0f;     // P gain for distance error
constexpr float POSE_K_ANGULAR = 4.0f;    // P gain for heading error
constexpr float POSE_DIST_TOL_M = 0.015f; // 1.5cm goal tolerance

// ═══════════════════════════════════════════════════════════════════════════
//  Trajectory controller
// ═══════════════════════════════════════════════════════════════════════════

constexpr float TRAJ_LOOKAHEAD_M = in_to_m(6.0f);    // pure pursuit dist
constexpr float TRAJ_CRUISE_V_MPS = inps_to_mps(12.0f);
constexpr float TRAJ_SLOWDOWN_M = in_to_m(8.0f);
constexpr float TRAJ_MIN_V_MPS = inps_to_mps(2.0f);
constexpr float TRAJ_POS_TOL_M = in_to_m(0.5f);
constexpr float TRAJ_HEADING_TOL_RAD = 0.1f;
constexpr float TRAJ_K_HEADING = 3.0f;
constexpr float TRAJ_ADVANCE_TOL_M = in_to_m(3.0f);

// ═══════════════════════════════════════════════════════════════════════════
//  Safety
// ═══════════════════════════════════════════════════════════════════════════

constexpr uint32_t COMMAND_TIMEOUT_MS = 500;  // stop if no cmd for this long

// ═══════════════════════════════════════════════════════════════════════════
//  Starting pose (meters, radians)
// ═══════════════════════════════════════════════════════════════════════════

constexpr float START_X = 0.0f;
constexpr float START_Y = 0.0f;
constexpr float START_THETA = 0.0f;

}  // namespace RobotConfig

#endif
