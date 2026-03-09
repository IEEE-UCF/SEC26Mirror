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

constexpr float TRACK_WIDTH_IN = 10.0f;           // center-to-center wheel dist
constexpr float WHEEL_DIAMETER_IN = 3.25f;         // Vex omniwheel outer diameter

// Metric equivalents (used by control loops)
constexpr float TRACK_WIDTH_M = in_to_m(TRACK_WIDTH_IN);
constexpr float WHEEL_DIAMETER_M = in_to_m(WHEEL_DIAMETER_IN);
constexpr float WHEEL_RADIUS_M = WHEEL_DIAMETER_M * 0.5f;
constexpr float WHEEL_CIRCUMFERENCE_M =
    static_cast<float>(M_PI) * WHEEL_DIAMETER_M;

// ═══════════════════════════════════════════════════════════════════════════
//  Motor specs (before any gear reduction)
// ═══════════════════════════════════════════════════════════════════════════

constexpr float MOTOR_RATED_RPM = 9592.0f;   // continuous rated speed
constexpr float MOTOR_MAX_RPM = 11880.0f;    // absolute max (no-load)

// ═══════════════════════════════════════════════════════════════════════════
//  Gear train & encoder configuration
// ═══════════════════════════════════════════════════════════════════════════

constexpr int RAW_TICKS_PER_REVOLUTION = 3;
constexpr int MOTOR_GEAR_RATIO = 34;         // internal gearbox
constexpr int EXTERNAL_GEAR_SMALL = 36;      // motor-side gear teeth
constexpr int EXTERNAL_GEAR_LARGE = 60;      // wheel-side gear teeth

// Total gear ratio: motor shaft → wheel
constexpr float TOTAL_GEAR_RATIO =
    static_cast<float>(MOTOR_GEAR_RATIO) *
    static_cast<float>(EXTERNAL_GEAR_LARGE) /
    static_cast<float>(EXTERNAL_GEAR_SMALL);  // 56.67

constexpr long TICKS_PER_REVOLUTION =
    RAW_TICKS_PER_REVOLUTION * MOTOR_GEAR_RATIO
    * EXTERNAL_GEAR_LARGE / EXTERNAL_GEAR_SMALL;  // 170

constexpr float DIST_PER_TICK_M =
    WHEEL_CIRCUMFERENCE_M / TICKS_PER_REVOLUTION;

// ═══════════════════════════════════════════════════════════════════════════
//  Physical wheel velocity limits (derived from motor specs + gear + wheel)
// ═══════════════════════════════════════════════════════════════════════════

// wheel_vel = (motor_rpm / gear_ratio) × wheel_circumference / 60
constexpr float WHEEL_VEL_AT_RATED_RPM =
    (MOTOR_RATED_RPM / TOTAL_GEAR_RATIO) * WHEEL_CIRCUMFERENCE_M / 60.0f;
    // ≈ 0.732 m/s

constexpr float WHEEL_VEL_AT_MAX_RPM =
    (MOTOR_MAX_RPM / TOTAL_GEAR_RATIO) * WHEEL_CIRCUMFERENCE_M / 60.0f;
    // ≈ 0.907 m/s

// ═══════════════════════════════════════════════════════════════════════════
//  Motor / encoder channel mapping (PCA9685 motor board indices)
// ═══════════════════════════════════════════════════════════════════════════

constexpr uint8_t LEFT_MOTOR_IDX = 1;
constexpr uint8_t RIGHT_MOTOR_IDX = 0;
constexpr uint8_t LEFT_ENCODER_IDX = 1;   // QTimer channel for left motor FG
constexpr uint8_t RIGHT_ENCODER_IDX = 0;  // QTimer channel for right motor FG

// Motor direction multipliers (1.0 = normal, -1.0 = reversed).
// Left motor is mounted opposite to right on a tank-drive chassis,
// so its command sign must be inverted for "forward" to mean "robot forward".
constexpr float LEFT_MOTOR_MULTIPLIER = -1.0f;
constexpr float RIGHT_MOTOR_MULTIPLIER = 1.0f;

// Encoder inversion: set true when the motor multiplier is negative.
// The encoder direction comes from MotorManager's intended direction (sign of
// the speed command). When a motor multiplier flips the command sign, the
// encoder counts in the opposite direction — this flag compensates.
constexpr bool LEFT_ENCODER_INVERTED = true;
constexpr bool RIGHT_ENCODER_INVERTED = false;

// ═══════════════════════════════════════════════════════════════════════════
//  Velocity / acceleration limits (in m/s, m/s^2, rad/s, rad/s^2)
// ═══════════════════════════════════════════════════════════════════════════

constexpr float MAX_LINEAR_VEL_MPS = 0.7f;                      // chassis speed limit (m/s)
constexpr float MAX_LINEAR_ACCEL_MPS2 = 0.7f;                   // m/s^2
constexpr float MAX_LINEAR_JERK_MPS3 = inps_to_mps(120.0f);     // ~3.05 m/s^3
constexpr float MAX_ANGULAR_VEL_RADPS = 6.0f;                   // rad/s
constexpr float MAX_ANGULAR_ACCEL_RADPS2 = 15.0f;               // rad/s^2
constexpr float MAX_ANGULAR_JERK_RADPS3 = 60.0f;                // rad/s^3

// Effective wheel speed limit used for saturation.
// Uses rated RPM (safe continuous). Change to WHEEL_VEL_AT_MAX_RPM for bursts.
constexpr float MAX_WHEEL_VEL_MPS = WHEEL_VEL_AT_RATED_RPM;

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
constexpr float POSE_HEADING_TOL_RAD = 0.05f; // ~3 deg final heading tolerance

// Gyro heading hold (straight-line drift correction in velocity mode)
constexpr float GYRO_HOLD_KP = 2.0f;           // P gain for heading correction
constexpr float GYRO_HOLD_THRESHOLD = 0.05f;   // rad/s — below this, hold heading

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
