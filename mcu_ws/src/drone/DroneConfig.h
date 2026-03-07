#pragma once
/**
 * @file DroneConfig.h
 * @brief Central configuration for drone PID gains, task rates, safety
 *        thresholds, and hardware enable flags.
 */

#include <pid_controller.h>

// Hardware enable flags (overridable via build_flags)
#ifndef DRONE_ENABLE_UWB
#define DRONE_ENABLE_UWB 1
#endif
#ifndef DRONE_ENABLE_HEIGHT
#define DRONE_ENABLE_HEIGHT 1
#endif

namespace Drone {
namespace Config {

// ── Motor PWM ──────────────────────────────────────────────
constexpr uint32_t MOTOR_PWM_FREQ = 20000;     // 20 kHz
constexpr uint8_t MOTOR_PWM_RESOLUTION = 10;   // 10-bit (0-1023)
constexpr uint32_t MOTOR_PWM_MAX = (1 << MOTOR_PWM_RESOLUTION) - 1;

// ── PID gains (cascaded: outer angle -> inner rate) ─────────

inline PIDController::Config rollAnglePID() {
  return {{5.0f, 0.5f, 0.0f},               // kp, ki, kd
          {-250.0f, 250.0f, -30.0f, 30.0f},  // out=deg/s, i bounded
          PIDController::DerivativeMode::OnMeasurement,
          true, 0.0f, 1e-6f, 0.1f};
}

inline PIDController::Config pitchAnglePID() {
  return {{5.0f, 0.5f, 0.0f},
          {-250.0f, 250.0f, -30.0f, 30.0f},
          PIDController::DerivativeMode::OnMeasurement,
          true, 0.0f, 1e-6f, 0.1f};
}

inline PIDController::Config rollRatePID() {
  return {{0.003f, 0.001f, 0.00003f},
          {-0.5f, 0.5f, -0.3f, 0.3f},       // i_limit <= out_limit!
          PIDController::DerivativeMode::OnMeasurement,
          true, 0.7f, 1e-6f, 0.1f};          // d_filter_alpha=0.7
}

inline PIDController::Config pitchRatePID() {
  return {{0.003f, 0.001f, 0.00003f},
          {-0.5f, 0.5f, -0.3f, 0.3f},
          PIDController::DerivativeMode::OnMeasurement,
          true, 0.7f, 1e-6f, 0.1f};
}

inline PIDController::Config yawRatePID() {
  return {{0.005f, 0.001f, 0.0f},
          {-0.3f, 0.3f, -0.2f, 0.2f},
          PIDController::DerivativeMode::OnError,
          true, 0.0f, 1e-6f, 0.1f};
}

inline PIDController::Config altitudePID() {
  return {{1.2f, 0.15f, 0.0f},              // no D — velocity feedforward instead
          {-0.3f, 0.3f, -0.2f, 0.2f},
          PIDController::DerivativeMode::OnMeasurement,
          true, 0.0f, 1e-6f, 1.0f};
}

// Altitude velocity damping (inner loop of altitude cascade)
inline PIDController::Config altitudeVelocityPID() {
  return {{0.5f, 0.0f, 0.0f},               // P-only velocity damping
          {-0.2f, 0.2f, -0.1f, 0.1f},
          PIDController::DerivativeMode::OnMeasurement,
          true, 0.0f, 1e-6f, 1.0f};
}

constexpr float HOVER_THROTTLE = 0.45f;

// ── Angle / rate limits ────────────────────────────────────
constexpr float MAX_ROLL_DEG = 30.0f;
constexpr float MAX_PITCH_DEG = 30.0f;
constexpr float MAX_YAW_RATE_DPS = 160.0f;

// ── cmd_vel mapping scales ─────────────────────────────────
constexpr float CMD_VEL_PITCH_SCALE = 15.0f;   // linear.x → pitch angle
constexpr float CMD_VEL_ROLL_SCALE = 15.0f;    // linear.y → roll angle
constexpr float CMD_VEL_ALT_SCALE = 0.5f;      // linear.z → altitude rate
constexpr float CMD_VEL_YAW_SCALE = 90.0f;     // angular.z → yaw rate

// ── Safety thresholds ──────────────────────────────────────
constexpr uint32_t HEIGHT_TIMEOUT_MS = 200;     // VL53L0X stale data
constexpr uint32_t UWB_TIMEOUT_MS = 2000;       // No valid UWB ranges
constexpr uint32_t MICROROS_TIMEOUT_MS = 3000;  // Agent disconnect
constexpr uint32_t CMD_VEL_TIMEOUT_MS = 500;    // No cmd_vel → hover
constexpr float ALTITUDE_CEILING_M = 2.0f;      // Max altitude
constexpr float LANDING_DESCENT_RATE = 0.15f;   // m/s during landing
constexpr float EMERGENCY_THROTTLE = 0.25f;     // Open-loop descent
constexpr uint32_t EMERGENCY_DISARM_MS = 5000;  // Blind landing timeout
constexpr float LANDED_ALT_M = 0.05f;           // On-ground threshold

// ── FreeRTOS task config ───────────────────────────────────
constexpr int FLIGHT_TASK_PRIORITY = 5;
constexpr int SAFETY_TASK_PRIORITY = 4;
constexpr int HEIGHT_TASK_PRIORITY = 3;
constexpr int UWB_TASK_PRIORITY = 3;

constexpr uint32_t FLIGHT_TASK_STACK = 2048;
constexpr uint32_t SAFETY_TASK_STACK = 1024;
constexpr uint32_t HEIGHT_TASK_STACK = 1536;
constexpr uint32_t UWB_TASK_STACK = 2048;

constexpr uint32_t FLIGHT_RATE_HZ = 250;
constexpr uint32_t HEIGHT_RATE_HZ = 50;
constexpr uint32_t UWB_RATE_HZ = 20;
constexpr uint32_t SAFETY_RATE_HZ = 10;
constexpr uint32_t STATE_PUB_RATE_HZ = 10;

// ── EKF noise parameters ──────────────────────────────────
constexpr float EKF_PROCESS_NOISE_POS = 0.01f;   // m²
constexpr float EKF_PROCESS_NOISE_VEL = 0.1f;    // (m/s)²
constexpr float EKF_MEASURE_NOISE_UWB = 0.15f;   // m² per range
constexpr float EKF_MAHALANOBIS_GATE_SQ = 9.0f;  // chi² gate (3σ for 1-DOF)
constexpr float EKF_COV_MAX_POS = 10.0f;         // max position variance
constexpr float EKF_COV_MAX_VEL = 5.0f;          // max velocity variance

// ── IMU config ─────────────────────────────────────────────
constexpr uint8_t IMU_I2C_ADDR = 0x4A;
constexpr uint32_t IMU_ROTATION_REPORT_US = 5000;   // 200 Hz
constexpr uint32_t IMU_GYRO_REPORT_US = 2500;       // 400 Hz
constexpr uint32_t IMU_ACCEL_REPORT_US = 5000;       // 200 Hz

// ── Height sensor config ───────────────────────────────────
constexpr uint16_t HEIGHT_TIMING_BUDGET_MS = 20;
constexpr float HEIGHT_MAX_VALID_M = 4.0f;
constexpr float HEIGHT_MIN_VALID_M = 0.01f;

}  // namespace Config
}  // namespace Drone
