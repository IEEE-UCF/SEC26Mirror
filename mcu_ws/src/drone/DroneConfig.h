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

// ── PID gains (cascaded: outer angle → inner rate) ─────────
// All use DerivativeMode::OnMeasurement for smooth response.

inline PIDController::Config rollAnglePID() {
  return {{0.2f, 0.3f, 0.05f},          // kp, ki, kd
          {-25.0f, 25.0f, -25.0f, 25.0f},  // out limits, i limits
          PIDController::DerivativeMode::OnMeasurement,
          true, 0.0f, 1e-6f, 0.1f};
}

inline PIDController::Config pitchAnglePID() {
  return {{0.2f, 0.3f, 0.05f},
          {-25.0f, 25.0f, -25.0f, 25.0f},
          PIDController::DerivativeMode::OnMeasurement,
          true, 0.0f, 1e-6f, 0.1f};
}

inline PIDController::Config rollRatePID() {
  return {{0.15f, 0.2f, 0.0002f},
          {-1.0f, 1.0f, -25.0f, 25.0f},
          PIDController::DerivativeMode::OnMeasurement,
          true, 0.0f, 1e-6f, 0.1f};
}

inline PIDController::Config pitchRatePID() {
  return {{0.15f, 0.2f, 0.0002f},
          {-1.0f, 1.0f, -25.0f, 25.0f},
          PIDController::DerivativeMode::OnMeasurement,
          true, 0.0f, 1e-6f, 0.1f};
}

inline PIDController::Config yawRatePID() {
  return {{0.3f, 0.05f, 0.00015f},
          {-1.0f, 1.0f, -25.0f, 25.0f},
          PIDController::DerivativeMode::OnError,
          true, 0.0f, 1e-6f, 0.1f};
}

inline PIDController::Config altitudePID() {
  return {{0.8f, 0.15f, 0.4f},
          {-0.3f, 0.3f, -0.3f, 0.3f},
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
constexpr float EKF_MEASURE_NOISE_UWB = 0.15f;   // m²
constexpr float EKF_OUTLIER_GATE_M = 1.0f;       // Residual reject

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
