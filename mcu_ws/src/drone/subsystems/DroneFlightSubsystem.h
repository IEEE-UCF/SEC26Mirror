#pragma once
/**
 * @file DroneFlightSubsystem.h
 * @brief Cascaded PID flight controller using lib PIDController.
 *
 * Architecture:
 *   Outer loop: desired angle → PID → desired angular rate
 *   Inner loop: desired rate → PID → motor correction
 *   Altitude:   desired alt → PID → throttle correction (+ hover baseline)
 *   Mixer:      X-quad FL/FR/BR/BL with roll/pitch/yaw corrections
 */

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <pid_controller.h>
#include <filters.h>

#include "../DroneConfig.h"
#include "GyroSubsystem.h"

namespace Drone {

// Setpoint from state machine or ROS command
struct FlightSetpoint {
  float roll_des = 0.0f;      // desired roll angle (degrees)
  float pitch_des = 0.0f;     // desired pitch angle (degrees)
  float yaw_rate_des = 0.0f;  // desired yaw rate (deg/s)
  float throttle = 0.0f;      // 0.0 to 1.0 (overridden if altitude hold)
  float altitude_des = 0.0f;  // desired altitude (meters)
  bool altitude_hold = false;
};

struct FlightMotorPins {
  uint8_t fl;
  uint8_t fr;
  uint8_t br;
  uint8_t bl;
};

class DroneFlightSubsystem {
 public:
  explicit DroneFlightSubsystem(const FlightMotorPins& pins) : pins_(pins) {}

  void init();
  void update(const IMUData& imu, float altitude_m, float dt);
  void arm();
  void disarm();
  bool isArmed() const { return armed_; }

  // Thread-safe setpoint access (written by state machine, read by flight task)
  void setSetpoint(const FlightSetpoint& sp) {
    xSemaphoreTake(sp_mutex_, portMAX_DELAY);
    sp_ = sp;
    xSemaphoreGive(sp_mutex_);
  }
  FlightSetpoint getSetpoint() const {
    xSemaphoreTake(sp_mutex_, portMAX_DELAY);
    FlightSetpoint copy = sp_;
    xSemaphoreGive(sp_mutex_);
    return copy;
  }

  // Direct motor override for bench testing (bypasses PID)
  void setMotorsOverride(const float speeds[4]);
  bool isOverrideActive() const { return override_active_; }
  void clearOverride() { override_active_ = false; }

  float getMotor(uint8_t idx) const { return (idx < 4) ? motors_[idx] : 0.0f; }

 private:
  void controlAngle(const IMUData& imu, const FlightSetpoint& sp, float dt);
  void controlAltitude(const FlightSetpoint& sp, float altitude_m, float dt);
  void controlMixer();
  void writeMotors();
  void resetPIDs();

  FlightMotorPins pins_;
  FlightSetpoint sp_;
  SemaphoreHandle_t sp_mutex_ = nullptr;
  bool armed_ = false;
  bool override_active_ = false;

  // Cascaded PID controllers
  PIDController roll_angle_pid_;
  PIDController pitch_angle_pid_;
  PIDController roll_rate_pid_;
  PIDController pitch_rate_pid_;
  PIDController yaw_rate_pid_;
  PIDController altitude_pid_;
  PIDController alt_vel_pid_;  // velocity damping for altitude

  // Altitude velocity estimate (runs at sensor rate, not control rate)
  secbot::utils::LowPass1P alt_vel_filter_;
  float alt_prev_ = 0.0f;
  float alt_vel_ = 0.0f;       // filtered vertical velocity (m/s)
  bool alt_initialized_ = false;
  uint32_t alt_update_count_ = 0;  // detect when sensor value changes

  // PID outputs
  float roll_correction_ = 0.0f;
  float pitch_correction_ = 0.0f;
  float yaw_correction_ = 0.0f;
  float throttle_ = 0.0f;

  // Motor outputs (0.0 to 1.0) [FL, FR, BR, BL]
  float motors_[4] = {};
};

}  // namespace Drone
