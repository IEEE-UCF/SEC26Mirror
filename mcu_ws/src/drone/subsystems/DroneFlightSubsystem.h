#pragma once
/**
 * @file DroneFlightSubsystem.h
 * @brief Cascaded PID flight controller as an RTOSSubsystem.
 *
 * Architecture:
 *   Outer loop: desired angle → PID → desired angular rate
 *   Inner loop: desired rate → PID → motor correction
 *   Altitude:   desired alt → PID → throttle correction (+ hover baseline)
 *   Mixer:      X-quad FL/FR/BR/BL with roll/pitch/yaw corrections
 *
 * Runs at 250Hz via beginThreadedPinned(). Reads cached IMU data from
 * GyroSubsystem and altitude from HeightSubsystem (thread-safe accessors).
 */

#include <Arduino.h>
#include <RTOSSubsystem.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <pid_controller.h>
#include <filters.h>

#include "../DroneConfig.h"
#include "GyroSubsystem.h"

#if DRONE_ENABLE_HEIGHT
#include "HeightSubsystem.h"
#endif

#include "DroneEKFSubsystem.h"

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

class DroneFlightSubsystem : public Subsystem::RTOSSubsystem {
 public:
  DroneFlightSubsystem(const FlightMotorPins& pins,
                       GyroSubsystem& gyro,
                       DroneEKFSubsystem& ekf
#if DRONE_ENABLE_HEIGHT
                       ,
                       HeightSubsystem& height
#endif
                       )
      : RTOSSubsystem(setup_),
        pins_(pins),
        gyro_(gyro),
        ekf_(ekf)
#if DRONE_ENABLE_HEIGHT
        ,
        height_(height)
#endif
  {
  }

  // RTOSSubsystem lifecycle
  bool init() override;
  void begin() override {}
  void update() override;
  void pause() override {}
  void reset() override {}
  const char* getInfo() override { return "flight"; }

  void arm();
  void disarm();
  void killMotors();  // Immediate stop, bypasses slew limiter
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

  // Runtime parameter tuning (returns true if param_name was recognized)
  bool setParam(const char* param_name, float value);

  // Runtime-mutable config (initialized from DroneConfig.h defaults)
  float hover_throttle = Config::HOVER_THROTTLE;
  float motor_max_slew = Config::MOTOR_MAX_SLEW;
  float max_roll_deg = Config::MAX_ROLL_DEG;
  float max_pitch_deg = Config::MAX_PITCH_DEG;
  float max_yaw_rate_dps = Config::MAX_YAW_RATE_DPS;

 private:
  void compute(const IMUData& imu, float altitude_m, float dt);
  void controlAngle(const IMUData& imu, const FlightSetpoint& sp, float dt);
  void controlAltitude(const FlightSetpoint& sp, float altitude_m, float dt);
  void controlMixer();
  void writeMotors();
  void resetPIDs();

  Classes::BaseSetup setup_{"flight"};
  FlightMotorPins pins_;
  GyroSubsystem& gyro_;
  DroneEKFSubsystem& ekf_;
#if DRONE_ENABLE_HEIGHT
  HeightSubsystem& height_;
#endif

  FlightSetpoint sp_;
  SemaphoreHandle_t sp_mutex_ = nullptr;
  bool armed_ = false;
  bool override_active_ = false;

  // dt tracking
  uint32_t last_us_ = 0;
  float last_dt_ = 0.0f;

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
  float alt_since_change_ = 0.0f;  // accumulated dt since last sensor change

  // PID outputs
  float roll_correction_ = 0.0f;
  float pitch_correction_ = 0.0f;
  float yaw_correction_ = 0.0f;
  float throttle_ = 0.0f;

  // Motor outputs (0.0 to 1.0) [FL, FR, BR, BL]
  float motors_[4] = {};
  float motors_prev_[4] = {};  // previous tick for slew rate limiting
};

}  // namespace Drone
