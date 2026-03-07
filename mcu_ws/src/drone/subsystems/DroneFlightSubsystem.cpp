#include "DroneFlightSubsystem.h"

#include <math_utils.h>

using secbot::utils::clamp;

namespace Drone {

void DroneFlightSubsystem::init() {
  sp_mutex_ = xSemaphoreCreateMutex();

  // Configure motor PWM (ESP32 LEDC old API)
  ledcSetup(0, Config::MOTOR_PWM_FREQ, Config::MOTOR_PWM_RESOLUTION);
  ledcSetup(1, Config::MOTOR_PWM_FREQ, Config::MOTOR_PWM_RESOLUTION);
  ledcSetup(2, Config::MOTOR_PWM_FREQ, Config::MOTOR_PWM_RESOLUTION);
  ledcSetup(3, Config::MOTOR_PWM_FREQ, Config::MOTOR_PWM_RESOLUTION);
  ledcAttachPin(pins_.fl, 0);
  ledcAttachPin(pins_.fr, 1);
  ledcAttachPin(pins_.br, 2);
  ledcAttachPin(pins_.bl, 3);

  // Configure PID controllers
  roll_angle_pid_.configure(Config::rollAnglePID());
  pitch_angle_pid_.configure(Config::pitchAnglePID());
  roll_rate_pid_.configure(Config::rollRatePID());
  pitch_rate_pid_.configure(Config::pitchRatePID());
  yaw_rate_pid_.configure(Config::yawRatePID());
  altitude_pid_.configure(Config::altitudePID());

  // Altitude velocity LPF (alpha=0.3)
  alt_vel_filter_.setAlpha(0.3f);

  disarm();
}

void DroneFlightSubsystem::arm() {
  resetPIDs();
  override_active_ = false;
  armed_ = true;
}

void DroneFlightSubsystem::disarm() {
  armed_ = false;
  override_active_ = false;
  motors_[0] = motors_[1] = motors_[2] = motors_[3] = 0.0f;
  writeMotors();
  resetPIDs();
}

void DroneFlightSubsystem::resetPIDs() {
  roll_angle_pid_.reset();
  pitch_angle_pid_.reset();
  roll_rate_pid_.reset();
  pitch_rate_pid_.reset();
  yaw_rate_pid_.reset();
  altitude_pid_.reset();
  alt_vel_filter_.reset(0.0f, false);
  alt_prev_ = 0.0f;
  alt_initialized_ = false;
  roll_correction_ = pitch_correction_ = yaw_correction_ = 0.0f;
  throttle_ = 0.0f;
}

void DroneFlightSubsystem::update(const IMUData& imu, float altitude_m,
                                  float dt) {
  if (!armed_ || dt <= 0.0f || dt > 0.1f) {
    motors_[0] = motors_[1] = motors_[2] = motors_[3] = 0.0f;
    writeMotors();
    return;
  }

  // If motor override is active, skip PID
  if (override_active_) {
    writeMotors();
    return;
  }

  // Read setpoint (thread-safe)
  FlightSetpoint sp;
  xSemaphoreTake(sp_mutex_, portMAX_DELAY);
  sp = sp_;
  xSemaphoreGive(sp_mutex_);

  // 1. Altitude hold (modifies throttle_)
  throttle_ = sp.throttle;
  if (sp.altitude_hold) {
    controlAltitude(sp, altitude_m, dt);
  }

  // Clamp desired angles
  sp.roll_des = clamp(sp.roll_des, -Config::MAX_ROLL_DEG, Config::MAX_ROLL_DEG);
  sp.pitch_des =
      clamp(sp.pitch_des, -Config::MAX_PITCH_DEG, Config::MAX_PITCH_DEG);

  // 2. Cascaded angle → rate PID (using clamped sp)
  controlAngle(imu, sp, dt);

  // 3. Motor mixing
  controlMixer();

  // 4. Write to ESCs
  writeMotors();
}

void DroneFlightSubsystem::controlAngle(const IMUData& imu,
                                        const FlightSetpoint& sp, float dt) {
  // Outer loop: angle → desired rate
  float desired_roll_rate =
      roll_angle_pid_.update(sp.roll_des, imu.roll, dt);
  float desired_pitch_rate =
      pitch_angle_pid_.update(sp.pitch_des, imu.pitch, dt);

  // Inner loop: rate → motor correction
  roll_correction_ =
      roll_rate_pid_.update(desired_roll_rate, imu.gyro_x, dt);
  pitch_correction_ =
      pitch_rate_pid_.update(desired_pitch_rate, imu.gyro_y, dt);

  // Yaw rate PID (single loop)
  yaw_correction_ =
      yaw_rate_pid_.update(sp.yaw_rate_des, imu.gyro_z, dt);

  // Reset integrators when throttle is very low (on the ground)
  if (throttle_ < 0.05f) {
    roll_angle_pid_.reset();
    pitch_angle_pid_.reset();
    roll_rate_pid_.reset();
    pitch_rate_pid_.reset();
    yaw_rate_pid_.reset();
  }
}

void DroneFlightSubsystem::controlAltitude(const FlightSetpoint& sp,
                                           float altitude_m, float dt) {
  // Estimate vertical velocity
  if (!alt_initialized_) {
    alt_prev_ = altitude_m;
    alt_initialized_ = true;
    return;
  }
  float raw_vel = (dt > 0.0f) ? (altitude_m - alt_prev_) / dt : 0.0f;
  alt_vel_filter_.update(raw_vel);
  alt_prev_ = altitude_m;

  // PID on altitude error (D-term uses measurement = altitude, so velocity
  // derivative is handled by OnMeasurement mode)
  float alt_correction = altitude_pid_.update(sp.altitude_des, altitude_m, dt);

  throttle_ = Config::HOVER_THROTTLE + alt_correction;
  throttle_ = clamp(throttle_, 0.0f, 1.0f);
}

void DroneFlightSubsystem::controlMixer() {
  // X-quad motor mixing
  //   FL (CCW)    FR (CW)
  //        \      /
  //         [DRONE]
  //        /      \
  //   BL (CW)    BR (CCW)
  motors_[0] =
      throttle_ + roll_correction_ - pitch_correction_ + yaw_correction_;  // FL
  motors_[1] =
      throttle_ - roll_correction_ - pitch_correction_ - yaw_correction_;  // FR
  motors_[2] =
      throttle_ - roll_correction_ + pitch_correction_ + yaw_correction_;  // BR
  motors_[3] =
      throttle_ + roll_correction_ + pitch_correction_ - yaw_correction_;  // BL

  for (int i = 0; i < 4; i++) {
    motors_[i] = clamp(motors_[i], 0.0f, 1.0f);
  }
}

void DroneFlightSubsystem::writeMotors() {
  auto duty = [](float val) -> uint32_t {
    return (uint32_t)(clamp(val, 0.0f, 1.0f) * Config::MOTOR_PWM_MAX);
  };

  ledcWrite(0, duty(motors_[0]));  // FL
  ledcWrite(1, duty(motors_[1]));  // FR
  ledcWrite(2, duty(motors_[2]));  // BR
  ledcWrite(3, duty(motors_[3]));  // BL
}

void DroneFlightSubsystem::setMotorsOverride(const float speeds[4]) {
  if (!armed_) return;
  for (int i = 0; i < 4; i++) {
    motors_[i] = clamp(speeds[i], 0.0f, 1.0f);
  }
  override_active_ = true;
  writeMotors();
}

}  // namespace Drone
