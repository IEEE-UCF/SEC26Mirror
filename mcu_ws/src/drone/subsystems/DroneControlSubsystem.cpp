#include "DroneControlSubsystem.h"

namespace Drone {

//  Lifecycle 

void DroneControlSubsystem::init() {
  // Setup ESP32 LEDC PWM channels for each motor (Arduino Core v2 API)
  ledcSetup(LEDC_FL, cfg_.motors.pwm_freq, cfg_.motors.pwm_resolution);
  ledcSetup(LEDC_FR, cfg_.motors.pwm_freq, cfg_.motors.pwm_resolution);
  ledcSetup(LEDC_RR, cfg_.motors.pwm_freq, cfg_.motors.pwm_resolution);
  ledcSetup(LEDC_RL, cfg_.motors.pwm_freq, cfg_.motors.pwm_resolution);

  ledcAttachPin(cfg_.motors.pin_fl, LEDC_FL);
  ledcAttachPin(cfg_.motors.pin_fr, LEDC_FR);
  ledcAttachPin(cfg_.motors.pin_rr, LEDC_RR);
  ledcAttachPin(cfg_.motors.pin_rl, LEDC_RL);

  // Motors off
  disarm();
}

void DroneControlSubsystem::arm() {
  resetPIDStates();
  armed_ = true;
}

void DroneControlSubsystem::disarm() {
  armed_ = false;
  motors_[0] = motors_[1] = motors_[2] = motors_[3] = 0.0f;
  writeMotors();
  resetPIDStates();
}

void DroneControlSubsystem::update(const IMUData& imu, float altitude_m,
                                    float dt) {
  if (!armed_ || dt <= 0.0f || dt > 0.1f) {
    motors_[0] = motors_[1] = motors_[2] = motors_[3] = 0.0f;
    writeMotors();
    return;
  }

  // 1. Altitude hold (modifies throttle_)
  throttle_ = sp_.throttle;
  if (sp_.altitude_hold) {
    controlAltitude(altitude_m, dt);
  }

  // 2. Angle PID (outer loop) -> outputs roll_pid_, pitch_pid_, yaw_pid_
  controlAngle(imu, dt);

  // 3. Motor mixing
  controlMixer();

  // 4. Write to ESCs
  writeMotors();
}

//  PID core
// use_measurement_derivative: if true, D-term uses raw sensor rate instead of differentiated error
// This is smoother and industry standard for attitude control

float DroneControlSubsystem::computePID(const PIDGains& g, PIDState& s,
                                         float error, float dt,
                                         bool use_measurement_derivative,
                                         float measurement_rate) {
  // Integral with anti-windup
  s.integral += error * dt;
  s.integral = constrain(s.integral, -g.i_limit, g.i_limit);

  // Derivative
  if (use_measurement_derivative) {
    s.derivative = measurement_rate;
  } else {
    s.derivative = (dt > 0.0f) ? (error - s.error_prev) / dt : 0.0f;
  }

  s.error_prev = error;

  // Note: minus sign on D when using measurement rate
  if (use_measurement_derivative) {
    return 0.01f * (g.kp * error + g.ki * s.integral - g.kd * s.derivative);
  }
  return 0.01f * (g.kp * error + g.ki * s.integral + g.kd * s.derivative);
}

void DroneControlSubsystem::resetPIDStates() {
  roll_angle_pid_ = {};
  pitch_angle_pid_ = {};
  yaw_rate_pid_ = {};
  altitude_pid_ = {};
  roll_pid_ = pitch_pid_ = yaw_pid_ = 0.0f;
  alt_velocity_ = 0.0f;
  alt_vel_filtered_ = 0.0f;
}

//  Angle stabilization
// Outer loop: desired angle -> PID -> correction
// D-term uses raw gyro rate (not differentiated error) for smoothness

void DroneControlSubsystem::controlAngle(const IMUData& imu, float dt) {
  // Roll
  float error_roll = sp_.roll_des - imu.roll;
  roll_pid_ = computePID(cfg_.roll_angle, roll_angle_pid_, error_roll, dt,
                          true, imu.gyro_x);

  // Pitch
  float error_pitch = sp_.pitch_des - imu.pitch;
  pitch_pid_ = computePID(cfg_.pitch_angle, pitch_angle_pid_, error_pitch, dt,
                           true, imu.gyro_y);

  // Yaw (rate-based  stabilize on gyro Z rate)
  float error_yaw = sp_.yaw_rate_des - imu.gyro_z;
  yaw_pid_ = computePID(cfg_.yaw_rate, yaw_rate_pid_, error_yaw, dt,
                          false, 0.0f);

  // Reset integrators when throttle is very low (on the ground)
  if (throttle_ < 0.05f) {
    roll_angle_pid_.integral = 0.0f;
    pitch_angle_pid_.integral = 0.0f;
    yaw_rate_pid_.integral = 0.0f;
  }
}

//  Altitude hold PID 
// Modifies throttle_ around hover_throttle baseline

void DroneControlSubsystem::controlAltitude(float altitude_m, float dt) {
  // Estimate vertical velocity with low-pass filter
  float raw_vel = (dt > 0.0f) ? (altitude_m - alt_prev_) / dt : 0.0f;
  alt_vel_filtered_ += ALT_LPF_ALPHA * (raw_vel - alt_vel_filtered_);
  alt_velocity_ = alt_vel_filtered_;
  alt_prev_ = altitude_m;

  // PID on altitude error
  float error = sp_.altitude_des - altitude_m;
  altitude_pid_.integral += error * dt;
  altitude_pid_.integral =
      constrain(altitude_pid_.integral, -cfg_.altitude.i_limit,
                cfg_.altitude.i_limit);

  // D-term: use negative filtered velocity (rising = less error correction)
  float alt_correction =
      cfg_.altitude.kp * error +
      cfg_.altitude.ki * altitude_pid_.integral +
      cfg_.altitude.kd * (-alt_velocity_);

  alt_correction = constrain(alt_correction, -0.3f, 0.3f);

  // Final throttle = hover baseline + PID correction
  throttle_ = cfg_.hover_throttle + alt_correction;
  throttle_ = constrain(throttle_, 0.0f, 1.0f);
}

//  X-quad motor mixing
//   Motor layout (top view, front = top):
//
//     FL (CCW)    FR (CW)
//          \      /
//           [DRONE]
//          /      \
//     RL (CW)    RR (CCW)
//
// Thanks internet for cool drone ascii thing :D

void DroneControlSubsystem::controlMixer() {
  motors_[0] = throttle_ + roll_pid_ - pitch_pid_ + yaw_pid_;  // FL
  motors_[1] = throttle_ - roll_pid_ - pitch_pid_ - yaw_pid_;  // FR
  motors_[2] = throttle_ - roll_pid_ + pitch_pid_ + yaw_pid_;  // RR
  motors_[3] = throttle_ + roll_pid_ + pitch_pid_ - yaw_pid_;  // RL

  for (int i = 0; i < 4; i++) {
    motors_[i] = constrain(motors_[i], 0.0f, 1.0f);
  }
}

//  Write motor values to ESP32 LEDC PWM 

void DroneControlSubsystem::writeMotors() {
  uint32_t max_duty = (1 << cfg_.motors.pwm_resolution) - 1;

  auto duty = [&](float val) -> uint32_t {
    return (uint32_t)(constrain(val, 0.0f, 1.0f) * max_duty);
  };

  ledcWrite(LEDC_FL, duty(motors_[0]));
  ledcWrite(LEDC_FR, duty(motors_[1]));
  ledcWrite(LEDC_RR, duty(motors_[2]));
  ledcWrite(LEDC_RL, duty(motors_[3]));
}

}  // namespace Drone
