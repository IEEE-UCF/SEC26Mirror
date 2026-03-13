#include "DroneFlightSubsystem.h"

#include <cstring>
#include <math_utils.h>

using secbot::utils::clamp;

namespace Drone {

bool DroneFlightSubsystem::init() {
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
  alt_vel_pid_.configure(Config::altitudeVelocityPID());

  // Altitude velocity LPF (alpha=0.6 — heavier filtering for noisy VL53L0X)
  alt_vel_filter_.setAlpha(0.6f);

  disarm();
  return true;
}

void DroneFlightSubsystem::arm() {
  resetPIDs();
  override_active_ = false;
  // Clear stale setpoint so motors don't immediately spin
  xSemaphoreTake(sp_mutex_, portMAX_DELAY);
  sp_ = FlightSetpoint{};
  xSemaphoreGive(sp_mutex_);
  // Reset altitude state
  alt_initialized_ = false;
  alt_vel_ = 0.0f;
  alt_since_change_ = 0.0f;
  throttle_ = 0.0f;
  roll_correction_ = 0.0f;
  pitch_correction_ = 0.0f;
  yaw_correction_ = 0.0f;
  armed_ = true;
}

void DroneFlightSubsystem::killMotors() {
  // Immediate motor stop — bypasses slew rate limiter for emergency/disarm.
  motors_[0] = motors_[1] = motors_[2] = motors_[3] = 0.0f;
  motors_prev_[0] = motors_prev_[1] = motors_prev_[2] = motors_prev_[3] = 0.0f;
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
}

void DroneFlightSubsystem::disarm() {
  armed_ = false;
  override_active_ = false;
  killMotors();
  resetPIDs();
  // Clear setpoint to prevent stale state on next arm
  xSemaphoreTake(sp_mutex_, portMAX_DELAY);
  sp_ = FlightSetpoint{};
  xSemaphoreGive(sp_mutex_);
  throttle_ = 0.0f;
  roll_correction_ = 0.0f;
  pitch_correction_ = 0.0f;
  yaw_correction_ = 0.0f;
}

void DroneFlightSubsystem::resetPIDs() {
  roll_angle_pid_.reset();
  pitch_angle_pid_.reset();
  roll_rate_pid_.reset();
  pitch_rate_pid_.reset();
  yaw_rate_pid_.reset();
  altitude_pid_.reset();
  alt_vel_pid_.reset();
  alt_vel_filter_.reset(0.0f, false);
  alt_prev_ = 0.0f;
  alt_vel_ = 0.0f;
  alt_initialized_ = false;
  alt_since_change_ = 0.0f;
  roll_correction_ = pitch_correction_ = yaw_correction_ = 0.0f;
  throttle_ = 0.0f;
}

void DroneFlightSubsystem::update() {
  // Skip all work if IMU not initialized — no point running PID
  if (!gyro_.isInitialized()) {
    vTaskDelay(pdMS_TO_TICKS(100));
    return;
  }

  // Compute dt
  uint32_t now_us = micros();
  float dt = (last_us_ > 0) ? (now_us - last_us_) * 1e-6f : 0.0f;
  last_us_ = now_us;

  // Read cached IMU data (thread-safe)
  IMUData imu = gyro_.getData();

  // EKF predict (world-frame accel via full body→world rotation)
  float ax_w, ay_w;
  gyro_.getAccelWorld(ax_w, ay_w);
  ekf_.predict(ax_w, ay_w, dt);

  // Read altitude
#if DRONE_ENABLE_HEIGHT
  float alt = height_.getAltitudeM();
#else
  float alt = 0.0f;
#endif

  // Run PID + motor mixer
  compute(imu, alt, dt);

  // Rate measurement
  rate_count_++;
  uint32_t now_ms = millis();
  if (now_ms - rate_start_ms_ >= 1000) {
    measured_hz_ = rate_count_ * 1000.0f / (now_ms - rate_start_ms_);
    rate_count_ = 0;
    rate_start_ms_ = now_ms;
  }
}

void DroneFlightSubsystem::compute(const IMUData& imu, float altitude_m,
                                   float dt) {
  last_dt_ = dt;
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
  sp.roll_des = clamp(sp.roll_des, -max_roll_deg, max_roll_deg);
  sp.pitch_des = clamp(sp.pitch_des, -max_pitch_deg, max_pitch_deg);

  // 2. Cascaded angle -> rate PID (using clamped sp)
  controlAngle(imu, sp, dt);

  // 3. Motor mixing
  controlMixer();

  // 4. Write to ESCs
  writeMotors();
}

void DroneFlightSubsystem::controlAngle(const IMUData& imu,
                                        const FlightSetpoint& sp, float dt) {
  // Outer loop: angle -> desired rate
  float desired_roll_rate =
      roll_angle_pid_.update(sp.roll_des, imu.roll, dt);
  float desired_pitch_rate =
      pitch_angle_pid_.update(sp.pitch_des, imu.pitch, dt);

  // Inner loop: rate -> motor correction
  roll_correction_ =
      roll_rate_pid_.update(desired_roll_rate, imu.gyro_x, dt);
  pitch_correction_ =
      pitch_rate_pid_.update(desired_pitch_rate, imu.gyro_y, dt);

  // Yaw rate PID (single loop)
  yaw_correction_ =
      yaw_rate_pid_.update(sp.yaw_rate_des, imu.gyro_z, dt);

  // Reset rate integrators when throttle is very low (on the ground).
  // Only reset rate PIDs — angle PIDs hold CG trim across brief ground contact.
  if (throttle_ < 0.05f) {
    roll_rate_pid_.reset();
    pitch_rate_pid_.reset();
    yaw_rate_pid_.reset();
  }
}

void DroneFlightSubsystem::controlAltitude(const FlightSetpoint& sp,
                                           float altitude_m, float dt) {
  // Estimate vertical velocity only when the sensor value actually changes.
  // VL53L0X updates at 50Hz but this runs at 250Hz — computing velocity
  // on stale data produces zero/spike artifacts that destroy the D-term.
  if (!alt_initialized_) {
    alt_prev_ = altitude_m;
    alt_initialized_ = true;
    return;
  }

  // Detect actual sensor update (value changed since last tick).
  // Track real elapsed time since last sensor change for accurate velocity.
  alt_since_change_ += dt;
  if (altitude_m != alt_prev_) {
    if (alt_since_change_ > 0.001f) {
      float raw_vel = (altitude_m - alt_prev_) / alt_since_change_;
      alt_vel_ = alt_vel_filter_.update(raw_vel);
    }
    alt_prev_ = altitude_m;
    alt_since_change_ = 0.0f;
  }

  // Cascaded altitude control:
  //   Outer: position error -> desired velocity
  //   Inner: velocity error -> throttle correction
  // This avoids the noisy D-term problem entirely.
  float desired_vel = altitude_pid_.update(sp.altitude_des, altitude_m, dt);
  float vel_correction = alt_vel_pid_.update(desired_vel, alt_vel_, dt);

  throttle_ = hover_throttle + vel_correction;
  throttle_ = clamp(throttle_, 0.0f, 1.0f);
}

void DroneFlightSubsystem::controlMixer() {
  // X-quad motor mixing
  //   FL (CCW, M3, D2)    FR (CW, M1, D0)
  //             \          /
  //              [DRONE]
  //             /          \
  //   BL (CW, M4, D3)    BR (CCW, M2, D1)
  motors_[0] =
      throttle_ + roll_correction_ + pitch_correction_ + yaw_correction_;  // FL
  motors_[1] =
      throttle_ - roll_correction_ + pitch_correction_ - yaw_correction_;  // FR
  motors_[2] =
      throttle_ - roll_correction_ - pitch_correction_ + yaw_correction_;  // BR
  motors_[3] =
      throttle_ + roll_correction_ - pitch_correction_ - yaw_correction_;  // BL

  for (int i = 0; i < 4; i++) {
    motors_[i] = clamp(motors_[i], 0.0f, 1.0f);
  }
}

void DroneFlightSubsystem::writeMotors() {
  // Slew rate limiter: cap motor change to MOTOR_MAX_SLEW per second,
  // scaled by actual dt (no assumption about loop rate).
  // IMPORTANT: Do NOT modify motors_[] — it holds the target value
  // (from PID mixer or override).  Only motors_prev_[] tracks actual output.
  float max_delta = motor_max_slew * last_dt_;
  if (max_delta <= 0.0f) max_delta = 1.0f;  // no limit if dt unknown
  for (int i = 0; i < 4; i++) {
    float target = clamp(motors_[i], 0.0f, 1.0f);
    float delta = target - motors_prev_[i];
    if (delta > max_delta) delta = max_delta;
    if (delta < -max_delta) delta = -max_delta;
    float output = clamp(motors_prev_[i] + delta, 0.0f, 1.0f);
    // Dead zone elimination: ESCs don't spin below this threshold
    if (output > 0.0f && output < motor_min_output) {
      output = motor_min_output;
    }
    motors_prev_[i] = output;
    ledcWrite(i, (uint32_t)(output * Config::MOTOR_PWM_MAX));
  }
}

void DroneFlightSubsystem::setMotorsOverride(const float speeds[4]) {
  if (!armed_) return;
  for (int i = 0; i < 4; i++) {
    motors_[i] = clamp(speeds[i], 0.0f, 1.0f);
  }
  override_active_ = true;
  writeMotors();
}

bool DroneFlightSubsystem::setParam(const char* name, float value) {
  if (!name) return false;

  // Roll angle PID
  if (strncmp(name, "roll_angle_", 11) == 0) {
    auto cfg = roll_angle_pid_.config();
    if (strcmp(name + 11, "kp") == 0) { cfg.gains.kp = value; roll_angle_pid_.configure(cfg); return true; }
    if (strcmp(name + 11, "ki") == 0) { cfg.gains.ki = value; roll_angle_pid_.configure(cfg); return true; }
    if (strcmp(name + 11, "kd") == 0) { cfg.gains.kd = value; roll_angle_pid_.configure(cfg); return true; }
  }
  // Pitch angle PID
  if (strncmp(name, "pitch_angle_", 12) == 0) {
    auto cfg = pitch_angle_pid_.config();
    if (strcmp(name + 12, "kp") == 0) { cfg.gains.kp = value; pitch_angle_pid_.configure(cfg); return true; }
    if (strcmp(name + 12, "ki") == 0) { cfg.gains.ki = value; pitch_angle_pid_.configure(cfg); return true; }
    if (strcmp(name + 12, "kd") == 0) { cfg.gains.kd = value; pitch_angle_pid_.configure(cfg); return true; }
  }
  // Roll rate PID
  if (strncmp(name, "roll_rate_", 10) == 0) {
    auto cfg = roll_rate_pid_.config();
    if (strcmp(name + 10, "kp") == 0) { cfg.gains.kp = value; roll_rate_pid_.configure(cfg); return true; }
    if (strcmp(name + 10, "ki") == 0) { cfg.gains.ki = value; roll_rate_pid_.configure(cfg); return true; }
    if (strcmp(name + 10, "kd") == 0) { cfg.gains.kd = value; roll_rate_pid_.configure(cfg); return true; }
  }
  // Pitch rate PID
  if (strncmp(name, "pitch_rate_", 11) == 0) {
    auto cfg = pitch_rate_pid_.config();
    if (strcmp(name + 11, "kp") == 0) { cfg.gains.kp = value; pitch_rate_pid_.configure(cfg); return true; }
    if (strcmp(name + 11, "ki") == 0) { cfg.gains.ki = value; pitch_rate_pid_.configure(cfg); return true; }
    if (strcmp(name + 11, "kd") == 0) { cfg.gains.kd = value; pitch_rate_pid_.configure(cfg); return true; }
  }
  // Yaw rate PID
  if (strncmp(name, "yaw_rate_", 9) == 0) {
    auto cfg = yaw_rate_pid_.config();
    if (strcmp(name + 9, "kp") == 0) { cfg.gains.kp = value; yaw_rate_pid_.configure(cfg); return true; }
    if (strcmp(name + 9, "ki") == 0) { cfg.gains.ki = value; yaw_rate_pid_.configure(cfg); return true; }
    if (strcmp(name + 9, "kd") == 0) { cfg.gains.kd = value; yaw_rate_pid_.configure(cfg); return true; }
  }
  // Altitude PID
  if (strncmp(name, "altitude_", 9) == 0) {
    auto cfg = altitude_pid_.config();
    if (strcmp(name + 9, "kp") == 0) { cfg.gains.kp = value; altitude_pid_.configure(cfg); return true; }
    if (strcmp(name + 9, "ki") == 0) { cfg.gains.ki = value; altitude_pid_.configure(cfg); return true; }
    if (strcmp(name + 9, "kd") == 0) { cfg.gains.kd = value; altitude_pid_.configure(cfg); return true; }
  }
  // Altitude velocity PID
  if (strncmp(name, "alt_vel_", 8) == 0) {
    auto cfg = alt_vel_pid_.config();
    if (strcmp(name + 8, "kp") == 0) { cfg.gains.kp = value; alt_vel_pid_.configure(cfg); return true; }
    if (strcmp(name + 8, "ki") == 0) { cfg.gains.ki = value; alt_vel_pid_.configure(cfg); return true; }
    if (strcmp(name + 8, "kd") == 0) { cfg.gains.kd = value; alt_vel_pid_.configure(cfg); return true; }
  }

  // Scalar parameters
  if (strcmp(name, "hover_throttle") == 0) { hover_throttle = value; return true; }
  if (strcmp(name, "motor_max_slew") == 0) { motor_max_slew = value; return true; }
  if (strcmp(name, "max_roll_deg") == 0) { max_roll_deg = value; return true; }
  if (strcmp(name, "max_pitch_deg") == 0) { max_pitch_deg = value; return true; }
  if (strcmp(name, "max_yaw_rate_dps") == 0) { max_yaw_rate_dps = value; return true; }
  if (strcmp(name, "motor_min_output") == 0) { motor_min_output = value; return true; }
  if (strcmp(name, "ready_throttle") == 0) { ready_throttle = value; return true; }

  return false;
}

bool DroneFlightSubsystem::getParam(const char* name, float& out) const {
  if (!name) return false;

  // Roll angle PID
  if (strcmp(name, "roll_angle_kp") == 0) { out = roll_angle_pid_.config().gains.kp; return true; }
  if (strcmp(name, "roll_angle_ki") == 0) { out = roll_angle_pid_.config().gains.ki; return true; }
  if (strcmp(name, "roll_angle_kd") == 0) { out = roll_angle_pid_.config().gains.kd; return true; }
  // Pitch angle PID
  if (strcmp(name, "pitch_angle_kp") == 0) { out = pitch_angle_pid_.config().gains.kp; return true; }
  if (strcmp(name, "pitch_angle_ki") == 0) { out = pitch_angle_pid_.config().gains.ki; return true; }
  if (strcmp(name, "pitch_angle_kd") == 0) { out = pitch_angle_pid_.config().gains.kd; return true; }
  // Roll rate PID
  if (strcmp(name, "roll_rate_kp") == 0) { out = roll_rate_pid_.config().gains.kp; return true; }
  if (strcmp(name, "roll_rate_ki") == 0) { out = roll_rate_pid_.config().gains.ki; return true; }
  if (strcmp(name, "roll_rate_kd") == 0) { out = roll_rate_pid_.config().gains.kd; return true; }
  // Pitch rate PID
  if (strcmp(name, "pitch_rate_kp") == 0) { out = pitch_rate_pid_.config().gains.kp; return true; }
  if (strcmp(name, "pitch_rate_ki") == 0) { out = pitch_rate_pid_.config().gains.ki; return true; }
  if (strcmp(name, "pitch_rate_kd") == 0) { out = pitch_rate_pid_.config().gains.kd; return true; }
  // Yaw rate PID
  if (strcmp(name, "yaw_rate_kp") == 0) { out = yaw_rate_pid_.config().gains.kp; return true; }
  if (strcmp(name, "yaw_rate_ki") == 0) { out = yaw_rate_pid_.config().gains.ki; return true; }
  if (strcmp(name, "yaw_rate_kd") == 0) { out = yaw_rate_pid_.config().gains.kd; return true; }
  // Altitude PID
  if (strcmp(name, "altitude_kp") == 0) { out = altitude_pid_.config().gains.kp; return true; }
  if (strcmp(name, "altitude_ki") == 0) { out = altitude_pid_.config().gains.ki; return true; }
  if (strcmp(name, "altitude_kd") == 0) { out = altitude_pid_.config().gains.kd; return true; }
  // Altitude velocity PID
  if (strcmp(name, "alt_vel_kp") == 0) { out = alt_vel_pid_.config().gains.kp; return true; }
  if (strcmp(name, "alt_vel_ki") == 0) { out = alt_vel_pid_.config().gains.ki; return true; }
  if (strcmp(name, "alt_vel_kd") == 0) { out = alt_vel_pid_.config().gains.kd; return true; }
  // Scalars
  if (strcmp(name, "hover_throttle") == 0) { out = hover_throttle; return true; }
  if (strcmp(name, "motor_max_slew") == 0) { out = motor_max_slew; return true; }
  if (strcmp(name, "max_roll_deg") == 0) { out = max_roll_deg; return true; }
  if (strcmp(name, "max_pitch_deg") == 0) { out = max_pitch_deg; return true; }
  if (strcmp(name, "max_yaw_rate_dps") == 0) { out = max_yaw_rate_dps; return true; }
  if (strcmp(name, "motor_min_output") == 0) { out = motor_min_output; return true; }
  if (strcmp(name, "ready_throttle") == 0) { out = ready_throttle; return true; }

  return false;
}

size_t DroneFlightSubsystem::getAllParams(float* values, const char** names, size_t max) const {
  static const char* const PARAM_NAMES[] = {
    "roll_angle_kp", "roll_angle_ki", "roll_angle_kd",
    "pitch_angle_kp", "pitch_angle_ki", "pitch_angle_kd",
    "roll_rate_kp", "roll_rate_ki", "roll_rate_kd",
    "pitch_rate_kp", "pitch_rate_ki", "pitch_rate_kd",
    "yaw_rate_kp", "yaw_rate_ki", "yaw_rate_kd",
    "altitude_kp", "altitude_ki", "altitude_kd",
    "alt_vel_kp", "alt_vel_ki", "alt_vel_kd",
    "hover_throttle", "motor_max_slew", "max_roll_deg",
    "max_pitch_deg", "max_yaw_rate_dps", "motor_min_output",
    "ready_throttle",
  };
  constexpr size_t NUM_PARAMS = sizeof(PARAM_NAMES) / sizeof(PARAM_NAMES[0]);
  size_t count = (max < NUM_PARAMS) ? max : NUM_PARAMS;

  for (size_t i = 0; i < count; i++) {
    names[i] = PARAM_NAMES[i];
    getParam(PARAM_NAMES[i], values[i]);
  }
  return count;
}

}  // namespace Drone
