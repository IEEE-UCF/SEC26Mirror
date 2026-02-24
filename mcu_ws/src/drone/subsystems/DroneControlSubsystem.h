#pragma once
// DroneControlSubsystem: PID flight controller for ESP32 quadcopter
// Made for BNO085 IMU (onboard fusion, no Madgwick needed) + VL53L1X altitude hold

#include <Arduino.h>

namespace Drone {

//  PID gains (tune on the real drone!) 
struct PIDGains {
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  float i_limit = 25.0f;  // integrator saturation
};

//  PID state for one axis 
struct PIDState {
  float error_prev = 0.0f;
  float integral = 0.0f;
  float derivative = 0.0f;
};

//  Motor pin config 
struct MotorConfig {
  uint8_t pin_fl = 12;  // front left (CCW)
  uint8_t pin_fr = 13;  // front right (CW)
  uint8_t pin_rr = 14;  // rear right (CCW)
  uint8_t pin_rl = 15;  // rear left (CW)
  uint32_t pwm_freq = 20000;   // Hz
  uint8_t pwm_resolution = 10; // bits (0-1023)
};

//  Full flight controller config 
struct FlightConfig {
  // Angle PID (outer loop)  output is desired rate
  PIDGains roll_angle  = {0.2f, 0.3f, 0.05f, 25.0f};
  PIDGains pitch_angle = {0.2f, 0.3f, 0.05f, 25.0f};

  // Rate PID (inner loop)  output is motor correction
  PIDGains roll_rate   = {0.15f, 0.2f, 0.0002f, 25.0f};
  PIDGains pitch_rate  = {0.15f, 0.2f, 0.0002f, 25.0f};
  PIDGains yaw_rate    = {0.3f, 0.05f, 0.00015f, 25.0f};

  // Altitude PID
  PIDGains altitude    = {0.8f, 0.15f, 0.4f, 0.3f};
  float hover_throttle = 0.45f;  // base throttle to hover (tune this!)

  // Angle limits (degrees)
  float max_roll  = 30.0f;
  float max_pitch = 30.0f;
  float max_yaw_rate = 160.0f;  // deg/s

  // Motors
  MotorConfig motors;
};

//  IMU data fed into the controller each tick 
struct IMUData {
  float roll  = 0.0f;  // degrees, from BNO085
  float pitch = 0.0f;  // degrees
  float yaw   = 0.0f;  // degrees
  float gyro_x = 0.0f; // deg/s, raw gyro
  float gyro_y = 0.0f;
  float gyro_z = 0.0f;
};

//  Setpoint from mission or ROS command 
struct FlightSetpoint {
  float roll_des   = 0.0f;  // desired roll angle (degrees)
  float pitch_des  = 0.0f;  // desired pitch angle (degrees)
  float yaw_rate_des = 0.0f; // desired yaw rate (deg/s)
  float throttle   = 0.0f;  // 0.0 to 1.0 (overridden if altitude hold active)
  float altitude_des = 0.0f; // desired altitude (meters, for altitude hold)
  bool altitude_hold = false;
};

class DroneControlSubsystem {
 public:
  explicit DroneControlSubsystem(const FlightConfig& cfg = FlightConfig{})
      : cfg_(cfg) {}

  //  Lifecycle 
  void init();
  void update(const IMUData& imu, float altitude_m, float dt);
  void disarm();
  void arm();
  bool isArmed() const { return armed_; }

  //  Set desired state 
  void setSetpoint(const FlightSetpoint& sp) { sp_ = sp; }

  //  Read motor outputs (0.0 to 1.0) 
  float getMotor(uint8_t idx) const { return (idx < 4) ? motors_[idx] : 0.0f; }

  //  Config access 
  FlightConfig& config() { return cfg_; }

 private:
  //  PID helpers 
  float computePID(const PIDGains& g, PIDState& s, float error, float dt,
                   bool use_measurement_derivative, float measurement_rate);
  void resetPIDStates();

  //  Control stages (dRehmFlight pattern) 
  void controlAngle(const IMUData& imu, float dt);
  void controlAltitude(float altitude_m, float dt);
  void controlMixer();
  void writeMotors();

  FlightConfig cfg_;
  FlightSetpoint sp_;
  bool armed_ = false;

  // PID outputs
  float roll_pid_  = 0.0f;
  float pitch_pid_ = 0.0f;
  float yaw_pid_   = 0.0f;
  float throttle_  = 0.0f;

  // PID states
  PIDState roll_angle_pid_;
  PIDState pitch_angle_pid_;
  PIDState yaw_rate_pid_;
  PIDState altitude_pid_;

  // Altitude velocity estimate
  float alt_prev_ = 0.0f;
  float alt_velocity_ = 0.0f;
  float alt_vel_filtered_ = 0.0f;
  static constexpr float ALT_LPF_ALPHA = 0.3f;

  // Motor outputs (0.0 to 1.0)
  // [0]=front_left, [1]=front_right, [2]=rear_right, [3]=rear_left
  float motors_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  // LEDC channel mapping (ESP32)
  static constexpr uint8_t LEDC_FL = 0;
  static constexpr uint8_t LEDC_FR = 1;
  static constexpr uint8_t LEDC_RR = 2;
  static constexpr uint8_t LEDC_RL = 3;
};

}  // namespace Drone
