#pragma once

/**
 * @file drive_primitives.h
 * @author Rafeed Khan
 * @brief Tank-drive motion primitives that output raw PWM (left/right) as tiny task state machines
 *
 * Pose units:
 *  - x,y in meters
 *  - theta in radians (CCW+)
 */

#include <cstdint>

namespace secbot {

/** @brief 2D pose used by the MCU controller */
struct Pose2D {
  float x;      ///< meters
  float y;      ///< meters
  float theta;  ///< radians, CCW+
};

/** @brief Raw tank command (what the Teensy actually drives). */
struct TankPwmCmd {
  int16_t left_pwm = 0;   ///< left motor PWM, signed (forward +)
  int16_t right_pwm = 0;  ///< right motor PWM, signed (forward +)
  bool finished = false;  ///< true when primitive is done (success or fail)
  bool success = false;   ///< true only when goal reached (not timeout/cancel)
};

/** @brief Status of a primitive's internal state machine */
enum class PrimitiveStatus : uint8_t {
  kIdle = 0,
  kRunning,
  kSucceeded,
  kFailed
};

/**
 * @brief Common PWM shaping (clamp + deadband)
 *
 * Inline so the compiler can kill overhead on Teensy
 */
struct PwmShape {
  int16_t max_pwm = 255;     ///< hard cap for safety
  int16_t min_pwm = 55;      ///< minimum to overcome static friction
  int16_t deadband = 10;     ///< treat tiny PWM as zero (motor/controller noise)

  int16_t clamp(int32_t v) const;
  int16_t applyDeadband(int16_t v) const;
  int16_t shape(int32_t v) const;
};

/**
 * @brief Continuous angle unwrapping helper
 *
 * Converts wrapped angles (-pi..pi) into a monotonic angle
 * Shared by TurnInPlace and DriveArc
 */
struct AngleUnwrap {
  float last = 0.0f;
  float acc = 0.0f;
  bool inited = false;

  void reset(float theta);
  float update(float theta);  ///< returns unwrapped angle
};

/**
 * @brief Drives a signed distance while holding the starting heading (tank drive)
 *
 * State machine:
 *  Idle -> Running -> Succeeded(pos_tol) OR Failed(timeout/cancel)
 */
class DriveStraight {
 public:
  struct Config {
    float distance_m = 0.0f;

    float pos_tol = 0.02f;
    float heading_tol = 0.12f;

    PwmShape pwm{};

    float k_pwm_per_m = 450.0f;
    float slow_zone_m = 0.12f;

    float k_heading_pwm = 180.0f;
    int16_t max_correction_pwm = 120;

    float timeout_s = 4.0f;
  };

  explicit DriveStraight(const Config& cfg);

  void start(const Pose2D& pose);
  void cancel();
  TankPwmCmd update(const Pose2D& pose, float dt);

  PrimitiveStatus status() const { return status_; }
  float progress() const { return progress_; }

 private:
  Config cfg_{};
  PrimitiveStatus status_ = PrimitiveStatus::kIdle;

  float t_elapsed_ = 0.0f;
  Pose2D start_{};
  float target_dist_ = 0.0f;
  float progress_ = 0.0f;
};

/**
 * @brief Turns in place by a relative angle (rad), tank drive
 */
class TurnInPlace {
 public:
  struct Config {
    float angle_rad = 0.0f;
    float ang_tol = 0.035f;

    PwmShape pwm{};

    float k_pwm_per_rad = 220.0f;
    float slow_zone_rad = 0.18f;

    float timeout_s = 3.0f;
  };

  explicit TurnInPlace(const Config& cfg);

  void start(const Pose2D& pose);
  void cancel();
  TankPwmCmd update(const Pose2D& pose, float dt);

  PrimitiveStatus status() const { return status_; }
  float progress() const { return progress_; }

 private:
  Config cfg_{};
  PrimitiveStatus status_ = PrimitiveStatus::kIdle;

  float t_elapsed_ = 0.0f;

  AngleUnwrap unwrap_{};
  float theta0_ = 0.0f;
  float theta_target_ = 0.0f;
  float progress_ = 0.0f;
};

/**
 * @brief Drives an arc defined by radius (m) and relative heading change (rad), tank drive
 */
class DriveArc {
 public:
  struct Config {
    float radius_m = 0.30f;
    float angle_rad = 0.0f;

    float track_width_m = 0.30f;

    float ang_tol = 0.045f;

    PwmShape pwm{};
    int16_t base_pwm = 120;

    float timeout_s = 5.0f;
  };

  explicit DriveArc(const Config& cfg);

  void start(const Pose2D& pose);
  void cancel();
  TankPwmCmd update(const Pose2D& pose, float dt);

  PrimitiveStatus status() const { return status_; }
  float progress() const { return progress_; }

 private:
  AngleUnwrap unwrap_{};

  Config cfg_{};
  PrimitiveStatus status_ = PrimitiveStatus::kIdle;

  float t_elapsed_ = 0.0f;
  float theta0_ = 0.0f;
  float theta_target_ = 0.0f;
  float progress_ = 0.0f;
};

}  // namespace secbot
