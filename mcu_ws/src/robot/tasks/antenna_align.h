#pragma once
/**
 * @file antenna_align.h
 * @author Rafeed Khan
 * @brief Align (dock) to an antenna task face using tank-drive raw PWM (left/right).
 *
 * This is a SMALL task state machine:
 *   ROTATE_TO_GOAL -> DRIVE_TO_STANDOFF -> FINAL_HEADING -> DONE
 *
 * Coordinate convention (trying to match the typical ROS map frame):
 *   - +X = East, +Y = North
 *   - theta = 0 along +X, CCW positive
 *
 * If your frame differs, change headingForFace_() ONLY.
 */

#include <cstdint>
#include "drive_primitives.h"  // uses: secbot::Pose2D, TankPwmCmd, PwmShape, PrimitiveStatus

namespace secbot {

/** @brief Cardinal direction of the antenna task face (the side we must dock to). */
enum class AntennaFace : uint8_t {
  kNorth = 0,
  kEast,
  kSouth,
  kWest,
  kUnknown
};

/**
 * @brief Tank-drive antenna alignment (final approach / squaring up) using raw PWM.
 *
 * Intended usage:
 *  - High level (NAV2 / Pi) brings robot near antenna
 *  - MCU runs this to do the last around 20 cm cleanly and square up to the correct face
 */
class AntennaAlign {
 public:
  struct Config {
    // goal: stop point measured FROM ANTENNA CENTER along the face normal (meters)
    // (5" cube -> half-size around 0.0635m, so 0.18m gives around 11cm gap from face)
    float standoff_from_center_m = 0.18f;

    // tolerances
    float pos_tol_m = 0.03f;        ///< meters, when "at standoff"
    float yaw_tol_rad = 0.10f;      ///< rad, final squaring tolerance (around 5.7 deg)
    float yaw_gate_rad = 0.35f;     ///< rad, if bigger than this, rotate first (no driving!!)

    // range-based stopping since we have a front ToF
    bool use_front_range_stop = false;
    float front_stop_range_m = 0.12f;   ///< meters, stop driving if range <= this

    // PWM shaping
    PwmShape pwm{};

    // distance controller (P in PWM space)
    float k_pwm_per_m = 420.0f;     ///< base_pwm = k * distance_remaining
    float slow_zone_m = 0.20f;      ///< inside this zone, allow PWM below min_pwm

    // heading correction while driving (P in PWM space)
    float k_heading_pwm = 170.0f;   ///< corr_pwm = k * yaw_err_to_goal
    int16_t max_correction_pwm = 120;

    // turning in place (P in PWM space)
    float k_turn_pwm_per_rad = 230.0f;  ///< turn_pwm = k * yaw_err
    float slow_zone_rad = 0.25f;        ///< inside this, allow PWM below min_pwm

    // safety
    float timeout_s = 6.0f;
  };

  explicit AntennaAlign(const Config& cfg);

  /**
   * @brief Start docking to a known antenna center using the ruleset face direction
   * @param antenna_id 1..4
   * @param antenna_x_m antenna center X in map frame (meters)
   * @param antenna_y_m antenna center Y in map frame (meters)
   */
  void start(uint8_t antenna_id, float antenna_x_m, float antenna_y_m);

  /**
   * @brief Start docking to an explicit standoff goal pose (override everything).
   * @param goal_x_m standoff point X in map frame (meters)
   * @param goal_y_m standoff point Y in map frame (meters)
   * @param final_heading_rad robot final heading at goal (rad)
   */
  void startCustom(float goal_x_m, float goal_y_m, float final_heading_rad);

  /** @brief Cancel immediately (fails and outputs 0 PWM) */
  void cancel();

  /**
   * @brief Tick once
   * @param pose current pose (meters, radians)
   * @param dt seconds
   * @param front_range_m optional front range (meters), pass <0 if unavailable
   */
  TankPwmCmd update(const Pose2D& pose, float dt, float front_range_m = -1.0f);

  PrimitiveStatus status() const { return status_; }

  // feedback
  float progress() const { return progress_; }                 ///< 0..1
  float distance_remaining_m() const { return dist_rem_m_; }   ///< meters
  float heading_error_rad() const { return heading_err_rad_; } ///< radians

 private:
  enum class State : uint8_t {
    kIdle = 0,
    kRotateToGoal,
    kDriveToStandoff,
    kFinalHeading
  };

  // rules-based mapping
  static AntennaFace faceForId_(uint8_t antenna_id);
  static AntennaFace opposite_(AntennaFace f);
  static float headingForFace_(AntennaFace f);  // map-frame heading (rad)

  // math helpers
  static float wrapAngle_(float a);
  static float clampf_(float x, float lo, float hi);
  static int16_t abs16_(int16_t v);

  void computeGoalFromAntenna_(uint8_t antenna_id, float ax, float ay);

  Config cfg_{};
  PrimitiveStatus status_ = PrimitiveStatus::kIdle;
  State state_ = State::kIdle;

  float t_elapsed_ = 0.0f;

  // target
  float goal_x_ = 0.0f;
  float goal_y_ = 0.0f;
  float final_heading_ = 0.0f;   // rad

  // feedback
  float start_dist_ = 0.0f;
  float dist_rem_m_ = 0.0f;
  float heading_err_rad_ = 0.0f;
  float progress_ = 0.0f;
};

}  // namespace secbot
