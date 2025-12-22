#pragma once
/**
 * @file crank_turn.h
 * @author Rafeed Khan
 * @brief Antenna #2 crank task: rotate the crank at least 540 degrees using a
 * small task state machine.
 *
 * This task DOES NOT move the drivetrain
 * It returns TankPwmCmd with left/right = 0 and uses IO callbacks to drive the
 * crank actuator
 */

#include <cstdint>

#include "drive_primitives.h"  // TankPwmCmd, PrimitiveStatus

namespace secbot {

/**
 * @brief Spin the crank until we hit target rotation (prefer encoder),
 * otherwise open-loop timed
 *
 * State machine:
 *   SETTLE -> SPIN -> HOLD -> DONE
 */
class CrankTurnTask {
 public:
  struct Config {
    // goal: required crank rotation (degrees)
    float target_deg =
        540.0f;  ///< minimum degrees to rotate (rules say >= 540)

    // direction: +1 = CW, -1 = CCW (pick one and keep it consistent with the
    // arm aldem)
    int8_t direction = +1;

    // actuator command (raw PWM to your crank motor / end-effector motor
    // driver)
    int16_t spin_pwm =
        180;  ///< magnitude to spin with (signed applied by direction)

    // if we somehow never make the encoder, we spin for time = target_deg /
    // est_deg_per_s (or override)
    float est_deg_per_s = 180.0f;  ///< YOU CALIBRATE THIS ON REAL HARDWARE
    float open_loop_spin_s =
        -1.0f;  ///< if >0, use this exact time instead of computed

    // tolerances (only used when encoder exists SO PLEASE SOMOENE MAKE THE
    // ENCODER)
    float deg_tol = 8.0f;  ///< stop when turned >= target_deg - deg_tol

    // timing
    float settle_s = 0.25f;  ///< brief settle before spinning
    float hold_s = 0.20f;    ///< stop & hold after finishing
    float timeout_s = 8.0f;  ///< global safety timeout
  };

  struct IO {
    // Command the crank actuator motor PWM (signed)
    void (*set_crank_pwm)(int16_t pwm) = nullptr;

    // This is optional, read crank angle in degrees (0..360 or any wrap is
    // fine)
    float (*read_crank_deg)(void) = nullptr;

    // ALSO OPTIONAL, if we have a “task complete” signal (vision, LED sensor,
    // etc)
    bool (*read_task_complete)(void) = nullptr;
  };

  CrankTurnTask(const Config& cfg, const IO& io);

  /** @brief Begin the crank task */
  void start();

  /** @brief Cancel immediately (stop motor, finished=true, success=false). */
  void cancel();

  /** @brief Reset back to idle clean state */
  void reset();

  /**
   * @brief Tick once
   * @param dt seconds
   * @return TankPwmCmd with left/right = 0 (task does not drive!!)
   */
  TankPwmCmd update(float dt);

  PrimitiveStatus status() const { return status_; }

  // feedback
  float progress() const { return progress_; }  ///< 0..1
  float degrees_turned() const { return turned_deg_; }

 private:
  enum class State : uint8_t {
    kIdle = 0,
    kSettle,
    kSpin,
    kHold,
    kDone,
    kFailed
  };

  struct DegUnwrap {
    void reset(float deg);
    float update(float deg);  // returns unwrapped degrees
    float last = 0.0f;
    float acc = 0.0f;
    bool inited = false;
  };

  void enterState_(State s);

  static float clamp01_(float x);
  static float wrapDeltaDeg_(float d);

  Config cfg_{};
  IO io_{};

  PrimitiveStatus status_ = PrimitiveStatus::kIdle;
  State state_ = State::kIdle;

  float t_state_ = 0.0f;
  float t_total_ = 0.0f;

  // encoder tracking (AHHH SOMEONE PLEASE MAKE THE ENCODER PLEASE)
  DegUnwrap unwrap_{};
  float start_unwrapped_deg_ = 0.0f;
  float turned_deg_ = 0.0f;

  // open-loop timing target
  float spin_goal_s_ = 0.0f;

  // feedback
  float progress_ = 0.0f;
};

}  // namespace secbot
