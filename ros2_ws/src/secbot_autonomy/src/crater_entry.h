/**
 * @file crater_entry.h
 * @author Rafeed Khan
 * @brief Crater Entry task as a tiny task state machine.
 *
 * Im keeping this task relatively simple:
 *  - drive forward to rim (approach)
 *  - drive forward a tuned distance so the drive mechanism touches the crater line
 *  - pause briefly (contact/settle)
 *  - reverse out (exit)
 *
 * IMPORTANT:
 *  - The crater "touch line" is specified by the rules as around 3 inches down from the rim
 *    So tune descend_distance_m on the real field so the tread/wheel reliably crosses that line
 *
 * Inputs:
 *  - Pose2D (meters, radians) from the odom estimator
 * Output:
 *  - Raw tank PWM (left/right)
 */

#ifndef CRATER_ENTRY_H
#define CRATER_ENTRY_H

#include <cstdint>

#include "drive_primitives.h"

namespace secbot {

/** @brief High-level crater entry task status */
enum class CraterEntryStatus : uint8_t {
  kIdle = 0,
  kRunning,
  kSucceeded,
  kFailed
};

/**
 * @brief Crater entry task (approach -> descend -> dwell -> exit).
 *
 * Caller responsibilities (were keeping this dumb on MCU):
 *  - Start this when you're already lined up with the crater entrance
 *  - Provide pose updates at a steady rate
 */
class CraterEntryTask {
 public:
  /** @brief Task states (internal FSM) */
  enum class State : uint8_t {
    kIdle = 0,

    // goal: get to the rim cleanly
    kApproachRim,

    // goal: move far enough that the drive mechanism touches the crater line
    kDescendToLine,

    // goal: pause to ensure contact/settling
    kDwellOnLine,

    // goal: reverse back out to where we started (or clear the crater)
    kExitCrater,

    kDone,
    kFail
  };

  struct Config {
    // --- Distances (ALDEM, PLEASE TUNE THIS ON THE REAL FIELD) ---
    float approach_distance_m = 0.25f;  ///< forward distance to reach rim staging point
    float descend_distance_m  = 0.35f;  ///< forward distance into crater so wheel/tread touches crater line (3")
    float exit_distance_m     = 0.60f;  ///< reverse distance to clear crater

    // --- Timing ---
    float dwell_time_s  = 0.25f;  ///< stop briefly on the line (contact/settle)
    float timeout_s     = 8.0f;   ///< total task timeout

    // --- Primitive configs ---
    DriveStraight::Config approach_cfg{}; ///< used for approach segment
    DriveStraight::Config descend_cfg{};  ///< used for descend segment
    DriveStraight::Config exit_cfg{};     ///< used for exit segment (reverse)

    // If true, we treat "primitive finished but failed" as task failure immediately
    bool fail_fast = true;
  };

  explicit CraterEntryTask(const Config& cfg);

  /** @brief Start crater entry task (captures start pose) */
  void start(const Pose2D& pose);

  /** @brief Cancel task immediately (outputs 0 PWM on next update) */
  void cancel();

  /**
   * @brief Tick crater entry task once.
   * @param pose Current pose estimate
   * @param dt  Time step (seconds)
   * @return Raw PWM command
   */
  TankPwmCmd update(const Pose2D& pose, float dt);

  // --- Observability ---
  CraterEntryStatus status() const { return status_; }
  State state() const { return state_; }
  float progress() const { return progress_; }  ///< 0..1 coarse progress through phases

 private:
  void enterState(State s, const Pose2D& pose);

  Config cfg_{};

  CraterEntryStatus status_ = CraterEntryStatus::kIdle;
  State state_ = State::kIdle;

  float t_total_ = 0.0f;
  float t_state_ = 0.0f;

  Pose2D start_pose_{};

  // Sub-primitives (kept as members since theres no heap haw haw)
  DriveStraight approach_;
  DriveStraight descend_;
  DriveStraight exit_;

  float progress_ = 0.0f;
};

}  // namespace secbot

#endif  // CRATER_ENTRY_H
