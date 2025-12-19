#pragma once

/**
 * @file pressure_clear.h
 * @author Rafeed Khan
 * @brief Declares the pressure plate “duck removal” task (Antenna #3)
 *
 * Competition intent (Antenna #3):
 * - A pressure plate is mounted on top of the antenna base with an Astro-Duck on top.
 * - The robot must remove the Astro-Duck from the pressure plate to complete the task.
 *
 * This is the main philosophy behind my implementation:
 * - SIMPLE timed state machine (non-blocking, no delay())
 * - Drivetrain output is always 0/0 (you should already be aligned when this thing is run lmfao)
 * - Manipulator outputs are provided via tiny function pointers so this runs clean on the Teensy
 */

#include <stdint.h>
#include "drive_primitives.h"  // TankPwmCmd

namespace secbot {

/**
 * @brief IO hooks for the pressure clear task
 *
 * Keeping this dumb and Teensy-safe: function pointers only
 */
struct PressureClearIO {
  /// Set the sweeper/arm position as a normalized value [0.0, 1.0]
  /// (Aldem, map this to servo microseconds or whatever the HAL uses)
  void (*set_sweeper_norm)(float norm) = nullptr;

  /// Optional fun thing: run an intake/roller motor to help capture/retain the duck
  /// Positive/negative meaning is your convention
  void (*set_intake_pwm)(int16_t pwm) = nullptr;

  /// Another optional: returns true if a “duck captured” sensor is active (breakbeam, limit switch, etc)
  bool (*duck_captured)() = nullptr;
};

/**
 * @brief Tunables for the pressure clear sequence
 *
 * Default values are conservative, PLEASE TUNE ON THE REAL ANTENNA
 */
struct PressureClearConfig {
  // Sweeper positions (normalized [0..1])
  float sweeper_safe_norm  = 0.0f;  ///< Retracted / travel-safe
  float sweeper_push_norm  = 1.0f;  ///< Extended / pushes duck off plate

  // Optional intake behavior
  int16_t intake_pwm = 0;           ///< 0 disables intake usage

  // Timing (seconds)
  uint8_t sweeps               = 2;     ///< how many push cycles to try before declaring success
  float settle_s               = 0.20f; ///< small pause after start
  float push_out_s             = 0.35f; ///< time to extend/push
  float push_hold_s            = 0.25f; ///< hold at push position to ensure duck clears sensor
  float retract_s              = 0.35f; ///< retract time
  float between_sweeps_s       = 0.20f; ///< pause between attempts
  float overall_timeout_s      = 4.00f; ///< hard safety timeout for whole task

  // If you have a duck_captured sensor: allow a short time window to trip it
  float capture_wait_s         = 0.20f;
};

/**
 * @brief Clears the Astro-Duck from Antenna #3 pressure plate
 *
 * Usage:
 *  - Construct with IO + (optional) config
 *  - call start()
 *  - call update(dt) periodically until isFinished()
 *
 * Drivetrain stays stopped, this only manipulates the sweeper/intake
 */
class PressureClearTask {
public:
  /**
   * @param io   Hardware hooks (required: set_sweeper_norm)
   * @param cfg  Tunables (safe defaults)
   */
  PressureClearTask(const PressureClearIO& io, const PressureClearConfig& cfg = PressureClearConfig{});

  /// Begin the sequence from the start
  void start();

  /// Abort immediately (outputs go safe)
  void cancel();

  /// True once task reaches Done or Failed
  bool isFinished() const { return finished_; }

  /// True only if we finished normally (not timeout/cancel)
  bool isSuccess() const { return success_; }

  /// [0..1] rough progress estimate (for UI/debug)
  float progress() const;

  /**
   * @brief Step the task forward by dt seconds
   * @return TankPwmCmd where left/right are always 0, with finished/success filled
   */
  TankPwmCmd update(float dt);

private:
  enum class State : uint8_t {
    kIdle,
    kSettle,
    kPushOut,
    kHold,
    kRetract,
    kBetween,
    kDone,
    kFailed
  };

  void enter_(State s);
  void applyOutputs_();

  PressureClearIO io_;
  PressureClearConfig cfg_;

  State state_ = State::kIdle;

  float state_time_s_ = 0.0f;
  float total_time_s_ = 0.0f;

  uint8_t sweeps_done_ = 0;

  bool finished_ = false;
  bool success_  = false;
  bool canceled_ = false;
};

} // namespace secbot
