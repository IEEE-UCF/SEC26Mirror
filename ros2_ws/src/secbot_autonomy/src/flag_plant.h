#pragma once
/**
 * @file flag_plant.h
 * @author Rafeed Khan
 * @brief Plant the final flag (release/drop) using a tiny non-blocking task state machine
 *
 * IMPORTANT:
 *  - This task does NOT drive the robot, caller must already be positioned outside the starting area
 *  - This task only releases the flag mechanism (latch/servo/etc)
 *
 * State machine:
 *   SETTLE -> UNLATCH_HOLD -> POST_DROP -> DONE
 */

#include <cstdint>
#include "drive_primitives.h"  // uses: secbot::TankPwmCmd, PrimitiveStatus (and the common types)

namespace secbot {

/**
 * @brief Flag planting task (release/drop mechanism).
 *
 * Wiring-agnostic on purpose:
 *  - You provide function pointers to actually command the latch servo / solenoid / whatever
 *  - Optional "flag present" sensor can confirm the flag actually left the robot
 */
class FlagPlantTask {
 public:
  struct Config {
    // goal: servo/actuator normalized positions (0..1)
    float latch_closed_norm = 0.0f;   ///< holding the flag (latched)
    float latch_open_norm   = 1.0f;   ///< releasing the flag (unlatched)

    // timing (seconds)
    float settle_s    = 0.20f;  ///< brief settle before unlatching
    float unlatch_s   = 0.80f;  ///< hold "open" long enough for flag to fall cleanly
    float post_drop_s = 0.40f;  ///< keep open a bit more (avoid re-catching)

    // optional sensor
    bool  use_flag_present_sensor = false;
    float sensor_debounce_s = 0.06f;  ///< stable time for present/not-present

    // safety!!!
    float timeout_s = 3.0f;
  };

  struct IO {
    // Command latch position (0..1). If null -> start() fails fast
    void (*set_latch_norm)(float norm) = nullptr;

    // Optional, returns true if the flag is still on the robot (beam break / switch / etc)
    bool (*read_flag_present)() = nullptr;
  };

  FlagPlantTask(const Config& cfg, const IO& io);

  /** @brief Start the task (will fail-fast if latch function is missing) */
  void start();

  /** @brief Cancel immediately (re-latch and mark failed) */
  void cancel();

  /** @brief Reset to idle (re-latch, clear counters) */
  void reset();

  /**
   * @brief Tick once (NON-BLOCKING!!!!!)
   * @param dt seconds since last tick
   * @return TankPwmCmd with motor PWM = 0 (task does not drive), plus finished/success/progress if your struct has them
   */
  TankPwmCmd update(float dt);

  PrimitiveStatus status() const { return status_; }
  float progress() const { return progress_; }

 private:
  enum class State : uint8_t {
    kIdle = 0,
    kSettle,
    kUnlatchHold,
    kPostDrop,
    kDone,
    kFailed
  };

  void enterState_(State s);

  static float clamp01_(float x);

  Config cfg_{};
  IO io_{};

  PrimitiveStatus status_ = PrimitiveStatus::kIdle;
  State state_ = State::kIdle;

  bool active_ = false;
  bool finished_ = true;
  bool success_ = false;

  float t_state_ = 0.0f;
  float t_total_ = 0.0f;

  // optional sensor debounce
  bool last_present_ = true;
  float present_stable_t_ = 0.0f;

  float progress_ = 0.0f;
};

}  // namespace secbot
