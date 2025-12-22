#pragma once
/**
 * @file keypad_enter.h
 * @author Rafeed Khan
 * @brief Enter the Antenna #4 keypad code (73738#) using a simple timed press sequence
 *
 * SMALL task state machine:
 *   SETTLE -> MOVE_TO_KEY -> PRESS_HOLD -> RELEASE_HOLD -> DONE
 *
 * NOTE:
 *  - This does NOT do arm kinematics.
 *  - It just calls the IO hooks (so it compiles NOW, and works once we make the arm code)
 */

#include <cstdint>
#include "drive_primitives.h"  // TankPwmCmd, PrimitiveStatus

namespace secbot {

/**
 * @brief Keypad entry task for typing "73738#" (ruleset reset code).
 *
 * Intended usage:
 *  - NAV2 brings robot into position near Antenna #4
 *  - Arm controller aligns over keypad (your code)
 *  - MCU runs this task to execute the timed press/release sequence cleanly
 */
class KeypadEnter {
 public:
  struct Config {
    // goal: what to type (ruleset says 73738#)
    const char* code = "73738#";

    // timings (seconds)
    float settle_s       = 0.25f;  ///< initial "calm down" time before first move
    float move_timeout_s = 0.80f;  ///< if no "at_key" feedback, we just wait this long
    float press_hold_s   = 0.18f;  ///< how long we hold the key down
    float release_hold_s = 0.18f;  ///< how long we wait released between keys

    // pusher positions (0..1 normalized)
    float press_pos_norm   = 1.0f; ///< pressed (down)
    float release_pos_norm = 0.0f; ///< released (up)

    // safety
    float timeout_s = 10.0f;       ///< whole-task timeout (don’t hang forever)
  };

  struct IO {
    /**
     * @brief Command the arm to hover over a specific key
     * goal: move end-effector over that key
     */
    void (*set_key_target)(char key) = nullptr;

    /**
     * @brief returns true when arm is over the key (if we have it)
     * If not available, leave null and we run purely timed open-loop moves
     */
    bool (*at_key_target)() = nullptr;

    /**
     * @brief Command the pusher (normalized 0..1)
     * goal: press_pos_norm = down, release_pos_norm = up
     */
    void (*set_pusher_norm)(float norm) = nullptr;
  };

  explicit KeypadEnter(const Config& cfg, const IO& io);

  /** @brief Start the keypad entry sequence from the beginning */
  void start();

  /** @brief Cancel immediately (finished=true, success=false) and release pusher */
  void cancel();

  /** @brief Reset to idle (releases pusher, clears progress/status) */
  void reset();

  /**
   * @brief Tick once.
   * @param dt seconds
   * @return TankPwmCmd (always 0 PWM, this task is manipulator-only)
   */
  TankPwmCmd update(float dt);

  PrimitiveStatus status() const { return status_; }

  // feedback
  float progress() const { return progress_; }     ///< 0..1
  uint8_t index() const { return idx_; }           ///< which character we're on
  char current_key() const { return cur_key_; }    ///< current key (for debug)

 private:
  enum class State : uint8_t {
    kIdle = 0,
    kSettle,
    kMoveToKey,
    kPressHold,
    kReleaseHold,
    kDone,
    kFailed
  };

  void enterState_(State s);

  // helpers
  static float clamp01_(float x);
  uint8_t codeLen_() const;

  Config cfg_{};
  IO io_{};

  PrimitiveStatus status_ = PrimitiveStatus::kIdle;
  State state_ = State::kIdle;

  float t_state_ = 0.0f;
  float t_total_ = 0.0f;

  // sequence tracking
  uint8_t idx_ = 0;
  char cur_key_ = '\0';

  // feedback
  float progress_ = 0.0f;
};

}  // namespace secbot
