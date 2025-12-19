#pragma once
/**
 * @file button_press.h
 * @author Rafeed Khan
 * @brief Non-blocking task that presses Antenna #1's button three times
 *
 *
 * This task:
 *   - Outputs 0 drive PWM (robot should already be aligned and stopped)
 *   - Commands a "pusher" actuator via a simple callback interface (arm code
 * can come later)
 */

#include <cstdint>

#include "drive_primitives.h"

namespace secbot {

/**
 * @brief Hardware hooks for the button press mechanism
 *
 * keeping this intentionally minimal so I can implement the task NOW
 * and implement the arm/actuator subsystem LATER
 */
struct ButtonPressIO {
  /**
   * @brief Command the pusher position (normalized).
   * @param pos_norm 0.0 = fully retracted, 1.0 = fully pressing
   *
   * ALDEM YOU WILL IMPLEMENT THIS LATER IN THE ARM SUBSYSTEM (servo, linear
   * actuator, etc)
   */
  void (*set_pusher_norm)(float pos_norm) = nullptr;

  /**
   * @brief contact sensor / limit switch (if we even have one man)
   * @return true if the pusher is contacting the button (or button is
   * depressed)
   *
   * If we dont have a sensor, the task still works using only timing because im
   * SO GOOD AT PROGRAMMMINGGGGGGG
   */
  bool (*read_contact)() = nullptr;
};

/**
 * @brief Tuning parameters for the button press sequence
 */
struct ButtonPressConfig {
  /// Total presses required by the ruleset for Antenna #1
  uint8_t presses_total = 3;

  /// Small delay before starting presses (lets the robot settle after docking)
  float settle_s = 0.20f;

  /// How long we hold the pusher in the "pressing" position each press
  float press_hold_s = 0.25f;

  /// How long we hold the pusher in the "released" position between presses
  float release_hold_s = 0.25f;

  /// Normalized pusher position for pressing the button
  float press_pos_norm = 1.0f;

  /// Normalized pusher position for releasing the button
  float release_pos_norm = 0.0f;

  /// Safety timeout for the entire task
  float timeout_s = 6.0f;

  /// If true AND read_contact exists, we'll watch it (still timed, but can be
  /// used for debug)
  bool use_contact_sensor = false;

  /// Contact debounce time (only used if use_contact_sensor && read_contact)
  float contact_debounce_s = 0.05f;
};

/**
 * @brief Press Antenna #1's button 3 times (non-blocking).
 *
 * Usage:
 *   - Call start()
 *   - Call update(dt) in your main loop until cmd.status.finished is true
 *   - This task never drives; it always returns left/right PWM = 0
 */
class ButtonPressTask {
 public:
  ButtonPressTask() = default;
  explicit ButtonPressTask(const ButtonPressConfig& cfg,
                           const ButtonPressIO& io = {})
      : cfg_(cfg), io_(io) {}

  /// Update hardware hooks (can be wired later when arm code exists LOL!!!!)
  void setIO(const ButtonPressIO& io) { io_ = io; }

  /// Update config/tuning.
  void setConfig(const ButtonPressConfig& cfg) { cfg_ = cfg; }

  /// Start the task (arms it)
  /// Robot should already be aligned and stopped
  void start();

  /// Cancel immediately (finished=true, success=false)
  void cancel();

  /// Reset internal state so the task can be started again
  void reset();

  /// Non-blocking tick, returns 0 PWM + status/progress
  TankPwmCmd update(float dt);

  bool isActive() const { return active_; }
  bool isFinished() const { return finished_; }
  bool isSuccess() const { return success_; }
  float progress() const { return progress_; }

 private:
  enum class State : uint8_t {
    kIdle = 0,
    kSettle,
    kPressHold,
    kReleaseHold,
    kDone,
    kFailed
  };

  void enterState(State s);

  // config + io
  ButtonPressConfig cfg_{};
  ButtonPressIO io_{};

  // state machine
  State state_ = State::kIdle;
  bool active_ = false;
  bool finished_ = true;  // idle == "nothing to do"
  bool success_ = true;

  uint8_t presses_done_ = 0;

  float t_state_ = 0.0f;  // time in current state
  float t_total_ = 0.0f;  // time since start()

  float contact_stable_t_ = 0.0f;
  bool last_contact_ = false;

  float progress_ = 0.0f;
};

}  // namespace secbot
