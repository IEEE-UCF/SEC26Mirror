/**
 * @file button_press.cpp
 * @author Rafeed Khan
 * @brief Implementation for ButtonPressTask (Antenna #1 button task)
 */

#include "button_press.h"

#include <cmath>

namespace secbot {

static float clamp01(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

void ButtonPressTask::enterState(State s) {
  state_ = s;
  t_state_ = 0.0f;

  // reset contact debounce tracking when entering states that care about it
  contact_stable_t_ = 0.0f;
  last_contact_ = false;
}

void ButtonPressTask::start() {
  // If you start without actuator wiring, fail FAST (so you don't "think it
  // worked")
  if (io_.set_pusher_norm == nullptr) {
    active_ = false;
    finished_ = true;
    success_ = false;
    progress_ = 0.0f;
    state_ = State::kFailed;
    return;
  }

  active_ = true;
  finished_ = false;
  success_ = false;  // becomes true only when we complete all presses
  presses_done_ = 0;

  t_total_ = 0.0f;
  progress_ = 0.0f;

  // ALWAYS start with pusher released
  io_.set_pusher_norm(cfg_.release_pos_norm);

  enterState(State::kSettle);
}

void ButtonPressTask::cancel() {
  // Stop commanding anything aggressive, Retract + fail
  if (io_.set_pusher_norm) {
    io_.set_pusher_norm(cfg_.release_pos_norm);
  }

  active_ = false;
  finished_ = true;
  success_ = false;
  progress_ = clamp01(progress_);
  state_ = State::kFailed;
}

void ButtonPressTask::reset() {
  // Return to a clean idle state
  if (io_.set_pusher_norm) {
    io_.set_pusher_norm(cfg_.release_pos_norm);
  }

  active_ = false;
  finished_ = true;
  success_ = true;
  presses_done_ = 0;

  t_state_ = 0.0f;
  t_total_ = 0.0f;

  contact_stable_t_ = 0.0f;
  last_contact_ = false;

  progress_ = 0.0f;
  state_ = State::kIdle;
}

TankPwmCmd ButtonPressTask::update(float dt) {
  TankPwmCmd out{};
  out.left_pwm = 0;
  out.right_pwm = 0;
  out.finished = finished_;
  out.success = success_;

  // If not active, behave like "no task loaded"
  if (!active_) {
    out.finished = true;
    out.success = success_;
    return out;
  }

  // Defensive dt
  if (dt < 0.0f) dt = 0.0f;

  t_state_ += dt;
  t_total_ += dt;

  // Global timeout: if something jams, we don't sit here forever
  if (cfg_.timeout_s > 0.0f && t_total_ >= cfg_.timeout_s) {
    cancel();
    out.finished = finished_;
    out.success = success_;
    return out;
  }

  // sensor is optional, timing always drives the press sequence (no blocking!!)
  bool contact = false;
  if (cfg_.use_contact_sensor && io_.read_contact) {
    contact = io_.read_contact();
    if (contact == last_contact_) {
      contact_stable_t_ += dt;
    } else {
      contact_stable_t_ = 0.0f;
      last_contact_ = contact;
    }
  }

  // Progress: presses_done_ is the main truth
  {
    const float total = (cfg_.presses_total == 0)
                            ? 1.0f
                            : static_cast<float>(cfg_.presses_total);
    float phase_frac = 0.0f;

    if (state_ == State::kSettle && cfg_.settle_s > 0.0f) {
      phase_frac = clamp01(t_state_ / cfg_.settle_s) * (1.0f / total) * 0.10f;
    } else if (state_ == State::kPressHold && cfg_.press_hold_s > 0.0f) {
      phase_frac =
          clamp01(t_state_ / cfg_.press_hold_s) * (1.0f / total) * 0.50f;
    } else if (state_ == State::kReleaseHold && cfg_.release_hold_s > 0.0f) {
      phase_frac =
          clamp01(t_state_ / cfg_.release_hold_s) * (1.0f / total) * 0.40f;
    }

    progress_ =
        clamp01((static_cast<float>(presses_done_) / total) + phase_frac);
  }

  // --------------------------
  // STATE MACHINE TIME!!!!
  // --------------------------
  switch (state_) {
    case State::kSettle: {
      // keep released while settling
      io_.set_pusher_norm(cfg_.release_pos_norm);

      if (t_state_ >= cfg_.settle_s) {
        // begin press #1
        io_.set_pusher_norm(cfg_.press_pos_norm);
        enterState(State::kPressHold);
      }
    } break;

    case State::kPressHold: {
      // PRESS position (this is the "button depressed" part!!!)
      io_.set_pusher_norm(cfg_.press_pos_norm);

      // optional contact check (still time-capped)
      const bool contact_ok =
          (!cfg_.use_contact_sensor || !io_.read_contact)
              ? false
              : (contact && contact_stable_t_ >= cfg_.contact_debounce_s);

      if (t_state_ >= cfg_.press_hold_s || contact_ok) {
        // release between presses
        io_.set_pusher_norm(cfg_.release_pos_norm);
        enterState(State::kReleaseHold);
      }
    } break;

    case State::kReleaseHold: {
      // RELEASE position
      io_.set_pusher_norm(cfg_.release_pos_norm);

      if (t_state_ >= cfg_.release_hold_s) {
        presses_done_++;

        // done after 3 presses (per rules yknow?!!?!!)
        if (presses_done_ >= cfg_.presses_total) {
          active_ = false;
          finished_ = true;
          success_ = true;
          progress_ = 1.0f;
          enterState(State::kDone);
        } else {
          // next press
          io_.set_pusher_norm(cfg_.press_pos_norm);
          enterState(State::kPressHold);
        }
      }
    } break;

    case State::kDone: {
      // keep released after finishing (don't keep pressing like an idiot)
      io_.set_pusher_norm(cfg_.release_pos_norm);
    } break;

    case State::kFailed: {
      io_.set_pusher_norm(cfg_.release_pos_norm);
    } break;

    case State::kIdle:
    default:
      // Shouldn't happen while active, but fail safe anyways HAAHAHAHA
      cancel();
      break;
  }

  out.finished = finished_;
  out.success = success_;
  return out;
}

}  // namespace secbot
