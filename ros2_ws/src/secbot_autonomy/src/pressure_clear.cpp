/**
 * @file pressure_clear.cpp
 * @author Rafeed Khan
 * @brief Implements clearing the duck at Antenna #3 (pressure plate task).
 */

#include "pressure_clear.h"
#include <cmath>

namespace secbot {

static inline float clamp01_(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

PressureClearTask::PressureClearTask(const PressureClearIO& io, const PressureClearConfig& cfg)
: io_(io), cfg_(cfg) {
  // If caller forgot the one required hook, the task can never function
  if (io_.set_sweeper_norm == nullptr) {
    state_ = State::kFailed;
    finished_ = true;
    success_ = false;
  }
}

void PressureClearTask::start() {
  if (io_.set_sweeper_norm == nullptr) return;

  canceled_ = false;
  finished_ = false;
  success_  = false;

  total_time_s_ = 0.0f;
  sweeps_done_  = 0;

  enter_(State::kSettle);
}

void PressureClearTask::cancel() {
  canceled_ = true;
  finished_ = true;
  success_  = false;
  enter_(State::kFailed);
}

float PressureClearTask::progress() const {
  // Making this a simple “time-based” estimate so dashboards don’t look dead
  // Not gospel!! The real “done” is when we reach Done/Failed
  const uint8_t sweeps = (cfg_.sweeps == 0) ? 1 : cfg_.sweeps;

  const float per_sweep =
      cfg_.push_out_s + cfg_.push_hold_s + cfg_.retract_s;

  const float est_total =
      cfg_.settle_s +
      (sweeps * per_sweep) +
      ((sweeps > 1) ? ((sweeps - 1) * cfg_.between_sweeps_s) : 0.0f);

  if (est_total <= 0.001f) return finished_ ? 1.0f : 0.0f;
  return clamp01_(total_time_s_ / est_total);
}

TankPwmCmd PressureClearTask::update(float dt) {
  TankPwmCmd out{};
  out.left_pwm  = 0;
  out.right_pwm = 0;
  out.finished  = finished_;
  out.success   = success_;

  // If we already failed in constructor, just keep returning “done”
  if (finished_) {
    applyOutputs_(); // keep hardware safe
    return out;
  }

  // If not started, keep outputs safe and do nothing
  if (state_ == State::kIdle) {
    applyOutputs_();
    return out;
  }

  // Defensive dt handling (Teensy loop hiccups shouldn't explode timing)
  if (dt < 0.0f) dt = 0.0f;
  if (dt > 0.10f) dt = 0.10f;

  total_time_s_ += dt;
  state_time_s_ += dt;

  // Hard safety timeout
  if (total_time_s_ >= cfg_.overall_timeout_s) {
    finished_ = true;
    success_  = false;
    enter_(State::kFailed);
    out.finished = finished_;
    out.success  = success_;
    applyOutputs_();
    return out;
  }

  switch (state_) {
    case State::kSettle: {
      // GOAL: make sure the manipulator starts from a known safe pose
      if (state_time_s_ >= cfg_.settle_s) {
        enter_(State::kPushOut);
      }
    } break;

    case State::kPushOut: {
      // GOAL: push the duck off the pressure sensor (this is the “clear” action)
      if (state_time_s_ >= cfg_.push_out_s) {
        enter_(State::kHold);
      }
    } break;

    case State::kHold: {
      // GOAL: hold a moment so the duck fully clears the pressure plate
      // Optional thing: if we have a duck_captured sensor, let it “win early”.
      if (io_.duck_captured && io_.duck_captured()) {
        // We likely grabbed it. Don’t waste time.
        if (state_time_s_ >= cfg_.capture_wait_s) {
          enter_(State::kRetract);
        }
      } else {
        if (state_time_s_ >= cfg_.push_hold_s) {
          enter_(State::kRetract);
        }
      }
    } break;

    case State::kRetract: {
      // GOAL: retract so we don't snag the antenna while backing away
      if (state_time_s_ >= cfg_.retract_s) {
        sweeps_done_++;

        // Little simple success condition:
        // - We attempted the configured number of sweeps without timing out/cancel
        // (The verification for this is visual real life yknowww, antenna turns on + green “complete” LED)
        if (sweeps_done_ >= cfg_.sweeps) {
          finished_ = true;
          success_  = !canceled_;
          enter_(State::kDone);
        } else {
          enter_(State::kBetween);
        }
      }
    } break;

    case State::kBetween: {
      // GOAL: tiny pause between attempts (reduces “bounce”/re-contact)
      if (state_time_s_ >= cfg_.between_sweeps_s) {
        enter_(State::kPushOut);
      }
    } break;

    case State::kDone:
    case State::kFailed:
    case State::kIdle:
    default:
      break;
  }

  applyOutputs_();

  out.finished = finished_;
  out.success  = success_;
  return out;
}

void PressureClearTask::enter_(State s) {
  state_ = s;
  state_time_s_ = 0.0f;
}

void PressureClearTask::applyOutputs_() {
  // Always keep sweeper controlled (prevents “last command” drift)
  float sweeper = cfg_.sweeper_safe_norm;
  int16_t intake = 0;

  switch (state_) {
    case State::kIdle:
    case State::kSettle:
    case State::kBetween:
    case State::kRetract:
    case State::kDone:
    case State::kFailed:
      sweeper = cfg_.sweeper_safe_norm;
      intake  = 0;
      break;

    case State::kPushOut:
    case State::kHold:
      sweeper = cfg_.sweeper_push_norm;
      intake  = cfg_.intake_pwm; // 0 means “disabled”
      break;
  }

  // APPLY: sweeper
  if (io_.set_sweeper_norm) {
    io_.set_sweeper_norm(clamp01_(sweeper));
  }

  // APPLY: intake (optional once again)
  if (io_.set_intake_pwm) {
    io_.set_intake_pwm(intake);
  }
}

} // namespace secbot
