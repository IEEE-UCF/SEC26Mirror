/**
 * @file flag_plant.cpp
 * @author Rafeed Khan
 * @brief Implementation for FlagPlantTask (final flag release/drop).
 */

#include "flag_plant.h"

namespace secbot {

float FlagPlantTask::clamp01_(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

FlagPlantTask::FlagPlantTask(const Config& cfg, const IO& io) : cfg_(cfg), io_(io) {
  // start clean
  reset();
}

void FlagPlantTask::enterState_(State s) {
  state_ = s;
  t_state_ = 0.0f;

  // reset sensor debounce when entering a new state
  present_stable_t_ = 0.0f;
  last_present_ = true;
}

void FlagPlantTask::start() {
  // If you start without actuator wiring, FAIL FAST SO WE DONT THINK IT WORRKEDDDD!!!!
  if (io_.set_latch_norm == nullptr) {
    active_ = false;
    finished_ = true;
    success_ = false;
    progress_ = 0.0f;
    status_ = PrimitiveStatus::kFailed;
    state_ = State::kFailed;
    return;
  }

  // begin
  active_ = true;
  finished_ = false;
  success_ = false;

  t_state_ = 0.0f;
  t_total_ = 0.0f;
  progress_ = 0.0f;

  // ALWAYS begin latched (safe)
  io_.set_latch_norm(cfg_.latch_closed_norm);

  status_ = PrimitiveStatus::kRunning;
  enterState_(State::kSettle);
}

void FlagPlantTask::cancel() {
  // goal: stop anything risky and keep the flag if possible
  if (io_.set_latch_norm) {
    io_.set_latch_norm(cfg_.latch_closed_norm);
  }

  active_ = false;
  finished_ = true;
  success_ = false;
  progress_ = clamp01_(progress_);

  status_ = PrimitiveStatus::kFailed;
  state_ = State::kFailed;
}

void FlagPlantTask::reset() {
  // goal: return to clean idle
  if (io_.set_latch_norm) {
    io_.set_latch_norm(cfg_.latch_closed_norm);
  }

  active_ = false;
  finished_ = true;
  success_ = false;

  t_state_ = 0.0f;
  t_total_ = 0.0f;

  last_present_ = true;
  present_stable_t_ = 0.0f;

  progress_ = 0.0f;

  status_ = PrimitiveStatus::kIdle;
  state_ = State::kIdle;
}

TankPwmCmd FlagPlantTask::update(float dt) {
  TankPwmCmd out{};

  // This task does NOT drive
  out.left_pwm = 0;
  out.right_pwm = 0;

  // Same output pattern as the other task state machines
  out.finished = finished_;
  out.success  = success_;

  // defensive dt
  if (dt < 0.0f) dt = 0.0f;

  // not active = behave like "no task loaded"
  if (!active_) {
    out.finished = true;
    out.success = success_;
    return out;
  }

  t_state_ += dt;
  t_total_ += dt;

  // global timeout
  if (cfg_.timeout_s > 0.0f && t_total_ >= cfg_.timeout_s) {
    cancel();
    out.finished = finished_;
    out.success  = success_;
    return out;
  }

  // optional, read sensor (true = still has flag)
  bool present = true;
  bool sensor_ok = (cfg_.use_flag_present_sensor && io_.read_flag_present != nullptr);
  if (sensor_ok) {
    present = io_.read_flag_present();

    // debounce stability
    if (present == last_present_) {
      present_stable_t_ += dt;
    } else {
      present_stable_t_ = 0.0f;
      last_present_ = present;
    }
  }

  // progress is simple, 0..1 across states
  if (state_ == State::kSettle && cfg_.settle_s > 0.0f) {
    progress_ = 0.10f * clamp01_(t_state_ / cfg_.settle_s);
  } else if (state_ == State::kUnlatchHold && cfg_.unlatch_s > 0.0f) {
    progress_ = 0.10f + 0.70f * clamp01_(t_state_ / cfg_.unlatch_s);
  } else if (state_ == State::kPostDrop && cfg_.post_drop_s > 0.0f) {
    progress_ = 0.80f + 0.20f * clamp01_(t_state_ / cfg_.post_drop_s);
  } else if (state_ == State::kDone) {
    progress_ = 1.0f;
  }

  // --------------------------
  // STATE. MACHINE. TIMEEEE!!!
  // --------------------------
  switch (state_) {
    case State::kSettle: {
      // goal: keep latched for a moment (no twitchy release)
      io_.set_latch_norm(cfg_.latch_closed_norm);

      if (t_state_ >= cfg_.settle_s) {
        io_.set_latch_norm(cfg_.latch_open_norm);
        enterState_(State::kUnlatchHold);
      }
    } break;

    case State::kUnlatchHold: {
      // goal: unlatch and HOLD it open long enough to actually drop
      io_.set_latch_norm(cfg_.latch_open_norm);

      // If we have a sensor and it says "flag is gone" stably, we can move on early
      if (sensor_ok && (present == false) && (present_stable_t_ >= cfg_.sensor_debounce_s)) {
        enterState_(State::kPostDrop);
        break;
      }

      if (t_state_ >= cfg_.unlatch_s) {
        enterState_(State::kPostDrop);
      }
    } break;

    case State::kPostDrop: {
      // goal: keep open briefly so we DON'T snag it again
      io_.set_latch_norm(cfg_.latch_open_norm);

      // If sensor exists, success is stronger if we confirm "not present"
      bool released_confirmed = true;
      if (sensor_ok) {
        released_confirmed = (present == false) && (present_stable_t_ >= cfg_.sensor_debounce_s);
      }

      if (t_state_ >= cfg_.post_drop_s) {
        active_ = false;
        finished_ = true;
        success_ = released_confirmed;   // Without a sensor: assume success after timing completes
                                        // With a sensor: require stable "flag not present" confirmation

        status_ = success_ ? PrimitiveStatus::kSucceeded : PrimitiveStatus::kFailed;
        enterState_(success_ ? State::kDone : State::kFailed);
      }
    } break;

    case State::kDone: {
      // goal: stay open (harmless) or we can change this to re-latch if you prefer aldem
      io_.set_latch_norm(cfg_.latch_open_norm);
    } break;

    case State::kFailed: {
      // goal: fail safe (close latch)
      io_.set_latch_norm(cfg_.latch_closed_norm);
    } break;

    case State::kIdle:
    default:
      cancel();
      break;
  }

  out.finished = finished_;
  out.success  = success_;
  return out;
}

}  // namespace secbot
