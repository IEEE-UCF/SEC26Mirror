/**
 * @file keypad_enter.cpp
 * @author Rafeed Khan
 * @brief Implementation for KeypadEnter (Antenna #4 keypad typing: 73738#)
 */

#include "keypad_enter.h"

namespace secbot {

static inline float safe_dt(float dt) { return (dt > 0.0f) ? dt : 0.0f; }

KeypadEnter::KeypadEnter(const Config& cfg, const IO& io) : cfg_(cfg), io_(io) {}

float KeypadEnter::clamp01_(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

uint8_t KeypadEnter::codeLen_() const {
  if (!cfg_.code) return 0;
  uint8_t n = 0;
  while (cfg_.code[n] != '\0' && n < 32) n++;  // cap so we don't go insane
  return n;
}

void KeypadEnter::enterState_(State s) {
  state_ = s;
  t_state_ = 0.0f;
}

void KeypadEnter::start() {
  // If arm hooks aren’t wired yet, fail FAST!!! (so you don't “think it worked”)
  if (io_.set_key_target == nullptr || io_.set_pusher_norm == nullptr || codeLen_() == 0) {
    status_ = PrimitiveStatus::kFailed;
    state_ = State::kFailed;
    progress_ = 0.0f;
    return;
  }

  // reset runtime
  t_state_ = 0.0f;
  t_total_ = 0.0f;

  // reset sequence
  idx_ = 0;
  cur_key_ = cfg_.code[0];

  // ALWAYS start released
  io_.set_pusher_norm(cfg_.release_pos_norm);

  status_ = PrimitiveStatus::kRunning;
  enterState_(State::kSettle);
  progress_ = 0.0f;
}

void KeypadEnter::cancel() {
  if (io_.set_pusher_norm) {
    io_.set_pusher_norm(cfg_.release_pos_norm);
  }
  status_ = PrimitiveStatus::kFailed;
  state_ = State::kFailed;
}

void KeypadEnter::reset() {
  if (io_.set_pusher_norm) {
    io_.set_pusher_norm(cfg_.release_pos_norm);
  }
  status_ = PrimitiveStatus::kIdle;
  state_ = State::kIdle;
  t_state_ = 0.0f;
  t_total_ = 0.0f;
  idx_ = 0;
  cur_key_ = '\0';
  progress_ = 0.0f;
}

TankPwmCmd KeypadEnter::update(float dt) {
  TankPwmCmd out{};
  out.left_pwm = 0;
  out.right_pwm = 0;
  out.finished = false;
  out.success = false;

  // idle: do nothing
  if (status_ == PrimitiveStatus::kIdle) {
    out.finished = true;
    out.success = false;
    return out;
  }

  // terminal states: keep returning stop
  if (status_ == PrimitiveStatus::kSucceeded || status_ == PrimitiveStatus::kFailed) {
    out.finished = true;
    out.success = (status_ == PrimitiveStatus::kSucceeded);
    return out;
  }

  // running
  const float dts = safe_dt(dt);
  t_state_ += dts;
  t_total_ += dts;

  // global timeout (don't hang forever)
  if (cfg_.timeout_s > 0.0f && t_total_ >= cfg_.timeout_s) {
    cancel();
    out.finished = true;
    out.success = false;
    return out;
  }

  // progress: idx / len (keeping it simple and predictable here fellas)
  {
    const float n = (codeLen_() == 0) ? 1.0f : static_cast<float>(codeLen_());
    progress_ = clamp01_(static_cast<float>(idx_) / n);
  }

  switch (state_) {
    case State::kSettle: {
      // chill, stay released
      io_.set_pusher_norm(cfg_.release_pos_norm);

      if (t_state_ >= cfg_.settle_s) {
        cur_key_ = cfg_.code[idx_];
        io_.set_key_target(cur_key_);
        enterState_(State::kMoveToKey);
      }
    } break;

    case State::kMoveToKey: {
      // command target EVERY tick (pretty cheap and robust huh?)
      io_.set_key_target(cur_key_);
      io_.set_pusher_norm(cfg_.release_pos_norm);

      const bool at_key = (io_.at_key_target != nullptr) ? io_.at_key_target() : false;

      // If we have "at_key" feedback, use it. Otherwise time out and move on
      if (at_key || t_state_ >= cfg_.move_timeout_s) {
        io_.set_pusher_norm(cfg_.press_pos_norm);
        enterState_(State::kPressHold);
      }
    } break;

    case State::kPressHold: {
      // PRESS (down)
      io_.set_pusher_norm(cfg_.press_pos_norm);

      if (t_state_ >= cfg_.press_hold_s) {
        io_.set_pusher_norm(cfg_.release_pos_norm);
        enterState_(State::kReleaseHold);
      }
    } break;

    case State::kReleaseHold: {
      // RELEASE (up)
      io_.set_pusher_norm(cfg_.release_pos_norm);

      if (t_state_ >= cfg_.release_hold_s) {
        // next character
        idx_++;

        if (idx_ >= codeLen_()) {
          // DONE DONE DONE
          io_.set_pusher_norm(cfg_.release_pos_norm);
          status_ = PrimitiveStatus::kSucceeded;
          progress_ = 1.0f;
          enterState_(State::kDone);
        } else {
          // move to next key
          cur_key_ = cfg_.code[idx_];
          io_.set_key_target(cur_key_);
          enterState_(State::kMoveToKey);
        }
      }
    } break;

    case State::kDone: {
      // keep released so we don't keep mashing keys like a dumbass
      io_.set_pusher_norm(cfg_.release_pos_norm);
    } break;

    case State::kFailed:
    case State::kIdle:
    default:
      cancel();
      break;
  }

  // return completion flags in the REAL TankPwmCmd fields (no out.status nonsense)
  out.finished = (status_ == PrimitiveStatus::kSucceeded || status_ == PrimitiveStatus::kFailed);
  out.success = (status_ == PrimitiveStatus::kSucceeded);
  return out;
}

}  // namespace secbot
