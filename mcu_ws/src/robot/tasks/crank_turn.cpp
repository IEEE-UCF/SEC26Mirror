/**
 * @file crank_turn.cpp
 * @author Rafeed Khan
 * @brief Implementation for CrankTurnTask (Antenna #2 crank task).
 */

#include "crank_turn.h"
#include <cmath>

namespace secbot {

static inline int16_t clampi16(int32_t x, int16_t lo, int16_t hi) {
  return (x < lo) ? lo : (x > hi) ? hi : static_cast<int16_t>(x);
}

// -------------------- DegUnwrap --------------------

float CrankTurnTask::wrapDeltaDeg_(float d) {
  // Wrap delta into (-180, 180]
  while (d <= -180.0f) d += 360.0f;
  while (d >  180.0f)  d -= 360.0f;
  return d;
}

void CrankTurnTask::DegUnwrap::reset(float deg) {
  last = deg;
  acc = deg;
  inited = true;
}

float CrankTurnTask::DegUnwrap::update(float deg) {
  if (!inited) {
    reset(deg);
    return acc;
  }
  const float d = CrankTurnTask::wrapDeltaDeg_(deg - last);
  acc += d;
  last = deg;
  return acc;
}

// -------------------- helpers --------------------

float CrankTurnTask::clamp01_(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

void CrankTurnTask::enterState_(State s) {
  state_ = s;
  t_state_ = 0.0f;
}

// -------------------- CrankTurnTask --------------------

CrankTurnTask::CrankTurnTask(const Config& cfg, const IO& io) : cfg_(cfg), io_(io) {
  reset();
}

void CrankTurnTask::start() {
  // If we cannot command the crank motor, FAIL FAST!!!!!
  if (io_.set_crank_pwm == nullptr) {
    status_ = PrimitiveStatus::kFailed;
    state_ = State::kFailed;
    progress_ = 0.0f;
    turned_deg_ = 0.0f;
    return;
  }

  // Compute open-loop spin time (only used if no encoder)
  if (cfg_.open_loop_spin_s > 0.0f) {
    spin_goal_s_ = cfg_.open_loop_spin_s;
  } else {
    const float rate = (cfg_.est_deg_per_s > 1e-3f) ? cfg_.est_deg_per_s : 1e-3f;
    spin_goal_s_ = cfg_.target_deg / rate;
  }

  // Reset timers/feedback
  t_state_ = 0.0f;
  t_total_ = 0.0f;
  progress_ = 0.0f;
  turned_deg_ = 0.0f;

  // Stop motor initially
  io_.set_crank_pwm(0);

  // Prime encoder tracking if available
  if (io_.read_crank_deg) {
    const float a0 = io_.read_crank_deg();
    unwrap_.reset(a0);
    start_unwrapped_deg_ = unwrap_.acc;
  } else {
    unwrap_.inited = false;
    start_unwrapped_deg_ = 0.0f;
  }

  status_ = PrimitiveStatus::kRunning;
  enterState_(State::kSettle);
}

void CrankTurnTask::cancel() {
  if (io_.set_crank_pwm) {
    io_.set_crank_pwm(0);
  }
  status_ = PrimitiveStatus::kFailed;
  enterState_(State::kFailed);
}

void CrankTurnTask::reset() {
  if (io_.set_crank_pwm) {
    io_.set_crank_pwm(0);
  }
  status_ = PrimitiveStatus::kIdle;
  state_ = State::kIdle;
  t_state_ = 0.0f;
  t_total_ = 0.0f;
  progress_ = 0.0f;
  turned_deg_ = 0.0f;
  spin_goal_s_ = 0.0f;
  unwrap_.inited = false;
  start_unwrapped_deg_ = 0.0f;
}

TankPwmCmd CrankTurnTask::update(float dt) {
  TankPwmCmd out{};
  out.left_pwm = 0;
  out.right_pwm = 0;
  out.finished = false;
  out.success = false;

  // Idle: nothing running
  if (status_ == PrimitiveStatus::kIdle) {
    out.finished = true;
    out.success = false;
    return out;
  }

  // Done states: keep returning stop
  if (status_ == PrimitiveStatus::kSucceeded || status_ == PrimitiveStatus::kFailed) {
    out.finished = true;
    out.success = (status_ == PrimitiveStatus::kSucceeded);
    return out;
  }

  if (dt < 0.0f) dt = 0.0f;
  t_state_ += dt;
  t_total_ += dt;

  // Global timeout
  if (cfg_.timeout_s > 0.0f && t_total_ >= cfg_.timeout_s) {
    cancel();
    out.finished = true;
    out.success = false;
    return out;
  }

  // Optional “task complete” signal wins immediately (LED/vision/etc)
  if (io_.read_task_complete && io_.read_task_complete()) {
    io_.set_crank_pwm(0);
    status_ = PrimitiveStatus::kSucceeded;
    enterState_(State::kDone);
    progress_ = 1.0f;

    out.finished = true;
    out.success = true;
    return out;
  }

  // Update turned degrees if encoder exists (once again someone please make the encoder)
  if (io_.read_crank_deg) {
    const float a = io_.read_crank_deg();
    const float uw = unwrap_.update(a);
    turned_deg_ = std::fabs(uw - start_unwrapped_deg_);
  } else {
    // Open-loop approximation: progress based on time in SPIN
    // (turned_deg_ becomes "estimated", not real)
    if (state_ == State::kSpin && spin_goal_s_ > 1e-3f) {
      turned_deg_ = cfg_.target_deg * clamp01_(t_state_ / spin_goal_s_);
    }
  }

  // Progress is always a simple fraction toward target
  if (cfg_.target_deg > 1e-3f) {
    progress_ = clamp01_(turned_deg_ / cfg_.target_deg);
  } else {
    progress_ = 1.0f;
  }

  // --------------------------
  // S.M.T (STATE MACHINE TIME!!!)
  // --------------------------
  switch (state_) {
    case State::kSettle: {
      // keep motor stopped while settling
      io_.set_crank_pwm(0);

      if (t_state_ >= cfg_.settle_s) {
        enterState_(State::kSpin);
      }
    } break;

    case State::kSpin: {
      // SPIN IT!!!
      const int16_t dir = (cfg_.direction >= 0) ? 1 : -1;
      const int16_t cmd = clampi16(static_cast<int32_t>(dir) * cfg_.spin_pwm, -255, 255);
      io_.set_crank_pwm(cmd);

      bool done = false;

      // If we have encoder: stop when turned >= target - tol
      if (io_.read_crank_deg) {
        if (turned_deg_ >= (cfg_.target_deg - cfg_.deg_tol)) done = true;
      } else {
        // No encoder: stop when time exceeded
        if (t_state_ >= spin_goal_s_) done = true;
      }

      if (done) {
        io_.set_crank_pwm(0);
        enterState_(State::kHold);
      }
    } break;

    case State::kHold: {
      // stop and let everything settle so you don't overshoot / bounce back
      io_.set_crank_pwm(0);

      if (t_state_ >= cfg_.hold_s) {
        status_ = PrimitiveStatus::kSucceeded;
        enterState_(State::kDone);
        progress_ = 1.0f;

        out.finished = true;
        out.success = true;
        return out;
      }
    } break;

    case State::kDone:
    case State::kFailed:
    case State::kIdle:
    default:
      // ADDING MORE FAILSAFES AHHH SHOULDNT BE HERE WHILE RUNNING
      cancel();
      out.finished = true;
      out.success = false;
      return out;
  }

  out.finished = false;
  out.success = false;
  return out;
}

}  // namespace secbot
