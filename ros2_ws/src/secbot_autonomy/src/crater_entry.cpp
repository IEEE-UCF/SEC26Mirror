#include "crater_entry.h"

#include <cmath>

namespace secbot {
namespace {

static inline float clampf(float x, float lo, float hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

}  // namespace

CraterEntryTask::CraterEntryTask(const Config& cfg)
    : cfg_(cfg),
      approach_(cfg_.approach_cfg),
      descend_(cfg_.descend_cfg),
      exit_(cfg_.exit_cfg) {}

void CraterEntryTask::start(const Pose2D& pose) {
  start_pose_ = pose;

  t_total_ = 0.0f;
  t_state_ = 0.0f;
  progress_ = 0.0f;

  // Force the segment distances into the sub-configs
  cfg_.approach_cfg.distance_m = cfg_.approach_distance_m;
  cfg_.descend_cfg.distance_m  = cfg_.descend_distance_m;
  cfg_.exit_cfg.distance_m     = -std::fabs(cfg_.exit_distance_m);  // reverse

  // Rebuild primitives with updated configs (pretty cheap and deterministic!)
  approach_ = DriveStraight(cfg_.approach_cfg);
  descend_  = DriveStraight(cfg_.descend_cfg);
  exit_     = DriveStraight(cfg_.exit_cfg);

  status_ = CraterEntryStatus::kRunning;
  enterState(State::kApproachRim, pose);
}

void CraterEntryTask::cancel() {
  status_ = CraterEntryStatus::kFailed;
  state_ = State::kFail;

  approach_.cancel();
  descend_.cancel();
  exit_.cancel();
}

void CraterEntryTask::enterState(State s, const Pose2D& pose) {
  state_ = s;
  t_state_ = 0.0f;

  switch (state_) {
    case State::kApproachRim:
      // goal: approach rim gently and straight
      approach_.start(pose);
      break;

    case State::kDescendToLine:
      // goal: go far enough down that the tread/wheel touches the crater line (around 3")
      descend_.start(pose);
      break;

    case State::kDwellOnLine:
      // goal: stop briefly to ensure contact/settle
      break;

    case State::kExitCrater:
      // goal: reverse out
      exit_.start(pose);
      break;

    case State::kDone:
      break;

    case State::kFail:
      break;

    default:
      break;
  }
}

TankPwmCmd CraterEntryTask::update(const Pose2D& pose, float dt) {
  TankPwmCmd out{};

  // Idle: do nothing!!!!!!!!
  if (status_ == CraterEntryStatus::kIdle) {
    out.finished = true;
    out.success = false;
    out.left_pwm = 0;
    out.right_pwm = 0;
    return out;
  }

  // Already terminal: keep returning stop
  if (status_ == CraterEntryStatus::kSucceeded || status_ == CraterEntryStatus::kFailed) {
    out.finished = true;
    out.success = (status_ == CraterEntryStatus::kSucceeded);
    out.left_pwm = 0;
    out.right_pwm = 0;
    return out;
  }

  // Running timers
  const float dt_s = (dt > 0.0f) ? dt : 0.0f;
  t_total_ += dt_s;
  t_state_ += dt_s;

  // Global timeout
  if (cfg_.timeout_s > 0.0f && t_total_ > cfg_.timeout_s) {
    status_ = CraterEntryStatus::kFailed;
    state_ = State::kFail;
    out.finished = true;
    out.success = false;
    out.left_pwm = 0;
    out.right_pwm = 0;
    return out;
  }

  // Coarse progress by phase (keeping this VERY simple on purpose)
  // approach = 0..0.33, descend = 0.33..0.66, exit = 0.66..1.0
  switch (state_) {
    case State::kApproachRim: {
      TankPwmCmd cmd = approach_.update(pose, dt_s);

      // progress slice
      progress_ = 0.33f * clampf(approach_.progress(), 0.0f, 1.0f);

      if (cmd.finished) {
        if (!cmd.success && cfg_.fail_fast) {
          status_ = CraterEntryStatus::kFailed;
          state_ = State::kFail;
          out.finished = true;
          out.success = false;
          out.left_pwm = 0;
          out.right_pwm = 0;
          return out;
        }
        enterState(State::kDescendToLine, pose);
        // fallthrough next update tick
        out.left_pwm = 0;
        out.right_pwm = 0;
        out.finished = false;
        out.success = false;
        return out;
      }

      return cmd;
    }

    case State::kDescendToLine: {
      TankPwmCmd cmd = descend_.update(pose, dt_s);

      // progress slice
      progress_ = 0.33f + 0.33f * clampf(descend_.progress(), 0.0f, 1.0f);

      if (cmd.finished) {
        if (!cmd.success && cfg_.fail_fast) {
          status_ = CraterEntryStatus::kFailed;
          state_ = State::kFail;
          out.finished = true;
          out.success = false;
          out.left_pwm = 0;
          out.right_pwm = 0;
          return out;
        }

        // I'm assuming we've reached the “line touch” region if descend distance is tuned right
        // The rules define the line around 3" down from the crater rim
        // so we have to tune cfg.descend_distance_m to guarantee contact
        enterState(State::kDwellOnLine, pose);
        out.left_pwm = 0;
        out.right_pwm = 0;
        out.finished = false;
        out.success = false;
        return out;
      }

      return cmd;
    }

    case State::kDwellOnLine: {
      // goal: hold still briefly (contact/settle)
      progress_ = 0.66f;

      out.left_pwm = 0;
      out.right_pwm = 0;
      out.finished = false;
      out.success = false;

      if (t_state_ >= cfg_.dwell_time_s) {
        enterState(State::kExitCrater, pose);
      }

      return out;
    }

    case State::kExitCrater: {
      TankPwmCmd cmd = exit_.update(pose, dt_s);

      progress_ = 0.66f + 0.34f * clampf(exit_.progress(), 0.0f, 1.0f);

      if (cmd.finished) {
        if (!cmd.success && cfg_.fail_fast) {
          status_ = CraterEntryStatus::kFailed;
          state_ = State::kFail;
          out.finished = true;
          out.success = false;
          out.left_pwm = 0;
          out.right_pwm = 0;
          return out;
        }

        status_ = CraterEntryStatus::kSucceeded;
        state_ = State::kDone;

        out.finished = true;
        out.success = true;
        out.left_pwm = 0;
        out.right_pwm = 0;
        progress_ = 1.0f;
        return out;
      }

      return cmd;
    }

    default:
      break;
  }

  // If we ever land here, fail safe!!!
  status_ = CraterEntryStatus::kFailed;
  state_ = State::kFail;
  out.finished = true;
  out.success = false;
  out.left_pwm = 0;
  out.right_pwm = 0;
  return out;
}

}  // namespace secbot
