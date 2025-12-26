// motion_profile.cpp

/**
 * @file motion_profile.cpp
 * @author Rafeed Khan
 * @brief Trapezoidal and S-curve motion profile implementations
 * No dynamic allocation, deterministic O(1) update time
 */

#include "motion_profile.h"

TrapezoidalMotionProfile::TrapezoidalMotionProfile(const Config& cfg) {
  configure(cfg);
}

void TrapezoidalMotionProfile::configure(const Config& cfg) {
  cfg_ = cfg;

  // sanitizing constraints (keep everything positive and nonzero-ish)
  cfg_.limits.v_max = sanitizePositive(cfg_.limits.v_max, 0.0f);
  cfg_.limits.a_max = sanitizePositive(cfg_.limits.a_max, 1e-6f);
  cfg_.limits.d_max = sanitizePositive(cfg_.limits.d_max, 1e-6f);

  // tolerances should not be negative
  if (cfg_.pos_tol < 0.0f) cfg_.pos_tol = 0.0f;
  if (cfg_.vel_tol < 0.0f) cfg_.vel_tol = 0.0f;

  reset(st_);
}

void TrapezoidalMotionProfile::setGoal(const MotionGoal& goal) {
  goal_ = goal;
  finished_ = false;

  // Check if we're already at the goal
  if (std::fabs(goal_.pos - st_.pos) <= cfg_.pos_tol &&
      std::fabs(goal_.vel - st_.vel) <= cfg_.vel_tol) {
    st_.pos = goal_.pos;
    st_.vel = goal_.vel;
    st_.acc = 0.0f;
    finished_ = true;
  }
}

const MotionGoal& TrapezoidalMotionProfile::goal() const { return goal_; }

void TrapezoidalMotionProfile::reset(const MotionState& state) {
  st_ = state;
  finished_ = false;

  // if we are already basically at the goal, just finish
  if (std::fabs(goal_.pos - st_.pos) <= cfg_.pos_tol &&
      std::fabs(goal_.vel - st_.vel) <= cfg_.vel_tol) {
    st_.pos = goal_.pos;
    st_.vel = goal_.vel;
    st_.acc = 0.0f;
    finished_ = true;
  }
}

const MotionState& TrapezoidalMotionProfile::state() const { return st_; }

bool TrapezoidalMotionProfile::isFinished() const { return finished_; }

MotionState TrapezoidalMotionProfile::update(float dt) {
  // if we're done, we're done
  if (finished_) return st_;

  // rejecting bad dt
  if (!(dt >= cfg_.min_dt) || !(dt <= cfg_.max_dt) || !isFinite(dt)) {
    // no update if dt is invalid
    return st_;
  }

  // if v_max is 0, this profile cannot move (so we just "finish" at goal)
  if (cfg_.limits.v_max <= 0.0f) {
    st_.pos = goal_.pos;
    st_.vel = goal_.vel;
    st_.acc = 0.0f;
    finished_ = true;
    return st_;
  }

  const float dx = goal_.pos - st_.pos;
  const float dv = goal_.vel - st_.vel;

  // deciding direction:
  // - prefer moving toward position goal
  // - if position is basically done, follow velocity direction
  int8_t dir = 0;
  if (std::fabs(dx) > cfg_.pos_tol) {
    dir = (dx > 0.0f) ? 1 : -1;
  } else if (std::fabs(dv) > cfg_.vel_tol) {
    dir = (dv > 0.0f) ? 1 : -1;
  } else {
    // we're basically at the goal
    st_.pos = goal_.pos;
    st_.vel = goal_.vel;
    st_.acc = 0.0f;
    finished_ = true;
    return st_;
  }

  // work in "positive" space (makes the logic identical for forward/backward)
  const float x = dir * st_.pos;
  const float v = dir * st_.vel;
  const float x_goal = dir * goal_.pos;
  const float v_goal = dir * goal_.vel;

  float dx_pos = x_goal - x;  // desired remaining distance (should be >= 0)
  if (dx_pos < 0.0f) dx_pos = 0.0f;  // if we overshot, treat as zero and brake

  // compute desired acceleration (bang-bang style)
  float a_des = 0.0f;

  // if moving opposite direction (v < 0), push toward the goal direction
  if (v < 0.0f) {
    a_des = cfg_.limits.a_max;
  } else {
    // distance needed to decel from v down to v_goal (only if v > v_goal)
    float d_slow = 0.0f;
    if (v > v_goal) {
      d_slow = (v * v - v_goal * v_goal) / (2.0f * cfg_.limits.d_max);
      if (d_slow < 0.0f) d_slow = 0.0f;
    }

    // if we need to start braking now to hit the goal, brake
    if (dx_pos <= d_slow) {
      a_des = -cfg_.limits.d_max;
    } else {
      // otherwise accelerate up to cruise speed (or cruise if already there)
      if (v < cfg_.limits.v_max) {
        a_des = cfg_.limits.a_max;
      } else {
        a_des = 0.0f;
      }
    }
  }

  // integrate (using average velocity for position update)
  const float v_next =
      clamp(v + a_des * dt, -cfg_.limits.v_max, cfg_.limits.v_max);
  const float v_avg = 0.5f * (v + v_next);

  MotionState next = st_;
  next.vel = dir * v_next;
  next.pos = st_.pos + (dir * v_avg) * dt;
  next.acc = dir * a_des;

  // finishing / snapping logic
  const float dx_after = goal_.pos - next.pos;

  // if we crossed past the goal position, snap to goal cleanly
  if ((dir * dx_after) < 0.0f) {
    next.pos = goal_.pos;
    next.vel = goal_.vel;
    next.acc = 0.0f;
    finished_ = true;
  }

  // if we're within tolerance and velocity is close, also finish
  if (!finished_ && std::fabs(goal_.pos - next.pos) <= cfg_.pos_tol &&
      std::fabs(goal_.vel - next.vel) <= cfg_.vel_tol) {
    next.pos = goal_.pos;
    next.vel = goal_.vel;
    next.acc = 0.0f;
    finished_ = true;
  }

  st_ = next;
  return st_;
}

float TrapezoidalMotionProfile::clamp(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

bool TrapezoidalMotionProfile::isFinite(float x) {
  return (x == x) && (x > -3.4e38f) && (x < 3.4e38f);  // filters NaN/Inf
}

int8_t TrapezoidalMotionProfile::signNonZero(float x) {
  return (x >= 0.0f) ? 1 : -1;
}

float TrapezoidalMotionProfile::sanitizePositive(float x, float fallback) {
  if (!isFinite(x)) return fallback;
  if (x < 0.0f) return -x;
  return x;
}

SCurveMotionProfile::SCurveMotionProfile(const Config& cfg) { configure(cfg); }

void SCurveMotionProfile::configure(const Config& cfg) {
  cfg_ = cfg;

  cfg_.limits.v_max = sanitizePositive(cfg_.limits.v_max, 0.0f);
  cfg_.limits.a_max = sanitizePositive(cfg_.limits.a_max, 1e-6f);
  cfg_.limits.d_max = sanitizePositive(cfg_.limits.d_max, 1e-6f);
  cfg_.limits.j_max = sanitizePositive(cfg_.limits.j_max, 0.0f);

  if (cfg_.pos_tol < 0.0f) cfg_.pos_tol = 0.0f;
  if (cfg_.vel_tol < 0.0f) cfg_.vel_tol = 0.0f;

  reset(st_);
}

void SCurveMotionProfile::setGoal(const MotionGoal& goal) {
  goal_ = goal;
  finished_ = false;

  // Check if we're already at the goal
  if (std::fabs(goal_.pos - st_.pos) <= cfg_.pos_tol &&
      std::fabs(goal_.vel - st_.vel) <= cfg_.vel_tol) {
    st_.pos = goal_.pos;
    st_.vel = goal_.vel;
    st_.acc = 0.0f;
    finished_ = true;
  }
}

const MotionGoal& SCurveMotionProfile::goal() const { return goal_; }

void SCurveMotionProfile::reset(const MotionState& state) {
  st_ = state;
  finished_ = false;

  if (std::fabs(goal_.pos - st_.pos) <= cfg_.pos_tol &&
      std::fabs(goal_.vel - st_.vel) <= cfg_.vel_tol) {
    st_.pos = goal_.pos;
    st_.vel = goal_.vel;
    st_.acc = 0.0f;
    finished_ = true;
  }
}

const MotionState& SCurveMotionProfile::state() const { return st_; }

bool SCurveMotionProfile::isFinished() const { return finished_; }

MotionState SCurveMotionProfile::update(float dt) {
  if (finished_) return st_;

  // rejecting bad dt
  if (!(dt >= cfg_.min_dt) || !(dt <= cfg_.max_dt) || !isFinite(dt)) {
    return st_;
  }

  if (cfg_.limits.v_max <= 0.0f) {
    st_.pos = goal_.pos;
    st_.vel = goal_.vel;
    st_.acc = 0.0f;
    finished_ = true;
    return st_;
  }

  const float dx = goal_.pos - st_.pos;
  const float dv = goal_.vel - st_.vel;

  int8_t dir = 0;
  if (std::fabs(dx) > cfg_.pos_tol) {
    dir = (dx > 0.0f) ? 1 : -1;
  } else if (std::fabs(dv) > cfg_.vel_tol) {
    dir = (dv > 0.0f) ? 1 : -1;
  } else {
    st_.pos = goal_.pos;
    st_.vel = goal_.vel;
    st_.acc = 0.0f;
    finished_ = true;
    return st_;
  }

  // positive-space variables
  const float x = dir * st_.pos;
  const float v = dir * st_.vel;
  const float a = dir * st_.acc;
  const float x_goal = dir * goal_.pos;
  const float v_goal = dir * goal_.vel;

  float dx_pos = x_goal - x;
  if (dx_pos < 0.0f) dx_pos = 0.0f;

  // compute desired accel like trapezoid
  float a_target = 0.0f;

  if (v < 0.0f) {
    a_target = cfg_.limits.a_max;
  } else {
    float d_slow = 0.0f;
    if (v > v_goal) {
      d_slow = (v * v - v_goal * v_goal) / (2.0f * cfg_.limits.d_max);
      if (d_slow < 0.0f) d_slow = 0.0f;
    }

    if (dx_pos <= d_slow) {
      a_target = -cfg_.limits.d_max;
    } else {
      if (v < cfg_.limits.v_max) {
        a_target = cfg_.limits.a_max;
      } else {
        a_target = 0.0f;
      }
    }
  }

  // jerk limiting (this is the whole point of s-curve people!!!!)
  float a_next = a_target;

  if (cfg_.limits.j_max > 0.0f) {
    a_next = approachWithJerk(a, a_target, cfg_.limits.j_max, dt);
  }

  // integrate (avg velocity for position)
  const float v_next =
      clamp(v + a_next * dt, -cfg_.limits.v_max, cfg_.limits.v_max);
  const float v_avg = 0.5f * (v + v_next);

  MotionState next = st_;
  next.vel = dir * v_next;
  next.pos = st_.pos + (dir * v_avg) * dt;
  next.acc = dir * a_next;

  const float dx_after = goal_.pos - next.pos;

  if ((dir * dx_after) < 0.0f) {
    next.pos = goal_.pos;
    next.vel = goal_.vel;
    next.acc = 0.0f;
    finished_ = true;
  }

  if (!finished_ && std::fabs(goal_.pos - next.pos) <= cfg_.pos_tol &&
      std::fabs(goal_.vel - next.vel) <= cfg_.vel_tol) {
    next.pos = goal_.pos;
    next.vel = goal_.vel;
    next.acc = 0.0f;
    finished_ = true;
  }

  st_ = next;
  return st_;
}

float SCurveMotionProfile::approachWithJerk(float a_now, float a_target,
                                            float j_max, float dt) {
  // a_next = a_now + clamp(a_target - a_now, -j_max*dt, +j_max*dt)
  const float da = a_target - a_now;
  const float max_step = j_max * dt;

  if (da > max_step) return a_now + max_step;
  if (da < -max_step) return a_now - max_step;
  return a_target;
}

float SCurveMotionProfile::clamp(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

bool SCurveMotionProfile::isFinite(float x) {
  return (x == x) && (x > -3.4e38f) && (x < 3.4e38f);  // filters NaN/Inf
}

int8_t SCurveMotionProfile::signNonZero(float x) {
  return (x >= 0.0f) ? 1 : -1;
}

float SCurveMotionProfile::sanitizePositive(float x, float fallback) {
  if (!isFinite(x)) return fallback;
  if (x < 0.0f) return -x;
  return x;
}
