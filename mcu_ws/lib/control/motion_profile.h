// motion_profile.h

#ifndef MOTION_PROFILE_H
#define MOTION_PROFILE_H

#include <cmath>
#include <cstdint>

/**
 * @file motion_profile.h
 * @author Rafeed Khan
 * @brief Motion profiling utilities for MCU control loops
 * No dynamic allocation, deterministic O(1) update time
 *
 * This provides two 1D point-to-point motion profiles:
 *  - TrapezoidalMotionProfile: accel-limited trapezoid (velocity vs time is
 * trapezoid-ish)
 *  - SCurveMotionProfile: jerk-limited (S-curve-ish, smoother accel
 * transitions)
 *
 * These generate setpoints (pos/vel/acc) you can feed into PID/FF control
 */

// Common state (position, velocity, acceleration)
struct MotionState {
  float pos = 0.0f;
  float vel = 0.0f;
  float acc = 0.0f;
};

// Goal state (usually end position with end velocity)
struct MotionGoal {
  float pos = 0.0f;
  float vel = 0.0f;  // usually 0 for "stop at target"
};

// Trapezoid constraints (max velocity, max accel, max decel)
struct TrapezoidConstraints {
  float v_max = 0.0f;  // max magnitude of velocity
  float a_max = 0.0f;  // max accel (speeding up)
  float d_max = 0.0f;  // max decel (slowing down)
};

// S-curve constraints (same + jerk limit)
struct SCurveConstraints {
  float v_max = 0.0f;
  float a_max = 0.0f;
  float d_max = 0.0f;
  float j_max = 0.0f;  // max jerk (rate of change of accel)
};

class TrapezoidalMotionProfile {
 public:
  struct Config {
    TrapezoidConstraints limits{};

    // dt guards (keeps us from exploding if scheduler stalls)
    float min_dt = 1e-6f;
    float max_dt = 0.25f;

    // snap tolerances (if we're basically done, we just finish)
    float pos_tol = 1e-4f;
    float vel_tol = 1e-4f;
  };

  TrapezoidalMotionProfile() = default;
  explicit TrapezoidalMotionProfile(const Config& cfg);

  void configure(const Config& cfg);

  void setGoal(const MotionGoal& goal);
  const MotionGoal& goal() const;

  void reset(const MotionState& state = MotionState{});
  const MotionState& state() const;

  // Step forward by dt, return new setpoint
  MotionState update(float dt);

  // Are we at the goal yet?
  bool isFinished() const;

 private:
  static float clamp(float x, float lo, float hi);
  static bool isFinite(float x);
  static int8_t signNonZero(float x);  // returns -1 or +1, never 0

  float sanitizePositive(float x, float fallback);

  Config cfg_{};
  MotionGoal goal_{};
  MotionState st_{};

  bool finished_ = true;
};

class SCurveMotionProfile {
 public:
  struct Config {
    SCurveConstraints limits{};

    float min_dt = 1e-6f;
    float max_dt = 0.25f;

    float pos_tol = 1e-4f;
    float vel_tol = 1e-4f;

    // if jerk is 0, we basically act like trapezoid (so no smoothing!!!)
    // (still safe, just not fancy as i wanted it to be lol)
  };

  SCurveMotionProfile() = default;
  explicit SCurveMotionProfile(const Config& cfg);

  void configure(const Config& cfg);

  void setGoal(const MotionGoal& goal);
  const MotionGoal& goal() const;

  void reset(const MotionState& state = MotionState{});
  const MotionState& state() const;

  MotionState update(float dt);
  bool isFinished() const;

 private:
  static float clamp(float x, float lo, float hi);
  static bool isFinite(float x);
  static int8_t signNonZero(float x);

  float sanitizePositive(float x, float fallback);

  // Ramp 'a' toward 'a_target', clamped by j_max * dt
  float approachWithJerk(float a_now, float a_target, float j_max, float dt);

  Config cfg_{};
  MotionGoal goal_{};
  MotionState st_{};

  bool finished_ = true;
};

#endif  // MOTION_PROFILE_H
