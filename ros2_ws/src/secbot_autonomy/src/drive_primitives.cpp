#include "drive_primitives.h"

#include <cmath>

namespace secbot {
namespace {

constexpr float kPi = 3.14159265358979323846f;

static inline float clampf(float x, float lo, float hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

static inline int16_t clampi16(int32_t x, int16_t lo, int16_t hi) {
  return (x < lo) ? lo : (x > hi) ? hi : static_cast<int16_t>(x);
}

static inline float wrapAngle(float a) {
  while (a <= -kPi) a += 2.0f * kPi;
  while (a > kPi) a -= 2.0f * kPi;
  return a;
}

static inline int16_t sgn_i16(int16_t x) { return (x < 0) ? -1 : 1; }
static inline float sgn_f(float x) { return (x < 0.0f) ? -1.0f : 1.0f; }

}  // namespace

// -------------------- PwmShape --------------------

int16_t PwmShape::clamp(int32_t v) const {
  return clampi16(v, static_cast<int16_t>(-max_pwm), max_pwm);
}

int16_t PwmShape::applyDeadband(int16_t v) const {
  // If it's inside deadband, force 0.
  if (v > -deadband && v < deadband) return 0;
  return v;
}

int16_t PwmShape::shape(int32_t v) const { return applyDeadband(clamp(v)); }

// -------------------- AngleUnwrap --------------------

void AngleUnwrap::reset(float theta) {
  last = theta;
  acc = theta;
  inited = true;
}

float AngleUnwrap::update(float theta) {
  if (!inited) {
    reset(theta);
    return acc;
  }

  const float d = wrapAngle(theta - last);
  acc += d;
  last = theta;
  return acc;
}

// -------------------- DriveStraight --------------------

DriveStraight::DriveStraight(const Config& cfg) : cfg_(cfg) {}

void DriveStraight::start(const Pose2D& pose) {
  start_ = pose;
  target_dist_ = cfg_.distance_m;
  t_elapsed_ = 0.0f;
  progress_ = 0.0f;
  status_ = PrimitiveStatus::kRunning;
}

void DriveStraight::cancel() { status_ = PrimitiveStatus::kFailed; }

TankPwmCmd DriveStraight::update(const Pose2D& pose, float dt) {
  TankPwmCmd out{};

  // if no start, do nothing
  if (status_ == PrimitiveStatus::kIdle) {
    out.finished = true;
    out.success = false;
    return out;
  }

  // if already done, keep returning stop
  if (status_ == PrimitiveStatus::kSucceeded ||
      status_ == PrimitiveStatus::kFailed) {
    out.finished = true;
    out.success = (status_ == PrimitiveStatus::kSucceeded);
    out.left_pwm = 0;
    out.right_pwm = 0;
    return out;
  }

  // running
  t_elapsed_ += (dt > 0.0f ? dt : 0.0f);

  // safety: timeout
  if (cfg_.timeout_s > 0.0f && t_elapsed_ > cfg_.timeout_s) {
    status_ = PrimitiveStatus::kFailed;
    out.finished = true;
    out.success = false;
    return out;
  }

  // Distance traveled along starting heading (projection)
  const float dx = pose.x - start_.x;
  const float dy = pose.y - start_.y;
  const float c0 = std::cos(start_.theta);
  const float s0 = std::sin(start_.theta);
  const float along = dx * c0 + dy * s0;

  const float remaining = target_dist_ - along;

  // progress (0..1)
  const float denom = std::fabs(target_dist_);
  if (denom > 1e-6f) {
    progress_ = clampf(std::fabs(along) / denom, 0.0f, 1.0f);
  } else {
    progress_ = 1.0f;
  }

  // done?
  if (std::fabs(remaining) <= cfg_.pos_tol) {
    // settle heading near finish so you don't drift off the board edge
    const float h_err = wrapAngle(start_.theta - pose.theta);
    if (std::fabs(h_err) <= cfg_.heading_tol) {
      status_ = PrimitiveStatus::kSucceeded;
      out.finished = true;
      out.success = true;
      out.left_pwm = 0;
      out.right_pwm = 0;
      return out;
    }

    // heading settle (no forward motion)
    int32_t corr = static_cast<int32_t>(cfg_.k_heading_pwm * h_err);
    corr = clampi16(corr, static_cast<int16_t>(-cfg_.max_correction_pwm),
                    static_cast<int16_t>(cfg_.max_correction_pwm));

    out.left_pwm = cfg_.pwm.shape(-corr);
    out.right_pwm = cfg_.pwm.shape(+corr);
    out.finished = false;
    out.success = false;
    return out;
  }

  // Base forward PWM from remaining distance (P controller in PWM space)
  int32_t base = static_cast<int32_t>(cfg_.k_pwm_per_m * remaining);

  // Clamp and enforce min PWM only when we're not in the slow zone
  int16_t base_pwm = cfg_.pwm.clamp(base);

  if (std::fabs(remaining) > cfg_.slow_zone_m) {
    // goal: don't stall when far away
    if (base_pwm != 0 && std::abs(base_pwm) < cfg_.pwm.min_pwm) {
      base_pwm = static_cast<int16_t>(cfg_.pwm.min_pwm * sgn_i16(base_pwm));
    }
  }

  // Heading correction (base +/- correction)
  const float h_err = wrapAngle(start_.theta - pose.theta);
  int32_t corr = static_cast<int32_t>(cfg_.k_heading_pwm * h_err);
  corr = clampi16(corr, static_cast<int16_t>(-cfg_.max_correction_pwm),
                  static_cast<int16_t>(cfg_.max_correction_pwm));

  // left = base - corr, right = base + corr
  out.left_pwm = cfg_.pwm.shape(static_cast<int32_t>(base_pwm) - corr);
  out.right_pwm = cfg_.pwm.shape(static_cast<int32_t>(base_pwm) + corr);

  out.finished = false;
  out.success = false;
  return out;
}

// -------------------- TurnInPlace --------------------

TurnInPlace::TurnInPlace(const Config& cfg) : cfg_(cfg) {}

void TurnInPlace::start(const Pose2D& pose) {
  t_elapsed_ = 0.0f;
  progress_ = 0.0f;

  unwrap_.reset(pose.theta);
  theta0_ = unwrap_.acc;
  theta_target_ = theta0_ + cfg_.angle_rad;

  status_ = PrimitiveStatus::kRunning;
}

void TurnInPlace::cancel() { status_ = PrimitiveStatus::kFailed; }

TankPwmCmd TurnInPlace::update(const Pose2D& pose, float dt) {
  TankPwmCmd out{};

  if (status_ == PrimitiveStatus::kIdle) {
    out.finished = true;
    out.success = false;
    return out;
  }

  if (status_ == PrimitiveStatus::kSucceeded ||
      status_ == PrimitiveStatus::kFailed) {
    out.finished = true;
    out.success = (status_ == PrimitiveStatus::kSucceeded);
    return out;
  }

  t_elapsed_ += (dt > 0.0f ? dt : 0.0f);

  if (cfg_.timeout_s > 0.0f && t_elapsed_ > cfg_.timeout_s) {
    status_ = PrimitiveStatus::kFailed;
    out.finished = true;
    out.success = false;
    return out;
  }

  const float theta = unwrap_.update(pose.theta);
  const float err = theta_target_ - theta;

  // progress (0..1)
  const float total = std::fabs(cfg_.angle_rad);
  if (total > 1e-6f) {
    progress_ = clampf(std::fabs(theta - theta0_) / total, 0.0f, 1.0f);
  } else {
    progress_ = 1.0f;
  }

  if (std::fabs(err) <= cfg_.ang_tol) {
    status_ = PrimitiveStatus::kSucceeded;
    out.finished = true;
    out.success = true;
    out.left_pwm = 0;
    out.right_pwm = 0;
    return out;
  }

  // P controller in PWM space.
  int32_t p = static_cast<int32_t>(cfg_.k_pwm_per_rad * err);
  int16_t turn_pwm = cfg_.pwm.clamp(p);

  if (std::fabs(err) > cfg_.slow_zone_rad) {
    // goal: don't stall while far away
    if (turn_pwm != 0 && std::abs(turn_pwm) < cfg_.pwm.min_pwm) {
      turn_pwm = static_cast<int16_t>(cfg_.pwm.min_pwm * sgn_i16(turn_pwm));
    }
  }

  // Tank turn-in-place: left=-turn, right=+turn
  out.left_pwm = cfg_.pwm.shape(-turn_pwm);
  out.right_pwm = cfg_.pwm.shape(+turn_pwm);

  out.finished = false;
  out.success = false;
  return out;
}

// -------------------- DriveArc --------------------

DriveArc::DriveArc(const Config& cfg) : cfg_(cfg) {}

void DriveArc::start(const Pose2D& pose) {
  t_elapsed_ = 0.0f;
  progress_ = 0.0f;

  unwrap_.reset(pose.theta);
  theta0_ = unwrap_.acc;
  theta_target_ = theta0_ + cfg_.angle_rad;

  status_ = PrimitiveStatus::kRunning;
}

void DriveArc::cancel() { status_ = PrimitiveStatus::kFailed; }

TankPwmCmd DriveArc::update(const Pose2D& pose, float dt) {
  TankPwmCmd out{};

  if (status_ == PrimitiveStatus::kIdle) {
    out.finished = true;
    out.success = false;
    return out;
  }

  if (status_ == PrimitiveStatus::kSucceeded ||
      status_ == PrimitiveStatus::kFailed) {
    out.finished = true;
    out.success = (status_ == PrimitiveStatus::kSucceeded);
    return out;
  }

  t_elapsed_ += (dt > 0.0f ? dt : 0.0f);

  if (cfg_.timeout_s > 0.0f && t_elapsed_ > cfg_.timeout_s) {
    status_ = PrimitiveStatus::kFailed;
    out.finished = true;
    out.success = false;
    return out;
  }

  const float theta = unwrap_.update(pose.theta);
  const float err = theta_target_ - theta;

  // progress
  const float total = std::fabs(cfg_.angle_rad);
  if (total > 1e-6f) {
    progress_ = clampf(std::fabs(theta - theta0_) / total, 0.0f, 1.0f);
  } else {
    progress_ = 1.0f;
  }

  if (std::fabs(err) <= cfg_.ang_tol) {
    status_ = PrimitiveStatus::kSucceeded;
    out.finished = true;
    out.success = true;
    out.left_pwm = 0;
    out.right_pwm = 0;
    return out;
  }

  // Arc wheel ratio from curvature:
  // v_left  = v * (1 - track/(2R))
  // v_right = v * (1 + track/(2R))
  // We'll apply the SAME ratio to PWM (good enough for a primitive).
  const float R = (cfg_.radius_m > 1e-3f) ? cfg_.radius_m : 1e-3f;
  const float half_track = 0.5f * cfg_.track_width_m;
  const float k =
      clampf(half_track / R, 0.0f, 0.95f);  // avoid negative/insane inner wheel

  float scale_l = 1.0f - k;
  float scale_r = 1.0f + k;

  // Direction of arc from sign(angle_rad)
  const int16_t dir = static_cast<int16_t>(sgn_f(cfg_.angle_rad));

  // For CW arcs, swap which side is "outer"
  if (cfg_.angle_rad < 0.0f) {
    const float tmp = scale_l;
    scale_l = scale_r;
    scale_r = tmp;
  }

  // Base PWM magnitude forward (signed)
  const int32_t base = static_cast<int32_t>(dir * cfg_.base_pwm);

  // Apply ratio + clamp
  const int32_t l = static_cast<int32_t>(base * scale_l);
  const int32_t r = static_cast<int32_t>(base * scale_r);

  out.left_pwm = cfg_.pwm.shape(l);
  out.right_pwm = cfg_.pwm.shape(r);

  out.finished = false;
  out.success = false;
  return out;
}

}  // namespace secbot
