/**
 * @file antenna_align.cpp
 * @author Rafeed Khan
 * @brief Implementation of antenna alignment task (raw PWM, tank drive)
 */

#include "antenna_align.h"

#include <cmath>

namespace secbot {
namespace {

// heee heee haaa big number again i love using this variant of Pi
constexpr float kPi = 3.14159265358979323846f;

static inline int16_t sign_i16(int16_t v) {
  return (v > 0) ? 1 : (v < 0) ? -1 : 0;
}

}  // namespace

// -------------------- helpers --------------------

float AntennaAlign::wrapAngle_(float a) {
  while (a <= -kPi) a += 2.0f * kPi;
  while (a >   kPi) a -= 2.0f * kPi;
  return a;
}

float AntennaAlign::clampf_(float x, float lo, float hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

int16_t AntennaAlign::abs16_(int16_t v) {
  return (v < 0) ? static_cast<int16_t>(-v) : v;
}

// -------------------- rules mapping --------------------

AntennaFace AntennaAlign::faceForId_(uint8_t antenna_id) {
  // per ruleset:
  //  - Antenna 1 faces SOUTH
  //  - Antenna 2 faces SOUTH
  //  - Antenna 3 (crater) faces WEST
  //  - Antenna 4 (keypad) faces NORTH
  switch (antenna_id) {
    case 1: return AntennaFace::kSouth;
    case 2: return AntennaFace::kSouth;
    case 3: return AntennaFace::kWest;
    case 4: return AntennaFace::kNorth;
    default: return AntennaFace::kUnknown;
  }
}

AntennaFace AntennaAlign::opposite_(AntennaFace f) {
  switch (f) {
    case AntennaFace::kNorth: return AntennaFace::kSouth;
    case AntennaFace::kSouth: return AntennaFace::kNorth;
    case AntennaFace::kEast:  return AntennaFace::kWest;
    case AntennaFace::kWest:  return AntennaFace::kEast;
    default: return AntennaFace::kUnknown;
  }
}

float AntennaAlign::headingForFace_(AntennaFace f) {
  // map-frame heading (rad)
  // East=0, North=+pi/2, West=pi, South=-pi/2
  switch (f) {
    case AntennaFace::kEast:  return 0.0f;
    case AntennaFace::kNorth: return 0.5f * kPi;
    case AntennaFace::kWest:  return kPi;
    case AntennaFace::kSouth: return -0.5f * kPi;
    default: return 0.0f;
  }
}

void AntennaAlign::computeGoalFromAntenna_(uint8_t antenna_id, float ax, float ay) {
  const AntennaFace face = faceForId_(antenna_id);
  const float face_h = headingForFace_(face);

  // goal: point in FRONT of the task face (same direction the face points)
  const float ux = std::cos(face_h);
  const float uy = std::sin(face_h);

  goal_x_ = ax + ux * cfg_.standoff_from_center_m;
  goal_y_ = ay + uy * cfg_.standoff_from_center_m;

  // final heading: robot should FACE the antenna (opposite of face direction)
  const float robot_h = wrapAngle_(face_h + kPi);
  final_heading_ = robot_h;
}

// -------------------- AntennaAlign --------------------

AntennaAlign::AntennaAlign(const Config& cfg) : cfg_(cfg) {}

void AntennaAlign::start(uint8_t antenna_id, float antenna_x_m, float antenna_y_m) {
  // goal: compute goal standoff point + final heading from rules
  computeGoalFromAntenna_(antenna_id, antenna_x_m, antenna_y_m);

  // reset runtime
  t_elapsed_ = 0.0f;
  status_ = PrimitiveStatus::kRunning;
  state_ = State::kRotateToGoal;

  // feedback init (will be set on first update)
  start_dist_ = 0.0f;
  dist_rem_m_ = 0.0f;
  heading_err_rad_ = 0.0f;
  progress_ = 0.0f;
}

void AntennaAlign::startCustom(float goal_x_m, float goal_y_m, float final_heading_rad) {
  goal_x_ = goal_x_m;
  goal_y_ = goal_y_m;
  final_heading_ = wrapAngle_(final_heading_rad);

  t_elapsed_ = 0.0f;
  status_ = PrimitiveStatus::kRunning;
  state_ = State::kRotateToGoal;

  start_dist_ = 0.0f;
  dist_rem_m_ = 0.0f;
  heading_err_rad_ = 0.0f;
  progress_ = 0.0f;
}

void AntennaAlign::cancel() {
  status_ = PrimitiveStatus::kFailed;
  state_ = State::kIdle;
}

TankPwmCmd AntennaAlign::update(const Pose2D& pose, float dt, float front_range_m) {
  TankPwmCmd out{};
  out.left_pwm = 0;
  out.right_pwm = 0;
  out.finished = false;
  out.success = false;

  // idle? DO NOTHING!!!
  if (status_ == PrimitiveStatus::kIdle) {
    out.finished = true;
    out.success = false;
    return out;
  }

  // already done? KEEP RETURNING STOP!!!
  if (status_ == PrimitiveStatus::kSucceeded || status_ == PrimitiveStatus::kFailed) {
    out.left_pwm = 0;
    out.right_pwm = 0;
    out.finished = true;
    out.success = (status_ == PrimitiveStatus::kSucceeded);
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

  // compute vector to goal
  const float dx = goal_x_ - pose.x;
  const float dy = goal_y_ - pose.y;
  const float dist = std::sqrt(dx * dx + dy * dy);

  dist_rem_m_ = dist;

  // progress (0..1)
  if (start_dist_ <= 1e-6f) start_dist_ = dist;  // latch on first tick
  if (start_dist_ > 1e-6f) {
    progress_ = clampf_(1.0f - (dist / start_dist_), 0.0f, 1.0f);
  } else {
    progress_ = 1.0f;
  }

  // desired heading to the goal point (for the approach)
  const float goal_heading = std::atan2(dy, dx);
  const float yaw_err_to_goal = wrapAngle_(goal_heading - pose.theta);
  const float yaw_err_final = wrapAngle_(final_heading_ - pose.theta);

  // range stop (if ToF says we're already close enough, stop driving)
  if (cfg_.use_front_range_stop && front_range_m > 0.0f && front_range_m <= cfg_.front_stop_range_m) {
    state_ = State::kFinalHeading;
  }

  switch (state_) {
    case State::kRotateToGoal: {
      // goal: point the robot toward the standoff point before driving
      heading_err_rad_ = yaw_err_to_goal;

      if (std::fabs(yaw_err_to_goal) <= cfg_.yaw_gate_rad) {
        state_ = State::kDriveToStandoff;
        // fallthrough next tick
        out.left_pwm = 0;
        out.right_pwm = 0;
        return out;
      }

      // P turn in PWM space
      int32_t p = static_cast<int32_t>(cfg_.k_turn_pwm_per_rad * yaw_err_to_goal);
      int16_t turn_pwm = cfg_.pwm.clamp(p);

      // goal: don't stall when far away
      if (std::fabs(yaw_err_to_goal) > cfg_.slow_zone_rad) {
        if (turn_pwm != 0 && abs16_(turn_pwm) < cfg_.pwm.min_pwm) {
          turn_pwm = static_cast<int16_t>(cfg_.pwm.min_pwm * sign_i16(turn_pwm));
        }
      }

      // tank turn in place: left = -turn, right = +turn
      out.left_pwm  = cfg_.pwm.shape(-turn_pwm);
      out.right_pwm = cfg_.pwm.shape(+turn_pwm);
      return out;
    }

    case State::kDriveToStandoff: {
      // if we got turned away somehow, rotate again (simple and safe)
      if (std::fabs(yaw_err_to_goal) > (cfg_.yaw_gate_rad * 1.6f)) {
        state_ = State::kRotateToGoal;
        out.left_pwm = 0;
        out.right_pwm = 0;
        return out;
      }

      // goal: arrive at standoff
      heading_err_rad_ = yaw_err_to_goal;

      if (dist <= cfg_.pos_tol_m) {
        state_ = State::kFinalHeading;
        out.left_pwm = 0;
        out.right_pwm = 0;
        return out;
      }

      // base forward PWM from remaining distance (P controller)
      int32_t base = static_cast<int32_t>(cfg_.k_pwm_per_m * dist);
      int16_t base_pwm = cfg_.pwm.clamp(base);

      // goal: don't stall when far away
      if (dist > cfg_.slow_zone_m) {
        if (base_pwm != 0 && abs16_(base_pwm) < cfg_.pwm.min_pwm) {
          base_pwm = static_cast<int16_t>(cfg_.pwm.min_pwm);
        }
      }

      // heading correction while driving
      int32_t corr = static_cast<int32_t>(cfg_.k_heading_pwm * yaw_err_to_goal);
      if (corr > cfg_.max_correction_pwm) corr = cfg_.max_correction_pwm;
      if (corr < -cfg_.max_correction_pwm) corr = -cfg_.max_correction_pwm;

      // left = base - corr, right = base + corr
      out.left_pwm  = cfg_.pwm.shape(static_cast<int32_t>(base_pwm) - corr);
      out.right_pwm = cfg_.pwm.shape(static_cast<int32_t>(base_pwm) + corr);
      return out;
    }

    case State::kFinalHeading: {
      // goal: square up to the antenna face (robot faces antenna)
      heading_err_rad_ = yaw_err_final;

      if (std::fabs(yaw_err_final) <= cfg_.yaw_tol_rad) {
        status_ = PrimitiveStatus::kSucceeded;
        out.left_pwm = 0;
        out.right_pwm = 0;
        out.finished = true;
        out.success = true;
        return out;
      }

      int32_t p = static_cast<int32_t>(cfg_.k_turn_pwm_per_rad * yaw_err_final);
      int16_t turn_pwm = cfg_.pwm.clamp(p);

      if (std::fabs(yaw_err_final) > cfg_.slow_zone_rad) {
        if (turn_pwm != 0 && abs16_(turn_pwm) < cfg_.pwm.min_pwm) {
          turn_pwm = static_cast<int16_t>(cfg_.pwm.min_pwm * sign_i16(turn_pwm));
        }
      }

      out.left_pwm  = cfg_.pwm.shape(-turn_pwm);
      out.right_pwm = cfg_.pwm.shape(+turn_pwm);
      return out;
    }

    default:
      break;
  }

  // should never hit, but lets stay safe
  status_ = PrimitiveStatus::kFailed;
  out.left_pwm = 0;
  out.right_pwm = 0;
  out.finished = true;
  out.success = false;
  return out;
}

}  // namespace secbot
