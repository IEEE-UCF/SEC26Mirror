/**
 * @file arm_kinematics.cpp
 * @author Rafeed Khan
 * @brief Implementation of minimal analytic 2-link planar arm kinematics.
 */

#include "arm_kinematics.h"

#include <cmath>

namespace secbot::arm {

static inline float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

float wrapPi(float rad) {
  // Wrap into (-pi, pi]
  while (rad <= -kPi) rad += 2.0f * kPi;
  while (rad > kPi) rad -= 2.0f * kPi;
  return rad;
}

bool withinLimits(const TwoLinkParams& p, float shoulder_rad, float elbow_rad) {
  return (shoulder_rad >= p.shoulder_min_rad &&
          shoulder_rad <= p.shoulder_max_rad && elbow_rad >= p.elbow_min_rad &&
          elbow_rad <= p.elbow_max_rad);
}

void forward2Link(const TwoLinkParams& p, float shoulder_rad, float elbow_rad,
                  float& x_m, float& y_m) {
  const float c1 = std::cos(shoulder_rad);
  const float s1 = std::sin(shoulder_rad);
  const float c12 = std::cos(shoulder_rad + elbow_rad);
  const float s12 = std::sin(shoulder_rad + elbow_rad);

  x_m = p.l1_m * c1 + p.l2_m * c12;
  y_m = p.l1_m * s1 + p.l2_m * s12;
}

IKResult inverse2Link(const TwoLinkParams& p, float x_m, float y_m,
                      bool elbow_up) {
  IKResult out{};
  out.ok = false;
  out.shoulder_rad = 0.0f;
  out.elbow_rad = 0.0f;
  out.reach_error_m = 0.0f;

  const float l1 = p.l1_m;
  const float l2 = p.l2_m;

  // Distance from base to target
  const float r2 = x_m * x_m + y_m * y_m;
  const float r = std::sqrt(r2);

  // Law of cosines for elbow:
  // cos(elbow) = (r^2 - l1^2 - l2^2) / (2*l1*l2)
  float c2 = (r2 - l1 * l1 - l2 * l2) / (2.0f * l1 * l2);

  // Detect reachability, clamping to keep acos() safe
  bool reachable = true;
  if (c2 < -1.0f || c2 > 1.0f) {
    reachable = false;
    c2 = clampf(c2, -1.0f, 1.0f);
  }

  const float elbow_mag = std::acos(c2);
  // Two branches: +acos(c2) and -acos(c2)
  const float elbow = elbow_up ? -elbow_mag : elbow_mag;

  // Shoulder:
  // shoulder = atan2(y, x) - atan2(l2*sin(elbow), l1 + l2*cos(elbow))
  const float k1 = l1 + l2 * std::cos(elbow);
  const float k2 = l2 * std::sin(elbow);

  const float shoulder = std::atan2(y_m, x_m) - std::atan2(k2, k1);

  out.shoulder_rad = wrapPi(shoulder);
  out.elbow_rad = wrapPi(elbow);

  // FK verification error (should be around 0)
  float fx = 0.0f, fy = 0.0f;
  forward2Link(p, out.shoulder_rad, out.elbow_rad, fx, fy);
  out.reach_error_m =
      std::sqrt((fx - x_m) * (fx - x_m) + (fy - y_m) * (fy - y_m));

  const bool limits_ok = withinLimits(p, out.shoulder_rad, out.elbow_rad);
  out.ok = reachable && limits_ok;
  return out;
}

IKResult inverse2LinkBest(const TwoLinkParams& p, float x_m, float y_m) {
  IKResult a = inverse2Link(p, x_m, y_m, /*elbow_up=*/true);
  IKResult b = inverse2Link(p, x_m, y_m, /*elbow_up=*/false);

  // Prefer any fully valid solution
  if (a.ok && !b.ok) return a;
  if (b.ok && !a.ok) return b;

  // If both valid, pick the one with smaller absolute elbow (often "cleaner"
  // motion)
  if (a.ok && b.ok) {
    return (std::fabs(a.elbow_rad) <= std::fabs(b.elbow_rad)) ? a : b;
  }

  // If neither valid, return the one that lands closer to the target
  return (a.reach_error_m <= b.reach_error_m) ? a : b;
}

}  // namespace secbot::arm
