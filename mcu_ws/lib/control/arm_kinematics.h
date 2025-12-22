#pragma once
/**
 * @file arm_kinematics.h
 * @author Rafeed Khan
 * @brief Minimal analytic kinematics helpers for a 2-DOF planar arm.
 *
 * Assumptions:
 *  - 2 revolute joints in a plane (shoulder, elbow)
 *  - link lengths l1, l2 (meters)
 *  - target is in the same plane as the arm, expressed as (x, y) meters
 *
 * Notes:
 *  - This is just closed-form IK/FK (law of cosines + atan2).
 */

#include <cstdint>

namespace secbot::arm {

// BIG STINKY NUMBER LOL MAN!!!!! LOL!!!!
inline constexpr float kPi = 3.14159265358979323846f;

struct TwoLinkParams {
  float l1_m;  // link 1 length (meters)
  float l2_m;  // link 2 length (meters)

  // Joint limits (in radians), set wide limits if you don't care.
  float shoulder_min_rad;
  float shoulder_max_rad;
  float elbow_min_rad;
  float elbow_max_rad;
};

struct IKResult {
  bool ok;              // true if target is reachable AND within joint limits
  float shoulder_rad;   // joint 1
  float elbow_rad;      // joint 2
  float reach_error_m;  // distance error (0 if reachable), useful when ok=false
};

/// Wrap angle to (-pi, pi]
float wrapPi(float rad);

/// Return true if angles are within the configured joint limits
bool withinLimits(const TwoLinkParams& p, float shoulder_rad, float elbow_rad);

/// Forward kinematics: (shoulder, elbow) -> (x, y)
void forward2Link(const TwoLinkParams& p, float shoulder_rad, float elbow_rad,
                  float& x_m, float& y_m);

/// Inverse kinematics (analytic): (x, y) -> (shoulder, elbow)
/// elbow_up selects the "elbow up" branch (the other branch is elbow_down)
IKResult inverse2Link(const TwoLinkParams& p, float x_m, float y_m,
                      bool elbow_up);

/// Try both branches and return the first valid solution
// If none are valid, it returns the one with smaller reach_error
IKResult inverse2LinkBest(const TwoLinkParams& p, float x_m, float y_m);

}  // namespace secbot::arm
