#pragma once
/**
 * @file math_utils.h
 * @author Rafeed Khan
 * @brief Small math helpers for control + navigation, math is hard yknow?
 */

#ifdef __cplusplus

#include <stdint.h>
#include <cmath>

namespace secbot::utils {

static constexpr float kPi = 3.14159265358979323846f;

// ----------------------------
// Basic helpers
// ----------------------------

// Constrain value to [lo, hi]
template <typename T>
constexpr T clamp(T v, T lo, T hi) {
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

// Generic absolute value (works with any numeric type)
template <typename T>
constexpr T absT(T v) {
  return (v < T(0)) ? -v : v;
}

// Returns -1 or +1 (note: returns +1 for zero)
template <typename T>
constexpr T signT(T v) {
  return (v < T(0)) ? T(-1) : T(1);
}

// ----------------------------
//   map
// - integer map uses integer math
// - float map uses float math
// - neither clamps by default
// ----------------------------

// This is like an Arduino-style integer map, but safer (int64 intermediate to reduce overflow risk!!!!)
constexpr int32_t map_i32(int32_t x,
                          int32_t in_min, int32_t in_max,
                          int32_t out_min, int32_t out_max) {
  if (in_max == in_min) return out_min; // avoid div-by-zero
  const int64_t num = int64_t(x - in_min) * int64_t(out_max - out_min);
  const int64_t den = int64_t(in_max - in_min);
  return int32_t(num / den + out_min);
}

// Float map (recommended for joystick -> velocity scaling)
constexpr float map_f(float x,
                      float in_min, float in_max,
                      float out_min, float out_max) {
  const float den = (in_max - in_min);
  if (den == 0.0f) return out_min; // avoid div-by-zero
  return (x - in_min) * (out_max - out_min) / den + out_min;
}

// Convenience: mapped + clamped to output bounds (handles inverted ranges)
constexpr float map_f_clamped(float x,
                              float in_min, float in_max,
                              float out_min, float out_max) {
  const float y = map_f(x, in_min, in_max, out_min, out_max);
  const float lo = (out_min < out_max) ? out_min : out_max;
  const float hi = (out_min < out_max) ? out_max : out_min;
  return clamp(y, lo, hi);
}

// ----------------------------
// Deadband (joystick / cmd inputs)
// ----------------------------

// Kills small values (noise/drift), rescales the rest to full range.
// Example: deadband=0.1, maxMag=1.0
// input 0.05  -> 0.0 (inside deadband)
// input 0.1   -> 0.0 (edge of deadband)
// input 0.55  -> 0.5 (halfway through remaining range)
// input 1.0   -> 1.0 (full scale)
inline float applyDeadband(float value, float deadband, float maxMagnitude = 1.0f) {
  deadband = std::fabs(deadband);
  if (maxMagnitude <= 0.0f) return 0.0f;

  const float absV = std::fabs(value);
  if (absV <= deadband) return 0.0f;

  const float denom = (maxMagnitude - deadband);
  if (denom <= 0.0f) {
    // deadband >= maxMagnitude, anything outside is just full-scale sign
    return (value < 0.0f) ? -maxMagnitude : maxMagnitude;
  }

  const float scaled = (absV - deadband) / denom * maxMagnitude;
  const float out = clamp(scaled, 0.0f, maxMagnitude);
  return (value < 0.0f) ? -out : out;
}

// ----------------------------
// Angle utilities (radians)
// ----------------------------

// Wrap angle to (-pi, pi] - useful for keeping headings sane
inline float normalizeAngleRad(float angle) {
  angle = std::fmod(angle, 2.0f * kPi);
  if (angle <= -kPi) angle += 2.0f * kPi;
  if (angle >  kPi)  angle -= 2.0f * kPi;
  return angle;
}

// Shortest rotation from one angle to another, result in [-pi, pi]
// Positive = counterclockwise, negative = clockwise
inline float shortestAngularDistanceRad(float from, float to) {
  return normalizeAngleRad(to - from);
}

// ----------------------------
// Degrees/radians
// ----------------------------
constexpr float deg2rad(float deg) { return deg * (kPi / 180.0f); }
constexpr float rad2deg(float rad) { return rad * (180.0f / kPi); }

} // namespace secbot::utils

#endif // __cplusplus