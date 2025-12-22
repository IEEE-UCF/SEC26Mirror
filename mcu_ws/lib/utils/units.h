#pragma once
/**
 * @file units.h
 * @author Rafeed Khan
 * @brief Unit conversion helpers.
 *
 * Rule: keep everything in SI internally where possible (meters, seconds,
 * radians). This keeps ROS/micro-ROS interop sane (ex: sensor_msgs/Imu uses
 * rad/s + m/s^2).
 */

#include <cstdint>

#include "math_utils.h"  // kPi, deg2rad(), rad2deg()

namespace secbot::utils::units {

// ----------------------------
// Length
// ----------------------------

// Millimeters <-> meters
constexpr float mm_to_m(float mm) { return mm * 0.001f; }
constexpr float m_to_mm(float m) { return m * 1000.0f; }

// Integer mm input convenience (common for ToF sensors)
constexpr float mm_i32_to_m(int32_t mm) { return float(mm) * 0.001f; }

// Centimeters <-> meters
constexpr float cm_to_m(float cm) { return cm * 0.01f; }
constexpr float m_to_cm(float m) { return m * 100.0f; }

// ----------------------------
// Linear speed
// ----------------------------

// mm/s <-> m/s
constexpr float mmps_to_mps(float mmps) { return mmps * 0.001f; }
constexpr float mps_to_mmps(float mps) { return mps * 1000.0f; }

// cm/s <-> m/s
constexpr float cmps_to_mps(float cmps) { return cmps * 0.01f; }
constexpr float mps_to_cmps(float mps) { return mps * 100.0f; }

// ----------------------------
// Angular speed
// ----------------------------

// rad/s <-> RPM
constexpr float radps_to_rpm(float radps) {
  return radps * (60.0f / (2.0f * secbot::utils::kPi));
}
constexpr float rpm_to_radps(float rpm) {
  return rpm * ((2.0f * secbot::utils::kPi) / 60.0f);
}

// deg/s <-> rad/s
constexpr float degps_to_radps(float degps) {
  return secbot::utils::deg2rad(degps);
}
constexpr float radps_to_degps(float radps) {
  return secbot::utils::rad2deg(radps);
}

// ----------------------------
// Time
// ----------------------------

// milliseconds <-> seconds
constexpr float ms_to_s(float ms) { return ms * 0.001f; }
constexpr float s_to_ms(float s) { return s * 1000.0f; }

// integer ms helpers (common in MCU timers)
constexpr float ms_u32_to_s(uint32_t ms) { return float(ms) * 0.001f; }
constexpr uint32_t s_to_ms_u32(float s) {
  return (s <= 0.0f) ? 0u : uint32_t(s * 1000.0f);
}

// ----------------------------
// Acceleration
// ----------------------------

// standard gravity (m/s^2)
static constexpr float kG_mps2 = 9.80665f;

// g <-> m/s^2
constexpr float g_to_mps2(float g) { return g * kG_mps2; }
constexpr float mps2_to_g(float a) { return a / kG_mps2; }

// ----------------------------
// Encoders (generic helper)
// ----------------------------

// ticks -> radians, given ticks_per_rev
constexpr float ticks_to_rad(int32_t ticks, int32_t ticks_per_rev) {
  if (ticks_per_rev == 0) return 0.0f;
  return (float(ticks) * (2.0f * secbot::utils::kPi)) / float(ticks_per_rev);
}

// radians -> ticks, given ticks_per_rev
constexpr int32_t rad_to_ticks(float rad, int32_t ticks_per_rev) {
  if (ticks_per_rev == 0) return 0;
  return int32_t((rad * float(ticks_per_rev)) / (2.0f * secbot::utils::kPi));
}

}  // namespace secbot::utils::units
