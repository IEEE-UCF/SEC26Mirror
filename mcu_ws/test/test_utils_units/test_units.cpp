/**
 * @file test_units.cpp
 * @brief Unit tests for unit conversion utilities
 * @date 12/22/2025
 */

#include <unity.h>
#include <units.h>
#include <math_utils.h>
#include <cmath>

using namespace secbot::utils::units;
using namespace secbot::utils;

// Helper function to compare floats with tolerance
bool floatEqual(float a, float b, float epsilon = 0.001f) {
  return fabs(a - b) <= epsilon;
}

void setUp(void) {}
void tearDown(void) {}

// ===== Length Conversion Tests =====

void test_mm_to_m() {
  TEST_ASSERT_TRUE(floatEqual(1.0f, mm_to_m(1000.0f)));
  TEST_ASSERT_TRUE(floatEqual(0.001f, mm_to_m(1.0f)));
  TEST_ASSERT_TRUE(floatEqual(0.0f, mm_to_m(0.0f)));
}

void test_m_to_mm() {
  TEST_ASSERT_TRUE(floatEqual(1000.0f, m_to_mm(1.0f)));
  TEST_ASSERT_TRUE(floatEqual(1.0f, m_to_mm(0.001f)));
  TEST_ASSERT_TRUE(floatEqual(0.0f, m_to_mm(0.0f)));
}

void test_mm_i32_to_m() {
  TEST_ASSERT_TRUE(floatEqual(1.0f, mm_i32_to_m(1000)));
  TEST_ASSERT_TRUE(floatEqual(0.5f, mm_i32_to_m(500)));
  TEST_ASSERT_TRUE(floatEqual(0.0f, mm_i32_to_m(0)));
}

void test_cm_to_m() {
  TEST_ASSERT_TRUE(floatEqual(1.0f, cm_to_m(100.0f)));
  TEST_ASSERT_TRUE(floatEqual(0.01f, cm_to_m(1.0f)));
  TEST_ASSERT_TRUE(floatEqual(0.0f, cm_to_m(0.0f)));
}

void test_m_to_cm() {
  TEST_ASSERT_TRUE(floatEqual(100.0f, m_to_cm(1.0f)));
  TEST_ASSERT_TRUE(floatEqual(1.0f, m_to_cm(0.01f)));
  TEST_ASSERT_TRUE(floatEqual(0.0f, m_to_cm(0.0f)));
}

void test_length_round_trip_mm() {
  float original = 1.234f;
  float converted = m_to_mm(mm_to_m(original));
  TEST_ASSERT_TRUE(floatEqual(original, converted, 0.0001f));
}

void test_length_round_trip_cm() {
  float original = 5.678f;
  float converted = m_to_cm(cm_to_m(original));
  TEST_ASSERT_TRUE(floatEqual(original, converted, 0.0001f));
}

// ===== Linear Speed Conversion Tests =====

void test_mmps_to_mps() {
  TEST_ASSERT_TRUE(floatEqual(1.0f, mmps_to_mps(1000.0f)));
  TEST_ASSERT_TRUE(floatEqual(0.001f, mmps_to_mps(1.0f)));
}

void test_mps_to_mmps() {
  TEST_ASSERT_TRUE(floatEqual(1000.0f, mps_to_mmps(1.0f)));
  TEST_ASSERT_TRUE(floatEqual(1.0f, mps_to_mmps(0.001f)));
}

void test_cmps_to_mps() {
  TEST_ASSERT_TRUE(floatEqual(1.0f, cmps_to_mps(100.0f)));
  TEST_ASSERT_TRUE(floatEqual(0.01f, cmps_to_mps(1.0f)));
}

void test_mps_to_cmps() {
  TEST_ASSERT_TRUE(floatEqual(100.0f, mps_to_cmps(1.0f)));
  TEST_ASSERT_TRUE(floatEqual(1.0f, mps_to_cmps(0.01f)));
}

void test_speed_round_trip() {
  float original = 2.5f;
  float converted = mps_to_mmps(mmps_to_mps(original));
  TEST_ASSERT_TRUE(floatEqual(original, converted, 0.0001f));
}

// ===== Angular Speed Conversion Tests =====

void test_radps_to_rpm() {
  // 1 revolution/s = 60 RPM
  // 1 rev/s = 2*pi rad/s
  float one_rev_per_sec = 2.0f * kPi;
  TEST_ASSERT_TRUE(floatEqual(60.0f, radps_to_rpm(one_rev_per_sec), 0.01f));
}

void test_rpm_to_radps() {
  // 60 RPM = 1 revolution/s = 2*pi rad/s
  TEST_ASSERT_TRUE(floatEqual(2.0f * kPi, rpm_to_radps(60.0f), 0.01f));
}

void test_degps_to_radps() {
  TEST_ASSERT_TRUE(floatEqual(kPi, degps_to_radps(180.0f), 0.01f));
  TEST_ASSERT_TRUE(floatEqual(kPi / 2.0f, degps_to_radps(90.0f), 0.01f));
}

void test_radps_to_degps() {
  TEST_ASSERT_TRUE(floatEqual(180.0f, radps_to_degps(kPi), 0.01f));
  TEST_ASSERT_TRUE(floatEqual(90.0f, radps_to_degps(kPi / 2.0f), 0.01f));
}

void test_angular_speed_round_trip_rpm() {
  float original = 120.0f;
  float converted = radps_to_rpm(rpm_to_radps(original));
  TEST_ASSERT_TRUE(floatEqual(original, converted, 0.01f));
}

void test_angular_speed_round_trip_deg() {
  float original = 45.0f;
  float converted = radps_to_degps(degps_to_radps(original));
  TEST_ASSERT_TRUE(floatEqual(original, converted, 0.01f));
}

// ===== Time Conversion Tests =====

void test_ms_to_s() {
  TEST_ASSERT_TRUE(floatEqual(1.0f, ms_to_s(1000.0f)));
  TEST_ASSERT_TRUE(floatEqual(0.001f, ms_to_s(1.0f)));
  TEST_ASSERT_TRUE(floatEqual(0.0f, ms_to_s(0.0f)));
}

void test_s_to_ms() {
  TEST_ASSERT_TRUE(floatEqual(1000.0f, s_to_ms(1.0f)));
  TEST_ASSERT_TRUE(floatEqual(1.0f, s_to_ms(0.001f)));
  TEST_ASSERT_TRUE(floatEqual(0.0f, s_to_ms(0.0f)));
}

void test_ms_u32_to_s() {
  TEST_ASSERT_TRUE(floatEqual(1.0f, ms_u32_to_s(1000u)));
  TEST_ASSERT_TRUE(floatEqual(0.5f, ms_u32_to_s(500u)));
  TEST_ASSERT_TRUE(floatEqual(0.0f, ms_u32_to_s(0u)));
}

void test_s_to_ms_u32() {
  TEST_ASSERT_EQUAL_UINT32(1000u, s_to_ms_u32(1.0f));
  TEST_ASSERT_EQUAL_UINT32(500u, s_to_ms_u32(0.5f));
  TEST_ASSERT_EQUAL_UINT32(0u, s_to_ms_u32(0.0f));
}

void test_s_to_ms_u32_negative() {
  // Negative time should clamp to 0
  TEST_ASSERT_EQUAL_UINT32(0u, s_to_ms_u32(-1.0f));
}

void test_time_round_trip() {
  float original = 3.456f;
  float converted = s_to_ms(ms_to_s(original));
  TEST_ASSERT_TRUE(floatEqual(original, converted, 0.0001f));
}

// ===== Acceleration Conversion Tests =====

void test_g_to_mps2() {
  TEST_ASSERT_TRUE(floatEqual(kG_mps2, g_to_mps2(1.0f), 0.01f));
  TEST_ASSERT_TRUE(floatEqual(2.0f * kG_mps2, g_to_mps2(2.0f), 0.01f));
  TEST_ASSERT_TRUE(floatEqual(0.0f, g_to_mps2(0.0f)));
}

void test_mps2_to_g() {
  TEST_ASSERT_TRUE(floatEqual(1.0f, mps2_to_g(kG_mps2), 0.01f));
  TEST_ASSERT_TRUE(floatEqual(2.0f, mps2_to_g(2.0f * kG_mps2), 0.01f));
  TEST_ASSERT_TRUE(floatEqual(0.0f, mps2_to_g(0.0f)));
}

void test_acceleration_round_trip() {
  float original = 1.5f;
  float converted = mps2_to_g(g_to_mps2(original));
  TEST_ASSERT_TRUE(floatEqual(original, converted, 0.01f));
}

void test_g_constant_value() {
  // Standard gravity should be approximately 9.80665 m/s^2
  TEST_ASSERT_TRUE(floatEqual(9.80665f, kG_mps2, 0.0001f));
}

// ===== Encoder Conversion Tests =====

void test_ticks_to_rad_full_revolution() {
  int32_t ticks_per_rev = 1000;
  float rad = ticks_to_rad(ticks_per_rev, ticks_per_rev);
  TEST_ASSERT_TRUE(floatEqual(2.0f * kPi, rad, 0.01f));
}

void test_ticks_to_rad_half_revolution() {
  int32_t ticks_per_rev = 1000;
  float rad = ticks_to_rad(500, ticks_per_rev);
  TEST_ASSERT_TRUE(floatEqual(kPi, rad, 0.01f));
}

void test_ticks_to_rad_zero_ticks() {
  TEST_ASSERT_TRUE(floatEqual(0.0f, ticks_to_rad(0, 1000)));
}

void test_ticks_to_rad_zero_ticks_per_rev() {
  // Division by zero protection
  TEST_ASSERT_EQUAL_FLOAT(0.0f, ticks_to_rad(100, 0));
}

void test_rad_to_ticks_full_revolution() {
  int32_t ticks_per_rev = 1000;
  int32_t ticks = rad_to_ticks(2.0f * kPi, ticks_per_rev);
  TEST_ASSERT_EQUAL_INT32(1000, ticks);
}

void test_rad_to_ticks_half_revolution() {
  int32_t ticks_per_rev = 1000;
  int32_t ticks = rad_to_ticks(kPi, ticks_per_rev);
  TEST_ASSERT_EQUAL_INT32(500, ticks);
}

void test_rad_to_ticks_zero_rad() {
  TEST_ASSERT_EQUAL_INT32(0, rad_to_ticks(0.0f, 1000));
}

void test_rad_to_ticks_zero_ticks_per_rev() {
  // Division by zero protection
  TEST_ASSERT_EQUAL_INT32(0, rad_to_ticks(kPi, 0));
}

void test_encoder_round_trip() {
  int32_t ticks_per_rev = 2048;
  int32_t original_ticks = 512;

  float rad = ticks_to_rad(original_ticks, ticks_per_rev);
  int32_t converted_ticks = rad_to_ticks(rad, ticks_per_rev);

  TEST_ASSERT_EQUAL_INT32(original_ticks, converted_ticks);
}

void test_encoder_negative_ticks() {
  int32_t ticks_per_rev = 1000;
  float rad = ticks_to_rad(-500, ticks_per_rev);
  TEST_ASSERT_TRUE(floatEqual(-kPi, rad, 0.01f));
}

// ===== Integration/Practical Tests =====

void test_typical_tof_sensor_reading() {
  // ToF sensor typically reports in mm
  int32_t tof_mm = 350;  // 350mm reading
  float meters = mm_i32_to_m(tof_mm);
  TEST_ASSERT_TRUE(floatEqual(0.35f, meters, 0.001f));
}

void test_typical_imu_reading() {
  // IMU might report 1.5g acceleration
  float accel_g = 1.5f;
  float accel_mps2 = g_to_mps2(accel_g);
  TEST_ASSERT_TRUE(floatEqual(14.71f, accel_mps2, 0.1f));
}

void test_typical_motor_speed() {
  // Motor running at 300 RPM
  float rpm = 300.0f;
  float radps = rpm_to_radps(rpm);
  TEST_ASSERT_TRUE(floatEqual(31.416f, radps, 0.1f));
}

void test_typical_servo_angle() {
  // Servo at 90 degrees
  float deg = 90.0f;
  float rad = degps_to_radps(deg);  // Using degps_to_radps for angle conversion
  TEST_ASSERT_TRUE(floatEqual(kPi / 2.0f, rad, 0.01f));
}

void test_dt_calculation() {
  // Loop running at 50Hz (20ms period)
  uint32_t loop_time_ms = 20u;
  float dt_seconds = ms_u32_to_s(loop_time_ms);
  TEST_ASSERT_TRUE(floatEqual(0.02f, dt_seconds));
}

int main(int argc, char **argv) {
  UNITY_BEGIN();

  // Length conversion tests
  RUN_TEST(test_mm_to_m);
  RUN_TEST(test_m_to_mm);
  RUN_TEST(test_mm_i32_to_m);
  RUN_TEST(test_cm_to_m);
  RUN_TEST(test_m_to_cm);
  RUN_TEST(test_length_round_trip_mm);
  RUN_TEST(test_length_round_trip_cm);

  // Linear speed conversion tests
  RUN_TEST(test_mmps_to_mps);
  RUN_TEST(test_mps_to_mmps);
  RUN_TEST(test_cmps_to_mps);
  RUN_TEST(test_mps_to_cmps);
  RUN_TEST(test_speed_round_trip);

  // Angular speed conversion tests
  RUN_TEST(test_radps_to_rpm);
  RUN_TEST(test_rpm_to_radps);
  RUN_TEST(test_degps_to_radps);
  RUN_TEST(test_radps_to_degps);
  RUN_TEST(test_angular_speed_round_trip_rpm);
  RUN_TEST(test_angular_speed_round_trip_deg);

  // Time conversion tests
  RUN_TEST(test_ms_to_s);
  RUN_TEST(test_s_to_ms);
  RUN_TEST(test_ms_u32_to_s);
  RUN_TEST(test_s_to_ms_u32);
  RUN_TEST(test_s_to_ms_u32_negative);
  RUN_TEST(test_time_round_trip);

  // Acceleration conversion tests
  RUN_TEST(test_g_to_mps2);
  RUN_TEST(test_mps2_to_g);
  RUN_TEST(test_acceleration_round_trip);
  RUN_TEST(test_g_constant_value);

  // Encoder conversion tests
  RUN_TEST(test_ticks_to_rad_full_revolution);
  RUN_TEST(test_ticks_to_rad_half_revolution);
  RUN_TEST(test_ticks_to_rad_zero_ticks);
  RUN_TEST(test_ticks_to_rad_zero_ticks_per_rev);
  RUN_TEST(test_rad_to_ticks_full_revolution);
  RUN_TEST(test_rad_to_ticks_half_revolution);
  RUN_TEST(test_rad_to_ticks_zero_rad);
  RUN_TEST(test_rad_to_ticks_zero_ticks_per_rev);
  RUN_TEST(test_encoder_round_trip);
  RUN_TEST(test_encoder_negative_ticks);

  // Integration tests
  RUN_TEST(test_typical_tof_sensor_reading);
  RUN_TEST(test_typical_imu_reading);
  RUN_TEST(test_typical_motor_speed);
  RUN_TEST(test_typical_servo_angle);
  RUN_TEST(test_dt_calculation);

  return UNITY_END();
}
