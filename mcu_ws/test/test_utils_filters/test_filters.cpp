/**
 * @file test_filters.cpp
 * @brief Unit tests for signal filtering utilities (median3, MovingAverage, LowPass1P)
 * @date 12/22/2025
 */

#include <unity.h>
#include <filters.h>
#include <cmath>

using namespace secbot::utils;

// Helper function to compare floats with tolerance
bool floatEqual(float a, float b, float epsilon = 0.001f) {
  return fabs(a - b) < epsilon;
}

void setUp(void) {}
void tearDown(void) {}

// ===== median3 Tests =====

void test_median3_ordered() {
  // Already sorted ascending
  TEST_ASSERT_EQUAL_FLOAT(2.0f, median3(1.0f, 2.0f, 3.0f));
}

void test_median3_reverse_ordered() {
  // Sorted descending
  TEST_ASSERT_EQUAL_FLOAT(2.0f, median3(3.0f, 2.0f, 1.0f));
}

void test_median3_middle_first() {
  TEST_ASSERT_EQUAL_FLOAT(2.0f, median3(2.0f, 1.0f, 3.0f));
}

void test_median3_middle_last() {
  TEST_ASSERT_EQUAL_FLOAT(2.0f, median3(1.0f, 3.0f, 2.0f));
}

void test_median3_all_equal() {
  TEST_ASSERT_EQUAL_FLOAT(5.0f, median3(5.0f, 5.0f, 5.0f));
}

void test_median3_two_equal_low() {
  TEST_ASSERT_EQUAL_FLOAT(1.0f, median3(1.0f, 1.0f, 5.0f));
}

void test_median3_two_equal_high() {
  TEST_ASSERT_EQUAL_FLOAT(5.0f, median3(1.0f, 5.0f, 5.0f));
}

void test_median3_negative_values() {
  TEST_ASSERT_EQUAL_FLOAT(-1.0f, median3(-5.0f, -1.0f, 3.0f));
}

void test_median3_spike_rejection() {
  // Spike at 100, should return median 2.0
  TEST_ASSERT_EQUAL_FLOAT(2.0f, median3(1.0f, 2.0f, 100.0f));
}

// ===== MovingAverage Tests =====

void test_moving_avg_initial_value() {
  MovingAverage<5> ma;
  TEST_ASSERT_EQUAL_FLOAT(0.0f, ma.value());
}

void test_moving_avg_reset() {
  MovingAverage<5> ma;
  ma.update(10.0f);
  ma.reset(5.0f);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, ma.value());
  TEST_ASSERT_FALSE(ma.filled());
}

void test_moving_avg_single_sample() {
  MovingAverage<3> ma;
  float result = ma.update(10.0f);
  TEST_ASSERT_EQUAL_FLOAT(10.0f, result);
  TEST_ASSERT_EQUAL_FLOAT(10.0f, ma.value());
}

void test_moving_avg_filling() {
  MovingAverage<3> ma;
  ma.update(1.0f);
  TEST_ASSERT_FALSE(ma.filled());
  ma.update(2.0f);
  TEST_ASSERT_FALSE(ma.filled());
  ma.update(3.0f);
  TEST_ASSERT_TRUE(ma.filled());
}

void test_moving_avg_count() {
  MovingAverage<5> ma;
  TEST_ASSERT_EQUAL(0, ma.count());
  ma.update(1.0f);
  TEST_ASSERT_EQUAL(1, ma.count());
  ma.update(2.0f);
  TEST_ASSERT_EQUAL(2, ma.count());
}

void test_moving_avg_steady_state() {
  MovingAverage<3> ma;
  ma.update(1.0f);  // avg = 1.0
  ma.update(2.0f);  // avg = 1.5
  ma.update(3.0f);  // avg = 2.0
  TEST_ASSERT_TRUE(floatEqual(2.0f, ma.value()));

  ma.update(4.0f);  // avg = 3.0 (drops 1.0, adds 4.0)
  TEST_ASSERT_TRUE(floatEqual(3.0f, ma.value()));
}

void test_moving_avg_constant_input() {
  MovingAverage<5> ma;
  for (int i = 0; i < 10; i++) {
    ma.update(7.5f);
  }
  TEST_ASSERT_TRUE(floatEqual(7.5f, ma.value()));
}

void test_moving_avg_alternating() {
  MovingAverage<2> ma;
  ma.update(0.0f);
  ma.update(10.0f);
  TEST_ASSERT_TRUE(floatEqual(5.0f, ma.value()));

  ma.update(0.0f);
  TEST_ASSERT_TRUE(floatEqual(5.0f, ma.value()));
}

void test_moving_avg_negative_values() {
  MovingAverage<3> ma;
  ma.update(-1.0f);
  ma.update(-2.0f);
  ma.update(-3.0f);
  TEST_ASSERT_TRUE(floatEqual(-2.0f, ma.value()));
}

// ===== LowPass1P Tests =====

void test_lowpass_initial_value() {
  LowPass1P lp;
  TEST_ASSERT_EQUAL_FLOAT(0.0f, lp.value());
  TEST_ASSERT_FALSE(lp.initialized());
}

void test_lowpass_reset() {
  LowPass1P lp;
  lp.update(10.0f);
  lp.reset(5.0f, true);
  TEST_ASSERT_EQUAL_FLOAT(5.0f, lp.value());
  TEST_ASSERT_TRUE(lp.initialized());
}

void test_lowpass_first_sample_initialization() {
  LowPass1P lp;
  lp.setAlpha(0.5f);
  float result = lp.update(10.0f);
  TEST_ASSERT_EQUAL_FLOAT(10.0f, result);
  TEST_ASSERT_TRUE(lp.initialized());
}

void test_lowpass_alpha_direct() {
  LowPass1P lp;
  lp.setAlpha(0.5f);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, lp.alpha());
}

void test_lowpass_alpha_clamp_low() {
  LowPass1P lp;
  lp.setAlpha(-0.5f);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, lp.alpha());  // Should clamp to 1.0 (zero becomes 1.0)
}

void test_lowpass_alpha_clamp_high() {
  LowPass1P lp;
  lp.setAlpha(1.5f);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, lp.alpha());
}

void test_lowpass_no_filtering_alpha_1() {
  LowPass1P lp;
  lp.setAlpha(1.0f);
  lp.reset(0.0f, true);

  lp.update(10.0f);
  TEST_ASSERT_EQUAL_FLOAT(10.0f, lp.value());

  lp.update(5.0f);
  TEST_ASSERT_EQUAL_FLOAT(5.0f, lp.value());
}

void test_lowpass_heavy_filtering_alpha_small() {
  LowPass1P lp;
  lp.setAlpha(0.1f);
  lp.reset(0.0f, true);

  lp.update(10.0f);
  TEST_ASSERT_TRUE(floatEqual(1.0f, lp.value()));  // 0 + 0.1*(10-0) = 1.0
}

void test_lowpass_configure_tau_dt_fast() {
  LowPass1P lp;
  float tau = 0.1f;
  float dt = 0.01f;
  lp.configureTauDtFast(tau, dt);

  float expected_alpha = dt / (tau + dt);  // 0.01 / 0.11 ≈ 0.0909
  TEST_ASSERT_TRUE(floatEqual(expected_alpha, lp.alpha(), 0.01f));
}

void test_lowpass_configure_tau_dt_exact() {
  LowPass1P lp;
  float tau = 0.1f;
  float dt = 0.01f;
  lp.configureTauDtExact(tau, dt);

  float expected_alpha = 1.0f - std::exp(-dt / tau);
  TEST_ASSERT_TRUE(floatEqual(expected_alpha, lp.alpha(), 0.01f));
}

void test_lowpass_configure_cutoff_hz_fast() {
  LowPass1P lp;
  float cutoff_hz = 10.0f;
  float dt = 0.01f;
  lp.configureCutoffHzFast(cutoff_hz, dt);

  // tau = 1 / (2*pi*fc)
  float tau = 1.0f / (2.0f * kPi * cutoff_hz);
  float expected_alpha = dt / (tau + dt);
  TEST_ASSERT_TRUE(floatEqual(expected_alpha, lp.alpha(), 0.01f));
}

void test_lowpass_configure_cutoff_hz_exact() {
  LowPass1P lp;
  float cutoff_hz = 10.0f;
  float dt = 0.01f;
  lp.configureCutoffHzExact(cutoff_hz, dt);

  float tau = 1.0f / (2.0f * kPi * cutoff_hz);
  float expected_alpha = 1.0f - std::exp(-dt / tau);
  TEST_ASSERT_TRUE(floatEqual(expected_alpha, lp.alpha(), 0.01f));
}

void test_lowpass_smoothing_constant_input() {
  LowPass1P lp;
  lp.setAlpha(0.5f);
  lp.reset(0.0f, true);

  // Repeated constant input should converge
  for (int i = 0; i < 20; i++) {
    lp.update(10.0f);
  }

  TEST_ASSERT_TRUE(floatEqual(10.0f, lp.value(), 0.01f));
}

void test_lowpass_step_response() {
  LowPass1P lp;
  lp.setAlpha(0.5f);
  lp.reset(0.0f, true);

  float y1 = lp.update(10.0f);
  TEST_ASSERT_TRUE(floatEqual(5.0f, y1));  // 0 + 0.5*(10-0)

  float y2 = lp.update(10.0f);
  TEST_ASSERT_TRUE(floatEqual(7.5f, y2));  // 5 + 0.5*(10-5)
}

void test_lowpass_zero_tau_fallback() {
  LowPass1P lp;
  lp.configureTauDtFast(0.0f, 0.01f);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, lp.alpha());  // Should fall back to no filtering
}

void test_lowpass_negative_cutoff_fallback() {
  LowPass1P lp;
  lp.configureCutoffHzFast(-10.0f, 0.01f);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, lp.alpha());
}

// ===== Integration Tests =====

void test_moving_avg_then_median() {
  MovingAverage<3> ma;
  ma.update(1.0f);
  ma.update(2.0f);
  ma.update(3.0f);

  float spike = 100.0f;
  float normal = 3.5f;

  float filtered = median3(ma.value(), spike, normal);
  TEST_ASSERT_TRUE(floatEqual(3.0f, filtered, 0.5f));
}

void test_median_then_lowpass() {
  LowPass1P lp;
  lp.setAlpha(0.3f);

  // Spike rejection then smoothing
  float raw1 = 10.0f;
  float raw2 = 100.0f;  // spike
  float raw3 = 12.0f;

  float m = median3(raw1, raw2, raw3);  // Should be 12.0
  lp.update(m);

  TEST_ASSERT_TRUE(floatEqual(12.0f, lp.value(), 2.0f));
}

int main(int argc, char **argv) {
  UNITY_BEGIN();

  // median3 tests
  RUN_TEST(test_median3_ordered);
  RUN_TEST(test_median3_reverse_ordered);
  RUN_TEST(test_median3_middle_first);
  RUN_TEST(test_median3_middle_last);
  RUN_TEST(test_median3_all_equal);
  RUN_TEST(test_median3_two_equal_low);
  RUN_TEST(test_median3_two_equal_high);
  RUN_TEST(test_median3_negative_values);
  RUN_TEST(test_median3_spike_rejection);

  // MovingAverage tests
  RUN_TEST(test_moving_avg_initial_value);
  RUN_TEST(test_moving_avg_reset);
  RUN_TEST(test_moving_avg_single_sample);
  RUN_TEST(test_moving_avg_filling);
  RUN_TEST(test_moving_avg_count);
  RUN_TEST(test_moving_avg_steady_state);
  RUN_TEST(test_moving_avg_constant_input);
  RUN_TEST(test_moving_avg_alternating);
  RUN_TEST(test_moving_avg_negative_values);

  // LowPass1P tests
  RUN_TEST(test_lowpass_initial_value);
  RUN_TEST(test_lowpass_reset);
  RUN_TEST(test_lowpass_first_sample_initialization);
  RUN_TEST(test_lowpass_alpha_direct);
  RUN_TEST(test_lowpass_alpha_clamp_low);
  RUN_TEST(test_lowpass_alpha_clamp_high);
  RUN_TEST(test_lowpass_no_filtering_alpha_1);
  RUN_TEST(test_lowpass_heavy_filtering_alpha_small);
  RUN_TEST(test_lowpass_configure_tau_dt_fast);
  RUN_TEST(test_lowpass_configure_tau_dt_exact);
  RUN_TEST(test_lowpass_configure_cutoff_hz_fast);
  RUN_TEST(test_lowpass_configure_cutoff_hz_exact);
  RUN_TEST(test_lowpass_smoothing_constant_input);
  RUN_TEST(test_lowpass_step_response);
  RUN_TEST(test_lowpass_zero_tau_fallback);
  RUN_TEST(test_lowpass_negative_cutoff_fallback);

  // Integration tests
  RUN_TEST(test_moving_avg_then_median);
  RUN_TEST(test_median_then_lowpass);

  return UNITY_END();
}
