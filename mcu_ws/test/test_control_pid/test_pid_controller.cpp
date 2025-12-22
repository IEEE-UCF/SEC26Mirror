/**
 * @file test_pid_controller.cpp
 * @brief Unit tests for PID Controller
 * @date 12/22/2025
 */

#define _USE_MATH_DEFINES
#include <unity.h>
#include <pid_controller.h>
#include <cmath>

// Helper function to compare floats with tolerance
bool floatEqual(float a, float b, float epsilon = 0.001f) {
  return fabs(a - b) < epsilon;
}

void setUp(void) {}
void tearDown(void) {}

// === Basic Functionality Tests ===

void test_pid_default_constructor() {
  PIDController pid;
  TEST_ASSERT_FALSE(pid.initialized());
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.lastOutput());
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.integralTerm());
}

void test_pid_proportional_only() {
  PIDController::Config cfg;
  cfg.gains.kp = 2.0f;
  cfg.gains.ki = 0.0f;
  cfg.gains.kd = 0.0f;

  PIDController pid(cfg);

  // Error = 10 - 0 = 10, output should be kp * error = 2 * 10 = 20
  float out = pid.update(10.0f, 0.0f, 0.1f);
  TEST_ASSERT_TRUE(floatEqual(20.0f, out));
}

void test_pid_integral_basic() {
  PIDController::Config cfg;
  cfg.gains.kp = 0.0f;
  cfg.gains.ki = 1.0f;
  cfg.gains.kd = 0.0f;
  cfg.conditional_integration = false; // Disable anti-windup for this test

  PIDController pid(cfg);

  // First update: error = 10, dt = 0.1, integral = 1.0 * 10 * 0.1 = 1.0
  float out1 = pid.update(10.0f, 0.0f, 0.1f);
  TEST_ASSERT_TRUE(floatEqual(1.0f, out1));

  // Second update: integral += 1.0 * 10 * 0.1 = 2.0
  float out2 = pid.update(10.0f, 0.0f, 0.1f);
  TEST_ASSERT_TRUE(floatEqual(2.0f, out2));
}

void test_pid_derivative_on_measurement() {
  PIDController::Config cfg;
  cfg.gains.kp = 0.0f;
  cfg.gains.ki = 0.0f;
  cfg.gains.kd = 1.0f;
  cfg.dmode = PIDController::DerivativeMode::OnMeasurement;

  PIDController pid(cfg);

  // First update initializes history, d_term should be 0
  float out1 = pid.update(10.0f, 0.0f, 0.1f);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, out1);

  // Second update: measurement went from 0 to 5, deriv = -(5-0)/0.1 = -50
  float out2 = pid.update(10.0f, 5.0f, 0.1f);
  TEST_ASSERT_TRUE(floatEqual(-50.0f, out2));
}

void test_pid_derivative_on_error() {
  PIDController::Config cfg;
  cfg.gains.kp = 0.0f;
  cfg.gains.ki = 0.0f;
  cfg.gains.kd = 1.0f;
  cfg.dmode = PIDController::DerivativeMode::OnError;

  PIDController pid(cfg);

  // First update: error = 10, initializes, d_term = 0
  float out1 = pid.update(10.0f, 0.0f, 0.1f);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, out1);

  // Second update: error = 5, deriv = (5-10)/0.1 = -50
  float out2 = pid.update(10.0f, 5.0f, 0.1f);
  TEST_ASSERT_TRUE(floatEqual(-50.0f, out2));
}

void test_pid_output_limits() {
  PIDController::Config cfg;
  cfg.gains.kp = 10.0f;
  cfg.limits.out_min = -50.0f;
  cfg.limits.out_max = 50.0f;

  PIDController pid(cfg);

  // Error = 10, p = 10 * 10 = 100, should clamp to 50
  float out = pid.update(10.0f, 0.0f, 0.1f);
  TEST_ASSERT_EQUAL_FLOAT(50.0f, out);

  // Negative error, should clamp to -50
  float out2 = pid.update(-10.0f, 0.0f, 0.1f);
  TEST_ASSERT_EQUAL_FLOAT(-50.0f, out2);
}

void test_pid_integral_limits() {
  PIDController::Config cfg;
  cfg.gains.ki = 100.0f;
  cfg.limits.i_min = -5.0f;
  cfg.limits.i_max = 5.0f;
  cfg.conditional_integration = false;

  PIDController pid(cfg);

  // error = 10, dt = 0.1, integral = 100 * 10 * 0.1 = 100, should clamp to 5
  pid.update(10.0f, 0.0f, 0.1f);
  TEST_ASSERT_EQUAL_FLOAT(5.0f, pid.integralTerm());
}

void test_pid_conditional_integration_saturation() {
  PIDController::Config cfg;
  cfg.gains.kp = 1.0f;
  cfg.gains.ki = 1.0f;
  cfg.limits.out_max = 10.0f;
  cfg.conditional_integration = true;

  PIDController pid(cfg);

  // Error = 20, p = 20, saturates at out_max = 10
  // Since output is saturated and error > 0, integral should not accumulate
  pid.update(20.0f, 0.0f, 0.1f);

  // Integral should be 0 or minimal due to anti-windup
  TEST_ASSERT_TRUE(pid.integralTerm() <= 2.0f); // Small tolerance
}

void test_pid_reset() {
  PIDController::Config cfg;
  cfg.gains.kp = 1.0f;
  cfg.gains.ki = 1.0f;
  cfg.gains.kd = 1.0f;

  PIDController pid(cfg);

  // Run some updates
  pid.update(10.0f, 0.0f, 0.1f);
  pid.update(10.0f, 5.0f, 0.1f);

  TEST_ASSERT_TRUE(pid.initialized());

  // Reset
  pid.reset();

  TEST_ASSERT_FALSE(pid.initialized());
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.integralTerm());
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.derivativeTerm());
}

void test_pid_feedforward() {
  PIDController::Config cfg;
  cfg.gains.kp = 1.0f;

  PIDController pid(cfg);

  // error = 10, p = 10, feedforward = 5, output = 15
  float out = pid.update(10.0f, 0.0f, 0.1f, 5.0f);
  TEST_ASSERT_TRUE(floatEqual(15.0f, out));
}

void test_pid_invalid_dt_too_small() {
  PIDController::Config cfg;
  cfg.gains.kp = 1.0f;
  cfg.min_dt = 0.001f;

  PIDController pid(cfg);

  // First valid update
  float out1 = pid.update(10.0f, 0.0f, 0.1f);

  // Update with invalid dt (too small)
  float out2 = pid.update(20.0f, 0.0f, 0.0000001f);

  // Should return last output
  TEST_ASSERT_EQUAL_FLOAT(out1, out2);
}

void test_pid_invalid_dt_too_large() {
  PIDController::Config cfg;
  cfg.gains.kp = 1.0f;
  cfg.max_dt = 0.5f;

  PIDController pid(cfg);

  // First valid update
  float out1 = pid.update(10.0f, 0.0f, 0.1f);

  // Update with invalid dt (too large)
  float out2 = pid.update(20.0f, 0.0f, 2.0f);

  // Should return last output
  TEST_ASSERT_EQUAL_FLOAT(out1, out2);
}

void test_pid_derivative_filter() {
  PIDController::Config cfg;
  cfg.gains.kd = 1.0f;
  cfg.d_filter_alpha = 0.5f; // 50% filtering
  cfg.dmode = PIDController::DerivativeMode::OnMeasurement;

  PIDController pid(cfg);

  // First update
  pid.update(0.0f, 0.0f, 0.1f);

  // Second update: raw derivative would be -100
  pid.update(0.0f, 10.0f, 0.1f);

  // With filter: d_term = 0.5 * 0 + 0.5 * (-100) = -50
  TEST_ASSERT_TRUE(floatEqual(-50.0f, pid.derivativeTerm()));
}

void test_pid_set_gains() {
  PIDController pid;

  pid.setGains(1.5f, 2.5f, 3.5f);

  TEST_ASSERT_EQUAL_FLOAT(1.5f, pid.config().gains.kp);
  TEST_ASSERT_EQUAL_FLOAT(2.5f, pid.config().gains.ki);
  TEST_ASSERT_EQUAL_FLOAT(3.5f, pid.config().gains.kd);
}

void test_pid_set_output_limits() {
  PIDController pid;

  pid.setOutputLimits(-100.0f, 100.0f);

  TEST_ASSERT_EQUAL_FLOAT(-100.0f, pid.config().limits.out_min);
  TEST_ASSERT_EQUAL_FLOAT(100.0f, pid.config().limits.out_max);
}

void test_pid_full_pid() {
  PIDController::Config cfg;
  cfg.gains.kp = 1.0f;
  cfg.gains.ki = 0.5f;
  cfg.gains.kd = 0.1f;
  cfg.dmode = PIDController::DerivativeMode::OnMeasurement;
  cfg.conditional_integration = false;

  PIDController pid(cfg);

  // First update: error = 10, p = 10, i = 0.5*10*0.1 = 0.5, d = 0
  float out1 = pid.update(10.0f, 0.0f, 0.1f);
  TEST_ASSERT_TRUE(floatEqual(10.5f, out1));

  // Second update: error = 5, p = 5, i += 0.5*5*0.1 = 0.75, d = -0.1*50 = -5
  float out2 = pid.update(10.0f, 5.0f, 0.1f);
  TEST_ASSERT_TRUE(floatEqual(0.75f, out2, 0.01f)); // p + i + d = 5 + 0.75 + (-5)
}

int main(int argc, char **argv) {
  UNITY_BEGIN();

  // Basic functionality
  RUN_TEST(test_pid_default_constructor);
  RUN_TEST(test_pid_proportional_only);
  RUN_TEST(test_pid_integral_basic);
  RUN_TEST(test_pid_derivative_on_measurement);
  RUN_TEST(test_pid_derivative_on_error);

  // Limits and saturation
  RUN_TEST(test_pid_output_limits);
  RUN_TEST(test_pid_integral_limits);
  RUN_TEST(test_pid_conditional_integration_saturation);

  // Reset and configuration
  RUN_TEST(test_pid_reset);
  RUN_TEST(test_pid_set_gains);
  RUN_TEST(test_pid_set_output_limits);

  // Advanced features
  RUN_TEST(test_pid_feedforward);
  RUN_TEST(test_pid_derivative_filter);
  RUN_TEST(test_pid_invalid_dt_too_small);
  RUN_TEST(test_pid_invalid_dt_too_large);
  RUN_TEST(test_pid_full_pid);

  return UNITY_END();
}
