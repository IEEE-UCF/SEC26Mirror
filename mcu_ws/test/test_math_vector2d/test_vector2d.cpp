/**
 * @file test_vector2d.cpp
 * @brief Unit tests for Vector2D class
 * @date 12/21/2025
 */

#include <unity.h>
#include <Vector2D.h>
#include <cmath>

// Helper function to compare floats with tolerance
bool floatEqual(float a, float b, float epsilon = 0.0001f) {
  return fabs(a - b) <= epsilon;
}

void setUp(void) {
  // Set up code here, runs before each test
}

void tearDown(void) {
  // Clean up code here, runs after each test
}

// Test: Default constructor
void test_vector2d_default_constructor() {
  Vector2D vec;
  TEST_ASSERT_EQUAL_FLOAT(0.0f, vec.x);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, vec.y);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, vec.theta);
}

// Test: Parameterized constructor
void test_vector2d_parameterized_constructor() {
  Vector2D vec(3.0f, 4.0f, 0.5f);
  TEST_ASSERT_EQUAL_FLOAT(3.0f, vec.x);
  TEST_ASSERT_EQUAL_FLOAT(4.0f, vec.y);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, vec.theta);
}

// Test: Getters
void test_vector2d_getters() {
  Vector2D vec(1.5f, 2.5f, 0.75f);
  TEST_ASSERT_EQUAL_FLOAT(1.5f, vec.getX());
  TEST_ASSERT_EQUAL_FLOAT(2.5f, vec.getY());
  TEST_ASSERT_EQUAL_FLOAT(0.75f, vec.getTheta());
}

// Test: magnitude basic
void test_vector2d_magnitude_basic() {
  Vector2D vec(3.0f, 4.0f, 0.0f);
  TEST_ASSERT_EQUAL_FLOAT(5.0f, vec.magnitude());
}

// Test: magnitude with zero vector
void test_vector2d_magnitude_zero() {
  Vector2D vec(0.0f, 0.0f, 0.0f);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, vec.magnitude());
}

// Test: magnitude with negative components
void test_vector2d_magnitude_negative() {
  Vector2D vec(-3.0f, -4.0f, 0.0f);
  TEST_ASSERT_EQUAL_FLOAT(5.0f, vec.magnitude());
}

// Test: magnitude unit vector
void test_vector2d_magnitude_unit() {
  Vector2D vec(1.0f, 0.0f, 0.0f);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, vec.magnitude());
}

// Test: add method basic
void test_vector2d_add_basic() {
  Vector2D vec1(1.0f, 2.0f, 0.5f);
  Vector2D vec2(3.0f, 4.0f, 0.3f);
  vec1.add(vec2);
  TEST_ASSERT_EQUAL_FLOAT(4.0f, vec1.x);
  TEST_ASSERT_EQUAL_FLOAT(6.0f, vec1.y);
  TEST_ASSERT_EQUAL_FLOAT(0.8f, vec1.theta);
}

// Test: add method with negative values
void test_vector2d_add_negative() {
  Vector2D vec1(5.0f, 10.0f, 1.0f);
  Vector2D vec2(-3.0f, -7.0f, -0.5f);
  vec1.add(vec2);
  TEST_ASSERT_EQUAL_FLOAT(2.0f, vec1.x);
  TEST_ASSERT_EQUAL_FLOAT(3.0f, vec1.y);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, vec1.theta);
}

// Test: add method with zeros
void test_vector2d_add_zeros() {
  Vector2D vec1(1.0f, 2.0f, 3.0f);
  Vector2D vec2(0.0f, 0.0f, 0.0f);
  vec1.add(vec2);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, vec1.x);
  TEST_ASSERT_EQUAL_FLOAT(2.0f, vec1.y);
  TEST_ASSERT_EQUAL_FLOAT(3.0f, vec1.theta);
}

// Test: subtract method basic
void test_vector2d_subtract_basic() {
  Vector2D vec1(5.0f, 8.0f, 1.0f);
  Vector2D vec2(2.0f, 3.0f, 0.4f);
  vec1.subtract(vec2);
  TEST_ASSERT_EQUAL_FLOAT(3.0f, vec1.x);
  TEST_ASSERT_EQUAL_FLOAT(5.0f, vec1.y);
  TEST_ASSERT_EQUAL_FLOAT(0.6f, vec1.theta);
}

// Test: subtract method resulting in negative
void test_vector2d_subtract_negative() {
  Vector2D vec1(2.0f, 3.0f, 0.5f);
  Vector2D vec2(5.0f, 8.0f, 1.0f);
  vec1.subtract(vec2);
  TEST_ASSERT_EQUAL_FLOAT(-3.0f, vec1.x);
  TEST_ASSERT_EQUAL_FLOAT(-5.0f, vec1.y);
  TEST_ASSERT_EQUAL_FLOAT(-0.5f, vec1.theta);
}

// Test: subtract method with zeros
void test_vector2d_subtract_zeros() {
  Vector2D vec1(1.0f, 2.0f, 3.0f);
  Vector2D vec2(0.0f, 0.0f, 0.0f);
  vec1.subtract(vec2);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, vec1.x);
  TEST_ASSERT_EQUAL_FLOAT(2.0f, vec1.y);
  TEST_ASSERT_EQUAL_FLOAT(3.0f, vec1.theta);
}

// Test: rotate method 90 degrees
void test_vector2d_rotate_90_degrees() {
  Vector2D vec(1.0f, 0.0f, 0.0f);
  vec.rotate(M_PI / 2.0f);
  TEST_ASSERT_TRUE(floatEqual(vec.x, 0.0f, 0.001f));
  TEST_ASSERT_TRUE(floatEqual(vec.y, 1.0f, 0.001f));
}

// Test: rotate method 180 degrees
void test_vector2d_rotate_180_degrees() {
  Vector2D vec(1.0f, 0.0f, 0.0f);
  vec.rotate(M_PI);
  TEST_ASSERT_TRUE(floatEqual(vec.x, -1.0f, 0.001f));
  TEST_ASSERT_TRUE(floatEqual(vec.y, 0.0f, 0.001f));
}

// Test: rotate method 360 degrees
void test_vector2d_rotate_360_degrees() {
  Vector2D vec(1.0f, 0.0f, 0.0f);
  vec.rotate(2.0f * M_PI);
  TEST_ASSERT_TRUE(floatEqual(vec.x, 1.0f, 0.001f));
  TEST_ASSERT_TRUE(floatEqual(vec.y, 0.0f, 0.001f));
}

// Test: rotate method negative angle
void test_vector2d_rotate_negative() {
  Vector2D vec(1.0f, 0.0f, 0.0f);
  vec.rotate(-M_PI / 2.0f);
  TEST_ASSERT_TRUE(floatEqual(vec.x, 0.0f, 0.001f));
  TEST_ASSERT_TRUE(floatEqual(vec.y, -1.0f, 0.001f));
}

// Test: rotate method zero angle
void test_vector2d_rotate_zero() {
  Vector2D vec(1.0f, 2.0f, 0.0f);
  vec.rotate(0.0f);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, vec.x);
  TEST_ASSERT_EQUAL_FLOAT(2.0f, vec.y);
}

// Test: scalarMult method basic
void test_vector2d_scalar_mult_basic() {
  Vector2D vec(2.0f, 3.0f, 0.5f);
  vec.scalarMult(2.0f);
  TEST_ASSERT_EQUAL_FLOAT(4.0f, vec.x);
  TEST_ASSERT_EQUAL_FLOAT(6.0f, vec.y);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, vec.theta);
}

// Test: scalarMult method by zero (division by zero scenario)
void test_vector2d_scalar_mult_zero() {
  Vector2D vec(2.0f, 3.0f, 0.5f);
  vec.scalarMult(0.0f);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, vec.x);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, vec.y);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, vec.theta);
}

// Test: scalarMult method by negative value
void test_vector2d_scalar_mult_negative() {
  Vector2D vec(2.0f, 3.0f, 0.5f);
  vec.scalarMult(-1.0f);
  TEST_ASSERT_EQUAL_FLOAT(-2.0f, vec.x);
  TEST_ASSERT_EQUAL_FLOAT(-3.0f, vec.y);
  TEST_ASSERT_EQUAL_FLOAT(-0.5f, vec.theta);
}

// Test: scalarMult method by fractional value
void test_vector2d_scalar_mult_fractional() {
  Vector2D vec(4.0f, 6.0f, 2.0f);
  vec.scalarMult(0.5f);
  TEST_ASSERT_EQUAL_FLOAT(2.0f, vec.x);
  TEST_ASSERT_EQUAL_FLOAT(3.0f, vec.y);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, vec.theta);
}

// Test: scalarMult method by one
void test_vector2d_scalar_mult_one() {
  Vector2D vec(2.0f, 3.0f, 0.5f);
  vec.scalarMult(1.0f);
  TEST_ASSERT_EQUAL_FLOAT(2.0f, vec.x);
  TEST_ASSERT_EQUAL_FLOAT(3.0f, vec.y);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, vec.theta);
}

// Test: elementMult method basic
void test_vector2d_element_mult_basic() {
  Vector2D vec1(2.0f, 3.0f, 0.5f);
  Vector2D vec2(4.0f, 5.0f, 2.0f);
  vec1.elementMult(vec2);
  TEST_ASSERT_EQUAL_FLOAT(8.0f, vec1.x);
  TEST_ASSERT_EQUAL_FLOAT(15.0f, vec1.y);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, vec1.theta);
}

// Test: elementMult method with zero (division by zero scenario)
void test_vector2d_element_mult_zero() {
  Vector2D vec1(2.0f, 3.0f, 0.5f);
  Vector2D vec2(0.0f, 0.0f, 0.0f);
  vec1.elementMult(vec2);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, vec1.x);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, vec1.y);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, vec1.theta);
}

// Test: elementMult method with negative values
void test_vector2d_element_mult_negative() {
  Vector2D vec1(2.0f, 3.0f, 0.5f);
  Vector2D vec2(-1.0f, -2.0f, -1.0f);
  vec1.elementMult(vec2);
  TEST_ASSERT_EQUAL_FLOAT(-2.0f, vec1.x);
  TEST_ASSERT_EQUAL_FLOAT(-6.0f, vec1.y);
  TEST_ASSERT_EQUAL_FLOAT(-0.5f, vec1.theta);
}

// Test: elementMult method with ones
void test_vector2d_element_mult_ones() {
  Vector2D vec1(2.0f, 3.0f, 0.5f);
  Vector2D vec2(1.0f, 1.0f, 1.0f);
  vec1.elementMult(vec2);
  TEST_ASSERT_EQUAL_FLOAT(2.0f, vec1.x);
  TEST_ASSERT_EQUAL_FLOAT(3.0f, vec1.y);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, vec1.theta);
}

// Test: constrainTheta method basic
void test_vector2d_constrain_theta_basic() {
  Vector2D vec(0.0f, 0.0f, 1.5f);
  vec.constrainTheta(1.0f);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, vec.theta);
}

// Test: constrainTheta method negative
void test_vector2d_constrain_theta_negative() {
  Vector2D vec(0.0f, 0.0f, -1.5f);
  vec.constrainTheta(1.0f);
  TEST_ASSERT_EQUAL_FLOAT(-1.0f, vec.theta);
}

// Test: constrainTheta method within bounds
void test_vector2d_constrain_theta_within_bounds() {
  Vector2D vec(0.0f, 0.0f, 0.5f);
  vec.constrainTheta(1.0f);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, vec.theta);
}

// Test: constrainTheta method at boundary
void test_vector2d_constrain_theta_at_boundary() {
  Vector2D vec(0.0f, 0.0f, 1.0f);
  vec.constrainTheta(1.0f);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, vec.theta);
}

// Test: constrainTheta method with zero constraint
void test_vector2d_constrain_theta_zero_constraint() {
  Vector2D vec(0.0f, 0.0f, 0.5f);
  vec.constrainTheta(0.0f);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, vec.theta);
}

// Test: Combined operations
void test_vector2d_combined_operations() {
  Vector2D vec(1.0f, 0.0f, 0.0f);
  vec.scalarMult(2.0f);         // (2, 0, 0)
  vec.rotate(M_PI / 2.0f);      // (0, 2, 0)
  Vector2D other(1.0f, 1.0f, 0.5f);
  vec.add(other);               // (1, 3, 0.5)
  vec.constrainTheta(0.25f);    // (1, 3, 0.25)

  TEST_ASSERT_TRUE(floatEqual(vec.x, 1.0f, 0.001f));
  TEST_ASSERT_TRUE(floatEqual(vec.y, 3.0f, 0.001f));
  TEST_ASSERT_EQUAL_FLOAT(0.25f, vec.theta);
}

int main(int argc, char **argv) {
  UNITY_BEGIN();

  RUN_TEST(test_vector2d_default_constructor);
  RUN_TEST(test_vector2d_parameterized_constructor);
  RUN_TEST(test_vector2d_getters);
  RUN_TEST(test_vector2d_magnitude_basic);
  RUN_TEST(test_vector2d_magnitude_zero);
  RUN_TEST(test_vector2d_magnitude_negative);
  RUN_TEST(test_vector2d_magnitude_unit);
  RUN_TEST(test_vector2d_add_basic);
  RUN_TEST(test_vector2d_add_negative);
  RUN_TEST(test_vector2d_add_zeros);
  RUN_TEST(test_vector2d_subtract_basic);
  RUN_TEST(test_vector2d_subtract_negative);
  RUN_TEST(test_vector2d_subtract_zeros);
  RUN_TEST(test_vector2d_rotate_90_degrees);
  RUN_TEST(test_vector2d_rotate_180_degrees);
  RUN_TEST(test_vector2d_rotate_360_degrees);
  RUN_TEST(test_vector2d_rotate_negative);
  RUN_TEST(test_vector2d_rotate_zero);
  RUN_TEST(test_vector2d_scalar_mult_basic);
  RUN_TEST(test_vector2d_scalar_mult_zero);
  RUN_TEST(test_vector2d_scalar_mult_negative);
  RUN_TEST(test_vector2d_scalar_mult_fractional);
  RUN_TEST(test_vector2d_scalar_mult_one);
  RUN_TEST(test_vector2d_element_mult_basic);
  RUN_TEST(test_vector2d_element_mult_zero);
  RUN_TEST(test_vector2d_element_mult_negative);
  RUN_TEST(test_vector2d_element_mult_ones);
  RUN_TEST(test_vector2d_constrain_theta_basic);
  RUN_TEST(test_vector2d_constrain_theta_negative);
  RUN_TEST(test_vector2d_constrain_theta_within_bounds);
  RUN_TEST(test_vector2d_constrain_theta_at_boundary);
  RUN_TEST(test_vector2d_constrain_theta_zero_constraint);
  RUN_TEST(test_vector2d_combined_operations);

  return UNITY_END();
}
