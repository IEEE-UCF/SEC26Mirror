// Copyright 2025 SEC26 Team
// Test trilateration algorithms

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>
#include <map>
#include <array>

// Simple trilateration function for testing
bool trilaterate2D(
  const std::map<int, std::array<double, 3>> & beacon_positions,
  const std::map<int, double> & ranges,
  std::array<double, 3> & position,
  double & residual)
{
  if (ranges.size() < 3) {
    return false;
  }

  // Get beacon IDs
  std::vector<int> beacon_ids;
  for (const auto & [id, dist] : ranges) {
    beacon_ids.push_back(id);
  }

  // Use first beacon as reference
  int ref_id = beacon_ids[0];
  auto ref_pos = beacon_positions.at(ref_id);
  double ref_dist = ranges.at(ref_id);

  // Build system of equations
  int n_equations = beacon_ids.size() - 1;
  Eigen::MatrixXd A(n_equations, 3);
  Eigen::VectorXd b(n_equations);

  int row = 0;
  for (size_t i = 1; i < beacon_ids.size(); ++i) {
    int beacon_id = beacon_ids[i];
    auto beacon_pos = beacon_positions.at(beacon_id);
    double dist = ranges.at(beacon_id);

    // Row of A: 2(a_ref - a_i)
    A(row, 0) = 2.0 * (ref_pos[0] - beacon_pos[0]);
    A(row, 1) = 2.0 * (ref_pos[1] - beacon_pos[1]);
    A(row, 2) = 2.0 * (ref_pos[2] - beacon_pos[2]);

    // Right-hand side
    double ref_norm_sq = ref_pos[0] * ref_pos[0] + ref_pos[1] * ref_pos[1] +
      ref_pos[2] * ref_pos[2];
    double beacon_norm_sq = beacon_pos[0] * beacon_pos[0] + beacon_pos[1] * beacon_pos[1] +
      beacon_pos[2] * beacon_pos[2];

    b(row) = ref_norm_sq - beacon_norm_sq + dist * dist - ref_dist * ref_dist;

    ++row;
  }

  // Solve using least squares
  Eigen::Vector3d solution = A.colPivHouseholderQr().solve(b);

  position[0] = solution(0);
  position[1] = solution(1);
  position[2] = solution(2);

  // Calculate residual
  double error_sum = 0.0;
  for (const auto & beacon_id : beacon_ids) {
    auto beacon_pos = beacon_positions.at(beacon_id);
    double dx = position[0] - beacon_pos[0];
    double dy = position[1] - beacon_pos[1];
    double dz = position[2] - beacon_pos[2];
    double predicted_dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    double error = predicted_dist - ranges.at(beacon_id);
    error_sum += error * error;
  }
  residual = std::sqrt(error_sum / beacon_ids.size());

  return true;
}

TEST(TrilaterationTest, PerfectMeasurements) {
  // Set up beacons in a triangle
  std::map<int, std::array<double, 3>> beacons;
  beacons[10] = {0.0, 0.0, 0.0};
  beacons[11] = {3.0, 0.0, 0.0};
  beacons[12] = {0.0, 3.0, 0.0};

  // Tag at (1, 1, 0)
  std::array<double, 3> true_position = {1.0, 1.0, 0.0};

  // Calculate perfect ranges
  std::map<int, double> ranges;
  for (const auto & [id, pos] : beacons) {
    double dx = true_position[0] - pos[0];
    double dy = true_position[1] - pos[1];
    double dz = true_position[2] - pos[2];
    ranges[id] = std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  // Run trilateration
  std::array<double, 3> estimated_pos;
  double residual;
  bool success = trilaterate2D(beacons, ranges, estimated_pos, residual);

  EXPECT_TRUE(success);
  EXPECT_NEAR(estimated_pos[0], true_position[0], 1e-6);
  EXPECT_NEAR(estimated_pos[1], true_position[1], 1e-6);
  EXPECT_NEAR(estimated_pos[2], true_position[2], 1e-6);
  EXPECT_LT(residual, 1e-6);
}

TEST(TrilaterationTest, FourBeacons) {
  // Overdetermined system with 4 beacons
  std::map<int, std::array<double, 3>> beacons;
  beacons[10] = {0.0, 0.0, 0.0};
  beacons[11] = {4.0, 0.0, 0.0};
  beacons[12] = {0.0, 4.0, 0.0};
  beacons[13] = {4.0, 4.0, 0.0};

  std::array<double, 3> true_position = {2.0, 2.0, 0.0};

  std::map<int, double> ranges;
  for (const auto & [id, pos] : beacons) {
    double dx = true_position[0] - pos[0];
    double dy = true_position[1] - pos[1];
    double dz = true_position[2] - pos[2];
    ranges[id] = std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  std::array<double, 3> estimated_pos;
  double residual;
  bool success = trilaterate2D(beacons, ranges, estimated_pos, residual);

  EXPECT_TRUE(success);
  EXPECT_NEAR(estimated_pos[0], true_position[0], 1e-6);
  EXPECT_NEAR(estimated_pos[1], true_position[1], 1e-6);
  EXPECT_NEAR(estimated_pos[2], true_position[2], 1e-6);
}

TEST(TrilaterationTest, InsufficientBeacons) {
  std::map<int, std::array<double, 3>> beacons;
  beacons[10] = {0.0, 0.0, 0.0};
  beacons[11] = {3.0, 0.0, 0.0};

  std::map<int, double> ranges;
  ranges[10] = 1.0;
  ranges[11] = 2.0;

  std::array<double, 3> estimated_pos;
  double residual;
  bool success = trilaterate2D(beacons, ranges, estimated_pos, residual);

  EXPECT_FALSE(success);
}

TEST(DistanceTest, EuclideanDistance2D) {
  std::array<double, 3> p1 = {0.0, 0.0, 0.0};
  std::array<double, 3> p2 = {3.0, 4.0, 0.0};

  double dx = p2[0] - p1[0];
  double dy = p2[1] - p1[1];
  double dz = p2[2] - p1[2];
  double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

  EXPECT_NEAR(distance, 5.0, 1e-6);
}

TEST(DistanceTest, EuclideanDistance3D) {
  std::array<double, 3> p1 = {0.0, 0.0, 0.0};
  std::array<double, 3> p2 = {1.0, 1.0, 1.0};

  double dx = p2[0] - p1[0];
  double dy = p2[1] - p1[1];
  double dz = p2[2] - p1[2];
  double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

  EXPECT_NEAR(distance, std::sqrt(3.0), 1e-6);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
