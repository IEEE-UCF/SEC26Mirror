// Copyright 2025 SEC26 Team
// Test BeaconConfig and TagConfig classes

#include <gtest/gtest.h>
#include "secbot_uwb/beacon_config.hpp"

using secbot_uwb::BeaconConfig;
using secbot_uwb::TagConfig;
using secbot_uwb::BeaconType;

TEST(BeaconConfigTest, StationaryBeacon) {
  std::array<double, 3> position = {1.0, 2.0, 0.15};
  std::vector<std::string> known_axes = {"x", "y", "z"};

  BeaconConfig beacon(10, BeaconType::STATIONARY, position, known_axes,
                      "", false, "Test beacon");

  EXPECT_EQ(beacon.getBeaconId(), 10);
  EXPECT_EQ(beacon.getType(), BeaconType::STATIONARY);
  EXPECT_EQ(beacon.getPosition()[0], 1.0);
  EXPECT_EQ(beacon.getPosition()[1], 2.0);
  EXPECT_EQ(beacon.getPosition()[2], 0.15);
  EXPECT_EQ(beacon.getDescription(), "Test beacon");
  EXPECT_TRUE(beacon.isAxisKnown("x"));
  EXPECT_TRUE(beacon.isAxisKnown("y"));
  EXPECT_TRUE(beacon.isAxisKnown("z"));
  EXPECT_FALSE(beacon.useOdometryFusion());
}

TEST(BeaconConfigTest, MovingBeacon) {
  std::array<double, 3> position = {0.0, 0.0, 0.20};
  std::vector<std::string> known_axes = {"z"};

  BeaconConfig beacon(12, BeaconType::MOVING, position, known_axes,
                      "/robot/pose", true, "Robot beacon");

  EXPECT_EQ(beacon.getBeaconId(), 12);
  EXPECT_EQ(beacon.getType(), BeaconType::MOVING);
  EXPECT_EQ(beacon.getPosition()[2], 0.20);
  EXPECT_EQ(beacon.getOdometryTopic(), "/robot/pose");
  EXPECT_TRUE(beacon.useOdometryFusion());
  EXPECT_TRUE(beacon.isAxisKnown("z"));
  EXPECT_FALSE(beacon.isAxisKnown("x"));
  EXPECT_FALSE(beacon.isAxisKnown("y"));
}

TEST(BeaconConfigTest, DefaultDescription) {
  std::array<double, 3> position = {0.0, 0.0, 0.0};
  std::vector<std::string> known_axes = {"x", "y", "z"};

  BeaconConfig beacon(15, BeaconType::STATIONARY, position, known_axes);

  EXPECT_EQ(beacon.getDescription(), "Beacon 15");
}

TEST(TagConfigTest, FullConfiguration) {
  TagConfig tag(20, "Drone tag", "/uwb/pose/drone", true);

  EXPECT_EQ(tag.getTagId(), 20);
  EXPECT_EQ(tag.getDescription(), "Drone tag");
  EXPECT_EQ(tag.getPublishTopic(), "/uwb/pose/drone");
  EXPECT_TRUE(tag.enable3D());
}

TEST(TagConfigTest, DefaultValues) {
  TagConfig tag(21, "", "", false);

  EXPECT_EQ(tag.getTagId(), 21);
  EXPECT_EQ(tag.getDescription(), "Tag 21");
  EXPECT_EQ(tag.getPublishTopic(), "/uwb/pose/tag_21");
  EXPECT_FALSE(tag.enable3D());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
