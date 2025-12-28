// Copyright 2025 SEC26 Team
// UWB Beacon Configuration Classes

#ifndef SECBOT_UWB__BEACON_CONFIG_HPP_
#define SECBOT_UWB__BEACON_CONFIG_HPP_

#include <string>
#include <vector>
#include <array>

namespace secbot_uwb
{

enum class BeaconType
{
  STATIONARY,
  MOVING
};

class BeaconConfig
{
public:
  BeaconConfig(
    int beacon_id,
    BeaconType type,
    const std::array<double, 3> & position,
    const std::vector<std::string> & known_axes,
    const std::string & odometry_topic = "",
    bool use_odometry_fusion = false,
    const std::string & description = "");

  int getBeaconId() const { return beacon_id_; }
  BeaconType getType() const { return type_; }
  const std::array<double, 3> & getPosition() const { return position_; }
  const std::vector<std::string> & getKnownAxes() const { return known_axes_; }
  const std::string & getOdometryTopic() const { return odometry_topic_; }
  bool useOdometryFusion() const { return use_odometry_fusion_; }
  const std::string & getDescription() const { return description_; }

  bool isAxisKnown(const std::string & axis) const;

private:
  int beacon_id_;
  BeaconType type_;
  std::array<double, 3> position_;
  std::vector<std::string> known_axes_;
  std::string odometry_topic_;
  bool use_odometry_fusion_;
  std::string description_;
};

class TagConfig
{
public:
  TagConfig(
    int tag_id,
    const std::string & description,
    const std::string & publish_topic,
    bool enable_3d_positioning = false);

  int getTagId() const { return tag_id_; }
  const std::string & getDescription() const { return description_; }
  const std::string & getPublishTopic() const { return publish_topic_; }
  bool enable3D() const { return enable_3d_positioning_; }

private:
  int tag_id_;
  std::string description_;
  std::string publish_topic_;
  bool enable_3d_positioning_;
};

}  // namespace secbot_uwb

#endif  // SECBOT_UWB__BEACON_CONFIG_HPP_
