// Copyright 2025 SEC26 Team
// UWB Beacon Configuration Implementation

#include "secbot_uwb/beacon_config.hpp"

#include <algorithm>

namespace secbot_uwb {

BeaconConfig::BeaconConfig(int beacon_id, BeaconType type,
                           const std::array<double, 3>& position,
                           const std::vector<std::string>& known_axes,
                           const std::string& odometry_topic,
                           bool use_odometry_fusion,
                           const std::string& description)
    : beacon_id_(beacon_id),
      type_(type),
      position_(position),
      known_axes_(known_axes),
      odometry_topic_(odometry_topic),
      use_odometry_fusion_(use_odometry_fusion),
      description_(description.empty() ? "Beacon " + std::to_string(beacon_id)
                                       : description) {}

bool BeaconConfig::isAxisKnown(const std::string& axis) const {
  return std::find(known_axes_.begin(), known_axes_.end(), axis) !=
         known_axes_.end();
}

TagConfig::TagConfig(int tag_id, const std::string& description,
                     const std::string& publish_topic,
                     bool enable_3d_positioning)
    : tag_id_(tag_id),
      description_(description.empty() ? "Tag " + std::to_string(tag_id)
                                       : description),
      publish_topic_(publish_topic.empty()
                         ? "/uwb/pose/tag_" + std::to_string(tag_id)
                         : publish_topic),
      enable_3d_positioning_(enable_3d_positioning) {}

}  // namespace secbot_uwb
