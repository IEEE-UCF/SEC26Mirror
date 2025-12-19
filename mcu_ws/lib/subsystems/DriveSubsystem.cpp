#include "DriveSubsystem.h"

namespace Subsystem {

void DriveSubsystem::update() {}

void DriveSubsystem::begin() {}

void DriveSubsystem::pause() {}

void DriveSubsystem::reset() {}

const char* DriveSubsystem::getInfo() {
  const char info[] = "DriveSubsystem";
  return info;
}

bool DriveSubsystem::onCreate(rcl_node_t* node, rclc_executor* executor) {}

bool DriveSubsystem::onDestroy() {}

void DriveSubsystem::publishData() {}

void DriveSubsystem::subscribeTwist() {}

}  // namespace Subsystem
