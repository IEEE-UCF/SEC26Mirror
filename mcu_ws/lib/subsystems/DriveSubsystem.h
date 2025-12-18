/**
 * @file DriveSubsystem.h
 * @brief
 * @author Trevor Cannon
 * @date 12/18/2025
 */

#pragma once

#include <BaseSubsystem.h>
#include <microros_manager_robot.h>

#include "robot/drive-base/RobotDriveBase.h"

namespace Subsystem {

class DriveSubsystemSetup : public Classes::BaseSetup {
 public:
  DriveSubsystem(const char* _id) : Classes::BaseSetup(_id) {}
}

class DriveSubsystem : public IMicroRosParticipant,
                       public Classes::BaseSubsystem {
 public:
  explicit DriveSubsystem(const DriveSubsystemSetup& setup)
      : Classes::BaseSubsystem(setup), setup_(setup) {}

 private:
  const DriveSubsystem setup_;
}
}  // namespace Subsystem
