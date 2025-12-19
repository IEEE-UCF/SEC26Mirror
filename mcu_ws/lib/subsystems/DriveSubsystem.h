/**
 * @file DriveSubsystem.h
 * @brief Drive subsystem
 * @author Trevor Cannon
 * @date 12/18/2025
 */

#pragma once

#include <BaseSubsystem.h>
#include <microros_manager_robot.h>

#include "robot/drive-base/RobotDriveBase.h"

// #include messages

namespace Subsystem {

class DriveSubsystemSetup : public Classes::BaseSetup {
 public:
  DriveBaseSetup driveBaseSetup_;

  DriveSubsystemSetup(const char* _id, const DriveBaseSetup& _driveBaseSetup)
      : Classes::BaseSetup(_id), driveBaseSetup_(_driveBaseSetup) {}
}

class DriveSubsystem : public IMicroRosParticipant,
                       public Classes::BaseSubsystem {
 public:
  explicit DriveSubsystem(const DriveSubsystemSetup& setup)
      : Classes::BaseSubsystem(setup),
        setup_(setup),
        driveBase_(setup.driveBaseSetup_) {}

  void update() override;
  void begin() override;
  void pause() override;
  void reset() override;
  const char* getInfo() override;

  bool onCreate(rcl_node_t* node, rclc_executor* executor) override;
  bool onDestroy() override;

  void publishData();
  void subscribeTwist();
  void subscribePose();

 private:
  const DriveSubsystem setup_;
  const RobotDriveBase driveBase_;

  rcl_node_t* node_ = nullptr;
  rcl_subscription_t twist_sub;
  rcl_publisher_t pub_;

  // messages here?
}
}  // namespace Subsystem
