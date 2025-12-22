/**
 * @file DriveSubsystem.h
 * @brief Drive subsystem
 * @author Trevor Cannon
 * @date 12/18/2025
 */

#pragma once

#include <BaseSubsystem.h>
#include <mcu_msgs/msg/drive_base.h>
#include <micro_ros_utilities/type_utilities.h>
#include <microros_manager_robot.h>

#include "TimedSubsystem.h"
#include "robot/drive-base/RobotDriveBase.h"
#include <Vector2D.h>
#include <Pose2D.h>

// #include messages

namespace Subsystem {

class DriveSubsystemSetup : public Classes::BaseSetup {
 public:
  DriveBaseSetup driveBaseSetup_;

  DriveSubsystemSetup(const char* _id, const DriveBaseSetup& _driveBaseSetup)
      : Classes::BaseSetup(_id), driveBaseSetup_(_driveBaseSetup) {}
};

class DriveSubsystem : public IMicroRosParticipant,
                       public Subsystem::TimedSubsystem {
 public:
  explicit DriveSubsystem(const DriveSubsystemSetup& setup)
      : Subsystem::TimedSubsystem(setup),
        setup_(setup),
        driveBase_(setup.driveBaseSetup_) {}

  void update() override;
  void begin() override;
  void pause() override;
  void reset() override;
  const char* getInfo() override;

  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override;
  void onDestroy() override;

  void publishData();

 private:
  static void drive_callback(const void* msvin, void* context);

  const DriveSubsystemSetup setup_;
  RobotDriveBase driveBase_;

  rcl_node_t* node_ = nullptr;
  rclc_executor_t* executor_ = nullptr;
  rcl_publisher_t drive_pub_;
  rcl_subscription_t drive_sub_;

  mcu_msgs__msg__DriveBase drive_msg_;
};
}  // namespace Subsystem
