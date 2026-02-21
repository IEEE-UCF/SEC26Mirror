#include <BaseSubsystem.h>
#include <Pose2D.h>
#include <Vector2D.h>
#include <micro_ros_utilities/type_utilities.h>
#include <microros_manager_robot.h>
#include <traj_controller.h>

#include "TimedSubsystem.h"
#include "src/robot/drive-base/RobotDriveBase.h"
namespace Subsystem {

class MiniBotDriveSusbsystemSetup : Classes::BaseSetup {
 public:
  MiniBotDriveBaseSetup mini_bot_drivebase_setup_;

  MiniBotDriveSusbsystemSetup(const char* _id,
                              const MiniBotDriveBaseSetup& setup)
      : Classes::BaseSetup(_id), mini_bot_drivebase_setup_(setup) {}

}

class MiniBotDriveSubsystem : public IMicroRosParticipant,
                              public Subsystem::TimedSubsystem {
 public:
  explicit MiniBotDriveSubsystem(const MiniBotDriveSusbsystemSetup& setup)
      : Subsystem::TimeSubsystem(setup), setup_(setup) {}

  void update() override;
  void begin() override;
  void pause() override;
  void reset() override;
  const char* getInfo() override;

  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override;
  void onDestroy() override;

 private:
  const MiniBotDriveSusbsystemSetup setup_;

  rcl_node_t* node_ = nullptr;
  rclc_executor_t* executor = nullptr;
  rcl_publisher_t mini_bot_pub_;
  rcl_subscription_t mini_bot_sub_;

  mcu_msgs_msg__MiniRobotState state_msg_;
  mcu_msgs_msg__MiniRobotControl control_msgs_;
};
}  // namespace Subsystem