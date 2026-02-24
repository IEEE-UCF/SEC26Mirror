#pragma once

#include <BaseSubsystem.h>
#include <microros_manager_robot.h>

#ifdef USE_TEENSYTHREADS
#include <TeensyThreads.h>
#endif

#include "Pose2D.h"
#include "TimedSubsystem.h"
#include "mcu_msgs/msg/mini_robot_state.h"

namespace Subsystem {

/**
 * @brief Mission states for the mini robot
 *        Runs entirely on the Teensy, no ROS dependency!
 */
enum class MiniRobotMissionState : uint8_t {
  IDLE = 0,               // No active mission, waiting
  DRIVING_TO_TARGET = 1,  // Actively driving toward target position
  AT_TARGET = 2,          // Reached target position
  RETURNING = 3,          // Driving back to home position
  ERROR = 4               // Comms failure, timeout, or other fault
};

/**
 * @brief Communication status with the ESP32
 */
enum class CommsStatus : uint8_t {
  DISCONNECTED = 0,
  CONNECTED = 1,
  TIMEOUT = 2,
  ERROR = 3
};

/**
 * @brief Commands that can be issued to the mini robot
 */
enum class MiniRobotCommand : uint8_t {
  NONE = 0,
  START_MISSION = 1,
  STOP = 2,
  RETURN_HOME = 3
};

/**
 * @brief Setup configuration for MiniRobotSubsystem
 */
class MiniRobotSubsystemSetup : public Classes::BaseSetup {
 public:
  MiniRobotSubsystemSetup(const char* _id, uint32_t comms_timeout_ms = 1000,
                          uint32_t mission_timeout_ms = 30000,
                          float arrival_threshold_m = 0.15f,
                          uint8_t esp32_i2c_addr = 0x42)
      : Classes::BaseSetup(_id),
        comms_timeout_ms_(comms_timeout_ms),
        mission_timeout_ms_(mission_timeout_ms),
        arrival_threshold_m_(arrival_threshold_m),
        esp32_i2c_addr_(esp32_i2c_addr) {}

  uint32_t comms_timeout_ms_;    // Time before declaring comms failure
  uint32_t mission_timeout_ms_;  // Max time for a mission before timeout
  float arrival_threshold_m_;    // Distance threshold to consider "arrived"
<<<<<<< HEAD
  uint8_t esp32_i2c_addr_;       // I2C address of the robotcomms ESP32
=======
  uint8_t esp32_i2c_addr_;       // I2C address of the companion ESP32
>>>>>>> 1efe8ca348ef17c6d21aa1f5e5f0f24367ae9fe0
};

/**
 * @brief Mini Robot Subsystem: manages communication and mission control
 *
 * This subsystem runs a fully autonomous state machine on the Teensy
 * ROS is only used for optional command input and state publishing
 * The mini robot will timeout and protect itself whether or not ROS is
 * connected.
 */
class MiniRobotSubsystem : public IMicroRosParticipant,
                           public Subsystem::TimedSubsystem {
 public:
  explicit MiniRobotSubsystem(const MiniRobotSubsystemSetup& setup)
      : Subsystem::TimedSubsystem(setup), setup_(setup) {}

  // Lifecycle Hooks (BaseSubsystem interface)
  bool init() override;
  void begin() override;
  void update() override;
  void pause() override;
  void reset() override;
  const char* getInfo() override;

  // Micro-ROS Hooks (IMicroRosParticipant interface)
  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override;
  void onDestroy() override;

  // Public Commands
  void startMission(float target_x, float target_y);
  void startMission(const Pose2D& target);
  void stop();
  void returnHome();

  // External Data Input
  // Called by UWB subsystem or other source to update position
  void updatePosition(float x, float y);
  void updatePosition(const Pose2D& pos);

  // State Queries
  MiniRobotMissionState getMissionState() const { return mission_state_; }
  CommsStatus getCommsStatus() const { return comms_status_; }
  bool isIdle() const { return mission_state_ == MiniRobotMissionState::IDLE; }
  bool isDriving() const {
    return mission_state_ == MiniRobotMissionState::DRIVING_TO_TARGET;
  }
  bool isAtTarget() const {
    return mission_state_ == MiniRobotMissionState::AT_TARGET;
  }
  bool isReturning() const {
    return mission_state_ == MiniRobotMissionState::RETURNING;
  }
  bool hasError() const {
    return mission_state_ == MiniRobotMissionState::ERROR;
  }
  bool isConnected() const { return comms_status_ == CommsStatus::CONNECTED; }

  // Position Queries
  Pose2D getCurrentPosition() const { return current_position_; }
  Pose2D getTargetPosition() const { return target_position_; }
  float getDistanceToTarget() const;

#ifdef USE_TEENSYTHREADS
  void beginThreaded(uint32_t stackSize, int /*priority*/ = 1,
                     uint32_t updateRateMs = 100) {
    task_delay_ms_ = updateRateMs;
    threads.addThread(taskFunction, this, stackSize);
  }

 private:
  static void taskFunction(void* pvParams) {
    auto* self = static_cast<MiniRobotSubsystem*>(pvParams);
    self->begin();
    while (true) {
      self->update();
      threads.delay(self->task_delay_ms_);
    }
  }
  uint32_t task_delay_ms_ = 100;
#endif

 private:
  // Internal State Machine Logic
  void updateStateMachine();
  void transitionTo(MiniRobotMissionState new_state);

  // Communication with ESP32
  void sendDriveCommand(float target_x, float target_y);
  void sendStopCommand();
  void sendReturnCommand();
  void updateCommsStatus();
  bool pollEsp32Status();

  // Distance Calculations
  float calculateDistance(const Pose2D& a, const Pose2D& b) const;
  bool hasArrivedAtTarget() const;
  bool hasArrivedAtHome() const;

  // ROS Publishing
  void publishState();

  // Configuration
  const MiniRobotSubsystemSetup setup_;

  // State Machine
  MiniRobotMissionState mission_state_ = MiniRobotMissionState::IDLE;
  MiniRobotCommand pending_command_ = MiniRobotCommand::NONE;

  // Timing
  uint32_t state_entry_time_ms_ = 0;
  uint32_t last_comms_time_ms_ = 0;
  uint32_t last_position_update_ms_ = 0;

  // Position State
  Pose2D current_position_;
  Pose2D target_position_;
  Pose2D home_position_;   // Where we started / return to
  Pose2D pending_target_;  // Target for pending START_MISSION command

  // Communication State
  CommsStatus comms_status_ = CommsStatus::DISCONNECTED;
  bool esp32_acknowledged_ = false;

  // ROS Entities
  rcl_publisher_t state_pub_{};
  mcu_msgs__msg__MiniRobotState state_msg_{};
  rcl_node_t* node_ = nullptr;

  // Diagnostics Slop
  uint32_t mission_count_ = 0;
  uint32_t error_count_ = 0;
};

}  // namespace Subsystem
