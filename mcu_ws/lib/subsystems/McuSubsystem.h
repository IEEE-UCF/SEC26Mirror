/**
 * @file McuSubsystem.h
 * @brief Defines the "main" logic for the robot
 * @author Aldem Pido
 * @date 12/15/2025
 */
#ifndef MCUSUBSYSTEM_H
#define MCUSUBSYSTEM_H
#include <BaseSubsystem.h>
#include <microros_manager_robot.h>
// micro-ROS generated message for MCU state
#include <mcu_msgs/msg/mcu_state.h>

namespace Subsystem {
// Shared MCU state enum used across setup/data/subsystem
enum class McuState {
  PRE_INIT,
  INIT,
  INIT_FAIL,
  INIT_SUCCESS,
  ARMED,
  RUNNING,
  STOPPED,
  RESET
};

class MCUSubsystemSetup : public Classes::BaseSetup {
 public:
  MCUSubsystemSetup(const char* _id) : Classes::BaseSetup(_id) {}
  McuState mcu_state = McuState::PRE_INIT;
};

class MCUSubsystemData {
 public:
  MCUSubsystemData() : mcu_state_(McuState::PRE_INIT) {}
  McuState mcu_state_;
};

class MCUSubsystemCallbacks {
 public:
  MCUSubsystemCallbacks() = default;
  /*
  Ran in series once.
  True: successful init.
  False unsuccessful init.
  */
  bool (*mcu_init_)() = nullptr;
  /* Can run in parallel many times.
  True: completion
  False: stay on current state
  */
  bool (*mcu_arm_)() = nullptr;
  /* Can run in parallel many times.
  True: completion
  False: stay on current state
  */
  bool (*mcu_begin_)() = nullptr;
  // Can be ran in parallel many times.
  void (*mcu_update_)() = nullptr;
  // Ran in series once.
  void (*mcu_stop_)() = nullptr;
  // Ran in series once.
  void (*mcu_reset_)() = nullptr;
};

class MCUSubsystem : public IMicroRosParticipant,
                     public Classes::BaseSubsystem {
 public:
  explicit MCUSubsystem(const MCUSubsystemSetup& setup,
                        const MCUSubsystemCallbacks& cb)
      : Classes::BaseSubsystem(setup), setup_(setup), cb_(cb), data_() {}

  bool init() override;
  void arm();
  void begin() override;
  void update() override;
  void pause() override {}
  void stop();
  void reset() override;
  const char* getInfo() override;

  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override;
  void onDestroy() override;
  void publishStatus();

 private:
  const MCUSubsystemSetup setup_;
  const MCUSubsystemCallbacks cb_;
  MCUSubsystemData data_;
  rcl_publisher_t pub_{};
  mcu_msgs__msg__McuState msg_{};
  rcl_node_t* node_ = nullptr;
};
};  // namespace Subsystem

#endif