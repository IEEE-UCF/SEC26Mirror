#pragma once
/**
 * @file autonomy_node.hpp
 * @author Rafeed Khan
 * @brief Main autonomy node that owns and runs all task state machines
 *
 * This node:
 *   - Owns instances of all competition tasks
 *   - Runs the active task on a timer (step loop)
 *   - Switches tasks based on service calls or topic commands
 *   - Publishes task status for debugging/monitoring
 */

#include <rclcpp/rclcpp.hpp>

#include "secbot_autonomy/task_base.hpp"
#include "secbot_autonomy/antenna_align.hpp"
#include "secbot_autonomy/button_press.hpp"
#include "secbot_autonomy/crank_turn.hpp"
#include "secbot_autonomy/crater_entry.hpp"
#include "secbot_autonomy/flag_plant.hpp"
#include "secbot_autonomy/keypad_enter.hpp"
#include "secbot_autonomy/pressure_clear.hpp"

#include <secbot_msgs/msg/task_status.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <string>
#include <unordered_map>

namespace secbot {

/** @brief Task IDs for external control */
enum class TaskId : uint8_t {
  kNone = 0,
  kAntennaAlign = 1,
  kButtonPress = 2,
  kCrankTurn = 3,
  kCraterEntry = 4,
  kFlagPlant = 5,
  kKeypadEnter = 6,
  kPressureClear = 7
};

/**
 * @brief Main autonomy node
 *
 * Orchestrates all competition task state machines
 */
class AutonomyNode : public rclcpp::Node {
 public:
  AutonomyNode();

  /** @brief Get the currently active task (nullptr if none) */
  TaskBase* activeTask() const { return active_task_; }

  /** @brief Get current task ID */
  TaskId activeTaskId() const { return active_task_id_; }

 private:
  // Task management
  void initializeTasks();
  void startTask(TaskId id);
  void stopCurrentTask();
  TaskBase* getTaskById(TaskId id);
  std::string taskIdToName(TaskId id) const;

  // Timer callback: steps the active task
  void stepCallback();

  // Subscribers
  void onTaskCommand(const std_msgs::msg::UInt8::SharedPtr msg);
  void onAntennaTarget(const std_msgs::msg::UInt8::SharedPtr msg);

  // Publishing
  void publishStatus();

  // Parameters
  double step_rate_hz_ = 50.0;  // How often to step tasks

  // Task instances
  std::unique_ptr<AntennaAlignTask> antenna_align_;
  std::unique_ptr<ButtonPressTask> button_press_;
  std::unique_ptr<CrankTurnTask> crank_turn_;
  std::unique_ptr<CraterEntryTask> crater_entry_;
  std::unique_ptr<FlagPlantTask> flag_plant_;
  std::unique_ptr<KeypadEnterTask> keypad_enter_;
  std::unique_ptr<PressureClearTask> pressure_clear_;

  // Active task tracking
  TaskBase* active_task_ = nullptr;
  TaskId active_task_id_ = TaskId::kNone;

  // For antenna task, which antenna to target
  uint8_t antenna_target_id_ = 1;

  // ROS interfaces
  rclcpp::TimerBase::SharedPtr step_timer_;
  rclcpp::Publisher<secbot_msgs::msg::TaskStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr task_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr antenna_target_sub_;
};

}  // namespace secbot
