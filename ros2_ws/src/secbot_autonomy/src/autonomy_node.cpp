/**
 * @file autonomy_node.cpp
 * @author Rafeed Khan
 * @brief Implementation of the main autonomy node
 */

#include "secbot_autonomy/autonomy_node.hpp"

#include <chrono>

namespace secbot {

AutonomyNode::AutonomyNode()
    : Node("autonomy_node") {
  // Declare parameters
  this->declare_parameter("step_rate_hz", 50.0);
  step_rate_hz_ = this->get_parameter("step_rate_hz").as_double();

  RCLCPP_INFO(this->get_logger(), "Autonomy node starting (step rate: %.1f Hz)",
              step_rate_hz_);

  // Initialize all tasks
  initializeTasks();

  // Create status publisher
  status_pub_ = this->create_publisher<secbot_msgs::msg::TaskStatus>(
      "autonomy/task_status", 10);

  // Subscribe to task commands
  task_cmd_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
      "autonomy/task_command", 10,
      std::bind(&AutonomyNode::onTaskCommand, this, std::placeholders::_1));

  // Subscribe to antenna target (for antenna_align task)
  antenna_target_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
      "autonomy/antenna_target", 10,
      std::bind(&AutonomyNode::onAntennaTarget, this, std::placeholders::_1));

  // Create step timer
  auto period = std::chrono::duration<double>(1.0 / step_rate_hz_);
  step_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&AutonomyNode::stepCallback, this));

  RCLCPP_INFO(this->get_logger(), "Autonomy node ready!");
}

void AutonomyNode::initializeTasks() {
  auto node_ptr = this->shared_from_this();

  // Create all task instances
  antenna_align_ = std::make_unique<AntennaAlignTask>(node_ptr);
  button_press_ = std::make_unique<ButtonPressTask>(node_ptr);
  crank_turn_ = std::make_unique<CrankTurnTask>(node_ptr);
  crater_entry_ = std::make_unique<CraterEntryTask>(node_ptr);
  flag_plant_ = std::make_unique<FlagPlantTask>(node_ptr);
  keypad_enter_ = std::make_unique<KeypadEnterTask>(node_ptr);
  pressure_clear_ = std::make_unique<PressureClearTask>(node_ptr);

  RCLCPP_INFO(this->get_logger(), "All tasks initialized");
}

TaskBase* AutonomyNode::getTaskById(TaskId id) {
  switch (id) {
    case TaskId::kAntennaAlign:  return antenna_align_.get();
    case TaskId::kButtonPress:   return button_press_.get();
    case TaskId::kCrankTurn:     return crank_turn_.get();
    case TaskId::kCraterEntry:   return crater_entry_.get();
    case TaskId::kFlagPlant:     return flag_plant_.get();
    case TaskId::kKeypadEnter:   return keypad_enter_.get();
    case TaskId::kPressureClear: return pressure_clear_.get();
    default: return nullptr;
  }
}

std::string AutonomyNode::taskIdToName(TaskId id) const {
  switch (id) {
    case TaskId::kNone:          return "none";
    case TaskId::kAntennaAlign:  return "antenna_align";
    case TaskId::kButtonPress:   return "button_press";
    case TaskId::kCrankTurn:     return "crank_turn";
    case TaskId::kCraterEntry:   return "crater_entry";
    case TaskId::kFlagPlant:     return "flag_plant";
    case TaskId::kKeypadEnter:   return "keypad_enter";
    case TaskId::kPressureClear: return "pressure_clear";
    default: return "unknown";
  }
}

void AutonomyNode::startTask(TaskId id) {
  // Stop current task if running
  stopCurrentTask();

  TaskBase* task = getTaskById(id);
  if (!task) {
    RCLCPP_WARN(this->get_logger(), "Invalid task ID: %d", static_cast<int>(id));
    return;
  }

  // Special handling for antenna_align - set target first
  if (id == TaskId::kAntennaAlign) {
    antenna_align_->setTarget(antenna_target_id_);
  }

  // Start the task
  active_task_ = task;
  active_task_id_ = id;
  task->start();

  RCLCPP_INFO(this->get_logger(), "Started task: %s", taskIdToName(id).c_str());
}

void AutonomyNode::stopCurrentTask() {
  if (active_task_ && active_task_->isRunning()) {
    active_task_->cancel();
    RCLCPP_INFO(this->get_logger(), "Stopped task: %s",
                taskIdToName(active_task_id_).c_str());
  }
  active_task_ = nullptr;
  active_task_id_ = TaskId::kNone;
}

void AutonomyNode::stepCallback() {
  // Step active task if running
  if (active_task_) {
    if (active_task_->isRunning()) {
      active_task_->step();
    }

    // Check for completion
    if (active_task_->isDone()) {
      bool success = (active_task_->status() == TaskStatus::kSucceeded);
      RCLCPP_INFO(this->get_logger(), "Task %s %s",
                  taskIdToName(active_task_id_).c_str(),
                  success ? "SUCCEEDED" : "FAILED");

      // Clear active task
      active_task_ = nullptr;
      active_task_id_ = TaskId::kNone;
    }
  }

  // Always publish status
  publishStatus();
}

void AutonomyNode::onTaskCommand(const std_msgs::msg::UInt8::SharedPtr msg) {
  TaskId id = static_cast<TaskId>(msg->data);

  if (id == TaskId::kNone) {
    // Command to stop
    stopCurrentTask();
  } else {
    startTask(id);
  }
}

void AutonomyNode::onAntennaTarget(const std_msgs::msg::UInt8::SharedPtr msg) {
  antenna_target_id_ = msg->data;
  RCLCPP_DEBUG(this->get_logger(), "Antenna target set to: %d", antenna_target_id_);
}

void AutonomyNode::publishStatus() {
  secbot_msgs::msg::TaskStatus status_msg;

  status_msg.task_id = static_cast<uint8_t>(active_task_id_);

  if (active_task_) {
    status_msg.ok = (active_task_->status() == TaskStatus::kRunning ||
                     active_task_->status() == TaskStatus::kSucceeded);
    if (active_task_->status() == TaskStatus::kFailed) {
      status_msg.error_msg = "Task failed";
    }
  } else {
    status_msg.ok = true;  // No task = no error
  }

  status_pub_->publish(status_msg);
}

}  // namespace secbot

// Main entry point
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<secbot::AutonomyNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
