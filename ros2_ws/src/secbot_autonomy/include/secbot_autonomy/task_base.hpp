#pragma once
/**
 * @file task_base.hpp
 * @author Rafeed Khan
 * @brief Base class for all competition task state machines.
 *
 * Each task is a state machine that:
 *   - start(): begin the task
 *   - step(): called periodically by autonomy node
 *   - cancel(): abort the task
 *   - status(): check current state (idle/running/succeeded/failed)
 *
 * Tasks request motion via ROS action clients (navigation, approach, etc)
 */

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace secbot {

/** @brief Status of a task state machine */
enum class TaskStatus : uint8_t {
  kIdle = 0,   ///< Not started or reset
  kRunning,    ///< Currently executing
  kSucceeded,  ///< Completed successfully
  kFailed      ///< Failed or timed out
};

/**
 * @brief Abstract base class for competition tasks
 *
 * All tasks share this interface so the autonomy node can
 * run them polymorphically
 */
class TaskBase {
 public:
  explicit TaskBase(rclcpp::Node::SharedPtr node) : node_(node) {}
  virtual ~TaskBase() = default;

  // Non-copyable
  TaskBase(const TaskBase&) = delete;
  TaskBase& operator=(const TaskBase&) = delete;

  /** @brief Start the task. Must be called before step() */
  virtual void start() = 0;

  /**
   * @brief Step the task state machine once
   * Called periodically by the autonomy node
   */
  virtual void step() = 0;

  /** @brief Cancel the task immediately */
  virtual void cancel() = 0;

  /** @brief Reset to idle state (for reuse) */
  virtual void reset() { status_ = TaskStatus::kIdle; }

  /** @brief Get current task status */
  TaskStatus status() const { return status_; }

  /** @brief Check if task is still running */
  bool isRunning() const { return status_ == TaskStatus::kRunning; }

  /** @brief Check if task finished (success or failure) */
  bool isDone() const {
    return status_ == TaskStatus::kSucceeded || status_ == TaskStatus::kFailed;
  }

  /** @brief Get task name for logging/debugging */
  virtual std::string name() const = 0;

  /** @brief Get progress 0.0 to 1.0 */
  virtual float progress() const { return progress_; }

 protected:
  rclcpp::Node::SharedPtr node_;
  TaskStatus status_ = TaskStatus::kIdle;
  float progress_ = 0.0f;
};

}  // namespace secbot
