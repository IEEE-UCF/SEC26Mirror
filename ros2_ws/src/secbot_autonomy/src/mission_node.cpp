/**
 * @file mission_node.cpp
 * @author Rafeed Khan
 * @brief Implementation of the mission sequencer state machine
 *
 * Minibot launches FIRST (state 2), antenna alignment folded into solve
 * states, intake rail replaces bridge abstraction for pressure plate.
 *
 * Drives robot via drive_base/command (DriveBase msg) — no Nav2.
 * Reads pose from drive_base/status.  MCU auto-stops after 500ms without
 * a command, so refreshDriveCommand() re-publishes every tick (100ms).
 */

#include "secbot_autonomy/mission_node.hpp"

#include <cmath>
#include <nav_msgs/msg/path.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace secbot {

MissionNode::MissionNode() : Node("mission_node") {
  RCLCPP_INFO(this->get_logger(), "Mission node starting...");

  // Publishers
  drive_cmd_pub_ =
      this->create_publisher<mcu_msgs::msg::DriveBase>("drive_base/command", 10);
  task_cmd_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
      "/autonomy/task_command", 10);
  antenna_target_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
      "/autonomy/antenna_target", 10);
  minibot_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/mcu_minibot/cmd_vel", 10);
  arm_cmd_pub_ =
      this->create_publisher<mcu_msgs::msg::ArmCommand>("/arm_command", 10);
  intake_cmd_pub_ = this->create_publisher<mcu_msgs::msg::IntakeCommand>(
      "/mcu_robot/intake/command", 10);
  drone_cmd_pub_ = this->create_publisher<mcu_msgs::msg::DroneControl>(
      "/mcu_drone/control", 10);

  // Subscribers
  drive_status_sub_ = this->create_subscription<mcu_msgs::msg::DriveBase>(
      "drive_base/status", 10,
      std::bind(&MissionNode::onDriveStatus, this, _1));
  task_status_sub_ = this->create_subscription<secbot_msgs::msg::TaskStatus>(
      "/autonomy/task_status", 10,
      std::bind(&MissionNode::onTaskStatus, this, _1));
  minibot_state_sub_ = this->create_subscription<mcu_msgs::msg::MiniRobotState>(
      "/mcu_minibot/state", 10,
      std::bind(&MissionNode::onMinibotState, this, _1));
  drone_state_sub_ = this->create_subscription<mcu_msgs::msg::DroneState>(
      "/mcu_drone/state", 10, std::bind(&MissionNode::onDroneState, this, _1));
  robot_inputs_sub_ = this->create_subscription<mcu_msgs::msg::RobotInputs>(
      "/mcu_robot/inputs", 10,
      std::bind(&MissionNode::onRobotInputs, this, _1));
  intake_state_sub_ = this->create_subscription<mcu_msgs::msg::IntakeState>(
      "/mcu_robot/intake/state", 10,
      std::bind(&MissionNode::onIntakeState, this, _1));
  duck_detect_sub_ =
      this->create_subscription<secbot_msgs::msg::DuckDetections>(
          "/duck_detections", 10,
          std::bind(&MissionNode::onDuckDetections, this, _1));
  antenna_marker_sub_ = this->create_subscription<mcu_msgs::msg::AntennaMarker>(
      "/antenna_markers", 10,
      std::bind(&MissionNode::onAntennaMarker, this, _1));

  // Start service — call to begin the mission
  start_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/mission/start",
      std::bind(&MissionNode::onStartService, this, _1, std::placeholders::_2));

  // 10 Hz step timer
  step_timer_ = this->create_wall_timer(
      100ms, std::bind(&MissionNode::stepMission, this));

  RCLCPP_INFO(this->get_logger(),
              "Mission node ready, waiting for start signal");
}

// Service callbacks

void MissionNode::onStartService(
    const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
    std_srvs::srv::Trigger::Response::SharedPtr res) {
  if (!start_button_pressed_) {
    start_button_pressed_ = true;
    RCLCPP_INFO(this->get_logger(), "Mission start triggered via service!");
    res->success = true;
    res->message = "Mission started";
  } else {
    res->success = false;
    res->message = "Mission already started";
  }
}

// Callbacks

void MissionNode::onDriveStatus(
    const mcu_msgs::msg::DriveBase::SharedPtr msg) {
  // Extract robot pose from transform
  robot_x_ = msg->transform.transform.translation.x;
  robot_y_ = msg->transform.transform.translation.y;
  auto& r = msg->transform.transform.rotation;
  robot_theta_ = 2.0 * std::atan2(r.z, r.w);

  // Goal-reached: distance AND heading within tolerance
  double dx = robot_x_ - nav_target_.x;
  double dy = robot_y_ - nav_target_.y;
  double dist = std::sqrt(dx * dx + dy * dy);
  double heading_err = std::abs(
      std::remainder(robot_theta_ - nav_target_.theta, 2.0 * M_PI));
  if (dist < GOAL_REACHED_TOL && heading_err < HEADING_REACHED_TOL &&
      !goal_reached_) {
    goal_reached_ = true;
    RCLCPP_INFO(this->get_logger(),
                "Navigation goal reached! (dist=%.3fm, heading_err=%.2frad)",
                dist, heading_err);
  }

  // Also detect mode transition: GOAL/TRAJ → DRIVE_VECTOR with zero velocity
  // indicates MCU finished its goal (position + heading)
  uint8_t mode = msg->drive_mode;
  if ((last_drive_mode_ == mcu_msgs::msg::DriveBase::DRIVE_GOAL ||
       last_drive_mode_ == mcu_msgs::msg::DriveBase::DRIVE_TRAJ) &&
      mode == mcu_msgs::msg::DriveBase::DRIVE_VECTOR) {
    double vx = msg->twist.linear.x;
    double omega = msg->twist.angular.z;
    if (std::abs(vx) < 0.01 && std::abs(omega) < 0.01 && !goal_reached_) {
      goal_reached_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "Navigation goal reached! (MCU went idle)");
    }
  }
  last_drive_mode_ = mode;
}

void MissionNode::onTaskStatus(
    const secbot_msgs::msg::TaskStatus::SharedPtr msg) {
  current_task_id_ = msg->task_id;
  task_idle_ = (msg->task_id == TASK_NONE);
}

void MissionNode::onMinibotState(
    const mcu_msgs::msg::MiniRobotState::SharedPtr msg) {
  minibot_state_ = msg->state;
  minibot_task_ = msg->current_task;
}

void MissionNode::onDroneState(const mcu_msgs::msg::DroneState::SharedPtr msg) {
  drone_state_ = msg->state;
  drone_task_ = msg->current_task;
}

void MissionNode::onRobotInputs(
    const mcu_msgs::msg::RobotInputs::SharedPtr msg) {
  // btn1 is the manual start button
  if (msg->btn1 && !start_button_pressed_) {
    start_button_pressed_ = true;
    RCLCPP_INFO(this->get_logger(), "Start button pressed!");
  }
}

void MissionNode::onIntakeState(
    const mcu_msgs::msg::IntakeState::SharedPtr msg) {
  intake_state_ = msg->state;
  intake_position_ = msg->position;
  intake_limit_extend_ = msg->limit_extend_active;
  intake_limit_retract_ = msg->limit_retract_active;
}

void MissionNode::onDuckDetections(
    const secbot_msgs::msg::DuckDetections::SharedPtr msg) {
  for (const auto& det : msg->detections) {
    if (det.confidence < 0.7) continue;

    ArenaPosition duck_pos = {robot_x_, robot_y_, 0.0};

    // Avoid storing duplicates (within 0.5m of existing)
    bool is_new = true;
    for (const auto& known : known_duck_positions_) {
      double dx = duck_pos.x - known.x;
      double dy = duck_pos.y - known.y;
      if (dx * dx + dy * dy < 0.25) {
        is_new = false;
        break;
      }
    }

    if (is_new) {
      known_duck_positions_.push_back(duck_pos);
      RCLCPP_INFO(this->get_logger(),
                  "Duck detected! Stored position (%.2f, %.2f), total: %zu",
                  duck_pos.x, duck_pos.y, known_duck_positions_.size());
    }

    // Flag for duck interrupt (post COLLECT_KNOWN_DUCKS)
    if (duck_interrupt_enabled_ && !pending_duck_interrupt_) {
      pending_duck_interrupt_ = true;
      interrupt_duck_pos_ = duck_pos;
    }
  }
}

void MissionNode::onAntennaMarker(
    const mcu_msgs::msg::AntennaMarker::SharedPtr msg) {
  if (msg->antenna_id >= 1 && msg->antenna_id <= 4) {
    uint8_t color_code = 0;
    if (msg->color_class == "red")
      color_code = mcu_msgs::msg::AntennaMarker::COLOR_RED;
    else if (msg->color_class == "blue")
      color_code = mcu_msgs::msg::AntennaMarker::COLOR_BLUE;
    else if (msg->color_class == "green")
      color_code = mcu_msgs::msg::AntennaMarker::COLOR_GREEN;
    else if (msg->color_class == "purple")
      color_code = mcu_msgs::msg::AntennaMarker::COLOR_PURPLE;

    if (antenna_colors_[msg->antenna_id] == 0 && color_code != 0) {
      antenna_colors_[msg->antenna_id] = color_code;
      antennas_read_++;
      RCLCPP_INFO(this->get_logger(), "Antenna %d LED color: %s (%d/%d read)",
                  msg->antenna_id, msg->color_class.c_str(), antennas_read_, 4);
    }
  }
}

// Navigation helpers

void MissionNode::sendDriveGoal(const ArenaPosition& pos, bool reverse) {
  goal_reached_ = false;
  nav_target_ = pos;
  auto msg = mcu_msgs::msg::DriveBase();
  msg.header.stamp = this->now();
  msg.drive_mode = mcu_msgs::msg::DriveBase::DRIVE_GOAL;
  msg.goal_transform.transform.translation.x = pos.x;
  msg.goal_transform.transform.translation.y = pos.y;
  msg.goal_transform.transform.translation.z = reverse ? -1.0 : 0.0;

  // Encode heading as yaw-only quaternion
  float half = static_cast<float>(pos.theta) * 0.5f;
  msg.goal_transform.transform.rotation.x = 0.0;
  msg.goal_transform.transform.rotation.y = 0.0;
  msg.goal_transform.transform.rotation.z = std::sin(half);
  msg.goal_transform.transform.rotation.w = std::cos(half);

  last_drive_cmd_ = msg;
  drive_cmd_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(),
              "Sent drive goal: (%.3f, %.3f, %.2frad)%s", pos.x, pos.y,
              pos.theta, reverse ? " [reverse]" : "");
}

void MissionNode::sendDrivePath(
    const std::vector<ArenaPosition>& waypoints, double final_heading) {
  goal_reached_ = false;
  if (!waypoints.empty()) {
    nav_target_ = waypoints.back();
    // Override nav_target heading if explicit final_heading given
    if (!std::isnan(final_heading)) {
      nav_target_.theta = final_heading;
    }
  }
  auto msg = mcu_msgs::msg::DriveBase();
  msg.header.stamp = this->now();
  msg.drive_mode = mcu_msgs::msg::DriveBase::DRIVE_TRAJ;

  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = "odom";

  for (const auto& wp : waypoints) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position.x = wp.x;
    ps.pose.position.y = wp.y;
    ps.pose.orientation.w = 1.0;
    path.poses.push_back(std::move(ps));
  }

  // Encode final heading in last waypoint quaternion
  double heading = std::isnan(final_heading) && !waypoints.empty()
                       ? waypoints.back().theta
                       : final_heading;
  if (!std::isnan(heading) && !path.poses.empty()) {
    float half = static_cast<float>(heading) * 0.5f;
    auto& last = path.poses.back();
    last.pose.orientation.z = std::sin(half);
    last.pose.orientation.w = std::cos(half);
  }

  msg.goal_path = path;

  // Also set goal_transform to last waypoint for the MCU
  if (!waypoints.empty()) {
    msg.goal_transform.transform.translation.x = waypoints.back().x;
    msg.goal_transform.transform.translation.y = waypoints.back().y;
  }

  last_drive_cmd_ = msg;
  drive_cmd_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Sent drive path with %zu waypoints",
              waypoints.size());
}

void MissionNode::sendVelocity(double vx, double omega) {
  auto msg = mcu_msgs::msg::DriveBase();
  msg.header.stamp = this->now();
  msg.drive_mode = mcu_msgs::msg::DriveBase::DRIVE_VECTOR;
  msg.goal_velocity.linear.x = vx;
  msg.goal_velocity.angular.z = omega;
  last_drive_cmd_ = msg;
  drive_cmd_pub_->publish(msg);
}

void MissionNode::stopRobot() { sendVelocity(0.0, 0.0); }

void MissionNode::refreshDriveCommand() {
  // Re-publish the last drive command every tick to prevent MCU 500ms timeout
  last_drive_cmd_.header.stamp = this->now();
  drive_cmd_pub_->publish(last_drive_cmd_);
}

// Task helpers

void MissionNode::sendTaskCommand(uint8_t task_id) {
  auto msg = std_msgs::msg::UInt8();
  msg.data = task_id;
  task_cmd_pub_->publish(msg);
  task_idle_ = false;
}

void MissionNode::sendAntennaTarget(uint8_t antenna_id) {
  auto msg = std_msgs::msg::UInt8();
  msg.data = antenna_id;
  antenna_target_pub_->publish(msg);
}

void MissionNode::sendArmCommand(uint8_t joint_id, int16_t position,
                                 uint8_t speed) {
  auto msg = mcu_msgs::msg::ArmCommand();
  msg.joint_id = joint_id;
  msg.position = position;
  msg.speed = speed;
  arm_cmd_pub_->publish(msg);
}

void MissionNode::sendIntakeCommand(uint8_t cmd, float value) {
  auto msg = mcu_msgs::msg::IntakeCommand();
  msg.command = cmd;
  msg.value = value;
  intake_cmd_pub_->publish(msg);
}

bool MissionNode::checkStateTimeout(double timeout_s) {
  auto elapsed = std::chrono::steady_clock::now() - phase_timer_;
  double sec = std::chrono::duration<double>(elapsed).count();
  if (sec > timeout_s) {
    RCLCPP_WARN(this->get_logger(), "State %s timeout (%.1fs > %.1fs)",
                phaseName(phase_), sec, timeout_s);
    return true;
  }
  return false;
}

void MissionNode::transitionTo(MissionPhase phase) {
  auto match_elapsed = std::chrono::steady_clock::now() - match_start_;
  double match_sec = std::chrono::duration<double>(match_elapsed).count();
  RCLCPP_INFO(this->get_logger(), "MISSION [%.1fs]: %s -> %s", match_sec,
              phaseName(phase_), phaseName(phase));
  phase_ = phase;
  phase_entry_ = true;
  phase_timer_ = std::chrono::steady_clock::now();
}

const char* MissionNode::phaseName(MissionPhase phase) const {
  switch (phase) {
    case MissionPhase::WAIT_FOR_START:
      return "WAIT_FOR_START";
    case MissionPhase::ROBOT_START:
      return "ROBOT_START";
    case MissionPhase::MINIBOT_LAUNCH:
      return "MINIBOT_LAUNCH";
    case MissionPhase::NAV_TO_UWB_CORNER:
      return "NAV_TO_UWB_CORNER";
    case MissionPhase::PLACE_UWB_BEACON:
      return "PLACE_UWB_BEACON";
    case MissionPhase::NAV_TO_BUTTON:
      return "NAV_TO_BUTTON";
    case MissionPhase::SOLVE_BUTTON:
      return "SOLVE_BUTTON";
    case MissionPhase::NAV_TO_CRANK_FLAG:
      return "NAV_TO_CRANK_FLAG";
    case MissionPhase::PLACE_UWB_FLAG:
      return "PLACE_UWB_FLAG";
    case MissionPhase::NAV_TO_CRANK:
      return "NAV_TO_CRANK";
    case MissionPhase::SOLVE_CRANK:
      return "SOLVE_CRANK";
    case MissionPhase::COLLECT_KNOWN_DUCKS:
      return "COLLECT_KNOWN_DUCKS";
    case MissionPhase::NAV_TO_KEYPAD:
      return "NAV_TO_KEYPAD";
    case MissionPhase::SOLVE_KEYPAD:
      return "SOLVE_KEYPAD";
    case MissionPhase::NAV_TO_PRESSURE:
      return "NAV_TO_PRESSURE";
    case MissionPhase::SOLVE_PRESSURE:
      return "SOLVE_PRESSURE";
    case MissionPhase::DEPOSIT_DUCK:
      return "DEPOSIT_DUCK";
    case MissionPhase::VIEW_LED_COLORS:
      return "VIEW_LED_COLORS";
    case MissionPhase::NAV_TO_LAUNCH:
      return "NAV_TO_LAUNCH";
    case MissionPhase::LAUNCH_DRONE:
      return "LAUNCH_DRONE";
    case MissionPhase::NAV_TO_FINISH:
      return "NAV_TO_FINISH";
    case MissionPhase::MISSION_COMPLETE:
      return "MISSION_COMPLETE";
    case MissionPhase::DUCK_INTERRUPT_NAV:
      return "DUCK_INTERRUPT_NAV";
    case MissionPhase::DUCK_INTERRUPT_CAPTURE:
      return "DUCK_INTERRUPT_CAPTURE";
    case MissionPhase::DUCK_INTERRUPT_DEPOSIT_NAV:
      return "DUCK_INTERRUPT_DEPOSIT_NAV";
    case MissionPhase::DUCK_INTERRUPT_DEPOSIT:
      return "DUCK_INTERRUPT_DEPOSIT";
    case MissionPhase::DUCK_INTERRUPT_RESUME:
      return "DUCK_INTERRUPT_RESUME";
    default:
      return "UNKNOWN";
  }
}

// Main State Machine

void MissionNode::stepMission() {
  // Duck interrupt check (active after COLLECT_KNOWN_DUCKS)
  if (duck_interrupt_enabled_ && pending_duck_interrupt_) {
    bool in_nav_phase = (phase_ == MissionPhase::NAV_TO_KEYPAD ||
                         phase_ == MissionPhase::NAV_TO_PRESSURE ||
                         phase_ == MissionPhase::NAV_TO_LAUNCH ||
                         phase_ == MissionPhase::NAV_TO_FINISH);
    if (in_nav_phase) {
      RCLCPP_WARN(
          this->get_logger(),
          "DUCK INTERRUPT! Saving phase %s, handling duck at (%.2f, %.2f)",
          phaseName(phase_), interrupt_duck_pos_.x, interrupt_duck_pos_.y);
      saved_phase_ = phase_;
      saved_phase_entry_ = phase_entry_;
      pending_duck_interrupt_ = false;
      transitionTo(MissionPhase::DUCK_INTERRUPT_NAV);
    }
  }

  switch (phase_) {
    // -- WAIT FOR START --
    case MissionPhase::WAIT_FOR_START: {
      if (phase_entry_) {
        RCLCPP_INFO(this->get_logger(),
                    "Waiting for start signal (button press or light bar)...");
        phase_entry_ = false;
      }
      if (start_button_pressed_) {
        match_start_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "MATCH STARTED (button press)!");
        transitionTo(MissionPhase::ROBOT_START);
      }
      break;
    }

    // -- ROBOT START (readiness checks + settle) --
    case MissionPhase::ROBOT_START: {
      if (phase_entry_) {
        RCLCPP_INFO(this->get_logger(), "Robot starting, settling for 0.5s...");
        phase_entry_ = false;
      }
      auto elapsed = std::chrono::steady_clock::now() - phase_timer_;
      if (std::chrono::duration<double>(elapsed).count() >= 0.5) {
        transitionTo(MissionPhase::MINIBOT_LAUNCH);
      }
      break;
    }

    // -- MINIBOT LAUNCH (moved to early in sequence) --
    case MissionPhase::MINIBOT_LAUNCH: {
      if (phase_entry_) {
        RCLCPP_INFO(this->get_logger(), "Deploying minibot!");
        // Open minibot container latch
        sendArmCommand(JOINT_MINIBOT_LATCH, SERVO_EXTENDED, SERVO_SPEED);
        phase_entry_ = false;
      }
      auto elapsed = std::chrono::steady_clock::now() - phase_timer_;
      double sec = std::chrono::duration<double>(elapsed).count();

      // After 1s latch open, send forward command to start minibot's run
      if (sec >= 1.0 && sec < 1.2) {
        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = 0.5;
        minibot_cmd_pub_->publish(cmd);
      }

      // After 3s, close latch and advance
      if (sec >= 3.0) {
        sendArmCommand(JOINT_MINIBOT_LATCH, SERVO_RETRACTED, SERVO_SPEED);
        RCLCPP_INFO(this->get_logger(),
                    "Minibot deployed, navigating to UWB corner");
        transitionTo(MissionPhase::NAV_TO_UWB_CORNER);
      }
      if (checkStateTimeout(5.0)) {
        sendArmCommand(JOINT_MINIBOT_LATCH, SERVO_RETRACTED, SERVO_SPEED);
        transitionTo(MissionPhase::NAV_TO_UWB_CORNER);
      }
      break;
    }

    // -- NAVIGATE TO UWB CORNER --
    case MissionPhase::NAV_TO_UWB_CORNER: {
      if (phase_entry_) {
        sendDriveGoal(POS_UWB_CORNER);
        phase_entry_ = false;
      } else {
        refreshDriveCommand();
      }
      if (goal_reached_) {
        transitionTo(MissionPhase::PLACE_UWB_BEACON);
      }
      if (checkStateTimeout(DEFAULT_NAV_TIMEOUT_S)) {
        transitionTo(MissionPhase::PLACE_UWB_BEACON);
      }
      break;
    }

    // -- PLACE UWB BEACON (FlagPlant task) --
    case MissionPhase::PLACE_UWB_BEACON: {
      if (phase_entry_) {
        RCLCPP_INFO(this->get_logger(), "Placing UWB beacon");
        sendTaskCommand(TASK_FLAG_PLANT);
        phase_entry_ = false;
      }
      if (task_idle_) {
        transitionTo(MissionPhase::NAV_TO_BUTTON);
      }
      if (checkStateTimeout(DEFAULT_TASK_TIMEOUT_S)) {
        sendTaskCommand(TASK_NONE);
        transitionTo(MissionPhase::NAV_TO_BUTTON);
      }
      break;
    }

    // -- NAVIGATE TO BUTTON BEACON --
    case MissionPhase::NAV_TO_BUTTON: {
      if (phase_entry_) {
        sendDriveGoal(POS_BUTTON_APPROACH);
        phase_entry_ = false;
      } else {
        refreshDriveCommand();
      }
      if (goal_reached_) {
        transitionTo(MissionPhase::SOLVE_BUTTON);
      }
      if (checkStateTimeout(DEFAULT_NAV_TIMEOUT_S)) {
        transitionTo(MissionPhase::SOLVE_BUTTON);
      }
      break;
    }

    // -- SOLVE BUTTON (align antenna 1, then press 3x) --
    case MissionPhase::SOLVE_BUTTON: {
      if (phase_entry_) {
        sub_step_ = SolveSubStep::ALIGN_ANTENNA;
        sendAntennaTarget(1);
        sendTaskCommand(TASK_ANTENNA_ALIGN);
        RCLCPP_INFO(this->get_logger(), "Aligning to antenna 1");
        phase_entry_ = false;
      }

      switch (sub_step_) {
        case SolveSubStep::ALIGN_ANTENNA:
          if (task_idle_) {
            sub_step_ = SolveSubStep::EXECUTE_TASK;
            sendTaskCommand(TASK_BUTTON_PRESS);
            RCLCPP_INFO(this->get_logger(), "Pressing button");
            phase_timer_ = std::chrono::steady_clock::now();
          }
          if (checkStateTimeout(DEFAULT_TASK_TIMEOUT_S)) {
            sub_step_ = SolveSubStep::EXECUTE_TASK;
            sendTaskCommand(TASK_BUTTON_PRESS);
            phase_timer_ = std::chrono::steady_clock::now();
          }
          break;
        case SolveSubStep::EXECUTE_TASK:
          if (task_idle_) {
            transitionTo(MissionPhase::NAV_TO_CRANK_FLAG);
          }
          if (checkStateTimeout(DEFAULT_TASK_TIMEOUT_S)) {
            sendTaskCommand(TASK_NONE);
            transitionTo(MissionPhase::NAV_TO_CRANK_FLAG);
          }
          break;
        case SolveSubStep::DONE:
          transitionTo(MissionPhase::NAV_TO_CRANK_FLAG);
          break;
      }
      break;
    }

    // -- NAVIGATE TO CRANK FLAG POSITION --
    case MissionPhase::NAV_TO_CRANK_FLAG: {
      if (phase_entry_) {
        sendDriveGoal(POS_CRANK_FLAG);
        phase_entry_ = false;
      } else {
        refreshDriveCommand();
      }
      if (goal_reached_) {
        transitionTo(MissionPhase::PLACE_UWB_FLAG);
      }
      if (checkStateTimeout(DEFAULT_NAV_TIMEOUT_S)) {
        transitionTo(MissionPhase::PLACE_UWB_FLAG);
      }
      break;
    }

    // -- PLACE UWB FLAG --
    case MissionPhase::PLACE_UWB_FLAG: {
      if (phase_entry_) {
        RCLCPP_INFO(this->get_logger(), "Placing UWB flag at crank beacon");
        sendTaskCommand(TASK_FLAG_PLANT);
        phase_entry_ = false;
      }
      if (task_idle_) {
        transitionTo(MissionPhase::NAV_TO_CRANK);
      }
      if (checkStateTimeout(DEFAULT_TASK_TIMEOUT_S)) {
        sendTaskCommand(TASK_NONE);
        transitionTo(MissionPhase::NAV_TO_CRANK);
      }
      break;
    }

    // -- NAVIGATE TO CRANK --
    case MissionPhase::NAV_TO_CRANK: {
      if (phase_entry_) {
        sendDriveGoal(POS_CRANK_APPROACH);
        phase_entry_ = false;
      } else {
        refreshDriveCommand();
      }
      if (goal_reached_) {
        transitionTo(MissionPhase::SOLVE_CRANK);
      }
      if (checkStateTimeout(DEFAULT_NAV_TIMEOUT_S)) {
        transitionTo(MissionPhase::SOLVE_CRANK);
      }
      break;
    }

    // -- SOLVE CRANK (align antenna 2, then 540 degrees) --
    case MissionPhase::SOLVE_CRANK: {
      if (phase_entry_) {
        sub_step_ = SolveSubStep::ALIGN_ANTENNA;
        sendAntennaTarget(2);
        sendTaskCommand(TASK_ANTENNA_ALIGN);
        RCLCPP_INFO(this->get_logger(), "Aligning to antenna 2");
        phase_entry_ = false;
      }

      switch (sub_step_) {
        case SolveSubStep::ALIGN_ANTENNA:
          if (task_idle_) {
            sub_step_ = SolveSubStep::EXECUTE_TASK;
            sendTaskCommand(TASK_CRANK_TURN);
            RCLCPP_INFO(this->get_logger(), "Turning crank");
            phase_timer_ = std::chrono::steady_clock::now();
          }
          if (checkStateTimeout(DEFAULT_TASK_TIMEOUT_S)) {
            sub_step_ = SolveSubStep::EXECUTE_TASK;
            sendTaskCommand(TASK_CRANK_TURN);
            phase_timer_ = std::chrono::steady_clock::now();
          }
          break;
        case SolveSubStep::EXECUTE_TASK:
          if (task_idle_) {
            transitionTo(MissionPhase::COLLECT_KNOWN_DUCKS);
          }
          if (checkStateTimeout(DEFAULT_TASK_TIMEOUT_S)) {
            sendTaskCommand(TASK_NONE);
            transitionTo(MissionPhase::COLLECT_KNOWN_DUCKS);
          }
          break;
        case SolveSubStep::DONE:
          transitionTo(MissionPhase::COLLECT_KNOWN_DUCKS);
          break;
      }
      break;
    }

    // -- COLLECT KNOWN DUCKS (multi-step loop) --
    case MissionPhase::COLLECT_KNOWN_DUCKS: {
      if (phase_entry_) {
        duck_collect_index_ = 0;
        duck_collect_step_ = DuckCollectStep::NAV_TO_DUCK;
        RCLCPP_INFO(this->get_logger(), "Collecting %zu known ducks",
                    known_duck_positions_.size());
        phase_entry_ = false;

        if (known_duck_positions_.empty()) {
          RCLCPP_INFO(this->get_logger(),
                      "No ducks to collect, enabling interrupt mode");
          duck_interrupt_enabled_ = true;
          transitionTo(MissionPhase::NAV_TO_KEYPAD);
          break;
        }
      }

      switch (duck_collect_step_) {
        case DuckCollectStep::NAV_TO_DUCK: {
          sendDriveGoal(known_duck_positions_[duck_collect_index_]);
          RCLCPP_INFO(this->get_logger(), "Navigating to duck %zu/%zu",
                      duck_collect_index_ + 1, known_duck_positions_.size());
          duck_collect_step_ = DuckCollectStep::INTAKE_ON;
          break;
        }
        case DuckCollectStep::INTAKE_ON: {
          refreshDriveCommand();
          if (goal_reached_) {
            sendIntakeCommand(
                mcu_msgs::msg::IntakeCommand::CMD_SET_INTAKE_SPEED, 1.0f);
            phase_timer_ = std::chrono::steady_clock::now();
            duck_collect_step_ = DuckCollectStep::WAIT_CAPTURE;
          }
          break;
        }
        case DuckCollectStep::WAIT_CAPTURE: {
          auto elapsed = std::chrono::steady_clock::now() - phase_timer_;
          if (std::chrono::duration<double>(elapsed).count() >= 3.0) {
            sendIntakeCommand(mcu_msgs::msg::IntakeCommand::CMD_STOP_INTAKE);
            duck_collect_step_ = DuckCollectStep::NAV_TO_DEPOSIT;
          }
          break;
        }
        case DuckCollectStep::NAV_TO_DEPOSIT: {
          sendDriveGoal(POS_DUCK_DEPOSIT);
          duck_collect_step_ = DuckCollectStep::EJECT;
          break;
        }
        case DuckCollectStep::EJECT: {
          refreshDriveCommand();
          if (goal_reached_) {
            sendIntakeCommand(
                mcu_msgs::msg::IntakeCommand::CMD_SET_INTAKE_SPEED, -1.0f);
            phase_timer_ = std::chrono::steady_clock::now();
            duck_collect_step_ = DuckCollectStep::WAIT_EJECT;
          }
          break;
        }
        case DuckCollectStep::WAIT_EJECT: {
          auto elapsed = std::chrono::steady_clock::now() - phase_timer_;
          if (std::chrono::duration<double>(elapsed).count() >= 2.0) {
            sendIntakeCommand(mcu_msgs::msg::IntakeCommand::CMD_STOP_INTAKE);
            duck_collect_step_ = DuckCollectStep::NEXT_DUCK;
          }
          break;
        }
        case DuckCollectStep::NEXT_DUCK: {
          duck_collect_index_++;
          if (duck_collect_index_ < known_duck_positions_.size()) {
            duck_collect_step_ = DuckCollectStep::NAV_TO_DUCK;
          } else {
            RCLCPP_INFO(this->get_logger(),
                        "All ducks collected! Enabling duck interrupt mode");
            duck_interrupt_enabled_ = true;
            transitionTo(MissionPhase::NAV_TO_KEYPAD);
          }
          break;
        }
      }
      // Overall duck collection timeout
      if (checkStateTimeout(60.0)) {
        sendIntakeCommand(mcu_msgs::msg::IntakeCommand::CMD_STOP_INTAKE);
        duck_interrupt_enabled_ = true;
        transitionTo(MissionPhase::NAV_TO_KEYPAD);
      }
      break;
    }

    // -- NAVIGATE TO KEYPAD BEACON --
    // Multi-waypoint: crank area → below crater → keypad (west side)
    case MissionPhase::NAV_TO_KEYPAD: {
      if (phase_entry_) {
        sendDrivePath({{1.04, 1.14, 0.0}, POS_KEYPAD_APPROACH});
        phase_entry_ = false;
      } else {
        refreshDriveCommand();
      }
      if (goal_reached_) {
        transitionTo(MissionPhase::SOLVE_KEYPAD);
      }
      if (checkStateTimeout(DEFAULT_NAV_TIMEOUT_S)) {
        transitionTo(MissionPhase::SOLVE_KEYPAD);
      }
      break;
    }

    // -- SOLVE KEYPAD (align antenna 4, then type code) --
    case MissionPhase::SOLVE_KEYPAD: {
      if (phase_entry_) {
        sub_step_ = SolveSubStep::ALIGN_ANTENNA;
        sendAntennaTarget(4);
        sendTaskCommand(TASK_ANTENNA_ALIGN);
        RCLCPP_INFO(this->get_logger(), "Aligning to antenna 4");
        phase_entry_ = false;
      }

      switch (sub_step_) {
        case SolveSubStep::ALIGN_ANTENNA:
          if (task_idle_) {
            sub_step_ = SolveSubStep::EXECUTE_TASK;
            sendTaskCommand(TASK_KEYPAD_ENTER);
            RCLCPP_INFO(this->get_logger(), "Entering keypad code");
            phase_timer_ = std::chrono::steady_clock::now();
          }
          if (checkStateTimeout(DEFAULT_TASK_TIMEOUT_S)) {
            sub_step_ = SolveSubStep::EXECUTE_TASK;
            sendTaskCommand(TASK_KEYPAD_ENTER);
            phase_timer_ = std::chrono::steady_clock::now();
          }
          break;
        case SolveSubStep::EXECUTE_TASK:
          if (task_idle_) {
            transitionTo(MissionPhase::NAV_TO_PRESSURE);
          }
          if (checkStateTimeout(DEFAULT_TASK_TIMEOUT_S)) {
            sendTaskCommand(TASK_NONE);
            transitionTo(MissionPhase::NAV_TO_PRESSURE);
          }
          break;
        case SolveSubStep::DONE:
          transitionTo(MissionPhase::NAV_TO_PRESSURE);
          break;
      }
      break;
    }

    // -- NAVIGATE TO PRESSURE PLATE --
    // Multi-waypoint: keypad → up left wall → approach crater from west
    case MissionPhase::NAV_TO_PRESSURE: {
      if (phase_entry_) {
        sendDrivePath({{0.13, 1.40, M_PI_2}, POS_PRESSURE_APPROACH});
        phase_entry_ = false;
      } else {
        refreshDriveCommand();
      }
      if (goal_reached_) {
        transitionTo(MissionPhase::SOLVE_PRESSURE);
      }
      if (checkStateTimeout(DEFAULT_NAV_TIMEOUT_S)) {
        transitionTo(MissionPhase::SOLVE_PRESSURE);
      }
      break;
    }

    // -- SOLVE PRESSURE (align antenna 3, then intake rail sub-FSM) --
    case MissionPhase::SOLVE_PRESSURE: {
      if (phase_entry_) {
        sub_step_ = SolveSubStep::ALIGN_ANTENNA;
        sendAntennaTarget(3);
        sendTaskCommand(TASK_ANTENNA_ALIGN);
        RCLCPP_INFO(this->get_logger(), "Aligning to antenna 3");
        phase_entry_ = false;
      }

      switch (sub_step_) {
        case SolveSubStep::ALIGN_ANTENNA:
          if (task_idle_) {
            sub_step_ = SolveSubStep::EXECUTE_TASK;
            rail_step_ = IntakeRailStep::EXTEND;
            RCLCPP_INFO(this->get_logger(),
                        "Antenna aligned, starting intake rail");
            // Extend rail and start intake spinner
            sendIntakeCommand(mcu_msgs::msg::IntakeCommand::CMD_EXTEND);
            sendIntakeCommand(
                mcu_msgs::msg::IntakeCommand::CMD_SET_INTAKE_SPEED, 1.0f);
            phase_timer_ = std::chrono::steady_clock::now();
          }
          if (checkStateTimeout(DEFAULT_TASK_TIMEOUT_S)) {
            sub_step_ = SolveSubStep::EXECUTE_TASK;
            rail_step_ = IntakeRailStep::EXTEND;
            sendIntakeCommand(mcu_msgs::msg::IntakeCommand::CMD_EXTEND);
            sendIntakeCommand(
                mcu_msgs::msg::IntakeCommand::CMD_SET_INTAKE_SPEED, 1.0f);
            phase_timer_ = std::chrono::steady_clock::now();
          }
          break;

        case SolveSubStep::EXECUTE_TASK: {
          auto elapsed = std::chrono::steady_clock::now() - phase_timer_;
          double sec = std::chrono::duration<double>(elapsed).count();

          switch (rail_step_) {
            case IntakeRailStep::EXTEND:
              if (intake_position_ >= 0.95 || intake_limit_extend_ ||
                  sec >= 3.0) {
                rail_step_ = IntakeRailStep::WAIT_CAPTURE;
                phase_timer_ = std::chrono::steady_clock::now();
                RCLCPP_INFO(this->get_logger(),
                            "Rail extended, waiting for duck capture");
              }
              break;
            case IntakeRailStep::WAIT_CAPTURE:
              if (sec >= 2.0) {
                rail_step_ = IntakeRailStep::RETRACT;
                sendIntakeCommand(mcu_msgs::msg::IntakeCommand::CMD_RETRACT);
                phase_timer_ = std::chrono::steady_clock::now();
                RCLCPP_INFO(this->get_logger(),
                            "Capture window elapsed, retracting rail");
              }
              break;
            case IntakeRailStep::RETRACT:
              if (intake_position_ <= 0.05 || intake_limit_retract_ ||
                  sec >= 3.0) {
                sendIntakeCommand(
                    mcu_msgs::msg::IntakeCommand::CMD_STOP_INTAKE);
                sendIntakeCommand(
                    mcu_msgs::msg::IntakeCommand::CMD_STOP_RAIL);
                rail_step_ = IntakeRailStep::DONE;
                RCLCPP_INFO(this->get_logger(),
                            "Rail retracted, pressure clear complete");
              }
              break;
            case IntakeRailStep::DONE:
              transitionTo(MissionPhase::DEPOSIT_DUCK);
              break;
          }

          // Overall pressure solve timeout
          if (checkStateTimeout(15.0)) {
            sendIntakeCommand(mcu_msgs::msg::IntakeCommand::CMD_STOP_RAIL);
            sendIntakeCommand(mcu_msgs::msg::IntakeCommand::CMD_STOP_INTAKE);
            transitionTo(MissionPhase::DEPOSIT_DUCK);
          }
          break;
        }

        case SolveSubStep::DONE:
          transitionTo(MissionPhase::DEPOSIT_DUCK);
          break;
      }
      break;
    }

    // -- DEPOSIT DUCK (nav to target zone + eject from intake) --
    // Multi-waypoint: exit crater west, go to Lunar Landing
    case MissionPhase::DEPOSIT_DUCK: {
      if (phase_entry_) {
        deposit_step_ = DepositStep::NAV_TO_ZONE;
        RCLCPP_INFO(this->get_logger(), "Depositing duck in target zone");
        sendDrivePath({{0.25, 1.40, 0.0}, POS_DUCK_DEPOSIT});
        phase_entry_ = false;
      }

      switch (deposit_step_) {
        case DepositStep::NAV_TO_ZONE: {
          refreshDriveCommand();
          if (goal_reached_) {
            sendIntakeCommand(
                mcu_msgs::msg::IntakeCommand::CMD_SET_INTAKE_SPEED, -1.0f);
            phase_timer_ = std::chrono::steady_clock::now();
            deposit_step_ = DepositStep::EJECT;
          }
          break;
        }
        case DepositStep::EJECT: {
          auto elapsed = std::chrono::steady_clock::now() - phase_timer_;
          if (std::chrono::duration<double>(elapsed).count() >= 2.0) {
            sendIntakeCommand(mcu_msgs::msg::IntakeCommand::CMD_STOP_INTAKE);
            deposit_step_ = DepositStep::WAIT_EJECT;
            phase_timer_ = std::chrono::steady_clock::now();
          }
          break;
        }
        case DepositStep::WAIT_EJECT: {
          auto elapsed = std::chrono::steady_clock::now() - phase_timer_;
          if (std::chrono::duration<double>(elapsed).count() >= 0.5) {
            deposit_step_ = DepositStep::DONE;
          }
          break;
        }
        case DepositStep::DONE: {
          RCLCPP_INFO(this->get_logger(), "Duck deposited");
          transitionTo(MissionPhase::VIEW_LED_COLORS);
          break;
        }
      }
      if (checkStateTimeout(DEFAULT_NAV_TIMEOUT_S)) {
        sendIntakeCommand(mcu_msgs::msg::IntakeCommand::CMD_STOP_INTAKE);
        transitionTo(MissionPhase::VIEW_LED_COLORS);
      }
      break;
    }

    // -- VIEW LED COLORS (extend camera mast, read antenna LEDs) --
    case MissionPhase::VIEW_LED_COLORS: {
      if (phase_entry_) {
        led_step_ = LedReadStep::EXTEND_CAMERA;
        antennas_read_ = 0;
        antenna_colors_.fill(0);
        RCLCPP_INFO(this->get_logger(),
                    "Extending camera to read antenna LED colors");
        phase_entry_ = false;
      }

      switch (led_step_) {
        case LedReadStep::EXTEND_CAMERA: {
          sendArmCommand(JOINT_CAMERA_MAST, SERVO_EXTENDED, SERVO_SPEED);
          phase_timer_ = std::chrono::steady_clock::now();
          led_step_ = LedReadStep::READING;
          break;
        }
        case LedReadStep::READING: {
          auto elapsed = std::chrono::steady_clock::now() - phase_timer_;
          if (antennas_read_ >= 4 ||
              std::chrono::duration<double>(elapsed).count() >= 5.0) {
            RCLCPP_INFO(this->get_logger(),
                        "LED reading complete (%d/4 antennas)", antennas_read_);
            led_step_ = LedReadStep::RETRACT_CAMERA;
          }
          break;
        }
        case LedReadStep::RETRACT_CAMERA: {
          sendArmCommand(JOINT_CAMERA_MAST, SERVO_RETRACTED, SERVO_SPEED);
          phase_timer_ = std::chrono::steady_clock::now();
          led_step_ = LedReadStep::DONE;
          break;
        }
        case LedReadStep::DONE: {
          auto elapsed = std::chrono::steady_clock::now() - phase_timer_;
          if (std::chrono::duration<double>(elapsed).count() >= 1.0) {
            transitionTo(MissionPhase::NAV_TO_LAUNCH);
          }
          break;
        }
      }
      if (checkStateTimeout(10.0)) {
        sendArmCommand(JOINT_CAMERA_MAST, SERVO_RETRACTED, SERVO_SPEED);
        transitionTo(MissionPhase::NAV_TO_LAUNCH);
      }
      break;
    }

    // -- NAVIGATE TO LAUNCH POINT --
    case MissionPhase::NAV_TO_LAUNCH: {
      if (phase_entry_) {
        sendDriveGoal(POS_LAUNCH_POINT);
        phase_entry_ = false;
      } else {
        refreshDriveCommand();
      }
      if (goal_reached_) {
        transitionTo(MissionPhase::LAUNCH_DRONE);
      }
      if (checkStateTimeout(DEFAULT_NAV_TIMEOUT_S)) {
        transitionTo(MissionPhase::LAUNCH_DRONE);
      }
      break;
    }

    // -- LAUNCH DRONE (release latch + send takeoff command) --
    case MissionPhase::LAUNCH_DRONE: {
      if (phase_entry_) {
        RCLCPP_INFO(this->get_logger(), "Deploying drone!");
        sendArmCommand(JOINT_DRONE_LATCH, SERVO_EXTENDED, SERVO_SPEED);
        phase_entry_ = false;
      }
      auto elapsed = std::chrono::steady_clock::now() - phase_timer_;
      double sec = std::chrono::duration<double>(elapsed).count();

      // After 1s latch open, send takeoff command
      if (sec >= 1.0 && sec < 1.2) {
        auto cmd = mcu_msgs::msg::DroneControl();
        cmd.header.stamp = this->now();
        cmd.twist.linear.z = 1.0;
        drone_cmd_pub_->publish(cmd);
      }

      // Wait for drone to take off (3s or drone reports RUNNING)
      if (sec >= 3.0 || drone_state_ == mcu_msgs::msg::DroneState::RUNNING) {
        sendArmCommand(JOINT_DRONE_LATCH, SERVO_RETRACTED, SERVO_SPEED);
        RCLCPP_INFO(this->get_logger(), "Drone deployed, navigating to finish");
        transitionTo(MissionPhase::NAV_TO_FINISH);
      }
      if (checkStateTimeout(5.0)) {
        sendArmCommand(JOINT_DRONE_LATCH, SERVO_RETRACTED, SERVO_SPEED);
        transitionTo(MissionPhase::NAV_TO_FINISH);
      }
      break;
    }

    // -- NAVIGATE TO FINISH / PARK --
    case MissionPhase::NAV_TO_FINISH: {
      if (phase_entry_) {
        sendDriveGoal(POS_FINISH);
        phase_entry_ = false;
      } else {
        refreshDriveCommand();
      }
      if (goal_reached_) {
        // Signal minibot to return
        RCLCPP_INFO(this->get_logger(),
                    "At finish, signaling minibot to enter start zone");
        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = 0.3;
        minibot_cmd_pub_->publish(cmd);
        transitionTo(MissionPhase::MISSION_COMPLETE);
      }
      if (checkStateTimeout(DEFAULT_NAV_TIMEOUT_S)) {
        transitionTo(MissionPhase::MISSION_COMPLETE);
      }
      break;
    }

    // -- MISSION COMPLETE --
    case MissionPhase::MISSION_COMPLETE: {
      if (phase_entry_) {
        auto elapsed = std::chrono::steady_clock::now() - match_start_;
        double seconds = std::chrono::duration<double>(elapsed).count();
        RCLCPP_INFO(this->get_logger(),
                    "MISSION COMPLETE! Total time: %.1f seconds", seconds);

        // Stop everything
        stopRobot();
        auto stop = geometry_msgs::msg::Twist();
        minibot_cmd_pub_->publish(stop);
        sendTaskCommand(TASK_NONE);
        sendIntakeCommand(mcu_msgs::msg::IntakeCommand::CMD_STOP_INTAKE);
        sendIntakeCommand(mcu_msgs::msg::IntakeCommand::CMD_STOP_RAIL);

        phase_entry_ = false;
      }
      break;
    }

    // -- DUCK INTERRUPT STATES --

    // Navigate to detected duck
    case MissionPhase::DUCK_INTERRUPT_NAV: {
      if (phase_entry_) {
        sendDriveGoal(interrupt_duck_pos_);
        phase_entry_ = false;
      } else {
        refreshDriveCommand();
      }
      if (goal_reached_) {
        transitionTo(MissionPhase::DUCK_INTERRUPT_CAPTURE);
      }
      if (checkStateTimeout(DEFAULT_NAV_TIMEOUT_S)) {
        transitionTo(MissionPhase::DUCK_INTERRUPT_RESUME);
      }
      break;
    }

    // Activate intake to capture duck
    case MissionPhase::DUCK_INTERRUPT_CAPTURE: {
      if (phase_entry_) {
        sendIntakeCommand(
            mcu_msgs::msg::IntakeCommand::CMD_SET_INTAKE_SPEED, 1.0f);
        phase_entry_ = false;
      }
      auto elapsed = std::chrono::steady_clock::now() - phase_timer_;
      if (std::chrono::duration<double>(elapsed).count() >= 3.0) {
        sendIntakeCommand(mcu_msgs::msg::IntakeCommand::CMD_STOP_INTAKE);
        transitionTo(MissionPhase::DUCK_INTERRUPT_DEPOSIT_NAV);
      }
      break;
    }

    // Navigate to deposit zone
    case MissionPhase::DUCK_INTERRUPT_DEPOSIT_NAV: {
      if (phase_entry_) {
        sendDriveGoal(POS_DUCK_DEPOSIT);
        phase_entry_ = false;
      } else {
        refreshDriveCommand();
      }
      if (goal_reached_) {
        transitionTo(MissionPhase::DUCK_INTERRUPT_DEPOSIT);
      }
      if (checkStateTimeout(DEFAULT_NAV_TIMEOUT_S)) {
        transitionTo(MissionPhase::DUCK_INTERRUPT_RESUME);
      }
      break;
    }

    // Eject duck at deposit zone
    case MissionPhase::DUCK_INTERRUPT_DEPOSIT: {
      if (phase_entry_) {
        sendIntakeCommand(
            mcu_msgs::msg::IntakeCommand::CMD_SET_INTAKE_SPEED, -1.0f);
        phase_entry_ = false;
      }
      auto elapsed = std::chrono::steady_clock::now() - phase_timer_;
      if (std::chrono::duration<double>(elapsed).count() >= 2.0) {
        sendIntakeCommand(mcu_msgs::msg::IntakeCommand::CMD_STOP_INTAKE);
        transitionTo(MissionPhase::DUCK_INTERRUPT_RESUME);
      }
      break;
    }

    // Resume the saved phase (re-send nav goal by setting phase_entry_ true)
    case MissionPhase::DUCK_INTERRUPT_RESUME: {
      RCLCPP_INFO(this->get_logger(), "Duck interrupt complete, resuming %s",
                  phaseName(saved_phase_));
      phase_ = saved_phase_;
      // Always set phase_entry_=true so nav goal is re-sent
      phase_entry_ = true;
      phase_timer_ = std::chrono::steady_clock::now();
      break;
    }

  }  // end switch

  // Match time check (3 minute limit, force finish at 2:55)
  if (phase_ != MissionPhase::WAIT_FOR_START &&
      phase_ != MissionPhase::MISSION_COMPLETE) {
    auto elapsed = std::chrono::steady_clock::now() - match_start_;
    double seconds = std::chrono::duration<double>(elapsed).count();
    if (seconds > 175.0) {
      RCLCPP_WARN(this->get_logger(),
                  "MATCH TIME LIMIT! Forcing finish (%.1fs)", seconds);
      transitionTo(MissionPhase::NAV_TO_FINISH);
    }
  }
}

}  // namespace secbot

// Main entry point
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<secbot::MissionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
