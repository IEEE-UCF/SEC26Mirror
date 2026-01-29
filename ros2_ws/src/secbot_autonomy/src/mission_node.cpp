/**
 * @file mission_node.cpp
 * @author Rafeed Khan
 * @brief Implementation of the mission sequencer state machine
 *  
 * THINGS NEEDED IN REAL LIFE FOR IT TO BE ACCURATE
 * - Arena waypoint coordinates (the 7 POS_* constants in the header, currently placeholder floats)
 * - Arm joint IDs 5/6/7 (camera mast, minibot latch, drone latch), need confirmation from mechanical
 * - Light bar detection for WAIT_FOR_START (currently only checks manual btn1 press)
 */

#include "secbot_autonomy/mission_node.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace secbot {

MissionNode::MissionNode() : Node("mission_node") {
  RCLCPP_INFO(this->get_logger(), "Mission node starting...");

  // Publishers
  goal_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
  task_cmd_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
      "/autonomy/task_command", 10);
  antenna_target_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
      "/autonomy/antenna_target", 10);
  cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  minibot_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/mcu_minibot/cmd_vel", 10);
  arm_cmd_pub_ =
      this->create_publisher<mcu_msgs::msg::ArmCommand>("/arm_command", 10);
  intake_speed_pub_ =
      this->create_publisher<std_msgs::msg::Int16>("/intake_speed", 10);
  drone_cmd_pub_ = this->create_publisher<mcu_msgs::msg::DroneControl>(
      "/mcu_drone/control", 10);

  // Subscribers
  goal_reached_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/nav/goal_reached", 10,
      std::bind(&MissionNode::onGoalReached, this, _1));
  task_status_sub_ = this->create_subscription<secbot_msgs::msg::TaskStatus>(
      "/autonomy/task_status", 10,
      std::bind(&MissionNode::onTaskStatus, this, _1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&MissionNode::onOdom, this, _1));
  minibot_state_sub_ = this->create_subscription<mcu_msgs::msg::MiniRobotState>(
      "/mcu_minibot/state", 10,
      std::bind(&MissionNode::onMinibotState, this, _1));
  drone_state_sub_ = this->create_subscription<mcu_msgs::msg::DroneState>(
      "/mcu_drone/state", 10, std::bind(&MissionNode::onDroneState, this, _1));
  robot_inputs_sub_ = this->create_subscription<mcu_msgs::msg::RobotInputs>(
      "/mcu_robot/inputs", 10,
      std::bind(&MissionNode::onRobotInputs, this, _1));
  intake_state_sub_ = this->create_subscription<mcu_msgs::msg::IntakeState>(
      "/mcu_robot/intake_state", 10,
      std::bind(&MissionNode::onIntakeState, this, _1));
  duck_detect_sub_ =
      this->create_subscription<secbot_msgs::msg::DuckDetections>(
          "/duck_detections", 10,
          std::bind(&MissionNode::onDuckDetections, this, _1));
  antenna_marker_sub_ = this->create_subscription<mcu_msgs::msg::AntennaMarker>(
      "/antenna_markers", 10,
      std::bind(&MissionNode::onAntennaMarker, this, _1));

  // 10 Hz step timer
  step_timer_ = this->create_wall_timer(
      100ms, std::bind(&MissionNode::stepMission, this));

  RCLCPP_INFO(this->get_logger(),
              "Mission node ready, waiting for start signal");
}

// Callbacks

void MissionNode::onGoalReached(const std_msgs::msg::Bool::SharedPtr msg) {
  goal_reached_ = msg->data;
  if (goal_reached_) {
    RCLCPP_INFO(this->get_logger(), "Navigation goal reached!");
  }
}

void MissionNode::onTaskStatus(
    const secbot_msgs::msg::TaskStatus::SharedPtr msg) {
  current_task_id_ = msg->task_id;
  task_idle_ = (msg->task_id == TASK_NONE);
}

void MissionNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
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
  duck_in_intake_ = msg->duck_detected;
}

void MissionNode::onDuckDetections(
    const secbot_msgs::msg::DuckDetections::SharedPtr msg) {
  // Store approximate duck world position (robot position when duck is seen
  // ofc) Only stores during pre-collection NAV phases, and only high-confidence
  // detections
  for (const auto& det : msg->detections) {
    if (det.confidence < 0.7) continue;

    ArenaPosition duck_pos = {robot_x_, robot_y_};

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

    // Flag for duck interrupt (post step-9)
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

// Helpers

void MissionNode::sendNavGoal(const ArenaPosition& pos) {
  goal_reached_ = false;
  auto msg = geometry_msgs::msg::PoseStamped();
  msg.header.frame_id = "map";
  msg.header.stamp = this->now();
  msg.pose.position.x = pos.x;
  msg.pose.position.y = pos.y;
  msg.pose.orientation.w = 1.0;
  goal_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Sent nav goal: (%.2f, %.2f)", pos.x, pos.y);
}

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

void MissionNode::sendBackup(double speed, double duration_sec) {
  auto msg = geometry_msgs::msg::Twist();
  msg.linear.x = -std::abs(speed);
  cmd_vel_pub_->publish(msg);
  phase_timer_ = std::chrono::steady_clock::now();
  backup_duration_sec_ = duration_sec;
}

void MissionNode::sendArmCommand(uint8_t joint_id, int16_t position,
                                 uint8_t speed) {
  auto msg = mcu_msgs::msg::ArmCommand();
  msg.joint_id = joint_id;
  msg.position = position;
  msg.speed = speed;
  arm_cmd_pub_->publish(msg);
}

void MissionNode::setIntakeSpeed(int16_t speed) {
  auto msg = std_msgs::msg::Int16();
  msg.data = speed;
  intake_speed_pub_->publish(msg);
}

void MissionNode::transitionTo(MissionPhase phase) {
  RCLCPP_INFO(this->get_logger(), "MISSION: %s -> %s", phaseName(phase_),
              phaseName(phase));
  phase_ = phase;
  phase_entry_ = true;
}

const char* MissionNode::phaseName(MissionPhase phase) const {
  switch (phase) {
    case MissionPhase::WAIT_FOR_START:
      return "WAIT_FOR_START";
    case MissionPhase::NAV_TO_BUTTON:
      return "NAV_TO_BUTTON";
    case MissionPhase::PLACE_UWB_MODULE:
      return "PLACE_UWB_MODULE";
    case MissionPhase::ALIGN_ANTENNA_1:
      return "ALIGN_ANTENNA_1";
    case MissionPhase::SOLVE_BUTTON:
      return "SOLVE_BUTTON";
    case MissionPhase::BACKUP_FROM_BUTTON:
      return "BACKUP_FROM_BUTTON";
    case MissionPhase::NAV_TO_CRANK:
      return "NAV_TO_CRANK";
    case MissionPhase::PLACE_UWB_FLAG:
      return "PLACE_UWB_FLAG";
    case MissionPhase::ALIGN_ANTENNA_2:
      return "ALIGN_ANTENNA_2";
    case MissionPhase::SOLVE_CRANK:
      return "SOLVE_CRANK";
    case MissionPhase::COLLECT_KNOWN_DUCKS:
      return "COLLECT_KNOWN_DUCKS";
    case MissionPhase::NAV_TO_KEYPAD:
      return "NAV_TO_KEYPAD";
    case MissionPhase::ALIGN_ANTENNA_4:
      return "ALIGN_ANTENNA_4";
    case MissionPhase::SOLVE_KEYPAD:
      return "SOLVE_KEYPAD";
    case MissionPhase::NAV_TO_PRESSURE:
      return "NAV_TO_PRESSURE";
    case MissionPhase::ALIGN_ANTENNA_3:
      return "ALIGN_ANTENNA_3";
    case MissionPhase::SOLVE_PRESSURE:
      return "SOLVE_PRESSURE";
    case MissionPhase::DEPOSIT_DUCK:
      return "DEPOSIT_DUCK";
    case MissionPhase::VIEW_LED_COLORS:
      return "VIEW_LED_COLORS";
    case MissionPhase::NAV_TO_LAUNCH:
      return "NAV_TO_LAUNCH";
    case MissionPhase::LAUNCH_MINIBOT:
      return "LAUNCH_MINIBOT";
    case MissionPhase::LAUNCH_DRONE:
      return "LAUNCH_DRONE";
    case MissionPhase::NAV_TO_FINISH:
      return "NAV_TO_FINISH";
    case MissionPhase::SIGNAL_MINIBOT_ENTER:
      return "SIGNAL_MINIBOT_ENTER";
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
  // Duck interrupt check (active after COLLECT_KNOWN_DUCKS, step 9)
  if (duck_interrupt_enabled_ && pending_duck_interrupt_) {
    // Only interrupt during NAV wait phases (not mid-task or mid-backup)
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
    //  WAIT FOR START
    case MissionPhase::WAIT_FOR_START: {
      if (phase_entry_) {
        RCLCPP_INFO(this->get_logger(),
                    "Waiting for start signal (button press or light bar)...");
        phase_entry_ = false;
      }
      // Start on manual button press (btn1 from RobotInputs)
      if (start_button_pressed_) {
        match_start_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "MATCH STARTED (button press)!");
        transitionTo(MissionPhase::NAV_TO_BUTTON);
      }
      break;
    }

    //  NAVIGATE TO BUTTON BEACON
    case MissionPhase::NAV_TO_BUTTON: {
      if (phase_entry_) {
        sendNavGoal(POS_BUTTON_BEACON);
        phase_entry_ = false;
      }
      if (goal_reached_) {
        transitionTo(MissionPhase::PLACE_UWB_MODULE);
      }
      break;
    }

    //  PLACE UWB MODULE (FlagPlant task, drops UWB module via latch servo)
    case MissionPhase::PLACE_UWB_MODULE: {
      if (phase_entry_) {
        RCLCPP_INFO(this->get_logger(),
                    "Placing UWB module near button beacon");
        sendTaskCommand(TASK_FLAG_PLANT);
        phase_entry_ = false;
      }
      if (task_idle_) {
        transitionTo(MissionPhase::ALIGN_ANTENNA_1);
      }
      break;
    }

    //  ALIGN TO ANTENNA 1
    case MissionPhase::ALIGN_ANTENNA_1: {
      if (phase_entry_) {
        sendAntennaTarget(1);
        sendTaskCommand(TASK_ANTENNA_ALIGN);
        phase_entry_ = false;
      }
      if (task_idle_) {
        transitionTo(MissionPhase::SOLVE_BUTTON);
      }
      break;
    }

    //  SOLVE BUTTON (press 3x)
    case MissionPhase::SOLVE_BUTTON: {
      if (phase_entry_) {
        sendTaskCommand(TASK_BUTTON_PRESS);
        phase_entry_ = false;
      }
      if (task_idle_) {
        transitionTo(MissionPhase::BACKUP_FROM_BUTTON);
      }
      break;
    }

    //  BACKUP FROM BUTTON
    case MissionPhase::BACKUP_FROM_BUTTON: {
      if (phase_entry_) {
        sendBackup(0.3, 1.0);
        phase_entry_ = false;
      }
      auto elapsed = std::chrono::steady_clock::now() - phase_timer_;
      if (std::chrono::duration<double>(elapsed).count() >=
          backup_duration_sec_) {
        auto stop = geometry_msgs::msg::Twist();
        cmd_vel_pub_->publish(stop);
        transitionTo(MissionPhase::NAV_TO_CRANK);
      }
      break;
    }

    //  NAVIGATE TO CRANK BEACON
    case MissionPhase::NAV_TO_CRANK: {
      if (phase_entry_) {
        sendNavGoal(POS_CRANK_BEACON);
        phase_entry_ = false;
      }
      if (goal_reached_) {
        transitionTo(MissionPhase::PLACE_UWB_FLAG);
      }
      break;
    }

    //  PLACE UWB FLAG (FlagPlant task, drops UWB flag via latch servo again
    //  lawl)
    case MissionPhase::PLACE_UWB_FLAG: {
      if (phase_entry_) {
        RCLCPP_INFO(this->get_logger(), "Placing UWB flag at crank beacon");
        sendTaskCommand(TASK_FLAG_PLANT);
        phase_entry_ = false;
      }
      if (task_idle_) {
        transitionTo(MissionPhase::ALIGN_ANTENNA_2);
      }
      break;
    }

    //  ALIGN TO ANTENNA 2
    case MissionPhase::ALIGN_ANTENNA_2: {
      if (phase_entry_) {
        sendAntennaTarget(2);
        sendTaskCommand(TASK_ANTENNA_ALIGN);
        phase_entry_ = false;
      }
      if (task_idle_) {
        transitionTo(MissionPhase::SOLVE_CRANK);
      }
      break;
    }

    //  SOLVE CRANK (540 degrees)
    case MissionPhase::SOLVE_CRANK: {
      if (phase_entry_) {
        sendTaskCommand(TASK_CRANK_TURN);
        phase_entry_ = false;
      }
      if (task_idle_) {
        transitionTo(MissionPhase::COLLECT_KNOWN_DUCKS);
      }
      break;
    }

    //  COLLECT KNOWN DUCKS (multi-step loop: nav -> capture -> deposit each)
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
          sendNavGoal(known_duck_positions_[duck_collect_index_]);
          RCLCPP_INFO(this->get_logger(), "Navigating to duck %zu/%zu",
                      duck_collect_index_ + 1, known_duck_positions_.size());
          duck_collect_step_ = DuckCollectStep::INTAKE_ON;
          break;
        }
        case DuckCollectStep::INTAKE_ON: {
          if (goal_reached_) {
            setIntakeSpeed(INTAKE_CAPTURE_SPEED);
            phase_timer_ = std::chrono::steady_clock::now();
            duck_collect_step_ = DuckCollectStep::WAIT_CAPTURE;
          }
          break;
        }
        case DuckCollectStep::WAIT_CAPTURE: {
          // Wait for duck capture or 3s timeout
          auto elapsed = std::chrono::steady_clock::now() - phase_timer_;
          if (duck_in_intake_ ||
              std::chrono::duration<double>(elapsed).count() >= 3.0) {
            setIntakeSpeed(INTAKE_OFF_SPEED);
            duck_collect_step_ = DuckCollectStep::NAV_TO_DEPOSIT;
          }
          break;
        }
        case DuckCollectStep::NAV_TO_DEPOSIT: {
          sendNavGoal(POS_DUCK_DEPOSIT);
          duck_collect_step_ = DuckCollectStep::EJECT;
          break;
        }
        case DuckCollectStep::EJECT: {
          if (goal_reached_) {
            setIntakeSpeed(INTAKE_EJECT_SPEED);
            phase_timer_ = std::chrono::steady_clock::now();
            duck_collect_step_ = DuckCollectStep::WAIT_EJECT;
          }
          break;
        }
        case DuckCollectStep::WAIT_EJECT: {
          auto elapsed = std::chrono::steady_clock::now() - phase_timer_;
          if (!duck_in_intake_ ||
              std::chrono::duration<double>(elapsed).count() >= 2.0) {
            setIntakeSpeed(INTAKE_OFF_SPEED);
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
      break;
    }

    //  NAVIGATE TO KEYPAD BEACON
    case MissionPhase::NAV_TO_KEYPAD: {
      if (phase_entry_) {
        sendNavGoal(POS_KEYPAD_BEACON);
        phase_entry_ = false;
      }
      if (goal_reached_) {
        transitionTo(MissionPhase::ALIGN_ANTENNA_4);
      }
      break;
    }

    //  ALIGN TO ANTENNA 4
    case MissionPhase::ALIGN_ANTENNA_4: {
      if (phase_entry_) {
        sendAntennaTarget(4);
        sendTaskCommand(TASK_ANTENNA_ALIGN);
        phase_entry_ = false;
      }
      if (task_idle_) {
        transitionTo(MissionPhase::SOLVE_KEYPAD);
      }
      break;
    }

    //  SOLVE KEYPAD (it types "73738#")
    case MissionPhase::SOLVE_KEYPAD: {
      if (phase_entry_) {
        sendTaskCommand(TASK_KEYPAD_ENTER);
        phase_entry_ = false;
      }
      if (task_idle_) {
        transitionTo(MissionPhase::NAV_TO_PRESSURE);
      }
      break;
    }

    //  NAVIGATE TO PRESSURE PLATE
    case MissionPhase::NAV_TO_PRESSURE: {
      if (phase_entry_) {
        sendNavGoal(POS_PRESSURE_PLATE);
        phase_entry_ = false;
      }
      if (goal_reached_) {
        transitionTo(MissionPhase::ALIGN_ANTENNA_3);
      }
      break;
    }

    //  ALIGN TO ANTENNA 3
    case MissionPhase::ALIGN_ANTENNA_3: {
      if (phase_entry_) {
        sendAntennaTarget(3);
        sendTaskCommand(TASK_ANTENNA_ALIGN);
        phase_entry_ = false;
      }
      if (task_idle_) {
        transitionTo(MissionPhase::SOLVE_PRESSURE);
      }
      break;
    }

    //  SOLVE PRESSURE (sweep duck off plate into intake)
    case MissionPhase::SOLVE_PRESSURE: {
      if (phase_entry_) {
        sendTaskCommand(TASK_PRESSURE_CLEAR);
        phase_entry_ = false;
      }
      if (task_idle_) {
        transitionTo(MissionPhase::DEPOSIT_DUCK);
      }
      break;
    }

    //  DEPOSIT DUCK (nav to target zone + eject from intake)
    case MissionPhase::DEPOSIT_DUCK: {
      if (phase_entry_) {
        deposit_step_ = DepositStep::NAV_TO_ZONE;
        RCLCPP_INFO(this->get_logger(), "Depositing duck in target zone");
        sendNavGoal(POS_DUCK_DEPOSIT);
        phase_entry_ = false;
      }

      switch (deposit_step_) {
        case DepositStep::NAV_TO_ZONE: {
          if (goal_reached_) {
            setIntakeSpeed(INTAKE_EJECT_SPEED);
            phase_timer_ = std::chrono::steady_clock::now();
            deposit_step_ = DepositStep::EJECT;
          }
          break;
        }
        case DepositStep::EJECT: {
          auto elapsed = std::chrono::steady_clock::now() - phase_timer_;
          if (!duck_in_intake_ ||
              std::chrono::duration<double>(elapsed).count() >= 2.0) {
            setIntakeSpeed(INTAKE_OFF_SPEED);
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
      break;
    }

    //  VIEW LED COLORS (extend camera mast, read antenna LEDs)
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
          sendArmCommand(JOINT_CAMERA_MAST, ARM_EXTENDED, ARM_SPEED);
          phase_timer_ = std::chrono::steady_clock::now();
          led_step_ = LedReadStep::READING;
          break;
        }
        case LedReadStep::READING: {
          // Wait for all 4 antenna colors or 5s timeout
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
          sendArmCommand(JOINT_CAMERA_MAST, ARM_RETRACTED, ARM_SPEED);
          phase_timer_ = std::chrono::steady_clock::now();
          led_step_ = LedReadStep::DONE;
          break;
        }
        case LedReadStep::DONE: {
          // Brief wait for retraction
          auto elapsed = std::chrono::steady_clock::now() - phase_timer_;
          if (std::chrono::duration<double>(elapsed).count() >= 1.0) {
            transitionTo(MissionPhase::NAV_TO_LAUNCH);
          }
          break;
        }
      }
      break;
    }

    //  NAVIGATE TO LAUNCH POINT
    case MissionPhase::NAV_TO_LAUNCH: {
      if (phase_entry_) {
        sendNavGoal(POS_LAUNCH_POINT);
        phase_entry_ = false;
      }
      if (goal_reached_) {
        transitionTo(MissionPhase::LAUNCH_MINIBOT);
      }
      break;
    }

    //  LAUNCH MINIBOT (release latch + send drive command)
    case MissionPhase::LAUNCH_MINIBOT: {
      if (phase_entry_) {
        RCLCPP_INFO(this->get_logger(), "Deploying minibot!");
        // Open minibot container latch
        sendArmCommand(JOINT_MINIBOT_LATCH, ARM_EXTENDED, ARM_SPEED);
        phase_timer_ = std::chrono::steady_clock::now();
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
        sendArmCommand(JOINT_MINIBOT_LATCH, ARM_RETRACTED, ARM_SPEED);
        RCLCPP_INFO(this->get_logger(),
                    "Minibot deployed, moving to drone launch");
        transitionTo(MissionPhase::LAUNCH_DRONE);
      }
      break;
    }

    //  LAUNCH DRONE (release latch + send takeoff command)
    case MissionPhase::LAUNCH_DRONE: {
      if (phase_entry_) {
        RCLCPP_INFO(this->get_logger(), "Deploying drone!");
        // Open drone container latch
        sendArmCommand(JOINT_DRONE_LATCH, ARM_EXTENDED, ARM_SPEED);
        phase_timer_ = std::chrono::steady_clock::now();
        phase_entry_ = false;
      }
      auto elapsed = std::chrono::steady_clock::now() - phase_timer_;
      double sec = std::chrono::duration<double>(elapsed).count();

      // After 1s latch open, send takeoff command
      if (sec >= 1.0 && sec < 1.2) {
        auto cmd = mcu_msgs::msg::DroneControl();
        cmd.header.stamp = this->now();
        cmd.twist.linear.z = 1.0;  // upward velocity for takeoff
        drone_cmd_pub_->publish(cmd);
      }

      // Wait for drone to take off (3s or drone reports RUNNING)
      if (sec >= 3.0 || drone_state_ == mcu_msgs::msg::DroneState::RUNNING) {
        sendArmCommand(JOINT_DRONE_LATCH, ARM_RETRACTED, ARM_SPEED);
        RCLCPP_INFO(this->get_logger(), "Drone deployed, navigating to finish");
        transitionTo(MissionPhase::NAV_TO_FINISH);
      }
      break;
    }

    //  NAVIGATE TO FINISH / PARK
    case MissionPhase::NAV_TO_FINISH: {
      if (phase_entry_) {
        sendNavGoal(POS_FINISH);
        phase_entry_ = false;
      }
      if (goal_reached_) {
        transitionTo(MissionPhase::SIGNAL_MINIBOT_ENTER);
      }
      break;
    }

    //  SIGNAL MINIBOT TO ENTER START ZONE (drive into robot intake)
    case MissionPhase::SIGNAL_MINIBOT_ENTER: {
      if (phase_entry_) {
        RCLCPP_INFO(this->get_logger(),
                    "Signaling minibot to enter start zone");
        // Send drive command to minibot to come back into the robot
        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = 0.3;
        minibot_cmd_pub_->publish(cmd);
        phase_timer_ = std::chrono::steady_clock::now();
        phase_entry_ = false;
      }

      auto elapsed = std::chrono::steady_clock::now() - phase_timer_;
      double sec = std::chrono::duration<double>(elapsed).count();

      // Wait for minibot to board (check TASK_BOARD_ROBOT) or 5s timeout
      if (minibot_task_ == mcu_msgs::msg::MiniRobotState::TASK_BOARD_ROBOT ||
          sec >= 5.0) {
        // Stop minibot
        auto stop = geometry_msgs::msg::Twist();
        minibot_cmd_pub_->publish(stop);
        transitionTo(MissionPhase::MISSION_COMPLETE);
      }
      break;
    }

    //  MISSION COMPLETE!!!
    case MissionPhase::MISSION_COMPLETE: {
      if (phase_entry_) {
        auto elapsed = std::chrono::steady_clock::now() - match_start_;
        double seconds = std::chrono::duration<double>(elapsed).count();
        RCLCPP_INFO(this->get_logger(),
                    "MISSION COMPLETE! Total time: %.1f seconds", seconds);

        // Stop everything
        auto stop = geometry_msgs::msg::Twist();
        cmd_vel_pub_->publish(stop);
        minibot_cmd_pub_->publish(stop);
        sendTaskCommand(TASK_NONE);
        setIntakeSpeed(INTAKE_OFF_SPEED);

        phase_entry_ = false;
      }
      break;
    }

    // DUCK INTERRUPT STATES (entered when duck detected post step 9)

    //  Navigate to detected duck
    case MissionPhase::DUCK_INTERRUPT_NAV: {
      if (phase_entry_) {
        sendNavGoal(interrupt_duck_pos_);
        phase_entry_ = false;
      }
      if (goal_reached_) {
        transitionTo(MissionPhase::DUCK_INTERRUPT_CAPTURE);
      }
      break;
    }

    //  Activate intake to capture duck
    case MissionPhase::DUCK_INTERRUPT_CAPTURE: {
      if (phase_entry_) {
        setIntakeSpeed(INTAKE_CAPTURE_SPEED);
        phase_timer_ = std::chrono::steady_clock::now();
        phase_entry_ = false;
      }
      auto elapsed = std::chrono::steady_clock::now() - phase_timer_;
      if (duck_in_intake_ ||
          std::chrono::duration<double>(elapsed).count() >= 3.0) {
        setIntakeSpeed(INTAKE_OFF_SPEED);
        transitionTo(MissionPhase::DUCK_INTERRUPT_DEPOSIT_NAV);
      }
      break;
    }

    //  Navigate to deposit zone
    case MissionPhase::DUCK_INTERRUPT_DEPOSIT_NAV: {
      if (phase_entry_) {
        sendNavGoal(POS_DUCK_DEPOSIT);
        phase_entry_ = false;
      }
      if (goal_reached_) {
        transitionTo(MissionPhase::DUCK_INTERRUPT_DEPOSIT);
      }
      break;
    }

    //  Eject duck at deposit zone
    case MissionPhase::DUCK_INTERRUPT_DEPOSIT: {
      if (phase_entry_) {
        setIntakeSpeed(INTAKE_EJECT_SPEED);
        phase_timer_ = std::chrono::steady_clock::now();
        phase_entry_ = false;
      }
      auto elapsed = std::chrono::steady_clock::now() - phase_timer_;
      if (!duck_in_intake_ ||
          std::chrono::duration<double>(elapsed).count() >= 2.0) {
        setIntakeSpeed(INTAKE_OFF_SPEED);
        transitionTo(MissionPhase::DUCK_INTERRUPT_RESUME);
      }
      break;
    }

    //  Resume the saved phase
    case MissionPhase::DUCK_INTERRUPT_RESUME: {
      RCLCPP_INFO(this->get_logger(), "Duck interrupt complete, resuming %s",
                  phaseName(saved_phase_));
      phase_ = saved_phase_;
      phase_entry_ = saved_phase_entry_;
      break;
    }

  }  // end switch

  // Match time check (5 minute safety, so we force finish at 4:55)
  if (phase_ != MissionPhase::WAIT_FOR_START &&
      phase_ != MissionPhase::MISSION_COMPLETE) {
    auto elapsed = std::chrono::steady_clock::now() - match_start_;
    double seconds = std::chrono::duration<double>(elapsed).count();
    if (seconds > 295.0) {
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
