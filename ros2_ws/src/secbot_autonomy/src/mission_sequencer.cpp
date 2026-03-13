/**
 * @file mission_sequencer.cpp
 * @brief Corrected mission sequencer — turn-before-drive + coord frame fix.
 *
 * Preserves all of Rafeed's waypoints and mission logic from mission_node.cpp.
 * Every navigation step is decomposed into: TURN_TO_FACE → DRIVE →
 * TURN_TO_HEADING with pauses between each sub-step.
 *
 * Coordinate conversion:
 *   Rafeed (cm, deg): +X=north, +Y=east, +angle=CW
 *   MCU (m, rad):     +X=north, +Y=west, +angle=CCW
 *   Y and angle are negated at the boundary.
 */

#include "secbot_autonomy/mission_sequencer.hpp"

#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace secbot {

// ============================================================================
// Constructor
// ============================================================================

MissionSequencer::MissionSequencer() : Node("mission_sequencer") {
  RCLCPP_INFO(this->get_logger(), "Mission sequencer starting...");

  // Publishers
  drive_cmd_pub_ = this->create_publisher<mcu_msgs::msg::DriveBase>(
      "drive_base/command", 10);
  task_cmd_pub_ =
      this->create_publisher<std_msgs::msg::UInt8>("/autonomy/task_command", 10);
  minibot_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/mcu_minibot/cmd_vel", 10);
  arm_cmd_pub_ = this->create_publisher<mcu_msgs::msg::ArmCommand>(
      "/arm_command", 10);
  intake_cmd_pub_ = this->create_publisher<mcu_msgs::msg::IntakeCommand>(
      "/mcu_robot/intake/command", 10);
  pose_reset_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(
      "drive_base/reset_pose", 10);

  // Subscribers — drive_base/status MUST use SensorDataQoS (best-effort)
  drive_status_sub_ = this->create_subscription<mcu_msgs::msg::DriveBase>(
      "drive_base/status", rclcpp::SensorDataQoS(),
      std::bind(&MissionSequencer::onDriveStatus, this, _1));

  task_status_sub_ = this->create_subscription<secbot_msgs::msg::TaskStatus>(
      "/autonomy/task_status", 10,
      std::bind(&MissionSequencer::onTaskStatus, this, _1));

  robot_inputs_sub_ = this->create_subscription<mcu_msgs::msg::RobotInputs>(
      "/mcu_robot/inputs", 10,
      std::bind(&MissionSequencer::onRobotInputs, this, _1));

  intake_state_sub_ = this->create_subscription<mcu_msgs::msg::IntakeState>(
      "/mcu_robot/intake/state", rclcpp::SensorDataQoS(),
      std::bind(&MissionSequencer::onIntakeState, this, _1));

  // Start service
  start_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/mission/start",
      std::bind(&MissionSequencer::onStartService, this, _1,
                std::placeholders::_2));

  // IMU tare service client
  imu_tare_client_ =
      this->create_client<mcu_msgs::srv::Reset>("/mcu_robot/imu/tare");

  // ReadBeaconColor action client
  beacon_color_client_ =
      rclcpp_action::create_client<ReadBeaconColor>(this, "read_beacon_color");

  // Main tick timer at 10 Hz
  tick_timer_ = this->create_wall_timer(100ms, [this]() { stepMission(); });

  RCLCPP_INFO(this->get_logger(),
              "Mission sequencer ready. Waiting for start...");
}

// ============================================================================
// Setup state machine (IMU tare → pose reset → settle)
// ============================================================================

void MissionSequencer::startSetup(double start_x_cm, double start_y_cm,
                                  double start_yaw_deg) {
  setup_pose_ = rafeedToMcu(start_x_cm, start_y_cm, start_yaw_deg);
  setup_state_ = SetupState::TARE_WAIT;
  tare_sent_ = false;
  RCLCPP_INFO(this->get_logger(), "[SETUP] Starting: tare → reset → settle");
}

bool MissionSequencer::tickSetup() {
  switch (setup_state_) {
    case SetupState::IDLE:
      return false;

    case SetupState::TARE_WAIT: {
      if (!tare_sent_) {
        if (!imu_tare_client_->wait_for_service(0s)) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                               "[SETUP] Waiting for /mcu_robot/imu/tare...");
          return false;
        }
        auto req = std::make_shared<mcu_msgs::srv::Reset::Request>();
        tare_future_ = imu_tare_client_->async_send_request(req);
        tare_sent_ = true;
        RCLCPP_INFO(this->get_logger(), "[SETUP] IMU tare request sent");
      }

      if (tare_future_.wait_for(0s) == std::future_status::ready) {
        auto result = tare_future_.get();
        if (result->success) {
          RCLCPP_INFO(this->get_logger(), "[SETUP] IMU tare SUCCESS");
        } else {
          RCLCPP_WARN(this->get_logger(), "[SETUP] IMU tare returned false");
        }
        setup_state_ = SetupState::POSE_RESET;
      }
      return false;
    }

    case SetupState::POSE_RESET: {
      resetPoseMcu(setup_pose_.x_m, setup_pose_.y_m, setup_pose_.theta_rad);
      RCLCPP_INFO(this->get_logger(),
                  "[SETUP] Pose reset to (%.3f, %.3f, %.3f rad)",
                  setup_pose_.x_m, setup_pose_.y_m, setup_pose_.theta_rad);
      settle_start_ = std::chrono::steady_clock::now();
      setup_state_ = SetupState::SETTLE;
      return false;
    }

    case SetupState::SETTLE: {
      double elapsed =
          std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                        settle_start_)
              .count();
      if (elapsed >= SETUP_SETTLE_S) {
        setup_state_ = SetupState::DONE;
        RCLCPP_INFO(this->get_logger(), "[SETUP] Complete");
      }
      return false;
    }

    case SetupState::DONE:
      return true;
  }
  return false;
}

// ============================================================================
// NavMove executor — decompose every goal into TURN → DRIVE → TURN
// ============================================================================

void MissionSequencer::startNavMove(double x_cm, double y_cm,
                                    double heading_deg, bool reverse) {
  nav_target_ = rafeedToMcu(x_cm, y_cm, heading_deg);
  nav_final_heading_ = nav_target_.theta_rad;
  nav_reverse_ = reverse;

  // Compute distance to target in MCU coords
  double dx = nav_target_.x_m - robot_x_m_;
  double dy = nav_target_.y_m - robot_y_m_;
  double dist = std::hypot(dx, dy);

  if (dist < MIN_DRIVE_DIST_M) {
    // Pure turn — skip drive, go straight to final heading
    nav_is_pure_turn_ = true;
    nav_travel_heading_ = nav_final_heading_;
    RCLCPP_INFO(this->get_logger(),
                "  NavMove: pure turn to %.1f deg (dist=%.3f m < threshold)",
                rad2deg(nav_final_heading_), dist);
    advanceNavSubStep(NavSubStep::TURN_TO_HEADING);
  } else {
    // Compute travel heading (direction to face the target)
    nav_travel_heading_ = std::atan2(dy, dx);
    if (reverse) {
      nav_travel_heading_ = normalizeAngle(nav_travel_heading_ + M_PI);
    }
    nav_is_pure_turn_ = false;
    RCLCPP_INFO(this->get_logger(),
                "  NavMove: (%.0f,%.0f,%.0f%s) → MCU(%.3f,%.3f) "
                "travel_hdg=%.1f° final_hdg=%.1f° dist=%.3f m",
                x_cm, y_cm, heading_deg, reverse ? " REV" : "",
                nav_target_.x_m, nav_target_.y_m,
                rad2deg(nav_travel_heading_), rad2deg(nav_final_heading_),
                dist);
    advanceNavSubStep(NavSubStep::TURN_TO_FACE);
  }
}

void MissionSequencer::tickNavMove() {
  switch (nav_sub_step_) {
    case NavSubStep::IDLE:
    case NavSubStep::DONE:
      return;

    case NavSubStep::TURN_TO_FACE: {
      // P-controller turn via DRIVE_VECTOR — no GOAL mode position correction
      if (turnReachedHeading(nav_travel_heading_)) {
        RCLCPP_INFO(this->get_logger(), "  NavMove: TURN_TO_FACE complete");
        advanceNavSubStep(NavSubStep::PAUSE);  // stopRobot() called inside
      } else {
        sendTurnInPlace(nav_travel_heading_);
        double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                          nav_substep_timer_)
                .count();
        if (elapsed > TURN_TIMEOUT_S) {
          RCLCPP_WARN(this->get_logger(),
                      "  NavMove: TURN_TO_FACE timed out (%.1f s)", elapsed);
          advanceNavSubStep(NavSubStep::PAUSE);  // stopRobot() called inside
        }
      }
      break;
    }

    case NavSubStep::DRIVE: {
      if (!goal_reached_) {
        double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                          nav_substep_timer_)
                .count();
        if (elapsed > DRIVE_TIMEOUT_S) {
          RCLCPP_WARN(this->get_logger(),
                      "  NavMove: DRIVE timed out (%.1f s)", elapsed);
          // Check if we need final heading turn
          double hdg_err =
              std::abs(normalizeAngle(nav_final_heading_ - nav_travel_heading_));
          if (hdg_err > HEADING_MATCH_RAD) {
            advanceNavSubStep(NavSubStep::PAUSE);
          } else {
            advanceNavSubStep(NavSubStep::DONE);
          }
        }
      } else {
        RCLCPP_INFO(this->get_logger(), "  NavMove: DRIVE complete");
        double hdg_err =
            std::abs(normalizeAngle(nav_final_heading_ - nav_travel_heading_));
        if (hdg_err > HEADING_MATCH_RAD) {
          advanceNavSubStep(NavSubStep::PAUSE);
        } else {
          advanceNavSubStep(NavSubStep::DONE);
        }
      }
      break;
    }

    case NavSubStep::TURN_TO_HEADING: {
      // P-controller turn via DRIVE_VECTOR
      if (turnReachedHeading(nav_final_heading_)) {
        RCLCPP_INFO(this->get_logger(), "  NavMove: TURN_TO_HEADING complete");
        advanceNavSubStep(NavSubStep::DONE);  // stopRobot() called inside
      } else {
        sendTurnInPlace(nav_final_heading_);
        double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                          nav_substep_timer_)
                .count();
        if (elapsed > TURN_TIMEOUT_S) {
          RCLCPP_WARN(this->get_logger(),
                      "  NavMove: TURN_TO_HEADING timed out (%.1f s)", elapsed);
          advanceNavSubStep(NavSubStep::DONE);  // stopRobot() called inside
        }
      }
      break;
    }

    case NavSubStep::PAUSE: {
      double elapsed =
          std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                        nav_substep_timer_)
              .count();
      if (elapsed >= INTER_MOVE_PAUSE_S) {
        // Determine what comes after this pause
        // We track where we were before PAUSE using a simple progression:
        // TURN_TO_FACE → (pause) → DRIVE
        // DRIVE → (pause) → TURN_TO_HEADING
        // TURN_TO_HEADING → (pause) → DONE  (shouldn't happen, but safe)
        //
        // We detect which transition based on what hasn't been done yet.
        // After TURN_TO_FACE pause, send DRIVE command.
        // After DRIVE pause, send TURN_TO_HEADING command.
        if (!nav_is_pure_turn_ && nav_pause_next_ == NavSubStep::DRIVE) {
          // Start drive
          goal_reached_ = false;
          goal_x_m_ = nav_target_.x_m;
          goal_y_m_ = nav_target_.y_m;
          goal_theta_rad_ = nav_travel_heading_;
          sendGoalMcu(nav_target_.x_m, nav_target_.y_m, nav_travel_heading_,
                      nav_reverse_);
          nav_sub_step_ = NavSubStep::DRIVE;
          nav_substep_timer_ = std::chrono::steady_clock::now();
          RCLCPP_INFO(this->get_logger(), "  NavMove: → DRIVE");
        } else if (nav_pause_next_ == NavSubStep::TURN_TO_HEADING) {
          // Check if heading turn is even needed
          double hdg_err = std::abs(
              normalizeAngle(nav_final_heading_ - robot_theta_rad_));
          if (hdg_err < HEADING_TOL_RAD) {
            RCLCPP_INFO(this->get_logger(),
                        "  NavMove: skip TURN_TO_HEADING (err=%.1f deg < tol)",
                        rad2deg(hdg_err));
            nav_sub_step_ = NavSubStep::DONE;
          } else {
            // Enter turn sub-step — P-controller runs on next tick
            goal_reached_ = false;  // clear stale flag from DRIVE
            nav_sub_step_ = NavSubStep::TURN_TO_HEADING;
            nav_substep_timer_ = std::chrono::steady_clock::now();
            RCLCPP_INFO(this->get_logger(), "  NavMove: → TURN_TO_HEADING");
          }
        } else {
          nav_sub_step_ = NavSubStep::DONE;
        }
      }
      break;
    }
  }
}

void MissionSequencer::advanceNavSubStep(NavSubStep next) {
  stopRobot();

  if (next == NavSubStep::TURN_TO_FACE) {
    // Check if we even need to turn — if already facing travel heading
    double hdg_err =
        std::abs(normalizeAngle(nav_travel_heading_ - robot_theta_rad_));
    if (hdg_err < HEADING_TOL_RAD) {
      RCLCPP_INFO(this->get_logger(),
                  "  NavMove: skip TURN_TO_FACE (err=%.1f deg < tol)",
                  rad2deg(hdg_err));
      nav_pause_next_ = NavSubStep::DRIVE;
      nav_sub_step_ = NavSubStep::PAUSE;
      nav_substep_timer_ = std::chrono::steady_clock::now();
      return;
    }
    // Enter turn sub-step — tickNavMove() runs P-controller via DRIVE_VECTOR
    nav_sub_step_ = NavSubStep::TURN_TO_FACE;
    nav_substep_timer_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "  NavMove: → TURN_TO_FACE (%.1f deg)",
                rad2deg(nav_travel_heading_));
  } else if (next == NavSubStep::PAUSE) {
    // Figure out what comes after this pause
    if (nav_sub_step_ == NavSubStep::TURN_TO_FACE) {
      nav_pause_next_ = NavSubStep::DRIVE;
    } else if (nav_sub_step_ == NavSubStep::DRIVE) {
      nav_pause_next_ = NavSubStep::TURN_TO_HEADING;
    } else {
      nav_pause_next_ = NavSubStep::DONE;
    }
    nav_sub_step_ = NavSubStep::PAUSE;
    nav_substep_timer_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "  NavMove: → PAUSE (%.1f s)",
                INTER_MOVE_PAUSE_S);
  } else if (next == NavSubStep::TURN_TO_HEADING) {
    // Check if heading turn is needed
    double hdg_err =
        std::abs(normalizeAngle(nav_final_heading_ - robot_theta_rad_));
    if (hdg_err < HEADING_TOL_RAD) {
      RCLCPP_INFO(this->get_logger(),
                  "  NavMove: skip TURN_TO_HEADING (err=%.1f deg < tol)",
                  rad2deg(hdg_err));
      nav_sub_step_ = NavSubStep::DONE;
      return;
    }
    // Enter turn sub-step — tickNavMove() runs P-controller via DRIVE_VECTOR
    nav_sub_step_ = NavSubStep::TURN_TO_HEADING;
    nav_substep_timer_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(),
                "  NavMove: → TURN_TO_HEADING (%.1f deg)",
                rad2deg(nav_final_heading_));
  } else if (next == NavSubStep::DONE) {
    nav_sub_step_ = NavSubStep::DONE;
    RCLCPP_INFO(this->get_logger(), "  NavMove: DONE");
  }
}

// ============================================================================
// Low-level MCU command helpers
// ============================================================================

void MissionSequencer::sendGoalMcu(double x_m, double y_m, double theta_rad,
                                   bool reverse) {
  auto msg = mcu_msgs::msg::DriveBase();
  msg.drive_mode = mcu_msgs::msg::DriveBase::DRIVE_GOAL;
  msg.goal_transform.transform.translation.x = x_m;
  msg.goal_transform.transform.translation.y = y_m;
  msg.goal_transform.transform.translation.z = reverse ? -1.0 : 0.0;

  float half = static_cast<float>(theta_rad) * 0.5f;
  msg.goal_transform.transform.rotation.x = 0.0;
  msg.goal_transform.transform.rotation.y = 0.0;
  msg.goal_transform.transform.rotation.z = std::sin(half);
  msg.goal_transform.transform.rotation.w = std::cos(half);

  last_drive_cmd_ = msg;
  last_drive_mode_ = mcu_msgs::msg::DriveBase::DRIVE_GOAL;
  drive_cmd_pub_->publish(msg);
}

void MissionSequencer::sendTurnInPlace(double target_rad) {
  double err = normalizeAngle(target_rad - robot_theta_rad_);
  double omega = TURN_KP * err;

  // Clamp magnitude
  if (omega > TURN_MAX_OMEGA) omega = TURN_MAX_OMEGA;
  if (omega < -TURN_MAX_OMEGA) omega = -TURN_MAX_OMEGA;

  // Apply minimum omega to overcome static friction (keep sign)
  if (std::abs(omega) < TURN_MIN_OMEGA && std::abs(err) > 0.01) {
    omega = (err > 0) ? TURN_MIN_OMEGA : -TURN_MIN_OMEGA;
  }

  sendVelocity(0.0, omega);
}

bool MissionSequencer::turnReachedHeading(double target_rad) const {
  return std::abs(normalizeAngle(target_rad - robot_theta_rad_)) <
         HEADING_TOL_RAD;
}

void MissionSequencer::sendVelocity(double vx_mps, double omega_radps) {
  auto msg = mcu_msgs::msg::DriveBase();
  msg.drive_mode = mcu_msgs::msg::DriveBase::DRIVE_VECTOR;
  msg.goal_velocity.linear.x = vx_mps;
  msg.goal_velocity.angular.z = omega_radps;

  last_drive_cmd_ = msg;
  last_drive_mode_ = mcu_msgs::msg::DriveBase::DRIVE_VECTOR;
  drive_cmd_pub_->publish(msg);
}

void MissionSequencer::sendNudge(double speed_mps) {
  RCLCPP_INFO(this->get_logger(), "  sendNudge(%.2f m/s)", speed_mps);
  sendVelocity(speed_mps, 0.0);
}

void MissionSequencer::stopRobot() { sendVelocity(0.0, 0.0); }

void MissionSequencer::refreshDriveCommand() {
  // Don't re-publish during pause — robot should be stopped
  if (nav_sub_step_ == NavSubStep::PAUSE) return;

  // Stop re-publishing once a DRIVE_GOAL is reached
  if (goal_reached_ &&
      last_drive_mode_ == mcu_msgs::msg::DriveBase::DRIVE_GOAL) {
    return;
  }
  if (last_drive_mode_ != 0) {
    drive_cmd_pub_->publish(last_drive_cmd_);
  }
}

void MissionSequencer::resetPoseMcu(double x_m, double y_m,
                                    double theta_rad) {
  auto msg = geometry_msgs::msg::Pose();
  msg.position.x = x_m;
  msg.position.y = y_m;
  msg.position.z = 0.0;

  float half = static_cast<float>(theta_rad) * 0.5f;
  msg.orientation.x = 0.0;
  msg.orientation.y = 0.0;
  msg.orientation.z = std::sin(half);
  msg.orientation.w = std::cos(half);

  pose_reset_pub_->publish(msg);
}

// ============================================================================
// Task / arm / intake helpers (unchanged from Rafeed)
// ============================================================================

void MissionSequencer::sendTaskCommand(uint8_t task_id) {
  auto msg = std_msgs::msg::UInt8();
  msg.data = task_id;
  task_cmd_pub_->publish(msg);
  task_idle_ = false;
  RCLCPP_INFO(this->get_logger(), "  sendTaskCommand(%d)", task_id);
}

void MissionSequencer::sendArmCommand(uint8_t joint_id, int16_t position,
                                      uint8_t speed) {
  auto msg = mcu_msgs::msg::ArmCommand();
  msg.joint_id = joint_id;
  msg.position = position;
  msg.speed = speed;
  arm_cmd_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "  sendArmCommand(joint=%d, pos=%d, spd=%d)",
              joint_id, position, speed);
}

void MissionSequencer::sendIntakeCommand(uint8_t cmd, float value) {
  auto msg = mcu_msgs::msg::IntakeCommand();
  msg.command = cmd;
  msg.value = value;
  intake_cmd_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "  sendIntakeCommand(cmd=%d, val=%.2f)", cmd,
              value);
}

void MissionSequencer::startAntennaRead(const char* label,
                                        float camera_angle) {
  antenna_read_active_ = true;
  last_antenna_color_ = "unknown";

  if (!beacon_color_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(),
                "  ReadBeaconColor action server not available, "
                "skipping %s antenna read",
                label);
    antenna_read_active_ = false;
    return;
  }

  auto goal = ReadBeaconColor::Goal();
  goal.camera_angle_goal = camera_angle;

  auto options = rclcpp_action::Client<ReadBeaconColor>::SendGoalOptions();
  options.result_callback =
      [this, lbl = std::string(label)](
          const BeaconGoalHandle::WrappedResult& result) {
        antenna_read_active_ = false;
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          last_antenna_color_ = result.result->color;
          RCLCPP_INFO(this->get_logger(), "  Antenna %s read: %s", lbl.c_str(),
                      last_antenna_color_.c_str());
        } else {
          last_antenna_color_ = "unknown";
          RCLCPP_WARN(this->get_logger(),
                      "  Antenna %s read failed (code=%d)", lbl.c_str(),
                      static_cast<int>(result.code));
        }
      };

  beacon_color_client_->async_send_goal(goal, options);
  RCLCPP_INFO(this->get_logger(),
              "  Reading %s antenna at camera angle %.1f deg", label,
              camera_angle);
}

// ============================================================================
// State machine tick
// ============================================================================

void MissionSequencer::stepMission() {
  // Refresh last drive command every tick to prevent MCU 500ms timeout
  refreshDriveCommand();

  switch (phase_) {
    // ── WAIT_FOR_START ───────────────────────────────────────────────
    case MissionPhase::WAIT_FOR_START: {
      if (phase_entry_) {
        phase_entry_ = false;
        RCLCPP_INFO(this->get_logger(), "=== WAITING FOR START ===");
      }
      if (start_button_pressed_) {
        transitionTo(MissionPhase::ROBOT_START);
      }
      break;
    }

    // ── ROBOT_START: IMU tare → pose reset → settle ─────────────────
    case MissionPhase::ROBOT_START: {
      if (phase_entry_) {
        phase_entry_ = false;
        RCLCPP_INFO(this->get_logger(), "=== ROBOT START ===");
        match_start_ = std::chrono::steady_clock::now();
        // Start setup: tare IMU, reset pose to (15, 15, 0) in Rafeed coords
        startSetup(15.0, 15.0, 0.0);
      }
      if (tickSetup()) {
        transitionTo(MissionPhase::PHASE_BUTTON);
      }
      break;
    }

    // ── PHASE_BUTTON: 14 steps ───────────────────────────────────────
    case MissionPhase::PHASE_BUTTON: {
      if (phase_entry_) {
        phase_entry_ = false;
        setStep(1);
        RCLCPP_INFO(this->get_logger(), "=== PHASE 1: BUTTON TASK ===");
      }

      switch (step_) {
        // Step 1: Start position (15, 15, 0) — already here
        case 1: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "BUTTON Step 1: Start at (15, 15, 0)");
          }
          setStep(2);
          break;
        }

        // Step 2: Drive to UWB placement (108, 38, 0)
        case 2: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "BUTTON Step 2: Drive to UWB placement (108, 38, 0)");
            startNavMove(108.0, 38.0, 0.0);
          }
          tickNavMove();
          if (navMoveComplete() || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
            setStep(3);
          }
          break;
        }

        // Step 3: Back up [rev] to (38, 38, 0)
        case 3: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "BUTTON Step 3: Reverse to (38, 38, 0)");
            startNavMove(38.0, 38.0, 0.0, true);
          }
          tickNavMove();
          if (navMoveComplete() || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
            setStep(4);
          }
          break;
        }

        // Step 4: Turn -45 degrees at (38, 38, -45)
        case 4: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "BUTTON Step 4: Turn to -45 deg at (38, 38, -45)");
            startNavMove(38.0, 38.0, -45.0);
          }
          tickNavMove();
          if (navMoveComplete() || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
            setStep(5);
          }
          break;
        }

        // Step 5: Drive forward to (54, 27, -45)
        case 5: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "BUTTON Step 5: Drive to (54, 27, -45)");
            startNavMove(54.0, 27.0, -45.0);
          }
          tickNavMove();
          if (navMoveComplete() || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
            setStep(6);
          }
          break;
        }

        // Step 6: Turn +45 at (54, 27, 0)
        case 6: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "BUTTON Step 6: Turn to 0 deg at (54, 27, 0)");
            startNavMove(54.0, 27.0, 0.0);
          }
          tickNavMove();
          if (navMoveComplete() || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
            setStep(7);
          }
          break;
        }

        // Step 7: Drive to button (78, 27, 0)
        case 7: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "BUTTON Step 7: Drive to button (78, 27, 0)");
            startNavMove(78.0, 27.0, 0.0);
          }
          tickNavMove();
          if (navMoveComplete() || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
            setStep(8);
          }
          break;
        }

        // Step 8: Nudge forward (tap button #1)
        case 8: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "BUTTON Step 8: Nudge forward (tap #1)");
            sendNudge(NUDGE_SPEED_MPS);
          }
          if (stepElapsed() > 1.0) {
            stopRobot();
            setStep(9);
          }
          break;
        }

        // Step 9: Back up [rev] to (78, 27, 0)
        case 9: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "BUTTON Step 9: Reverse to (78, 27, 0)");
            startNavMove(78.0, 27.0, 0.0, true);
          }
          tickNavMove();
          if (navMoveComplete() || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
            setStep(10);
          }
          break;
        }

        // Step 10: Nudge forward (tap button #2)
        case 10: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "BUTTON Step 10: Nudge forward (tap #2)");
            sendNudge(NUDGE_SPEED_MPS);
          }
          if (stepElapsed() > 1.0) {
            stopRobot();
            setStep(11);
          }
          break;
        }

        // Step 11: Back up [rev] to (78, 28, 0)
        case 11: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "BUTTON Step 11: Reverse to (78, 28, 0)");
            startNavMove(78.0, 28.0, 0.0, true);
          }
          tickNavMove();
          if (navMoveComplete() || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
            setStep(12);
          }
          break;
        }

        // Step 12: Nudge forward (tap button #3)
        case 12: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "BUTTON Step 12: Nudge forward (tap #3)");
            sendNudge(NUDGE_SPEED_MPS);
          }
          if (stepElapsed() > 1.0) {
            stopRobot();
            setStep(13);
          }
          break;
        }

        // Step 13: Back up [rev] to (30, 27, 0)
        case 13: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "BUTTON Step 13: Reverse to (30, 27, 0)");
            startNavMove(30.0, 27.0, 0.0, true);
          }
          tickNavMove();
          if (navMoveComplete() || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
            setStep(14);
          }
          break;
        }

        // Step 14: Read beacon antenna
        case 14: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "BUTTON Step 14: Read beacon antenna");
            startAntennaRead("beacon", CAMERA_ANGLE_DEFAULT);
          }
          if (antennaReadComplete() || checkStepTimeout(10.0)) {
            antenna_colors_[0] = last_antenna_color_;
            RCLCPP_INFO(this->get_logger(),
                        "=== PHASE 1 COMPLETE: Button task done ===");
            transitionTo(MissionPhase::PHASE_KEYPAD_CRATER);
          }
          break;
        }

        default:
          RCLCPP_WARN(this->get_logger(), "BUTTON: Unknown step %d", step_);
          transitionTo(MissionPhase::PHASE_KEYPAD_CRATER);
          break;
      }
      break;
    }

    // ── PHASE_KEYPAD_CRATER: 22 steps ────────────────────────────────
    case MissionPhase::PHASE_KEYPAD_CRATER: {
      if (phase_entry_) {
        phase_entry_ = false;
        setStep(1);
        RCLCPP_INFO(this->get_logger(),
                    "=== PHASE 2: KEYPAD + CRATER + CRANK ===");
      }

      switch (step_) {
        // Step 1: Turn +45 at (30, 27, 45)
        case 1: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "KEYPAD Step 1: Turn to 45 deg at (30, 27, 45)");
            startNavMove(30.0, 27.0, 45.0);
          }
          tickNavMove();
          if (navMoveComplete() || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
            setStep(2);
          }
          break;
        }

        // Step 2: Drive to keypad area (81, 95, 45)
        case 2: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "KEYPAD Step 2: Drive to (81, 95, 45)");
            startNavMove(81.0, 95.0, 45.0);
          }
          tickNavMove();
          if (navMoveComplete() || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
            setStep(3);
          }
          break;
        }

        // Step 3: Turn to 0 at (81, 95, 0)
        case 3: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "KEYPAD Step 3: Turn to 0 deg at (81, 95, 0)");
            startNavMove(81.0, 95.0, 0.0);
          }
          tickNavMove();
          if (navMoveComplete() || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
            setStep(4);
          }
          break;
        }

        // Step 4: Reverse to keypad [rev] (38, 95, 0)
        case 4: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "KEYPAD Step 4: Reverse to keypad (38, 95, 0)");
            startNavMove(38.0, 95.0, 0.0, true);
          }
          tickNavMove();
          if (navMoveComplete() || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
            setStep(5);
          }
          break;
        }

        // Step 5: Call keypad action
        case 5: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "KEYPAD Step 5: Execute keypad task");
            sendTaskCommand(TASK_KEYPAD_ENTER);
          }
          if (task_idle_ || checkStepTimeout(30.0)) {
            setStep(6);
          }
          break;
        }

        // Step 6: Drive forward to (55, 95, 0)
        case 6: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "KEYPAD Step 6: Drive forward to (55, 95, 0)");
            startNavMove(55.0, 95.0, 0.0);
          }
          tickNavMove();
          if (navMoveComplete() || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
            setStep(7);
          }
          break;
        }

        // Step 7: Turn to 90 deg at (55, 95, 90)
        case 7: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "KEYPAD Step 7: Turn to 90 deg at (55, 95, 90)");
            startNavMove(55.0, 95.0, 90.0);
          }
          tickNavMove();
          if (navMoveComplete() || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
            setStep(8);
          }
          break;
        }

        // Step 8: Launch minibot
        case 8: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(), "KEYPAD Step 8: Launch minibot");
            sendArmCommand(JOINT_MINIBOT_LATCH, SERVO_EXTENDED, SERVO_SPEED);
          }
          if (stepElapsed() > 1.0) {
            setStep(9);
          }
          break;
        }

        // Step 9: Wait for minibot deploy
        case 9: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "KEYPAD Step 9: Waiting for minibot deploy");
          }
          if (stepElapsed() > 1.0) {
            setStep(10);
          }
          break;
        }

        // Step 10: Deploy intake extender
        case 10: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "KEYPAD Step 10: Deploy intake extender");
            sendIntakeCommand(mcu_msgs::msg::IntakeCommand::CMD_EXTEND);
          }
          if (intake_state_ == mcu_msgs::msg::IntakeState::STATE_IDLE ||
              checkStepTimeout(10.0)) {
            setStep(11);
          }
          break;
        }

        // Step 11: Turn to 180 deg at (55, 95, 180)
        case 11: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "KEYPAD Step 11: Turn to 180 deg at (55, 95, 180)");
            startNavMove(55.0, 95.0, 180.0);
          }
          tickNavMove();
          if (navMoveComplete() || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
            setStep(12);
          }
          break;
        }

        // Step 12: Reverse [rev] to (91, 95, 180)
        case 12: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "KEYPAD Step 12: Reverse to (91, 95, 180)");
            startNavMove(91.0, 95.0, 180.0, true);
          }
          tickNavMove();
          if (navMoveComplete() || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
            setStep(13);
          }
          break;
        }

        // Step 13: Read keypad antenna
        case 13: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "KEYPAD Step 13: Read keypad antenna");
            startAntennaRead("keypad", CAMERA_ANGLE_DEFAULT);
          }
          if (antennaReadComplete() || checkStepTimeout(10.0)) {
            antenna_colors_[1] = last_antenna_color_;
            setStep(14);
          }
          break;
        }

        // Step 14: Turn to 115 deg at (91, 95, 115)
        case 14: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "KEYPAD Step 14: Turn to 115 deg at (91, 95, 115)");
            startNavMove(91.0, 95.0, 115.0);
          }
          tickNavMove();
          if (navMoveComplete() || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
            setStep(15);
          }
          break;
        }

        // Step 15: Read crater antenna
        case 15: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "KEYPAD Step 15: Read crater antenna");
            startAntennaRead("crater", CAMERA_ANGLE_DEFAULT);
          }
          if (antennaReadComplete() || checkStepTimeout(10.0)) {
            antenna_colors_[2] = last_antenna_color_;
            setStep(16);
          }
          break;
        }

        // Step 16: Drive to wall alignment (90, 110, 45)
        case 16: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "KEYPAD Step 16: Wall alignment (90, 110, 45)");
            startNavMove(90.0, 110.0, 45.0);
          }
          tickNavMove();
          if (navMoveComplete() || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
            setStep(17);
          }
          break;
        }

        // Step 17: Safe path up (100, 190, 90)
        case 17: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "KEYPAD Step 17: Safe path to (100, 190, 90)");
            startNavMove(100.0, 190.0, 90.0);
          }
          tickNavMove();
          if (navMoveComplete() || checkStepTimeout(20.0)) {
            setStep(18);
          }
          break;
        }

        // Step 18: Align with crank (53, 225, 180)
        case 18: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "KEYPAD Step 18: Align with crank (53, 225, 180)");
            startNavMove(53.0, 225.0, 180.0);
          }
          tickNavMove();
          if (navMoveComplete() || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
            setStep(19);
          }
          break;
        }

        // Step 19: Reverse to crank [rev] (53, 225, 150)
        case 19: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "KEYPAD Step 19: Reverse to crank (53, 225, 150)");
            startNavMove(53.0, 225.0, 150.0, true);
          }
          tickNavMove();
          if (navMoveComplete() || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
            setStep(20);
          }
          break;
        }

        // Step 20: Adjust position (53, 228, 180)
        case 20: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "KEYPAD Step 20: Fine adjust (53, 228, 180)");
            startNavMove(53.0, 228.0, 180.0);
          }
          tickNavMove();
          if (navMoveComplete() || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
            setStep(21);
          }
          break;
        }

        // Step 21: Turn to 0 deg (53, 228, 0) — spin 180
        case 21: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "KEYPAD Step 21: Turn to 0 deg at (53, 228, 0)");
            startNavMove(53.0, 228.0, 0.0);
          }
          tickNavMove();
          if (navMoveComplete() || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
            setStep(22);
          }
          break;
        }

        // Step 22: Read crank antenna
        case 22: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "KEYPAD Step 22: Read crank antenna");
            startAntennaRead("crank", CAMERA_ANGLE_DEFAULT);
          }
          if (antennaReadComplete() || checkStepTimeout(10.0)) {
            antenna_colors_[3] = last_antenna_color_;
            RCLCPP_INFO(
                this->get_logger(),
                "=== PHASE 2 COMPLETE: Keypad + Crater + Crank done ===");
            transitionTo(MissionPhase::MISSION_COMPLETE);
          }
          break;
        }

        default:
          RCLCPP_WARN(this->get_logger(), "KEYPAD: Unknown step %d", step_);
          transitionTo(MissionPhase::MISSION_COMPLETE);
          break;
      }
      break;
    }

    // ── MISSION_COMPLETE ─────────────────────────────────────────────
    case MissionPhase::MISSION_COMPLETE: {
      if (phase_entry_) {
        phase_entry_ = false;
        stopRobot();
        auto elapsed = std::chrono::steady_clock::now() - match_start_;
        double secs =
            std::chrono::duration_cast<std::chrono::milliseconds>(elapsed)
                .count() /
            1000.0;
        RCLCPP_INFO(this->get_logger(),
                    "=== MISSION COMPLETE === Total time: %.1f seconds", secs);
      }
      break;
    }
  }
}

// ============================================================================
// Phase / step management
// ============================================================================

void MissionSequencer::transitionTo(MissionPhase phase) {
  RCLCPP_INFO(this->get_logger(), "Phase transition: %s -> %s",
              phaseName(phase_), phaseName(phase));
  phase_ = phase;
  phase_entry_ = true;
  phase_timer_ = std::chrono::steady_clock::now();
  step_ = 0;
  step_entry_ = true;
  nav_sub_step_ = NavSubStep::IDLE;
}

const char* MissionSequencer::phaseName(MissionPhase phase) const {
  switch (phase) {
    case MissionPhase::WAIT_FOR_START:
      return "WAIT_FOR_START";
    case MissionPhase::ROBOT_START:
      return "ROBOT_START";
    case MissionPhase::PHASE_BUTTON:
      return "PHASE_BUTTON";
    case MissionPhase::PHASE_KEYPAD_CRATER:
      return "PHASE_KEYPAD_CRATER";
    case MissionPhase::MISSION_COMPLETE:
      return "MISSION_COMPLETE";
    default:
      return "UNKNOWN";
  }
}

void MissionSequencer::setStep(int step) {
  step_ = step;
  step_entry_ = true;
  goal_reached_ = false;
  nav_sub_step_ = NavSubStep::IDLE;
  step_timer_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(this->get_logger(), "[%s] -> Step %d", phaseName(phase_), step);
}

double MissionSequencer::stepElapsed() const {
  auto now = std::chrono::steady_clock::now();
  return std::chrono::duration<double>(now - step_timer_).count();
}

bool MissionSequencer::checkStepTimeout(double timeout_s) {
  if (stepElapsed() > timeout_s) {
    RCLCPP_WARN(this->get_logger(),
                "[%s] Step %d TIMED OUT after %.1f s, advancing",
                phaseName(phase_), step_, timeout_s);
    return true;
  }
  return false;
}

// ============================================================================
// Subscriber callbacks
// ============================================================================

void MissionSequencer::onDriveStatus(
    const mcu_msgs::msg::DriveBase::SharedPtr msg) {
  // Extract pose: MCU sends meters and radians
  auto& t = msg->transform.transform.translation;
  auto& r = msg->transform.transform.rotation;

  robot_x_m_ = t.x;
  robot_y_m_ = t.y;
  robot_theta_rad_ = 2.0 * std::atan2(r.z, r.w);
  has_drive_status_ = true;

  // Check if current goal is reached (all in MCU coords)
  if (last_drive_mode_ == mcu_msgs::msg::DriveBase::DRIVE_GOAL &&
      !goal_reached_) {
    double dx = robot_x_m_ - goal_x_m_;
    double dy = robot_y_m_ - goal_y_m_;
    double dist = std::hypot(dx, dy);

    double heading_err =
        std::abs(normalizeAngle(robot_theta_rad_ - goal_theta_rad_));

    if (dist < DIST_TOL_M && heading_err < HEADING_TOL_RAD) {
      goal_reached_ = true;
      auto raf = mcuToRafeed(robot_x_m_, robot_y_m_, robot_theta_rad_);
      RCLCPP_INFO(this->get_logger(),
                  "  GOAL REACHED at MCU(%.3f, %.3f, %.1f deg) "
                  "Rafeed(%.1f, %.1f, %.1f deg) "
                  "err: dist=%.3f m, heading=%.1f deg",
                  robot_x_m_, robot_y_m_, rad2deg(robot_theta_rad_),
                  raf.x_cm, raf.y_cm, raf.yaw_deg, dist, rad2deg(heading_err));
    }
  }

  // Periodic debug logging (~1 Hz)
  if (++debug_status_tick_ >= 10) {
    debug_status_tick_ = 0;
    if (phase_ == MissionPhase::PHASE_BUTTON ||
        phase_ == MissionPhase::PHASE_KEYPAD_CRATER) {
      auto raf = mcuToRafeed(robot_x_m_, robot_y_m_, robot_theta_rad_);
      RCLCPP_INFO(this->get_logger(),
                  "  [POSE] MCU(%.3f, %.3f, %.1f deg) "
                  "Rafeed(%.1f, %.1f, %.1f deg) "
                  "goal_MCU(%.3f, %.3f, %.1f deg) navStep=%d",
                  robot_x_m_, robot_y_m_, rad2deg(robot_theta_rad_),
                  raf.x_cm, raf.y_cm, raf.yaw_deg, goal_x_m_, goal_y_m_,
                  rad2deg(goal_theta_rad_),
                  static_cast<int>(nav_sub_step_));
    }
  }
}

void MissionSequencer::onTaskStatus(
    const secbot_msgs::msg::TaskStatus::SharedPtr msg) {
  if (msg->task_id == TASK_NONE) {
    task_idle_ = true;
  } else {
    task_idle_ = false;
    if (!msg->ok) {
      RCLCPP_WARN(this->get_logger(), "Task %d error: %s", msg->task_id,
                  msg->error_msg.c_str());
    }
  }
}

void MissionSequencer::onRobotInputs(
    const mcu_msgs::msg::RobotInputs::SharedPtr msg) {
  if (msg->btn1 && !start_button_pressed_) {
    start_button_pressed_ = true;
    RCLCPP_INFO(this->get_logger(), "START BUTTON PRESSED!");
  }
}

void MissionSequencer::onIntakeState(
    const mcu_msgs::msg::IntakeState::SharedPtr msg) {
  intake_state_ = msg->state;
}

// ============================================================================
// Service callbacks
// ============================================================================

void MissionSequencer::onStartService(
    const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
    std_srvs::srv::Trigger::Response::SharedPtr res) {
  if (!start_button_pressed_) {
    start_button_pressed_ = true;
    res->success = true;
    res->message = "Mission started!";
    RCLCPP_INFO(this->get_logger(), "Mission start triggered via service");
  } else {
    res->success = false;
    res->message = "Mission already started";
  }
}

}  // namespace secbot

// ============================================================================
// main
// ============================================================================

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<secbot::MissionSequencer>());
  rclcpp::shutdown();
  return 0;
}
