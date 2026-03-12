/**
 * @file mission_node.cpp
 * @author Rafeed Khan
 * @brief Implementation of the mission sequencer state machine
 *
 * State machine from statemachine.md:
 *   Phase 1: Button task (14 steps)
 *   Phase 2: Keypad + Crater + Crank (22 steps)
 *
 * Coordinate system:
 *   Origin (0,0) = bottom-right corner of field
 *   +X = LEFT (0 -> 120 cm), +Y = UP (0 -> 240 cm)
 *   Yaw 0 = facing +X (left), positive = CCW
 *   All coordinates in cm, all yaw in degrees internally.
 *   MCU uses meters and radians, so we convert at the boundary.
 *
 * Drives robot via drive_base/command (DriveBase msg), no Nav2.
 * Reads pose from drive_base/status.  MCU auto-stops after 500ms without
 * a command, so refreshDriveCommand() re-publishes every tick (100ms).
 */

#include "secbot_autonomy/mission_node.hpp"

#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace secbot {

// ============================================================================
// Constructor
// ============================================================================

MissionNode::MissionNode() : Node("mission_node") {
  RCLCPP_INFO(this->get_logger(), "Mission node starting...");

  // Publishers
  drive_cmd_pub_ =
      this->create_publisher<mcu_msgs::msg::DriveBase>("drive_base/command", 10);
  task_cmd_pub_ =
      this->create_publisher<std_msgs::msg::UInt8>("/autonomy/task_command", 10);
  minibot_cmd_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/mcu_minibot/cmd_vel", 10);
  arm_cmd_pub_ =
      this->create_publisher<mcu_msgs::msg::ArmCommand>("/arm_command", 10);
  intake_cmd_pub_ = this->create_publisher<mcu_msgs::msg::IntakeCommand>(
      "/mcu_robot/intake/command", 10);
  pose_reset_pub_ =
      this->create_publisher<geometry_msgs::msg::Pose>("drive_base/reset_pose", 10);

  // Subscribers -- drive_base/status MUST use SensorDataQoS (best-effort)
  drive_status_sub_ = this->create_subscription<mcu_msgs::msg::DriveBase>(
      "drive_base/status", rclcpp::SensorDataQoS(),
      std::bind(&MissionNode::onDriveStatus, this, _1));

  task_status_sub_ = this->create_subscription<secbot_msgs::msg::TaskStatus>(
      "/autonomy/task_status", 10,
      std::bind(&MissionNode::onTaskStatus, this, _1));

  robot_inputs_sub_ = this->create_subscription<mcu_msgs::msg::RobotInputs>(
      "/mcu_robot/inputs", 10,
      std::bind(&MissionNode::onRobotInputs, this, _1));

  intake_state_sub_ = this->create_subscription<mcu_msgs::msg::IntakeState>(
      "/mcu_robot/intake/state", rclcpp::SensorDataQoS(),
      std::bind(&MissionNode::onIntakeState, this, _1));

  // Start service
  start_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/mission/start",
      std::bind(&MissionNode::onStartService, this, _1, std::placeholders::_2));

  // Main tick timer at 10 Hz (100ms)
  tick_timer_ = this->create_wall_timer(100ms, [this]() { stepMission(); });

  RCLCPP_INFO(this->get_logger(), "Mission node ready. Waiting for start...");
}

// ============================================================================
// State machine tick
// ============================================================================

void MissionNode::stepMission() {
  // Refresh the last drive command every tick to prevent MCU 500ms timeout
  refreshDriveCommand();

  switch (phase_) {
    // ── WAIT_FOR_START ─────────────────────────────────────────────────
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

    // ── ROBOT_START: reset pose, then begin button phase ───────────────
    case MissionPhase::ROBOT_START: {
      if (phase_entry_) {
        phase_entry_ = false;
        setStep(0);
        RCLCPP_INFO(this->get_logger(), "=== ROBOT START ===");
        // Reset pose to starting position: (15, 15, 0) in cm/degrees
        resetPose(15.0, 15.0, 0.0);
        match_start_ = std::chrono::steady_clock::now();
      }
      // Wait a brief moment for pose reset to take effect
      if (stepElapsed() > 0.5) {
        transitionTo(MissionPhase::PHASE_BUTTON);
      }
      break;
    }

    // ── PHASE_BUTTON: 14 steps ─────────────────────────────────────────
    case MissionPhase::PHASE_BUTTON: {
      if (phase_entry_) {
        phase_entry_ = false;
        setStep(1);
        RCLCPP_INFO(this->get_logger(), "=== PHASE 1: BUTTON TASK ===");
      }

      switch (step_) {
        // Step 1: Start position (15, 15, 0) -- we are already here
        case 1: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "BUTTON Step 1: Start at (15, 15, 0)");
          }
          // Immediately advance, we are already at start
          setStep(2);
          break;
        }

        // Step 2: Drive to UWB placement (108, 38, 0)
        case 2: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "BUTTON Step 2: Drive to UWB placement (108, 38, 0)");
            sendGoal(108.0, 38.0, 0.0);
          }
          if (goal_reached_ || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
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
            sendGoal(38.0, 38.0, 0.0, true);
          }
          if (goal_reached_ || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
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
            sendGoal(38.0, 38.0, -45.0);
          }
          if (goal_reached_ || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
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
            sendGoal(54.0, 27.0, -45.0);
          }
          if (goal_reached_ || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
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
            sendGoal(54.0, 27.0, 0.0);
          }
          if (goal_reached_ || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
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
            sendGoal(78.0, 27.0, 0.0);
          }
          if (goal_reached_ || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
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
            sendGoal(78.0, 27.0, 0.0, true);
          }
          if (goal_reached_ || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
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
            sendGoal(78.0, 28.0, 0.0, true);
          }
          if (goal_reached_ || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
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
            sendGoal(30.0, 27.0, 0.0, true);
          }
          if (goal_reached_ || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
            setStep(14);
          }
          break;
        }

        // Step 14: Read beacon antenna (DUMMY)
        case 14: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "BUTTON Step 14: Read beacon antenna");
            readAntennaDummy("beacon");
          }
          if (stepElapsed() > 1.0) {
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

    // ── PHASE_KEYPAD_CRATER: 22 steps ──────────────────────────────────
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
            sendGoal(30.0, 27.0, 45.0);
          }
          if (goal_reached_ || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
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
            sendGoal(81.0, 95.0, 45.0);
          }
          if (goal_reached_ || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
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
            sendGoal(81.0, 95.0, 0.0);
          }
          if (goal_reached_ || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
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
            sendGoal(38.0, 95.0, 0.0, true);
          }
          if (goal_reached_ || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
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
          // Wait for task to complete or timeout
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
            sendGoal(55.0, 95.0, 0.0);
          }
          if (goal_reached_ || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
            setStep(7);
          }
          break;
        }

        // Step 7: Turn to 90 deg at (55, 95, 90) -- facing +Y, crater ahead
        case 7: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "KEYPAD Step 7: Turn to 90 deg at (55, 95, 90)");
            sendGoal(55.0, 95.0, 90.0);
          }
          if (goal_reached_ || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
            setStep(8);
          }
          break;
        }

        // Step 8: Launch minibot
        case 8: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(), "KEYPAD Step 8: Launch minibot");
            // Release minibot latch servo
            sendArmCommand(JOINT_MINIBOT_LATCH, SERVO_EXTENDED, SERVO_SPEED);
          }
          if (stepElapsed() > 1.0) {
            setStep(9);
          }
          break;
        }

        // Step 9: Wait 1 second for minibot to deploy
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
          // Wait for intake to finish extending or timeout
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
            sendGoal(55.0, 95.0, 180.0);
          }
          if (goal_reached_ || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
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
            sendGoal(91.0, 95.0, 180.0, true);
          }
          if (goal_reached_ || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
            setStep(13);
          }
          break;
        }

        // Step 13: Read keypad antenna (DUMMY)
        case 13: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "KEYPAD Step 13: Read keypad antenna");
            readAntennaDummy("keypad");
          }
          if (stepElapsed() > 1.0) {
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
            sendGoal(91.0, 95.0, 115.0);
          }
          if (goal_reached_ || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
            setStep(15);
          }
          break;
        }

        // Step 15: Read crater antenna (DUMMY)
        case 15: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "KEYPAD Step 15: Read crater antenna");
            readAntennaDummy("crater");
          }
          if (stepElapsed() > 1.0) {
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
            sendGoal(90.0, 110.0, 45.0);
          }
          if (goal_reached_ || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
            setStep(17);
          }
          break;
        }

        // Step 17: Safe path up (100, 190, 90) -- avoiding crater
        case 17: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "KEYPAD Step 17: Safe path to (100, 190, 90)");
            sendGoal(100.0, 190.0, 90.0);
          }
          if (goal_reached_ || checkStepTimeout(20.0)) {
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
            sendGoal(53.0, 225.0, 180.0);
          }
          if (goal_reached_ || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
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
            sendGoal(53.0, 225.0, 150.0, true);
          }
          if (goal_reached_ || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
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
            sendGoal(53.0, 228.0, 180.0);
          }
          if (goal_reached_ || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
            setStep(21);
          }
          break;
        }

        // Step 21: Turn to 0 deg (53, 228, 0) -- spin 180
        case 21: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "KEYPAD Step 21: Turn to 0 deg at (53, 228, 0)");
            sendGoal(53.0, 228.0, 0.0);
          }
          if (goal_reached_ || checkStepTimeout(DEFAULT_STEP_TIMEOUT_S)) {
            setStep(22);
          }
          break;
        }

        // Step 22: Read crank antenna (DUMMY)
        case 22: {
          if (step_entry_) {
            step_entry_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "KEYPAD Step 22: Read crank antenna");
            readAntennaDummy("crank");
          }
          if (stepElapsed() > 1.0) {
            RCLCPP_INFO(this->get_logger(),
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

    // ── MISSION_COMPLETE ───────────────────────────────────────────────
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

void MissionNode::transitionTo(MissionPhase phase) {
  RCLCPP_INFO(this->get_logger(), "Phase transition: %s -> %s",
              phaseName(phase_), phaseName(phase));
  phase_ = phase;
  phase_entry_ = true;
  phase_timer_ = std::chrono::steady_clock::now();
  step_ = 0;
  step_entry_ = true;
}

const char* MissionNode::phaseName(MissionPhase phase) const {
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

void MissionNode::setStep(int step) {
  step_ = step;
  step_entry_ = true;
  goal_reached_ = false;
  step_timer_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(this->get_logger(), "[%s] -> Step %d", phaseName(phase_), step);
}

double MissionNode::stepElapsed() const {
  auto now = std::chrono::steady_clock::now();
  return std::chrono::duration<double>(now - step_timer_).count();
}

bool MissionNode::checkStepTimeout(double timeout_s) {
  if (stepElapsed() > timeout_s) {
    RCLCPP_WARN(this->get_logger(),
                "[%s] Step %d TIMED OUT after %.1f s, advancing",
                phaseName(phase_), step_, timeout_s);
    return true;
  }
  return false;
}

// ============================================================================
// Navigation helpers
// ============================================================================

void MissionNode::sendGoal(double x_cm, double y_cm, double yaw_deg,
                           bool reverse) {
  // Convert cm -> meters, degrees -> radians for MCU
  double x_m = cm2m(x_cm);
  double y_m = cm2m(y_cm);
  double theta_rad = deg2rad(yaw_deg);

  // Track goal in cm/degrees for goal-reached detection
  goal_x_cm_ = x_cm;
  goal_y_cm_ = y_cm;
  goal_yaw_deg_ = yaw_deg;
  goal_reached_ = false;

  auto msg = mcu_msgs::msg::DriveBase();
  msg.drive_mode = mcu_msgs::msg::DriveBase::DRIVE_GOAL;
  msg.goal_transform.transform.translation.x = x_m;
  msg.goal_transform.transform.translation.y = y_m;
  msg.goal_transform.transform.translation.z = reverse ? -1.0 : 0.0;

  // Encode heading as yaw-only quaternion: q = (0, 0, sin(theta/2), cos(theta/2))
  float half = static_cast<float>(theta_rad) * 0.5f;
  msg.goal_transform.transform.rotation.x = 0.0;
  msg.goal_transform.transform.rotation.y = 0.0;
  msg.goal_transform.transform.rotation.z = std::sin(half);
  msg.goal_transform.transform.rotation.w = std::cos(half);

  last_drive_cmd_ = msg;
  last_drive_mode_ = mcu_msgs::msg::DriveBase::DRIVE_GOAL;
  drive_cmd_pub_->publish(msg);

  RCLCPP_INFO(this->get_logger(),
              "  sendGoal(%.0f, %.0f, %.0f deg%s) -> (%.3f, %.3f, %.3f rad)",
              x_cm, y_cm, yaw_deg, reverse ? " REV" : "", x_m, y_m,
              theta_rad);
}

void MissionNode::sendVelocity(double vx_mps, double omega_radps) {
  auto msg = mcu_msgs::msg::DriveBase();
  msg.drive_mode = mcu_msgs::msg::DriveBase::DRIVE_VECTOR;
  msg.goal_velocity.linear.x = vx_mps;
  msg.goal_velocity.angular.z = omega_radps;

  last_drive_cmd_ = msg;
  last_drive_mode_ = mcu_msgs::msg::DriveBase::DRIVE_VECTOR;
  drive_cmd_pub_->publish(msg);
}

void MissionNode::sendNudge(double speed_mps) {
  RCLCPP_INFO(this->get_logger(), "  sendNudge(%.2f m/s)", speed_mps);
  sendVelocity(speed_mps, 0.0);
}

void MissionNode::stopRobot() {
  sendVelocity(0.0, 0.0);
}

void MissionNode::refreshDriveCommand() {
  // Re-publish last command to prevent MCU 500ms timeout auto-stop.
  // Stop re-publishing once a DRIVE_GOAL is reached, so the robot stops.
  if (goal_reached_ &&
      last_drive_mode_ == mcu_msgs::msg::DriveBase::DRIVE_GOAL) {
    return;
  }
  if (last_drive_mode_ != 0) {
    drive_cmd_pub_->publish(last_drive_cmd_);
  }
}

void MissionNode::resetPose(double x_cm, double y_cm, double yaw_deg) {
  double x_m = cm2m(x_cm);
  double y_m = cm2m(y_cm);
  double theta_rad = deg2rad(yaw_deg);

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
  RCLCPP_INFO(this->get_logger(),
              "  resetPose(%.0f, %.0f, %.0f deg) -> (%.3f, %.3f, %.3f rad)",
              x_cm, y_cm, yaw_deg, x_m, y_m, theta_rad);
}

// ============================================================================
// Task helpers
// ============================================================================

void MissionNode::sendTaskCommand(uint8_t task_id) {
  auto msg = std_msgs::msg::UInt8();
  msg.data = task_id;
  task_cmd_pub_->publish(msg);
  task_idle_ = false;
  RCLCPP_INFO(this->get_logger(), "  sendTaskCommand(%d)", task_id);
}

void MissionNode::sendArmCommand(uint8_t joint_id, int16_t position,
                                 uint8_t speed) {
  auto msg = mcu_msgs::msg::ArmCommand();
  msg.joint_id = joint_id;
  msg.position = position;
  msg.speed = speed;
  arm_cmd_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "  sendArmCommand(joint=%d, pos=%d, spd=%d)",
              joint_id, position, speed);
}

void MissionNode::sendIntakeCommand(uint8_t cmd, float value) {
  auto msg = mcu_msgs::msg::IntakeCommand();
  msg.command = cmd;
  msg.value = value;
  intake_cmd_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "  sendIntakeCommand(cmd=%d, val=%.2f)", cmd,
              value);
}

void MissionNode::readAntennaDummy(const char* label) {
  RCLCPP_INFO(this->get_logger(),
              "  [DUMMY] Reading %s antenna... (placeholder, returning green)",
              label);
}

// ============================================================================
// Subscriber callbacks
// ============================================================================

void MissionNode::onDriveStatus(
    const mcu_msgs::msg::DriveBase::SharedPtr msg) {
  // Extract pose from status: MCU sends meters and radians
  auto& t = msg->transform.transform.translation;
  auto& r = msg->transform.transform.rotation;

  double x_m = t.x;
  double y_m = t.y;
  double theta_rad = 2.0 * std::atan2(r.z, r.w);

  // Store in cm / degrees
  robot_x_cm_ = m2cm(x_m);
  robot_y_cm_ = m2cm(y_m);
  robot_yaw_deg_ = rad2deg(theta_rad);
  has_drive_status_ = true;

  // Check if current goal is reached
  if (last_drive_mode_ == mcu_msgs::msg::DriveBase::DRIVE_GOAL && !goal_reached_) {
    double dx = robot_x_cm_ - goal_x_cm_;
    double dy = robot_y_cm_ - goal_y_cm_;
    double dist_cm = std::hypot(dx, dy);

    // Normalize heading error to [-180, 180]
    double heading_err = robot_yaw_deg_ - goal_yaw_deg_;
    while (heading_err > 180.0) heading_err -= 360.0;
    while (heading_err < -180.0) heading_err += 360.0;

    if (dist_cm < GOAL_DIST_TOL_CM &&
        std::abs(heading_err) < GOAL_HEADING_TOL_DEG) {
      goal_reached_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "  GOAL REACHED at (%.1f, %.1f, %.1f deg) "
                  "err: dist=%.1f cm, heading=%.1f deg",
                  robot_x_cm_, robot_y_cm_, robot_yaw_deg_, dist_cm,
                  heading_err);
    }
  }

  // Periodic debug logging (every 10th callback, around 1 Hz at 10 Hz status)
  if (++debug_status_tick_ >= 10) {
    debug_status_tick_ = 0;
    if (phase_ == MissionPhase::PHASE_BUTTON ||
        phase_ == MissionPhase::PHASE_KEYPAD_CRATER) {
      RCLCPP_INFO(this->get_logger(),
                  "  [POSE] (%.1f, %.1f, %.1f deg) goal=(%.1f, %.1f, %.1f deg)",
                  robot_x_cm_, robot_y_cm_, robot_yaw_deg_, goal_x_cm_,
                  goal_y_cm_, goal_yaw_deg_);
    }
  }
}

void MissionNode::onTaskStatus(
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

void MissionNode::onRobotInputs(
    const mcu_msgs::msg::RobotInputs::SharedPtr msg) {
  // Use button 1 as start trigger
  if (msg->btn1 && !start_button_pressed_) {
    start_button_pressed_ = true;
    RCLCPP_INFO(this->get_logger(), "START BUTTON PRESSED!");
  }
}

void MissionNode::onIntakeState(
    const mcu_msgs::msg::IntakeState::SharedPtr msg) {
  intake_state_ = msg->state;
}

// ============================================================================
// Service callbacks
// ============================================================================

void MissionNode::onStartService(
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
