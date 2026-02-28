/**
 * @file DriveBaseConfig.example.h
 * @brief Example configuration for DriveBase with motion profiles and
 * trajectory controller
 * @date 12/24/2025
 *
 * This file demonstrates how to properly configure the DriveBaseSetup with:
 * - Motor and encoder drivers
 * - Wheel velocity PID controllers
 * - S-Curve motion profiles for smooth velocity ramping
 * - Trajectory controller for path following
 *
 * Copy this configuration into your RobotLogic.h and adjust parameters for your
 * robot.
 */

#pragma once

#include "robot/drive-base/DriveSubsystem.h"
#include "robot/drive-base/EncoderDriver.h"
#include "robot/drive-base/MotorDriver.h"

// Example robot physical constants
namespace DriveConfig {
constexpr float WHEEL_TRACK_WIDTH = 12.0f;   // inches between left/right wheels
constexpr float WHEEL_DIAMETER = 4.0f;       // inches
constexpr int ENCODER_TICKS_PER_REV = 2048;  // encoder resolution
constexpr float INCHES_PER_TICK =
    (WHEEL_DIAMETER * M_PI) / ENCODER_TICKS_PER_REV;

// Robot performance limits
constexpr float MAX_LINEAR_VELOCITY = 24.0f;  // inches/sec
constexpr float MAX_LINEAR_ACCEL = 12.0f;     // inches/sec^2
constexpr float MAX_LINEAR_JERK = 48.0f;      // inches/sec^3
constexpr float MAX_ANGULAR_VELOCITY = 3.0f;  // rad/sec
constexpr float MAX_ANGULAR_ACCEL = 6.0f;     // rad/sec^2
constexpr float MAX_ANGULAR_JERK = 24.0f;     // rad/sec^3
}  // namespace DriveConfig

// --- Motor driver setups ---
// Left motors
static Drivers::MotorDriverSetup g_left_motor1_setup("left_motor_1",
                                                     /*pwm_pin*/ 3,
                                                     /*dir_pin*/ 4);
static Drivers::MotorDriverSetup g_left_motor2_setup("left_motor_2",
                                                     /*pwm_pin*/ 5,
                                                     /*dir_pin*/ 6);

// Right motors
static Drivers::MotorDriverSetup g_right_motor1_setup("right_motor_1",
                                                      /*pwm_pin*/ 7,
                                                      /*dir_pin*/ 8);
static Drivers::MotorDriverSetup g_right_motor2_setup("right_motor_2",
                                                      /*pwm_pin*/ 9,
                                                      /*dir_pin*/ 10);

// --- Encoder driver setups ---
static Drivers::EncoderDriverSetup g_left_encoder_setup("left_encoder",
                                                        /*pinA*/ 18,
                                                        /*pinB*/ 19);
static Drivers::EncoderDriverSetup g_right_encoder_setup("right_encoder",
                                                         /*pinA*/ 20,
                                                         /*pinB*/ 21);

// --- Wheel velocity PID configurations ---
// These PIDs control individual wheel velocities
static PIDController::Config createWheelPIDConfig() {
  PIDController::Config cfg;

  // PID gains (tune these for your robot!)
  cfg.gains.kp = 0.8f;   // Proportional gain
  cfg.gains.ki = 0.1f;   // Integral gain
  cfg.gains.kd = 0.05f;  // Derivative gain

  // Output limits (motor PWM range)
  cfg.limits.out_min = -255.0f;
  cfg.limits.out_max = 255.0f;

  // Integral anti-windup limits
  cfg.limits.i_min = -50.0f;
  cfg.limits.i_max = 50.0f;

  // Use derivative on measurement to avoid kick on setpoint changes
  cfg.dmode = PIDController::DerivativeMode::OnMeasurement;

  // Optional derivative filtering for noise reduction
  cfg.d_filter_alpha = 0.1f;

  // Conditional integration for better anti-windup
  cfg.conditional_integration = true;

  return cfg;
}

// --- S-Curve motion profile configurations ---
// Linear motion profile for forward/backward velocity
static SCurveMotionProfile::Config createLinearProfileConfig() {
  SCurveMotionProfile::Config cfg;

  cfg.limits.v_max = DriveConfig::MAX_LINEAR_VELOCITY;
  cfg.limits.a_max = DriveConfig::MAX_LINEAR_ACCEL;
  cfg.limits.d_max = DriveConfig::MAX_LINEAR_ACCEL;  // same as accel
  cfg.limits.j_max = DriveConfig::MAX_LINEAR_JERK;

  // Tolerances for "close enough"
  cfg.pos_tol = 0.1f;  // inches
  cfg.vel_tol = 0.5f;  // inches/sec

  // dt guards
  cfg.min_dt = 1e-6f;
  cfg.max_dt = 0.25f;

  return cfg;
}

// Angular motion profile for rotation
static SCurveMotionProfile::Config createAngularProfileConfig() {
  SCurveMotionProfile::Config cfg;

  cfg.limits.v_max = DriveConfig::MAX_ANGULAR_VELOCITY;
  cfg.limits.a_max = DriveConfig::MAX_ANGULAR_ACCEL;
  cfg.limits.d_max = DriveConfig::MAX_ANGULAR_ACCEL;
  cfg.limits.j_max = DriveConfig::MAX_ANGULAR_JERK;

  cfg.pos_tol = 0.01f;  // radians
  cfg.vel_tol = 0.1f;   // rad/sec

  cfg.min_dt = 1e-6f;
  cfg.max_dt = 0.25f;

  return cfg;
}

// --- Trajectory controller configuration ---
static TrajectoryController::Config createTrajControllerConfig() {
  TrajectoryController::Config cfg;

  // Pure pursuit lookahead distance
  // Larger = smoother but cuts corners more, Smaller = tighter following
  cfg.lookahead_dist = 6.0f;  // inches

  // Default cruise speed
  cfg.cruise_v = 12.0f;  // inches/sec

  // Velocity and angular velocity limits
  cfg.max_v = DriveConfig::MAX_LINEAR_VELOCITY;
  cfg.max_w = DriveConfig::MAX_ANGULAR_VELOCITY;

  // Preserve curvature when angular velocity saturates
  cfg.preserve_curvature_on_w_saturation = true;

  // Slow down as we approach the goal
  cfg.slowdown_dist = 8.0f;    // inches
  cfg.min_v_near_goal = 2.0f;  // inches/sec minimum creep speed

  // Goal tolerances
  cfg.pos_tol = 0.5f;      // inches
  cfg.heading_tol = 0.1f;  // radians (~5.7 degrees)

  // Final heading control
  cfg.use_final_heading = true;
  cfg.k_heading = 3.0f;  // P gain for heading correction

  // Progress advancement tolerance
  cfg.advance_tol = 3.0f;  // inches

  // dt guards
  cfg.min_dt = 1e-6f;
  cfg.max_dt = 0.25f;

  return cfg;
}

// --- Tank drive localization setup ---
static Drive::TankDriveLocalizationSetup createLocalizationSetup() {
  Drive::TankDriveLocalizationSetup setup;
  setup.track_width = DriveConfig::WHEEL_TRACK_WIDTH;
  setup.dist_per_tick = DriveConfig::INCHES_PER_TICK;
  return setup;
}

// --- Complete DriveBaseSetup ---
static DriveBaseSetup createDriveBaseSetup() {
  DriveBaseSetup setup;

  // Motor configurations
  setup.motorSetups = {g_left_motor1_setup, g_left_motor2_setup,
                       g_right_motor1_setup, g_right_motor2_setup};

  // Encoder configurations
  setup.encoderSetups = {g_left_encoder_setup, g_right_encoder_setup};

  // Wheel velocity PID configs
  setup.leftWheelPIDSetup = createWheelPIDConfig();
  setup.rightWheelPIDSetup = createWheelPIDConfig();

  // Localization setup
  setup.localizationSetup = createLocalizationSetup();

  // Motion profile configs (NEW!)
  setup.linearProfileConfig = createLinearProfileConfig();
  setup.angularProfileConfig = createAngularProfileConfig();

  // Trajectory controller config (NEW!)
  setup.trajControllerConfig = createTrajControllerConfig();

  // Legacy max values (kept for backward compatibility)
  setup.maxVelocity = DriveConfig::MAX_LINEAR_VELOCITY;
  setup.maxAcceleration = DriveConfig::MAX_LINEAR_ACCEL;
  setup.maxAngularVelocity = DriveConfig::MAX_ANGULAR_VELOCITY;
  setup.maxAngularAcceleration = DriveConfig::MAX_ANGULAR_ACCEL;

  return setup;
}

// --- Usage example in RobotLogic.h ---
/*

#include "robot/drive-base/DriveBaseConfig.example.h"

// Create drive base setup
static DriveBaseSetup g_drive_base_setup = createDriveBaseSetup();
static DriveSubsystemSetup g_drive_setup("drive_subsystem", g_drive_base_setup);
static DriveSubsystem g_drive(g_drive_setup);

// In mcu_init_cb():
ok = ok && g_drive.init();

// In mcu_begin_cb():
g_mr.registerParticipant(&g_drive);

// In mcu_update_cb():
g_drive.update();

// Example usage: Drive to a waypoint
TrajectoryController::Waypoint waypoints[] = {
    {0.0f, 0.0f},      // Start at origin
    {24.0f, 0.0f},     // Drive 24 inches forward
    {24.0f, 24.0f},    // Turn and drive 24 inches to the right
    {0.0f, 24.0f}      // Return to X=0
};
g_drive.driveBase_.driveTrajectory(waypoints, 4);

*/
