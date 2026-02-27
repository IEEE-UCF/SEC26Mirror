/**
 * @file mcu_subsystem_sim.hpp
 * @brief MCU Subsystem Simulator — acts like the real Teensy with full control
 * pipeline
 * @date 2025-12-25
 */

#pragma once

#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mcu_msgs/msg/battery_health.hpp>
#include <mcu_msgs/msg/drive_base.hpp>
#include <mcu_msgs/msg/arm_command.hpp>
#include <mcu_msgs/msg/intake_bridge_command.hpp>
#include <mcu_msgs/msg/intake_bridge_state.hpp>
#include <mcu_msgs/msg/intake_state.hpp>
#include <mcu_msgs/msg/mcu_state.hpp>
#include <std_msgs/msg/int16.hpp>
#include <mcu_msgs/msg/mini_robot_state.hpp>
#include <mcu_msgs/msg/rc.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>

// Real MCU control classes (compiled from mcu_ws, no Arduino deps)
#include <Pose2D.h>
#include <TankDriveLocalization.h>
#include <Vector2D.h>
#include <motion_profile.h>
#include <pid_controller.h>
#include <traj_controller.h>

namespace secbot_sim {

enum class DriveMode { MANUAL, VELOCITY_DRIVE, POSE_DRIVE, TRAJECTORY_DRIVE };

class McuSubsystemSimulator : public rclcpp::Node {
 public:
  McuSubsystemSimulator();

 private:
  // ── Real MCU control objects (same as RobotDriveBase members) ──
  PIDController left_pid_;
  PIDController right_pid_;
  SCurveMotionProfile linear_profile_;
  SCurveMotionProfile angular_profile_;
  TrajectoryController traj_controller_;

  // Localization (unique_ptr: setup has no default ctor, localization holds a
  // ref)
  std::unique_ptr<Drive::TankDriveLocalizationSetup> loc_setup_;
  std::unique_ptr<Drive::TankDriveLocalization> localization_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // ── Drive state ──
  DriveMode current_mode_;
  Vector2D target_velocity_;
  Vector2D current_velocity_;
  Pose2D target_pose_;
  Pose2D prev_pose_;

  // ── Simulated motor state (PWM output from PID) ──
  int left_motor_pwm_;
  int right_motor_pwm_;

  // ── Simulated encoder state ──
  double left_encoder_accum_;  // fractional tick accumulator (precision)
  double right_encoder_accum_;
  long prev_left_ticks_;
  long prev_right_ticks_;

  // ── Simulated IMU state ──
  float sim_yaw_;
  double prev_vel_x_;  // for IMU linear acceleration

  // ── Config (from ROS params, matching DriveBaseConfig.example.h) ──
  float track_width_;
  float wheel_diameter_;
  int encoder_ticks_per_rev_;
  double gear_ratio_;
  float max_velocity_;
  float max_angular_velocity_;
  float max_wheel_velocity_;  // max single-wheel vel at PWM=255
  float inches_per_tick_;     // derived

  // Parameters
  int num_tof_sensors_;

  // Timing
  rclcpp::Time last_update_time_;

  // ── Publishers (all 8 MCU subsystems) ──
  rclcpp::Publisher<mcu_msgs::msg::DriveBase>::SharedPtr drive_status_pub_;
  rclcpp::Publisher<mcu_msgs::msg::BatteryHealth>::SharedPtr battery_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr tof_pub_;
  rclcpp::Publisher<mcu_msgs::msg::RC>::SharedPtr rc_pub_;
  rclcpp::Publisher<mcu_msgs::msg::IntakeState>::SharedPtr intake_pub_;
  rclcpp::Publisher<mcu_msgs::msg::IntakeBridgeState>::SharedPtr bridge_pub_;
  rclcpp::Publisher<mcu_msgs::msg::MiniRobotState>::SharedPtr mini_robot_pub_;
  rclcpp::Publisher<mcu_msgs::msg::McuState>::SharedPtr mcu_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_feedback_pub_;

  // ── Subscribers ──
  rclcpp::Subscription<mcu_msgs::msg::DriveBase>::SharedPtr drive_command_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gz_odom_sub_;
  rclcpp::Subscription<mcu_msgs::msg::IntakeBridgeCommand>::SharedPtr
      bridge_cmd_sub_;
  rclcpp::Subscription<mcu_msgs::msg::ArmCommand>::SharedPtr arm_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr intake_speed_sub_;

  // ── Gazebo ground-truth odom (for sim trajectory tracking) ──
  double gz_odom_x_ = 0.0;
  double gz_odom_y_ = 0.0;
  double gz_odom_yaw_ = 0.0;
  bool gz_odom_received_ = false;

  std::vector<TrajectoryController::Waypoint> active_traj_;

  // ── Commanded velocity for Gazebo (bypasses internal PID/physics) ──
  double gz_cmd_v_ = 0.0;  // m/s
  double gz_cmd_w_ = 0.0;  // rad/s

  // ── Timers ──
  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::TimerBase::SharedPtr drive_publish_timer_;
  rclcpp::TimerBase::SharedPtr battery_publish_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_publish_timer_;
  rclcpp::TimerBase::SharedPtr imu_publish_timer_;
  rclcpp::TimerBase::SharedPtr tof_publish_timer_;
  rclcpp::TimerBase::SharedPtr rc_publish_timer_;
  rclcpp::TimerBase::SharedPtr intake_publish_timer_;
  rclcpp::TimerBase::SharedPtr bridge_publish_timer_;
  rclcpp::TimerBase::SharedPtr mini_robot_publish_timer_;
  rclcpp::TimerBase::SharedPtr mcu_state_publish_timer_;

  // ── Control methods (mirror RobotDriveBase) ──
  void velocityControl(float dt);
  void setPointControl(float dt);
  void trajectoryControl(float dt);
  void writeMotorSpeeds(int left_pwm, int right_pwm);
  void simulatePhysics(float dt);

  // ── Callbacks ──
  void driveCommandCallback(const mcu_msgs::msg::DriveBase::SharedPtr msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void updateTimerCallback();
  void drivePublishCallback();
  void batteryPublishCallback();
  void heartbeatPublishCallback();
  void imuPublishCallback();
  void tofPublishCallback();
  void rcPublishCallback();
  void intakePublishCallback();
  void bridgePublishCallback();
  void bridgeCommandCallback(
      const mcu_msgs::msg::IntakeBridgeCommand::SharedPtr msg);
  void armCommandCallback(const mcu_msgs::msg::ArmCommand::SharedPtr msg);
  void intakeSpeedCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void miniRobotPublishCallback();
  void mcuStatePublishCallback();
  void trajectoryCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void gzOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // -- Simulated bridge state --
  uint8_t sim_bridge_state_ = 0;  // STATE_STOWED
  std::chrono::steady_clock::time_point bridge_cmd_time_;
  uint8_t bridge_target_state_ = 0;

  // -- Simulated intake state (improved) --
  uint8_t sim_intake_state_ = 0;  // STATE_IDLE
  bool sim_duck_detected_ = false;
  int16_t sim_intake_speed_ = 0;
  std::chrono::steady_clock::time_point intake_state_time_;
};

}  // namespace secbot_sim
