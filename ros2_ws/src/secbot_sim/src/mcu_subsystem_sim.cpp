/**
 * @file mcu_subsystem_sim.cpp
 * @author Rafeed Khan
 * @brief MCU Subsystem Simulator! Full control pipeline that matches the real
 * Teensy
 * @date 2025-12-25
 *
 * Simulates all MCU subsystems: Drive (with real S-curve + PID + localization),
 * Battery, Heartbeat, IMU, TOF, RC, Intake, and MiniRobot.
 *
 * The drive control path mirrors RobotDriveBase exactly:
 *   cmd_vel / drive_base/command
 *     -> driveVelocity() (reset S-curve profiles, set goals)
 *     -> update loop:
 *         simulatePhysics() (motor PWM -> wheel vel -> encoder ticks)
 *         getCurrentVelocity() (encoder tick deltas)
 *         localization_.update(ticks, yaw)
 *         velocityControl(): S-curve -> inverse kinematics -> PID -> motor PWM
 */

#include "secbot_sim/mcu_subsystem_sim.hpp"

namespace secbot_sim {

// BNO085-based covariance values (from ImuSubsystem.h)
static constexpr double kOrientationVariance = 0.001;
static constexpr double kAngularVelocityVariance = 0.00003;
static constexpr double kLinearAccelerationVariance = 0.0001;

McuSubsystemSimulator::McuSubsystemSimulator()
    : Node("mcu_subsystem_sim"),
      current_mode_(DriveMode::MANUAL),
      left_motor_pwm_(0),
      right_motor_pwm_(0),
      left_encoder_accum_(0.0),
      right_encoder_accum_(0.0),
      prev_left_ticks_(0),
      prev_right_ticks_(0),
      sim_yaw_(0.0f),
      prev_vel_x_(0.0) {
  RCLCPP_INFO(this->get_logger(), "Starting MCU Subsystem Simulator");

  // Declare all parameters
  this->declare_parameter("num_tof_sensors", 4);
  num_tof_sensors_ = this->get_parameter("num_tof_sensors").as_int();

  // Drive config (matching DriveBaseConfig.example.h defaults)
  this->declare_parameter("track_width", 12.0);
  this->declare_parameter("wheel_diameter", 4.0);
  this->declare_parameter("encoder_ticks_per_rev", 2048);
  this->declare_parameter("gear_ratio", 1);
  this->declare_parameter("max_velocity", 24.0);
  this->declare_parameter("max_angular_velocity", 3.0);
  this->declare_parameter("max_wheel_velocity", 48.0);

  // PID parameters
  this->declare_parameter("pid_kp", 0.8);
  this->declare_parameter("pid_ki", 0.1);
  this->declare_parameter("pid_kd", 0.05);
  this->declare_parameter("pid_out_min", -255.0);
  this->declare_parameter("pid_out_max", 255.0);
  this->declare_parameter("pid_i_min", -50.0);
  this->declare_parameter("pid_i_max", 50.0);
  this->declare_parameter("pid_d_filter_alpha", 0.1);

  // Linear S-curve parameters
  this->declare_parameter("linear_v_max", 24.0);
  this->declare_parameter("linear_a_max", 12.0);
  this->declare_parameter("linear_d_max", 12.0);
  this->declare_parameter("linear_j_max", 48.0);

  // Angular S-curve parameters
  this->declare_parameter("angular_v_max", 3.0);
  this->declare_parameter("angular_a_max", 6.0);
  this->declare_parameter("angular_d_max", 6.0);
  this->declare_parameter("angular_j_max", 24.0);

  // Read parameters
  track_width_ =
      static_cast<float>(this->get_parameter("track_width").as_double());
  wheel_diameter_ =
      static_cast<float>(this->get_parameter("wheel_diameter").as_double());
  encoder_ticks_per_rev_ =
      this->get_parameter("encoder_ticks_per_rev").as_int();
  gear_ratio_ = this->get_parameter("gear_ratio").as_int();
  max_velocity_ =
      static_cast<float>(this->get_parameter("max_velocity").as_double());
  max_angular_velocity_ = static_cast<float>(
      this->get_parameter("max_angular_velocity").as_double());
  max_wheel_velocity_ =
      static_cast<float>(this->get_parameter("max_wheel_velocity").as_double());

  // Derived constants (same math as TankDriveLocalizationSetup constructor)
  float wheel_circumference = static_cast<float>(M_PI) * wheel_diameter_;
  long ticks_per_rev = static_cast<long>(encoder_ticks_per_rev_) * gear_ratio_;
  dist_per_tick_ = wheel_circumference / static_cast<float>(ticks_per_rev);

  // Configure PID controllers (same config for both wheels)
  PIDController::Config pid_cfg;
  pid_cfg.gains.kp =
      static_cast<float>(this->get_parameter("pid_kp").as_double());
  pid_cfg.gains.ki =
      static_cast<float>(this->get_parameter("pid_ki").as_double());
  pid_cfg.gains.kd =
      static_cast<float>(this->get_parameter("pid_kd").as_double());
  pid_cfg.limits.out_min =
      static_cast<float>(this->get_parameter("pid_out_min").as_double());
  pid_cfg.limits.out_max =
      static_cast<float>(this->get_parameter("pid_out_max").as_double());
  pid_cfg.limits.i_min =
      static_cast<float>(this->get_parameter("pid_i_min").as_double());
  pid_cfg.limits.i_max =
      static_cast<float>(this->get_parameter("pid_i_max").as_double());
  pid_cfg.dmode = PIDController::DerivativeMode::OnMeasurement;
  pid_cfg.d_filter_alpha =
      static_cast<float>(this->get_parameter("pid_d_filter_alpha").as_double());

  left_pid_.configure(pid_cfg);
  right_pid_.configure(pid_cfg);

  // Configure S-curve motion profiles
  SCurveMotionProfile::Config linear_cfg;
  linear_cfg.limits.v_max =
      static_cast<float>(this->get_parameter("linear_v_max").as_double());
  linear_cfg.limits.a_max =
      static_cast<float>(this->get_parameter("linear_a_max").as_double());
  linear_cfg.limits.d_max =
      static_cast<float>(this->get_parameter("linear_d_max").as_double());
  linear_cfg.limits.j_max =
      static_cast<float>(this->get_parameter("linear_j_max").as_double());
  linear_profile_.configure(linear_cfg);

  SCurveMotionProfile::Config angular_cfg;
  angular_cfg.limits.v_max =
      static_cast<float>(this->get_parameter("angular_v_max").as_double());
  angular_cfg.limits.a_max =
      static_cast<float>(this->get_parameter("angular_a_max").as_double());
  angular_cfg.limits.d_max =
      static_cast<float>(this->get_parameter("angular_d_max").as_double());
  angular_cfg.limits.j_max =
      static_cast<float>(this->get_parameter("angular_j_max").as_double());
  angular_profile_.configure(angular_cfg);

  // Configure localization
  loc_setup_ = std::make_unique<Drive::TankDriveLocalizationSetup>(
      "sim", track_width_, wheel_diameter_, encoder_ticks_per_rev_,
      gear_ratio_);
  localization_ = std::make_unique<Drive::TankDriveLocalization>(*loc_setup_);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  // Initialize last update time
  last_update_time_ = this->now();

  // Publishers (all 8 MCU subsystems)
  drive_status_pub_ =
      this->create_publisher<mcu_msgs::msg::DriveBase>("drive_base/status", 10);

  battery_pub_ = this->create_publisher<mcu_msgs::msg::BatteryHealth>(
      "/mcu_robot/battery_health", 10);

  heartbeat_pub_ =
      this->create_publisher<std_msgs::msg::String>("/mcu_robot/heartbeat", 10);

  imu_pub_ =
      this->create_publisher<sensor_msgs::msg::Imu>("/mcu_robot/imu/data", 10);

  tof_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/mcu_robot/tof_distances", 10);

  rc_pub_ = this->create_publisher<mcu_msgs::msg::RC>("mcu_robot/rc", 10);

  intake_pub_ = this->create_publisher<mcu_msgs::msg::IntakeState>(
      "/mcu_robot/intake/state", 10);

  bridge_pub_ = this->create_publisher<mcu_msgs::msg::IntakeBridgeState>(
      "/mcu_robot/intake_bridge/state", 10);

  mini_robot_pub_ = this->create_publisher<mcu_msgs::msg::MiniRobotState>(
      "/mcu_robot/mini_robot/state", 10);

  mcu_state_pub_ = this->create_publisher<mcu_msgs::msg::McuState>(
      "/mcu_robot/mcu_state", 10);

  cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_out", 10);

  // Subscribers
  drive_command_sub_ = this->create_subscription<mcu_msgs::msg::DriveBase>(
      "drive_base/command", 10,
      std::bind(&McuSubsystemSimulator::driveCommandCallback, this,
                std::placeholders::_1));

  auto traj_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  trajectory_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "drive_base/trajectory", traj_qos,
      std::bind(&McuSubsystemSimulator::trajectoryCallback, this,
                std::placeholders::_1));

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_in", 10,
      std::bind(&McuSubsystemSimulator::cmdVelCallback, this,
                std::placeholders::_1));

  // Bridge command subscriber
  bridge_cmd_sub_ =
      this->create_subscription<mcu_msgs::msg::IntakeBridgeCommand>(
          "/mcu_robot/intake_bridge/command", 10,
          std::bind(&McuSubsystemSimulator::bridgeCommandCallback, this,
                    std::placeholders::_1));

  // Arm command subscriber (for camera mast, latches)
  arm_cmd_sub_ = this->create_subscription<mcu_msgs::msg::ArmCommand>(
      "/arm_command", 10,
      std::bind(&McuSubsystemSimulator::armCommandCallback, this,
                std::placeholders::_1));

  // Intake speed subscriber
  intake_speed_sub_ = this->create_subscription<std_msgs::msg::Int16>(
      "/intake_speed", 10,
      std::bind(&McuSubsystemSimulator::intakeSpeedCallback, this,
                std::placeholders::_1));

  // Gazebo ground-truth odometry — used for trajectory tracking in sim
  gz_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&McuSubsystemSimulator::gzOdomCallback, this,
                std::placeholders::_1));

  // Timers
  // Physics + control update @ 100 Hz (10 ms)
  update_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&McuSubsystemSimulator::updateTimerCallback, this));

  // Drive + IMU publish @ 50 Hz (20 ms) — EKF needs frequent odom/imu
  drive_publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&McuSubsystemSimulator::drivePublishCallback, this));

  imu_publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&McuSubsystemSimulator::imuPublishCallback, this));

  // Battery @ 1 Hz
  battery_publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&McuSubsystemSimulator::batteryPublishCallback, this));

  // Heartbeat @ 5 Hz
  heartbeat_publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&McuSubsystemSimulator::heartbeatPublishCallback, this));

  // TOF @ 10 Hz
  tof_publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&McuSubsystemSimulator::tofPublishCallback, this));

  // RC @ 20 Hz
  rc_publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&McuSubsystemSimulator::rcPublishCallback, this));

  // Intake @ 20 Hz
  intake_publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&McuSubsystemSimulator::intakePublishCallback, this));

  // Bridge @ 20 Hz
  bridge_publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&McuSubsystemSimulator::bridgePublishCallback, this));

  // MiniRobot @ 10 Hz
  mini_robot_publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&McuSubsystemSimulator::miniRobotPublishCallback, this));

  // MCU State @ 50 Hz (20 ms) — same as real MCU (MS_20 timer)
  mcu_state_publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&McuSubsystemSimulator::mcuStatePublishCallback, this));

  RCLCPP_INFO(this->get_logger(),
              "MCU Subsystem Simulator initialized (9 subsystems + real "
              "control pipeline)");
}

void McuSubsystemSimulator::trajectoryCallback(
    const nav_msgs::msg::Path::SharedPtr msg) {
  active_traj_.clear();
  active_traj_.reserve(msg->poses.size());

  // Keep waypoints in meters — TrajectoryController config (lookahead,
  // tolerances) is in meters, so the pose and waypoints must also be in meters.
  for (const auto& ps : msg->poses) {
    TrajectoryController::Waypoint wp{};
    wp.x = static_cast<float>(ps.pose.position.x);
    wp.y = static_cast<float>(ps.pose.position.y);
    active_traj_.push_back(wp);
  }

  if (active_traj_.empty()) {
    traj_controller_.clearTrajectory();
    writeMotorSpeeds(0, 0);
    gz_cmd_v_ = 0.0;
    gz_cmd_w_ = 0.0;
    current_mode_ = DriveMode::VELOCITY_DRIVE;
    target_velocity_ = Vector2D(0, 0, 0);
    return;
  }

  // Reset controller state BEFORE setTrajectory if your API supports it
  traj_controller_.clearTrajectory();  // safe even if empty
  traj_controller_.setTrajectory(active_traj_.data(), active_traj_.size());

  current_mode_ = DriveMode::TRAJECTORY_DRIVE;

  left_pid_.reset();
  right_pid_.reset();

  // prevent a big dt on the next update
  last_update_time_ = this->now();

  // optional: stop any previous motor command immediately
  writeMotorSpeeds(0, 0);

  RCLCPP_INFO(this->get_logger(), "Loaded trajectory: %zu waypoints",
              active_traj_.size());
}

// Drive command callback, mirrors DriveSubsystem::drive_callback
void McuSubsystemSimulator::driveCommandCallback(
    const mcu_msgs::msg::DriveBase::SharedPtr msg) {
  switch (msg->drive_mode) {
    case mcu_msgs::msg::DriveBase::DRIVE_VECTOR: {
      if (current_mode_ == DriveMode::TRAJECTORY_DRIVE &&
          !active_traj_.empty()) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Ignoring DRIVE_VECTOR because trajectory is active");
        return;
      }

      // Same as RobotDriveBase::driveVelocity()
      float vx = msg->goal_velocity.linear.x;
      float omega = msg->goal_velocity.angular.z;

      current_mode_ = DriveMode::VELOCITY_DRIVE;
      target_velocity_ = Vector2D(vx, 0.0f, omega);

      // Reset profiles to current velocity to avoid jumps
      linear_profile_.reset(
          {current_velocity_.getX(), current_velocity_.getX(), 0.0f});
      angular_profile_.reset(
          {current_velocity_.getTheta(), current_velocity_.getTheta(), 0.0f});

      // Set target velocity as goal
      linear_profile_.setGoal(
          {target_velocity_.getX(), target_velocity_.getX()});
      angular_profile_.setGoal(
          {target_velocity_.getTheta(), target_velocity_.getTheta()});

      left_pid_.reset();
      right_pid_.reset();

      RCLCPP_DEBUG(this->get_logger(), "Drive VELOCITY: vx=%.2f, omega=%.2f",
                   vx, omega);
      break;
    }

    case mcu_msgs::msg::DriveBase::DRIVE_GOAL: {
      // Same as DriveSubsystem::drive_callback DRIVE_GOAL case
      double qz = msg->goal_transform.transform.rotation.z;
      double qw = msg->goal_transform.transform.rotation.w;
      float target_yaw = std::atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz));

      current_mode_ = DriveMode::POSE_DRIVE;
      target_pose_ =
          Pose2D(msg->goal_transform.transform.translation.x,
                 msg->goal_transform.transform.translation.y, target_yaw);

      left_pid_.reset();
      right_pid_.reset();

      RCLCPP_DEBUG(this->get_logger(), "Drive POSE: x=%.2f, y=%.2f, theta=%.2f",
                   target_pose_.getX(), target_pose_.getY(), target_yaw);
      break;
    }

    default: {
      // Stop (same as real MCU default case)
      current_mode_ = DriveMode::VELOCITY_DRIVE;
      target_velocity_ = Vector2D(0, 0, 0);
      gz_cmd_v_ = 0.0;
      gz_cmd_w_ = 0.0;

      linear_profile_.reset(
          {current_velocity_.getX(), current_velocity_.getX(), 0.0f});
      angular_profile_.reset(
          {current_velocity_.getTheta(), current_velocity_.getTheta(), 0.0f});
      linear_profile_.setGoal({0.0f, 0.0f});
      angular_profile_.setGoal({0.0f, 0.0f});

      left_pid_.reset();
      right_pid_.reset();
      break;
    }
  }
}

// cmd_vel callback, convenience bridge, same as driveVelocity
void McuSubsystemSimulator::cmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  if (current_mode_ == DriveMode::TRAJECTORY_DRIVE && !active_traj_.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Ignoring /cmd_vel_in because trajectory is active");
    return;
  }
  static constexpr float M_TO_IN = 39.37007874f;
  float vx = msg->linear.x * M_TO_IN;
  float omega = msg->angular.z;

  current_mode_ = DriveMode::VELOCITY_DRIVE;
  target_velocity_ = Vector2D(vx, 0.0f, omega);

  linear_profile_.reset(
      {current_velocity_.getX(), current_velocity_.getX(), 0.0f});
  angular_profile_.reset(
      {current_velocity_.getTheta(), current_velocity_.getTheta(), 0.0f});

  linear_profile_.setGoal({target_velocity_.getX(), target_velocity_.getX()});
  angular_profile_.setGoal(
      {target_velocity_.getTheta(), target_velocity_.getTheta()});

  left_pid_.reset();
  right_pid_.reset();

  RCLCPP_DEBUG(this->get_logger(), "cmd_vel: vx=%.2f, omega=%.2f", vx, omega);
}

// Gazebo ground-truth odom callback - gives us the real robot position in sim
void McuSubsystemSimulator::gzOdomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  gz_odom_x_ = msg->pose.pose.position.x;
  gz_odom_y_ = msg->pose.pose.position.y;

  // Extract yaw from quaternion
  double qz = msg->pose.pose.orientation.z;
  double qw = msg->pose.pose.orientation.w;
  gz_odom_yaw_ = 2.0 * std::atan2(qz, qw);

  gz_odom_received_ = true;
}

// Main update loop, mirrors RobotDriveBase::update(yaw, dt)
void McuSubsystemSimulator::updateTimerCallback() {
  rclcpp::Time current_time = this->now();
  float dt = static_cast<float>((current_time - last_update_time_).seconds());
  last_update_time_ = current_time;

  if (dt < 0.001f) return;

  // First we simulate physics, motor PWM -> wheel velocity -> encoder ticks
  simulatePhysics(dt);

  // Then we read simulated encoders (cast accumulator to long, same as real
  // encoder read)
  long left_ticks = static_cast<long>(left_encoder_accum_);
  long right_ticks = static_cast<long>(right_encoder_accum_);

  // Then compute current velocity from encoder tick deltas
  // Which should be the same as RobotDriveBase::getCurrentVelocity
  long d_left = left_ticks - prev_left_ticks_;
  long d_right = right_ticks - prev_right_ticks_;

  float left_dist = static_cast<float>(d_left) * dist_per_tick_;
  float right_dist = static_cast<float>(d_right) * dist_per_tick_;
  float left_vel = left_dist / dt;
  float right_vel = right_dist / dt;

  float vx = 0.5f * (left_vel + right_vel);
  float omega = (right_vel - left_vel) / track_width_;

  current_velocity_ = Vector2D(vx, 0.0f, omega);

  // Then integrate yaw from wheel angular velocity (simulated IMU)
  sim_yaw_ += omega * dt;
  while (sim_yaw_ > static_cast<float>(M_PI))
    sim_yaw_ -= 2.0f * static_cast<float>(M_PI);
  while (sim_yaw_ < -static_cast<float>(M_PI))
    sim_yaw_ += 2.0f * static_cast<float>(M_PI);

  // Then we update localization (same call as real MCU)
  localization_->update(left_ticks, right_ticks, sim_yaw_);

  // Then we dispatch control mode (same switch as RobotDriveBase::update)
  switch (current_mode_) {
    case DriveMode::MANUAL:
      gz_cmd_v_ = 0.0;
      gz_cmd_w_ = 0.0;
      break;
    case DriveMode::VELOCITY_DRIVE:
      velocityControl(dt);
      break;
    case DriveMode::POSE_DRIVE:
      setPointControl(dt);
      break;
    case DriveMode::TRAJECTORY_DRIVE:
      trajectoryControl(dt);
      break;
  }

  // THEN WE FINALLY SAVE PREVIOUS STATE (same as real MCU)
  prev_pose_ = localization_->getPose();
  prev_left_ticks_ = left_ticks;
  prev_right_ticks_ = right_ticks;

  // Convert inches/s -> m/s if you're running inches internally
  static constexpr double IN_TO_M = 0.0254;

  // Publish commanded velocity directly to Gazebo (not internal encoder
  // velocity)
  geometry_msgs::msg::Twist out;
  out.linear.x = gz_cmd_v_;
  out.angular.z = gz_cmd_w_;
  cmd_vel_pub_->publish(out);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                       "[CMD_VEL] mode=%d  cmd_vel_out v=%.4f m/s w=%.3f rad/s",
                       static_cast<int>(current_mode_), out.linear.x,
                       out.angular.z);
}

// velocityControl, mirrors RobotDriveBase::velocityControl exactly
void McuSubsystemSimulator::velocityControl(float dt) {
  if (dt < 0.001f) return;

  // S-curve motion profiles -> smooth ramped velocities
  MotionState linear_state = linear_profile_.update(dt);
  MotionState angular_state = angular_profile_.update(dt);

  float ramped_v = linear_state.vel;
  float ramped_omega = angular_state.vel;

  // Send S-curve profiled velocity directly to Gazebo (bypass internal PID)
  static constexpr float IN_TO_M_VEL = 0.0254f;
  gz_cmd_v_ = ramped_v * IN_TO_M_VEL;
  gz_cmd_w_ = ramped_omega;

  // Inverse kinematics: chassis (v, omega) -> wheel velocities
  float target_left_vel = ramped_v - (ramped_omega * track_width_) * 0.5f;
  float target_right_vel = ramped_v + (ramped_omega * track_width_) * 0.5f;

  // Actual wheel velocities from encoder deltas
  float actual_left_vel =
      (static_cast<float>(static_cast<long>(left_encoder_accum_) -
                          prev_left_ticks_) *
       dist_per_tick_) /
      dt;
  float actual_right_vel =
      (static_cast<float>(static_cast<long>(right_encoder_accum_) -
                          prev_right_ticks_) *
       dist_per_tick_) /
      dt;

  // PID control on wheel velocities
  float left_output = left_pid_.update(target_left_vel, actual_left_vel, dt);
  float right_output =
      right_pid_.update(target_right_vel, actual_right_vel, dt);

  writeMotorSpeeds(static_cast<int>(left_output),
                   static_cast<int>(right_output));
}

// setPointControl, mirrors RobotDriveBase::setPointControl exactly
void McuSubsystemSimulator::setPointControl(float dt) {
  Pose2D current_pose = localization_->getPose();

  float delta_x = target_pose_.getX() - current_pose.getX();
  float delta_y = target_pose_.getY() - current_pose.getY();
  float distance_error = std::hypot(delta_x, delta_y);
  float target_angle = std::atan2(delta_y, delta_x);
  float angle_error = std::remainder(target_angle - current_pose.getTheta(),
                                     2.0f * static_cast<float>(M_PI));

  // Stop if close to target (same threshold as real MCU)
  const float DISTANCE_THRESHOLD = 0.5f;
  if (distance_error < DISTANCE_THRESHOLD) {
    target_velocity_ = Vector2D(0, 0, 0);
    velocityControl(dt);
    return;
  }

  // P-controller (same gains as real MCU)
  const float K_LINEAR = 2.0f;
  const float K_ANGULAR = 4.0f;

  float v_cmd = K_LINEAR * distance_error;
  float omega_cmd = K_ANGULAR * angle_error;

  // Slow down when not facing target (same optimization as real MCU)
  v_cmd *= std::max(0.0f, std::cos(angle_error));

  // Clamp to limits
  v_cmd = std::clamp(v_cmd, -max_velocity_, max_velocity_);
  omega_cmd =
      std::clamp(omega_cmd, -max_angular_velocity_, max_angular_velocity_);

  target_velocity_ = Vector2D(v_cmd, 0.0f, omega_cmd);
  velocityControl(dt);
}

// trajectoryControl, mirrors RobotDriveBase::trajectoryControl exactly
void McuSubsystemSimulator::trajectoryControl(float dt) {
  if (dt < 0.001f) return;

  static constexpr float M_TO_IN = 39.37007874f;

  // In simulation, use Gazebo's ground-truth odom so the trajectory controller
  // knows where the robot ACTUALLY is (not where the simulated encoders think).
  // This prevents the robot from overshooting the goal.
  TrajectoryController::Pose2D traj_pose;
  if (gz_odom_received_) {
    traj_pose.x = static_cast<float>(gz_odom_x_);
    traj_pose.y = static_cast<float>(gz_odom_y_);
    traj_pose.theta = static_cast<float>(gz_odom_yaw_);
  } else {
    // Fallback to internal localization (inches -> meters) if no odom yet
    static constexpr float IN_TO_M = 0.0254f;
    Pose2D current_pose = localization_->getPose();
    traj_pose.x = current_pose.getX() * IN_TO_M;
    traj_pose.y = current_pose.getY() * IN_TO_M;
    traj_pose.theta = current_pose.getTheta();
  }

  TrajectoryController::Command cmd = traj_controller_.update(traj_pose, dt);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                       "[TRAJ] src=%s pose=(%.3f,%.3f,%.2f) cmd v=%.3f m/s "
                       "w=%.3f rad/s finished=%d",
                       gz_odom_received_ ? "gz_odom" : "FALLBACK", traj_pose.x,
                       traj_pose.y, traj_pose.theta, cmd.v, cmd.w,
                       cmd.finished);

  if (cmd.finished) {
    RCLCPP_INFO(this->get_logger(),
                "[TRAJ] Trajectory FINISHED at pose=(%.3f,%.3f,%.2f)",
                traj_pose.x, traj_pose.y, traj_pose.theta);
    writeMotorSpeeds(0, 0);
    gz_cmd_v_ = 0.0;
    gz_cmd_w_ = 0.0;
    active_traj_.clear();
    traj_controller_.clearTrajectory();
    current_mode_ = DriveMode::VELOCITY_DRIVE;
    target_velocity_ = Vector2D(0, 0, 0);
    return;
  }

  // Send trajectory controller output directly to Gazebo (bypass internal PID)
  gz_cmd_v_ = cmd.v;
  gz_cmd_w_ = cmd.w;
}

// writeMotorSpeeds, stores PWM for physics simulation
void McuSubsystemSimulator::writeMotorSpeeds(int left_pwm, int right_pwm) {
  left_motor_pwm_ = std::clamp(left_pwm, -255, 255);
  right_motor_pwm_ = std::clamp(right_pwm, -255, 255);
}

// simulatePhysics, motor PWM -> wheel velocity -> encoder tick accumulation
void McuSubsystemSimulator::simulatePhysics(float dt) {
  // Simple motor model: PWM -> wheel velocity (in/s)
  // At PWM=255 the wheel spins at max_wheel_velocity_
  float left_wheel_vel =
      (static_cast<float>(left_motor_pwm_) / 255.0f) * max_wheel_velocity_;
  float right_wheel_vel =
      (static_cast<float>(right_motor_pwm_) / 255.0f) * max_wheel_velocity_;

  // Accumulate encoder ticks (fractional, cast to long on read)
  left_encoder_accum_ +=
      static_cast<double>(left_wheel_vel) * dt / dist_per_tick_;
  right_encoder_accum_ +=
      static_cast<double>(right_wheel_vel) * dt / dist_per_tick_;
}

// Drive publish, mirrors DriveSubsystem::publishData
void McuSubsystemSimulator::drivePublishCallback() {
  static constexpr double IN_TO_M = 0.0254;

  auto msg = mcu_msgs::msg::DriveBase();

  msg.header.stamp = this->now();
  msg.header.frame_id = "odom";

  // Transform (pose) — use Gazebo odom in sim for accurate TF
  msg.transform.header.stamp = msg.header.stamp;
  msg.transform.header.frame_id = "odom";
  msg.transform.child_frame_id = "base_link";

  double pub_x, pub_y;
  float yaw;
  if (gz_odom_received_) {
    pub_x = gz_odom_x_;
    pub_y = gz_odom_y_;
    yaw = static_cast<float>(gz_odom_yaw_);
  } else {
    Pose2D pose = localization_->getPose();
    pub_x = pose.getX() * IN_TO_M;
    pub_y = pose.getY() * IN_TO_M;
    yaw = pose.getTheta();
  }

  msg.transform.transform.translation.x = pub_x;
  msg.transform.transform.translation.y = pub_y;
  msg.transform.transform.translation.z = 0.0;

  msg.transform.transform.rotation.x = 0.0;
  msg.transform.transform.rotation.y = 0.0;
  msg.transform.transform.rotation.z = std::sin(yaw / 2.0);
  msg.transform.transform.rotation.w = std::cos(yaw / 2.0);

  // Twist (velocity) — convert in/s to m/s for ROS convention
  msg.twist.linear.x = current_velocity_.getX() * IN_TO_M;
  msg.twist.linear.y = current_velocity_.getY() * IN_TO_M;
  msg.twist.linear.z = 0.0;
  msg.twist.angular.x = 0.0;
  msg.twist.angular.y = 0.0;
  msg.twist.angular.z = current_velocity_.getTheta();

  msg.drive_mode = mcu_msgs::msg::DriveBase::DRIVE_VECTOR;

  drive_status_pub_->publish(msg);

  RCLCPP_DEBUG(
      this->get_logger(),
      "Drive status: x=%.4f m, y=%.4f m, theta=%.2f, vx=%.4f m/s, omega=%.2f",
      pub_x, pub_y, yaw, current_velocity_.getX() * IN_TO_M,
      current_velocity_.getTheta());

  // TF broadcast so "odom" exists in /tf
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = msg.header.stamp;
  t.header.frame_id = "odom";
  t.child_frame_id = "base_link";
  t.transform = msg.transform.transform;
  tf_broadcaster_->sendTransform(t);
}

// IMU publish
void McuSubsystemSimulator::imuPublishCallback() {
  auto msg = sensor_msgs::msg::Imu();

  msg.header.stamp = this->now();
  msg.header.frame_id = "imu_link";

  // Orientation from simulated yaw
  msg.orientation.x = 0.0;
  msg.orientation.y = 0.0;
  msg.orientation.z = std::sin(sim_yaw_ / 2.0);
  msg.orientation.w = std::cos(sim_yaw_ / 2.0);

  msg.orientation_covariance[0] = kOrientationVariance;
  msg.orientation_covariance[4] = kOrientationVariance;
  msg.orientation_covariance[8] = kOrientationVariance;

  // Angular velocity
  msg.angular_velocity.x = 0.0;
  msg.angular_velocity.y = 0.0;
  msg.angular_velocity.z = current_velocity_.getTheta();

  msg.angular_velocity_covariance[0] = kAngularVelocityVariance;
  msg.angular_velocity_covariance[4] = kAngularVelocityVariance;
  msg.angular_velocity_covariance[8] = kAngularVelocityVariance;

  // Linear acceleration from velocity change
  double dt = 0.02;  // 50 Hz publish rate
  double accel_x =
      (static_cast<double>(current_velocity_.getX()) - prev_vel_x_) / dt;
  prev_vel_x_ = current_velocity_.getX();

  msg.linear_acceleration.x = accel_x;
  msg.linear_acceleration.y = 0.0;
  msg.linear_acceleration.z = 9.81;

  msg.linear_acceleration_covariance[0] = kLinearAccelerationVariance;
  msg.linear_acceleration_covariance[4] = kLinearAccelerationVariance;
  msg.linear_acceleration_covariance[8] = kLinearAccelerationVariance;

  imu_pub_->publish(msg);
}

// TOF publish
void McuSubsystemSimulator::tofPublishCallback() {
  auto msg = std_msgs::msg::Float32MultiArray();
  msg.data.resize(num_tof_sensors_, 1.0f);
  tof_pub_->publish(msg);
}

// RC publish
void McuSubsystemSimulator::rcPublishCallback() {
  auto msg = mcu_msgs::msg::RC();

  msg.header.stamp = this->now();
  msg.header.frame_id = "base_link";

  for (int i = 0; i < 10; i++) {
    msg.channels[i] = 0;
  }

  msg.swa = false;
  msg.swb = false;
  msg.swc = false;
  msg.swd = false;
  msg.knobl = 0;
  msg.knobr = 0;
  msg.lx = 0;
  msg.ly = 0;
  msg.rx = 0;
  msg.ry = 0;

  rc_pub_->publish(msg);
}

// Intake publish (improved: uses simulated state instead of always IDLE)
void McuSubsystemSimulator::intakePublishCallback() {
  auto msg = mcu_msgs::msg::IntakeState();

  msg.header.stamp = this->now();
  msg.header.frame_id = "base_link";

  msg.state = sim_intake_state_;
  msg.duck_detected = sim_duck_detected_;

  if (sim_intake_speed_ > 0)
    msg.motor_state = mcu_msgs::msg::IntakeState::MOTOR_FORWARD;
  else if (sim_intake_speed_ < 0)
    msg.motor_state = mcu_msgs::msg::IntakeState::MOTOR_REVERSE;
  else
    msg.motor_state = mcu_msgs::msg::IntakeState::MOTOR_OFF;

  msg.capture_count = 0;
  msg.jam_count = 0;

  auto elapsed = std::chrono::steady_clock::now() - intake_state_time_;
  msg.time_in_state_ms = static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count());

  intake_pub_->publish(msg);
}

// Intake speed callback
void McuSubsystemSimulator::intakeSpeedCallback(
    const std_msgs::msg::Int16::SharedPtr msg) {
  sim_intake_speed_ = msg->data;

  if (msg->data > 0 &&
      sim_intake_state_ == mcu_msgs::msg::IntakeState::STATE_IDLE) {
    sim_intake_state_ = mcu_msgs::msg::IntakeState::STATE_SPINNING;
    intake_state_time_ = std::chrono::steady_clock::now();
    sim_duck_detected_ = false;
    RCLCPP_INFO(this->get_logger(), "[SIM] Intake spinning (speed=%d)",
                msg->data);
  } else if (msg->data < 0) {
    sim_intake_state_ = mcu_msgs::msg::IntakeState::STATE_EJECTING;
    intake_state_time_ = std::chrono::steady_clock::now();
    sim_duck_detected_ = false;
    RCLCPP_INFO(this->get_logger(), "[SIM] Intake ejecting");
  } else if (msg->data == 0) {
    sim_intake_state_ = mcu_msgs::msg::IntakeState::STATE_IDLE;
    intake_state_time_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "[SIM] Intake off");
  }
}

// Bridge command callback
void McuSubsystemSimulator::bridgeCommandCallback(
    const mcu_msgs::msg::IntakeBridgeCommand::SharedPtr msg) {
  bridge_cmd_time_ = std::chrono::steady_clock::now();

  switch (msg->command) {
    case mcu_msgs::msg::IntakeBridgeCommand::CMD_EXTEND:
      if (sim_bridge_state_ == mcu_msgs::msg::IntakeBridgeState::STATE_STOWED) {
        sim_bridge_state_ = mcu_msgs::msg::IntakeBridgeState::STATE_EXTENDING;
        RCLCPP_INFO(this->get_logger(), "[SIM] Bridge extending");
      }
      break;
    case mcu_msgs::msg::IntakeBridgeCommand::CMD_RETRACT:
      if (sim_bridge_state_ ==
              mcu_msgs::msg::IntakeBridgeState::STATE_EXTENDED ||
          sim_bridge_state_ ==
              mcu_msgs::msg::IntakeBridgeState::STATE_EXTENDING) {
        sim_bridge_state_ = mcu_msgs::msg::IntakeBridgeState::STATE_RETRACTING;
        bridge_cmd_time_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "[SIM] Bridge retracting");
      }
      break;
    case mcu_msgs::msg::IntakeBridgeCommand::CMD_STOW:
    default:
      sim_bridge_state_ = mcu_msgs::msg::IntakeBridgeState::STATE_STOWED;
      RCLCPP_INFO(this->get_logger(), "[SIM] Bridge stowed");
      break;
  }
}

// Arm command callback (logs servo commands for camera mast, latches)
void McuSubsystemSimulator::armCommandCallback(
    const mcu_msgs::msg::ArmCommand::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(),
              "[SIM] Arm command: joint=%d, position=%d, speed=%d",
              msg->joint_id, msg->position, msg->speed);
}

// Bridge publish callback (with time-based state transitions)
void McuSubsystemSimulator::bridgePublishCallback() {
  // Time-based state transitions
  auto elapsed = std::chrono::steady_clock::now() - bridge_cmd_time_;
  double sec =
      std::chrono::duration_cast<std::chrono::duration<double>>(elapsed)
          .count();

  if (sim_bridge_state_ == mcu_msgs::msg::IntakeBridgeState::STATE_EXTENDING &&
      sec >= 2.0) {
    sim_bridge_state_ = mcu_msgs::msg::IntakeBridgeState::STATE_EXTENDED;
    RCLCPP_INFO(this->get_logger(), "[SIM] Bridge fully extended");
  } else if (sim_bridge_state_ ==
                 mcu_msgs::msg::IntakeBridgeState::STATE_RETRACTING &&
             sec >= 2.0) {
    sim_bridge_state_ = mcu_msgs::msg::IntakeBridgeState::STATE_STOWED;
    RCLCPP_INFO(this->get_logger(), "[SIM] Bridge fully stowed");
  }

  auto msg = mcu_msgs::msg::IntakeBridgeState();
  msg.header.stamp = this->now();
  msg.header.frame_id = "base_link";
  msg.state = sim_bridge_state_;

  // Simulate duck detection when extended (auto-detect after 1s for testing)
  if (sim_bridge_state_ == mcu_msgs::msg::IntakeBridgeState::STATE_EXTENDED) {
    auto ext_elapsed = std::chrono::steady_clock::now() - bridge_cmd_time_;
    double ext_sec =
        std::chrono::duration_cast<std::chrono::duration<double>>(ext_elapsed)
            .count();
    msg.duck_detected = (ext_sec >= 1.0);  // auto-detect after 1s in sim
    msg.tof_distance_mm = msg.duck_detected ? 30 : 200;
  } else {
    msg.duck_detected = false;
    msg.tof_distance_mm = 0;
  }

  if (sim_bridge_state_ == mcu_msgs::msg::IntakeBridgeState::STATE_EXTENDING)
    msg.rack_motor_state = mcu_msgs::msg::IntakeBridgeState::MOTOR_EXTENDING;
  else if (sim_bridge_state_ ==
           mcu_msgs::msg::IntakeBridgeState::STATE_RETRACTING)
    msg.rack_motor_state = mcu_msgs::msg::IntakeBridgeState::MOTOR_RETRACTING;
  else
    msg.rack_motor_state = mcu_msgs::msg::IntakeBridgeState::MOTOR_OFF;

  msg.extend_count = 0;
  msg.retract_count = 0;

  auto state_elapsed = std::chrono::steady_clock::now() - bridge_cmd_time_;
  msg.time_in_state_ms = static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(state_elapsed)
          .count());

  bridge_pub_->publish(msg);
}

// MiniRobot publish
void McuSubsystemSimulator::miniRobotPublishCallback() {
  auto msg = mcu_msgs::msg::MiniRobotState();

  msg.header.stamp = this->now();
  msg.header.frame_id = "base_link";

  msg.state = mcu_msgs::msg::MiniRobotState::ARMED;
  msg.current_task = mcu_msgs::msg::MiniRobotState::TASK_NONE;

  mini_robot_pub_->publish(msg);
}

// Battery publish
void McuSubsystemSimulator::batteryPublishCallback() {
  auto msg = mcu_msgs::msg::BatteryHealth();

  msg.header.stamp = this->now();
  msg.header.frame_id = "base_link";

  msg.voltage = 12.0;
  msg.shunt_voltage = 0.0;
  msg.current = 1.0;
  msg.temperature = 25.0;
  msg.power = 12.0;
  msg.energy = 0.0;
  msg.charge_use = 0.0;

  battery_pub_->publish(msg);

  RCLCPP_DEBUG(this->get_logger(), "Battery health published");
}

// Heartbeat publish
void McuSubsystemSimulator::heartbeatPublishCallback() {
  auto msg = std_msgs::msg::String();
  msg.data = "HEARTBEAT";

  heartbeat_pub_->publish(msg);

  RCLCPP_DEBUG(this->get_logger(), "Heartbeat published");
}

// MCU State publish, always RUNNING in sim
void McuSubsystemSimulator::mcuStatePublishCallback() {
  auto msg = mcu_msgs::msg::McuState();

  msg.header.stamp = this->now();
  msg.header.frame_id = "base_link";
  msg.state = mcu_msgs::msg::McuState::RUNNING;

  mcu_state_pub_->publish(msg);
}

}  // namespace secbot_sim

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<secbot_sim::McuSubsystemSimulator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
