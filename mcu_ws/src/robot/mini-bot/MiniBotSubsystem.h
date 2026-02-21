/**
 * @file MiniBotIMUSubsystem.h
 * @brief Mini robot node for overall control, imu etc.
 * @author Trevor Cannon
 * @date 2/20/2026
 */

#include <BaseSubsystem.h>
#include <I2CPowerDriver.h>
#include <MPU6050.h>
#include <Pose2D.h>
#include <TimedSubsystem.h>
#include <UWBDriver.h>
#include <Vector2D.h>
#include <micro_ros_utilities/type_utilities.h>
#include <microros_manager_robot.h>
#include <traj_controller.h>

namespace Susbsytem {
class MiniBotSubsystemSetup : Classes::BaseSetup {
 public:
  Drivers::MPU6050Driver mpu_setup;
  Drivers::UWBDriver uwb_setup;
  // MiniBotDriveBaseSetup minibot_drivebase_setup;

  MiniBotSubsystemSetup(const char* _id,
                        const Drivers::MPU6050DriverSetup& _mpu_setup,
                        const Drivers::UWBDriverSetup& _uwb_setup)
      : Classes::BaseSetup(_id), mpu_setup(_mpu_setup), uwb_setup(_uwb_setup) {}
};

class MiniBotSubsystem : public Subsystem::TimedSubsystem,
                         public IMicroRosParticipant {
 public:
  explicit MiniBotSubsystemSetup(const MiniBotSubsystemSetup& setup)
      : Subsystem::TimedSubsystem(setup_) {}

  void begin() override;
  void update() override;
  void reset() override;
  void pause() override;
  const char* getInfo() override;
  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override;
  void onDestroy() override;

  void imu_pub();
  void uwb_pub();
  void battery_pub();
  void heartbeat_pub();

 private:
  void drive_callback(const void* msgin, void* context);
  void led_callback(const void* msgin, void* context);

  void calibrate_imu();

  constexpr int CALIBRATE_SAMPLES = 100;
  bool calibrated_ = false;

  float a_x_off_ = 0.0f;
  float a_y_off_ = 0.0f;
  float a_z_off_ = 0.0f;

  float g_x_off_ = 0.0f;
  float g_y_off_ = 0.0f;
  float g_z_off_ = 0.0f;

  const Drivers::MPU6050Driver mpu_;
  const Drivers::UWBDriver uwb_;
  const Drivers::I2CPowerDriver batt_;
  // const drivebase

  Drivers::MPU6050DriverData mpu_data_;
  Drivers::I2CPowerDriverData batt_data_;
  Drivers::UWBDriverData uwb_data_;

  rcl_node_t* node_ = nullptr;
  rclc_executor_t* executor_ = nullptr;

  rcl_publisher_t imu_pub_;
  rcl_publisher_t uwb_pub_;
  rcl_publisher_t bat_pub_;
  rcl_publisher_t heart_pub_;

  rcl_subscription_t drive_sub_;
  rcl_subscription_t led_sub_;

  sensor_msgs__msg__Imu mpu_msg_;
  mcu_msgs__msg__UWBRanging uwb_msg_;
  sensor_msgs__msg__BatteryHealth batt_msg_;
  std_msgs__msg__Bool led_msg_;
};

}  // namespace Susbsytem