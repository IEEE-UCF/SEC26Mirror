/**
 * @file MiniBotIMUSubsystem.h
 * @brief IMU subsystem for MiniBot using MPU6050
 * @author Trevor Cannon
 * @date 2/17/2026
 */

#pragma once

#include <TimedSubsystem.h>
#include <micro_ros_utilities/type_utilities.h>
#include <microros_manager_robot.h>
#include <sensor_msgs/msg/imu.h>

#include "MPU6050.h"

namespace Subsystem {

class MiniBotIMUSubsystemSetup : public Classes::BaseSetup {
 public:
  Drivers::MPU6050DriverSetup mpuSetup_;

  MiniBotIMUSubsystemSetup() = delete;
  MiniBotIMUSubsystemSetup(const char* _id,
                           const Drivers::MPU6050DriverSetup& _mpuSetup)
      : Classes::BaseSetup(_id), mpuSetup_(_mpuSetup) {}
};

class MiniBotIMUSubsystem : public IMicroRosParticipant,
                            public Subsystem::TimedSubsystem {
 public:
  explicit MiniBotIMUSubsystem(const MiniBotIMUSubsystemSetup& setup)
      : Subsystem::TimedSubsystem(setup),
        setup_(setup),
        mpu_(setup_.mpuSetup_) {}

  bool init() override;
  void begin() override;
  void update() override;
  void pause() override;
  void reset() override;
  const char* getInfo() override;

  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override;
  void onDestroy() override;

 private:
  void publishData();
  void calibrate();

  static constexpr int CALIBRATE_SAMPLES = 100;
  bool calibrated_ = false;

  float a_x_off_ = 0.0f;
  float a_y_off_ = 0.0f;
  float a_z_off_ = 0.0f;

  float g_x_off_ = 0.0f;
  float g_y_off_ = 0.0f;
  float g_z_off_ = 0.0f;

  const MiniBotIMUSubsystemSetup setup_;
  Drivers::MPU6050Driver mpu_;
  Drivers::MPU6050DriverData data_;

  rcl_node_t* node_ = nullptr;
  rcl_publisher_t mpu_pub_{};

  sensor_msgs__msg__Imu mpu_msg_{};
};

}  // namespace Subsystem
