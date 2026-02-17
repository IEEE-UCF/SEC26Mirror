/**
 * @file MiniBotIMUSubsystem.h
 * @brief IMU subsystem for MiniBot using MPU6050
 * @author Trevor Cannon
 * @date 2/17/2026
 */

#pragma once

#include <BaseSubsystem.h>
#include <TimedSubsystem.h>
#include <micro_ros_utilities/type_utilities.h>
#include <microros_manager_robot.h>

#include "MPU6050.h"

namespace Subsystem {

class MiniBotIMUSubsystemSetup : public Classes::BaseSetup {
 public:
  Drivers::MPU6050DriverSetup mpuSetup_;

  MiniBotIMUSubsystemSetup() = delete;
  MiniBotIMUSubsystemSetup(const char* _id,
                           const Drivers::MPU6050DriverSetup& _mpuSetup)
      : Classes::BaseSetup(_id), mpuSetup_(_mpuSetup) {};
}

class MiniBotIMUSubsystem : public IMicroRosParticipant,
                            public Subsystem::TimedSubsystem {
 public:
  explicit MiniBotIMUSubsystem(const MiniBotIMUSubsystemSetup& setup)
      : Subsystem::TimeSubsystem(setup), setup_(setup), mpu_(mpuSetup_) {};

  void begin() override;
  void update() override;
  void pause() override;
  void reset() override;
  const char* getInfo() override;

  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override;
  void onDestroy() override;

  void publishData() override;

 private:
  const MiniBotIMUSubsystemSetup setup_;
  Drivers::MPU6050Driver mpu_;
  Drivers::MPU6050DriverData data_;

  rcl_node_t* node_ = nullptr;
  rclc_executor_t* executor_ = nullptr;
  rcl_publisher_t mpu_pub_;

  sensor_msgs__msg__Imu mpu_msg_;
}
}  // namespace Subsystem
