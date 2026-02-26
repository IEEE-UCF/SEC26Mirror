/**
 * @file ImuSubsystem.h
 * @author Rafeed Khan
 * @brief IMU Subsystem for the SEC2026 Robot.
 *
 * Publishes sensor_msgs/Imu to /mcu_robot/imu/data for EKF sensor fusion.
 */

#ifndef IMUSUBSYSTEM_H
#define IMUSUBSYSTEM_H

#include <BNO085.h>
<<<<<<< HEAD
<<<<<<< HEAD

=======
#include <BaseSubsystem.h>
>>>>>>> 788e3a1a00f0f8138e28a6ee774f71d46d6817ca
=======
#include <BaseSubsystem.h>
>>>>>>> 1efe8ca348ef17c6d21aa1f5e5f0f24367ae9fe0
#include <microros_manager_robot.h>
#include <sensor_msgs/msg/imu.h>

#ifdef USE_FREERTOS
#include "arduino_freertos.h"
#endif

namespace Subsystem {

class ImuSubsystemSetup : public Classes::BaseSetup {
 public:
  ImuSubsystemSetup(const char* _id, Drivers::BNO085Driver* driver)
      : Classes::BaseSetup(_id), driver_(driver) {}
  Drivers::BNO085Driver* driver_ = nullptr;
};

class ImuSubsystem : public IMicroRosParticipant,
                     public Classes::BaseSubsystem {
 public:
  explicit ImuSubsystem(const ImuSubsystemSetup& setup)
      : Classes::BaseSubsystem(setup), setup_(setup) {}

  bool init() override;
  void begin() override {}
  void update() override;
  void pause() override {}
  void reset() override;
  const char* getInfo() override;

  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override;
  void onDestroy() override;

  void publishData();

#ifdef USE_FREERTOS
  void beginThreaded(uint32_t stackSize, UBaseType_t priority,
                     uint32_t updateRateMs = 20) {
    task_delay_ms_ = updateRateMs;
    xTaskCreate(taskFunction, getInfo(), stackSize, this, priority, nullptr);
  }

 private:
  static void taskFunction(void* pvParams) {
    auto* self = static_cast<ImuSubsystem*>(pvParams);
    self->begin();
    while (true) {
      self->update();
      vTaskDelay(pdMS_TO_TICKS(self->task_delay_ms_));
    }
  }
  uint32_t task_delay_ms_ = 20;
#endif

 private:
  void initCovariances();

  const ImuSubsystemSetup setup_;
  rcl_publisher_t pub_{};
  sensor_msgs__msg__Imu msg_{};
  rcl_node_t* node_ = nullptr;

  // Gyro calibration offsets (they're calculated during init)
  float gyro_offset_x_ = 0.0f;
  float gyro_offset_y_ = 0.0f;
  float gyro_offset_z_ = 0.0f;
  bool calibrated_ = false;

  // Dead zone threshold for angular velocity (rad/s)
  // Values below this are zeroed to reduce noise!
  static constexpr float kGyroDeadZone = 0.01f;

  // Covariance values based on BNO085 typical specs
  // Orientation: around 2 degrees accuracy -> variance around 0.001 rad^2
  static constexpr double kOrientationVariance = 0.001;
  // Angular velocity: around 0.1 deg/s noise -> variance around 0.00003
  // rad^2/s^2
  static constexpr double kAngularVelocityVariance = 0.00003;
  // Linear acceleration: around 0.01 m/s^2 noise -> variance around 0.0001
  // m^2/s^4
  static constexpr double kLinearAccelerationVariance = 0.0001;

  static constexpr int kCalibrationSamples = 40;
  static constexpr int kCalibrationDelayMs = 50;
};

}  // namespace Subsystem

#endif
