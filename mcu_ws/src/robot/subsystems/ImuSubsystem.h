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
#include <RTOSSubsystem.h>
#include <mcu_msgs/srv/reset.h>
#include <microros_manager_robot.h>
#include <sensor_msgs/msg/imu.h>
#include <atomic>

namespace Subsystem {

class ImuSubsystemSetup : public Classes::BaseSetup {
 public:
  ImuSubsystemSetup(const char* _id, Drivers::BNO085Driver* driver)
      : Classes::BaseSetup(_id), driver_(driver) {}
  Drivers::BNO085Driver* driver_ = nullptr;
};

class ImuSubsystem : public IMicroRosParticipant,
                     public Subsystem::RTOSSubsystem {
 public:
  explicit ImuSubsystem(const ImuSubsystemSetup& setup)
      : Subsystem::RTOSSubsystem(setup), setup_(setup) {}

  bool init() override;
  void begin() override {}
  void update() override;
  void pause() override {}
  void reset() override;
  const char* getInfo() override;

  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override;
  void onDestroy() override;
  void publishAll() override;

  void publishData();

  /** @brief Get latest yaw (Z-axis rotation) in radians. Thread-safe (std::atomic). */
  float getYaw() const { return yaw_rad_.load(std::memory_order_relaxed); }

  /** @brief Zero the IMU heading via BNO085 hardware tare. */
  bool tare();

 private:
  void initCovariances();

  const ImuSubsystemSetup setup_;
  rcl_publisher_t pub_{};
  sensor_msgs__msg__Imu msg_{};
  rcl_node_t* node_ = nullptr;
  std::atomic<float> yaw_rad_{0.0f};

  // Orientation covariance (BNO085 Game Rotation Vector: ~2 deg accuracy)
  static constexpr double kOrientationVariance = 0.001;

  // Tare service
  rcl_service_t tare_srv_{};
  mcu_msgs__srv__Reset_Request tare_req_{};
  mcu_msgs__srv__Reset_Response tare_res_{};

  static void tareCallback(const void* req, void* res, void* ctx);

  bool data_ready_ = false;
  Threads::Mutex data_mutex_;

 public:
  // Diagnostic counters (for external monitoring / debug)
  volatile uint32_t diag_update_count_ = 0;
  volatile uint32_t diag_pub_count_ = 0;
  volatile uint32_t diag_last_update_ms_ = 0;
};

}  // namespace Subsystem

#endif
