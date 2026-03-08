#include "GyroSubsystem.h"

namespace Drone {

bool GyroSubsystem::init() {
  // Create driver setup with drone-specific config: gyro enabled, high rates
  driver_setup_ = new Drivers::BNO085DriverSetup(
      "bno085_drone",
      (cfg_.reset_pin != 255) ? (int8_t)cfg_.reset_pin : (int8_t)-1,
      Wire,  // ESP32 uses default Wire bus
      cfg_.i2c_addr, cfg_.int_pin,
      /*enable_gyro=*/true, cfg_.rotation_report_us, cfg_.gyro_report_us);

  driver_ = new Drivers::BNO085Driver(*driver_setup_);

  if (!driver_->init()) {
    initialized_ = false;
    return false;
  }

  initialized_ = true;
  return true;
}

void GyroSubsystem::update() {
  if (!initialized_) return;

  driver_->update();

  // Copy BNO085 driver data into drone IMUData (degrees for the PID controller)
  const auto& d = driver_->getData();
  data_.roll = d.roll * RAD_TO_DEG;
  data_.pitch = d.pitch * RAD_TO_DEG;
  data_.yaw = d.yaw * RAD_TO_DEG;
  data_.gyro_x = d.gyro_x * RAD_TO_DEG;
  data_.gyro_y = d.gyro_y * RAD_TO_DEG;
  data_.gyro_z = d.gyro_z * RAD_TO_DEG;
}

}  // namespace Drone
