#include "IMU.h"

namespace Drivers {

bool IMUDriver::init() {
  if (!imu_.begin_I2C()) {
    initSuccess_ = false;
  }
  if (!imu_.enableReport(SH2_ACCELEROMETER) &&
      !imu_.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    initSuccess_ = false;
  }
}

void IMUDriver::update() {
  if (!imu_.getSensorEvent(&sensorValue_)) {
    return;
  }

  data_.accel_x = sensorValue_.un.accelerometer.x;
  data_.accel_y = sensorValue_.un.accelerometer.y;
  data_.accel_z = sensorValue_.un.accelerometer.z;

  data_.gyro_x = sensorValue_.un.gyroscope.x;
  data_.gyro_y = sensorValue_.un.gyroscope.y;
  data_.gyro_z = sensorValue_.un.gyroscope.z;
}

char* IMUDriver::getInfo() {}

};  // namespace Drivers
