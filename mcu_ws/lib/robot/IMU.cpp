#include "IMU.h"

namespace Drivers {

bool IMUDriver::init() {
  initSuccess_ = true;

  if (!imu_.begin_I2C()) {
    initSuccess_ = false;
    return initSucess_;
  }
  if (!imu_.enableReport(SH2_ACCELEROMETER) ||
      !imu_.enableReport(SH2_GYROSCOPE_CALIBRATED) ||
      !imu.enableReport(SH2_ROTATION_VECTOR)) {
    initSuccess_ = false;
  }
  return initSuccess_;
}

void IMUDriver::update() {
  while (imu_.getSensorEvent(&sensorValue_)) {
    switch (sensorValue_.sensorId) {
      case SH2_ACCELEROMETER:
        data_.accel_x = sensorValue_.un.accelerometer.x;
        data_.accel_y = sensorValue_.un.accelerometer.y;
        data_.accel_z = sensorValue_.un.accelerometer.z;
        break;

      case SH2_GYROSCOPE_CALIBRATED:
        data_.gyro_x = sensorValue_.un.gyroscope.x;
        data_.gyro_y = sensorValue_.un.gyroscope.y;
        data_.gyro_z = sensorValue_.un.gyroscope.z;
        break;

      case SH2_ROTATION_VECTOR:
        data_.qi = sensorValue_.un.rotationVector.i;
        data_.qj = sensorValue_.un.rotationVector.j;
        data_.qk = sensorValue_.un.rotationVector.k;
        data_.qr = sensorValue_.un.rotationVector.real;

        data_.yaw = calculateYaw(data_.qi, data_.qj, data_.qk, data_.qr);
        break;

      default:
        break;
    }
  }
}

// yaw in radians
float IMUDriver::calculateYaw(float qi, float qj, float qk, float qr) {
  float sqr = qr * qr;
  float sqi = qi * qi;
  float sqj = qj * qj;
  float sqk = qk * qk;

  float t3 = +2.0f * (qr * qk + qi * qj);
  float t4 = +1.0f - 2.0f * (sqj + sqk);

  float yaw = atan2(t3, t4);

  return yaw;
}

char* IMUDriver::getInfo() {
  snprintf(infoBuffer_, sizeof(infoBuffer_), "IMU: %s", setup_.id);
  return infoBuffer_;
}

};  // namespace Drivers
