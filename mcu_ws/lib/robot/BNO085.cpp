#include "BNO085.h"

namespace Drivers {

bool BNO085Driver::init() {
  I2CBus::Lock lock(setup_.wire_);
  setup_.wire_.begin();

  initSuccess_ = true;

  if (!imu_.begin_I2C(BNO08x_I2CADDR_DEFAULT, &setup_.wire_)) {
    initSuccess_ = false;
    return initSuccess_;
  }

  if (!imu_.enableReport(SH2_ACCELEROMETER) ||
      !imu_.enableReport(SH2_GYROSCOPE_CALIBRATED) ||
      !imu_.enableReport(SH2_ROTATION_VECTOR)) {
    initSuccess_ = false;
  }

  return initSuccess_;
}

void BNO085Driver::update() {
  I2CBus::Lock lock(setup_.wire_);

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
        data_.qx = sensorValue_.un.rotationVector.i;
        data_.qy = sensorValue_.un.rotationVector.j;
        data_.qz = sensorValue_.un.rotationVector.k;
        data_.qw = sensorValue_.un.rotationVector.real;

        data_.yaw = calculateYaw(data_.qx, data_.qy, data_.qz, data_.qw);
        break;

      default:
        break;
    }
  }
}

// yaw in radians
float BNO085Driver::calculateYaw(float qx, float qy, float qz, float qw) {
  float t3 = 2.0f * (qw * qz + qx * qy);
  float t4 = 1.0f - 2.0f * (qy * qy + qz * qz);
  return atan2(t3, t4);
}

const char* BNO085Driver::getInfo() {
  snprintf(infoBuffer_, sizeof(infoBuffer_), "BNO085: %s", setup_.getId());
  return infoBuffer_;
}

}  // namespace Drivers
