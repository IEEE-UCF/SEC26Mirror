#include "BNO085.h"

#include "DebugLog.h"

namespace Drivers {

bool BNO085Driver::init() {
  I2CBus::Lock lock(setup_.wire_);
  setup_.wire_.begin();
  setup_.wire_.setClock(400000);

  initSuccess_ = true;

  if (!imu_.begin_I2C(setup_.addr_, &setup_.wire_)) {
    initSuccess_ = false;
    return initSuccess_;
  }

  if (!enableReports()) {
    initSuccess_ = false;
  }

  return initSuccess_;
}

bool BNO085Driver::enableReports() {
  // 10000us = 10ms = 100Hz per report
  return imu_.enableReport(SH2_ACCELEROMETER, 10000) &&
         imu_.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000) &&
         imu_.enableReport(SH2_ROTATION_VECTOR, 10000);
}

void BNO085Driver::update() {
  if (!initSuccess_) return;
  I2CBus::Lock lock(setup_.wire_);

  // BNO085 can spontaneously reset — re-enable reports when it does
  if (imu_.wasReset()) {
    delay(10);  // give BNO085 time to be ready after reset
    if (!enableReports()) {
      DEBUG_PRINTLN("[BNO085] WARNING: enableReports failed after reset");
    } else {
      DEBUG_PRINTLN("[BNO085] Re-enabled reports after reset");
    }
  }

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
