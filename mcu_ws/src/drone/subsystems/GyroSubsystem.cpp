#include "GyroSubsystem.h"

namespace Drone {

bool GyroSubsystem::init() {
  if (cfg_.reset_pin != (uint8_t)-1) {
    if (!bno_.begin_I2C(cfg_.i2c_addr)) {
      initialized_ = false;
      return false;
    }
  } else {
    if (!bno_.begin_I2C(cfg_.i2c_addr)) {
      initialized_ = false;
      return false;
    }
  }

  // Enable stabilized rotation vector for attitude (Euler angles)
  if (!bno_.enableReport(SH2_ARVR_STABILIZED_RV, cfg_.rotation_report_us)) {
    return false;
  }

  // Enable calibrated gyroscope for rate PID inner loop
  if (!bno_.enableReport(SH2_GYROSCOPE_CALIBRATED, cfg_.gyro_report_us)) {
    return false;
  }

  initialized_ = true;
  return true;
}

void GyroSubsystem::update() {
  if (!initialized_) return;

  sh2_SensorValue_t val;
  while (bno_.getSensorEvent(&val)) {
    switch (val.sensorId) {
      case SH2_ARVR_STABILIZED_RV: {
        // Quaternion -> Euler angles (degrees)
        float qr = val.un.arvrStabilizedRV.real;
        float qi = val.un.arvrStabilizedRV.i;
        float qj = val.un.arvrStabilizedRV.j;
        float qk = val.un.arvrStabilizedRV.k;

        data_.roll = atan2f(2.0f * (qr * qi + qj * qk),
                            1.0f - 2.0f * (qi * qi + qj * qj)) *
                     57.29577951f;
        data_.pitch = asinf(constrain(2.0f * (qr * qj - qk * qi), -0.999999f,
                                      0.999999f)) *
                      57.29577951f;
        data_.yaw = atan2f(2.0f * (qr * qk + qi * qj),
                           1.0f - 2.0f * (qj * qj + qk * qk)) *
                    57.29577951f;
        break;
      }
      case SH2_GYROSCOPE_CALIBRATED: {
        // Gyro in rad/s -> convert to deg/s
        data_.gyro_x = val.un.gyroscope.x * 57.29577951f;
        data_.gyro_y = val.un.gyroscope.y * 57.29577951f;
        data_.gyro_z = val.un.gyroscope.z * 57.29577951f;
        break;
      }
    }
  }
}

}  // namespace Drone
