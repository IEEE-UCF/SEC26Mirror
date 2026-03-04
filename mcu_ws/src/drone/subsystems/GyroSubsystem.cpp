#include "GyroSubsystem.h"

namespace Drone {

bool GyroSubsystem::init() {
  mutex_ = xSemaphoreCreateMutex();

  // Hardware reset if pin configured
  if (cfg_.reset_pin >= 0) {
    pinMode(cfg_.reset_pin, OUTPUT);
    digitalWrite(cfg_.reset_pin, LOW);
    delay(10);
    digitalWrite(cfg_.reset_pin, HIGH);
    delay(100);
  }

  if (!bno_.begin_I2C(cfg_.i2c_addr, cfg_.wire)) {
    initialized_ = false;
    return false;
  }

  // Enable stabilized rotation vector for attitude (Euler angles)
  if (!bno_.enableReport(SH2_ARVR_STABILIZED_RV, cfg_.rotation_report_us)) {
    return false;
  }

  // Enable calibrated gyroscope for rate PID inner loop
  if (!bno_.enableReport(SH2_GYROSCOPE_CALIBRATED, cfg_.gyro_report_us)) {
    return false;
  }

  // Enable linear acceleration (gravity-free) for EKF
  if (!bno_.enableReport(SH2_LINEAR_ACCELERATION, cfg_.accel_report_us)) {
    return false;
  }

  initialized_ = true;
  return true;
}

void GyroSubsystem::update() {
  if (!initialized_) return;

  sh2_SensorValue_t val;
  while (bno_.getSensorEvent(&val)) {
    if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);
    switch (val.sensorId) {
      case SH2_ARVR_STABILIZED_RV: {
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
        data_.gyro_x = val.un.gyroscope.x * 57.29577951f;
        data_.gyro_y = val.un.gyroscope.y * 57.29577951f;
        data_.gyro_z = val.un.gyroscope.z * 57.29577951f;
        break;
      }
      case SH2_LINEAR_ACCELERATION: {
        data_.accel_x = val.un.linearAcceleration.x;
        data_.accel_y = val.un.linearAcceleration.y;
        data_.accel_z = val.un.linearAcceleration.z;
        break;
      }
    }
    if (mutex_) xSemaphoreGive(mutex_);
  }
}

}  // namespace Drone
