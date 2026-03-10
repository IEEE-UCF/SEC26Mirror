#include "GyroSubsystem.h"

namespace Drone {

bool GyroSubsystem::init() {
  data_mutex_ = xSemaphoreCreateMutex();

  // Hardware reset: active-LOW on BNO085 (same sequence as robot BNO085Driver)
  if (cfg_.reset_pin >= 0) {
    pinMode(cfg_.reset_pin, OUTPUT);
    digitalWrite(cfg_.reset_pin, HIGH);
    delay(10);
    digitalWrite(cfg_.reset_pin, LOW);
    delay(10);
    digitalWrite(cfg_.reset_pin, HIGH);
    delay(300);  // BNO085 needs ~300ms to boot after reset
  }

  if (!bno_.begin_I2C(cfg_.i2c_addr, cfg_.wire)) {
    DRONE_PRINTLN("[Gyro] begin_I2C FAIL");
    initialized_ = false;
    return false;
  }

  // Robot-proven approach: 3000ms delay after begin_I2C for reliable init
  delay(3000);

  // Consume the initial wasReset flag from the boot sequence
  (void)bno_.wasReset();
  delay(100);

  if (!enableReports()) {
    DRONE_PRINTLN("[Gyro] enableReports FAIL");
    initialized_ = false;
    return false;
  }

  // Wait for the first sensor events to become available
  delay(100);

  initialized_ = true;
  return true;
}

bool GyroSubsystem::enableReports() {
  if (!bno_.enableReport(SH2_ARVR_STABILIZED_RV, cfg_.rotation_report_us)) {
    return false;
  }
  if (!bno_.enableReport(SH2_GYROSCOPE_CALIBRATED, cfg_.gyro_report_us)) {
    return false;
  }
  if (!bno_.enableReport(SH2_LINEAR_ACCELERATION, cfg_.accel_report_us)) {
    return false;
  }
  return true;
}

void GyroSubsystem::update() {
  if (!initialized_) return;

  // Acquire I2C mutex if available (shared bus with height sensor)
  if (cfg_.i2c_mutex) xSemaphoreTake(*cfg_.i2c_mutex, portMAX_DELAY);

  // BNO085 can spontaneously reset — re-enable reports when it does.
  if (bno_.wasReset()) {
    ++reset_count_;
    DRONE_PRINTF("[Gyro] Reset detected (#%lu) — waiting 300ms\n",
                 reset_count_);
    delay(300);
    if (!enableReports()) {
      DRONE_PRINTLN("[Gyro] WARNING: enableReports failed after reset");
    } else {
      DRONE_PRINTLN("[Gyro] Re-enabled reports after reset");
    }
  }

  // Drain all available sensor events from the SHTP queue.
  sh2_SensorValue_t val;
  while (bno_.getSensorEvent(&val)) {
    xSemaphoreTake(data_mutex_, portMAX_DELAY);
    switch (val.sensorId) {
      case SH2_ARVR_STABILIZED_RV: {
        float qr = val.un.arvrStabilizedRV.real;
        float qi = val.un.arvrStabilizedRV.i;
        float qj = val.un.arvrStabilizedRV.j;
        float qk = val.un.arvrStabilizedRV.k;

        data_.roll = atan2f(2.0f * (qr * qi + qj * qk),
                            1.0f - 2.0f * (qi * qi + qj * qj)) *
                     57.29577951f;
        data_.pitch = asinf(fmaxf(-1.0f, fminf(1.0f,
                     2.0f * (qr * qj - qk * qi)))) *
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
    xSemaphoreGive(data_mutex_);
  }

  if (cfg_.i2c_mutex) xSemaphoreGive(*cfg_.i2c_mutex);
}

}  // namespace Drone
