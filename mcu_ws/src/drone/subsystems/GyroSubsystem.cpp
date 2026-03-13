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

  // BNO085 can spontaneously reset — re-enable reports after boot delay.
  // Use non-blocking wait to avoid starving flight control on same core.
  if (bno_.wasReset()) {
    ++reset_count_;
    DRONE_PRINTF("[Gyro] Reset detected (#%lu)\n", reset_count_);
    reset_wait_until_ = millis() + 300;
  }
  if (reset_wait_until_ > 0) {
    if (millis() < reset_wait_until_) {
      // Still waiting for BNO085 boot — release I2C and yield
      if (cfg_.i2c_mutex) xSemaphoreGive(*cfg_.i2c_mutex);
      vTaskDelay(pdMS_TO_TICKS(10));
      return;
    }
    // Boot delay elapsed, re-enable reports
    reset_wait_until_ = 0;
    if (!enableReports()) {
      DRONE_PRINTLN("[Gyro] WARNING: enableReports failed after reset");
    } else {
      DRONE_PRINTLN("[Gyro] Re-enabled reports after reset");
    }
  }

  // Drain sensor events from the SHTP queue, capped to prevent unbounded
  // I2C reads.  Each getSensorEvent() does a full I2C read (~1-2ms at 400kHz).
  // At 800 events/sec and 250Hz task rate, expect ~3-4 events per tick.
  // Cap at 10 to handle brief backlogs without starving other tasks.
  static constexpr int MAX_EVENTS_PER_UPDATE = 10;
  sh2_SensorValue_t val;
  int events_read = 0;
  while (events_read < MAX_EVENTS_PER_UPDATE && bno_.getSensorEvent(&val)) {
    ++events_read;
    xSemaphoreTake(data_mutex_, portMAX_DELAY);
    switch (val.sensorId) {
      case SH2_ARVR_STABILIZED_RV: {
        // BNO085 mounted vertically, dot forward.
        // Empirical sign mapping for this mounting:
        //   roll:  negate (right-wing-down = positive)
        //   pitch: keep   (nose-up = positive)
        //   yaw:   negate (CW from above = positive)
        float qr = val.un.arvrStabilizedRV.real;
        float qi = val.un.arvrStabilizedRV.i;
        float qj = val.un.arvrStabilizedRV.j;
        float qk = val.un.arvrStabilizedRV.k;

        float bno_roll = atan2f(2.0f * (qr * qi + qj * qk),
                                1.0f - 2.0f * (qi * qi + qj * qj)) *
                         57.29577951f;
        float bno_pitch = asinf(fmaxf(-1.0f, fminf(1.0f,
                          2.0f * (qr * qj - qk * qi)))) *
                          57.29577951f;
        float bno_yaw = atan2f(2.0f * (qr * qk + qi * qj),
                               1.0f - 2.0f * (qj * qj + qk * qk)) *
                        57.29577951f;

        data_.roll  = -bno_roll  - roll_offset_;
        data_.pitch =  bno_pitch - pitch_offset_;
        data_.yaw   = -bno_yaw  - yaw_offset_;

        // Auto-tare on first valid quaternion reading
        if (auto_tare_pending_) {
          auto_tare_pending_ = false;
          roll_offset_ = data_.roll;
          pitch_offset_ = data_.pitch;
          yaw_offset_ = data_.yaw;
          data_.roll = 0.0f;
          data_.pitch = 0.0f;
          data_.yaw = 0.0f;
          DRONE_PRINTF("[Gyro] Auto-tare: roll=%.1f pitch=%.1f yaw=%.1f deg\n",
                       roll_offset_, pitch_offset_, yaw_offset_);
        }
        break;
      }
      case SH2_GYROSCOPE_CALIBRATED: {
        // Gyro signs must match Euler angle signs for PID consistency.
        // roll negated, pitch kept, yaw negated.
        data_.gyro_x = -val.un.gyroscope.x * 57.29577951f;
        data_.gyro_y =  val.un.gyroscope.y * 57.29577951f;
        data_.gyro_z = -val.un.gyroscope.z * 57.29577951f;
        break;
      }
      case SH2_LINEAR_ACCELERATION: {
        // Remap BNO085 accel to drone body frame.
        // Axis mapping (180° about Y): negate X and Z, keep Y.
        data_.accel_x = -val.un.linearAcceleration.x;
        data_.accel_y =  val.un.linearAcceleration.y;
        data_.accel_z = -val.un.linearAcceleration.z;
        break;
      }
    }
    xSemaphoreGive(data_mutex_);
  }

  if (cfg_.i2c_mutex) xSemaphoreGive(*cfg_.i2c_mutex);

  if (events_read >= MAX_EVENTS_PER_UPDATE) {
    DRONE_PRINTF("[Gyro] WARN: hit event cap (%d)\n", events_read);
  }

  // Rate measurement — count only ticks with new sensor data
  if (events_read > 0) rate_count_++;
  uint32_t now_ms = millis();
  if (now_ms - rate_start_ms_ >= 1000) {
    measured_hz_ = rate_count_ * 1000.0f / (now_ms - rate_start_ms_);
    rate_count_ = 0;
    rate_start_ms_ = now_ms;
  }
}

}  // namespace Drone
