#include "BNO085.h"

#include "DebugLog.h"


namespace Drivers {

bool BNO085Driver::init() {
  I2CBus::Lock lock(setup_.wire_);

  // I2C bus recovery: if the MCU reset mid-transaction, the BNO085 may be
  // holding SDA low.  Toggle SCL manually to clock out the stuck byte.
  // This must happen BEFORE Wire.begin() claims the pins.
  if (&setup_.wire_ == &Wire1) {
    const uint8_t sda = 17, scl = 16;
    pinMode(sda, INPUT_PULLUP);
    pinMode(scl, OUTPUT);
    for (int i = 0; i < 16; ++i) {
      digitalWriteFast(scl, LOW);
      delayMicroseconds(5);
      digitalWriteFast(scl, HIGH);
      delayMicroseconds(5);
    }
    // Generate STOP condition: SDA low→high while SCL high
    pinMode(sda, OUTPUT);
    digitalWriteFast(sda, LOW);
    delayMicroseconds(5);
    digitalWriteFast(sda, HIGH);
    delayMicroseconds(5);
    // Release pins for Wire peripheral
    pinMode(sda, INPUT);
    pinMode(scl, INPUT);
  }

  // Hardware reset the BNO085 BEFORE Wire.begin() so the chip is fully
  // booted when I2C starts.  We do this ourselves (instead of relying on
  // the Adafruit library's hal_hardwareReset which only waits 10ms) because
  // the BNO085 needs ~300ms after reset to bring up its SHTP interface.
  if (setup_.reset_pin >= 0) {
    pinMode(setup_.reset_pin, OUTPUT);
    digitalWriteFast(setup_.reset_pin, HIGH);
    delay(10);
    digitalWriteFast(setup_.reset_pin, LOW);
    delay(10);
    digitalWriteFast(setup_.reset_pin, HIGH);
    delay(300);  // BNO085 needs ~300ms to boot after reset
  }

  setup_.wire_.begin();
  setup_.wire_.setClock(400000);

  initSuccess_ = true;

  // Pass -1 as reset pin to skip the library's internal hardware reset
  // (which only waits 10ms — insufficient for the BNO085 bootloader).
  // We already did a proper reset above.
  if (!imu_.begin_I2C(setup_.addr_, &setup_.wire_)) {
    initSuccess_ = false;
    return initSuccess_;
  }

  // Consume the initial wasReset flag from the boot sequence
  (void)imu_.wasReset();
  delay(100);

  if (!enableReports()) {
    initSuccess_ = false;
  }

  // Wait for the first sensor events to become available
  delay(100);

  return initSuccess_;
}

bool BNO085Driver::enableReports() {
  // 50000us = 50ms = 20Hz per report — matches the IMU subsystem publish rate.
  // Higher rates (100Hz) cause event backlog in the getSensorEvent() loop.
  return imu_.enableReport(SH2_ACCELEROMETER, 50000) &&
         imu_.enableReport(SH2_GYROSCOPE_CALIBRATED, 50000) &&
         imu_.enableReport(SH2_GAME_ROTATION_VECTOR, 50000);
}

void BNO085Driver::update() {
  if (!initSuccess_) return;

  I2CBus::Lock lock(setup_.wire_);

  bool wasRst = imu_.wasReset();

  if (wasRst) {
    ++resetCount_;
    DEBUG_PRINTF("[BNO085] Reset detected (#%lu) — waiting 300ms\n", resetCount_);
    lock.unlock();
    delay(300);
    lock.relock();
    if (!enableReports()) {
      DEBUG_PRINTLN("[BNO085] WARNING: enableReports failed after reset");
      return;
    }
  }

  int eventsRead = 0;
  while (eventsRead < kMaxEventsPerUpdate && imu_.getSensorEvent(&sensorValue_)) {
    ++eventsRead;
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

      case SH2_GAME_ROTATION_VECTOR:
        data_.qx = sensorValue_.un.gameRotationVector.i;
        data_.qy = sensorValue_.un.gameRotationVector.j;
        data_.qz = sensorValue_.un.gameRotationVector.k;
        data_.qw = sensorValue_.un.gameRotationVector.real;

        data_.yaw = calculateYaw(data_.qx, data_.qy, data_.qz, data_.qw);
        break;

      default:
        break;
    }
  }

  if (eventsRead >= kMaxEventsPerUpdate) {
    DEBUG_PRINTLN("[BNO085] WARNING: hit max events per update — possible I2C issue");
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
