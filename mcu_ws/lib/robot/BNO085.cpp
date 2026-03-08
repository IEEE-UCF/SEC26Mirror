#include "BNO085.h"

#include "DebugLog.h"


namespace Drivers {

bool BNO085Driver::init() {
#if defined(USE_FREERTOS)
  I2CBus::Lock lock(setup_.wire_);
#endif

#if defined(__IMXRT1062__)  // Teensy 4.x — I2C bus recovery for Wire1
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
#endif

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

  // Configure INT pin (active LOW when data ready)
  if (setup_.int_pin_ >= 0) {
    pinMode(setup_.int_pin_, INPUT_PULLUP);
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
  if (!imu_.enableReport(SH2_GAME_ROTATION_VECTOR, setup_.rotation_report_us_)) {
    return false;
  }
  if (setup_.enable_gyro_) {
    if (!imu_.enableReport(SH2_GYROSCOPE_CALIBRATED, setup_.gyro_report_us_)) {
      return false;
    }
  }
  return true;
}

void BNO085Driver::update() {
  if (!initSuccess_) return;
#if defined(USE_FREERTOS)
  I2CBus::Lock lock(setup_.wire_);
#endif

  bool wasRst = imu_.wasReset();

  if (wasRst) {
    ++resetCount_;
    DEBUG_PRINTF("[BNO085] Reset detected (#%lu) — waiting 300ms\n", resetCount_);
#if defined(USE_FREERTOS)
    lock.unlock();
#endif
    delay(300);  // BNO085 needs ~300ms after reset before accepting commands
#if defined(USE_FREERTOS)
    lock.relock();
#endif
    if (!enableReports()) {
      DEBUG_PRINTLN("[BNO085] WARNING: enableReports failed after reset");
      return;
    }
  }

  // INT pin is active LOW when data is ready.  Skip the I2C read if no
  // data is queued — keeps poll cycles cheap (single GPIO read).
  if (setup_.int_pin_ >= 0 && digitalReadFast(setup_.int_pin_) == HIGH) {
    return;
  }

  // Drain all available sensor events from the SHTP queue.
  while (imu_.getSensorEvent(&sensorValue_)) {
    switch (sensorValue_.sensorId) {
      case SH2_GAME_ROTATION_VECTOR: {
        data_.qx = sensorValue_.un.gameRotationVector.i;
        data_.qy = sensorValue_.un.gameRotationVector.j;
        data_.qz = sensorValue_.un.gameRotationVector.k;
        data_.qw = sensorValue_.un.gameRotationVector.real;

        // Euler angles from quaternion
        float qi = data_.qx, qj = data_.qy, qk = data_.qz, qr = data_.qw;
        data_.roll = atan2f(2.0f * (qr * qi + qj * qk),
                            1.0f - 2.0f * (qi * qi + qj * qj));
        data_.pitch = asinf(fmaxf(-1.0f, fminf(1.0f, 2.0f * (qr * qj - qk * qi))));
        data_.yaw = calculateYaw(qi, qj, qk, qr);
        break;
      }
      case SH2_GYROSCOPE_CALIBRATED: {
        data_.gyro_x = sensorValue_.un.gyroscope.x;  // rad/s
        data_.gyro_y = sensorValue_.un.gyroscope.y;
        data_.gyro_z = sensorValue_.un.gyroscope.z;
        break;
      }
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
