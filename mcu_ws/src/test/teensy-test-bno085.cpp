/**
 * @file teensy-test-bno085.cpp
 * @brief Raw BNO085 IMU test — direct Adafruit API, serial output.
 *
 * Wiring (Teensy 4.1):
 *   Wire1 SDA (pin 17) / SCL (pin 16) -> BNO085
 *   BNO085 I2C address: 0x4B
 *   Reset pin: 40
 */

#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <Wire.h>

static constexpr uint8_t BNO_ADDR = 0x4B;
static constexpr int8_t BNO_RST = 40;

static Adafruit_BNO08x bno(BNO_RST);
static sh2_SensorValue_t sensorValue;

void setup() {
  Serial.begin(921600);
  while (!Serial && millis() < 3000) {}

  if (CrashReport) {
    Serial.print(CrashReport);
    Serial.println();
    Serial.flush();
  }

  Serial.println("\r\nBNO085 raw test (reset pin 40)\r\n");

  Wire1.begin();
  Wire1.setClock(400000);

  if (!bno.begin_I2C(BNO_ADDR, &Wire1)) {
    Serial.println("ERROR: Failed to find BNO08x — check wiring");
    while (1) { delay(1000); }
  }
  Serial.println("BNO08x found!");

  Wire1.setClock(400000);

  // 10000us = 10ms = 100Hz report rate
  if (!bno.enableReport(SH2_GAME_ROTATION_VECTOR, 10000)) {
    Serial.println("ERROR: Could not enable game rotation vector");
  }
  if (!bno.enableReport(SH2_ACCELEROMETER, 10000)) {
    Serial.println("ERROR: Could not enable accelerometer");
  }
  if (!bno.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000)) {
    Serial.println("ERROR: Could not enable gyroscope");
  }

  Serial.println("Reports enabled — streaming data\r\n");
}

void loop() {
  if (bno.wasReset()) {
    Serial.println("BNO08x reset detected — re-enabling reports");
    Wire1.setClock(400000);
    bno.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
    bno.enableReport(SH2_ACCELEROMETER, 10000);
    bno.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
  }

  // Drain all pending events
  while (bno.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
    case SH2_GAME_ROTATION_VECTOR: {
      float qr = sensorValue.un.gameRotationVector.real;
      float qi = sensorValue.un.gameRotationVector.i;
      float qj = sensorValue.un.gameRotationVector.j;
      float qk = sensorValue.un.gameRotationVector.k;

      float sqr = sq(qr);
      float sqi = sq(qi);
      float sqj = sq(qj);
      float sqk = sq(qk);

      float yaw = atan2(2.0f * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
      float pitch = asin(-2.0f * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
      float roll = atan2(2.0f * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

      Serial.printf("ROT  yaw: %7.2f  pitch: %7.2f  roll: %7.2f deg\n",
                     yaw * RAD_TO_DEG, pitch * RAD_TO_DEG, roll * RAD_TO_DEG);
      break;
    }
    case SH2_ACCELEROMETER:
      Serial.printf("ACC  x: %7.3f  y: %7.3f  z: %7.3f m/s2\n",
                     sensorValue.un.accelerometer.x,
                     sensorValue.un.accelerometer.y,
                     sensorValue.un.accelerometer.z);
      break;
    case SH2_GYROSCOPE_CALIBRATED:
      Serial.printf("GYR  x: %7.3f  y: %7.3f  z: %7.3f rad/s\n",
                     sensorValue.un.gyroscope.x,
                     sensorValue.un.gyroscope.y,
                     sensorValue.un.gyroscope.z);
      break;
    default:
      break;
    }
  }

  delay(10);
}
