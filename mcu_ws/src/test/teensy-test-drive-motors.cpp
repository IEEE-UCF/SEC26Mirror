/**
 * @file teensy-test-drive-motors.cpp
 * @brief Bare-metal drive motor test — no micro-ROS, no threads.
 *
 * Cycles all 8 motors through a speed pattern:
 *   1. Forward  100%  (1 s)
 *   2. Forward   50%  (1 s)
 *   3. Stop            (1 s)
 *   4. Reverse  50%   (1 s)
 *   5. Reverse 100%   (1 s)
 *   6. Stop            (1 s)
 *   (repeat)
 *
 * Motor control (NFPShop brushless on PCA9685):
 *   Even channel = PWM  (inverted: 4095 = stopped, 0 = full speed)
 *   Odd  channel = DIR  (HIGH = forward, LOW = reverse)
 *
 * Hardware:
 *   Wire2 (SDA=24, SCL=25) — PCA9685 #1 at 0x41, OE = pin 29
 *
 * Build & flash:
 *   pio run -e teensy-test-drive-motors --target upload
 *   pio device monitor -e teensy-test-drive-motors
 */

#include <Arduino.h>

#include "I2CBusLock.h"
#include "PCA9685Driver.h"
#include "robot/RobotConstants.h"
#include "robot/RobotPins.h"

// ═══════════════════════════════════════════════════════════════════════════
//  Constants
// ═══════════════════════════════════════════════════════════════════════════

static constexpr uint16_t MAX_PWM = 4095;
static constexpr uint32_t STEP_DURATION_MS = 1000;

// Speed pattern: {speed_fraction, label}
// Positive = forward, negative = reverse
struct SpeedStep {
  float speed;
  const char* label;
};

static const SpeedStep pattern[] = {
    {+1.00f, "FWD 100%"}, {+0.50f, "FWD  50%"}, {0.00f, "STOP    "},
    {-0.50f, "REV  50%"}, {-1.00f, "REV 100%"}, {0.00f, "STOP    "},
};
static constexpr int PATTERN_LEN = sizeof(pattern) / sizeof(pattern[0]);

// ═══════════════════════════════════════════════════════════════════════════
//  PCA9685 motor driver (direct, no subsystem)
// ═══════════════════════════════════════════════════════════════════════════

static Robot::PCA9685DriverSetup g_pca_setup("pca_motor", I2C_ADDR_MOTOR,
                                             MOTOR_PCA9685_FREQ, Wire2);
static Robot::PCA9685Driver g_pca(g_pca_setup);

static void setMotor(uint8_t motor, float speed) {
  speed = constrain(speed, -1.0f, 1.0f);
  bool dir = (speed >= 0.0f);
  uint16_t duty = MAX_PWM - (uint16_t)(fabsf(speed) * MAX_PWM);

  uint8_t pwmCh = motor * 2;
  uint8_t dirCh = motor * 2 + 1;

  g_pca.bufferDigital(dirCh, dir);
  g_pca.bufferPWM(pwmCh, duty);
}

static void stopAll() {
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    setMotor(i, 0.0f);
  }
  g_pca.applyBuffered();
}

// ═══════════════════════════════════════════════════════════════════════════
//  Arduino entry points
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(921600);
  delay(500);
  if (CrashReport) {
    Serial.print(CrashReport);
    Serial.println();
    Serial.flush();
  }

  Serial.println("\r\n═══════════════════════════════════════════");
  Serial.println("  SEC26 Drive Motor Test (bare-metal)");
  Serial.println("  No micro-ROS, no threads");
  Serial.println("═══════════════════════════════════════════\n");

  pinMode(LED_BUILTIN, OUTPUT);

  // Init I2C bus locks and motor PCA9685
  I2CBus::initLocks();

  Serial.print("PCA9685 init (0x41, Wire2)... ");
  if (!g_pca.init()) {
    Serial.println("FAILED — check I2C wiring. Halting.");
    while (true) {
      delay(100);
    }
  }
  Serial.println("OK");

  // Enable motor output (OE active LOW)
  pinMode(PIN_MOTOR_OE, OUTPUT);
  digitalWrite(PIN_MOTOR_OE, LOW);

  // Ensure all motors start stopped
  stopAll();

  Serial.printf("Motors: %d  |  OE pin: %d  |  Step: %lu ms\n", NUM_MOTORS,
                PIN_MOTOR_OE, STEP_DURATION_MS);
  Serial.println(
      "Pattern: FWD 100% → FWD 50% → STOP → REV 50% → REV 100% → STOP");
  Serial.println("\nStarting in 2 seconds...\n");
  delay(2000);
}

static int step_idx = 0;
static uint32_t step_start = 0;
static bool first = true;

void loop() {
  uint32_t now = millis();

  if (first || (now - step_start >= STEP_DURATION_MS)) {
    first = false;
    step_start = now;

    const SpeedStep& s = pattern[step_idx];

    // Apply speed to all motors
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      setMotor(i, s.speed);
    }
    g_pca.applyBuffered();

    // Print status
    uint16_t duty = MAX_PWM - (uint16_t)(fabsf(s.speed) * MAX_PWM);
    Serial.printf("[%6lu ms]  %s  (speed=%+.2f  duty=%4d  dir=%s)\n", now,
                  s.label, s.speed, duty, s.speed >= 0 ? "FWD" : "REV");

    digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));

    step_idx = (step_idx + 1) % PATTERN_LEN;
  }
}
