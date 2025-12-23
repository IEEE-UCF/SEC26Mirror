/**
 * @file teensy-test-battery-subsystem.cpp
 * @brief MCU test for BatterySubsystem on Teensy41
 * @date 12/22/2025
 *
 * Test Requirements:
 * - Teensy 4.1 board
 * - INA228 power sensor connected via I2C
 * - Battery connected to sensor
 *
 * Test Procedure:
 * 1. Verifies subsystem exists and can be instantiated
 * 2. Tests init() returns true (sensor responds)
 * 3. Runs update() cycles
 * 4. Verifies voltage, current, power readings are sensible
 * 5. Prints results to Serial
 */

#include <Arduino.h>
#include <I2CPowerDriver.h>
#include <robot/subsystems/BatterySubsystem.h>

// Test configuration
const uint32_t TEST_DURATION_MS = 10000;  // Run test for 10 seconds
const uint32_t UPDATE_INTERVAL_MS = 100;  // Update every 100ms

// Test state
bool testsPassed = true;
uint32_t testStartTime = 0;
uint32_t lastUpdateTime = 0;
uint32_t updateCount = 0;

// Driver and subsystem
Drivers::I2CPowerDriverSetup driverSetup("battery_power_sensor", 0x40, 10.0f, 0.015f);
Drivers::I2CPowerDriver* powerDriver = nullptr;
Subsystem::BatterySubsystem* batterySubsystem = nullptr;

void printTestHeader(const char* testName) {
  Serial.println();
  Serial.println("================================================");
  Serial.print("TEST: ");
  Serial.println(testName);
  Serial.println("================================================");
}

void printTestResult(const char* testName, bool passed) {
  Serial.print("[");
  Serial.print(passed ? "PASS" : "FAIL");
  Serial.print("] ");
  Serial.println(testName);

  if (!passed) {
    testsPassed = false;
  }
}

void testSubsystemExists() {
  printTestHeader("Subsystem Existence");

  // Create driver
  powerDriver = new Drivers::I2CPowerDriver(driverSetup);
  bool driverExists = (powerDriver != nullptr);
  printTestResult("I2CPowerDriver instantiated", driverExists);

  // Create subsystem
  Subsystem::BatterySubsystemSetup setup("battery_subsystem", powerDriver);
  batterySubsystem = new Subsystem::BatterySubsystem(setup);
  bool subsystemExists = (batterySubsystem != nullptr);
  printTestResult("BatterySubsystem instantiated", subsystemExists);

  // Check getInfo()
  if (batterySubsystem) {
    const char* info = batterySubsystem->getInfo();
    bool hasInfo = (info != nullptr && strlen(info) > 0);
    printTestResult("getInfo() returns valid string", hasInfo);
    if (hasInfo) {
      Serial.print("  Info: ");
      Serial.println(info);
    }
  }
}

void testSubsystemInit() {
  printTestHeader("Subsystem Initialization");

  if (!batterySubsystem) {
    printTestResult("BatterySubsystem init() - subsystem null", false);
    return;
  }

  // Test init
  bool initSuccess = batterySubsystem->init();
  printTestResult("init() returns true", initSuccess);

  if (initSuccess) {
    Serial.println("  Sensor initialized successfully");
    Serial.print("  I2C Address: 0x");
    Serial.println(driverSetup._address, HEX);
  } else {
    Serial.println("  ERROR: Sensor failed to initialize");
    Serial.println("  Check:");
    Serial.println("    - INA228 sensor is connected");
    Serial.println("    - I2C wiring is correct (SDA, SCL)");
    Serial.println("    - I2C address matches (default 0x40)");
    Serial.println("    - Sensor has power");
  }
}

void testSubsystemUpdate() {
  printTestHeader("Subsystem Update");

  if (!batterySubsystem) {
    printTestResult("BatterySubsystem update() - subsystem null", false);
    return;
  }

  // Run multiple update cycles
  Serial.println("Running update cycles...");
  const int numCycles = 10;

  for (int i = 0; i < numCycles; i++) {
    batterySubsystem->update();
    delay(150);  // Allow time for readings

    // Print progress
    if (i % 2 == 0) {
      Serial.print(".");
    }
  }
  Serial.println();

  printTestResult("update() executes without crashing", true);
  Serial.print("  Completed ");
  Serial.print(numCycles);
  Serial.println(" update cycles");
}

void testSubsystemFunctionality() {
  printTestHeader("Subsystem Functionality");

  if (!powerDriver) {
    printTestResult("Power driver functionality - driver null", false);
    return;
  }

  // Update driver to get latest readings
  powerDriver->update();
  delay(100);

  // Get readings
  float voltage = powerDriver->getVoltage();
  float current = powerDriver->getCurrentmA();
  float power = powerDriver->getPowermW();
  float temp = powerDriver->getTemp();

  // Print readings
  Serial.println("Sensor Readings:");
  Serial.print("  Voltage: ");
  Serial.print(voltage);
  Serial.println(" V");

  Serial.print("  Current: ");
  Serial.print(current);
  Serial.println(" mA");

  Serial.print("  Power: ");
  Serial.print(power);
  Serial.println(" mW");

  Serial.print("  Temperature: ");
  Serial.print(temp);
  Serial.println(" C");

  // Validate readings are in reasonable ranges
  bool voltageValid = (voltage >= 0.0f && voltage < 30.0f);  // Typical battery range
  printTestResult("Voltage reading is reasonable (0-30V)", voltageValid);

  bool currentValid = (current >= -20000.0f && current < 20000.0f);  // ±20A
  printTestResult("Current reading is reasonable (±20A)", currentValid);

  bool powerValid = (power >= -600000.0f && power < 600000.0f);  // ±600W
  printTestResult("Power reading is reasonable (±600W)", powerValid);

  bool tempValid = (temp >= -40.0f && temp < 125.0f);  // Sensor range
  printTestResult("Temperature reading is reasonable (-40-125C)", tempValid);

  // Check if readings are not stuck at zero (indicates sensor is working)
  bool notAllZero = (voltage != 0.0f || current != 0.0f || power != 0.0f);
  if (voltage > 1.0f) {  // If battery connected, voltage should be > 1V
    printTestResult("Sensor producing non-zero readings", notAllZero);
  } else {
    Serial.println("[INFO] Voltage < 1V - battery may not be connected");
  }
}

void testSubsystemReset() {
  printTestHeader("Subsystem Reset");

  if (!batterySubsystem) {
    printTestResult("BatterySubsystem reset() - subsystem null", false);
    return;
  }

  batterySubsystem->reset();
  printTestResult("reset() executes without crashing", true);
}

void printSummary() {
  Serial.println();
  Serial.println("================================================");
  Serial.println("TEST SUMMARY");
  Serial.println("================================================");

  Serial.print("Total Updates: ");
  Serial.println(updateCount);

  Serial.print("Test Duration: ");
  Serial.print((millis() - testStartTime) / 1000.0f);
  Serial.println(" seconds");

  Serial.println();
  if (testsPassed) {
    Serial.println("✓ ALL TESTS PASSED");
  } else {
    Serial.println("✗ SOME TESTS FAILED");
  }
  Serial.println("================================================");
}

void setup() {
  // Initialize Serial
  Serial.begin(115200);
  while (!Serial && millis() < 3000);  // Wait up to 3s for serial

  delay(1000);

  Serial.println();
  Serial.println("================================================");
  Serial.println("BATTERY SUBSYSTEM MCU TEST");
  Serial.println("Teensy 4.1");
  Serial.println("================================================");
  Serial.println();

  testStartTime = millis();

  // Run tests sequentially
  testSubsystemExists();
  delay(500);

  testSubsystemInit();
  delay(500);

  testSubsystemUpdate();
  delay(500);

  testSubsystemFunctionality();
  delay(500);

  testSubsystemReset();
  delay(500);

  printSummary();

  lastUpdateTime = millis();
}

void loop() {
  uint32_t now = millis();

  // Continue running updates to demonstrate continuous operation
  if (now - lastUpdateTime >= UPDATE_INTERVAL_MS) {
    if (batterySubsystem) {
      batterySubsystem->update();
      updateCount++;

      // Print periodic status
      if (updateCount % 50 == 0) {
        Serial.print("Running... Updates: ");
        Serial.println(updateCount);

        if (powerDriver) {
          Serial.print("  V=");
          Serial.print(powerDriver->getVoltage());
          Serial.print("V, I=");
          Serial.print(powerDriver->getCurrentmA());
          Serial.print("mA, P=");
          Serial.print(powerDriver->getPowermW());
          Serial.println("mW");
        }
      }
    }

    lastUpdateTime = now;
  }

  // Blink LED to show test is running
  static uint32_t lastBlink = 0;
  if (now - lastBlink >= 500) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    lastBlink = now;
  }
}
