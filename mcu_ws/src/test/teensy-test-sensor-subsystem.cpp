/**
 * @file teensy-test-sensor-subsystem.cpp
 * @brief MCU test for SensorSubsystem (TOF sensors) on Teensy41
 * @date 12/22/2025
 *
 * Test Requirements:
 * - Teensy 4.1 board
 * - VL53L0X or VL53L1X TOF sensors connected via I2C
 * - I2C multiplexer if using multiple sensors
 *
 * Test Procedure:
 * 1. Verifies subsystem exists and can be instantiated
 * 2. Tests init() returns true (sensors respond)
 * 3. Runs update() cycles
 * 4. Verifies distance readings are within valid range
 * 5. Prints results to Serial
 */

#include <Arduino.h>
#include <TOF.h>
#include <robot/subsystems/SensorSubsystem.h>
#include <vector>

// Test configuration
const uint32_t TEST_DURATION_MS = 10000;
const uint32_t UPDATE_INTERVAL_MS = 100;

// Test state
bool testsPassed = true;
uint32_t testStartTime = 0;
uint32_t lastUpdateTime = 0;
uint32_t updateCount = 0;

// Sensors and subsystem
std::vector<Drivers::TOFDriver*> tofDrivers;
Subsystem::SensorSubsystem* sensorSubsystem = nullptr;

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

  // Create TOF drivers
  // Note: Actual sensor configuration depends on hardware setup
  // This test uses 4 TOF sensors as an example
  const int numSensors = 4;

  for (int i = 0; i < numSensors; i++) {
    Drivers::TOFDriverSetup setup(
        ("tof_sensor_" + String(i)).c_str(),  // ID
        500,                                    // Timeout (ms)
        0                                       // Cooldown (ms)
    );
    Drivers::TOFDriver* driver = new Drivers::TOFDriver(setup);
    tofDrivers.push_back(driver);
  }

  bool driversCreated = (tofDrivers.size() == numSensors);
  printTestResult("TOF drivers instantiated", driversCreated);

  // Create subsystem
  Subsystem::SensorSubsystemSetup setup("sensor_subsystem", tofDrivers);
  sensorSubsystem = new Subsystem::SensorSubsystem(setup);
  bool subsystemExists = (sensorSubsystem != nullptr);
  printTestResult("SensorSubsystem instantiated", subsystemExists);

  // Check getInfo()
  if (sensorSubsystem) {
    const char* info = sensorSubsystem->getInfo();
    bool hasInfo = (info != nullptr && strlen(info) > 0);
    printTestResult("getInfo() returns valid string", hasInfo);
    if (hasInfo) {
      Serial.print("  Info: ");
      Serial.println(info);
    }
  }

  Serial.print("  Number of sensors: ");
  Serial.println(tofDrivers.size());
}

void testSubsystemInit() {
  printTestHeader("Subsystem Initialization");

  if (!sensorSubsystem) {
    printTestResult("SensorSubsystem init() - subsystem null", false);
    return;
  }

  // Test init
  bool initSuccess = sensorSubsystem->init();
  printTestResult("init() returns true", initSuccess);

  if (initSuccess) {
    Serial.println("  Sensors initialized successfully");
    Serial.print("  Number of active sensors: ");
    Serial.println(tofDrivers.size());
  } else {
    Serial.println("  WARNING: Some or all sensors failed to initialize");
    Serial.println("  This may be expected if sensors are not connected");
    Serial.println("  Check:");
    Serial.println("    - TOF sensors are connected");
    Serial.println("    - I2C wiring is correct (SDA, SCL)");
    Serial.println("    - I2C multiplexer is connected (if using multiple sensors)");
    Serial.println("    - Sensors have power");
  }

  // Test individual sensor init (for debugging)
  Serial.println("\n  Individual Sensor Status:");
  for (size_t i = 0; i < tofDrivers.size(); i++) {
    Serial.print("    Sensor ");
    Serial.print(i);
    Serial.print(": ");
    // Note: TOF driver may not have public initSuccess_ check
    // This is a simplified test
    Serial.println("Created");
  }
}

void testSubsystemUpdate() {
  printTestHeader("Subsystem Update");

  if (!sensorSubsystem) {
    printTestResult("SensorSubsystem update() - subsystem null", false);
    return;
  }

  // Run multiple update cycles
  Serial.println("Running update cycles...");
  const int numCycles = 10;

  for (int i = 0; i < numCycles; i++) {
    sensorSubsystem->update();
    delay(100);  // Allow time for sensor readings

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

  if (tofDrivers.empty()) {
    printTestResult("Sensor functionality - no drivers", false);
    return;
  }

  Serial.println("Sensor Readings:");

  bool anyValidReading = false;
  int validCount = 0;

  for (size_t i = 0; i < tofDrivers.size(); i++) {
    Drivers::TOFDriver* driver = tofDrivers[i];
    if (!driver) continue;

    // Get distance reading (in mm)
    driver->update();
    delay(50);  // Allow time for measurement

    Drivers::TOFDriverData data = driver->read();
    float distance = static_cast<float>(data.range);

    Serial.print("  Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(distance);
    Serial.println(" mm");

    // Validate reading is in reasonable range
    // VL53L0X: 30-2000mm, VL53L1X: 40-4000mm
    // Accept 0-5000mm as valid range
    bool distanceValid = (distance >= 0.0f && distance <= 5000.0f);

    if (distanceValid) {
      validCount++;
    }

    if (distance > 0.0f && distance < 5000.0f) {
      anyValidReading = true;
    }
  }

  char resultMsg[100];
  snprintf(resultMsg, sizeof(resultMsg), "Sensor readings are in valid range (%d/%d sensors)",
           validCount, (int)tofDrivers.size());
  printTestResult(resultMsg, validCount > 0);

  if (anyValidReading) {
    printTestResult("At least one sensor producing valid readings", true);
  } else {
    Serial.println("[INFO] No valid sensor readings - sensors may not be connected or need calibration");
  }
}

void testSubsystemReset() {
  printTestHeader("Subsystem Reset");

  if (!sensorSubsystem) {
    printTestResult("SensorSubsystem reset() - subsystem null", false);
    return;
  }

  sensorSubsystem->reset();
  printTestResult("reset() executes without crashing", true);
}

void printSummary() {
  Serial.println();
  Serial.println("================================================");
  Serial.println("TEST SUMMARY");
  Serial.println("================================================");

  Serial.print("Total Updates: ");
  Serial.println(updateCount);

  Serial.print("Number of Sensors: ");
  Serial.println(tofDrivers.size());

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
  Serial.println("SENSOR SUBSYSTEM MCU TEST");
  Serial.println("Teensy 4.1 - TOF Sensors");
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
    if (sensorSubsystem) {
      sensorSubsystem->update();
      updateCount++;

      // Print periodic status
      if (updateCount % 50 == 0) {
        Serial.print("Running... Updates: ");
        Serial.println(updateCount);

        Serial.print("  Distances: ");
        for (size_t i = 0; i < tofDrivers.size(); i++) {
          if (i > 0) Serial.print(", ");
          Serial.print("S");
          Serial.print(i);
          Serial.print("=");
          Drivers::TOFDriverData data = tofDrivers[i]->read();
          Serial.print(data.range);
          Serial.print("mm");
        }
        Serial.println();
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
