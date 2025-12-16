#include <Arduino.h>
#include <vector>

#include "robot/machines/RobotManager.h"
#include <BaseSubsystem.h>

using namespace Subsystem;

// A minimal serial-printing subsystem managed by RobotManager
class ExampleSerialSetup : public Classes::BaseSetup {
 public:
  explicit ExampleSerialSetup(const char* id) : Classes::BaseSetup(id) {}
};

class ExampleSerialSubsystem : public Classes::BaseSubsystem {
 public:
  explicit ExampleSerialSubsystem(const ExampleSerialSetup& setup)
      : Classes::BaseSubsystem(setup), setup_(setup) {}

  bool init() override {
    // Serial initialization is typically handled by core, but ensure it's available
    // Teensy usually has Serial ready; add a tiny delay for stability
    delay(10);
    return true;
  }

  void begin() override {
    Serial.println("[ExampleSerialSubsystem] begin");
  }

  void update() override {
    // This will be called at the rate selected in RobotObject (e.g., 50 Hz)
    static uint32_t counter = 0;
    Serial.print("[ExampleSerialSubsystem] tick ");
    Serial.print(counter++);
    Serial.print(" ms=");
    Serial.println(millis());
  }

  void pause() override {}

  void reset() override {
    Serial.println("[ExampleSerialSubsystem] reset");
  }

  const char* getInfo() override { static const char info[] = "ExampleSerialSubsystem"; return info; }

 private:
  const ExampleSerialSetup setup_;
};

// --- RobotManager wiring ---
static ExampleSerialSetup g_ex_setup("example_serial");
static ExampleSerialSubsystem g_example(g_ex_setup);

// RobotManager expects RobotObjects each tied to a TimerConfig bucket that RobotManager services
// RobotManager.cpp currently provides timekeepers for MS_20 (50 Hz) and MS_2 (500 Hz)
static RobotObject g_obj_example(g_example, MS_20); // run at 50 Hz

static std::vector<RobotObject*> g_objects{&g_obj_example};
static RobotManagerSetup g_rm_setup("robot_manager_basic", g_objects);
static RobotManager g_rm(g_rm_setup);

void setup() {
  // Initialize the RobotManager and the contained subsystem
  (void)g_rm.init();
  // BaseSubsystem::begin() is called by RobotManager::init() for subsystems
}

void loop() {
  g_rm.update();
}
