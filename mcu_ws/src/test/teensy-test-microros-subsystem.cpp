#include <Arduino.h>

#include "ExampleMicrorosSubsystem.h"
#include "microros_manager_robot.h"

Subsystem::MicrorosManagerSetup managerSetup("microros_manager_test");
Subsystem::ExampleSubsystemSetup exampleSetup("example_subsystem_test");
Subsystem::MicrorosManager manager(managerSetup);
Subsystem::ExampleSubsystem example(exampleSetup);

void setup() {
  manager.init();
  manager.begin();
  // Register example subsystem so it can create its pubs/subs
  manager.registerParticipant(&example);
}

void loop() {
  manager.update();
  // Publish a status periodically from the example subsystem when connected
  static uint32_t last_ms = 0;
  uint32_t now = millis();
  if (now - last_ms > 1000) {
    example.publishStatus("OK");
    last_ms = now;
  }
}
