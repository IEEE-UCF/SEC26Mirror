#include <controller.h>

uint8_t buttonPin = 17;  // placeholder

// controller configuration
Field::ControllerSetup setupController =
    Field::ControllerSetup("controller", buttonPin);
Field::ControllerDriver Controller1 = Field::ControllerDriver(setupController);

void setup() { Controller1.init(); }

void loop() {
  Controller1.update();

  if (Controller1.getStatus()) {
    Serial.println("All tasks finished");
  }
}