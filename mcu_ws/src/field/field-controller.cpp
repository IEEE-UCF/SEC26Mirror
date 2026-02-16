#include "controller.h"

// Pin configuration
// I2C LCD address (typically 0x27 or 0x3F)
const uint8_t LCD_ADDRESS = 0x27;

// Joystick pins
const uint8_t JOY_X_PIN = 34;
const uint8_t JOY_Y_PIN = 35;
const uint8_t JOY_BTN_PIN = 32;

// Button pins
const uint8_t BTN_A_PIN = 33;
const uint8_t BTN_B_PIN = 25;

// Controller setup
Field::ControllerSetup setupController = {.lcdAddress = LCD_ADDRESS,
                                          .joystickXPin = JOY_X_PIN,
                                          .joystickYPin = JOY_Y_PIN,
                                          .joystickButtonPin = JOY_BTN_PIN,
                                          .buttonAPin = BTN_A_PIN,
                                          .buttonBPin = BTN_B_PIN};

Field::ControllerDriver controller(setupController);

void setup() {
  Serial.begin(115200);
  Serial.println("Field-Controller starting...");

  if (!controller.init()) {
    Serial.println("Controller initialization failed!");
    while (1) {
      delay(1000);
    }
  }

  Serial.println("Controller ready");
}

void loop() { controller.update(); }
