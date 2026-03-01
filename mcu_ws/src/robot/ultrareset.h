#pragma once
#include "RobotPins.h"
extern "C" void reset_hook(void);
  void reset_hook() {
    pinMode(PIN_SERVO_OE, OUTPUT);
    digitalWriteFast(PIN_SERVO_OE, HIGH);
    pinMode(PIN_MOTOR_OE, OUTPUT);
    digitalWriteFast(PIN_MOTOR_OE, HIGH);
}