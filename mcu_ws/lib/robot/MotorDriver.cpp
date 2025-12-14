#include "MotorDriver.h"

namespace Drivers {

bool MotorDriver::init() {
  initSuccess_ = true;

  if (setup_.pwm_pin >= 0) {
    analogWriteFrequency(setup_.pwm_pin, 18310.55);
    pinMode(setup_.pwm_pin, OUTPUT);
  } else {
    initSuccess_ = false;
  }

  if (setup_.dir_pin >= 0) {
    pinMode(setup_.dir_pin, OUTPUT);
  } else {
    initSuccess_ = false;
  }

  return initSuccess_;
}

void MotorDriver::setPWM(int speed) {
  pwmOut = map(abs(speed), 0, SPEED_MAX, 0, PWM_MAX);
  pwmOut = constrain(pwmOut, 0, 255);
  motorDirection = (speed >= 0);
}

void MotorDriver::update() {
  digitalWrite(setup_.dir_pin, motorDirection);
  analogWrite(setup_.pwm_pin, pwmOut);
}

char* MotorDriver::getInfo() {
  snprintf(infoBuffer_, sizeof(infoBuffer_), "Motor: %s, PWM: %d, Dir: %d",
           setup_.getId(), pwmOut, motorDirection);
  return infoBuffer_;
}

}  // namespace Drivers