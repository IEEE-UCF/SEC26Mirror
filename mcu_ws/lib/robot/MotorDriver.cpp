#include "MotorDriver.h"

namespace Drivers {

bool MotorDriver::init() {
  initSuccess_ = true;

  if (setup_.pwm_pin) {
    setup_.pwm_pin->pinMode(HAL::PinMode::PIN_OUTPUT);
  } else {
    initSuccess_ = false;
  }

  if (setup_.dir_pin) {
    setup_.dir_pin->pinMode(HAL::PinMode::PIN_OUTPUT);
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
  if (prevReverseTime > 103) {
    prevReverseTime = 0;
  }

  if (prevReverseTime > 100) {
    setup_.dir_pin->digitalWrite(!motorDirection
                                     ? HAL::DigitalState::STATE_HIGH
                                     : HAL::DigitalState::STATE_LOW);
    setup_.pwm_pin->analogWrite(250);
  } else {
    setup_.dir_pin->digitalWrite(motorDirection ? HAL::DigitalState::STATE_HIGH
                                                : HAL::DigitalState::STATE_LOW);
    setup_.pwm_pin->analogWrite(pwmOut);
  }
}

const char* MotorDriver::getInfo() {
  snprintf(infoBuffer_, sizeof(infoBuffer_), "Motor: %s, PWM: %d, Dir: %d",
           setup_.getId(), pwmOut, motorDirection);
  return infoBuffer_;
}

}  // namespace Drivers