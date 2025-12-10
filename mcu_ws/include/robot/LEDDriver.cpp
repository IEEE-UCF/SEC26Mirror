#include "LEDDriver.h"

namespace Drivers {

LEDDriver::LEDDriver(uint8_t pin, bool startOn) : pin_(pin), state_(startOn) {}

void LEDDriver::init() {
  pinMode(pin_, OUTPUT);
  digitalWrite(pin_, state_ ? HIGH : LOW);
}

void LEDDriver::update() { return; }

void LEDDriver::on() {
  state_ = true;
  digitalWrite(pin_, HIGH);
}

void LEDDriver::off() {
  state_ = false;
  digitalWrite(pin_, LOW);
}

bool LEDDriver::getState() const { return state_; }

}  // namespace Drivers
