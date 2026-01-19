#include "button.h"

namespace Field {

bool ButtonDriver::init() {
  pinMode(_setup._rygPins[0], OUTPUT);  // r
  pinMode(_setup._rygPins[1], OUTPUT);  // y
  pinMode(_setup._rygPins[2], OUTPUT);  // g

  reset();
  Serial.println("Button initialized");
  return true;
}

void ButtonDriver::update() {
  button.loop();
  if (button.isPressed()) {
    _counter++;

    switch (_counter) {
      case 1:
        digitalWrite(_setup._rygPins[0], HIGH);  // r
        break;
      case 2:
        digitalWrite(_setup._rygPins[1], HIGH);  // y
        break;
      case 3:
        digitalWrite(_setup._rygPins[2], HIGH);  // g
        task = COMPLETE;
        break;
      default:
        break;
    }
  }
}

bool ButtonDriver::getStatus() {
  if (task == COMPLETE) {
    digitalWrite(_setup._ledPin, HIGH);
    return 1;
  } else {
    return 0;
  }
}

const char* ButtonDriver::getInfo() {
  return ("\nID: " + std::string(setup_.getId()) +
          "\nCounter: " + std::to_string(_counter) + "\nRed: pin-" +
          std::to_string(_setup._rygPins[0]) + "\nYellow: pin-" +
          std::to_string(_setup._rygPins[1]) + "\nGreen: pin-" +
          std::to_string(_setup._rygPins[2]))
      .c_str();
}

void ButtonDriver::reset() {
  _counter = 0;
  task = NOTCOMPLETE;

  digitalWrite(_setup._rygPins[0], LOW);  // r
  digitalWrite(_setup._rygPins[1], LOW);  // y
  digitalWrite(_setup._rygPins[2], LOW);  // g
  digitalWrite(_setup._ledPin, LOW);
}

};  // namespace Field