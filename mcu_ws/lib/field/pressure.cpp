
#include "pressure.h"

namespace Field {

bool PressureDriver::init() {
  pinMode(_setup._forcePin, INPUT);
  Serial.println("Pressure initialized");
  return true;
}

void PressureDriver::update() {
  unsigned long currentmiliseconds = millis();
  unsigned timeInterval = currentmiliseconds - _timeTracker;
  if (timeInterval >= 500) {
    _timeTracker = currentmiliseconds;
    _analogReading = analogRead(_setup._forcePin);
    Serial.print("Force: ");
    Serial.println(_analogReading);
    if (_analogReading <= 30) {
      task = COMPLETE;  // Duck OFF plate
    } else {
      task = NOTCOMPLETE;  // Duck ON plate
    }
  }
}

const char* PressureDriver::getInfo() {
  return ("\nID: " + std::string(setup_.getId()) +
          "\nAnalog Reading: " + std::to_string(_analogReading) +
          "\nTime Tracker: " + std::to_string(_timeTracker) +
          "\nForce Pin: " + std::to_string(_setup._forcePin))
      .c_str();
}

bool PressureDriver::getStatus() {
  if (task == COMPLETE) {
    return 1;

  } else {
    return 0;
  }
}

void PressureDriver::reset() {
  _analogReading = 0;
  _timeTracker = 0;
  task = NOTCOMPLETE;
};
};  // namespace Field