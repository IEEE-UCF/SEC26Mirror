
#include "crank.h"

namespace Field {

bool CrankDriver::init() {
  Serial.begin(9600);
  reset();
  _prevClk = digitalRead(_setup._clk_k);

  return true;
}

void CrankDriver::update() {
  _clkState = digitalRead(_setup._clk_k);

  if (_clkState != _prevClk &&
      _clkState == HIGH) {  // counter increments on rising edge
    if (digitalRead(_setup._dt_k) == HIGH) {  // dt for direction
      _counter--;
    } else {
      _counter++;
      digitalWrite(2, HIGH);
    }

    Serial.print(" - counter: ");
    Serial.println(_counter);
    if ((_counter == 18) || (_counter == -18)) {
      digitalWrite(_setup._ledPin, HIGH);
      task = COMPLETE;
    }
  }
  _prevClk = _clkState;  // save last clock state
}

bool CrankDriver::getStatus() {
  if (task == COMPLETE) {
    return 1;
  } else {
    return 0;
  }
}

const char* CrankDriver::getInfo() {
  return ("\nID: " + std::string(setup_.getId()) +
          "\nCounter: " + std::to_string(_counter) + "\nCLK: pin-" +
          std::to_string(_setup._clk_k) + "\nDT: pin-" +
          std::to_string(_setup._dt_k) + "\nLED: pin-" +
          std::to_string(_setup._ledPin))
      .c_str();
}

void CrankDriver::reset() {
  _prevClk = digitalRead(_setup._clk_k);
  _counter = 0;
  task = NOTCOMPLETE;
}
};  // namespace Field
