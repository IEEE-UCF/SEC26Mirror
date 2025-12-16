#include "controller.h"

namespace Field {

bool ControllerDriver::init() {
  _score[{false}];

  return true;
}

void ControllerDriver::update() {
  /*
   * placeholder for antenna network logic
   * message received from earth would go here
   * getting each task is my assumption
   * reset button also checked here
   */

  int buttonState = digitalRead(_setup._buttonPin);
  if (buttonState == HIGH) {
    reset();
  }

  int antenna;
  switch (antenna) {
    case 0:
      _score[0] = true;
      break;
    case 1:
      _score[1] = true;
      break;
    case 2:
      _score[2] = true;
      break;
    case 3:
      _score[3] = true;
      break;
    default:
      break;
  }
}

const char* ControllerDriver::getInfo() {
  return ("\nID: " + std::string(_setup.getId()) +
          "\nAntenna 1 Done: " + std::to_string(_score[0]) +
          "\nAntenna 2 Done: " + std::to_string(_score[1]) +
          "\nAntenna 3 Done: " + std::to_string(_score[2]) +
          "\nAntenna 4 Done: " + std::to_string(_score[3]))
      .c_str();
}

bool ControllerDriver::getStatus() {
  if (_score[0] & _score[1] & _score[2] & _score[3]) {
    return true;
  }

  return false;
}

void ControllerDriver::reset() {
  /*
   * placeholder for reset
   * message received from earth would go here
   * getting each task is my assumption
   */
  _score[{false}];
}

}  // namespace Field