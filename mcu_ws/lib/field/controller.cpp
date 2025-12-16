#include "controller.h"

namespace Field {

bool ControllerDriver::init() {
  _score[{false}];

  return true;
}

void ControllerDriver::update() {
    int antenna; // placeholder for antenna network logic
    switch(antenna){
        case 0: _score[0] = true;
        break;
        case 1: _score[1] = true;
        break;
        case 2: _score[2] = true;
        break;
        case 3: _score[3] = true;
        break;
        default:
        break;
    }
}

const char* ControllerDriver::getInfo() {
  return ("\nID: " + std::string(_setup.getId()) +
          "\nAntenna 1 Score: " + std::to_string(_score[0]) +
          "\nAntenna 2 Score: " + std::to_string(_score[1]) +
          "\nAntenna 3 Score: " + std::to_string(_score[2]) +
          "\nAntenna 4 Score: " + std::to_string(_score[3]))
      .c_str();
}

bool ControllerDriver::getStatus() {
    if(_score[0] & _score[1] & _score[2] & _score[3]){
        return true;
    }

    return false;
}

void ControllerDriver::reset() {
    _score[{false}];
}

}  // namespace Field