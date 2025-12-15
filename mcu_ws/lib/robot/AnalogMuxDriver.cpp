#include "AnalogMuxDriver.h"

namespace Drivers {

bool AnalogMuxDriver::init() { 
     
    return true; 
}

int AnalogMuxDriver::readMux(uint8_t ch) {
  CD74HC4067 aMux(_setup._sPins[0], _setup._sPins[1], _setup._sPins[2],
            _setup._sPins[3]);

  aMux.channel(ch);

  int value = analogRead(_setup._sigPin);
  return value;
}

const char* AnalogMuxDriver::getInfo() {
  return ("\nID: " + std::string(_setup.getId()) +
          "\nSIG Pin: " + std::to_string(_setup._sigPin) + "\nS0,S1,S2,S3: " +
          std::to_string(_setup._sPins[0]) + std::to_string(_setup._sPins[1]) +
          std::to_string(_setup._sPins[2]) + std::to_string(_setup._sPins[3]))
      .c_str();
}

};  // namespace Drivers
