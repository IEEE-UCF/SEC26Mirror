
#include "I2CPowerDriver.h"

namespace Drivers {

bool I2CPowerDriver::init() {
  _sensor = Adafruit_INA228();

  Wire.begin();

  if (!_sensor.begin()) {
    return false;
  }

  _sensor.setShunt(_setup._shuntRes, _setup._maxCurrent);
  _sensor.setAveragingCount(INA228_COUNT_16);
  _sensor.setVoltageConversionTime(INA228_TIME_150_us);
  _sensor.setCurrentConversionTime(INA228_TIME_280_us);

  return true;
}

void I2CPowerDriver::update() {
  _data.currentmA = _sensor.getCurrent_mA();
  _data.busVoltage = _sensor.getBusVoltage_V();
  _data.shuntVoltagemW = _sensor.getShuntVoltage_mV();
  _data.powermW = _sensor.getPower_mW();
  _data.energy = _sensor.readEnergy();
  _data.charge = _sensor.readCharge();
  _data.temp = _sensor.readDieTemp();
}

float I2CPowerDriver::getVoltage() { return _data.busVoltage; }

float I2CPowerDriver::getCurrentmW() { return _data.currentmA; }

float I2CPowerDriver::getPowermW() { return _data.powermW; }

float I2CPowerDriver::getTemp() { return _data.temp; }

const char* I2CPowerDriver::getInfo() {
  return ("\nID: " + std::string(_setup.getId()) +
          "\nCurrent: " + std::to_string(_data.currentmA) + " mA" +
          "\nBus Voltage: " + std::to_string(_data.busVoltage) + " V" +
          "\nShunt Voltage: " + std::to_string(_data.shuntVoltagemW) + " mV" +
          "\nPower: " + std::to_string(_data.powermW) + " mW" +
          "\nEnergy: " + std::to_string(_data.energy) + " J" +
          "\nCharge: " + std::to_string(_data.charge) + " C" +
          "\nTemperature: " + std::to_string(_data.temp) + " *C")
      .c_str();
}

};  // namespace Drivers
