#include "I2CPowerDriver.h"

#include "I2CMuxDriver.h"

namespace Drivers {

bool I2CPowerDriver::init() {
  I2CBus::Lock lock(_setup._wire);
  _setup._wire.begin();

  // Route I2C through the mux channel if one is configured.
  // selectChannel() also acquires the bus lock (recursive mutex — safe).
  if (_setup._mux != nullptr &&
      !_setup._mux->selectChannel(_setup._muxChannel)) {
    initSuccess_ = false;
    return initSuccess_;
  }

  if (!_sensor.begin(&_setup._wire)) {
    initSuccess_ = false;
    return initSuccess_;
  }

  // setCalibration_32V_2A() is used solely for its hardware config:
  //   BVOLTAGERANGE_32V, GAIN_8 (±320 mV shunt range), 12-bit ADC, continuous.
  // Current and power are NOT read from the library's calibration-derived
  // registers — they are computed in update() from the raw shunt voltage so
  // that any shunt resistor value works correctly.
  _sensor.setCalibration_32V_2A();

  initSuccess_ = true;
  return initSuccess_;
}

void I2CPowerDriver::update() {
  if (!initSuccess_) return;
  I2CBus::Lock lock(_setup._wire);

  if (_setup._mux != nullptr) {
    _setup._mux->selectChannel(_setup._muxChannel);
  }

  _data.shuntVoltagemV = _sensor.getShuntVoltage_mV();
  _data.busVoltage = _sensor.getBusVoltage_V();

  // Derive current and power from raw shunt voltage so the result is correct
  // for any shunt resistor, not just the 0.1 Ω assumed by the library.
  //   I (A)  = V_shunt (V)  / R_shunt (Ω)
  //   I (mA) = V_shunt (mV) / R_shunt (Ω)
  //   P (mW) = V_bus  (V)   * I (mA)      [V × mA = mW]
  _data.currentmA = _data.shuntVoltagemV / _setup._shuntOhm;
  _data.powermW = _data.busVoltage * _data.currentmA;
  _data.loadVoltage = _data.busVoltage + (_data.shuntVoltagemV / 1000.0f);
}

float I2CPowerDriver::getVoltage() const { return _data.loadVoltage; }
float I2CPowerDriver::getCurrentmA() const { return _data.currentmA; }
float I2CPowerDriver::getPowermW() const { return _data.powermW; }
float I2CPowerDriver::getShuntVoltagemV() const { return _data.shuntVoltagemV; }

const char* I2CPowerDriver::getInfo() {
  snprintf(_infoBuf, sizeof(_infoBuf),
           "ID: %s\nLoad V: %.3f V\nCurrent: %.1f mA\nPower: %.1f mW",
           _setup.getId(), _data.loadVoltage, _data.currentmA, _data.powermW);
  return _infoBuf;
}

}  // namespace Drivers
