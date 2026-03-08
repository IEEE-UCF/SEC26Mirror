#include "I2CPowerDriver.h"

#include "I2CDMABus.h"
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
  if (dma_bus_) return;

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

// ── DMA support ──────────────────────────────────────────────────────────

void I2CPowerDriver::queueDMAReads() {
  if (!dma_bus_ || !initSuccess_) return;
  dma_bus_->queueRead(_setup._address, 0x01, dma_rx_shunt_, 2);
  dma_bus_->queueRead(_setup._address, 0x02, dma_rx_bus_, 2);
}

void I2CPowerDriver::processDMAResults() {
  if (!initSuccess_) return;

  // Shunt voltage reg 0x01: signed 16-bit, 10 uV/LSB -> 0.01 mV/LSB
  int16_t raw_shunt = (int16_t)((dma_rx_shunt_[0] << 8) | dma_rx_shunt_[1]);
  _data.shuntVoltagemV = raw_shunt * 0.01f;

  // Bus voltage reg 0x02: bits [15:3], 4 mV/LSB
  uint16_t raw_bus =
      (uint16_t)(((uint16_t)dma_rx_bus_[0] << 8) | dma_rx_bus_[1]) >> 3;
  _data.busVoltage = raw_bus * 0.004f;

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
