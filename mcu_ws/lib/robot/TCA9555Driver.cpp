#include "TCA9555Driver.h"

namespace Drivers {

// ── Register addresses
// ────────────────────────────────────────────────────────

static constexpr uint8_t REG_INPUT0 = 0x00;
static constexpr uint8_t REG_INPUT1 = 0x01;
static constexpr uint8_t REG_OUTPUT0 = 0x02;
static constexpr uint8_t REG_OUTPUT1 = 0x03;
static constexpr uint8_t REG_CONFIG0 = 0x06;
static constexpr uint8_t REG_CONFIG1 = 0x07;

// ── Private helpers
// ───────────────────────────────────────────────────────────

bool TCA9555Driver::writeReg(uint8_t reg, uint8_t value) {
  setup_.wire_.beginTransmission(setup_.address_);
  setup_.wire_.write(reg);
  setup_.wire_.write(value);
  return setup_.wire_.endTransmission() == 0;
}

uint8_t TCA9555Driver::readReg(uint8_t reg) {
  setup_.wire_.beginTransmission(setup_.address_);
  setup_.wire_.write(reg);
  setup_.wire_.endTransmission(false);  // repeated start
  setup_.wire_.requestFrom(setup_.address_, static_cast<uint8_t>(1));
  return setup_.wire_.available() ? setup_.wire_.read() : 0x00;
}

// ── Lifecycle
// ─────────────────────────────────────────────────────────────────

bool TCA9555Driver::init() {
  I2CBus::Lock lock(setup_.wire_);
  setup_.wire_.begin();

  // Push shadow registers to chip — both ports default to all-input.
  bool ok = writeReg(REG_OUTPUT0, data_.outputPort0) &&
            writeReg(REG_OUTPUT1, data_.outputPort1) &&
            writeReg(REG_CONFIG0, data_.configPort0) &&
            writeReg(REG_CONFIG1, data_.configPort1);

  initSuccess_ = ok;
  return ok;
}

void TCA9555Driver::update() {
  I2CBus::Lock lock(setup_.wire_);
  data_.inputPort0 = readReg(REG_INPUT0);
  data_.inputPort1 = readReg(REG_INPUT1);
}

const char* TCA9555Driver::getInfo() {
  snprintf(infoBuf_, sizeof(infoBuf_),
           "TCA9555: ID=%s addr=0x%02X p0=0x%02X p1=0x%02X", setup_.getId(),
           setup_.address_, data_.inputPort0, data_.inputPort1);
  return infoBuf_;
}

// ── Pin-level API
// ─────────────────────────────────────────────────────────────

bool TCA9555Driver::pinMode(uint8_t pin, uint8_t mode) {
  if (pin > 15) return false;
  I2CBus::Lock lock(setup_.wire_);

  if (pin < 8) {
    if (mode == OUTPUT)
      data_.configPort0 &= ~(1 << pin);
    else
      data_.configPort0 |= (1 << pin);
    return writeReg(REG_CONFIG0, data_.configPort0);
  } else {
    uint8_t bit = pin - 8;
    if (mode == OUTPUT)
      data_.configPort1 &= ~(1 << bit);
    else
      data_.configPort1 |= (1 << bit);
    return writeReg(REG_CONFIG1, data_.configPort1);
  }
}

bool TCA9555Driver::digitalWrite(uint8_t pin, bool val) {
  if (pin > 15) return false;
  I2CBus::Lock lock(setup_.wire_);

  if (pin < 8) {
    if (val)
      data_.outputPort0 |= (1 << pin);
    else
      data_.outputPort0 &= ~(1 << pin);
    return writeReg(REG_OUTPUT0, data_.outputPort0);
  } else {
    uint8_t bit = pin - 8;
    if (val)
      data_.outputPort1 |= (1 << bit);
    else
      data_.outputPort1 &= ~(1 << bit);
    return writeReg(REG_OUTPUT1, data_.outputPort1);
  }
}

bool TCA9555Driver::digitalRead(uint8_t pin) const {
  if (pin < 8) return (data_.inputPort0 >> pin) & 0x01;
  return (data_.inputPort1 >> (pin - 8)) & 0x01;
}

// ── Port-level API
// ────────────────────────────────────────────────────────────

bool TCA9555Driver::writePort(uint8_t port, uint8_t value) {
  I2CBus::Lock lock(setup_.wire_);
  if (port == 0) {
    data_.outputPort0 = value;
    return writeReg(REG_OUTPUT0, value);
  }
  if (port == 1) {
    data_.outputPort1 = value;
    return writeReg(REG_OUTPUT1, value);
  }
  return false;
}

uint8_t TCA9555Driver::readPort(uint8_t port) const {
  if (port == 0) return data_.inputPort0;
  if (port == 1) return data_.inputPort1;
  return 0;
}

bool TCA9555Driver::configurePort(uint8_t port, uint8_t mask) {
  I2CBus::Lock lock(setup_.wire_);
  if (port == 0) {
    data_.configPort0 = mask;
    return writeReg(REG_CONFIG0, mask);
  }
  if (port == 1) {
    data_.configPort1 = mask;
    return writeReg(REG_CONFIG1, mask);
  }
  return false;
}

}  // namespace Drivers
