#include "PCA9685Driver.h"

using namespace Robot;

PCA9685Driver::PCA9685Driver(const PCA9685DriverSetup& setup)
    : Classes::BaseDriver(setup), pwm_(setup.i2c_addr_) {
  // initialize buffers
  for (int i = 0; i < 16; ++i) {
    buffer_[i] = 0;
    buffer_dirty_[i] = false;
  }
}

bool PCA9685Driver::init() {
  pwm_.begin();
  pwm_.setPWMFreq(50); // default 50Hz for servos
  initSuccess_ = true;
  return true;
}

void PCA9685Driver::writeDigital(uint8_t channel, bool on) {
  if (channel > 15) return;
  if (on) {
    pwm_.setPWM(channel, 4096, 0); // full on
  } else {
    pwm_.setPWM(channel, 0, 4096); // full off
  }
}

void PCA9685Driver::writePWM(uint8_t channel, uint16_t duty) {
  if (channel > 15) return;
  if (duty >= 4095) {
    pwm_.setPWM(channel, 4096, 0);
  } else if (duty == 0) {
    pwm_.setPWM(channel, 0, 4096);
  } else {
    pwm_.setPWM(channel, 0, duty);
  }
}

void PCA9685Driver::bufferDigital(uint8_t channel, bool on) {
  if (channel > 15) return;
  buffer_[channel] = on ? 4095 : 0;
  buffer_dirty_[channel] = true;
}

void PCA9685Driver::bufferPWM(uint8_t channel, uint16_t duty) {
  if (channel > 15) return;
  buffer_[channel] = duty > 4095 ? 4095 : duty;
  buffer_dirty_[channel] = true;
}

void PCA9685Driver::applyBuffered() {
  for (uint8_t ch = 0; ch < 16; ++ch) {
    if (!buffer_dirty_[ch]) continue;
    uint16_t duty = buffer_[ch];
    if (duty >= 4095) {
      pwm_.setPWM(ch, 4096, 0);
    } else if (duty == 0) {
      pwm_.setPWM(ch, 0, 4096);
    } else {
      pwm_.setPWM(ch, 0, duty);
    }
    buffer_dirty_[ch] = false;
  }
}

void PCA9685Driver::update() {
  // Default driver update applies buffered values
  applyBuffered();
}
