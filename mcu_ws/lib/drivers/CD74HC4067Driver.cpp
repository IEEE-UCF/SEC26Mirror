/**
 * @file CD74HC4067Driver.cpp
 * @brief Implementation of CD74HC4067 multiplexer driver
 * @author Claude Code
 * @date 12/24/2025
 */

#include "CD74HC4067Driver.h"

namespace Drivers {

CD74HC4067Driver::CD74HC4067Driver(const CD74HC4067DriverSetup& setup)
    : BaseDriver(setup), setup_(setup), current_channel_(0) {}

bool CD74HC4067Driver::init() {
  // Configure select pins as outputs
  pinMode(setup_.s0_, OUTPUT);
  pinMode(setup_.s1_, OUTPUT);
  pinMode(setup_.s2_, OUTPUT);
  pinMode(setup_.s3_, OUTPUT);

  // Configure signal pin as input by default
  pinMode(setup_.sig_pin_, INPUT);

  // Select channel 0 by default
  selectChannel(0);

  initSuccess_ = true;
  return true;
}

void CD74HC4067Driver::update() {
  // Multiplexer operations are synchronous, no buffering needed
}

const char* CD74HC4067Driver::getInfo() {
  snprintf(infoBuffer_, sizeof(infoBuffer_),
           "CD74HC4067 S[%d,%d,%d,%d] SIG:%d CH:%d", setup_.s0_, setup_.s1_,
           setup_.s2_, setup_.s3_, setup_.sig_pin_, current_channel_);
  return infoBuffer_;
}

void CD74HC4067Driver::selectChannel(uint8_t channel) {
  if (channel >= 16) return;  // Only 16 channels available

  current_channel_ = channel;

  // Set select pins according to channel number
  ::digitalWrite(setup_.s0_, (channel & 0x01) ? HIGH : LOW);
  ::digitalWrite(setup_.s1_, (channel & 0x02) ? HIGH : LOW);
  ::digitalWrite(setup_.s2_, (channel & 0x04) ? HIGH : LOW);
  ::digitalWrite(setup_.s3_, (channel & 0x08) ? HIGH : LOW);

  // Small delay to allow multiplexer to settle
  delayMicroseconds(1);
}

uint8_t CD74HC4067Driver::digitalRead() {
  return ::digitalRead(setup_.sig_pin_);
}

void CD74HC4067Driver::digitalWrite(uint8_t value) {
  // Configure signal pin as output
  pinMode(setup_.sig_pin_, OUTPUT);
  ::digitalWrite(setup_.sig_pin_, value);
}

uint16_t CD74HC4067Driver::analogRead() {
  return ::analogRead(setup_.sig_pin_);
}

}  // namespace Drivers
