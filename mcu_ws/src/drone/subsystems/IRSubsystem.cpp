#include "IRSubsystem.h"

// IRremote library: NEC protocol
#define SEND_PWM_BY_TIMER
#include <IRremote.hpp>

namespace Drone {

bool IRSubsystem::init() {
  IrSender.begin(cfg_.ir_pin);
  initialized_ = true;
  return true;
}

uint8_t IRSubsystem::antennaIdToCode(uint8_t antenna_id) {
  switch (antenna_id) {
    case 1:
      return ANTENNA_1_CODE;
    case 2:
      return ANTENNA_2_CODE;
    case 3:
      return ANTENNA_3_CODE;
    case 4:
      return ANTENNA_4_CODE;
    default:
      return 0x00;
  }
}

void IRSubsystem::transmitAntennaColor(uint8_t antenna_id, uint8_t color_code) {
  if (!initialized_) return;

  uint8_t command = antennaIdToCode(antenna_id) | color_code;
  IrSender.sendNEC(cfg_.address, command, 0);

  // Small delay between transmissions to avoid overlap
  delay(100);
}

void IRSubsystem::transmitAll(const uint8_t colors[4]) {
  if (!initialized_) return;

  for (uint8_t i = 0; i < 4; i++) {
    if (colors[i] != 0) {
      transmitAntennaColor(i + 1, colors[i]);
      delay(200);  // gap between messages for Earth Arduino to process
    }
  }
}

}  // namespace Drone
