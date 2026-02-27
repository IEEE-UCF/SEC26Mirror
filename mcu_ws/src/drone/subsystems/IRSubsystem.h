#pragma once
// IRSubsystem: IR NEC transmitter for communicating antenna LED colors to Earth
// Protocol: NEC, Address: 0xBB
// Command byte: upper 4 bits = antenna code, lower 4 bits = color code
// Table from ruleset:
//   Antenna 1=0x00, 2=0x30, 3=0x50, 4=0x60
//   Red=0x09, Green=0x0A, Blue=0x0C, Purple=0x0F

#include <Arduino.h>

namespace Drone {

struct IRConfig {
  uint8_t ir_pin = 4;       // GPIO pin for IR LED
  uint16_t address = 0xBB;  // NEC address (from ruleset)
};

// Antenna codes (upper nibble of command byte)
static constexpr uint8_t ANTENNA_1_CODE = 0x00;
static constexpr uint8_t ANTENNA_2_CODE = 0x30;
static constexpr uint8_t ANTENNA_3_CODE = 0x50;
static constexpr uint8_t ANTENNA_4_CODE = 0x60;

// Color codes (lower nibble of command byte)
static constexpr uint8_t COLOR_RED = 0x09;
static constexpr uint8_t COLOR_GREEN = 0x0A;
static constexpr uint8_t COLOR_BLUE = 0x0C;
static constexpr uint8_t COLOR_PURPLE = 0x0F;

class IRSubsystem {
 public:
  explicit IRSubsystem(const IRConfig& cfg = IRConfig{}) : cfg_(cfg) {}

  bool init();

  // Transmit a single antenna + color combo
  // antenna_id: 1-4, color: one of COLOR_* constants
  void transmitAntennaColor(uint8_t antenna_id, uint8_t color_code);

  // Transmit all 4 antenna colors (call with the array from mission)
  // colors[0] = antenna 1 color, colors[1] = antenna 2, etc
  void transmitAll(const uint8_t colors[4]);

  bool isInitialized() const { return initialized_; }

 private:
  uint8_t antennaIdToCode(uint8_t antenna_id);

  IRConfig cfg_;
  bool initialized_ = false;
};

}  // namespace Drone
