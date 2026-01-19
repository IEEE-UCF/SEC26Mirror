#include "earth.h"

namespace Field {

bool EarthDriver::init() {
  IrReceiver.begin(_setup._irPin, ENABLE_LED_FEEDBACK);
  reset();
  Serial.println("Earth IR receiver initialized");
  return true;
}

void EarthDriver::update() {
  if (IrReceiver.decode()) {
    if (IrReceiver.decodedIRData.protocol == NEC) {
      uint8_t cmd = IrReceiver.decodedIRData.command;
      int8_t antennaIndex = decodeAntenna(cmd);
      AntennaColor color = decodeColor(cmd);

      if (antennaIndex >= 0) {
        _hasReceivedTransmission = true;
        _antennaReceived[antennaIndex] = true;

        if (color != AntennaColor::NONE) {
          _antennaColors[antennaIndex] = color;
          Serial.print("Antenna ");
          Serial.print(antennaIndex + 1);
          Serial.print(": ");
          Serial.print(colorToString(color));

          // Check if color matches expected
          if (_expectedColors[antennaIndex] != FieldColor::NONE) {
            bool correct = isColorCorrect(antennaIndex);
            Serial.print(correct ? " (CORRECT)" : " (WRONG)");
          }
          Serial.println();
        } else {
          if (_wrongColorCount < MAX_WRONG_COUNT) {
            _wrongColorCount++;
          }
          Serial.print("Antenna ");
          Serial.print(antennaIndex + 1);
          Serial.println(": Invalid color");
        }
      } else {
        Serial.println("Invalid antenna code received");
        _hasReceivedTransmission = true;
      }
    }
    IrReceiver.resume();
  }
}

bool EarthDriver::getStatus() {
  for (int i = 0; i < NUM_ANTENNAS; i++) {
    if (!_antennaReceived[i] || _antennaColors[i] == AntennaColor::NONE) {
      return false;
    }
  }
  return true;
}

bool EarthDriver::isAntennaReceived(uint8_t antennaIndex) const {
  if (antennaIndex >= NUM_ANTENNAS) return false;
  return _antennaReceived[antennaIndex];
}

AntennaColor EarthDriver::getAntennaColor(uint8_t antennaIndex) const {
  if (antennaIndex >= NUM_ANTENNAS) return AntennaColor::NONE;
  return _antennaColors[antennaIndex];
}

FieldColor EarthDriver::getExpectedColor(uint8_t antennaIndex) const {
  if (antennaIndex >= NUM_ANTENNAS) return FieldColor::NONE;
  return _expectedColors[antennaIndex];
}

void EarthDriver::setExpectedColor(uint8_t antennaIndex, FieldColor color) {
  if (antennaIndex >= NUM_ANTENNAS) return;
  _expectedColors[antennaIndex] = color;
  Serial.print("Expected color for antenna ");
  Serial.print(antennaIndex + 1);
  Serial.print(": ");
  Serial.println(Field::colorToString(color));
}

bool EarthDriver::isColorCorrect(uint8_t antennaIndex) const {
  if (antennaIndex >= NUM_ANTENNAS) return false;
  if (_antennaColors[antennaIndex] == AntennaColor::NONE) return false;
  if (_expectedColors[antennaIndex] == FieldColor::NONE) return false;

  return antennaToFieldColor(_antennaColors[antennaIndex]) ==
         _expectedColors[antennaIndex];
}

uint8_t EarthDriver::getCorrectCount() const {
  uint8_t count = 0;
  for (int i = 0; i < NUM_ANTENNAS; i++) {
    if (isColorCorrect(i)) count++;
  }
  return count;
}

uint8_t EarthDriver::getWrongColorCount() const { return _wrongColorCount; }

bool EarthDriver::hasReceivedTransmission() const {
  return _hasReceivedTransmission;
}

void EarthDriver::reset() {
  for (int i = 0; i < NUM_ANTENNAS; i++) {
    _antennaReceived[i] = false;
    _antennaColors[i] = AntennaColor::NONE;
    _expectedColors[i] = FieldColor::NONE;
  }
  _wrongColorCount = 0;
  _hasReceivedTransmission = false;
}

FieldColor EarthDriver::antennaToFieldColor(AntennaColor color) {
  switch (color) {
    case AntennaColor::RED:
      return FieldColor::RED;
    case AntennaColor::GREEN:
      return FieldColor::GREEN;
    case AntennaColor::BLUE:
      return FieldColor::BLUE;
    case AntennaColor::PURPLE:
      return FieldColor::PURPLE;
    default:
      return FieldColor::NONE;
  }
}

AntennaColor EarthDriver::fieldToAntennaColor(FieldColor color) {
  switch (color) {
    case FieldColor::RED:
      return AntennaColor::RED;
    case FieldColor::GREEN:
      return AntennaColor::GREEN;
    case FieldColor::BLUE:
      return AntennaColor::BLUE;
    case FieldColor::PURPLE:
      return AntennaColor::PURPLE;
    default:
      return AntennaColor::NONE;
  }
}

const char* EarthDriver::colorToString(AntennaColor color) {
  switch (color) {
    case AntennaColor::RED:
      return "RED";
    case AntennaColor::GREEN:
      return "GREEN";
    case AntennaColor::BLUE:
      return "BLUE";
    case AntennaColor::PURPLE:
      return "PURPLE";
    case AntennaColor::NONE:
    default:
      return "NONE";
  }
}

const char* EarthDriver::getInfo() {
  static char buffer[256];
  snprintf(buffer, sizeof(buffer),
           "\nID: %s\n"
           "Ant1: %s (%s) exp:%s %s\n"
           "Ant2: %s (%s) exp:%s %s\n"
           "Ant3: %s (%s) exp:%s %s\n"
           "Ant4: %s (%s) exp:%s %s\n"
           "Correct: %d, Wrong: %d",
           setup_.getId(),
           _antennaReceived[0] ? "ON" : "OFF", colorToString(_antennaColors[0]),
           Field::colorToString(_expectedColors[0]),
           isColorCorrect(0) ? "OK" : "",
           _antennaReceived[1] ? "ON" : "OFF", colorToString(_antennaColors[1]),
           Field::colorToString(_expectedColors[1]),
           isColorCorrect(1) ? "OK" : "",
           _antennaReceived[2] ? "ON" : "OFF", colorToString(_antennaColors[2]),
           Field::colorToString(_expectedColors[2]),
           isColorCorrect(2) ? "OK" : "",
           _antennaReceived[3] ? "ON" : "OFF", colorToString(_antennaColors[3]),
           Field::colorToString(_expectedColors[3]),
           isColorCorrect(3) ? "OK" : "",
           getCorrectCount(), _wrongColorCount);
  return buffer;
}

int8_t EarthDriver::decodeAntenna(uint8_t cmd) {
  uint8_t antennaBits = cmd & ANTENNA_MASK;
  for (int i = 0; i < NUM_ANTENNAS; i++) {
    if (antennaBits == ANTENNA_CODES[i]) {
      return i;
    }
  }
  return -1;
}

AntennaColor EarthDriver::decodeColor(uint8_t cmd) {
  uint8_t colorBits = cmd & COLOR_MASK;
  switch (colorBits) {
    case static_cast<uint8_t>(AntennaColor::RED):
      return AntennaColor::RED;
    case static_cast<uint8_t>(AntennaColor::GREEN):
      return AntennaColor::GREEN;
    case static_cast<uint8_t>(AntennaColor::BLUE):
      return AntennaColor::BLUE;
    case static_cast<uint8_t>(AntennaColor::PURPLE):
      return AntennaColor::PURPLE;
    default:
      return AntennaColor::NONE;
  }
}

}  // namespace Field
