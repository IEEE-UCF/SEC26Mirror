/**
 * @file earth.h
 * @brief Earth element IR receiver for field antennas
 * @date 01/18/2026
 *
 * Receives IR signals from antennas using NEC protocol.
 * Decodes antenna ID and color from received commands.
 * Also receives expected colors via ESP-NOW for verification.
 */

#ifndef EARTH_H
#define EARTH_H

#include <Arduino.h>
#include <BaseDriver.h>

#include <array>

#include "field-message.h"

namespace Field {

/// @brief Color codes received from antennas (IR protocol)
enum class AntennaColor : uint8_t {
  NONE = 0x00,
  RED = 0x09,
  GREEN = 0x0A,
  BLUE = 0x0C,
  PURPLE = 0x0F
};

/// @brief Number of antennas (excluding earth)
constexpr uint8_t NUM_ANTENNAS = 4;

/// @brief Antenna ID codes (first nibble of command)
constexpr uint8_t ANTENNA_CODES[NUM_ANTENNAS] = {0x00, 0x30, 0x50, 0x60};

/// @brief Setup configuration for EarthDriver
class EarthSetup : public Classes::BaseSetup {
 public:
  ~EarthSetup() = default;
  EarthSetup() = delete;

  /**
   * @brief Constructor for EarthSetup
   * @param _id A unique ID for the earth element
   * @param irPin Pin connected to IR receiver
   */
  EarthSetup(const char* _id, const uint8_t irPin)
      : BaseSetup(_id), _irPin(irPin) {}

  const uint8_t _irPin;
};

/// @brief Earth element driver for receiving antenna IR signals
class EarthDriver : public Classes::BaseDriver {
 public:
  ~EarthDriver() override = default;

  EarthDriver(const EarthSetup& setup) : BaseDriver(setup), _setup(setup) {
    reset();
  }

  /// @brief Initialize driver (start IR receiver)
  /// @return Success
  bool init() override;

  /// @brief Update driver (check for IR signals)
  void update() override;

  /// @brief Get info in the form of a data string
  /// @return data string
  const char* getInfo() override;

  /// @brief Check if all antennas have been received with valid colors
  /// @return true if all 4 antennas received valid colors
  bool getStatus();

  /// @brief Check if a specific antenna has been received
  /// @param antennaIndex Index (0-3) of antenna
  /// @return true if antenna received
  bool isAntennaReceived(uint8_t antennaIndex) const;

  /// @brief Get the color received via IR for a specific antenna
  /// @param antennaIndex Index (0-3) of antenna
  /// @return Color received (NONE if not received)
  AntennaColor getAntennaColor(uint8_t antennaIndex) const;

  /// @brief Get the expected color for a specific antenna
  /// @param antennaIndex Index (0-3) of antenna
  /// @return Expected color (NONE if not set)
  FieldColor getExpectedColor(uint8_t antennaIndex) const;

  /// @brief Set the expected color for a specific antenna (from ESP-NOW)
  /// @param antennaIndex Index (0-3) of antenna
  /// @param color Expected color
  void setExpectedColor(uint8_t antennaIndex, FieldColor color);

  /// @brief Check if a specific antenna's color is correct
  /// @param antennaIndex Index (0-3) of antenna
  /// @return true if received color matches expected
  bool isColorCorrect(uint8_t antennaIndex) const;

  /// @brief Get count of correct colors
  /// @return Number of antennas with correct colors
  uint8_t getCorrectCount() const;

  /// @brief Get the count of invalid/wrong color transmissions
  /// @return Wrong color count
  uint8_t getWrongColorCount() const;

  /// @brief Check if any transmission has been received
  /// @return true if at least one valid transmission received
  bool hasReceivedTransmission() const;

  /// @brief Reset all antenna states
  void reset();

  /// @brief Convert AntennaColor to FieldColor
  static FieldColor antennaToFieldColor(AntennaColor color);

  /// @brief Convert FieldColor to AntennaColor
  static AntennaColor fieldToAntennaColor(FieldColor color);

  /// @brief Convert color enum to string
  /// @param color The antenna color
  /// @return String representation of color
  static const char* colorToString(AntennaColor color);

 private:
  const EarthSetup _setup;

  /// @brief Track which antennas have received signals
  bool _antennaReceived[NUM_ANTENNAS];

  /// @brief Store received colors (from IR) for each antenna
  AntennaColor _antennaColors[NUM_ANTENNAS];

  /// @brief Store expected colors (from ESP-NOW) for each antenna
  FieldColor _expectedColors[NUM_ANTENNAS];

  /// @brief Count of wrong/invalid color transmissions
  uint8_t _wrongColorCount;

  /// @brief Flag indicating if any valid transmission received
  bool _hasReceivedTransmission;

  /// @brief Decode antenna index from command byte
  /// @param cmd Raw command byte
  /// @return Antenna index (0-3) or -1 if invalid
  int8_t decodeAntenna(uint8_t cmd);

  /// @brief Decode color from command byte
  /// @param cmd Raw command byte
  /// @return Color enum value
  AntennaColor decodeColor(uint8_t cmd);

  static constexpr uint8_t ANTENNA_MASK = 0xF0;
  static constexpr uint8_t COLOR_MASK = 0x0F;
  static constexpr uint8_t MAX_WRONG_COUNT = 8;
};

}  // namespace Field

#endif
