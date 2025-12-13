/**
 * @file button.h
 * @author Alexander Peacock
 * @brief Defines button functionality
 * @date 12/12/2025
 */

#ifndef BUTTON_H
#define BUTTON_H

#include <Arduino.h>
#include <BaseDriver.h>
#include <ezButton.h>

#include <array>

namespace Field {

class ButtonSetup : public Classes::BaseSetup {
 public:
  ~ButtonSetup() = default;
  ButtonSetup() = delete;

  ButtonSetup(const char* _id, const std::array<uint8_t, 3>& pins,
              const uint8_t ledPin, const uint8_t swPin)
      : BaseSetup(_id), _rygPins(pins), _ledPin(ledPin), _swPin(swPin) {};

  const std::array<uint8_t, 3> _rygPins;  // r, y, g
  const uint8_t _ledPin, _swPin;
};

class ButtonDriver : public Classes::BaseDriver {
 public:
  ~ButtonDriver() override = default;

  ButtonDriver(const ButtonSetup& setup)
      : BaseDriver(setup), _setup(setup), _counter(0) {
    pinMode(setup._ledPin, OUTPUT);  // LED PIN
    task = NOTCOMPLETE;
  };

  /// @brief  Initialize driver
  /// @return Success
  bool init() override;

  /// @brief Update driver
  void update() override;

  /// @brief Get info in the form of a data string
  /// @return data string
  const char* getInfo() override;

  /// @brief  Get status
  /// @return true if task completed
  bool getStatus();

  /// @brief  Reset button task
  void reset();

 private:
  const ButtonSetup _setup;
  int _counter;

  enum TASK { COMPLETE, NOTCOMPLETE } task;
};
};  // namespace Field

#endif
