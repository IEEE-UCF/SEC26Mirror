/**
 * @file controller.h
 * @author Alexander Peacock
 * @brief Defines main controller for field
 * @date 12/13/2025
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>
#include <BaseDriver.h>

#include <array>
#include <string>

namespace Field {

class ControllerSetup : public Classes::BaseSetup {
 public:
  ~ControllerSetup() = default;
  ControllerSetup() = delete;

  ControllerSetup(const char* _id, uint8_t buttonPin)
      : Classes::BaseSetup(_id), _buttonPin(buttonPin) {

        };

  const uint8_t _buttonPin;
};

class ControllerDriver : public Classes::BaseDriver {
 public:
  ~ControllerDriver() = default;

  ControllerDriver(const ControllerSetup& setup)
      : BaseDriver(setup), _setup(setup), _score({false}) {
    pinMode(setup._buttonPin, INPUT);
  };
  /// @brief  Initialize driver
  /// @return Success
  bool init() override;

  /// @brief update based on changes made to field
  void update() override;

  /// @brief Get info in the form of a data string
  /// @return data string
  const char* getInfo() override;

  /// @brief tracks the scores of each task
  /// @return status of all tasks completed
  bool getStatus();

  /// @brief reset the entire field
  void reset();

 private:
  const ControllerSetup _setup;
  std::array<bool, 4> _score;
};

};  // namespace Field

#endif
