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

  ControllerSetup(const char* _id) : Classes::BaseSetup(_id) {};
};

class ControllerDriver : public Classes::BaseDriver {
 public:
  ~ControllerDriver() = default;

  ControllerDriver(const ControllerSetup& setup)
      : BaseDriver(setup), _setup(setup), _score({false}) {};

  bool init() override;
  void update() override;
  const char* getInfo() override;

  /// @brief tracks the scores of each task
  bool getStatus();

  /// @brief reset the entire field
  void reset();

 private:
  const ControllerSetup _setup;
  std::array<bool, 4> _score;
};

};  // namespace Field

#endif
