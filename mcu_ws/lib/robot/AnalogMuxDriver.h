/**
 * @file AnalogMuxDriver.h
 * @author Alexander Peacock
 * @brief Defines the Driver for the analog mux CD74HC4067
 * @date 12/14/2025
 */

#ifndef ANALOGMUXDRIVER_H
#define ANALOGMUXDRIVER_H

#include <Arduino.h>
#include <BaseDriver.h>
#include <CD74HC4067.h>

#include <array>
#include <string>

#include "AnalogRead.h"

namespace Drivers {

class AnalogMuxDriverSetup : public Classes::BaseSetup {
 public:
  ~AnalogMuxDriverSetup() = default;
  AnalogMuxDriverSetup() = delete;

  AnalogMuxDriverSetup(const char* _id, std::array<uint8_t, 4> sPins,
                       uint8_t sigPin)
      : Classes::BaseSetup(_id), _sPins(sPins), _sigPin(sigPin) {

        };

  const std::array<uint8_t, 4> _sPins;
  const float _sigPin;
};

class AnalogMuxDriver : public Classes::BaseDriver {
 public:
  AnalogMuxDriver(const AnalogMuxDriverSetup& setup)
      : BaseDriver(setup), _setup(setup) {
    pinMode(_setup._sigPin, OUTPUT);
  };
  ~AnalogMuxDriver() override = default;

  /// @brief  Initialize driver
  /// @return Success
  bool init() override;

  /// @brief read from select channel
  /// @return int
  int readMux(uint8_t ch);

  /// @brief Get info in the form of a data string
  /// @return data string
  const char* getInfo() override;

 private:
  const AnalogMuxDriverSetup _setup;
};

};  // namespace Drivers

#endif