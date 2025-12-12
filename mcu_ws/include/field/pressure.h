/**
 * @file pressure.h
 * @author Jonathan Hernandez-Morales
 * @brief Defines pressure functionality
 * @date 12/12/2025
 */

#ifndef PRESSURE_H
#define PRESSURE_H

#include <Arduino.h>
#include <BaseDriver.h>

namespace Field {

class PressureSetup : public Classes::BaseSetup {
 public:
  ~PressureSetup() = default;
  PressureSetup() = delete;

  PressureSetup(const char* _id, const uint8_t forcePin)
      : BaseSetup(_id), _forcePin(forcePin) {};

  const uint8_t _forcePin;
};

class PressureDriver : public Classes::BaseDriver {
 public:
  ~PressureDriver() override = default;

  PressureDriver(const PressureSetup& setup)
      : BaseDriver(setup), _setup(setup), _analogReading(0), _timeTracker(0) {
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

  /// @brief  Reset pressure task
  void reset();

 private:
  const PressureSetup _setup;
  int _analogReading;
  unsigned long _timeTracker;

  enum TASK { COMPLETE, NOTCOMPLETE } task;
};
};  // namespace Field
#endif