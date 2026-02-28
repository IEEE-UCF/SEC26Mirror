/**
 * @file crank.h
 * @author Alexander Peacock
 * @brief Defines crank functionality
 * @date 12/12/2025
 */

#ifndef CRANK_H
#define CRANK_H

#include <Arduino.h>
#include <BaseDriver.h>

namespace Field {

// defines crank program
typedef struct {
  uint8_t led, clk_k, dt_k;
} CrankConfig_t;

class CrankSetup : public Classes::BaseSetup {
 public:
  ~CrankSetup() = default;
  CrankSetup() = delete;

  CrankSetup(const char* _id, const uint8_t ledPin, const uint8_t clk_k,
             const uint8_t dt_k)
      : BaseSetup(_id), _ledPin(ledPin), _clk_k(clk_k), _dt_k(dt_k){};

  const uint8_t _ledPin;
  const uint8_t _clk_k;
  const uint8_t _dt_k;
};

/// @brief crank setup
class CrankDriver : public Classes::BaseDriver {
 public:
  ~CrankDriver() override = default;

  CrankDriver(const CrankSetup& setup)
      : BaseDriver(setup),
        _setup(setup),
        _counter(0),
        _prevClk(0),
        _clkState(0) {
    pinMode(_setup._clk_k, INPUT);    // CLK_PIN
    pinMode(_setup._dt_k, INPUT);     // DT_PIN
    pinMode(_setup._ledPin, OUTPUT);  // LED PIN
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
  const CrankSetup _setup;
  int _counter;
  int _prevClk, _clkState;

  enum TASK { COMPLETE, NOTCOMPLETE } task;
};
};  // namespace Field

#endif