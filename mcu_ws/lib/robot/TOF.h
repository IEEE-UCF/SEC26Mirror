/**
 * @file TOF.h
 * @author Trevor Cannon
 * @brief VL53L0X time-of-flight distance sensor driver.
 * @date 12/10/2025 (updated 2026-02-23)
 */

#ifndef VL53L0XWRAPPER_H
#define VL53L0XWRAPPER_H

#include <BaseDriver.h>
#include <VL53L0X.h>
#include <Wire.h>

#include "I2CBusLock.h"

namespace Drivers {

class TOFDriverSetup : public Classes::BaseSetup {
 public:
  const uint16_t timeout;
  const uint16_t cooldown;

  ~TOFDriverSetup() = default;
  TOFDriverSetup() = delete;

  /**
   * @param _id      Driver identifier string.
   * @param _timeout Range timeout in ms (default 500).
   * @param _cooldown Continuous-mode inter-measurement period ms (default 0).
   * @param wire     I2C bus the sensor is on (default Wire / Wire0).
   */
  TOFDriverSetup(const char* _id, uint16_t _timeout = 500,
                 uint16_t _cooldown = 0, TwoWire& wire = Wire)
      : Classes::BaseSetup(_id),
        timeout(_timeout),
        cooldown(_cooldown),
        wire_(wire) {}

  TwoWire& wire_;
};

struct TOFDriverData {
  uint16_t range = 0;
};

class TOFDriver : public Classes::BaseDriver {
 public:
  explicit TOFDriver(const TOFDriverSetup& setup)
      : BaseDriver(setup), setup_(setup) {}

  ~TOFDriver() override = default;

  bool init() override;
  void update() override;
  TOFDriverData read() { return range_; }
  const char* getInfo() override;

 private:
  const TOFDriverSetup setup_;
  VL53L0X sensor_;
  TOFDriverData range_;
};

}  // namespace Drivers

#endif
