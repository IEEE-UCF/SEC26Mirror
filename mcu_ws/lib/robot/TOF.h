/**
 * @file TOF.h
 * @author Trevor Cannon
 * @brief Defines TOF driver wrapper for
 * VL53L0X TOF sensor
 * @date 12/10/2025
 */

#ifndef VL53L0XWRAPPER_H
#define VL53L0XWRAPPER_H

#include <BaseDriver.h>
#include <VL53L0X.h>

#include <string>

namespace Drivers {

class TOFDriverSetup : public Classes::BaseSetup {
 public:
  const uint16_t timeout;
  const uint16_t cooldown;

  ~TOFDriverSetup() = default;
  TOFDriverSetup() = delete;

  TOFDriverSetup(const char* _id, uint16_t _timeout = 500,
                 uint16_t _cooldown = 0)
      : Classes::BaseSetup(_id), timeout(_timeout), cooldown(_cooldown) {};

 private:
};

struct TOFDriverData {
  uint16_t range;
};

class TOFDriver : public Classes::BaseDriver {
 public:
  TOFDriver(const TOFDriverSetup& setup) : BaseDriver(setup), setup_(setup) {};

  ~TOFDriver() override = default;

  /// @brief  Initialize driver
  /// @return Success
  bool init() override;

  /// @brief Update driver
  void update() override;

  /// @brief  Get data
  /// @return data
  TOFDriverData read() { return range_; };

  /// @brief Get info in the form of a data string
  /// @return data string
  const char* getInfo() override;

 private:
  const TOFDriverSetup setup_;
  VL53L0X sensor_;
  TOFDriverData range_;
};
};  // namespace Drivers

#endif