#ifndef VL53L0XWRAPPER_H
#define VL53L0XWRAPPER_H

#include <BaseDriver.h>
#include <VL53L0X.h>

#include <string>

namespace Drivers {
struct TOFDriverSetup : public Classes::BaseSetup {
  uint16_t timeout = 500;
  uint16_t cooldown = 0;
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
  std::string getInfo() override;

 private:
  TOFDriverSetup setup_;
  VL53L0X sensor_;
  TOFDriverData range_;
};
};  // namespace Drivers

#endif