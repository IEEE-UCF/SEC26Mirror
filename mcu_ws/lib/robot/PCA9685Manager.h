#pragma once

#include <Arduino.h>
#include <BaseDriver.h>

#include <vector>

#include "PCA9685Driver.h"

class I2CDMABus;

namespace Robot {

class PCA9685ManagerSetup : public Classes::BaseSetup {
 public:
  PCA9685ManagerSetup(const char* _id) : Classes::BaseSetup(_id) {}
};

class PCA9685Manager : public Classes::BaseDriver {
 public:
  static constexpr uint16_t MAX_BUF_PER_DRIVER = 67;

  explicit PCA9685Manager(const PCA9685ManagerSetup& setup);
  ~PCA9685Manager() override;

  bool init() override;
  void update() override;
  const char* getInfo() override {
    static const char info[] = "PCA9685Manager";
    return info;
  }

  // Manager API
  PCA9685Driver* createDriver(const PCA9685DriverSetup& setup);
  size_t count() const { return drivers_.size(); }
  PCA9685Driver* get(size_t i) {
    return i < drivers_.size() ? drivers_[i] : nullptr;
  }

  // Shared DMA bus support
  void setDMABus(I2CDMABus* bus) { dma_bus_ = bus; }
  uint16_t buildInto();

 private:
  std::vector<PCA9685Driver*> drivers_;
  I2CDMABus* dma_bus_ = nullptr;

  uint16_t buildForDriver(PCA9685Driver* drv, uint16_t* out);
};

}  // namespace Robot
