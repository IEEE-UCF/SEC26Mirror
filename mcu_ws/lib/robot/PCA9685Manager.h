#pragma once

#include <Arduino.h>
#include <BaseDriver.h>

#include <vector>

#include "PCA9685Driver.h"

namespace Robot {

class PCA9685ManagerSetup : public Classes::BaseSetup {
 public:
  PCA9685ManagerSetup(const char* _id) : Classes::BaseSetup(_id) {}
};

class PCA9685Manager : public Classes::BaseDriver {
 public:
  explicit PCA9685Manager(const PCA9685ManagerSetup& setup);
  ~PCA9685Manager() override;

  bool init() override;
  void update() override;  // apply buffered updates
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

 private:
  std::vector<PCA9685Driver*> drivers_;
};

}  // namespace Robot
