#include "PCA9685Manager.h"

using namespace Robot;

PCA9685Manager::PCA9685Manager(const PCA9685ManagerSetup& setup)
    : Classes::BaseDriver(setup) {}

PCA9685Manager::~PCA9685Manager() {
  for (auto d : drivers_) delete d;
  drivers_.clear();
}

bool PCA9685Manager::init() {
  // initialize each contained driver
  bool ok = true;
  for (auto d : drivers_) {
    ok = d->init() && ok;
  }
  initSuccess_ = ok;
  return ok;
}

void PCA9685Manager::update() {
  for (auto d : drivers_) {
    d->applyBuffered();
  }
}

PCA9685Driver* PCA9685Manager::createDriver(const PCA9685DriverSetup& setup) {
  PCA9685Driver* d = new PCA9685Driver(setup);
  drivers_.push_back(d);
  return d;
}
