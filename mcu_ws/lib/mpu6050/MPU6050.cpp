#include "MPU6050.h"

namespace Drivers {
bool MPU6050Driver::init() {
  initSuccess_ = true;
  if (!mpu_.begin()) {
    initSuccess_ = false;
    return initSuccess_;
  }

  mpu_.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu_.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu_.setFilterBandwidth(MPU6050_BAND_21_HZ);

  return initSuccess_;
}

void MPU6050Driver::update() {

  if (!mpu_.getEvent(&accel_, &gyro_, &temp_))
    return;

  data_.a_x = accel_.acceleration.x;
  data_.a_y = accel_.acceleration.y;
  data_.a_z = accel_.acceleration.z;

  data_.g_x = gyro_.gyro.x;
  data_.g_y = gyro_.gyro.y;
  data_.g_z = gyro_.gyro.z;

  data_.temp = temp_.temperature;
}

const char *MPU6050Driver::getInfo() {
  snprintf(infoBuffer_, sizeof(infoBuffer_), "MPU6050: %s", setup_.getId());
  return infoBuffer_;
}

} // namespace Drivers