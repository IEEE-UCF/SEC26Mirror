#include "BNO085.h"

namespace Drivers {

bool BNO085Driver::init() {
  initSuccess_ = true;

  if (!imu_.begin_I2C()) {
    initSuccess_ = false;
    return initSuccess_;
  }
  if (!imu_.enableReport(SH2_ACCELEROMETER) ||
      !imu_.enableReport(SH2_GYROSCOPE_CALIBRATED) ||
      !imu_.enableReport(SH2_ROTATION_VECTOR)) {
    initSuccess_ = false;
  }
  return initSuccess_;
}

void BNO085Driver::update() {
  while (imu_.getSensorEvent(&sensorValue_)) {
    switch (sensorValue_.sensorId) {
      case SH2_ACCELEROMETER:
        data_.accel_x = sensorValue_.un.accelerometer.x;
        data_.accel_y = sensorValue_.un.accelerometer.y;
        data_.accel_z = sensorValue_.un.accelerometer.z;
        break;

      case SH2_GYROSCOPE_CALIBRATED:
        data_.gyro_x = sensorValue_.un.gyroscope.x;
        data_.gyro_y = sensorValue_.un.gyroscope.y;
        data_.gyro_z = sensorValue_.un.gyroscope.z;
        break;

      case SH2_ROTATION_VECTOR:
        data_.qx = sensorValue_.un.rotationVector.i;
        data_.qy = sensorValue_.un.rotationVector.j;
        data_.qz = sensorValue_.un.rotationVector.k;
        data_.qw = sensorValue_.un.rotationVector.real;

        data_.yaw = calculateYaw(data_.qx, data_.qy, data_.qz, data_.qw);
        data_.pitch = calculatePitch(data_.qx, data_.qy,data_.qz,data_.qw);
        data_.roll = calculateRoll(data_.qx, data_.qy,data_.qz,data_.qw);
        break;

      default:
        break;
    }
  }
}
//--------------------------Old Code--------------------------//
// yaw in radians
// float BNO085Driver::calculateYaw(float qx, float qy, float qz, float qw) {
//   float t3 = 2.0f * (qw * qz + qx * qy);
//   float t4 = 1.0f - 2.0f * (qy * qy + qz * qz);
//   return atan2(t3, t4);//handles for all scenarios
// }
// float BNO085Driver::calculateRoll(float qx, float qy, float qz, float qw){

// }
// float BNO085Driver::calculatePitch(float qx,float qy,float qz,float qw){

// }
//--------------------------Old Code--------------------------//


//Code straight from the Adafruit BNO08x examples....
//put simply Quanterions is a 4d unit, we need to convert it to a 3D matrix to get anything useful out of it
//in doing so we also get the rotational angles needed to determine where the object(robot/drone) needs to be....0
float BNO085Driver::calculateYaw(float qx, float qy, float qz, float real){
  return atan2(2.0*(qx*qy+qz*real), (sq(qx)-sq(qy)-sq(qz)+sq(real)));
}
float BNO085Driver::calculatePitch(float qx, float qy, float qz, float real){
  return asin(-2.0*(qx*qz-qx*real)/(sq(qx)+sq(qy)+sq(qz)+sq(real)));
}
float BNO085Driver::calculateRoll(float qx, float qy, float qz, float real){
  return atan2(2.0*(qy*qz+qx*real), (-sq(qx)-sq(qy)+sq(qz)+sq(real)));
}
float BNO085Driver::radians_to_degree(float angle){
  return angle*=RAD_TO_DEG;
}
const char* BNO085Driver::getInfo() {
  snprintf(infoBuffer_, sizeof(infoBuffer_), "BNO085: %s", setup_.getId());
  return infoBuffer_;
}

};  // namespace Drivers
