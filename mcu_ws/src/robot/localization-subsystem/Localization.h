/**
 * @file Localization.h
 * @author Trevor Cannon
 * @brief Track robot's position using gyro and encoder readings
 * @date 12/11/2025
 */

#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <Arduino.h>

#include "Pose2D.h"
#include "RobotConfig.h"
using namespace RobotConfig;  // maybe change this im lazy

class Localization {
 public:
  ~Localization() = default;
  Localization() : currentPose(START_X, START_Y, START_THETA) {};

  Pose2D getPose() { return currentPose; }
  void update(long leftTicks, long rightTicks, float yaw);
  void getInfo();

 private:
  Pose2D currentPose;
};

#endif