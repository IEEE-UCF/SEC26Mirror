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
  Localization(float _x = START_X, float _y = START_Y,
               float _theta = START_THETA)
      : currentPose(_x, _y, _theta) {};

  Pose2D getPose() { return currentPose; }
  void update(long leftTicks, long rightTicks, float yaw);
  void printInfo();

 private:
  Pose2D currentPose;
  long prevLeftTicks = 0;
  long prevRightTicks = 0;
};

#endif