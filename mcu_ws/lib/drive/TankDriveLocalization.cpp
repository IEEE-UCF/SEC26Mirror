/**
 * @file TankDriveLocalization.cpp
 * @author Trevor Cannon
 * @date 12/14/2025
 * @refactored 12/21/2025 - Aldem Pido - Added setup object pattern
 * @refactored 12/22/2025 - Renamed to TankDriveLocalization
 */

#include "TankDriveLocalization.h"

namespace Drive {

void TankDriveLocalization::update(long leftTicks, long rightTicks, float yaw) {
  last_raw_yaw_ = yaw;
  const float corrected_yaw = yaw - yaw_offset_;

  const long deltaLeftTicks = leftTicks - prevLeftTicks;
  const long deltaRightTicks = rightTicks - prevRightTicks;

  prevLeftTicks = leftTicks;
  prevRightTicks = rightTicks;

  const float leftDist = deltaLeftTicks * setup_.dist_per_tick;
  const float rightDist = deltaRightTicks * setup_.dist_per_tick;

  const float distTravel = (leftDist + rightDist) * 0.5f;

  const float deltaX = distTravel * cosf(corrected_yaw);
  const float deltaY = distTravel * sinf(corrected_yaw);

  currentPose.theta = corrected_yaw;
  currentPose.normalizeTheta();
  currentPose.x += deltaX;
  currentPose.y += deltaY;
}

void TankDriveLocalization::setPose(float x, float y, float theta) {
  yaw_offset_ = last_raw_yaw_ - theta;
  currentPose.x = x;
  currentPose.y = y;
  currentPose.theta = theta;
  currentPose.normalizeTheta();
}

const char* TankDriveLocalization::getInfo() {
  snprintf(infoBuffer_, sizeof(infoBuffer_), "X: %.2f Y: %.2f Theta: %.2f",
           currentPose.x, currentPose.y, currentPose.theta);
  return infoBuffer_;
}

void TankDriveLocalization::reset() {
  yaw_offset_ = last_raw_yaw_ - setup_.start_theta;
  currentPose.x = setup_.start_x;
  currentPose.y = setup_.start_y;
  currentPose.theta = setup_.start_theta;
  prevLeftTicks = 0;
  prevRightTicks = 0;
}

}  // namespace Drive
