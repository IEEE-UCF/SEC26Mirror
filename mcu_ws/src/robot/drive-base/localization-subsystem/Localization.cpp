#include "Localization.h"

void Localization::update(long leftTicks, long rightTicks, float yaw) {
  const long deltaLeftTicks = leftTicks - prevLeftTicks;
  const long deltaRightTicks = rightTicks - prevRightTicks;

  prevLeftTicks = leftTicks;
  prevRightTicks = rightTicks;

  const float leftDist = deltaLeftTicks * IN_PER_TICK;
  const float rightDist = deltaRightTicks * IN_PER_TICK;

  const float distTravel = (leftDist + rightDist) * 0.5f;

  const float deltaX = distTravel * cosf(yaw);
  const float deltaY = distTravel * sinf(yaw);

  currentPose.theta = yaw;
  currentPose.normalizeTheta();
  currentPose.x += deltaX;
  currentPose.y += deltaY;
}

void Localization::printInfo() {
  Serial.print("X: ");
  Serial.print(currentPose.x);
  Serial.print("Y: ");
  Serial.print(currentPose.y);
  Serial.print("Theta: ");
  Serial.print(currentPose.theta);
}
