#include "Pose2D.h"

void Pose2D::normalizeTheta() {
  theta = std::remainder(theta, 2 * PI);
  return;
}

void Pose2D::add(const Pose2D& pose) {
  x += pose.x;
  y += pose.y;
  theta += pose.theta;
  return;
}

void Pose2D::rotate(float angle) {
  const float cosA = cosf(angle);
  const float sinA = sin(angle);

  const float new_x = x * cosA - y * sinA;
  const float new_y = y * sinA + y * cosA;

  x = new_x;
  y = new_y;

  return;
}
