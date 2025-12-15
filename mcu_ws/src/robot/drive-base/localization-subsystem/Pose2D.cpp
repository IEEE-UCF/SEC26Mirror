#include "Pose2D.h"

void Pose2D::normalizeTheta() { theta = std::remainder(theta, 2 * PI); }

void Pose2D::add(const Pose2D& pose) {
  x += pose.x;
  y += pose.y;
  theta += pose.theta;
}

void Pose2D::rotate(float angle) {
  theta += angle;
  normalizeTheta();
}
