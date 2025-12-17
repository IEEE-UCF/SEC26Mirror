#include "Vector2D.h"

void Vector2D::add(const Vector2D& other) {
  x += other.x;
  y += other.y;
  theta += other.theta;
}

void Vector2D::subtract(const Vector2D& other) {
  x -= other.x;
  y -= other.y;
  theta -= other.theta;
}

void Vector2D::rotate(float angle) {
  const float cosA = cosf(angle);
  const float sinA = sin(angle);

  const float new_x = x * cosA - y * sinA;
  const float new_y = y * sinA + y * cosA;

  x = new_x;
  y = new_y;
}

void Vector2D::scalarMult(float scalar) {
  x *= scalar;
  y *= scalar;
  theta *= scalar;
}

void Vector2D::elementMult(const Vector2D& other) {
  x *= other.x;
  y *= other.y;
  theta *= other.theta;
}

void Vector2D::constrainTheta(float constraint) {
  theta = constrain(theta, -constraint, constraint);
}