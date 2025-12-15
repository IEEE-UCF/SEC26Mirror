/**
 * @file Vector2D.h
 * @author Trevor Cannon
 * @brief Vector2D object for 2D velocity + acceleration
 * @date 12/14/2025
 */

#ifndef VECTOR2D_H
#define VECTOR2D_H

#include <Arduino.h>

#include <cmath>

class Vector2D {
 public:
  float x;
  float y;
  float theta;

  ~Vector2D() = default;
  Vector2D(float _x = 0.0f, float _y = 0.0f, float _theta = 0.0f)
      : x(_x), y(_y), theta(_theta) {};

  float getX() const { return x; };
  float getY() const { return y; };
  float getTheta() const { return theta; };

  void add(const Vector2D& other);
  void subtract(const Vector2D& other);
  void rotate(float angle);
  void scalarMult(float scalar);
  void elementMult(const Vector2D& other);
  void constrainTheta(float constraint);
  float magnitude() const { return hypot(x, y); };

 private:
};

#endif