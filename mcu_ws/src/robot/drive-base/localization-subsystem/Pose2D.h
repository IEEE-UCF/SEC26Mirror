/**
 * @file Pose2D.h
 * @author Trevor Cannon
 * @brief Pose2D object to represent objects in 2D space
 * @date 12/14/2025
 */

#ifndef POSE2D_H
#define POSE2D_H

#define PI 3.1415926535897932384626433832795

#include <cmath>

class Pose2D {
 public:
  float x;
  float y;
  float theta;

  ~Pose2D() = default;
  Pose2D(float _x = 0.0f, float _y = 0.0f, float _theta = 0.0f)
      : x(_x), y(_y), theta(_theta) {};

  float getX() const { return x; };
  float getY() const { return y; };
  float getTheta() const { return theta; };

  void normalizeTheta();
  void add(const Pose2D& pose);
  void rotate(float angle);

 private:
};

#endif