#ifndef POSE2D_H
#define POSE2D_H

#include <Arduino.h>

#include <cmath>

class Pose2D {
 public:
  float x;
  float y;
  float theta;

  ~Pose2D() = default;
  Pose2D(float _x = 0, float _y = 0, float _theta = 0)
      : x(_x), y(_y), theta(_theta) {};

  void normalize();
  void add();
  void rotate();

 private:
};

#endif