/**
 * @file Pose2D.h
 * @author Trevor Cannon
 * @brief Pose2D object to represent objects in 2D space
 * @date 12/11/2025
 */

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

  void normalizeTheta();
  void add(const Pose2D& pose);
  void rotate(float angle);

 private:
};

#endif