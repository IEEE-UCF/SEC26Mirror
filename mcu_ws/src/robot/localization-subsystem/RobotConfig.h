/**
 * @file RobotConfig.h
 * @author Trevor Cannon
 * @brief Robot constants for calculations
 * @date 12/11/2025
 */

#ifndef ROBOTCONFIG_H
#define ROBOTCONFIG_H

#include <cmath>

namespace RobotConfig {

// need to actually get these values

constexpr float TRACK_WIDTH;

constexpr int RAW_TICKS_PER_REVOLUTION;
constexpr int GEAR_RATIO;
constexpr long TICKS_PER_REVOLUTION = RAW_TICKS_PER_REVOLUTION * GEAR_RATIO;
constexpr float IN_PER_TICK = WHEEL_CIRCUMFERENCE / TICKS_PER_REVOLUTION;

constexpr float WHEEL_DIAMETER;
constexpr float WHEEL_RADIUS;
constexpr float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;

constexpr float START_X = 0.0f;
constexpr float START_Y = 0.0f;
constexpr float START_THETA = 0.0f;

}  // namespace RobotConfig

#endif