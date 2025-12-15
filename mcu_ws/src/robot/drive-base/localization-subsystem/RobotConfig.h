/**
 * @file RobotConfig.h
 * @author Trevor Cannon
 * @brief Robot constants for calculations
 * @date 12/11/2025
 */

#ifndef ROBOTCONFIG_H
#define ROBOTCONFIG_H

namespace RobotConfig {

// THESE ARE NOT ACCURATE CHANGE LATER!!!!!!!!!!!!!!!!!!

constexpr float TRACK_WIDTH = 10.0f;

constexpr float WHEEL_DIAMETER = 3.25;
constexpr float WHEEL_RADIUS = WHEEL_DIAMETER * 0.5f;
constexpr float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;

constexpr int RAW_TICKS_PER_REVOLUTION = 3;
constexpr int GEAR_RATIO = 34;
constexpr long TICKS_PER_REVOLUTION = RAW_TICKS_PER_REVOLUTION * GEAR_RATIO;
constexpr float IN_PER_TICK = WHEEL_CIRCUMFERENCE / TICKS_PER_REVOLUTION;

constexpr float START_X = 0.0f;
constexpr float START_Y = 0.0f;
constexpr float START_THETA = 0.0f;

}  // namespace RobotConfig

#endif