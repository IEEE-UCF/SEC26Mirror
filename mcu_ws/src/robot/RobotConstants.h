/**
 * @file RobotConstants.h
 * @brief Robot hardware constants used across drivers
 * @date 12/22/2025
 */

#ifndef ROBOTCONSTANTS_H
#define ROBOTCONSTANTS_H

// PCA9685 PWM Driver constants
#define PCA_CHANNELS 16
#define DEFAULT_PCA9685_ADDR 0x40
#define DEFAULT_PCA9685_FREQ 50  // Servo PWM frequency (Hz)
#define MOTOR_PCA9685_FREQ 1000  // Motor PWM frequency (Hz) — max 1526

// Battery constants (3S LiPo)
constexpr float BATTERY_VOLTAGE_FULL = 12.6f;  // 4.2V × 3 cells
constexpr float BATTERY_VOLTAGE_EMPTY = 9.9f;  // 3.3V × 3 cells (safe cutoff)

#endif
