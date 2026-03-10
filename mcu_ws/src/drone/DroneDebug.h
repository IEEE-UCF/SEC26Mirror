#pragma once
/**
 * @file DroneDebug.h
 * @brief Conditional serial debug macros for drone firmware.
 *
 * Define DRONE_SERIAL_DEBUG in build flags to enable serial output.
 * Production builds omit the flag for zero serial overhead during flight.
 */

#ifdef DRONE_SERIAL_DEBUG
#include <Arduino.h>
#define DRONE_PRINT(...) Serial.print(__VA_ARGS__)
#define DRONE_PRINTLN(...) Serial.println(__VA_ARGS__)
#define DRONE_PRINTF(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#else
#define DRONE_PRINT(...) ((void)0)
#define DRONE_PRINTLN(...) ((void)0)
#define DRONE_PRINTF(fmt, ...) ((void)0)
#endif
