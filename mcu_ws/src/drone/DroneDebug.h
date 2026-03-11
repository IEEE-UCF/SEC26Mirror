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
// Only print when USB CDC is connected — otherwise Serial.print() blocks
// indefinitely on ESP32-S3, starving micro-ROS and flight tasks.
#define DRONE_PRINT(...) do { if (Serial) Serial.print(__VA_ARGS__); } while(0)
#define DRONE_PRINTLN(...) do { if (Serial) Serial.println(__VA_ARGS__); } while(0)
#define DRONE_PRINTF(fmt, ...) do { if (Serial) Serial.printf(fmt, ##__VA_ARGS__); } while(0)
#else
#define DRONE_PRINT(...) ((void)0)
#define DRONE_PRINTLN(...) ((void)0)
#define DRONE_PRINTF(fmt, ...) ((void)0)
#endif
