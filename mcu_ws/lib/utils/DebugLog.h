/**
 * @file DebugLog.h
 * @brief Debug logging macros routed to SerialUSB1 (Teensy dual-serial USB).
 *
 * When SERIAL_DEBUG is defined (via build flag), all DEBUG_* macros print to
 * SerialUSB1 — the second CDC virtual COM port exposed by USB_DUAL_SERIAL.
 * The primary Serial port remains dedicated to micro-ROS transport.
 *
 * When SERIAL_DEBUG is not defined, all macros compile to no-ops with zero
 * overhead.
 *
 * Usage:
 *   DEBUG_BEGIN();                          // call once in setup()
 *   DEBUG_PRINTLN("[IMU] init OK");
 *   DEBUG_PRINTF("[BAT] voltage=%.2fV\n", v);
 */
#pragma once

#ifdef SERIAL_DEBUG

#include <Arduino.h>

// USB CDC ignores baud rate — the parameter is only for API compatibility.
#define DEBUG_BEGIN()         do { SerialUSB1.begin(115200); } while (0)
#define DEBUG_PRINT(...)      SerialUSB1.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...)    SerialUSB1.println(__VA_ARGS__)
#define DEBUG_PRINTF(fmt, ...) SerialUSB1.printf(fmt, ##__VA_ARGS__)
#define DEBUG_FLUSH()         SerialUSB1.flush()

#else

#define DEBUG_BEGIN()          ((void)0)
#define DEBUG_PRINT(...)       ((void)0)
#define DEBUG_PRINTLN(...)     ((void)0)
#define DEBUG_PRINTF(fmt, ...) ((void)0)
#define DEBUG_FLUSH()          ((void)0)

#endif
