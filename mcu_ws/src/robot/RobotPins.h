/**
 * @file RobotPins.h
 * @brief SEC26 robot pin assignments and I2C device addresses.
 * @date 2026-02-24
 *
 * All Teensy 4.1 GPIO, SPI, I2C, and peripheral pin assignments for the
 * SEC26 competition robot.  Keep this file in sync with the actual PCB wiring.
 */

#pragma once

#include <stdint.h>

// ═══════════════════════════════════════════════════════════════════════════
//  Teensy 4.1 GPIO pin assignments
// ═══════════════════════════════════════════════════════════════════════════

// ── Motor FG Encoder Inputs (GPIO 2-9) ───────────────────────────────
// FG (frequency generator) speed signals from motor encoders.
// Configured as QTimer inputs via XBAR crossbar (see QTimerEncoder.cpp).
// Pin 2: Motor 1 FG (EMC_04 -> XBAR In 6  -> QTimer1 Timer 0)
// Pin 3: Motor 2 FG (EMC_05 -> XBAR In 7  -> QTimer1 Timer 1)
// Pin 4: Motor 3 FG (EMC_06 -> XBAR In 8  -> QTimer1 Timer 2)
// Pin 5: Motor 4 FG (EMC_08 -> XBAR In 17 -> QTimer1 Timer 3)
// Pin 6: Motor 5 FG (B0_10  -> direct      -> QTimer4 Timer 1)
// Pin 7: Motor 6 FG (B1_01  -> XBAR In 15 -> QTimer2 Timer 0)
// Pin 8: Motor 7 FG (B1_00  -> XBAR In 14 -> QTimer2 Timer 1)
// Pin 9: Motor 8 FG (B0_11  -> direct      -> QTimer4 Timer 2)

// ── UWB Module (SPI0) ───────────────────────────────────────────────
constexpr uint8_t PIN_UWB_CS = 10;    // SPI0 CS0
constexpr uint8_t PIN_UWB_MOSI = 11;  // SPI0 MOSI
constexpr uint8_t PIN_UWB_MISO = 12;  // SPI0 MISO
constexpr uint8_t PIN_UWB_CLK = 13;   // SPI0 SCK

// ── I2C Bus: Wire1 — BNO085 IMU ──────────────────────────────────────
constexpr uint8_t PIN_GYRO_SCL = 16;  // Wire1 SCL
constexpr uint8_t PIN_GYRO_SDA = 17;  // Wire1 SDA
constexpr uint8_t PIN_GYRO_RST = 40;  // BNO085 reset
constexpr uint8_t PIN_GYRO_INT = 41;  // BNO085 interrupt

// ── I2C Bus: Wire0 — Sensors / Mux / GPIO expander ───────────────────
constexpr uint8_t PIN_SENSORS_SDA = 18;  // Wire0 SDA (default)
constexpr uint8_t PIN_SENSORS_SCL = 19;  // Wire0 SCL (default)

// ── I2C Bus: Wire2 — PCA9685 motor + servo boards ────────────────────
constexpr uint8_t PIN_SERVO_MOTOR_SCL = 24;  // Wire2 SCL
constexpr uint8_t PIN_SERVO_MOTOR_SDA = 25;  // Wire2 SDA

// ── I2C Mux Reset ───────────────────────────────────────────────────
constexpr uint8_t PIN_MUX_RESET = 23;  // TCA9548A active-LOW reset

// ── Display (SSD1306 software SPI) ──────────────────────────────────
constexpr uint8_t PIN_DISP_MOSI = 26;
constexpr uint8_t PIN_DISP_CLK = 27;
constexpr uint8_t PIN_DISP_CS = 38;
constexpr uint8_t PIN_DISP_RST = 33;  // Display reset
constexpr uint8_t PIN_DISP_DC = 37;   // Data / Command (CD)

// ── PCA9685 Output Enable ────────────────────────────────────────────
constexpr uint8_t PIN_SERVO_OE = 28;  // PCA9685 #0 OE (active LOW)
constexpr uint8_t PIN_MOTOR_OE = 29;  // PCA9685 #1 OE (active LOW)

// ── Misc GPIO ───────────────────────────────────────────────────────
constexpr uint8_t PIN_RC_RX = 34;     // FlySky IBUS receiver (Serial8 RX)
constexpr uint8_t PIN_RGB_LEDS = 35;  // WS2812B data line
constexpr uint8_t PIN_BUTTON_INTERRUPT = 36;  // Standalone reset button

// ── Not Connected ───────────────────────────────────────────────────
// Pin 14: NC
// Pin 15: NC
// Pin 20: NC
// Pin 21: NC
// Pin 22: NC
// Pin 33: Display RST (PIN_DISP_RST)
// Pin 39: NC

// ═══════════════════════════════════════════════════════════════════════════
//  I2C device addresses
// ═══════════════════════════════════════════════════════════════════════════

constexpr uint8_t I2C_ADDR_MUX = 0x70;   // TCA9548A mux (Wire0)
constexpr uint8_t I2C_ADDR_GPIO = 0x20;  // TCA9555 GPIO expander (Wire0)
constexpr uint8_t I2C_ADDR_POWER =
    0x40;  // INA219 power sensor (Wire0, mux ch0)
constexpr uint8_t I2C_ADDR_SERVO = 0x40;  // PCA9685 #0 — servos (Wire2)
constexpr uint8_t I2C_ADDR_MOTOR = 0x41;  // PCA9685 #1 — motors (Wire2)
constexpr uint8_t I2C_ADDR_IMU = 0x4B;    // BNO085 (Wire1)

// ═══════════════════════════════════════════════════════════════════════════
//  TCA9548A mux channel assignments
// ═══════════════════════════════════════════════════════════════════════════

constexpr uint8_t MUX_CH_BATTERY = 0;  // INA219 power sensor
// Channels 1-7 available for future use

// ═══════════════════════════════════════════════════════════════════════════
//  TCA9555 GPIO expander pin mapping
// ═══════════════════════════════════════════════════════════════════════════

// Port 0 (pins 0-7):   DIP switches (active HIGH = ON)
// Port 1 (pins 8-15):  Push buttons (active HIGH = pressed)
//   Datasheet labels Port 1 pins as P10-P17.

// ═══════════════════════════════════════════════════════════════════════════
//  PCA9685 channel mapping
// ═══════════════════════════════════════════════════════════════════════════

// PCA9685 #0 (I2C_ADDR_SERVO) — Servos
//   Channel 0: Servo 1
//   Channel 1: Servo 2
//   Channel 2: Servo 3
//   Channel 3: Servo 4
//   Channel 4: Servo 5
//   Channel 5: Servo 6
//   Channel 6: Servo 7
//   Channel 7: Servo 8

// PCA9685 #1 (I2C_ADDR_MOTOR) — Motors (PWM + DIR pairs)
//   Channel 0,1:   Motor 1 (PWM, DIR)
//   Channel 2,3:   Motor 2 (PWM, DIR)
//   Channel 4,5:   Motor 3 (PWM, DIR)
//   Channel 6,7:   Motor 4 (PWM, DIR)
//   Channel 8,9:   Motor 5 (PWM, DIR)
//   Channel 10,11: Motor 6 (PWM, DIR)
//   Channel 12,13: Motor 7 (PWM, DIR)
//   Channel 14,15: Motor 8 (PWM, DIR)

// ═══════════════════════════════════════════════════════════════════════════
//  Peripheral configuration constants
// ═══════════════════════════════════════════════════════════════════════════

constexpr uint8_t NUM_SERVOS = 8;
constexpr uint8_t NUM_MOTORS = 8;
constexpr uint8_t NUM_RGB_LEDS = 5;

constexpr float SHUNT_RESISTANCE_OHM = 0.005f;  // INA219 shunt resistor
