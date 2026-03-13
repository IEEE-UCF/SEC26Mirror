#pragma once
/**
 * @file DronePins.h
 * @brief GPIO pin assignments for the drone ESP32-S3 (Seeed XIAO).
 */

namespace Drone {
namespace Pins {

// Motor PWM outputs (ESP32-S3 LEDC)
// Motor 1-4 → D0-D3 in order
constexpr uint8_t MOTOR_FR = D0;  // Motor 1: Front-Right (CW)
constexpr uint8_t MOTOR_BR = D1;  // Motor 2: Back-Right (CCW)
constexpr uint8_t MOTOR_FL = D2;  // Motor 3: Front-Left (CCW)
constexpr uint8_t MOTOR_BL = D3;  // Motor 4: Back-Left (CW)

// I2C bus (BNO085 + VL53L0X share this bus)
constexpr uint8_t I2C_SDA = D4;
constexpr uint8_t I2C_SCL = D5;

// IR LED transmitter
constexpr uint8_t IR_LED = D6;

// BNO085 IMU
constexpr int8_t IMU_RST = D7;
constexpr int8_t IMU_INT = 22;  // GPIO22, not a Dx alias

// UWB DW3000 SPI (uses default SPI bus)
constexpr uint8_t UWB_CS = 21;  // GPIO21

// UWB device ID for drone tag
constexpr uint8_t UWB_DEVICE_ID = 15;

}  // namespace Pins
}  // namespace Drone
