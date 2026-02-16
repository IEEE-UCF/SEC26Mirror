/**
 * @file DriveBaseHALConfig.example.h
 * @brief Example configuration for Drive Base using HAL
 * @author Claude Code
 * @date 12/24/2025
 *
 * This file demonstrates how to configure the drive base motors using the HAL
 * system, allowing motors to be controlled via:
 * - Direct MCU pins
 * - PCA9685 PWM channels
 * - MCP23017 I/O expander pins
 * - Or any combination of the above
 */

#pragma once

#include <hal/CD74HC4067GPIO.h>
#include <hal/MCP23017GPIO.h>
#include <hal/NativeGPIO.h>
#include <hal/PCA9685GPIO.h>

#include "DriveSubsystem.h"
#include "EncoderDriver.h"
#include "MotorDriver.h"
#include "CD74HC4067Driver.h"
#include "MCP23017Driver.h"
#include "robot/PCA9685Driver.h"

namespace DriveConfig {

// =============================================================================
// Example 1: All Native GPIO (traditional setup)
// =============================================================================

namespace Example1_AllNative {

// Create HAL pins for native GPIO
static HAL::NativeGPIO left_motor1_pwm(3);
static HAL::NativeGPIO left_motor1_dir(4);
static HAL::NativeGPIO left_motor2_pwm(5);
static HAL::NativeGPIO left_motor2_dir(6);

static HAL::NativeGPIO right_motor1_pwm(7);
static HAL::NativeGPIO right_motor1_dir(8);
static HAL::NativeGPIO right_motor2_pwm(9);
static HAL::NativeGPIO right_motor2_dir(10);

// Motor driver setups using HAL pins
static Drivers::MotorDriverSetup left_motor1_setup("left_motor_1",
                                                   &left_motor1_pwm,
                                                   &left_motor1_dir);
static Drivers::MotorDriverSetup left_motor2_setup("left_motor_2",
                                                   &left_motor2_pwm,
                                                   &left_motor2_dir);
static Drivers::MotorDriverSetup right_motor1_setup("right_motor_1",
                                                    &right_motor1_pwm,
                                                    &right_motor1_dir);
static Drivers::MotorDriverSetup right_motor2_setup("right_motor_2",
                                                    &right_motor2_pwm,
                                                    &right_motor2_dir);

}  // namespace Example1_AllNative

// =============================================================================
// Example 2: PCA9685 for PWM, Native GPIO for Direction
// =============================================================================

namespace Example2_PCA9685_PWM {

// PCA9685 driver (shared for all PWM channels)
static Robot::PCA9685DriverSetup pca_setup("pca0", 0x40, 1000);  // 1kHz
static Robot::PCA9685Driver pca_driver(pca_setup);

// PWM pins on PCA9685
static HAL::PCA9685GPIO left_motor1_pwm(&pca_driver, 0);
static HAL::PCA9685GPIO left_motor2_pwm(&pca_driver, 1);
static HAL::PCA9685GPIO right_motor1_pwm(&pca_driver, 2);
static HAL::PCA9685GPIO right_motor2_pwm(&pca_driver, 3);

// Direction pins on native GPIO
static HAL::NativeGPIO left_motor1_dir(4);
static HAL::NativeGPIO left_motor2_dir(5);
static HAL::NativeGPIO right_motor1_dir(6);
static HAL::NativeGPIO right_motor2_dir(7);

// Motor driver setups
static Drivers::MotorDriverSetup left_motor1_setup("left_motor_1",
                                                   &left_motor1_pwm,
                                                   &left_motor1_dir);
static Drivers::MotorDriverSetup left_motor2_setup("left_motor_2",
                                                   &left_motor2_pwm,
                                                   &left_motor2_dir);
static Drivers::MotorDriverSetup right_motor1_setup("right_motor_1",
                                                    &right_motor1_pwm,
                                                    &right_motor1_dir);
static Drivers::MotorDriverSetup right_motor2_setup("right_motor_2",
                                                    &right_motor2_pwm,
                                                    &right_motor2_dir);

// Don't forget to initialize the PCA9685 driver in your setup:
// pca_driver.init();

}  // namespace Example2_PCA9685_PWM

// =============================================================================
// Example 3: MCP23017 for Direction, Native for PWM
// =============================================================================

namespace Example3_MCP23017_Direction {

// MCP23017 driver for direction pins
static Drivers::MCP23017DriverSetup mcp_setup("mcp0", 0x20);
static Drivers::MCP23017Driver mcp_driver(mcp_setup);

// PWM pins on native GPIO
static HAL::NativeGPIO left_motor1_pwm(9);
static HAL::NativeGPIO left_motor2_pwm(10);
static HAL::NativeGPIO right_motor1_pwm(11);
static HAL::NativeGPIO right_motor2_pwm(12);

// Direction pins on MCP23017
static HAL::MCP23017GPIO left_motor1_dir(&mcp_driver, 0);
static HAL::MCP23017GPIO left_motor2_dir(&mcp_driver, 1);
static HAL::MCP23017GPIO right_motor1_dir(&mcp_driver, 2);
static HAL::MCP23017GPIO right_motor2_dir(&mcp_driver, 3);

// Motor driver setups
static Drivers::MotorDriverSetup left_motor1_setup("left_motor_1",
                                                   &left_motor1_pwm,
                                                   &left_motor1_dir);
static Drivers::MotorDriverSetup left_motor2_setup("left_motor_2",
                                                   &left_motor2_pwm,
                                                   &left_motor2_dir);
static Drivers::MotorDriverSetup right_motor1_setup("right_motor_1",
                                                    &right_motor1_pwm,
                                                    &right_motor1_dir);
static Drivers::MotorDriverSetup right_motor2_setup("right_motor_2",
                                                    &right_motor2_pwm,
                                                    &right_motor2_dir);

// Don't forget to initialize the MCP23017 driver in your setup:
// mcp_driver.init();

}  // namespace Example3_MCP23017_Direction

// =============================================================================
// Example 4: Mixed Hardware (PCA9685 + MCP23017)
// =============================================================================

namespace Example4_Mixed {

// PCA9685 for PWM
static Robot::PCA9685DriverSetup pca_setup("pca0", 0x40, 1000);
static Robot::PCA9685Driver pca_driver(pca_setup);

// MCP23017 for direction pins
static Drivers::MCP23017DriverSetup mcp_setup("mcp0", 0x20);
static Drivers::MCP23017Driver mcp_driver(mcp_setup);

// Left motors: PWM on PCA9685, direction on MCP23017
static HAL::PCA9685GPIO left_motor1_pwm(&pca_driver, 0);
static HAL::MCP23017GPIO left_motor1_dir(&mcp_driver, 0);
static HAL::PCA9685GPIO left_motor2_pwm(&pca_driver, 1);
static HAL::MCP23017GPIO left_motor2_dir(&mcp_driver, 1);

// Right motors: PWM on PCA9685, direction on MCP23017
static HAL::PCA9685GPIO right_motor1_pwm(&pca_driver, 2);
static HAL::MCP23017GPIO right_motor1_dir(&mcp_driver, 2);
static HAL::PCA9685GPIO right_motor2_pwm(&pca_driver, 3);
static HAL::MCP23017GPIO right_motor2_dir(&mcp_driver, 3);

// Motor driver setups
static Drivers::MotorDriverSetup left_motor1_setup("left_motor_1",
                                                   &left_motor1_pwm,
                                                   &left_motor1_dir);
static Drivers::MotorDriverSetup left_motor2_setup("left_motor_2",
                                                   &left_motor2_pwm,
                                                   &left_motor2_dir);
static Drivers::MotorDriverSetup right_motor1_setup("right_motor_1",
                                                    &right_motor1_pwm,
                                                    &right_motor1_dir);
static Drivers::MotorDriverSetup right_motor2_setup("right_motor_2",
                                                    &right_motor2_pwm,
                                                    &right_motor2_dir);

// Initialize both drivers in your setup:
// pca_driver.init();
// mcp_driver.init();

}  // namespace Example4_Mixed

// =============================================================================
// Complete DriveBase Setup Example
// =============================================================================

// This shows how to use the motor setups in a complete drive base
// configuration. Choose one of the examples above and follow this pattern:

/*
#include "DriveBaseHALConfig.example.h"

// Using Example 2 (PCA9685 for PWM)
using namespace DriveConfig::Example2_PCA9685_PWM;

// In your initialization code:
void setupDriveBase() {
  // Initialize hardware drivers
  pca_driver.init();

  // Create encoders (these still use native pins typically)
  static Drivers::EncoderDriverSetup left_encoder_setup("left_encoder", 18,
19); static Drivers::EncoderDriverSetup right_encoder_setup("right_encoder",
20, 21);

  // Create DriveBaseSetup
  DriveBaseSetup drive_setup = createDriveBaseSetup();  // From
DriveBaseConfig.example.h

  // Override motor setups with HAL versions
  drive_setup.motorSetups = {
    left_motor1_setup,
    left_motor2_setup,
    right_motor1_setup,
    right_motor2_setup
  };

  drive_setup.encoderSetups = {
    left_encoder_setup,
    right_encoder_setup
  };

  // Create DriveSubsystem
  static DriveSubsystemSetup drive_subsystem_setup("drive", drive_setup);
  static DriveSubsystem drive_subsystem(drive_subsystem_setup);

  // Initialize
  drive_subsystem.init();
}
*/

}  // namespace DriveConfig
