/**
 * @file MinibotLogic.h
 * @author Rafeed Khan
 * @brief Wiring logic for the minibot ESP32 (UWB tag + motor control over
 * micro-ROS WiFi)
 */
#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <microros_manager_robot.h>

#include "../MinibotDriveSubsystem.h"
#include "MotorDriver.h"
#include "NativeGPIO.h"
#include "UWBDriver.h"
#include "UWBSubsystem.h"

using namespace Subsystem;

// UWB Pin Definitions
#define UWB_CS_PIN 5
#define UWB_RST_PIN 4

// Motor Pin Definitions (ADJUST FOR OUR SPECIFIC WIRING)
#define MOTOR_LEFT_PWM_PIN 25
#define MOTOR_LEFT_DIR_PIN 26
#define MOTOR_RIGHT_PWM_PIN 27
#define MOTOR_RIGHT_DIR_PIN 14

// UWB Configuration
#define MINIBOT_TAG_ID 2
static const uint8_t ANCHOR_IDS[] = {10, 11, 12, 13};
static const uint8_t NUM_ANCHORS = 4;

// SELF EXPLAINTORY!!!
static MicrorosManagerSetup g_mr_setup("microros_manager");
static MicrorosManager g_mr(g_mr_setup);

// UWB Driver and Subsystem
static Drivers::UWBDriverSetup g_uwb_driver_setup("uwb_driver",
                                                  Drivers::UWBMode::TAG,
                                                  MINIBOT_TAG_ID, UWB_CS_PIN,
                                                  UWB_RST_PIN);
static Drivers::UWBDriver g_uwb_driver(g_uwb_driver_setup);

static UWBSubsystemSetup g_uwb_setup("uwb_subsystem", &g_uwb_driver);
static UWBSubsystem g_uwb(g_uwb_setup);

// Motor Drivers
static HAL::NativeGPIO g_left_pwm_pin(MOTOR_LEFT_PWM_PIN);
static HAL::NativeGPIO g_left_dir_pin(MOTOR_LEFT_DIR_PIN);
static HAL::NativeGPIO g_right_pwm_pin(MOTOR_RIGHT_PWM_PIN);
static HAL::NativeGPIO g_right_dir_pin(MOTOR_RIGHT_DIR_PIN);

static Drivers::MotorDriverSetup g_left_setup("left_motor", &g_left_pwm_pin,
                                              &g_left_dir_pin);
static Drivers::MotorDriver g_left_motor(g_left_setup);

static Drivers::MotorDriverSetup g_right_setup("right_motor", &g_right_pwm_pin,
                                               &g_right_dir_pin);
static Drivers::MotorDriver g_right_motor(g_right_setup);

// Drive Subsystem
static MinibotDriveSubsystemSetup g_drive_setup("minibot_drive", &g_left_motor,
                                                &g_right_motor);
static MinibotDriveSubsystem g_drive(g_drive_setup);

// Arduino Entry Points

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Minibot starting...");

  SPI.begin();

  bool ok = true;
  ok = ok && g_mr.init();
  ok = ok && g_uwb.init();
  ok = ok && g_left_motor.init();
  ok = ok && g_right_motor.init();
  ok = ok && g_drive.init();

  if (!ok) {
    Serial.println("ERROR: Init failed!");
    while (1) delay(1000);
  }

  g_uwb_driver.setTargetAnchors(ANCHOR_IDS, NUM_ANCHORS);

  // Register micro-ROS participants
  g_mr.registerParticipant(&g_uwb);
  g_mr.registerParticipant(&g_drive);
  g_mr.begin();

  g_uwb.begin();
  g_drive.begin();

  Serial.println("Minibot initialized!");
}

void loop() {
  g_mr.update();
  g_uwb.update();
  g_drive.update();

  delay(1);
}
