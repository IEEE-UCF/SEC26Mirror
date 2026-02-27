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
#include "../MinibotMotorDriver.h"
#include "UWBDriver.h"
#include "UWBSubsystem.h"

using namespace Subsystem;

// UWB Pin Definitions
#define UWB_CS_PIN 5
#define UWB_RST_PIN 4

// Motor Pin Definitions (A1/A2 = left, B1/B2 = right)
#define MOTOR_A1_PIN 25  // Left forward
#define MOTOR_A2_PIN 26  // Left reverse
#define MOTOR_B1_PIN 27  // Right forward
#define MOTOR_B2_PIN 14  // Right reverse

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

// Motor Drivers (A channel = left, B channel = right)
static Drivers::MinibotMotorDriver g_left_motor(MOTOR_A1_PIN, MOTOR_A2_PIN);
static Drivers::MinibotMotorDriver g_right_motor(MOTOR_B1_PIN, MOTOR_B2_PIN);

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

  // Motor test: forward → stop → forward (500ms each)
  Serial.println("Motor test: forward 10/255");
  g_left_motor.setPWM(10);
  g_right_motor.setPWM(10);
  g_left_motor.update();
  g_right_motor.update();
  delay(500);

  Serial.println("Motor test: stop");
  g_left_motor.setPWM(0);
  g_right_motor.setPWM(0);
  g_left_motor.update();
  g_right_motor.update();
  delay(500);

  Serial.println("Motor test: forward 10/255");
  g_left_motor.setPWM(10);
  g_right_motor.setPWM(10);
  g_left_motor.update();
  g_right_motor.update();
  delay(500);

  Serial.println("Motor test: stop");
  g_left_motor.setPWM(0);
  g_right_motor.setPWM(0);
  g_left_motor.update();
  g_right_motor.update();

  Serial.println("Motor test complete.");
}

void loop() {
  g_mr.update();
  g_uwb.update();
  g_drive.update();

  delay(1);
}
