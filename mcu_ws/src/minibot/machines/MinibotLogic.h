/**
 * @file MinibotLogic.h
 * @author Rafeed Khan
 * @brief Wiring logic for the minibot ESP32-S3 (UWB tag + motor control +
 * IMU over micro-ROS WiFi)
 */
#pragma once

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESP32WifiSubsystem.h>
#include <SPI.h>
#include <WiFi.h>
#include <Wire.h>
#include <microros_manager_robot.h>

#include "../MiniBotIMUSubsystem.h"
#include "../MinibotDriveSubsystem.h"
#include "../MinibotMotorDriver.h"
#include "UWBDriver.h"
#include "UWBSubsystem.h"

using namespace Subsystem;

// Set true during OTA update to skip blocking micro-ROS pings
static volatile bool g_ota_in_progress = false;

// ── Pin Definitions (ESP32-S3 XIAO Plus — update when hardware is wired) ──

// I2C for MPU6050
#define I2C_SDA_PIN 5   // TODO: assign real pin
#define I2C_SCL_PIN 6   // TODO: assign real pin

// SPI for UWB DW3000
#define UWB_CS_PIN  7   // TODO: assign real pin
#define UWB_RST_PIN 8   // TODO: assign real pin

// Motor A (left): IN1 + IN2 direction, PWM speed
#define MOTOR_A_IN1_PIN 1   // TODO: assign real pin
#define MOTOR_A_IN2_PIN 2   // TODO: assign real pin
#define MOTOR_A_PWM_PIN 3   // TODO: assign real pin

// Motor B (right): IN1 + IN2 direction, PWM speed
#define MOTOR_B_IN1_PIN 4   // TODO: assign real pin
#define MOTOR_B_IN2_PIN 9   // TODO: assign real pin
#define MOTOR_B_PWM_PIN 10  // TODO: assign real pin

// ── UWB Configuration ──
#define MINIBOT_TAG_ID 2
static const uint8_t ANCHOR_IDS[] = {10, 11, 12, 13};
static const uint8_t NUM_ANCHORS = 4;

// ── WiFi subsystem (manages connection lifecycle + auto-reconnect) ──
static IPAddress g_local_ip(LOCAL_IP);
static ESP32WifiSubsystemSetup g_wifi_setup(
    "wifi", WIFI_SSID, WIFI_PASSWORD, g_local_ip,
    IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0),
    /*connection_timeout_ms=*/10000,
    /*reconnect_interval_ms=*/5000,
    /*max_retries=*/0);  // 0 = infinite retries
static ESP32WifiSubsystem g_wifi(g_wifi_setup);

// ── micro-ROS Manager ──
static MicrorosManagerSetup g_mr_setup("microros_manager");
static MicrorosManager g_mr(g_mr_setup);

// ── UWB Driver and Subsystem ──
static Drivers::UWBDriverSetup g_uwb_driver_setup(
    "uwb_driver", Drivers::UWBMode::TAG, MINIBOT_TAG_ID, UWB_CS_PIN,
    UWB_RST_PIN);
static Drivers::UWBDriver g_uwb_driver(g_uwb_driver_setup);

static UWBSubsystemSetup g_uwb_setup("uwb_subsystem", &g_uwb_driver);
static UWBSubsystem g_uwb(g_uwb_setup);

// ── Motor Drivers (3-pin: IN1 + IN2 + PWM per channel) ──
static Drivers::MinibotMotorDriver g_left_motor(MOTOR_A_IN1_PIN,
                                                MOTOR_A_IN2_PIN,
                                                MOTOR_A_PWM_PIN);
static Drivers::MinibotMotorDriver g_right_motor(MOTOR_B_IN1_PIN,
                                                 MOTOR_B_IN2_PIN,
                                                 MOTOR_B_PWM_PIN);

// ── Drive Subsystem ──
static MinibotDriveSubsystemSetup g_drive_setup("minibot_drive", &g_left_motor,
                                                &g_right_motor);
static MinibotDriveSubsystem g_drive(g_drive_setup);

// ── IMU Subsystem ──
static Drivers::MPU6050DriverSetup g_mpu_driver_setup("mpu6050");
static MiniBotIMUSubsystemSetup g_imu_setup("minibot_imu", g_mpu_driver_setup);
static MiniBotIMUSubsystem g_imu(g_imu_setup);

// ── Arduino Entry Points ──

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Minibot starting...");

  // WiFi connection
  g_wifi.init();
  g_wifi.begin();
  Serial.println("Waiting for WiFi...");
  while (!g_wifi.isConnected()) {
    g_wifi.update();
    delay(10);
  }
  Serial.printf("WiFi connected: %s\n", WiFi.localIP().toString().c_str());

  // ArduinoOTA for wireless firmware updates
  ArduinoOTA.setHostname("sec26-minibot");
  ArduinoOTA.onStart([]() {
    g_ota_in_progress = true;
    Serial.println("[OTA] Update starting — pausing micro-ROS");
  });
  ArduinoOTA.onEnd([]() {
    g_ota_in_progress = false;
    Serial.println("[OTA] Update complete");
  });
  ArduinoOTA.onError([](ota_error_t error) {
    g_ota_in_progress = false;
    Serial.printf("[OTA] Error %u\n", error);
  });
  ArduinoOTA.begin();
  Serial.println("[OTA] Ready");

  // I2C for MPU6050
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // SPI for UWB
  SPI.begin();

  // Init all subsystems
  bool ok = true;
  ok = ok && g_mr.init();
  ok = ok && g_uwb.init();
  ok = ok && g_left_motor.init();
  ok = ok && g_right_motor.init();
  ok = ok && g_drive.init();
  ok = ok && g_imu.init();

  if (!ok) {
    Serial.println("ERROR: One or more subsystems failed init!");
    // Continue anyway — non-critical subsystems may have failed
  }

  g_uwb_driver.setTargetAnchors(ANCHOR_IDS, NUM_ANCHORS);

  // Register micro-ROS participants
  g_mr.registerParticipant(&g_uwb);
  g_mr.registerParticipant(&g_drive);
  g_mr.registerParticipant(&g_imu);
  g_mr.begin();

  g_uwb.begin();
  g_drive.begin();
  g_imu.begin();

  Serial.println("Minibot initialized!");
}

void loop() {
  g_wifi.update();
  ArduinoOTA.handle();

  if (!g_ota_in_progress) {
    g_mr.update();
    g_uwb.update();
    g_drive.update();
    g_imu.update();
  }

  delay(1);
}
