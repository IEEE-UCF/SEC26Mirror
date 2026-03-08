/**
 * @file main.cpp
 * @brief Drone ESP32 firmware with WiFi, micro-ROS, BNO085 IMU, and ArduinoOTA.
 * @date 2026-02-26
 *
 * Static IP: 192.168.4.25 (configured via platformio.ini build flags)
 */

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESP32WifiSubsystem.h>
#include <WiFi.h>
#include <Wire.h>
#include <microros_manager_robot.h>

#include "robot/machines/HeartbeatSubsystem.h"
#include "drone/subsystems/GyroSubsystem.h"

using namespace Subsystem;

// --- WiFi subsystem (manages connection lifecycle + auto-reconnect) ---
static IPAddress g_local_ip(LOCAL_IP);
static ESP32WifiSubsystemSetup g_wifi_setup(
    "wifi", WIFI_SSID, WIFI_PASSWORD, g_local_ip,
    IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0),
    /*connection_timeout_ms=*/10000,
    /*reconnect_interval_ms=*/5000,
    /*max_retries=*/0);  // 0 = infinite retries
static ESP32WifiSubsystem g_wifi(g_wifi_setup);

// --- micro-ROS manager ---
static MicrorosManagerSetup g_mr_setup("microros_manager");
static MicrorosManager g_mr(g_mr_setup);

// --- Heartbeat subsystem ---
static HeartbeatSubsystemSetup g_hb_setup("heartbeat_subsystem");
static HeartbeatSubsystem g_hb(g_hb_setup);

// --- IMU (BNO085 via shared driver) ---
static Drone::GyroConfig g_gyro_cfg = {
    .i2c_addr = 0x4A,
    .reset_pin = 255,
    .int_pin = -1,
    .rotation_report_us = 5000,   // 200 Hz quaternion
    .gyro_report_us = 2500,       // 400 Hz gyro rates
};
static Drone::GyroSubsystem g_gyro(g_gyro_cfg);

void setup() {
  Serial.begin(921600);
  delay(2000);
  Serial.println("Drone starting...");

  // WiFi connection (managed by ESP32WifiSubsystem with auto-reconnect)
  g_wifi.init();
  g_wifi.begin();
  Serial.println("Waiting for WiFi...");
  while (!g_wifi.isConnected()) {
    g_wifi.update();
    delay(10);
  }
  Serial.printf("WiFi connected: %s\n", WiFi.localIP().toString().c_str());

  // ArduinoOTA for wireless firmware updates (needs WiFi up)
  ArduinoOTA.setHostname("sec26-drone");
  ArduinoOTA.begin();
  Serial.println("[OTA] Ready as sec26-drone");

  // I2C for BNO085 IMU
  Wire.begin();

  // Init subsystems
  bool ok = true;
  ok &= g_mr.init();
  ok &= g_hb.init();
  ok &= g_gyro.init();

  if (!ok) {
    Serial.println("ERROR: One or more subsystems failed init!");
  }

  // Register micro-ROS participants
  g_mr.registerParticipant(&g_hb);

  g_mr.begin();
  g_hb.begin();

  Serial.println("Drone initialized!");
}

void loop() {
  g_wifi.update();
  ArduinoOTA.handle();
  g_mr.update();
  g_hb.update();
  g_gyro.update();

  delay(1);
}
