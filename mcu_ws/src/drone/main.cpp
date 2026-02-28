/**
 * @file main.cpp
 * @brief Drone ESP32 firmware with WiFi, micro-ROS, and ArduinoOTA.
 * @date 2026-02-26
 *
 * Extends the base drone with WiFi connectivity for micro-ROS telemetry
 * and ArduinoOTA for wireless firmware updates.
 *
 * Static IP: 192.168.4.25 (configured via platformio.ini build flags)
 */

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <microros_manager_robot.h>

#include "robot/machines/HeartbeatSubsystem.h"

using namespace Subsystem;

// --- micro-ROS manager ---
static MicrorosManagerSetup g_mr_setup("microros_manager");
static MicrorosManager g_mr(g_mr_setup);

// --- Heartbeat subsystem ---
static HeartbeatSubsystemSetup g_hb_setup("heartbeat_subsystem");
static HeartbeatSubsystem g_hb(g_hb_setup);

void setup() {
  Serial.begin(921600);
  delay(2000);
  Serial.println("Drone starting...");

  // WiFi + micro-ROS transport
  g_mr.init();
  g_mr.begin();
  Serial.printf("WiFi status: %d (3=connected)\n", WiFi.status());

  // ArduinoOTA for wireless firmware updates
  ArduinoOTA.setHostname("sec26-drone");
  ArduinoOTA.begin();
  Serial.println("[OTA] Ready as sec26-drone");

  // Initialize subsystems
  g_hb.init();

  // Register micro-ROS participants
  g_mr.registerParticipant(&g_hb);

  g_hb.begin();

  Serial.println("Drone initialized!");
}

void loop() {
  ArduinoOTA.handle();
  g_mr.update();
  g_hb.update();

  delay(1);
}
