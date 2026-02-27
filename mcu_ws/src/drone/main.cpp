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
#include <ESP32WifiSubsystem.h>
#include <WiFi.h>
#include <microros_manager_robot.h>

#include "robot/machines/HeartbeatSubsystem.h"

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

void setup() {
  Serial.begin(921600);
  delay(2000);
  Serial.println("Drone starting...");

  // Step 1: WiFi connection (managed by ESP32WifiSubsystem with auto-reconnect)
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

  // micro-ROS transport (registers UDP callbacks, no WiFi.begin())
  g_mr.init();

  // Initialize subsystems
  g_hb.init();

  // Register micro-ROS participants
  g_mr.registerParticipant(&g_hb);

  g_mr.begin();  // Registers UDP transport only
  g_hb.begin();

  Serial.println("Drone initialized!");
}

void loop() {
  g_wifi.update();  // WiFi reconnection monitoring
  ArduinoOTA.handle();
  g_mr.update();
  g_hb.update();

  delay(1);
}
