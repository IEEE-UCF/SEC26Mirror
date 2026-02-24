/**
 * @file BeaconLogic.h
 * @date 12/22/2025
 * @author SEC26 Team
 * @brief Logic for stationary UWB beacon (anchor mode)
 */

#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <microros_manager_robot.h>

#include "UWBDriver.h"
#include "beacon/machines/UWBSubsystem.h"
#include "robot/machines/HeartbeatSubsystem.h"

using namespace Subsystem;

// Pin definitions for ESP32 DW3000 connection
// Adjust these based on your hardware wiring
#define UWB_CS_PIN 5    // SPI Chip Select
#define UWB_RST_PIN 4   // Reset pin (255 if not used)
#define UWB_IRQ_PIN 17  // Interrupt pin (not used in polling mode)

// IMPORTANT: Set the beacon ID here (must be unique for each beacon)
// ID reservation: 10-12 = beacons, 13 = robot, 14 = minibot, 15 = drone
#ifndef BEACON_ID
#define BEACON_ID 10  // Change this for each beacon!
#endif

// All beacon IDs in the system (lower ID initiates inter-beacon ranging)
static constexpr uint8_t ALL_BEACON_IDS[] = {10, 11, 12};
static constexpr uint8_t NUM_ALL_BEACONS = 3;

// Peer beacons this device will range to (those with higher IDs)
static uint8_t g_peer_ids[NUM_ALL_BEACONS];
static uint8_t g_num_peers = 0;

// --- micro-ROS manager ---
static MicrorosManagerSetup g_mr_setup("microros_manager");
static MicrorosManager g_mr(g_mr_setup);

// --- Heartbeat subsystem ---
static HeartbeatSubsystemSetup g_hb_setup("heartbeat_subsystem");
static HeartbeatSubsystem g_hb(g_hb_setup);

// --- UWB Driver and Subsystem ---
static Drivers::UWBDriverSetup g_uwb_driver_setup(
    "uwb_driver",
    Drivers::UWBMode::ANCHOR,  // Beacon is an ANCHOR (responder)
    BEACON_ID, UWB_CS_PIN, UWB_RST_PIN, UWB_IRQ_PIN);
static Drivers::UWBDriver g_uwb_driver(g_uwb_driver_setup);

static UWBSubsystemSetup g_uwb_setup("uwb_subsystem", &g_uwb_driver);
static UWBSubsystem g_uwb(g_uwb_setup);

// --- Arduino sketch entry points ---
void setup() {
  // Use 921600 to match set_microros_transports() which re-inits at 921600.
  // This avoids a mid-setup baud rate change that can corrupt serial output.
  Serial.begin(921600);
  delay(2000);  // Wait for serial
  Serial.print("UWB Beacon ");
  Serial.print(BEACON_ID);
  Serial.println(" starting...");
  Serial.flush();

  // Step 1: WiFi transport first — must match esp32-test-microros-wifi pattern.
  // begin() calls set_microros_transports() which connects WiFi and configures
  // the UDP transport to the agent. All other init must follow this.
  Serial.println("[DBG] g_mr.init");
  Serial.flush();
  g_mr.init();
  Serial.println("[DBG] g_mr.begin (WiFi connect)");
  Serial.flush();
  g_mr.begin();
  Serial.println("[DBG] g_mr.begin done");
  Serial.flush();

  Serial.printf("WiFi status: %d (3=connected)\n", WiFi.status());
  Serial.flush();

  // Step 2: SPI for DW3000
  SPI.begin();

  // Step 3: Configure inter-beacon ranging BEFORE g_uwb.init() so hasPeers()
  // is correct during subsystem init (determines if peer publishers are
  // created). Lower-ID beacon always initiates ranging to higher-ID peers.
  g_num_peers = 0;
  for (uint8_t i = 0; i < NUM_ALL_BEACONS; i++) {
    if (ALL_BEACON_IDS[i] > BEACON_ID) {
      g_peer_ids[g_num_peers++] = ALL_BEACON_IDS[i];
    }
  }
  if (g_num_peers > 0) {
    g_uwb_driver.setPeerBeacons(g_peer_ids, g_num_peers, 500);  // 2 Hz
  }

  // Step 4: Initialize remaining subsystems
  bool ok = true;
  ok = ok && g_hb.init();
  ok = ok && g_uwb.init();

  if (!ok) {
    Serial.println("ERROR: Failed to initialize subsystems!");
    Serial.flush();
    while (1) {
      delay(1000);
    }
  }

  // Step 5: Register participants (must be before loop() calls update())
  g_mr.registerParticipant(&g_hb);
  g_mr.registerParticipant(&g_uwb);

  // Step 6: Begin subsystems
  g_hb.begin();
  g_uwb.begin();

  // Step 7: Power savings — set after WiFi is already connected.
  // Drop to 80 MHz (minimum that keeps WiFi alive), saves ~30 mA.
  setCpuFrequencyMhz(80);
  // Reduce TX power to 10 dBm (default 20 dBm). Adjust up if beacon is far from
  // AP.
  esp_wifi_set_max_tx_power(40);
  // NOTE: WIFI_PS_MAX_MODEM is intentionally NOT enabled here. Max modem sleep
  // causes the WiFi modem to miss micro-ROS UDP pings, triggering spurious
  // agent-disconnect cycles and crashing in onDestroy().

  Serial.print("Beacon ");
  Serial.print(BEACON_ID);
  Serial.println(" initialized and waiting for ranging requests!");
  Serial.flush();
}

void loop() {
  g_mr.update();
  g_hb.update();
  g_uwb.update();

  delay(1);  // Feed watchdog and yield to WiFi stack
}
