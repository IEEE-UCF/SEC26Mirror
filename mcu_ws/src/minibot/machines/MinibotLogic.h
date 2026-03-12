/**
 * @file MinibotLogic.h
 * @brief Wiring logic for the minibot ESP32-S3: service-triggered crater
 *        navigation + micro-ROS WiFi + OTA.
 *
 * FreeRTOS tasks:
 *   MicrorosManager        (pri 2, 100Hz, 8192 stack)
 *   MinibotMission         (pri 2, 20Hz,  2048 stack)
 *   HeartbeatSubsystem     (pri 1, 1Hz,   2048 stack)
 *   WiFi+OTA               (pri 1, 10Hz,  raw xTaskCreate)
 */
#pragma once

#include <Arduino.h>
#include <ArduinoOTA.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <ESP32WifiSubsystem.h>
#include <WiFi.h>
#include <microros_manager_robot.h>

#include "../MinibotMotorDriver.h"
#include "../subsystems/MinibotMissionSubsystem.h"
#include "robot/machines/HeartbeatSubsystem.h"

using namespace Subsystem;

// ── Pin Definitions (from MinibotCorrectCode.ino) ──
#define MOTOR_A_IN1_PIN 5
#define MOTOR_A_IN2_PIN 15
#define MOTOR_A_PWM_PIN 2

#define MOTOR_B_IN1_PIN 18
#define MOTOR_B_IN2_PIN 19
#define MOTOR_B_PWM_PIN 4

// ── WiFi subsystem ──
static IPAddress g_local_ip(LOCAL_IP);
static ESP32WifiSubsystemSetup g_wifi_setup(
    "wifi", WIFI_SSID, WIFI_PASSWORD, g_local_ip,
    IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0),
    10000, 5000, 0);
static ESP32WifiSubsystem g_wifi(g_wifi_setup);

// ── micro-ROS Manager ──
static MicrorosManagerSetup g_mr_setup("microros_manager", "minibot_manager",
                                       "/mcu_minibot/debug");
static MicrorosManager g_mr(g_mr_setup);

// ── Heartbeat ──
static HeartbeatSubsystemSetup g_hb_setup("heartbeat",
                                          "/mcu_minibot/heartbeat");
static HeartbeatSubsystem g_hb(g_hb_setup);

// ── Motor Drivers ──
static Drivers::MinibotMotorDriver g_left_motor(MOTOR_A_IN1_PIN,
                                                MOTOR_A_IN2_PIN,
                                                MOTOR_A_PWM_PIN);
static Drivers::MinibotMotorDriver g_right_motor(MOTOR_B_IN1_PIN,
                                                 MOTOR_B_IN2_PIN,
                                                 MOTOR_B_PWM_PIN);

// ── Mission Subsystem ──
static MinibotMissionSubsystem g_mission("minibot_mission", g_left_motor,
                                         g_right_motor);

// ── Reset reason (published over micro-ROS debug topic on first connect) ──
static char g_reset_reason_str[48] = {0};
static bool g_reset_reason_published = false;

// ════════════════════════════════════════════════════════════
//  Arduino setup() / loop()
// ════════════════════════════════════════════════════════════

void setup() {
  delay(3000);

  // Disable brownout detector — WiFi current causes transient voltage dips
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // Capture reset reason
  {
    esp_reset_reason_t reason = esp_reset_reason();
    const char* reasons[] = {"UNKNOWN", "POWERON", "EXT",       "SW",
                             "PANIC",   "INT_WDT", "TASK_WDT",  "WDT",
                             "DEEPSLEEP","BROWNOUT","SDIO"};
    int r = (int)reason;
    snprintf(g_reset_reason_str, sizeof(g_reset_reason_str),
             "[Minibot] Reset: %s (%d)",
             (r >= 0 && r <= 10) ? reasons[r] : "?", r);
    g_reset_reason_published = false;
  }

  Serial.begin(921600);
  delay(500);
  Serial.println("[Minibot] Starting...");

  // ── WiFi (15s timeout, continue anyway) ──
  g_wifi.init();
  g_wifi.begin();
  Serial.println("[Minibot] Waiting for WiFi...");
  {
    uint32_t wifi_start = millis();
    while (!g_wifi.isConnected()) {
      g_wifi.update();
      delay(50);
      if (millis() - wifi_start > 15000) {
        Serial.println("[Minibot] WiFi timeout — continuing anyway");
        break;
      }
    }
  }
  Serial.printf("[Minibot] WiFi: %s\n", WiFi.localIP().toString().c_str());

  // ── ArduinoOTA ──
  ArduinoOTA.setHostname("sec26-minibot");
  ArduinoOTA.begin();
  Serial.println("[Minibot] OTA ready");

  // ── micro-ROS ──
  g_mr.init();

  // ── Initialize subsystems ──
  g_left_motor.init();
  g_right_motor.init();
  g_mission.init();
  g_hb.init();

  // ── Register micro-ROS participants ──
  g_mr.registerParticipant(&g_mission);
  g_mr.registerParticipant(&g_hb);

  // ── Start FreeRTOS tasks ──
  g_mr.beginThreaded(8192, 2, 10);
  g_mission.beginThreaded(2048, 2, 50);
  g_hb.beginThreaded(2048, 1, 1000);

  // WiFi + OTA task
  xTaskCreate([](void*) {
    while (true) {
      g_wifi.update();
      ArduinoOTA.handle();

      if (g_mr.isConnected()) {
        if (!g_reset_reason_published && g_reset_reason_str[0]) {
          g_mr.debugLog(g_reset_reason_str);
          g_reset_reason_published = true;
        }
      }

      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }, "wifi_ota", 4096, nullptr, 1, nullptr);

  Serial.printf("[Minibot] Free heap: %u bytes\n", ESP.getFreeHeap());
  Serial.println("[Minibot] All tasks started. Ready!");
}

// loop() empty — all work runs in FreeRTOS tasks.
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
