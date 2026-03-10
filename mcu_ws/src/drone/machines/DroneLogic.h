#pragma once
/**
 * @file DroneLogic.h
 * @brief Main wiring for drone firmware: globals, setup/loop.
 *
 * All subsystems use RTOSSubsystem::beginThreaded() or
 * beginThreadedPinned() — no raw xTaskCreatePinnedToCore() calls.
 *
 * RTOSSubsystem tasks:
 *   GyroSubsystem          (core 1, pri 5, 250Hz, precise)
 *   MicrorosManager         (any,    pri 4, 100Hz, standard)
 *   DroneFlightSubsystem   (core 1, pri 4, 250Hz, precise)
 *   HeightSubsystem        (core 1, pri 3, 50Hz,  precise)
 *   DroneUWBSubsystem      (core 0, pri 3, 20Hz,  precise)
 *   DroneSafetySubsystem   (any,    pri 3, 10Hz,  standard)
 *   DroneStateSubsystem    (any,    pri 2, 10Hz,  standard)
 *   HeartbeatSubsystem     (any,    pri 1, 1Hz,   standard)
 *   WiFi+OTA               (any,    pri 1, 10Hz,  raw task)
 */

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESP32WifiSubsystem.h>
#include <SPI.h>
#include <WiFi.h>
#include <Wire.h>
#include <microros_manager_robot.h>

#include "../DroneDebug.h"
#include "../DronePins.h"
#include "../DroneConfig.h"
#include "robot/machines/HeartbeatSubsystem.h"

// Subsystems
#include "../subsystems/GyroSubsystem.h"
#include "../subsystems/DroneFlightSubsystem.h"
#include "../subsystems/DroneEKFSubsystem.h"
#include "../subsystems/DroneStateSubsystem.h"
#include "../subsystems/DroneSafetySubsystem.h"
#include "../subsystems/DroneIRSubsystem.h"
#include "../subsystems/IRSubsystem.h"

#if DRONE_ENABLE_HEIGHT
#include "../subsystems/HeightSubsystem.h"
#endif

#if DRONE_ENABLE_UWB
#include <UWBDriver.h>
#include "../subsystems/DroneUWBSubsystem.h"
#endif

using namespace Subsystem;
using namespace Drone;

// ── I2C mutex (shared between gyro and height tasks) ──
static SemaphoreHandle_t g_i2c_mutex = nullptr;

// ── WiFi subsystem ──
static IPAddress g_local_ip(LOCAL_IP);
static ESP32WifiSubsystemSetup g_wifi_setup(
    "wifi", WIFI_SSID, WIFI_PASSWORD, g_local_ip,
    IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0),
    10000, 5000, 0);
static ESP32WifiSubsystem g_wifi(g_wifi_setup);

// ── micro-ROS manager (drone namespace) ──
static MicrorosManagerSetup g_mr_setup("microros_manager", "drone_manager",
                                       "/mcu_drone/debug");
static MicrorosManager g_mr(g_mr_setup);

// ── Heartbeat (drone namespace) ──
static HeartbeatSubsystemSetup g_hb_setup("heartbeat", "/mcu_drone/heartbeat");
static HeartbeatSubsystem g_hb(g_hb_setup);

// ── IMU (BNO085) ──
static Drone::GyroConfig g_gyro_cfg{
    .wire = &Wire,
    .i2c_addr = Config::IMU_I2C_ADDR,
    .reset_pin = Pins::IMU_RST,
    .int_pin = Pins::IMU_INT,
    .rotation_report_us = Config::IMU_ROTATION_REPORT_US,
    .gyro_report_us = Config::IMU_GYRO_REPORT_US,
    .accel_report_us = Config::IMU_ACCEL_REPORT_US,
    .i2c_mutex = &g_i2c_mutex,
};
static Drone::GyroSubsystem g_gyro(g_gyro_cfg);

// ── Height sensor (VL53L0X) ──
#if DRONE_ENABLE_HEIGHT
static Drone::HeightConfig g_height_cfg{
    .wire = &Wire,
    .xshut_pin = 255,
    .timing_budget_ms = Config::HEIGHT_TIMING_BUDGET_MS,
    .max_valid_m = Config::HEIGHT_MAX_VALID_M,
    .min_valid_m = Config::HEIGHT_MIN_VALID_M,
    .i2c_mutex = &g_i2c_mutex,
};
static Drone::HeightSubsystem g_height(g_height_cfg);
#endif

// ── IR transmitter ──
static Drone::IRConfig g_ir_cfg{.ir_pin = Pins::IR_LED};
static Drone::IRSubsystem g_ir(g_ir_cfg);

// ── EKF ──
static Drone::DroneEKFSubsystem g_ekf;

// ── Flight controller ──
static Drone::FlightMotorPins g_motor_pins{
    .fl = Pins::MOTOR_FL,
    .fr = Pins::MOTOR_FR,
    .br = Pins::MOTOR_BR,
    .bl = Pins::MOTOR_BL,
};
static Drone::DroneFlightSubsystem g_flight(
    g_motor_pins, g_gyro, g_ekf
#if DRONE_ENABLE_HEIGHT
    ,
    g_height
#endif
);

// ── UWB ──
#if DRONE_ENABLE_UWB
static Drivers::UWBDriverSetup g_uwb_setup("uwb_driver", Drivers::UWBMode::TAG,
                                            Pins::UWB_DEVICE_ID, Pins::UWB_CS);
static Drivers::UWBDriver g_uwb(g_uwb_setup);
static Drone::DroneUWBSubsystem g_drone_uwb(g_uwb, g_ekf);
#endif

// ── State machine ──
static Drone::DroneStateSubsystem g_state(
    "drone_state", g_flight, g_gyro, g_ekf
#if DRONE_ENABLE_HEIGHT
    ,
    g_height
#endif
);

// ── Safety monitor ──
static Drone::DroneSafetySubsystem g_safety(
    "drone_safety", g_state, g_flight, g_gyro, g_mr
#if DRONE_ENABLE_HEIGHT
    ,
    g_height
#endif
);

// ── IR ROS2 wrapper ──
static Drone::DroneIRSubsystem g_drone_ir(g_ir, g_state);

// ════════════════════════════════════════════════════════════
//  Arduino setup() / loop()
// ════════════════════════════════════════════════════════════

void setup() {
#ifdef DRONE_SERIAL_DEBUG
  Serial.begin(921600);
  delay(2000);
#endif
  DRONE_PRINTLN("[Drone] Starting...");

  // I2C mutex
  g_i2c_mutex = xSemaphoreCreateMutex();

  // I2C bus
  Wire.begin(Pins::I2C_SDA, Pins::I2C_SCL);
  Wire.setClock(400000);

  // ── WiFi ──
  g_wifi.init();
  g_wifi.begin();
  DRONE_PRINTLN("[Drone] Waiting for WiFi...");
  {
    uint32_t wifi_start = millis();
    while (!g_wifi.isConnected()) {
      g_wifi.update();
      delay(50);  // yield to watchdog
      if (millis() - wifi_start > 15000) {
        DRONE_PRINTLN("[Drone] WiFi timeout — continuing anyway");
        break;
      }
    }
  }
  DRONE_PRINTF("[Drone] WiFi connected: %s\n",
                WiFi.localIP().toString().c_str());

  // ── ArduinoOTA ──
  ArduinoOTA.setHostname("sec26-drone");
  ArduinoOTA.begin();
  DRONE_PRINTLN("[Drone] OTA ready");

  // ── micro-ROS ──
  g_mr.init();

  // ── Initialize subsystems ──
  DRONE_PRINTLN("[Drone] Initializing subsystems...");

  xSemaphoreTake(g_i2c_mutex, portMAX_DELAY);
  bool gyro_ok = g_gyro.init();
  xSemaphoreGive(g_i2c_mutex);
  DRONE_PRINTF("[Drone] IMU: %s\n", gyro_ok ? "OK" : "FAIL");

#if DRONE_ENABLE_HEIGHT
  xSemaphoreTake(g_i2c_mutex, portMAX_DELAY);
  bool height_ok = g_height.init();
  xSemaphoreGive(g_i2c_mutex);
  DRONE_PRINTF("[Drone] Height: %s\n", height_ok ? "OK" : "FAIL");
#endif

  g_ir.init();
  DRONE_PRINTLN("[Drone] IR: OK");

  g_flight.init();
  DRONE_PRINTLN("[Drone] Flight controller: OK");

  g_ekf.init();
  DRONE_PRINTLN("[Drone] EKF: OK");

#if DRONE_ENABLE_UWB
  SPI.begin();
  bool uwb_ok = g_uwb.init();
  DRONE_PRINTF("[Drone] UWB: %s\n", uwb_ok ? "OK" : "FAIL");
  if (uwb_ok) {
    uint8_t anchor_ids[] = {10, 11, 12};
    g_uwb.setTargetAnchors(anchor_ids, 3);
    g_uwb.begin();
  }
#endif

  g_hb.init();
  g_state.init();
  g_safety.init();

  // ── Register micro-ROS participants ──
  g_mr.registerParticipant(&g_hb);
  g_mr.registerParticipant(&g_state);
  g_mr.registerParticipant(&g_drone_ir);
#if DRONE_ENABLE_UWB
  g_mr.registerParticipant(&g_drone_uwb);
#endif

  // Check sensor readiness
  g_state.setAllSensorsReady(g_safety.checkSensorsReady());

  // ── Start RTOSSubsystem tasks ──
  // Flight-critical: beginThreadedPinned with precise timing (vTaskDelayUntil)
  //                          stack  pri  rate(ms) core
  g_gyro.beginThreadedPinned(Config::GYRO_TASK_STACK, 5, 4, 1);  // 250Hz, core 1
  g_flight.beginThreadedPinned(2048,  4,   4,       1);   // 250Hz, core 1
#if DRONE_ENABLE_HEIGHT
  g_height.beginThreadedPinned(Config::HEIGHT_TASK_STACK, 3, 20, 1);  // 50Hz, core 1
#endif
#if DRONE_ENABLE_UWB
  g_drone_uwb.beginThreadedPinned(2048, 3, 50,      0);   // 20Hz,  core 0
#endif

  // Non-flight subsystems: standard beginThreaded (vTaskDelay)
  g_mr.beginThreaded(          8192,   4,    10);   // micro-ROS agent (100Hz)
  g_state.beginThreaded(       2048,   2,   100);   // state machine (10Hz)
  g_safety.beginThreaded(      2048,   3,   100);   // safety monitor (10Hz)
  g_hb.beginThreaded(          2048,   1,  1000);   // heartbeat (1Hz)

  // WiFi + OTA task (ESP32WifiSubsystem is TimedSubsystem, not RTOSSubsystem)
  xTaskCreate([](void*) {
    while (true) {
      g_wifi.update();
      ArduinoOTA.handle();
#ifdef DRONE_SERIAL_DEBUG
      // Periodic status dashboard
      static uint32_t s_last_status_ms = 0;
      uint32_t now = millis();
      if (now - s_last_status_ms >= 2000) {
        s_last_status_ms = now;
        IMUData imu = g_gyro.getData();
        DRONE_PRINTF("\n[%u] ═══ Drone Status ═══\n", now);
        DRONE_PRINTF("  WiFi: %s  uROS: %s\n",
                      g_wifi.isConnected() ? "OK" : "DOWN",
                      g_mr.isConnected() ? "CONN" : "WAIT");
        DRONE_PRINTF("  IMU:    init=%c  roll=%.1f pitch=%.1f yaw=%.1f\n",
                      g_gyro.isInitialized() ? 'Y' : 'N',
                      imu.roll, imu.pitch, imu.yaw);
#if DRONE_ENABLE_HEIGHT
        DRONE_PRINTF("  Height: init=%c  alt=%.3fm  valid=%c\n",
                      g_height.isInitialized() ? 'Y' : 'N',
                      g_height.getAltitudeM(),
                      g_height.isValid() ? 'Y' : 'N');
#else
        DRONE_PRINTLN("  Height: disabled");
#endif
        DRONE_PRINTF("  Flight: armed=%c\n",
                      g_flight.isArmed() ? 'Y' : 'N');
        DRONE_PRINTF("  Motors: FL=%.2f FR=%.2f RR=%.2f RL=%.2f\n",
                      g_flight.getMotor(0), g_flight.getMotor(1),
                      g_flight.getMotor(2), g_flight.getMotor(3));
        DRONE_PRINTF("  Heap: %u / min: %u\n",
                      ESP.getFreeHeap(), ESP.getMinFreeHeap());
      }
#endif
      vTaskDelay(pdMS_TO_TICKS(100));  // 10Hz
    }
  }, "wifi_ota", 4096, nullptr, 1, nullptr);

  DRONE_PRINTF("[Drone] Free heap: %u bytes\n", ESP.getFreeHeap());
  DRONE_PRINTLN("[Drone] All tasks started. Ready!");
}

// loop() empty — all work runs in FreeRTOS tasks (matches robot pattern).
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
