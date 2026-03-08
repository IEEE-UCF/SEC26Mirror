#pragma once
/**
 * @file DroneLogic.h
 * @brief Main wiring for drone firmware: globals, FreeRTOS tasks, setup/loop.
 *
 * Follows the RobotLogic.h / MinibotLogic.h pattern.
 * FreeRTOS tasks:
 *   flightControlTask (core 1, pri 5, 250Hz) — IMU read → PID → motors, EKF predict
 *   heightSensorTask  (core 1, pri 3, 50Hz)  — VL53L0X read (guarded)
 *   uwbTask           (core 0, pri 3, 20Hz)  — UWB ranging + EKF update (guarded)
 *   safetyTask        (core 1, pri 4, 10Hz)  — Failure detection
 *   Arduino loop()    (any,    pri 1)         — WiFi, OTA, micro-ROS, state machine
 */

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESP32WifiSubsystem.h>
#include <SPI.h>
#include <WiFi.h>
#include <Wire.h>
#include <microros_manager_robot.h>

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

// ── I2C mutex (shared between flight task and height task) ──
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
};
static Drone::HeightSubsystem g_height(g_height_cfg);
#endif

// ── IR transmitter ──
static Drone::IRConfig g_ir_cfg{.ir_pin = Pins::IR_LED};
static Drone::IRSubsystem g_ir(g_ir_cfg);

// ── Flight controller ──
static Drone::FlightMotorPins g_motor_pins{
    .fl = Pins::MOTOR_FL,
    .fr = Pins::MOTOR_FR,
    .br = Pins::MOTOR_BR,
    .bl = Pins::MOTOR_BL,
};
static Drone::DroneFlightSubsystem g_flight(g_motor_pins);

// ── EKF ──
static Drone::DroneEKFSubsystem g_ekf;

// ── UWB ──
#if DRONE_ENABLE_UWB
static Drivers::UWBDriverSetup g_uwb_setup("uwb_driver", Drivers::UWBMode::TAG,
                                            Pins::UWB_DEVICE_ID, Pins::UWB_CS);
static Drivers::UWBDriver g_uwb(g_uwb_setup);
static Drone::DroneUWBSubsystem g_drone_uwb(g_uwb);
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
//  FreeRTOS Tasks
// ════════════════════════════════════════════════════════════

// Flight control task: IMU read → PID → motors, EKF predict (250Hz, core 1)
static void flightControlTask(void* /*param*/) {
  TickType_t prev_wake = xTaskGetTickCount();
  const TickType_t period =
      pdMS_TO_TICKS(1000 / Config::FLIGHT_RATE_HZ);
  uint32_t last_us = micros();

  for (;;) {
    // Skip all work if IMU not initialized — no point running PID
    if (!g_gyro.isInitialized()) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    uint32_t now_us = micros();
    float dt = (now_us - last_us) * 1e-6f;
    last_us = now_us;

    // Read IMU (under I2C mutex)
    xSemaphoreTake(g_i2c_mutex, portMAX_DELAY);
    g_gyro.update();
    xSemaphoreGive(g_i2c_mutex);

    IMUData imu = g_gyro.getData();

    // EKF predict (world-frame accel via full body→world rotation)
    float ax_w, ay_w;
    g_gyro.getAccelWorld(ax_w, ay_w);
    g_ekf.predict(ax_w, ay_w, dt);

    // Flight controller update
#if DRONE_ENABLE_HEIGHT
    float alt = g_height.getAltitudeM();
#else
    float alt = 0.0f;
#endif
    g_flight.update(imu, alt, dt);

    vTaskDelayUntil(&prev_wake, period);
  }
}

// Height sensor task: VL53L0X read (50Hz, core 1, guarded)
#if DRONE_ENABLE_HEIGHT
static void heightSensorTask(void* /*param*/) {
  TickType_t prev_wake = xTaskGetTickCount();
  const TickType_t period =
      pdMS_TO_TICKS(1000 / Config::HEIGHT_RATE_HZ);

  for (;;) {
    xSemaphoreTake(g_i2c_mutex, portMAX_DELAY);
    g_height.update();
    xSemaphoreGive(g_i2c_mutex);

    vTaskDelayUntil(&prev_wake, period);
  }
}
#endif

// UWB task: ranging + EKF update (20Hz, core 0, guarded)
#if DRONE_ENABLE_UWB
static void uwbTask(void* /*param*/) {
  TickType_t prev_wake = xTaskGetTickCount();
  const TickType_t period =
      pdMS_TO_TICKS(1000 / Config::UWB_RATE_HZ);

  for (;;) {
    g_uwb.update();
    g_uwb.startRanging();

    // Feed valid ranges to EKF (per-range scalar updates)
    const auto& data = g_uwb.getData();
    float distances[Drivers::UWBDriverData::MAX_ANCHORS];
    uint8_t ids[Drivers::UWBDriverData::MAX_ANCHORS];
    uint8_t n = 0;
    for (uint8_t i = 0; i < Drivers::UWBDriverData::MAX_ANCHORS; i++) {
      if (data.ranges[i].valid) {
        distances[n] = data.ranges[i].distance_cm / 100.0f;
        ids[n] = data.ranges[i].peer_id;
        n++;
      }
    }
    if (n > 0) {
      g_ekf.updateUWB(distances, ids, n);
    }

    // Publish UWB ranging data
    g_drone_uwb.publish();

    vTaskDelayUntil(&prev_wake, period);
  }
}
#endif

// ════════════════════════════════════════════════════════════
//  Arduino setup() / loop()
// ════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(921600);
  delay(2000);
  Serial.println("[Drone] Starting...");

  // I2C mutex
  g_i2c_mutex = xSemaphoreCreateMutex();

  // I2C bus
  Wire.begin(Pins::I2C_SDA, Pins::I2C_SCL);
  Wire.setClock(400000);

  // ── WiFi ──
  g_wifi.init();
  g_wifi.begin();
  Serial.println("[Drone] Waiting for WiFi...");
  {
    uint32_t wifi_start = millis();
    while (!g_wifi.isConnected()) {
      g_wifi.update();
      delay(50);  // yield to watchdog
      if (millis() - wifi_start > 15000) {
        Serial.println("[Drone] WiFi timeout — continuing anyway");
        break;
      }
    }
  }
  Serial.printf("[Drone] WiFi connected: %s\n",
                WiFi.localIP().toString().c_str());

  // ── ArduinoOTA ──
  ArduinoOTA.setHostname("sec26-drone");
  ArduinoOTA.begin();
  Serial.println("[Drone] OTA ready");

  // ── micro-ROS ──
  g_mr.init();

  // ── Initialize subsystems ──
  Serial.println("[Drone] Initializing subsystems...");

  xSemaphoreTake(g_i2c_mutex, portMAX_DELAY);
  bool gyro_ok = g_gyro.init();
  xSemaphoreGive(g_i2c_mutex);
  Serial.printf("[Drone] IMU: %s\n", gyro_ok ? "OK" : "FAIL");

#if DRONE_ENABLE_HEIGHT
  xSemaphoreTake(g_i2c_mutex, portMAX_DELAY);
  bool height_ok = g_height.init();
  xSemaphoreGive(g_i2c_mutex);
  Serial.printf("[Drone] Height: %s\n", height_ok ? "OK" : "FAIL");
#endif

  g_ir.init();
  Serial.println("[Drone] IR: OK");

  g_flight.init();
  Serial.println("[Drone] Flight controller: OK");

  g_ekf.init();
  Serial.println("[Drone] EKF: OK");

#if DRONE_ENABLE_UWB
  SPI.begin();
  bool uwb_ok = g_uwb.init();
  Serial.printf("[Drone] UWB: %s\n", uwb_ok ? "OK" : "FAIL");
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

  // ── Create FreeRTOS tasks ──
  // Flight-critical tasks use xTaskCreatePinnedToCore for core affinity
  // and vTaskDelayUntil for precise timing.
  xTaskCreatePinnedToCore(flightControlTask, "flight", Config::FLIGHT_TASK_STACK,
                          nullptr, Config::FLIGHT_TASK_PRIORITY, nullptr, 1);

#if DRONE_ENABLE_HEIGHT
  xTaskCreatePinnedToCore(heightSensorTask, "height", Config::HEIGHT_TASK_STACK,
                          nullptr, Config::HEIGHT_TASK_PRIORITY, nullptr, 1);
#endif

#if DRONE_ENABLE_UWB
  xTaskCreatePinnedToCore(uwbTask, "uwb", Config::UWB_TASK_STACK, nullptr,
                          Config::UWB_TASK_PRIORITY, nullptr, 0);
#endif

  // Non-flight subsystems use beginThreaded() (same pattern as robot).
  //                            stack  pri  rate(ms)
  g_mr.beginThreaded(          4096,   4,   100);   // micro-ROS agent (10Hz)
  g_state.beginThreaded(       2048,   2,   100);   // state machine (10Hz)
  g_safety.beginThreaded(      1024,   3,   100);   // safety monitor (10Hz)
  g_hb.beginThreaded(          2048,   1,   1000);  // heartbeat (1Hz)

  Serial.printf("[Drone] Free heap: %u bytes\n", ESP.getFreeHeap());
  Serial.println("[Drone] All tasks started. Ready!");
}

// loop() is minimal — all subsystems run in FreeRTOS tasks.
// Only WiFi + OTA remain here (ESP32 Arduino WiFi stack requirement).
void loop() {
  g_wifi.update();
  ArduinoOTA.handle();
  vTaskDelay(pdMS_TO_TICKS(10));
}
