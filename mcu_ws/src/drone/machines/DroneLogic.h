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
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <ESP32WifiSubsystem.h>
#include <SPI.h>
#include <WiFi.h>
#include <Wire.h>
#include <microros_manager_robot.h>

#include "../DroneDebug.h"
#include "../DronePins.h"
#include "../DroneConfig.h"
#include "robot/machines/HeartbeatSubsystem.h"

// Debug stages:
//   0 = WiFi + micro-ROS + heartbeat only (isolate micro-ROS)
//   1 = + Gyro/IMU (isolate I2C)
//   2 = Full system (all subsystems)
#ifndef DRONE_DEBUG_STAGE
#define DRONE_DEBUG_STAGE 2
#endif

// Subsystems
#if DRONE_DEBUG_STAGE >= 1
#include "../subsystems/GyroSubsystem.h"
#endif
#if DRONE_DEBUG_STAGE >= 2
#include "../subsystems/DroneFlightSubsystem.h"
#include "../subsystems/DroneEKFSubsystem.h"
#include "../subsystems/DroneStateSubsystem.h"
#include "../subsystems/DroneSafetySubsystem.h"
#include "../subsystems/DroneIRSubsystem.h"
#include "../subsystems/IRSubsystem.h"
#endif

#if DRONE_ENABLE_HEIGHT && DRONE_DEBUG_STAGE >= 2
#include "../subsystems/HeightSubsystem.h"
#endif

#if DRONE_ENABLE_UWB && DRONE_DEBUG_STAGE >= 2
#include <UWBDriver.h>
#include "../subsystems/DroneUWBSubsystem.h"
#endif

using namespace Subsystem;

// ── I2C mutex (shared between gyro and height tasks) ──
#if DRONE_DEBUG_STAGE >= 1
static SemaphoreHandle_t g_i2c_mutex = nullptr;
#endif

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

// ── IMU (BNO085) ── [Stage 1+]
#if DRONE_DEBUG_STAGE >= 1
static Drone::GyroConfig g_gyro_cfg{
    .wire = &Wire,
    .i2c_addr = Drone::Config::IMU_I2C_ADDR,
    .reset_pin = Drone::Pins::IMU_RST,
    .int_pin = Drone::Pins::IMU_INT,
    .rotation_report_us = Drone::Config::IMU_ROTATION_REPORT_US,
    .gyro_report_us = Drone::Config::IMU_GYRO_REPORT_US,
    .accel_report_us = Drone::Config::IMU_ACCEL_REPORT_US,
    .i2c_mutex = &g_i2c_mutex,
};
static Drone::GyroSubsystem g_gyro(g_gyro_cfg);
#endif

// ── Height sensor (VL53L0X) ── [Stage 2+]
#if DRONE_ENABLE_HEIGHT && DRONE_DEBUG_STAGE >= 2
static Drone::HeightConfig g_height_cfg{
    .wire = &Wire,
    .xshut_pin = 255,
    .timing_budget_ms = Drone::Config::HEIGHT_TIMING_BUDGET_MS,
    .max_valid_m = Drone::Config::HEIGHT_MAX_VALID_M,
    .min_valid_m = Drone::Config::HEIGHT_MIN_VALID_M,
    .i2c_mutex = &g_i2c_mutex,
};
static Drone::HeightSubsystem g_height(g_height_cfg);
#endif

// ── Stage 2+: IR, EKF, Flight, UWB, State, Safety ──
#if DRONE_DEBUG_STAGE >= 2
static Drone::IRConfig g_ir_cfg{.ir_pin = Drone::Pins::IR_LED};
static Drone::IRSubsystem g_ir(g_ir_cfg);

static Drone::DroneEKFSubsystem g_ekf;

static Drone::FlightMotorPins g_motor_pins{
    .fl = Drone::Pins::MOTOR_FL,
    .fr = Drone::Pins::MOTOR_FR,
    .br = Drone::Pins::MOTOR_BR,
    .bl = Drone::Pins::MOTOR_BL,
};
static Drone::DroneFlightSubsystem g_flight(
    g_motor_pins, g_gyro, g_ekf
#if DRONE_ENABLE_HEIGHT
    ,
    g_height
#endif
);

#if DRONE_ENABLE_UWB
static Drivers::UWBDriverSetup g_uwb_setup("uwb_driver", Drivers::UWBMode::TAG,
                                            Drone::Pins::UWB_DEVICE_ID,
                                            Drone::Pins::UWB_CS);
static Drivers::UWBDriver g_uwb(g_uwb_setup);
static Drone::DroneUWBSubsystem g_drone_uwb(g_uwb, g_ekf);
#endif

static Drone::DroneStateSubsystem g_state(
    "drone_state", g_flight, g_gyro, g_ekf
#if DRONE_ENABLE_HEIGHT
    ,
    g_height
#endif
);

static Drone::DroneSafetySubsystem g_safety(
    "drone_safety", g_state, g_flight, g_gyro, g_mr
#if DRONE_ENABLE_HEIGHT
    ,
    g_height
#endif
);

static Drone::DroneIRSubsystem g_drone_ir(g_ir, g_state);
#endif  // DRONE_DEBUG_STAGE >= 2

// ── Reset reason (published over micro-ROS on first connect) ──
static char g_reset_reason_str[48] = {0};
static bool g_reset_reason_published = false;

// Forward declarations
#if DRONE_DEBUG_STAGE >= 2
static const char* stateToStr(Drone::DroneState s);
#endif
#ifdef DRONE_SERIAL_DEBUG
static void debugDashboardTask(void*);
#endif

// ════════════════════════════════════════════════════════════
//  Arduino setup() / loop()
// ════════════════════════════════════════════════════════════

void setup() {
  delay(3000);
  // Disable brownout detector — WiFi + motor current causes transient voltage
  // dips that trigger the default threshold, but the hardware handles it fine
  // (bare motor test runs 4 motors at 100% without issues).
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // Capture reset reason — will be published over micro-ROS debug topic
  // once connected (serial is useless since drone is WiFi-only).
  {
    esp_reset_reason_t reason = esp_reset_reason();
    const char* reasons[] = {"UNKNOWN","POWERON","EXT","SW","PANIC",
                             "INT_WDT","TASK_WDT","WDT","DEEPSLEEP",
                             "BROWNOUT","SDIO"};
    int r = (int)reason;
    snprintf(g_reset_reason_str, sizeof(g_reset_reason_str),
             "[Drone] Reset: %s (%d)",
             (r >= 0 && r <= 10) ? reasons[r] : "?", r);
    g_reset_reason_published = false;
  }
  Serial.begin(921600);

#ifdef DRONE_SERIAL_DEBUG
  // Already called Serial.begin above, just wait for terminal
  delay(1500);
#endif
  DRONE_PRINTF("[Drone] Starting... (DEBUG_STAGE=%d)\n", DRONE_DEBUG_STAGE);

#if DRONE_DEBUG_STAGE >= 1
  // I2C mutex
  g_i2c_mutex = xSemaphoreCreateMutex();

  // I2C bus
  Wire.begin(Drone::Pins::I2C_SDA, Drone::Pins::I2C_SCL);
  Wire.setClock(400000);
  DRONE_PRINTLN("[Drone] I2C bus initialized (400kHz)");
#endif

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

#if DRONE_DEBUG_STAGE >= 1
  xSemaphoreTake(g_i2c_mutex, portMAX_DELAY);
  bool gyro_ok = g_gyro.init();
  xSemaphoreGive(g_i2c_mutex);
  DRONE_PRINTF("[Drone] IMU: %s\n", gyro_ok ? "OK" : "FAIL");
#endif

#if DRONE_ENABLE_HEIGHT && DRONE_DEBUG_STAGE >= 2
  xSemaphoreTake(g_i2c_mutex, portMAX_DELAY);
  bool height_ok = g_height.init();
  xSemaphoreGive(g_i2c_mutex);
  DRONE_PRINTF("[Drone] Height: %s\n", height_ok ? "OK" : "FAIL");
#endif

#if DRONE_DEBUG_STAGE >= 2
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

  g_state.init();
  g_safety.init();
#endif  // DRONE_DEBUG_STAGE >= 2

  g_hb.init();

  // ── Register micro-ROS participants ──
  g_mr.registerParticipant(&g_hb);
#if DRONE_DEBUG_STAGE >= 2
  g_mr.registerParticipant(&g_state);
  g_mr.registerParticipant(&g_drone_ir);
#if DRONE_ENABLE_UWB
  g_mr.registerParticipant(&g_drone_uwb);
#endif
  // Check sensor readiness
  g_state.setAllSensorsReady(g_safety.checkSensorsReady());
#endif  // DRONE_DEBUG_STAGE >= 2

  // ── Start RTOSSubsystem tasks ──
#if DRONE_DEBUG_STAGE >= 1
  g_gyro.beginThreadedPinned(Drone::Config::GYRO_TASK_STACK, 5, 4, 1);
#endif

#if DRONE_DEBUG_STAGE >= 2
  g_flight.beginThreadedPinned(4096, 4, 4, 1);
#if DRONE_ENABLE_HEIGHT
  g_height.beginThreadedPinned(Drone::Config::HEIGHT_TASK_STACK, 3, 20, 1);
#endif
#if DRONE_ENABLE_UWB
  g_drone_uwb.beginThreadedPinned(2048, 3, 50, 0);
#endif
  g_state.beginThreaded(2048, 2, 100);
  g_safety.beginThreaded(2048, 3, 100);
#endif  // DRONE_DEBUG_STAGE >= 2

  g_mr.beginThreaded(8192, 2, 10);  // Lower than flight (4) and safety (3)
  g_hb.beginThreaded(2048, 1, 1000);

  // WiFi + OTA + debug telemetry task
  xTaskCreate([](void*) {
    uint32_t last_rate_pub = 0;
    while (true) {
      g_wifi.update();
      ArduinoOTA.handle();

      if (g_mr.isConnected()) {
        // Publish reset reason once
        if (!g_reset_reason_published && g_reset_reason_str[0]) {
          g_mr.debugLog(g_reset_reason_str);
          g_reset_reason_published = true;
        }

#if DRONE_DEBUG_STAGE >= 2
        // Publish loop rates every 5s
        uint32_t now = millis();
        if (now - last_rate_pub >= 5000) {
          last_rate_pub = now;
          char buf[64];
          snprintf(buf, sizeof(buf), "Hz: gyro=%.0f flight=%.0f",
                   g_gyro.getHz(), g_flight.getHz());
          g_mr.debugLog(buf);
        }
#endif
      }

      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }, "wifi_ota", 4096, nullptr, 1, nullptr);

#ifdef DRONE_SERIAL_DEBUG
  xTaskCreate(debugDashboardTask, "debug_dash", 4096, nullptr, 1, nullptr);
#endif

  DRONE_PRINTF("[Drone] Free heap: %u bytes\n", ESP.getFreeHeap());
  DRONE_PRINTF("[Drone] All tasks started (Stage %d). Ready!\n",
               DRONE_DEBUG_STAGE);
}

// loop() empty — all work runs in FreeRTOS tasks (matches robot pattern).
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

#if DRONE_DEBUG_STAGE >= 2
static const char* stateToStr(Drone::DroneState s) {
  switch (s) {
    case Drone::DroneState::INIT:             return "INIT";
    case Drone::DroneState::UNARMED:          return "UNARMED";
    case Drone::DroneState::ARMED:            return "ARMED";
    case Drone::DroneState::LAUNCHING:        return "LAUNCHING";
    case Drone::DroneState::VELOCITY_CONTROL: return "VELOCITY_CONTROL";
    case Drone::DroneState::LANDING:          return "LANDING";
    case Drone::DroneState::EMERGENCY_LAND:   return "EMERGENCY_LAND";
    default:                                  return "UNKNOWN";
  }
}
#endif

#ifdef DRONE_SERIAL_DEBUG
static void debugDashboardTask(void*) {
  while (true) {
    uint32_t now = millis();

    DRONE_PRINTF("\n[%lu] ═══ Drone Status (Stage %d) ═══\n", now,
                  DRONE_DEBUG_STAGE);
    DRONE_PRINTF("  State:     %s\n", stateToStr(g_state.getState()));
    DRONE_PRINTF("  uROS:      %s\n",
                  g_mr.isConnected() ? "CONNECTED" : "WAITING");
    DRONE_PRINTF("  WiFi:      %s (%s)\n",
                  g_wifi.isConnected() ? "OK" : "DISCONNECTED",
                  WiFi.localIP().toString().c_str());

#if DRONE_DEBUG_STAGE >= 1
    {
      Drone::IMUData imu = g_gyro.getData();
      DRONE_PRINTLN("  ── IMU (BNO085) ──");
      DRONE_PRINTF("  Initialized: %s\n",
                    g_gyro.isInitialized() ? "YES" : "NO");
      DRONE_PRINTF("  Euler:     roll=%.1f  pitch=%.1f  yaw=%.1f deg\n",
                    imu.roll, imu.pitch, imu.yaw);
      DRONE_PRINTF("  Gyro:      gx=%.1f  gy=%.1f  gz=%.1f deg/s\n",
                    imu.gyro_x, imu.gyro_y, imu.gyro_z);
      DRONE_PRINTF("  Accel:     ax=%.2f  ay=%.2f  az=%.2f m/s2\n",
                    imu.accel_x, imu.accel_y, imu.accel_z);

      float yaw_norm = imu.yaw;
      const char* dir = "?";
      if (yaw_norm >= -22.5f && yaw_norm < 22.5f) dir = "FORWARD (0)";
      else if (yaw_norm >= 22.5f && yaw_norm < 67.5f) dir = "RIGHT-FWD (45)";
      else if (yaw_norm >= 67.5f && yaw_norm < 112.5f) dir = "RIGHT (90)";
      else if (yaw_norm >= 112.5f || yaw_norm < -157.5f) dir = "BACKWARD (180)";
      else if (yaw_norm >= -112.5f && yaw_norm < -67.5f) dir = "LEFT (-90)";
      else if (yaw_norm >= -67.5f && yaw_norm < -22.5f) dir = "LEFT-FWD (-45)";
      else dir = "REAR-SIDE";
      DRONE_PRINTF("  Heading:   %s  (yaw=%.1f)\n", dir, yaw_norm);
    }
#endif

#if DRONE_DEBUG_STAGE >= 2 && DRONE_ENABLE_HEIGHT
    {
      DRONE_PRINTLN("  ── Height (VL53L0X) ──");
      DRONE_PRINTF("  Initialized: %s\n",
                    g_height.isInitialized() ? "YES" : "NO");
      DRONE_PRINTF("  Altitude:  %.3f m  (valid=%s, last_valid=%lums ago)\n",
                    g_height.getAltitudeM(),
                    g_height.isValid() ? "YES" : "NO",
                    now - g_height.lastValidMs());
    }
#endif

#if DRONE_DEBUG_STAGE >= 2
    {
      DRONE_PRINTLN("  ── Flight Controller ──");
      DRONE_PRINTF("  Armed:     %s\n", g_flight.isArmed() ? "YES" : "NO");
      DRONE_PRINTF("  Override:  %s\n",
                    g_flight.isOverrideActive() ? "YES" : "NO");
      DRONE_PRINTF("  Motors:    FL=%.3f  FR=%.3f  BR=%.3f  BL=%.3f\n",
                    g_flight.getMotor(0), g_flight.getMotor(1),
                    g_flight.getMotor(2), g_flight.getMotor(3));

      Drone::EKFState ekf = g_ekf.getState();
      DRONE_PRINTLN("  ── EKF ──");
      DRONE_PRINTF("  Position:  x=%.3f  y=%.3f m\n", ekf.x, ekf.y);
      DRONE_PRINTF("  Velocity:  vx=%.3f  vy=%.3f m/s\n", ekf.vx, ekf.vy);
      DRONE_PRINTF("  Anchors:   %u configured\n", g_ekf.getAnchorCount());
    }

    {
      DRONE_PRINTLN("  ── IR ──");
      DRONE_PRINTF("  IR initialized: %s\n",
                    g_ir.isInitialized() ? "YES" : "NO");
    }

#if DRONE_ENABLE_UWB
    {
      DRONE_PRINTLN("  ── UWB ──");
      const auto& data = g_uwb.getData();
      DRONE_PRINTF("  Valid ranges: %u\n", data.num_valid_ranges);
      for (uint8_t i = 0; i < Drivers::UWBDriverData::MAX_ANCHORS; i++) {
        if (data.ranges[i].valid) {
          DRONE_PRINTF("    Anchor %u: %.1f cm\n",
                        data.ranges[i].peer_id,
                        data.ranges[i].distance_cm);
        }
      }
    }
#endif

    DRONE_PRINTLN("  ── Safety ──");
    DRONE_PRINTF("  Sensors ready: %s\n",
                  g_safety.checkSensorsReady() ? "YES" : "NO");
#endif  // DRONE_DEBUG_STAGE >= 2

    DRONE_PRINTLN("  ── System ──");
    DRONE_PRINTF("  Heap: %u / min: %u bytes\n",
                  ESP.getFreeHeap(), ESP.getMinFreeHeap());
    DRONE_PRINTF("  Uptime: %lus\n", now / 1000);

    vTaskDelay(pdMS_TO_TICKS(2000));  // Print every 2s
  }
}
#endif  // DRONE_SERIAL_DEBUG
