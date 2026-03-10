/**
 * @file esp32-test-all-drone-subsystems.cpp
 * @brief Integration test — wires every drone subsystem to micro-ROS for
 *        hardware validation and regression testing.
 * @date 2026-03-07
 *
 * === DEBUG STAGE BRING-UP ===
 * Set DEBUG_STAGE to incrementally enable subsystems:
 *   1 = WiFi + micro-ROS + Heartbeat only
 *   2 = + IMU (BNO085)
 *   3 = + Height sensor (VL53L0X)
 *   4 = + Flight controller (PID cascaded) + EKF
 *   5 = + IR transmitter
 *   6 = + UWB
 *   7 = + Safety monitor (full system)
 *
 * Workflow: change DEBUG_STAGE, flash, verify via serial + `ros2 topic hz`
 *
 * Hardware expected (see DronePins.h for pin map):
 *   I2C (D4/D5) — BNO085 IMU (0x4A), VL53L0X height (0x29)
 *   GPIO D0-D3  — Motor PWM (FL/FR/BR/BL)
 *   GPIO D6     — IR LED transmitter
 *   GPIO D7     — BNO085 reset
 *   SPI         — DW3000 UWB tag (CS=GPIO21)
 *
 * micro-ROS topics published:
 *   /mcu_drone/heartbeat          std_msgs/String          1 Hz
 *   /mcu_drone/state              mcu_msgs/DroneState     10 Hz
 *   /mcu_drone/uwb/ranging        mcu_msgs/UWBRanging    ~20 Hz
 *
 * micro-ROS services:
 *   /mcu_drone/arm                mcu_msgs/srv/DroneArm
 *   /mcu_drone/takeoff            mcu_msgs/srv/DroneTakeoff
 *   /mcu_drone/land               mcu_msgs/srv/DroneLand
 *   /mcu_drone/set_anchors        mcu_msgs/srv/DroneSetAnchors
 *   /mcu_drone/set_motors         mcu_msgs/srv/DroneSetMotors
 *   /mcu_drone/transmit_ir        mcu_msgs/srv/DroneTransmitIR
 *
 * micro-ROS subscriptions:
 *   /mcu_drone/cmd_vel            geometry_msgs/Twist
 */

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESP32WifiSubsystem.h>
#include <SPI.h>
#include <WiFi.h>
#include <Wire.h>
#include <microros_manager_robot.h>

#include "drone/DroneDebug.h"
#include "drone/DronePins.h"
#include "drone/DroneConfig.h"
#include "robot/machines/HeartbeatSubsystem.h"

// Subsystems
#include "drone/subsystems/GyroSubsystem.h"
#include "drone/subsystems/DroneFlightSubsystem.h"
#include "drone/subsystems/DroneEKFSubsystem.h"
#include "drone/subsystems/DroneStateSubsystem.h"
#include "drone/subsystems/DroneSafetySubsystem.h"
#include "drone/subsystems/DroneIRSubsystem.h"
#include "drone/subsystems/IRSubsystem.h"

#if DRONE_ENABLE_HEIGHT
#include "drone/subsystems/HeightSubsystem.h"
#endif

#if DRONE_ENABLE_UWB
#include <UWBDriver.h>
#include "drone/subsystems/DroneUWBSubsystem.h"
#endif

using namespace Subsystem;
using namespace Drone;

// ═══════════════════════════════════════════════════════════════════════════
//  DEBUG STAGE — change this value to control which subsystems are active
//  Stage 1: WiFi + micro-ROS + Heartbeat
//  Stage 2: + IMU (BNO085)
//  Stage 3: + Height sensor (VL53L0X)
//  Stage 4: + Flight controller + EKF
//  Stage 5: + IR transmitter
//  Stage 6: + UWB
//  Stage 7: + Safety monitor (full system)
// ═══════════════════════════════════════════════════════════════════════════
#ifndef DEBUG_STAGE
#define DEBUG_STAGE 7
#endif

// ── I2C mutex (shared between gyro and height tasks) ──
static SemaphoreHandle_t g_i2c_mutex = nullptr;

// ── WiFi subsystem ──
static IPAddress g_local_ip(LOCAL_IP);
static ESP32WifiSubsystemSetup g_wifi_setup(
    "wifi", WIFI_SSID, WIFI_PASSWORD, g_local_ip,
    IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0),
    10000, 5000, 0);
static ESP32WifiSubsystem g_wifi(g_wifi_setup);

// ── micro-ROS manager ──
static MicrorosManagerSetup g_mr_setup("microros_manager", "drone_manager",
                                       "/mcu_drone/debug");
static MicrorosManager g_mr(g_mr_setup);

// ── Heartbeat ──
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

// ── Forward declarations ──
static const char* stateToStr(Drone::DroneState s);
#ifdef DRONE_SERIAL_DEBUG
static void debugDashboardTask(void*);
#endif

// ════════════════════════════════════════════════════════════════
//  Arduino setup() / loop()
// ════════════════════════════════════════════════════════════════

void setup() {
#ifdef DRONE_SERIAL_DEBUG
  Serial.begin(921600);
  delay(2000);
#endif
  DRONE_PRINTLN("\r\n=== SEC26 Drone — All Subsystems Test ===");
  DRONE_PRINTF("[CONFIG] DEBUG_STAGE = %d\n", DEBUG_STAGE);
  DRONE_PRINTF("[CONFIG] DRONE_ENABLE_HEIGHT = %d\n", DRONE_ENABLE_HEIGHT);
  DRONE_PRINTF("[CONFIG] DRONE_ENABLE_UWB = %d\n", DRONE_ENABLE_UWB);

  // I2C mutex
  g_i2c_mutex = xSemaphoreCreateMutex();

  // I2C bus
  Wire.begin(Pins::I2C_SDA, Pins::I2C_SCL);
  Wire.setClock(400000);

  // ─── Stage 1: WiFi + micro-ROS + Heartbeat ──────────────────────
  DRONE_PRINTLN("[INIT] --- Stage 1: WiFi + micro-ROS + Heartbeat ---");
  g_wifi.init();
  g_wifi.begin();
  DRONE_PRINTLN("[INIT] WiFi initialized, waiting for connection...");
  {
    uint32_t wifi_start = millis();
    while (!g_wifi.isConnected()) {
      g_wifi.update();
      delay(50);
      if (millis() - wifi_start > 15000) {
        DRONE_PRINTLN("[INIT] WiFi timeout — continuing anyway");
        break;
      }
    }
  }
  DRONE_PRINTF("[INIT] WiFi: %s (IP: %s)\n",
                g_wifi.isConnected() ? "CONNECTED" : "TIMEOUT",
                WiFi.localIP().toString().c_str());

  ArduinoOTA.setHostname("sec26-drone-test");
  ArduinoOTA.begin();
  DRONE_PRINTLN("[INIT] OTA ready (sec26-drone-test)");

  g_mr.init();
  DRONE_PRINTLN("[INIT] MicrorosManager OK");

  g_hb.init();
  DRONE_PRINTLN("[INIT] Heartbeat OK");

  g_mr.registerParticipant(&g_hb);

  // ─── Stage 2: + IMU ─────────────────────────────────────────────
#if DEBUG_STAGE >= 2
  DRONE_PRINTLN("[INIT] --- Stage 2: + IMU (BNO085) ---");
  xSemaphoreTake(g_i2c_mutex, portMAX_DELAY);
  bool gyro_ok = g_gyro.init();
  xSemaphoreGive(g_i2c_mutex);
  DRONE_PRINTF("[INIT] IMU (BNO085 @ 0x%02X): %s\n",
                Config::IMU_I2C_ADDR, gyro_ok ? "OK" : "FAIL");
  if (gyro_ok) {
    DRONE_PRINTF("[INIT]   Rotation report:  %u us (%u Hz)\n",
                  Config::IMU_ROTATION_REPORT_US,
                  1000000 / Config::IMU_ROTATION_REPORT_US);
    DRONE_PRINTF("[INIT]   Gyroscope report: %u us (%u Hz)\n",
                  Config::IMU_GYRO_REPORT_US,
                  1000000 / Config::IMU_GYRO_REPORT_US);
    DRONE_PRINTF("[INIT]   Accel report:     %u us (%u Hz)\n",
                  Config::IMU_ACCEL_REPORT_US,
                  1000000 / Config::IMU_ACCEL_REPORT_US);
  }
#endif

  // ─── Stage 3: + Height sensor ───────────────────────────────────
#if DEBUG_STAGE >= 3 && DRONE_ENABLE_HEIGHT
  DRONE_PRINTLN("[INIT] --- Stage 3: + Height (VL53L0X) ---");
  xSemaphoreTake(g_i2c_mutex, portMAX_DELAY);
  bool height_ok = g_height.init();
  xSemaphoreGive(g_i2c_mutex);
  DRONE_PRINTF("[INIT] Height sensor: %s (budget=%ums, range=%.2f-%.2fm)\n",
                height_ok ? "OK" : "FAIL",
                Config::HEIGHT_TIMING_BUDGET_MS,
                Config::HEIGHT_MIN_VALID_M, Config::HEIGHT_MAX_VALID_M);
#elif DEBUG_STAGE >= 3
  DRONE_PRINTLN("[INIT] --- Stage 3: Height DISABLED (DRONE_ENABLE_HEIGHT=0) ---");
#endif

  // ─── Stage 4: + Flight controller + EKF ─────────────────────────
#if DEBUG_STAGE >= 4
  DRONE_PRINTLN("[INIT] --- Stage 4: + Flight controller + EKF ---");
  g_flight.init();
  DRONE_PRINTF("[INIT] Flight controller: OK (PWM=%uHz, %u-bit)\n",
                Config::MOTOR_PWM_FREQ, Config::MOTOR_PWM_RESOLUTION);
  DRONE_PRINTF("[INIT]   Hover throttle: %.2f\n", Config::HOVER_THROTTLE);
  DRONE_PRINTF("[INIT]   Max angles: roll=%.0f, pitch=%.0f, yaw_rate=%.0f\n",
                Config::MAX_ROLL_DEG, Config::MAX_PITCH_DEG,
                Config::MAX_YAW_RATE_DPS);

  g_ekf.init();
  DRONE_PRINTLN("[INIT] EKF: OK");
  DRONE_PRINTF("[INIT]   Process noise: pos=%.3f, vel=%.3f\n",
                Config::EKF_PROCESS_NOISE_POS, Config::EKF_PROCESS_NOISE_VEL);
  DRONE_PRINTF("[INIT]   Measure noise: %.3f, Mahalanobis gate²: %.1f\n",
                Config::EKF_MEASURE_NOISE_UWB, Config::EKF_MAHALANOBIS_GATE_SQ);
#endif

  // ─── Stage 5: + IR transmitter ──────────────────────────────────
#if DEBUG_STAGE >= 5
  DRONE_PRINTLN("[INIT] --- Stage 5: + IR transmitter ---");
  g_ir.init();
  DRONE_PRINTF("[INIT] IR LED: OK (pin D%u, NEC addr=0x%02X)\n",
                Pins::IR_LED, g_ir_cfg.address);
#endif

  // ─── Stage 6: + UWB ─────────────────────────────────────────────
#if DEBUG_STAGE >= 6 && DRONE_ENABLE_UWB
  DRONE_PRINTLN("[INIT] --- Stage 6: + UWB (DW3000 TAG) ---");
  SPI.begin();
  bool uwb_ok = g_uwb.init();
  DRONE_PRINTF("[INIT] UWB DW3000: %s (ID=%u, CS=GPIO%u)\n",
                uwb_ok ? "OK" : "FAIL", Pins::UWB_DEVICE_ID, Pins::UWB_CS);
  if (uwb_ok) {
    uint8_t anchor_ids[] = {10, 11, 12};
    g_uwb.setTargetAnchors(anchor_ids, 3);
    g_uwb.begin();
    DRONE_PRINTLN("[INIT]   Target anchors: 10, 11, 12");
  }
#elif DEBUG_STAGE >= 6
  DRONE_PRINTLN("[INIT] --- Stage 6: UWB DISABLED (DRONE_ENABLE_UWB=0) ---");
#endif

  // ─── Stage 7: + Safety monitor ──────────────────────────────────
#if DEBUG_STAGE >= 7
  DRONE_PRINTLN("[INIT] --- Stage 7: + Safety monitor ---");
  DRONE_PRINTLN("[INIT] Safety thresholds:");
  DRONE_PRINTF("[INIT]   Height timeout: %ums\n", Config::HEIGHT_TIMEOUT_MS);
  DRONE_PRINTF("[INIT]   UWB timeout:    %ums\n", Config::UWB_TIMEOUT_MS);
  DRONE_PRINTF("[INIT]   uROS timeout:   %ums\n", Config::MICROROS_TIMEOUT_MS);
  DRONE_PRINTF("[INIT]   cmd_vel timeout: %ums\n", Config::CMD_VEL_TIMEOUT_MS);
  DRONE_PRINTF("[INIT]   Alt ceiling:    %.1fm\n", Config::ALTITUDE_CEILING_M);
  DRONE_PRINTF("[INIT]   Emergency throttle: %.2f\n", Config::EMERGENCY_THROTTLE);
#endif

  // ── Register micro-ROS participants ──
  g_mr.registerParticipant(&g_state);
  g_mr.registerParticipant(&g_drone_ir);
#if DEBUG_STAGE >= 6 && DRONE_ENABLE_UWB
  g_mr.registerParticipant(&g_drone_uwb);
#endif

  // Check sensor readiness
  g_state.setAllSensorsReady(g_safety.checkSensorsReady());
  DRONE_PRINTF("[INIT] Sensor readiness: %s\n",
                g_safety.checkSensorsReady() ? "ALL READY" : "NOT READY");

  // ── Start RTOSSubsystem tasks ──
  DRONE_PRINTLN("[INIT] --- Starting RTOSSubsystem tasks ---");

#if DEBUG_STAGE >= 2
  g_gyro.beginThreadedPinned(Config::GYRO_TASK_STACK, 5, 4, 1);
  DRONE_PRINTF("[INIT] Task 'gyro':    core=1, pri=5, stack=%u, rate=250Hz (precise)\n",
                Config::GYRO_TASK_STACK);
#endif

#if DEBUG_STAGE >= 4
  g_flight.beginThreadedPinned(2048, 4, 4, 1);
  DRONE_PRINTF("[INIT] Task 'flight':  core=1, pri=4, rate=250Hz (precise)\n");
#endif

#if DEBUG_STAGE >= 3 && DRONE_ENABLE_HEIGHT
  g_height.beginThreadedPinned(Config::HEIGHT_TASK_STACK, 3, 20, 1);
  DRONE_PRINTF("[INIT] Task 'height':  core=1, pri=3, stack=%u, rate=50Hz (precise)\n",
                Config::HEIGHT_TASK_STACK);
#endif

#if DEBUG_STAGE >= 6 && DRONE_ENABLE_UWB
  g_drone_uwb.beginThreadedPinned(2048, 3, 50, 0);
  DRONE_PRINTF("[INIT] Task 'uwb':     core=0, pri=3, rate=20Hz (precise)\n");
#endif

#if DEBUG_STAGE >= 7
  g_state.beginThreaded(2048, 2, 100);
  g_safety.beginThreaded(2048, 3, 100);
  DRONE_PRINTLN("[INIT] Tasks 'drone_state' (10Hz) + 'drone_safety' (10Hz) started");
#endif

  // micro-ROS + heartbeat: threaded (matches robot pattern)
  g_mr.beginThreaded(8192, 4, 10);
  DRONE_PRINTLN("[INIT] Task 'microros_manager': pri=4, rate=100Hz");
  g_hb.beginThreaded(2048, 1, 1000);
  DRONE_PRINTLN("[INIT] Task 'heartbeat': pri=1, rate=1Hz");

  // WiFi + OTA task (ESP32WifiSubsystem is TimedSubsystem, not RTOSSubsystem)
  xTaskCreate([](void*) {
    while (true) {
      g_wifi.update();
      ArduinoOTA.handle();
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }, "wifi_ota", 4096, nullptr, 1, nullptr);
  DRONE_PRINTLN("[INIT] Task 'wifi_ota': pri=1, rate=10Hz");

#ifdef DRONE_SERIAL_DEBUG
  xTaskCreate(debugDashboardTask, "debug_dash", 4096, nullptr, 1, nullptr);
  DRONE_PRINTLN("[INIT] Task 'debug_dash': pri=1, rate=0.5Hz");
#endif

  DRONE_PRINTF("[INIT] Free heap: %u bytes\n", ESP.getFreeHeap());
  DRONE_PRINTLN("[INIT] All tasks started. Entering main loop.\n");
}

// Drone state name helper
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

// Debug dashboard task — prints status every 2s (only with DRONE_SERIAL_DEBUG)
#ifdef DRONE_SERIAL_DEBUG
static void debugDashboardTask(void*) {
  while (true) {
    uint32_t now = millis();

    DRONE_PRINTF("\n[%lu] ═══ Drone Status (Stage %d) ═══\n", now,
                  DEBUG_STAGE);
    DRONE_PRINTF("  State:     %s\n", stateToStr(g_state.getState()));
    DRONE_PRINTF("  uROS:      %s\n",
                  g_mr.isConnected() ? "CONNECTED" : "WAITING");
    DRONE_PRINTF("  WiFi:      %s (%s)\n",
                  g_wifi.isConnected() ? "OK" : "DISCONNECTED",
                  WiFi.localIP().toString().c_str());

#if DEBUG_STAGE >= 2
    {
      IMUData imu = g_gyro.getData();
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

#if DEBUG_STAGE >= 3 && DRONE_ENABLE_HEIGHT
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

#if DEBUG_STAGE >= 4
    {
      DRONE_PRINTLN("  ── Flight Controller ──");
      DRONE_PRINTF("  Armed:     %s\n", g_flight.isArmed() ? "YES" : "NO");
      DRONE_PRINTF("  Override:  %s\n",
                    g_flight.isOverrideActive() ? "YES" : "NO");
      DRONE_PRINTF("  Motors:    FL=%.3f  FR=%.3f  BR=%.3f  BL=%.3f\n",
                    g_flight.getMotor(0), g_flight.getMotor(1),
                    g_flight.getMotor(2), g_flight.getMotor(3));

      EKFState ekf = g_ekf.getState();
      DRONE_PRINTLN("  ── EKF ──");
      DRONE_PRINTF("  Position:  x=%.3f  y=%.3f m\n", ekf.x, ekf.y);
      DRONE_PRINTF("  Velocity:  vx=%.3f  vy=%.3f m/s\n", ekf.vx, ekf.vy);
      DRONE_PRINTF("  Anchors:   %u configured\n", g_ekf.getAnchorCount());
    }
#endif

#if DEBUG_STAGE >= 5
    {
      DRONE_PRINTLN("  ── IR ──");
      DRONE_PRINTF("  IR initialized: %s\n",
                    g_ir.isInitialized() ? "YES" : "NO");
    }
#endif

#if DEBUG_STAGE >= 6 && DRONE_ENABLE_UWB
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

#if DEBUG_STAGE >= 7
    DRONE_PRINTLN("  ── Safety ──");
    DRONE_PRINTF("  Sensors ready: %s\n",
                  g_safety.checkSensorsReady() ? "YES" : "NO");
#endif

    DRONE_PRINTLN("  ── System ──");
    DRONE_PRINTF("  Heap: %u / min: %u bytes\n",
                  ESP.getFreeHeap(), ESP.getMinFreeHeap());
    DRONE_PRINTF("  Uptime: %lus\n", now / 1000);

    vTaskDelay(pdMS_TO_TICKS(2000));  // Print every 2s
  }
}
#endif  // DRONE_SERIAL_DEBUG

// loop() empty — all work runs in FreeRTOS tasks (matches robot pattern).
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
