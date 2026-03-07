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

// ── I2C mutex (shared between flight task and height task) ──
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
    g_flight, g_gyro, g_ekf
#if DRONE_ENABLE_HEIGHT
    ,
    g_height
#endif
);

// ── Safety monitor ──
static Drone::DroneSafetySubsystem g_safety(
    g_state, g_flight, g_gyro, g_mr
#if DRONE_ENABLE_HEIGHT
    ,
    g_height
#endif
);

// ── IR ROS2 wrapper ──
static Drone::DroneIRSubsystem g_drone_ir(g_ir, g_state);

// ── Debug counters ──
static uint32_t g_flight_task_count = 0;
static uint32_t g_height_task_count = 0;
static uint32_t g_uwb_task_count = 0;
static uint32_t g_safety_task_count = 0;
static uint32_t g_imu_update_count = 0;

// ════════════════════════════════════════════════════════════════
//  FreeRTOS Tasks
// ════════════════════════════════════════════════════════════════

#if DEBUG_STAGE >= 4
// Flight control task: IMU read → PID → motors, EKF predict (250Hz, core 1)
static void flightControlTask(void* /*param*/) {
  TickType_t prev_wake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000 / Config::FLIGHT_RATE_HZ);
  uint32_t last_us = micros();

  for (;;) {
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
    g_imu_update_count++;

    IMUData imu = g_gyro.getData();

    // EKF predict (world-frame accel)
    float ax_w, ay_w;
    float yaw_rad = imu.yaw * 0.01745329252f;
    g_gyro.getAccelWorld(yaw_rad, ax_w, ay_w);
    g_ekf.predict(ax_w, ay_w, dt);

    // Flight controller update
#if DRONE_ENABLE_HEIGHT
    float alt = g_height.getAltitudeM();
#else
    float alt = 0.0f;
#endif
    g_flight.update(imu, alt, dt);

    g_flight_task_count++;
    vTaskDelayUntil(&prev_wake, period);
  }
}
#endif

// Height sensor task (50Hz, core 1)
#if DEBUG_STAGE >= 3 && DRONE_ENABLE_HEIGHT
static void heightSensorTask(void* /*param*/) {
  TickType_t prev_wake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000 / Config::HEIGHT_RATE_HZ);

  for (;;) {
    xSemaphoreTake(g_i2c_mutex, portMAX_DELAY);
    g_height.update();
    xSemaphoreGive(g_i2c_mutex);
    g_height_task_count++;

    vTaskDelayUntil(&prev_wake, period);
  }
}
#endif

// UWB task (20Hz, core 0)
#if DEBUG_STAGE >= 6 && DRONE_ENABLE_UWB
static void uwbTask(void* /*param*/) {
  TickType_t prev_wake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000 / Config::UWB_RATE_HZ);

  for (;;) {
    g_uwb.update();
    g_uwb.startRanging();

    const auto& data = g_uwb.getData();
    if (data.num_valid_ranges >= 3) {
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
      g_ekf.updateUWB(distances, ids, n);
    }

    g_drone_uwb.publish();
    g_uwb_task_count++;

    vTaskDelayUntil(&prev_wake, period);
  }
}
#endif

// Safety task (10Hz, core 1)
#if DEBUG_STAGE >= 7
static void safetyTask(void* /*param*/) {
  TickType_t prev_wake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000 / Config::SAFETY_RATE_HZ);

  for (;;) {
    g_safety.update();
    g_safety_task_count++;
    vTaskDelayUntil(&prev_wake, period);
  }
}
#endif

// ════════════════════════════════════════════════════════════════
//  Arduino setup() / loop()
// ════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(921600);
  delay(2000);
  Serial.println("\r\n=== SEC26 Drone — All Subsystems Test ===");
  Serial.printf("[CONFIG] DEBUG_STAGE = %d\n", DEBUG_STAGE);
  Serial.printf("[CONFIG] DRONE_ENABLE_HEIGHT = %d\n", DRONE_ENABLE_HEIGHT);
  Serial.printf("[CONFIG] DRONE_ENABLE_UWB = %d\n", DRONE_ENABLE_UWB);

  // I2C mutex
  g_i2c_mutex = xSemaphoreCreateMutex();

  // I2C bus
  Wire.begin(Pins::I2C_SDA, Pins::I2C_SCL);
  Wire.setClock(400000);

  // ─── Stage 1: WiFi + micro-ROS + Heartbeat ──────────────────────
  Serial.println("[INIT] --- Stage 1: WiFi + micro-ROS + Heartbeat ---");
  g_wifi.init();
  g_wifi.begin();
  Serial.println("[INIT] WiFi initialized, waiting for connection...");
  {
    uint32_t wifi_start = millis();
    while (!g_wifi.isConnected()) {
      g_wifi.update();
      delay(50);
      if (millis() - wifi_start > 15000) {
        Serial.println("[INIT] WiFi timeout — continuing anyway");
        break;
      }
    }
  }
  Serial.printf("[INIT] WiFi: %s (IP: %s)\n",
                g_wifi.isConnected() ? "CONNECTED" : "TIMEOUT",
                WiFi.localIP().toString().c_str());

  ArduinoOTA.setHostname("sec26-drone-test");
  ArduinoOTA.begin();
  Serial.println("[INIT] OTA ready (sec26-drone-test)");

  g_mr.init();
  Serial.println("[INIT] MicrorosManager OK");

  g_hb.init();
  Serial.println("[INIT] Heartbeat OK");

  g_mr.registerParticipant(&g_hb);

  // ─── Stage 2: + IMU ─────────────────────────────────────────────
#if DEBUG_STAGE >= 2
  Serial.println("[INIT] --- Stage 2: + IMU (BNO085) ---");
  xSemaphoreTake(g_i2c_mutex, portMAX_DELAY);
  bool gyro_ok = g_gyro.init();
  xSemaphoreGive(g_i2c_mutex);
  Serial.printf("[INIT] IMU (BNO085 @ 0x%02X): %s\n",
                Config::IMU_I2C_ADDR, gyro_ok ? "OK" : "FAIL");
  if (gyro_ok) {
    Serial.printf("[INIT]   Rotation report:  %u us (%u Hz)\n",
                  Config::IMU_ROTATION_REPORT_US,
                  1000000 / Config::IMU_ROTATION_REPORT_US);
    Serial.printf("[INIT]   Gyroscope report: %u us (%u Hz)\n",
                  Config::IMU_GYRO_REPORT_US,
                  1000000 / Config::IMU_GYRO_REPORT_US);
    Serial.printf("[INIT]   Accel report:     %u us (%u Hz)\n",
                  Config::IMU_ACCEL_REPORT_US,
                  1000000 / Config::IMU_ACCEL_REPORT_US);
  }
#endif

  // ─── Stage 3: + Height sensor ───────────────────────────────────
#if DEBUG_STAGE >= 3 && DRONE_ENABLE_HEIGHT
  Serial.println("[INIT] --- Stage 3: + Height (VL53L0X) ---");
  xSemaphoreTake(g_i2c_mutex, portMAX_DELAY);
  bool height_ok = g_height.init();
  xSemaphoreGive(g_i2c_mutex);
  Serial.printf("[INIT] Height sensor: %s (budget=%ums, range=%.2f-%.2fm)\n",
                height_ok ? "OK" : "FAIL",
                Config::HEIGHT_TIMING_BUDGET_MS,
                Config::HEIGHT_MIN_VALID_M, Config::HEIGHT_MAX_VALID_M);
#elif DEBUG_STAGE >= 3
  Serial.println("[INIT] --- Stage 3: Height DISABLED (DRONE_ENABLE_HEIGHT=0) ---");
#endif

  // ─── Stage 4: + Flight controller + EKF ─────────────────────────
#if DEBUG_STAGE >= 4
  Serial.println("[INIT] --- Stage 4: + Flight controller + EKF ---");
  g_flight.init();
  Serial.printf("[INIT] Flight controller: OK (PWM=%uHz, %u-bit)\n",
                Config::MOTOR_PWM_FREQ, Config::MOTOR_PWM_RESOLUTION);
  Serial.printf("[INIT]   Hover throttle: %.2f\n", Config::HOVER_THROTTLE);
  Serial.printf("[INIT]   Max angles: roll=%.0f, pitch=%.0f, yaw_rate=%.0f\n",
                Config::MAX_ROLL_DEG, Config::MAX_PITCH_DEG,
                Config::MAX_YAW_RATE_DPS);

  g_ekf.init();
  Serial.println("[INIT] EKF: OK");
  Serial.printf("[INIT]   Process noise: pos=%.3f, vel=%.3f\n",
                Config::EKF_PROCESS_NOISE_POS, Config::EKF_PROCESS_NOISE_VEL);
  Serial.printf("[INIT]   Measure noise: %.3f, outlier gate: %.1fm\n",
                Config::EKF_MEASURE_NOISE_UWB, Config::EKF_OUTLIER_GATE_M);
#endif

  // ─── Stage 5: + IR transmitter ──────────────────────────────────
#if DEBUG_STAGE >= 5
  Serial.println("[INIT] --- Stage 5: + IR transmitter ---");
  g_ir.init();
  Serial.printf("[INIT] IR LED: OK (pin D%u, NEC addr=0x%02X)\n",
                Pins::IR_LED, g_ir_cfg.address);
#endif

  // ─── Stage 6: + UWB ─────────────────────────────────────────────
#if DEBUG_STAGE >= 6 && DRONE_ENABLE_UWB
  Serial.println("[INIT] --- Stage 6: + UWB (DW3000 TAG) ---");
  SPI.begin();
  bool uwb_ok = g_uwb.init();
  Serial.printf("[INIT] UWB DW3000: %s (ID=%u, CS=GPIO%u)\n",
                uwb_ok ? "OK" : "FAIL", Pins::UWB_DEVICE_ID, Pins::UWB_CS);
  if (uwb_ok) {
    uint8_t anchor_ids[] = {10, 11, 12};
    g_uwb.setTargetAnchors(anchor_ids, 3);
    g_uwb.begin();
    Serial.println("[INIT]   Target anchors: 10, 11, 12");
  }
#elif DEBUG_STAGE >= 6
  Serial.println("[INIT] --- Stage 6: UWB DISABLED (DRONE_ENABLE_UWB=0) ---");
#endif

  // ─── Stage 7: + Safety monitor ──────────────────────────────────
#if DEBUG_STAGE >= 7
  Serial.println("[INIT] --- Stage 7: + Safety monitor ---");
  Serial.printf("[INIT] Safety thresholds:\n");
  Serial.printf("[INIT]   Height timeout: %ums\n", Config::HEIGHT_TIMEOUT_MS);
  Serial.printf("[INIT]   UWB timeout:    %ums\n", Config::UWB_TIMEOUT_MS);
  Serial.printf("[INIT]   uROS timeout:   %ums\n", Config::MICROROS_TIMEOUT_MS);
  Serial.printf("[INIT]   cmd_vel timeout: %ums\n", Config::CMD_VEL_TIMEOUT_MS);
  Serial.printf("[INIT]   Alt ceiling:    %.1fm\n", Config::ALTITUDE_CEILING_M);
  Serial.printf("[INIT]   Emergency throttle: %.2f\n", Config::EMERGENCY_THROTTLE);
#endif

  // ── Register micro-ROS participants ──
  g_mr.registerParticipant(&g_state);
  g_mr.registerParticipant(&g_drone_ir);
#if DEBUG_STAGE >= 6 && DRONE_ENABLE_UWB
  g_mr.registerParticipant(&g_drone_uwb);
#endif

  g_mr.begin();
  g_hb.begin();

  // Check sensor readiness
  g_state.setAllSensorsReady(g_safety.checkSensorsReady());
  Serial.printf("[INIT] Sensor readiness: %s\n",
                g_safety.checkSensorsReady() ? "ALL READY" : "NOT READY");

  // ── Create FreeRTOS tasks ──
  Serial.println("[INIT] --- Starting FreeRTOS tasks ---");

#if DEBUG_STAGE >= 4
  xTaskCreatePinnedToCore(flightControlTask, "flight", Config::FLIGHT_TASK_STACK,
                          nullptr, Config::FLIGHT_TASK_PRIORITY, nullptr, 1);
  Serial.printf("[INIT] Task 'flight':  core=1, pri=%d, stack=%u, rate=%uHz\n",
                Config::FLIGHT_TASK_PRIORITY, Config::FLIGHT_TASK_STACK,
                Config::FLIGHT_RATE_HZ);
#endif

#if DEBUG_STAGE >= 3 && DRONE_ENABLE_HEIGHT
  xTaskCreatePinnedToCore(heightSensorTask, "height", Config::HEIGHT_TASK_STACK,
                          nullptr, Config::HEIGHT_TASK_PRIORITY, nullptr, 1);
  Serial.printf("[INIT] Task 'height':  core=1, pri=%d, stack=%u, rate=%uHz\n",
                Config::HEIGHT_TASK_PRIORITY, Config::HEIGHT_TASK_STACK,
                Config::HEIGHT_RATE_HZ);
#endif

#if DEBUG_STAGE >= 6 && DRONE_ENABLE_UWB
  xTaskCreatePinnedToCore(uwbTask, "uwb", Config::UWB_TASK_STACK, nullptr,
                          Config::UWB_TASK_PRIORITY, nullptr, 0);
  Serial.printf("[INIT] Task 'uwb':     core=0, pri=%d, stack=%u, rate=%uHz\n",
                Config::UWB_TASK_PRIORITY, Config::UWB_TASK_STACK,
                Config::UWB_RATE_HZ);
#endif

#if DEBUG_STAGE >= 7
  xTaskCreatePinnedToCore(safetyTask, "safety", Config::SAFETY_TASK_STACK,
                          nullptr, Config::SAFETY_TASK_PRIORITY, nullptr, 1);
  Serial.printf("[INIT] Task 'safety':  core=1, pri=%d, stack=%u, rate=%uHz\n",
                Config::SAFETY_TASK_PRIORITY, Config::SAFETY_TASK_STACK,
                Config::SAFETY_RATE_HZ);
#endif

  Serial.printf("[INIT] Free heap: %u bytes\n", ESP.getFreeHeap());
  Serial.println("[INIT] All tasks started. Entering main loop.\n");
}

// ── Debug output interval ──
static uint32_t s_last_debug_ms = 0;
static constexpr uint32_t DEBUG_INTERVAL_MS = 2000;  // Print every 2s
static uint32_t s_last_mr_update_ms = 0;
static uint32_t s_last_heap_print_ms = 0;

// Task count snapshots for rate calculation
static uint32_t s_prev_flight_count = 0;
static uint32_t s_prev_height_count = 0;
static uint32_t s_prev_uwb_count = 0;
static uint32_t s_prev_imu_count = 0;

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

void loop() {
  uint32_t now = millis();

  g_wifi.update();
  ArduinoOTA.handle();

  // Rate-limit micro-ROS update
  if (now - s_last_mr_update_ms >= 100) {
    g_mr.update();
    s_last_mr_update_ms = now;
  }

  g_hb.update();
  g_state.update();

  // ── Periodic debug output ──
  if (now - s_last_debug_ms >= DEBUG_INTERVAL_MS) {
    float elapsed_s = (now - s_last_debug_ms) / 1000.0f;
    s_last_debug_ms = now;

    Serial.printf("\n[%lu] ═══ Drone Status (Stage %d) ═══\n", now,
                  DEBUG_STAGE);
    Serial.printf("  State:     %s\n", stateToStr(g_state.getState()));
    Serial.printf("  uROS:      %s\n",
                  g_mr.isConnected() ? "CONNECTED" : "WAITING");
    Serial.printf("  WiFi:      %s (%s)\n",
                  g_wifi.isConnected() ? "OK" : "DISCONNECTED",
                  WiFi.localIP().toString().c_str());

#if DEBUG_STAGE >= 2
    {
      IMUData imu = g_gyro.getData();
      Serial.printf("  ── IMU (BNO085) ──\n");
      Serial.printf("  Initialized: %s\n",
                    g_gyro.isInitialized() ? "YES" : "NO");
      Serial.printf("  Euler:     roll=%.1f  pitch=%.1f  yaw=%.1f deg\n",
                    imu.roll, imu.pitch, imu.yaw);
      Serial.printf("  Gyro:      gx=%.1f  gy=%.1f  gz=%.1f deg/s\n",
                    imu.gyro_x, imu.gyro_y, imu.gyro_z);
      Serial.printf("  Accel:     ax=%.2f  ay=%.2f  az=%.2f m/s2\n",
                    imu.accel_x, imu.accel_y, imu.accel_z);

      // BNO orientation hint: the yaw angle tells which way it faces
      // 0=North, 90=East, 180/-180=South, -90=West (relative to power-on heading)
      float yaw_norm = imu.yaw;
      const char* dir = "?";
      if (yaw_norm >= -22.5f && yaw_norm < 22.5f) dir = "FORWARD (0)";
      else if (yaw_norm >= 22.5f && yaw_norm < 67.5f) dir = "RIGHT-FWD (45)";
      else if (yaw_norm >= 67.5f && yaw_norm < 112.5f) dir = "RIGHT (90)";
      else if (yaw_norm >= 112.5f || yaw_norm < -157.5f) dir = "BACKWARD (180)";
      else if (yaw_norm >= -112.5f && yaw_norm < -67.5f) dir = "LEFT (-90)";
      else if (yaw_norm >= -67.5f && yaw_norm < -22.5f) dir = "LEFT-FWD (-45)";
      else dir = "REAR-SIDE";
      Serial.printf("  Heading:   %s  (yaw=%.1f)\n", dir, yaw_norm);

      uint32_t imu_diff = g_imu_update_count - s_prev_imu_count;
      s_prev_imu_count = g_imu_update_count;
      Serial.printf("  IMU rate:  %.0f Hz (updates in last %.1fs)\n",
                    imu_diff / elapsed_s, elapsed_s);
    }
#endif

#if DEBUG_STAGE >= 3 && DRONE_ENABLE_HEIGHT
    {
      Serial.printf("  ── Height (VL53L0X) ──\n");
      Serial.printf("  Initialized: %s\n",
                    g_height.isInitialized() ? "YES" : "NO");
      Serial.printf("  Altitude:  %.3f m  (valid=%s, last_valid=%lums ago)\n",
                    g_height.getAltitudeM(),
                    g_height.isValid() ? "YES" : "NO",
                    now - g_height.lastValidMs());

      uint32_t h_diff = g_height_task_count - s_prev_height_count;
      s_prev_height_count = g_height_task_count;
      Serial.printf("  Height rate: %.0f Hz\n", h_diff / elapsed_s);
    }
#endif

#if DEBUG_STAGE >= 4
    {
      Serial.printf("  ── Flight Controller ──\n");
      Serial.printf("  Armed:     %s\n", g_flight.isArmed() ? "YES" : "NO");
      Serial.printf("  Override:  %s\n",
                    g_flight.isOverrideActive() ? "YES" : "NO");
      Serial.printf("  Motors:    FL=%.3f  FR=%.3f  BR=%.3f  BL=%.3f\n",
                    g_flight.getMotor(0), g_flight.getMotor(1),
                    g_flight.getMotor(2), g_flight.getMotor(3));

      uint32_t f_diff = g_flight_task_count - s_prev_flight_count;
      s_prev_flight_count = g_flight_task_count;
      Serial.printf("  Flight rate: %.0f Hz\n", f_diff / elapsed_s);

      EKFState ekf = g_ekf.getState();
      Serial.printf("  ── EKF ──\n");
      Serial.printf("  Position:  x=%.3f  y=%.3f m\n", ekf.x, ekf.y);
      Serial.printf("  Velocity:  vx=%.3f  vy=%.3f m/s\n", ekf.vx, ekf.vy);
      Serial.printf("  Anchors:   %u configured\n", g_ekf.getAnchorCount());
    }
#endif

#if DEBUG_STAGE >= 5
    {
      Serial.printf("  ── IR ──\n");
      Serial.printf("  IR initialized: %s\n",
                    g_ir.isInitialized() ? "YES" : "NO");
    }
#endif

#if DEBUG_STAGE >= 6 && DRONE_ENABLE_UWB
    {
      Serial.printf("  ── UWB ──\n");
      uint32_t u_diff = g_uwb_task_count - s_prev_uwb_count;
      s_prev_uwb_count = g_uwb_task_count;
      Serial.printf("  UWB rate:  %.0f Hz\n", u_diff / elapsed_s);

      const auto& data = g_uwb.getData();
      Serial.printf("  Valid ranges: %u\n", data.num_valid_ranges);
      for (uint8_t i = 0; i < Drivers::UWBDriverData::MAX_ANCHORS; i++) {
        if (data.ranges[i].valid) {
          Serial.printf("    Anchor %u: %.1f cm\n",
                        data.ranges[i].peer_id,
                        data.ranges[i].distance_cm);
        }
      }
    }
#endif

#if DEBUG_STAGE >= 7
    Serial.printf("  ── Safety ──\n");
    Serial.printf("  Sensors ready: %s\n",
                  g_safety.checkSensorsReady() ? "YES" : "NO");
    Serial.printf("  Safety checks: %lu\n", g_safety_task_count);
#endif

    Serial.printf("  ── System ──\n");
    Serial.printf("  Heap: %u / min: %u bytes\n",
                  ESP.getFreeHeap(), ESP.getMinFreeHeap());
    Serial.printf("  Uptime: %lus\n", now / 1000);
  }

  delay(10);
}
