/**
 * @file teensy-test-oled-subsystem.cpp
 * @brief OLED SPI bisect test — adds one subsystem at a time via serial menu.
 *
 * Flash once, then press keys in the serial monitor to progressively enable
 * subsystems.  Watch the OLED after each step to find the conflict.
 *
 * Build & flash:
 *   pio run -e teensy-test-oled-subsystem --target upload
 *   pio device monitor -e teensy-test-oled-subsystem
 */

#include <Arduino.h>
#include <SPI.h>
#include <TeensyThreads.h>
#include <microros_manager_robot.h>

#include "robot/RobotConstants.h"
#include "robot/RobotPins.h"

// Drivers
#include "BNO085.h"
#include "I2CBusLock.h"
#include "I2CMuxDriver.h"
#include "I2CPowerDriver.h"
#include "PCA9685Manager.h"
#include "TCA9555Driver.h"
#include "TOF.h"
#include "UWBDriver.h"

// Subsystems
#include "robot/machines/HeartbeatSubsystem.h"
#include "robot/subsystems/BatterySubsystem.h"
#include "robot/subsystems/ButtonSubsystem.h"
#include "robot/subsystems/DipSwitchSubsystem.h"
#include "robot/subsystems/ImuSubsystem.h"
#include "robot/subsystems/LEDSubsystem.h"
#include "robot/subsystems/MotorManagerSubsystem.h"
#include "robot/subsystems/OLEDSubsystem.h"
#include "robot/subsystems/RCSubsystem.h"
#include "robot/subsystems/SensorSubsystem.h"
#include "robot/subsystems/ServoSubsystem.h"
#include "robot/subsystems/UWBSubsystem.h"

using namespace Subsystem;

// ═══════════════════════════════════════════════════════════════════════════
//  Global instances (same as teensy-test-all-subsystems)
// ═══════════════════════════════════════════════════════════════════════════

// --- micro-ROS manager ---
static MicrorosManagerSetup g_mr_setup("microros_manager");
static MicrorosManager g_mr(g_mr_setup);

// --- OLED (always active, hardware SPI1) ---
static OLEDSubsystemSetup g_oled_setup("oled_subsystem", &SPI1, PIN_DISP_DC,
                                       PIN_DISP_RST, PIN_DISP_CS);
static OLEDSubsystem g_oled(g_oled_setup);

// --- Heartbeat ---
static HeartbeatSubsystemSetup g_hb_setup("heartbeat_subsystem");
static HeartbeatSubsystem g_hb(g_hb_setup);

// --- Wire0: I2C mux ---
static Drivers::I2CMuxDriverSetup g_mux_setup("i2c_mux", I2C_ADDR_MUX, Wire);
static Drivers::I2CMuxDriver g_mux(g_mux_setup);

// --- Wire0: GPIO expander ---
static Drivers::TCA9555DriverSetup g_gpio_setup("gpio_expander", I2C_ADDR_GPIO,
                                                Wire);
static Drivers::TCA9555Driver g_gpio(g_gpio_setup);

// --- Wire0: Battery (INA219 on mux ch0) ---
static Drivers::I2CPowerDriverSetup g_power_setup("power_driver",
                                                  I2C_ADDR_POWER, &g_mux,
                                                  MUX_CH_BATTERY, Wire,
                                                  SHUNT_RESISTANCE_OHM);
static Drivers::I2CPowerDriver g_power_driver(g_power_setup);
static BatterySubsystemSetup g_battery_setup("battery_subsystem",
                                             &g_power_driver);
static BatterySubsystem g_battery(g_battery_setup);

// --- Wire0: Sensor (TOF) ---
static Drivers::TOFDriverSetup g_tof_setup("tof_sensor", 500, 0, Wire);
static Drivers::TOFDriver g_tof_driver(g_tof_setup);
static std::vector<Drivers::TOFDriver*> g_tof_drivers = {&g_tof_driver};
static SensorSubsystemSetup g_sensor_setup("sensor_subsystem", g_tof_drivers);
static SensorSubsystem g_sensor(g_sensor_setup);

// --- Wire1: IMU (BNO085) ---
static Drivers::BNO085DriverSetup g_imu_driver_setup("imu_driver", PIN_GYRO_RST,
                                                     Wire1);
static Drivers::BNO085Driver g_imu_driver(g_imu_driver_setup);
static ImuSubsystemSetup g_imu_setup("imu_subsystem", &g_imu_driver);
static ImuSubsystem g_imu(g_imu_setup);

// --- Wire2: PCA9685 manager ---
static Robot::PCA9685ManagerSetup g_pca_mgr_setup("pca_manager");
static Robot::PCA9685Manager g_pca_mgr(g_pca_mgr_setup);

static Robot::PCA9685Driver* g_pca_servo =
    g_pca_mgr.createDriver(Robot::PCA9685DriverSetup(
        "pca_servo", I2C_ADDR_SERVO, DEFAULT_PCA9685_FREQ, Wire2));
static Robot::PCA9685Driver* g_pca_motor =
    g_pca_mgr.createDriver(Robot::PCA9685DriverSetup(
        "pca_motor", I2C_ADDR_MOTOR, DEFAULT_PCA9685_FREQ, Wire2));

// --- RC (FlySky) ---
static RCSubsystemSetup g_rc_setup("rc_subsystem", &Serial8);
static RCSubsystem g_rc(g_rc_setup);

// --- DIP switch ---
static DipSwitchSubsystemSetup g_dip_setup("dip_switch_subsystem", &g_gpio);
static DipSwitchSubsystem g_dip(g_dip_setup);

// --- Button ---
static ButtonSubsystemSetup g_btn_setup("button_subsystem", &g_gpio);
static ButtonSubsystem g_btn(g_btn_setup);

// --- LED (WS2812B) ---
static LEDSubsystemSetup g_led_setup("led_subsystem", PIN_RGB_LEDS,
                                     NUM_RGB_LEDS);
static LEDSubsystem g_led(g_led_setup);

// --- Servo ---
static ServoSubsystemSetup g_servo_setup("servo_subsystem", g_pca_servo,
                                         PIN_SERVO_OE, NUM_SERVOS);
static ServoSubsystem g_servo(g_servo_setup);

// --- Motor ---
static MotorManagerSubsystemSetup g_motor_setup("motor_subsystem", g_pca_motor,
                                                PIN_MOTOR_OE, NUM_MOTORS);
static MotorManagerSubsystem g_motor(g_motor_setup);

// --- UWB ---
static Drivers::UWBDriverSetup g_uwb_driver_setup("uwb_driver",
                                                  Drivers::UWBMode::TAG,
                                                  ROBOT_UWB_TAG_ID, PIN_UWB_CS);
static Drivers::UWBDriver g_uwb_driver(g_uwb_driver_setup);
static UWBSubsystemSetup g_uwb_setup("uwb_subsystem", &g_uwb_driver,
                                     ROBOT_UWB_TOPIC);
static UWBSubsystem g_uwb(g_uwb_setup);

// --- PCA9685 flush task ---
static bool pca_thread_started = false;
static void pca_task(void*) {
  while (true) {
    g_pca_mgr.update();
    threads.delay(20);
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Step table — each entry adds one subsystem
// ═══════════════════════════════════════════════════════════════════════════

struct Step {
  const char* name;
  void (*run)();
};

static int current_step = 0;

// Helper: print to both serial and OLED
static void announce(const char* msg) {
  Serial.printf("  >> %s\n", msg);
  g_oled.appendText(msg);
}

// --- Step functions ---

static void step_microros() {
  g_mr.init();
  g_mr.registerParticipant(&g_oled);
  g_mr.beginThreaded(8192, 4);
  announce("+ micro-ROS manager");
}

static void step_heartbeat() {
  g_hb.init();
  g_mr.registerParticipant(&g_hb);
  g_hb.beginThreaded(1024, 1, 200);
  announce("+ Heartbeat (5 Hz)");
}

static void step_i2c_bus() {
  I2CBus::initLocks();
  pinMode(PIN_MUX_RESET, OUTPUT);
  digitalWrite(PIN_MUX_RESET, HIGH);
  g_mux.init();
  g_gpio.init();
  announce("+ I2C bus + mux + GPIO");
}

static void step_battery() {
  g_battery.init();
  g_battery.setOLED(&g_oled);
  g_mr.registerParticipant(&g_battery);
  g_battery.beginThreaded(1024, 1, 100);
  announce("+ Battery (10 Hz)");
}

static void step_sensor() {
  g_sensor.init();
  g_mr.registerParticipant(&g_sensor);
  g_sensor.beginThreaded(1024, 1, 100);
  announce("+ Sensor/TOF (10 Hz)");
}

static void step_imu() {
  g_imu.init();
  g_mr.registerParticipant(&g_imu);
  g_imu.beginThreaded(2048, 3, 20);
  announce("+ IMU (50 Hz, pri 3)");
}

static void step_rc() {
  g_rc.init();
  g_mr.registerParticipant(&g_rc);
  g_rc.beginThreaded(1024, 3, 5);
  announce("+ RC (200 Hz, pri 3)");
}

static void step_pca9685() {
  g_pca_mgr.init();
  threads.addThread(pca_task, nullptr, 1024);
  pca_thread_started = true;
  announce("+ PCA9685 flush (50 Hz)");
}

static void step_servo() {
  g_servo.init();
  g_mr.registerParticipant(&g_servo);
  g_servo.beginThreaded(1024, 2, 50);
  announce("+ Servo (20 Hz, pri 2)");
}

static void step_motor() {
  g_motor.init();
  g_mr.registerParticipant(&g_motor);
  g_motor.beginThreaded(1024, 2, 50);
  announce("+ Motor (20 Hz, pri 2)");
}

static void step_dip_switch() {
  g_dip.init();
  g_mr.registerParticipant(&g_dip);
  g_dip.beginThreaded(1024, 1, 500);
  announce("+ DipSwitch (2 Hz)");
}

static void step_button() {
  g_btn.init();
  g_mr.registerParticipant(&g_btn);
  g_btn.beginThreaded(1024, 1, 20);
  announce("+ Button (50 Hz)");
}

static void step_led() {
  g_led.init();
  g_led.setAll(0, 32, 0);
  FastLED.show();
  g_mr.registerParticipant(&g_led);
  g_led.beginThreaded(1024, 1, 50);
  announce("+ LED/WS2812B (20 Hz)");
}

static void step_uwb() {
  SPI.begin();
  g_uwb.init();
  g_uwb_driver.setTargetAnchors(ROBOT_UWB_ANCHOR_IDS, ROBOT_UWB_NUM_ANCHORS);
  g_mr.registerParticipant(&g_uwb);
  g_uwb.beginThreaded(2048, 2, 50);
  announce("+ UWB SPI0 (20 Hz, pri 2)");
}

// Step table — ordered to match typical conflict suspicion (most likely last)
static Step steps[] = {
    {"micro-ROS manager", step_microros},
    {"Heartbeat", step_heartbeat},
    {"I2C bus/mux/GPIO", step_i2c_bus},
    {"Battery (INA219)", step_battery},
    {"Sensor (TOF)", step_sensor},
    {"IMU (BNO085)", step_imu},
    {"RC (FlySky)", step_rc},
    {"PCA9685 flush", step_pca9685},
    {"Servo", step_servo},
    {"Motor", step_motor},
    {"DipSwitch", step_dip_switch},
    {"Button", step_button},
    {"LED (WS2812B)", step_led},
    {"UWB (DW3000 SPI0)", step_uwb},
};
static constexpr int NUM_STEPS = sizeof(steps) / sizeof(steps[0]);

// ═══════════════════════════════════════════════════════════════════════════
//  Menu
// ═══════════════════════════════════════════════════════════════════════════

static void printMenu() {
  Serial.println();
  Serial.println("─────────────────────────────────────────");
  Serial.printf("  Step %d/%d", current_step, NUM_STEPS);
  if (current_step < NUM_STEPS) {
    Serial.printf("  —  next: %s", steps[current_step].name);
  } else {
    Serial.print("  —  ALL subsystems enabled");
  }
  Serial.println();
  Serial.println("─────────────────────────────────────────");
  Serial.println("  [n] Add next subsystem");
  Serial.println("  [a] Add ALL remaining subsystems");
  Serial.println("  [r] Reboot (start over)");
  Serial.println("  [t] Write test line to OLED");
  Serial.println("─────────────────────────────────────────");
  Serial.flush();
}

static void addNext() {
  if (current_step >= NUM_STEPS) {
    Serial.println("  All subsystems already enabled.");
    return;
  }
  Serial.printf("\n── Adding step %d: %s ──\n", current_step + 1,
                steps[current_step].name);
  Serial.flush();
  steps[current_step].run();
  current_step++;
  Serial.println("  Done. Check if OLED still works.");
  Serial.println("  Done.");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Arduino entry points
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(921600);
  delay(1000);
  if (CrashReport) {
    Serial.print(CrashReport);
    Serial.println();
    Serial.flush();
  }

  Serial.println("\r\n═══════════════════════════════════════════");
  Serial.println("  SEC26 OLED SPI Bisect Test");
  Serial.println("  Add subsystems one at a time to find");
  Serial.println("  what breaks the OLED.");
  Serial.println("═══════════════════════════════════════════\n");

  pinMode(LED_BUILTIN, OUTPUT);

  // OLED is always on — this is the baseline
  Serial.print("Initializing OLED... ");
  if (!g_oled.init()) {
    Serial.println("FAILED — check wiring. Halting.");
    while (true) {
      delay(100);
    }
  }
  Serial.println("OK");

  g_oled.appendText("OLED bisect test");
  g_oled.appendText("Press 'n' in serial");
  g_oled.appendText("to add subsystems");
  g_oled.setStatusLine("step 0 / OLED only");

  // Start OLED thread
  g_oled.beginThreaded(2048, 1, 50);

  printMenu();
}

static uint32_t last_tick = 0;
static uint32_t tick_count = 0;

void loop() {
  // Handle serial input
  if (Serial.available()) {
    char c = Serial.read();
    while (Serial.available()) Serial.read();  // drain

    switch (c) {
      case 'n':
      case 'N':
        addNext();
        if (current_step <= NUM_STEPS) {
          char status[OLEDSubsystem::MAX_LINE_LEN + 1];
          snprintf(status, sizeof(status), "step %d/%d", current_step,
                   NUM_STEPS);
          g_oled.setStatusLine(status);
        }
        printMenu();
        break;

      case 'a':
      case 'A':
        Serial.println("\n── Adding ALL remaining subsystems ──");
        while (current_step < NUM_STEPS) {
          Serial.printf("  step %d: %s... ", current_step + 1,
                        steps[current_step].name);
          steps[current_step].run();
          current_step++;
          Serial.println("done");
          delay(100);
        }
        g_oled.setStatusLine("ALL enabled");
        Serial.println("  Done.");
        printMenu();
        break;

      case 'r':
      case 'R':
        Serial.println("Rebooting...");
        Serial.flush();
        delay(100);
        SCB_AIRCR = 0x05FA0004;  // Teensy software reset
        break;

      case 't':
      case 'T': {
        char buf[OLEDSubsystem::MAX_LINE_LEN + 1];
        snprintf(buf, sizeof(buf), "test @%lus", millis() / 1000);
        g_oled.appendText(buf);
        Serial.printf("  Wrote: %s\n", buf);
        break;
      }
    }
  }

  // Periodic uptime line every 5s to keep OLED visibly updating
  if (millis() - last_tick > 5000) {
    last_tick = millis();
    char buf[OLEDSubsystem::MAX_LINE_LEN + 1];
    snprintf(buf, sizeof(buf), "up %lus #%lu", millis() / 1000, tick_count++);
    g_oled.appendText(buf);
  }

  digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));
  delay(500);
}
