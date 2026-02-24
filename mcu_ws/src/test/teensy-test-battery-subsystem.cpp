/**
 * @file teensy-test-battery-subsystem.cpp
 * @brief Smoke-test for BatterySubsystem (INA219 via TCA9548A mux) on Teensy41.
 *
 * Wiring:
 *   Teensy SDA (pin 18) / SCL (pin 19) → TCA9548A SDA/SCL
 *   TCA9548A address: 0x70 (A0=A1=A2=GND)
 *   INA219 on TCA9548A channel 0 (SD0/SC0), address 0x40
 *   Battery load connected across INA219 IN+ / IN- with shunt
 *
 * micro-ROS agent (on host):
 *   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b
 * 921600
 *
 * Monitor battery topic:
 *   ros2 topic echo /mcu_robot/battery_health
 */

#include <Arduino.h>
#include <I2CBusLock.h>
#include <I2CMuxDriver.h>
#include <TeensyThreads.h>
#include <microros_manager_robot.h>
#include <robot/subsystems/BatterySubsystem.h>

// ── Hardware config
// ───────────────────────────────────────────────────────────

static constexpr uint8_t MUX_ADDR = 0x70;     // TCA9548A default address
static constexpr uint8_t MUX_CHANNEL = 0;     // Channel the INA219 is on
static constexpr uint8_t INA219_ADDR = 0x40;  // INA219 default address

// ── Globals
// ───────────────────────────────────────────────────────────────────

static Subsystem::MicrorosManagerSetup g_mr_setup("battery_test_mr");
static Subsystem::MicrorosManager g_mr(g_mr_setup);

static Drivers::I2CMuxDriverSetup g_mux_setup("i2c_mux", MUX_ADDR, Wire);
static Drivers::I2CMuxDriver g_mux(g_mux_setup);

static Drivers::I2CPowerDriverSetup g_power_setup("power_sensor", INA219_ADDR,
                                                  &g_mux, MUX_CHANNEL, Wire,
                                                  /*shuntOhm*/ 0.005f);
static Drivers::I2CPowerDriver g_power_driver(g_power_setup);

static Subsystem::BatterySubsystemSetup g_battery_setup("battery_subsystem",
                                                        &g_power_driver);
static Subsystem::BatterySubsystem g_battery(g_battery_setup);

// ── Threads
// ───────────────────────────────────────────────────────────────────

static void blink_task(void*) {
  pinMode(LED_BUILTIN, OUTPUT);
  while (true) {
    digitalWriteFast(LED_BUILTIN, HIGH);
    threads.delay(500);
    digitalWriteFast(LED_BUILTIN, LOW);
    threads.delay(500);
  }
}

// Prints battery readings and ROS connection status every 5 s.
static void print_task(void*) {
  char buf[80];
  uint32_t tick = 0;

  while (true) {
    const char* ros_state = g_mr.isConnected() ? "ROS:OK" : "ROS:--";
    snprintf(buf, sizeof(buf), "[%s #%lu] V=%.3fV  I=%.1fmA  P=%.1fmW",
             ros_state, tick, g_power_driver.getVoltage(),
             g_power_driver.getCurrentmA(), g_power_driver.getPowermW());
    Serial.println(buf);
    ++tick;
    threads.delay(5000);
  }
}

// ── Arduino entry points
// ──────────────────────────────────────────────────────

void setup() {
  Serial.begin(921600);
  if (CrashReport) {
    Serial.print(CrashReport);
    Serial.println();
    Serial.flush();
  }

  Serial.println(PSTR("\r\nBattery subsystem test — TeensyThreads\r\n"));
  Serial.println(PSTR("Hardware: INA219 on TCA9548A mux ch0"));

  I2CBus::initLocks();

  g_mr.init();

  if (!g_mux.init()) {
    Serial.println(PSTR("ERROR: TCA9548A init failed — check I2C wiring"));
  }

  if (!g_battery.init()) {
    Serial.println(
        PSTR("ERROR: INA219 init failed — check mux channel/address"));
  }

  g_mr.registerParticipant(&g_battery);

  //                            stackSize  priority  rateMs
  g_mr.beginThreaded(8192, 4);
  g_battery.beginThreaded(1024, 1, 100);
  threads.addThread(print_task, nullptr, 1024);
  threads.addThread(blink_task, nullptr, 512);

  Serial.println(PSTR("setup(): threads started."));
  Serial.flush();
}

void loop() { threads.delay(100); }
