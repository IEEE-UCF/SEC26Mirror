/**
 * @file teensy-test-oled-subsystem.cpp
 * @brief Smoke-test for OLEDSubsystem (serial-terminal mode) on Teensy41.
 *
 * Wiring (software SPI):
 *   SSD1306 D1 (MOSI) →  Teensy pin 11
 *   SSD1306 D0 (SCK)  →  Teensy pin 13
 *   SSD1306 DC        →  Teensy pin  9
 *   SSD1306 RST       →  Teensy pin  3
 *   SSD1306 CS        →  Teensy pin 10
 *   SSD1306 VCC       →  3.3 V
 *   SSD1306 GND       →  GND
 *
 * micro-ROS agent (on host):
 *   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b
 * 6000000
 *
 * Append a line from ROS2:
 *   ros2 service call /mcu_robot/lcd/append mcu_msgs/srv/LCDAppend "text:
 * 'Hello'"
 *
 * Scroll up (see older lines):
 *   ros2 topic pub --once /mcu_robot/lcd/scroll std_msgs/msg/Int8 "data: -1"
 *
 * Scroll down (back to live):
 *   ros2 topic pub --once /mcu_robot/lcd/scroll std_msgs/msg/Int8 "data: 1"
 */

#include <Arduino.h>
#include <TeensyThreads.h>
#include <microros_manager_robot.h>
#include <robot/subsystems/OLEDSubsystem.h>

// ── Pin config (software SPI)
// ─────────────────────────────────────────────────

static constexpr int8_t OLED_MOSI_PIN = 11;
static constexpr int8_t OLED_CLK_PIN = 13;
static constexpr int8_t OLED_DC_PIN = 9;
static constexpr int8_t OLED_RST_PIN = 3;
static constexpr int8_t OLED_CS_PIN = 10;

// ── Globals
// ───────────────────────────────────────────────────────────────────

static Subsystem::MicrorosManagerSetup g_mr_setup("oled_test_mr");
static Subsystem::MicrorosManager g_mr(g_mr_setup);

static Subsystem::OLEDSubsystemSetup g_oled_setup("oled_subsystem",
                                                  OLED_MOSI_PIN, OLED_CLK_PIN,
                                                  OLED_DC_PIN, OLED_RST_PIN,
                                                  OLED_CS_PIN);
static Subsystem::OLEDSubsystem g_oled(g_oled_setup);

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

// Appends uptime and micro-ROS status lines every 5 s.
static void heartbeat_task(void*) {
  char buf[Subsystem::OLEDSubsystem::MAX_LINE_LEN + 1];
  uint32_t tick = 0;

  while (true) {
    uint32_t now = millis();
    uint32_t secs = now / 1000;
    uint32_t mins = secs / 60;
    secs %= 60;
    const char* ros_state = g_mr.isConnected() ? "ROS:OK" : "ROS:--";
    snprintf(buf, sizeof(buf), "%s up%02lum%02lus #%lu", ros_state, mins, secs,
             tick);
    g_oled.appendText(buf);
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

  Serial.println(PSTR("\r\nOLED subsystem test — TeensyThreads\r\n"));

  g_mr.init();
  if (!g_oled.init()) {
    Serial.println(PSTR("ERROR: OLED init failed — check SPI wiring"));
  }

  g_oled.appendText("SEC26 OLED test");
  g_oled.appendText("Connecting...");

  g_mr.registerParticipant(&g_oled);

  g_mr.beginThreaded(8192, 4);
  g_oled.beginThreaded(2048, 1, 50);
  threads.addThread(heartbeat_task, nullptr, 1024);
  threads.addThread(blink_task, nullptr, 512);

  Serial.println(PSTR("setup(): threads started."));
  Serial.flush();
}

void loop() { threads.delay(100); }
