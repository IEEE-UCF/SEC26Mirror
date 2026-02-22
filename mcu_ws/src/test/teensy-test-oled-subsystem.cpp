/**
 * @file teensy-test-oled-subsystem.cpp
 * @brief Minimal smoke-test for OLEDSubsystem on Teensy41.
 *
 * Wiring (4-wire hardware SPI — Teensy 4.1 SPI0):
 *   SSD1306 D1 (MOSI) →  Teensy pin 11  (hardware)
 *   SSD1306 D0 (SCK)  →  Teensy pin 13  (hardware)
 *   SSD1306 DC        →  Teensy pin 12  (OLED_DC_PIN)
 *   SSD1306 CS        →  Teensy pin 10  (OLED_CS_PIN)
 *   SSD1306 VCC       →  3.3V
 *   SSD1306 GND       →  GND
 *   (no RST pin on this module)
 *
 * micro-ROS agent (on host):
 *   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 6000000
 *
 * Screen layout while running:
 *   ┌──────────────────────────┐
 *   │ [TOP]  micro-ROS status  │  ← updated by status_task
 *   ├──────────────────────────┤
 *   │ [BOT]  uptime counter    │  ← updated by counter_task
 *   └──────────────────────────┘
 *
 * After DEMO_BITMAP_AFTER_MS the bottom zone switches to a checkerboard
 * bitmap to verify MODE_BITMAP rendering, then reverts to text.
 *
 * ROS2 service test (once agent is connected):
 *   ros2 service call /mcu_robot/oled_control mcu_msgs/srv/OLEDControl \
 *     "{zone: 0, mode: 0, text: 'Hello\nfrom ROS2!'}"
 */

#include <Arduino.h>
#include <microros_manager_robot.h>
#include <robot/subsystems/OLEDSubsystem.h>

#include "arduino_freertos.h"

// ── Pin config (software SPI) ─────────────────────────────────────────────────

static constexpr int8_t OLED_MOSI_PIN = 11;
static constexpr int8_t OLED_CLK_PIN  = 13;
static constexpr int8_t OLED_DC_PIN   = 9;
static constexpr int8_t OLED_RST_PIN  = 3;
static constexpr int8_t OLED_CS_PIN   = 10;

static constexpr uint32_t DEMO_BITMAP_AFTER_MS = 15000;  // show bitmap at 15 s
static constexpr uint32_t DEMO_BITMAP_DUR_MS   = 5000;   // bitmap for 5 s

// ── Globals ───────────────────────────────────────────────────────────────────

static Subsystem::MicrorosManagerSetup g_mr_setup("oled_test_mr");
static Subsystem::MicrorosManager      g_mr(g_mr_setup);

static Subsystem::OLEDSubsystemSetup g_oled_setup("oled_subsystem",
                                                   OLED_MOSI_PIN,
                                                   OLED_CLK_PIN,
                                                   OLED_DC_PIN,
                                                   OLED_RST_PIN,
                                                   OLED_CS_PIN);
static Subsystem::OLEDSubsystem g_oled(g_oled_setup);

// ── Helper: build a 128×32 checkerboard bitmap ────────────────────────────────
//   Each 8×8 block alternates black/white, giving a classic test pattern.

static void fillCheckerboard(uint8_t* buf, size_t len) {
  // 128 px wide = 16 bytes per row.  Rows 0-31 → 512 bytes total.
  for (size_t byte_idx = 0; byte_idx < len; byte_idx++) {
    uint8_t row = byte_idx / 16;          // which scanline (0..31)
    uint8_t col_block = (byte_idx % 16) / 1;  // which byte in the row (0..15)
    // Invert pattern every 8 rows and every byte
    bool invert = ((row / 8) + (col_block / 1)) % 2;
    buf[byte_idx] = invert ? 0xF0 : 0x0F;
  }
}

// ── FreeRTOS tasks ────────────────────────────────────────────────────────────

static void blink_task(void*) {
  pinMode(LED_BUILTIN, OUTPUT);
  while (true) {
    digitalWriteFast(LED_BUILTIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
    digitalWriteFast(LED_BUILTIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// Updates the top zone with the current micro-ROS connection state.
static void status_task(void*) {
  while (true) {
    if (g_mr.isConnected()) {
      g_oled.setTopText("microROS: OK\nSvc ready");
    } else {
      g_oled.setTopText("microROS:\nConnecting...");
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// Updates the bottom zone with an uptime counter.
// After DEMO_BITMAP_AFTER_MS, briefly renders a checkerboard to verify bitmap.
static void counter_task(void*) {
  static uint8_t checker[Subsystem::OLEDSubsystem::BITMAP_HALF];
  fillCheckerboard(checker, sizeof(checker));

  bool bitmap_shown = false;
  bool demo_done    = false;
  uint32_t bitmap_start = 0;

  char buf[48];
  uint32_t tick = 0;

  while (true) {
    uint32_t now = millis();

    // Trigger bitmap demo once
    if (!demo_done && !bitmap_shown && now >= DEMO_BITMAP_AFTER_MS) {
      g_oled.setBottomBitmap(checker, sizeof(checker));
      bitmap_shown = true;
      bitmap_start = now;
    }

    // Revert to text after the bitmap demo period
    if (bitmap_shown && (now - bitmap_start) >= DEMO_BITMAP_DUR_MS) {
      bitmap_shown = false;
      demo_done    = true;
    }

    // Write uptime text while not in bitmap demo
    if (!bitmap_shown) {
      uint32_t secs  = now / 1000;
      uint32_t mins  = secs / 60;
      secs %= 60;
      snprintf(buf, sizeof(buf), "Up %02lum%02lus\ntick %lu", mins, secs, tick);
      g_oled.setBottomText(buf);
    }

    tick++;
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// ── Arduino entry points ──────────────────────────────────────────────────────

void setup() {
  Serial.begin(921600);
  if (CrashReport) {
    Serial.print(CrashReport);
    Serial.println();
    Serial.flush();
  }

  Serial.println(PSTR(
      "\r\nOLED subsystem test — FreeRTOS " tskKERNEL_VERSION_NUMBER "\r\n"));

  // 1. Init
  g_mr.init();
  if (!g_oled.init()) {
    Serial.println(PSTR("ERROR: OLED init failed — check SPI wiring"));
  }

  // 2. Show startup splash on the full screen before micro-ROS connects
  g_oled.setFullText("SEC26 Robot\nOLED test\nConnecting...");

  // 3. Register OLED as a micro-ROS participant (exposes the service)
  g_mr.registerParticipant(&g_oled);

  // 4. Start tasks
  g_mr.beginThreaded(8192, 4);                // micro-ROS agent — highest pri
  g_oled.beginThreaded(2048, 1, 50);          // display refresh — low pri
  xTaskCreate(status_task,  "oled_status",  512, nullptr, 2, nullptr);
  xTaskCreate(counter_task, "oled_counter", 512, nullptr, 2, nullptr);
  xTaskCreate(blink_task,   "blink",        128, nullptr, 1, nullptr);

  Serial.println(PSTR("setup(): starting scheduler..."));
  Serial.flush();

  vTaskStartScheduler();
}

void loop() {}  // never reached
