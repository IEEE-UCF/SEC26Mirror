/**
 * @file OLEDSubsystem.h
 * @date 2026-02-22
 * @brief SSD1306 128x64 OLED display subsystem — serial-terminal style.
 *
 * Behaves like a serial terminal: text is appended to a ring buffer and the
 * display shows a scrollable window over the most-recent lines.
 *
 * ── ROS2 interface ───────────────────────────────────────────────────────────
 *   /mcu_robot/lcd/append   subscription  std_msgs/String
 *                           Append text to the terminal (\n splits into lines)
 *
 *   /mcu_robot/lcd/scroll   subscription  std_msgs/Int8
 *                           -1 = scroll up (older), +1 = scroll down (newer)
 *
 * ── Internal C++ API (thread-safe) ───────────────────────────────────────────
 *   appendText(text)     — same effect as calling /mcu_robot/lcd/append
 *   scrollBy(delta)      — same effect as publishing to /mcu_robot/lcd/scroll
 *
 * ── Limits ───────────────────────────────────────────────────────────────────
 *   MAX_LINES    (128) ring-buffer depth; oldest lines overwritten when full
 *   MAX_LINE_LEN (21)  characters per line; longer input is truncated
 *   TEXT_BUF_CAP       max bytes per service request (MAX_LINES × 22 = 2816)
 */

#ifndef OLEDSUBSYSTEM_H
#define OLEDSUBSYSTEM_H

#include <Adafruit_SSD1306.h>
#include <BaseSubsystem.h>
#include <microros_manager_robot.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/string.h>

#ifdef USE_TEENSYTHREADS
#include <TeensyThreads.h>
#endif

namespace Subsystem {

// ── Setup
// ─────────────────────────────────────────────────────────────────────

class OLEDSubsystemSetup : public Classes::BaseSetup {
 public:
  /**
   * @param id        Subsystem identifier string.
   * @param mosi_pin  MOSI / D1 pin (software SPI).
   * @param clk_pin   CLK  / D0 pin (software SPI).
   * @param dc_pin    Data/Command select pin.
   * @param rst_pin   Reset pin, or -1 if not connected.
   * @param cs_pin    Chip-select pin.
   */
  OLEDSubsystemSetup(const char* id, int8_t mosi_pin, int8_t clk_pin,
                     int8_t dc_pin, int8_t rst_pin, int8_t cs_pin)
      : Classes::BaseSetup(id),
        mosi_pin_(mosi_pin),
        clk_pin_(clk_pin),
        dc_pin_(dc_pin),
        rst_pin_(rst_pin),
        cs_pin_(cs_pin) {}

  int8_t mosi_pin_ = -1;
  int8_t clk_pin_ = -1;
  int8_t dc_pin_ = -1;
  int8_t rst_pin_ = -1;
  int8_t cs_pin_ = -1;
};

// ── Subsystem
// ─────────────────────────────────────────────────────────────────

class OLEDSubsystem : public IMicroRosParticipant,
                      public Classes::BaseSubsystem {
 public:
  // ── Display geometry ──────────────────────────────────────────────────────
  static constexpr uint8_t DISPLAY_W = 128;
  static constexpr uint8_t DISPLAY_H = 64;
  static constexpr int LINES_VISIBLE = 8;  // 64 px / 8 px per text-size-1 row
  static constexpr int MAX_LINE_LEN = 21;  // floor(128 px / 6 px per char)
  static constexpr int MAX_LINES = 128;    // ring-buffer depth
  static constexpr int TEXT_BUF_CAP = MAX_LINES * (MAX_LINE_LEN + 1);  // 2816 B

  explicit OLEDSubsystem(const OLEDSubsystemSetup& setup);

  // ── BaseSubsystem lifecycle ───────────────────────────────────────────────
  bool init() override;
  void begin() override {}
  void update() override;
  void pause() override {}
  void reset() override;
  const char* getInfo() override;

  // ── IMicroRosParticipant ──────────────────────────────────────────────────
  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override;
  void onDestroy() override;

  // ── Internal C++ API (thread-safe) ───────────────────────────────────────

  /** Append text to the terminal. Newlines (\\n) split into separate lines. */
  void appendText(const char* text);

  /**
   * Adjust the scroll position.
   * @param delta  -1 = scroll up (older lines), +1 = scroll down (newer).
   *               Clamped to valid range automatically.
   */
  void scrollBy(int8_t delta);

  /**
   * Set a persistent status line displayed at the top of the screen (row 0).
   * When set, the scrollable terminal area shifts to rows 1–7.
   * Pass nullptr or "" to clear the status line and reclaim the full display.
   */
  void setStatusLine(const char* text);

#ifdef USE_TEENSYTHREADS
  void beginThreaded(uint32_t stackSize, int /*priority*/ = 1,
                     uint32_t updateRateMs = 50) {
    task_delay_ms_ = updateRateMs;
    threads.addThread(taskFunction, this, stackSize);
  }
#endif

 private:
  // ── Status line (persistent top row) ─────────────────────────────────────
  char status_line_[MAX_LINE_LEN + 1] = {};
  bool has_status_line_ = false;

  // ── Ring buffer ───────────────────────────────────────────────────────────
  char lines_[MAX_LINES][MAX_LINE_LEN + 1] = {};
  int next_write_ = 0;     // next write index in ring
  int total_written_ = 0;  // lines ever written, capped at MAX_LINES
  int view_offset_ = 0;    // 0 = live (newest visible); N = scrolled N back
  bool dirty_ = false;

  // ── Render helpers ────────────────────────────────────────────────────────
  void appendOneLine(const char* s, int len);
  void renderLines();
  void flushDisplay();

  // ── micro-ROS: LCD append subscription ───────────────────────────────────
  static void appendCallback(const void* msg, void* ctx);

  char lcd_text_buf_[TEXT_BUF_CAP] = {};
  std_msgs__msg__String lcd_msg_{};
  rcl_subscription_t lcd_sub_{};

  // ── micro-ROS: scroll topic ───────────────────────────────────────────────
  static void scrollCallback(const void* msg, void* ctx);

  std_msgs__msg__Int8 scroll_msg_ = {};
  rcl_subscription_t scroll_sub_ = {};

#ifdef USE_TEENSYTHREADS
  static void taskFunction(void* pv) {
    auto* self = static_cast<OLEDSubsystem*>(pv);
    self->begin();
    while (true) {
      self->update();
      threads.delay(self->task_delay_ms_);
    }
  }

  Threads::Mutex mutex_;
  uint32_t task_delay_ms_ = 50;

  bool takeMutex() {
    mutex_.lock();
    return true;
  }
  void giveMutex() { mutex_.unlock(); }
#else
  bool takeMutex() { return true; }
  void giveMutex() {}
#endif

  const OLEDSubsystemSetup setup_;
  Adafruit_SSD1306 display_;

  rcl_node_t* node_ = nullptr;
};

}  // namespace Subsystem

#endif  // OLEDSUBSYSTEM_H
