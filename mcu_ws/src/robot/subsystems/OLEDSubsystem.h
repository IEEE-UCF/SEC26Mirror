/**
 * @file OLEDSubsystem.h
 * @date 2026-02-22
 * @brief SSD1306 128x64 OLED display subsystem for the SEC26 robot.
 *
 * The screen is split into two independent 128x32 zones so separate subsystems
 * can each own one half without interfering with each other:
 *
 *   ┌──────────────────────┐  y=0
 *   │   TOP  zone (128×32) │
 *   ├──────────────────────┤  y=32
 *   │ BOTTOM zone (128×32) │
 *   └──────────────────────┘  y=63
 *
 * ── Internal C++ API (call from any subsystem) ───────────────────────────────
 *   setTopText(text)            setBottomText(text)
 *   setTopBitmap(data, len)     setBottomBitmap(data, len)
 *   setFullText(text)           setFullBitmap(data, len)
 *
 * ── ROS2 service ─────────────────────────────────────────────────────────────
 *   /mcu_robot/oled_control  →  mcu_msgs/srv/OLEDControl
 *   Fields: zone (FULL/TOP/BOTTOM), mode (TEXT/BITMAP), text, bitmap[1024]
 *
 * ── Thread safety ────────────────────────────────────────────────────────────
 *   All public setters and the micro-ROS service callback are guarded by a
 *   FreeRTOS mutex.  The display hardware is only touched inside update(),
 *   which runs in the subsystem's own task.
 *
 * ── Sharing with other subsystems ────────────────────────────────────────────
 *   Declare g_oled as a non-static global in RobotLogic.h (remove `static`)
 *   and forward-declare it in any subsystem header that needs it:
 *
 *     extern Subsystem::OLEDSubsystem g_oled;
 *     // then call: g_oled.setBottomText("imu: 1.5 g");
 */

#ifndef OLEDSUBSYSTEM_H
#define OLEDSUBSYSTEM_H

#include <Adafruit_SSD1306.h>
#include <BaseSubsystem.h>
#include <mcu_msgs/srv/oled_control.h>
#include <microros_manager_robot.h>

#ifdef USE_FREERTOS
#include "arduino_freertos.h"
#endif

namespace Subsystem {

// ── Setup ─────────────────────────────────────────────────────────────────────

class OLEDSubsystemSetup : public Classes::BaseSetup {
 public:
  /**
   * @param id        Subsystem identifier string.
   * @param mosi_pin  MOSI / D1 pin.
   * @param clk_pin   CLK  / D0 pin.
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
  int8_t clk_pin_  = -1;
  int8_t dc_pin_   = -1;
  int8_t rst_pin_  = -1;
  int8_t cs_pin_   = -1;
};

// ── Subsystem ─────────────────────────────────────────────────────────────────

class OLEDSubsystem : public IMicroRosParticipant,
                      public Classes::BaseSubsystem {
 public:
  // ── Zone / mode constants (mirror the .srv file) ──────────────────────────
  static constexpr uint8_t ZONE_FULL   = 0;
  static constexpr uint8_t ZONE_TOP    = 1;
  static constexpr uint8_t ZONE_BOTTOM = 2;

  static constexpr uint8_t MODE_TEXT   = 0;
  static constexpr uint8_t MODE_BITMAP = 1;

  // ── Display geometry ──────────────────────────────────────────────────────
  static constexpr uint8_t DISPLAY_W = 128;
  static constexpr uint8_t DISPLAY_H = 64;
  static constexpr uint8_t HALF_H    = 32;

  // Byte counts for raw bitmaps (1-bpp, horizontal scanlines)
  static constexpr size_t BITMAP_HALF = (DISPLAY_W * HALF_H) / 8;   // 512 B
  static constexpr size_t BITMAP_FULL = (DISPLAY_W * DISPLAY_H) / 8; // 1024 B

  explicit OLEDSubsystem(const OLEDSubsystemSetup& setup);

  // ── BaseSubsystem lifecycle ───────────────────────────────────────────────
  bool        init()    override;
  void        begin()   override {}
  void        update()  override;
  void        pause()   override {}
  void        reset()   override;
  const char* getInfo() override;

  // ── IMicroRosParticipant ──────────────────────────────────────────────────
  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override;
  void onDestroy() override;

  // ── Internal display API (thread-safe, callable from any subsystem) ───────

  /** Render text in the top 128x32 half.  Newlines (\n) are honoured. */
  void setTopText(const char* text);

  /** Render text in the bottom 128x32 half. */
  void setBottomText(const char* text);

  /**
   * Render text starting at the top-left of the full 128x64 screen.
   * Text that overflows 4 lines will naturally continue into the bottom half.
   * Clears any previous top/bottom zone content.
   */
  void setFullText(const char* text);

  /**
   * Write a raw 1-bpp bitmap into the top half.
   * @param data  BITMAP_HALF (512) bytes — 128 px wide × 32 px tall,
   *              horizontal scanlines, MSB = leftmost pixel.
   * @param len   Number of bytes provided; clamped to BITMAP_HALF internally.
   */
  void setTopBitmap(const uint8_t* data, size_t len);

  /** Write a raw 1-bpp bitmap into the bottom half (512 bytes). */
  void setBottomBitmap(const uint8_t* data, size_t len);

  /**
   * Write a raw 1-bpp bitmap across the full 128x64 screen.
   * @param data  BITMAP_FULL (1024) bytes, MSB = leftmost pixel.
   * @param len   Byte count; clamped to BITMAP_FULL internally.
   */
  void setFullBitmap(const uint8_t* data, size_t len);

#ifdef USE_FREERTOS
  void beginThreaded(uint32_t stackSize, UBaseType_t priority,
                     uint32_t updateRateMs = 50) {
    task_delay_ms_ = updateRateMs;
    xTaskCreate(taskFunction, getInfo(), stackSize, this, priority, nullptr);
  }
#endif

 private:
  // ── Zone state ───────────────────────────────────────────────────────────
  struct ZoneState {
    uint8_t mode          = MODE_TEXT;
    char    text[128]     = {};       // Null-terminated; 4 lines × 21 chars
    uint8_t bitmap[BITMAP_HALF] = {}; // 128×32 px, 1-bpp
    bool    dirty         = false;
  };

  ZoneState top_;
  ZoneState bottom_;

  // Full-screen overrides (bypass zone system when active)
  bool    full_bitmap_active_ = false;
  bool    full_text_active_   = false;
  uint8_t full_bitmap_[BITMAP_FULL] = {}; // 128×64 px, 1-bpp
  char    full_text_[256]           = {}; // Up to 8 lines × ~21 chars

  // ── Render helpers ────────────────────────────────────────────────────────
  void renderTop();
  void renderBottom();
  void renderFullBitmap();
  void renderFullText();
  void flushDisplay();

  // ── micro-ROS service ─────────────────────────────────────────────────────
  void handleService(const mcu_msgs__srv__OLEDControl_Request*  req,
                     mcu_msgs__srv__OLEDControl_Response*       res);
  static void serviceCallback(const void* req, void* res, void* ctx);

  // Pre-allocated buffers for service request/response string fields
  char srv_text_buf_[128] = {};
  char srv_msg_buf_[64]   = {};

#ifdef USE_FREERTOS
  static void taskFunction(void* pv) {
    auto* self = static_cast<OLEDSubsystem*>(pv);
    self->begin();
    while (true) {
      self->update();
      vTaskDelay(pdMS_TO_TICKS(self->task_delay_ms_));
    }
  }

  SemaphoreHandle_t mutex_         = nullptr;
  uint32_t          task_delay_ms_ = 50;

  bool takeMutex() {
    return mutex_ && xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE;
  }
  void giveMutex() {
    if (mutex_) xSemaphoreGive(mutex_);
  }
#else
  bool takeMutex() { return true; }
  void giveMutex() {}
#endif

  const OLEDSubsystemSetup setup_;
  Adafruit_SSD1306         display_;

  rcl_node_t*    node_ = nullptr;
  rcl_service_t  srv_{};
  mcu_msgs__srv__OLEDControl_Request  srv_req_{};
  mcu_msgs__srv__OLEDControl_Response srv_res_{};
};

}  // namespace Subsystem

#endif  // OLEDSUBSYSTEM_H
