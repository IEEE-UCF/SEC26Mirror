#include "OLEDSubsystem.h"

#include <cstring>

namespace Subsystem {

// ── Constructor ───────────────────────────────────────────────────────────────

OLEDSubsystem::OLEDSubsystem(const OLEDSubsystemSetup& setup)
    : Classes::BaseSubsystem(setup),
      setup_(setup),
      display_(DISPLAY_W, DISPLAY_H, setup.mosi_pin_, setup.clk_pin_,
               setup.dc_pin_, setup.rst_pin_, setup.cs_pin_) {}

// ── Lifecycle ─────────────────────────────────────────────────────────────────

bool OLEDSubsystem::init() {
#ifdef USE_FREERTOS
  mutex_ = xSemaphoreCreateMutex();
  if (!mutex_) return false;
#endif

  if (!display_.begin(SSD1306_SWITCHCAPVCC)) {
    return false;
  }

  display_.clearDisplay();
  display_.setTextSize(1);
  display_.setTextColor(SSD1306_WHITE);
  display_.cp437(true);  // Enable full 256-char CP437 glyph set
  display_.display();
  return true;
}

void OLEDSubsystem::reset() {
  if (!takeMutex()) return;

  top_    = ZoneState{};
  bottom_ = ZoneState{};
  full_bitmap_active_ = false;
  full_text_active_   = false;

  display_.clearDisplay();
  display_.display();

  giveMutex();
}

const char* OLEDSubsystem::getInfo() {
  static const char info[] = "OLEDSubsystem";
  return info;
}

// ── Update ────────────────────────────────────────────────────────────────────

void OLEDSubsystem::update() {
  if (!takeMutex()) return;

  bool needs_flush = false;

  if (full_bitmap_active_) {
    renderFullBitmap();
    full_bitmap_active_ = false;
    needs_flush = true;
  } else if (full_text_active_) {
    renderFullText();
    full_text_active_ = false;
    needs_flush = true;
  } else {
    if (top_.dirty) {
      renderTop();
      top_.dirty  = false;
      needs_flush = true;
    }
    if (bottom_.dirty) {
      renderBottom();
      bottom_.dirty = false;
      needs_flush   = true;
    }
  }

  if (needs_flush) {
    flushDisplay();
  }

  giveMutex();
}

// ── Render helpers ────────────────────────────────────────────────────────────

void OLEDSubsystem::renderTop() {
  display_.fillRect(0, 0, DISPLAY_W, HALF_H, SSD1306_BLACK);
  if (top_.mode == MODE_TEXT) {
    display_.setCursor(0, 0);
    display_.print(top_.text);
  } else {
    display_.drawBitmap(0, 0, top_.bitmap, DISPLAY_W, HALF_H, SSD1306_WHITE);
  }
}

void OLEDSubsystem::renderBottom() {
  display_.fillRect(0, HALF_H, DISPLAY_W, HALF_H, SSD1306_BLACK);
  if (bottom_.mode == MODE_TEXT) {
    display_.setCursor(0, HALF_H);
    display_.print(bottom_.text);
  } else {
    display_.drawBitmap(0, HALF_H, bottom_.bitmap, DISPLAY_W, HALF_H,
                        SSD1306_WHITE);
  }
}

void OLEDSubsystem::renderFullBitmap() {
  display_.clearDisplay();
  display_.drawBitmap(0, 0, full_bitmap_, DISPLAY_W, DISPLAY_H, SSD1306_WHITE);
}

void OLEDSubsystem::renderFullText() {
  display_.clearDisplay();
  display_.setCursor(0, 0);
  display_.print(full_text_);
}

void OLEDSubsystem::flushDisplay() {
#ifdef USE_FREERTOS
  // Software SPI is bit-banged; a FreeRTOS task switch mid-transfer shifts the
  // SPI clock phase by one bit, manifesting as a 1-pixel display shake.
  // The full framebuffer transfer is ~150 µs at Teensy 4.1 speeds — safe.
  taskENTER_CRITICAL();
  display_.display();
  taskEXIT_CRITICAL();
#else
  display_.display();
#endif
}

// ── micro-ROS ─────────────────────────────────────────────────────────────────

bool OLEDSubsystem::onCreate(rcl_node_t* node, rclc_executor_t* executor) {
  node_ = node;

  // Pre-wire static string buffers so the executor can deserialise into them
  srv_req_.text.data     = srv_text_buf_;
  srv_req_.text.capacity = sizeof(srv_text_buf_);
  srv_req_.text.size     = 0;
  srv_text_buf_[0]       = '\0';

  srv_res_.message.data     = srv_msg_buf_;
  srv_res_.message.capacity = sizeof(srv_msg_buf_);
  srv_res_.message.size     = 0;
  srv_msg_buf_[0]           = '\0';

  if (rclc_service_init_default(
          &srv_, node_,
          ROSIDL_GET_SRV_TYPE_SUPPORT(mcu_msgs, srv, OLEDControl),
          "/mcu_robot/oled_control") != RCL_RET_OK) {
    return false;
  }

  if (rclc_executor_add_service_with_context(
          executor, &srv_, &srv_req_, &srv_res_,
          &OLEDSubsystem::serviceCallback, this) != RCL_RET_OK) {
    return false;
  }

  return true;
}

void OLEDSubsystem::onDestroy() {
  if (srv_.impl) {
    rcl_service_fini(&srv_, node_);
  }
  node_ = nullptr;
}

void OLEDSubsystem::serviceCallback(const void* req, void* res, void* ctx) {
  auto* self = static_cast<OLEDSubsystem*>(ctx);
  self->handleService(
      static_cast<const mcu_msgs__srv__OLEDControl_Request*>(req),
      static_cast<mcu_msgs__srv__OLEDControl_Response*>(res));
}

void OLEDSubsystem::handleService(
    const mcu_msgs__srv__OLEDControl_Request*  req,
    mcu_msgs__srv__OLEDControl_Response*       res) {
  if (!takeMutex()) {
    res->accepted = false;
    return;
  }

  const char* text =
      (req->text.data && req->text.size > 0) ? req->text.data : "";

  switch (req->zone) {
    // ── Top zone ────────────────────────────────────────────────────────────
    case mcu_msgs__srv__OLEDControl_Request__ZONE_TOP:
      full_bitmap_active_ = false;
      full_text_active_   = false;
      if (req->mode == mcu_msgs__srv__OLEDControl_Request__MODE_TEXT) {
        strncpy(top_.text, text, sizeof(top_.text) - 1);
        top_.text[sizeof(top_.text) - 1] = '\0';
        top_.mode = MODE_TEXT;
      } else {
        memcpy(top_.bitmap, req->bitmap, BITMAP_HALF);
        top_.mode = MODE_BITMAP;
      }
      top_.dirty = true;
      break;

    // ── Bottom zone ─────────────────────────────────────────────────────────
    case mcu_msgs__srv__OLEDControl_Request__ZONE_BOTTOM:
      full_bitmap_active_ = false;
      full_text_active_   = false;
      if (req->mode == mcu_msgs__srv__OLEDControl_Request__MODE_TEXT) {
        strncpy(bottom_.text, text, sizeof(bottom_.text) - 1);
        bottom_.text[sizeof(bottom_.text) - 1] = '\0';
        bottom_.mode = MODE_TEXT;
      } else {
        memcpy(bottom_.bitmap, req->bitmap, BITMAP_HALF);
        bottom_.mode = MODE_BITMAP;
      }
      bottom_.dirty = true;
      break;

    // ── Full screen ─────────────────────────────────────────────────────────
    case mcu_msgs__srv__OLEDControl_Request__ZONE_FULL:
    default:
      if (req->mode == mcu_msgs__srv__OLEDControl_Request__MODE_TEXT) {
        strncpy(full_text_, text, sizeof(full_text_) - 1);
        full_text_[sizeof(full_text_) - 1] = '\0';
        full_text_active_   = true;
        full_bitmap_active_ = false;
      } else {
        memcpy(full_bitmap_, req->bitmap, BITMAP_FULL);
        full_bitmap_active_ = true;
        full_text_active_   = false;
      }
      break;
  }

  giveMutex();
  res->accepted = true;
}

// ── Internal API (thread-safe) ────────────────────────────────────────────────

void OLEDSubsystem::setTopText(const char* text) {
  if (!takeMutex()) return;
  strncpy(top_.text, text, sizeof(top_.text) - 1);
  top_.text[sizeof(top_.text) - 1] = '\0';
  top_.mode           = MODE_TEXT;
  top_.dirty          = true;
  full_bitmap_active_ = false;
  full_text_active_   = false;
  giveMutex();
}

void OLEDSubsystem::setBottomText(const char* text) {
  if (!takeMutex()) return;
  strncpy(bottom_.text, text, sizeof(bottom_.text) - 1);
  bottom_.text[sizeof(bottom_.text) - 1] = '\0';
  bottom_.mode        = MODE_TEXT;
  bottom_.dirty       = true;
  full_bitmap_active_ = false;
  full_text_active_   = false;
  giveMutex();
}

void OLEDSubsystem::setFullText(const char* text) {
  if (!takeMutex()) return;
  strncpy(full_text_, text, sizeof(full_text_) - 1);
  full_text_[sizeof(full_text_) - 1] = '\0';
  full_text_active_   = true;
  full_bitmap_active_ = false;
  giveMutex();
}

void OLEDSubsystem::setTopBitmap(const uint8_t* data, size_t len) {
  if (!takeMutex()) return;
  memcpy(top_.bitmap, data, len < BITMAP_HALF ? len : BITMAP_HALF);
  top_.mode           = MODE_BITMAP;
  top_.dirty          = true;
  full_bitmap_active_ = false;
  full_text_active_   = false;
  giveMutex();
}

void OLEDSubsystem::setBottomBitmap(const uint8_t* data, size_t len) {
  if (!takeMutex()) return;
  memcpy(bottom_.bitmap, data, len < BITMAP_HALF ? len : BITMAP_HALF);
  bottom_.mode        = MODE_BITMAP;
  bottom_.dirty       = true;
  full_bitmap_active_ = false;
  full_text_active_   = false;
  giveMutex();
}

void OLEDSubsystem::setFullBitmap(const uint8_t* data, size_t len) {
  if (!takeMutex()) return;
  memcpy(full_bitmap_, data, len < BITMAP_FULL ? len : BITMAP_FULL);
  full_bitmap_active_ = true;
  full_text_active_   = false;
  giveMutex();
}

}  // namespace Subsystem
