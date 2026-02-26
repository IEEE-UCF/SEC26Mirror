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
  display_.cp437(true);
  display_.display();
  return true;
}

void OLEDSubsystem::reset() {
  if (!takeMutex()) return;

  memset(lines_, 0, sizeof(lines_));
  next_write_    = 0;
  total_written_ = 0;
  view_offset_   = 0;
  dirty_         = false;

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

  if (dirty_) {
    renderLines();
    flushDisplay();
    dirty_ = false;
  }

  giveMutex();
}

// ── Ring-buffer helpers ───────────────────────────────────────────────────────

void OLEDSubsystem::appendOneLine(const char* s, int len) {
  int n = len < MAX_LINE_LEN ? len : MAX_LINE_LEN;
  memcpy(lines_[next_write_], s, n);
  lines_[next_write_][n] = '\0';
  next_write_ = (next_write_ + 1) % MAX_LINES;
  if (total_written_ < MAX_LINES) ++total_written_;
}

// ── Render ────────────────────────────────────────────────────────────────────

void OLEDSubsystem::renderLines() {
  display_.clearDisplay();
  display_.setTextSize(1);
  display_.setTextColor(SSD1306_WHITE);

  int count = total_written_ < MAX_LINES ? total_written_ : MAX_LINES;

  // Row 0 = oldest visible, row (LINES_VISIBLE-1) = newest visible.
  // from_end=0 is the most-recently appended line; larger = older.
  for (int row = 0; row < LINES_VISIBLE; ++row) {
    int from_end = view_offset_ + (LINES_VISIBLE - 1 - row);
    if (from_end >= count) continue;

    int idx = ((next_write_ - 1 - from_end) % MAX_LINES + MAX_LINES) % MAX_LINES;
    display_.setCursor(0, row * 8);
    display_.print(lines_[idx]);
  }
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

// ── micro-ROS: LCD append service ─────────────────────────────────────────────

bool OLEDSubsystem::onCreate(rcl_node_t* node, rclc_executor_t* executor) {
  node_ = node;

  // Wire pre-allocated buffer into the service request string field.
  lcd_req_.text.data     = lcd_text_buf_;
  lcd_req_.text.capacity = sizeof(lcd_text_buf_);
  lcd_req_.text.size     = 0;
  lcd_text_buf_[0]       = '\0';

  if (rclc_service_init_default(
          &lcd_srv_, node_,
          ROSIDL_GET_SRV_TYPE_SUPPORT(mcu_msgs, srv, LCDAppend),
          "/mcu_robot/lcd/append") != RCL_RET_OK) {
    return false;
  }

  if (rclc_executor_add_service_with_context(
          executor, &lcd_srv_, &lcd_req_, &lcd_res_,
          &OLEDSubsystem::appendServiceCallback, this) != RCL_RET_OK) {
    return false;
  }

  if (rclc_subscription_init_default(
          &scroll_sub_, node_,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
          "/mcu_robot/lcd/scroll") != RCL_RET_OK) {
    return false;
  }

  if (rclc_executor_add_subscription_with_context(
          executor, &scroll_sub_, &scroll_msg_,
          &OLEDSubsystem::scrollCallback, this,
          ON_NEW_DATA) != RCL_RET_OK) {
    return false;
  }

  return true;
}

void OLEDSubsystem::onDestroy() {
  if (lcd_srv_.impl)    rcl_service_fini(&lcd_srv_,    node_);
  if (scroll_sub_.impl) rcl_subscription_fini(&scroll_sub_, node_);
  node_ = nullptr;
}

void OLEDSubsystem::appendServiceCallback(const void* req, void* res, void* ctx) {
  auto* self = static_cast<OLEDSubsystem*>(ctx);
  self->handleAppend(
      static_cast<const mcu_msgs__srv__LCDAppend_Request*>(req),
      static_cast<mcu_msgs__srv__LCDAppend_Response*>(res));
}

void OLEDSubsystem::handleAppend(const mcu_msgs__srv__LCDAppend_Request*  req,
                                       mcu_msgs__srv__LCDAppend_Response* res) {
  if (!takeMutex()) {
    res->accepted = false;
    return;
  }

  const char* text =
      (req->text.data && req->text.size > 0) ? req->text.data : "";
  appendText(text);

  giveMutex();
  res->accepted = true;
}

// ── micro-ROS: scroll topic ───────────────────────────────────────────────────

void OLEDSubsystem::scrollCallback(const void* msg, void* ctx) {
  auto* self  = static_cast<OLEDSubsystem*>(ctx);
  const auto* m = static_cast<const std_msgs__msg__Int8*>(msg);
  if (!self->takeMutex()) return;
  self->scrollBy(m->data);
  self->giveMutex();
}

// ── Internal C++ API ──────────────────────────────────────────────────────────

void OLEDSubsystem::appendText(const char* text) {
  const char* p = text;
  while (*p) {
    const char* nl = p;
    while (*nl && *nl != '\n') ++nl;
    appendOneLine(p, static_cast<int>(nl - p));
    p = (*nl == '\n') ? nl + 1 : nl;
  }
  dirty_ = true;
}

void OLEDSubsystem::scrollBy(int8_t delta) {
  int count      = total_written_ < MAX_LINES ? total_written_ : MAX_LINES;
  int max_scroll = count > LINES_VISIBLE ? count - LINES_VISIBLE : 0;

  // -1 = up (older), +1 = down (newer)
  view_offset_ -= delta;
  if (view_offset_ < 0)          view_offset_ = 0;
  if (view_offset_ > max_scroll) view_offset_ = max_scroll;

  dirty_ = true;
}

}  // namespace Subsystem
