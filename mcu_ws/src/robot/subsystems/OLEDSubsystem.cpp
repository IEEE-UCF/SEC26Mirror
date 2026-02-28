#include "OLEDSubsystem.h"

#include <cstdio>
#include <cstring>

#include "BatterySubsystem.h"
#include "DipSwitchSubsystem.h"
#include "ImuSubsystem.h"
#include "UWBSubsystem.h"
#include "DebugLog.h"

namespace Subsystem {

// ── Constructor
// ───────────────────────────────────────────────────────────────

OLEDSubsystem::OLEDSubsystem(const OLEDSubsystemSetup& setup)
    : Classes::BaseSubsystem(setup),
      setup_(setup),
      display_(DISPLAY_W, DISPLAY_H, setup.spi_, setup.dc_pin_, setup.rst_pin_,
               setup.cs_pin_, setup.bitrate_) {}

// ── Lifecycle
// ─────────────────────────────────────────────────────────────────

bool OLEDSubsystem::init() {
  if (!display_.begin(SSD1306_SWITCHCAPVCC)) {
    DEBUG_PRINTLN("[OLED] init FAIL: display begin");
    return false;
  }

  display_.clearDisplay();
  display_.setTextSize(1);
  display_.setTextColor(SSD1306_WHITE);
  display_.cp437(true);
  display_.display();
  DEBUG_PRINTLN("[OLED] init OK (128x64 SPI1)");
  return true;
}

void OLEDSubsystem::reset() {
  if (!takeMutex()) return;

  memset(lines_, 0, sizeof(lines_));
  next_write_ = 0;
  total_written_ = 0;
  view_offset_ = 0;
  dirty_ = false;

  display_.clearDisplay();
  display_.display();

  giveMutex();
}

const char* OLEDSubsystem::getInfo() {
  static const char info[] = "OLEDSubsystem";
  return info;
}

// ── Update
// ────────────────────────────────────────────────────────────────────

void OLEDSubsystem::update() {
  if (!takeMutex()) return;

  if (isDashboardMode()) {
    renderDashboard();
    flushDisplay();
  } else if (dirty_) {
    renderLines();
    flushDisplay();
    dirty_ = false;
  }

  giveMutex();
}

// ── Ring-buffer helpers
// ───────────────────────────────────────────────────────

void OLEDSubsystem::appendOneLine(const char* s, int len) {
  int n = len < MAX_LINE_LEN ? len : MAX_LINE_LEN;
  memcpy(lines_[next_write_], s, n);
  lines_[next_write_][n] = '\0';
  next_write_ = (next_write_ + 1) % MAX_LINES;
  if (total_written_ < MAX_LINES) ++total_written_;
}

// ── Render
// ────────────────────────────────────────────────────────────────────

void OLEDSubsystem::renderLines() {
  display_.clearDisplay();
  display_.setTextSize(1);
  display_.setTextColor(SSD1306_WHITE);

  int first_row = 0;  // first row available for terminal content
  int visible = LINES_VISIBLE;

  // Persistent status line at row 0 (e.g. battery info)
  if (has_status_line_) {
    display_.setCursor(0, 0);
    display_.print(status_line_);
    first_row = 1;
    visible = LINES_VISIBLE - 1;  // 7 terminal rows at y=8,16,...,56
  }

  int count = total_written_ < MAX_LINES ? total_written_ : MAX_LINES;

  // Terminal rows: oldest visible at first_row, newest at bottom.
  // from_end=0 is the most-recently appended line; larger = older.
  for (int row = 0; row < visible; ++row) {
    int from_end = view_offset_ + (visible - 1 - row);
    if (from_end >= count) continue;

    int idx =
        ((next_write_ - 1 - from_end) % MAX_LINES + MAX_LINES) % MAX_LINES;
    int y = has_status_line_ ? (8 + row * 8) : (row * 8);
    display_.setCursor(0, y);
    display_.print(lines_[idx]);
  }
}

void OLEDSubsystem::flushDisplay() { display_.display(); }

// ── micro-ROS: LCD append subscription
// ─────────────────────────────────────────

bool OLEDSubsystem::onCreate(rcl_node_t* node, rclc_executor_t* executor) {
  node_ = node;

  // Wire pre-allocated buffer into the subscription message string field.
  lcd_msg_.data.data = lcd_text_buf_;
  lcd_msg_.data.capacity = sizeof(lcd_text_buf_);
  lcd_msg_.data.size = 0;
  lcd_text_buf_[0] = '\0';

  if (rclc_subscription_init_best_effort(
          &lcd_sub_, node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
          "/mcu_robot/lcd/append") != RCL_RET_OK) {
    return false;
  }

  if (rclc_executor_add_subscription_with_context(
          executor, &lcd_sub_, &lcd_msg_, &OLEDSubsystem::appendCallback, this,
          ON_NEW_DATA) != RCL_RET_OK) {
    return false;
  }

  if (rclc_subscription_init_best_effort(
          &scroll_sub_, node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
          "/mcu_robot/lcd/scroll") != RCL_RET_OK) {
    return false;
  }

  if (rclc_executor_add_subscription_with_context(
          executor, &scroll_sub_, &scroll_msg_, &OLEDSubsystem::scrollCallback,
          this, ON_NEW_DATA) != RCL_RET_OK) {
    DEBUG_PRINTLN("[OLED] onCreate FAIL: scroll subscription");
    return false;
  }

  DEBUG_PRINTLN("[OLED] onCreate OK (lcd/append + lcd/scroll)");
  return true;
}

void OLEDSubsystem::onDestroy() {
  // destroy_entities() finalises the rcl_node before calling onDestroy, so
  // rcl_*_fini would leave impl non-NULL on error; reset local state only.
  lcd_sub_ = rcl_get_zero_initialized_subscription();
  scroll_sub_ = rcl_get_zero_initialized_subscription();
  node_ = nullptr;
}

void OLEDSubsystem::appendCallback(const void* msg, void* ctx) {
  auto* self = static_cast<OLEDSubsystem*>(ctx);
  const auto* m = static_cast<const std_msgs__msg__String*>(msg);
  if (!self->takeMutex()) return;

  const char* text = (m->data.data && m->data.size > 0) ? m->data.data : "";
  self->appendText(text);

  self->giveMutex();
}

// ── micro-ROS: scroll topic
// ───────────────────────────────────────────────────

void OLEDSubsystem::scrollCallback(const void* msg, void* ctx) {
  auto* self = static_cast<OLEDSubsystem*>(ctx);
  const auto* m = static_cast<const std_msgs__msg__Int8*>(msg);
  if (!self->takeMutex()) return;
  self->scrollBy(m->data);
  self->giveMutex();
}

// ── Internal C++ API
// ──────────────────────────────────────────────────────────

void OLEDSubsystem::appendText(const char* text) {
  static constexpr int WRAP_INDENT = 2;
  const char* p = text;
  while (*p) {
    const char* nl = p;
    while (*nl && *nl != '\n') ++nl;
    int seg_len = static_cast<int>(nl - p);
    if (seg_len <= MAX_LINE_LEN) {
      appendOneLine(p, seg_len);
    } else {
      appendOneLine(p, MAX_LINE_LEN);
      int remaining = seg_len - MAX_LINE_LEN;
      const char* wp = p + MAX_LINE_LEN;
      int wrap_width = MAX_LINE_LEN - WRAP_INDENT;
      while (remaining > 0) {
        char wrap_buf[MAX_LINE_LEN + 1];
        memset(wrap_buf, ' ', WRAP_INDENT);
        int chunk = remaining < wrap_width ? remaining : wrap_width;
        memcpy(wrap_buf + WRAP_INDENT, wp, chunk);
        appendOneLine(wrap_buf, WRAP_INDENT + chunk);
        wp += chunk;
        remaining -= chunk;
      }
    }
    p = (*nl == '\n') ? nl + 1 : nl;
  }
  dirty_ = true;
}

void OLEDSubsystem::scrollBy(int8_t delta) {
  int count = total_written_ < MAX_LINES ? total_written_ : MAX_LINES;
  int visible = has_status_line_ ? (LINES_VISIBLE - 1) : LINES_VISIBLE;
  int max_scroll = count > visible ? count - visible : 0;

  // -1 = up (older), +1 = down (newer)
  view_offset_ -= delta;
  if (view_offset_ < 0) view_offset_ = 0;
  if (view_offset_ > max_scroll) view_offset_ = max_scroll;

  dirty_ = true;
}

void OLEDSubsystem::setStatusLine(const char* text) {
  if (!takeMutex()) return;

  if (text && text[0] != '\0') {
    int n = strlen(text);
    if (n > MAX_LINE_LEN) n = MAX_LINE_LEN;
    memcpy(status_line_, text, n);
    status_line_[n] = '\0';
    has_status_line_ = true;
  } else {
    status_line_[0] = '\0';
    has_status_line_ = false;
  }
  dirty_ = true;

  giveMutex();
}

// ── Dashboard mode
// ──────────────────────────────────────────────────────────

void OLEDSubsystem::setDashboardSources(DipSwitchSubsystem* dip,
                                         BatterySubsystem* battery,
                                         ImuSubsystem* imu,
                                         UWBSubsystem* uwb) {
  dash_dip_ = dip;
  dash_battery_ = battery;
  dash_imu_ = imu;
  dash_uwb_ = uwb;
}

bool OLEDSubsystem::isDashboardMode() const {
  return dash_dip_ && dash_dip_->isSwitchOn(DipSwitchSubsystem::DIP_OLED_DEBUG);
}

void OLEDSubsystem::renderDashboard() {
  display_.clearDisplay();
  display_.setTextSize(1);
  display_.setTextColor(SSD1306_WHITE);

  char buf[MAX_LINE_LEN + 1];

  // Row 0: header
  display_.setCursor(0, 0);
  display_.print("== DEBUG DASHBOARD ==");

  // Row 1: battery voltage and current
  display_.setCursor(0, 8);
  if (dash_battery_) {
    snprintf(buf, sizeof(buf), "BAT: %.1fV  %.1fA",
             dash_battery_->getVoltage(), dash_battery_->getCurrentA());
  } else {
    snprintf(buf, sizeof(buf), "BAT: N/A");
  }
  display_.print(buf);

  // Row 2: power
  display_.setCursor(0, 16);
  if (dash_battery_) {
    float w = dash_battery_->getVoltage() * dash_battery_->getCurrentA();
    snprintf(buf, sizeof(buf), "PWR: %.1fW", w);
  } else {
    snprintf(buf, sizeof(buf), "PWR: N/A");
  }
  display_.print(buf);

  // Row 3: yaw
  display_.setCursor(0, 24);
  if (dash_imu_) {
    float yaw_deg = dash_imu_->getYaw() * 57.2958f;  // rad to deg
    snprintf(buf, sizeof(buf), "YAW: %.1f deg", yaw_deg);
  } else {
    snprintf(buf, sizeof(buf), "YAW: N/A");
  }
  display_.print(buf);

  // Row 4: UWB ranging count
  display_.setCursor(0, 32);
  if (dash_uwb_) {
    const auto& uwb = dash_uwb_->getDriverData();
    snprintf(buf, sizeof(buf), "UWB: %lu rng", uwb.ranging_count);
  } else {
    snprintf(buf, sizeof(buf), "UWB: OFF");
  }
  display_.print(buf);

  // Row 5: DIP switches binary
  display_.setCursor(0, 40);
  if (dash_dip_) {
    uint8_t state = dash_dip_->getState();
    snprintf(buf, sizeof(buf), "DIP: %d%d%d%d%d%d%d%d",
             (state >> 7) & 1, (state >> 6) & 1, (state >> 5) & 1,
             (state >> 4) & 1, (state >> 3) & 1, (state >> 2) & 1,
             (state >> 1) & 1, state & 1);
  } else {
    snprintf(buf, sizeof(buf), "DIP: N/A");
  }
  display_.print(buf);

  // Row 6: uptime
  display_.setCursor(0, 48);
  snprintf(buf, sizeof(buf), "UP: %lus", millis() / 1000);
  display_.print(buf);

  // Row 7: reserved
  display_.setCursor(0, 56);
  display_.print("                     ");
}

}  // namespace Subsystem
