/**
 * @file ESP32WifiSubsystem.cpp
 * @author Rafeed Khan
 * @brief Implementation of ESP32 WiFi connection management subsystem
 */

// Only compile for ESP32 platform
#ifdef ESP32

#include "ESP32WifiSubsystem.h"

namespace Subsystem {

// Static instance pointer for WiFi event callback
ESP32WifiSubsystem* ESP32WifiSubsystem::s_instance_ = nullptr;

// WiFi Event Callback (runs in WiFi task context)
void ESP32WifiSubsystem::onWifiEvent(WiFiEvent_t event) {
  if (!s_instance_) return;

  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      s_instance_->event_connected_ = true;
      break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      s_instance_->event_disconnected_ = true;
      break;

    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      s_instance_->event_got_ip_ = true;
      break;

    default:
      break;
  }
}

// Lifecycle Implementation

bool ESP32WifiSubsystem::init() {
  state_ = WifiState::DISCONNECTED;
  current_ap_index_ = 0;
  retry_count_ = 0;
  event_connected_ = false;
  event_disconnected_ = false;
  event_got_ip_ = false;

  // Validate we have at least ONE AP configured
  if (setup_.ap_count_ == 0 || setup_.ap_list_[0].ssid == nullptr) {
    return false;
  }

  return true;
}

void ESP32WifiSubsystem::begin() {
  // Register as singleton for event callbacks
  s_instance_ = this;

  // Set WiFi mode to station
  WiFi.mode(WIFI_STA);

  // Register event handler
  WiFi.onEvent(onWifiEvent);

  // Start connection attempt
  state_entry_time_ms_ = millis();
  transitionTo(WifiState::CONNECTING);
  attemptConnection();
}

void ESP32WifiSubsystem::update() {
  uint32_t now = millis();

  // Process our WiFi events (set by callback in WiFi task context)
  if (event_got_ip_) {
    event_got_ip_ = false;
    event_connected_ = false;
    handleConnected();
  }

  if (event_disconnected_) {
    event_disconnected_ = false;
    handleDisconnected();
  }

  // State machine logic! My favorite
  switch (state_) {
    case WifiState::DISCONNECTED:
      // Nothing to do, its gonna wait for begin() or reconnect()
      break;

    case WifiState::CONNECTING:
    case WifiState::RECONNECTING: {
      // Check for connection timeout
      uint32_t time_in_state = now - state_entry_time_ms_;
      if (time_in_state >= setup_.connection_timeout_ms_) {
        // Connection attempt timed out
        WiFi.disconnect(true);
        retry_count_++;

        if (retry_count_ >= setup_.max_retries_) {
          transitionTo(WifiState::FAILED);
        } else {
          // Schedule next retry
          last_attempt_time_ms_ = now;
          transitionTo(state_ == WifiState::CONNECTING
                           ? WifiState::CONNECTING
                           : WifiState::RECONNECTING);
        }
      }

      // Check if we should retry after interval
      if (retry_count_ > 0 &&
          (now - last_attempt_time_ms_) >= setup_.reconnect_interval_ms_) {
        attemptConnection();
      }
      break;
    }

    case WifiState::CONNECTED:
      // Checking the connection quality every 5 seconds
      if (everyMs(5000)) {
        if (WiFi.status() != WL_CONNECTED) {
          handleDisconnected();
        }
      }
      break;

    case WifiState::FAILED:
      // Stay in failed state until clearFailedState() is called
      break;
  }
}

void ESP32WifiSubsystem::pause() {
  WiFi.disconnect(true);
  transitionTo(WifiState::DISCONNECTED);
}

void ESP32WifiSubsystem::reset() {
  WiFi.disconnect(true);
  retry_count_ = 0;
  current_ap_index_ = 0;
  event_connected_ = false;
  event_disconnected_ = false;
  event_got_ip_ = false;
  transitionTo(WifiState::DISCONNECTED);
}

const char* ESP32WifiSubsystem::getInfo() {
  static const char info[] = "ESP32WifiSubsystem";
  return info;
}

// Status and Control Methods

void ESP32WifiSubsystem::disconnect() {
  WiFi.disconnect(true);
  transitionTo(WifiState::DISCONNECTED);
}

void ESP32WifiSubsystem::reconnect() {
  if (state_ == WifiState::FAILED) {
    clearFailedState();
  }

  retry_count_ = 0;
  transitionTo(WifiState::RECONNECTING);
  attemptConnection();
}

void ESP32WifiSubsystem::clearFailedState() {
  if (state_ == WifiState::FAILED) {
    retry_count_ = 0;
    transitionTo(WifiState::DISCONNECTED);
  }
}

// Internal Implementation

void ESP32WifiSubsystem::transitionTo(WifiState new_state) {
  if (state_ != new_state) {
    state_ = new_state;
    state_entry_time_ms_ = millis();
  }
}

void ESP32WifiSubsystem::attemptConnection() {
  // For multi-AP mode, scan and find best AP!
  if (setup_.ap_count_ > 1) {
    int8_t best_ap = scanForBestAP();
    if (best_ap >= 0) {
      current_ap_index_ = best_ap;
    }
    // If scan fails, use current_ap_index_ (it cycles through on retry)
  }

  const WifiCredentials& creds = setup_.ap_list_[current_ap_index_];

  // Disconnect any existing connection first
  WiFi.disconnect(true);
  delay(100);

  // Attempt connection
  WiFi.begin(creds.ssid, creds.password);
  state_entry_time_ms_ = millis();
  last_attempt_time_ms_ = millis();
}

void ESP32WifiSubsystem::handleConnected() {
  retry_count_ = 0;
  transitionTo(WifiState::CONNECTED);
}

void ESP32WifiSubsystem::handleDisconnected() {
  // Only transition if we were connected (ignore during initial connection)
  if (state_ == WifiState::CONNECTED) {
    retry_count_ = 0;
    transitionTo(WifiState::RECONNECTING);
    attemptConnection();
  } else if (state_ == WifiState::CONNECTING ||
             state_ == WifiState::RECONNECTING) {
    // Connection attempt failed, increment retry
    retry_count_++;

    if (retry_count_ >= setup_.max_retries_) {
      transitionTo(WifiState::FAILED);
    } else {
      // For multi-AP, try next AP on failure
      if (setup_.ap_count_ > 1) {
        current_ap_index_ = (current_ap_index_ + 1) % setup_.ap_count_;
      }
      last_attempt_time_ms_ = millis();
    }
  }
}

int8_t ESP32WifiSubsystem::scanForBestAP() {
  int16_t num_networks = WiFi.scanNetworks(false, false, false, 300);

  if (num_networks <= 0) {
    return -1;
  }

  int8_t best_index = -1;
  int32_t best_rssi = -999;

  // Find the configured AP with the strongest signal
  for (int16_t i = 0; i < num_networks; ++i) {
    String scanned_ssid = WiFi.SSID(i);
    int32_t rssi = WiFi.RSSI(i);

    // Check if this SSID matches any of our configured APs
    for (uint8_t ap_idx = 0; ap_idx < setup_.ap_count_; ++ap_idx) {
      if (scanned_ssid.equals(setup_.ap_list_[ap_idx].ssid)) {
        if (rssi > best_rssi) {
          best_rssi = rssi;
          best_index = ap_idx;
        }
        break;
      }
    }
  }

  WiFi.scanDelete();
  return best_index;
}

}  // namespace Subsystem

#endif  // ESP32
