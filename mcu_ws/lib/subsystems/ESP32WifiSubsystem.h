/**
 * @file ESP32WifiSubsystem.h
 * @author Rafeed Khan
 * @brief ESP32 WiFi connection management subsystem for micro-ROS
 *
 * This subsystem manages WiFi connectivity for ESP32-based devices that need
 * to communicate over WiFi with a micro-ROS agent. It handles:
 * - Initial WiFi connection
 * - Connection monitoring
 * - Automatic reconnection on disconnect
 * - Optional multi-AP support (connect to strongest signal)
 */
#pragma once

// Only compile for ESP32 platform
#ifdef ESP32

#include <Arduino.h>
#include <BaseSubsystem.h>
#include <WiFi.h>

#include "TimedSubsystem.h"

namespace Subsystem {

/**
 * @brief WiFi connection states
 */
enum class WifiState : uint8_t {
  DISCONNECTED = 0,  // Not connected to any AP
  CONNECTING = 1,    // Attempting to connect
  CONNECTED = 2,     // Successfully connected
  RECONNECTING = 3,  // Lost connection, attempting to reconnect
  FAILED = 4         // Connection failed after max retries
};

/**
 * @brief Single WiFi access point credentials
 */
struct WifiCredentials {
  const char* ssid;
  const char* password;
};

/**
 * @brief Setup configuration for ESP32WifiSubsystem
 *
 * Supports single AP or multiple APs. For multi-AP mode, we populate the
 * ap_list array with credentials and set ap_count accordingly. The subsystem
 * will scan and connect to the AP with strongest signal.
 */
class ESP32WifiSubsystemSetup : public Classes::BaseSetup {
 public:
  /**
   * @brief Construct setup for single AP mode
   */
  ESP32WifiSubsystemSetup(const char* _id, const char* ssid,
                          const char* password,
                          uint32_t connection_timeout_ms = 10000,
                          uint32_t reconnect_interval_ms = 5000,
                          uint8_t max_retries = 5)
      : Classes::BaseSetup(_id),
        connection_timeout_ms_(connection_timeout_ms),
        reconnect_interval_ms_(reconnect_interval_ms),
        max_retries_(max_retries),
        ap_count_(1) {
    ap_list_[0] = {ssid, password};
  }

  /**
   * @brief Construct setup for multi-AP mode
   * @param ap_list Array of WifiCredentials (max 4 ofc)
   * @param ap_count Number of APs in the list
   */
  ESP32WifiSubsystemSetup(const char* _id, const WifiCredentials* ap_list,
                          uint8_t ap_count,
                          uint32_t connection_timeout_ms = 10000,
                          uint32_t reconnect_interval_ms = 5000,
                          uint8_t max_retries = 5)
      : Classes::BaseSetup(_id),
        connection_timeout_ms_(connection_timeout_ms),
        reconnect_interval_ms_(reconnect_interval_ms),
        max_retries_(max_retries),
        ap_count_(ap_count > kMaxAPs ? kMaxAPs : ap_count) {
    for (uint8_t i = 0; i < ap_count_; ++i) {
      ap_list_[i] = ap_list[i];
    }
  }

  static constexpr uint8_t kMaxAPs = 4;

  uint32_t connection_timeout_ms_;  // Timeout for initial connection attempt
  uint32_t reconnect_interval_ms_;  // Delay between reconnection attempts
  uint8_t max_retries_;  // Max consecutive failed attempts before FAILED state
  uint8_t ap_count_;     // Number of configured APs
  WifiCredentials ap_list_[kMaxAPs] = {};  // List of AP credentials
};

/**
 * @brief ESP32 WiFi Subsystem for micro-ROS connectivity
 *
 * Manages the WiFi connection lifecycle using ESP32's event system for that
 * strongdisconnect detection and reconnection. Works alongside MicrorosManager.
 * T This subsystem handles WiFi layer, MicrorosManager handles the micro-ROS
 * agent connection.
 *
 * Usage:
 * 1. Create setup with WiFi credentials
 * 2. Instantiate subsystem
 * 3. Register with RobotManager (or call lifecycle methods manually)
 * 4. MicrorosManager can check isConnected() before attempting agent ping
 */
class ESP32WifiSubsystem : public Subsystem::TimedSubsystem {
 public:
  explicit ESP32WifiSubsystem(const ESP32WifiSubsystemSetup& setup)
      : Subsystem::TimedSubsystem(setup), setup_(setup) {}

  // Lifecycle Hooks (BaseSubsystem interface)
  bool init() override;
  void begin() override;
  void update() override;
  void pause() override;
  void reset() override;
  const char* getInfo() override;

  // Status Queries
  WifiState getState() const { return state_; }
  bool isConnected() const { return state_ == WifiState::CONNECTED; }
  bool isConnecting() const {
    return state_ == WifiState::CONNECTING || state_ == WifiState::RECONNECTING;
  }
  bool hasFailed() const { return state_ == WifiState::FAILED; }

  // Network Info (valid only when connected)
  IPAddress getLocalIP() const { return WiFi.localIP(); }
  int32_t getRSSI() const { return WiFi.RSSI(); }
  const char* getSSID() const { return WiFi.SSID().c_str(); }

  // Manual control
  void disconnect();
  void reconnect();
  void clearFailedState();  // Reset from FAILED to allow retry

 private:
  // Internal state management
  void transitionTo(WifiState new_state);
  void attemptConnection();
  void handleConnected();
  void handleDisconnected();

  // Multi-AP support
  int8_t scanForBestAP();  // Returns index of best AP, -1 if none found

  // WiFi event callback (static + instance pointer pattern)
  static void onWifiEvent(WiFiEvent_t event);
  static ESP32WifiSubsystem* s_instance_;

  // Configuration
  const ESP32WifiSubsystemSetup setup_;

  // State machine
  WifiState state_ = WifiState::DISCONNECTED;
  uint8_t current_ap_index_ = 0;  // Which AP we're connected/connecting to
  uint8_t retry_count_ = 0;       // Consecutive failed connection attempts
  uint32_t state_entry_time_ms_ = 0;
  uint32_t last_attempt_time_ms_ = 0;

  // Flags set by WiFi event callbacks (checked in update())
  volatile bool event_connected_ = false;
  volatile bool event_disconnected_ = false;
  volatile bool event_got_ip_ = false;
};

}  // namespace Subsystem

#endif  // ESP32
