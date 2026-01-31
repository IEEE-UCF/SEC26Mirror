/**
 * @file field-element.h
 * @brief Base class for field elements with ESP-NOW communication
 * @date 01/18/2026
 */

#ifndef FIELD_ELEMENT_H
#define FIELD_ELEMENT_H

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include "field-message.h"

namespace Field {

/// @brief Callback type for command handling
typedef void (*CommandCallback)(FieldCommand cmd);

/// @brief Base class for field elements with ESP-NOW communication
class FieldElement {
 public:
  FieldElement(ElementId id) : _elementId(id), _status(ElementStatus::NOT_ONLINE),
                                _sequenceNum(0), _commandCallback(nullptr),
                                _cycleDisplayActive(false) {}

  virtual ~FieldElement() = default;

  /// @brief Initialize ESP-NOW communication
  /// @return true if successful
  bool initComms() {
    // Ensure clean WiFi state
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(100);

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);  // Disable WiFi sleep for reliable ESP-NOW
    delay(100);

    if (esp_now_init() != ESP_OK) {
      Serial.println("ESP-NOW init failed");
      return false;
    }

    // Register callbacks
    s_instance = this;
    esp_now_register_recv_cb(onDataRecv);
    esp_now_register_send_cb(onDataSent);

    // Add broadcast peer for receiving from any device
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, BROADCAST_MAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add broadcast peer");
      return false;
    }

    _status = ElementStatus::READY;
    Serial.println("ESP-NOW initialized");
    printMac();
    return true;
  }

  /// @brief Send status update to controller
  void sendStatus() {
    StatusUpdateMessage msg;
    msg.header.type = MessageType::STATUS_UPDATE;
    msg.header.senderId = _elementId;
    msg.header.sequenceNum = _sequenceNum++;
    msg.status = _status;

    esp_now_send(BROADCAST_MAC, (uint8_t*)&msg, sizeof(msg));
  }

  /// @brief Send color report (for antenna elements)
  void sendColorReport(FieldColor color) {
    ColorReportMessage msg;
    msg.header.type = MessageType::COLOR_REPORT;
    msg.header.senderId = _elementId;
    msg.header.sequenceNum = _sequenceNum++;
    msg.color = color;

    esp_now_send(BROADCAST_MAC, (uint8_t*)&msg, sizeof(msg));
  }

  /// @brief Set command callback
  void setCommandCallback(CommandCallback callback) {
    _commandCallback = callback;
  }

  /// @brief Get current status
  ElementStatus getStatus() const { return _status; }

  /// @brief Set current status
  void setStatus(ElementStatus status) {
    _status = status;
    sendStatus();
  }

  /// @brief Get element ID
  ElementId getElementId() const { return _elementId; }

  /// @brief Check if cycle display animation is active
  bool isCycleDisplayActive() const { return _cycleDisplayActive; }

  /// @brief Print MAC address
  void printMac() {
    uint8_t mac[6];
    WiFi.macAddress(mac);
    Serial.printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  }

  /// @brief Process incoming messages (call in loop)
  void processMessages() {
    if (_hasNewMessage) {
      _hasNewMessage = false;
      handleMessage(_receivedMessage);
    }
  }

 protected:
  ElementId _elementId;
  ElementStatus _status;
  uint8_t _sequenceNum;
  CommandCallback _commandCallback;
  bool _cycleDisplayActive;

  /// @brief Handle received message
  virtual void handleMessage(const FieldMessage& msg) {
    if (msg.header.type == MessageType::COMMAND) {
      const CommandMessage& cmd = msg.command;

      // Check if message is for us or broadcast
      if (cmd.targetId != _elementId &&
          static_cast<uint8_t>(cmd.targetId) != 0xFF) {
        return;
      }

      switch (cmd.command) {
        case FieldCommand::RESET:
          Serial.println("RESET command received");
          ESP.restart();
          break;

        case FieldCommand::START:
          Serial.println("START command received");
          _status = ElementStatus::NOT_ACTIVATED;
          _cycleDisplayActive = false;
          sendStatus();
          if (_commandCallback) _commandCallback(FieldCommand::START);
          break;

        case FieldCommand::CYCLE_DISPLAY:
          Serial.println("CYCLE_DISPLAY command received");
          _cycleDisplayActive = true;
          if (_commandCallback) _commandCallback(FieldCommand::CYCLE_DISPLAY);
          break;

        case FieldCommand::PING:
          sendStatus();
          break;

        default:
          break;
      }
    }
  }

 private:
  static FieldElement* s_instance;
  static FieldMessage _receivedMessage;
  static volatile bool _hasNewMessage;

  static void onDataRecv(const uint8_t* mac, const uint8_t* data, int len) {
    if (s_instance && len <= sizeof(FieldMessage)) {
      memcpy(&_receivedMessage, data, len);
      _hasNewMessage = true;
    }
  }

  static void onDataSent(const uint8_t* mac, esp_now_send_status_t status) {
    // Optional: handle send confirmation
  }
};

// Static member definitions (must be in .cpp or inline in header)
inline FieldElement* FieldElement::s_instance = nullptr;
inline FieldMessage FieldElement::_receivedMessage = {};
inline volatile bool FieldElement::_hasNewMessage = false;

}  // namespace Field

#endif
