/**
 * @file field-message.h
 * @brief Shared message protocol for field element ESP-NOW communication
 * @date 01/18/2026
 */

#ifndef FIELD_MESSAGE_H
#define FIELD_MESSAGE_H

#include <Arduino.h>

namespace Field {

/// @brief Field element IDs (antenna numbers)
enum class ElementId : uint8_t {
  CONTROLLER = 0,
  BUTTON = 1,
  CRANK = 2,
  PRESSURE = 3,
  KEYPAD = 4,
  EARTH = 5
};

/// @brief Status of a field element
enum class ElementStatus : uint8_t {
  NOT_ONLINE = 0,
  READY = 1,
  NOT_ACTIVATED = 2,
  ACTIVATED = 3
};

/// @brief Commands from controller to elements
enum class FieldCommand : uint8_t {
  NONE = 0,
  RESET = 1,
  START = 2,
  CYCLE_DISPLAY = 3,
  PING = 4
};

/// @brief Color codes for antenna RGB (matches earth.h AntennaColor)
enum class FieldColor : uint8_t {
  NONE = 0x00,
  RED = 0x09,
  GREEN = 0x0A,
  BLUE = 0x0C,
  PURPLE = 0x0F
};

/// @brief Message type identifier
enum class MessageType : uint8_t {
  STATUS_UPDATE = 0,   // Element -> Controller: status update
  COMMAND = 1,         // Controller -> Elements: command
  COLOR_REPORT = 2,    // Antenna -> Earth: color report
  COLOR_RESULT = 3     // Earth -> Controller: color verification result
};

/// @brief Base message header
struct FieldMessageHeader {
  MessageType type;
  ElementId senderId;
  uint8_t sequenceNum;
} __attribute__((packed));

/// @brief Status update message (element -> controller)
struct StatusUpdateMessage {
  FieldMessageHeader header;
  ElementStatus status;
  uint8_t reserved[2];
} __attribute__((packed));

/// @brief Command message (controller -> elements)
struct CommandMessage {
  FieldMessageHeader header;
  FieldCommand command;
  ElementId targetId;  // 0xFF for broadcast
  uint8_t reserved;
} __attribute__((packed));

/// @brief Color report message (antenna -> earth)
struct ColorReportMessage {
  FieldMessageHeader header;
  FieldColor color;
  uint8_t reserved[2];
} __attribute__((packed));

/// @brief Color result for earth scoring
struct AntennaColorResult {
  ElementId antennaId;
  FieldColor expectedColor;
  FieldColor receivedColor;
  bool correct;
} __attribute__((packed));

/// @brief Earth color verification result (earth -> controller)
struct ColorResultMessage {
  FieldMessageHeader header;
  uint8_t correctCount;
  uint8_t wrongCount;
  AntennaColorResult results[4];
} __attribute__((packed));

/// @brief Union of all message types for easy handling
union FieldMessage {
  FieldMessageHeader header;
  StatusUpdateMessage statusUpdate;
  CommandMessage command;
  ColorReportMessage colorReport;
  ColorResultMessage colorResult;
  uint8_t raw[32];
} __attribute__((packed));

/// @brief Convert ElementStatus to string
inline const char* statusToString(ElementStatus status) {
  switch (status) {
    case ElementStatus::NOT_ONLINE:
      return "NOT_ONLINE";
    case ElementStatus::READY:
      return "READY";
    case ElementStatus::NOT_ACTIVATED:
      return "NOT_ACTIVATED";
    case ElementStatus::ACTIVATED:
      return "ACTIVATED";
    default:
      return "UNKNOWN";
  }
}

/// @brief Convert ElementId to string
inline const char* elementIdToString(ElementId id) {
  switch (id) {
    case ElementId::CONTROLLER:
      return "CONTROLLER";
    case ElementId::BUTTON:
      return "BUTTON";
    case ElementId::CRANK:
      return "CRANK";
    case ElementId::PRESSURE:
      return "PRESSURE";
    case ElementId::KEYPAD:
      return "KEYPAD";
    case ElementId::EARTH:
      return "EARTH";
    default:
      return "UNKNOWN";
  }
}

/// @brief Convert FieldColor to string
inline const char* colorToString(FieldColor color) {
  switch (color) {
    case FieldColor::RED:
      return "RED";
    case FieldColor::GREEN:
      return "GREEN";
    case FieldColor::BLUE:
      return "BLUE";
    case FieldColor::PURPLE:
      return "PURPLE";
    case FieldColor::NONE:
    default:
      return "NONE";
  }
}

/// @brief Broadcast MAC address (all 0xFF)
constexpr uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

}  // namespace Field

#endif
