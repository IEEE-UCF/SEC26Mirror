/**
 * @file controller.h
 * @brief Field controller with LCD menu, joystick, and ESP-NOW communication
 * @date 01/18/2026
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <esp_now.h>

#include "field-message.h"

namespace Field {

/// @brief Number of field elements (excluding controller)
constexpr uint8_t NUM_ELEMENTS = 5;

/// @brief Menu item types
enum class MenuItemType { MENU_FOLDER, MENU_DISPLAY, MENU_ACTION };

/// @brief Forward declaration
class ControllerDriver;

/// @brief Menu item structure
struct MenuItem {
  const char* name;
  MenuItemType type;
  MenuItem* parent;
  MenuItem* children;
  uint8_t childCount;
  void (*action)(ControllerDriver*);
  ElementId elementId;  // For display items
};

/// @brief Earth sub-display data
struct EarthDisplayData {
  FieldColor expectedColors[4];
  FieldColor receivedColors[4];
  bool colorCorrect[4];
  uint8_t correctCount;
  uint8_t wrongCount;
};

/// @brief Setup configuration for ControllerDriver
struct ControllerSetup {
  uint8_t lcdAddress;
  uint8_t joystickXPin;
  uint8_t joystickYPin;
  uint8_t joystickButtonPin;
  uint8_t buttonAPin;
  uint8_t buttonBPin;
};

/// @brief Field controller driver
class ControllerDriver {
 public:
  ControllerDriver(const ControllerSetup& setup);
  ~ControllerDriver() = default;

  /// @brief Initialize controller
  bool init();

  /// @brief Update controller (call in loop)
  void update();

  /// @brief Print all element statuses to serial
  void printStatusToSerial();

  /// @brief Get element status
  ElementStatus getElementStatus(ElementId id) const;

  /// @brief Get earth display data
  const EarthDisplayData& getEarthData() const { return _earthData; }

  /// @brief Send command to all elements
  void sendCommand(FieldCommand cmd);

  /// @brief Send command to specific element
  void sendCommand(FieldCommand cmd, ElementId target);

  // Action methods (public for menu callbacks)
  void actionReset();
  void actionStart();
  void actionCycleDisplay();

 private:
  const ControllerSetup& _setup;
  LiquidCrystal_I2C _lcd;

  // Element status tracking
  ElementStatus _elementStatus[NUM_ELEMENTS];
  unsigned long _lastStatusTime[NUM_ELEMENTS];
  FieldColor _elementColors[NUM_ELEMENTS];  // Randomized colors for antennas

  // Earth scoring data
  EarthDisplayData _earthData;

  // Menu state
  MenuItem* _currentMenu;
  uint8_t _menuIndex;
  uint8_t _menuScroll;

  // Input state
  int _joystickX, _joystickY;
  bool _joystickButton, _buttonA, _buttonB;
  bool _prevJoystickButton, _prevButtonA, _prevButtonB;
  unsigned long _lastInputTime;
  static constexpr unsigned long INPUT_DEBOUNCE = 300;

  // Joystick thresholds
  static constexpr int JOY_THRESHOLD_LOW = 1000;
  static constexpr int JOY_THRESHOLD_HIGH = 3000;

  // Status timeout (mark as NOT_ONLINE if no update received)
  static constexpr unsigned long STATUS_TIMEOUT = 5000;

  // Serial print timing
  unsigned long _lastSerialPrint;
  static constexpr unsigned long SERIAL_PRINT_INTERVAL = 1000;

  // Display update timing (prevent flickering)
  unsigned long _lastDisplayUpdate;
  static constexpr unsigned long DISPLAY_UPDATE_INTERVAL = 100;
  bool _displayNeedsUpdate;

  // ESP-NOW
  uint8_t _sequenceNum;

  // Menu structure
  static MenuItem _menuRoot;
  static MenuItem _menuField;
  static MenuItem _menuFieldStatus;
  static MenuItem _menuControl;
  static MenuItem _statusItems[NUM_ELEMENTS];
  static MenuItem _fieldActions[2];
  static MenuItem _controlActions[1];

  // Static instance for callbacks
  static ControllerDriver* s_instance;
  static FieldMessage _receivedMessage;
  static volatile bool _hasNewMessage;

  // Methods
  void initMenu();
  void initEspNow();
  void readInputs();
  void handleNavigation();
  void updateDisplay();
  void processMessages();
  void handleMessage(const FieldMessage& msg);
  void checkStatusTimeouts();
  void displayMenuItem(uint8_t row, MenuItem* item, bool selected);
  void displayElementStatus(ElementId id);
  void displayEarthStatus();

  // ESP-NOW callbacks
  static void onDataRecv(const uint8_t* mac, const uint8_t* data, int len);
  static void onDataSent(const uint8_t* mac, esp_now_send_status_t status);

  // Action callbacks for menu
  static void menuActionReset(ControllerDriver* ctrl);
  static void menuActionStart(ControllerDriver* ctrl);
  static void menuActionCycleDisplay(ControllerDriver* ctrl);
};

}  // namespace Field

#endif
