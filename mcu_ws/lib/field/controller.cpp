#include "controller.h"

namespace Field {

// Static members
ControllerDriver* ControllerDriver::s_instance = nullptr;
FieldMessage ControllerDriver::_receivedMessage = {};
volatile bool ControllerDriver::_hasNewMessage = false;

// Menu structure definitions
MenuItem ControllerDriver::_statusItems[NUM_STATUS_ITEMS] = {
    {"button.disp", MenuItemType::MENU_DISPLAY, nullptr, nullptr, 0, nullptr, ElementId::BUTTON},
    {"crank.disp", MenuItemType::MENU_DISPLAY, nullptr, nullptr, 0, nullptr, ElementId::CRANK},
    {"pressure.disp", MenuItemType::MENU_DISPLAY, nullptr, nullptr, 0, nullptr, ElementId::PRESSURE},
    {"keypad.disp", MenuItemType::MENU_DISPLAY, nullptr, nullptr, 0, nullptr, ElementId::KEYPAD},
    {"earth.disp", MenuItemType::MENU_DISPLAY, nullptr, nullptr, 0, nullptr, ElementId::EARTH},
    {"timeleft.disp", MenuItemType::MENU_DISPLAY, nullptr, nullptr, 0, nullptr, ElementId::CONTROLLER}
};

MenuItem ControllerDriver::_fieldActions[2] = {
    {"reset.action", MenuItemType::MENU_ACTION, nullptr, nullptr, 0, menuActionReset, ElementId::CONTROLLER},
    {"start.action", MenuItemType::MENU_ACTION, nullptr, nullptr, 0, menuActionStart, ElementId::CONTROLLER}
};

MenuItem ControllerDriver::_controlActions[1] = {
    {"cycledisplay.action", MenuItemType::MENU_ACTION, nullptr, nullptr, 0, menuActionCycleDisplay, ElementId::CONTROLLER}
};

MenuItem ControllerDriver::_menuFieldStatus = {
    "status", MenuItemType::MENU_FOLDER, nullptr, _statusItems, NUM_ELEMENTS, nullptr, ElementId::CONTROLLER
};

MenuItem ControllerDriver::_menuField = {
    "field", MenuItemType::MENU_FOLDER, nullptr, nullptr, 0, nullptr, ElementId::CONTROLLER
};

MenuItem ControllerDriver::_menuControl = {
    "control", MenuItemType::MENU_FOLDER, nullptr, _controlActions, 1, nullptr, ElementId::CONTROLLER
};

MenuItem ControllerDriver::_menuRoot = {
    "/", MenuItemType::MENU_FOLDER, nullptr, nullptr, 0, nullptr, ElementId::CONTROLLER
};

ControllerDriver::ControllerDriver(const ControllerSetup& setup)
    : _setup(setup),
      _lcd(setup.lcdAddress, 16, 2),
      _currentMenu(&_menuRoot),
      _menuIndex(0),
      _menuScroll(0),
      _joystickX(2048),
      _joystickY(2048),
      _joystickButton(false),
      _buttonA(false),
      _buttonB(false),
      _prevJoystickButton(false),
      _prevButtonA(false),
      _prevButtonB(false),
      _lastInputTime(0),
      _lastSerialPrint(0),
      _lastDisplayUpdate(0),
      _displayNeedsUpdate(true),
      _sequenceNum(0),
      _timerRunning(false),
      _timerStartTime(0) {
  for (int i = 0; i < NUM_ELEMENTS; i++) {
    _elementStatus[i] = ElementStatus::NOT_ONLINE;
    _lastStatusTime[i] = 0;
    _elementColors[i] = FieldColor::NONE;
  }
  memset(&_earthData, 0, sizeof(_earthData));
}

bool ControllerDriver::init() {
  // Initialize LCD
  _lcd.init();
  _lcd.backlight();
  _lcd.clear();
  _lcd.setCursor(0, 0);
  _lcd.print("Field Controller");
  _lcd.setCursor(0, 1);
  _lcd.print("Initializing...");

  // Initialize input pins
  pinMode(_setup.joystickButtonPin, INPUT_PULLUP);
  pinMode(_setup.buttonAPin, INPUT_PULLUP);
  pinMode(_setup.buttonBPin, INPUT_PULLUP);

  // Initialize menu structure
  initMenu();

  // Initialize ESP-NOW
  initEspNow();

  delay(1000);
  _lcd.clear();
  updateDisplay();

  Serial.println("Controller initialized");
  return true;
}

void ControllerDriver::initMenu() {
  // Set up parent-child relationships using pointers to static members
  // (avoid copying structs which would lose updates)

  // Status folder contains element displays and timer
  _menuFieldStatus.children = _statusItems;
  _menuFieldStatus.childCount = NUM_STATUS_ITEMS;
  _menuFieldStatus.parent = &_menuField;
  for (int i = 0; i < NUM_STATUS_ITEMS; i++) {
    _statusItems[i].parent = &_menuFieldStatus;
  }

  // Field folder contains: status, reset.action, start.action
  static MenuItem* fieldChildPtrs[3] = {&_menuFieldStatus, &_fieldActions[0], &_fieldActions[1]};
  static MenuItem fieldChildren[3];
  fieldChildren[0] = _menuFieldStatus;
  fieldChildren[1] = _fieldActions[0];
  fieldChildren[2] = _fieldActions[1];
  _menuField.children = fieldChildren;
  _menuField.childCount = 3;
  _menuField.parent = &_menuRoot;
  for (int i = 0; i < 3; i++) {
    _menuField.children[i].parent = &_menuField;
  }
  // Re-link status children since we copied
  _menuField.children[0].children = _statusItems;
  _menuField.children[0].childCount = NUM_STATUS_ITEMS;

  // Control folder contains: cycledisplay.action
  _menuControl.children = _controlActions;
  _menuControl.childCount = 1;
  _menuControl.parent = &_menuRoot;
  _controlActions[0].parent = &_menuControl;

  // Root contains: field, control
  static MenuItem rootChildren[2];
  rootChildren[0] = _menuField;
  rootChildren[1] = _menuControl;
  _menuRoot.children = rootChildren;
  _menuRoot.childCount = 2;
  for (int i = 0; i < 2; i++) {
    _menuRoot.children[i].parent = &_menuRoot;
  }
  // Re-link field children since we copied
  _menuRoot.children[0].children = _menuField.children;
  _menuRoot.children[0].childCount = _menuField.childCount;
  // Re-link control children
  _menuRoot.children[1].children = _controlActions;
  _menuRoot.children[1].childCount = 1;

  _currentMenu = &_menuRoot;
}

void ControllerDriver::initEspNow() {
  // Ensure clean WiFi state
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(100);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // Disable WiFi sleep for reliable ESP-NOW
  delay(100);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  s_instance = this;
  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  // Add broadcast peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, BROADCAST_MAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  // Print MAC
  uint8_t mac[6];
  WiFi.macAddress(mac);
  Serial.printf("Controller MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void ControllerDriver::update() {
  unsigned long now = millis();

  readInputs();
  handleNavigation();
  processMessages();
  checkStatusTimeouts();

  // Force display update when timer is running and timer display is shown
  if (_timerRunning && _currentMenu->childCount > 0) {
    MenuItem* selected = &_currentMenu->children[_menuIndex];
    if (selected->type == MenuItemType::MENU_DISPLAY &&
        selected->elementId == ElementId::CONTROLLER) {
      _displayNeedsUpdate = true;
    }
  }

  // Only update display when needed and throttled
  if (_displayNeedsUpdate && (now - _lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL)) {
    _lastDisplayUpdate = now;
    _displayNeedsUpdate = false;
    updateDisplay();
  }

  // Print status to serial every second
  if (now - _lastSerialPrint >= SERIAL_PRINT_INTERVAL) {
    _lastSerialPrint = now;
    printStatusToSerial();
  }
}

void ControllerDriver::readInputs() {
  _joystickX = analogRead(_setup.joystickXPin);
  _joystickY = analogRead(_setup.joystickYPin);

  _prevJoystickButton = _joystickButton;
  _prevButtonA = _buttonA;
  _prevButtonB = _buttonB;

  _joystickButton = !digitalRead(_setup.joystickButtonPin);
  _buttonA = !digitalRead(_setup.buttonAPin);
  _buttonB = !digitalRead(_setup.buttonBPin);
}

void ControllerDriver::handleNavigation() {
  unsigned long now = millis();
  if (now - _lastInputTime < INPUT_DEBOUNCE) return;

  bool inputHandled = false;

  // Joystick up/down for menu navigation
  if (_joystickY < JOY_THRESHOLD_LOW && _currentMenu->childCount > 0) {
    // Up
    if (_menuIndex > 0) {
      _menuIndex--;
      if (_menuIndex < _menuScroll) _menuScroll = _menuIndex;
      inputHandled = true;
    }
  } else if (_joystickY > JOY_THRESHOLD_HIGH && _currentMenu->childCount > 0) {
    // Down
    if (_menuIndex < _currentMenu->childCount - 1) {
      _menuIndex++;
      if (_menuIndex >= _menuScroll + 2) _menuScroll = _menuIndex - 1;
      inputHandled = true;
    }
  }

  // Button A or Joystick button: select/enter
  if ((_buttonA && !_prevButtonA) || (_joystickButton && !_prevJoystickButton)) {
    if (_currentMenu->childCount > 0) {
      MenuItem* selected = &_currentMenu->children[_menuIndex];

      if (selected->type == MenuItemType::MENU_FOLDER) {
        _currentMenu = selected;
        _menuIndex = 0;
        _menuScroll = 0;
        inputHandled = true;
      } else if (selected->type == MenuItemType::MENU_ACTION && selected->action) {
        selected->action(this);
        inputHandled = true;
      } else if (selected->type == MenuItemType::MENU_DISPLAY) {
        // Display items show on LCD when selected (handled in updateDisplay)
        inputHandled = true;
      }
    }
  }

  // Button B or Joystick X (left): back/parent
  if ((_buttonB && !_prevButtonB) || (_joystickX < JOY_THRESHOLD_LOW)) {
    if (_currentMenu->parent) {
      _currentMenu = _currentMenu->parent;
      _menuIndex = 0;
      _menuScroll = 0;
      inputHandled = true;
    }
  }

  if (inputHandled) {
    _lastInputTime = now;
    _displayNeedsUpdate = true;
  }
}

void ControllerDriver::updateDisplay() {
  _lcd.clear();

  if (_currentMenu->childCount == 0) {
    // Empty folder
    _lcd.setCursor(0, 0);
    _lcd.print(_currentMenu->name);
    _lcd.setCursor(0, 1);
    _lcd.print("(empty)");
    return;
  }

  MenuItem* selected = &_currentMenu->children[_menuIndex];

  // If display item is selected, show its content
  if (selected->type == MenuItemType::MENU_DISPLAY) {
    if (selected->elementId == ElementId::EARTH) {
      displayEarthStatus();
    } else if (selected->elementId == ElementId::CONTROLLER) {
      displayTimeLeft();
    } else {
      displayElementStatus(selected->elementId);
    }
    return;
  }

  // Show folder contents
  for (int row = 0; row < 2; row++) {
    uint8_t itemIdx = _menuScroll + row;
    if (itemIdx < _currentMenu->childCount) {
      displayMenuItem(row, &_currentMenu->children[itemIdx], itemIdx == _menuIndex);
    }
  }
}

void ControllerDriver::displayMenuItem(uint8_t row, MenuItem* item, bool selected) {
  _lcd.setCursor(0, row);
  _lcd.print(selected ? ">" : " ");
  _lcd.print(item->name);
}

void ControllerDriver::displayElementStatus(ElementId id) {
  uint8_t idx = static_cast<uint8_t>(id) - 1;  // Offset since CONTROLLER=0
  if (idx >= NUM_ELEMENTS) return;

  _lcd.setCursor(0, 0);
  _lcd.print(elementIdToString(id));

  _lcd.setCursor(0, 1);
  _lcd.print(statusToString(_elementStatus[idx]));
}

void ControllerDriver::displayEarthStatus() {
  _lcd.setCursor(0, 0);
  _lcd.print("EARTH ");
  _lcd.print(statusToString(_elementStatus[4]));

  _lcd.setCursor(0, 1);
  _lcd.print("C:");
  _lcd.print(_earthData.correctCount);
  _lcd.print(" W:");
  _lcd.print(_earthData.wrongCount);
}

void ControllerDriver::displayTimeLeft() {
  _lcd.setCursor(0, 0);
  _lcd.print("TIME LEFT");

  _lcd.setCursor(0, 1);
  if (!_timerRunning) {
    _lcd.print("--:--");
  } else {
    unsigned long elapsed = millis() - _timerStartTime;
    if (elapsed >= TIMER_DURATION_MS) {
      _lcd.print("00:00");
      _timerRunning = false;
    } else {
      unsigned long remaining = TIMER_DURATION_MS - elapsed;
      uint8_t minutes = remaining / 60000;
      uint8_t seconds = (remaining % 60000) / 1000;
      char buf[6];
      snprintf(buf, sizeof(buf), "%02d:%02d", minutes, seconds);
      _lcd.print(buf);
    }
  }
}

void ControllerDriver::processMessages() {
  if (_hasNewMessage) {
    _hasNewMessage = false;
    handleMessage(_receivedMessage);
  }
}

void ControllerDriver::handleMessage(const FieldMessage& msg) {
  switch (msg.header.type) {
    case MessageType::STATUS_UPDATE: {
      uint8_t idx = static_cast<uint8_t>(msg.statusUpdate.header.senderId) - 1;
      if (idx < NUM_ELEMENTS) {
        _elementStatus[idx] = msg.statusUpdate.status;
        _lastStatusTime[idx] = millis();
        _displayNeedsUpdate = true;
      }
      break;
    }

    case MessageType::COLOR_REPORT: {
      // Store color for antenna element
      uint8_t idx = static_cast<uint8_t>(msg.colorReport.header.senderId) - 1;
      if (idx < 4) {  // Antennas 1-4
        _elementColors[idx] = msg.colorReport.color;
      }
      break;
    }

    case MessageType::COLOR_RESULT: {
      // Update earth scoring data
      _earthData.correctCount = msg.colorResult.correctCount;
      _earthData.wrongCount = msg.colorResult.wrongCount;
      for (int i = 0; i < 4; i++) {
        _earthData.expectedColors[i] = msg.colorResult.results[i].expectedColor;
        _earthData.receivedColors[i] = msg.colorResult.results[i].receivedColor;
        _earthData.colorCorrect[i] = msg.colorResult.results[i].correct;
      }
      _displayNeedsUpdate = true;
      break;
    }

    default:
      break;
  }
}

void ControllerDriver::checkStatusTimeouts() {
  unsigned long now = millis();
  for (int i = 0; i < NUM_ELEMENTS; i++) {
    if (_elementStatus[i] != ElementStatus::NOT_ONLINE &&
        now - _lastStatusTime[i] > STATUS_TIMEOUT) {
      _elementStatus[i] = ElementStatus::NOT_ONLINE;
    }
  }
}

void ControllerDriver::printStatusToSerial() {
  Serial.println("=== Field Status ===");
  Serial.printf("BUTTON:   %s\n", statusToString(_elementStatus[0]));
  Serial.printf("CRANK:    %s\n", statusToString(_elementStatus[1]));
  Serial.printf("PRESSURE: %s\n", statusToString(_elementStatus[2]));
  Serial.printf("KEYPAD:   %s\n", statusToString(_elementStatus[3]));
  Serial.printf("EARTH:    %s (C:%d W:%d)\n",
                statusToString(_elementStatus[4]),
                _earthData.correctCount, _earthData.wrongCount);
  Serial.println("====================");
}

ElementStatus ControllerDriver::getElementStatus(ElementId id) const {
  uint8_t idx = static_cast<uint8_t>(id) - 1;
  if (idx >= NUM_ELEMENTS) return ElementStatus::NOT_ONLINE;
  return _elementStatus[idx];
}

void ControllerDriver::sendCommand(FieldCommand cmd) {
  sendCommand(cmd, static_cast<ElementId>(0xFF));  // Broadcast
}

void ControllerDriver::sendCommand(FieldCommand cmd, ElementId target) {
  CommandMessage msg;
  msg.header.type = MessageType::COMMAND;
  msg.header.senderId = ElementId::CONTROLLER;
  msg.header.sequenceNum = _sequenceNum++;
  msg.command = cmd;
  msg.targetId = target;

  esp_now_send(BROADCAST_MAC, (uint8_t*)&msg, sizeof(msg));
  Serial.printf("Sent command: %d to %d\n", static_cast<int>(cmd), static_cast<int>(target));
}

void ControllerDriver::actionReset() {
  Serial.println("Executing RESET action");
  sendCommand(FieldCommand::RESET);
  _timerRunning = false;
  _timerStartTime = 0;
}

void ControllerDriver::actionStart() {
  Serial.println("Executing START action");
  sendCommand(FieldCommand::START);
  _timerRunning = true;
  _timerStartTime = millis();
}

void ControllerDriver::actionCycleDisplay() {
  Serial.println("Executing CYCLE_DISPLAY action");
  sendCommand(FieldCommand::CYCLE_DISPLAY);
}

// Static menu action callbacks
void ControllerDriver::menuActionReset(ControllerDriver* ctrl) {
  ctrl->actionReset();
}

void ControllerDriver::menuActionStart(ControllerDriver* ctrl) {
  ctrl->actionStart();
}

void ControllerDriver::menuActionCycleDisplay(ControllerDriver* ctrl) {
  ctrl->actionCycleDisplay();
}

// ESP-NOW callbacks
void ControllerDriver::onDataRecv(const uint8_t* mac, const uint8_t* data, int len) {
  if (s_instance && len <= sizeof(FieldMessage)) {
    memcpy((void*)&_receivedMessage, data, len);
    _hasNewMessage = true;
  }
}

void ControllerDriver::onDataSent(const uint8_t* mac, esp_now_send_status_t status) {
  // Optional logging
}

}  // namespace Field
