
#include "keypad.h"

namespace Field {

bool KeypadDriver::init() {
  // Reinitialize pins (constructor runs before Serial.begin)
  for (int col = 0; col < _num_cols; col++) {
    pinMode(_column_pins[col], OUTPUT);
    digitalWrite(_column_pins[col], HIGH);
  }
  for (int row = 0; row < _num_rows; row++) {
    pinMode(_row_pins[row], INPUT_PULLUP);
  }

  Serial.println("Keypad initialized");
  return true;
}

void KeypadDriver::update() {
  for (int col = 0; col < _num_cols; col++) {
    digitalWrite(_column_pins[col], LOW);
    for (int row = 0; row < _num_rows; row++) {
      if (digitalRead(_row_pins[row]) == LOW) {
        char key = _keymap[row * _num_cols + col];

        if ((_prev_key != key) ||
            (_prev_key == key && (millis() - _prev_time) > _debounce_time)) {
          _prev_key = key;
          _prev_time = millis();
          Serial.println(key);

          if (key == '#') {
            // Enter key pressed - check password
            if (password == actualPassword) {
              Serial.println("PASSWORD CORRECT!");
              task = COMPLETE;
            } else {
              Serial.println("PASSWORD INCORRECT!");
              task = NOTCOMPLETE;
            }
            password = "";
          } else {
            // Add character to password
            password += key;

            // Reset if more than 5 characters entered
            if (password.length() > sizepassword) {
              Serial.println("TOO MANY CHARS - RESET");
              password = "";
            }
          }

          digitalWrite(_column_pins[col], HIGH);
        }
      }
    }
    digitalWrite(_column_pins[col], HIGH);
  }
}

const char* KeypadDriver::getInfo() {
  return ("\nID: " + std::string(setup_.getId()) +
          "\nNum Rows: " + std::to_string(_num_rows) + "\nNum Cols: " +
          std::to_string(_num_cols) + "\nKeymap: " + std::to_string(*_keymap) +
          "\nRow Pins: " + std::to_string(*_row_pins) +
          "\nCol Pins: " + std::to_string(*_column_pins) +
          "\nPrev Key: " + std::to_string(_prev_key) +
          "\nDebounce Time: " + std::to_string(_debounce_time) +
          "\nPrev Time: " + std::to_string(_prev_time))
      .c_str();
}

bool KeypadDriver::getStatus() {
  if (task == COMPLETE) {
    return 1;
  } else {
    return 0;
  }
}

char KeypadDriver::getKey() {
  char key = _prev_key;
  _prev_key = 0;
  return key;
}

void KeypadDriver::setDebounceTime(unsigned long time) {
  _debounce_time = time;
}

void KeypadDriver::reset() {
  _prev_key = 0;
  _debounce_time = 300;
  _prev_time = 0;

  task = NOTCOMPLETE;

  for (int col = 0; col < _num_cols; col++) {
    pinMode(_column_pins[col], OUTPUT);
    digitalWrite(_column_pins[col], HIGH);
    for (int row = 0; row < _num_rows; row++) {
      pinMode(_row_pins[row], INPUT_PULLUP);
    }
  }
}

};  // namespace Field
