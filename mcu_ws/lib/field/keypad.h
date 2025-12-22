
/**
 * @file keypad.h
 * @author Jonathan Hernandez-Morales
 * @brief Defines pressure functionality
 * @date 12/12/2025
 */

#ifndef KEYPAD_H
#define KEYPAD_H

#include <Arduino.h>
#include <BaseDriver.h>

// just a pointer to whatever variable x maybe.....
#define makeKeymap(x) ((char*)x)
#define sizepassword 6

namespace Field {

class KeypadSetup : public Classes::BaseSetup {
 public:
  ~KeypadSetup() = default;
  KeypadSetup() = delete;

  KeypadSetup(const char* _id) : BaseSetup(_id){};
};

class KeypadDriver : public Classes::BaseDriver {
 public:
  ~KeypadDriver() = default;

  KeypadDriver(const KeypadSetup& setup, char* keymap, byte* rowPins,
               byte* columnPins, byte numRows, byte numCols)
      : BaseDriver(setup),
        _keymap(keymap),
        _row_pins(rowPins),
        _column_pins(columnPins),
        _num_rows(numRows),
        _num_cols(numCols),
        _prev_key(0),
        _debounce_time(300),  // default value is 300ms
        _prev_time(0) {
    task = NOTCOMPLETE;

    for (int col = 0; col < _num_cols; col++) {
      pinMode(_column_pins[col], OUTPUT);
      digitalWrite(_column_pins[col], HIGH);
      for (int row = 0; row < _num_rows; row++) {
        pinMode(_row_pins[row], INPUT_PULLUP);
      }
    }
  };

  enum TASK { COMPLETE, NOTCOMPLETE } task;

  /// @brief  Initialize driver
  /// @return Success
  bool init() override;

  /// @brief Update driver
  void update() override;

  /// @brief Get info in the form of a data string
  /// @return data string
  const char* getInfo() override;

  /// @brief  Get status
  /// @return true if task completed
  bool getStatus();

  /// @brief Get the key
  char getKey();

  /// @brief Set the debounce time
  void setDebounceTime(unsigned long time);

  /// @brief  Reset keypad task
  void reset();

 private:
  byte _num_rows;
  byte _num_cols;
  char* _keymap;
  // example r1,r2,r3,r4-->23,22,21,19
  // Pinout for keypad-->2,7,6,4
  byte* _row_pins;
  // example c1,c2,c3-->25,33,18
  // Pinout for keypad-->3,1,5
  byte* _column_pins;
  char _prev_key;
  unsigned long _debounce_time;
  unsigned long _prev_time;
  const String actualPassword = "73738#";
  String password;
};

};  // namespace Field
#endif
