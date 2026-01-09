# Hardware Abstraction Layer (HAL) Implementation Summary

## Overview

A complete Hardware Abstraction Layer has been implemented for the SEC26 robot firmware, allowing subsystems to access GPIO pins through different hardware backends without knowing the underlying implementation.

## What Was Implemented

### 1. HAL Core Interface
**File:** `mcu_ws/lib/hal/HALPin.h`

- Abstract `HALPin` class defining common GPIO interface
- Methods: `pinMode()`, `digitalWrite()`, `digitalRead()`, `analogWrite()`, `analogRead()`
- Capability checking: `supportsDigitalRead()`, `supportsAnalogWrite()`, `supportsAnalogRead()`
- Pin modes: `INPUT`, `OUTPUT`, `INPUT_PULLUP`
- Digital states: `LOW`, `HIGH`

### 2. HAL Backends

#### NativeGPIO (`mcu_ws/lib/hal/NativeGPIO.h`)
- Direct access to MCU GPIO pins
- Full support: digital I/O, PWM, ADC
- Uses standard Arduino functions

#### PCA9685GPIO (`mcu_ws/lib/hal/PCA9685GPIO.h`)
- Access to PCA9685 PWM driver channels (16 channels)
- Supports: digitalWrite, analogWrite (12-bit PWM)
- Output-only (no read capability)

#### MCP23017GPIO (`mcu_ws/lib/hal/MCP23017GPIO.h`)
- Access to MCP23017 I/O expander pins (16 pins)
- Supports: digitalWrite, digitalRead, pinMode with pullup
- Digital-only (no analog)

#### CD74HC4067GPIO (`mcu_ws/lib/hal/CD74HC4067GPIO.h`)
- Access to CD74HC4067 multiplexer channels (16 channels)
- Supports: digitalRead, digitalWrite, analogRead
- Auto-selects channel on each operation

### 3. New Drivers Created

#### MCP23017Driver (`mcu_ws/lib/drivers/MCP23017Driver.h/cpp`)
- **NEW** driver for MCP23017 16-bit I2C I/O expander
- Inherits from `BaseDriver` (SEC-Base-Classes pattern)
- Methods: `pinMode()`, `digitalWrite()`, `digitalRead()`
- Uses Adafruit_MCP23X17 library

#### CD74HC4067Driver (`mcu_ws/lib/drivers/CD74HC4067Driver.h/cpp`)
- **REFACTORED** from minimal stub to proper driver
- Inherits from `BaseDriver`
- Methods: `selectChannel()`, `digitalWrite()`, `digitalRead()`, `analogRead()`
- Manages 4 select pins + signal pin

### 4. Updated Subsystems

#### MotorDriver (`mcu_ws/src/robot/drive-base/MotorDriver.h/cpp`)
- **REFACTORED** to use HAL pins instead of hardcoded pin numbers
- `MotorDriverSetup` now takes `HAL::HALPin*` instead of `int` pin numbers
- Fully backward compatible through HAL
- Supports any combination of hardware backends

## File Structure

```
mcu_ws/
├── lib/
│   ├── hal/
│   │   ├── HALPin.h                          # Core HAL interface
│   │   ├── NativeGPIO.h                      # Native MCU pins
│   │   ├── PCA9685GPIO.h                     # PCA9685 PWM channels
│   │   ├── MCP23017GPIO.h                    # MCP23017 I/O pins
│   │   ├── CD74HC4067GPIO.h                  # Multiplexer channels
│   │   ├── README.md                         # HAL documentation
│   │   └── HAL_IMPLEMENTATION_SUMMARY.md     # This file
│   │
│   └── drivers/
│       ├── MCP23017Driver.h/cpp              # NEW: MCP23017 driver
│       └── CD74HC4067Driver.h/cpp            # REFACTORED: Proper driver
│
└── src/robot/drive-base/
    ├── MotorDriver.h/cpp                     # UPDATED: Uses HAL
    └── DriveBaseHALConfig.example.h          # Example configurations
```

## Hardware Capabilities Matrix

| Hardware      | digitalWrite | digitalRead | analogWrite | analogRead |
|---------------|--------------|-------------|-------------|------------|
| NativeGPIO    | ✓            | ✓           | ✓ (8-bit)   | ✓ (10-bit) |
| PCA9685GPIO   | ✓            | ✗           | ✓ (12-bit)  | ✗          |
| MCP23017GPIO  | ✓            | ✓           | ✗           | ✗          |
| CD74HC4067GPIO| ✓            | ✓           | ✗           | ✓ (10-bit) |

## Usage Examples

### Basic Usage
```cpp
#include <hal/NativeGPIO.h>

// Create a HAL pin for native GPIO pin 13
HAL::NativeGPIO led(13);
led.pinMode(HAL::PinMode::OUTPUT);
led.digitalWrite(HAL::DigitalState::HIGH);
```

### Motor with PCA9685 PWM
```cpp
#include <hal/PCA9685GPIO.h>
#include <hal/NativeGPIO.h>

// PCA9685 for PWM, native GPIO for direction
Robot::PCA9685Driver pca(...);
HAL::PCA9685GPIO pwm_pin(&pca, 0);
HAL::NativeGPIO dir_pin(8);

Drivers::MotorDriverSetup motor_setup("motor1", &pwm_pin, &dir_pin);
Drivers::MotorDriver motor(motor_setup);
motor.init();
motor.setPWM(128);
motor.update();
```

### Multiple Sensors with Multiplexer
```cpp
#include <hal/CD74HC4067GPIO.h>

Drivers::CD74HC4067Driver mux(...);
HAL::CD74HC4067GPIO sensor0(&mux, 0);
HAL::CD74HC4067GPIO sensor1(&mux, 1);

uint16_t val0 = sensor0.analogRead();  // Auto-selects channel 0
uint16_t val1 = sensor1.analogRead();  // Auto-selects channel 1
```

## Benefits

1. **Hardware Flexibility**: Change pin backends without modifying driver code
2. **Pin Conservation**: Use I/O expanders when running out of MCU pins
3. **Testability**: Can mock HAL pins for unit testing
4. **Expandability**: Easy to add new hardware support
5. **Backward Compatibility**: Existing code works with HAL through NativeGPIO
6. **Mixed Configurations**: Different pins can use different hardware

## Migration Guide

### Old Code (Hardcoded Pins)
```cpp
Drivers::MotorDriverSetup motor_setup("motor", 9, 8);  // PWM=9, DIR=8
```

### New Code (HAL)
```cpp
HAL::NativeGPIO pwm_pin(9);
HAL::NativeGPIO dir_pin(8);
Drivers::MotorDriverSetup motor_setup("motor", &pwm_pin, &dir_pin);
```

### With PCA9685
```cpp
Robot::PCA9685Driver pca(...);
HAL::PCA9685GPIO pwm_pin(&pca, 0);    // PCA9685 channel 0
HAL::NativeGPIO dir_pin(8);            // Still use native for direction
Drivers::MotorDriverSetup motor_setup("motor", &pwm_pin, &dir_pin);
```

## Adding New Hardware

To add support for new hardware:

1. Create a driver (if needed) inheriting from `BaseDriver`
2. Create HAL backend class inheriting from `HALPin`
3. Implement all virtual methods
4. Set capability flags correctly
5. Add to documentation

See `lib/hal/README.md` for detailed instructions.

## Testing

The HAL system should be tested with:

1. **Native GPIO**: Verify all pin modes work
2. **PCA9685**: Test PWM output and digital on/off
3. **MCP23017**: Test digital I/O and pullups
4. **CD74HC4067**: Test channel selection and analog/digital read
5. **Mixed configs**: Verify different backends work together

## Dependencies

The HAL system requires:

- **Arduino framework**: Core GPIO functions
- **SEC-Base-Classes**: BaseDriver, BaseSetup patterns
- **Adafruit_PWMServoDriver**: For PCA9685
- **Adafruit_MCP23X17**: For MCP23017

These are already defined in `platformio.ini`.

## Notes

- EncoderDriver was NOT updated to use HAL because encoder pins need interrupt capability, which should remain on native hardware
- The old `CD74HC4067.h` stub is kept for backward compatibility with AnalogMuxDriver
- All HAL backends use const char* getInfo() following the project pattern
- PWM values use full resolution of backend (8-bit for native, 12-bit for PCA9685)

## Future Enhancements

Potential improvements:

1. Add interrupt support to HAL interface
2. Create HAL backends for other I/O expanders (PCF8574, etc.)
3. Add SPI-based GPIO expanders
4. Create HAL-aware test mocks
5. Add pin configuration validation
6. Implement pin aliasing/naming system

## Conclusion

The HAL system provides a clean, extensible architecture for GPIO access across multiple hardware backends. It maintains backward compatibility while enabling flexible hardware configurations for the SEC26 robot.
