# Hardware Abstraction Layer (HAL) for GPIO Pins

The HAL system provides a unified interface for accessing GPIO pins across different hardware backends, allowing subsystems to work with pins without knowing whether they're connected to:
- Native MCU pins
- PCA9685 PWM driver channels
- MCP23017 I/O expander pins
- CD74HC4067 multiplexer channels

## Architecture

### HAL Pin Interface (`HALPin.h`)
Abstract base class defining the common pin interface:
- `pinMode()` - Configure pin mode
- `digitalWrite()` - Write digital value
- `digitalRead()` - Read digital value
- `analogWrite()` - Write PWM/analog value
- `analogRead()` - Read analog value

### HAL Backends

#### NativeGPIO (`NativeGPIO.h`)
Direct access to MCU GPIO pins.

**Capabilities:**
- ✓ digitalWrite
- ✓ digitalRead
- ✓ analogWrite (PWM)
- ✓ analogRead (ADC)

**Example:**
```cpp
#include <hal/NativeGPIO.h>

// Create a native GPIO pin for pin 13
HAL::NativeGPIO led(13);
led.pinMode(HAL::PinMode::OUTPUT);
led.digitalWrite(HAL::DigitalState::HIGH);
```

#### PCA9685GPIO (`PCA9685GPIO.h`)
Access to PCA9685 PWM driver channels (16 channels per device).

**Capabilities:**
- ✓ digitalWrite (full on/off)
- ✗ digitalRead (output only)
- ✓ analogWrite (12-bit PWM, 0-4095)
- ✗ analogRead (output only)

**Example:**
```cpp
#include <hal/PCA9685GPIO.h>
#include <robot/PCA9685Driver.h>

// Create PCA9685 driver
Robot::PCA9685DriverSetup pca_setup("pca0", 0x40, 50);
Robot::PCA9685Driver pca_driver(pca_setup);
pca_driver.init();

// Create HAL pin for channel 0
HAL::PCA9685GPIO servo_pin(&pca_driver, 0);
servo_pin.analogWrite(2048);  // 50% duty cycle
```

#### MCP23017GPIO (`MCP23017GPIO.h`)
Access to MCP23017 I/O expander pins (16 pins per device).

**Capabilities:**
- ✓ digitalWrite
- ✓ digitalRead
- ✗ analogWrite (digital only)
- ✗ analogRead (digital only)

**Example:**
```cpp
#include <hal/MCP23017GPIO.h>
#include <drivers/MCP23017Driver.h>

// Create MCP23017 driver
Drivers::MCP23017DriverSetup mcp_setup("mcp0", 0x20);
Drivers::MCP23017Driver mcp_driver(mcp_setup);
mcp_driver.init();

// Create HAL pin for pin 5
HAL::MCP23017GPIO button_pin(&mcp_driver, 5);
button_pin.pinMode(HAL::PinMode::INPUT_PULLUP);
if (button_pin.digitalRead() == HAL::DigitalState::LOW) {
    // Button pressed
}
```

#### CD74HC4067GPIO (`CD74HC4067GPIO.h`)
Access to CD74HC4067 multiplexer channels (16 channels per device).

**Capabilities:**
- ✓ digitalWrite
- ✓ digitalRead
- ✗ analogWrite (passive mux)
- ✓ analogRead

**Important:** Each operation automatically selects the channel on the multiplexer.

**Example:**
```cpp
#include <hal/CD74HC4067GPIO.h>
#include <drivers/CD74HC4067Driver.h>

// Create CD74HC4067 driver (S0-S3 select pins, sig pin)
Drivers::CD74HC4067DriverSetup mux_setup("mux0", 2, 3, 4, 5, A0);
Drivers::CD74HC4067Driver mux_driver(mux_setup);
mux_driver.init();

// Create HAL pins for channels
HAL::CD74HC4067GPIO sensor0(&mux_driver, 0);
HAL::CD74HC4067GPIO sensor1(&mux_driver, 1);

// Read analog values (channel is auto-selected)
uint16_t value0 = sensor0.analogRead();
uint16_t value1 = sensor1.analogRead();
```

## Usage in Drivers

### Example: Motor Driver with HAL

```cpp
#include <hal/NativeGPIO.h>
#include <hal/PCA9685GPIO.h>
#include "MotorDriver.h"

// Option 1: Using native GPIO pins
HAL::NativeGPIO pwm_pin(9);
HAL::NativeGPIO dir_pin(8);

Drivers::MotorDriverSetup motor_setup("motor1", &pwm_pin, &dir_pin);
Drivers::MotorDriver motor(motor_setup);
motor.init();

// Option 2: Using PCA9685 for PWM
Robot::PCA9685Driver pca_driver(...);
HAL::PCA9685GPIO pwm_pin_pca(&pca_driver, 0);
HAL::NativeGPIO dir_pin_native(8);

Drivers::MotorDriverSetup motor_setup2("motor2", &pwm_pin_pca, &dir_pin_native);
Drivers::MotorDriver motor2(motor_setup2);
motor2.init();
```

## Capability Checking

Always check capabilities before using features:

```cpp
HAL::HALPin* pin = ...;  // Could be any backend

if (pin->supportsAnalogWrite()) {
    pin->analogWrite(128);
}

if (pin->supportsDigitalRead()) {
    auto state = pin->digitalRead();
}
```

## Adding New Hardware Backends

To add support for new hardware:

1. **Create a driver** (if needed) inheriting from `BaseDriver`
2. **Create HAL backend** inheriting from `HALPin`
3. **Implement all virtual methods** from `HALPin`
4. **Set capability flags** correctly (`supports*()` methods)

Example skeleton:

```cpp
class MyCustomGPIO : public HAL::HALPin {
public:
  MyCustomGPIO(MyDriver* driver, uint8_t pin)
    : driver_(driver), pin_(pin) {}

  void pinMode(HAL::PinMode mode) override { /* ... */ }
  void digitalWrite(HAL::DigitalState state) override { /* ... */ }
  HAL::DigitalState digitalRead() override { /* ... */ }
  void analogWrite(uint16_t value) override { /* ... */ }
  uint16_t analogRead() override { /* ... */ }
  const char* getInfo() override { /* ... */ }

  bool supportsDigitalRead() const override { return true; }
  bool supportsAnalogWrite() const override { return false; }
  bool supportsAnalogRead() const override { return true; }

private:
  MyDriver* driver_;
  uint8_t pin_;
  char infoBuffer_[32];
};
```

## Benefits

1. **Flexibility**: Change pin backends without modifying driver code
2. **Testability**: Mock HAL pins for unit testing
3. **Expandability**: Easy to add new hardware support
4. **Pin conservation**: Use I/O expanders when running out of MCU pins
5. **Abstraction**: Subsystems don't need hardware-specific knowledge
