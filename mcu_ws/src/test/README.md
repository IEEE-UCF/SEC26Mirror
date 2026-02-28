# Hardware Tests

Each test targets a specific subsystem or integration scenario. All PlatformIO commands must be run **inside the Docker container** from `/home/ubuntu/mcu_workspaces/sec26mcu`.

General workflow for any test:

```bash
pio run -e <ENV>                          # Build
pio run -e <ENV> --target upload          # Flash
pio device monitor -e <ENV>               # Monitor serial output
```

For tests that use **micro-ROS** (serial transport), start the agent on the host machine:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 921600
```

---

## ESP32 Tests

### esp32-test-simple-wifi

Minimal WiFi connectivity check. Connects to `UCFIEEEBot` and prints connection status in a loop. No micro-ROS.

| | |
|---|---|
| **Environment** | `esp32-test-simple-wifi` |
| **Hardware** | ESP32 dev board |
| **micro-ROS** | No |
| **Baud rate** | 115200 |

```bash
pio run -e esp32-test-simple-wifi --target upload
pio device monitor -e esp32-test-simple-wifi -b 115200
```

**Expected output:**

```
Status: 6
Status: 6
Main loop
Main loop
```

Status `6` = `WL_DISCONNECTED`, then `Main loop` once connected. If it never reaches `Main loop`, check that the `UCFIEEEBot` network is available and broadcasting on 2.4 GHz.

---

### esp32-test-microros-wifi

micro-ROS over WiFi UDP. Publishes a heartbeat string topic over wireless.

| | |
|---|---|
| **Environment** | `esp32-test-microros-wifi` |
| **Hardware** | ESP32 dev board on same WiFi as agent |
| **micro-ROS** | Yes (WiFi UDP) |

See the dedicated [esp32-test-microros-wifi/README.md](esp32-test-microros-wifi/README.md) for full network setup, configuration, and troubleshooting.

---

## Teensy Tests — Individual Subsystems

### teensy-test-battery-subsystem

Tests INA219 power monitor behind TCA9548A I2C mux. Publishes battery health to micro-ROS and prints voltage/current/power to serial.

| | |
|---|---|
| **Environment** | `teensy-test-battery-subsystem` |
| **Hardware** | Teensy 4.1 + TCA9548A mux (0x70 on Wire0) + INA219 (0x40 on mux ch0) |
| **micro-ROS** | Yes (serial) |

**Wiring:**

| Device | Teensy Pin |
|--------|------------|
| TCA9548A SDA | 18 (Wire0 SDA) |
| TCA9548A SCL | 19 (Wire0 SCL) |
| INA219 | mux channel 0 (SD0/SC0) |
| Battery load | INA219 IN+/IN- with shunt |

```bash
pio run -e teensy-test-battery-subsystem --target upload
pio device monitor -e teensy-test-battery-subsystem
```

**Expected output:**

```
Battery subsystem test — TeensyThreads
Hardware: INA219 on TCA9548A mux ch0
setup(): threads started.
[ROS:--  #0] V=12.340V  I=150.2mA  P=1854.1mW
[ROS:OK  #1] V=12.338V  I=149.8mA  P=1851.5mW
```

Verify on ROS2 side:

```bash
ros2 topic echo /mcu_robot/battery_health
```

If you see `ERROR: TCA9548A init failed`, check I2C wiring. If `ERROR: INA219 init failed`, check the mux channel and INA219 address.

---

### teensy-test-sensor-subsystem

Tests VL53L0X/VL53L1X TOF distance sensors. Runs a sequence of unit-style tests (existence, init, update, readings, reset) then continuously prints distances.

| | |
|---|---|
| **Environment** | `teensy-test-sensor-subsystem` |
| **Hardware** | Teensy 4.1 + VL53L0X/VL53L1X TOF sensors on I2C |
| **micro-ROS** | No |
| **Baud rate** | 115200 |

```bash
pio run -e teensy-test-sensor-subsystem --target upload
pio device monitor -e teensy-test-sensor-subsystem -b 115200
```

**Expected output:**

```
SENSOR SUBSYSTEM MCU TEST
Teensy 4.1 - TOF Sensors
================================================

TEST: Subsystem Existence
[PASS] TOF drivers instantiated
[PASS] SensorSubsystem instantiated
...
TEST SUMMARY
...
ALL TESTS PASSED
================================================
Running... Updates: 50
  Distances: S0=123mm, S1=456mm, S2=789mm, S3=1012mm
```

The test creates 4 TOF drivers by default. If sensors are not physically connected, init may fail — this is expected and noted in the output.

---

### teensy-test-oled-subsystem

Tests the OLED subsystem with micro-ROS integration. The display acts as a serial terminal — lines can be appended via ROS2 service and scrolled via a topic.

| | |
|---|---|
| **Environment** | `teensy-test-oled-subsystem` |
| **Hardware** | Teensy 4.1 + SSD1306 OLED (software SPI) |
| **micro-ROS** | Yes (serial) |

**Wiring:**

| SSD1306 Pin | Teensy Pin |
|-------------|------------|
| D1 (MOSI) | 11 |
| D0 (SCK) | 13 |
| DC | 9 |
| CS | 10 |
| RST | 3 |
| VCC | 3.3V |
| GND | GND |

```bash
pio run -e teensy-test-oled-subsystem --target upload
pio device monitor -e teensy-test-oled-subsystem
```

**Expected behavior:** OLED shows `SEC26 OLED test` and `Connecting...`, then a heartbeat line every 5 seconds with uptime and ROS connection status.

**ROS2 interaction:**

```bash
# Append a line to the OLED
ros2 service call /mcu_robot/lcd/append mcu_msgs/srv/LCDAppend "text: 'Hello from ROS2'"

# Scroll up (see older lines)
ros2 topic pub --once /mcu_robot/lcd/scroll std_msgs/msg/Int8 "data: -1"

# Scroll down (back to live)
ros2 topic pub --once /mcu_robot/lcd/scroll std_msgs/msg/Int8 "data: 1"
```

---

### teensy-test-rc-subsystem

Tests the FlySky RC receiver via IBUS protocol. Publishes channel data to micro-ROS.

| | |
|---|---|
| **Environment** | `teensy-test-rc-subsystem` |
| **Hardware** | Teensy 4.1 + FlySky receiver (IBUS output on Serial8 RX) |
| **micro-ROS** | Yes (serial) |

**Wiring:**

| FlySky Receiver | Teensy Pin |
|-----------------|------------|
| IBUS output | 34 (Serial8 RX) |
| VCC | 5V |
| GND | GND |

```bash
pio run -e teensy-test-rc-subsystem --target upload
pio device monitor -e teensy-test-rc-subsystem
```

**Expected output:**

```
SEC26 Robot — RC Subsystem Test
setup(): RC test threads started.
RC test running — agent waiting...
RC test running — agent CONNECTED
```

Verify on ROS2 side:

```bash
ros2 topic echo /mcu_robot/rc
```

Move the RC sticks to see channel values change. Channels map 1000-2000 raw IBUS values to -255 to 255.

---

### teensy-test-arm-servos

Tests PCA9685-driven servos with micro-ROS control. Publishes servo angles and accepts angle commands via a SetServo service.

| | |
|---|---|
| **Environment** | `teensy-test-arm-servos` |
| **Hardware** | Teensy 4.1 + PCA9685 servo board (0x40 on Wire2) |
| **micro-ROS** | Yes (serial) |

**Wiring:**

| Device | Teensy Pin |
|--------|------------|
| PCA9685 SDA | 24 (Wire2 SDA) |
| PCA9685 SCL | 25 (Wire2 SCL) |
| PCA9685 OE | 20 |
| Servos | PCA9685 channels 0-15 |

```bash
pio run -e teensy-test-arm-servos --target upload
pio device monitor -e teensy-test-arm-servos
```

**Expected output:**

```
SEC26 Robot — Arm Servo Test
setup(): arm servo test threads started.
Servo test — agent waiting... | angles: 0 0 0 0 0 0 0 0
Servo test — agent CONNECTED | angles: 0 0 0 0 0 0 0 0
```

**ROS2 interaction:**

```bash
# Read servo states
ros2 topic echo /mcu_robot/servo/state

# Set servo 0 to 90 degrees
ros2 service call /mcu_robot/servo/set mcu_msgs/srv/SetServo "{index: 0, angle: 90.0}"

# Set servo 2 to 45 degrees
ros2 service call /mcu_robot/servo/set mcu_msgs/srv/SetServo "{index: 2, angle: 45.0}"
```

---

### teensy-test-drive-motors

Tests PCA9685-driven DC motors with micro-ROS control. Each motor uses two PCA9685 channels (PWM + direction). Publishes motor speeds and accepts speed commands via a SetMotor service.

| | |
|---|---|
| **Environment** | `teensy-test-drive-motors` |
| **Hardware** | Teensy 4.1 + PCA9685 motor board (0x41 on Wire2) |
| **micro-ROS** | Yes (serial) |

**Wiring:**

| Device | Teensy Pin |
|--------|------------|
| PCA9685 SDA | 24 (Wire2 SDA) |
| PCA9685 SCL | 25 (Wire2 SCL) |
| PCA9685 OE | 21 |
| Motors | PCA9685 channels (even=PWM, odd=DIR) |

Motor channel mapping: Motor 0 = ch 0,1 | Motor 1 = ch 2,3 | ... | Motor 7 = ch 14,15

```bash
pio run -e teensy-test-drive-motors --target upload
pio device monitor -e teensy-test-drive-motors
```

**Expected output:**

```
SEC26 Robot — Drive Motor Test
setup(): drive motor test threads started.
Motor test — agent waiting... | speeds: +0.00 +0.00 +0.00 +0.00
Motor test — agent CONNECTED | speeds: +0.00 +0.00 +0.00 +0.00
```

**ROS2 interaction:**

```bash
# Read motor states
ros2 topic echo /mcu_robot/motor/state

# Set motor 0 to 50% forward
ros2 service call /mcu_robot/motor/set mcu_msgs/srv/SetMotor "{index: 0, speed: 0.5}"

# Set motor 1 to 30% reverse
ros2 service call /mcu_robot/motor/set mcu_msgs/srv/SetMotor "{index: 1, speed: -0.3}"
```

Speed range is -1.0 (full reverse) to 1.0 (full forward).

---

## Teensy Tests — Full Integration

### teensy-test-all-subsystems

Wires **every** robot subsystem together in a single firmware image. Useful for verifying the full robot hardware stack before running `env:robot`.

| | |
|---|---|
| **Environment** | `teensy-test-all-subsystems` |
| **Hardware** | Full robot hardware |
| **micro-ROS** | Yes (serial) |

```bash
pio run -e teensy-test-all-subsystems --target upload
pio device monitor -e teensy-test-all-subsystems
```

See the dedicated [teensy-test-all-subsystems/README.md](teensy-test-all-subsystems/README.md) for full hardware requirements, ROS2 topic/service reference, and comprehensive testing commands.

---

## Troubleshooting

### micro-ROS agent not connecting

1. Check the USB cable is connected and recognized: `ls /dev/ttyACM*`
2. Start the agent with the correct baud rate:
   ```bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 921600
   ```
3. If the Teensy appears as a different device (e.g., `/dev/ttyACM1`), update the device path
4. Power-cycle the Teensy after starting the agent

### I2C device not found

1. Check wiring (SDA/SCL swapped is a common mistake)
2. Verify pull-up resistors are present (4.7k on each line)
3. Use an I2C scanner to confirm the device address:
   ```cpp
   Wire.begin();
   for (uint8_t addr = 1; addr < 127; addr++) {
     Wire.beginTransmission(addr);
     if (Wire.endTransmission() == 0) Serial.printf("Found 0x%02X\n", addr);
   }
   ```

### Teensy not uploading

1. Press the physical button on the Teensy to enter bootloader mode
2. Make sure `teensy-cli` is available (upload protocol in `platformio.ini`)
3. Check that no serial monitor is holding the port open

### CrashReport

Several tests print `CrashReport` on startup. If you see crash data, it means the previous firmware crashed. Read the report for stack trace / fault info, then power-cycle.
