# Field Element Firmware

Firmware for the SEC26 competition field elements. Five task elements (button, crank, pressure, keypad, earth) and one controller communicate via ESP-NOW wireless protocol. These do **not** use micro-ROS.

## Overview

| Element | ID | Task | Build Env |
|---------|-----|------|-----------|
| Controller | 0 | LCD menu, monitoring, commands | `field-controller` |
| Button | 1 | Press button 3 times | `field-button` |
| Crank | 2 | Rotate encoder ±18 clicks | `field-crank` |
| Pressure | 3 | Remove duck from pressure pad | `field-pressure` |
| Keypad | 4 | Enter password "73738" | `field-keypad` |
| Earth | 5 | Receive and verify antenna IR signals | `field-earth` |

All elements are ESP32-based and communicate via **ESP-NOW** broadcast (not micro-ROS/ROS2).

## Building and Flashing

All commands run inside the Docker container.

```bash
cd /home/ubuntu/mcu_workspaces/sec26mcu

# Build individual elements
pio run -e field-button
pio run -e field-crank
pio run -e field-pressure
pio run -e field-keypad
pio run -e field-earth
pio run -e field-controller

# Flash via USB
pio run -e field-button --target upload

# Serial monitor (921600 baud)
pio device monitor -e field-button
```

## Hardware

### Button Element

| Function | Pin |
|----------|-----|
| Switch input | GPIO 5 |
| LED indicator | GPIO 32 |
| RGB Red | GPIO 15 |
| RGB Green | GPIO 2 |
| RGB Blue | GPIO 4 |
| Status LEDs (R/Y/G) | GPIO 26, 25, 33 |

**Task:** Press button 3 times. Status LEDs light up sequentially (R→Y→G). On completion, RGB shows assigned antenna color.

**Libraries:** `ezButton` (50ms debounce)

### Crank Element

| Function | Pin |
|----------|-----|
| Encoder CLK | GPIO 25 |
| Encoder DT | GPIO 33 |
| LED indicator | GPIO 13 |
| RGB Red | GPIO 27 |
| RGB Green | GPIO 14 |
| RGB Blue | GPIO 26 |

**Task:** Rotate KY-040 encoder ±18 clicks in either direction. LED turns on at threshold. RGB shows assigned color.

### Pressure Element

| Function | Pin |
|----------|-----|
| Force sensor (ADC) | GPIO 34 (ADC1 only — ADC2 blocked by WiFi) |
| LED indicator | GPIO 4 |
| RGB Red | GPIO 18 |
| RGB Green | GPIO 5 |
| RGB Blue | GPIO 19 |

**Task flow:**
1. **READY** (before START): Duck should be on pad. RGB blinks if duck is missing.
2. **NOT_ACTIVATED** (after START): Waiting for duck removal.
3. **ACTIVATED**: Duck removed (ADC reading ≤ 30). RGB shows assigned color.

ADC threshold: ≤30 = duck OFF, >30 = duck ON. Sampled every 500ms.

### Keypad Element

| Function | Pin |
|----------|-----|
| Row pins (INPUT_PULLUP) | GPIO 23, 22, 21, 19 |
| Column pins (OUTPUT) | GPIO 33, 25, 18 |
| LED indicator | GPIO 4 |
| RGB Red | GPIO 5 |
| RGB Green | GPIO 16 |
| RGB Blue | GPIO 17 |

**Matrix layout:**
```
1 2 3
4 5 6
7 8 9
* 0 #
```

**Task:** Enter password `73738`, then press `#` to submit. Incorrect entry resets. 300ms debounce.

### Earth Element

| Function | Pin |
|----------|-----|
| IR receiver | GPIO 15 |

**Task:** Receive NEC IR signals from antennas and verify colors match expectations.

**IR command byte format:**
```
[Antenna Code (4 bits)] | [Color Code (4 bits)]

Antenna codes: 0x00, 0x30, 0x50, 0x60 (antennas 0–3)
Color codes: 0x09=RED, 0x0A=GREEN, 0x0C=BLUE, 0x0F=PURPLE
```

### Controller

| Function | Pin |
|----------|-----|
| LCD I2C address | 0x27 (16×2 display) |
| Joystick X | GPIO 34 |
| Joystick Y | GPIO 35 |
| Joystick button | GPIO 32 |
| Button A (select) | GPIO 33 |
| Button B (back) | GPIO 25 |

**LCD menu structure:**
```
/ (root)
├─ field/
│  ├─ status/
│  │  ├─ button.disp    → shows BUTTON status
│  │  ├─ crank.disp     → shows CRANK status
│  │  ├─ pressure.disp  → shows PRESSURE status
│  │  ├─ keypad.disp    → shows KEYPAD status
│  │  ├─ earth.disp     → shows EARTH + color results (C:X W:Y)
│  │  └─ timeleft.disp  → 3-minute countdown timer
│  ├─ reset.action       → broadcast RESET command
│  └─ start.action       → broadcast START, begin 3-min timer
└─ control/
   └─ cycledisplay.action → broadcast CYCLE_DISPLAY (rainbow animation)
```

## Communication Protocol

### ESP-NOW Messages

All elements communicate via ESP-NOW broadcast (`FF:FF:FF:FF:FF:FF`). Message types:

#### Status Update (element → controller, 1 Hz)

```cpp
struct StatusUpdateMessage {
  FieldMessageHeader header;  // type=STATUS_UPDATE, senderId, sequenceNum
  ElementStatus status;       // NOT_ONLINE, READY, NOT_ACTIVATED, ACTIVATED
  uint8_t reserved[2];
};
```

#### Command (controller → all elements)

```cpp
struct CommandMessage {
  FieldMessageHeader header;  // type=COMMAND
  FieldCommand command;       // RESET, START, CYCLE_DISPLAY, PING
  ElementId targetId;         // Specific element or 0xFF for broadcast
  uint8_t reserved;
};
```

#### Color Report (antenna element → earth)

```cpp
struct ColorReportMessage {
  FieldMessageHeader header;  // type=COLOR_REPORT
  FieldColor color;           // RED, GREEN, BLUE, PURPLE, NONE
  uint8_t reserved[2];
};
```

#### Color Result (earth → controller)

```cpp
struct ColorResultMessage {
  FieldMessageHeader header;  // type=COLOR_RESULT
  uint8_t correctCount;
  uint8_t wrongCount;
  AntennaColorResult results[4];  // Per-antenna: expected, received, match
};
```

### Message Flow

```
Controller ──RESET/START──► All elements
                              │
Elements ──STATUS_UPDATE──► Controller (1 Hz)
                              │
Antenna elements ──COLOR_REPORT──► Earth
                              │
Earth ──COLOR_RESULT──► Controller
```

### Element Status States

| Status | Value | Description |
|--------|-------|-------------|
| NOT_ONLINE | 0 | No status update for >5 seconds |
| READY | 1 | Initialized, waiting for START |
| NOT_ACTIVATED | 2 | Task started but not completed |
| ACTIVATED | 3 | Task completed successfully |

## IR NEC Protocol

Used by the drone to transmit antenna colors to the Earth receiver.

- **Carrier:** 38 kHz
- **Protocol:** NEC standard (9ms lead + 4.5ms space + 32-bit payload)
- **Payload:** 8-bit address (0xBB) + inverted + 8-bit command + inverted
- **Command byte:** `[antenna_code | color_code]`

### Color Verification Flow

1. Each antenna element generates a random color on startup
2. Elements send COLOR_REPORT with assigned color via ESP-NOW
3. Drone flies over and transmits IR with matching color codes
4. Earth element receives IR, decodes antenna + color
5. Earth compares received vs expected and sends COLOR_RESULT to controller
6. Controller displays correct/wrong counts on LCD

## Testing

### Manual test procedure

1. Flash controller first → LCD shows "Initializing..."
2. Flash each element individually
3. Controller should detect elements via STATUS_UPDATE messages
4. Navigate menu to verify status: `field → status → <element>`
5. Execute **RESET** action → all elements restart
6. Execute **START** action → timer starts, all go NOT_ACTIVATED
7. Complete each task manually → observe ACTIVATED status
8. Execute **CYCLE_DISPLAY** → RGB rainbow animation on all elements

### Serial debugging

Each element logs initialization, received commands, and task progress at 921600 baud. Use `pio device monitor -e <env>` to view.

## Dependencies

| Library | Version | Used By |
|---------|---------|---------|
| `ezButton` | 1.0.0 | Button (debounce) |
| `IRremote` | 4.4.1 | Earth (IR receive) |
| `LiquidCrystal_I2C` | 1.1.4 | Controller (LCD) |
| WiFi/esp_now | built-in | All (ESP-NOW transport) |
