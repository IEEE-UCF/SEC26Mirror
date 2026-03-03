# QTimer-Based Motor Encoder Subsystem — Implementation Plan

## Overview

Create a hardware QTimer + XBAR crossbar pulse counter for 8 motor FG (frequency generator) signals on Teensy pins 2-9. Publish signed tick rates to micro-ROS as a standalone subsystem, fully decoupled from the drivebase.

## Pin-to-QTimer Routing

| Pin | IMXRT Pad | Route | QTimer Channel |
|-----|-----------|-------|----------------|
| 2 | EMC_04 | XBAR In 6 → Out 86 | QTimer1 Timer 0 |
| 3 | EMC_05 | XBAR In 7 → Out 87 | QTimer1 Timer 1 |
| 4 | EMC_06 | XBAR In 8 → Out 88 | QTimer1 Timer 2 |
| 5 | EMC_08 | XBAR In 17 → Out 89 | QTimer1 Timer 3 |
| 6 | B0_10 | **Direct** TMR4 ch1 | QTimer4 Timer 1 |
| 7 | B1_01 | XBAR In 15 → Out 90 | QTimer2 Timer 0 |
| 8 | B1_00 | XBAR In 14 → Out 91 | QTimer2 Timer 1 |
| 9 | B0_11 | **Direct** TMR4 ch2 | QTimer4 Timer 2 |

Uses QTimer1 (4 ch), QTimer2 (2 ch), QTimer4 (2 ch). QTimer3 remains free.

## Direction Strategy

- Read `MotorManagerSubsystem::dirs_[motor]` via new `getIntendedDirection(motor)` accessor
- This is the **intended** direction — immune to NFPShop 3ms reverse pulses (which flip the PCA9685 buffer but don't modify `dirs_[]`)
- PCA9685 flush latency (~20ms) acknowledged: during direction transitions, brief misattribution is acceptable since motor inertia means physical direction change is gradual anyway
- Encoder channel N maps to MotorManagerSubsystem motor index N (pin 2 = motor 0, ..., pin 9 = motor 7)

## Files to Create

### 1. `mcu_ws/lib/encoders/QTimerEncoder.h` — Hardware driver header
- `QTimerEncoder` class with `init()`, `captureAll(dirs)`, `getTicks(ch)`, `resetTicks(ch)`, `resetAll()`, `readRaw(ch)`
- Stores per-channel: CNTR register pointer, last count, accumulated signed ticks
- Pure hardware abstraction, no ROS dependency

### 2. `mcu_ws/lib/encoders/QTimerEncoder.cpp` — Hardware driver implementation
- `enableClockGates()`: Enable XBAR1 (CCM_CCGR2), QTimer1/2/4 (CCM_CCGR6)
- `configureIOMUX()`: Set pin ALT modes (ALT3 for XBAR pins, ALT1 for direct/B-port pins), set XBAR input daisy chain select registers for B1_00/B1_01
- `configureXBAR()`: Set XBARA1_SEL43/44/45 to route inputs to QTimer outputs
- `configureQTimers()`: For each channel — set CTRL (count rising edges, PCS=counter N input), FILT (debounce), COMP1=0xFFFF (free-running), enable via TMRx_ENBL
- `captureAll()`: Read all 8 CNTR registers, compute unsigned deltas (handles 16-bit wrap), apply direction sign, accumulate into int32 ticks
- Define `TMR_CTRL_CM(n)` and `TMR_CTRL_PCS(n)` macros locally (not in Teensy core)

### 3. `mcu_ws/src/robot/subsystems/EncoderSubsystem.h` — Micro-ROS subsystem (header-only)
- Follows existing subsystem pattern (BaseSubsystem + IMicroRosParticipant + beginThreaded)
- Setup takes: QTimerEncoder*, MotorManagerSubsystem*, topic name, publish rate
- `update()`: read directions from motor manager, call `captureAll()`, compute ticks/sec, publish at configured rate
- Publishes `/mcu_robot/encoders` as `Float32MultiArray` (8 floats = signed ticks/sec)
- Publisher only — consumes 0 executor handles
- Exposes `getTickRate(motor)` for internal use by other subsystems

## Files to Modify

### 4. `mcu_ws/src/robot/subsystems/MotorManagerSubsystem.h`
- Add public method `getIntendedDirection(uint8_t motor)` returning `dirs_[motor]`
- Single bool read, atomic on ARM — no mutex needed

### 5. `mcu_ws/lib/microros/microros_manager_robot.h`
- Bump `MAX_PARTICIPANTS` from 16 → 20 (currently at capacity with 16 registered)

### 6. `mcu_ws/custom_microros.meta`
- Bump `RMW_UXRCE_MAX_PUBLISHERS` from 16 → 18
- **Requires `clean_microros` rebuild**

### 7. `mcu_ws/src/robot/RobotPins.h`
- Update comments for pins 2-9 to document them as FG encoder inputs (not "Motor output")

### 8. `mcu_ws/src/robot/machines/RobotLogic.h`
- Add includes for `QTimerEncoder.h` and `EncoderSubsystem.h`
- Add static instances: `g_qtimer_encoder`, `g_encoder_sub_setup`, `g_encoder_sub`
- In `setup()`: call `g_encoder_sub.init()`, register with `g_mr`, start with `beginThreaded(1024, 2, 20)` (50 Hz, priority 2)

## Register Details (IMXRT1062)

**IOMUX**: Pins 2-5 use ALT3, pins 6-9 use ALT1. Pad control 0x10B0 (hysteresis, 100k pulldown, fast slew).

**XBAR SEL registers** (16-bit, bits [6:0] = even output, [14:8] = odd output):
- SEL43 = `6 | (7 << 8)` — Out 86←In6, Out 87←In7
- SEL44 = `8 | (17 << 8)` — Out 88←In8, Out 89←In17
- SEL45 = `15 | (14 << 8)` — Out 90←In15, Out 91←In14

**QTimer CTRL**: `TMR_CTRL_CM(1) | TMR_CTRL_PCS(n)` — count rising edges, primary source = counter N input

**XBAR input daisy chain**: `IOMUXC_XBAR1_IN14_SELECT_INPUT = 1`, `IOMUXC_XBAR1_IN15_SELECT_INPUT = 1` (select B1_00/B1_01 pads)

## Build & Verification

```bash
# Inside Docker container:
pio run -e robot -t clean_microros && pio run -e robot
```

Test by subscribing to `/mcu_robot/encoders` — should show 8 float values that change with motor speed and flip sign with direction changes.

## Assumptions to Confirm
- Pins 2-9 are FG signal **inputs** from motor encoders (not GPIO outputs to motors)
- Encoder channel 0 (pin 2) corresponds to MotorManagerSubsystem motor index 0, channel 1 (pin 3) → motor 1, etc.
- The "Motors 1-4 from 2:5, Motors 5-8 from 9:5" refers to PCA9685 direction channel wiring, which is handled transparently by reading `dirs_[]` from MotorManagerSubsystem
