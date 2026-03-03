# Motor FG Encoder System

Hardware QTimer-based pulse counting for 8 motor frequency generator (FG) signals on Teensy 4.1 (IMXRT1062).

## How It Works

Each motor has a built-in FG output that produces a pulse train proportional to shaft speed. Rather than counting pulses in software (interrupts), this system uses the IMXRT1062's hardware QTimer peripherals to count edges with zero CPU overhead. The 16-bit counter registers are read periodically and direction is applied from the motor driver's commanded state.

## Pin-to-QTimer Routing

Six of the eight FG pins require routing through the XBAR crossbar to reach a QTimer input. Pins 6 and 9 connect directly to QTimer4.

```
Pin 2 (EMC_04) ──XBAR In 6──▶ XBAR Out 86 ──▶ QTimer1 Timer 0
Pin 3 (EMC_05) ──XBAR In 7──▶ XBAR Out 87 ──▶ QTimer1 Timer 1
Pin 4 (EMC_06) ──XBAR In 8──▶ XBAR Out 88 ──▶ QTimer1 Timer 2
Pin 5 (EMC_08) ──XBAR In 17─▶ XBAR Out 89 ──▶ QTimer1 Timer 3
Pin 6 (B0_10)  ─────────────── direct ───────▶ QTimer4 Timer 1
Pin 7 (B1_01)  ──XBAR In 15─▶ XBAR Out 90 ──▶ QTimer2 Timer 0
Pin 8 (B1_00)  ──XBAR In 14─▶ XBAR Out 91 ──▶ QTimer2 Timer 1
Pin 9 (B0_11)  ─────────────── direct ───────▶ QTimer4 Timer 2
```

**QTimers used:** QTimer1 (all 4 channels), QTimer2 (channels 0-1), QTimer4 (channels 1-2). QTimer3 is free for other uses.

## Motor-to-Encoder Mapping

| Encoder Index | Teensy Pin | Motor | PCA9685 DIR Channel | Notes |
|:---:|:---:|:---:|:---:|---|
| 0 | 2 | Motor 1 | 2 | Motors 1-4: DIR channels 2,3,4,5 |
| 1 | 3 | Motor 2 | 3 | |
| 2 | 4 | Motor 3 | 4 | |
| 3 | 5 | Motor 4 | 5 | |
| 4 | 6 | Motor 5 | 9 | Motors 5-8: DIR channels 9,8,7,6 |
| 5 | 7 | Motor 6 | 8 | |
| 6 | 8 | Motor 7 | 7 | |
| 7 | 9 | Motor 8 | 6 | |

## Direction Handling

The FG signal only encodes speed (pulse frequency), not direction. Direction is determined from the PCA9685 motor driver's commanded state via `MotorManagerSubsystem::getIntendedDirection()`.

**Why "intended" direction?** The NFPShop brushless motor controllers require a brief reverse pulse (~3 ms every ~103 ms) to prevent faulting. During this pulse the PCA9685 direction channel is temporarily flipped, but the motor doesn't actually reverse due to inertia. `getIntendedDirection()` returns the logical direction from `dirs_[]` which is only modified by `setSpeed()`, making it immune to the transient reverse pulse.

**PCA9685 flush latency:** Motor commands are buffered and flushed to I2C every ~20 ms by `pca_task`. During a direction change, there is up to 20 ms where the commanded direction differs from the physical motor direction. This is acceptable because mechanical direction reversal is gradual anyway.

## Register Configuration

### Clock Gates
```
CCM_CCGR2: XBAR1 enabled
CCM_CCGR6: QTimer1, QTimer2, QTimer4 enabled
```

### IOMUX
| Pin | Pad | ALT Mode | Function |
|-----|-----|:--------:|----------|
| 2 | GPIO_EMC_04 | ALT3 | XBAR1_INOUT06 |
| 3 | GPIO_EMC_05 | ALT3 | XBAR1_INOUT07 |
| 4 | GPIO_EMC_06 | ALT3 | XBAR1_INOUT08 |
| 5 | GPIO_EMC_08 | ALT3 | XBAR1_INOUT17 |
| 6 | GPIO_B0_10  | ALT1 | QTIMER4_TIMER1 |
| 7 | GPIO_B1_01  | ALT1 | XBAR1_INOUT15 |
| 8 | GPIO_B1_00  | ALT1 | XBAR1_INOUT14 |
| 9 | GPIO_B0_11  | ALT1 | QTIMER4_TIMER2 |

Pad control: `0x10B0` (hysteresis, 100k pulldown, fast slew).

### XBAR Crossbar

Each `XBARA1_SELn` register is 16 bits: `[6:0]` = even output source, `[14:8]` = odd output source.

| Register | Even Output | Source | Odd Output | Source |
|----------|-------------|--------|------------|--------|
| SEL43 | Out 86 → TMR1_T0 | In 6 (Pin 2) | Out 87 → TMR1_T1 | In 7 (Pin 3) |
| SEL44 | Out 88 → TMR1_T2 | In 8 (Pin 4) | Out 89 → TMR1_T3 | In 17 (Pin 5) |
| SEL45 | Out 90 → TMR2_T0 | In 15 (Pin 7) | Out 91 → TMR2_T1 | In 14 (Pin 8) |

### XBAR Input Daisy Chain

B-port pads have multiple possible sources. Select input registers disambiguate:
- `IOMUXC_XBAR1_IN14_SELECT_INPUT = 1` (selects GPIO_B1_00)
- `IOMUXC_XBAR1_IN15_SELECT_INPUT = 1` (selects GPIO_B1_01)

EMC pads use default value `0`.

### QTimer Channel Config

All 8 channels use identical settings:

| Register | Value | Meaning |
|----------|-------|---------|
| CTRL | `TMR_CTRL_CM(1) \| TMR_CTRL_PCS(N)` | Count rising edges; primary source = counter N input |
| CNTR | 0 | Start at zero |
| LOAD | 0 | Unused in free-running mode |
| COMP1 | 0xFFFF | Free-running rollover at 65535 |
| SCTRL | 0 | Non-inverted input |
| FILT | 3 | ~60 ns debounce (3 IPbus clocks at 150 MHz) |

`TMR_CTRL_PCS(N)` must match the timer channel index within its QTimer block (e.g., QTimer1 Timer 2 uses `PCS(2)`).

## Software Architecture

```
QTimerEncoder (lib/encoders/)         EncoderSubsystem (src/robot/subsystems/)
┌─────────────────────────┐           ┌──────────────────────────────────┐
│ Hardware driver          │           │ micro-ROS integration            │
│                          │           │                                  │
│ init()                   │◀──────── │ init() calls encoder->init()     │
│   enableClockGates()     │           │                                  │
│   configureIOMUX()       │           │ update() @ 50 Hz:               │
│   configureXBAR()        │           │   read dirs from MotorManager   │
│   configureQTimers()     │           │   call encoder->captureAll()    │
│                          │           │   compute ticks/sec             │
│ captureAll(dirs[8])      │◀──────── │   publish Float32MultiArray     │
│   read 8 CNTR registers │           │                                  │
│   unsigned delta (wraps) │           │ topic: /mcu_robot/encoders      │
│   apply direction sign   │           │   8 floats = signed ticks/sec   │
│   accumulate int32 ticks │           │                                  │
│                          │           │ getTickRate(ch) for internal use │
│ getTicks(ch) / resetAll()│           └──────────────────────────────────┘
└─────────────────────────┘
         ▲
         │ directions
         │
┌────────┴────────────────┐
│ MotorManagerSubsystem    │
│ getIntendedDirection(m)  │
│   returns dirs_[m]       │
│   (immune to NFPShop     │
│    reverse pulses)       │
└──────────────────────────┘
```

## Thread Safety

- **CNTR register reads**: Atomic 16-bit memory-mapped reads. No mutex needed.
- **`dirs_[]` reads**: Single-byte bools, naturally atomic on ARM Cortex-M7.
- **`rcl_publish()`**: Protected by `g_microros_mutex` per project convention.
- **`captureAll()` / `resetAll()`**: Called only from the encoder thread. No contention.

## ROS2 Topic

**`/mcu_robot/encoders`** — `std_msgs/msg/Float32MultiArray`

8 float values: signed ticks per second for motors 1-8 (index 0-7). Positive = forward, negative = reverse. Published at 50 Hz. Publisher only — consumes 0 executor handles.

## Building

```bash
# Full rebuild (required after custom_microros.meta change):
pio run -e robot -t clean_microros && pio run -e robot

# Test environment:
pio run -e teensy-test-all-subsystems
```

## Adding to XBAR Reference

The IMXRT1062 XBAR is documented in the reference manual starting at page 3310. Select registers start at page 3335. Each register controls two outputs — find the register containing your desired QTimer output and set the corresponding bits to the XBAR input number for your pin.

Any XBAR input can be routed to any XBAR output, making this system very flexible for future pin reassignment.
