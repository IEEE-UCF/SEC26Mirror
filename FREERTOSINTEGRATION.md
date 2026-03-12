# FreeRTOS Integration on Teensy 4.1

## Repository

`workingrtos/` — PlatformIO project running FreeRTOS on a Teensy 4.1.

## Platform & Dependencies

Uses [tsandmann/platform-teensy](https://github.com/tsandmann/platform-teensy.git) and [tsandmann/freertos-teensy](https://github.com/tsandmann/freertos-teensy.git) (FreeRTOS 11.2.0, Teensy 4.x port with proper USB serial support).

### platformio.ini

```ini
[env:teensy41]
platform = https://github.com/tsandmann/platform-teensy.git
board = teensy41
framework = arduino
lib_deps = https://github.com/tsandmann/freertos-teensy.git
build_flags = -Wformat=1 -DUSB_SERIAL -DTEENSY_OPT_FASTER_LTO
upload_flags = -v
upload_protocol = teensy-cli
monitor_speed = 921600
```

## What Was Done

1. **Tried the IEEE-UCF/FreeRTOS-Teensy4 library first** — it failed because its `FreeRTOSConfig_default.h` sets `configPRIO_BITS = 8` (falls back from `__NVIC_PRIO_BITS` which is never defined through the Teensy Arduino.h include chain). The iMXRT1062 has 4 priority bits, so `configPRIO_BITS` must be 4. With the wrong value, `configMAX_SYSCALL_INTERRUPT_PRIORITY` becomes 2 instead of 32 (0x20), causing `basepri=2` during every FreeRTOS critical section. This masks all interrupts except priority 0, starving the USB interrupt and disconnecting serial immediately after the scheduler starts.

2. **Created `include/Teensy4FreeRTOSConfig.h`** to override with correct `configPRIO_BITS = 4`. USB still disconnected — the old library's port has additional compatibility issues with the Teensy 4.1 USB stack.

3. **Switched to `tsandmann/freertos-teensy`** — a maintained FreeRTOS 11.2 port that properly handles USB serial, interrupt priorities, and the Teensy 4.x SysTick (100kHz external clock). Works out of the box.

## Current Code (src/main.cpp)

Semaphore-based LED blink demonstrating task synchronization:

- **Thread 1** (priority 2): waits on a binary semaphore, turns LED off when signaled.
- **Thread 2** (priority 1): turns LED on, delays 500ms, gives semaphore to Thread 1, delays 500ms, repeats.
- Uses `xSemaphoreCreateBinary`, `xSemaphoreTake`, `xSemaphoreGive`.
- Serial output shows task handoff messages over USB CDC.

Header to use: `#include "arduino_freertos.h"` and `#include "semphr.h"`.

## Build & Flash

```bash
pio run -e teensy41           # build
pio run -t upload -e teensy41 # flash (press program button if prompted)
pio device monitor -b 921600  # serial monitor
```

## Key Lessons

- The Teensy 4.x `__NVIC_PRIO_BITS` is only defined in `core_cm7.h` (via `arm_math.h`), NOT through the `Arduino.h` include chain. Any FreeRTOS config relying on `#ifdef __NVIC_PRIO_BITS` will silently get the wrong value.
- `configMINIMAL_STACK_SIZE` of 90 words (360 bytes) is too small for tasks using USB serial. Use at least 128–256 words.
- The `tsandmann/freertos-teensy` library handles all of this correctly and is the recommended path for Teensy 4.x + FreeRTOS + PlatformIO.
