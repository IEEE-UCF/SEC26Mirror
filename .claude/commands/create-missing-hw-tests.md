Scan the MCU codebase and create any missing hardware test environments for robot subsystems.

## Step 1: Inventory subsystems and existing tests

1. Read every `.h` file in `mcu_ws/src/robot/subsystems/` to get the full list of concrete subsystems.
2. Read every `.cpp` file name in `mcu_ws/src/test/` to get the list of existing hardware tests.
3. Read `mcu_ws/platformio.ini` to get the list of existing `teensy-test-*` and `esp32-test-*` environments.

Cross-reference to determine which subsystems have dedicated tests and which do not.

## Step 2: Decide which are testable

Skip subsystems that fall into any of these categories:
- **Pseudo-code / incomplete**: ROS entities are commented out or the file says "pseudo-code" (e.g., ArmSubsystem as of Feb 2026).
- **Abstract base classes**: `McuSubsystem`, `TimedSubsystem`, `RobotManager`, etc. in `mcu_ws/lib/subsystems/`.
- **Trivial / no I/O**: `HeartbeatSubsystem` (just a string publisher, already covered by all-subsystems test).
- **Complex multi-device setups**: Only skip if the subsystem cannot be tested without an additional secondary MCU that isn't normally connected (e.g., MiniRobotSubsystem needs an ESP32 I2C slave).

For each remaining subsystem that lacks a dedicated test, proceed to Step 3.

Present the user a summary table of what will be created before writing any files.

## Step 3: Create test files

For each missing test, create `mcu_ws/src/test/teensy-test-<subsystem-name>.cpp` following these established patterns:

### File structure template

```cpp
/**
 * @file teensy-test-<name>.cpp
 * @brief Focused test — <one-line description>.
 * @date <today>
 *
 * Hardware expected:
 *   <pin / bus / device list>
 *
 * micro-ROS topics published:
 *   <topic>   <msg type>   <rate>
 *
 * micro-ROS services (if any):
 *   <service>   <srv type>
 *
 * Usage:
 *   <ros2 commands to verify>
 */

#include <Arduino.h>
#include <TeensyThreads.h>
#include <microros_manager_robot.h>
// ... subsystem-specific includes from robot/RobotPins.h, I2CBusLock.h, drivers, etc.

using namespace Subsystem;

// --- micro-ROS manager ---
static MicrorosManagerSetup g_mr_setup("<test_name>_mr");
static MicrorosManager g_mr(g_mr_setup);

// --- subsystem driver + instance ---
// (use RobotPins.h constants for addresses and pins)

// --- blink task (visual heartbeat) ---
static void blink_task(void*) { /* LED_BUILTIN toggle at 1 Hz */ }

// --- print task (debug) ---
static void print_task(void*) { /* Serial.printf status every 2 s */ }

void setup() {
  Serial.begin(921600);
  delay(500);
  Serial.println(PSTR("\r\nSEC26 Robot — <Test Name>\r\n"));

  I2CBus::initLocks();  // only if I2C is used

  g_mr.init();
  // init drivers and subsystem
  // register micro-ROS participant
  // start threads with appropriate priorities
}

void loop() { threads.delay(100); }
```

### Key conventions

- Use constants from `robot/RobotPins.h` (never hardcode pin numbers or I2C addresses).
- If the subsystem uses I2C, call `I2CBus::initLocks()` in setup.
- If PCA9685 is involved, create a `PCA9685Manager` + flush task at 20 ms.
- micro-ROS manager always gets highest priority (4) and 8192 stack.
- Subsystem thread priorities: sensors/RC at 3, actuators at 2, display/status at 1.
- The `print_task` should show meaningful state (connection status + key data from the subsystem).
- Only access public API of subsystems — do not access private members.

## Step 4: Add platformio.ini environments

For each new test, append an environment block to `mcu_ws/platformio.ini` after the existing test environments:

```ini
[env:teensy-test-<name>]
extends = teensy_microros
build_src_filter =
    +<test/teensy-test-<name>.cpp>
    +<robot/subsystems/<Subsystem>.cpp>     ; only if the subsystem has a .cpp file
    +<platform/atomic_stubs_arm.c>
build_flags = ${teensy_microros.build_flags} -DUSE_TEENSYTHREADS
```

Header-only subsystems (no `.cpp` in `src/robot/subsystems/`) do NOT need a `build_src_filter` entry for the subsystem source.

If the subsystem needs additional driver `.cpp` files that aren't in `lib/` (libraries are auto-discovered), add those to `build_src_filter` as well.

## Step 5: Update documentation

1. Add a section for each new test in `mcu_ws/src/test/README.md` following the format of existing entries (environment table, wiring table, build/flash/monitor commands, expected output, ROS2 verification commands).
2. Update `CLAUDE.md` if new test environments are added — add them to the hardware tests listing.
3. Update `project_status.md` hardware tests table if it exists.

## Step 6: Build verification

Build each new test environment inside Docker to confirm it compiles:

```bash
docker compose exec devcontainer bash -c "cd /home/ubuntu/mcu_workspaces/sec26mcu && pio run -e teensy-test-<name> 2>&1"
```

Build all new environments in parallel where possible. Report any build errors, fix them, and rebuild.

## Step 7: Summary

Print a final summary of:
- Tests created (file path + environment name)
- Tests skipped and why
- Build results (SUCCESS / FAIL for each)
